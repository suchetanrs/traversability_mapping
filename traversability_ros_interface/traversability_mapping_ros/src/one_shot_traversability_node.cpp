/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * Thin ROS adapter for a ONE-SHOT traversability map built from a static PLY cloud.
 *
 * Same core (System / LocalMap / KeyFrame) as the streaming nodes, driven once: a
 * pre-built reconstructed cloud (already expressed in the map frame) is read from a
 * PLY file and ingested as a SINGLE keyframe at identity pose in a SINGLE map. The
 * LocalMap worker bins it exactly once and computes traversability; the resulting
 * grid_map + occupancy grid are published on LATCHED (transient_local) topics, so the
 * finished map is delivered to every new subscriber without ever recomputing.
 *
 * Because the cloud is already a full map (not a sensor scan), the core's per-scan
 * range/height pruning must be disabled in the params file (see
 * traversabilityParamsOneShot.yaml) or pruneToBase would discard almost every point.
 * Extrinsics are identity: cloud coordinates ARE map coordinates.
 *
 *   startup -> load PLY -> System::addNewKeyFrameWithPCL (kf 0, map 0) ->
 *              System::updateKeyFrame(identity) to trigger the (only) bin.
 *   onUpdate -> the worker fires this once the bin + hazard recompute finishes;
 *              publish the full grid_map + occupancy, latched.
 */
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "traversability_mapping/System.hpp"
#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/Parameters.hpp"

#include "parameter_bridge.hpp"

namespace tmap = traversability_mapping;

class OneShotTraversabilityNode : public rclcpp::Node
{
public:
    OneShotTraversabilityNode() : Node("one_shot_traversability_node")
    {
        // --- ROS params ---
        ply_file_path_ = declare_parameter<std::string>("ply_file_path", "");
        // Frame the static cloud already lives in; stamped on every published message.
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        const std::string parameter_file_path = declare_parameter<std::string>("parameter_file_path", "");

        // Load the core params, then expose them as ROS params (introspective bridge).
        // Must precede System construction, which caches several of these values.
        tmap::ParameterHandler::getInstance(parameter_file_path);
        param_cb_handle_ = tmap_ros::bridgeCoreParameters(this);
        system_ = std::make_shared<tmap::System>();
        system_->setMapFrame(map_frame_);

        // Identity extrinsics: the PLY is already a full map in the map frame, so no
        // base<-lidar transform is applied and (with pruning disabled in the params)
        // the cloud passes through unchanged. No TF listener needed.
        system_->setExtrinsicParameters(Eigen::Affine3f::Identity(), Eigen::Affine3f::Identity());

        // --- Latched publishers (transient_local): the finished map is held and
        // replayed to every late subscriber; publish unconditionally on the (single) bin. ---
        gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
            "one_shot_traversability_gridmap", rclcpp::QoS(1).transient_local());
        occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "traversability_occupancy", rclcpp::QoS(1).transient_local());

        // Single map. The worker fires onMapUpdated() once the keyframe is binned and
        // hazards are recomputed.
        system_->addNewLocalMap(0, [this]() { onMapUpdated(); });

        // Read + ingest the cloud. Heavy (whole-file load + bin), but this runs in the
        // constructor before spinning; the actual bin/recompute happens on the worker.
        ingest();

        RCLCPP_INFO(get_logger(), "One-shot traversability adapter ready.");
    }

    ~OneShotTraversabilityNode() override
    {
        // Join the LocalMap workers before the publishers die (an in-flight
        // onMapUpdated() on a worker thread must not touch a dead publisher).
        system_.reset();
    }

private:
    // Load the PLY, ingest it as a single keyframe, and trigger the one bin.
    void ingest()
    {
        if (ply_file_path_.empty())
        {
            RCLCPP_ERROR(get_logger(), "No 'ply_file_path' set; nothing to ingest.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Loading PLY: %s", ply_file_path_.c_str());
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPLYFile(ply_file_path_, *cloud) < 0 || cloud->empty())
        {
            RCLCPP_ERROR(get_logger(), "Failed to load (or empty) PLY: %s", ply_file_path_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu points; ingesting as a single keyframe...",
                    cloud->size());

        constexpr std::uint64_t kKfId = 0;
        constexpr std::uint64_t kMapId = 0;
        constexpr unsigned long long kTimestampNs = 0ULL;  // static map: timestamp irrelevant
        system_->addNewKeyFrameWithPCL(kTimestampNs, kKfId, kMapId, cloud);
        // Identity pose: cloud is already in the map frame. Supplies the pose AND
        // triggers the (only) bin on the worker thread.
        system_->updateKeyFrame(kKfId, Eigen::Affine3f::Identity());

        RCLCPP_INFO(get_logger(), "Keyframe queued; binning + hazard recompute on worker thread.");
    }

    // Fired by the LocalMap worker once the keyframe is binned and hazards recomputed.
    // Runs on the worker thread (rclcpp publishers are thread-safe and the grid lock has
    // been released). Publishes the full grid_map + occupancy, latched.
    void onMapUpdated()
    {
        auto lm = system_->getLocalMap();
        if (!lm)
            return;

        std::unique_ptr<grid_map_msgs::msg::GridMap> gm;
        nav_msgs::msg::OccupancyGrid og;
        {
            std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
            gm = grid_map::GridMapRosConverter::toMessage(lm->getGridMap(), navLayers_);
            grid_map::GridMapRosConverter::toOccupancyGrid(lm->getGridMap(), "hazard", 0.0, 1.0, og);
        }

        const auto stamp = now();
        gm->header.frame_id = map_frame_;
        gm->header.stamp = stamp;
        gridmap_pub_->publish(*gm);

        og.header.frame_id = map_frame_;
        og.header.stamp = stamp;
        occupancy_pub_->publish(og);

        RCLCPP_INFO(get_logger(), "Published one-shot traversability map (%u x %u cells).",
                    og.info.width, og.info.height);
    }

    // ---- members ------------------------------------------------------------
    std::string ply_file_path_;
    std::string map_frame_;

    // The grid layers published in the debug grid_map (occupancy uses "hazard").
    const std::vector<std::string> navLayers_ = {
        "normal_x", "normal_y", "normal_z", "slope_haz", "border_haz",
        "step_haz", "elevation", "roughness_haz", "hazard", "step_haz_inflated"};

    std::shared_ptr<tmap::System> system_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OneShotTraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
