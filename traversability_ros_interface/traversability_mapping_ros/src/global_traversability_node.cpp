/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * Thin ROS adapter for the moment-fused traversability core.
 *
 * It owns NO mapping logic: it resolves the robot extrinsics from TF, converts
 * messages <-> plain data (see conversions.hpp), drives the core via the legacy
 * System API, and publishes. All mapping (grid, moments, hazards, threads, sparse
 * deltas) lives in the ROS-free core (System / LocalMap / KeyFrame).
 *
 *   additions -> per keyframe: sensor_msgs->PCL, System::addNewKeyFrameWithPCL,
 *                then System::updateKeyFrame(pose) to set the pose and trigger the
 *                first bin.
 *   updates   -> per keyframe: System::updateKeyFrame(pose) (O(1), latest-wins).
 *   timer     -> sparse delta (deployment); full grid_map + occupancy only when a
 *                subscriber is present (debug); full snapshot on (re)connect.
 */
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>

#include <traversability_msgs/msg/key_frame_additions.hpp>
#include <traversability_msgs/msg/key_frame_updates.hpp>
#include <traversability_msgs/msg/traversability_sparse_update.hpp>
#include <traversability_msgs/srv/get_global_pointcloud.hpp>

#include "traversability_mapping/System.hpp"
#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/Parameters.hpp"

#include "conversions.hpp"
#include "common.hpp"
#include "parameter_bridge.hpp"

namespace tmap = traversability_mapping;

class GlobalTraversabilityNode : public rclcpp::Node
{
public:
    GlobalTraversabilityNode() : Node("global_traversability_node")
    {
        // --- ROS params (names match traversability_gt_ros_params.yaml) ---
        additions_topic_ = declare_parameter<std::string>("additions_topic_name", "traversability_keyframe_additions");
        updates_topic_ = declare_parameter<std::string>("updates_topic_name", "traversability_keyframe_updates");
        slam_frame_ = declare_parameter<std::string>("slam_frame", "base_footprint");
        robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
        lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic_name", "lidar/points");
        const std::string parameter_file_path = declare_parameter<std::string>("parameter_file_path", "");

        // Load the core params, then expose them as ROS params (introspective
        // bridge; no parameter is re-listed here) so they are visible to
        // `ros2 param get/set`. Must precede System construction, which caches
        // several of these values.
        tmap::ParameterHandler::getInstance(parameter_file_path);
        param_cb_handle_ = tmap_ros::bridgeCoreParameters(this);
        system_ = std::make_shared<tmap::System>();
        system_->setMapFrame(map_frame_);

        // --- Extrinsics from TF ---
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        Eigen::Affine3f Tsv, Tbs;
        populateTransforms(slam_frame_, robot_base_frame_, lidar_frame_, get_clock(), get_logger(),
                           tf_buffer_, Tsv, Tbs);
        system_->setExtrinsicParameters(Tsv, Tbs);

        // Single map for now (multi-map-ready). The worker fires onMapUpdated()
        // after every grid-changing keyframe op so the debug grid_map + occupancy
        // refresh on every recompute (not just on the timer).
        system_->addNewLocalMap(0, [this]() { onMapUpdated(); });

        // --- ROS I/O ---
        // Deep queues: additions carry clouds and binning can briefly block the
        // callback on the map mutex; a shallow queue would drop keyframes under load
        // (showing up later as "update for unknown keyframe N").
        additions_sub_ = create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_, 100,
            std::bind(&GlobalTraversabilityNode::additionsCallback, this, std::placeholders::_1));
        updates_sub_ = create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_, 100,
            std::bind(&GlobalTraversabilityNode::updatesCallback, this, std::placeholders::_1));

        // Optional raw-cloud buffering path (SLAM that announces keyframes by ts).
        if (tmap::ParameterHandler::getInstance().getValue<bool>("ingestion/use_pointcloud_buffer"))
        {
            cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud_topic_, rclcpp::SensorDataQoS(),
                std::bind(&GlobalTraversabilityNode::cloudCallback, this, std::placeholders::_1));
        }

        gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
            "global_traversability_gridmap", rclcpp::QoS(1).transient_local());
        occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "global_traversability_occupancy", rclcpp::QoS(1).transient_local());
        sparse_pub_ = create_publisher<traversability_msgs::msg::TraversabilitySparseUpdate>(
            "global_traversability_updates", rclcpp::QoS(10).reliable());
        global_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "kf_pointcloud", rclcpp::QoS(1).transient_local());

        cloud_srv_ = create_service<traversability_msgs::srv::GetGlobalPointcloud>(
            "publish_global_pointcloud",
            std::bind(&GlobalTraversabilityNode::globalCloudService, this,
                      std::placeholders::_1, std::placeholders::_2));

        const double rate = tmap::ParameterHandler::getInstance().getValue<double>("node/publish_rate_hz");
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(0.1, rate)),
            std::bind(&GlobalTraversabilityNode::publish, this));

        RCLCPP_INFO(get_logger(), "Thin traversability adapter ready.");
    }

    ~GlobalTraversabilityNode() override
    {
        // Tear down the core (joins the LocalMap worker threads) BEFORE the node's
        // publishers are destroyed, so an in-flight onMapUpdated() callback running
        // on a worker thread can't touch a dead publisher.
        system_.reset();
    }

private:
    // ---- callbacks ----------------------------------------------------------

    void additionsCallback(const traversability_msgs::msg::KeyFrameAdditions::SharedPtr msg)
    {
        for (const auto &kf : msg->keyframes)
        {
            auto cloud = tmap_ros::toPCL(kf.kf_pointcloud);
            std::cout << "\033[32mROS NODE: Adding KF: " << kf.kf_id << "\033[0m" << std::endl;
            system_->addNewKeyFrameWithPCL(kf.kf_timestamp_in_nanosec, kf.kf_id, kf.map_id, cloud);
            // Supplies the pose AND triggers the first bin (additions register without
            // binning at the placeholder pose).
            system_->updateKeyFrame(kf.kf_id, tmap_ros::poseToAffine(kf.kf_pose));
        }
    }

    void updatesCallback(const traversability_msgs::msg::KeyFrameUpdates::SharedPtr msg)
    {
        for (const auto &kf : msg->keyframes)
            system_->updateKeyFrame(kf.kf_id, tmap_ros::poseToAffine(kf.kf_pose));
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        system_->pushToBuffer(msg);
    }

    // ---- publish ------------------------------------------------------------

    // Timer: sparse deltas only (the deployment path, steady cadence). The full
    // grid_map + occupancy publish per-recompute via onMapUpdated() instead.
    void publish()
    {
        publishSparse();
    }

    // Fired by a LocalMap worker after each grid-changing keyframe op (every
    // recompute). Runs on the worker thread; rclcpp publishers are thread-safe and
    // the worker has released the grid lock, so reading the grid here is safe.
    // Subscriber-gated inside each publish*, so it costs nothing when nobody is
    // viewing. NOTE: during a large PGO batch this serializes the full grid once per
    // keyframe -- heavy but matches the legacy real-time-morph behavior.
    void onMapUpdated()
    {
        publishFullMap();
        publishOccupancy();
    }

    void publishSparse()
    {
        auto lm = system_->getLocalMap();
        if (!lm)
            return;
        // No consumer: drain (to keep the changed-cell set bounded) and arm a full
        // snapshot so the next subscriber starts consistent.
        if (sparse_pub_->get_subscription_count() == 0)
        {
            {
                std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
                lm->takeChangedCells();
            }
            need_full_snapshot_ = true;
            return;
        }

        // Take the cell keys AND read their layer values under one grid lock so the
        // two stay consistent. The core map only hands out cell ids + the grid + the
        // lattice; this node owns navLayers_ and decides what to publish.
        const bool is_full = need_full_snapshot_;
        traversability_msgs::msg::TraversabilitySparseUpdate out;
        bool ready = false;
        {
            std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
            const std::vector<std::uint64_t> keys =
                is_full ? lm->occupiedCellKeys() : lm->takeChangedCells();
            if (is_full || !keys.empty())
            {
                tmap_ros::fillSparseUpdate(lm->getGridMap(), lm->getLattice(), navLayers_,
                                           keys, is_full, out);
                ready = true;
            }
        }
        if (!ready)
            return;
        need_full_snapshot_ = false;
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        sparse_pub_->publish(out);
    }

    void publishFullMap()
    {
        if (gridmap_pub_->get_subscription_count() == 0)
            return;
        auto lm = system_->getLocalMap();
        if (!lm)
            return;
        std::unique_ptr<grid_map_msgs::msg::GridMap> gm;
        {
            std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
            gm = grid_map::GridMapRosConverter::toMessage(lm->getGridMap(), navLayers_);
        }
        gm->header.frame_id = map_frame_;
        gm->header.stamp = now();
        gridmap_pub_->publish(*gm);
    }

    void publishOccupancy()
    {
        if (occupancy_pub_->get_subscription_count() == 0)
            return;
        auto lm = system_->getLocalMap();
        if (!lm)
            return;
        nav_msgs::msg::OccupancyGrid og;
        {
            std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
            grid_map::GridMapRosConverter::toOccupancyGrid(lm->getGridMap(), "hazard", 0.0, 1.0, og);
        }
        og.header.frame_id = map_frame_;
        og.header.stamp = now();
        occupancy_pub_->publish(og);
    }

    void globalCloudService(
        const std::shared_ptr<traversability_msgs::srv::GetGlobalPointcloud::Request> req,
        std::shared_ptr<traversability_msgs::srv::GetGlobalPointcloud::Response> /*res*/)
    {
        auto cloud = system_->getGlobalPointCloud(req->voxel_size_x, req->voxel_size_y, req->voxel_size_z);
        if (!cloud)
            return;
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*cloud, out);
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        global_cloud_pub_->publish(out);
    }

    // ---- members ------------------------------------------------------------
    std::string additions_topic_, updates_topic_, pointcloud_topic_;
    std::string slam_frame_, robot_base_frame_, lidar_frame_, map_frame_;

    // The grid layers this adapter publishes (sparse update + debug grid_map). The
    // core LocalMap is layer-agnostic; this subset is owned here on the ROS side.
    const std::vector<std::string> navLayers_ = {
        "normal_x", "normal_y", "normal_z", "slope_haz",
        "step_haz", "elevation", "roughness_haz", "hazard"};

    std::shared_ptr<tmap::System> system_;
    bool need_full_snapshot_ = true;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr additions_sub_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr updates_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::Publisher<traversability_msgs::msg::TraversabilitySparseUpdate>::SharedPtr sparse_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_pub_;
    rclcpp::Service<traversability_msgs::srv::GetGlobalPointcloud>::SharedPtr cloud_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
