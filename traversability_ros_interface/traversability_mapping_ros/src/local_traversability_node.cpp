/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * Thin ROS adapter for the LOCAL moment-fused traversability map.
 *
 * Same core (System / LocalMap / KeyFrame), driven differently: a lidar scan becomes a
 * keyframe once the robot has moved keyframe_min_displacement_m since the last one, added
 * at its synced pose and evicted after a rolling window of N keyframes.
 * The map stays a small robot-centered disc; the underlying grid buffer grows with the
 * trajectory (accepted) but is never published whole -- publish crops a fixed W x H
 * submap around the latest pose. NO PGO: each scan is binned once (clouds dropped via
 * mapping/is_kf_optimization_enabled=false); eviction subtracts the retained per-cell
 * moments exactly. The core is unchanged from the global node -- all local behavior
 * (sync, window eviction, submap crop) lives here.
 *
 *   sync   -> per scan: (cloud, pose) paired by ApproximateTime; scans within
 *             keyframe_min_displacement_m of the last keyframe are dropped. Otherwise
 *             sensor_msgs->PCL, System::addNewKeyFrameWithPCL, then
 *             System::updateKeyFrame(pose) to set the pose and trigger the (only) bin,
 *             then System::deleteKeyFrame(id - N).
 *   timer  -> cropped grid_map + occupancy around the robot (debug, subscriber-gated)
 *             and the sparse delta (deployment). Fixed nav cadence, decoupled from scans.
 */
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>

#include <traversability_msgs/msg/traversability_sparse_update.hpp>
#include <traversability_msgs/srv/get_global_pointcloud.hpp>

#include "traversability_mapping/System.hpp"
#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/Parameters.hpp"

#include "conversions.hpp"
#include "common.hpp"
#include "parameter_bridge.hpp"

namespace tmap = traversability_mapping;

class LocalTraversabilityNode : public rclcpp::Node
{
public:
    LocalTraversabilityNode() : Node("local_traversability_node")
    {
        // --- ROS params ---
        pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic_name", "lidar/points");
        odom_topic_ = declare_parameter<std::string>("odom_topic_name", "odom");
        slam_frame_ = declare_parameter<std::string>("slam_frame", "base_footprint");
        robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
        lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
        // Fixed accumulation + published frame. Frame-agnostic core: whatever frame the
        // synced poses live in (odom is recommended -- a jumping SLAM 'map' frame would
        // smear the never-rebinned window). Stamped on every published message.
        map_frame_ = declare_parameter<std::string>("map_frame", "odom");
        // Rolling window: keep the last N keyframes; keyframe id-N is evicted when id lands.
        window_size_ = static_cast<std::uint64_t>(declare_parameter<int>("window_size", 15));
        // A scan taken less than this from the last keyframe's position is dropped: it would
        // re-bin near-duplicate points into the same cells for no new information, and it
        // would push still-useful keyframes out of the window while standing still.
        // Translation only; rotation is deliberately ignored for now.
        keyframe_min_displacement_m_ = declare_parameter<double>("keyframe_min_displacement_m", 0.05);
        // Robot-centered crop published to consumers (m). Independent of the growing buffer.
        crop_size_x_ = declare_parameter<double>("crop_size_x", 15.0);
        crop_size_y_ = declare_parameter<double>("crop_size_y", 15.0);
        const std::string parameter_file_path = declare_parameter<std::string>("parameter_file_path", "");

        // Load the core params, then expose them as ROS params (introspective bridge).
        // Must precede System construction, which caches several of these values.
        tmap::ParameterHandler::getInstance(parameter_file_path);
        param_cb_handle_ = tmap_ros::bridgeCoreParameters(this);
        system_ = std::make_shared<tmap::System>();
        system_->setMapFrame(map_frame_);

        // --- Extrinsics from TF (base <- lidar; used to prune clouds into the base
        // frame). Set slam_frame == robot_base_frame so Tbs = I and Tbv = base <- lidar. ---
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        Eigen::Affine3f Tsv, Tbs;
        populateTransforms(slam_frame_, robot_base_frame_, lidar_frame_, get_clock(), get_logger(),
                           tf_buffer_, Tsv, Tbs);
        system_->setExtrinsicParameters(Tsv, Tbs);

        // Single map. No onUpdate: the local map publishes on the timer only (fixed nav
        // cadence), not per-recompute.
        system_->addNewLocalMap(0);

        // --- ROS I/O ---
        // Marry each scan to its pose by stamp. Deep sync queue: binning can briefly block
        // on the map mutex, and a shallow queue would drop scans (= lost keyframes).
        cloud_sub_.subscribe(this, pointcloud_topic_, rmw_qos_profile_default);
        odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_default);
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(100), cloud_sub_, odom_sub_);
        sync_->registerCallback(std::bind(&LocalTraversabilityNode::scanCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
            "local_traversability_gridmap", rclcpp::QoS(1).transient_local());
        occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "local_traversability_occupancy", rclcpp::QoS(1).transient_local());
        sparse_pub_ = create_publisher<traversability_msgs::msg::TraversabilitySparseUpdate>(
            "local_traversability_updates", rclcpp::QoS(10).reliable());
        window_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "local_kf_pointcloud", rclcpp::QoS(1).transient_local());

        cloud_srv_ = create_service<traversability_msgs::srv::GetGlobalPointcloud>(
            "publish_local_pointcloud",
            std::bind(&LocalTraversabilityNode::localCloudService, this,
                      std::placeholders::_1, std::placeholders::_2));

        const double rate = tmap::ParameterHandler::getInstance().getValue<double>("node/publish_rate_hz");
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(0.1, rate)),
            std::bind(&LocalTraversabilityNode::publish, this));

        RCLCPP_INFO(get_logger(),
                    "Local traversability adapter ready (window=%lu keyframes, "
                    "min displacement=%.3f m).",
                    static_cast<unsigned long>(window_size_), keyframe_min_displacement_m_);
    }

    ~LocalTraversabilityNode() override
    {
        // Join the LocalMap workers before the publishers die (same rationale as global).
        system_.reset();
    }

private:
    // ---- callbacks ----------------------------------------------------------

    // One synced (cloud, pose) pair -> at most one keyframe. Scans that have not moved far
    // enough since the last keyframe are dropped. Otherwise add + bin at the pose, then
    // evict the keyframe that just fell out of the window.
    void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        const auto &p = odom_msg->pose.pose.position;
        const Eigen::Vector3d pos(p.x, p.y, p.z);

        // The crop follows the robot on every scan, including dropped ones.
        robot_x_ = p.x;
        robot_y_ = p.y;
        have_pose_ = true;

        // Displacement gate. The first scan always lands; after that a scan only becomes a
        // keyframe once the robot has moved far enough. ids stay contiguous because
        // next_id_ advances only here -- the window eviction below relies on that.
        if (have_last_kf_pose_ && (pos - last_kf_pos_).norm() < keyframe_min_displacement_m_)
            return;
        last_kf_pos_ = pos;
        have_last_kf_pose_ = true;

        const std::uint64_t id = next_id_++;
        const auto ts = static_cast<unsigned long long>(
            rclcpp::Time(cloud_msg->header.stamp).nanoseconds());
        auto cloud = tmap_ros::toPCL(*cloud_msg);

        system_->addNewKeyFrameWithPCL(ts, id, 0, cloud);
        // Supplies the pose AND triggers the (only) bin.
        system_->updateKeyFrame(id, tmap_ros::poseToAffine(odom_msg->pose.pose));

        // Rolling window: once N+1 are in flight, drop the oldest (subtracts its retained
        // moments exactly, even though its cloud was dropped after the first bin).
        if (id >= window_size_)
            system_->deleteKeyFrame(id - window_size_);
    }

    // ---- publish (fixed nav cadence) ----------------------------------------

    void publish()
    {
        publishCroppedMap();
        publishCroppedOccupancy();
        publishSparse();
    }

    // Copy a fixed W x H submap around the robot out of the (growing) grid under the lock,
    // then serialize the small copy outside it.
    bool cropAroundRobot(grid_map::GridMap &out)
    {
        if (!have_pose_)
            return false;
        auto lm = system_->getLocalMap();
        if (!lm)
            return false;
        const grid_map::Position center(robot_x_, robot_y_);
        const grid_map::Length length(crop_size_x_, crop_size_y_);
        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
            out = lm->getGridMap().getSubmap(center, length, ok);
        }
        return ok;
    }

    void publishCroppedMap()
    {
        if (gridmap_pub_->get_subscription_count() == 0)
            return;
        grid_map::GridMap sub;
        if (!cropAroundRobot(sub))
            return;
        auto gm = grid_map::GridMapRosConverter::toMessage(sub, navLayers_);
        gm->header.frame_id = map_frame_;
        gm->header.stamp = now();
        gridmap_pub_->publish(*gm);
    }

    void publishCroppedOccupancy()
    {
        if (occupancy_pub_->get_subscription_count() == 0)
            return;
        grid_map::GridMap sub;
        if (!cropAroundRobot(sub))
            return;
        nav_msgs::msg::OccupancyGrid og;
        grid_map::GridMapRosConverter::toOccupancyGrid(sub, "hazard", 0.0, 1.0, og);
        og.header.frame_id = map_frame_;
        og.header.stamp = now();
        occupancy_pub_->publish(og);
    }

    // Sparse delta: changed cells only (births ahead + deaths behind, NaN = clear), so a
    // consumer self-rolls its own window. Not cropped -- the delta is already bounded by
    // per-tick window activity. Identical semantics to the global node.
    void publishSparse()
    {
        auto lm = system_->getLocalMap();
        if (!lm)
            return;
        if (sparse_pub_->get_subscription_count() == 0)
        {
            {
                std::lock_guard<std::mutex> lock(lm->getGridMapMutex());
                lm->takeChangedCells();
            }
            need_full_snapshot_ = true;
            return;
        }

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

    // On request, stitch the raw base-frame clouds of the keyframes currently in the
    // window at their poses and publish them (map frame). This is the window's input
    // points, NOT the fused map: the cells' moments are not reconstructed from it. The
    // clouds exist only because mapping/is_kf_optimization_enabled is true; with it off
    // the core drops each cloud after its first bin and this returns empty.
    void localCloudService(
        const std::shared_ptr<traversability_msgs::srv::GetGlobalPointcloud::Request> req,
        std::shared_ptr<traversability_msgs::srv::GetGlobalPointcloud::Response> /*res*/)
    {
        auto cloud = system_->getGlobalPointCloud(req->voxel_size_x, req->voxel_size_y, req->voxel_size_z);
        if (!cloud)
            return;
        if (cloud->empty())
        {
            RCLCPP_WARN(get_logger(),
                        "publish_local_pointcloud: no keyframe retained a cloud "
                        "(mapping/is_kf_optimization_enabled off?); nothing published.");
            return;
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*cloud, out);
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        window_cloud_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "publish_local_pointcloud: %zu points from the window.",
                    cloud->size());
    }

    // ---- members ------------------------------------------------------------
    std::string pointcloud_topic_, odom_topic_;
    std::string slam_frame_, robot_base_frame_, lidar_frame_, map_frame_;
    std::uint64_t window_size_ = 15;
    double crop_size_x_ = 15.0, crop_size_y_ = 15.0;
    double keyframe_min_displacement_m_ = 0.05;

    // The grid layers this adapter publishes (sparse update + debug grid_map). The core
    // LocalMap is layer-agnostic; this subset is owned here on the ROS side.
    const std::vector<std::string> navLayers_ = {
        "normal_x", "normal_y", "normal_z", "slope_haz",
        "step_haz", "elevation", "roughness_haz", "hazard"};

    std::shared_ptr<tmap::System> system_;
    std::uint64_t next_id_ = 0;
    bool need_full_snapshot_ = true;
    bool have_pose_ = false;
    double robot_x_ = 0.0, robot_y_ = 0.0;
    bool have_last_kf_pose_ = false;
    Eigen::Vector3d last_kf_pos_ = Eigen::Vector3d::Zero();  ///< position of the last accepted keyframe

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<Synchronizer> sync_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::Publisher<traversability_msgs::msg::TraversabilitySparseUpdate>::SharedPtr sparse_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr window_cloud_pub_;
    rclcpp::Service<traversability_msgs::srv::GetGlobalPointcloud>::SharedPtr cloud_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalTraversabilityNode>());
    rclcpp::shutdown();
    return 0;
}
