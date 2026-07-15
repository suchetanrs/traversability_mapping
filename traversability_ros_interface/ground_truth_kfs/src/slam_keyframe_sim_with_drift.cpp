#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "traversability_msgs/msg/key_frame_additions.hpp"
#include "traversability_msgs/msg/key_frame_updates.hpp"

#include <atomic>
#include <cmath>
#include <vector>

// SLAM keyframe simulator that injects accumulating pose drift. Each kept
// keyframe is published on the additions topic with a corrupted pose (the true
// pose plus a drift that grows linearly with the number of keyframes since the
// last loop closure). The true poses are retained so that, on a "trigger_lc_pgo"
// service call, every keyframe's corrected (drift-free) pose is republished at
// once on the updates topic and the drift accumulators reset to zero -- exactly
// what a loop-closure + PGO event would produce for a real SLAM system.
class SLAMKeyFrameSimulator : public rclcpp::Node {
public:
    SLAMKeyFrameSimulator() : Node("slam_keyframe_pcl_simulator") {
        std::string pose_topic;
        std::string pcl_topic;
        this->declare_parameter("odom_topic", "ground_truth_pose");
        this->get_parameter("odom_topic", pose_topic);
        this->declare_parameter("pcl_topic", "lidar/points");
        this->get_parameter("pcl_topic", pcl_topic);
        this->declare_parameter("pose_type", "odom");  // "odom" or "pose_stamped"
        this->get_parameter("pose_type", pose_type_);
        this->declare_parameter("keyframe_publish_rate_hz", 2.5);
        this->get_parameter("keyframe_publish_rate_hz", keyframe_publish_rate_hz_);

        // Drift injected per kept keyframe. The error accumulates: the N-th
        // keyframe since the last loop closure carries N times these values.
        this->declare_parameter("per_keyframe_translation_error_m", 0.05);
        this->get_parameter("per_keyframe_translation_error_m", per_kf_trans_err_m_);
        this->declare_parameter("per_keyframe_yaw_error_deg", 1.0);
        this->get_parameter("per_keyframe_yaw_error_deg", per_kf_yaw_err_deg_);

        // A new keyframe is only kept once the robot has moved more than this
        // distance from the last kept keyframe (in addition to the rate limit).
        this->declare_parameter("keyframe_min_displacement_m", 0.05);
        this->get_parameter("keyframe_min_displacement_m", min_displacement_m_);

        RCLCPP_INFO(this->get_logger(),
            "SLAMKeyFrameSimulator (with drift) initialized: pose_topic=%s, pcl_topic=%s, pose_type=%s, rate=%.2f hz, "
            "trans_err=%.3f m/kf, yaw_err=%.3f deg/kf, min_disp=%.3f m",
            pose_topic.c_str(), pcl_topic.c_str(), pose_type_.c_str(), keyframe_publish_rate_hz_,
            per_kf_trans_err_m_, per_kf_yaw_err_deg_, min_displacement_m_);

        pcl_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pcl_topic);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        keyframe_addition_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameAdditions>("traversability_keyframe_additions", 10);
        keyframe_update_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameUpdates>("traversability_keyframe_updates", 10);

        trigger_lc_pgo_service_ = this->create_service<std_srvs::srv::SetBool>(
            "trigger_lc_pgo",
            std::bind(&SLAMKeyFrameSimulator::triggerLoopClosure, this, std::placeholders::_1, std::placeholders::_2));

        // Toggle keyframe mapping on/off at runtime. When disabled, no keyframe
        // additions/updates are published; TF broadcasting is unaffected.
        enable_mapping_service_ = this->create_service<std_srvs::srv::SetBool>(
            "enable_mapping",
            std::bind(&SLAMKeyFrameSimulator::setMappingEnabled, this, std::placeholders::_1, std::placeholders::_2));

        if (pose_type_ == "pose_stamped") {
            pose_subscriber_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, pose_topic);
            pose_sync_ = std::make_shared<message_filters::Synchronizer<PoseSyncPolicy>>(PoseSyncPolicy(10), *pose_subscriber_, *pcl_subscriber_);
            pose_sync_->registerCallback(std::bind(&SLAMKeyFrameSimulator::poseCallback, this, std::placeholders::_1, std::placeholders::_2));
            pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                pose_topic, 10, std::bind(&SLAMKeyFrameSimulator::poseTopicCallback, this, std::placeholders::_1));
        } else {
            odom_subscriber_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, pose_topic);
            odom_sync_ = std::make_shared<message_filters::Synchronizer<OdomSyncPolicy>>(OdomSyncPolicy(10), *odom_subscriber_, *pcl_subscriber_);
            odom_sync_->registerCallback(std::bind(&SLAMKeyFrameSimulator::odomCallback, this, std::placeholders::_1, std::placeholders::_2));
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                pose_topic, 10, std::bind(&SLAMKeyFrameSimulator::odomTopicCallback, this, std::placeholders::_1));
        }

        current_kf_id_ = 0;
        prev_time = std::chrono::high_resolution_clock::now();
    }

private:
    using OdomSyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
    using PoseSyncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, sensor_msgs::msg::PointCloud2>;

    // A kept keyframe, holding the original drift-free pose so it can be
    // republished as a correction when a loop closure is triggered.
    struct StoredKeyFrame {
        std::uint64_t kf_id;
        std::uint64_t kf_timestamp_in_nanosec;
        std::uint64_t map_id;
        geometry_msgs::msg::Pose true_pose;
    };

    // Yaw (rotation about global Z) of a quaternion.
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // Pre-multiply q by a rotation of `yaw` radians about the global Z axis.
    static geometry_msgs::msg::Quaternion applyYaw(const geometry_msgs::msg::Quaternion &q, double yaw) {
        const double s = std::sin(0.5 * yaw);
        const double c = std::cos(0.5 * yaw);
        geometry_msgs::msg::Quaternion out;
        out.w = c * q.w - s * q.z;
        out.x = c * q.x - s * q.y;
        out.y = c * q.y + s * q.x;
        out.z = c * q.z + s * q.w;
        return out;
    }

    // Build the corrupted pose for a keyframe by adding the currently
    // accumulated drift to the true pose.
    geometry_msgs::msg::Pose applyDrift(const geometry_msgs::msg::Pose &true_pose) const {
        geometry_msgs::msg::Pose drifted = true_pose;
        drifted.position.x += accumulated_trans_err_x_;
        drifted.position.y += accumulated_trans_err_y_;
        // Drift is planar: z is untouched, only yaw + in-plane translation drift.
        drifted.orientation = applyYaw(true_pose.orientation, accumulated_yaw_err_rad_);
        return drifted;
    }

    void publishKeyframe(const geometry_msgs::msg::Pose &pose, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        if (!mapping_enabled_.load()) return;  // mapping disabled: skip additions (TF still published)

        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - prev_time;
        if (elapsed.count() < 1.0 / keyframe_publish_rate_hz_) return;

        // Displacement gate: only keep a keyframe once the robot has moved more
        // than the threshold from the last kept keyframe (true position).
        if (have_last_kf_position_) {
            const double dx = pose.position.x - last_kf_position_x_;
            const double dy = pose.position.y - last_kf_position_y_;
            const double dz = pose.position.z - last_kf_position_z_;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < min_displacement_m_) return;
        }

        // Accumulate drift for this keyframe. Translation error grows along the
        // robot's true travel heading; yaw error grows monotonically.
        const double true_yaw = yawFromQuaternion(pose.orientation);
        accumulated_trans_err_x_ += per_kf_trans_err_m_ * std::cos(true_yaw);
        accumulated_trans_err_y_ += per_kf_trans_err_m_ * std::sin(true_yaw);
        accumulated_yaw_err_rad_ += per_kf_yaw_err_deg_ * M_PI / 180.0;

        ++current_kf_id_;
        const std::uint64_t timestamp_ns =
            static_cast<std::uint64_t>(pcl_msg->header.stamp.sec) * 1000000000ULL + pcl_msg->header.stamp.nanosec;

        // Retain the drift-free pose so a future loop closure can correct it.
        StoredKeyFrame stored;
        stored.kf_id = current_kf_id_;
        stored.kf_timestamp_in_nanosec = timestamp_ns;
        stored.map_id = 0;
        stored.true_pose = pose;
        stored_keyframes_.push_back(stored);

        traversability_msgs::msg::KeyFrame kf;
        kf.kf_timestamp_in_nanosec = timestamp_ns;
        kf.kf_id = current_kf_id_;
        kf.kf_pose = applyDrift(pose);
        kf.map_id = 0;
        kf.kf_pointcloud = *pcl_msg;
        traversability_msgs::msg::KeyFrameAdditions additions;
        additions.keyframes.push_back(kf);
        keyframe_addition_publisher_->publish(additions);

        last_kf_position_x_ = pose.position.x;
        last_kf_position_y_ = pose.position.y;
        last_kf_position_z_ = pose.position.z;
        have_last_kf_position_ = true;
        prev_time = std::chrono::high_resolution_clock::now();
    }

    // Loop-closure + PGO trigger: republish every keyframe's corrected
    // (drift-free) pose on the updates topic, then reset the drift accumulators
    // so subsequent keyframes start drifting again from zero.
    void triggerLoopClosure(const std::shared_ptr<std_srvs::srv::SetBool::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!mapping_enabled_.load()) {  // mapping disabled: publish no keyframe updates
            response->success = false;
            response->message = "Mapping disabled; no keyframe updates published.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        traversability_msgs::msg::KeyFrameUpdates updates;
        updates.keyframes.reserve(stored_keyframes_.size());
        for (const auto &stored : stored_keyframes_) {
            traversability_msgs::msg::KeyFrame kf;
            kf.kf_timestamp_in_nanosec = stored.kf_timestamp_in_nanosec;
            kf.kf_id = stored.kf_id;
            kf.kf_pose = stored.true_pose;  // corrected pose, no drift
            kf.map_id = stored.map_id;
            // Pose-only update: leave kf_pointcloud empty.
            updates.keyframes.push_back(kf);
        }
        keyframe_update_publisher_->publish(updates);

        accumulated_trans_err_x_ = 0.0;
        accumulated_trans_err_y_ = 0.0;
        accumulated_yaw_err_rad_ = 0.0;

        RCLCPP_INFO(this->get_logger(),
            "trigger_lc_pgo: published %zu corrected keyframe poses, drift accumulators reset.",
            updates.keyframes.size());

        response->success = true;
        response->message = "Published " + std::to_string(updates.keyframes.size()) +
                            " corrected keyframe poses; drift reset.";
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        // RCLCPP_INFO(this->get_logger(), "Sync time difference (odom - pcl): %.6f seconds",
        //             (rclcpp::Time(odom_msg->header.stamp) - rclcpp::Time(pcl_msg->header.stamp)).seconds());
        geometry_msgs::msg::Pose pose;
        pose.position = odom_msg->pose.pose.position;
        pose.orientation = odom_msg->pose.pose.orientation;
        publishKeyframe(pose, pcl_msg);
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        RCLCPP_INFO(this->get_logger(), "Sync time difference (pose - pcl): %.6f seconds",
                    (rclcpp::Time(pose_msg->header.stamp) - rclcpp::Time(pcl_msg->header.stamp)).seconds());
        publishKeyframe(pose_msg->pose, pcl_msg);
    }

    // Enable (data=true) or disable (data=false) keyframe mapping. Affects only keyframe
    // additions/updates; TF broadcasting continues regardless.
    void setMappingEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        mapping_enabled_.store(request->data);
        response->success = true;
        response->message = request->data ? "Keyframe mapping ENABLED."
                                          : "Keyframe mapping DISABLED (TF still publishing).";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void broadcastTF(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp) {
        std::string ns = static_cast<std::string>(this->get_namespace());
        geometry_msgs::msg::TransformStamped tf;

        // odom -> base_footprint
        tf.header.stamp = stamp;
        tf.header.frame_id = (ns == "/") ? "odom" : ns + "/odom";
        tf.child_frame_id  = (ns == "/") ? "base_footprint" : ns + "/base_footprint";
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        // map -> odom (identity)
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id  = (ns == "/") ? "odom" : ns + "/odom";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = geometry_msgs::msg::Quaternion();
        tf_broadcaster_->sendTransform(tf);
    }

    void odomTopicCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::Pose pose;
        pose.position = msg->pose.pose.position;
        pose.orientation = msg->pose.pose.orientation;
        broadcastTF(pose, msg->header.stamp);
    }

    void poseTopicCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        broadcastTF(msg->pose, msg->header.stamp);
    }

    std::string pose_type_;
    double keyframe_publish_rate_hz_;
    double per_kf_trans_err_m_;
    double per_kf_yaw_err_deg_;
    double min_displacement_m_;
    long unsigned current_kf_id_;
    std::chrono::_V2::system_clock::time_point prev_time;
    std::atomic<bool> mapping_enabled_{true};  ///< toggled by the enable_mapping service

    // Drift accumulators (map frame), reset on loop closure.
    double accumulated_trans_err_x_ = 0.0;
    double accumulated_trans_err_y_ = 0.0;
    double accumulated_yaw_err_rad_ = 0.0;

    // Displacement gate state (last kept keyframe true position).
    bool have_last_kf_position_ = false;
    double last_kf_position_x_ = 0.0;
    double last_kf_position_y_ = 0.0;
    double last_kf_position_z_ = 0.0;

    std::vector<StoredKeyFrame> stored_keyframes_;

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subscriber_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pcl_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<OdomSyncPolicy>> odom_sync_;
    std::shared_ptr<message_filters::Synchronizer<PoseSyncPolicy>> pose_sync_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyframe_addition_publisher_;
    rclcpp::Publisher<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr keyframe_update_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trigger_lc_pgo_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_mapping_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMKeyFrameSimulator>());
    rclcpp::shutdown();
    return 0;
}
