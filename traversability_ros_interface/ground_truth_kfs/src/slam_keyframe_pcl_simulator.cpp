#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "traversability_msgs/msg/key_frame_additions.hpp"

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

        RCLCPP_INFO(this->get_logger(),
            "SLAMKeyFrameSimulator initialized: pose_topic=%s, pcl_topic=%s, pose_type=%s, rate=%.2f hz",
            pose_topic.c_str(), pcl_topic.c_str(), pose_type_.c_str(), keyframe_publish_rate_hz_);

        pcl_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pcl_topic);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        keyframe_addition_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameAdditions>("traversability_keyframe_additions", 10);

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

    void publishKeyframe(const geometry_msgs::msg::Pose &pose, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - prev_time;
        if (elapsed.count() < 1.0 / keyframe_publish_rate_hz_) return;

        ++current_kf_id_;
        traversability_msgs::msg::KeyFrame kf;
        kf.kf_timestamp_in_nanosec = (pcl_msg->header.stamp.sec * 1e9) + pcl_msg->header.stamp.nanosec;
        kf.kf_id = current_kf_id_;
        kf.kf_pose = pose;
        kf.map_id = 0;
        kf.kf_pointcloud = *pcl_msg;
        traversability_msgs::msg::KeyFrameAdditions additions;
        additions.keyframes.push_back(kf);
        keyframe_addition_publisher_->publish(additions);
        prev_time = std::chrono::high_resolution_clock::now();
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        RCLCPP_INFO(this->get_logger(), "Sync time difference (odom - pcl): %.6f seconds",
                    (rclcpp::Time(odom_msg->header.stamp) - rclcpp::Time(pcl_msg->header.stamp)).seconds());
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
    long unsigned current_kf_id_;
    std::chrono::_V2::system_clock::time_point prev_time;

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subscriber_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pcl_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<OdomSyncPolicy>> odom_sync_;
    std::shared_ptr<message_filters::Synchronizer<PoseSyncPolicy>> pose_sync_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyframe_addition_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMKeyFrameSimulator>());
    rclcpp::shutdown();
    return 0;
}
