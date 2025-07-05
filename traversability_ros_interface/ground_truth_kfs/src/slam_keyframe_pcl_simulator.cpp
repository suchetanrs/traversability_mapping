#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "traversability_msgs/msg/key_frame_additions.hpp"

class SLAMKeyFrameSimulator : public rclcpp::Node {
public:
    SLAMKeyFrameSimulator() : Node("slam_keyframe_pcl_simulator") {
        std::string odom_topic_;
        std::string pcl_topic_;
        this->declare_parameter("odom_topic", "ground_truth_pose");
        this->get_parameter("odom_topic", odom_topic_);
        this->declare_parameter("pcl_topic", "lidar/points");
        this->get_parameter("pcl_topic", pcl_topic_);

        this->declare_parameter("keyframe_publish_rate_hz", 2.5);
        this->get_parameter("keyframe_publish_rate_hz", keyframe_publish_rate_hz_);

        RCLCPP_INFO(this->get_logger(), "SLAMKeyFrameSimulator initialized with odom_topic: %s, pcl_topic: %s, keyframe_publish_rate_hz: %f", odom_topic_.c_str(), pcl_topic_.c_str(), keyframe_publish_rate_hz_);
        
        // Subscribers for the topics
        odom_subscriber_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_topic_);
        pcl_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pcl_topic_);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&SLAMKeyFrameSimulator::topic_callback, this, std::placeholders::_1));

        // Approximate Time Synchronization
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *odom_subscriber_, *pcl_subscriber_);
        sync_->registerCallback(std::bind(&SLAMKeyFrameSimulator::callback, this, std::placeholders::_1, std::placeholders::_2));
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        keyframe_addition_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameAdditions>("traversability_keyframe_additions", 10);

        current_kf_id_ = 0;
        prev_time = std::chrono::high_resolution_clock::now();
    }

private:
    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg) {
        rclcpp::Time time1 = odom_msg->header.stamp;
        rclcpp::Time time2 = pcl_msg->header.stamp;

        // Calculate time difference
        auto time_diff = (time1 - time2).seconds();

        // RCLCPP_INFO(this->get_logger(), "Time difference: %f seconds", time_diff);

        // Process the synchronized messages with closest timestamps
        // RCLCPP_INFO(this->get_logger(), "Synchronized messages received.");
        // RCLCPP_INFO(this->get_logger(), "Odometry timestamp: %f", odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9);
        // RCLCPP_INFO(this->get_logger(), "PointCloud2 timestamp: %f", pcl_msg->header.stamp.sec + pcl_msg->header.stamp.nanosec * 1e-9);
        
        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - prev_time;
        double dt = elapsed_time.count();

        if(dt > 1 / keyframe_publish_rate_hz_)
        {
            // RCLCPP_INFO(this->get_logger(), "Time difference: %f seconds", dt);
            ++current_kf_id_;
            // RCLCPP_WARN_STREAM(this->get_logger(), "Publishing with KF ID:" << current_kf_id_);
            auto keyframePose = convertOdomToPose(*odom_msg);
            traversability_msgs::msg::KeyFrame current_kf_;
            current_kf_.kf_timestamp_in_nanosec = (pcl_msg->header.stamp.sec * 1e9) + (pcl_msg->header.stamp.nanosec);
            current_kf_.kf_id = current_kf_id_;
            current_kf_.kf_pose = keyframePose;
            current_kf_.map_id = 0;
            current_kf_.kf_pointcloud = *pcl_msg;
            traversability_msgs::msg::KeyFrameAdditions kf_additions_msg_;
            kf_additions_msg_.keyframes.push_back(current_kf_);
            keyframe_addition_publisher_->publish(kf_additions_msg_);
            prev_time = std::chrono::high_resolution_clock::now();
        }
        // Add your processing logic here
    }

    geometry_msgs::msg::Pose convertOdomToPose(nav_msgs::msg::Odometry odomMsg)
    {
        geometry_msgs::msg::Pose poseMsg;
        poseMsg.position = odomMsg.pose.pose.position;
        poseMsg.orientation = odomMsg.pose.pose.orientation;
        return poseMsg;
    }


    void topic_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received odometry message: x: '%f', y: '%f'", msg->pose.pose.position.x, msg->pose.pose.position.y);
    
        geometry_msgs::msg::TransformStamped transform_stamped;

        // odom to base_footprint
        transform_stamped.header.stamp = msg->header.stamp;
        if(static_cast<std::string>(this->get_namespace()) == "/")
        {
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_footprint";
        }
        else
        {
            transform_stamped.header.frame_id = static_cast<std::string>(this->get_namespace()) + "/odom";
            transform_stamped.child_frame_id = static_cast<std::string>(this->get_namespace()) + "/base_footprint";
        }

        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;
        transform_stamped.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);

        // map to odom (0 transform)
        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = "map";
        if(static_cast<std::string>(this->get_namespace()) == "/")
        {
            transform_stamped.child_frame_id = "odom";
        }
        else
        {
            transform_stamped.child_frame_id = static_cast<std::string>(this->get_namespace()) + "/odom";
        }

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        geometry_msgs::msg::Quaternion default_quat;
        transform_stamped.transform.rotation = default_quat;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pcl_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyframe_addition_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    long unsigned current_kf_id_;
    std::chrono::_V2::system_clock::time_point prev_time;
    double keyframe_publish_rate_hz_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMKeyFrameSimulator>());
    rclcpp::shutdown();
    return 0;
}