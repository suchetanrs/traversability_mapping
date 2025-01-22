#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "traversability_msgs/msg/key_frame_additions.hpp"

class SLAMKeyFrameSimulator : public rclcpp::Node
{
public:
    SLAMKeyFrameSimulator()
        : Node("slam_keyframe_simulator")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth_pose", 10, std::bind(&SLAMKeyFrameSimulator::topic_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&SLAMKeyFrameSimulator::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        keyframe_addition_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameAdditions>("traversability_keyframe_additions", 10);

        // keyframe_updates_publisher_ = this->create_publisher<traversability_msgs::msg::KeyFrameUpdates>("traversability_keyframe_updates", 10);
        
        latest_odometry_message_ = nullptr;
        current_kf_id_ = 0;
    }

private:
    void topic_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odometry_msg_lock_);
        latest_odometry_message_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Received odometry message: x: '%f', y: '%f'", msg->pose.pose.position.x, msg->pose.pose.position.y);
    
        geometry_msgs::msg::TransformStamped transform_stamped;

        // odom to base_link
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

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(odometry_msg_lock_);
        if(latest_odometry_message_)
        {
            ++current_kf_id_;
            RCLCPP_WARN_STREAM(this->get_logger(), "Publishing KF ID:" << current_kf_id_);
            auto keyframePose = convertOdomToPose(*latest_odometry_message_);
            traversability_msgs::msg::KeyFrame current_kf_;
            current_kf_.kf_timestamp_in_nanosec = (latest_odometry_message_->header.stamp.sec * 1e9) + (latest_odometry_message_->header.stamp.nanosec);
            current_kf_.kf_id = current_kf_id_;
            current_kf_.kf_pose = keyframePose;
            current_kf_.map_id = 0;
            traversability_msgs::msg::KeyFrameAdditions kf_additions_msg_;
            kf_additions_msg_.keyframes.push_back(current_kf_);
            keyframe_addition_publisher_->publish(kf_additions_msg_);
        }
    }

    geometry_msgs::msg::Pose convertOdomToPose(nav_msgs::msg::Odometry odomMsg)
    {
        geometry_msgs::msg::Pose poseMsg;
        poseMsg.position = odomMsg.pose.pose.position;
        poseMsg.orientation = odomMsg.pose.pose.orientation;
        return poseMsg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyframe_addition_publisher_;
    // rclcpp::Publisher<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr keyframe_updates_publisher_;
    std::mutex odometry_msg_lock_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_message_;
    long unsigned current_kf_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMKeyFrameSimulator>());
    rclcpp::shutdown();
    return 0;
}