#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

inline void populateTransforms(std::string slam_frame_id, std::string robot_base_frame_id, std::string lidar_frame_id, 
    rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    bool tf1_found = false;
    bool tf2_found = false;

    while(!tf1_found)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            // TimePointZero = latest static
            tf_stamped = tf_buffer->lookupTransform(
                slam_frame_id,
                lidar_frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0));  
            tf1_found = true;
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(logger,
                "Could not get %s→%s : %s",
                slam_frame_id.c_str(), lidar_frame_id.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
            continue;
        }

        RCLCPP_INFO_STREAM(logger, "Got transform from " << slam_frame_id << " to " << lidar_frame_id << 
            "x: " << tf_stamped.transform.translation.x << " y: " << tf_stamped.transform.translation.y << " z: " << tf_stamped.transform.translation.z <<
            "qx: " << tf_stamped.transform.rotation.x << " qy: " << tf_stamped.transform.rotation.y << " qz: " << tf_stamped.transform.rotation.z << " qw: " << tf_stamped.transform.rotation.w);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/translation/x", tf_stamped.transform.translation.x);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/translation/y", tf_stamped.transform.translation.y);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/translation/z", tf_stamped.transform.translation.z);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/quaternion/w", tf_stamped.transform.rotation.w);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/quaternion/x", tf_stamped.transform.rotation.x);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/quaternion/y", tf_stamped.transform.rotation.y);
            parameterInstance.setValue<float>("T_SLAMFrameToLidarFrame/quaternion/z", tf_stamped.transform.rotation.z);
    }

    while(!tf2_found)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            // TimePointZero = latest static
            tf_stamped = tf_buffer->lookupTransform(
                robot_base_frame_id,
                slam_frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0));
            tf2_found = true;
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(logger,
            "Could not get %s→%s : %s",
            robot_base_frame_id.c_str(), slam_frame_id.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
            continue;
        }

        RCLCPP_INFO_STREAM(logger, "Got transform from " << robot_base_frame_id << " to " << slam_frame_id <<
        "x: " << tf_stamped.transform.translation.x << " y: " << tf_stamped.transform.translation.y << " z: " << tf_stamped.transform.translation.z <<
        "qx: " << tf_stamped.transform.rotation.x << " qy: " << tf_stamped.transform.rotation.y << " qz: " << tf_stamped.transform.rotation.z << " qw: " << tf_stamped.transform.rotation.w);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/translation/x", tf_stamped.transform.translation.x);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/translation/y", tf_stamped.transform.translation.y);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/translation/z", tf_stamped.transform.translation.z);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/quaternion/w", tf_stamped.transform.rotation.w);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/quaternion/x", tf_stamped.transform.rotation.x);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/quaternion/y", tf_stamped.transform.rotation.y);
        parameterInstance.setValue<float>("T_BasefootprintToSLAM/quaternion/z", tf_stamped.transform.rotation.z);
    }
}
