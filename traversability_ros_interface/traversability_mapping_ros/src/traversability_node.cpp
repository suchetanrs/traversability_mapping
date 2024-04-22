#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "traversability_msgs/msg/key_frame.hpp"
#include "traversability_msgs/msg/key_frame_additions.hpp"
#include "traversability_msgs/msg/key_frame_updates.hpp"

#include "traversability_mapping/System.hpp"

class TraversabilityNode : public rclcpp::Node
{
public:
    TraversabilityNode() : Node("traversability_node")
    {
        this->declare_parameter("additions_topic_name", rclcpp::ParameterValue("/traversability_keyframe_additions"));
        this->get_parameter("additions_topic_name", additions_topic_name_);

        this->declare_parameter("updates_topic_name", rclcpp::ParameterValue("/traversability_keyframe_updates"));
        this->get_parameter("updates_topic_name", updates_topic_name_);

        this->declare_parameter("pointcloud_topic_name", rclcpp::ParameterValue("/scout_2/velodyne_points"));
        this->get_parameter("pointcloud_topic_name", pointcloud_topic_name_);

        keyFrameAdditionsSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameAdditionsCallback, this, std::placeholders::_1));

        keyFrameUpdatesSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameUpdatesCallback, this, std::placeholders::_1));

        traversabilitySystem_ = std::make_shared<traversability_mapping::System>();
        traversabilitySystem_->addNewLocalMap(0);
        RCLCPP_INFO(this->get_logger(), "Added new map");
    }

private:
    void keyFrameAdditionsCallback(const traversability_msgs::msg::KeyFrameAdditions::SharedPtr msg)
    {
        // Process received KeyFrameAdditions message
        RCLCPP_INFO(this->get_logger(), "Received KeyFrameAdditions message");
        for (const auto &keyframe : msg->keyframes)
        {
            // Assuming keyframe data structure contains necessary information for mapping
            // Add the keyframe to the traversability map using pTraversability_
            traversabilitySystem_->addNewLocalMap(keyframe.map_id);
            traversabilitySystem_->addNewKeyFrame(keyframe.kf_timestamp_in_nanosec, keyframe.kf_id, keyframe.map_id);
            Eigen::Affine3d keyFramePoseEigen;
            tf2::fromMsg(keyframe.kf_pose, keyFramePoseEigen);
            traversabilitySystem_->updateKeyFrame(keyframe.kf_id, keyFramePoseEigen);
        }
    }

    void keyFrameUpdatesCallback(const traversability_msgs::msg::KeyFrameUpdates::SharedPtr msg)
    {
        // Process received KeyFrameUpdates message
        RCLCPP_INFO(this->get_logger(), "Received KeyFrameUpdates message");
        for (const auto &keyframe : msg->keyframes)
        {
            // Assuming keyframe data structure contains necessary information for mapping
            // update the keyframe to the traversability map using pTraversability_
            Eigen::Affine3d keyFramePoseEigen;
            tf2::fromMsg(keyframe.kf_pose, keyFramePoseEigen);
            traversabilitySystem_->updateKeyFrame(keyframe.kf_id, keyFramePoseEigen);
        }
    }

    std::string additions_topic_name_;
    std::string updates_topic_name_;
    std::string pointcloud_topic_name_;

    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyFrameAdditionsSubscriber_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr keyFrameUpdatesSubscriber_;

    std::shared_ptr<traversability_mapping::System> traversabilitySystem_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TraversabilityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}