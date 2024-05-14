#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "traversability_msgs/msg/key_frame.hpp"
#include "traversability_msgs/msg/key_frame_additions.hpp"
#include "traversability_msgs/msg/key_frame_updates.hpp"

#include "traversability_mapping/System.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

class TraversabilityNode : public rclcpp::Node
{
public:
    TraversabilityNode() : Node("traversability_node")
    {
        this->declare_parameter("additions_topic_name", rclcpp::ParameterValue("/traversability_keyframe_additions"));
        this->get_parameter("additions_topic_name", additions_topic_name_);

        this->declare_parameter("updates_topic_name", rclcpp::ParameterValue("/traversability_keyframe_updates"));
        this->get_parameter("updates_topic_name", updates_topic_name_);

        this->declare_parameter("pointcloud_topic_name", rclcpp::ParameterValue("/ouster/points"));
        this->get_parameter("pointcloud_topic_name", pointcloud_topic_name_);

        this->declare_parameter("use_lidar_pointcloud", rclcpp::ParameterValue(false));
        this->get_parameter("use_lidar_pointcloud", use_lidar_pointcloud_);
        RCLCPP_WARN_STREAM(this->get_logger(), "Using lidar pointcloud? " << use_lidar_pointcloud_);

        keyFrameAdditionsSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameAdditionsCallback, this, std::placeholders::_1));

        keyFrameUpdatesSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameUpdatesCallback, this, std::placeholders::_1));

        lidarPointCloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_name_, 10, std::bind(&TraversabilityNode::pointCloudCallback, this, std::placeholders::_1));

        publishTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TraversabilityNode::publishTraversabilityData, this));

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_traversability_map", 10);
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
        pclPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kf_pointcloud", 10);
        posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_pose", 10);

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
            if (use_lidar_pointcloud_)
                traversabilitySystem_->addNewKeyFrame(keyframe.kf_timestamp_in_nanosec, keyframe.kf_id, keyframe.map_id);
            else
                traversabilitySystem_->addNewKeyFrame(keyframe.kf_timestamp_in_nanosec, keyframe.kf_id, keyframe.map_id, keyframe.kf_pointcloud);
            Eigen::Affine3d keyFramePoseEigen;
            tf2::fromMsg(keyframe.kf_pose, keyFramePoseEigen);
            traversabilitySystem_->updateKeyFrame(keyframe.kf_id, keyFramePoseEigen);

            pclPublisher_->publish(keyframe.kf_pointcloud);

            geometry_msgs::msg::PoseStamped kf_pose_stamped;
            kf_pose_stamped.header.frame_id = "world";
            kf_pose_stamped.pose = keyframe.kf_pose;
            posePublisher_->publish(kf_pose_stamped);
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

    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        traversabilitySystem_->pushToBuffer(*msg);
    }

    void publishTraversabilityData()
    {
        std::shared_ptr<grid_map::GridMap> traversabilitymap;
        std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridmap;
        auto localMap = traversabilitySystem_->getLocalMap();
        if (localMap != nullptr)
        {
            auto keyFramesMap_ = traversabilitySystem_->getLocalMap()->getKeyFramesMap();
            for (auto &pair : keyFramesMap_)
            {
                auto keyFramePtr = pair.second;

                // Check if the pointer is valid before calling recomputeCache
                if (keyFramePtr)
                {
                    auto kfPCL = keyFramePtr->getPointCloud();
                    auto trav_done = traversabilitySystem_->getLocalMap()->getGridMap();
                    auto gridmap_done = traversabilitySystem_->getLocalMap()->getOccupancyMap();
                    if (trav_done)
                    {
                        traversabilitymap = trav_done;
                        gridmap = gridmap_done;
                    }
                }
            }
            if (traversabilitymap && gridmap)
            {
                auto message = *grid_map::GridMapRosConverter::toMessage(*traversabilitymap);
                occupancy_grid_publisher_->publish(*gridmap);
                traversabilityPub_->publish(message);
                return;
            }
            else
            {
                nav_msgs::msg::OccupancyGrid temp;
                grid_map_msgs::msg::GridMap temp_grid;
                return;
            }
        }
        nav_msgs::msg::OccupancyGrid temp;
        grid_map_msgs::msg::GridMap temp_grid;
        return;
    }

    std::string additions_topic_name_;
    std::string updates_topic_name_;
    std::string pointcloud_topic_name_;
    bool use_lidar_pointcloud_;
    rclcpp::TimerBase::SharedPtr publishTimer_;

    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyFrameAdditionsSubscriber_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr keyFrameUpdatesSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPointCloudSubscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversabilityPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;

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