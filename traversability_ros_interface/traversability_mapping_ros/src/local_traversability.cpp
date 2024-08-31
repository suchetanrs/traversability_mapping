#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include "traversability_mapping/KeyFrame.hpp" // Replace with your actual KeyFrame class header
#include "traversability_mapping/Parameters.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

class LocalTraversabilityNode : public rclcpp::Node
{
public:
    LocalTraversabilityNode() : Node("local_traversability_node")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor started.");
        // Subscribe to the PointCloud2 topic
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS().best_effort().durability_volatile();
        
        this->declare_parameter("pointcloud_topic_name", rclcpp::ParameterValue("/ouster/points"));
        this->get_parameter("pointcloud_topic_name", pointcloud_topic_name_);
        
        pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_name_, qos_profile, std::bind(&LocalTraversabilityNode::pointCloudCallback, this, std::placeholders::_1));

        // Other params
        Eigen::Translation3f translation(
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/translation/x"),
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/translation/y"),
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/translation/z"));
        Eigen::Quaternion<float> quaternion(
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/quaternion/w"),
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/quaternion/x"),
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/quaternion/y"),
            parameterInstance.getValue<float>("T_SLAMFrameToLidarFrame/quaternion/z"));
        Tbv_ = translation * quaternion;

        grid_map::GridMap gridMap_({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("map");
        gridMap_.setGeometry(grid_map::Length(2. * parameterInstance.getValue<double>("half_size_local_map"), 2. * parameterInstance.getValue<double>("half_size_local_map")), parameterInstance.getValue<double>("resolution_local_map"));
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor ended.");
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_traversability_map", 10);
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
    }

private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "PCL callback.");
        // Initialize your KeyFrame class
        auto keyframe_ = std::make_shared<traversability_mapping::KeyFrame>(1, pGridMap_, Tbv_); // Replace with your actual initialization logic
        // Call computeLocalTraversability function from your KeyFrame class

        // set pose
        Eigen::Translation3f translation(0.2f, 0.3f, 0.0f);
        Eigen::Quaternionf rotation(1.0f, 0.0f, 0.0f, 0.0f);
        Eigen::Affine3f Tmb_ = translation * rotation;
        keyframe_->setPose(Tmb_);

        auto Tmv_ = Tmb_ * Tbv_;
        sensor_msgs::msg::PointCloud2 pointCloudCorrected_;
        traversability_mapping::doTransformPCL(*msg, pointCloudCorrected_, Tmv_);
        auto pointCloudMap_ = std::make_shared<sensor_msgs::msg::PointCloud2>(pointCloudCorrected_);
        pointCloudMap_->header.frame_id = "map";

        keyframe_->computeLocalTraversability(*pointCloudMap_); // Assuming keyframe_ is a shared pointer to your KeyFrame object

        nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
        traversability_mapping::gridMapToOccupancyGrid(*pGridMap_, "hazard", 0., 1., occupancyGrid_msg);
        gridMapOccupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancyGrid_msg);
        gridMapOccupancy_->header.frame_id = static_cast<std::string>(this->get_namespace()) + "/base_footprint";
        gridMapOccupancy_->header.stamp = msg->header.stamp;
        occupancy_grid_publisher_->publish(*gridMapOccupancy_);

        auto message = *grid_map::GridMapRosConverter::toMessage(*pGridMap_);
        traversabilityPub_->publish(message);
        keyframe_->clearStrayValuesInGrid();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversabilityPub_;
    std::string pointcloud_topic_name_;
    std::shared_ptr<grid_map::GridMap> pGridMap_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridMapOccupancy_;
    Eigen::Affine3f Tbv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalTraversabilityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}