#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include "traversability_mapping/KeyFrame.hpp" // Replace with your actual KeyFrame class header
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

class LocalTraversabilityNode : public rclcpp::Node
{
public:
    LocalTraversabilityNode() : Node("traversability_node")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor started.");
        // Subscribe to the PointCloud2 topic
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS().best_effort().durability_volatile();
        pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", qos_profile, std::bind(&LocalTraversabilityNode::pointCloudCallback, this, std::placeholders::_1));

        // Load YAML file and retrieve parameters
        YAML::Node loaded_node = YAML::LoadFile("/usr/local/params/traversabilityParams.yaml");
        
        // Traversability Params
        half_size_gridmap_ = loaded_node["half_size_local_map"].as<double>();
        
        loadedKFParams_.resolution_ = loaded_node["resolution_local_map"].as<double>();
        resolution_ = loadedKFParams_.resolution_;
        loadedKFParams_.half_size_traversability_ = loaded_node["half_size_traversability"].as<double>();
        loadedKFParams_.security_distance_ = loaded_node["security_distance"].as<double>();
        loadedKFParams_.ground_clearance_ = loaded_node["ground_clearance"].as<double>();
        loadedKFParams_.max_slope_ = loaded_node["max_slope"].as<double>();
        loadedKFParams_.robot_height_ = loaded_node["robot_height"].as<double>();

        // Other params
        Eigen::Translation3f translation(
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["x"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["y"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["translation"]["z"].as<float>());
        Eigen::Quaternion<float> quaternion(
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["w"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["x"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["y"].as<float>(),
            loaded_node["T_SLAMFrameToLidarFrame"]["quaternion"]["z"].as<float>());
        Tbv_ = translation * quaternion;

        grid_map::GridMap gridMap_({"hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("os_sensor");
        gridMap_.setGeometry(grid_map::Length(2. * half_size_gridmap_, 2. * half_size_gridmap_), resolution_);
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor ended.");
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_traversability_map", 10);
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
    }

private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "PCL callback.");
        // Initialize your KeyFrame class
        auto keyframe_ = std::make_shared<traversability_mapping::KeyFrame>(1, pGridMap_, Tbv_, loadedKFParams_); // Replace with your actual initialization logic
        // Call computeLocalTraversability function from your KeyFrame class

        //set pose
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
        gridMapOccupancy_->header.frame_id = "os_sensor";
        gridMapOccupancy_->header.stamp = msg->header.stamp;
        occupancy_grid_publisher_->publish(*gridMapOccupancy_);

        auto message = *grid_map::GridMapRosConverter::toMessage(*pGridMap_);
        traversabilityPub_->publish(message);
        keyframe_->clearStrayValuesInGrid();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversabilityPub_;
    std::shared_ptr<grid_map::GridMap> pGridMap_;
    double half_size_gridmap_ = 17.0;
    double resolution_ = 0.25;
    // Traversability Params
    traversability_mapping::KeyFrameParameters loadedKFParams_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridMapOccupancy_;
    Eigen::Affine3f Tbv_;
    std::string SLAMSystem_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalTraversabilityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}