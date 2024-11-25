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

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

        this->declare_parameter("expected_frequency", rclcpp::ParameterValue(4.0));
        this->get_parameter("expected_frequency", expected_frequency_);

        callback_interval_ = 1.0 / expected_frequency_;

        this->declare_parameter("publish_local_gridmap", rclcpp::ParameterValue(false));
        this->get_parameter("publish_local_gridmap", publish_local_gridmap_);

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

        grid_map::GridMap gridMap_({"num_additions", "hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("map");
        gridMap_.setGeometry(grid_map::Length(2. * parameterInstance.getValue<double>("half_size_traversability"), 2. * parameterInstance.getValue<double>("half_size_traversability")), parameterInstance.getValue<double>("resolution_local_map"));
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor ended.");
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_traversability_map", rclcpp::QoS(1).transient_local());
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
        last_callback_time_ = std::chrono::high_resolution_clock::now();

        tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
        parameterInstance.setValue<bool>("use_averaging", true);
        parameterInstance.setValue<bool>("use_probabilistic_update", false);
    }

private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        pcl::PointCloud<pcl::PointXYZ> pointcloudInput;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, pointcloudInput);
        auto conversion_time = std::chrono::high_resolution_clock::now();
        auto conversion_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(conversion_time - current_time);
        std::chrono::duration<double> elapsed = current_time - last_callback_time_;
        // if (elapsed.count() < callback_interval_)
        //     return;
        last_callback_time_ = current_time;
        // RCLCPP_INFO_STREAM(this->get_logger(), "PCL callback.");
        // RCLCPP_INFO_STREAM(this->get_logger(), "Conversion took: " << conversion_elapsed.count());
        // Initialize your KeyFrame class
        std::shared_ptr<std::mutex> mapMutex_ = std::make_shared<std::mutex>();
        auto keyframe_ = std::make_shared<traversability_mapping::KeyFrame>(1, pGridMap_, mapMutex_, Tbv_); // Replace with your actual initialization logic
        // Call computeLocalTraversability function from your KeyFrame class

        // set pose
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            // transformStamped = tf_buffer_ptr_->lookupTransform("map", static_cast<std::string>(this->get_namespace()).substr(1) + "/map", tf2::TimePointZero);
            transformStamped = tf_buffer_ptr_->lookupTransform("map", "map", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from map to base_footprint: %s", ex.what());
            return;
        }
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "NEW!");
            Eigen::Translation3f translation(0.0f, 0.0f, 0.0f);
            Eigen::Quaternionf rotation(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
            Eigen::Affine3f Tmb_ = translation * rotation;
            keyframe_->setPose(Tmb_);

            auto Tmv_ = Tmb_ * Tbv_;
            pcl::PointCloud<pcl::PointXYZ> pointCloudCorrected_;
            traversability_mapping::doTransformPCL(pointcloudInput, pointCloudCorrected_, Tmv_);

            keyframe_->computeLocalTraversability(pointCloudCorrected_); // Assuming keyframe_ is a shared pointer to your KeyFrame object

            nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
            traversability_mapping::gridMapToOccupancyGrid(*pGridMap_, "hazard", 0., 1., occupancyGrid_msg);
            gridMapOccupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancyGrid_msg);
            gridMapOccupancy_->info.origin.position.x = transformStamped.transform.translation.x - parameterInstance.getValue<double>("half_size_traversability");
            gridMapOccupancy_->info.origin.position.y = transformStamped.transform.translation.y - parameterInstance.getValue<double>("half_size_traversability");
            // gridMapOccupancy_->info.origin.orientation = transformStamped.transform.rotation;
            gridMapOccupancy_->header.frame_id = "map";
            gridMapOccupancy_->header.stamp = msg->header.stamp;
            occupancy_grid_publisher_->publish(*gridMapOccupancy_);

            if (publish_local_gridmap_)
            {
                auto message = *grid_map::GridMapRosConverter::toMessage(*pGridMap_);
                traversabilityPub_->publish(message);
            }
            keyframe_->clearStrayValuesInGrid();
            // End time measurement
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            // Log the duration
            RCLCPP_INFO(this->get_logger(), "Iteration time: %ld ms", duration.count());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversabilityPub_;
    std::string pointcloud_topic_name_;
    std::shared_ptr<grid_map::GridMap> pGridMap_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridMapOccupancy_;
    Eigen::Affine3f Tbv_;
    double expected_frequency_;
    double callback_interval_;
    bool publish_local_gridmap_;
    std::chrono::high_resolution_clock::time_point last_callback_time_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_; //!< Unique pointer to a buffer of Map to BaseLink tfs
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_; //!< Shared pointer to a TransformListener, used to listen to Map To BaseLink transforms
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalTraversabilityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}