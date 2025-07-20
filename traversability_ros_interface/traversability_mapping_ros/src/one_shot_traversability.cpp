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

#include "common.hpp"

class OneShotTraversability : public rclcpp::Node
{
public:
    OneShotTraversability() : Node("local_traversability_node")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor started.");
        // Subscribe to the PointCloud2 topic
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS().best_effort().durability_volatile();

        std::string parameter_file_path_;
        this->declare_parameter("parameter_file_path", rclcpp::ParameterValue(""));
        this->get_parameter("parameter_file_path", parameter_file_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameter file path: " << parameter_file_path_);

        ParameterHandler::getInstance(parameter_file_path_);

        pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kf_pointcloud", 1, std::bind(&OneShotTraversability::pointCloudCallback, this, std::placeholders::_1));

        tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_, this, true);

        populateTransforms("map", "map", "map", this->get_clock(), this->get_logger(), tf_buffer_ptr_, Tsv_, Tbs_);

        grid_map::GridMap gridMap_({"num_additions", "hazard", "step_haz", "roughness_haz", "slope_haz", "border_haz", "elevation", "kfid"});
        gridMap_.setFrameId("map");
        gridMap_.setGeometry(grid_map::Length(2. * parameterInstance.getValue<double>("half_size_traversability"), 2. * parameterInstance.getValue<double>("half_size_traversability")), parameterInstance.getValue<double>("resolution_local_map"));
        pGridMap_ = std::make_shared<grid_map::GridMap>(gridMap_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Constructor ended.");
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_traversability_map", rclcpp::QoS(1).transient_local());
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());

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
        // RCLCPP_INFO_STREAM(this->get_logger(), "PCL callback.");
        // RCLCPP_INFO_STREAM(this->get_logger(), "Conversion took: " << conversion_elapsed.count());
        // Initialize your KeyFrame class
        std::shared_ptr<std::mutex> mapMutex_ = std::make_shared<std::mutex>();
        auto keyframe_ = std::make_shared<traversability_mapping::KeyFrame>(1, pGridMap_, mapMutex_, Tsv_, Tbs_); // Replace with your actual initialization logic
        // Call computeLocalTraversability function from your KeyFrame class

        // set pose
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            // transformStamped = tf_buffer_ptr_->lookupTransform("odom", static_cast<std::string>(this->get_namespace()).substr(1) + "/map", tf2::TimePointZero);
            transformStamped = tf_buffer_ptr_->lookupTransform(msg->header.frame_id, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Could not transform from map to " << "map" << ex.what());
            return;
        }
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "NEW!");
            Eigen::Translation3f translation(0.0f, 0.0f, 0.0f);
            Eigen::Quaternionf rotation(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
            Eigen::Affine3f Tms_ = translation * rotation;
            keyframe_->setPose(Tms_);

            auto Tmv = Tms_ * Tsv_;
            auto Tmb = Tms_ * Tbs_.inverse();
            pcl::PointCloud<pcl::PointXYZ> pointCloudCorrected_;
            traversability_mapping::doTransformPCL(pointcloudInput, pointCloudCorrected_, Tmv);

            keyframe_->computeLocalTraversability(pointCloudCorrected_, Tmb); // Assuming keyframe_ is a shared pointer to your KeyFrame object

            nav_msgs::msg::OccupancyGrid occupancyGrid_msg;
            traversability_mapping::gridMapToOccupancyGrid(*pGridMap_, "hazard", 0., 1., occupancyGrid_msg);
            gridMapOccupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occupancyGrid_msg);
            gridMapOccupancy_->info.origin.position.x = transformStamped.transform.translation.x - parameterInstance.getValue<double>("half_size_traversability");
            gridMapOccupancy_->info.origin.position.y = transformStamped.transform.translation.y - parameterInstance.getValue<double>("half_size_traversability");
            // gridMapOccupancy_->info.origin.orientation = transformStamped.transform.rotation;
            gridMapOccupancy_->header.frame_id = "map";
            gridMapOccupancy_->header.stamp = msg->header.stamp;
            occupancy_grid_publisher_->publish(*gridMapOccupancy_);
            auto message = *grid_map::GridMapRosConverter::toMessage(*pGridMap_);
            traversabilityPub_->publish(message);
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
    Eigen::Affine3f Tsv_;
    Eigen::Affine3f Tbs_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_; //!< shared pointer to a buffer of Map to BaseLink tfs
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_; //!< Shared pointer to a TransformListener, used to listen to Map To BaseLink transforms
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OneShotTraversability>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}