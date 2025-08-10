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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common.hpp"

#include "traversability_msgs/srv/get_global_pointcloud.hpp"

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

        this->declare_parameter("publish_traversability_grid", rclcpp::ParameterValue(false));
        this->get_parameter("publish_traversability_grid", publish_traversability_grid_);

        this->declare_parameter("slam_frame", rclcpp::ParameterValue("camera_link"));
        this->get_parameter("slam_frame", slam_frame_id_);

        this->declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("lidar_frame", rclcpp::ParameterValue("lidar_link"));
        this->get_parameter("lidar_frame", lidar_frame_id_);

        std::string parameter_file_path_;
        this->declare_parameter("parameter_file_path", rclcpp::ParameterValue(""));
        this->get_parameter("parameter_file_path", parameter_file_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameter file path: " << parameter_file_path_);

        this->declare_parameter("publish_grid_on_addition", rclcpp::ParameterValue(true));
        this->get_parameter("publish_grid_on_addition", publish_grid_on_addition_);

        ParameterHandler::getInstance(parameter_file_path_);

        bool kf_opt;
        this->declare_parameter("is_kf_optimization_enabled", rclcpp::ParameterValue(false));
        this->get_parameter("is_kf_optimization_enabled", kf_opt);

        RCLCPP_WARN_STREAM(this->get_logger(), "Using lidar pointcloud? " << use_lidar_pointcloud_);
        parameterInstance.setValue<bool>("is_kf_optimization_enabled", kf_opt);
        parameterInstance.setValue<std::string>("SLAM_System", "ISAE");

        tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

        Eigen::Affine3f tf_SlamToLidar;
        Eigen::Affine3f tf_BaseToSlam;
        populateTransforms(slam_frame_id_, robot_base_frame_id_, lidar_frame_id_, this->get_clock(), this->get_logger(), tf_buffer_ptr_, tf_SlamToLidar, tf_BaseToSlam);
        tf_buffer_ptr_.reset();
        tf_listener_ptr_.reset();

        keyFrameAdditionsSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameAdditions>(
            additions_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameAdditionsCallback, this, std::placeholders::_1));

        keyFrameUpdatesSubscriber_ = this->create_subscription<traversability_msgs::msg::KeyFrameUpdates>(
            updates_topic_name_, 10, std::bind(&TraversabilityNode::keyFrameUpdatesCallback, this, std::placeholders::_1));
        
        if (use_lidar_pointcloud_)
            lidarPointCloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud_topic_name_, 10, std::bind(&TraversabilityNode::pointCloudCallback, this, std::placeholders::_1));

        publishTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TraversabilityNode::publishTraversabilityData, this));

        publish_global_pointcloud_service_ = this->create_service<traversability_msgs::srv::GetGlobalPointcloud>(
            "publish_global_pointcloud", std::bind(&TraversabilityNode::publishGlobalPointCloud, this, std::placeholders::_1, std::placeholders::_2));

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_traversability_map", 10);
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
        pclPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kf_pointcloud", 10);
        posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("kf_pose", 10);

        traversabilitySystem_ = std::make_shared<traversability_mapping::System>();
        traversabilitySystem_->setExtrinsicParameters(tf_SlamToLidar, tf_BaseToSlam);
        traversabilitySystem_->addNewLocalMap(0);
        RCLCPP_INFO(this->get_logger(), "Added new map");
    }

private:
    void publishGlobalPointCloud(traversability_msgs::srv::GetGlobalPointcloud::Request::SharedPtr request, traversability_msgs::srv::GetGlobalPointcloud::Response::SharedPtr response)
    {
        (void)response;
        // 1) grab the stitched PCL cloud (map-frame)
        auto cloud_ptr = traversabilitySystem_->getGlobalPointCloud(request->voxel_size_x, request->voxel_size_y, request->voxel_size_z);
        if (!cloud_ptr || cloud_ptr->empty()) {
            RCLCPP_WARN(get_logger(), "Global cloud is empty, not publishing.");
            return;
        }

        // 2) convert PCL<T> -> pcl::PCLPointCloud2
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloud_ptr, pcl_pc2);

        // 3) convert pcl::PCLPointCloud2 -> sensor_msgs::msg::PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(pcl_pc2, output);

        // 4) stamp & frame
        output.header.stamp = this->now();
        output.header.frame_id = "map";

        // 5) publish
        pclPublisher_->publish(output);
    }

    void keyFrameAdditionsCallback(const traversability_msgs::msg::KeyFrameAdditions::SharedPtr msg)
    {
        // Process received KeyFrameAdditions message
        RCLCPP_INFO(this->get_logger(), "Received KeyFrameAdditions message");
        for (const auto &keyframe : msg->keyframes)
        {
            // Assuming keyframe data structure contains necessary information for mapping
            // Add the keyframe to the traversability map using pTraversability_
            if (use_lidar_pointcloud_)
                traversabilitySystem_->addNewKeyFrameTsULong(keyframe.kf_timestamp_in_nanosec, keyframe.kf_id, keyframe.map_id);
            else
            {
                pcl::PointCloud<pcl::PointXYZRGB> pointcloudInput;
                pcl::fromROSMsg(keyframe.kf_pointcloud, pointcloudInput);
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pclPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pointcloudInput);
                traversabilitySystem_->addNewKeyFrameWithPCL(keyframe.kf_timestamp_in_nanosec, keyframe.kf_id, keyframe.map_id, pclPtr);
            }
            Eigen::Affine3d keyFramePoseEigen;
            tf2::fromMsg(keyframe.kf_pose, keyFramePoseEigen);
            traversabilitySystem_->updateKeyFrame(keyframe.kf_id, keyFramePoseEigen, 1);

            // pclPublisher_->publish(keyframe.kf_pointcloud);

            geometry_msgs::msg::PoseStamped kf_pose_stamped;
            kf_pose_stamped.header.frame_id = "map";
            kf_pose_stamped.pose = keyframe.kf_pose;
            posePublisher_->publish(kf_pose_stamped);
        }
        if (publish_grid_on_addition_)
        {
            // also publish when new keyframe are added
            publishTraversabilityData();
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
            traversabilitySystem_->updateKeyFrame(keyframe.kf_id, keyFramePoseEigen, 1);
        }
    }

    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        traversabilitySystem_->pushToBuffer(msg);
    }

    void publishTraversabilityData()
    {
        std::shared_ptr<grid_map::GridMap> traversabilitymap;
        std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridmap;
        auto localMap = traversabilitySystem_->getLocalMap();
        if (localMap != nullptr)
        {
            auto trav_done = traversabilitySystem_->getLocalMap()->getGridMap();
            auto gridmap_done = traversabilitySystem_->getLocalMap()->getOccupancyMap();
            if (trav_done)
            {
                traversabilitymap = trav_done;
                gridmap = gridmap_done;
            }
            if (traversabilitymap && gridmap)
            {
                occupancy_grid_publisher_->publish(*gridmap);
                if(publish_traversability_grid_)
                {
                    auto message = *grid_map::GridMapRosConverter::toMessage(*traversabilitymap);
                    traversabilityPub_->publish(message);
                }
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
    std::string slam_frame_id_;
    std::string lidar_frame_id_;
    std::string robot_base_frame_id_;
    bool use_lidar_pointcloud_;
    bool publish_traversability_grid_;
    bool publish_grid_on_addition_;
    rclcpp::TimerBase::SharedPtr publishTimer_;

    rclcpp::Subscription<traversability_msgs::msg::KeyFrameAdditions>::SharedPtr keyFrameAdditionsSubscriber_;
    rclcpp::Subscription<traversability_msgs::msg::KeyFrameUpdates>::SharedPtr keyFrameUpdatesSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPointCloudSubscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr traversabilityPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;

    rclcpp::Service<traversability_msgs::srv::GetGlobalPointcloud>::SharedPtr publish_global_pointcloud_service_;

    std::shared_ptr<traversability_mapping::System> traversabilitySystem_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_; //!< shared pointer to a buffer of Map to BaseLink tfs
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_; //!< Shared pointer to a TransformListener, used to listen to Map To BaseLink transforms
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TraversabilityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}