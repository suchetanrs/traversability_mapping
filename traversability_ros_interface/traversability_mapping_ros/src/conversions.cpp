/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * See conversions.hpp.
 */
#include "conversions.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace tmap_ros
{
    Eigen::Affine3f poseToAffine(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Translation3f t(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        return t * q;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> toPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(ros_cloud, pcl_pc2);
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2, *out);
        return out;
    }

    void packSparse(const traversability_mapping::NavDelta &d, const rclcpp::Time &stamp,
                    traversability_msgs::msg::TraversabilitySparseUpdate &msg)
    {
        msg.header.frame_id = d.frame_id;
        msg.header.stamp = stamp;
        msg.resolution = d.resolution;
        msg.origin_x = d.origin_x;
        msg.origin_y = d.origin_y;
        msg.layers = d.layers;
        msg.is_full_snapshot = d.is_full_snapshot;
        msg.cell_keys = d.cell_keys;
        msg.values = d.values;
    }
}  // namespace tmap_ros
