/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * ROS <-> plain-data conversions for the thin traversability adapter node. All
 * message <-> core-data and PCL translation lives here so the node body only wires
 * topics and timers, and the core library stays ROS-free.
 */
#pragma once

#include <memory>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "traversability_mapping/LocalMap.hpp"  // traversability_mapping::NavDelta
#include "traversability_msgs/msg/traversability_sparse_update.hpp"

namespace tmap_ros
{
    /// geometry_msgs/Pose (map<-base) -> Eigen::Affine3f.
    Eigen::Affine3f poseToAffine(const geometry_msgs::msg::Pose &pose);

    /// sensor_msgs/PointCloud2 (sensor frame) -> PCL cloud.
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> toPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud);

    /// Pack a core NavDelta into the sparse-update message.
    void packSparse(const traversability_mapping::NavDelta &d, const rclcpp::Time &stamp,
                    traversability_msgs::msg::TraversabilitySparseUpdate &msg);
}  // namespace tmap_ros
