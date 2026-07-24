/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * ROS <-> plain-data conversions for the thin traversability adapter node. All
 * message <-> core-data and PCL translation lives here so the node body only wires
 * topics and timers, and the core library stays ROS-free.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_core/GridMap.hpp>

#include "traversability_mapping/Moments.hpp"  // traversability_mapping::Lattice
#include "traversability_msgs/msg/traversability_sparse_update.hpp"

namespace tmap_ros
{
    /// geometry_msgs/Pose (map<-base) -> Eigen::Affine3f.
    Eigen::Affine3f poseToAffine(const geometry_msgs::msg::Pose &pose);

    /// sensor_msgs/PointCloud2 (sensor frame) -> PCL cloud.
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> toPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud);

    /// Build a sparse-update message by reading `layers` from the live grid at each
    /// cell key's position (row-major: one float per layer in `layers` order). Keys
    /// outside the grid are dropped. This is where the ROS adapter decides WHICH
    /// layers to publish -- the core LocalMap is layer-agnostic. The caller fills
    /// msg.header (frame_id + stamp) and MUST hold the LocalMap grid mutex.
    void fillSparseUpdate(const grid_map::GridMap &grid,
                          const traversability_mapping::Lattice &lattice,
                          const std::vector<std::string> &layers,
                          const std::vector<std::uint64_t> &cell_keys, bool is_full,
                          traversability_msgs::msg::TraversabilitySparseUpdate &msg);
}  // namespace tmap_ros
