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

    void fillSparseUpdate(const grid_map::GridMap &grid,
                          const traversability_mapping::Lattice &lattice,
                          const std::vector<std::string> &layers,
                          const std::vector<std::uint64_t> &cell_keys, bool is_full,
                          traversability_msgs::msg::TraversabilitySparseUpdate &msg)
    {
        msg.resolution = lattice.res;
        msg.origin_x = lattice.x0;
        msg.origin_y = lattice.y0;
        msg.layers = layers;
        msg.is_full_snapshot = is_full;
        msg.cell_keys.clear();
        msg.values.clear();
        msg.cell_keys.reserve(cell_keys.size());
        msg.values.reserve(cell_keys.size() * layers.size());
        for (auto id : cell_keys)
        {
            int ci, cj;
            traversability_mapping::Lattice::unkey(id, ci, cj);
            const Eigen::Vector2d c = lattice.centerOf(ci, cj);
            const grid_map::Position p(c.x(), c.y());
            if (!grid.isInside(p))
                continue;
            msg.cell_keys.push_back(id);
            for (const auto &l : layers)
                msg.values.push_back(grid.atPosition(l, p));
        }
    }
}  // namespace tmap_ros
