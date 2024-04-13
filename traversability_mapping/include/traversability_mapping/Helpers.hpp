#ifndef TRAVERSABILITY_HELPERS_HPP_
#define TRAVERSABILITY_HELPERS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace traversability_mapping
{
    void doTransformPCL(
        const sensor_msgs::msg::PointCloud2 &p_in, sensor_msgs::msg::PointCloud2 &p_out,
        const Eigen::Affine3f &t);

    void gridMapToOccupancyGrid(
        const grid_map::GridMap &gridMap,
        const std::string &layer, float dataMin, float dataMax,
        nav_msgs::msg::OccupancyGrid &occupancyGrid);
}
#endif // TRAVERSABILITY_HELPERS_HPP_