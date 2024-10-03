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

    inline double probabilityToLogOdds(double p) {
        return std::log(p / (1.0 - p));
    }

    inline double logOddsToProbability(double l) {
        return 1.0 - 1.0 / (1.0 + std::exp(l));
    }

    // Function to update the occupancy of a cell
    inline double updateCellLogOdds(float& log_odds, double new_observation) {
        // If the cell is uninitialized (NaN), initialize with neutral log-odds
        if (std::isnan(log_odds)) {
            log_odds = 0.0;  // Neutral belief (50%)
        }

        double new_log_odds = probabilityToLogOdds(new_observation);

        log_odds += new_log_odds;

        return logOddsToProbability(log_odds);
    }
}
#endif // TRAVERSABILITY_HELPERS_HPP_