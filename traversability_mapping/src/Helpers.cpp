#include <traversability_mapping/Helpers.hpp>

namespace traversability_mapping
{
    void doTransformPCL(
        const sensor_msgs::msg::PointCloud2 &p_in, sensor_msgs::msg::PointCloud2 &p_out,
        const Eigen::Affine3f &t)
    {
        p_out = p_in;
        sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, std::string("x"));
        sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, std::string("y"));
        sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, std::string("z"));

        sensor_msgs::PointCloud2Iterator<float> x_out(p_out, std::string("x"));
        sensor_msgs::PointCloud2Iterator<float> y_out(p_out, std::string("y"));
        sensor_msgs::PointCloud2Iterator<float> z_out(p_out, std::string("z"));

        Eigen::Vector3f point;
        for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
        {
            point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
            *x_out = point.x();
            *y_out = point.y();
            *z_out = point.z();
        }
    }

    void gridMapToOccupancyGrid(
        const grid_map::GridMap &gridMap,
        const std::string &layer, float dataMin, float dataMax,
        nav_msgs::msg::OccupancyGrid &occupancyGrid)
    {
        occupancyGrid.header.frame_id = gridMap.getFrameId();
        // occupancyGrid.header.stamp = rclcpp::Time(gridMap.getTimestamp());
        // Same as header stamp as we do not load the map.
        occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;
        occupancyGrid.info.resolution = gridMap.getResolution();
        occupancyGrid.info.width = gridMap.getSize()(0);
        occupancyGrid.info.height = gridMap.getSize()(1);
        grid_map::Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
        occupancyGrid.info.origin.position.x = position.x();
        occupancyGrid.info.origin.position.y = position.y();
        occupancyGrid.info.origin.position.z = 0.0;
        occupancyGrid.info.origin.orientation.x = 0.0;
        occupancyGrid.info.origin.orientation.y = 0.0;
        occupancyGrid.info.origin.orientation.z = 0.0;
        occupancyGrid.info.origin.orientation.w = 1.0;
        size_t nCells = gridMap.getSize().prod();
        occupancyGrid.data.resize(nCells);

        // Occupancy probabilities are in the range [0,100]. Unknown is -1.
        const float cellMin = 0;
        const float cellMax = 100;
        const float cellRange = cellMax - cellMin;

        for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
        {
            float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
            if (std::isnan(value))
            {
                value = -1;
            }
            else
            {
                value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
            }
            size_t index = grid_map::getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
            // Reverse cell order because of different conventions between occupancy grid and grid map.
            occupancyGrid.data[nCells - index - 1] = value;
        }
    }
}