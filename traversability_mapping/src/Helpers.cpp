/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see
 * <https://www.gnu.org/licenses/>.
 */
#include <traversability_mapping/Helpers.hpp>

namespace traversability_mapping
{
    void doTransformPCL(
        const pcl::PointCloud<pcl::PointXYZRGB> &p_in, pcl::PointCloud<pcl::PointXYZRGB> &p_out,
        const Eigen::Affine3f &t)
    {
        pcl:: transformPointCloud(p_in, p_out, t);
    }
#ifdef WITH_ROS2_SENSOR_MSGS
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
#endif
}