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
#ifndef POINTCLOUDBUFFERROS_HPP_
#define POINTCLOUDBUFFERROS_HPP_

#include <deque>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping_common/type_conversion.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    class PointCloudBufferROS
    {
    public:
        PointCloudBufferROS();
#ifdef WITH_ROS2_SENSOR_MSGS

        void addPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr pointCloud, double timestamp);

        // Function to find the closest point cloud to the queried timestamp
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getClosestPointCloud(const double &query_time);

        // Function to delete all points before the queried timestamp in the buffer
        void deletePointsBefore(const double &query_time);

    private:
        std::deque<std::pair<double, sensor_msgs::msg::PointCloud2::SharedPtr>> buffer_;
        std::recursive_mutex bufferMutex_;
#endif
    };
}

#endif // POINTCLOUDBUFFER_HPP_