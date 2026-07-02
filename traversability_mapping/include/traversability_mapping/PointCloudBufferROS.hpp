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
    /**
     * @brief Timestamp-keyed ring buffer holding raw sensor_msgs clouds.
     *
     * The sensor_msgs variant of PointCloudBuffer, compiled only when ROS message
     * types are available (WITH_ROS2_SENSOR_MSGS); the pure-PCL PointCloudBuffer is
     * the always-available fallback.
     */
    class PointCloudBufferROS
    {
    public:
        /// @brief Construct an empty buffer.
        PointCloudBufferROS();
#ifdef WITH_ROS2_SENSOR_MSGS

        /// @brief Store a cloud under its acquisition time.
        /// @param pointCloud the cloud. @param timestamp acquisition time (s).
        void addPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr pointCloud, double timestamp);

        /// @brief Fetch the buffered cloud closest in time to @p query_time.
        /// @param query_time query time (s). @return the closest cloud (nullptr if empty).
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getClosestPointCloud(const double &query_time);

        /// @brief Drop every buffered cloud older than @p query_time.
        /// @param query_time cutoff time (s).
        void deletePointsBefore(const double &query_time);

    private:
        std::deque<std::pair<double, sensor_msgs::msg::PointCloud2::SharedPtr>> buffer_;
        std::recursive_mutex bufferMutex_;
#endif
    };
}

#endif // POINTCLOUDBUFFERROS_HPP_
