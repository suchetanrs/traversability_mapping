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

#include "traversability_mapping/PointCloudBufferROS.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    PointCloudBufferROS::PointCloudBufferROS()
    {
    }

#ifdef WITH_ROS2_SENSOR_MSGS
    void PointCloudBufferROS::addPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr pointCloud, double timestamp)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        buffer_.push_back(std::make_pair(timestamp, pointCloud));
        if (std::abs(buffer_.front().first - buffer_.back().first) > 25.0)
            deletePointsBefore(timestamp - 5.0);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBufferROS::getClosestPointCloud(const double &query_time)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        if (buffer_.empty())
        {
            std::cerr << "Error: Point cloud buffer is empty." << std::endl;
            return nullptr;
        }
        auto closest = std::min_element(buffer_.begin(), buffer_.end(),
            [&query_time](const auto &a, const auto &b)
            { return std::abs(a.first - query_time) < std::abs(b.first - query_time); });

        pcl::PCLPointCloud2 pcl_pc2;
        toPCL(*(closest->second), pcl_pc2);
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2, *out);
        return out;
    }

    void PointCloudBufferROS::deletePointsBefore(const double &query_time)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        while (!buffer_.empty() && buffer_.front().first < query_time)
            buffer_.pop_front();
    }
#endif
}
