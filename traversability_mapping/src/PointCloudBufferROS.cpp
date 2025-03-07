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
        // Initialize the buffer
        // buffer_.clear();
    }

#ifdef WITH_ROS2_SENSOR_MSGS
    void PointCloudBufferROS::addPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr pointCloud, double timestamp)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        auto pair = std::make_pair(timestamp, pointCloud);
        buffer_.push_back(pair);
        if(abs(buffer_.front().first - buffer_.back().first) > 25.0)
        {
            deletePointsBefore(timestamp - 5.0);
        }
        // std::cout << "The size of buffer is: " << buffer_.size() << std::endl;
    }

    // Function to find the closest point cloud to the queried timestamp
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBufferROS::getClosestPointCloud(const double &query_time)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        // Check if buffer is empty
        if (buffer_.empty())
        {
            std::cerr << "Error: Point cloud buffer is empty." << std::endl;
            // Return an empty point cloud or handle the error condition based on your requirements
            return nullptr;
        }

        auto closest_cloud_iter = std::min_element(buffer_.begin(), buffer_.end(),
                                                   [&query_time](const auto &a, const auto &b)
                                                   {
                                                       return std::abs(a.first - query_time) < std::abs(b.first - query_time);
                                                   });

        std::cout << "pcl ts is: " << closest_cloud_iter->first << std::endl;
        std::cout << "Difference in closest pcl ts is: " << query_time - closest_cloud_iter->first << std::endl;
        
        pcl::PCLPointCloud2 pcl_pc2;
        toPCL(*(closest_cloud_iter->second), pcl_pc2);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloudInput = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2, *pointcloudInput);
        
        return pointcloudInput;
    }

    // Function to delete all points before the queried timestamp in the buffer
    void PointCloudBufferROS::deletePointsBefore(const double &query_time)
    {
        PROFILE_FUNCTION;
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);

        // Remove elements from the front while their timestamp is less than the query time
        while (!buffer_.empty() && buffer_.front().first < query_time)
        {
            buffer_.pop_front();
        }

        // buffer_.erase(std::remove_if(buffer_.begin(), buffer_.end(),
        //                              [&query_time](const auto &entry)
        //                              {
        //                                  return entry.first < query_time;
        //                              }),
        //               buffer_.end());
    }
#endif
}