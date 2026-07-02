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

#include "traversability_mapping/PointCloudBuffer.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    PointCloudBuffer::PointCloudBuffer()
    {
        buffer_.clear();
    }

    void PointCloudBuffer::addPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud, double timestamp)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        buffer_.push_back(std::make_pair(timestamp, pointCloud));
        if (std::abs(buffer_.front().first - buffer_.back().first) > 25.0)
            deletePointsBefore(timestamp - 5.0);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBuffer::getClosestPointCloud(const double &query_time)
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
        return closest->second;
    }

    void PointCloudBuffer::deletePointsBefore(const double &query_time)
    {
        std::lock_guard<std::recursive_mutex> lock(bufferMutex_);
        while (!buffer_.empty() && buffer_.front().first < query_time)
            buffer_.pop_front();
    }
}
