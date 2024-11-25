#include "traversability_mapping/PointCloudBuffer.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    PointCloudBuffer::PointCloudBuffer()
    {
        // Initialize the buffer
        buffer_.clear();
    }

    void PointCloudBuffer::addPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud, double timestamp)
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
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBuffer::getClosestPointCloud(const double &query_time)
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
        return closest_cloud_iter->second;
    }

    // Function to delete all points before the queried timestamp in the buffer
    void PointCloudBuffer::deletePointsBefore(const double &query_time)
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
}