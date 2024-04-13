#include "traversability_mapping/PointCloudBuffer.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    PointCloudBuffer::PointCloudBuffer()
    {
        // Initialize the buffer
        buffer_.clear();
    }

    void PointCloudBuffer::addPointCloud(sensor_msgs::msg::PointCloud2 &pointCloud, double timestamp)
    {
        auto pair = std::make_pair(timestamp, pointCloud);
        buffer_.push_back(pair);
        // std::cout << "The size of buffer is: " << buffer_.size() << std::endl;
    }

    // Function to find the closest point cloud to the queried timestamp
    sensor_msgs::msg::PointCloud2 PointCloudBuffer::getClosestPointCloud(const double &query_time)
    {
        // std::cout << "PointCloudBuffer::getClosestPointCloud" << std::endl;
        // Check if buffer is empty
        if (buffer_.empty())
        {
            std::cerr << "Error: Point cloud buffer is empty." << std::endl;
            // Return an empty point cloud or handle the error condition based on your requirements
            return sensor_msgs::msg::PointCloud2();
        }

        auto closest_cloud_iter = std::min_element(buffer_.begin(), buffer_.end(),
                                                   [&query_time](const auto &a, const auto &b)
                                                   {
                                                       return std::abs(a.first - query_time) < std::abs(b.first - query_time);
                                                   });

        return closest_cloud_iter->second;
    }

    // Function to delete all points before the queried timestamp in the buffer
    void PointCloudBuffer::deletePointsBefore(const double &query_time)
    {
        buffer_.erase(std::remove_if(buffer_.begin(), buffer_.end(),
                                     [&query_time](const auto &entry)
                                     {
                                         return entry.first < query_time;
                                     }),
                      buffer_.end());
    }
}