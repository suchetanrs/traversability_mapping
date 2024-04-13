#ifndef POINTCLOUDBUFFER_HPP_
#define POINTCLOUDBUFFER_HPP_

#include <deque>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace traversability_mapping
{
    class PointCloudBuffer
    {
    public:
        PointCloudBuffer();

        void addPointCloud(sensor_msgs::msg::PointCloud2 &pointCloud, double timestamp);

        // Function to find the closest point cloud to the queried timestamp
        sensor_msgs::msg::PointCloud2 getClosestPointCloud(const double &query_time);

        // Function to delete all points before the queried timestamp in the buffer
        void deletePointsBefore(const double &query_time);

    private:
        std::deque<std::pair<double, sensor_msgs::msg::PointCloud2>> buffer_;
    };
}

#endif // POINTCLOUDBUFFER_HPP_