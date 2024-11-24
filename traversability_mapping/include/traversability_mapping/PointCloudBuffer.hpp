#ifndef POINTCLOUDBUFFER_HPP_
#define POINTCLOUDBUFFER_HPP_

#include <deque>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "traversability_mapping/Helpers.hpp"

using namespace std::chrono_literals;

namespace traversability_mapping
{
    class PointCloudBuffer
    {
    public:
        PointCloudBuffer();

        void addPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud, double timestamp);

        // Function to find the closest point cloud to the queried timestamp
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getClosestPointCloud(const double &query_time);

        // Function to delete all points before the queried timestamp in the buffer
        void deletePointsBefore(const double &query_time);

    private:
        std::deque<std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>> buffer_;
        std::recursive_mutex bufferMutex_;
    };
}

#endif // POINTCLOUDBUFFER_HPP_