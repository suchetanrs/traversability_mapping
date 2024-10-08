// keyFrame.hpp
#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_

#include <mutex>
#include <vector>
#include <thread>
#include <deque>
#include <sophus/se3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "traversability_mapping/TraversabilityGrid.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping/Parameters.hpp"

namespace traversability_mapping
{
    class KeyFrame
    {
    public:
        KeyFrame(double timestamp,
                 long unsigned int kfID,
                 sensor_msgs::msg::PointCloud2 &pointCloud,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 std::shared_ptr<std::mutex> masterGridMapMutex,
                 long unsigned int mapID,
                 Eigen::Affine3f Tbv);

        KeyFrame(long unsigned int kfID,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 std::shared_ptr<std::mutex> masterGridMapMutex,
                 Eigen::Affine3f Tbv);
        
        ~KeyFrame();

        // GETTERS
        const double &getTimestamp();

        const long unsigned int &getKfID();

        const Eigen::Affine3f &getPose();

        const Sophus::SE3f &getSLAMPose();

        // SETTERS
        void setPose(const Eigen::Affine3f &pose);

        void setSLAMPose(const Sophus::SE3f &pose);

        void setMap(long unsigned int mapID, std::shared_ptr<grid_map::GridMap> gridMap, std::shared_ptr<std::mutex> masterGridMapMutex);

        // TRAVERSABILITY FUNCTION
        void computeLocalTraversability(sensor_msgs::msg::PointCloud2 &kFpcl);

        // CACHE RECOMPUTE
        void recomputeCache();

        void processUpdateQueue();

        void updateGridAfterMapChange();

        void clearStrayValuesInGrid();

    private:
        // set only one time in constructor. thread safety not needed since it is only read.
        double timestamp_;
        long unsigned int kfID_;
        sensor_msgs::msg::PointCloud2 pointCloudLidar_;
        Eigen::Affine3f Tbv_;

        std::mutex poseMutex_;
        std::unique_ptr<Sophus::SE3f> poseSLAMCoord_;
        std::unique_ptr<Eigen::Affine3f> poseTraversabilityCoord_;
        
        std::mutex gridMapMutex_; // this mutex prevents access of pGridMap_ when pGridMap_ itself is changing.
        std::shared_ptr<std::mutex> masterGridMapMutex_; // this mutex prevents concurrency between kfs.
        std::shared_ptr<grid_map::GridMap> pGridMap_;
        long unsigned int parentMapID_;
        std::vector<Eigen::Vector2d> markedCells_;

        std::mutex poseUpdateQueueMutex_;
        std::deque<Eigen::Affine3f> poseUpdates_; //< Vector to store the pose updates to be processed later.
    };
}

#endif // KEYFRAME_HPP_
