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

namespace traversability_mapping
{
    struct KeyFrameParameters
    {
        double resolution_;
        double half_size_traversability_;
        double security_distance_;
        double ground_clearance_;
        double max_slope_;
        double robot_height_;
        double translation_change_threshold_;
        double rotation_change_threshold_;
        bool is_kf_optimization_enabled_;
    };

    class KeyFrame
    {
    public:
        KeyFrame(double timestamp,
                 long unsigned int kfID,
                 sensor_msgs::msg::PointCloud2 &pointCloud,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 long unsigned int mapID,
                 Eigen::Affine3f Tbv,
                 KeyFrameParameters kfParams);

        KeyFrame(long unsigned int kfID,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 Eigen::Affine3f Tbv,
                 KeyFrameParameters kfParams);

        // GETTERS
        const double &getTimestamp();

        const long unsigned int &getKfID();

        const Eigen::Affine3f &getPose();

        const Sophus::SE3f &getSLAMPose();

        const std::shared_ptr<sensor_msgs::msg::PointCloud2> &getPointCloud();

        // SETTERS
        void setPose(const Eigen::Affine3f &pose);

        void setSLAMPose(const Sophus::SE3f &pose);

        void setMap(long unsigned int mapID, std::shared_ptr<grid_map::GridMap> gridMap);

        // TRAVERSABILITY FUNCTION
        void computeLocalTraversability(sensor_msgs::msg::PointCloud2 &kFpcl);

        // CACHE RECOMPUTE
        void recomputeCache();

        void processUpdateQueue();

        void updateGridAfterMapChange();

        void clearStrayValuesInGrid();

    private:
        // set only one time in constructor.
        double timestamp_;
        long unsigned int kfID_;
        sensor_msgs::msg::PointCloud2 pointCloudLidar_;

        std::mutex poseMutex_;
        std::unique_ptr<Sophus::SE3f> poseSLAMCoord_;
        std::unique_ptr<Eigen::Affine3f> poseTraversabilityCoord_;
        
        
        std::mutex gridMapMutex_;
        std::shared_ptr<grid_map::GridMap> pGridMap_;
        long unsigned int parentMapID_;
        KeyFrameParameters kfParams_;
        std::vector<Eigen::Vector2d> markedCells_;

        std::mutex poseUpdateQueueMutex_;
        std::deque<Eigen::Affine3f> poseUpdates_; //< Vector to store the pose updates to be processed later.
        
        Eigen::Affine3f Tbv_;
        std::shared_ptr<sensor_msgs::msg::PointCloud2> pointCloudMap_; //< Temporary pointcloud to return. Can delete later.
    };
}

#endif // KEYFRAME_HPP_
