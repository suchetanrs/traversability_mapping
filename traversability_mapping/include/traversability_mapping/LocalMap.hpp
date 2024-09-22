#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include <map>
#include <memory>
#include <filesystem>
#include <queue>
#include "traversability_mapping/KeyFrame.hpp"
#include "traversability_mapping_common/type_conversion.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping/Parameters.hpp"

using UpdateQueue = std::queue<std::pair<long unsigned int, Sophus::SE3f>>;

namespace traversability_mapping
{
    class LocalMap
    {
    public:
        // Constructor
        LocalMap(long unsigned int mapID, std::shared_ptr<UpdateQueue> keyFrameUpdateQueue, std::mutex &updateQueueMutex);

        // Destructor
        ~LocalMap();

        void RunUpdateQueue();

        void RunTraversability();

        void RunLocalKeyFrames();

        // Function to add a new keyframe to the map
        std::shared_ptr<KeyFrame> addNewKeyFrame(double timestamp,
                                                 long unsigned int kfID,
                                                 sensor_msgs::msg::PointCloud2 &pointCloud,
                                                 long unsigned int mapID);

        void addAlreadyDeclaredKF(std::shared_ptr<KeyFrame> keyFrame);

        void deleteKeyFrame(long unsigned int kfID);

        const std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> &getKeyFramesMap() const;

        const std::shared_ptr<nav_msgs::msg::OccupancyGrid> getOccupancyMap();

        const std::shared_ptr<grid_map::GridMap> getGridMap() const;

        std::shared_ptr<std::mutex> getGridMapMutex()
        {
            return masterGridMapMutex_;
        }

        void setActiveMap(bool activity)
        {
            activeMap_ = activity;
        }

        void processUpdateQueue();

    private:
        // set only one time in constructor.
        long unsigned int mapID_;
        std::shared_ptr<TraversabilityTypeConversions> typeConversion_;
        Eigen::Affine3f Tbv_;

        std::mutex &updateQueueMutex_;
        std::shared_ptr<std::queue<std::pair<long unsigned int, Sophus::SE3f>>> keyFrameUpdateQueue_;
        
        std::shared_ptr<std::mutex> masterGridMapMutex_;
        std::shared_ptr<grid_map::GridMap> pGridMap_;

        // Map to store KeyFrame ids and corresponding KeyFrameData pointers
        std::mutex keyFramesMapMutex;
        std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> keyFramesMap_;
        std::mutex localKeyFramesMutex;
        std::deque<std::shared_ptr<KeyFrame>> mLocalKeyFrames_;

        std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridMapOccupancy_;
        bool activeMap_ = false;
    };
}

#endif // LOCAL_MAP_HPP_