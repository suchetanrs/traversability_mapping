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

struct KeyFrameUpdateData
{
    long unsigned int id;
    Sophus::SE3f pose;
    long unsigned int numConnections;

    KeyFrameUpdateData() : id(0), pose(Sophus::SE3f()), numConnections(0) {}
    
    KeyFrameUpdateData(long unsigned int id, Sophus::SE3f p, long unsigned int connections) 
        : id(id), pose(p), numConnections(connections) {}
};

using UpdateQueue = std::queue<KeyFrameUpdateData>;

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

        void clearEntireMap();

        // Function to add a new keyframe to the map
        std::shared_ptr<KeyFrame> addNewKeyFrame(double timestamp,
                                                 long unsigned int kfID,
                                                 pcl::PointCloud<pcl::PointXYZ> &pointCloud,
                                                 long unsigned int mapID);

        void addAlreadyDeclaredKF(std::shared_ptr<KeyFrame> keyFrame);

        void deleteKeyFrame(long unsigned int kfID);

        const std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> &getKeyFramesMap() const;
#ifdef WITH_ROS2_SENSOR_MSGS
        const std::shared_ptr<nav_msgs::msg::OccupancyGrid> getOccupancyMap();
#endif
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
        Eigen::Affine3f Tsv_;
        Eigen::Affine3f Tbs_;

        std::mutex &updateQueueMutex_;
        std::shared_ptr<UpdateQueue> keyFrameUpdateQueue_;
        
        std::shared_ptr<std::mutex> masterGridMapMutex_;
        std::shared_ptr<grid_map::GridMap> pGridMap_;

        // Map to store KeyFrame ids and corresponding KeyFrameData pointers
        std::mutex keyFramesMapMutex;
        std::unordered_map<long unsigned int, std::shared_ptr<KeyFrame>> keyFramesMap_;
        std::mutex localKeyFramesMutex;
        std::deque<std::shared_ptr<KeyFrame>> mLocalKeyFrames_;
#ifdef WITH_ROS2_SENSOR_MSGS
        std::shared_ptr<nav_msgs::msg::OccupancyGrid> gridMapOccupancy_;
#endif
        bool activeMap_ = false;
        bool globalMappingRunning_ = false;
        bool localMappingRunning_ = false;
    };
}

#endif // LOCAL_MAP_HPP_