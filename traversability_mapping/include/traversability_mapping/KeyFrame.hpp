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
#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_

#include <mutex>
#include <vector>
#include <thread>
#include <deque>
#include <sophus/se3.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#ifdef WITH_ROS2_SENSOR_MSGS
#include <nav_msgs/msg/occupancy_grid.hpp>
#endif
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "traversability_mapping/TraversabilityGrid.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping/Parameters.hpp"
#include "traversability_mapping/HashGrid.hpp"

namespace traversability_mapping
{
    class KeyFrame
    {
    public:
        // Tsv: T_slam_to_velodyne
        // Tbs: T_basefootprint_to_slam
        KeyFrame(double timestamp,
                 long unsigned int kfID,
                 pcl::PointCloud<pcl::PointXYZRGB> &pointCloud,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 std::shared_ptr<std::mutex> masterGridMapMutex,
                 long unsigned int mapID,
                 Eigen::Affine3f Tsv,
                 Eigen::Affine3f Tbs);

        // Tsv: T_slam_to_velodyne
        // Tbs: T_basefootprint_to_slam
        KeyFrame(long unsigned int kfID,
                 std::shared_ptr<grid_map::GridMap> gridMap,
                 std::shared_ptr<std::mutex> masterGridMapMutex,
                 Eigen::Affine3f Tsv,
                 Eigen::Affine3f Tbs);
        
        ~KeyFrame();

        // GETTERS
        const double &getTimestamp();

        const long unsigned int &getKfID();

        const Eigen::Affine3f &getPose();

        const Sophus::SE3f &getSLAMPose();

        const long unsigned int &getConnections();

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> getPointCloudLidarFrame();

        // SETTERS
        void setPose(const Eigen::Affine3f &pose);

        void setSLAMPose(const Sophus::SE3f &pose);

        void setMap(long unsigned int mapID, std::shared_ptr<grid_map::GridMap> gridMap, std::shared_ptr<std::mutex> masterGridMapMutex);

        void setConnections(long unsigned int numConnections);

        // TRAVERSABILITY FUNCTION
        // kFpcl : pointcloud transformed into the world frame.
        // traversabilityPose : the pose of the robot in map frame. This is the !!base_footprint!! in the map frame.
        void computeLocalTraversability(pcl::PointCloud<pcl::PointXYZRGB> &kFpcl, Eigen::Affine3f& traversabilityPose);

        // CACHE RECOMPUTE
        void recomputeCache(bool useHashGrid);

        void processUpdateQueue();

        void updateGridAfterMapChange();

        void clearStrayValuesInGrid();

    private:
        // set only one time in constructor. thread safety not needed since it is only read.
        double timestamp_;
        long unsigned int kfID_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pointCloudLidar_;
        Eigen::Affine3f Tsv_; // transform from slam frame to velodyne (lidar) frame.
        Eigen::Affine3f Tsb_; // transform from slam frame to base_footprint frame.

        std::mutex connectionMutex_;
        long unsigned int numConnections_;

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
