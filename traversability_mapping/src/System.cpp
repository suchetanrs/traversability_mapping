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
#include "traversability_mapping/System.hpp"
#include "traversability_mapping/Parameters.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace traversability_mapping
{
    System::System()
    {
        const double cx = parameterInstance.getValue<double>("grid/grid_center_x");
        const double cy = parameterInstance.getValue<double>("grid/grid_center_y");
        const double res = parameterInstance.getValue<double>("grid/resolution_local_map");
        lattice_ = Lattice(cx, cy, res);

        robot_height_ = parameterInstance.getValue<double>("ingestion/robot_height");
        max_range_base_frame_ = parameterInstance.getValue<double>("ingestion/max_range_base_frame");
        min_range_base_frame_ = parameterInstance.getValue<double>("ingestion/min_range_base_frame");

        usePointCloudBuffer_ = parameterInstance.getValue<bool>("ingestion/use_pointcloud_buffer");
        useROSBuffer_ = parameterInstance.getValue<bool>("ingestion/use_ros_buffer");
        if (usePointCloudBuffer_)
        {
            pointCloudBuffer_ = std::make_shared<PointCloudBuffer>();
            pointCloudBufferROS_ = std::make_shared<PointCloudBufferROS>();
        }
    }

    void System::setExtrinsicParameters(const Eigen::Affine3f &Tsv, const Eigen::Affine3f &Tbs)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        Tsv_ = Tsv;
        Tbs_ = Tbs;
        Tbv_ = Tbs_ * Tsv_;  // base <- lidar
    }

    void System::addNewLocalMap(std::uint64_t mapID, std::function<void()> onUpdate)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (localMapsSet_.count(mapID))
        {
            std::cerr << "[System] Local map " << mapID << " already exists; not recreating." << std::endl;
            return;
        }
        localMapsSet_[mapID] = std::make_shared<LocalMap>(mapID, lattice_, mapFrame_, std::move(onUpdate));
        setCurrentMap(mapID);
    }

    void System::setCurrentMap(std::uint64_t mapID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto it = localMapsSet_.find(mapID);
        if (it != localMapsSet_.end())
            localMap_ = it->second;
        else
            std::cerr << "[System] cannot set current map " << mapID << "; not initialized." << std::endl;
    }

    // ---- additions ----------------------------------------------------------

    double System::nanosecToSec(unsigned long long timestamp_ns)
    {
        const unsigned long long sec = timestamp_ns / 1000000000ULL;
        const unsigned long long nano = timestamp_ns % 1000000000ULL;
        return static_cast<double>(sec) + static_cast<double>(nano) * 1e-9;
    }

    std::vector<Eigen::Vector3f> System::pruneToBase(const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar) const
    {
        std::vector<Eigen::Vector3f> cloud_base;
        cloud_base.reserve(cloud_lidar.size());
        for (const auto &p : cloud_lidar)
        {
            if (p.x == 0.f && p.y == 0.f)
                continue;  // ego / invalid return (lidar frame)
            const Eigen::Vector3f p_base = Tbv_ * Eigen::Vector3f(p.x, p.y, p.z);
            if (p_base.z() > static_cast<float>(robot_height_))
                continue;  // ceiling / overhang
            const float range = p_base.norm();
            if (range > static_cast<float>(max_range_base_frame_) ||
                range < static_cast<float>(min_range_base_frame_))
                continue;  // out of range
            cloud_base.push_back(p_base);
        }
        return cloud_base;
    }

    void System::addNewKeyFrameToMap(double timestamp, std::uint64_t kfID, std::uint64_t mapID,
                                  const pcl::PointCloud<pcl::PointXYZ> &sensorPointCloud)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        if (keyFramesMap_.count(kfID))
        {
            std::cerr << "[System] keyframe " << kfID << " already added; ignoring." << std::endl;
            return;
        }
        auto mapIt = localMapsSet_.find(mapID);
        if (mapIt == localMapsSet_.end())
        {
            std::cerr << "[System] map " << mapID << " not initialized; keyframe " << kfID
                      << " dropped." << std::endl;
            return;
        }
        std::vector<Eigen::Vector3f> cloud_base = pruneToBase(sensorPointCloud);
        if (cloud_base.empty())
        {
            std::cerr << "[System] keyframe " << kfID << " pruned to empty; not stored." << std::endl;
            return;
        }
        const std::size_t npts = cloud_base.size();
        auto kf = std::make_shared<KeyFrame>(kfID, timestamp, Eigen::Affine3f::Identity(),
                                             std::move(cloud_base), mapID);
        keyFramesMap_[kfID] = kf;
        allKeyFramesSet_[kfID] = mapID;
        mapIt->second->addAlreadyDeclaredKF(kf);
        std::cout << "[System] ADD kf " << kfID << " -> map " << mapID << " (" << npts
                  << " base pts; registry now holds " << keyFramesMap_.size() << " kfs)." << std::endl;
    }

    void System::addNewKeyFrameWithPCL(unsigned long long timestamp_ns, std::uint64_t kfID,
                                       std::uint64_t mapID,
                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud)
    {
        if (kfID > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        {
            throw std::runtime_error(
                "The given KF ID was negative. Please make sure the keyFrame IDs are positive. KF ID: " +
                std::to_string(static_cast<std::int64_t>(kfID)));
        }
        if (!sensorPointCloud)
            return;
        addNewKeyFrameToMap(nanosecToSec(timestamp_ns), kfID, mapID, *sensorPointCloud);
    }

    void System::addNewKeyFrameTsDouble(double timestamp, std::uint64_t kfID, std::uint64_t mapID)
    {
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
#ifdef WITH_ROS2_SENSOR_MSGS
        cloud = useROSBuffer_ ? pointCloudBufferROS_->getClosestPointCloud(timestamp)
                              : pointCloudBuffer_->getClosestPointCloud(timestamp);
#else
        cloud = pointCloudBuffer_ ? pointCloudBuffer_->getClosestPointCloud(timestamp) : nullptr;
#endif
        if (!cloud)
            return;
        addNewKeyFrameToMap(timestamp, kfID, mapID, *cloud);
    }

    void System::addNewKeyFrameTsULong(unsigned long long timestamp_ns, std::uint64_t kfID, std::uint64_t mapID)
    {
        addNewKeyFrameTsDouble(nanosecToSec(timestamp_ns), kfID, mapID);
    }

    // ---- pose updates -------------------------------------------------------

    void System::updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3f &pose_map_base,
                                std::uint64_t /*numConnections*/)
    {
        std::shared_ptr<LocalMap> map;
        std::shared_ptr<KeyFrame> kf;
        {
            std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
            auto rit = allKeyFramesSet_.find(kfID);
            if (rit == allKeyFramesSet_.end())
            {
                std::cerr << "[System] update for unknown keyframe " << kfID << "; ignored." << std::endl;
                return;
            }
            auto mapIt = localMapsSet_.find(rit->second);
            if (mapIt == localMapsSet_.end())
            {
                std::cerr << "[System] update for kf " << kfID << ": owning map "
                          << rit->second << " missing; ignored." << std::endl;
                return;
            }
            map = mapIt->second;
            auto kfIt = keyFramesMap_.find(kfID);
            if (kfIt != keyFramesMap_.end())
                kf = kfIt->second;
        }
        // O(1), never blocks: route the pose through the owning map so it stores the
        // latest pose AND enqueues the keyframe for rebin on the local work queue.
        map->setKeyFramePose(kf, pose_map_base);
        std::cout << "[System] UPDATE kf " << kfID << ": pose queued for rebin." << std::endl;
    }

    void System::updateKeyFrame(std::uint64_t kfID, const Sophus::SE3f &pose_map_base,
                                std::uint64_t numConnections)
    {
        updateKeyFrame(kfID, Eigen::Affine3f(pose_map_base.matrix()), numConnections);
    }

    void System::updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3d &pose_map_base,
                                std::uint64_t numConnections)
    {
        updateKeyFrame(kfID, Eigen::Affine3f(pose_map_base.cast<float>()), numConnections);
    }

    // ---- lifecycle ----------------------------------------------------------

    void System::deleteKeyFrame(std::uint64_t kfID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto rit = allKeyFramesSet_.find(kfID);
        if (rit == allKeyFramesSet_.end())
        {
            std::cerr << "[System] cannot delete unknown keyframe " << kfID << "." << std::endl;
            return;
        }
        auto mapIt = localMapsSet_.find(rit->second);
        if (mapIt != localMapsSet_.end())
            mapIt->second->deleteKeyFrame(kfID);
        keyFramesMap_.erase(kfID);
        allKeyFramesSet_.erase(rit);
    }

    void System::updateKFMap(std::uint64_t kfID, std::uint64_t mapID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto rit = allKeyFramesSet_.find(kfID);
        if (rit == allKeyFramesSet_.end())
        {
            std::cerr << "[System] cannot move unknown keyframe " << kfID << "." << std::endl;
            return;
        }
        if (localMapsSet_.find(mapID) == localMapsSet_.end())
        {
            std::cerr << "[System] target map " << mapID << " does not exist." << std::endl;
            return;
        }
        const std::uint64_t oldMapID = rit->second;
        if (oldMapID == mapID)
            return;
        auto kf = localMapsSet_[oldMapID]->detachKeyFrame(kfID);  // subtract from old grid + recompute
        if (!kf)
            return;
        kf->setMap(mapID);
        localMapsSet_[mapID]->addAlreadyDeclaredKF(kf);   // register in the new map (not yet binned)
        // Re-set the current pose THROUGH the new map: raises pending + enqueues it so
        // the new map's local worker rebins and adds it.
        localMapsSet_[mapID]->setKeyFramePose(kf, kf->getPose());
        rit->second = mapID;
    }

    void System::informLoopClosure()
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto it = localMapsSet_.find(0);
        if (it != localMapsSet_.end())
            it->second->clearEntireMap();
    }

    // ---- buffers ------------------------------------------------------------

    void System::pushToBuffer(double timestamp, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl)
    {
        if (pointCloudBuffer_)
            pointCloudBuffer_->addPointCloud(pcl, timestamp);
    }

#ifdef WITH_ROS2_SENSOR_MSGS
    void System::pushToBuffer(sensor_msgs::msg::PointCloud2::SharedPtr pcl)
    {
        if (!pointCloudBufferROS_)
            return;
        const double seconds = pcl->header.stamp.sec + pcl->header.stamp.nanosec * 1e-9;
        pointCloudBufferROS_->addPointCloud(pcl, seconds);
    }
#endif

    // ---- access -------------------------------------------------------------

    std::shared_ptr<LocalMap> System::getLocalMap()
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        return localMap_;
    }

    std::shared_ptr<LocalMap> System::getLocalMap(std::uint64_t mapID)
    {
        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        auto it = localMapsSet_.find(mapID);
        return it != localMapsSet_.end() ? it->second : nullptr;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> System::getGlobalPointCloud(
        float voxel_size_x, float voxel_size_y, float voxel_size_z)
    {
        std::vector<std::shared_ptr<LocalMap>> maps;
        {
            std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
            for (auto &kv : localMapsSet_)
                maps.push_back(kv.second);
        }
        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (auto &m : maps)
        {
            auto c = m->getStitchedPointCloud(voxel_size_x, voxel_size_y, voxel_size_z);
            if (c)
                *out += *c;
        }
        return out;
    }
}  // namespace traversability_mapping
