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
#ifndef TRAVERSABILITY_SYSTEM_HPP_
#define TRAVERSABILITY_SYSTEM_HPP_

// Multi-map-ready orchestrator (runs map_id = 0 only for now). Owns the map
// registry, the authoritative shared keyframe registry, the kf->map routing, the
// point-cloud buffers, and the robot extrinsics, and exposes the legacy public API
// a SLAM front-end drives. ROS-free: a SLAM/ROS adapter feeds PCL clouds + poses
// through this API.
//
// Pose convention: a keyframe's pose is the robot BASE pose in the MAP frame
// (Tmb = map <- base_footprint), matching KeyFrame.msg::kf_pose. The cloud handed
// to addNewKeyFrame* is in the LIDAR/sensor frame and is pruned to the base frame
// here (ego/ceiling/range gates + the Tbv extrinsic). Additions register a keyframe
// without binning; the pose arrives via updateKeyFrame, which sets the keyframe's
// pending pose and triggers the first bin (so nothing is ever binned at the
// placeholder identity pose).

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef WITH_ROS2_SENSOR_MSGS
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

#include "traversability_mapping/LocalMap.hpp"
#include "traversability_mapping/KeyFrame.hpp"
#include "traversability_mapping/PointCloudBuffer.hpp"
#include "traversability_mapping/PointCloudBufferROS.hpp"
#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    class System
    {
    public:
        System();

        /// Tsv = slam <- lidar, Tbs = base <- slam. Composes Tbv = Tbs * Tsv used to
        /// prune clouds into the base frame.
        void setExtrinsicParameters(const Eigen::Affine3f &Tsv, const Eigen::Affine3f &Tbs);

        /// Optional: grid frame id stamped on each map (cosmetic; default "map").
        void setMapFrame(const std::string &mapFrame) { mapFrame_ = mapFrame; }

        /// Create a map (and its two worker threads). The first one created becomes
        /// the active map. Multi-map-ready; today only map_id = 0 is used.
        /// `onUpdate` (optional) is invoked by the map's workers after each
        /// grid-changing keyframe op (e.g. for the adapter to publish on recompute).
        void addNewLocalMap(std::uint64_t mapID, std::function<void()> onUpdate = {});

        // --- additions ---------------------------------------------------------
        /// Add a keyframe with a directly supplied LIDAR-frame cloud.
        void addNewKeyFrameWithPCL(unsigned long long timestamp_ns, std::uint64_t kfID,
                                   std::uint64_t mapID,
                                   std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud);
        /// Add a keyframe whose cloud is fetched from the buffer by timestamp.
        void addNewKeyFrameTsDouble(double timestamp, std::uint64_t kfID, std::uint64_t mapID);
        void addNewKeyFrameTsULong(unsigned long long timestamp_ns, std::uint64_t kfID, std::uint64_t mapID);

        // --- pose updates (PGO); O(1) reception, latest-wins, never blocks -----
        void updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3f &pose_map_base,
                            std::uint64_t numConnections = 0);
        void updateKeyFrame(std::uint64_t kfID, const Sophus::SE3f &pose_map_base,
                            std::uint64_t numConnections = 0);
        void updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3d &pose_map_base,
                            std::uint64_t numConnections = 0);

        // --- lifecycle ---------------------------------------------------------
        void deleteKeyFrame(std::uint64_t kfID);
        /// Re-parent a keyframe to another map (subtract from the old grid, re-add to
        /// the new one). Multi-map-ready; unused while only map_id = 0 runs.
        void updateKFMap(std::uint64_t kfID, std::uint64_t mapID);
        /// Clear + rebuild map 0 (loop-closure response).
        void informLoopClosure();

        // --- buffers -----------------------------------------------------------
        void pushToBuffer(double timestamp, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl);
#ifdef WITH_ROS2_SENSOR_MSGS
        void pushToBuffer(sensor_msgs::msg::PointCloud2::SharedPtr pcl);
#endif

        // --- access ------------------------------------------------------------
        std::shared_ptr<LocalMap> getLocalMap();
        std::shared_ptr<LocalMap> getLocalMap(std::uint64_t mapID);
        /// Voxel-downsampled stitch of every map's retained clouds (legacy global cloud).
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getGlobalPointCloud(
            float voxel_size_x, float voxel_size_y, float voxel_size_z);

    private:
        void addNewKeyFrameToMap(double timestamp, std::uint64_t kfID, std::uint64_t mapID,
                                 const pcl::PointCloud<pcl::PointXYZ> &sensorPointCloud);
        void setCurrentMap(std::uint64_t mapID);
        std::vector<Eigen::Vector3f> pruneToBase(const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar) const;
        static double nanosecToSec(unsigned long long timestamp_ns);

        Lattice lattice_;
        std::string mapFrame_ = "map";
        Eigen::Affine3f Tsv_ = Eigen::Affine3f::Identity();
        Eigen::Affine3f Tbs_ = Eigen::Affine3f::Identity();
        Eigen::Affine3f Tbv_ = Eigen::Affine3f::Identity();   // base <- lidar
        double robot_height_, max_range_base_frame_, min_range_base_frame_;

        std::recursive_mutex localMapMutex_;
        std::unordered_map<std::uint64_t, std::shared_ptr<LocalMap>> localMapsSet_;
        std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> keyFramesMap_;   // authoritative
        std::unordered_map<std::uint64_t, std::uint64_t> allKeyFramesSet_;            // kfID -> mapID routing
        std::shared_ptr<LocalMap> localMap_ = nullptr;                                // current/active map

        bool usePointCloudBuffer_ = false;
        bool useROSBuffer_ = false;
        std::shared_ptr<PointCloudBuffer> pointCloudBuffer_;
        std::shared_ptr<PointCloudBufferROS> pointCloudBufferROS_;
    };
}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_SYSTEM_HPP_
