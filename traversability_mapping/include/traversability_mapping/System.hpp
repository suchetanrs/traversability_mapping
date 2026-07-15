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

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

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
    /**
     * @brief Multi-map-ready orchestrator (runs map_id = 0 only for now).
     *
     * Owns the map registry, the authoritative keyframe registry, the kf->map routing,
     * the point-cloud buffers, and the robot extrinsics, and exposes the public API a
     * SLAM front-end drives. ROS-free.
     *
     * Pose convention: a keyframe's pose is the robot BASE pose in the MAP frame
     * (Tmb = map <- base_footprint). Clouds are supplied in the LIDAR frame and pruned to
     * the base frame here. Additions register a keyframe without binning; the first bin
     * happens once updateKeyFrame supplies a pose.
     */
    class System
    {
    public:
        System();

        /// @brief Set the extrinsics used to prune clouds into the base frame.
        /// @param Tsv slam <- lidar. @param Tbs base <- slam (composes Tbv = Tbs * Tsv).
        void setExtrinsicParameters(const Eigen::Affine3f &Tsv, const Eigen::Affine3f &Tbs);

        /// @brief Configure the obstacle-layer sensor (e.g. a depth camera on its own frame).
        ///        Distinct from the lidar extrinsic/ranges so obstacle clouds prune with the
        ///        depth cam's mounting and its own gates. Must be set before addObstacleKeyFrame.
        /// @param Tb_obs base <- obstacle_sensor (resolved once from the cloud's header frame).
        /// @param maxRange,minRange base-frame range gates for the obstacle sensor (m).
        void setObstacleParameters(const Eigen::Affine3f &Tb_obs, double maxRange, double minRange);

        /// @brief Set the grid frame id stamped on each map (cosmetic; default "map").
        /// @param mapFrame frame id.
        void setMapFrame(const std::string &mapFrame) { mapFrame_ = mapFrame; }

        /// @brief Create a map (and its two worker threads); the first becomes active.
        /// @param mapID new map id.
        /// @param onUpdate invoked by the map's workers after each grid-changing op.
        void addNewLocalMap(std::uint64_t mapID, std::function<void()> onUpdate = {});

        /// @name Additions
        /// @{
        /// @brief Add a keyframe with a directly supplied LIDAR-frame cloud.
        /// @param timestamp_ns acquisition time (ns). @param kfID keyframe id.
        /// @param mapID owning map. @param sensorPointCloud LIDAR-frame cloud.
        /// @throws std::runtime_error if @p kfID exceeds INT64_MAX, which is what a
        ///         negative id from a signed-id front-end converts to.
        void addNewKeyFrameWithPCL(unsigned long long timestamp_ns, std::uint64_t kfID,
                                   std::uint64_t mapID,
                                   std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud);
        /// @brief Add a keyframe whose cloud is fetched from the buffer by timestamp.
        /// @param timestamp acquisition time (s). @param kfID keyframe id. @param mapID owning map.
        void addNewKeyFrameTsDouble(double timestamp, std::uint64_t kfID, std::uint64_t mapID);
        /// @brief Add a keyframe whose cloud is fetched from the buffer by nanosecond timestamp.
        /// @param timestamp_ns acquisition time (ns). @param kfID keyframe id. @param mapID owning map.
        void addNewKeyFrameTsULong(unsigned long long timestamp_ns, std::uint64_t kfID, std::uint64_t mapID);
        /// @}

        /// @name Pose updates (PGO)
        /// O(1) reception, latest-wins, never blocks. Sophus / double overloads forward
        /// to the Affine3f form.
        /// @{
        /// @param kfID keyframe id. @param pose_map_base new pose (map <- base). @param numConnections unused.
        void updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3f &pose_map_base,
                            std::uint64_t numConnections = 0);
        /// @param kfID keyframe id. @param pose_map_base new pose (map <- base). @param numConnections unused.
        void updateKeyFrame(std::uint64_t kfID, const Sophus::SE3f &pose_map_base,
                            std::uint64_t numConnections = 0);
        /// @param kfID keyframe id. @param pose_map_base new pose (map <- base). @param numConnections unused.
        void updateKeyFrame(std::uint64_t kfID, const Eigen::Affine3d &pose_map_base,
                            std::uint64_t numConnections = 0);
        /// @}

        /// @name Lifecycle
        /// @{
        /// @brief Remove a keyframe (subtract its contribution and drop it).
        /// @param kfID keyframe id.
        void deleteKeyFrame(std::uint64_t kfID);
        /// @brief Re-parent a keyframe to another map (subtract from the old grid, re-add to the new).
        /// @param kfID keyframe id. @param mapID target map id.
        void updateKFMap(std::uint64_t kfID, std::uint64_t mapID);
        /// @brief Clear + rebuild map 0 (loop-closure response).
        void informLoopClosure();
        /// @}

        /// @name Obstacle layer (Nav2-style transient obstacles)
        /// Live sensor clouds added as independent keyframes into the active map, on a
        /// dedicated id band (INT64_MAX down, ids reused via a free-list) so they never
        /// collide with the SLAM front-end's low, upward ids. Each is binned once (no PGO);
        /// the caller ages them out with deleteObstacleKeyFrame after its temporal buffer.
        /// @{
        /// @brief Add a transient obstacle keyframe from a live obstacle-sensor cloud.
        ///        Allocates an obstacle-band id, prunes with the obstacle extrinsic + ranges,
        ///        and registers it in the active map. The caller then calls updateKeyFrame(id,
        ///        pose) to supply the map<-base pose and trigger the (only) bin.
        /// @param timestamp_ns acquisition time (ns). @param sensorPointCloud obstacle-sensor-frame cloud.
        /// @return the allocated keyframe id, or 0 if params were unset / the cloud pruned empty.
        std::uint64_t addObstacleKeyFrame(unsigned long long timestamp_ns,
                                          std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensorPointCloud);
        /// @brief Delete an obstacle keyframe (subtract its moments) and reclaim its id.
        /// @param kfID obstacle keyframe id previously returned by addObstacleKeyFrame.
        void deleteObstacleKeyFrame(std::uint64_t kfID);
        /// @}

        /// @name Buffers
        /// @{
        /// @brief Buffer a PCL cloud by timestamp for later fetch-by-time.
        /// @param timestamp acquisition time (s). @param pcl the cloud.
        void pushToBuffer(double timestamp, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl);
#ifdef WITH_ROS2_SENSOR_MSGS
        /// @brief Buffer a sensor_msgs cloud (timestamp read from its header).
        /// @param pcl the cloud.
        void pushToBuffer(sensor_msgs::msg::PointCloud2::SharedPtr pcl);
#endif
        /// @}

        /// @name Access
        /// @{
        /// @brief The active map. @return the active map, or nullptr if none created yet.
        std::shared_ptr<LocalMap> getLocalMap();
        /// @brief A map by id. @param mapID map id. @return the map, or nullptr if absent.
        std::shared_ptr<LocalMap> getLocalMap(std::uint64_t mapID);
        /// @brief Voxel-downsampled stitch of every map's retained clouds.
        /// @param voxel_size_x,voxel_size_y,voxel_size_z voxel leaf size (<=0 disables downsampling).
        /// @return the global cloud.
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getGlobalPointCloud(
            float voxel_size_x, float voxel_size_y, float voxel_size_z);
        /// @}

    private:
        void addNewKeyFrameToMap(double timestamp, std::uint64_t kfID, std::uint64_t mapID,
                                 const pcl::PointCloud<pcl::PointXYZ> &sensorPointCloud);
        /// @brief Register an already-pruned base-frame cloud as a keyframe in @p mapID.
        /// @return true if registered; false on duplicate id / missing map / empty cloud.
        bool registerKeyFrame(double timestamp, std::uint64_t kfID, std::uint64_t mapID,
                              std::vector<Eigen::Vector3f> &&cloud_base);
        void setCurrentMap(std::uint64_t mapID);
        std::vector<Eigen::Vector3f> pruneToBase(const pcl::PointCloud<pcl::PointXYZ> &cloud_lidar) const;
        /// @brief Prune a sensor-frame cloud into the base frame with an explicit extrinsic + gates.
        /// @param cloud_sensor sensor-frame cloud. @param Tb_sensor base <- sensor.
        /// @param maxRange,minRange base-frame range gates (m).
        std::vector<Eigen::Vector3f> prune(const pcl::PointCloud<pcl::PointXYZ> &cloud_sensor,
                                           const Eigen::Affine3f &Tb_sensor,
                                           double maxRange, double minRange) const;
        /// @brief Allocate an obstacle-band id (free-list first, else the next decrement).
        std::uint64_t allocObstacleId();
        static double nanosecToSec(unsigned long long timestamp_ns);

        Lattice lattice_;
        std::string mapFrame_ = "map";
        Eigen::Affine3f Tsv_ = Eigen::Affine3f::Identity();
        Eigen::Affine3f Tbs_ = Eigen::Affine3f::Identity();
        Eigen::Affine3f Tbv_ = Eigen::Affine3f::Identity();   ///< base <- lidar
        double robot_height_, max_range_base_frame_, min_range_base_frame_;

        /// @name Obstacle layer, guarded by localMapMutex_
        /// @{
        Eigen::Affine3f Tb_obs_ = Eigen::Affine3f::Identity();   ///< base <- obstacle sensor
        double obstacle_max_range_ = 0.0, obstacle_min_range_ = 0.0;
        bool obstacleParamsSet_ = false;
        /// Ids handed out top-down from INT64_MAX (stays under addNewKeyFrame's negative-id
        /// guard, never meets the SLAM band). freeObstacleIds_ recycles ids after deletion.
        std::uint64_t nextObstacleId_ =
            static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
        std::vector<std::uint64_t> freeObstacleIds_;
        /// @}

        std::recursive_mutex localMapMutex_;
        std::unordered_map<std::uint64_t, std::shared_ptr<LocalMap>> localMapsSet_;
        std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> keyFramesMap_;   ///< authoritative keyframe registry
        std::unordered_map<std::uint64_t, std::uint64_t> allKeyFramesSet_;            ///< kfID -> mapID routing
        std::shared_ptr<LocalMap> localMap_ = nullptr;                                ///< current/active map

        bool usePointCloudBuffer_ = false;
        bool useROSBuffer_ = false;
        std::shared_ptr<PointCloudBuffer> pointCloudBuffer_;
        std::shared_ptr<PointCloudBufferROS> pointCloudBufferROS_;
    };
}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_SYSTEM_HPP_
