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

#ifndef TRAVERSABILITY_LOCALMAP_HPP_
#define TRAVERSABILITY_LOCALMAP_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "traversability_mapping/Moments.hpp"
#include "traversability_mapping/Helpers.hpp"
#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    /**
     * @brief Per-submap moment-fused map: one growing grid_map, its keyframes, and two
     *        worker threads that turn keyframe contributions into traversability.
     *
     * Workers: RunLocalKeyFrames drains the pose work queue (fast); RunTraversability is
     * a backstop that sweeps all keyframes. Both run the same op: subtract old partials
     * -> rebin -> add -> recompute hazards over the dilated touched cells.
     *
     * Three independent locks, nested in this order: masterGridMapMutex_ (grid +
     * changed-cell set) -> keyFramesMapMutex_ (keyframe registry) -> poseQueueMutex_
     * (pose work queue). setKeyFramePose takes only the queue lock, so pose updates never
     * block a recompute.
     *
     * ROS-free and output-format agnostic: it exposes the grid, its mutex, the lattice,
     * and the changed-cell set; the adapter decides which layers to publish.
     */
    class LocalMap
    {
    public:
        /// @brief Construct a map and start its two worker threads.
        /// @param mapID this map's id.
        /// @param lattice shared absolute lattice (same across maps so partials transfer on re-parent).
        /// @param mapFrame grid frame id (cosmetic in the core).
        /// @param onUpdate invoked by a worker after each grid-changing op, outside the grid lock.
        LocalMap(std::uint64_t mapID, const Lattice &lattice, std::string mapFrame = "map",
                 std::function<void()> onUpdate = {});
        ~LocalMap();

        LocalMap(const LocalMap &) = delete;
        LocalMap &operator=(const LocalMap &) = delete;

        /// @name Keyframe lifecycle
        /// @{

        /// @brief Register a keyframe in the working set (does NOT flag it pending).
        /// @param kf keyframe to register; a pose is supplied later via setKeyFramePose.
        void addAlreadyDeclaredKF(const std::shared_ptr<KeyFrame> &kf);

        /// @brief Set a keyframe's latest pose and enqueue it for rebin (queue lock only).
        /// @param kf keyframe to update (no-op if null); passed directly to avoid a registry lookup.
        /// @param pose new pose (map <- base_footprint).
        void setKeyFramePose(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose);

        /// @brief Subtract a keyframe's contribution and remove it from the working set.
        /// @param id keyframe id. @return the detached keyframe, or nullptr if unknown.
        std::shared_ptr<KeyFrame> detachKeyFrame(std::uint64_t id);

        /// @brief detachKeyFrame + discard (the caller releases the last reference).
        /// @param id keyframe id.
        void deleteKeyFrame(std::uint64_t id) { (void)detachKeyFrame(id); }

        /// @brief Blank the grid and rebuild from retained clouds (loop-closure response).
        ///        Keyframes whose cloud was dropped cannot be rebuilt.
        void clearEntireMap();

        /// @brief Read-only view of the keyframe registry.
        const std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> &getKeyFramesMap() const
        {
            return keyFramesMap_;
        }
        /// @}

        /// @name Outputs
        /// The adapter holds getGridMapMutex(), takes the cell keys (takeChangedCells /
        /// occupiedCellKeys), then reads layers from getGridMap() at each key via
        /// getLattice() — all under one held lock so keys and values stay consistent.
        /// @{

        /// @brief The live grid (hold getGridMapMutex() while accessing it).
        const grid_map::GridMap &getGridMap() const { return gridMap_; }
        /// @brief The mutex guarding the grid and the changed-cell set.
        std::mutex &getGridMapMutex() { return masterGridMapMutex_; }

        /// @brief The absolute lattice (cell ids <-> map-frame positions), immutable after construction.
        const Lattice &getLattice() const { return lattice_; }

        /// @brief Cell ids changed since the last call; clears the set.
        /// @return the changed cell ids. @note caller must hold getGridMapMutex().
        std::vector<std::uint64_t> takeChangedCells();

        /// @brief Cell ids of every occupied (N>=1) cell, for a full snapshot.
        /// @return the occupied cell ids. @note caller must hold getGridMapMutex().
        std::vector<std::uint64_t> occupiedCellKeys() const;

        /// @brief Voxel-downsampled stitch of every retained keyframe cloud, in the map frame.
        /// @param voxel_size_x,voxel_size_y,voxel_size_z voxel leaf size (<=0 disables downsampling).
        /// @return the stitched cloud.
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getStitchedPointCloud(
            float voxel_size_x, float voxel_size_y, float voxel_size_z);
        /// @}

    private:
        /// @brief Grow the grid to cover every cell in @p partials.
        /// @param partials cells that must fit in the grid.
        void growToIncludeCells(const std::unordered_map<std::uint64_t, NodeMetaData> &partials);

        /// @brief Add (sign>0) or subtract (sign<0) one keyframe's cell moment.
        /// @param cellId target cell. @param m the moment. @param sign +1 to add, -1 to subtract.
        /// @param layers pre-resolved moment layers of gridMap_ (rebuild after any grow).
        void addPartialToGrid(std::uint64_t cellId, const NodeMetaData &m, double sign,
                              const MomentLayers &layers);

        /// @name Recompute (caller must hold masterGridMapMutex_)
        /// @{
        /// @brief Recompute one cell's hazards from its vicinity.
        /// @param id cell id. @param layers pre-resolved moment layers of gridMap_.
        /// @return true if the cell was (re)written.
        bool recomputeCell(std::uint64_t id, const MomentLayers &layers);
        /// @brief Recompute every cell in @p dirty, recording those that changed.
        /// @param dirty cell ids to recompute.
        void recomputeDirty(const std::unordered_set<std::uint64_t> &dirty);
        /// @}

        /// @brief Per-keyframe op: subtract old, rebin at @p pose, add, recompute hazards.
        ///        Manages its own locking (caller must NOT hold masterGridMapMutex_).
        /// @param kf keyframe to (re)process. @param pose pose to bin at.
        /// @return true iff it (re)wrote the grid (false on a skip).
        bool recomputeKeyFrame(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose);

        /// @name Worker threads
        /// @{
        /// @brief Fast worker: drain the pose work queue.
        void RunLocalKeyFrames();
        /// @brief Slow backstop worker: sweep all keyframes for a pending pose.
        void RunTraversability();
        /// @}

        /// @name Identity / config
        /// @{
        std::uint64_t mapID_;
        Lattice lattice_;
        std::string frameId_;
        double res_;
        double groundClearance_, maxSlope_, minOccupiedFraction_;
        unsigned int minVicinityPoints_;
        double robotRadius_;                                 ///< footprint radius (m)
        std::vector<std::pair<int, int>> discOffsets_;       ///< footprint as cell offsets; the fit vicinity
        int globalSleepMs_;
        bool kfOptimizationEnabled_;
        std::function<void()> onUpdate_;        ///< fired after each grid-changing keyframe op
        std::vector<std::string> layers_;       ///< all internal layers (moments + derived)
        /// @}

        /// @name Grid state, guarded by masterGridMapMutex_
        /// Taken only in short bursts around grid touches (a recompute releases it around
        /// the keyframe-local rebin); pose updates never take it.
        /// @{
        std::mutex masterGridMapMutex_;
        grid_map::GridMap gridMap_;
        std::unordered_set<std::uint64_t> changedCells_;  ///< cell ids changed since last drain
        /// @}

        /// @name Keyframe registry, guarded by keyFramesMapMutex_
        /// Separate lock so an add/detach never waits on the grid lock.
        /// @{
        std::mutex keyFramesMapMutex_;
        std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> keyFramesMap_;
        /// @}

        /// @name Pose work queue, guarded by poseQueueMutex_
        /// Held only for O(1) enqueue/dequeue; holds shared_ptrs so the worker needs no
        /// registry lookup to drain it.
        /// @{
        std::mutex poseQueueMutex_;
        std::deque<std::shared_ptr<KeyFrame>> localQueue_;   ///< keyframes awaiting rebin; depth == backlog
        std::unordered_set<std::uint64_t> localQueued_;      ///< membership set (kf ids): dedup enqueues
        /// @}

        /// @name Threads
        /// @{
        std::atomic<bool> running_{true};
        std::thread localThread_;
        std::thread globalThread_;
        /// @}
    };
}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_LOCALMAP_HPP_
