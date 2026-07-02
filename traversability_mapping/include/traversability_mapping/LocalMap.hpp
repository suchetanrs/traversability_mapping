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

// Per-submap moment-fused map. Owns ONE fixed-frame, growing grid_map whose layers
// hold the fused moments + derived hazard/nav layers, the keyframes routed to it,
// and TWO worker threads that turn keyframe contributions into traversability:
//
//   * RunLocalKeyFrames - drains the local work queue (keyframes enqueued by
//                          setKeyFramePose), fast.
//   * RunTraversability  - a backstop that sweeps ALL keyframes, slower; a keyframe
//                          with no pending pose is skipped (getPendingPose() returns
//                          false), so it catches anything not drained via the queue.
//
// Both run the same per-keyframe op (subtract old partials -> rebin at the pending
// pose -> add -> recompute hazards over the dilated touched cells). Locking uses TWO
// independent mutexes: masterGridMapMutex_ (the heavy lock, held for the whole
// recompute) guards the grid + keyframe registry; poseQueueMutex_ (a light lock) guards
// only the pose work queue. setKeyFramePose takes ONLY the queue lock, so a pose update
// never blocks behind an in-progress recompute. getPendingPose() hands the pending pose
// to exactly one worker and clears the flag, so a double-touch by both workers is
// idempotent.
//
// ROS-free and output-format agnostic: LocalMap owns the whole grid_map and the set
// of cell ids changed since the last drain, and exposes the grid + its mutex + the
// lattice + the changed-cell set. It has NO notion of "nav layers" or any publish
// format -- the ROS adapter decides which layers to publish for the changed cells.
// The keyframe never touches the grid; LocalMap performs all grid arithmetic.

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
#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    class LocalMap
    {
    public:
        /// `lattice` is the shared absolute lattice (same across all maps so a
        /// keyframe's partials transfer on re-parent). `mapFrame` is the grid frame
        /// id (cosmetic in the core; the adapter sets the published header frame).
        /// `onUpdate`, if set, is invoked by a worker AFTER it finishes a keyframe
        /// op and AFTER releasing the grid lock (so the callback may read the grid /
        /// drain the changed-cell set without deadlocking). Used by the adapter to
        /// publish on every recompute. Must be set at construction (read by the workers).
        LocalMap(std::uint64_t mapID, const Lattice &lattice, std::string mapFrame = "map",
                 std::function<void()> onUpdate = {});
        ~LocalMap();

        LocalMap(const LocalMap &) = delete;
        LocalMap &operator=(const LocalMap &) = delete;

        std::uint64_t mapID() const { return mapID_; }

        // --- keyframe lifecycle ------------------------------------------------

        /// Register a keyframe in this map's working set. Does NOT flag it pending or
        /// enqueue it; the caller sets a pose (setKeyFramePose) once it is ready to be
        /// binned. Used for both fresh additions and re-parented KFs, and when multiple
        /// maps are fused together.
        void addAlreadyDeclaredKF(const std::shared_ptr<KeyFrame> &kf);

        /// Set a keyframe's latest pose AND push it onto the local work queue so a
        /// worker rebins it. This is the single entry point for pose updates: routing
        /// them through the map (rather than KeyFrame::setPose directly) is what lets
        /// the local worker be a real work queue whose depth == the rebin backlog.
        /// Takes ONLY poseQueueMutex_ (never the grid lock), so it never blocks behind
        /// a recompute. The caller passes the keyframe directly (it already holds the
        /// shared_ptr), so no grid-locked keyFramesMap_ lookup is needed. Cheap,
        /// non-blocking; no-op if `kf` is null.
        void setKeyFramePose(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose);

        /// Subtract a keyframe's contribution from the grid (recomputing the cells
        /// it vacated), remove it from the working set/window, and return it (or
        /// nullptr). Used by delete and by re-parent-out.
        std::shared_ptr<KeyFrame> detachKeyFrame(std::uint64_t id);

        /// detachKeyFrame + discard (the caller releases the last reference).
        void deleteKeyFrame(std::uint64_t id) { (void)detachKeyFrame(id); }

        /// Clear the whole grid and rebuild from retained clouds: blank every layer,
        /// record every occupied cell as changed, clear every keyframe's partials,
        /// and flag them pending so the workers re-add them. Keyframes whose cloud was
        /// dropped (optimization off) cannot be rebuilt.
        void clearEntireMap();

        const std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> &getKeyFramesMap() const
        {
            return keyFramesMap_;
        }

        // --- outputs -----------------------------------------------------------
        //
        // LocalMap hands out the raw grid + the set of changed cell ids; it does NOT
        // know which layers a consumer publishes. To build any sparse/full update the
        // adapter holds getGridMapMutex(), takes the cell keys (takeChangedCells /
        // occupiedCellKeys), then reads whatever layers it wants from getGridMap() at
        // each key's position via getLattice(). Doing it under one held lock keeps the
        // key set and the values it reads consistent.

        /// Live grid + its mutex. The caller must hold getGridMapMutex() while
        /// touching getGridMap() or calling takeChangedCells / occupiedCellKeys.
        const grid_map::GridMap &getGridMap() const { return gridMap_; }
        std::mutex &getGridMapMutex() { return masterGridMapMutex_; }

        /// The absolute lattice (maps cell ids <-> map-frame positions). Immutable
        /// after construction.
        const Lattice &getLattice() const { return lattice_; }

        /// Cell ids changed since the last call; clears the set. CALLER MUST HOLD
        /// getGridMapMutex() (so the keys stay consistent with the grid it reads).
        std::vector<std::uint64_t> takeChangedCells();

        /// Cell ids of every occupied (N>=1) cell, for a full snapshot. CALLER MUST
        /// HOLD getGridMapMutex().
        std::vector<std::uint64_t> occupiedCellKeys() const;

        /// Voxel-downsampled stitch of every retained keyframe cloud in the map
        /// frame (legacy getStitchedPointCloud).
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getStitchedPointCloud(
            float voxel_size_x, float voxel_size_y, float voxel_size_z);

    private:
        // ---- grid growth (delegates to growGridToInclude in Helpers.hpp) ----
        void growToIncludeCells(const std::unordered_map<std::uint64_t, NodeMetaData> &partials);

        // ---- moment -> grid ----
        void addPartialToGrid(std::uint64_t cellId, const NodeMetaData &m, double sign);

        // ---- recompute ----
        bool recomputeCell(std::uint64_t id);
        void recomputeDirty(const std::unordered_set<std::uint64_t> &dirty);

        // ---- per-keyframe op (caller must NOT hold masterGridMapMutex_) ----
        // Returns true iff it actually (re)wrote the grid (false on a skip), so the
        // worker only fires onUpdate_ when something changed.
        bool recomputeKeyFrame(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose);

        // ---- worker threads ----
        void RunLocalKeyFrames();
        void RunTraversability();

        // ---- identity / config ----
        std::uint64_t mapID_;
        Lattice lattice_;
        std::string frameId_;
        double res_;
        double groundClearance_, maxSlope_, minOccupiedFraction_;
        unsigned int minVicinityPoints_;
        int deltaInd_;
        int globalSleepMs_;
        bool kfOptimizationEnabled_;
        std::function<void()> onUpdate_;        // fired after each grid-changing keyframe op

        std::vector<std::string> layers_;       // all internal layers (moments + derived)

        // ---- grid + registry state, guarded by masterGridMapMutex_ ----
        // recomputeKeyFrame releases it around the keyframe-local rebin, so a long rebin
        // no longer blocks grid readers. Pose updates never take it at all.
        std::mutex masterGridMapMutex_;          // guards gridMap_, changedCells_
        grid_map::GridMap gridMap_;
        std::unordered_set<std::uint64_t> changedCells_;  // cell ids changed since last drain

        // keyFramesMap_ has its OWN light lock so a keyframe add/detach never waits on
        // the grid lock (held across the whole recompute). Lock order where both are
        // held: masterGridMapMutex_ -> keyFramesMapMutex_.
        std::mutex keyFramesMapMutex_;
        std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> keyFramesMap_;

        // ---- pose work queue, guarded by its OWN light lock ----
        // Held only for O(1) enqueue/dequeue, never during a recompute. The queue
        // holds shared_ptrs so the worker needs no keyFramesMap_ lookup to drain it.
        std::mutex poseQueueMutex_;              // guards localQueue_, localQueued_
        std::deque<std::shared_ptr<KeyFrame>> localQueue_;   // keyframes awaiting rebin; depth == backlog
        std::unordered_set<std::uint64_t> localQueued_;      // membership set (kf ids): dedup enqueues

        // ---- threads ----
        std::atomic<bool> running_{true};
        std::thread localThread_;
        std::thread globalThread_;
    };
}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_LOCALMAP_HPP_
