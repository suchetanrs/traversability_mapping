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

// Lean, ROS-free keyframe record for the moment-fused global map.
//
// A keyframe owns:
//   * its latest (post-PGO) pose (map <- base_footprint),
//   * its pruned raw cloud in the ROBOT BASE frame (pose-invariant, rigidly
//     attached to the robot, so it stays valid across any pose correction),
//   * the per-cell moments it currently contributes to the global grid, keyed by
//     ABSOLUTE lattice cell id.
//
// The partition is NOT frozen. On a PGO pose correction the node re-transforms
// the stored base-frame cloud by the new pose and RE-BINS from scratch via
// rebin(): the cell membership is recomputed each time (handles both relabel and
// split exactly), regardless of how small the pose change is. The partials' keys
// ARE the "list of cells it wrote to" (set M) and are used by the node to
// subtract the keyframe's previous contribution before re-adding the new one.
//
// The node owns the global grid_map and performs all grid arithmetic; the
// keyframe never touches the grid directly. The keyframe is kept PCL-free
// (Eigen only) so it can be unit-tested in isolation.

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <mutex>
#include <Eigen/Geometry>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    class KeyFrame
    {
    public:
        /// Primary constructor. `cloud_base` is the already-pruned cloud expressed
        /// in the robot base frame; it is moved in and retained for re-binning
        /// under PGO. `timestamp` is the acquisition time (seconds); `parentMapID`
        /// is the map this keyframe currently belongs to.
        KeyFrame(std::uint64_t id, double timestamp, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloud_base, std::uint64_t parentMapID = 0);

        /// Convenience overload (timestamp = 0, parentMapID = 0). Retained so the
        /// transitional monolithic node keeps compiling; prefer the primary ctor.
        KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloud_base);

        std::uint64_t id() const { return id_; }
        double timestamp() const { return timestamp_; }
        std::uint64_t parentMapID() const { return parentMapID_; }

        const Eigen::Affine3f &pose() const { return pose_; }
        /// Cheap, non-blocking. Does NOT re-bin; call rebin() explicitly after.
        void setPose(const Eigen::Affine3f &p);

        /// Re-parent this keyframe to a different map. Only updates the recorded
        /// parent id; moving the partials between map grids is the caller's job
        /// (System / LocalMap).
        void setMap(std::uint64_t parentMapID) { parentMapID_ = parentMapID; }

        // --- latest-wins pending-pose slot (PGO reception) ----------------------
        // The reception path (System::updateKeyFrame) writes the newest corrected
        // pose here in O(1); a worker later swaps it out and commits via setPose().
        // A newer correction overwrites an unconsumed one (latest-wins), which
        // collapses bursts and bounds memory.

        /// Store the newest corrected pose (overwrites any unconsumed one).
        void setPendingPose(const Eigen::Affine3f &p);

        /// If a pending pose is present, move it into `out`, clear the slot, and
        /// return true; otherwise return false. Thread-safe against setPendingPose.
        bool takePendingPose(Eigen::Affine3f &out);

        // --- dirty / claim primitive (work dispatch) ----------------------------
        // Single source of truth for "this keyframe needs (re)processing". A worker
        // claims via claim(): the first caller to flip dirty true->false wins and
        // owns the work, so the two LocalMap workers never double-process a frame.

        /// Mark the keyframe as needing (re)processing.
        void markDirty() { dirty_.store(true, std::memory_order_release); }
        /// Atomically take ownership of pending work: returns true exactly once per
        /// markDirty() across concurrent callers.
        bool claim() { bool expected = true; return dirty_.compare_exchange_strong(expected, false); }
        bool isDirty() const { return dirty_.load(std::memory_order_acquire); }

        /// (Re)compute this keyframe's contribution: transform the stored
        /// base-frame cloud by the current pose, bin into cell-local moments on
        /// `lattice` (origin at each cell's centre, z about 0), and REPLACE the
        /// partials. Clears any previous partials first.
        void rebin(const Lattice &lattice);

        std::unordered_map<std::uint64_t, NodeMetaData> &partials() { return partials_; }
        const std::unordered_map<std::uint64_t, NodeMetaData> &partials() const { return partials_; }

        bool empty() const { return partials_.empty(); }

        /// Whether the pruned base-frame cloud is still retained (false once it has
        /// been dropped under !is_kf_optimization_enabled — keyframe then cannot be
        /// re-binned).
        bool hasCloud() const { return !cloud_base_.empty(); }
        /// Drop the retained cloud to reclaim memory; the keyframe can no longer be
        /// re-binned afterwards.
        void dropCloud() { cloud_base_.clear(); cloud_base_.shrink_to_fit(); }

    private:
        std::uint64_t id_;
        double timestamp_ = 0.0;
        std::uint64_t parentMapID_ = 0;
        Eigen::Affine3f pose_;                                       ///< map <- base_footprint
        std::vector<Eigen::Vector3f> cloud_base_;                    ///< pruned cloud, base frame
        std::unordered_map<std::uint64_t, NodeMetaData> partials_;   ///< cellId -> cell-local moments

        std::mutex slotMutex_;                                       ///< guards the pending-pose slot
        Eigen::Affine3f pendingPose_;                                ///< newest unconsumed PGO pose
        bool hasPending_ = false;

        std::atomic<bool> dirty_{false};                             ///< needs (re)processing
    };
}  // namespace traversability_mapping

#endif  // KEYFRAME_HPP_
