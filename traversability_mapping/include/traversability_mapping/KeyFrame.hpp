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
        KeyFrame(std::uint64_t kfID, double timestamp, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloudBase, std::uint64_t parentMapID = 0);

        /// Convenience overload (timestamp = 0, parentMapID = 0). Retained so the
        /// transitional monolithic node keeps compiling; prefer the primary ctor.
        KeyFrame(std::uint64_t kfID, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloudBase);

        const std::uint64_t &getKfID() const { return kfID_; }
        const double &getTimestamp() const { return timestamp_; }
        std::uint64_t getParentMapID() const { return parentMapID_; }

        /// Re-parent this keyframe to a different map. Only updates the recorded
        /// parent id; moving the partials between map grids is the caller's job
        /// (System / LocalMap).
        void setMap(std::uint64_t parentMapID) { parentMapID_ = parentMapID; }

        /// Store the newest pose (map <- base_footprint) AND flag the keyframe as
        /// needing a rebin + recompute. Cheap, non-blocking. Thread-safe against
        /// getPendingPose: a pose that arrives while one is being consumed simply
        /// waits for the (short) lock, so latestPose_ and hasPending_ never tear.
        void setPose(const Eigen::Affine3f &p);

        /// If a pose is pending, copy it into `out`, clear the pending flag, and
        /// return true; otherwise return false. Non-blocking, thread-safe against
        /// setPose. A true result is the signal to rebin() + recompute with `out`.
        bool getPendingPose(Eigen::Affine3f &out);

        /// The latest known pose (map <- base_footprint). Non-blocking copy.
        Eigen::Affine3f getPose() const;

        /// (Re)compute this keyframe's contribution: transform the stored
        /// base-frame cloud by `pose`, bin into cell-local moments on `lattice`
        /// (origin at each cell's centre, z about 0), and REPLACE the partials.
        /// Clears any previous partials first.
        void rebin(const Lattice &lattice, const Eigen::Affine3f &pose);

        std::unordered_map<std::uint64_t, NodeMetaData> &partials() { return partials_; }
        const std::unordered_map<std::uint64_t, NodeMetaData> &partials() const { return partials_; }

        bool empty() const { return partials_.empty(); }

        /// The retained pruned cloud in the robot base frame (empty once dropped).
        /// Used by LocalMap::getStitchedPointCloud to rebuild map-frame geometry.
        const std::vector<Eigen::Vector3f> &cloudBase() const { return cloudBase_; }

        /// Whether the pruned base-frame cloud is still retained (false once it has
        /// been dropped under !is_kf_optimization_enabled — keyframe then cannot be
        /// re-binned).
        bool hasCloud() const { return !cloudBase_.empty(); }
        /// Drop the retained cloud to reclaim memory; the keyframe can no longer be
        /// re-binned afterwards.
        void dropCloud() { cloudBase_.clear(); cloudBase_.shrink_to_fit(); }

    private:
        // set only once in the constructor; thread-safety not needed (read-only).
        std::uint64_t kfID_;
        double timestamp_ = 0.0;
        std::uint64_t parentMapID_ = 0;
        std::vector<Eigen::Vector3f> cloudBase_;                     ///< pruned cloud, base frame
        std::unordered_map<std::uint64_t, NodeMetaData> partials_;   ///< cellId -> cell-local moments

        // latestPose_ and hasPending_ are written together in setPose and consumed
        // together in getPendingPose, both under poseMutex_. "Has a pending pose"
        // IS the "needs rebin + recompute" signal -- there is no separate dirty flag.
        mutable std::mutex poseMutex_;
        Eigen::Affine3f latestPose_;                                 ///< map <- base_footprint
        bool hasPending_ = false;                                    ///< latestPose_ awaits a rebin
    };
}  // namespace traversability_mapping

#endif  // KEYFRAME_HPP_
