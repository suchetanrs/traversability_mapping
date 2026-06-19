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
#include <Eigen/Geometry>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    class KeyFrame
    {
    public:
        /// `cloud_base` is the already-pruned cloud expressed in the robot base
        /// frame; it is moved in and retained for re-binning under PGO.
        KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base,
                 std::vector<Eigen::Vector3f> &&cloud_base);

        std::uint64_t id() const { return id_; }

        const Eigen::Affine3f &pose() const { return pose_; }
        /// Cheap, non-blocking. Does NOT re-bin; call rebin() explicitly after.
        void setPose(const Eigen::Affine3f &p);

        /// (Re)compute this keyframe's contribution: transform the stored
        /// base-frame cloud by the current pose, bin into cell-local moments on
        /// `lattice` (origin at each cell's centre, z about 0), and REPLACE the
        /// partials. Clears any previous partials first.
        void rebin(const Lattice &lattice);

        std::unordered_map<std::uint64_t, NodeMetaData> &partials() { return partials_; }
        const std::unordered_map<std::uint64_t, NodeMetaData> &partials() const { return partials_; }

        bool empty() const { return partials_.empty(); }

    private:
        std::uint64_t id_;
        Eigen::Affine3f pose_;                                       ///< map <- base_footprint
        std::vector<Eigen::Vector3f> cloud_base_;                    ///< pruned cloud, base frame
        std::unordered_map<std::uint64_t, NodeMetaData> partials_;   ///< cellId -> cell-local moments
    };
}  // namespace traversability_mapping

#endif  // KEYFRAME_HPP_
