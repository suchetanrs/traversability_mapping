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
// A keyframe owns ONLY its own contribution to the global grid:
//   * its latest (post-PGO) pose,
//   * the per-cell moments it added, keyed by ABSOLUTE lattice cell id.
//
// The cell-id keys ARE the "list of cells it wrote to" (set M). They are stored
// exactly (no neighbours) so the contribution can be subtracted out exactly when
// the keyframe's pose is later corrected. The node owns the global grid_map and
// performs all grid arithmetic; the keyframe never touches the grid directly.
//
// The raw lidar cloud is intentionally NOT stored yet (the partition is frozen,
// so no re-binning is needed for small PGO updates). It will be added back when
// true re-binning for large pose changes is implemented.

#include <cstdint>
#include <unordered_map>
#include <Eigen/Geometry>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    class KeyFrame
    {
    public:
        KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base);

        std::uint64_t id() const { return id_; }

        const Eigen::Affine3f &pose() const { return pose_; }
        void setPose(const Eigen::Affine3f &p);

        /// Accumulate one point (already expressed in the cell-local frame of
        /// `cellId`, i.e. relative to that cell's centre) into this keyframe's
        /// contribution.
        void addPoint(std::uint64_t cellId, double lx, double ly, double lz);

        std::unordered_map<std::uint64_t, NodeMetaData> &partials() { return partials_; }
        const std::unordered_map<std::uint64_t, NodeMetaData> &partials() const { return partials_; }

        bool empty() const { return partials_.empty(); }

    private:
        std::uint64_t id_;
        Eigen::Affine3f pose_;                                       ///< map <- base_footprint
        std::unordered_map<std::uint64_t, NodeMetaData> partials_;   ///< cellId -> cell-local moments
    };
}  // namespace traversability_mapping

#endif  // KEYFRAME_HPP_
