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
#ifndef TRAVERSABILITY_TRAVERSABILITYMETRICS_HPP_
#define TRAVERSABILITY_TRAVERSABILITYMETRICS_HPP_

// Traversability metrics computed from fused cell moments.

#include <array>
#include <vector>
#include <cstddef>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    /// Indices into the result array returned by computeGoodness.
    enum Hazard : std::size_t
    {
        HAZ_OVERALL = 0,
        HAZ_BORDER = 1,
        HAZ_ELEVATION = 2,
        HAZ_SLOPE = 3,
        HAZ_STEP = 4,
        HAZ_ROUGHNESS = 5,
        // Surface normal components (map frame) of the fitted plane.
        HAZ_NORMAL_X = 6,
        HAZ_NORMAL_Y = 7,
        HAZ_NORMAL_Z = 8,
        HAZ_COUNT = 9
    };

    /// PCA over ALL points in the vicinity:
    ///   slope     = arccos(|n_z|) / max_pitch
    ///   roughness = sqrt(lambda_min) / ground_clearance
    ///   step      = (max-min of cell centroid) / ground_clearance
    ///   elevation = query cell centroid z
    /// Returns NaN entries and a non-zero HAZ_BORDER when a cell is rejected.
    ///
    /// @param query             the query cell's own moments + centre (must be occupied)
    /// @param occupied          occupied cells in the vicinity window (must include query) - these are the neighbours
    /// @param vicinity_cell_count total cells in the window incl. empty (for the
    ///                          occupied-fraction gate)
    std::array<double, HAZ_COUNT> computeGoodness(
        const CellMoment &query,
        const std::vector<CellMoment> &occupied,
        int vicinity_cell_count,
        double ground_clearance,
        double max_pitch,
        unsigned int min_vicinity_points,
        double min_occupied_fraction);

}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_TRAVERSABILITYMETRICS_HPP_
