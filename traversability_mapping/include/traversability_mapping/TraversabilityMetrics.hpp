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

/**
 * @file TraversabilityMetrics.hpp
 * @brief Traversability metrics computed from fused cell moments.
 */

#include <array>
#include <vector>
#include <cstddef>

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    /// @brief Indices into the result array returned by computeGoodness.
    enum Hazard : std::size_t
    {
        HAZ_OVERALL = 0,
        HAZ_BORDER = 1,
        HAZ_ELEVATION = 2,
        HAZ_SLOPE = 3,
        HAZ_STEP = 4,
        HAZ_ROUGHNESS = 5,
        HAZ_NORMAL_X = 6,  ///< Surface normal component (map frame) of the fitted plane.
        HAZ_NORMAL_Y = 7,  ///< Surface normal component (map frame) of the fitted plane.
        HAZ_NORMAL_Z = 8,  ///< Surface normal component (map frame) of the fitted plane.
        HAZ_COUNT = 9
    };

    /**
     * @brief Per-cell traversability from a PCA over ALL points in the vicinity.
     *
     * Components (indexed by ::Hazard):
     *   - slope     = arccos(|n_z|) / max_slope
     *   - roughness = sqrt(lambda_min) / ground_clearance
     *   - step      = (max-min of cell centroid) / ground_clearance
     *   - elevation = query cell centroid z
     *
     * @param query               the query cell's own moments + centre (must be occupied).
     * @param occupied            occupied cells in the vicinity window (must include the
     *                            query) — these are the neighbours fused for the fit.
     * @param vicinity_cell_count total cells in the window incl. empty ones, for the
     *                            border gate.
     * @param ground_clearance    normaliser for the roughness and step hazards (m).
     * @param max_slope           normaliser for the slope hazard (rad).
     * @param min_points_per_grid minimum point count each vicinity cell (and the query
     *                            cell) must carry for the geometric fit; any short or
     *                            missing cell leaves the fitted metrics NaN. It is also
     *                            the saturation count for HAZ_BORDER.
     * @return the ::HAZ_COUNT hazard values. HAZ_BORDER is the query cell's density
     *         completeness min(N / min_points_per_grid, 1): 1.0 == enough data, a
     *         fraction when sparse, NaN when the query cell is unobserved. Every other
     *         component (elevation and the fitted metrics) is NaN whenever the vicinity
     *         fails the point-count gate.
     */
    std::array<double, HAZ_COUNT> computeGoodness(
        const CellMoment &query,
        const std::vector<CellMoment> &occupied,
        int vicinity_cell_count,
        double ground_clearance,
        double max_slope,
        unsigned int min_points_per_grid);

}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_TRAVERSABILITYMETRICS_HPP_
