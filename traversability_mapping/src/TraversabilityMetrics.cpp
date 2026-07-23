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

#include "traversability_mapping/TraversabilityMetrics.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace traversability_mapping
{
    std::array<double, HAZ_COUNT> computeGoodness(
        const CellMoment &query,
        const std::vector<CellMoment> &occupied,
        int vicinity_cell_count,
        double ground_clearance,
        double max_slope,
        unsigned int min_points_per_grid)
    {
        std::array<double, HAZ_COUNT> haz;
        haz.fill(std::numeric_limits<double>::quiet_NaN());

        if (query.data.N < 1)
            return haz;

        // Elevation is the query cell's own centroid height (map frame). It is
        // available whenever the cell has points, independent of the gates.
        haz[HAZ_ELEVATION] = query.data.sz / static_cast<double>(query.data.N)
                             + query.center.z();

        // border_haz reports the query cell's point-density completeness: it scales
        // linearly from 0 (empty) up to 1.0 once the cell carries the required minimum,
        // then saturates. Note the polarity: 1.0 means the cell HAS enough data, not
        // that it is a border. It is set before the gate so it is reported even when the
        // geometric fit below is rejected.
        haz[HAZ_BORDER] = std::min(static_cast<double>(query.data.N) /
                                       static_cast<double>(std::max(1u, min_points_per_grid)),
                                   1.0);

        // Gate for the geometric fit: the query cell and every cell under its footprint
        // must each carry at least min_points_per_grid points. A cell missing from
        // `occupied` is unobserved (zero points) and fails the same test, so any short or
        // empty cell in the vicinity leaves the fitted metrics NaN (border_haz stands).
        bool border = query.data.N < min_points_per_grid ||
                      static_cast<int>(occupied.size()) < vicinity_cell_count;
        for (const auto &c : occupied)
        {
            if (c.data.N < min_points_per_grid)
            {
                border = true;
                break;
            }
        }
        if (border)
            return haz;

        // Fuse the vicinity moments, re-expressing each cell about the query
        // cell centre (common origin) so the raw moments add cleanly.
        NodeMetaData fused;
        std::vector<Eigen::Vector3d> barycenters;
        barycenters.reserve(occupied.size());
        for (const auto &c : occupied)
        {
            NodeMetaData m = c.data;
            m.shift(c.center - query.center);  // re-centre to query origin
            barycenters.push_back(m.barycenter());
            fused.fuseWith(m);
        }

        const Eigen::Vector3d mu = fused.barycenter();
        const Eigen::Matrix3d cov = fused.Q() / static_cast<double>(fused.N)
                                    - mu * mu.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        // Eigenvalues are returned in ascending order: col(0) is the normal.
        Eigen::Vector3d n = es.eigenvectors().col(0);
        const double lambda_min = es.eigenvalues()(0);
        if (n.z() < 0.0)
            n = -n;
        n.normalize();

        // Expose the fitted surface normal components (map frame).
        haz[HAZ_NORMAL_X] = n.x();
        haz[HAZ_NORMAL_Y] = n.y();
        haz[HAZ_NORMAL_Z] = n.z();

        // Slope: tilt of the surface normal from map up.
        const double slope = std::acos(std::min(1.0, std::abs(n.z())));
        haz[HAZ_SLOPE] = std::min(slope / max_slope, 1.0);

        // Roughness: point-to-plane residual std (rotation-invariant).
        const double roughness = std::sqrt(std::max(0.0, lambda_min));
        haz[HAZ_ROUGHNESS] = std::min(roughness / ground_clearance, 1.0);

        // Step: spread of per-cell barycentres along the normal.
        double min_d = std::numeric_limits<double>::infinity();
        double max_d = -std::numeric_limits<double>::infinity();
        for (const auto &b : barycenters)
        {
            const double d = (b - mu).dot(n);
            min_d = std::min(min_d, d);
            max_d = std::max(max_d, d);
        }
        const double step = max_d - min_d;
        // Normalise like roughness so HAZ_STEP is a [0,1] cost (and HAZ_OVERALL no
        // longer mixes raw metres with normalised terms). This is also the field the
        // inflation layer seeds from.
        haz[HAZ_STEP] = std::min(step / ground_clearance, 1.0);

        haz[HAZ_OVERALL] = std::max(haz[HAZ_SLOPE],
                               std::max(haz[HAZ_STEP], haz[HAZ_ROUGHNESS]));
        return haz;
    }

}  // namespace traversability_mapping
