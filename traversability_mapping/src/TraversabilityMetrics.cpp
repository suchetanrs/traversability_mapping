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
        unsigned int min_vicinity_points,
        double min_occupied_fraction)
    {
        std::array<double, HAZ_COUNT> haz;
        haz.fill(std::numeric_limits<double>::quiet_NaN());

        if (query.data.N < 1)
            return haz;

        // Elevation is the query cell's own centroid height (map frame). It is
        // available whenever the cell has points, independent of the gates.
        haz[HAZ_ELEVATION] = query.data.sz / static_cast<double>(query.data.N)
                             + query.center.z();

        // Gate 1: enough of the surrounding cells must be observed, so a lone
        // dense cell cannot drive a fit while its neighbours are empty.
        double occupied_fraction;
        if (vicinity_cell_count > 0)
        {
            occupied_fraction = static_cast<double>(occupied.size()) / static_cast<double>(vicinity_cell_count);
        }
        else
        {
            occupied_fraction = 0.0;
        }

        if (occupied_fraction < min_occupied_fraction)
        {
            haz[HAZ_BORDER] = 1.0;
            return haz;
        }

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

        // Gate 2: enough points to fit a plane at all.
        if (fused.N < min_vicinity_points)
        {
            haz[HAZ_BORDER] = 1.0;
            return haz;
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
        haz[HAZ_STEP] = std::min(step / ground_clearance, 1.0);

        haz[HAZ_OVERALL] = std::max(haz[HAZ_SLOPE],
                               std::max(haz[HAZ_STEP], haz[HAZ_ROUGHNESS]));
        haz[HAZ_BORDER] = 0.0;
        return haz;
    }
}  // namespace traversability_mapping
