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
#ifndef TRAVERSABILITY_MOMENTS_HPP_
#define TRAVERSABILITY_MOMENTS_HPP_

// Header-only, Eigen-only core of the moment-fused traversability map.
//
// Everything here is deliberately free of ROS / grid_map / PCL so it can be
// unit-tested in isolation. It implements:
//   * NodeMetaData : per-cell sufficient statistics (raw moments) of the points
//                    that fall in a cell, expressed in CELL-LOCAL, MAP-ALIGNED
//                    coordinates (origin at the cell centre) for numerical
//                    conditioning and exact cross-keyframe summation.
//   * Lattice      : the fixed global cell lattice (absolute cell ids).
//   * computeGoodness : PCA plane fit over the fused vicinity moments -> the
//                    slope / step / roughness / elevation hazards.
//
// The maths (moment transform formulas, Sigma = Q/N - mu mu^T, PCA normal) is
// documented in obsidian_vault .../Traversability/traversability_moments_verbatim.md

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>

namespace traversability_mapping
{
    /// Raw (un-centred) moments of a point set, about a fixed local origin.
    ///
    /// Because the moments are raw sums they add unconditionally as long as
    /// every contribution is expressed in the SAME frame (same origin, same
    /// axes). We always express them in the cell's own centre with map-aligned
    /// axes, so contributions from different keyframes to the same cell sum
    /// directly, and a keyframe's contribution can be subtracted exactly later.
    class NodeMetaData
    {
    public:
        unsigned int N = 0;
        // S = sum(p)
        double sx = 0., sy = 0., sz = 0.;
        // Q = sum(p p^T) : diagonal then off-diagonal
        double sx2 = 0., sy2 = 0., sz2 = 0.;
        double sxy = 0., sxz = 0., syz = 0.;

        void reset() { *this = NodeMetaData(); }

        bool empty() const { return N == 0; }

        /// Accumulate one point given in this node's local (cell-centred) frame.
        inline void insert(double x, double y, double z)
        {
            ++N;
            sx += x;  sy += y;  sz += z;
            sx2 += x * x;  sy2 += y * y;  sz2 += z * z;
            sxy += x * y;  sxz += x * z;  syz += y * z;
        }

        /// Add another contribution. Requires the same local frame.
        inline void fuseWith(const NodeMetaData &o)
        {
            N += o.N;
            sx += o.sx;  sy += o.sy;  sz += o.sz;
            sx2 += o.sx2;  sy2 += o.sy2;  sz2 += o.sz2;
            sxy += o.sxy;  sxz += o.sxz;  syz += o.syz;
        }

        /// Exact inverse of fuseWith (used to back a keyframe out on a PGO update).
        inline void removeWith(const NodeMetaData &o)
        {
            N -= o.N;
            sx -= o.sx;  sy -= o.sy;  sz -= o.sz;
            sx2 -= o.sx2;  sy2 -= o.sy2;  sz2 -= o.sz2;
            sxy -= o.sxy;  sxz -= o.sxz;  syz -= o.syz;
        }

        Eigen::Vector3d S() const { return Eigen::Vector3d(sx, sy, sz); }

        Eigen::Matrix3d Q() const
        {
            Eigen::Matrix3d m;
            m << sx2, sxy, sxz,
                 sxy, sy2, syz,
                 sxz, syz, sz2;
            return m;
        }

        void setSQ(const Eigen::Vector3d &s, const Eigen::Matrix3d &q)
        {
            sx = s.x();  sy = s.y();  sz = s.z();
            sx2 = q(0, 0);  sy2 = q(1, 1);  sz2 = q(2, 2);
            sxy = q(0, 1);  sxz = q(0, 2);  syz = q(1, 2);
        }

        /// Centroid in the local frame (add the cell centre to get map coords).
        Eigen::Vector3d barycenter() const { return S() / static_cast<double>(N); }

        /// Covariance Sigma = Q/N - mu mu^T (origin-invariant).
        Eigen::Matrix3d covariance() const
        {
            const Eigen::Vector3d mu = barycenter();
            return Q() / static_cast<double>(N) - mu * mu.transpose();
        }

        /// Apply a rigid transform to the underlying points: p -> R p + t,
        /// expressed in this node's local frame. Exact, no points needed:
        ///   S' = R S + N t
        ///   Q' = R Q R^T + R S t^T + t (R S)^T + N t t^T
        void transform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
        {
            const Eigen::Vector3d s = S();
            const Eigen::Matrix3d q = Q();
            const double n = static_cast<double>(N);
            const Eigen::Vector3d Rs = R * s;
            const Eigen::Vector3d s2 = Rs + n * t;
            const Eigen::Matrix3d q2 = R * q * R.transpose()
                                     + Rs * t.transpose()
                                     + t * Rs.transpose()
                                     + n * t * t.transpose();
            setSQ(s2, q2);
        }

        /// Re-express the moments about a new origin shifted so that the new
        /// local coordinates equal (old coordinates + d). Equivalent to
        /// transform(I, d). Used to bring neighbouring cells (each stored about
        /// their own centre) into a common origin before fusing.
        void shift(const Eigen::Vector3d &d)
        {
            transform(Eigen::Matrix3d::Identity(), d);
        }
    };

    /// Fixed global cell lattice. Absolute, map-origin-independent so that cell
    /// ids and cell centres are stable across any grid_map resize/recentre.
    struct Lattice
    {
        double x0 = 0., y0 = 0., res = 0.25;

        Lattice() = default;
        Lattice(double x0_, double y0_, double res_) : x0(x0_), y0(y0_), res(res_) {}

        inline void cellOf(double x, double y, int &ci, int &cj) const
        {
            ci = static_cast<int>(std::lround((x - x0) / res));
            cj = static_cast<int>(std::lround((y - y0) / res));
        }

        inline Eigen::Vector2d centerOf(int ci, int cj) const
        {
            return Eigen::Vector2d(x0 + ci * res, y0 + cj * res);
        }

        /// Pack an absolute cell id into a single key (forward index for partials).
        static inline uint64_t key(int ci, int cj)
        {
            return (static_cast<uint64_t>(static_cast<uint32_t>(ci)) << 32)
                 | static_cast<uint64_t>(static_cast<uint32_t>(cj));
        }
        static inline void unkey(uint64_t k, int &ci, int &cj)
        {
            ci = static_cast<int>(static_cast<uint32_t>(k >> 32));
            cj = static_cast<int>(static_cast<uint32_t>(k & 0xffffffffu));
        }
    };

    /// A cell's fused moments together with its map-frame cell centre.
    struct CellMoment
    {
        NodeMetaData data;
        Eigen::Vector3d center;  ///< cell centre in map frame
    };

    /// Indices into the hazard array returned by computeGoodness.
    enum Hazard : std::size_t
    {
        HAZ_OVERALL = 0,
        HAZ_BORDER = 1,
        HAZ_ELEVATION = 2,
        HAZ_SLOPE = 3,
        HAZ_STEP = 4,
        HAZ_ROUGHNESS = 5
    };

    /// Compute the traversability hazards for a query cell from the fused
    /// moments of its vicinity. PCA over ALL points in the vicinity:
    ///   slope     = arccos(|n_z|) / max_pitch          (n = min-eigenvector of Sigma)
    ///   roughness = sqrt(lambda_min) / ground_clearance (point-to-plane residual)
    ///   step      = (max-min of cell barycentres projected on n) / ground_clearance
    ///   elevation = query cell centroid z (map frame)
    /// Returns NaN entries and a non-zero HAZ_BORDER when a cell is rejected.
    ///
    /// @param query             the query cell's own moments + centre (must be occupied)
    /// @param occupied          occupied cells in the vicinity window (must include query)
    /// @param vicinity_cell_count total cells in the window incl. empty (for the
    ///                          occupied-fraction gate)
    inline std::array<double, 6> computeGoodness(
        const CellMoment &query,
        const std::vector<CellMoment> &occupied,
        int vicinity_cell_count,
        double ground_clearance,
        double max_pitch,
        unsigned int min_vicinity_points,
        double min_occupied_fraction)
    {
        std::array<double, 6> haz;
        haz.fill(std::numeric_limits<double>::quiet_NaN());

        if (query.data.N < 1)
            return haz;

        // Elevation is the query cell's own centroid height (map frame). It is
        // available whenever the cell has points, independent of the gates.
        haz[HAZ_ELEVATION] = query.data.sz / static_cast<double>(query.data.N)
                             + query.center.z();

        // Gate 1: enough of the surrounding cells must be observed, so a lone
        // dense cell cannot drive a fit while its neighbours are empty.
        const double occupied_fraction = vicinity_cell_count > 0
            ? static_cast<double>(occupied.size()) / static_cast<double>(vicinity_cell_count)
            : 0.0;
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

        // Slope: tilt of the surface normal from map up.
        const double slope = std::acos(std::min(1.0, std::abs(n.z())));
        haz[HAZ_SLOPE] = std::min(slope / max_pitch, 1.0);

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

#endif  // TRAVERSABILITY_MOMENTS_HPP_
