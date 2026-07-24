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

// Data structures for the moment-fused traversability map (no ROS / grid_map /
// PCL dependency, so they can be unit-tested in isolation):
//   * NodeMetaData : per-cell raw moments (sufficient statistics) of the points
//                    that fall in a cell, in CELL-LOCAL, MAP-ALIGNED coordinates.
//   * Lattice      : the fixed global cell lattice (absolute cell ids).
//   * CellMoment   : a cell's moments together with its map-frame centre.
//
// The traversability metrics computed FROM these structures live in
// TraversabilityMetrics.hpp. The maths is documented in
// obsidian_vault .../Traversability/traversability_moments_verbatim.md

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cstdint>
#include <cmath>

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
        // Q = sum(p p^T)
        double sx2 = 0., sy2 = 0., sz2 = 0.;
        double sxy = 0., sxz = 0., syz = 0.;

        void reset();

        bool empty() const { return N == 0; }

        /// Accumulate one point given in this node's local (cell-centred) frame.
        /// Hot path (called per point), kept inline.
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

        Eigen::Vector3d S() const;
        Eigen::Matrix3d Q() const;
        void setSQ(const Eigen::Vector3d &s, const Eigen::Matrix3d &q);

        /// Centroid in the local frame (add the cell centre to get map coords).
        Eigen::Vector3d barycenter() const;

        /// Covariance Sigma = Q/N - mu mu^T (origin-invariant).
        Eigen::Matrix3d covariance() const;

        /// Apply a rigid transform to the underlying points: p -> R p + t,
        /// expressed in this node's local frame. Exact, no points needed:
        ///   S' = R S + N t
        ///   Q' = R Q R^T + R S t^T + t (R S)^T + N t t^T
        void transform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

        /// Re-express the moments about a new origin so the new local
        /// coordinates equal (old coordinates + d). Equivalent to transform(I, d).
        /// Used to bring neighbouring cells (each stored about their own centre)
        /// into a common origin before fusing.
        void shift(const Eigen::Vector3d &d);
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

        // TODO(suchetan): This could be catered by a hash function since it is sotred as an unordered_map in Keyframe::partials_. 
        // But we need unkey because we also want to do reverse lookup. A hash function would give us only one way lookup. 
        // Is there a better approach to do this?
        static inline std::uint64_t key(int ci, int cj)
        {
            return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ci)) << 32)
                 | static_cast<std::uint64_t>(static_cast<std::uint32_t>(cj));
        }
        static inline void unkey(std::uint64_t k, int &ci, int &cj)
        {
            ci = static_cast<int>(static_cast<std::uint32_t>(k >> 32));
            cj = static_cast<int>(static_cast<std::uint32_t>(k & 0xffffffffu));
        }
    };

    /// A cell's fused moments together with its map-frame cell centre.
    struct CellMoment
    {
        NodeMetaData data;
        Eigen::Vector3d center;  ///< cell centre in map frame
    };

}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_MOMENTS_HPP_
