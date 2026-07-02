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

/**
 * @file Moments.hpp
 * @brief Data structures for the moment-fused traversability map (no ROS/grid_map/PCL).
 *
 * NodeMetaData holds a cell's raw moments in cell-local, map-aligned coordinates;
 * Lattice is the fixed absolute cell lattice; CellMoment pairs moments with a map-frame
 * centre. The metrics computed from these live in TraversabilityMetrics.hpp.
 */

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cstdint>
#include <cmath>

namespace traversability_mapping
{
    /**
     * @brief Raw (un-centred) moments of a point set about a fixed local origin.
     *
     * Raw sums add unconditionally as long as every contribution shares the same frame,
     * so per-keyframe contributions to a cell sum directly and can be subtracted exactly.
     */
    class NodeMetaData
    {
    public:
        unsigned int N = 0;                    ///< point count
        double sx = 0., sy = 0., sz = 0.;      ///< first moments S = sum(p)
        double sx2 = 0., sy2 = 0., sz2 = 0.;   ///< second moments Q = sum(p p^T), diagonal
        double sxy = 0., sxz = 0., syz = 0.;   ///< second moments Q = sum(p p^T), off-diagonal

        /// @brief Zero all moments.
        void reset();

        /// @brief True if no points have been accumulated.
        bool empty() const { return N == 0; }

        /// @brief Accumulate one point in this node's local (cell-centred) frame.
        /// @param x,y,z point coordinates in the local frame.
        inline void insert(double x, double y, double z)
        {
            ++N;
            sx += x;  sy += y;  sz += z;
            sx2 += x * x;  sy2 += y * y;  sz2 += z * z;
            sxy += x * y;  sxz += x * z;  syz += y * z;
        }

        /// @brief Add another contribution (requires the same local frame).
        /// @param o moments to add.
        inline void fuseWith(const NodeMetaData &o)
        {
            N += o.N;
            sx += o.sx;  sy += o.sy;  sz += o.sz;
            sx2 += o.sx2;  sy2 += o.sy2;  sz2 += o.sz2;
            sxy += o.sxy;  sxz += o.sxz;  syz += o.syz;
        }

        /// @brief Exact inverse of fuseWith (back a keyframe out on a PGO update).
        /// @param o moments to subtract.
        inline void removeWith(const NodeMetaData &o)
        {
            N -= o.N;
            sx -= o.sx;  sy -= o.sy;  sz -= o.sz;
            sx2 -= o.sx2;  sy2 -= o.sy2;  sz2 -= o.sz2;
            sxy -= o.sxy;  sxz -= o.sxz;  syz -= o.syz;
        }

        /// @brief First-moment vector.
        /// @return S = sum(p).
        Eigen::Vector3d S() const;
        /// @brief Second-moment matrix.
        /// @return Q = sum(p p^T).
        Eigen::Matrix3d Q() const;
        /// @brief Set the moments from S and Q.
        /// @param s first moment. @param q second moment.
        void setSQ(const Eigen::Vector3d &s, const Eigen::Matrix3d &q);

        /// @brief Centroid in the local frame (add the cell centre for map coords).
        /// @return mean point.
        Eigen::Vector3d barycenter() const;

        /// @brief Origin-invariant covariance.
        /// @return Sigma = Q/N - mu mu^T.
        Eigen::Matrix3d covariance() const;

        /// @brief Rigidly transform the underlying points (p -> R p + t), exactly.
        /// @param R rotation. @param t translation.
        void transform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

        /// @brief Re-express the moments about a new origin (new coords = old + d).
        /// @param d origin offset; used to bring cells into a common origin before fusing.
        void shift(const Eigen::Vector3d &d);
    };

    /**
     * @brief Fixed global cell lattice.
     *
     * Absolute and map-origin-independent, so cell ids and centres are stable across any
     * grid_map resize/recentre.
     */
    struct Lattice
    {
        double x0 = 0., y0 = 0., res = 0.25;

        Lattice() = default;
        Lattice(double x0_, double y0_, double res_) : x0(x0_), y0(y0_), res(res_) {}

        /// @brief Cell indices containing a map-frame point.
        /// @param x,y map-frame point. @param ci,cj [out] cell indices.
        inline void cellOf(double x, double y, int &ci, int &cj) const
        {
            ci = static_cast<int>(std::lround((x - x0) / res));
            cj = static_cast<int>(std::lround((y - y0) / res));
        }

        /// @brief Map-frame centre of a cell.
        /// @param ci,cj cell indices. @return cell centre.
        inline Eigen::Vector2d centerOf(int ci, int cj) const
        {
            return Eigen::Vector2d(x0 + ci * res, y0 + cj * res);
        }

        /// @brief Pack signed cell indices into one absolute id.
        /// @param ci,cj cell indices. @return packed id.
        /// @todo(suchetan) A hash could serve, but unkey() needs the reverse lookup
        ///       (partials_ is keyed by this id); a hash would be one-way only.
        static inline std::uint64_t key(int ci, int cj)
        {
            return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ci)) << 32)
                 | static_cast<std::uint64_t>(static_cast<std::uint32_t>(cj));
        }
        /// @brief Unpack a key() id back into cell indices.
        /// @param k packed id. @param ci,cj [out] cell indices.
        static inline void unkey(std::uint64_t k, int &ci, int &cj)
        {
            ci = static_cast<int>(static_cast<std::uint32_t>(k >> 32));
            cj = static_cast<int>(static_cast<std::uint32_t>(k & 0xffffffffu));
        }
    };

    /// @brief A cell's fused moments together with its map-frame cell centre.
    struct CellMoment
    {
        NodeMetaData data;
        Eigen::Vector3d center;  ///< cell centre in map frame
    };

}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_MOMENTS_HPP_
