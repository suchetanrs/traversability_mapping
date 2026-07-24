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

#include "traversability_mapping/Moments.hpp"

namespace traversability_mapping
{
    void NodeMetaData::reset()
    {
        *this = NodeMetaData();
    }

    Eigen::Vector3d NodeMetaData::S() const
    {
        return Eigen::Vector3d(sx, sy, sz);
    }

    Eigen::Matrix3d NodeMetaData::Q() const
    {
        Eigen::Matrix3d m;
        m << sx2, sxy, sxz,
             sxy, sy2, syz,
             sxz, syz, sz2;
        return m;
    }

    void NodeMetaData::setSQ(const Eigen::Vector3d &s, const Eigen::Matrix3d &q)
    {
        sx = s.x();  sy = s.y();  sz = s.z();
        sx2 = q(0, 0);  sy2 = q(1, 1);  sz2 = q(2, 2);
        sxy = q(0, 1);  sxz = q(0, 2);  syz = q(1, 2);
    }

    Eigen::Vector3d NodeMetaData::barycenter() const
    {
        return S() / static_cast<double>(N);
    }

    void NodeMetaData::shift(const Eigen::Vector3d &d)
    {
        // A pure translation of the underlying points by d: S -> S + N d, and
        // Q -> Q + S d^T + d S^T + N d d^T.
        const Eigen::Vector3d s = S();
        const double n = static_cast<double>(N);
        const Eigen::Matrix3d q2 = Q() + s * d.transpose() + d * s.transpose()
                                 + n * d * d.transpose();
        setSQ(s + n * d, q2);
    }

}  // namespace traversability_mapping
