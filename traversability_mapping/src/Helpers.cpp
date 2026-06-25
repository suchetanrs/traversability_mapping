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
#include <traversability_mapping/Helpers.hpp>

namespace traversability_mapping
{
    void doTransformPCL(
        const pcl::PointCloud<pcl::PointXYZ> &p_in, pcl::PointCloud<pcl::PointXYZ> &p_out,
        const Eigen::Affine3f &t)
    {
        pcl::transformPointCloud(p_in, p_out, t);
    }
}