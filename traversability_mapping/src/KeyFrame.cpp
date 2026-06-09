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
#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    KeyFrame::KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base)
        : id_(id), pose_(Tm_base) {}

    void KeyFrame::setPose(const Eigen::Affine3f &p)
    {
        pose_ = p;
    }

    void KeyFrame::addPoint(std::uint64_t cellId, double lx, double ly, double lz)
    {
        partials_[cellId].insert(lx, ly, lz);
    }
}  // namespace traversability_mapping
