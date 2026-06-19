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
    KeyFrame::KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base,
                       std::vector<Eigen::Vector3f> &&cloud_base)
        : id_(id), pose_(Tm_base), cloud_base_(std::move(cloud_base)) {}

    void KeyFrame::setPose(const Eigen::Affine3f &p)
    {
        pose_ = p;
    }

    void KeyFrame::rebin(const Lattice &lattice)
    {
        partials_.clear();
        for (const auto &p_base : cloud_base_)
        {
            const Eigen::Vector3f p_map = pose_ * p_base;
            int ci, cj;
            lattice.cellOf(p_map.x(), p_map.y(), ci, cj);
            const Eigen::Vector2d c = lattice.centerOf(ci, cj);
            // Cell-local, map-aligned: origin at the cell centre (x,y), z about 0.
            partials_[Lattice::key(ci, cj)].insert(
                static_cast<double>(p_map.x()) - c.x(),
                static_cast<double>(p_map.y()) - c.y(),
                static_cast<double>(p_map.z()));
        }
    }
}  // namespace traversability_mapping
