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

#include <vector>

namespace traversability_mapping
{
    KeyFrame::KeyFrame(std::uint64_t id, double timestamp, const Eigen::Affine3f &Tm_base,
                       std::vector<Eigen::Vector3f> &&cloud_base, std::uint64_t parentMapID)
        : id_(id), timestamp_(timestamp), parentMapID_(parentMapID),
          pose_(Tm_base), cloud_base_(std::move(cloud_base)) {}

    KeyFrame::KeyFrame(std::uint64_t id, const Eigen::Affine3f &Tm_base,
                       std::vector<Eigen::Vector3f> &&cloud_base)
        : KeyFrame(id, 0.0, Tm_base, std::move(cloud_base), 0) {}

    void KeyFrame::setPose(const Eigen::Affine3f &p)
    {
        pose_ = p;
    }

    void KeyFrame::setPendingPose(const Eigen::Affine3f &p)
    {
        std::lock_guard<std::mutex> lock(slotMutex_);
        pendingPose_ = p;
        hasPending_ = true;
    }

    bool KeyFrame::takePendingPose(Eigen::Affine3f &out)
    {
        std::lock_guard<std::mutex> lock(slotMutex_);
        if (!hasPending_)
            return false;
        out = pendingPose_;
        hasPending_ = false;
        return true;
    }

    void KeyFrame::rebin(const Lattice &lattice)
    {
        // Bin the stored base-frame cloud, transformed by the current pose, into
        // cell-local moments and REPLACE the partials. Moments are raw sums, so the
        // partition can be recomputed from scratch every call (not frozen).
        // Single-threaded: OpenMP is deferred (see PRD); parallelism comes only
        // from the LocalMap worker-thread structure.
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
