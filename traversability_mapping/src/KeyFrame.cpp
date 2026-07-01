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
    KeyFrame::KeyFrame(std::uint64_t kfID, double timestamp, const Eigen::Affine3f &Tm_base,
                       std::vector<Eigen::Vector3f> &&cloudBase, std::uint64_t parentMapID)
        : kfID_(kfID), timestamp_(timestamp), parentMapID_(parentMapID),
          cloudBase_(std::move(cloudBase)), latestPose_(Tm_base) {}

    KeyFrame::KeyFrame(std::uint64_t kfID, const Eigen::Affine3f &Tm_base,
                       std::vector<Eigen::Vector3f> &&cloudBase)
        : KeyFrame(kfID, 0.0, Tm_base, std::move(cloudBase), 0) {}

    void KeyFrame::setPose(const Eigen::Affine3f &p)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        latestPose_ = p;
        hasPending_ = true;
    }

    bool KeyFrame::getPendingPose(Eigen::Affine3f &out)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        if (!hasPending_)
            return false;
        out = latestPose_;
        hasPending_ = false;
        return true;
    }

    Eigen::Affine3f KeyFrame::getPose() const
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        return latestPose_;
    }

    void KeyFrame::rebin(const Lattice &lattice, const Eigen::Affine3f &pose)
    {
        // Bin the stored base-frame cloud, transformed by `pose`, into cell-local
        // moments and REPLACE the partials. `pose` is passed by value/snapshot (not
        // read from a shared member) so a concurrent setPose cannot tear it mid-loop.
        // Moments are raw sums, so the partition can be recomputed from scratch every
        // call (not frozen). Single-threaded: OpenMP is deferred (see PRD);
        // parallelism comes only from the LocalMap worker-thread structure.
        partials_.clear();
        for (const auto &p_base : cloudBase_)
        {
            const Eigen::Vector3f p_map = pose * p_base;
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
