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

#ifdef _OPENMP
#include <omp.h>
#endif

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

        // Bin into PER-THREAD maps (the shared partials_ map is not thread-safe),
        // then fuse them. Moments are raw sums, so per-thread partials of the same
        // cell fuse exactly. With one thread this is just a single local map.
#ifdef _OPENMP
        const int maxThreads = std::max(1, omp_get_max_threads());
#else
        const int maxThreads = 1;
#endif
        std::vector<std::unordered_map<std::uint64_t, NodeMetaData>> local(maxThreads);

        #pragma omp parallel
        {
#ifdef _OPENMP
            const int tid = omp_get_thread_num();
#else
            const int tid = 0;
#endif
            auto &lm = local[tid];
            #pragma omp for schedule(static)
            for (std::size_t i = 0; i < cloud_base_.size(); ++i)
            {
                const Eigen::Vector3f p_map = pose_ * cloud_base_[i];
                int ci, cj;
                lattice.cellOf(p_map.x(), p_map.y(), ci, cj);
                const Eigen::Vector2d c = lattice.centerOf(ci, cj);
                // Cell-local, map-aligned: origin at the cell centre (x,y), z about 0.
                lm[Lattice::key(ci, cj)].insert(
                    static_cast<double>(p_map.x()) - c.x(),
                    static_cast<double>(p_map.y()) - c.y(),
                    static_cast<double>(p_map.z()));
            }
        }

        for (auto &lm : local)
            for (auto &kv : lm)
                partials_[kv.first].fuseWith(kv.second);
    }
}  // namespace traversability_mapping
