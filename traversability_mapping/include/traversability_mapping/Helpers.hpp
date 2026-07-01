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

#ifndef TRAVERSABILITY_HELPERS_HPP_
#define TRAVERSABILITY_HELPERS_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_set>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <grid_map_core/GridMap.hpp>

#include "traversability_mapping/Moments.hpp"

// NOTE: the ROS occupancy-grid conversion (gridMapToOccupancyGrid) lives in the ROS
// adapter, not here, so the core library carries zero ROS headers.

namespace traversability_mapping
{
    // --- generic grid <-> lattice utilities ---------------------------------
    // Free functions shared by the map machinery. They are stateless: every
    // dependency (the grid, the lattice, the layer list, the resolution) is passed
    // in, so they can be reused and unit-tested without a LocalMap instance.

    /// grid_map position of cell (ci,cj)'s centre on `lattice`.
    grid_map::Position cellPos(const Lattice &lattice, int ci, int cj);

    /// Build a fresh, all-NaN grid_map covering +/-(halfX,halfY) about the lattice
    /// origin. ODD cell count per axis so cell centres land exactly on the lattice;
    /// the geometry position stays the lattice origin so absolute cell ids survive
    /// any resize.
    grid_map::GridMap makeGridMap(const std::vector<std::string> &layers,
                                  const std::string &frameId, const Lattice &lattice,
                                  double res, double halfX, double halfY);

    /// Grow `grid` IN PLACE so it covers the given map-frame bounds, enlarging in
    /// steps of `extend` metres and copying every non-NaN cell across. No-op when the
    /// bounds already fit. `layers`/`lattice`/`res` describe `grid`.
    void growGridToInclude(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                           const Lattice &lattice, double res, double extend,
                           double minx, double maxx, double miny, double maxy);

    /// Add `v` to one layer at `p`, treating a NaN cell as 0 (lazy initialisation).
    void addToLayer(grid_map::GridMap &grid, const std::string &layer,
                    const grid_map::Position &p, double v);

    /// Set every `layers` cell at `p` to NaN.
    void blankCell(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                   const grid_map::Position &p);

    /// Read the fused moment stored at cell (ci,cj) into `out`; false if the cell is
    /// outside the grid or unobserved (N<1).
    bool readCellMoment(const grid_map::GridMap &grid, const Lattice &lattice,
                        int ci, int cj, NodeMetaData &out);

    /// Absolute lattice ids of every occupied (N>=1) cell currently in `grid`.
    std::vector<std::uint64_t> allOccupiedKeys(const grid_map::GridMap &grid,
                                               const Lattice &lattice);

    /// Dilate a set of cell keys by +/-`delta` cells on each axis (square window).
    std::unordered_set<std::uint64_t> dilate(const std::unordered_set<std::uint64_t> &touched,
                                             int delta);

    class Profiler
    {
    public:
        Profiler(const std::string & functionName) : functionName(functionName)
        {
            start = std::chrono::high_resolution_clock::now();
        }

        ~Profiler()
        {
            auto end      = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            float seconds = duration.count() / 1e6;
            std::cout << "\033[1;94m" << functionName << " Execution Time: " << seconds << " Seconds\033[0m" << std::endl;
        }

    private:
        std::string functionName;
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
    };

    #define PROFILE_FUNCTION Profiler profiler_instance(__func__);
}
#endif // TRAVERSABILITY_HELPERS_HPP_