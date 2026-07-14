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
#include <utility>
#include <vector>
#include <unordered_set>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <grid_map_core/GridMap.hpp>

#include "traversability_mapping/Moments.hpp"

/**
 * @file Helpers.hpp
 * @brief Stateless grid <-> lattice utilities shared by the map machinery.
 *
 * Every dependency is passed in, so these free functions are reusable and unit-testable
 * without a LocalMap.
 * @note The ROS occupancy-grid conversion lives in the ROS adapter, so the core carries
 *       zero ROS headers.
 */

namespace traversability_mapping
{
    /// @brief grid_map position of a cell's centre.
    /// @param lattice the cell lattice. @param ci,cj cell indices. @return centre position.
    grid_map::Position cellPos(const Lattice &lattice, int ci, int cj);

    /// @brief Build a fresh all-NaN grid covering +/-(halfX,halfY) about the lattice origin.
    /// @param layers layer names. @param frameId grid frame id. @param lattice the lattice.
    /// @param res cell size (m). @param halfX,halfY half-extents (m).
    /// @return the new grid (odd cell count per axis so centres land on the lattice).
    grid_map::GridMap makeGridMap(const std::vector<std::string> &layers,
                                  const std::string &frameId, const Lattice &lattice,
                                  double res, double halfX, double halfY);

    /// @brief Grow @p grid in place to cover the given map-frame bounds, copying cells across.
    /// @param grid grid to enlarge (no-op if the bounds already fit).
    /// @param layers,lattice,res describe @p grid. @param extend growth step (m).
    /// @param minx,maxx,miny,maxy map-frame bounds to include.
    void growGridToInclude(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                           const Lattice &lattice, double res, double extend,
                           double minx, double maxx, double miny, double maxy);

    /// @brief Set every @p layers cell at @p p to NaN.
    /// @param grid target grid. @param layers layers to blank. @param p cell position.
    void blankCell(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                   const grid_map::Position &p);

    /**
     * @brief Handles to a grid's ten moment layers, resolved once.
     *
     * grid_map's at(layerName, position) hashes the layer name and re-derives the buffer
     * index on every scalar access, and the moment layers are always touched ten at a
     * time. Resolving the layer matrices up front and indexing them directly turns that
     * into one bounds-checked position->index lookup per cell.
     *
     * @warning Holds raw pointers into @p grid: rebuild after anything that replaces the
     *          grid (growGridToInclude).
     */
    struct MomentLayers
    {
        /// @param grid grid whose moment layers to resolve.
        explicit MomentLayers(grid_map::GridMap &grid);

        /// @brief Read the moment at a buffer index.
        /// @param idx buffer index. @param out [out] the moment.
        /// @return false if the cell is unobserved (N<1).
        bool read(const grid_map::Index &idx, NodeMetaData &out) const;

        /// @brief Accumulate sign * @p m at a buffer index, treating a NaN cell as 0.
        /// @param idx buffer index. @param m the moment. @param sign +1 to add, -1 to subtract.
        void add(const grid_map::Index &idx, const NodeMetaData &m, double sign) const;

        grid_map::Matrix *N;
        grid_map::Matrix *sx, *sy, *sz;
        grid_map::Matrix *sx2, *sy2, *sz2;
        grid_map::Matrix *sxy, *sxz, *syz;
    };

    /// @brief Read the fused moment stored at cell (ci,cj), against pre-resolved layers.
    /// @param grid,lattice the map. @param layers layers of @p grid. @param ci,cj cell indices.
    /// @param out [out] the moment.
    /// @return false if the cell is outside the grid or unobserved (N<1).
    bool readCellMoment(const grid_map::GridMap &grid, const MomentLayers &layers,
                        const Lattice &lattice, int ci, int cj, NodeMetaData &out);

    /// @brief Absolute ids of every occupied (N>=1) cell in @p grid.
    /// @param grid,lattice the map. @return the occupied cell ids.
    std::vector<std::uint64_t> allOccupiedKeys(const grid_map::GridMap &grid,
                                               const Lattice &lattice);

    /// @brief Cell offsets whose centres lie within @p radius of a cell centre (a disc).
    ///
    /// This is the robot footprint expressed in cells: the plane fit for a cell uses
    /// exactly the cells the robot standing on it would cover. Always includes (0,0), and
    /// a radius under one cell is clamped up to one so the fit is never handed a single cell.
    /// @param radius footprint radius (m). @param res cell size (m).
    /// @return the offsets, as (di,dj) pairs.
    std::vector<std::pair<int, int>> discOffsets(double radius, double res);

    /// @brief Dilate a set of cell keys by a structuring element (e.g. discOffsets()).
    ///
    /// With a symmetric element this yields exactly the cells whose neighbourhood contains
    /// a touched cell — i.e. the cells whose value the touched cells can have changed.
    /// @param touched input keys. @param offsets structuring element. @return the dilated set.
    std::unordered_set<std::uint64_t> dilate(const std::unordered_set<std::uint64_t> &touched,
                                             const std::vector<std::pair<int, int>> &offsets);

    /// @brief Scoped timer that logs the wall-clock lifetime of its enclosing scope.
    class Profiler
    {
    public:
        /// @param functionName label printed with the measured duration.
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

    /// @brief Profile the current function: drop this macro at the top of its body.
    #define PROFILE_FUNCTION Profiler profiler_instance(__func__);
}
#endif // TRAVERSABILITY_HELPERS_HPP_
