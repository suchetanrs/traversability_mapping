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

#include "traversability_mapping/Helpers.hpp"

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace traversability_mapping
{
    namespace
    {
        constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    }

    grid_map::Position cellPos(const Lattice &lattice, int ci, int cj)
    {
        const Eigen::Vector2d c = lattice.centerOf(ci, cj);
        return grid_map::Position(c.x(), c.y());
    }

    grid_map::GridMap makeGridMap(const std::vector<std::string> &layers,
                                  const std::string &frameId, const Lattice &lattice,
                                  double res, double halfX, double halfY)
    {
        // ODD cell count per axis so cell centres land exactly on the lattice;
        // position stays the lattice origin forever so partials keyed by absolute
        // id remain valid across resizes.
        const int kx = static_cast<int>(std::ceil(halfX / res));
        const int ky = static_cast<int>(std::ceil(halfY / res));
        grid_map::GridMap m(layers);
        m.setFrameId(frameId);
        m.setGeometry(grid_map::Length((2 * kx + 1) * res, (2 * ky + 1) * res), res,
                      grid_map::Position(lattice.x0, lattice.y0));
        for (const auto &l : layers)
            m[l].setConstant(kNaN);
        return m;
    }

    void growGridToInclude(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                           const Lattice &lattice, double res, double extend,
                           double minx, double maxx, double miny, double maxy)
    {
        const double margin = res;
        const double curHalfX = grid.getLength().x() / 2.0;
        const double curHalfY = grid.getLength().y() / 2.0;
        const double needX = std::max(std::abs(maxx - lattice.x0), std::abs(minx - lattice.x0)) + margin;
        const double needY = std::max(std::abs(maxy - lattice.y0), std::abs(miny - lattice.y0)) + margin;
        if (needX <= curHalfX && needY <= curHalfY)
            return;

        double newHalfX = curHalfX, newHalfY = curHalfY;
        while (newHalfX < needX) newHalfX += extend;
        while (newHalfY < needY) newHalfY += extend;

        grid_map::GridMap old = grid;
        grid = makeGridMap(layers, old.getFrameId(), lattice, res, newHalfX, newHalfY);
        for (grid_map::GridMapIterator it(old); !it.isPastEnd(); ++it)
        {
            grid_map::Position p;
            old.getPosition(*it, p);
            if (!grid.isInside(p))
                continue;
            for (const auto &l : layers)
            {
                const float v = old.at(l, *it);
                if (!std::isnan(v))
                    grid.atPosition(l, p) = v;
            }
        }
    }

    void addToLayer(grid_map::GridMap &grid, const std::string &layer,
                    const grid_map::Position &p, double v)
    {
        float &cell = grid.atPosition(layer, p);
        if (std::isnan(cell)) cell = 0.f;
        cell += static_cast<float>(v);
    }

    void blankCell(grid_map::GridMap &grid, const std::vector<std::string> &layers,
                   const grid_map::Position &p)
    {
        for (const auto &l : layers)
            grid.atPosition(l, p) = kNaN;
    }

    bool readCellMoment(const grid_map::GridMap &grid, const Lattice &lattice,
                        int ci, int cj, NodeMetaData &out)
    {
        const grid_map::Position p = cellPos(lattice, ci, cj);
        if (!grid.isInside(p))
            return false;
        const float n = grid.atPosition("N", p);
        if (std::isnan(n) || n < 1.f)
            return false;
        out.N = static_cast<unsigned int>(std::lround(n));
        out.sx = grid.atPosition("sx", p);   out.sy = grid.atPosition("sy", p);   out.sz = grid.atPosition("sz", p);
        out.sx2 = grid.atPosition("sx2", p); out.sy2 = grid.atPosition("sy2", p); out.sz2 = grid.atPosition("sz2", p);
        out.sxy = grid.atPosition("sxy", p); out.sxz = grid.atPosition("sxz", p); out.syz = grid.atPosition("syz", p);
        return true;
    }

    std::vector<std::uint64_t> allOccupiedKeys(const grid_map::GridMap &grid,
                                               const Lattice &lattice)
    {
        std::vector<std::uint64_t> keys;
        for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it)
        {
            const float n = grid.at("N", *it);
            if (std::isnan(n) || n < 1.f)
                continue;
            grid_map::Position p;
            grid.getPosition(*it, p);
            int ci, cj;
            lattice.cellOf(p.x(), p.y(), ci, cj);
            keys.push_back(Lattice::key(ci, cj));
        }
        return keys;
    }

    std::unordered_set<std::uint64_t> dilate(const std::unordered_set<std::uint64_t> &touched,
                                             int delta)
    {
        std::unordered_set<std::uint64_t> out;
        out.reserve(touched.size() * (2 * delta + 1) * (2 * delta + 1));
        for (auto id : touched)
        {
            int ci, cj;
            Lattice::unkey(id, ci, cj);
            for (int di = -delta; di <= delta; ++di)
                for (int dj = -delta; dj <= delta; ++dj)
                    out.insert(Lattice::key(ci + di, cj + dj));
        }
        return out;
    }
}  // namespace traversability_mapping
