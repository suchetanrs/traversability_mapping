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
#include "traversability_mapping/LocalMap.hpp"

#include "traversability_mapping/TraversabilityMetrics.hpp"
#include "traversability_mapping/Parameters.hpp"

#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace traversability_mapping
{
    namespace
    {
        constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    }

    LocalMap::LocalMap(std::uint64_t mapID, const Lattice &lattice, std::string mapFrame)
        : mapID_(mapID), lattice_(lattice), frameId_(std::move(mapFrame)), res_(lattice.res)
    {
        ground_clearance_ = parameterInstance.getValue<double>("ground_clearance");
        max_slope_ = parameterInstance.getValue<double>("max_slope");
        min_occupied_fraction_ = parameterInstance.getValue<double>("min_occupied_fraction");
        min_vicinity_points_ = static_cast<unsigned int>(parameterInstance.getValue<int>("min_vicinity_points"));
        const double security_distance = parameterInstance.getValue<double>("security_distance");
        // Vicinity radius in cells (symmetric window of side 2*delta_ind_+1).
        delta_ind_ = std::max(1, static_cast<int>(std::ceil((security_distance / 2.0) / res_)));
        windowCap_ = static_cast<std::size_t>(std::max(1, parameterInstance.getValue<int>("num_local_keyframes")));
        globalSleepMs_ = parameterInstance.getValue<int>("global_adjustment_sleep");
        kfOptimizationEnabled_ = parameterInstance.getValue<bool>("is_kf_optimization_enabled");

        layers_ = {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz",
                   "hazard", "elevation", "slope_haz", "step_haz", "roughness_haz",
                   "normal_x", "normal_y", "normal_z"};
        nav_layers_ = {"normal_x", "normal_y", "normal_z", "slope_haz",
                       "step_haz", "elevation", "roughness_haz", "hazard"};

        const double half = parameterInstance.getValue<double>("half_size_traversability");
        gridMap_ = freshMap(half, half);

        localThread_ = std::thread(&LocalMap::RunLocalKeyFrames, this);
        globalThread_ = std::thread(&LocalMap::RunTraversability, this);
    }

    LocalMap::~LocalMap()
    {
        running_.store(false);
        if (localThread_.joinable())
            localThread_.join();
        if (globalThread_.joinable())
            globalThread_.join();
    }

    // ---- grid construction / growth ----------------------------------------

    grid_map::GridMap LocalMap::freshMap(double halfX, double halfY) const
    {
        // ODD cell count per axis so cell centres land exactly on the lattice;
        // position stays the lattice origin forever so partials keyed by absolute
        // id remain valid across resizes.
        const int kx = static_cast<int>(std::ceil(halfX / res_));
        const int ky = static_cast<int>(std::ceil(halfY / res_));
        grid_map::GridMap m(layers_);
        m.setFrameId(frameId_);
        m.setGeometry(grid_map::Length((2 * kx + 1) * res_, (2 * ky + 1) * res_), res_,
                      grid_map::Position(lattice_.x0, lattice_.y0));
        for (const auto &l : layers_)
            m[l].setConstant(kNaN);
        return m;
    }

    void LocalMap::growToInclude(double minx, double maxx, double miny, double maxy)
    {
        const double margin = res_;
        const double curHalfX = gridMap_.getLength().x() / 2.0;
        const double curHalfY = gridMap_.getLength().y() / 2.0;
        const double needX = std::max(std::abs(maxx - lattice_.x0), std::abs(minx - lattice_.x0)) + margin;
        const double needY = std::max(std::abs(maxy - lattice_.y0), std::abs(miny - lattice_.y0)) + margin;
        if (needX <= curHalfX && needY <= curHalfY)
            return;

        const double extend = parameterInstance.getValue<double>("extend_length_every_resize_by");
        double newHalfX = curHalfX, newHalfY = curHalfY;
        while (newHalfX < needX) newHalfX += extend;
        while (newHalfY < needY) newHalfY += extend;

        grid_map::GridMap old = gridMap_;
        gridMap_ = freshMap(newHalfX, newHalfY);
        for (grid_map::GridMapIterator it(old); !it.isPastEnd(); ++it)
        {
            grid_map::Position p;
            old.getPosition(*it, p);
            if (!gridMap_.isInside(p))
                continue;
            for (const auto &l : layers_)
            {
                const float v = old.at(l, *it);
                if (!std::isnan(v))
                    gridMap_.atPosition(l, p) = v;
            }
        }
    }

    void LocalMap::growToIncludeCells(const std::unordered_map<std::uint64_t, NodeMetaData> &partials)
    {
        if (partials.empty())
            return;
        double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
        for (const auto &kv : partials)
        {
            int ci, cj;
            Lattice::unkey(kv.first, ci, cj);
            const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
            minx = std::min(minx, c.x()); maxx = std::max(maxx, c.x());
            miny = std::min(miny, c.y()); maxy = std::max(maxy, c.y());
        }
        growToInclude(minx, maxx, miny, maxy);
    }

    // ---- moment <-> grid helpers -------------------------------------------

    grid_map::Position LocalMap::cellPos(int ci, int cj) const
    {
        const Eigen::Vector2d c = lattice_.centerOf(ci, cj);
        return grid_map::Position(c.x(), c.y());
    }

    void LocalMap::addToLayer(const std::string &l, const grid_map::Position &p, double v)
    {
        float &cell = gridMap_.atPosition(l, p);
        if (std::isnan(cell)) cell = 0.f;
        cell += static_cast<float>(v);
    }

    void LocalMap::addPartialToGrid(std::uint64_t cellId, const NodeMetaData &m, double sign)
    {
        int ci, cj;
        Lattice::unkey(cellId, ci, cj);
        const grid_map::Position p = cellPos(ci, cj);
        if (!gridMap_.isInside(p))
            return;
        addToLayer("N", p, sign * m.N);
        addToLayer("sx", p, sign * m.sx);   addToLayer("sy", p, sign * m.sy);   addToLayer("sz", p, sign * m.sz);
        addToLayer("sx2", p, sign * m.sx2); addToLayer("sy2", p, sign * m.sy2); addToLayer("sz2", p, sign * m.sz2);
        addToLayer("sxy", p, sign * m.sxy); addToLayer("sxz", p, sign * m.sxz); addToLayer("syz", p, sign * m.syz);
        // A subtraction that empties the cell blanks it and tells nav it is cleared.
        if (sign < 0 && std::lround(gridMap_.atPosition("N", p)) <= 0)
        {
            blankCell(p);
            dirty_for_nav_.insert(cellId);
        }
    }

    void LocalMap::blankCell(const grid_map::Position &p)
    {
        for (const auto &l : layers_)
            gridMap_.atPosition(l, p) = kNaN;
    }

    bool LocalMap::readCellMoment(int ci, int cj, NodeMetaData &out) const
    {
        const grid_map::Position p = cellPos(ci, cj);
        if (!gridMap_.isInside(p))
            return false;
        const float n = gridMap_.atPosition("N", p);
        if (std::isnan(n) || n < 1.f)
            return false;
        out.N = static_cast<unsigned int>(std::lround(n));
        out.sx = gridMap_.atPosition("sx", p);   out.sy = gridMap_.atPosition("sy", p);   out.sz = gridMap_.atPosition("sz", p);
        out.sx2 = gridMap_.atPosition("sx2", p); out.sy2 = gridMap_.atPosition("sy2", p); out.sz2 = gridMap_.atPosition("sz2", p);
        out.sxy = gridMap_.atPosition("sxy", p); out.sxz = gridMap_.atPosition("sxz", p); out.syz = gridMap_.atPosition("syz", p);
        return true;
    }

    // ---- recompute ----------------------------------------------------------

    std::unordered_set<std::uint64_t> LocalMap::dilate(const std::unordered_set<std::uint64_t> &touched) const
    {
        std::unordered_set<std::uint64_t> out;
        out.reserve(touched.size() * (2 * delta_ind_ + 1) * (2 * delta_ind_ + 1));
        for (auto id : touched)
        {
            int ci, cj;
            Lattice::unkey(id, ci, cj);
            for (int di = -delta_ind_; di <= delta_ind_; ++di)
                for (int dj = -delta_ind_; dj <= delta_ind_; ++dj)
                    out.insert(Lattice::key(ci + di, cj + dj));
        }
        return out;
    }

    bool LocalMap::recomputeCell(std::uint64_t id)
    {
        int ci, cj;
        Lattice::unkey(id, ci, cj);
        const grid_map::Position qp = cellPos(ci, cj);
        if (!gridMap_.isInside(qp))
            return false;

        NodeMetaData qd;
        if (!readCellMoment(ci, cj, qd))
            return false;  // query cell unobserved -> nothing to write

        CellMoment query{qd, Eigen::Vector3d(qp.x(), qp.y(), 0.0)};
        std::vector<CellMoment> occupied;
        int total = 0;
        for (int i = ci - delta_ind_; i <= ci + delta_ind_; ++i)
            for (int j = cj - delta_ind_; j <= cj + delta_ind_; ++j)
            {
                ++total;
                NodeMetaData d;
                if (readCellMoment(i, j, d))
                {
                    const Eigen::Vector2d c2 = lattice_.centerOf(i, j);
                    occupied.push_back({d, Eigen::Vector3d(c2.x(), c2.y(), 0.0)});
                }
            }

        const auto haz = computeGoodness(query, occupied, total, ground_clearance_,
                                         max_slope_, min_vicinity_points_, min_occupied_fraction_);

        if (!std::isnan(haz[HAZ_ELEVATION]))
            gridMap_.atPosition("elevation", qp) = static_cast<float>(haz[HAZ_ELEVATION]);

        if (std::isnan(haz[HAZ_OVERALL]))
        {
            gridMap_.atPosition("hazard", qp) = kNaN;
            gridMap_.atPosition("slope_haz", qp) = kNaN;
            gridMap_.atPosition("step_haz", qp) = kNaN;
            gridMap_.atPosition("roughness_haz", qp) = kNaN;
            gridMap_.atPosition("normal_x", qp) = kNaN;
            gridMap_.atPosition("normal_y", qp) = kNaN;
            gridMap_.atPosition("normal_z", qp) = kNaN;
            return true;
        }
        gridMap_.atPosition("hazard", qp) = static_cast<float>(haz[HAZ_OVERALL]);
        gridMap_.atPosition("slope_haz", qp) = static_cast<float>(haz[HAZ_SLOPE]);
        gridMap_.atPosition("step_haz", qp) = static_cast<float>(haz[HAZ_STEP]);
        gridMap_.atPosition("roughness_haz", qp) = static_cast<float>(haz[HAZ_ROUGHNESS]);
        gridMap_.atPosition("normal_x", qp) = static_cast<float>(haz[HAZ_NORMAL_X]);
        gridMap_.atPosition("normal_y", qp) = static_cast<float>(haz[HAZ_NORMAL_Y]);
        gridMap_.atPosition("normal_z", qp) = static_cast<float>(haz[HAZ_NORMAL_Z]);
        return true;
    }

    void LocalMap::recomputeDirty(const std::unordered_set<std::uint64_t> &dirty)
    {
        for (auto id : dirty)
            if (recomputeCell(id))
                dirty_for_nav_.insert(id);
    }

    // ---- per-keyframe op (mapMutex_ held by caller) -------------------------

    void LocalMap::processKeyframeLocked(const std::shared_ptr<KeyFrame> &kf)
    {
        // Deleted between snapshot and now?
        if (keyframes_.find(kf->id()) == keyframes_.end())
            return;
        // Cloud dropped (optimization off, already binned once): cannot reprocess;
        // leave the existing contribution intact.
        if (!kf->hasCloud())
            return;

        std::unordered_set<std::uint64_t> touched;  // M_old (subtracted) U M_new (added)

        // 1. subtract the OLD contribution (computed at the old pose).
        for (auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, -1.0);
            touched.insert(kv.first);
        }

        // 2. commit any pending PGO pose BEFORE re-binning.
        Eigen::Affine3f newPose;
        if (kf->takePendingPose(newPose))
            kf->setPose(newPose);

        // 3. re-bin the stored base-frame cloud at the current pose (partition NOT frozen).
        kf->rebin(lattice_);

        // 4. grow to fit, then add the fresh contribution.
        growToIncludeCells(kf->partials());
        for (auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, +1.0);
            touched.insert(kv.first);
        }

        // 5. recompute hazards over the dilated union of cells left and newly occupied.
        recomputeDirty(dilate(touched));

        // 6. drop the cloud if optimization is disabled (keyframe becomes non-rebinnable).
        if (!kfOptimizationEnabled_)
            kf->dropCloud();
    }

    // ---- keyframe lifecycle -------------------------------------------------

    void LocalMap::addKeyFrame(const std::shared_ptr<KeyFrame> &kf)
    {
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            kf->setMap(mapID_);
            keyframes_[kf->id()] = kf;
            window_.push_front(kf);
            while (window_.size() > windowCap_)
                window_.pop_back();
        }
        kf->markDirty();
    }

    std::shared_ptr<KeyFrame> LocalMap::detachKeyFrame(std::uint64_t id)
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        auto it = keyframes_.find(id);
        if (it == keyframes_.end())
            return nullptr;
        auto kf = it->second;

        std::unordered_set<std::uint64_t> touched;
        for (auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, -1.0);
            touched.insert(kv.first);
        }
        recomputeDirty(dilate(touched));
        kf->partials().clear();

        keyframes_.erase(it);
        for (auto wit = window_.begin(); wit != window_.end();)
        {
            if ((*wit)->id() == id) wit = window_.erase(wit);
            else ++wit;
        }
        return kf;
    }

    void LocalMap::clearEntireMap()
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        // Tell nav every currently-occupied cell is cleared.
        for (auto k : allOccupiedKeys())
            dirty_for_nav_.insert(k);
        // Blank every cell of every layer.
        for (const auto &l : layers_)
            gridMap_[l].setConstant(kNaN);
        // Clear each keyframe's contribution record and mark it for rebuild.
        for (auto &kv : keyframes_)
        {
            kv.second->partials().clear();
            kv.second->markDirty();
        }
    }

    // ---- nav output ---------------------------------------------------------

    std::vector<std::uint64_t> LocalMap::allOccupiedKeys() const
    {
        std::vector<std::uint64_t> keys;
        for (grid_map::GridMapIterator it(gridMap_); !it.isPastEnd(); ++it)
        {
            const float n = gridMap_.at("N", *it);
            if (std::isnan(n) || n < 1.f)
                continue;
            grid_map::Position p;
            gridMap_.getPosition(*it, p);
            int ci, cj;
            lattice_.cellOf(p.x(), p.y(), ci, cj);
            keys.push_back(Lattice::key(ci, cj));
        }
        return keys;
    }

    NavDelta LocalMap::fillNav(const std::vector<std::uint64_t> &keys, bool full) const
    {
        NavDelta d;
        d.frame_id = frameId_;
        d.resolution = res_;
        d.origin_x = lattice_.x0;
        d.origin_y = lattice_.y0;
        d.layers = nav_layers_;
        d.is_full_snapshot = full;
        d.cell_keys.reserve(keys.size());
        d.values.reserve(keys.size() * nav_layers_.size());
        for (auto id : keys)
        {
            int ci, cj;
            Lattice::unkey(id, ci, cj);
            const grid_map::Position p = cellPos(ci, cj);
            if (!gridMap_.isInside(p))
                continue;
            d.cell_keys.push_back(id);
            for (const auto &l : nav_layers_)
                d.values.push_back(gridMap_.atPosition(l, p));
        }
        return d;
    }

    NavDelta LocalMap::drainNavDelta()
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        const std::vector<std::uint64_t> keys(dirty_for_nav_.begin(), dirty_for_nav_.end());
        NavDelta d = fillNav(keys, /*full=*/false);
        dirty_for_nav_.clear();
        return d;
    }

    NavDelta LocalMap::fullNavSnapshot()
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return fillNav(allOccupiedKeys(), /*full=*/true);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LocalMap::getStitchedPointCloud(
        float voxel_size_x, float voxel_size_y, float voxel_size_z)
    {
        auto stitched = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::vector<std::shared_ptr<KeyFrame>> snapshot;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            snapshot.reserve(keyframes_.size());
            for (auto &kv : keyframes_)
                snapshot.push_back(kv.second);
        }
        for (auto &kf : snapshot)
        {
            if (!kf->hasCloud())
                continue;
            const Eigen::Affine3f Tmb = kf->pose();
            for (const auto &pb : kf->cloudBase())
            {
                const Eigen::Vector3f pm = Tmb * pb;
                stitched->push_back(pcl::PointXYZ(pm.x(), pm.y(), pm.z()));
            }
        }
        if (voxel_size_x > 0.f && voxel_size_y > 0.f && voxel_size_z > 0.f && !stitched->empty())
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(stitched);
            vg.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
            auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            vg.filter(*filtered);
            return filtered;
        }
        return stitched;
    }

    // ---- worker threads -----------------------------------------------------

    void LocalMap::RunLocalKeyFrames()
    {
        // Fast worker: service dirty keyframes in the last-N window at a high rate.
        while (running_.load())
        {
            std::vector<std::shared_ptr<KeyFrame>> snapshot;
            {
                std::lock_guard<std::mutex> lock(mapMutex_);
                snapshot.assign(window_.begin(), window_.end());
            }
            for (auto &kf : snapshot)
            {
                if (!running_.load())
                    return;
                if (kf->claim())
                {
                    std::lock_guard<std::mutex> lock(mapMutex_);
                    processKeyframeLocked(kf);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void LocalMap::RunTraversability()
    {
        // Slow worker: sweep ALL keyframes; a keyframe with nothing pending fails
        // claim() and is skipped, so the sweep is cheap, not redundant re-binning.
        while (running_.load())
        {
            std::vector<std::shared_ptr<KeyFrame>> snapshot;
            {
                std::lock_guard<std::mutex> lock(mapMutex_);
                snapshot.reserve(keyframes_.size());
                for (auto &kv : keyframes_)
                    snapshot.push_back(kv.second);
            }
            for (auto &kf : snapshot)
            {
                if (!running_.load())
                    return;
                if (kf->claim())
                {
                    std::lock_guard<std::mutex> lock(mapMutex_);
                    processKeyframeLocked(kf);
                }
                if (globalSleepMs_ > 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(globalSleepMs_));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}  // namespace traversability_mapping
