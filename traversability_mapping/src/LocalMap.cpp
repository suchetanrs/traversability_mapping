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
#include "traversability_mapping/Helpers.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace traversability_mapping
{
    namespace
    {
        constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    }

    LocalMap::LocalMap(std::uint64_t mapID, const Lattice &lattice, std::string mapFrame,
                       std::function<void()> onUpdate)
        : mapID_(mapID), lattice_(lattice), frameId_(std::move(mapFrame)), res_(lattice.res),
          onUpdate_(std::move(onUpdate))
    {
        groundClearance_ = parameterInstance.getValue<double>("traversability/ground_clearance");
        maxSlope_ = parameterInstance.getValue<double>("traversability/max_slope");
        minOccupiedFraction_ = parameterInstance.getValue<double>("traversability/min_occupied_fraction");
        minVicinityPoints_ = static_cast<unsigned int>(parameterInstance.getValue<int>("traversability/min_vicinity_points"));
        const double security_distance = parameterInstance.getValue<double>("traversability/security_distance");
        // Vicinity radius in cells (symmetric window of side 2*deltaInd_+1).
        deltaInd_ = std::max(1, static_cast<int>(std::ceil((security_distance / 2.0) / res_)));
        globalSleepMs_ = parameterInstance.getValue<int>("mapping/global_adjustment_sleep");
        kfOptimizationEnabled_ = parameterInstance.getValue<bool>("mapping/is_kf_optimization_enabled");

        layers_ = {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz",
                   "hazard", "elevation", "slope_haz", "step_haz", "roughness_haz",
                   "normal_x", "normal_y", "normal_z"};

        const double half = parameterInstance.getValue<double>("grid/half_size_traversability");
        gridMap_ = makeGridMap(layers_, frameId_, lattice_, res_, half, half);

        localThread_ = std::thread(&LocalMap::RunLocalKeyFrames, this);
        // globalThread_ = std::thread(&LocalMap::RunTraversability, this);
    }

    LocalMap::~LocalMap()
    {
        running_.store(false);
        if (localThread_.joinable())
            localThread_.join();
        if (globalThread_.joinable())
            globalThread_.join();
    }

    // ---- grid growth --------------------------------------------------------

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
        const double extend = parameterInstance.getValue<double>("grid/extend_length_every_resize_by");
        growGridToInclude(gridMap_, layers_, lattice_, res_, extend, minx, maxx, miny, maxy);
    }

    // ---- moment -> grid -----------------------------------------------------

    void LocalMap::addPartialToGrid(std::uint64_t cellId, const NodeMetaData &m, double sign)
    {
        int ci, cj;
        Lattice::unkey(cellId, ci, cj);
        const grid_map::Position p = cellPos(lattice_, ci, cj);
        if (!gridMap_.isInside(p))
            return;
        addToLayer(gridMap_, "N", p, sign * m.N);
        addToLayer(gridMap_, "sx", p, sign * m.sx);   addToLayer(gridMap_, "sy", p, sign * m.sy);   addToLayer(gridMap_, "sz", p, sign * m.sz);
        addToLayer(gridMap_, "sx2", p, sign * m.sx2); addToLayer(gridMap_, "sy2", p, sign * m.sy2); addToLayer(gridMap_, "sz2", p, sign * m.sz2);
        addToLayer(gridMap_, "sxy", p, sign * m.sxy); addToLayer(gridMap_, "sxz", p, sign * m.sxz); addToLayer(gridMap_, "syz", p, sign * m.syz);
        // A subtraction that empties the cell blanks it and records it as changed.
        if (sign < 0 && std::lround(gridMap_.atPosition("N", p)) <= 0)
        {
            blankCell(gridMap_, layers_, p);
            changedCells_.insert(cellId);
        }
    }

    // ---- recompute ----------------------------------------------------------

    bool LocalMap::recomputeCell(std::uint64_t id)
    {
        int ci, cj;
        Lattice::unkey(id, ci, cj);
        const grid_map::Position qp = cellPos(lattice_, ci, cj);
        if (!gridMap_.isInside(qp))
            return false;

        NodeMetaData qd;
        if (!readCellMoment(gridMap_, lattice_, ci, cj, qd))
            return false;  // query cell unobserved -> nothing to write

        CellMoment query{qd, Eigen::Vector3d(qp.x(), qp.y(), 0.0)};
        std::vector<CellMoment> occupied;
        int total = 0;
        for (int i = ci - deltaInd_; i <= ci + deltaInd_; ++i)
            for (int j = cj - deltaInd_; j <= cj + deltaInd_; ++j)
            {
                ++total;
                NodeMetaData d;
                if (readCellMoment(gridMap_, lattice_, i, j, d))
                {
                    const Eigen::Vector2d c2 = lattice_.centerOf(i, j);
                    occupied.push_back({d, Eigen::Vector3d(c2.x(), c2.y(), 0.0)});
                }
            }

        const auto haz = computeGoodness(query, occupied, total, groundClearance_,
                                         maxSlope_, minVicinityPoints_, minOccupiedFraction_);

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
                changedCells_.insert(id);
    }

    // ---- per-keyframe

    bool LocalMap::recomputeKeyFrame(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose)
    {
        // Deleted between snapshot and now?
        {
            std::lock_guard<std::mutex> lock(keyFramesMapMutex_);
            if (keyFramesMap_.find(kf->getKfID()) == keyFramesMap_.end())
            {
                std::cout << "[LocalMap " << mapID_ << "] kf " << kf->getKfID()
                          << ": not in working set (deleted?); skip." << std::endl;
                return false;
            }
        }
        // Cloud dropped (optimization off, already binned once): cannot reprocess;
        // leave the existing contribution intact. THIS is why a PGO pose update has
        // no effect when is_kf_optimization_enabled is false.
        if (!kf->hasCloud())
        {
            std::cout << "[LocalMap " << mapID_ << "] kf " << kf->getKfID()
                      << ": SKIP reprocess -- no retained cloud "
                         "(is_kf_optimization_enabled=false); pose update NOT applied."
                      << std::endl;
            return false;
        }

        std::unordered_set<std::uint64_t> touched;  // M_old (subtracted) U M_new (added)

        // 1. subtract the OLD contribution (computed at the old pose).
        const std::size_t nOld = kf->partials().size();
        {
            std::lock_guard<std::mutex> lock(masterGridMapMutex_);
            for (auto &kv : kf->partials())
            {
                addPartialToGrid(kv.first, kv.second, -1.0);
                touched.insert(kv.first);
            }
        }

        // 2. re-bin the stored base-frame cloud at the pending pose (partition NOT frozen).
        kf->rebin(lattice_, pose);

        // 3. grow to fit, then add the fresh contribution.
        {
            std::lock_guard<std::mutex> lock(masterGridMapMutex_);
            growToIncludeCells(kf->partials());
            const std::size_t nNew = kf->partials().size();
            for (auto &kv : kf->partials())
            {
                addPartialToGrid(kv.first, kv.second, +1.0);
                touched.insert(kv.first);
            }

            // 4. recompute hazards over the dilated union of cells left and newly occupied.
            const auto dirty = dilate(touched, deltaInd_);
            recomputeDirty(dirty);

            std::cout << "[LocalMap " << mapID_ << "] kf " << kf->getKfID()
                      << ": -" << nOld << " cells, +" << nNew << " cells, recomputed "
                      << dirty.size() << " cells, changed=" << changedCells_.size()
                      << "." << std::endl;
        }

        // 5. drop the cloud if optimization is disabled (keyframe becomes non-rebinnable).
        if (!kfOptimizationEnabled_)
            kf->dropCloud();

        return true;
    }

    // ---- keyframe lifecycle -------------------------------------------------

    void LocalMap::addAlreadyDeclaredKF(const std::shared_ptr<KeyFrame> &kf)
    {
        // Pure registration: does NOT flag the keyframe pending or enqueue it. The
        // caller sets a pose via setKeyFramePose once it is ready to be (re)processed;
        // that is what puts it on the local work queue. This avoids ever binning at
        // the placeholder identity pose.
        std::lock_guard<std::mutex> lock(keyFramesMapMutex_);
        kf->setMap(mapID_);
        keyFramesMap_[kf->getKfID()] = kf;
    }

    void LocalMap::setKeyFramePose(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose)
    {
        if (!kf)
            return;
        kf->setPose(pose);                // store latest pose + raise pending (kf's own mutex)
        // ONLY the queue lock -- never the grid lock -- so this never waits on a recompute.
        std::lock_guard<std::mutex> lock(poseQueueMutex_);
        if (localQueued_.insert(kf->getKfID()).second)
            localQueue_.push_back(kf);      // enqueue once (dedup on the membership set)
        std::cout << "[LocalMap " << mapID_ << "] local queue depth: "
                    << localQueue_.size() << "." << std::endl;
    }

    std::shared_ptr<KeyFrame> LocalMap::detachKeyFrame(std::uint64_t id)
    {
        std::lock_guard<std::mutex> lock(masterGridMapMutex_);           // grid (subtract)
        std::lock_guard<std::mutex> kfLock(keyFramesMapMutex_);          // registry (find/erase)
        auto it = keyFramesMap_.find(id);
        if (it == keyFramesMap_.end())
            return nullptr;
        auto kf = it->second;

        std::unordered_set<std::uint64_t> touched;
        for (auto &kv : kf->partials())
        {
            addPartialToGrid(kv.first, kv.second, -1.0);
            touched.insert(kv.first);
        }
        recomputeDirty(dilate(touched, deltaInd_));
        kf->partials().clear();

        keyFramesMap_.erase(it);
        // Clear the dedup marker so a future re-add of this id can re-enqueue; any stale
        // shared_ptr left in localQueue_ self-cleans when the worker pops it (the id is
        // no longer in keyFramesMap_, so recomputeKeyFrame's guard skips it).
        {
            std::lock_guard<std::mutex> qlock(poseQueueMutex_);
            localQueued_.erase(id);
        }
        return kf;
    }

    void LocalMap::clearEntireMap()
    {
        throw std::runtime_error("This function was not supposed to be executed! getStitchedPointCloud()");
        std::lock_guard<std::mutex> lock(masterGridMapMutex_);
        // Record every currently-occupied cell as changed (it is about to be blanked).
        for (auto k : allOccupiedKeys(gridMap_, lattice_))
            changedCells_.insert(k);
        // Blank every cell of every layer.
        for (const auto &l : layers_)
            gridMap_[l].setConstant(kNaN);
        // Clear each keyframe's contribution record, flag it for rebuild at its current
        // pose (re-setting the same pose raises the pending flag), and enqueue it so the
        // local worker re-adds it (keyframes with a dropped cloud simply skip on rebin).
        // Lock order: grid (held) -> keyFramesMap -> queue.
        std::lock_guard<std::mutex> kfLock(keyFramesMapMutex_);
        std::lock_guard<std::mutex> qlock(poseQueueMutex_);
        for (auto &kv : keyFramesMap_)
        {
            kv.second->partials().clear();
            kv.second->setPose(kv.second->getPose());
            if (localQueued_.insert(kv.first).second)
                localQueue_.push_back(kv.second);
        }
    }

    // ---- changed-cell output (layer-agnostic) -------------------------------

    std::vector<std::uint64_t> LocalMap::takeChangedCells()
    {
        // Caller holds masterGridMapMutex_ (so these keys stay consistent with the
        // grid it then reads).
        std::vector<std::uint64_t> keys(changedCells_.begin(), changedCells_.end());
        changedCells_.clear();
        return keys;
    }

    std::vector<std::uint64_t> LocalMap::occupiedCellKeys() const
    {
        // Caller holds masterGridMapMutex_.
        return allOccupiedKeys(gridMap_, lattice_);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LocalMap::getStitchedPointCloud(
        float voxel_size_x, float voxel_size_y, float voxel_size_z)
    {
        throw std::runtime_error("This function was not supposed to be executed! getStitchedPointCloud()");
        auto stitched = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::vector<std::shared_ptr<KeyFrame>> snapshot;
        {
            std::lock_guard<std::mutex> lock(keyFramesMapMutex_);
            snapshot.reserve(keyFramesMap_.size());
            for (auto &kv : keyFramesMap_)
                snapshot.push_back(kv.second);
        }
        for (auto &kf : snapshot)
        {
            if (!kf->hasCloud())
                continue;
            const Eigen::Affine3f Tmb = kf->getPose();
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
        // Fast worker: drain the local work queue at a high rate. The queue is fed by
        // setKeyFramePose, so its depth is exactly how far behind rebinning is.
        while (running_.load())
        {
            // Pop keyframes until the queue empties. Removal is unconditional: even if
            // the pose was already consumed (e.g. by the global backstop), the stale
            // entry leaves the queue here.
            while (running_.load())
            {
                std::shared_ptr<KeyFrame> kf;
                {
                    // Queue lock only: pop is O(1) and never waits on a recompute.
                    std::lock_guard<std::mutex> lock(poseQueueMutex_);
                    if (localQueue_.empty())
                        break;
                    kf = localQueue_.front();     // queue holds the shared_ptr directly
                    localQueue_.pop_front();
                    localQueued_.erase(kf->getKfID());
                }
                Eigen::Affine3f pose;
                if (!kf->getPendingPose(pose))
                    continue;                     // already handled (e.g. global backstop)
                // recomputeKeyFrame owns the grid lock (do NOT wrap it here); its guard
                // skips a keyframe detached since it was enqueued, so a stale shared_ptr
                // in the queue is harmless.
                const bool changed = recomputeKeyFrame(kf, pose);
                if (changed && onUpdate_)
                    onUpdate_();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void LocalMap::RunTraversability()
    {
        // Slow worker: sweep ALL keyframes; a keyframe with no pending pose returns
        // false from getPendingPose() and is skipped, so the sweep is cheap, not
        // redundant re-binning.
        while (running_.load())
        {
            std::vector<std::shared_ptr<KeyFrame>> snapshot;
            {
                std::lock_guard<std::mutex> lock(keyFramesMapMutex_);
                snapshot.reserve(keyFramesMap_.size());
                for (auto &kv : keyFramesMap_)
                    snapshot.push_back(kv.second);
            }
            for (auto &kf : snapshot)
            {
                if (!running_.load())
                    return;
                Eigen::Affine3f pose;
                if (kf->getPendingPose(pose))
                {
                    // recomputeKeyFrame owns the grid lock; do NOT wrap it here.
                    const bool changed = recomputeKeyFrame(kf, pose);
                    if (changed && onUpdate_)
                        onUpdate_();
                }
                if (globalSleepMs_ > 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(globalSleepMs_));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}  // namespace traversability_mapping
