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
#ifndef TRAVERSABILITY_LOCALMAP_HPP_
#define TRAVERSABILITY_LOCALMAP_HPP_

// Per-submap moment-fused map. Owns ONE fixed-frame, growing grid_map whose layers
// hold the fused moments + derived hazard/nav layers, the keyframes routed to it,
// and TWO worker threads that turn keyframe contributions into traversability:
//
//   * RunLocalKeyFrames - services dirty keyframes in the last-N window, fast.
//   * RunTraversability  - services ALL dirty keyframes, slower; a keyframe that
//                          has nothing pending is skipped (claim() fails), so the
//                          sweep is cheap, not redundant.
//
// Both run the same per-keyframe op (subtract old partials -> rebin at current pose
// -> add -> recompute hazards over the dilated touched cells). All grid mutation is
// serialized on a single mutex; the per-keyframe atomic dirty/claim flag makes a
// double-touch by both workers idempotent.
//
// ROS-free: the grid uses grid_map_core; the nav output is handed out as plain data
// (NavDelta), and the live grid + its mutex are exposed for the debug publish path.
// The keyframe never touches the grid; LocalMap performs all grid arithmetic.

#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>

#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "traversability_mapping/Moments.hpp"
#include "traversability_mapping/KeyFrame.hpp"

namespace traversability_mapping
{
    /// Plain-data sparse nav update (mirrors TraversabilitySparseUpdate.msg minus
    /// the ROS header stamp). Values are row-major: for each cell key in order, one
    /// float per layer in `layers` order. A cell whose nav layers are all NaN is
    /// still emitted (it tells nav to clear that cell).
    struct NavDelta
    {
        std::string frame_id;
        double resolution = 0.0;
        double origin_x = 0.0;
        double origin_y = 0.0;
        std::vector<std::string> layers;            ///< the nav-layer subset
        bool is_full_snapshot = false;
        std::vector<std::uint64_t> cell_keys;       ///< absolute lattice ids
        std::vector<float> values;                  ///< cell-major, layer-minor
    };

    class LocalMap
    {
    public:
        /// `lattice` is the shared absolute lattice (same across all maps so a
        /// keyframe's partials transfer on re-parent). `mapFrame` is the grid frame
        /// id (cosmetic in the core; the adapter sets the published header frame).
        LocalMap(std::uint64_t mapID, const Lattice &lattice, std::string mapFrame = "map");
        ~LocalMap();

        LocalMap(const LocalMap &) = delete;
        LocalMap &operator=(const LocalMap &) = delete;

        std::uint64_t mapID() const { return mapID_; }

        // --- keyframe lifecycle ------------------------------------------------

        /// Register a keyframe in this map's working set, push it to the local
        /// window, and mark it dirty so a worker bins+adds it. Used for both fresh
        /// additions and re-parented keyframes.
        void addKeyFrame(const std::shared_ptr<KeyFrame> &kf);

        /// Subtract a keyframe's contribution from the grid (recomputing the cells
        /// it vacated), remove it from the working set/window, and return it (or
        /// nullptr). Used by delete and by re-parent-out.
        std::shared_ptr<KeyFrame> detachKeyFrame(std::uint64_t id);

        /// detachKeyFrame + discard (the caller releases the last reference).
        void deleteKeyFrame(std::uint64_t id) { (void)detachKeyFrame(id); }

        /// Clear the whole grid and rebuild from retained clouds: blank every layer,
        /// tell nav the occupied cells are cleared, clear every keyframe's partials,
        /// and mark them dirty so the workers re-add them. Keyframes whose cloud was
        /// dropped (optimization off) cannot be rebuilt.
        void clearEntireMap();

        const std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> &keyframes() const
        {
            return keyframes_;
        }

        // --- outputs -----------------------------------------------------------

        /// Consume the cells changed since the last call and return their nav-layer
        /// values as plain data (the deployment path). Taken under the grid mutex.
        NavDelta drainNavDelta();

        /// Full nav snapshot of every occupied cell (for first connect / reconnect).
        NavDelta fullNavSnapshot();

        /// Live grid + its mutex, for the debug (grid_map / occupancy) publish path.
        /// The caller must hold getGridMapMutex() while touching getGridMap().
        const grid_map::GridMap &getGridMap() const { return gridMap_; }
        std::mutex &getGridMapMutex() { return mapMutex_; }

        /// Voxel-downsampled stitch of every retained keyframe cloud in the map
        /// frame (legacy getStitchedPointCloud).
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getStitchedPointCloud(
            float voxel_size_x, float voxel_size_y, float voxel_size_z);

    private:
        // ---- grid construction / growth ----
        grid_map::GridMap freshMap(double halfX, double halfY) const;
        void growToInclude(double minx, double maxx, double miny, double maxy);
        void growToIncludeCells(const std::unordered_map<std::uint64_t, NodeMetaData> &partials);

        // ---- moment <-> grid helpers ----
        grid_map::Position cellPos(int ci, int cj) const;
        void addToLayer(const std::string &l, const grid_map::Position &p, double v);
        void addPartialToGrid(std::uint64_t cellId, const NodeMetaData &m, double sign);
        void blankCell(const grid_map::Position &p);
        bool readCellMoment(int ci, int cj, NodeMetaData &out) const;

        // ---- recompute ----
        std::unordered_set<std::uint64_t> dilate(const std::unordered_set<std::uint64_t> &touched) const;
        bool recomputeCell(std::uint64_t id);
        void recomputeDirty(const std::unordered_set<std::uint64_t> &dirty);

        // ---- per-keyframe op (mapMutex_ held by caller) ----
        void processKeyframeLocked(const std::shared_ptr<KeyFrame> &kf);

        // ---- nav output helpers (mapMutex_ held by caller) ----
        std::vector<std::uint64_t> allOccupiedKeys() const;
        NavDelta fillNav(const std::vector<std::uint64_t> &keys, bool full) const;

        // ---- worker threads ----
        void RunLocalKeyFrames();
        void RunTraversability();

        // ---- identity / config ----
        std::uint64_t mapID_;
        Lattice lattice_;
        std::string frameId_;
        double res_;
        double ground_clearance_, max_slope_, min_occupied_fraction_;
        unsigned int min_vicinity_points_;
        int delta_ind_;
        std::size_t windowCap_;
        int globalSleepMs_;
        bool kfOptimizationEnabled_;

        std::vector<std::string> layers_;       // all internal layers (moments + derived)
        std::vector<std::string> nav_layers_;   // subset handed to navigation

        // ---- state guarded by mapMutex_ ----
        std::mutex mapMutex_;                    // guards gridMap_, keyframes_, window_, dirty_for_nav_
        grid_map::GridMap gridMap_;
        std::unordered_map<std::uint64_t, std::shared_ptr<KeyFrame>> keyframes_;
        std::deque<std::shared_ptr<KeyFrame>> window_;        // last-N, newest at front
        std::unordered_set<std::uint64_t> dirty_for_nav_;     // cells changed since last drain

        // ---- threads ----
        std::atomic<bool> running_{true};
        std::thread localThread_;
        std::thread globalThread_;
    };
}  // namespace traversability_mapping

#endif  // TRAVERSABILITY_LOCALMAP_HPP_
