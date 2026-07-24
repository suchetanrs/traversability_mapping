/* Unit tests for LocalMap (LocalMap.hpp / LocalMap.cpp).
 *
 * LocalMap owns a fixed-frame growing grid_map, the keyframes routed to it, and TWO
 * worker threads that turn keyframe contributions into traversability. Its private
 * grid arithmetic (addPartialToGrid / recomputeCell / recomputeKeyFrame /
 * growToIncludeCells) and the worker loops are exercised INDIRECTLY through the
 * public surface: register a keyframe (addAlreadyDeclaredKF), set its pose to raise
 * the pending flag, and wait for the onUpdate callback a worker fires once it has
 * (re)binned the keyframe into the grid. State is then read under the grid lock via
 * occupiedCellKeys / takeChangedCells / getGridMap.
 *
 * The ParameterHandler singleton is seeded once in main() from a temp YAML; per-test
 * config (is_kf_optimization_enabled) is flipped via setValue BEFORE the map is
 * constructed, since LocalMap snapshots config in its constructor.
 *
 * These need grid_map_core + PCL + Eigen, all pulled in transitively by
 * traversabilitySystem.
 */
#include <gtest/gtest.h>

#include <traversability_mapping/LocalMap.hpp>
#include <traversability_mapping/KeyFrame.hpp>
#include <traversability_mapping/Parameters.hpp>
#include <traversability_mapping/Moments.hpp>

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

using traversability_mapping::KeyFrame;
using traversability_mapping::Lattice;
using traversability_mapping::LocalMap;
using traversability_mapping::ParameterHandler;

namespace
{
    Eigen::Affine3f translation(float x, float y, float z)
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() = Eigen::Vector3f(x, y, z);
        return t;
    }

    std::set<std::uint64_t> toSet(const std::vector<std::uint64_t> &v)
    {
        return std::set<std::uint64_t>(v.begin(), v.end());
    }

    // A complete params file mirroring traversabilityParams.yaml (res 0.25 => a 3x3
    // vicinity window; initial grid half-extent 7.5 m so a 20 m point forces growth).
    const char *kYaml = R"yaml(
grid/resolution_local_map: 0.25
grid/grid_center_x: 0.0
grid/grid_center_y: 0.0
grid/half_size_traversability: 7.5
grid/extend_length_every_resize_by: 30.0
traversability/robot_radius: 0.4
traversability/ground_clearance: 0.15
traversability/max_slope: 0.4
traversability/min_vicinity_points: 15
traversability/min_occupied_fraction: 0.5
mapping/is_kf_optimization_enabled: true
mapping/num_local_keyframes: 10
mapping/global_adjustment_sleep: 0
ingestion/robot_height: 3.5
ingestion/max_range_base_frame: 300.0
ingestion/min_range_base_frame: 1.0
ingestion/use_pointcloud_buffer: false
ingestion/use_ros_buffer: false
node/publish_rate_hz: 1.0
)yaml";
}  // namespace

// A fixture that owns the map + the update-counter machinery the async workers drive.
class LocalMapTest : public ::testing::Test
{
protected:
    std::mutex m_;
    std::condition_variable cv_;
    int updates_ = 0;      // onUpdate fires (one per grid-changing keyframe op)
    int expected_ = 0;     // updates we have deliberately triggered
    Lattice lat_{0.0, 0.0, 0.25};
    std::unique_ptr<LocalMap> map_;

    void onUpdate()
    {
        std::lock_guard<std::mutex> lk(m_);
        ++updates_;
        cv_.notify_all();
    }

    // Construct the map. `opt` is is_kf_optimization_enabled, snapshotted by the ctor.
    void start(bool opt = true, std::uint64_t id = 1)
    {
        parameterInstance.setValue<bool>("mapping/is_kf_optimization_enabled", opt);
        map_ = std::make_unique<LocalMap>(id, lat_, "map", [this] { onUpdate(); });
    }

    void waitForUpdates(int target, std::chrono::milliseconds to = std::chrono::seconds(5))
    {
        std::unique_lock<std::mutex> lk(m_);
        ASSERT_TRUE(cv_.wait_for(lk, to, [&] { return updates_ >= target; }))
            << "timed out waiting for " << target << " updates (got " << updates_ << ")";
    }

    // Register a keyframe (does NOT bin it yet -- no pending pose raised).
    std::shared_ptr<KeyFrame> add(std::uint64_t id, std::vector<Eigen::Vector3f> pts,
                                  const Eigen::Affine3f &pose = Eigen::Affine3f::Identity())
    {
        auto kf = std::make_shared<KeyFrame>(id, pose, std::move(pts));
        map_->addAlreadyDeclaredKF(kf);
        return kf;
    }

    // Set the pose through the map (enqueues onto the local work queue) and block
    // until a worker has processed it.
    void processOnce(const std::shared_ptr<KeyFrame> &kf, const Eigen::Affine3f &pose)
    {
        ++expected_;
        map_->setKeyFramePose(kf, pose);
        waitForUpdates(expected_);
    }

    // ---- grid reads (must hold the grid lock) ----
    std::vector<std::uint64_t> occupied()
    {
        std::lock_guard<std::mutex> lk(map_->getGridMapMutex());
        return map_->occupiedCellKeys();
    }
    std::vector<std::uint64_t> drainChanged()
    {
        std::lock_guard<std::mutex> lk(map_->getGridMapMutex());
        return map_->takeChangedCells();
    }
    double gridLengthX()
    {
        std::lock_guard<std::mutex> lk(map_->getGridMapMutex());
        return map_->getGridMap().getLength().x();
    }
};

// --- Construction + immutable accessors -------------------------------------

TEST_F(LocalMapTest, ConstructionExposesIdAndLattice)
{
    start(true, 42);
    EXPECT_DOUBLE_EQ(map_->getLattice().x0, 0.0);
    EXPECT_DOUBLE_EQ(map_->getLattice().y0, 0.0);
    EXPECT_DOUBLE_EQ(map_->getLattice().res, 0.25);
    EXPECT_TRUE(map_->getKeyFramesMap().empty());
}

TEST_F(LocalMapTest, GridHasAllMomentAndDerivedLayers)
{
    start();
    std::lock_guard<std::mutex> lk(map_->getGridMapMutex());
    const auto &g = map_->getGridMap();
    for (const char *l : {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz",
                          "hazard", "elevation", "slope_haz", "step_haz", "roughness_haz",
                          "normal_x", "normal_y", "normal_z"})
        EXPECT_TRUE(g.exists(l)) << "missing layer " << l;
    // Fresh grid: nothing occupied.
    EXPECT_TRUE(map_->occupiedCellKeys().empty());
}

// --- Registration (no grid mutation until a pose is set) ---------------------

TEST_F(LocalMapTest, AddRegistersWithoutMutatingGrid)
{
    start();
    auto kf = add(7, {{0.f, 0.f, 0.f}});
    EXPECT_EQ(map_->getKeyFramesMap().count(7), 1u);
    // No pending pose was raised -> workers skip it -> grid stays empty.
    EXPECT_TRUE(occupied().empty());
    EXPECT_TRUE(drainChanged().empty());
}

TEST_F(LocalMapTest, AddMultipleTracksAllInKeyFramesMap)
{
    start();
    add(1, {{0.f, 0.f, 0.f}});
    add(2, {{1.f, 0.f, 0.f}});
    add(3, {{2.f, 0.f, 0.f}});
    EXPECT_EQ(map_->getKeyFramesMap().size(), 3u);
    EXPECT_EQ(map_->getKeyFramesMap().count(2), 1u);
}

// --- Processing: keyframe -> grid (workers + recomputeKeyFrame + addPartial) --

TEST_F(LocalMapTest, SetPosePopulatesGridCells)
{
    start();
    // 4 points at map coords (0,0),(1,0),(0,1),(2,2) => cells (0,0),(4,0),(0,4),(8,8).
    auto kf = add(1, {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {2.f, 2.f, 0.f}});
    processOnce(kf, Eigen::Affine3f::Identity());

    const std::set<std::uint64_t> want = {Lattice::key(0, 0), Lattice::key(4, 0),
                                          Lattice::key(0, 4), Lattice::key(8, 8)};
    EXPECT_EQ(toSet(occupied()), want);
}

TEST_F(LocalMapTest, ChangedCellsRecordedThenClearedOnDrain)
{
    start();
    auto kf = add(1, {{0.f, 0.f, 0.f}, {2.f, 2.f, 0.f}});  // cells (0,0),(8,8)
    processOnce(kf, Eigen::Affine3f::Identity());

    const std::set<std::uint64_t> changed = toSet(drainChanged());
    EXPECT_EQ(changed.count(Lattice::key(0, 0)), 1u);
    EXPECT_EQ(changed.count(Lattice::key(8, 8)), 1u);
    // takeChangedCells clears the set: a second drain is empty.
    EXPECT_TRUE(drainChanged().empty());
}

TEST_F(LocalMapTest, GridGrowsToIncludeFarKeyframe)
{
    start();
    const double before = gridLengthX();  // ~15.25 m (half 7.5)
    // Map point (20,0,0) is well outside the initial grid -> forces a resize.
    auto kf = add(1, {{20.f, 0.f, 0.f}});
    processOnce(kf, Eigen::Affine3f::Identity());

    EXPECT_GT(gridLengthX(), before);
    EXPECT_EQ(toSet(occupied()).count(Lattice::key(80, 0)), 1u);  // round(20/0.25)=80
}

TEST_F(LocalMapTest, PoseUpdateMovesContribution)
{
    start();
    auto kf = add(1, {{0.f, 0.f, 0.f}});          // cell (0,0) at identity
    processOnce(kf, Eigen::Affine3f::Identity());
    ASSERT_EQ(toSet(occupied()).count(Lattice::key(0, 0)), 1u);

    // Re-bin at a +10 m x translation: old cell subtracted+blanked, new one added.
    processOnce(kf, translation(10.f, 0.f, 0.f));  // map (10,0,0) => cell (40,0)
    const std::set<std::uint64_t> occ = toSet(occupied());
    EXPECT_EQ(occ.count(Lattice::key(40, 0)), 1u);
    EXPECT_EQ(occ.count(Lattice::key(0, 0)), 0u);   // vacated cell blanked
}

// --- detach / delete ---------------------------------------------------------

TEST_F(LocalMapTest, DetachRemovesContributionAndKeyFrame)
{
    start();
    auto kf = add(1, {{0.f, 0.f, 0.f}, {2.f, 2.f, 0.f}});
    processOnce(kf, Eigen::Affine3f::Identity());
    ASSERT_FALSE(occupied().empty());

    auto detached = map_->detachKeyFrame(1);
    ASSERT_NE(detached, nullptr);
    EXPECT_EQ(detached->getKfID(), 1u);
    EXPECT_EQ(map_->getKeyFramesMap().count(1), 0u);   // gone from the working set
    EXPECT_TRUE(occupied().empty());                    // its cells were subtracted
}

TEST_F(LocalMapTest, DetachUnknownIdReturnsNull)
{
    start();
    EXPECT_EQ(map_->detachKeyFrame(999), nullptr);
}

TEST_F(LocalMapTest, DeleteKeyFrameRemovesIt)
{
    start();
    auto kf = add(5, {{0.f, 0.f, 0.f}});
    processOnce(kf, Eigen::Affine3f::Identity());
    ASSERT_EQ(map_->getKeyFramesMap().count(5), 1u);

    map_->deleteKeyFrame(5);
    EXPECT_EQ(map_->getKeyFramesMap().count(5), 0u);
    EXPECT_TRUE(occupied().empty());
}

// --- clearEntireMap ----------------------------------------------------------

TEST_F(LocalMapTest, ClearBlanksGridAndRecordsChanged)
{
    // Optimization OFF so the retained cloud is dropped after the first bin; clear
    // then re-raises pending but the workers cannot re-add (no cloud) -> the blanked
    // state is stable, making the assertion race-free.
    start(/*opt=*/false);
    auto kf = add(1, {{0.f, 0.f, 0.f}, {2.f, 2.f, 0.f}});  // cells (0,0),(8,8)
    processOnce(kf, Eigen::Affine3f::Identity());
    const std::set<std::uint64_t> wasOccupied = toSet(occupied());
    ASSERT_FALSE(wasOccupied.empty());

    map_->clearEntireMap();

    EXPECT_TRUE(occupied().empty());  // every layer blanked, nothing re-added
    const std::set<std::uint64_t> changed = toSet(drainChanged());
    for (auto k : wasOccupied)
        EXPECT_EQ(changed.count(k), 1u) << "cleared cell not recorded as changed";
}

// --- getStitchedPointCloud (synchronous; no worker involvement) --------------

TEST_F(LocalMapTest, StitchTransformsRetainedCloudsByPose)
{
    start();
    // Pose translates the base cloud into the map frame; no voxel filtering.
    add(1, {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}}, translation(5.f, -3.f, 2.f));
    auto stitched = map_->getStitchedPointCloud(0.f, 0.f, 0.f);
    ASSERT_EQ(stitched->size(), 2u);

    std::set<std::tuple<float, float, float>> pts;
    for (const auto &p : *stitched)
        pts.insert({p.x, p.y, p.z});
    EXPECT_EQ(pts.count({6.f, -3.f, 2.f}), 1u);   // (1,0,0)+(5,-3,2)
    EXPECT_EQ(pts.count({5.f, -2.f, 2.f}), 1u);   // (0,1,0)+(5,-3,2)
}

TEST_F(LocalMapTest, StitchSkipsKeyFramesWithoutCloud)
{
    start();
    add(1, {{1.f, 1.f, 1.f}});    // has a cloud
    add(2, {});                    // empty cloud -> hasCloud() == false -> skipped
    auto stitched = map_->getStitchedPointCloud(0.f, 0.f, 0.f);
    EXPECT_EQ(stitched->size(), 1u);
}

TEST_F(LocalMapTest, StitchVoxelDownsampleReducesCount)
{
    start();
    // Ten points inside a 1 cm cube collapse to a single voxel at leaf size 1 m.
    std::vector<Eigen::Vector3f> pts;
    for (int i = 0; i < 10; ++i)
        pts.push_back({0.001f * i, 0.f, 0.f});
    add(1, std::move(pts));
    auto stitched = map_->getStitchedPointCloud(1.f, 1.f, 1.f);
    EXPECT_GE(stitched->size(), 1u);
    EXPECT_LT(stitched->size(), 10u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // Seed the ParameterHandler singleton from a temp YAML BEFORE any LocalMap is
    // constructed (the ctor reads config through the singleton).
    const std::string path = ::testing::TempDir() + "/tmap_localmap_params.yaml";
    {
        std::ofstream out(path);
        out << kYaml;
    }
    (void)ParameterHandler::getInstance(path);
    return RUN_ALL_TESTS();
}
