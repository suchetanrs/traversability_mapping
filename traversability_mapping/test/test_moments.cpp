/* Unit tests for the Eigen-only moment-fusion core (Moments.hpp).
 *
 * These tests intentionally depend only on Eigen + gtest so they can be built
 * and run without ROS / colcon. They exercise seam 1 (NodeMetaData moment
 * algebra) and seam 2 (computeGoodness PCA hazards).
 */
#include <gtest/gtest.h>
#include <traversability_mapping/Moments.hpp>
#include <traversability_mapping/TraversabilityMetrics.hpp>

#include <functional>
#include <random>

using traversability_mapping::NodeMetaData;
using traversability_mapping::Lattice;
using traversability_mapping::CellMoment;
using traversability_mapping::computeGoodness;
namespace tmap = traversability_mapping;

namespace
{
    // Build a CellMoment for a single cell from raw map-frame points, storing
    // moments cell-locally about `center`.
    CellMoment makeCell(const Eigen::Vector3d &center,
                        const std::vector<Eigen::Vector3d> &pts_map)
    {
        CellMoment c;
        c.center = center;
        for (const auto &p : pts_map)
        {
            const Eigen::Vector3d q = p - center;
            c.data.insert(q.x(), q.y(), q.z());
        }
        return c;
    }

    // Sigma = Q/N - mu mu^T. The library derives this inline where it needs it
    // (computeGoodness); the tests re-derive it here rather than depend on an accessor.
    Eigen::Matrix3d covarianceOf(const NodeMetaData &m)
    {
        const Eigen::Vector3d mu = m.barycenter();
        return m.Q() / static_cast<double>(m.N) - mu * mu.transpose();
    }
}  // namespace

// --- Seam 1: moment algebra --------------------------------------------------

TEST(Moments, FuseEqualsCombinedInsert)
{
    NodeMetaData a, b, combined;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u(-0.1, 0.1);
    for (int i = 0; i < 100; ++i)
    {
        double x = u(rng), y = u(rng), z = u(rng);
        if (i % 2 == 0) a.insert(x, y, z); else b.insert(x, y, z);
        combined.insert(x, y, z);
    }
    NodeMetaData fused = a;
    fused.fuseWith(b);
    EXPECT_EQ(fused.N, combined.N);
    EXPECT_NEAR(fused.sx2, combined.sx2, 1e-9);
    EXPECT_NEAR(fused.sxy, combined.sxy, 1e-9);
    EXPECT_NEAR(fused.sz, combined.sz, 1e-9);
}

TEST(Moments, ResetClearsAllMoments)
{
    NodeMetaData m;
    for (int i = 0; i < 10; ++i) m.insert(0.1 * i, -0.2 * i, 0.3 * i);
    ASSERT_EQ(m.N, 10u);
    m.reset();
    EXPECT_EQ(m.N, 0u);
    EXPECT_EQ(m.sx, 0.0);
    EXPECT_EQ(m.sy, 0.0);
    EXPECT_EQ(m.sz, 0.0);
    EXPECT_EQ(m.sx2, 0.0);
    EXPECT_EQ(m.sy2, 0.0);
    EXPECT_EQ(m.sz2, 0.0);
    EXPECT_EQ(m.sxy, 0.0);
    EXPECT_EQ(m.sxz, 0.0);
    EXPECT_EQ(m.syz, 0.0);
}

TEST(Moments, BarycenterIsMeanOfPoints)
{
    std::vector<Eigen::Vector3d> pts;
    std::mt19937 rng(13);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    NodeMetaData m;
    for (int i = 0; i < 100; ++i)
    {
        Eigen::Vector3d p(u(rng), u(rng), u(rng));
        m.insert(p.x(), p.y(), p.z());
        sum += p;
    }
    Eigen::Vector3d expected = sum / 100.0;
    EXPECT_TRUE(m.barycenter().isApprox(expected, 1e-12));
}

TEST(Moments, ShiftReExpressesAboutNewOrigin)
{
    // Two cells store the same points about DIFFERENT local origins. After
    // shifting cell B into cell A's origin, the two must describe the same
    // points: barycenters coincide (in A's frame) and covariance is unchanged.
    std::vector<Eigen::Vector3d> pts;
    std::mt19937 rng(17);
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    for (int i = 0; i < 150; ++i) pts.emplace_back(u(rng), u(rng), u(rng));

    const Eigen::Vector3d originA(1.0, 2.0, 0.0);
    const Eigen::Vector3d originB(1.25, 2.0, 0.0);  // one cell over in x

    NodeMetaData a, b;
    for (const auto &p : pts)
    {
        Eigen::Vector3d qa = p - originA;
        Eigen::Vector3d qb = p - originB;
        a.insert(qa.x(), qa.y(), qa.z());
        b.insert(qb.x(), qb.y(), qb.z());
    }

    // Bring b from its own origin into a's origin: local coords gain (originB - originA).
    b.shift(originB - originA);

    EXPECT_TRUE(a.barycenter().isApprox(b.barycenter(), 1e-12));
    EXPECT_TRUE(covarianceOf(a).isApprox(covarianceOf(b), 1e-12));
}

TEST(Moments, ShiftedCellsFuseAsCombinedInsert)
{
    // The reason shift exists: fuse neighbouring cells (each about their own
    // centre) by re-expressing them about a common origin first. The result
    // must equal inserting all points directly about that common origin.
    std::vector<Eigen::Vector3d> ptsA, ptsB;
    std::mt19937 rng(19);
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    for (int i = 0; i < 60; ++i) ptsA.emplace_back(u(rng), u(rng), u(rng));
    for (int i = 0; i < 40; ++i) ptsB.emplace_back(2.0 + u(rng), u(rng), u(rng));

    const Eigen::Vector3d originA(0.0, 0.0, 0.0);
    const Eigen::Vector3d originB(2.0, 0.0, 0.0);

    NodeMetaData a, b;
    for (const auto &p : ptsA) { Eigen::Vector3d q = p - originA; a.insert(q.x(), q.y(), q.z()); }
    for (const auto &p : ptsB) { Eigen::Vector3d q = p - originB; b.insert(q.x(), q.y(), q.z()); }

    // Fuse about A's origin.
    b.shift(originB - originA);
    NodeMetaData fused = a;
    fused.fuseWith(b);

    NodeMetaData combined;
    for (const auto &p : ptsA) { Eigen::Vector3d q = p - originA; combined.insert(q.x(), q.y(), q.z()); }
    for (const auto &p : ptsB) { Eigen::Vector3d q = p - originA; combined.insert(q.x(), q.y(), q.z()); }

    EXPECT_EQ(fused.N, combined.N);
    EXPECT_NEAR(fused.sx, combined.sx, 1e-9);
    EXPECT_NEAR(fused.sx2, combined.sx2, 1e-9);
    EXPECT_NEAR(fused.sxy, combined.sxy, 1e-9);
    EXPECT_NEAR(fused.sxz, combined.sxz, 1e-9);
    EXPECT_TRUE(fused.barycenter().isApprox(combined.barycenter(), 1e-9));
}

// --- Seam 2: PCA hazards -----------------------------------------------------

// Build a 3x3 vicinity of cells sampling a height field z = f(x,y).
static std::vector<CellMoment> sampleField(const Lattice &lat, int cx, int cy,
                                           std::function<double(double, double)> f,
                                           double noise = 0.0, unsigned seed = 5)
{
    std::vector<CellMoment> cells;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.4 * lat.res, 0.4 * lat.res);
    std::uniform_real_distribution<double> nz(-noise, noise);
    for (int i = cx - 1; i <= cx + 1; ++i)
        for (int j = cy - 1; j <= cy + 1; ++j)
        {
            Eigen::Vector2d ctr = lat.centerOf(i, j);
            std::vector<Eigen::Vector3d> pts;
            for (int k = 0; k < 20; ++k)
            {
                double x = ctr.x() + jitter(rng);
                double y = ctr.y() + jitter(rng);
                double z = f(x, y) + nz(rng);
                pts.emplace_back(x, y, z);
            }
            cells.push_back(makeCell(Eigen::Vector3d(ctr.x(), ctr.y(), 0.0), pts));
        }
    return cells;
}

TEST(Goodness, FlatGroundIsFlatAndSmooth)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 2.0; });
    CellMoment query = cells[4];  // the centre cell (i=0,j=0 is the 5th of 3x3)
    auto haz = computeGoodness(query, cells, 9, 0.15, 0.4, 15);
    EXPECT_NEAR(haz[tmap::HAZ_SLOPE], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_ROUGHNESS], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_STEP], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_ELEVATION], 2.0, 1e-2);
    // 20 points per cell against min_points_per_grid=15 -> density completeness saturates.
    EXPECT_EQ(haz[tmap::HAZ_BORDER], 1.0);
}

TEST(Goodness, RampRecoversSlopeAngle)
{
    Lattice lat(0.0, 0.0, 0.25);
    const double angle = 20.0 * M_PI / 180.0;
    const double m = std::tan(angle);
    auto cells = sampleField(lat, 0, 0, [m](double x, double) { return m * x; });
    CellMoment query = cells[4];
    const double max_pitch = 0.4;  // rad
    auto haz = computeGoodness(query, cells, 9, 0.15, max_pitch, 15);
    double slope_rad = haz[tmap::HAZ_SLOPE] * max_pitch;
    EXPECT_NEAR(slope_rad, angle, 2.0 * M_PI / 180.0);  // within 2 deg
}

TEST(Goodness, RoughTerrainHasRoughness)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto flat = sampleField(lat, 0, 0, [](double, double) { return 0.0; }, 0.0);
    auto rough = sampleField(lat, 0, 0, [](double, double) { return 0.0; }, 0.05);
    auto hf = computeGoodness(flat[4], flat, 9, 0.15, 0.4, 15);
    auto hr = computeGoodness(rough[4], rough, 9, 0.15, 0.4, 15);
    EXPECT_GT(hr[tmap::HAZ_ROUGHNESS], hf[tmap::HAZ_ROUGHNESS]);
}

TEST(Goodness, OccupiedFractionGateRejectsSparse)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 0.0; });
    // keep only query + 1 neighbour out of 9 -> 22% occupied < 50%
    std::vector<CellMoment> sparse = {cells[4], cells[0]};
    auto haz = computeGoodness(cells[4], sparse, 9, 0.15, 0.4, 15);
    EXPECT_EQ(haz[tmap::HAZ_BORDER], 1.0);
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_OVERALL]));
}

TEST(Goodness, FusionUsesAllPointsAcrossKeyframes)
{
    // Two keyframes each contribute half the points of the same cell; the fused
    // normal must match fitting all points at once.
    Lattice lat(0.0, 0.0, 0.25);
    const double m = std::tan(15.0 * M_PI / 180.0);
    auto kfA = sampleField(lat, 0, 0, [m](double x, double) { return m * x; }, 0.0, 11);
    auto kfB = sampleField(lat, 0, 0, [m](double x, double) { return m * x; }, 0.0, 22);

    std::vector<CellMoment> fused;
    for (size_t i = 0; i < kfA.size(); ++i)
    {
        CellMoment c = kfA[i];
        c.data.fuseWith(kfB[i].data);  // same cell, same centre -> direct sum
        fused.push_back(c);
    }
    auto hz = computeGoodness(fused[4], fused, 9, 0.15, 0.4, 30);
    double slope_rad = hz[tmap::HAZ_SLOPE] * 0.4;
    EXPECT_NEAR(slope_rad, 15.0 * M_PI / 180.0, 2.0 * M_PI / 180.0);
    EXPECT_EQ(fused[4].data.N, kfA[4].data.N + kfB[4].data.N);
}

TEST(Goodness, EmptyQueryReturnsAllNaN)
{
    // A query cell with no points cannot produce any hazard.
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 0.0; });
    CellMoment emptyQuery;
    emptyQuery.center = Eigen::Vector3d(0.0, 0.0, 0.0);  // data.N == 0
    auto haz = computeGoodness(emptyQuery, cells, 9, 0.15, 0.4, 15);
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_OVERALL]));
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_ELEVATION]));
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_BORDER]));
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_SLOPE]));
}

TEST(Goodness, ElevationAvailableEvenWhenGated)
{
    // Elevation comes from the query cell's own centroid and must be reported
    // even when the vicinity gates reject the cell (BORDER == 1).
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 2.0; });
    std::vector<CellMoment> sparse = {cells[4], cells[0]};  // 22% occupied < 50%
    auto haz = computeGoodness(cells[4], sparse, 9, 0.15, 0.4, 15);
    EXPECT_EQ(haz[tmap::HAZ_BORDER], 1.0);
    EXPECT_FALSE(std::isnan(haz[tmap::HAZ_ELEVATION]));
    EXPECT_NEAR(haz[tmap::HAZ_ELEVATION], 2.0, 1e-2);
}

TEST(Goodness, InsufficientVicinityPointsGate)
{
    // All 9 cells are present, but the per-cell point count (20) falls short of
    // the min_points_per_grid demand -> BORDER == 1.
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 0.0; });
    // Each cell holds 20 points; demand far more per grid than that.
    auto haz = computeGoodness(cells[4], cells, 9, 0.15, 0.4, /*min_points_per_grid=*/100000);
    // border_haz scales the query's own count against the demand (20 / 100000).
    EXPECT_NEAR(haz[tmap::HAZ_BORDER], 20.0 / 100000.0, 1e-9);
    EXPECT_TRUE(std::isnan(haz[tmap::HAZ_OVERALL]));
    // Elevation is still available (set before the gates).
    EXPECT_FALSE(std::isnan(haz[tmap::HAZ_ELEVATION]));
}

TEST(Goodness, FlatNormalPointsUp)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 1.0; });
    auto haz = computeGoodness(cells[4], cells, 9, 0.15, 0.4, 15);
    Eigen::Vector3d n(haz[tmap::HAZ_NORMAL_X], haz[tmap::HAZ_NORMAL_Y], haz[tmap::HAZ_NORMAL_Z]);
    EXPECT_NEAR(n.norm(), 1.0, 1e-6);          // unit normal
    EXPECT_GT(n.z(), 0.0);                      // sign-corrected to point up
    EXPECT_TRUE(n.isApprox(Eigen::Vector3d::UnitZ(), 1e-2));
}

TEST(Goodness, RampNormalMatchesSlope)
{
    // For z = m*x the upward unit normal is (-m, 0, 1)/sqrt(1+m^2), i.e.
    // n.z() = cos(angle), n.x() = -sin(angle).
    Lattice lat(0.0, 0.0, 0.25);
    const double angle = 20.0 * M_PI / 180.0;
    const double m = std::tan(angle);
    auto cells = sampleField(lat, 0, 0, [m](double x, double) { return m * x; });
    auto haz = computeGoodness(cells[4], cells, 9, 0.15, 0.4, 15);
    Eigen::Vector3d n(haz[tmap::HAZ_NORMAL_X], haz[tmap::HAZ_NORMAL_Y], haz[tmap::HAZ_NORMAL_Z]);
    EXPECT_NEAR(n.norm(), 1.0, 1e-6);
    EXPECT_NEAR(n.z(), std::cos(angle), 1e-2);
    EXPECT_NEAR(n.x(), -std::sin(angle), 2e-2);
    EXPECT_NEAR(n.y(), 0.0, 1e-2);
}

TEST(Goodness, SteepSlopeClampsToOne)
{
    // slope / max_slope > 1 must saturate at 1.0, not exceed it.
    Lattice lat(0.0, 0.0, 0.25);
    const double m = std::tan(20.0 * M_PI / 180.0);
    auto cells = sampleField(lat, 0, 0, [m](double x, double) { return m * x; });
    auto haz = computeGoodness(cells[4], cells, 9, 0.15, /*max_slope=*/0.05, 15);
    EXPECT_EQ(haz[tmap::HAZ_SLOPE], 1.0);
}

TEST(Goodness, StepDiscontinuityProducesStep)
{
    // A height discontinuity across the vicinity must surface as a step hazard
    // well above the flat-ground baseline.
    Lattice lat(0.0, 0.0, 0.25);
    auto flat = sampleField(lat, 0, 0, [](double, double) { return 0.0; });
    auto stepped = sampleField(lat, 0, 0, [](double x, double) { return x < 0.0 ? 0.0 : 0.4; });
    auto hf = computeGoodness(flat[4], flat, 9, 0.15, 0.4, 15);
    auto hs = computeGoodness(stepped[4], stepped, 9, 0.15, 0.4, 15);
    EXPECT_GT(hs[tmap::HAZ_STEP], hf[tmap::HAZ_STEP]);
    EXPECT_GT(hs[tmap::HAZ_STEP], 0.3);
}

TEST(Goodness, OverallIsMaxOfComponentHazards)
{
    Lattice lat(0.0, 0.0, 0.25);
    const double m = std::tan(10.0 * M_PI / 180.0);
    auto cells = sampleField(lat, 0, 0, [m](double x, double) { return m * x; }, 0.02);
    auto haz = computeGoodness(cells[4], cells, 9, 0.15, 0.4, 15);
    const double expected = std::max(haz[tmap::HAZ_SLOPE],
                              std::max(haz[tmap::HAZ_STEP], haz[tmap::HAZ_ROUGHNESS]));
    EXPECT_EQ(haz[tmap::HAZ_OVERALL], expected);
}

// --- Lattice -----------------------------------------------------------------

TEST(Lattice, RoundTripAndKey)
{
    Lattice lat(0.0, 0.0, 0.25);
    int ci, cj;
    lat.cellOf(1.04, -0.36, ci, cj);
    EXPECT_EQ(ci, 4);    // round(1.04/0.25)=round(4.16)=4
    EXPECT_EQ(cj, -1);   // round(-0.36/0.25)=round(-1.44)=-1
    uint64_t k = Lattice::key(ci, cj);
    int ci2, cj2;
    Lattice::unkey(k, ci2, cj2);
    EXPECT_EQ(ci, ci2);
    EXPECT_EQ(cj, cj2);
}

TEST(Lattice, CellCenterIndependentOfMapOrigin)
{
    // The lattice is absolute, so a cell's centre does not depend on any
    // grid_map buffer position -> partials survive a resize.
    Lattice lat(0.0, 0.0, 0.25);
    Eigen::Vector2d c = lat.centerOf(10, -7);
    EXPECT_NEAR(c.x(), 2.5, 1e-12);
    EXPECT_NEAR(c.y(), -1.75, 1e-12);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
