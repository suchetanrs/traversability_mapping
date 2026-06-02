/* Unit tests for the Eigen-only moment-fusion core (Moments.hpp).
 *
 * These tests intentionally depend only on Eigen + gtest so they can be built
 * and run without ROS / colcon. They exercise seam 1 (NodeMetaData moment
 * algebra) and seam 2 (computeGoodness PCA hazards).
 */
#include <gtest/gtest.h>
#include <traversability_mapping/Moments.hpp>

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

TEST(Moments, RemoveIsInverseOfFuse)
{
    NodeMetaData total, part;
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-0.1, 0.1);
    for (int i = 0; i < 50; ++i) total.insert(u(rng), u(rng), u(rng));
    for (int i = 0; i < 20; ++i) { double x=u(rng),y=u(rng),z=u(rng); total.insert(x,y,z); part.insert(x,y,z); }
    NodeMetaData backed = total;
    backed.removeWith(part);
    // backed should equal total-before-part: re-derive
    NodeMetaData expect;
    // recompute expect by replaying first 50 only is messy; instead check
    // fuse(remove) round trip:
    NodeMetaData rt = backed;
    rt.fuseWith(part);
    EXPECT_EQ(rt.N, total.N);
    EXPECT_NEAR(rt.sx2, total.sx2, 1e-9);
    EXPECT_NEAR(rt.syz, total.syz, 1e-9);
}

TEST(Moments, CovarianceIsTranslationInvariant)
{
    // Covariance must not depend on the local origin choice.
    std::vector<Eigen::Vector3d> pts;
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    for (int i = 0; i < 200; ++i) pts.emplace_back(u(rng), u(rng), 0.01 * u(rng));

    NodeMetaData m1, m2;
    Eigen::Vector3d o2(5.0, -3.0, 2.0);
    for (const auto &p : pts)
    {
        m1.insert(p.x(), p.y(), p.z());
        Eigen::Vector3d q = p - o2;
        m2.insert(q.x(), q.y(), q.z());
    }
    EXPECT_TRUE(m1.covariance().isApprox(m2.covariance(), 1e-9));
}

TEST(Moments, CovarianceEigenvaluesRotationInvariant)
{
    std::vector<Eigen::Vector3d> pts;
    std::mt19937 rng(2);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    for (int i = 0; i < 300; ++i) pts.emplace_back(u(rng), u(rng), 0.05 * u(rng));

    NodeMetaData m;
    for (const auto &p : pts) m.insert(p.x(), p.y(), p.z());
    Eigen::Vector3d ev_before = m.covariance().selfadjointView<Eigen::Lower>().eigenvalues();

    Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.5, 0.84).normalized()).toRotationMatrix();
    m.transform(R, Eigen::Vector3d(1.0, 2.0, 3.0));
    Eigen::Vector3d ev_after = m.covariance().selfadjointView<Eigen::Lower>().eigenvalues();

    EXPECT_TRUE(ev_before.isApprox(ev_after, 1e-7));
}

TEST(Moments, TransformMatchesDirectPointTransform)
{
    std::vector<Eigen::Vector3d> pts;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    for (int i = 0; i < 100; ++i) pts.emplace_back(u(rng), u(rng), u(rng));

    Eigen::Matrix3d R = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d t(0.4, -0.2, 1.1);

    NodeMetaData viaMoments;
    for (const auto &p : pts) viaMoments.insert(p.x(), p.y(), p.z());
    viaMoments.transform(R, t);

    NodeMetaData viaPoints;
    for (const auto &p : pts) { Eigen::Vector3d q = R * p + t; viaPoints.insert(q.x(), q.y(), q.z()); }

    EXPECT_NEAR(viaMoments.sx, viaPoints.sx, 1e-9);
    EXPECT_NEAR(viaMoments.sx2, viaPoints.sx2, 1e-9);
    EXPECT_NEAR(viaMoments.sxz, viaPoints.sxz, 1e-9);
    EXPECT_NEAR(viaMoments.syz, viaPoints.syz, 1e-9);
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
    auto haz = computeGoodness(query, cells, 9, 0.15, 0.4, 15, 0.5);
    EXPECT_NEAR(haz[tmap::HAZ_SLOPE], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_ROUGHNESS], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_STEP], 0.0, 1e-3);
    EXPECT_NEAR(haz[tmap::HAZ_ELEVATION], 2.0, 1e-2);
    EXPECT_EQ(haz[tmap::HAZ_BORDER], 0.0);
}

TEST(Goodness, RampRecoversSlopeAngle)
{
    Lattice lat(0.0, 0.0, 0.25);
    const double angle = 20.0 * M_PI / 180.0;
    const double m = std::tan(angle);
    auto cells = sampleField(lat, 0, 0, [m](double x, double) { return m * x; });
    CellMoment query = cells[4];
    const double max_pitch = 0.4;  // rad
    auto haz = computeGoodness(query, cells, 9, 0.15, max_pitch, 15, 0.5);
    double slope_rad = haz[tmap::HAZ_SLOPE] * max_pitch;
    EXPECT_NEAR(slope_rad, angle, 2.0 * M_PI / 180.0);  // within 2 deg
}

TEST(Goodness, RoughTerrainHasRoughness)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto flat = sampleField(lat, 0, 0, [](double, double) { return 0.0; }, 0.0);
    auto rough = sampleField(lat, 0, 0, [](double, double) { return 0.0; }, 0.05);
    auto hf = computeGoodness(flat[4], flat, 9, 0.15, 0.4, 15, 0.5);
    auto hr = computeGoodness(rough[4], rough, 9, 0.15, 0.4, 15, 0.5);
    EXPECT_GT(hr[tmap::HAZ_ROUGHNESS], hf[tmap::HAZ_ROUGHNESS]);
}

TEST(Goodness, OccupiedFractionGateRejectsSparse)
{
    Lattice lat(0.0, 0.0, 0.25);
    auto cells = sampleField(lat, 0, 0, [](double, double) { return 0.0; });
    // keep only query + 1 neighbour out of 9 -> 22% occupied < 50%
    std::vector<CellMoment> sparse = {cells[4], cells[0]};
    auto haz = computeGoodness(cells[4], sparse, 9, 0.15, 0.4, 15, 0.5);
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
    auto hz = computeGoodness(fused[4], fused, 9, 0.15, 0.4, 30, 0.5);
    double slope_rad = hz[tmap::HAZ_SLOPE] * 0.4;
    EXPECT_NEAR(slope_rad, 15.0 * M_PI / 180.0, 2.0 * M_PI / 180.0);
    EXPECT_EQ(fused[4].data.N, kfA[4].data.N + kfB[4].data.N);
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
