/* Unit tests for the generic grid/lattice helpers (Helpers.hpp / Helpers.cpp).
 *
 * These exercise the stateless free functions that LocalMap delegates to:
 * cellPos, makeGridMap, growGridToInclude, addToLayer, blankCell, readCellMoment,
 * allOccupiedKeys, and dilate. They need grid_map_core + Eigen but no ROS.
 */
#include <gtest/gtest.h>
#include <traversability_mapping/Helpers.hpp>
#include <traversability_mapping/Moments.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

using traversability_mapping::Lattice;
using traversability_mapping::NodeMetaData;
namespace tmap = traversability_mapping;

namespace
{
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

    // The moment layers readCellMoment / addToLayer operate on, plus a couple of
    // derived layers so blankCell has more than the moment set to clear.
    std::vector<std::string> momentLayers()
    {
        return {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz", "hazard"};
    }

    // A NodeMetaData with distinct values per field so a mis-mapped layer is caught.
    NodeMetaData sampleMoment()
    {
        NodeMetaData m;
        m.N = 4;
        m.sx = 1.0; m.sy = 2.0; m.sz = 3.0;
        m.sx2 = 4.0; m.sy2 = 5.0; m.sz2 = 6.0;
        m.sxy = 7.0; m.sxz = 8.0; m.syz = 9.0;
        return m;
    }
}  // namespace

// --- cellPos -----------------------------------------------------------------

TEST(Helpers, CellPosIsLatticeCentre)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::Position p = tmap::cellPos(lat, 10, -7);
    EXPECT_NEAR(p.x(), 2.5, 1e-12);
    EXPECT_NEAR(p.y(), -1.75, 1e-12);
}

TEST(Helpers, CellPosHonoursLatticeOrigin)
{
    Lattice lat(1.0, -2.0, 0.5);
    grid_map::Position p = tmap::cellPos(lat, 2, 4);
    EXPECT_NEAR(p.x(), 1.0 + 2 * 0.5, 1e-12);
    EXPECT_NEAR(p.y(), -2.0 + 4 * 0.5, 1e-12);
}

// --- makeGridMap -------------------------------------------------------------

TEST(Helpers, MakeGridMapHasLayersFrameAndNaNFill)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);

    EXPECT_EQ(g.getFrameId(), "map");
    for (const auto &l : momentLayers())
        EXPECT_TRUE(g.exists(l));
    // Position pinned to the lattice origin so absolute cell ids survive resize.
    EXPECT_NEAR(g.getPosition().x(), 0.0, 1e-12);
    EXPECT_NEAR(g.getPosition().y(), 0.0, 1e-12);
    // Every cell starts NaN.
    EXPECT_TRUE(std::isnan(g.atPosition("N", grid_map::Position(0.0, 0.0))));
}

TEST(Helpers, MakeGridMapOddCellCountCentresOnLattice)
{
    // ODD cell count per axis => a cell centre sits exactly on the lattice origin.
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Size sz = g.getSize();
    EXPECT_EQ(sz(0) % 2, 1);
    EXPECT_EQ(sz(1) % 2, 1);
    EXPECT_TRUE(g.isInside(grid_map::Position(0.0, 0.0)));
}

// --- addToLayer --------------------------------------------------------------

TEST(Helpers, AddToLayerInitialisesNaNAsZero)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Position p(0.0, 0.0);
    ASSERT_TRUE(std::isnan(g.atPosition("N", p)));
    tmap::addToLayer(g, "N", p, 3.0);
    EXPECT_FLOAT_EQ(g.atPosition("N", p), 3.0f);
    tmap::addToLayer(g, "N", p, 2.0);   // accumulates, not overwrites
    EXPECT_FLOAT_EQ(g.atPosition("N", p), 5.0f);
}

// --- blankCell ---------------------------------------------------------------

TEST(Helpers, BlankCellSetsEveryLayerNaN)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Position p(0.0, 0.0);
    for (const auto &l : momentLayers())
        tmap::addToLayer(g, l, p, 1.0);
    tmap::blankCell(g, momentLayers(), p);
    for (const auto &l : momentLayers())
        EXPECT_TRUE(std::isnan(g.atPosition(l, p))) << "layer " << l << " not blanked";
}

// --- addToLayer + readCellMoment round trip ----------------------------------

TEST(Helpers, ReadCellMomentRoundTripsWrittenMoment)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 2.0, 2.0);
    const NodeMetaData m = sampleMoment();
    const grid_map::Position p = tmap::cellPos(lat, 3, -2);

    tmap::addToLayer(g, "N", p, m.N);
    tmap::addToLayer(g, "sx", p, m.sx);   tmap::addToLayer(g, "sy", p, m.sy);   tmap::addToLayer(g, "sz", p, m.sz);
    tmap::addToLayer(g, "sx2", p, m.sx2); tmap::addToLayer(g, "sy2", p, m.sy2); tmap::addToLayer(g, "sz2", p, m.sz2);
    tmap::addToLayer(g, "sxy", p, m.sxy); tmap::addToLayer(g, "sxz", p, m.sxz); tmap::addToLayer(g, "syz", p, m.syz);

    NodeMetaData out;
    ASSERT_TRUE(tmap::readCellMoment(g, lat, 3, -2, out));
    EXPECT_EQ(out.N, m.N);
    EXPECT_DOUBLE_EQ(out.sx, m.sx);   EXPECT_DOUBLE_EQ(out.sy, m.sy);   EXPECT_DOUBLE_EQ(out.sz, m.sz);
    EXPECT_DOUBLE_EQ(out.sx2, m.sx2); EXPECT_DOUBLE_EQ(out.sy2, m.sy2); EXPECT_DOUBLE_EQ(out.sz2, m.sz2);
    EXPECT_DOUBLE_EQ(out.sxy, m.sxy); EXPECT_DOUBLE_EQ(out.sxz, m.sxz); EXPECT_DOUBLE_EQ(out.syz, m.syz);
}

TEST(Helpers, ReadCellMomentFalseOnUnobservedCell)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    NodeMetaData out;
    EXPECT_FALSE(tmap::readCellMoment(g, lat, 0, 0, out));  // N is NaN
}

TEST(Helpers, ReadCellMomentFalseOutsideGrid)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    NodeMetaData out;
    // Cell far outside a +/-1 m grid.
    EXPECT_FALSE(tmap::readCellMoment(g, lat, 100000, 100000, out));
}

// --- allOccupiedKeys ---------------------------------------------------------

TEST(Helpers, AllOccupiedKeysReturnsOnlyObservedCells)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 2.0, 2.0);
    // Occupy two cells (N>=1), leave the rest NaN.
    tmap::addToLayer(g, "N", tmap::cellPos(lat, 1, 1), 5.0);
    tmap::addToLayer(g, "N", tmap::cellPos(lat, -3, 2), 2.0);

    std::vector<std::uint64_t> keys = tmap::allOccupiedKeys(g, lat);
    std::sort(keys.begin(), keys.end());
    std::vector<std::uint64_t> expected = {Lattice::key(1, 1), Lattice::key(-3, 2)};
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(keys, expected);
}

TEST(Helpers, AllOccupiedKeysIgnoresZeroAndNaNCells)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    g.atPosition("N", tmap::cellPos(lat, 0, 0)) = 0.f;  // explicitly zero -> not occupied
    EXPECT_TRUE(tmap::allOccupiedKeys(g, lat).empty());
}

// --- dilate ------------------------------------------------------------------

TEST(Helpers, DilateZeroDeltaIsIdentity)
{
    std::unordered_set<std::uint64_t> in = {Lattice::key(0, 0), Lattice::key(5, -3)};
    auto out = tmap::dilate(in, 0);
    EXPECT_EQ(out, in);
}

TEST(Helpers, DilateExpandsSquareWindow)
{
    std::unordered_set<std::uint64_t> in = {Lattice::key(0, 0)};
    auto out = tmap::dilate(in, 1);
    // (2*1+1)^2 = 9 cells around the origin.
    EXPECT_EQ(out.size(), 9u);
    for (int di = -1; di <= 1; ++di)
        for (int dj = -1; dj <= 1; ++dj)
            EXPECT_EQ(out.count(Lattice::key(di, dj)), 1u);
}

TEST(Helpers, DilateMergesOverlappingWindows)
{
    // Two adjacent centres, delta 1: windows overlap, so fewer than 2*9 = 18 cells.
    std::unordered_set<std::uint64_t> in = {Lattice::key(0, 0), Lattice::key(1, 0)};
    auto out = tmap::dilate(in, 1);
    // Union of {-1..1}x{-1..1} and {0..2}x{-1..1} = x in [-1..2] (4) * y in [-1..1] (3) = 12.
    EXPECT_EQ(out.size(), 12u);
}

// --- growGridToInclude -------------------------------------------------------

TEST(Helpers, GrowGridIsNoOpWhenBoundsAlreadyFit)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 5.0, 5.0);
    const grid_map::Length before = g.getLength();
    tmap::growGridToInclude(g, momentLayers(), lat, 0.25, /*extend=*/10.0,
                            -1.0, 1.0, -1.0, 1.0);  // well inside
    EXPECT_NEAR(g.getLength().x(), before.x(), 1e-9);
    EXPECT_NEAR(g.getLength().y(), before.y(), 1e-9);
}

TEST(Helpers, GrowGridEnlargesToCoverBounds)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Length before = g.getLength();
    // Demand a point at x=20 (way outside the +/-1 m grid).
    tmap::growGridToInclude(g, momentLayers(), lat, 0.25, /*extend=*/30.0,
                            20.0, 20.0, 0.0, 0.0);
    EXPECT_GT(g.getLength().x(), before.x());
    EXPECT_TRUE(g.isInside(grid_map::Position(20.0, 0.0)));
    // Origin stays pinned to the lattice (absolute ids preserved).
    EXPECT_NEAR(g.getPosition().x(), 0.0, 1e-9);
    EXPECT_NEAR(g.getPosition().y(), 0.0, 1e-9);
}

TEST(Helpers, GrowGridPreservesExistingCells)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Position p = tmap::cellPos(lat, 2, 1);  // inside the small grid
    tmap::addToLayer(g, "N", p, 7.0);

    tmap::growGridToInclude(g, momentLayers(), lat, 0.25, 30.0, 20.0, 20.0, 0.0, 0.0);
    // The previously-written cell must survive the resize at the SAME absolute id.
    EXPECT_TRUE(g.isInside(p));
    EXPECT_FLOAT_EQ(g.atPosition("N", p), 7.0f);
    NodeMetaData out;
    ASSERT_TRUE(tmap::readCellMoment(g, lat, 2, 1, out));
    EXPECT_EQ(out.N, 7u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
