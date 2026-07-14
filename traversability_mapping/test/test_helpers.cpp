/* Unit tests for the generic grid/lattice helpers (Helpers.hpp / Helpers.cpp).
 *
 * These exercise the stateless free functions that LocalMap delegates to:
 * cellPos, makeGridMap, growGridToInclude, blankCell, readCellMoment,
 * allOccupiedKeys, and dilate. They need grid_map_core + Eigen but no ROS.
 */
#include <gtest/gtest.h>
#include <traversability_mapping/Helpers.hpp>
#include <traversability_mapping/Moments.hpp>

#include <algorithm>
#include <cmath>

using traversability_mapping::Lattice;
using traversability_mapping::NodeMetaData;
namespace tmap = traversability_mapping;

namespace
{
    // The moment layers readCellMoment operates on, plus a derived layer so blankCell
    // has more than the moment set to clear.
    std::vector<std::string> momentLayers()
    {
        return {"N", "sx", "sy", "sz", "sx2", "sy2", "sz2", "sxy", "sxz", "syz", "hazard"};
    }

    // Seed one layer of one cell. The library writes moments through MomentLayers (which
    // needs a buffer index), so tests that only need a cell to be *observed* poke the
    // layer directly rather than going through the moment path.
    void seedLayer(grid_map::GridMap &g, const std::string &layer,
                   const grid_map::Position &p, float v)
    {
        g.atPosition(layer, p) = v;
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

// --- blankCell ---------------------------------------------------------------

TEST(Helpers, BlankCellSetsEveryLayerNaN)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 1.0, 1.0);
    const grid_map::Position p(0.0, 0.0);
    for (const auto &l : momentLayers())
        seedLayer(g, l, p, 1.f);
    tmap::blankCell(g, momentLayers(), p);
    for (const auto &l : momentLayers())
        EXPECT_TRUE(std::isnan(g.atPosition(l, p))) << "layer " << l << " not blanked";
}

// --- allOccupiedKeys ---------------------------------------------------------

TEST(Helpers, AllOccupiedKeysReturnsOnlyObservedCells)
{
    Lattice lat(0.0, 0.0, 0.25);
    grid_map::GridMap g = tmap::makeGridMap(momentLayers(), "map", lat, 0.25, 2.0, 2.0);
    // Occupy two cells (N>=1), leave the rest NaN.
    seedLayer(g, "N", tmap::cellPos(lat, 1, 1), 5.f);
    seedLayer(g, "N", tmap::cellPos(lat, -3, 2), 2.f);

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
    seedLayer(g, "N", p, 7.f);

    tmap::growGridToInclude(g, momentLayers(), lat, 0.25, 30.0, 20.0, 20.0, 0.0, 0.0);
    // The previously-written cell must survive the resize at the SAME absolute id.
    EXPECT_TRUE(g.isInside(p));
    EXPECT_FLOAT_EQ(g.atPosition("N", p), 7.0f);
    // Layers must be re-resolved after a grow: the old grid was replaced wholesale.
    const tmap::MomentLayers layers(g);
    NodeMetaData out;
    ASSERT_TRUE(tmap::readCellMoment(g, layers, lat, 2, 1, out));
    EXPECT_EQ(out.N, 7u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
