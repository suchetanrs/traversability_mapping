/* Unit tests for the KeyFrame class (KeyFrame.hpp / KeyFrame.cpp).
 *
 * KeyFrame is deliberately PCL-free (Eigen only), so these build and run without
 * ROS / PCL. They cover: construction + accessors, the pose state machine
 * (setPose / getPendingPose / getPose), re-binning into cell-local moments, and
 * the retained-cloud lifecycle.
 */
#include <gtest/gtest.h>
#include <traversability_mapping/KeyFrame.hpp>
#include <traversability_mapping/Moments.hpp>

#include <vector>

using traversability_mapping::KeyFrame;
using traversability_mapping::Lattice;
using traversability_mapping::NodeMetaData;

namespace
{
    std::vector<Eigen::Vector3f> cloud(std::initializer_list<Eigen::Vector3f> pts)
    {
        return std::vector<Eigen::Vector3f>(pts);
    }

    Eigen::Affine3f translation(float x, float y, float z)
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() = Eigen::Vector3f(x, y, z);
        return t;
    }

    // A keyframe with a known pose and a small base-frame cloud.
    KeyFrame makeKF(std::uint64_t id = 7, double ts = 1.5,
                    const Eigen::Affine3f &pose = Eigen::Affine3f::Identity(),
                    std::vector<Eigen::Vector3f> pts = {{1.f, 2.f, 3.f}},
                    std::uint64_t mapID = 0)
    {
        return KeyFrame(id, ts, pose, std::move(pts), mapID);
    }
}  // namespace

// --- Construction + immutable accessors -------------------------------------

TEST(KeyFrame, PrimaryConstructorStoresFields)
{
    Eigen::Affine3f pose = translation(5.f, -1.f, 0.5f);
    KeyFrame kf(42, 12.25, pose, cloud({{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}}), 3);

    EXPECT_EQ(kf.getKfID(), 42u);
    EXPECT_DOUBLE_EQ(kf.getTimestamp(), 12.25);
    EXPECT_EQ(kf.getParentMapID(), 3u);
    EXPECT_TRUE(kf.getPose().matrix().isApprox(pose.matrix()));
    EXPECT_TRUE(kf.hasCloud());
    EXPECT_EQ(kf.cloudBase().size(), 2u);
    EXPECT_TRUE(kf.empty());          // nothing binned yet
}

TEST(KeyFrame, ConvenienceConstructorDefaultsTimestampAndMap)
{
    Eigen::Affine3f pose = translation(1.f, 2.f, 3.f);
    KeyFrame kf(9, pose, cloud({{0.f, 0.f, 0.f}}));
    EXPECT_EQ(kf.getKfID(), 9u);
    EXPECT_DOUBLE_EQ(kf.getTimestamp(), 0.0);
    EXPECT_EQ(kf.getParentMapID(), 0u);
    EXPECT_TRUE(kf.getPose().matrix().isApprox(pose.matrix()));
}

TEST(KeyFrame, SetMapUpdatesParentID)
{
    KeyFrame kf = makeKF();
    EXPECT_EQ(kf.getParentMapID(), 0u);
    kf.setMap(5);
    EXPECT_EQ(kf.getParentMapID(), 5u);
}

// --- Pose state machine ------------------------------------------------------

TEST(KeyFrame, FreshKeyFrameHasNoPendingPose)
{
    // Construction sets the pose but must NOT raise the pending flag, otherwise a
    // keyframe would bin at its placeholder pose before any real pose arrives.
    KeyFrame kf = makeKF();
    Eigen::Affine3f sentinel = translation(99.f, 99.f, 99.f);
    Eigen::Affine3f out = sentinel;
    EXPECT_FALSE(kf.getPendingPose(out));
    EXPECT_TRUE(out.matrix().isApprox(sentinel.matrix()));  // out left untouched
}

TEST(KeyFrame, SetPoseRaisesPendingAndGetPendingConsumesOnce)
{
    KeyFrame kf = makeKF();
    Eigen::Affine3f p = translation(2.f, -3.f, 4.f);
    kf.setPose(p);

    Eigen::Affine3f out;
    ASSERT_TRUE(kf.getPendingPose(out));
    EXPECT_TRUE(out.matrix().isApprox(p.matrix()));

    // The flag is consumed: a second take returns false.
    Eigen::Affine3f out2;
    EXPECT_FALSE(kf.getPendingPose(out2));
}

TEST(KeyFrame, SetPoseLatestWins)
{
    KeyFrame kf = makeKF();
    kf.setPose(translation(1.f, 1.f, 1.f));
    Eigen::Affine3f p2 = translation(8.f, 9.f, 10.f);
    kf.setPose(p2);  // overwrites the unconsumed pose

    Eigen::Affine3f out;
    ASSERT_TRUE(kf.getPendingPose(out));
    EXPECT_TRUE(out.matrix().isApprox(p2.matrix()));
}

TEST(KeyFrame, GetPoseReflectsLatestEvenAfterConsume)
{
    KeyFrame kf = makeKF();
    Eigen::Affine3f p = translation(7.f, 0.f, -2.f);
    kf.setPose(p);

    Eigen::Affine3f out;
    ASSERT_TRUE(kf.getPendingPose(out));      // consume the pending flag
    // getPose still returns the value; only the pending flag was cleared.
    EXPECT_TRUE(kf.getPose().matrix().isApprox(p.matrix()));
}

// --- rebin -------------------------------------------------------------------

TEST(KeyFrame, RebinBinsCloudIntoCellLocalMoments)
{
    Lattice lat(0.0, 0.0, 0.25);
    // One point at map (1.04, -0.36, 2.0) under identity pose.
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(), {{1.04f, -0.36f, 2.0f}});
    kf.rebin(lat, Eigen::Affine3f::Identity());

    ASSERT_EQ(kf.partials().size(), 1u);
    // round(1.04/0.25)=4, round(-0.36/0.25)=-1
    const std::uint64_t key = Lattice::key(4, -1);
    ASSERT_EQ(kf.partials().count(key), 1u);

    const NodeMetaData &m = kf.partials().at(key);
    EXPECT_EQ(m.N, 1u);
    // Stored cell-locally about the cell centre (1.0, -0.25), z about 0.
    Eigen::Vector3d bary = m.barycenter();
    EXPECT_NEAR(bary.x(), 0.04, 1e-4);
    EXPECT_NEAR(bary.y(), -0.11, 1e-4);
    EXPECT_NEAR(bary.z(), 2.0, 1e-4);
    EXPECT_FALSE(kf.empty());
}

TEST(KeyFrame, RebinAppliesPoseTransform)
{
    Lattice lat(0.0, 0.0, 0.25);
    // Base point (1,0,0); a +90 deg yaw about z maps it to (0,1,0).
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(), {{1.f, 0.f, 0.f}});
    Eigen::Affine3f yaw90 = Eigen::Affine3f::Identity();
    yaw90.linear() = Eigen::AngleAxisf(M_PI / 2.f, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    kf.rebin(lat, yaw90);
    ASSERT_EQ(kf.partials().size(), 1u);
    // map (0,1,0): round(0)=0, round(1/0.25)=4
    EXPECT_EQ(kf.partials().count(Lattice::key(0, 4)), 1u);
}

TEST(KeyFrame, RebinUsesPassedPoseNotLatestPose)
{
    // Crucial post-refactor invariant: rebin works on the pose ARGUMENT, never the
    // internally-stored latestPose_ (which a concurrent setPose could be changing).
    Lattice lat(0.0, 0.0, 0.25);
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(), {{1.04f, -0.36f, 2.0f}});
    kf.setPose(translation(10.f, 0.f, 0.f));  // latestPose_ now far away, pending raised

    kf.rebin(lat, Eigen::Affine3f::Identity());  // but bin at identity
    ASSERT_EQ(kf.partials().size(), 1u);
    EXPECT_EQ(kf.partials().count(Lattice::key(4, -1)), 1u);    // identity cell
    EXPECT_EQ(kf.partials().count(Lattice::key(44, -1)), 0u);   // NOT the translated cell
}

TEST(KeyFrame, RebinReplacesPreviousPartials)
{
    Lattice lat(0.0, 0.0, 0.25);
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(), {{1.04f, -0.36f, 2.0f}});

    kf.rebin(lat, Eigen::Affine3f::Identity());
    ASSERT_EQ(kf.partials().count(Lattice::key(4, -1)), 1u);

    // Re-bin at a translated pose: old cell must be gone, only the new one remains.
    kf.rebin(lat, translation(10.f, 0.f, 0.f));
    EXPECT_EQ(kf.partials().size(), 1u);
    EXPECT_EQ(kf.partials().count(Lattice::key(4, -1)), 0u);
    EXPECT_EQ(kf.partials().count(Lattice::key(44, -1)), 1u);
}

TEST(KeyFrame, RebinAccumulatesPointsInSameCell)
{
    Lattice lat(0.0, 0.0, 0.25);
    // Two points that both round to cell (0,0).
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(),
                         {{0.02f, -0.03f, 1.0f}, {-0.01f, 0.04f, 1.2f}});
    kf.rebin(lat, Eigen::Affine3f::Identity());
    ASSERT_EQ(kf.partials().size(), 1u);
    EXPECT_EQ(kf.partials().at(Lattice::key(0, 0)).N, 2u);
}

TEST(KeyFrame, RebinSeparatesPointsInDifferentCells)
{
    Lattice lat(0.0, 0.0, 0.25);
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(),
                         {{0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}});
    kf.rebin(lat, Eigen::Affine3f::Identity());
    EXPECT_EQ(kf.partials().size(), 2u);
    EXPECT_EQ(kf.partials().count(Lattice::key(0, 0)), 1u);
    EXPECT_EQ(kf.partials().count(Lattice::key(20, 0)), 1u);
}

TEST(KeyFrame, RebinEmptyCloudYieldsNoPartials)
{
    Lattice lat(0.0, 0.0, 0.25);
    KeyFrame kf(1, 0.0, Eigen::Affine3f::Identity(), {}, 0);
    EXPECT_FALSE(kf.hasCloud());
    kf.rebin(lat, Eigen::Affine3f::Identity());
    EXPECT_TRUE(kf.empty());
    EXPECT_EQ(kf.partials().size(), 0u);
}

// --- partials accessor ------------------------------------------------------

TEST(KeyFrame, PartialsMutableAccessorReflectsInConst)
{
    KeyFrame kf = makeKF();
    kf.partials()[Lattice::key(2, 2)].insert(0.0, 0.0, 0.0);
    const KeyFrame &ckf = kf;
    EXPECT_EQ(ckf.partials().size(), 1u);
    EXPECT_EQ(ckf.partials().at(Lattice::key(2, 2)).N, 1u);
    EXPECT_FALSE(ckf.empty());
}

// --- retained-cloud lifecycle -----------------------------------------------

TEST(KeyFrame, CloudBaseReturnsStoredPoints)
{
    KeyFrame kf = makeKF(1, 0.0, Eigen::Affine3f::Identity(),
                         {{1.f, 2.f, 3.f}, {4.f, 5.f, 6.f}});
    ASSERT_EQ(kf.cloudBase().size(), 2u);
    EXPECT_TRUE(kf.cloudBase()[0].isApprox(Eigen::Vector3f(1.f, 2.f, 3.f)));
    EXPECT_TRUE(kf.cloudBase()[1].isApprox(Eigen::Vector3f(4.f, 5.f, 6.f)));
}

TEST(KeyFrame, DropCloudClearsRetainedCloud)
{
    KeyFrame kf = makeKF();
    ASSERT_TRUE(kf.hasCloud());
    kf.dropCloud();
    EXPECT_FALSE(kf.hasCloud());
    EXPECT_TRUE(kf.cloudBase().empty());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
