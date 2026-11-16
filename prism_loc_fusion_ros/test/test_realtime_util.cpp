#include <gtest/gtest.h>
#include <cmath>
#include <deque>
#include "prism_loc_fusion_ros/realtime_util.hpp"
using namespace prism_loc_fusion_ros;

static Eigen::Isometry3d pose(const Eigen::Vector3d& t, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = t;
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

TEST(MotionCompensation, IdentityWhenTheStateDidNotMove) {
  const Eigen::Isometry3d g = pose({3, -2, 0.5}, 0.7);
  const Eigen::Isometry3d z = pose({3.1, -2.05, 0.5}, 0.72);
  EXPECT_TRUE(compensateForMotion(z, g, g).matrix().isApprox(z.matrix(), 1e-12));
}

TEST(MotionCompensation, PureTranslationIsAddedInTheMapFrame) {
  // Heading 90 deg: the robot moved 1 m along its own x, that is +y in the map.
  const Eigen::Isometry3d g = pose({10, 0, 0}, M_PI / 2);
  const Eigen::Isometry3d n = pose({10, 1, 0}, M_PI / 2);
  const Eigen::Isometry3d z = pose({10.2, 0, 0}, M_PI / 2);  // NDT: 0.2 m ahead of the guess in x
  const Eigen::Isometry3d out = compensateForMotion(z, g, n);
  EXPECT_NEAR(out.translation().x(), 10.2, 1e-12);
  EXPECT_NEAR(out.translation().y(), 1.0, 1e-12);
}

TEST(MotionCompensation, PureRotationAboutTheBodyIsAddedToTheNdtPose) {
  const Eigen::Isometry3d g = pose({1, 2, 0}, 0.0);
  const Eigen::Isometry3d n = pose({1, 2, 0}, 0.3);  // turned 0.3 rad while aligning
  const Eigen::Isometry3d z = pose({1.5, 2.0, 0}, 0.1);
  const Eigen::Isometry3d out = compensateForMotion(z, g, n);
  EXPECT_NEAR(out.translation().x(), 1.5, 1e-12);
  EXPECT_NEAR(out.translation().y(), 2.0, 1e-12);
  const Eigen::Matrix3d R = out.linear();
  EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), 0.4, 1e-12);
}

TEST(MotionCompensation, AppliesTheMotionOfTheStateNotTheCorrection) {
  // z differs from g by a correction c: out = n * (g^-1 z) in the body frame sense is NOT
  // what is wanted; the correction stays in the map frame: out = z * g^-1 * n.
  const Eigen::Isometry3d g = pose({0, 0, 0}, 0.0);
  const Eigen::Isometry3d n = pose({2, 0, 0}, 0.0);
  const Eigen::Isometry3d z = pose({0, 0.3, 0}, 0.0);  // NDT says: 0.3 m to the left of the guess
  const Eigen::Isometry3d out = compensateForMotion(z, g, n);
  EXPECT_NEAR(out.translation().x(), 2.0, 1e-12);
  EXPECT_NEAR(out.translation().y(), 0.3, 1e-12);
}

TEST(CloudStale, OlderThanMaxAgeBehindTheLatestImuIsStale) {
  EXPECT_TRUE(cloudIsStale(10.0, 10.6, 0.5));
  EXPECT_FALSE(cloudIsStale(10.0, 10.4, 0.5));
  EXPECT_FALSE(cloudIsStale(10.0, 10.5, 0.5));  // exactly the limit is kept
}

TEST(CloudStale, ACloudNewerThanTheImuIsNeverStale) {
  EXPECT_FALSE(cloudIsStale(11.0, 10.0, 0.5));
}

TEST(CloudStale, ZeroMaxAgeDisablesTheCheck) {
  EXPECT_FALSE(cloudIsStale(0.0, 1e6, 0.0));
  EXPECT_FALSE(cloudIsStale(10.0, 99.0, 0.0));
}

TEST(CloudStale, AnUnstampedCloudOrAnUnknownImuTimeIsKept) {
  EXPECT_FALSE(cloudIsStale(0.0, 100.0, 0.5));   // stamp 0: the driver sets none
  EXPECT_FALSE(cloudIsStale(10.0, 0.0, 0.5));    // no IMU stamp yet
  EXPECT_FALSE(cloudIsStale(std::nan(""), 10.0, 0.5));
}

// Order of the product: z * g^-1 * n and not n * g^-1 * z. g at the origin heading +y, n 1 m to
// the right of g's heading (at +x, same heading), z shifted along x with no turn. Right order:
// translation (5, -1, 0). Swapped order: (6, 0, 0).
TEST(MotionCompensation, OrderOfTheProductMatters) {
  const Eigen::Isometry3d g = pose({0, 0, 0}, M_PI / 2);
  const Eigen::Isometry3d n = pose({1, 0, 0}, M_PI / 2);
  const Eigen::Isometry3d z = pose({5, 0, 0}, 0.0);
  const Eigen::Isometry3d out = compensateForMotion(z, g, n);
  EXPECT_NEAR(out.translation().x(), 5.0, 1e-12);
  EXPECT_NEAR(out.translation().y(), -1.0, 1e-12);
  EXPECT_NEAR(out.translation().z(), 0.0, 1e-12);
  EXPECT_TRUE(out.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

static PoseSample sampleAt(double t, double x, double yaw, double dr_x = 0.0) {
  PoseSample s;
  s.t = t;
  s.p = Eigen::Vector3d(x, 0, 0);
  s.q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  s.dr_p = Eigen::Vector3d(dr_x, 0, 0);
  s.dr_q = Eigen::Quaterniond::Identity();
  return s;
}

TEST(PoseHistory, EmptyAndOutsideGiveNothing) {
  std::deque<PoseSample> h;
  PoseAtTime out;
  EXPECT_FALSE(interpolatePose(h, 1.0, out));
  recordPose(h, sampleAt(1.0, 0, 0), 2.0);
  recordPose(h, sampleAt(1.1, 1, 0), 2.0);
  EXPECT_FALSE(interpolatePose(h, 0.99, out));  // before the oldest
  EXPECT_FALSE(interpolatePose(h, 1.11, out));  // after the newest
  EXPECT_FALSE(interpolatePose(h, 0.0, out));   // an unstamped cloud
}

TEST(PoseHistory, ExactSampleAndInterpolation) {
  std::deque<PoseSample> h;
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  recordPose(h, sampleAt(1.2, 2.0, 0.4, 0.5), 2.0);
  PoseAtTime out;
  ASSERT_TRUE(interpolatePose(h, 1.0, out));
  EXPECT_NEAR(out.state.translation().x(), 0.0, 1e-12);
  ASSERT_TRUE(interpolatePose(h, 1.2, out));
  EXPECT_NEAR(out.state.translation().x(), 2.0, 1e-12);
  ASSERT_TRUE(interpolatePose(h, 1.05, out));  // a quarter of the way
  EXPECT_NEAR(out.state.translation().x(), 0.5, 1e-12);
  const Eigen::Matrix3d R = out.state.linear();
  EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), 0.1, 1e-9);
  EXPECT_NEAR(out.dr.translation().x(), 0.125, 1e-12);
}

TEST(PoseHistory, KeepsOnlyTheLastSecondsAndRestartsWhenTimeGoesBack) {
  std::deque<PoseSample> h;
  for (int i = 0; i <= 30; ++i) recordPose(h, sampleAt(0.1 * i, i, 0), 2.0);
  EXPECT_NEAR(h.front().t, 1.0, 1e-9);  // 3.0 - 2.0
  EXPECT_NEAR(h.back().t, 3.0, 1e-9);
  recordPose(h, sampleAt(0.5, 0, 0), 2.0);  // a bag loop: stamps jump back
  ASSERT_EQ(h.size(), 1u);
  EXPECT_NEAR(h.front().t, 0.5, 1e-12);
}

TEST(DeadReckoning, ChainsOnlyThePredictionIncrement) {
  const Eigen::Isometry3d d0 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d before = pose({10, 0, 0}, M_PI / 2);
  const Eigen::Isometry3d after = pose({10, 0.1, 0}, M_PI / 2);  // 0.1 m along the body x
  const Eigen::Isometry3d d1 = advanceDeadReckoning(d0, before, after);
  EXPECT_NEAR(d1.translation().x(), 0.1, 1e-12);
  EXPECT_NEAR(d1.translation().y(), 0.0, 1e-12);
  // A GNSS jump between two predicts is in `before`, not in the increment.
  const Eigen::Isometry3d jumped = pose({50, 20, 0}, 2.0);
  const Eigen::Isometry3d d2 = advanceDeadReckoning(d1, jumped, jumped * pose({0.1, 0, 0}, 0.0));
  EXPECT_NEAR(d2.translation().x(), 0.2, 1e-12);
}

TEST(NdtVerdict, ReasonsInOrder) {
  EXPECT_EQ(ndtVerdict(true, 0.01, 0.9, 0.06, 0.5), NdtVerdict::kAccept);
  EXPECT_EQ(ndtVerdict(false, 0.01, 0.9, 0.06, 0.5), NdtVerdict::kNotConverged);
  EXPECT_EQ(ndtVerdict(false, 9.0, 0.1, 0.06, 0.5), NdtVerdict::kNotConverged);  // first reason
  EXPECT_EQ(ndtVerdict(true, 0.07, 0.9, 0.06, 0.5), NdtVerdict::kFitness);
  EXPECT_EQ(ndtVerdict(true, 0.07, 0.1, 0.06, 0.5), NdtVerdict::kFitness);       // fitness before inliers
  EXPECT_EQ(ndtVerdict(true, 0.01, 0.49, 0.06, 0.5), NdtVerdict::kInliers);
  EXPECT_EQ(ndtVerdict(true, 0.06, 0.5, 0.06, 0.5), NdtVerdict::kAccept);        // limits are inclusive
  EXPECT_EQ(ndtVerdict(true, std::nan(""), 0.9, 0.06, 0.5), NdtVerdict::kFitness);
  EXPECT_EQ(ndtVerdict(true, 0.01, std::nan(""), 0.06, 0.5), NdtVerdict::kInliers);
  EXPECT_STREQ(ndtVerdictName(NdtVerdict::kInliers), "inliers");
}

TEST(AlignDiscard, DiscardsOnAnyGenerationChangeOrLostFilter) {
  EXPECT_FALSE(discardAlignResult(4, 4, true));
  EXPECT_TRUE(discardAlignResult(4, 5, true));
  EXPECT_TRUE(discardAlignResult(4, 4, false));
}

TEST(ClockOffset, SuspectedWhenCloudsAreDroppedAlthoughTheAlignIsFast) {
  EXPECT_TRUE(clockOffsetSuspected(true, 30.0, 0.5, 3));    // 30 ms align, 3 drops in a row
  EXPECT_TRUE(clockOffsetSuspected(false, 0.0, 0.5, 3));    // never aligned at all
  EXPECT_FALSE(clockOffsetSuspected(true, 30.0, 0.5, 2));   // not yet
  EXPECT_FALSE(clockOffsetSuspected(true, 400.0, 0.5, 9));  // a slow align explains the drops
}

TEST(CloudTime, TheOffsetMovesTheStampToTheTimeTheScanRepresents) {
  // A driver that stamps the start of a 0.1 s sweep: the registration of the unskewed scan is
  // the pose at about the middle of the sweep.
  EXPECT_DOUBLE_EQ(cloudTime(100.0, 0.05), 100.05);
  EXPECT_DOUBLE_EQ(cloudTime(100.0, 0.0), 100.0);
  EXPECT_DOUBLE_EQ(cloudTime(100.0, -0.05), 99.95);  // a driver that stamps the end
}

TEST(CloudTime, AnUnstampedCloudStaysUnstamped) {
  EXPECT_DOUBLE_EQ(cloudTime(0.0, 0.05), 0.0);
  EXPECT_FALSE(std::isfinite(cloudTime(std::nan(""), 0.05)));
}

TEST(PoseHistory, ThreeSamplesPickTheRightBracket) {
  std::deque<PoseSample> h;
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  recordPose(h, sampleAt(1.1, 1.0, 0.0, 0.0), 2.0);
  recordPose(h, sampleAt(1.2, 3.0, 0.0, 0.0), 2.0);
  PoseAtTime out;
  ASSERT_TRUE(interpolatePose(h, 1.1, out));  // the middle sample exactly
  EXPECT_NEAR(out.state.translation().x(), 1.0, 1e-12);
  ASSERT_TRUE(interpolatePose(h, 1.15, out));  // between the second and the third
  EXPECT_NEAR(out.state.translation().x(), 2.0, 1e-12);
  ASSERT_TRUE(interpolatePose(h, 1.05, out));  // between the first and the second
  EXPECT_NEAR(out.state.translation().x(), 0.5, 1e-12);
}

TEST(ClockOffset, NoAlignYetOrAFastAlignPointsAtTheClocks) {
  EXPECT_TRUE(clockOffsetSuspected(false, 400.0, 0.5, 3));   // nothing registered yet: not the align
  EXPECT_TRUE(clockOffsetSuspected(true, 249.0, 0.5, 3));    // under half of points_max_age
  EXPECT_FALSE(clockOffsetSuspected(true, 250.0, 0.5, 3));   // at the boundary: the align may explain it
  EXPECT_FALSE(clockOffsetSuspected(false, 0.0, 0.5, 2));    // not yet three in a row
}

TEST(GuessAt, InsideTheHistoryInterpolates) {
  std::deque<PoseSample> h;
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  recordPose(h, sampleAt(1.2, 2.0, 0.0, 0.0), 2.0);
  PoseAtTime out;
  EXPECT_EQ(guessAt(h, 1.1, out), GuessSource::Interpolated);
  EXPECT_NEAR(out.state.translation().x(), 1.0, 1e-12);
}

TEST(GuessAt, ALittleAfterTheNewestSampleUsesTheNewest) {
  // A cloud time a few ms after the newest IMU sample (offset, IMU delivery): the newest state.
  std::deque<PoseSample> h;
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  recordPose(h, sampleAt(1.2, 2.0, 0.0, 0.5), 2.0);
  PoseAtTime out;
  EXPECT_EQ(guessAt(h, 1.25, out), GuessSource::Newest);
  EXPECT_NEAR(out.state.translation().x(), 2.0, 1e-12);
  EXPECT_NEAR(out.dr.translation().x(), 0.5, 1e-12);
  EXPECT_EQ(guessAt(h, 1.31, out), GuessSource::Fallback);  // more than 0.1 s ahead
}

TEST(GuessAt, BeforeTheOldestSampleIsReported) {
  // The history was cleared (filter restart, attitude replaced, heading aligned) after the cloud
  // was taken: its motion since then cannot be compensated.
  std::deque<PoseSample> h;
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  PoseAtTime out;
  EXPECT_EQ(guessAt(h, 0.9, out), GuessSource::BeforeHistory);
}

TEST(GuessAt, UnstampedOrEmptyFallsBack) {
  std::deque<PoseSample> h;
  PoseAtTime out;
  EXPECT_EQ(guessAt(h, 1.0, out), GuessSource::Fallback);  // empty history
  recordPose(h, sampleAt(1.0, 0.0, 0.0, 0.0), 2.0);
  EXPECT_EQ(guessAt(h, 0.0, out), GuessSource::Fallback);  // unstamped cloud
  EXPECT_EQ(guessAt(h, std::nan(""), out), GuessSource::Fallback);
}
