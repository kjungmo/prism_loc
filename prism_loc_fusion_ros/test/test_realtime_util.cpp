#include <gtest/gtest.h>
#include <cmath>
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
