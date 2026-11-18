#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Geometry>
#include "prism_loc/cloud_level.hpp"
using prism_loc::levelCloud;
using prism_loc::tiltRotation;
using prism_loc::tiltFromRotation;
using prism_loc::rotationFromQuaternion;

namespace {
// tf2 getRPY convention: R = Rz(yaw) Ry(pitch) Rx(roll)
Eigen::Matrix3d fullRotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}
Eigen::Matrix3d rz(double yaw) { return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
}  // namespace

TEST(CloudLevel, ZeroTiltIsTheIdentityAndLeavesPointsAlone) {
  EXPECT_TRUE(tiltRotation(0.0, 0.0).isApprox(Eigen::Matrix3d::Identity(), 0.0));
  std::vector<Eigen::Vector3d> pts{{1, 2, 3}, {-4, 0.5, 7}};
  const auto before = pts;
  levelCloud(0.0, 0.0, pts);
  EXPECT_EQ(pts, before);  // exactly: a level mount must change nothing
}

// A flat floor 1.2 m under a sensor pitched 0.1 rad about its y axis (yaw 0.7 on top of it):
// the points in the sensor frame lie on a tilted plane; levelled, they have one z.
TEST(CloudLevel, FloorSeenByAPitchedSensorBecomesLevel) {
  const double roll = 0.0, pitch = 0.1, yaw = 0.7;
  const Eigen::Matrix3d R = fullRotation(roll, pitch, yaw);
  std::vector<Eigen::Vector3d> sensor;
  for (double x = -5; x <= 5; x += 0.5)
    for (double y = -5; y <= 5; y += 0.5) {
      const Eigen::Vector3d in_base_without_t(x, y, -1.2);  // the floor, in the base frame
      sensor.push_back(R.transpose() * in_base_without_t);   // what the sensor sees
    }
  // seen raw, the floor is not level
  double zmin = 1e9, zmax = -1e9;
  for (const auto& p : sensor) { zmin = std::min(zmin, p.z()); zmax = std::max(zmax, p.z()); }
  EXPECT_GT(zmax - zmin, 0.5);
  levelCloud(roll, pitch, sensor);
  // The model then rotates by the yaw (through sensor_in_base): the floor is level again.
  for (const auto& p : sensor) EXPECT_NEAR((rz(yaw) * p).z(), -1.2, 1e-12);
  for (const auto& p : sensor) EXPECT_NEAR(p.z(), -1.2, 1e-12);  // z is not touched by Rz
}

// Rz(yaw) * R_tilt is the full rotation: applying the tilt and then the yaw the model
// applies equals the full rotation; a roll together with a pitch too.
TEST(CloudLevel, TiltThenYawEqualsTheFullRotation) {
  for (double roll : {-0.3, 0.0, 0.05, 0.4})
    for (double pitch : {-0.5, -0.1, 0.0, 0.1, 0.9})
      for (double yaw : {-3.0, 0.0, 0.7, 2.5}) {
        const Eigen::Matrix3d full = fullRotation(roll, pitch, yaw);
        EXPECT_TRUE((rz(yaw) * tiltRotation(roll, pitch)).isApprox(full, 1e-12));
        // the yaw removed from the full rotation leaves exactly the tilt
        EXPECT_TRUE((rz(-yaw) * full).isApprox(tiltRotation(roll, pitch), 1e-12));
        const Eigen::Vector3d p(0.3, -2.0, 1.5);
        Eigen::Vector3d q = p;
        std::vector<Eigen::Vector3d> v{q};
        levelCloud(roll, pitch, v);
        EXPECT_TRUE((rz(yaw) * v[0]).isApprox(full * p, 1e-12));
      }
}

TEST(CloudLevel, RotationIsOrthonormalAndFiniteNearGimbalLock) {
  const double half_pi = std::acos(-1.0) / 2.0;
  for (double pitch : {half_pi - 1e-3, half_pi - 1e-9, -half_pi + 1e-9, half_pi, -half_pi})
    for (double roll : {0.0, 0.3, -2.0}) {
      const Eigen::Matrix3d R = tiltRotation(roll, pitch);
      EXPECT_TRUE(R.allFinite());
      EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
      EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
      std::vector<Eigen::Vector3d> v{{1, 2, 3}};
      levelCloud(roll, pitch, v);
      EXPECT_TRUE(v[0].allFinite());
      EXPECT_NEAR(v[0].norm(), Eigen::Vector3d(1, 2, 3).norm(), 1e-12);
    }
}

TEST(CloudLevel, EmptyCloudIsFine) {
  std::vector<Eigen::Vector3d> none;
  levelCloud(0.1, 0.2, none);
  EXPECT_TRUE(none.empty());
}

namespace {
// the node's yaw: atan2(2(wz + xy), 1 - 2(y^2 + z^2)) of a unit quaternion
double quatYaw(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
}  // namespace

// For any rotation, Rz(yaw) * R_tilt is the rotation itself, with the yaw the node uses.
TEST(CloudLevel, TiltFromRotationComposesBackExactlyForRandomRotations) {
  std::mt19937 rng(11);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int i = 0; i < 500; ++i) {
    Eigen::Quaterniond q(n(rng), n(rng), n(rng), n(rng));
    q.normalize();
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const double yaw = quatYaw(q);
    const Eigen::Matrix3d T = tiltFromRotation(R, yaw);
    EXPECT_TRUE((Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T).isApprox(R, 1e-12));
    EXPECT_TRUE((T * T.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  }
}

// A mount with only roll and pitch (yaw 0): the tilt is Ry(pitch) Rx(roll), as tiltRotation says.
TEST(CloudLevel, TiltFromRotationAgreesWithRollPitchForOrdinaryMounts) {
  for (double roll : {-0.3, 0.0, 0.2})
    for (double pitch : {-0.5, 0.1, 0.9})
      for (double yaw : {-2.0, 0.0, 1.3}) {
        const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        EXPECT_TRUE(tiltFromRotation(q.toRotationMatrix(), quatYaw(q)).isApprox(tiltRotation(roll, pitch), 1e-12));
      }
}

// Pitch within a hair of +-90 degrees (gimbal lock): the composition still holds exactly.
TEST(CloudLevel, TiltFromRotationAtGimbalLock) {
  const double half_pi = std::acos(-1.0) / 2.0;
  for (double pitch : {half_pi - 1e-6, -half_pi + 1e-6, half_pi, -half_pi})
    for (double roll : {0.0, 0.4})
      for (double yaw : {0.0, 0.8}) {
        Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        q.normalize();
        const Eigen::Matrix3d R = q.toRotationMatrix();
        const double y = quatYaw(q);
        const Eigen::Matrix3d T = tiltFromRotation(R, y);
        EXPECT_TRUE(T.allFinite());
        EXPECT_TRUE((Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix() * T).isApprox(R, 1e-9));
        std::vector<Eigen::Vector3d> v{{1, 2, 3}};
        prism_loc::levelCloud(T, v);
        EXPECT_NEAR(v[0].norm(), std::sqrt(14.0), 1e-12);
      }
}

// A quaternion that is not of unit length (a drifting TF) is normalised; zero or NaN gives the identity.
TEST(CloudLevel, RotationFromQuaternionNormalises) {
  const Eigen::Quaterniond q = Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()) * Eigen::Quaterniond::Identity();
  const Eigen::Matrix3d R = rotationFromQuaternion(1.01 * q.x(), 1.01 * q.y(), 1.01 * q.z(), 1.01 * q.w());
  EXPECT_TRUE(R.isApprox(q.toRotationMatrix(), 1e-12));
  EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(rotationFromQuaternion(0, 0, 0, 0).isApprox(Eigen::Matrix3d::Identity(), 0.0));
  EXPECT_TRUE(rotationFromQuaternion(std::nan(""), 0, 0, 1).isApprox(Eigen::Matrix3d::Identity(), 0.0));
}
