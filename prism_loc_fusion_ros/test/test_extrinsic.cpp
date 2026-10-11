#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include "prism_loc_fusion_ros/extrinsic.hpp"
using namespace prism_loc_fusion_ros;

namespace {
Eigen::Isometry3d pose(const Eigen::Vector3d& t, const Eigen::Matrix3d& R) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = t;
  T.linear() = R;
  return T;
}
Eigen::Matrix3d rotZ(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
Eigen::Matrix3d rotX(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d rotY(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
bool sameBits(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return std::memcmp(a.matrix().data(), b.matrix().data(), 16 * sizeof(double)) == 0;
}
bool sameBits(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::memcmp(a.data(), b.data(), 3 * sizeof(double)) == 0;
}
// Awkward values: negative zero, a denormal, values that do not survive x*1+y*0 rounding
// tricks if any arithmetic were applied to them with a non-identity partner.
const std::vector<Eigen::Vector3d> kVectors = {
    {0.0, -0.0, 9.81}, {1e-310, -3.3, 1e300}, {0.1, 0.2, 0.3}, {-12345.678, 1e-17, -0.0}};
}  // namespace

// A sensor whose frame is base_frame: every conversion returns its input bit for bit, so
// a robot that already feeds base_frame data sees exactly the values it saw before the
// mount was taken into account.
TEST(Extrinsic, IdentityReturnsTheInputBitForBit) {
  const Extrinsic by_name;                                             // frame == base_frame
  const Extrinsic by_tf = makeExtrinsic(Eigen::Isometry3d::Identity());  // identity found on TF
  EXPECT_TRUE(by_name.identity);
  EXPECT_TRUE(by_tf.identity);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()));
  for (const Extrinsic& e : {by_name, by_tf}) {
    for (const auto& v : kVectors) {
      const Eigen::Isometry3d T = pose(v, q.toRotationMatrix());
      EXPECT_TRUE(sameBits(sensorPoseFromBasePose(e, T), T));
      EXPECT_TRUE(sameBits(basePoseFromSensorPose(e, T), T));
      EXPECT_TRUE(sameBits(vectorInBase(e, v), v));
      EXPECT_TRUE(sameBits(basePositionFromAntenna(e, q, v), v));
    }
    // the attitude plays no part for the identity: not even an invalid one touches the fix
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d fix(1.5, -2.5, 3.5);
    EXPECT_TRUE(sameBits(basePositionFromAntenna(e, Eigen::Quaterniond(nan, nan, nan, nan), fix), fix));
  }
}

// "Identity" means exactly the identity matrix: the smallest offset or rotation is a mount.
TEST(Extrinsic, AnythingButTheExactIdentityIsAMount) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation().z() = 1e-12;
  EXPECT_FALSE(makeExtrinsic(T).identity);
  EXPECT_FALSE(makeExtrinsic(pose(Eigen::Vector3d::Zero(), rotZ(1e-6))).identity);
  EXPECT_FALSE(makeExtrinsic(pose(Eigen::Vector3d(0.2, 0.0, 0.5), Eigen::Matrix3d::Identity())).identity);
  // and it is then applied, not skipped
  const Extrinsic e = makeExtrinsic(T);
  EXPECT_DOUBLE_EQ(basePoseFromSensorPose(e, Eigen::Isometry3d::Identity()).translation().z(), -1e-12);
}

// Hand-computed case. base_link is at (2, 3, 1) in the map, heading +90 deg (its x axis
// is map +y). The lidar sits 0.2 m ahead of and 0.5 m above base_link, turned -90 deg
// about z. Its origin in the map is therefore (2, 3.2, 1.5) and its axes are the map's.
TEST(Extrinsic, LidarPoseToBasePoseClosedForm) {
  const Extrinsic e = makeExtrinsic(pose(Eigen::Vector3d(0.2, 0.0, 0.5), rotZ(-M_PI / 2)));
  const Eigen::Isometry3d map_sensor = pose(Eigen::Vector3d(2.0, 3.2, 1.5), Eigen::Matrix3d::Identity());
  const Eigen::Isometry3d map_base = basePoseFromSensorPose(e, map_sensor);
  EXPECT_NEAR(map_base.translation().x(), 2.0, 1e-12);
  EXPECT_NEAR(map_base.translation().y(), 3.0, 1e-12);
  EXPECT_NEAR(map_base.translation().z(), 1.0, 1e-12);
  Eigen::Matrix3d expect;  // Rz(+90 deg)
  expect << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  EXPECT_LT((map_base.linear() - expect).norm(), 1e-12);
  // and the other direction: the registration guess for that base pose is the lidar pose
  const Eigen::Isometry3d guess = sensorPoseFromBasePose(e, pose(Eigen::Vector3d(2.0, 3.0, 1.0), expect));
  EXPECT_LT((guess.translation() - Eigen::Vector3d(2.0, 3.2, 1.5)).norm(), 1e-12);
  EXPECT_LT((guess.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

// General 6-DoF mount and pose against the formula written out with plain matrices:
// R_mb = R_ms R_bs^T,  p_mb = p_ms - R_mb t_bs   (T_map_base = T_map_sensor * inverse(T_base_sensor)).
TEST(Extrinsic, LidarPoseToBasePoseGeneralSixDof) {
  const Eigen::Matrix3d R_bs = rotZ(0.4) * rotY(-0.25) * rotX(0.6);
  const Eigen::Vector3d t_bs(0.31, -0.12, 0.74);
  const Eigen::Matrix3d R_ms = rotZ(-2.1) * rotY(0.15) * rotX(-0.35);
  const Eigen::Vector3d p_ms(12.5, -7.25, 1.9);
  const Extrinsic e = makeExtrinsic(pose(t_bs, R_bs));
  const Eigen::Isometry3d map_base = basePoseFromSensorPose(e, pose(p_ms, R_ms));
  const Eigen::Matrix3d R_mb = R_ms * R_bs.transpose();
  EXPECT_LT((map_base.linear() - R_mb).norm(), 1e-12);
  EXPECT_LT((map_base.translation() - (p_ms - R_mb * t_bs)).norm(), 1e-12);
  // the mount really matters here: the two poses are 0.8 m apart
  EXPECT_NEAR((map_base.translation() - p_ms).norm(), t_bs.norm(), 1e-12);
  EXPECT_GT(t_bs.norm(), 0.8);
  // round trip, and the forward formula R_ms = R_mb R_bs, p_ms = p_mb + R_mb t_bs
  const Eigen::Isometry3d back = sensorPoseFromBasePose(e, map_base);
  EXPECT_LT((back.linear() - R_ms).norm(), 1e-12);
  EXPECT_LT((back.translation() - p_ms).norm(), 1e-12);
  // the wrong order, inverse(T_base_sensor) * T_map_sensor, is a different pose
  const Eigen::Isometry3d wrong = pose(t_bs, R_bs).inverse() * pose(p_ms, R_ms);
  EXPECT_GT((wrong.translation() - map_base.translation()).norm(), 1.0);
}

// IMU turned +90 deg about z on the robot: its x axis is base_link's +y. Upside down
// (180 deg about x): gravity reads -9.81 on its z. Only the rotation is applied.
TEST(Extrinsic, ImuVectorsAreRotatedIntoBaseAndTheLeverArmIsIgnored) {
  const Extrinsic yawed = makeExtrinsic(pose(Eigen::Vector3d(5.0, -4.0, 3.0), rotZ(M_PI / 2)));
  const Eigen::Vector3d v = vectorInBase(yawed, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(v.x(), 0.0, 1e-12);
  EXPECT_NEAR(v.y(), 1.0, 1e-12);
  EXPECT_NEAR(v.z(), 0.0, 1e-12);
  const Extrinsic flipped = makeExtrinsic(pose(Eigen::Vector3d::Zero(), rotX(M_PI)));
  const Eigen::Vector3d f = vectorInBase(flipped, Eigen::Vector3d(0.5, 0.25, -9.81));
  EXPECT_NEAR(f.x(), 0.5, 1e-12);
  EXPECT_NEAR(f.y(), -0.25, 1e-12);
  EXPECT_NEAR(f.z(), 9.81, 1e-12);
}

// Antenna 1 m ahead of and 2 m above base_link; the robot heads +90 deg (map +y). A fix
// at (10, 20, 5) puts base_link at (10, 19, 3). The antenna frame's own rotation is unused.
TEST(Extrinsic, GnssFixIsMovedFromTheAntennaToBase) {
  const Eigen::Quaterniond q_map_base(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  for (const Eigen::Matrix3d& R : {Eigen::Matrix3d(Eigen::Matrix3d::Identity()), rotY(0.9)}) {
    const Extrinsic e = makeExtrinsic(pose(Eigen::Vector3d(1.0, 0.0, 2.0), R));
    const Eigen::Vector3d p = basePositionFromAntenna(e, q_map_base, Eigen::Vector3d(10.0, 20.0, 5.0));
    EXPECT_NEAR(p.x(), 10.0, 1e-12);
    EXPECT_NEAR(p.y(), 19.0, 1e-12);
    EXPECT_NEAR(p.z(), 3.0, 1e-12);
  }
}
