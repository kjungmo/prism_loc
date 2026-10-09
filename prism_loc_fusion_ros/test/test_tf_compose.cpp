#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion_ros/tf_util.hpp"
using namespace prism_loc_fusion_ros;
TEST(TfCompose, MapOdomRoundTrip) {
  Eigen::Isometry3d map_base = Eigen::Isometry3d::Identity();
  map_base.translate(Eigen::Vector3d(2, -1, 0.5));
  map_base.rotate(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
  odom_base.translate(Eigen::Vector3d(5, 5, 0));
  odom_base.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d map_odom = computeMapToOdom(map_base, odom_base);
  EXPECT_TRUE((map_odom * odom_base).isApprox(map_base, 1e-9));
}

// A robot heading +90 deg (yaw) that drives forward moves along world +y; its
// Odometry twist must read +x (forward) in base_link, not +y.
TEST(TfCompose, OdometryTwistIsInTheBodyFrame) {
  const Eigen::Quaterniond q_wb(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d v_b = bodyVelocity(q_wb, Eigen::Vector3d(0.0, 1.5, 0.0));
  EXPECT_NEAR(v_b.x(), 1.5, 1e-12);
  EXPECT_NEAR(v_b.y(), 0.0, 1e-12);
  EXPECT_NEAR(v_b.z(), 0.0, 1e-12);
  // Pitched 30 deg nose-down while moving horizontally: part of the speed is along body z.
  const Eigen::Quaterniond q_p(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d v_p = bodyVelocity(q_p, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(v_p.x(), std::cos(M_PI / 6), 1e-12);
  EXPECT_NEAR(v_p.z(), std::sin(M_PI / 6), 1e-12);
  EXPECT_NEAR((q_p * v_p - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
}
