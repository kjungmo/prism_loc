#include <gtest/gtest.h>
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
