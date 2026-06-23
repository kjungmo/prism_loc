#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/types.hpp"
using namespace prism_loc_core;

// The node estimates T_map_base and must broadcast T_map_odom such that
// T_map_odom (+) T_odom_base == T_map_base.
TEST(TfMath, MapOdomComposition) {
  Pose2D base_in_map{2.0, -1.0, 0.7};
  Pose2D odom_base{5.0, 5.0, 0.2};
  Pose2D map_odom = compose(base_in_map, inverse(odom_base));
  Pose2D recovered = compose(map_odom, odom_base);
  EXPECT_NEAR(recovered.x, base_in_map.x, 1e-9);
  EXPECT_NEAR(recovered.y, base_in_map.y, 1e-9);
  EXPECT_NEAR(normalizeAngle(recovered.yaw - base_in_map.yaw), 0.0, 1e-9);
}
