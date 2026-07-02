#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc/localization_node.hpp"
using namespace prism_loc;

TEST(Adapter, OccupancyGridCopiesGeometry) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = 3; g.info.height = 2; g.info.resolution = 0.25;
  g.info.origin.position.x = -1.0; g.info.origin.position.y = 2.0;
  g.data = {0, 50, 100, -1, 0, 0};
  auto core = fromOccupancyGrid(g);
  EXPECT_EQ(core.width, 3); EXPECT_EQ(core.height, 2);
  EXPECT_NEAR(core.resolution, 0.25, 1e-9);
  EXPECT_NEAR(core.origin_x, -1.0, 1e-9);
  EXPECT_EQ(core.data[2], 100);
}

TEST(Adapter, LaserScanCopiesAnglesAndRanges) {
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -1.0; s.angle_increment = 0.5; s.range_min = 0.1; s.range_max = 10.0;
  s.ranges = {1.0f, 2.0f, 3.0f};
  prism_loc_core::Pose2D sib{0.1, 0.0, 0.0};
  auto core = fromLaserScan(s, sib);
  EXPECT_NEAR(core.angle_min, -1.0, 1e-9);
  EXPECT_NEAR(core.angle_increment, 0.5, 1e-9);
  ASSERT_EQ(core.ranges.size(), 3u);
  EXPECT_NEAR(core.ranges[1], 2.0, 1e-6);
  EXPECT_NEAR(core.sensor_in_base.x, 0.1, 1e-9);
}

TEST(Adapter, QuaternionToYaw) {
  geometry_msgs::msg::Transform t;
  const double yaw = 1.2;
  t.rotation.z = std::sin(yaw / 2); t.rotation.w = std::cos(yaw / 2);
  auto p = toPose2D(t);
  EXPECT_NEAR(p.yaw, yaw, 1e-9);
}
