#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/bbs.hpp"
#include "test_helpers.hpp"
using namespace prism_loc_core;

// Asymmetric room (a stub wall breaks the 4-fold symmetry so the global pose is unique).
static GridMap asymRoom() {
  GridMap g = test::makeRoomGrid(60, 60, 0.1);     // 6x6 m
  for (int y = 8; y < 30; ++y) g.data[y * 60 + 30] = 100;   // internal vertical wall stub
  for (int x = 30; x < 45; ++x) g.data[20 * 60 + x] = 100;  // internal horizontal stub
  return g;
}

TEST(Bbs, RecoversGlobalPoseFromSingleScan) {
  GridMap g = asymRoom();
  Pose2D truth{1.7, 4.3, 0.6}, sensor{0, 0, 0};
  LaserScan2D scan = test::raycastScan(g, truth, sensor, 180, 12.0);
  BbsParams p;
  p.linear_window = 3.0; p.angular_window = M_PI; p.angular_step = 0.0175;
  p.max_depth = 5; p.sigma_hit = 0.2; p.max_beams = 120; p.min_score_fraction = 0.4;
  BranchAndBoundMatcher m(g, p);
  BbsResult r = m.match(scan, Pose2D{3.0, 3.0, 0.0});   // search centered on map middle
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.pose.x, truth.x, 0.20);
  EXPECT_NEAR(r.pose.y, truth.y, 0.20);
  EXPECT_NEAR(normalizeAngle(r.pose.yaw - truth.yaw), 0.0, 0.06);
}

TEST(Bbs, EmptyScanIsInvalid) {
  GridMap g = asymRoom();
  LaserScan2D scan;                 // no ranges
  scan.angle_min = -M_PI; scan.angle_increment = 0.01; scan.range_min = 0.0; scan.range_max = 12.0;
  BranchAndBoundMatcher m(g, BbsParams{});
  EXPECT_FALSE(m.match(scan, Pose2D{3, 3, 0}).valid);
}
