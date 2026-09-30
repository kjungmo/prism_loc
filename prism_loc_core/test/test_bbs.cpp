#include <gtest/gtest.h>
#include <cmath>
#include <random>
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

// ---------------------------------------------------------------------------
// Search-window geometry, map-sized global search and feasibility filter.
// ---------------------------------------------------------------------------

// A 40 m x 30 m warehouse-like map (0.1 m/cell) with irregular internal structure
// so a 360-deg scan pins the pose. Far larger than the legacy +-10 m window.
static GridMap largeMap() {
  GridMap g = test::makeRoomGrid(400, 300, 0.1);
  auto block = [&](int x0, int y0, int x1, int y1) {
    for (int y = y0; y < y1; ++y)
      for (int x = x0; x < x1; ++x) g.data[y * g.width + x] = 100;
  };
  block(40, 40, 70, 45);     // shelves / walls of differing length
  block(40, 90, 45, 160);
  block(120, 60, 200, 64);
  block(150, 150, 154, 260);
  block(230, 30, 236, 120);
  block(260, 180, 360, 186);
  block(300, 60, 330, 90);   // solid pillar
  block(90, 220, 120, 228);
  block(340, 230, 345, 290);
  return g;
}

static BbsParams defaultParams() {
  BbsParams p;
  p.angular_window = M_PI; p.angular_step = 0.0175; p.max_depth = 6;
  p.sigma_hit = 0.2; p.max_beams = 120; p.min_score_fraction = 0.4;
  return p;
}

TEST(Bbs, GlobalSearchFindsKidnappedPoseFarFromCentreOnLargeMap) {
  GridMap g = largeMap();
  // 17 m / 12 m from the map centre (20, 15): outside any +-10 m window.
  Pose2D truth{3.05, 2.85, -2.3}, sensor{0, 0, 0};
  LaserScan2D scan = test::raycastScan(g, truth, sensor, 360, 30.0);
  BranchAndBoundMatcher m(g, defaultParams());
  BbsResult r = m.matchGlobal(scan);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.window_covers_map);
  EXPECT_NEAR(r.pose.x, truth.x, 0.2);
  EXPECT_NEAR(r.pose.y, truth.y, 0.2);
  EXPECT_NEAR(normalizeAngle(r.pose.yaw - truth.yaw), 0.0, 0.06);
}

TEST(Bbs, GlobalWindowIsCappedAndReportsPartialCoverage) {
  GridMap g = largeMap();
  BbsParams p = defaultParams();
  p.max_linear_window = 5.0;   // cap below the 20 m / 15 m half-extent
  BranchAndBoundMatcher m(g, p);
  Pose2D truth{20.3, 16.1, 0.4}, sensor{0, 0, 0};
  LaserScan2D scan = test::raycastScan(g, truth, sensor, 360, 30.0);
  BbsResult r = m.matchGlobal(scan);
  EXPECT_FALSE(r.window_covers_map);
  // Whatever it returns lies inside the capped window around the map centre.
  EXPECT_LE(std::fabs(r.pose.x - 20.05), 5.0 + 1e-9);
  EXPECT_LE(std::fabs(r.pose.y - 15.05), 5.0 + 1e-9);
}

TEST(Bbs, SearchWindowIsSymmetricAboutCentre) {
  // With L = 20 cells and 2^D = 16 the legacy root enumeration covered offsets
  // [-20, +27]. The true pose sits at +2.7 m (+27 cells): inside the old
  // asymmetric superset, outside the stated +-2 m window. The matcher must not
  // return anything beyond +-L.
  GridMap g = asymRoom();
  BbsParams p = defaultParams();
  p.linear_window = 2.0; p.max_depth = 4;
  BranchAndBoundMatcher m(g, p);
  const Pose2D center{2.5, 3.0, 0.0};
  const Pose2D sensor{0, 0, 0};
  LaserScan2D scan = test::raycastScan(g, Pose2D{5.2, 3.0, 0.3}, sensor, 180, 12.0);
  BbsResult r = m.match(scan, center);
  EXPECT_LE(std::fabs(r.pose.x - center.x), 2.0 + 1e-9);
  EXPECT_LE(std::fabs(r.pose.y - center.y), 2.0 + 1e-9);
  // The negative corner of the window (-L, -L) is searched and recovered.
  LaserScan2D scan2 = test::raycastScan(g, Pose2D{0.5, 1.0, -0.8}, sensor, 180, 12.0);
  BbsResult r2 = m.match(scan2, center);
  ASSERT_TRUE(r2.valid);
  EXPECT_NEAR(r2.pose.x, 0.5, 0.15);
  EXPECT_NEAR(r2.pose.y, 1.0, 0.15);
}

// Candidates whose robot position lies in an occupied cell are never returned.
TEST(Bbs, NeverReturnsPoseInsideOccupiedCell) {
  GridMap g = test::makeRoomGrid(60, 60, 0.1);
  for (int y = 20; y < 40; ++y)
    for (int x = 20; x < 40; ++x) g.data[y * 60 + x] = 100;  // 2x2 m solid pillar
  // Very short returns in every direction: unconstrained, the best "pose" is
  // buried inside the pillar where every endpoint lands on an occupied cell.
  LaserScan2D scan;
  scan.angle_min = -M_PI; scan.angle_increment = 2.0 * M_PI / 90;
  scan.range_min = 0.0; scan.range_max = 12.0;
  scan.ranges.assign(90, 0.3f);
  BbsParams p = defaultParams();
  p.min_score_fraction = 0.0;
  BranchAndBoundMatcher m(g, p);
  BbsResult r = m.matchGlobal(scan);
  int mx, my;
  ASSERT_TRUE(worldToMap(g, r.pose.x, r.pose.y, mx, my));
  EXPECT_LT(g.data[my * 60 + mx], 50);
}

// Candidates whose robot position lies off the grid are never returned.
TEST(Bbs, NeverReturnsPoseOffTheMap) {
  GridMap g; g.width = 60; g.height = 60; g.resolution = 0.1;
  g.origin_x = 0.0; g.origin_y = 0.0; g.data.assign(60 * 60, 0);
  for (int y = 0; y < 60; ++y) g.data[y * 60] = 100;  // single wall on the left edge
  // Forward-looking beams hitting something 1 m ahead: best unconstrained pose is
  // 1 m left of the wall, i.e. off the grid.
  LaserScan2D scan;
  scan.angle_min = -0.2; scan.angle_increment = 0.01;
  scan.range_min = 0.0; scan.range_max = 12.0;
  scan.ranges.assign(41, 1.0f);
  BbsParams p = defaultParams();
  p.angular_window = 0.0; p.linear_window = 3.0; p.min_score_fraction = 0.0;
  BranchAndBoundMatcher m(g, p);
  BbsResult r = m.match(scan, Pose2D{0.5, 3.0, 0.0});   // yaw fixed facing +x
  int mx, my;
  EXPECT_TRUE(worldToMap(g, r.pose.x, r.pose.y, mx, my));
}

// Branch-and-bound returns exactly the exhaustive maximum over the feasible,
// symmetric search set (admissible bound => optimality), on a small case.
// The exhaustive-agreement check, shared by the optimality test and by the
// negative controls below: number of cases (out of 12) where branch-and-bound
// and exhaustive search disagree on the best score or on validity.
static int exhaustiveDisagreements(const BranchAndBoundMatcher& m, const GridMap& g) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.6, 5.4), uy(-0.3, 0.3), un(0.0, 1.0);
  int bad = 0;
  for (int k = 0; k < 12; ++k) {
    Pose2D truth{u(rng), u(rng), uy(rng)};
    LaserScan2D scan = test::raycastScan(g, truth, Pose2D{0, 0, 0}, 120, 12.0);
    for (float& rr : scan.ranges) if (un(rng) < 0.3) rr = static_cast<float>(0.2 + 8.0 * un(rng));
    const Pose2D center{3.0 + 0.3 * (un(rng) - 0.5), 3.0 + 0.3 * (un(rng) - 0.5), 0.0};
    const BbsResult bb = m.match(scan, center);
    const BbsResult ex = m.matchExhaustive(scan, center);
    if (bb.score != ex.score || bb.valid != ex.valid) ++bad;
  }
  return bad;
}

static BbsParams agreementParams() {
  BbsParams p = defaultParams();
  p.linear_window = 1.5; p.max_depth = 3;
  p.angular_window = 0.35; p.angular_step = 0.0175; p.max_beams = 60;
  return p;
}

TEST(Bbs, BranchAndBoundMatchesExhaustiveSearch) {
  GridMap g = asymRoom();
  BranchAndBoundMatcher m(g, agreementParams());
  EXPECT_EQ(exhaustiveDisagreements(m, g), 0);
}

// Negative controls: the agreement check must be able to fail. An inflated
// coarse bound is still admissible (only slower) and must still agree; a
// deflated bound, or coarse levels that drop off-map cells like the leaves do
// (the pre-fix v0.1 behaviour), are inadmissible and must be detected.
TEST(Bbs, AgreementCheckAcceptsInflatedAdmissibleBound) {
  GridMap g = asymRoom();
  BranchAndBoundMatcher m(g, agreementParams());
  m.setBoundMutationForTesting(BoundMutationForTesting{1.5, false});
  EXPECT_EQ(exhaustiveDisagreements(m, g), 0);
}

TEST(Bbs, AgreementCheckDetectsDeflatedBound) {
  GridMap g = asymRoom();
  BranchAndBoundMatcher m(g, agreementParams());
  m.setBoundMutationForTesting(BoundMutationForTesting{0.8, false});
  EXPECT_GT(exhaustiveDisagreements(m, g), 0);
}

TEST(Bbs, AgreementCheckDetectsCoarseLevelsDroppingOffMapCells) {
  GridMap g = asymRoom();
  BranchAndBoundMatcher m(g, agreementParams());
  m.setBoundMutationForTesting(BoundMutationForTesting{1.0, true});
  EXPECT_GT(exhaustiveDisagreements(m, g), 0);
}
