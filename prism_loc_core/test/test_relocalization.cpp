#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>
#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/relocalization.hpp"
#include "test_helpers.hpp"
using namespace prism_loc_core;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

static void block(GridMap& g, int x0, int y0, int x1, int y1) {
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) g.data[y * g.width + x] = 100;
}

// 24 m x 5 m corridor (0.1 m/cell) split by a wall at x = 12 m into rooms A
// (x in 0..12) and B (x in 12..24). The first 6 m of both rooms are identical
// cell for cell (a 120-cell translation), so with a 3 m sensor range a robot in
// the left part of A sees exactly what it would see in the left part of B. The
// far halves differ: A has one pillar, B has two elsewhere.
static GridMap twinRooms() {
  GridMap g = test::makeRoomGrid(240, 50, 0.1);
  block(g, 120, 0, 121, 50);   // separating wall
  block(g, 78, 13, 82, 17);    // A: pillar at (7.8..8.2, 1.3..1.7)
  block(g, 195, 33, 199, 37);  // B: pillar at (19.5..19.9, 3.3..3.7)
  block(g, 208, 8, 212, 12);   // B: pillar at (20.8..21.2, 0.8..1.2)
  return g;
}

static constexpr double kRange = 3.0;
static constexpr int kBeams = 180;

static BbsParams params() {
  BbsParams p;
  p.angular_window = M_PI; p.angular_step = 0.0175; p.max_depth = 6;
  p.sigma_hit = 0.2; p.max_beams = 120; p.min_score_fraction = 0.4;
  return p;
}

static RelocVerifierParams verifierParams(int k, int m) {
  RelocVerifierParams v;
  v.top_k = k; v.verify_scans = m;
  return v;
}

// Drives the verifier along world poses path[0..], scanning `world` at each.
static RelocStatus drive(RelocalizationVerifier& ver, const GridMap& world,
                         const std::vector<Pose2D>& path) {
  RelocStatus st = ver.start(test::raycastScan(world, path[0], Pose2D{}, kBeams, kRange));
  for (size_t i = 1; i < path.size() && st == RelocStatus::kPending; ++i) {
    const Pose2D delta = compose(inverse(path[i - 1]), path[i]);
    st = ver.update(delta, test::raycastScan(world, path[i], Pose2D{}, kBeams, kRange));
  }
  return st;
}

static std::vector<Pose2D> straightPath(Pose2D start, double step, int n) {
  std::vector<Pose2D> p;
  for (int i = 0; i < n; ++i)
    p.push_back(Pose2D{start.x + i * step * std::cos(start.yaw),
                       start.y + i * step * std::sin(start.yaw), start.yaw});
  return p;
}

// ---------------------------------------------------------------------------
// Top-K distinct modes
// ---------------------------------------------------------------------------

TEST(BbsTopK, FirstModeIsTheSingleBestMatch) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> ux(0.6, 23.4), uy(0.6, 4.4), ua(-M_PI, M_PI);
  for (int k = 0; k < 6; ++k) {
    const Pose2D truth{ux(rng), uy(rng), ua(rng)};
    int mx, my;
    ASSERT_TRUE(worldToMap(g, truth.x, truth.y, mx, my));
    if (g.data[my * g.width + mx] >= 50) continue;
    const LaserScan2D scan = test::raycastScan(g, truth, Pose2D{}, kBeams, kRange);
    const BbsResult one = m.matchGlobal(scan);
    const std::vector<BbsResult> top = m.matchGlobalTopK(scan, BbsModeParams{5, 1.0, 0.35});
    ASSERT_FALSE(top.empty());
    EXPECT_DOUBLE_EQ(top[0].score, one.score) << "case " << k;
    EXPECT_DOUBLE_EQ(top[0].pose.x, one.pose.x);
    EXPECT_DOUBLE_EQ(top[0].pose.y, one.pose.y);
    EXPECT_DOUBLE_EQ(top[0].pose.yaw, one.pose.yaw);
    EXPECT_EQ(top[0].valid, one.valid);
  }
}

TEST(BbsTopK, ModesAreSortedAndMutuallyDistinct) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  const BbsModeParams mp{6, 1.0, 0.35};
  const LaserScan2D scan =
      test::raycastScan(g, Pose2D{2.55, 2.55, 0.2}, Pose2D{}, kBeams, kRange);
  const std::vector<BbsResult> top = m.matchGlobalTopK(scan, mp);
  ASSERT_GE(top.size(), 2u);
  EXPECT_LE(top.size(), 6u);
  for (size_t i = 0; i < top.size(); ++i)
    for (size_t j = i + 1; j < top.size(); ++j) {
      EXPECT_GE(top[i].score, top[j].score);
      const bool same_place = std::hypot(top[i].pose.x - top[j].pose.x,
                                         top[i].pose.y - top[j].pose.y) <= mp.nms_xy;
      const bool same_yaw =
          std::fabs(normalizeAngle(top[i].pose.yaw - top[j].pose.yaw)) <= mp.nms_yaw;
      EXPECT_FALSE(same_place && same_yaw) << i << " vs " << j;
    }
}

TEST(BbsTopK, AliasedTwinsAreBothReturnedWithEqualScore) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  const LaserScan2D scan =
      test::raycastScan(g, Pose2D{2.55, 2.55, 0.2}, Pose2D{}, kBeams, kRange);
  const std::vector<BbsResult> top = m.matchGlobalTopK(scan, BbsModeParams{5, 1.0, 0.35});
  ASSERT_GE(top.size(), 2u);
  EXPECT_DOUBLE_EQ(top[0].score, top[1].score);
  EXPECT_NEAR(std::fabs(top[0].pose.x - top[1].pose.x), 12.0, 1e-6);
  EXPECT_NEAR(top[0].pose.y, top[1].pose.y, 1e-9);
}

// ---------------------------------------------------------------------------
// Multi-scan verification
// ---------------------------------------------------------------------------

// The world differs from the map by an unmapped box beside the robot's start in
// room A; the map has the same box at the twin spot in room B. A single scan
// therefore prefers the WRONG room. Driving forward pushes the box out of range
// and brings A's own pillar into range; the verifier must end in room A.
// (With too few scans it may stop short, but only as ambiguous, never in B.)
TEST(RelocVerifier, MultiScanResolvesAliasThatFoolsSingleScan) {
  GridMap map = twinRooms();
  block(map, 128, 36, 131, 39);    // small box mapped in B only
  GridMap world = map;
  block(world, 8, 36, 11, 39);     // the same box, unmapped, in A
  BranchAndBoundMatcher m(map, params());
  const std::vector<Pose2D> path = straightPath(Pose2D{2.55, 2.55, 0.0}, 0.5, 10);

  const BbsResult single = m.matchGlobal(
      test::raycastScan(world, path[0], Pose2D{}, kBeams, kRange));
  ASSERT_TRUE(single.valid);
  ASSERT_GT(single.pose.x, 12.0) << "fixture must fool the single-scan matcher";

  RelocalizationVerifier ver(m, verifierParams(5, 10));
  const RelocStatus st = drive(ver, world, path);
  ASSERT_EQ(st, RelocStatus::kAccepted);
  EXPECT_EQ(ver.scansUsed(), 10);
  EXPECT_NEAR(ver.pose().x, path.back().x, 0.3);
  EXPECT_NEAR(ver.pose().y, path.back().y, 0.3);
  EXPECT_NEAR(normalizeAngle(ver.pose().yaw - path.back().yaw), 0.0, 0.1);
  EXPECT_GE(ver.bestPosterior(), verifierParams(5, 10).min_posterior);
}

// Nothing within sensor range ever distinguishes A from B: the verifier must
// report ambiguity instead of committing to either room.
TEST(RelocVerifier, TrulyAmbiguousPlaceIsReportedAmbiguous) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  const std::vector<Pose2D> path = straightPath(Pose2D{1.55, 2.55, 0.2}, 0.2, 5);
  RelocalizationVerifier ver(m, verifierParams(5, 5));
  const RelocStatus st = drive(ver, g, path);
  EXPECT_EQ(st, RelocStatus::kAmbiguous);
  EXPECT_LT(ver.bestPosterior(), verifierParams(5, 5).min_posterior);
  // Both twins are still live hypotheses.
  int in_a = 0, in_b = 0;
  for (const RelocHypothesis& h : ver.hypotheses()) {
    if (!h.alive) continue;
    if (std::hypot(h.pose.x - path.back().x, h.pose.y - path.back().y) < 0.3) ++in_a;
    if (std::hypot(h.pose.x - path.back().x - 12.0, h.pose.y - path.back().y) < 0.3) ++in_b;
  }
  EXPECT_GE(in_a, 1);
  EXPECT_GE(in_b, 1);
}

// Unambiguous place: verification accepts the same pose the single scan finds.
TEST(RelocVerifier, UnambiguousPlaceIsAcceptedAtTheTruePose) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  // Right half of room B, next to both B pillars: unique within range.
  const std::vector<Pose2D> path = straightPath(Pose2D{20.05, 2.25, 1.0}, 0.2, 5);
  const BbsResult single = m.matchGlobal(
      test::raycastScan(g, path[0], Pose2D{}, kBeams, kRange));
  ASSERT_TRUE(single.valid);
  RelocalizationVerifier ver(m, verifierParams(5, 5));
  ASSERT_EQ(drive(ver, g, path), RelocStatus::kAccepted);
  EXPECT_NEAR(ver.pose().x, path.back().x, 0.2);
  EXPECT_NEAR(ver.pose().y, path.back().y, 0.2);
  EXPECT_NEAR(normalizeAngle(ver.pose().yaw - path.back().yaw), 0.0, 0.06);
}

// K = 1, M = 1 is exactly the legacy single-scan rule.
TEST(RelocVerifier, SingleHypothesisSingleScanReproducesSingleScanMatch) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  const LaserScan2D scan =
      test::raycastScan(g, Pose2D{20.05, 2.25, 1.0}, Pose2D{}, kBeams, kRange);
  const BbsResult single = m.matchGlobal(scan);
  RelocalizationVerifier ver(m, verifierParams(1, 1));
  ASSERT_EQ(ver.start(scan), single.valid ? RelocStatus::kAccepted : RelocStatus::kNoCandidate);
  EXPECT_DOUBLE_EQ(ver.pose().x, single.pose.x);
  EXPECT_DOUBLE_EQ(ver.pose().y, single.pose.y);
  EXPECT_DOUBLE_EQ(ver.pose().yaw, single.pose.yaw);
}

TEST(RelocVerifier, EmptyScanYieldsNoCandidate) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  LaserScan2D scan;
  scan.angle_min = -M_PI; scan.angle_increment = 0.01; scan.range_min = 0.0;
  scan.range_max = 3.0;
  RelocalizationVerifier ver(m, verifierParams(5, 5));
  EXPECT_EQ(ver.start(scan), RelocStatus::kNoCandidate);
}

// ---------------------------------------------------------------------------
// Motion gate
// ---------------------------------------------------------------------------

// The alias fixture of MultiScanResolvesAliasThatFoolsSingleScan, but the robot
// stands still: the node feeds every scan, so without a motion gate the verifier
// re-scores the same view verify_scans times, multiplies the first scan's small
// preference for the wrong room into a near-certain posterior and commits to B.
static GridMap aliasMap() {
  GridMap map = twinRooms();
  block(map, 128, 36, 131, 39);    // small box mapped in B only
  return map;
}
static GridMap aliasWorld() {
  GridMap world = aliasMap();
  block(world, 8, 36, 11, 39);     // the same box, unmapped, in A
  return world;
}

TEST(RelocVerifierMotionGate, StationaryRepeatsCommitToTheWrongRoomWithoutGate) {
  GridMap map = aliasMap(), world = aliasWorld();
  BranchAndBoundMatcher m(map, params());
  const Pose2D here{2.55, 2.55, 0.0};
  RelocalizationVerifier ver(m, verifierParams(5, 9));
  RelocStatus st = ver.start(test::raycastScan(world, here, Pose2D{}, kBeams, kRange));
  ASSERT_EQ(st, RelocStatus::kPending);
  const double first = ver.bestPosterior();
  ASSERT_LT(first, verifierParams(5, 9).min_posterior) << "one scan alone must not decide";
  for (int i = 0; i < 20 && st == RelocStatus::kPending; ++i)
    st = ver.update(Pose2D{}, test::raycastScan(world, here, Pose2D{}, kBeams, kRange));
  // No new information arrived, yet the verifier became certain - of the wrong room.
  EXPECT_EQ(st, RelocStatus::kAccepted);
  EXPECT_GT(ver.pose().x, 12.0);
  EXPECT_GE(ver.bestPosterior(), verifierParams(5, 9).min_posterior);
}

TEST(RelocVerifierMotionGate, GateHoldsStationaryRobotPendingThenResolvesOnTheMove) {
  GridMap map = aliasMap(), world = aliasWorld();
  BranchAndBoundMatcher m(map, params());
  RelocVerifierParams vp = verifierParams(5, 10);
  vp.min_translation = 0.45;  // 0.1 m steps: every fifth scan counts
  vp.min_rotation = 0.3;
  RelocalizationVerifier ver(m, vp);
  const std::vector<Pose2D> path = straightPath(Pose2D{2.55, 2.55, 0.0}, 0.1, 46);
  RelocStatus st = ver.start(test::raycastScan(world, path[0], Pose2D{}, kBeams, kRange));
  // Standing still: every scan is skipped, nothing is decided.
  for (int i = 0; i < 20; ++i)
    st = ver.update(Pose2D{}, test::raycastScan(world, path[0], Pose2D{}, kBeams, kRange));
  EXPECT_EQ(st, RelocStatus::kPending);
  EXPECT_EQ(ver.scansUsed(), 1);
  EXPECT_EQ(ver.scansSkipped(), 20);
  // Driving 0.1 m per scan: only every fifth scan counts (0.5 m apart, the spacing of
  // the moving test above), and the odometry of the skipped ones is carried into the
  // counted one.
  for (size_t i = 1; i < path.size() && st == RelocStatus::kPending; ++i)
    st = ver.update(compose(inverse(path[i - 1]), path[i]),
                    test::raycastScan(world, path[i], Pose2D{}, kBeams, kRange));
  ASSERT_EQ(st, RelocStatus::kAccepted);
  EXPECT_EQ(ver.scansUsed(), 10);
  EXPECT_LT(ver.pose().x, 12.0) << "must end in room A";
  EXPECT_LT(std::hypot(ver.pose().x - path[45].x, ver.pose().y - path[45].y), 0.3);
}

TEST(RelocVerifierMotionGate, RotationAloneOpensTheGate) {
  GridMap g = twinRooms();
  BranchAndBoundMatcher m(g, params());
  RelocVerifierParams vp = verifierParams(5, 5);
  vp.min_translation = 0.3;
  vp.min_rotation = 0.3;
  RelocalizationVerifier ver(m, vp);
  Pose2D p{20.05, 2.25, 1.0};
  ASSERT_EQ(ver.start(test::raycastScan(g, p, Pose2D{}, kBeams, kRange)), RelocStatus::kPending);
  const Pose2D turn{0.0, 0.0, 0.2};
  p.yaw += 0.2;
  ver.update(turn, test::raycastScan(g, p, Pose2D{}, kBeams, kRange));
  EXPECT_EQ(ver.scansUsed(), 1);  // 0.2 rad: below the gate
  p.yaw += 0.2;
  ver.update(turn, test::raycastScan(g, p, Pose2D{}, kBeams, kRange));
  EXPECT_EQ(ver.scansUsed(), 2);  // 0.4 rad accumulated: counted
}
