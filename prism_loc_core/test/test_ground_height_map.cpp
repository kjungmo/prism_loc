#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include "prism_loc_core/ground_height_map.hpp"
using namespace prism_loc_core;

namespace {
using Pts = std::vector<Eigen::Vector3d>;

// Points on a surface z = f(x, y) over [x0, x1] x [y0, y1], `step` apart (offset off the
// cell borders so that every cell gets the same count).
Pts surface(double x0, double x1, double y0, double y1, double step,
            const std::function<double(double, double)>& f) {
  Pts pts;
  for (double x = x0 + 0.013; x < x1; x += step)
    for (double y = y0 + 0.017; y < y1; y += step) pts.emplace_back(x, y, f(x, y));
  return pts;
}
void append(Pts& a, const Pts& b) { a.insert(a.end(), b.begin(), b.end()); }
}  // namespace

TEST(GroundHeightMap, FlatGround) {
  const Pts pts = surface(0, 10, 0, 10, 0.1, [](double, double) { return 0.3; });
  GroundHeightMap g(pts);
  EXPECT_EQ(g.numCells(), 100u);
  EXPECT_EQ(g.numCandidates(), 100u);
  for (double x : {0.2, 3.5, 9.9})
    for (double y : {0.4, 5.5, 9.7}) {
      const auto h = g.heightAt(x, y, 0.3);
      ASSERT_TRUE(h.has_value());
      EXPECT_NEAR(*h, 0.3, 1e-9);
    }
  // a hint 0.4 m off is still within max_step 0.5 and gets the ground, not the hint
  EXPECT_NEAR(*g.heightAt(5.0, 5.0, 0.7), 0.3, 1e-9);
}

// z = 0.2 x: the candidate is the plane value at the cell centre and the query blends the
// planes, so the height follows the slope.
TEST(GroundHeightMap, SlopeFollowsWithinCellTimesSlope) {
  const double slope = 0.2;
  const Pts pts = surface(0, 20, 0, 6, 0.1, [&](double x, double) { return slope * x; });
  GroundHeightMap g(pts);
  for (double x = 1.3; x < 19.0; x += 0.37) {
    const auto h = g.heightAt(x, 3.0, slope * x);
    ASSERT_TRUE(h.has_value()) << x;
    EXPECT_NEAR(*h, slope * x, 1.0 * slope) << x;
  }
}

// A wall standing on the ground adds points above it in the same cells: they only lift the
// sub-columns they stand in, which the robust pass drops: the ground is still the ground.
TEST(GroundHeightMap, WallOnGroundReturnsGround) {
  Pts pts = surface(0, 6, 0, 6, 0.1, [](double, double) { return 0.0; });
  for (double y = 2.0; y < 4.0; y += 0.05)
    for (double z = 0.1; z < 2.5; z += 0.05) pts.emplace_back(3.02, y, z);
  GroundHeightMap g(pts);
  const auto h = g.heightAt(3.02, 3.0, 0.0);
  ASSERT_TRUE(h.has_value());
  EXPECT_NEAR(*h, 0.0, 0.05);
}

TEST(GroundHeightMap, NoiseBelowGroundIsIgnored) {
  Pts pts = surface(0, 4, 0, 4, 0.1, [](double, double) { return 1.0; });  // 100 per cell
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.0, 4.0), dz(0.2, 0.9);
  // 3 stray points per 1 m cell (3 % of the cell) up to 0.9 m below the ground; the
  // the strays are separate small groups (a gap over layer_gap, fewer than min_points): dropped
  for (int cx = 0; cx < 4; ++cx)
    for (int cy = 0; cy < 4; ++cy)
      for (int k = 0; k < 3; ++k) pts.emplace_back(cx + 0.5 + 0.1 * k, cy + 0.5, 1.0 - dz(rng));
  GroundHeightMap g(pts);
  for (double x : {0.5, 1.5, 2.5, 3.5}) {
    const auto h = g.heightAt(x, 2.5, 1.0);
    ASSERT_TRUE(h.has_value());
    EXPECT_NEAR(*h, 1.0, 1e-9);
  }
}

// Two drivable levels 3 m apart over the same xy area.
TEST(GroundHeightMap, TwoLevelsHintSelectsTheLevel) {
  Pts pts = surface(0, 6, 0, 6, 0.1, [](double, double) { return 0.0; });
  append(pts, surface(0, 6, 0, 6, 0.1, [](double, double) { return 3.0; }));
  GroundHeightMap g(pts);
  EXPECT_EQ(g.numCells(), 36u);
  EXPECT_EQ(g.numCandidates(), 72u);
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 0.1), 0.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 2.9), 3.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 3.4), 3.0, 1e-9);
  // 1 m from a level (1.0: 1 m from 0, 2 m from 3): rejected by max_step 0.5
  EXPECT_FALSE(g.heightAt(3.0, 3.0, 1.0).has_value());
  EXPECT_FALSE(g.heightAt(3.0, 3.0, 2.0).has_value());
  // the reseed query takes the nearest level whatever the distance
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 1.0), 0.0, 1e-9);
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 2.0), 3.0, 1e-9);
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 0.0), 0.0, 1e-9);   // RViz: z = 0
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 100.0), 3.0, 1e-9);
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, -50.0), 0.0, 1e-9);
  // a tie between two levels goes to the lower one
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 1.5), 0.0, 1e-9);
}

// Surfaces closer than layer_gap (0.2 m) are one group: one candidate. Further apart: two.
TEST(GroundHeightMap, SurfacesCloserThanLayerGapMerge) {
  Pts pts = surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; });
  append(pts, surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.1; }));
  GroundHeightMap g(pts);
  EXPECT_EQ(g.numCandidates(), g.numCells());
  EXPECT_NEAR(*g.heightAt(1.5, 1.5, 0.0), 0.05, 0.05);  // one candidate between the two surfaces (their median)
  Pts far = surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; });
  append(far, surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.3; }));
  GroundHeightMap g2(far);
  EXPECT_EQ(g2.numCandidates(), 2 * g2.numCells());
  EXPECT_NEAR(*g2.heightAt(1.5, 1.5, 0.3), 0.3, 1e-9);
}

// A thin group (fewer than min_points) gives no candidate, e.g. a few points of a cable.
TEST(GroundHeightMap, SmallGroupGivesNoCandidate) {
  Pts pts = surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; });
  for (int k = 0; k < 4; ++k) pts.emplace_back(1.5 + 0.01 * k, 1.5, 5.0);
  GroundHeightMap g(pts);
  EXPECT_EQ(g.numCandidates(), g.numCells());
  EXPECT_FALSE(g.heightAt(1.5, 1.5, 5.0).has_value());
  EXPECT_NEAR(*g.heightAt(1.5, 1.5, 0.0), 0.0, 1e-9);
  // a cell with fewer than min_points points altogether is empty
  GroundHeightMap sparse(Pts{{0.5, 0.5, 0.0}, {0.6, 0.5, 0.0}}, GroundHeightParams{});
  EXPECT_EQ(sparse.numCells(), 0u);
}

// An empty cell is filled by the planes of its neighbours (bilinear blend); a hole three
// cells wide stays empty in its middle.
TEST(GroundHeightMap, HolesAreFilledByNeighboursFarHolesAreNot) {
  Pts pts = surface(0, 9, 0, 9, 0.1, [](double, double) { return 0.0; });
  // cut a 1x1 hole at cell (4, 4) and a 4x4 hole at cells (6..9) x (6..9)
  pts.erase(std::remove_if(pts.begin(), pts.end(),
                           [](const Eigen::Vector3d& p) {
                             const bool h1 = p.x() >= 4 && p.x() < 5 && p.y() >= 4 && p.y() < 5;
                             const bool h2 = p.x() >= 5.0 && p.x() < 9 && p.y() >= 5.0 && p.y() < 9 &&
                                             p.x() >= 6.0 && p.y() >= 6.0;
                             return h1 || h2;
                           }),
            pts.end());
  GroundHeightMap g(pts);
  EXPECT_NEAR(*g.heightAt(4.5, 4.5, 0.0), 0.0, 1e-9);  // hole cell: neighbours answer
  EXPECT_FALSE(g.heightAt(7.5, 7.5, 0.0).has_value()) << "centre of a 3x3-cell hole";
  EXPECT_FALSE(g.heightAt(50.0, 50.0, 0.0).has_value());
  EXPECT_FALSE(g.heightAt(-30.0, 2.0, 0.0).has_value());
  EXPECT_FALSE(g.heightNearest(50.0, 50.0, 0.0).has_value());
}

// The height is interpolated between the cell centres: at a centre it is that cell's value,
// halfway between two centres the mean, and it never leaves the range of the cells' values.
TEST(GroundHeightMap, InterpolatesBetweenCellCentres) {
  Pts pts = surface(0, 4, 0, 4, 0.1, [](double, double) { return 0.0; });
  for (auto& p : pts)
    if (p.x() >= 2 && p.y() >= 2 && p.x() < 3 && p.y() < 3) p.z() = 0.1;  // the cell (2, 2)
  GroundHeightMap g(pts);
  EXPECT_NEAR(*g.heightAt(1.5, 1.5, 0.1), 0.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(2.5, 2.5, 0.1), 0.1, 1e-9);
  EXPECT_NEAR(*g.heightAt(2.0, 2.0, 0.05), 0.025, 1e-9);  // the corner of four cells, one at 0.1
  EXPECT_NEAR(*g.heightAt(2.5, 2.0, 0.05), 0.05, 1e-9);   // halfway between the centres (2,1) 0 and (2,2) 0.1
  // a step of 0.4 m is another surface (more than layer_gap): not blended across
  for (auto& p : pts)
    if (p.x() >= 2 && p.y() >= 2 && p.x() < 3 && p.y() < 3) p.z() = 0.4;
  GroundHeightMap step(pts);
  EXPECT_NEAR(*step.heightAt(2.0, 2.0, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(*step.heightAt(2.0, 2.0, 0.4), 0.4, 1e-9);
}

TEST(GroundHeightMap, EmptyMapAndBadArguments) {
  GroundHeightMap g(Pts{});
  EXPECT_EQ(g.numCells(), 0u);
  EXPECT_EQ(g.numCandidates(), 0u);
  EXPECT_FALSE(g.heightAt(0, 0, 0).has_value());
  EXPECT_FALSE(g.heightNearest(0, 0, 0).has_value());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  GroundHeightMap flat(surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; }));
  EXPECT_FALSE(flat.heightAt(nan, 1, 0).has_value());
  EXPECT_FALSE(flat.heightAt(1, inf, 0).has_value());
  EXPECT_FALSE(flat.heightAt(1, 1, nan).has_value());
  EXPECT_FALSE(flat.heightNearest(1, 1, nan).has_value());
  EXPECT_FALSE(flat.heightAt(1e300, 1, 0).has_value());
  // non-finite map points are skipped
  Pts bad = surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; });
  bad.emplace_back(nan, 1.0, 0.0);
  bad.emplace_back(1.0, inf, 0.0);
  bad.emplace_back(1.0, 1.0, nan);
  EXPECT_NO_THROW(GroundHeightMap{bad});
  EXPECT_NEAR(*GroundHeightMap(bad).heightAt(1.5, 1.5, 0.0), 0.0, 1e-9);
}

// The same points in another order give the same map: identical outputs, bit for bit.
TEST(GroundHeightMap, DeterministicUnderShuffle) {
  Pts pts = surface(0, 12, 0, 12, 0.15, [](double x, double y) { return 0.1 * x + 0.05 * std::sin(y); });
  append(pts, surface(0, 12, 0, 12, 0.2, [](double, double) { return 4.0; }));
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0.0, 12.0);
  for (int i = 0; i < 300; ++i) pts.emplace_back(u(rng), u(rng), u(rng) * 0.5);  // clutter
  GroundHeightMap a(pts);
  for (int seed : {1, 2, 3}) {
    Pts shuffled = pts;
    std::shuffle(shuffled.begin(), shuffled.end(), std::mt19937(seed));
    GroundHeightMap b(shuffled);
    EXPECT_EQ(a.numCells(), b.numCells());
    EXPECT_EQ(a.numCandidates(), b.numCandidates());
    for (double x = -1.0; x < 13.0; x += 0.31)
      for (double y = -1.0; y < 13.0; y += 0.43)
        for (double hint : {0.0, 1.0, 4.0}) {
          const auto ha = a.heightAt(x, y, hint), hb = b.heightAt(x, y, hint);
          ASSERT_EQ(ha.has_value(), hb.has_value());
          if (ha) { EXPECT_EQ(*ha, *hb); }
          const auto na = a.heightNearest(x, y, hint), nb = b.heightNearest(x, y, hint);
          ASSERT_EQ(na.has_value(), nb.has_value());
          if (na) { EXPECT_EQ(*na, *nb); }
        }
  }
}

TEST(GroundHeightMap, NegativeCoordinatesAndCellBorders) {
  Pts pts = surface(-5, 5, -5, 5, 0.1, [](double, double) { return -2.0; });
  GroundHeightMap g(pts);
  EXPECT_NEAR(*g.heightAt(-4.9, -4.9, -2.0), -2.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(0.0, 0.0, -2.2), -2.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(-1.0, -1.0, -2.0), -2.0, 1e-9);
}

// The error of heightAt against a constant slope is below 2 cm inside the map, mid-cell and
// on cell borders alike (the candidate is the plane value at the cell centre, the query
// interpolates between centres), and the height is continuous.
TEST(GroundHeightMap, ConstantSlopesAreFollowedWithinTwoCentimetres) {
  for (double s : {0.05, 0.1})
    for (int dir = 0; dir < 3; ++dir) {
      const double ax = dir == 0 || dir == 2 ? s : 0.0, ay = dir == 1 || dir == 2 ? s : 0.0;
      const auto f = [&](double x, double y) { return 0.7 + ax * x + ay * y; };
      GroundHeightMap g(surface(0, 12, 0, 12, 0.1, f));
      double worst = 0.0, worst_jump = 0.0;
      for (double x = 1.0; x < 11.0; x += 0.037)
        for (double y = 1.0; y < 11.0; y += 0.041) {
          const auto h = g.heightAt(x, y, f(x, y));
          ASSERT_TRUE(h.has_value());
          worst = std::max(worst, std::fabs(*h - f(x, y)));
          const auto h2 = g.heightAt(x + 0.01, y, f(x, y));
          worst_jump = std::max(worst_jump, std::fabs(*h2 - *h));
        }
      EXPECT_LT(worst, 0.02) << "slope " << s << " dir " << dir;
      EXPECT_LT(worst_jump, ax * 0.01 + 0.002) << "slope " << s << " dir " << dir;
    }
}

// A wall standing on a slope: the ground under and next to it is still the ground.
TEST(GroundHeightMap, WallOnASlopeReturnsTheGround) {
  const auto f = [](double x, double) { return 0.1 * x; };
  Pts pts = surface(0, 8, 0, 8, 0.1, f);
  for (double y = 2.0; y < 6.0; y += 0.05)
    for (double h = 0.05; h < 2.5; h += 0.05) pts.emplace_back(3.02, y, f(3.02, y) + h);
  GroundHeightMap g(pts);
  for (double x : {2.6, 3.2, 3.5, 3.9, 4.4})
    for (double y : {2.5, 3.5, 5.0}) {
      const auto h = g.heightAt(x, y, f(x, y));
      ASSERT_TRUE(h.has_value());
      EXPECT_NEAR(*h, f(x, y), 0.02) << x << " " << y;
    }
}

// Floor at 0, a ceiling underside at 3.0 and a floor above at 3.3: three surfaces; the hint picks.
TEST(GroundHeightMap, FloorCeilingFloorAboveAreSeparateSurfaces) {
  Pts pts = surface(0, 6, 0, 6, 0.1, [](double, double) { return 0.0; });
  append(pts, surface(0, 6, 0, 6, 0.1, [](double, double) { return 3.0; }));
  append(pts, surface(0, 6, 0, 6, 0.1, [](double, double) { return 3.3; }));
  GroundHeightMap g(pts);
  EXPECT_EQ(g.numCandidates(), 3 * g.numCells());
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 3.3), 3.3, 1e-9);
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(*g.heightAt(3.0, 3.0, 3.0), 3.0, 1e-9);   // the ceiling underside is a candidate too
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 3.4), 3.3, 1e-9);
  EXPECT_NEAR(*g.heightNearest(3.0, 3.0, 1.0), 0.0, 1e-9);
}

// A pillar from the floor to the ceiling in one cell: that cell's lower group holds the floor
// and the pillar, the floor above stays separate. The pillar's sub-columns are lifted; the
// robust pass drops them (the error there stays small) and the hint selects the floor.
TEST(GroundHeightMap, PillarCellKeepsFloorAndFloorAbove) {
  Pts pts = surface(0, 6, 0, 6, 0.1, [](double, double) { return 0.0; });
  append(pts, surface(0, 6, 0, 6, 0.1, [](double, double) { return 3.3; }));
  for (double z = 0.05; z < 3.0; z += 0.05)
    for (double dx = 0.0; dx < 0.3; dx += 0.05)
      for (double dy = 0.0; dy < 0.3; dy += 0.05) pts.emplace_back(2.6 + dx, 2.6 + dy, z);
  GroundHeightMap g(pts);
  EXPECT_NEAR(*g.heightAt(2.75, 2.75, 0.0), 0.0, 0.05);
  EXPECT_NEAR(*g.heightAt(2.75, 2.75, 3.3), 3.3, 0.01);
  EXPECT_NEAR(*g.heightAt(1.5, 1.5, 0.0), 0.0, 1e-9);  // next to it: untouched
}

// Wall points every 0.5 m in z: each height with enough points is a candidate of its own,
// the hint (the ground of the estimate) still picks the ground.
TEST(GroundHeightMap, SparseWallPointsDoNotMoveTheGround) {
  Pts pts = surface(0, 6, 0, 6, 0.1, [](double, double) { return 0.0; });
  for (double z = 0.5; z < 3.0; z += 0.5)
    for (double y = 2.0; y < 4.0; y += 0.1) pts.emplace_back(3.02, y, z);
  GroundHeightMap g(pts);
  EXPECT_GT(g.numCandidates(), g.numCells());  // the wall rows are candidates
  for (double hint : {0.0, 0.1, -0.1})
    for (double y : {2.5, 3.0, 3.5}) EXPECT_NEAR(*g.heightAt(3.3, y, hint), 0.0, 1e-9);
}

// Plane fit refused: the sub-columns lie on a line, or fewer than 3: the mean of their values.
TEST(GroundHeightMap, IllConditionedFitFallsBackToTheMeanZ) {
  Pts line;  // one row of points across a cell, z rising 0.1 m: a line in xy
  for (int k = 0; k < 20; ++k) line.emplace_back(0.025 + 0.045 * k, 0.5, 0.005 * k);
  const GroundHeightParams p;
  const auto h = GroundHeightMap(line, p).heightAt(0.5, 0.5, 0.05);
  ASSERT_TRUE(h.has_value());
  // 4 sub-columns with median z 0.01, 0.0375, 0.0675, 0.09 on a line in xy: their mean, slope 0
  EXPECT_NEAR(*h, 0.05125, 0.002);
  // two sub-columns only: fewer than 3 used
  Pts two;
  for (int k = 0; k < 10; ++k) { two.emplace_back(0.1, 0.1, 0.0); two.emplace_back(0.9, 0.9, 0.2); }
  EXPECT_NEAR(*GroundHeightMap(two, p).heightAt(0.5, 0.5, 0.1), 0.1, 1e-9);
}

TEST(GroundHeightMap, CountsPointsOutOfRange) {
  Pts pts = surface(0, 3, 0, 3, 0.1, [](double, double) { return 0.0; });
  EXPECT_EQ(GroundHeightMap(pts).numSkippedOutOfRange(), 0u);
  pts.emplace_back(2e6, 0.0, 0.0);
  pts.emplace_back(0.0, -3e6, 0.0);
  pts.emplace_back(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);  // not finite: not counted
  EXPECT_EQ(GroundHeightMap(pts).numSkippedOutOfRange(), 2u);
  GroundHeightParams p;
  p.resolution = 10.0;  // 2e6 m is 2e5 cells: in range now
  EXPECT_EQ(GroundHeightMap(pts, p).numSkippedOutOfRange(), 0u);
}

TEST(GroundHeightMap, ApproxBytesIsTheSizeOfWhatIsHeld) {
  GroundHeightMap g(surface(0, 20, 0, 20, 0.1, [](double, double) { return 0.0; }));
  ASSERT_EQ(g.numCells(), 400u);
  // 400 cells, one candidate each: at least a node (48) and a candidate array (16) per cell
  EXPECT_GE(g.approxBytes(), 400u * 64u);
  EXPECT_LE(g.approxBytes(), 400u * 64u + 400u * 8u * 4u);  // plus the bucket array
}

// A hole in a sloped ground must not make the height jump: the cells that remain are blended
// with their own planes, exact on a constant slope with any subset of cells.
TEST(GroundHeightMap, HolesOnASlopeDoNotJump) {
  for (double s : {0.1, 0.3})
    for (int hole = 1; hole <= 2; ++hole) {
      const auto f = [&](double x, double y) { return 0.4 + s * x + 0.5 * s * y; };
      Pts pts = surface(0, 10, 0, 10, 0.1, f);
      pts.erase(std::remove_if(pts.begin(), pts.end(),
                               [&](const Eigen::Vector3d& p) {
                                 return p.y() >= 4 && p.y() < 5 && p.x() >= 4 && p.x() < 4 + hole;
                               }),
                pts.end());
      GroundHeightMap g(pts);
      double worst = 0.0, jump = 0.0;
      for (double x = 2.0; x < 8.0; x += 0.037)
        for (double y = 2.0; y < 8.0; y += 0.043) {
          const auto h = g.heightAt(x, y, f(x, y));
          ASSERT_TRUE(h.has_value()) << x << " " << y;
          worst = std::max(worst, std::fabs(*h - f(x, y)));
          jump = std::max(jump, std::fabs(*g.heightAt(x + 0.01, y, f(x, y)) - *h));
        }
      EXPECT_LT(worst, 0.02) << "slope " << s << " hole " << hole;
      EXPECT_LT(jump, s * 0.01 + 0.002) << "slope " << s << " hole " << hole;
    }
}

// A 3 m wall at 0.02 m spacing on ground at 0.1 m spacing: far more wall points than ground
// points in the cell; the ground under and beside it is still the ground.
TEST(GroundHeightMap, DenseWallDoesNotLiftTheGround) {
  Pts pts = surface(0, 8, 0, 8, 0.1, [](double, double) { return 0.0; });
  for (double y = 2.0; y < 4.0; y += 0.02)
    for (double z = 0.02; z <= 3.0; z += 0.02) pts.emplace_back(3.02, y, z);
  GroundHeightMap g(pts);
  for (double x : {2.5, 3.0, 3.5, 4.5})
    for (double y : {2.3, 3.0, 3.5, 3.9}) {
      const auto h = g.heightAt(x, y, 0.0);
      ASSERT_TRUE(h.has_value());
      EXPECT_NEAR(*h, 0.0, 0.03) << x << " " << y;
    }
}

// Flat ground with Gaussian noise: no bias, at any map density.
TEST(GroundHeightMap, NoisyFlatGroundHasNoBiasAtAnyDensity) {
  for (int per_cell : {20, 100, 1000}) {
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(0.0, 10.0);
    std::normal_distribution<double> n(0.0, 0.03);
    Pts pts;
    for (int i = 0; i < 100 * per_cell; ++i) pts.emplace_back(u(rng), u(rng), n(rng));
    GroundHeightMap g(pts);
    double sum = 0.0;
    int cnt = 0;
    for (double x = 1.0; x < 9.0; x += 0.25)
      for (double y = 1.0; y < 9.0; y += 0.25) {
        const auto h = g.heightAt(x, y, 0.0);
        ASSERT_TRUE(h.has_value());
        sum += *h;
        ++cnt;
      }
    EXPECT_LT(std::fabs(sum / cnt), 0.01) << per_cell << " points per cell";
  }
}

// A few points far below the ground (fewer than min_points in a cell) are noise: they do not
// move the ground height.
TEST(GroundHeightMap, PointsFarBelowTheGroundDoNotMoveIt) {
  Pts pts = surface(0, 8, 0, 8, 0.1, [](double, double) { return 0.0; });
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int cx = 0; cx < 8; ++cx)
    for (int cy = 0; cy < 8; ++cy)
      for (int k = 0; k < 3; ++k) pts.emplace_back(cx + u(rng), cy + u(rng), -0.5 - 0.01 * k);
  GroundHeightMap g(pts);
  for (double x = 1.0; x < 7.0; x += 0.37)
    for (double y = 1.0; y < 7.0; y += 0.41) EXPECT_NEAR(*g.heightAt(x, y, 0.0), 0.0, 0.02);
}

// Ceiling underside at 3.0 m, floor above at 3.3 m, and one cell without the 3.3 surface: with
// the hint on the upper floor, the cell with only the 3.0 surface contributes nothing; the
// result never lies between the two surfaces.
TEST(GroundHeightMap, InterpolationDoesNotBlendTwoSurfaces) {
  Pts pts = surface(0, 8, 0, 8, 0.1, [](double, double) { return 3.0; });
  Pts upper = surface(0, 8, 0, 8, 0.1, [](double, double) { return 3.3; });
  upper.erase(std::remove_if(upper.begin(), upper.end(),
                             [](const Eigen::Vector3d& p) { return p.x() >= 4 && p.x() < 5 && p.y() >= 4 && p.y() < 5; }),
              upper.end());
  append(pts, upper);
  GroundHeightMap g(pts);
  for (double x = 2.0; x < 7.0; x += 0.037)
    for (double y = 2.0; y < 7.0; y += 0.043) {
      const auto h = g.heightAt(x, y, 3.3);
      ASSERT_TRUE(h.has_value()) << x << " " << y;
      EXPECT_NEAR(*h, 3.3, 1e-6) << x << " " << y;  // the ground is exactly flat: 3.3 and nothing in between
    }
}

// The anchor of a reseed query comes from the cell that contains (x, y) when it has a
// candidate; a neighbour's level counts only when that cell has none.
TEST(GroundHeightMap, HeightNearestPrefersTheContainingCell) {
  Pts pts = surface(0, 1, 0, 1, 0.1, [](double, double) { return 0.0; });
  append(pts, surface(1, 2, 0, 1, 0.1, [](double, double) { return 3.0; }));
  GroundHeightMap g(pts);
  EXPECT_NEAR(*g.heightNearest(0.9, 0.5, 3.0), 0.0, 1e-9);   // in the level-0 cell, guess 3
  EXPECT_NEAR(*g.heightNearest(1.1, 0.5, 0.0), 3.0, 1e-9);   // in the level-3 cell, guess 0
  // a cell with no candidate takes its neighbours' nearest level
  GroundHeightMap h(surface(0, 1, 0, 1, 0.1, [](double, double) { return 5.0; }));
  EXPECT_NEAR(*h.heightNearest(1.1, 0.5, 0.0), 5.0, 1e-9);
}

TEST(GroundHeightMap, ParameterValidation) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NO_THROW(validate(GroundHeightParams{}));
  auto bad = [](GroundHeightParams p, const char* field) {
    try {
      validate(p);
      ADD_FAILURE() << "accepted bad " << field;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find(field), std::string::npos) << e.what();
    }
  };
  GroundHeightParams p;
  p.resolution = 0.0; bad(p, "resolution");
  p = {}; p.resolution = nan; bad(p, "resolution");
  p = {}; p.min_points = 0; bad(p, "min_points");
  p = {}; p.layer_gap = 0.0; bad(p, "layer_gap");
  p = {}; p.max_step = 0.0; bad(p, "max_step");
  p = {}; p.max_step = nan; bad(p, "max_step");
  p = {}; p.band = -0.1; bad(p, "band");
  p = {}; p.band = nan; bad(p, "band");
  p = {}; p.resolution = -1.0;
  EXPECT_THROW(GroundHeightMap(Pts{}, p), std::invalid_argument);
}
