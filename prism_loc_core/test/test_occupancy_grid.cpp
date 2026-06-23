#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/occupancy_grid.hpp"
using namespace prism_loc_core;

static GridMap makeGrid(int w, int h, double res) {
  GridMap g; g.width = w; g.height = h; g.resolution = res;
  g.origin_x = 0.0; g.origin_y = 0.0; g.data.assign(w * h, 0);
  return g;
}

TEST(Grid, WorldMapRoundTrip) {
  GridMap g = makeGrid(10, 10, 0.5);
  int mx, my; ASSERT_TRUE(worldToMap(g, 1.25, 2.75, mx, my));
  EXPECT_EQ(mx, 2); EXPECT_EQ(my, 5);
  double wx, wy; mapToWorld(g, 2, 5, wx, wy);
  EXPECT_NEAR(wx, 1.25, 1e-9); EXPECT_NEAR(wy, 2.75, 1e-9);  // cell center
}

TEST(Grid, OutOfBounds) {
  GridMap g = makeGrid(4, 4, 1.0);
  int mx, my; EXPECT_FALSE(worldToMap(g, -0.1, 0.0, mx, my));
  EXPECT_FALSE(worldToMap(g, 4.0, 0.0, mx, my));
}

TEST(LikelihoodField, DistancesFromSingleOccupiedCell) {
  GridMap g = makeGrid(5, 5, 1.0);
  g.data[2 * 5 + 2] = 100;  // occupied cell at (mx=2,my=2) -> center world (2.5,2.5)
  LikelihoodField lf(g, /*occupied_threshold=*/50, /*max_dist=*/10.0);
  EXPECT_NEAR(lf.distanceAt(2.5, 2.5), 0.0, 1e-6);
  EXPECT_NEAR(lf.distanceAt(3.5, 2.5), 1.0, 1e-6);                 // one cell east
  EXPECT_NEAR(lf.distanceAt(4.5, 2.5), 2.0, 1e-6);                 // two cells east
  EXPECT_NEAR(lf.distanceAt(3.5, 3.5), std::sqrt(2.0), 1e-6);      // diagonal
}

TEST(LikelihoodField, ClampsAndEmptyMap) {
  GridMap g = makeGrid(5, 5, 1.0);  // no occupied cells
  LikelihoodField lf(g, 50, 3.0);
  EXPECT_NEAR(lf.distanceAt(2.5, 2.5), 3.0, 1e-6);  // clamped to max_dist
}
