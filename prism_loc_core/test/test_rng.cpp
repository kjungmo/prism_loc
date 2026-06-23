#include <gtest/gtest.h>
#include "prism_loc_core/rng.hpp"
using namespace prism_loc_core;

TEST(Rng, SameSeedSameSequence) {
  Rng a(123), b(123);
  for (int i = 0; i < 5; ++i) EXPECT_DOUBLE_EQ(a.uniform(), b.uniform());
}
TEST(Rng, DifferentSeedDiffers) {
  Rng a(1), b(2);
  bool any_diff = false;
  for (int i = 0; i < 5; ++i) any_diff |= (a.uniform() != b.uniform());
  EXPECT_TRUE(any_diff);
}
TEST(Rng, GaussianMeanApprox) {
  Rng r(7);
  double s = 0; const int N = 20000;
  for (int i = 0; i < N; ++i) s += r.gaussian(2.0, 1.0);
  EXPECT_NEAR(s / N, 2.0, 0.05);
}
