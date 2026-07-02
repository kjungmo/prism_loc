#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/rng.hpp"
using namespace prism_loc_core;

TEST(Motion, ZeroDeltaZeroNoiseUnchanged) {
  OdometryMotionModel mm(MotionParams{0,0,0,0});
  Rng rng(1);
  Pose2D p{3.0, -1.0, 0.5};
  Pose2D o{10.0, 10.0, 1.2};
  Pose2D out = mm.sample(p, o, o, rng);  // prev == cur
  EXPECT_NEAR(out.x, p.x, 1e-12);
  EXPECT_NEAR(out.y, p.y, 1e-12);
  EXPECT_NEAR(out.yaw, p.yaw, 1e-12);
}

TEST(Motion, PureTranslationNoNoiseMovesAlongHeading) {
  OdometryMotionModel mm(MotionParams{0,0,0,0});
  Rng rng(1);
  Pose2D particle{0.0, 0.0, M_PI / 2};            // facing +y
  Pose2D prev{0.0, 0.0, 0.0}, cur{1.0, 0.0, 0.0}; // odom moved +1 in x, no rotation
  Pose2D out = mm.sample(particle, prev, cur, rng);
  EXPECT_NEAR(out.x, 0.0, 1e-9);                  // moved 1 along particle heading (+y)
  EXPECT_NEAR(out.y, 1.0, 1e-9);
  EXPECT_NEAR(out.yaw, M_PI / 2, 1e-9);
}

TEST(Motion, NoisySamplesMeanApproxCommanded) {
  OdometryMotionModel mm(MotionParams{0.02,0.02,0.02,0.02});
  Rng rng(5);
  Pose2D prev{0,0,0}, cur{1.0,0,0};
  double sx = 0; const int N = 5000;
  for (int i = 0; i < N; ++i) sx += mm.sample(Pose2D{0,0,0}, prev, cur, rng).x;
  EXPECT_NEAR(sx / N, 1.0, 0.05);
}
