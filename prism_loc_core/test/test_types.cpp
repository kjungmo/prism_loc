#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/types.hpp"
using namespace prism_loc_core;

TEST(Types, NormalizeAngleWraps) {
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(-3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(0.5), 0.5, 1e-9);
}

TEST(Types, ComposeAndInverseRoundTrip) {
  Pose2D a{1.0, 2.0, M_PI / 4};
  Pose2D b{0.5, -0.3, M_PI / 6};
  Pose2D c = compose(a, b);
  Pose2D back = compose(inverse(a), c);  // a^-1 * (a*b) == b
  EXPECT_NEAR(back.x, b.x, 1e-9);
  EXPECT_NEAR(back.y, b.y, 1e-9);
  EXPECT_NEAR(normalizeAngle(back.yaw - b.yaw), 0.0, 1e-9);
}

TEST(Types, TransformPointRotatesAndTranslates) {
  Pose2D T{1.0, 0.0, M_PI / 2};            // +90°, then +x by 1
  Eigen::Vector2d p(1.0, 0.0);
  Eigen::Vector2d q = transformPoint(T, p);  // (1,0) rotated 90° -> (0,1), +translation -> (1,1)
  EXPECT_NEAR(q.x(), 1.0, 1e-9);
  EXPECT_NEAR(q.y(), 1.0, 1e-9);
}
