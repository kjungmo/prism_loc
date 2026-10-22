#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "prism_loc/ndt_status.hpp"
using prism_loc::IgnoredSensorTf;
using prism_loc::MatchMonitor;
using prism_loc::defaultMinNeffFraction;
using prism_loc::ignoredSensorTf;

namespace {
void add(MatchMonitor& m, double fraction) { m.update(false, fraction); }
}  // namespace

// laser2d keeps its n_eff check at the documented 0.005; ndt3d has it off by default.
TEST(NdtStatus, MinNeffFractionDefaultPerBackend) {
  EXPECT_DOUBLE_EQ(defaultMinNeffFraction("laser2d"), 0.005);
  EXPECT_DOUBLE_EQ(defaultMinNeffFraction("ndt3d"), 0.0);
  EXPECT_DOUBLE_EQ(prism_loc::kDefaultMinMatchFraction, 0.65);
}

// Row 1 of the table in ndt_status.hpp: nothing to judge before the first update.
TEST(MatchMonitor, EmptyIsNeverLow) {
  MatchMonitor m;
  EXPECT_EQ(m.size(), 0u);
  EXPECT_EQ(m.mean(), 0.0);
  EXPECT_FALSE(m.low(0.65));
  EXPECT_FALSE(m.low(1.0));
}

// Rows 2 and 3, with the threshold itself on the "not low" side.
TEST(MatchMonitor, LowOnlyBelowTheThreshold) {
  MatchMonitor m;
  add(m, 0.5);
  EXPECT_EQ(m.size(), 1u);
  EXPECT_DOUBLE_EQ(m.mean(), 0.5);
  EXPECT_FALSE(m.low(0.5));          // exactly at the threshold
  EXPECT_FALSE(m.low(0.4999));       // above it
  EXPECT_TRUE(m.low(0.5001));        // below it
  // a scan with no usable point scores 0 and counts as a mismatch
  MatchMonitor none;
  add(none, 0.0);
  EXPECT_TRUE(none.low(0.65));
}

// Row 4: threshold 0 switches the check off whatever the values are.
TEST(MatchMonitor, ZeroThresholdIsOff) {
  MatchMonitor m;
  add(m, 0.0);
  EXPECT_FALSE(m.low(0.0));
  EXPECT_FALSE(m.low(-1.0));
}

// The mean is over the last kWindow (5) updates: one bad scan among good ones does not
// trip it, five in a row do, and old values leave the window.
TEST(MatchMonitor, MeanOfTheLastFive) {
  ASSERT_EQ(MatchMonitor::kWindow, 5u);
  MatchMonitor m;
  for (double v : {0.8, 0.8, 0.8, 0.8}) add(m, v);
  EXPECT_DOUBLE_EQ(m.mean(), 0.8);   // fewer than five: the mean of what there is
  add(m, 0.2);                        // one occluded scan
  EXPECT_EQ(m.size(), 5u);
  EXPECT_NEAR(m.mean(), 0.68, 1e-12);
  EXPECT_FALSE(m.low(0.65));
  add(m, 0.2);                        // the oldest 0.8 leaves: (0.8 * 3 + 0.2 * 2) / 5
  EXPECT_EQ(m.size(), 5u);
  EXPECT_NEAR(m.mean(), 0.56, 1e-12);
  EXPECT_TRUE(m.low(0.65));
  for (int i = 0; i < 3; ++i) add(m, 0.2);
  EXPECT_NEAR(m.mean(), 0.2, 1e-12);  // nothing of the 0.8 values is left
  // recovery: five good updates and the warning is gone, with no trace of the bad ones
  for (int i = 0; i < 4; ++i) add(m, 0.9);
  EXPECT_NEAR(m.mean(), 0.76, 1e-12);
  EXPECT_FALSE(m.low(0.65));
  add(m, 0.9);
  EXPECT_NEAR(m.mean(), 0.9, 1e-12);
}

// The update forced by a reseed (/initialpose) is not judged, whatever its own match is,
// and it empties the window: no value from before it is judged either.
TEST(MatchMonitor, ReseedForgetsEverythingAndIsNotJudged) {
  MatchMonitor m;
  for (int i = 0; i < 7; ++i) add(m, 0.1);
  EXPECT_TRUE(m.low(0.65));
  m.update(true, 0.05);              // the forced update scores badly itself
  EXPECT_EQ(m.size(), 0u);
  EXPECT_EQ(m.mean(), 0.0);
  EXPECT_FALSE(m.low(0.65));
  add(m, 0.9);
  EXPECT_EQ(m.size(), 1u);
  EXPECT_DOUBLE_EQ(m.mean(), 0.9);   // not averaged with 0.05 or with the 0.1 values still in the buffer
  EXPECT_FALSE(m.low(0.65));
  // a reseed on an empty or partly filled window is fine too
  MatchMonitor fresh;
  fresh.update(true, 1.0);
  EXPECT_EQ(fresh.size(), 0u);
  add(fresh, 0.3);
  fresh.update(true, 1.0);
  EXPECT_EQ(fresh.size(), 0u);
  EXPECT_FALSE(fresh.low(0.65));
}

// After a reseed the pose is not verified until n updates have been judged; a reseed
// starts the count again; n = 0 switches the check off.
TEST(MatchMonitor, VerifiedAfterNUpdatesSinceTheReseed) {
  MatchMonitor m;
  EXPECT_EQ(m.sinceReseed(), 0u);
  EXPECT_FALSE(m.verified(15));
  EXPECT_TRUE(m.verified(0));
  m.update(true, 0.9);                    // the reseed update itself does not count
  EXPECT_EQ(m.sinceReseed(), 0u);
  for (int i = 0; i < 14; ++i) add(m, 0.9);
  EXPECT_EQ(m.sinceReseed(), 14u);
  EXPECT_FALSE(m.verified(15));           // one short
  EXPECT_TRUE(m.verified(14));
  add(m, 0.9);
  EXPECT_TRUE(m.verified(15));
  for (int i = 0; i < 100; ++i) add(m, 0.9);
  EXPECT_EQ(m.sinceReseed(), 115u);       // not capped at the window
  EXPECT_TRUE(m.verified(15));
  m.update(true, 0.9);                    // /initialpose again
  EXPECT_EQ(m.sinceReseed(), 0u);
  EXPECT_FALSE(m.verified(15));
  EXPECT_FALSE(m.verified(1));
  add(m, 0.1);                            // a low match counts as judged: low() reports it
  EXPECT_TRUE(m.verified(1));
  EXPECT_TRUE(m.low(0.65));
  EXPECT_EQ(prism_loc::kDefaultMatchVerifyUpdates, 15);
}

// A level lidar whose TF z equals base_height: nothing that ndt3d ignores matters.
TEST(IgnoredSensorTf, LevelSensorAtBaseHeightIsFine) {
  EXPECT_FALSE(ignoredSensorTf(0.0, 0.0, 0.5, 0.5).any());
  EXPECT_FALSE(ignoredSensorTf(0.0, 0.0, 0.0, 0.0).any());
}

// Tolerances: 0.01 rad of tilt and 0.05 m of height, both inclusive.
TEST(IgnoredSensorTf, ToleranceOnBothSides) {
  EXPECT_FALSE(ignoredSensorTf(0.01, 0.0, 0.5, 0.5).tilt);
  EXPECT_TRUE(ignoredSensorTf(0.0101, 0.0, 0.5, 0.5).tilt);
  EXPECT_FALSE(ignoredSensorTf(0.0, -0.01, 0.5, 0.5).tilt);
  EXPECT_TRUE(ignoredSensorTf(0.0, -0.0101, 0.5, 0.5).tilt);
  EXPECT_TRUE(ignoredSensorTf(M_PI, 0.0, 0.5, 0.5).tilt);       // mounted upside down
  EXPECT_FALSE(ignoredSensorTf(0.0, 0.0, 0.5, 0.5).height);
  EXPECT_FALSE(ignoredSensorTf(0.0, 0.0, 0.549, 0.5).height);
  EXPECT_TRUE(ignoredSensorTf(0.0, 0.0, 0.551, 0.5).height);
  EXPECT_FALSE(ignoredSensorTf(0.0, 0.0, 0.451, 0.5).height);
  EXPECT_TRUE(ignoredSensorTf(0.0, 0.0, 0.449, 0.5).height);
  // the two findings are independent
  const IgnoredSensorTf tilt_only = ignoredSensorTf(0.2, 0.0, 0.5, 0.5);
  EXPECT_TRUE(tilt_only.tilt);
  EXPECT_FALSE(tilt_only.height);
  EXPECT_TRUE(tilt_only.any());
  const IgnoredSensorTf height_only = ignoredSensorTf(0.0, 0.0, 0.5, 0.0);  // base_height left at 0
  EXPECT_FALSE(height_only.tilt);
  EXPECT_TRUE(height_only.height);
  EXPECT_TRUE(height_only.any());
  // explicit tolerances replace the defaults
  EXPECT_FALSE(ignoredSensorTf(0.05, 0.0, 0.7, 0.5, 0.1, 0.3).any());
  EXPECT_TRUE(ignoredSensorTf(0.05, 0.0, 0.5, 0.5, 0.04, 0.3).tilt);
  EXPECT_TRUE(ignoredSensorTf(0.0, 0.0, 0.7, 0.5, 0.1, 0.1).height);
}

TEST(IgnoredSensorTf, NonFiniteCountsAsIgnoredPart) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(ignoredSensorTf(nan, 0.0, 0.5, 0.5).tilt);
  EXPECT_TRUE(ignoredSensorTf(0.0, nan, 0.5, 0.5).tilt);
  EXPECT_TRUE(ignoredSensorTf(0.0, 0.0, nan, 0.5).height);
  EXPECT_TRUE(ignoredSensorTf(0.0, 0.0, 0.5, nan).height);
}
