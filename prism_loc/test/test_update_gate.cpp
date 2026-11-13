#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "prism_loc/update_gate.hpp"

using prism_loc::UpdateGate;
using prism_loc::UpdateReason;
using prism_loc::UpdateStamp;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The gate as it was before the time-based option: forced || dd >= min_d || da >= min_a.
bool legacyGate(bool forced, double dd, double da, double min_d, double min_a) {
  return forced || dd >= min_d || da >= min_a;
}
}  // namespace

TEST(UpdateGate, PeriodZeroReproducesTheMotionGateExactly) {
  const double ds[] = {0.0, 0.005, 0.01, 0.1, 0.1999999, 0.2, 0.2000001, 1.0, kNaN};
  const double as[] = {0.0, 0.005, 0.01, 0.1, 0.1999999, 0.2, 0.2000001, 3.0, kNaN};
  const double elapsed[] = {-5.0, 0.0, 0.05, 1.0, 1e6, kNaN, kInf, -kInf};
  const double thresholds[][2] = {{0.2, 0.2}, {0.0, 0.0}, {0.0, 0.2}, {1.0, 0.05}};
  for (const auto& th : thresholds) {
    UpdateGate g;
    g.min_d = th[0];
    g.min_a = th[1];
    g.min_period_s = 0.0;
    for (bool forced : {false, true})
      for (double dd : ds)
        for (double da : as)
          for (double t : elapsed) {
            const auto d = g.decide(forced, dd, da, t);
            EXPECT_EQ(d.update, legacyGate(forced, dd, da, th[0], th[1]))
                << "forced=" << forced << " dd=" << dd << " da=" << da << " t=" << t;
            EXPECT_NE(d.reason, UpdateReason::Time);  // never a timed update when off
            EXPECT_EQ(d.update, d.reason != UpdateReason::None);
          }
  }
}

TEST(UpdateGate, ReasonOrderIsForcedThenMotionThenTime) {
  UpdateGate g;
  g.min_period_s = 1.0;
  EXPECT_EQ(g.decide(true, 0.5, 0.0, 2.0).reason, UpdateReason::Forced);
  EXPECT_EQ(g.decide(false, 0.5, 0.0, 2.0).reason, UpdateReason::Motion);
  EXPECT_EQ(g.decide(false, 0.0, 0.5, 2.0).reason, UpdateReason::Motion);
  EXPECT_EQ(g.decide(false, 0.05, 0.0, 2.0).reason, UpdateReason::Time);
  EXPECT_EQ(g.decide(false, 0.05, 0.0, 0.5).reason, UpdateReason::None);
}

TEST(UpdateGate, TimedUpdateFiresAfterThePeriodWithSmallMotion) {
  UpdateGate g;  // defaults: 0.2 m, 0.2 rad
  g.min_period_s = 1.0;
  EXPECT_FALSE(g.decide(false, 0.05, 0.0, 0.99).update);  // period not reached
  const auto d = g.decide(false, 0.05, 0.0, 1.0);          // exactly the period counts
  EXPECT_TRUE(d.update);
  EXPECT_EQ(d.reason, UpdateReason::Time);
  EXPECT_TRUE(g.decide(false, 0.0, 0.02, 1.5).update);  // a small turn counts as well
}

TEST(UpdateGate, StandingRobotNeverGetsATimedUpdate) {
  UpdateGate g;
  g.min_period_s = 0.5;
  const double fd = UpdateGate::kStandingFraction * g.min_d;  // 0.01 m
  const double fa = UpdateGate::kStandingFraction * g.min_a;  // 0.01 rad
  for (double t : {0.5, 1.0, 60.0, 1e9}) {
    EXPECT_FALSE(g.decide(false, 0.0, 0.0, t).update) << t;
    EXPECT_FALSE(g.decide(false, 0.99 * fd, 0.99 * fa, t).update) << t;
    EXPECT_TRUE(g.decide(false, fd, 0.0, t).update) << t;  // at the fraction: moved
    EXPECT_TRUE(g.decide(false, 0.0, fa, t).update) << t;
  }
}

TEST(UpdateGate, StandingFractionIsFivePercentOfTheThresholds) {
  EXPECT_DOUBLE_EQ(UpdateGate::kStandingFraction, 0.05);
  UpdateGate g;
  g.min_d = 1.0;
  g.min_a = 0.5;
  g.min_period_s = 1.0;
  EXPECT_FALSE(g.decide(false, 0.049, 0.024, 2.0).update);
  EXPECT_TRUE(g.decide(false, 0.05, 0.0, 2.0).update);
  EXPECT_TRUE(g.decide(false, 0.0, 0.025, 2.0).update);
}

TEST(UpdateGate, ZeroThresholdsMeanEveryScanUpdatesAsToday) {
  UpdateGate g;
  g.min_d = 0.0;
  g.min_a = 0.0;
  g.min_period_s = 1.0;
  const auto d = g.decide(false, 0.0, 0.0, 0.0);
  EXPECT_TRUE(d.update);
  EXPECT_EQ(d.reason, UpdateReason::Motion);  // the motion gate already passes
}

TEST(UpdateGate, ForcedAlwaysUpdates) {
  UpdateGate g;
  g.min_period_s = 1.0;
  for (double t : {-10.0, 0.0, 1.0, kNaN, kInf}) {
    const auto d = g.decide(true, 0.0, 0.0, t);
    EXPECT_TRUE(d.update) << t;
    EXPECT_EQ(d.reason, UpdateReason::Forced) << t;
  }
}

TEST(UpdateGate, BackwardsOrNonFiniteElapsedNeverGivesATimedUpdate) {
  UpdateGate g;
  g.min_period_s = 1.0;
  for (double t : {-0.001, -1.0, -1e9, -kInf, kNaN, kInf})
    EXPECT_FALSE(g.decide(false, 0.05, 0.0, t).update) << t;
  // Motion and forced updates do not depend on the clock.
  EXPECT_TRUE(g.decide(false, 0.5, 0.0, -3.0).update);
  EXPECT_TRUE(g.decide(false, 0.5, 0.0, kNaN).update);
}

TEST(UpdateGate, NonFiniteMotionNeverGivesATimedUpdate) {
  UpdateGate g;
  g.min_period_s = 1.0;
  EXPECT_FALSE(g.decide(false, kNaN, kNaN, 5.0).update);
}

TEST(UpdateStamp, FirstUpdateEverHasNoElapsedTime) {
  UpdateStamp s;
  EXPECT_TRUE(std::isnan(s.elapsed(100.0)));
  UpdateGate g;
  g.min_period_s = 1.0;
  EXPECT_FALSE(g.decide(false, 0.05, 0.0, s.elapsed(100.0)).update);  // waits for a real update
  s.mark(100.0);
  EXPECT_DOUBLE_EQ(s.elapsed(101.5), 1.5);
}

TEST(UpdateStamp, StuckClockNeverReachesThePeriod) {
  UpdateStamp s;
  s.mark(50.0);
  UpdateGate g;
  g.min_period_s = 1.0;
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(s.elapsed(50.0), 0.0);
    EXPECT_FALSE(g.decide(false, 0.05, 0.0, s.elapsed(50.0)).update);
  }
}

TEST(UpdateStamp, BackwardsStampReanchorsSoTheGateIsNotWedged) {
  UpdateStamp s;
  s.mark(1000.0);  // last update, then the bag loops and the stamps restart near 10
  UpdateGate g;
  g.min_period_s = 1.0;
  // The jump back gives elapsed 0: no timed update, and the stored stamp moves to 10.
  EXPECT_DOUBLE_EQ(s.elapsed(10.0), 0.0);
  EXPECT_FALSE(g.decide(false, 0.05, 0.0, s.elapsed(10.0)).update);
  EXPECT_DOUBLE_EQ(s.last(), 10.0);
  // One period after the restart the timed update fires again.
  EXPECT_FALSE(g.decide(false, 0.05, 0.0, s.elapsed(10.9)).update);
  const auto d = g.decide(false, 0.05, 0.0, s.elapsed(11.0));
  EXPECT_TRUE(d.update);
  EXPECT_EQ(d.reason, UpdateReason::Time);
  s.mark(11.0);
  EXPECT_NEAR(s.elapsed(11.4), 0.4, 1e-9);
}

TEST(UpdateStamp, NonFiniteStampsAreIgnored) {
  UpdateStamp s;
  s.mark(5.0);
  EXPECT_TRUE(std::isnan(s.elapsed(kNaN)));
  EXPECT_TRUE(std::isnan(s.elapsed(kInf)));
  EXPECT_DOUBLE_EQ(s.last(), 5.0);
  s.mark(kNaN);
  EXPECT_DOUBLE_EQ(s.last(), 5.0);
}
