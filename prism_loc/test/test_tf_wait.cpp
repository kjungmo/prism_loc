#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include "prism_loc/tf_wait_rule.hpp"
using prism_loc::TfWaitRule;

namespace {
using ClockModel = std::function<double(double)>;  // ROS seconds at a given steady second

struct Stop { double steady, ros; };

// One wait starting at steady time t0, polled every millisecond as lookupTransformWait
// does (observe, then stop). The rule object persists across waits.
Stop waitFrom(TfWaitRule& rule, const ClockModel& ros_at, double t0) {
  const double r0 = ros_at(t0);
  rule.observe(t0, r0);
  for (int ms = 0; ms <= 5000; ++ms) {
    const double t = t0 + ms * 1e-3;
    const double r = ros_at(t);
    rule.observe(t, r);
    // The node takes the ROS difference in integer nanoseconds; do the same.
    const double dr = 1e-9 * std::round((r - r0) * 1e9);
    if (rule.stop(t - t0, dr, t)) return {t - t0, dr};
  }
  return {1e9, 1e9};
}

// A /clock published in steps of `step_s` of sim time at real-time factor `factor`.
ClockModel stepped(double factor, double step_s) {
  return [factor, step_s](double t) { return std::floor(factor * t / step_s + 1e-9) * step_s; };
}
}  // namespace

TEST(TfWait, LiveClockWaitsTheBudget) {
  TfWaitRule rule;
  const Stop s = waitFrom(rule, [](double t) { return t; }, 5.0);
  EXPECT_NEAR(s.steady, 0.1, 1.5e-3);
}

TEST(TfWait, TenthSpeedClockGetsTheBudgetInRosTime) {
  TfWaitRule rule;
  const Stop s = waitFrom(rule, [](double t) { return 0.1 * t; }, 5.0);
  EXPECT_GE(s.ros, 0.1 - 1e-3);
  EXPECT_LE(s.steady, 1.0 + 1e-3);
}

TEST(TfWait, CoarseClockGetsItsFullBudget) {
  // 10 Hz sim-time steps: one step every 0.2 s of wall time at 0.5x, every 0.5 s at
  // 0.2x. Neither may look frozen inside a wait; each must get 0.1 s of ROS time.
  for (double factor : {0.5, 0.2}) {
    TfWaitRule rule;
    const ClockModel clk = stepped(factor, 0.1);
    for (double t0 : {3.01, 3.37, 3.69}) {  // several waits, starting between steps
      const Stop s = waitFrom(rule, clk, t0);
      EXPECT_GE(s.ros, 0.1 - 1e-9) << "factor " << factor << " t0 " << t0;
      EXPECT_LE(s.steady, 0.1 / factor + 1e-3) << "factor " << factor << " t0 " << t0;
    }
  }
}

TEST(TfWait, FrozenFromTheStartHoldsOneWaitThenNone) {
  TfWaitRule rule;
  const ClockModel frozen = [](double) { return 0.0; };  // use_sim_time, no /clock
  const Stop first = waitFrom(rule, frozen, 2.0);
  EXPECT_NEAR(first.steady, 1.0, 1.5e-3);
  const Stop second = waitFrom(rule, frozen, 3.1);
  EXPECT_NEAR(second.steady, 0.0, 1e-9);
}

TEST(TfWait, ClockThatStartsThenFreezesStopsAtTheCapThenReturnsAtOnce) {
  TfWaitRule rule;
  const ClockModel clk = [](double t) { return std::min(t, 2.03); };
  const Stop first = waitFrom(rule, clk, 2.0);
  EXPECT_NEAR(first.steady, 1.0, 1.5e-3);
  EXPECT_NEAR(first.ros, 0.03, 1e-9);
  const Stop second = waitFrom(rule, clk, 3.2);
  EXPECT_NEAR(second.steady, 0.0, 1e-9);
}

TEST(TfWait, ClockJumpingBackwardEndsTheWait) {
  // A looping bag rewinds /clock: elapsed ROS time goes negative.
  TfWaitRule rule;
  const Stop s = waitFrom(rule, [](double t) { return t < 5.01 ? t : t - 30.0; }, 5.0);
  EXPECT_NEAR(s.steady, 0.01, 1.5e-3);
}
