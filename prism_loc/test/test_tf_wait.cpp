#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include "prism_loc/tf_wait_rule.hpp"
using prism_loc::TfWait;
using prism_loc::TfWaitRule;

namespace {
using ClockModel = std::function<double(double)>;  // ROS seconds at a given steady second

struct Result {
  bool resolved;     // the transform became available before the wait gave up
  double waited;     // steady seconds from start to resolution or give-up
  double ros_waited; // ROS seconds over the same span
};

std::int64_t toNs(double s) { return static_cast<std::int64_t>(std::llround(s * 1e9)); }

// One wait starting at steady time t0, simulated the way lookupTransformWait runs it:
// check the transform, then ask the rule, then sleep one poll period (10 ms in the
// node, like tf2_ros; `poll` here). The transform appears at steady time `available`.
// The rule object persists across waits, as the node's member does.
Result waitFrom(TfWaitRule& rule, const ClockModel& ros_at, double t0, double available = 1e9,
                double poll = 0.01) {
  TfWait w(rule, t0, toNs(ros_at(t0)));
  const double r0 = ros_at(t0);
  for (int k = 0; k < 100000; ++k) {
    const double t = t0 + k * poll;
    if (t >= available) return {true, t - t0, ros_at(t) - r0};
    if (w.stop(t, toNs(ros_at(t)))) return {false, t - t0, ros_at(t) - r0};
  }
  return {false, 1e9, 1e9};
}

// A /clock published in steps of `step_s` of sim time at real-time factor `factor`.
ClockModel stepped(double factor, double step_s) {
  return [factor, step_s](double t) { return std::floor(factor * t / step_s + 1e-9) * step_s; };
}
}  // namespace

TEST(TfWait, LiveClockWaitsTheBudgetWithoutGrace) {
  TfWaitRule rule;
  const Result r = waitFrom(rule, [](double t) { return t; }, 5.0);
  EXPECT_FALSE(r.resolved);
  EXPECT_NEAR(r.waited, 0.1, 1e-6);
}

TEST(TfWait, SmoothClockGetsNoGraceAfterTheBudget) {
  // A transform arriving 15 ms after the budget on a live clock is not waited for: a
  // live robot behaves as with tf2_ros.
  TfWaitRule rule;
  const Result r = waitFrom(rule, [](double t) { return t; }, 5.0, 5.115);
  EXPECT_FALSE(r.resolved);
}

TEST(TfWait, TenthSpeedClockGetsTheBudgetInRosTime) {
  TfWaitRule rule;
  const Result r = waitFrom(rule, [](double t) { return 0.1 * t; }, 5.0);
  EXPECT_GE(r.ros_waited, 0.1 - 1e-9);
  EXPECT_LE(r.waited, 1.0 + 1e-9);
}

TEST(TfWait, CoarseClockGetsItsFullBudgetPlusTheStepGrace) {
  // 10 Hz sim-time steps: one step every 0.2 s of wall time at 0.5x, every 0.5 s at
  // 0.2x. Neither may look frozen inside a wait; each gets 0.1 s of ROS time, then
  // the 30 ms step grace.
  for (double factor : {0.5, 0.2}) {
    TfWaitRule rule;
    const ClockModel clk = stepped(factor, 0.1);
    for (double t0 : {3.005, 3.375, 3.695}) {  // several waits, starting between steps
      const Result r = waitFrom(rule, clk, t0);
      EXPECT_FALSE(r.resolved);
      EXPECT_NEAR(r.ros_waited, 0.1, 1e-9) << "factor " << factor << " t0 " << t0;
      EXPECT_LE(r.waited, 0.1 / factor + 0.03 + 0.011) << "factor " << factor << " t0 " << t0;
    }
  }
}

TEST(TfWait, SteppedClockResolvesATransformSentWithTheStep) {
  // 0.5x, 10 Hz steps: the clock reaches the budget at steady 3.2; the transform
  // published with that step arrives 15 ms later and is still found.
  TfWaitRule rule;
  const Result r = waitFrom(rule, stepped(0.5, 0.1), 3.005, 3.215);
  EXPECT_TRUE(r.resolved);
}

TEST(TfWait, StepGraceNeverExceedsTheCap) {
  // A 0.1 s step every 0.98 s of wall time: the budget is reached at 0.98 s and the
  // grace would end at 1.01 s, but the 1 s steady cap ends the wait first.
  TfWaitRule rule;
  const ClockModel clk = [](double t) { return std::floor((t - 9.999) / 0.98 + 1e-9) * 0.1; };
  const Result r = waitFrom(rule, clk, 10.0);
  EXPECT_FALSE(r.resolved);
  EXPECT_NEAR(r.waited, 1.0, 1e-6);
}

TEST(TfWait, FrozenFromTheStartHoldsOneWaitThenNone) {
  TfWaitRule rule;
  const ClockModel frozen = [](double) { return 0.0; };  // use_sim_time, no /clock
  EXPECT_NEAR(waitFrom(rule, frozen, 2.0).waited, 1.0, 1e-6);
  EXPECT_NEAR(waitFrom(rule, frozen, 3.1).waited, 0.0, 1e-9);
}

TEST(TfWait, ClockThatStartsThenFreezesStopsAtTheCapThenReturnsAtOnce) {
  TfWaitRule rule;
  const ClockModel clk = [](double t) { return std::min(t, 2.03); };
  const Result first = waitFrom(rule, clk, 2.0);
  EXPECT_NEAR(first.waited, 1.0, 1e-6);
  EXPECT_NEAR(first.ros_waited, 0.03, 1e-9);
  EXPECT_NEAR(waitFrom(rule, clk, 3.2).waited, 0.0, 1e-9);
}

TEST(TfWait, ClockJumpingBackwardEndsTheWait) {
  // A looping bag rewinds /clock: the wait ends at the first poll that sees it.
  TfWaitRule rule;
  const Result r = waitFrom(rule, [](double t) { return t < 5.015 ? t : t - 30.0; }, 5.0);
  EXPECT_NEAR(r.waited, 0.02, 1e-6);
}
