#include <gtest/gtest.h>
#include <algorithm>
#include <functional>
#include "prism_loc/tf_wait_rule.hpp"
using prism_loc::TfWaitRule;

namespace {
struct Stop { double steady, ros; };
// Polls the rule every millisecond of steady time against a ROS-clock model (ROS
// seconds elapsed as a function of steady seconds elapsed), as lookupTransformWait does.
Stop waitWith(const std::function<double(double)>& ros_of_steady) {
  const TfWaitRule rule;
  for (int ms = 0; ms <= 5000; ++ms) {
    const double steady = ms * 1e-3;
    if (rule.stop(steady, ros_of_steady(steady))) return {steady, ros_of_steady(steady)};
  }
  return {1e9, 1e9};
}
}  // namespace

TEST(TfWait, LiveClockWaitsTheBudget) {
  const Stop s = waitWith([](double t) { return t; });
  EXPECT_NEAR(s.steady, 0.1, 1.5e-3);
}

TEST(TfWait, SlowSimClockStillGetsTheBudgetInRosTime) {
  // Playback at 0.25x: 0.1 s of ROS time takes 0.4 s, and the wait gives it all of it.
  const Stop quarter = waitWith([](double t) { return 0.25 * t; });
  EXPECT_NEAR(quarter.ros, 0.1, 1e-3);
  EXPECT_NEAR(quarter.steady, 0.4, 5e-3);
  // At 0.1x the 1 s steady cap and the budget coincide.
  const Stop tenth = waitWith([](double t) { return 0.1 * t; });
  EXPECT_GE(tenth.ros, 0.1 - 1e-3);
  EXPECT_LE(tenth.steady, 1.0 + 1e-3);
  // Slower than 0.1x the steady cap shortens the ROS-time budget.
  const Stop twentieth = waitWith([](double t) { return 0.05 * t; });
  EXPECT_NEAR(twentieth.steady, 1.0, 1.5e-3);
  EXPECT_NEAR(twentieth.ros, 0.05, 1e-3);
}

TEST(TfWait, FrozenClockGivesUpAfterTheBudgetOfSteadyTime) {
  const Stop s = waitWith([](double) { return 0.0; });  // use_sim_time, no /clock
  EXPECT_NEAR(s.steady, 0.1, 1.5e-3);
}

TEST(TfWait, ClockThatStartsThenFreezesStopsAtTheHardCap) {
  const Stop s = waitWith([](double t) { return std::min(t, 0.03); });
  EXPECT_NEAR(s.steady, 1.0, 1.5e-3);
  EXPECT_NEAR(s.ros, 0.03, 1e-9);
}

TEST(TfWait, ClockJumpingBackwardIsTreatedAsFrozen) {
  // A looping bag rewinds /clock: elapsed ROS time goes negative.
  const Stop s = waitWith([](double t) { return t < 0.01 ? t : -30.0; });
  EXPECT_NEAR(s.steady, 0.1, 1.5e-3);
}
