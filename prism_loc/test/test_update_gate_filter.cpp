#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include "prism_loc/update_gate.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "test_helpers.hpp"  // prism_loc_core/test: room grid + synthetic scan

// Why the time-based update has a "moved at all" rule, and what the gate allows. The
// particle filter is driven through UpdateGate::decide, as the node does it. No ROS.
using namespace prism_loc_core;
using prism_loc::UpdateGate;
using prism_loc::UpdateReason;

namespace {
constexpr double kScanDt = 0.1;  // s between scans
constexpr int kSteps = 100;

double xyVar(const ParticleFilter& pf) {
  Eigen::Matrix<double, 6, 6> cov;
  pf.estimate(&cov);
  return cov(0, 0) + cov(1, 1);
}

struct DriveResult {
  double after_first{0.0};  // xy variance after the first (forced) update
  double final_var{0.0};
  int timed{0}, motion{0};
};

// The robot moves `step_m` per scan along x (odometry and truth agree). `bypass`: update
// at every scan, ignoring the gate.
DriveResult drive(double step_m, bool bypass, double period_s) {
  const GridMap g = test::makeRoomGrid(60, 60, 0.1);
  const Pose2D sensor_in_base{0, 0, 0};
  Laser2DLikelihoodField model(g, LaserParams{});
  Rng rng(7);
  ParticleFilterParams pp;
  pp.min_particles = 800;
  pp.max_particles = 800;
  ParticleFilter pf(pp, rng);
  const Pose2D start{2.0, 3.0, 0.0};
  pf.initializeGaussian(start, Pose2D{0.3, 0.3, 0.1}, 800);
  OdometryMotionModel mm(MotionParams{0.2, 0.2, 0.2, 0.2});

  UpdateGate gate;
  gate.min_period_s = period_s;
  prism_loc::UpdateStamp last;
  Pose2D last_odom{0, 0, 0};

  DriveResult r;
  model.setScan(test::raycastScan(g, start, sensor_in_base, 180, 10.0));
  pf.predict(mm, last_odom, last_odom);  // the first update, forced like a reseed
  pf.correct(model);
  pf.resample();
  last.mark(0.0);
  r.after_first = xyVar(pf);

  for (int i = 1; i <= kSteps; ++i) {
    const double t = i * kScanDt;
    const Pose2D odom{step_m * i, 0.0, 0.0};
    const Pose2D truth{start.x + odom.x, start.y, 0.0};
    const double dd = std::hypot(odom.x - last_odom.x, odom.y - last_odom.y);
    const auto d = gate.decide(false, dd, std::fabs(odom.yaw - last_odom.yaw), last.elapsed(t));
    if (bypass || d.update) {
      if (d.reason == UpdateReason::Time) ++r.timed;
      if (d.reason == UpdateReason::Motion) ++r.motion;
      model.setScan(test::raycastScan(g, truth, sensor_in_base, 180, 10.0));
      pf.predict(mm, last_odom, odom);
      pf.correct(model);
      pf.resample();
      last_odom = odom;
      last.mark(t);
    }
  }
  r.final_var = xyVar(pf);
  return r;
}
}  // namespace

// (a) A standing robot, period on: the gate never opens, the particle set is untouched.
TEST(UpdateGateFilter, StandingRobotGetsNoUpdateAndKeepsItsSpread) {
  const DriveResult r = drive(0.0, false, 0.5);
  EXPECT_EQ(r.timed + r.motion, 0);
  EXPECT_DOUBLE_EQ(r.final_var, r.after_first);
}

// (b) The reason for the rule: the same standing loop updating at every scan (the rule
// bypassed) repeats correct + resample on one view and collapses the spread.
TEST(UpdateGateFilter, WithoutTheStandingRuleRepeatedUpdatesCollapseTheSpread) {
  const DriveResult r = drive(0.0, true, 0.5);
  std::printf("[measured] standing, update every scan: xy variance %.3e -> %.3e m^2 (x%.1e)\n",
              r.after_first, r.final_var, r.final_var / r.after_first);
  EXPECT_LT(r.final_var, 0.01 * r.after_first);
}

// (c) 0.01 m of odometry per scan (0.1 m/s), period 0.5 s: 0.05 m accumulates per period,
// above the 0.01 m standing fraction and below the 0.2 m motion threshold, so only timed
// updates run (20 in 100 scans). Each has real odometry motion, so the spread narrows by
// the scan's information but does not collapse. Measured (seed 7): 1.05e-3 -> 4.3e-4 m^2;
// the bound below is conservative.
TEST(UpdateGateFilter, CreepingRobotGetsTimedUpdatesAndKeepsASpread) {
  const DriveResult r = drive(0.01, false, 0.5);
  std::printf("[measured] creeping 0.01 m/scan, period 0.5 s: timed %d motion %d, "
              "xy variance %.3e -> %.3e m^2\n", r.timed, r.motion, r.after_first, r.final_var);
  EXPECT_EQ(r.timed, 20);
  EXPECT_EQ(r.motion, 0);
  EXPECT_GT(r.final_var, 0.1 * r.after_first);
  
}
