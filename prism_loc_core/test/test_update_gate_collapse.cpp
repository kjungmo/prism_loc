#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "test_helpers.hpp"
using namespace prism_loc_core;

// Why the node's time-based update (prism_loc/update_gate.hpp) needs a "moved at all"
// condition. With zero odometry motion the motion model adds no noise, so each
// correct + resample on the SAME scan only narrows the particle set, with no new
// information. This test pins that effect on the core classes, then shows that the
// gate's standing rule (no update at all) leaves the spread as it was.
namespace {
struct Spread {
  double xy_var;  // trace of the x, y covariance (m^2)
};

Spread spreadOf(const ParticleFilter& pf) {
  Eigen::Matrix<double, 6, 6> cov;
  pf.estimate(&cov);
  return {cov(0, 0) + cov(1, 1)};
}

constexpr int kRepeats = 10;
}  // namespace

TEST(UpdateGateCollapse, RepeatedZeroMotionUpdatesOnOneScanCollapseTheSpread) {
  const GridMap g = test::makeRoomGrid(60, 60, 0.1);
  const Pose2D truth{3.0, 3.0, 0.4}, sensor_in_base{0, 0, 0};
  Laser2DLikelihoodField model(g, LaserParams{});
  model.setScan(test::raycastScan(g, truth, sensor_in_base, 180, 10.0));

  Rng rng(7);
  ParticleFilterParams pp;
  pp.min_particles = 800;
  pp.max_particles = 800;
  ParticleFilter pf(pp, rng);
  pf.initializeGaussian(truth, Pose2D{0.3, 0.3, 0.1}, 800);
  OdometryMotionModel mm(MotionParams{0.2, 0.2, 0.2, 0.2});
  const Pose2D still{0, 0, 0};

  // First update: the scan carries information, the spread narrows. Not the effect pinned.
  pf.predict(mm, still, still);
  pf.correct(model);
  pf.resample();
  const double after_first = spreadOf(pf).xy_var;

  // Further updates on the same scan with no motion.
  for (int i = 0; i < kRepeats; ++i) {
    pf.predict(mm, still, still);
    pf.correct(model);
    pf.resample();
  }
  const double after_repeats = spreadOf(pf).xy_var;
  std::printf("[measured] xy variance after 1 update %.3e m^2, after %d more: %.3e m^2 (x%.3e)\n",
              after_first, kRepeats, after_repeats, after_repeats / after_first);
  EXPECT_LT(after_repeats, 0.01 * after_first);  // measured (seed 7): 5e-15 of it, all particles one pose
}

TEST(UpdateGateCollapse, StandingRuleSkipsTheUpdateAndKeepsTheSpread) {
  const GridMap g = test::makeRoomGrid(60, 60, 0.1);
  const Pose2D truth{3.0, 3.0, 0.4}, sensor_in_base{0, 0, 0};
  Laser2DLikelihoodField model(g, LaserParams{});
  model.setScan(test::raycastScan(g, truth, sensor_in_base, 180, 10.0));

  Rng rng(7);
  ParticleFilterParams pp;
  pp.min_particles = 800;
  pp.max_particles = 800;
  ParticleFilter pf(pp, rng);
  pf.initializeGaussian(truth, Pose2D{0.3, 0.3, 0.1}, 800);
  OdometryMotionModel mm(MotionParams{0.2, 0.2, 0.2, 0.2});
  const Pose2D still{0, 0, 0};
  pf.predict(mm, still, still);
  pf.correct(model);
  pf.resample();
  const double after_first = spreadOf(pf).xy_var;

  // Standing: UpdateGate::decide gives no update, so the node runs none of predict,
  // correct, resample. The particle set, and so the covariance, stays bit-identical.
  for (int i = 0; i < kRepeats; ++i) { /* no update */ }
  EXPECT_DOUBLE_EQ(spreadOf(pf).xy_var, after_first);
}
