// mcl_tracking.cpp — HONEST synthetic MCL tracking experiment for PRISM-Loc.
//
// This driver exercises the REAL prism_loc_core estimator: a synthetic
// occupancy-grid world (built with the same makeRoomGrid/raycastScan machinery
// used by the integration tests), a simulated robot driven around a closed-loop
// trajectory for 65 s at 10 Hz, drifting odometry, and the actual ParticleFilter
// wired with the Laser2DLikelihoodField measurement model and KLD-adaptive
// resampling — exactly the ordering the ROS node (localization_node.cpp) uses:
//   predict(motion, prev_odom, cur_odom) -> correct(laser) -> resample()
// gated by update_min_d / update_min_a.
//
// ALL DATA PRODUCED HERE IS SYNTHETIC. No real sensor or robot is involved.
// Every logged number is written to CSV; the paper figures read only that CSV.
// Seeds are fixed constants (below), so the run is fully reproducible.
//
// Parameters mirror prism_loc/params/laser2d.yaml.
//
// Build (no CMake / no ROS needed):
//   g++ -O2 -std=c++17 \
//     -I prism_loc_core/include -I prism_loc_core/test -I /usr/include/eigen3 \
//     prism_loc_core/src/*.cpp experiments/mcl_tracking.cpp -o /tmp/exp_mcl
//   /tmp/exp_mcl   # writes docs/paper/data/mcl_tracking.csv and mcl_particles.csv

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/rng.hpp"
#include "prism_loc_core/types.hpp"
#include "test_helpers.hpp"

using namespace prism_loc_core;

// ---- fixed, reproducible seeds (NOT time-based) -------------------------------
static const unsigned SEEDS[3] = {101u, 202u, 303u};

// ---- experiment constants (match laser2d.yaml / the ROS node) -----------------
static constexpr double kDt = 0.1;         // 10 Hz
static constexpr int kSteps = 650;         // 65 s > 60 s requirement
static constexpr double kRes = 0.1;        // m/cell (as in integration test)
static constexpr int kBeams = 180;         // dense synthetic scan
static constexpr double kMaxRange = 10.0;  // m (as in integration test)
static constexpr double kUpdateMinD = 0.2; // m  (yaml update_min_d)
static constexpr double kUpdateMinA = 0.2; // rad (yaml update_min_a)

// Build a 12 m x 10 m room (non-square, plus two interior pillars) to break pose
// symmetry — same border-cell convention as test::makeRoomGrid, then a couple of
// occupied blocks stamped into the interior.
static GridMap makeWorld() {
  const int W = 120, H = 100;  // 12.0 m x 10.0 m at 0.1 m/cell
  GridMap g = test::makeRoomGrid(W, H, kRes);
  auto stampBlock = [&](int cx, int cy, int half) {
    for (int y = cy - half; y <= cy + half; ++y)
      for (int x = cx - half; x <= cx + half; ++x)
        if (x >= 0 && x < W && y >= 0 && y < H) g.data[y * W + x] = 100;
  };
  stampBlock(35, 65, 4);  // pillar near (3.5, 6.5) m
  stampBlock(85, 30, 4);  // pillar near (8.5, 3.0) m
  return g;
}

// Ground-truth closed-loop (oval) trajectory. theta sweeps a full 2*pi over the
// run, so the robot returns to its start — a genuine loop.
static Pose2D truthAt(int step) {
  const double cx = 6.0, cy = 5.0, a = 3.6, b = 2.6;
  const double theta = 2.0 * M_PI * (static_cast<double>(step) / kSteps);
  Pose2D p;
  p.x = cx + a * std::cos(theta);
  p.y = cy + b * std::sin(theta);
  // heading = direction of travel (tangent): d/dtheta (x, y)
  p.yaw = std::atan2(b * std::cos(theta), -a * std::sin(theta));
  return p;
}

// Decompose motion prev->cur into the odometry (rot1, trans, rot2) triple.
struct OdomIncr { double rot1, trans, rot2; };
static OdomIncr decompose(const Pose2D& prev, const Pose2D& cur) {
  const double dx = cur.x - prev.x, dy = cur.y - prev.y;
  const double trans = std::hypot(dx, dy);
  OdomIncr d;
  if (trans < 1e-3) {
    d.rot1 = 0.0;
    d.rot2 = normalizeAngle(cur.yaw - prev.yaw);
  } else {
    d.rot1 = normalizeAngle(std::atan2(dy, dx) - prev.yaw);
    d.rot2 = normalizeAngle(cur.yaw - prev.yaw - d.rot1);
  }
  d.trans = trans;
  return d;
}

// Result rows for one seed.
struct StepRow { double t, pos_err, yaw_err; int n_particles; };

static std::vector<StepRow> runSeed(unsigned seed, const GridMap& world) {
  const Pose2D sensor_in_base{0, 0, 0};

  // ParticleFilter params — laser2d.yaml defaults.
  ParticleFilterParams pp;
  pp.min_particles = 500;
  pp.max_particles = 2000;
  pp.resample_threshold = 0.5;
  pp.kld_err = 0.05;
  pp.kld_z = 2.33;
  pp.kld_bin_xy = 0.5;
  pp.kld_bin_yaw = 0.17;

  // MotionParams and LaserParams — laser2d.yaml defaults.
  OdometryMotionModel motion(MotionParams{0.2, 0.2, 0.2, 0.2});
  Laser2DLikelihoodField laser(world, LaserParams{60, 0.5, 0.5, 0.2, 2.0});

  Rng pf_rng(seed);              // filter RNG
  Rng odom_rng(seed + 777u);     // independent RNG for odometry drift synthesis

  ParticleFilter pf(pp, pf_rng);
  const Pose2D start = truthAt(0);
  // Seed a broad Gaussian at the initial pose (the node's /initialpose path takes
  // its spread from the pose covariance). A deliberately wide prior (sigma = 1 m,
  // 0.4 rad) with the full max_particles set exercises KLD adaptation: the count
  // starts high and must shrink as the scans concentrate the posterior.
  pf.initializeGaussian(start, Pose2D{1.0, 1.0, 0.4}, pp.max_particles);

  // Odometry is a drifting frame: integrate NOISY per-step increments of the
  // true motion (alpha model from the yaml). This makes dead-reckoning diverge
  // while MCL must stay locked via the scan correction.
  const MotionParams oa{0.2, 0.2, 0.2, 0.2};
  Pose2D odom = start;      // current (drifting) odometry pose
  Pose2D last_odom = odom;  // odom at the last filter update
  Pose2D prev_truth = start;

  std::vector<StepRow> rows;
  rows.reserve(kSteps);
  Pose2D est = pf.estimate();
  int n_particles = static_cast<int>(pf.particles().size());

  for (int step = 0; step < kSteps; ++step) {
    const Pose2D truth = truthAt(step);

    if (step > 0) {
      // Advance the drifting odometry by the noise-corrupted true increment.
      const OdomIncr d = decompose(prev_truth, truth);
      auto n = [&](double var) { return odom_rng.gaussian(0.0, std::sqrt(std::max(var, 0.0))); };
      const double r1 = d.rot1 + n(oa.alpha1 * d.rot1 * d.rot1 + oa.alpha2 * d.trans * d.trans);
      const double tr = d.trans + n(oa.alpha3 * d.trans * d.trans +
                                    oa.alpha4 * (d.rot1 * d.rot1 + d.rot2 * d.rot2));
      const double r2 = d.rot2 + n(oa.alpha1 * d.rot2 * d.rot2 + oa.alpha2 * d.trans * d.trans);
      odom.x += tr * std::cos(odom.yaw + r1);
      odom.y += tr * std::sin(odom.yaw + r1);
      odom.yaw = normalizeAngle(odom.yaw + r1 + r2);
    }
    prev_truth = truth;

    // Filter update gating — exactly the node's runUpdate() condition.
    const double dd = std::hypot(odom.x - last_odom.x, odom.y - last_odom.y);
    const double da = std::fabs(normalizeAngle(odom.yaw - last_odom.yaw));
    if (dd >= kUpdateMinD || da >= kUpdateMinA) {
      // Synthetic scan at the TRUE pose (this is the "sensor" input).
      LaserScan2D scan = test::raycastScan(world, truth, sensor_in_base, kBeams, kMaxRange);
      laser.setScan(scan);
      pf.predict(motion, last_odom, odom);
      pf.correct(laser);
      pf.resample();
      last_odom = odom;
      est = pf.estimate();
      n_particles = static_cast<int>(pf.particles().size());
    }

    StepRow row;
    row.t = step * kDt;
    row.pos_err = std::hypot(est.x - truth.x, est.y - truth.y);
    row.yaw_err = std::fabs(normalizeAngle(est.yaw - truth.yaw));
    row.n_particles = n_particles;
    rows.push_back(row);
  }
  return rows;
}

int main() {
  const GridMap world = makeWorld();

  std::vector<std::vector<StepRow>> all;
  for (unsigned s : SEEDS) all.push_back(runSeed(s, world));

  const std::string base =
      "docs/paper/data/";  // relative to the repo root — run from there

  // Subsample so each CSV stays well under ~200 rows.
  const int stride = 5;  // 650 / 5 = 130 rows

  // --- tracking CSV: position + yaw error per seed --------------------------
  {
    FILE* f = std::fopen((base + "mcl_tracking.csv").c_str(), "w");
    std::fprintf(f, "t,pos_err_s0,pos_err_s1,pos_err_s2,yaw_err_s0,yaw_err_s1,yaw_err_s2\n");
    for (int i = 0; i < kSteps; i += stride) {
      std::fprintf(f, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", all[0][i].t,
                   all[0][i].pos_err, all[1][i].pos_err, all[2][i].pos_err,
                   all[0][i].yaw_err, all[1][i].yaw_err, all[2][i].yaw_err);
    }
    std::fclose(f);
  }

  // --- particle-count CSV: KLD adaptation per seed --------------------------
  {
    FILE* f = std::fopen((base + "mcl_particles.csv").c_str(), "w");
    std::fprintf(f, "t,n_s0,n_s1,n_s2\n");
    for (int i = 0; i < kSteps; i += stride) {
      std::fprintf(f, "%.2f,%d,%d,%d\n", all[0][i].t, all[0][i].n_particles,
                   all[1][i].n_particles, all[2][i].n_particles);
    }
    std::fclose(f);
  }

  // --- console summary (steady-state = after 10 s of convergence) -----------
  const int warm = static_cast<int>(10.0 / kDt);
  double max_pos = 0.0, sum_pos = 0.0, max_yaw = 0.0, sum_yaw = 0.0;
  int cnt = 0, min_n = 1 << 30, max_n = 0;
  for (auto& rows : all) {
    for (int i = warm; i < kSteps; ++i) {
      max_pos = std::max(max_pos, rows[i].pos_err);
      sum_pos += rows[i].pos_err;
      max_yaw = std::max(max_yaw, rows[i].yaw_err);
      sum_yaw += rows[i].yaw_err;
      ++cnt;
    }
    for (int i = 0; i < kSteps; ++i) {
      min_n = std::min(min_n, rows[i].n_particles);
      max_n = std::max(max_n, rows[i].n_particles);
    }
  }
  std::printf("steady-state (t>=10s, 3 seeds): mean_pos_err=%.4f m  max_pos_err=%.4f m\n",
              sum_pos / cnt, max_pos);
  std::printf("steady-state (t>=10s, 3 seeds): mean_yaw_err=%.4f rad max_yaw_err=%.4f rad\n",
              sum_yaw / cnt, max_yaw);
  std::printf("particle count over run: min=%d  max=%d\n", min_n, max_n);
  return 0;
}
