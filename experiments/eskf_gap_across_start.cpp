// eskf_gap_across_start.cpp -- the node scenario gnss_gap_across_start (scripts/fusion_extrinsic_check.py) on the real Eskf core, for many seeds.
//
// SYNTHETIC EXPERIMENT (study 6, sixth pass). The scenario: GNSS and IMU only, antenna A (0.5, 0, 1.0), a stand of 300 s with a gyro bias of 0.5 deg/s on
// each axis (1, -1, 1), then the start-stop cycle (still 6 s, 0 -> 1.5 m/s in 3 s, 20 s at 1.5 m/s, 1.5 -> 0 in 3 s, still), the fixes (5 Hz) out for 120 s from 10 s
// before the start. The node runs it once; this runs it for NSEED seeds (default 100) and counts the runs whose tilt exceeds 0.05 rad or whose height error
// exceeds 0.5 m in the 60 s after the fixes return (the bounds of the node check). It was written when the first version of the sixth review's fixes
// (the test of the norms of the two changes of velocity, lever_align_norm 0.3, and the spread of the covariance after an outage, lever_outage_spread 1)
// made the node scenario fail once: the table is the evidence for the shipped values (README, "Sixth pass").
//
//   g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3 prism_loc_fusion/src/*.cpp experiments/eskf_gap_across_start.cpp -o /tmp/gap
//   NSEED=300 /tmp/gap            # the sweep: lever_outage_spread 0 ... 1 against lever_align_norm 0, 0.15, 0.3; deterministic for one compiler and standard library
//   VERBOSE=1 NSEED=12 /tmp/gap   # one line per seed and setting
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
#include "prism_loc_fusion/so3.hpp"

using namespace prism_loc_fusion;

namespace {
struct Out { double tilt{0.0}, z{0.0}, head{0.0}; int aligned{0}; };

Out run(EskfParams ep, const Eigen::Vector3d& lever, double stand_s, double gap_before, double gap_s, unsigned seed) {
  Rng rng(seed);
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  auto noise3 = [&rng](double s) { return Eigen::Vector3d(rng.gaussian(0, s), rng.gaussian(0, s), rng.gaussian(0, s)); };
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, 0.1));   // the filter starts 0.1 rad off in heading
  x0.p = lever + noise3(0.02) - x0.q * lever;
  f.initialize(x0, P0);
  f.markHeadingUnknown();   // the default boot: no /initialpose, initial_yaw_known false
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  const double b = 0.5 * M_PI / 180.0;
  const Eigen::Vector3d bias(b, -b, b);
  const double dt = 0.01;
  Out out;
  const double t_back = gap_s - gap_before;   // time since the start at which the fixes return
  const int n_stand = static_cast<int>(stand_s / dt), n = n_stand + static_cast<int>((t_back + 60.0) / dt);
  for (int k = 1; k <= n; ++k) {
    const double t = k * dt - stand_s;
    double a = 0.0, x = 0.0;
    if (t > 0.0) {
      const double cyc = std::floor(t / 40.0), r = t - 40.0 * cyc;
      if (r < 6.0) { x = 0.0; }
      else if (r < 9.0) { const double u = r - 6.0; x = 0.25 * u * u; a = 0.5; }
      else if (r < 29.0) { x = 2.25 + 1.5 * (r - 9.0); }
      else if (r < 32.0) { const double u = r - 29.0; x = 32.25 + 1.5 * u - 0.25 * u * u; a = -0.5; }
      else { x = 34.5; }
      x += 34.5 * cyc;
    }
    f.predict(Eigen::Vector3d(a, 0, 9.81) + noise3(0.02), bias + noise3(0.002), dt);
    const bool in_gap = t >= -gap_before && t < t_back;
    if (k % 20 == 0 && !in_gap) {
      f.updatePositionLever(Eigen::Vector3d(x, 0, 0) + lever + noise3(0.02), lever, R);
      if (f.leverAligned()) ++out.aligned;
    }
    if (t >= t_back + 2.0) {
      const Eigen::Vector3d att = so3Log(f.state().q);
      out.tilt = std::max(out.tilt, att.head<2>().norm());
      out.z = std::max(out.z, std::fabs(f.state().p.z()));
    }
  }
  out.head = std::fabs(so3Log(f.state().q).z());
  return out;
}
}  // namespace

int main() {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  const unsigned nseed = static_cast<unsigned>(std::getenv("NSEED") ? std::atoi(std::getenv("NSEED")) : 100);
  const double spreads[] = {0.0, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0};
  const double norms[] = {0.0, 0.15, 0.3};
  std::printf("gnss_gap_across_start on the core, antenna A, %u seeds: runs with tilt > 0.05 rad or height error > 0.5 m after the fixes return\n", nseed);
  std::printf("%-14s %-10s %5s %12s %12s\n", "outage spread", "norm test", "bad", "worst z (m)", "worst tilt");
  for (double sp : spreads)
    for (double nm : norms) {
      int bad = 0;
      double wz = 0.0, wt = 0.0;
      for (unsigned seed = 1; seed <= nseed; ++seed) {
        EskfParams ep;
        ep.lever_outage_spread = sp;
        ep.lever_align_norm = nm;
        const Out r = run(ep, lever, 300.0, 10.0, 120.0, seed);
        bad += (r.z > 0.5 || r.tilt > 0.05) ? 1 : 0;
        wz = std::max(wz, r.z);
        wt = std::max(wt, r.tilt);
        if (std::getenv("VERBOSE")) std::printf("  spread %g norm %g seed %u: z %.3f tilt %.4f heading error %.3f aligned %d\n", sp, nm, seed, r.z, r.tilt, r.head, r.aligned);
      }
      std::printf("%-14g %-10g %5d %12.3f %12.4f\n", sp, nm, bad, wz, wt);
    }
  return 0;
}
