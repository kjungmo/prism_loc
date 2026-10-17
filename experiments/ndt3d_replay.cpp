// ndt3d_replay.cpp -- the ndt3d update loop of prism_loc::LocalizationNode, driven offline.
//
// SYNTHETIC REPLAY. Drives the REAL cores (prism_loc_core::ParticleFilter, Ndt3DModel,
// NdtMap, OdometryMotionModel and prism_loc::MatchMonitor from prism_loc/ndt_status.hpp)
// with the clouds, odometry and /initialpose messages that experiments/ndt3d_replay_gen.py
// writes, in the order LocalizationNode::runUpdate (prism_loc/src/localization_node.cpp)
// uses: skip a cloud until the robot has moved update_min_d / update_min_a (0.2 m, 0.2 rad)
// or a reseed forces the update; predict; correct; resample; estimate; matchStats at the
// estimate; MatchMonitor::update; status inputs. The loop itself (the order and the gate)
// is a copy of the node's, not the node: no ROS, no TF, no timers. The node-level
// confirmation is scripts/synthetic_e2e.py (the ndt3d scenarios) and the /diagnostics
// table in the README, which came from an integration harness outside this repository.
//
//   ndt3d_replay MAP.bin FRAMES.bin OUT.csv [--base-height H] [--seed S] [--thr T] [--sens]
//
// One row per filter update: time, forced (reseed), particles, n_eff, estimate, errors
// against the truth, match fraction, points scored, mean of the last 5, low flag, share of
// points inside a map voxel, mean log-likelihood per point, wall-clock ms of correct() and
// matchStats() (these two columns depend on the machine). --sens prints, per run, the mean
// match fraction at the truth displaced along and across the heading and in yaw.
//
// Build (from the repository root):
//   g++ -O2 -std=c++17 -I prism_loc_core/include -I prism_loc/include -I /usr/include/eigen3
//       prism_loc_core/src/*.cpp experiments/ndt3d_replay.cpp -o /tmp/ndt3d_replay
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/ndt_map.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc/ndt_status.hpp"
using namespace prism_loc_core;

struct Tempered : MeasurementModel {
  const Ndt3DModel& m;
  double s;
  Tempered(const Ndt3DModel& mm, double ss) : m(mm), s(ss) {}
  double logLikelihood(const Pose2D& p) const override { return s * m.logLikelihood(p); }
};

template <class T>
bool rd(std::ifstream& f, T* v, size_t n = 1) {
  return static_cast<bool>(f.read(reinterpret_cast<char*>(v), sizeof(T) * n));
}

int main(int argc, char** argv) {
  if (argc < 4) return 2;
  double base_height = 0.5, temper = 0.0, thr = 0.65;
  std::uint64_t seed = 42;
  std::string hex;
  bool sens = false;
  for (int i = 4; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--base-height") base_height = atof(argv[++i]);
    else if (a == "--seed") seed = strtoull(argv[++i], nullptr, 10);
    else if (a == "--temper") temper = atof(argv[++i]);
    else if (a == "--hex") hex = argv[++i];
    else if (a == "--sens") sens = true;
    else if (a == "--thr") thr = atof(argv[++i]);
  }
  std::ifstream fm(argv[1], std::ios::binary);
  std::int32_t nm = 0;
  rd(fm, &nm);
  std::vector<double> raw(3 * nm);
  rd(fm, raw.data(), raw.size());
  std::vector<Eigen::Vector3d> mp(nm);
  for (int i = 0; i < nm; ++i) mp[i] = Eigen::Vector3d(raw[3 * i], raw[3 * i + 1], raw[3 * i + 2]);
  auto map = std::make_shared<NdtMap>(mp, 1.0, 5);
  NdtParams np;
  np.max_points = 500;
  np.base_height = base_height;
  Ndt3DModel model(map, np);
  ParticleFilterParams pp;
  MotionParams mpz;
  mpz.alpha1 = mpz.alpha2 = mpz.alpha3 = mpz.alpha4 = 0.2;
  Rng rng(seed);
  ParticleFilter pf(pp, rng);
  OdometryMotionModel motion(mpz);

  std::ifstream ff(argv[2], std::ios::binary);
  double sib_a[3];
  rd(ff, sib_a, 3);
  const Pose2D sib{sib_a[0], sib_a[1], sib_a[2]};
  FILE* out = fopen(argv[3], "w");
  FILE* hx = hex.empty() ? nullptr : fopen(hex.c_str(), "w");
  fprintf(out, "t,forced,n,neff,ex,ey,eyaw,err,yawerr,frac,pts,mean5,low,inmap,ll_pt,t_correct_ms,t_match_ms\n");
  bool init = false, force = false, have_last = false;
  Pose2D last_odom;
  prism_loc::MatchMonitor mon;
  const double offs[] = {0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 2.0};
  const double yoffs[] = {0.035, 0.087, 0.175};
  double s_fwd[8] = {0}, s_lat[8] = {0}, s_yaw[3] = {0};
  int s_n = 0;
  double tot_c = 0, tot_m = 0;
  int n_up = 0;
  while (true) {
    double hd[10];
    std::int32_t ipf, n;
    if (!rd(ff, hd, 7)) break;
    rd(ff, &ipf);
    rd(ff, hd + 7, 3);
    rd(ff, &n);
    std::vector<float> p(3 * n);
    rd(ff, p.data(), p.size());
    const double t = hd[0];
    const Pose2D odom{hd[1], hd[2], hd[3]}, truth{hd[4], hd[5], hd[6]};
    if (ipf) {
      const int cur = pf.particles().empty() ? 2000 : static_cast<int>(pf.particles().size());
      pf.initializeGaussian(Pose2D{hd[7], hd[8], hd[9]}, Pose2D{0.5, 0.5, std::sqrt(0.0685)}, std::max(cur, 500));
      init = true; force = true; have_last = false;
    }
    if (!init) continue;
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) pts.emplace_back(p[3 * i], p[3 * i + 1], p[3 * i + 2]);
    model.setCloud(pts, sib);
    if (!have_last) { last_odom = odom; have_last = true; }
    const double dd = std::hypot(odom.x - last_odom.x, odom.y - last_odom.y);
    const double da = std::fabs(normalizeAngle(odom.yaw - last_odom.yaw));
    if (!(force || dd >= 0.2 || da >= 0.2)) continue;
    const bool reseeded = force;
    pf.predict(motion, last_odom, odom);
    const auto c0 = std::chrono::steady_clock::now();
    if (temper != 0.0) {
      const int step = std::max(1, n / 500);
      const int scored = (n + step - 1) / step;
      Tempered tm(model, temper < 0 ? 1.0 / std::max(1, scored) : temper);
      pf.correct(tm);
    } else {
      pf.correct(model);
    }
    const auto c1 = std::chrono::steady_clock::now();
    const double neff = pf.effectiveSampleSize();
    const size_t npart = pf.particles().size();
    if (hx) {
      fprintf(hx, "u t %a neff %a n %zu\n", t, neff, npart);
      for (const auto& q : pf.particles()) fprintf(hx, "%a %a %a %a\n", q.pose.x, q.pose.y, q.pose.yaw, q.weight);
    }
    pf.resample();
    last_odom = odom;
    force = false;
    Eigen::Matrix<double, 6, 6> cov;
    const Pose2D est = pf.estimate(&cov);
    double frac = -1, mean5 = -1, tm_ms = 0;
    int low = 0, mpts = 0;
    const auto m0 = std::chrono::steady_clock::now();
    const NdtMatchStats ms = model.matchStats(est);
    const auto m1 = std::chrono::steady_clock::now();
    tm_ms = std::chrono::duration<double, std::milli>(m1 - m0).count();
    mon.update(reseeded, ms.fraction());
    frac = ms.fraction(); mpts = ms.points;
    mean5 = mon.size() ? mon.mean() : -1;
    low = mon.low(thr);
    if (hx) {
      fprintf(hx, "e %a %a %a cov %a %a %a n %zu\n", est.x, est.y, est.yaw, cov(0, 0), cov(1, 1), cov(5, 5), pf.particles().size());
      for (const auto& q : pf.particles()) fprintf(hx, "%a %a %a %a\n", q.pose.x, q.pose.y, q.pose.yaw, q.weight);
    }
    // points in a map cell, mean log-lik per point at the estimate
    int inmap = 0, scored = 0;
    {
      const Eigen::Isometry3d T = toIsometry3(est, base_height) * toIsometry3(sib, 0.0);
      const int step = std::max(1, n / 500);
      for (int i = 0; i < n; i += step) { ++scored; if (map->hasVoxel(T * pts[i])) ++inmap; }
    }
    const double llpt = scored ? model.logLikelihood(est) / scored : 0.0;
    const double tc_ms = std::chrono::duration<double, std::milli>(c1 - c0).count();
    tot_c += tc_ms; tot_m += tm_ms; ++n_up;
    fprintf(out, "%.3f,%d,%zu,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d,%.4f,%.3f,%.3f,%.4f\n", t, reseeded ? 1 : 0,
            npart, neff, est.x, est.y, est.yaw, std::hypot(est.x - truth.x, est.y - truth.y),
            std::fabs(normalizeAngle(est.yaw - truth.yaw)), frac, mpts, mean5, low,
            scored ? static_cast<double>(inmap) / scored : 0.0, llpt, tc_ms, tm_ms);
    if (sens && t > 10.0 && !reseeded) {
      ++s_n;
      for (int k = 0; k < 8; ++k) {
        Pose2D a = truth, b = truth;
        a.x += offs[k] * std::cos(truth.yaw); a.y += offs[k] * std::sin(truth.yaw);
        b.x += -offs[k] * std::sin(truth.yaw); b.y += offs[k] * std::cos(truth.yaw);
        s_fwd[k] += model.matchStats(a).fraction();
        s_lat[k] += model.matchStats(b).fraction();
      }
      for (int k = 0; k < 3; ++k) {
        Pose2D a = truth; a.yaw += yoffs[k];
        s_yaw[k] += model.matchStats(a).fraction();
      }
    }
  }
  fclose(out);
  if (hx) fclose(hx);
  fprintf(stderr, "updates %d mean correct %.3f ms mean matchStats %.4f ms\n", n_up, n_up ? tot_c / n_up : 0, n_up ? tot_m / n_up : 0);
  if (sens && s_n) {
    printf("SENS n=%d fwd:", s_n);
    for (int k = 0; k < 8; ++k) printf(" %.2f=%.3f", offs[k], s_fwd[k] / s_n);
    printf(" | lat:");
    for (int k = 0; k < 8; ++k) printf(" %.2f=%.3f", offs[k], s_lat[k] / s_n);
    printf(" | yaw:");
    for (int k = 0; k < 3; ++k) printf(" %.3f=%.3f", yoffs[k], s_yaw[k] / s_n);
    printf("\n");
  }
  return 0;
}
