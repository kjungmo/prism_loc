// Multi-scan relocalization verification study (single scan vs top-K + M scans).
//
// HONEST SYNTHETIC EXPERIMENT. Every number written to the CSVs comes from
// running the real prism_loc_core::BranchAndBoundMatcher and
// prism_loc_core::RelocalizationVerifier over synthetic warehouse grids and
// ray-cast scans. Two modes:
//
//   calib  CALIBRATION run (docs/paper/data/bbs_verify_calib.csv). Maps and
//          queries come from seed kCalibSeed, disjoint from the study seed.
//          The verifier runs with the widest setting (top_k = kCalibK,
//          verify_scans = kCalibM) and every hypothesis' per-scan score
//          fraction f, liveness and pose error are logged, so that
//          experiments/select_verify_params.py can replay the decision rule
//          for any (K <= kCalibK, M <= kCalibM, evidence_gain, min_posterior)
//          and choose the defaults.
//   study  STUDY run (docs/paper/data/bbs_verify.csv). Same maps, query poses
//          and first scans as experiments/bbs_largemap.cpp (seed 20260928;
//          identical RNG draw order, so the "single" rows reproduce that
//          study's "new" rows). Each query compares
//            single: matchGlobal() on the first scan, accepted if valid
//                    (the v0.2 node rule), and
//            verify: RelocalizationVerifier with the RelocVerifierParams
//                    defaults (the calibrated values) over the trajectory.
//
// World and first scan: exactly as bbs_largemap.cpp (warehouse with repeated
// 6 m-pitch shelf rows, 180 beams, 30 m range, per-query clutter fraction
// U[0, 0.8]). After the first scan the robot drives a short random trajectory
// (0.3 m forward steps with N(0, 0.15 rad) heading changes; blocked steps become
// an in-place turn), one scan per step with the same clutter fraction (drawn
// from separate RNG streams, so the first scan is unchanged). The verifier gets
// noisy odometry increments (sigma = 0.02 |d| + 0.005 m per axis,
// 0.01 + 0.02 |dyaw| rad).
//
// Success: position error < 0.5 m AND yaw error < 10 deg against the true pose
// at the scan the method commits on (first scan for single, last for verify).
// Outcome: accepted / ambiguous / no-candidate. False accept = accepted and not
// a success.
//
// Build & run from the repository root:
//   g++ -O2 -std=c++17 -I prism_loc_core/include -I prism_loc_core/test
//     -I /usr/include/eigen3 prism_loc_core/src/*.cpp experiments/bbs_verify.cpp
//     -pthread -o /tmp/exp_bbs_verify
//   /tmp/exp_bbs_verify calib 10 && python3 experiments/select_verify_params.py
//   /tmp/exp_bbs_verify study 3
// The optional second argument is the number of worker threads (queries are
// generated sequentially first, so it changes nothing but wall-clock time).
// time_ms is wall-clock (machine-dependent); every other column is deterministic.
// The paper's study latencies were measured with 3 workers on a 12-core machine.

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdlib>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/relocalization.hpp"
#include "prism_loc_core/types.hpp"
#include "test_helpers.hpp"

using namespace prism_loc_core;

static constexpr unsigned kStudySeed = 20260928u;  // == bbs_largemap.cpp kSeed
static constexpr unsigned kCalibSeed = 20261105u;  // disjoint calibration seed
static constexpr int kStudyPerStratum = 60;
static constexpr int kCalibPerStratum = 40;
static constexpr int kCalibK = 8;
static constexpr int kCalibM = 12;
static constexpr double kClutterMax = 0.8;
static constexpr double kRes = 0.1;
static constexpr double kMaxRange = 30.0;
static constexpr int kBeams = 180;
static constexpr double kStep = 0.3;

// ---- world generation: verbatim from bbs_largemap.cpp ----
static GridMap warehouse(double side_m, unsigned seed) {
  const int n = static_cast<int>(std::lround(side_m / kRes));
  GridMap g = test::makeRoomGrid(n, n, kRes);
  std::mt19937 rng(seed);
  auto fill = [&](double x0, double y0, double x1, double y1) {
    const int cx0 = std::max(1, static_cast<int>(x0 / kRes)), cy0 = std::max(1, static_cast<int>(y0 / kRes));
    const int cx1 = std::min(n - 1, static_cast<int>(x1 / kRes)), cy1 = std::min(n - 1, static_cast<int>(y1 / kRes));
    for (int y = cy0; y < cy1; ++y)
      for (int x = cx0; x < cx1; ++x) g.data[y * n + x] = 100;
  };
  std::uniform_real_distribution<double> ugap(4.0, side_m - 6.0);
  std::uniform_int_distribution<int> ngap(1, 2);
  for (double y = 3.0; y < side_m - 2.5; y += 6.0) {
    std::vector<double> gaps;
    for (int k = ngap(rng); k > 0; --k) gaps.push_back(ugap(rng));
    std::sort(gaps.begin(), gaps.end());
    double x = 2.0;
    for (double gx : gaps) {
      if (gx > x) fill(x, y, gx, y + 0.5);
      x = std::max(x, gx + 1.5);
    }
    if (side_m - 2.0 > x) fill(x, y, side_m - 2.0, y + 0.5);
  }
  std::uniform_real_distribution<double> up(1.0, side_m - 1.4);
  const int npill = static_cast<int>(side_m * side_m / 16.0);
  for (int k = 0; k < npill; ++k) {
    const double px = up(rng), py = up(rng);
    fill(px, py, px + 0.4, py + 0.4);
  }
  return g;
}

static bool freeWithClearance(const GridMap& g, double wx, double wy, double clearance) {
  int mx, my;
  if (!worldToMap(g, wx, wy, mx, my)) return false;
  const int r = static_cast<int>(std::ceil(clearance / g.resolution));
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx) {
      const int nx = mx + dx, ny = my + dy;
      if (nx < 0 || ny < 0 || nx >= g.width || ny >= g.height) return false;
      if (g.data[ny * g.width + nx] >= 50) return false;
    }
  return true;
}

static double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

struct Err { double pos; double yaw_deg; bool ok; };
static Err err(const Pose2D& est, const Pose2D& truth) {
  const double pe = std::hypot(est.x - truth.x, est.y - truth.y);
  const double ye = std::fabs(normalizeAngle(est.yaw - truth.yaw)) * 180.0 / M_PI;
  return {pe, ye, pe < 0.5 && ye < 10.0};
}

struct Query {
  int si{0};
  double side{0.0};
  int idx{0};
  bool in_old{false};
  double clutter{0.0};
  std::vector<Pose2D> poses;
  std::vector<LaserScan2D> scans;
  std::vector<Pose2D> odo;  // odo[t]: noisy increment poses[t-1] -> poses[t]
};

static std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
static std::string fmt(const char* f, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

int main(int argc, char** argv) {
  const bool calib = argc > 1 && std::strcmp(argv[1], "calib") == 0;
  const bool study = argc > 1 && std::strcmp(argv[1], "study") == 0;
  if (!calib && !study) {
    std::fprintf(stderr, "usage: %s calib|study [workers]\n", argv[0]);
    return 2;
  }
  // Queries are generated sequentially (deterministic) and then evaluated by a
  // pool of worker threads; only the wall-clock time_ms column depends on it.
  const int workers = argc > 2 ? std::max(1, std::atoi(argv[2])) : 1;
  const unsigned seed = calib ? kCalibSeed : kStudySeed;
  const int per_stratum = calib ? kCalibPerStratum : kStudyPerStratum;
  RelocVerifierParams vp;  // defaults == calibrated values
  if (calib) { vp.top_k = kCalibK; vp.verify_scans = kCalibM; }
  const int n_scans = vp.verify_scans;

  // ---- phase 1: maps, matchers and queries ----
  const double sizes[] = {20.0, 40.0, 80.0};
  std::vector<GridMap> maps;
  std::vector<std::unique_ptr<BranchAndBoundMatcher>> matchers;
  std::vector<Query> queries;
  for (int si = 0; si < 3; ++si) {
    const double side = sizes[si];
    maps.push_back(warehouse(side, seed + 17u * static_cast<unsigned>(si + 1)));
  }
  for (int si = 0; si < 3; ++si) {
    const double side = sizes[si];
    const GridMap& g = maps[si];
    BbsParams pn;
    pn.angular_window = M_PI; pn.angular_step = 0.0175; pn.sigma_hit = 0.2;
    pn.min_score_fraction = 0.4; pn.max_depth = 6; pn.max_beams = 120;
    pn.max_linear_window = 50.0;
    matchers.push_back(std::make_unique<BranchAndBoundMatcher>(g, pn));
    // Old v0.1 searched set about the map midpoint (stratum label only).
    const Pose2D c_old{g.origin_x + 0.5 * g.width * g.resolution,
                       g.origin_y + 0.5 * g.height * g.resolution, 0.0};
    const int L = static_cast<int>(std::lround(10.0 / kRes));
    const int coarse = 1 << 6;
    const int span = coarse * ((2 * L + coarse) / coarse);
    const double old_lo = -L * kRes, old_hi = (-L + span) * kRes;

    std::mt19937 rng(seed + 1000u * static_cast<unsigned>(si + 1));
    std::uniform_real_distribution<double> upos(0.0, side), uyaw(-M_PI, M_PI), u01(0.0, 1.0);
    std::uniform_real_distribution<double> urange(0.2, kMaxRange);
    int n_in = 0, n_out = 0, idx = 0, tries = 0;
    while ((n_in < per_stratum || n_out < per_stratum) && tries < 200000) {
      ++tries;
      Pose2D truth{upos(rng), upos(rng), uyaw(rng)};
      if (!freeWithClearance(g, truth.x, truth.y, 0.3)) continue;
      const double ox = truth.x - c_old.x, oy = truth.y - c_old.y;
      const bool in_old = ox >= old_lo && ox < old_hi && oy >= old_lo && oy < old_hi;
      if (in_old ? n_in >= per_stratum : n_out >= per_stratum) continue;
      (in_old ? n_in : n_out)++;

      Query q;
      q.si = si; q.side = side; q.idx = idx; q.in_old = in_old;
      // First scan: identical draws to bbs_largemap.cpp.
      q.scans.push_back(test::raycastScan(g, truth, Pose2D{0, 0, 0}, kBeams, kMaxRange));
      q.clutter = u01(rng) * kClutterMax;
      for (float& r : q.scans[0].ranges)
        if (u01(rng) < q.clutter) r = static_cast<float>(urange(rng));

      // Trajectory, later scans and odometry noise: separate per-query streams.
      std::mt19937 trng(seed + 500000u + 10000u * static_cast<unsigned>(si + 1) + idx);
      std::mt19937 crng(seed + 900000u + 10000u * static_cast<unsigned>(si + 1) + idx);
      std::mt19937 orng(seed + 1300000u + 10000u * static_cast<unsigned>(si + 1) + idx);
      std::normal_distribution<double> n01(0.0, 1.0);
      std::uniform_real_distribution<double> c01(0.0, 1.0), cr(0.2, kMaxRange), turn(-M_PI, M_PI);
      q.poses.push_back(truth);
      for (int t = 1; t < n_scans; ++t) {
        const Pose2D cur = q.poses.back();
        const double yaw = normalizeAngle(cur.yaw + 0.15 * n01(trng));
        Pose2D nxt{cur.x + kStep * std::cos(yaw), cur.y + kStep * std::sin(yaw), yaw};
        if (!freeWithClearance(g, nxt.x, nxt.y, 0.3))
          nxt = Pose2D{cur.x, cur.y, normalizeAngle(cur.yaw + turn(trng))};
        q.poses.push_back(nxt);
        LaserScan2D s = test::raycastScan(g, nxt, Pose2D{0, 0, 0}, kBeams, kMaxRange);
        for (float& r : s.ranges)
          if (c01(crng) < q.clutter) r = static_cast<float>(cr(crng));
        q.scans.push_back(s);
      }
      q.odo.push_back(Pose2D{});
      for (int t = 1; t < n_scans; ++t) {
        Pose2D d = compose(inverse(q.poses[t - 1]), q.poses[t]);
        const double dl = std::hypot(d.x, d.y);
        d.x += (0.02 * dl + 0.005) * n01(orng);
        d.y += (0.02 * dl + 0.005) * n01(orng);
        d.yaw = normalizeAngle(d.yaw + (0.01 + 0.02 * std::fabs(d.yaw)) * n01(orng));
        q.odo.push_back(d);
      }
      queries.push_back(std::move(q));
      ++idx;
      if (in_old && side <= 20.0 && n_in >= per_stratum) break;  // 20 m: no "out" region
    }
    std::fprintf(stderr, "map %.0f m: queries in=%d out=%d\n", side, n_in, n_out);
  }

  // ---- phase 2: evaluate ----
  std::vector<std::string> out(queries.size());
  std::atomic<size_t> next{0};
  auto work = [&]() {
    for (size_t qi = next++; qi < queries.size(); qi = next++) {
      const Query& q = queries[qi];
      const BranchAndBoundMatcher& m = *matchers[q.si];
      const char* strat = q.in_old ? "in" : "out";
      std::string& o = out[qi];
      RelocalizationVerifier ver(m, vp);
      if (calib) {
        auto t0 = std::chrono::steady_clock::now();
        ver.start(q.scans[0]);
        double tms = ms(t0, std::chrono::steady_clock::now());
        std::vector<double> prev_sum;
        for (int t = 0; t < n_scans; ++t) {
          if (t > 0) {
            t0 = std::chrono::steady_clock::now();
            ver.update(q.odo[t], q.scans[t]);
            tms = ms(t0, std::chrono::steady_clock::now());
          }
          const auto& hs = ver.hypotheses();
          prev_sum.resize(hs.size(), 0.0);
          for (size_t h = 0; h < hs.size(); ++h) {
            const Err e = err(hs[h].pose, q.poses[t]);
            const double fr = hs[h].fraction_sum - prev_sum[h];
            prev_sum[h] = hs[h].fraction_sum;
            o += fmt("%.0f,%d,%s,%.4f,%d,%zu,%d,%.6f,%.4f,%.4f,%.3f\n", q.side, q.idx, strat,
                     q.clutter, t, h, hs[h].alive ? 1 : 0, hs[h].alive ? fr : 0.0, e.pos,
                     e.yaw_deg, tms);
          }
          if (hs.empty())
            o += fmt("%.0f,%d,%s,%.4f,%d,-1,0,0,0,0,%.3f\n", q.side, q.idx, strat, q.clutter, t, tms);
        }
      } else {
        // single-scan rule
        auto t0 = std::chrono::steady_clock::now();
        const BbsResult r = m.matchGlobal(q.scans[0]);
        const double t_single = ms(t0, std::chrono::steady_clock::now());
        const Err es = err(r.pose, q.poses[0]);
        o += fmt("%.0f,%d,%s,%.4f,single,1,1,0,0,%s,1,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.3f\n",
                 q.side, q.idx, strat, q.clutter, r.valid ? "accepted" : "no-candidate",
                 r.valid ? 1.0 : 0.0, r.found ? 1 : 0, r.pose.x, r.pose.y, r.pose.yaw,
                 q.poses[0].x, q.poses[0].y, q.poses[0].yaw, es.pos, es.yaw_deg, es.ok ? 1 : 0,
                 r.valid ? 1 : 0, (r.valid && !es.ok) ? 1 : 0, t_single);
        // multi-scan verification
        t0 = std::chrono::steady_clock::now();
        RelocStatus st = ver.start(q.scans[0]);
        for (int t = 1; t < n_scans && st == RelocStatus::kPending; ++t)
          st = ver.update(q.odo[t], q.scans[t]);
        const double t_ver = ms(t0, std::chrono::steady_clock::now());
        const int last = std::max(0, ver.scansUsed() - 1);
        const Err ev = err(ver.pose(), q.poses[last]);
        const bool acc = st == RelocStatus::kAccepted;
        o += fmt("%.0f,%d,%s,%.4f,verify,%d,%d,%.3f,%.4f,%s,%d,%.6f,%zu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.3f\n",
                 q.side, q.idx, strat, q.clutter, vp.top_k, vp.verify_scans, vp.evidence_gain,
                 vp.min_posterior, toString(st), ver.scansUsed(), ver.bestPosterior(),
                 ver.hypotheses().size(), ver.pose().x, ver.pose().y, ver.pose().yaw,
                 q.poses[last].x, q.poses[last].y, q.poses[last].yaw, ev.pos, ev.yaw_deg,
                 ev.ok ? 1 : 0, acc ? 1 : 0, (acc && !ev.ok) ? 1 : 0, t_ver);
      }
    }
  };
  std::vector<std::thread> pool;
  for (int w = 0; w < workers; ++w) pool.emplace_back(work);
  for (auto& th : pool) th.join();

  // ---- phase 3: write in query order ----
  const char* path = calib ? "docs/paper/data/bbs_verify_calib.csv" : "docs/paper/data/bbs_verify.csv";
  FILE* f = std::fopen(path, "w");
  if (!f) { std::perror("open csv"); return 1; }
  if (calib)
    std::fprintf(f, "map_m,idx,stratum,clutter_frac,scan,hyp,alive,frac,pos_err_m,yaw_err_deg,time_ms\n");
  else
    std::fprintf(f,
                 "map_m,idx,stratum,clutter_frac,method,top_k,verify_scans,evidence_gain,"
                 "min_posterior,outcome,scans_used,best_posterior,n_hyp,rec_x,rec_y,rec_yaw,"
                 "true_x,true_y,true_yaw,pos_err_m,yaw_err_deg,success,accepted,false_accept,"
                 "time_ms\n");
  for (const std::string& o : out) std::fputs(o.c_str(), f);
  std::fclose(f);
  return 0;
}
