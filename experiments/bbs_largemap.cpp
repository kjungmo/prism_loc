// Large-map BBS relocalization study (old v0.1 window vs map-sized window).
//
// HONEST SYNTHETIC EXPERIMENT. Every number written to the CSV comes from running
// the real matchers over synthetic occupancy grids and ray-cast scans:
//   * "old": prism_loc v0.1 BranchAndBoundMatcher (verbatim copy in
//     experiments/legacy/), centred on the map centre exactly as v0.1
//     localization_node.cpp did, with the v0.1 default window W = 10 m. Its root
//     enumeration actually searched offsets [-L, -L + 2^D ceil((2L+1)/2^D)) per
//     axis, i.e. -10 m .. +15.5 m about the centre, with no free-space filter.
//   * "new": the current prism_loc_core::BranchAndBoundMatcher::matchGlobal():
//     symmetric window sized from the map extent (cap bbs_max_linear_window =
//     50 m, not reached here) and robot-cell feasibility filter.
// Both use the laser2d.yaml bbs_* defaults otherwise (pi / 0.0175 rad, D = 6,
// 120 beams, sigma_hit 0.2, rho = 0.40).
//
// Worlds: square warehouse-like grids of side 20, 40 and 80 m at 0.1 m/cell:
// border walls, REPEATED shelf rows (regular 6 m pitch, 0.5 m deep, gaps at
// random positions), and random 0.4 m pillars. The repetition is deliberate: it
// creates near-ambiguous places, the regime in which a wrong match can outscore
// the threshold. One map per size, generated from a fixed per-size seed.
//
// Queries: per map, kidnapped poses are drawn uniformly over free space (0.3 m
// clearance, uniform yaw) and STRATIFIED by whether the true pose lies inside
// the old searched set (stratum "in") or outside it ("out"): up to kPerStratum
// of each (the 20 m map is entirely inside the old set, so it has no "out"
// stratum). One 180-beam, 30 m-range scan is ray-cast per pose and corrupted by
// the same clutter model as the 6 m room study: each beam is replaced by a
// uniform spurious range with a per-query probability drawn from U[0, 0.8].
// Both matchers see the identical scan.
//
// Success: position error < 0.5 m AND yaw error < 10 deg.
// Accepted: the matcher's own valid flag (score >= rho * used beams).
// False accept: accepted AND not a success.
// Also logged: whether the returned pose lies in a free, on-map cell.
//
// Build & run from the repository root (writes docs/paper/data/bbs_largemap.csv):
//   g++ -O2 -std=c++17 -I prism_loc_core/include -I prism_loc_core/test -I experiments
//     -I /usr/include/eigen3 prism_loc_core/src/*.cpp experiments/legacy/bbs_v01.cpp
//     experiments/bbs_largemap.cpp -o /tmp/exp_bbs_large && /tmp/exp_bbs_large
// time_ms is wall-clock (machine-dependent); every other column is deterministic.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "legacy/bbs_v01.hpp"
#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/types.hpp"
#include "test_helpers.hpp"

using namespace prism_loc_core;

static constexpr unsigned kSeed = 20260928u;  // fixed, never wall-clock
static constexpr int kPerStratum = 60;
static constexpr double kClutterMax = 0.8;
static constexpr double kRes = 0.1;
static constexpr double kMaxRange = 30.0;
static constexpr int kBeams = 180;

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
  // Repeated shelf rows: pitch 6 m, 0.5 m deep, starting 3 m from the wall,
  // leaving 2 m aisles at both ends; each row gets 1-2 random 1.5 m gaps.
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
  // Random pillars, density ~ 1 per 16 m^2 of floor.
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

static bool cellFree(const GridMap& g, double wx, double wy) {
  int mx, my;
  if (!worldToMap(g, wx, wy, mx, my)) return false;
  return g.data[my * g.width + mx] < 50;
}

static int usedBeams(const LaserScan2D& scan, int max_beams) {
  const int ns = static_cast<int>(scan.ranges.size());
  const int step = std::max(1, ns / std::max(1, max_beams));
  int c = 0;
  for (int i = 0; i < ns; i += step) {
    const double r = scan.ranges[i];
    if (std::isfinite(r) && r > scan.range_min && r < scan.range_max) ++c;
  }
  return c;
}

int main() {
  FILE* f = std::fopen("docs/paper/data/bbs_largemap.csv", "w");
  if (!f) { std::perror("open csv"); return 1; }
  std::fprintf(f,
               "map_m,idx,stratum,true_x,true_y,true_yaw,clutter_frac,method,rec_x,rec_y,rec_yaw,"
               "score,used_beams,score_frac,pos_err_m,yaw_err_deg,time_ms,success,accepted,"
               "rec_free\n");

  const double sizes[] = {20.0, 40.0, 80.0};
  for (int si = 0; si < 3; ++si) {
    const double side = sizes[si];
    const GridMap g = warehouse(side, kSeed + 17u * static_cast<unsigned>(si + 1));

    legacy_v01::BbsParams po;  // v0.1 defaults == laser2d.yaml bbs_* (W = 10 m)
    po.linear_window = 10.0; po.angular_window = M_PI; po.angular_step = 0.0175;
    po.sigma_hit = 0.2; po.min_score_fraction = 0.4; po.max_depth = 6; po.max_beams = 120;
    BbsParams pn;
    pn.angular_window = M_PI; pn.angular_step = 0.0175; pn.sigma_hit = 0.2;
    pn.min_score_fraction = 0.4; pn.max_depth = 6; pn.max_beams = 120;
    pn.max_linear_window = 50.0;
    const legacy_v01::BranchAndBoundMatcher old_m(g, po);
    const BranchAndBoundMatcher new_m(g, pn);
    // v0.1 node centre: map midpoint (cell corner), localization_node.cpp v0.1.
    const Pose2D c_old{g.origin_x + 0.5 * g.width * g.resolution,
                       g.origin_y + 0.5 * g.height * g.resolution, 0.0};
    const int L = static_cast<int>(std::lround(po.linear_window / kRes));
    const int coarse = 1 << po.max_depth;
    const int span = coarse * ((2 * L + coarse) / coarse);  // == 2^D * ceil((2L+1)/2^D)
    const double old_lo = -L * kRes, old_hi = (-L + span) * kRes;  // [lo, hi) about c_old
    std::fprintf(stderr, "map %.0f m: new window covers map = %d; old window [%.1f, %.1f) m\n",
                 side, new_m.globalWindowCoversMap() ? 1 : 0, old_lo, old_hi);

    std::mt19937 rng(kSeed + 1000u * static_cast<unsigned>(si + 1));
    std::uniform_real_distribution<double> upos(0.0, side), uyaw(-M_PI, M_PI), u01(0.0, 1.0);
    std::uniform_real_distribution<double> urange(0.2, kMaxRange);
    int n_in = 0, n_out = 0, idx = 0, tries = 0;
    while ((n_in < kPerStratum || n_out < kPerStratum) && tries < 200000) {
      ++tries;
      Pose2D truth{upos(rng), upos(rng), uyaw(rng)};
      if (!freeWithClearance(g, truth.x, truth.y, 0.3)) continue;
      const double ox = truth.x - c_old.x, oy = truth.y - c_old.y;
      const bool in_old = ox >= old_lo && ox < old_hi && oy >= old_lo && oy < old_hi;
      if (in_old ? n_in >= kPerStratum : n_out >= kPerStratum) continue;
      (in_old ? n_in : n_out)++;

      LaserScan2D scan = test::raycastScan(g, truth, Pose2D{0, 0, 0}, kBeams, kMaxRange);
      const double clutter = u01(rng) * kClutterMax;
      for (float& r : scan.ranges)
        if (u01(rng) < clutter) r = static_cast<float>(urange(rng));
      const int nb = usedBeams(scan, 120);

      for (int m = 0; m < 2; ++m) {
        Pose2D rp; double score; bool valid;
        const auto t0 = std::chrono::steady_clock::now();
        if (m == 0) {
          const auto r = old_m.match(scan, c_old);
          rp = r.pose; score = r.score; valid = r.valid;
        } else {
          const auto r = new_m.matchGlobal(scan);
          rp = r.pose; score = r.score; valid = r.valid;
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double pe = std::hypot(rp.x - truth.x, rp.y - truth.y);
        const double ye = std::fabs(normalizeAngle(rp.yaw - truth.yaw)) * 180.0 / M_PI;
        const bool ok = pe < 0.5 && ye < 10.0;
        std::fprintf(f, "%.0f,%d,%s,%.4f,%.4f,%.4f,%.4f,%s,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.3f,%d,%d,%d\n",
                     side, idx, in_old ? "in" : "out", truth.x, truth.y, truth.yaw, clutter,
                     m == 0 ? "old" : "new", rp.x, rp.y, rp.yaw, score, nb,
                     nb > 0 ? score / nb : 0.0, pe, ye, ms, ok ? 1 : 0, valid ? 1 : 0,
                     cellFree(g, rp.x, rp.y) ? 1 : 0);
      }
      ++idx;
      if (in_old && side <= 20.0 && n_in >= kPerStratum) break;  // 20 m: no "out" region
    }
    std::fflush(f);
    std::fprintf(stderr, "  queries: in=%d out=%d\n", n_in, n_out);
  }
  std::fclose(f);
  return 0;
}
