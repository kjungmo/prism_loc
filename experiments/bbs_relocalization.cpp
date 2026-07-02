// BBS global-relocalization (kidnapped-robot) experiment for the PRISM-Loc paper.
//
// HONEST SYNTHETIC EXPERIMENT. Every number written to the CSV comes from running
// the real prism_loc_core::BranchAndBoundMatcher over a synthetic occupancy grid
// and a synthetic (raycast) LaserScan2D. No hand-tuned or fabricated results.
//
// Setup: an asymmetric 6x6 m room (the same world used by the BBS unit test in
// prism_loc_core/test/test_bbs.cpp; two internal wall stubs break the 4-fold
// symmetry so that a single 360-deg scan pins the global pose uniquely). For
// N=100 true poses drawn uniformly over free space with uniform yaw (seed fixed
// by a compile-time constant, NOT wall-clock), we raycast one synthetic scan and
// run the real BbsMatcher over the whole map (search centered on the map middle,
// linear/angular windows covering the entire grid). We record the true pose, the
// recovered pose, the raw BBS score, the number of beams the matcher actually
// used, the position error, the yaw error, and the wall-clock time of the query.
//
// To span an easy->hard difficulty range (a kidnapped robot rarely sees a clean
// scan: people, clutter, glass and objects absent from the static map produce
// spurious returns), each scan is corrupted by SYNTHETIC CLUTTER: a per-query
// clutter fraction is drawn uniformly in [0, kClutterMax], and each beam is with
// that probability replaced by a uniform-random spurious range. The clutter
// fraction is recorded per query. Low-clutter scans are trivially solved; heavy
// clutter buries the true likelihood peak and is where the score drops toward the
// acceptance threshold.
//
// Success criterion: position error < 0.5 m AND yaw error < 10 deg.
//
// BbsParams are the laser2d.yaml defaults (bbs_* keys):
//   linear_window=10.0, angular_window=pi, angular_step=0.0175, sigma_hit=0.2,
//   min_score_fraction=0.4, max_depth=6, max_beams=120.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/types.hpp"
#include "test_helpers.hpp"

using namespace prism_loc_core;

// Reproducibility: seed is a fixed constant, never time-based.
static constexpr unsigned kSeed = 20240607u;
static constexpr int kNumPoses = 100;
static constexpr double kClutterMax = 0.8;  // per-query clutter fraction ~ U[0, kClutterMax]

// Same asymmetric room as prism_loc_core/test/test_bbs.cpp (6x6 m, 0.1 m/cell).
static GridMap asymRoom() {
  GridMap g = test::makeRoomGrid(60, 60, 0.1);            // 6x6 m border walls
  for (int y = 8; y < 30; ++y) g.data[y * 60 + 30] = 100;   // internal vertical stub
  for (int x = 30; x < 45; ++x) g.data[20 * 60 + x] = 100;  // internal horizontal stub
  return g;
}

// True if world point (wx,wy) lies in free space (cell value 0) with at least
// `clearance` metres to the nearest occupied cell, so the sampled pose is a
// physically plausible robot location (not embedded in a wall).
static bool isFreeWithClearance(const GridMap& g, double wx, double wy, double clearance) {
  int mx, my;
  if (!worldToMap(g, wx, wy, mx, my)) return false;
  if (g.data[my * g.width + mx] >= 50) return false;
  const int r = static_cast<int>(std::ceil(clearance / g.resolution));
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx) {
      int nx = mx + dx, ny = my + dy;
      if (nx < 0 || ny < 0 || nx >= g.width || ny >= g.height) return false;
      if (g.data[ny * g.width + nx] >= 50) return false;
    }
  return true;
}

// Count the beams the BbsMatcher actually uses for scoring, replicating exactly
// the subsampling/validity test in BranchAndBoundMatcher::match(). This is the
// denominator behind the min_score_fraction acceptance test, so recording it lets
// the figure normalize the score into a beam-count-independent fraction.
static int usedBeams(const LaserScan2D& scan, int max_beams) {
  const int ns = static_cast<int>(scan.ranges.size());
  if (ns == 0) return 0;
  const int step = std::max(1, ns / std::max(1, max_beams));
  int c = 0;
  for (int i = 0; i < ns; i += step) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
    ++c;
  }
  return c;
}

int main() {
  const GridMap g = asymRoom();

  // BBS parameters == laser2d.yaml bbs_* defaults.
  BbsParams p;
  p.linear_window = 10.0;        // bbs_linear_window
  p.angular_window = M_PI;       // bbs_angular_window
  p.angular_step = 0.0175;       // bbs_angular_step
  p.sigma_hit = 0.2;             // sigma_hit
  p.min_score_fraction = 0.4;    // bbs_min_score_fraction
  p.max_depth = 6;               // bbs_max_depth
  p.max_beams = 120;             // bbs_max_beams

  // Build the pyramid once (matcher is reused across all queries, as in deployment).
  const BranchAndBoundMatcher matcher(g, p);

  const Pose2D sensor{0.0, 0.0, 0.0};
  const Pose2D center{3.0, 3.0, 0.0};   // search centered on the map middle
  const int n_beams = 180;
  const double max_range = 12.0;        // > room diagonal, matches test_bbs.cpp

  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> ux(0.5, 5.5);   // stay inside border walls
  std::uniform_real_distribution<double> uy(0.5, 5.5);
  std::uniform_real_distribution<double> uyaw(-M_PI, M_PI);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::uniform_real_distribution<double> urange(0.2, max_range);  // spurious return range

  FILE* f = std::fopen(
      "docs/paper/data/bbs_relocalization.csv",  // relative to the repo root
      "w");
  std::fprintf(f,
               "idx,true_x,true_y,true_yaw,clutter_frac,rec_x,rec_y,rec_yaw,score,"
               "used_beams,score_frac,pos_err_m,yaw_err_deg,time_ms,success\n");

  std::vector<double> pos_errs, yaw_errs, times, score_fracs;
  int n_success = 0;      // pos_err<0.5 m && yaw_err<10 deg
  int n_valid = 0;        // matcher self-reported valid (score_frac >= min_score_fraction)
  int n_false_accept = 0; // accepted (valid) but actually wrong
  int n_false_reject = 0; // rejected (invalid) but actually correct

  for (int i = 0; i < kNumPoses; ++i) {
    // Rejection-sample a free-space pose with clearance so the robot is not in a wall.
    Pose2D truth;
    do {
      truth.x = ux(rng);
      truth.y = uy(rng);
      truth.yaw = uyaw(rng);
    } while (!isFreeWithClearance(g, truth.x, truth.y, 0.15));

    LaserScan2D scan = test::raycastScan(g, truth, sensor, n_beams, max_range);

    // Synthetic clutter: replace each beam with prob = clutter_frac by a spurious range.
    const double clutter_frac = u01(rng) * kClutterMax;
    for (float& r : scan.ranges)
      if (u01(rng) < clutter_frac) r = static_cast<float>(urange(rng));

    const int nb = usedBeams(scan, p.max_beams);

    const auto t0 = std::chrono::steady_clock::now();
    const BbsResult r = matcher.match(scan, center);
    const auto t1 = std::chrono::steady_clock::now();
    const double time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    const double dx = r.pose.x - truth.x;
    const double dy = r.pose.y - truth.y;
    const double pos_err = std::sqrt(dx * dx + dy * dy);
    const double yaw_err_deg =
        std::fabs(normalizeAngle(r.pose.yaw - truth.yaw)) * 180.0 / M_PI;
    const double score_frac = nb > 0 ? r.score / static_cast<double>(nb) : 0.0;
    const bool success = (pos_err < 0.5) && (yaw_err_deg < 10.0);
    if (success) ++n_success;
    if (r.valid) ++n_valid;
    if (r.valid && !success) ++n_false_accept;
    if (!r.valid && success) ++n_false_reject;

    pos_errs.push_back(pos_err);
    yaw_errs.push_back(yaw_err_deg);
    times.push_back(time_ms);
    score_fracs.push_back(score_frac);

    std::fprintf(f,
                 "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.3f,%d\n",
                 i, truth.x, truth.y, truth.yaw, clutter_frac, r.pose.x, r.pose.y,
                 r.pose.yaw, r.score, nb, score_frac, pos_err, yaw_err_deg, time_ms,
                 success ? 1 : 0);
  }
  std::fclose(f);

  // Summary statistics printed to stdout (also the source of the paper headline numbers).
  auto median = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n == 0 ? 0.0 : (n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]));
  };
  auto pct = [](std::vector<double> v, double q) {
    std::sort(v.begin(), v.end());
    if (v.empty()) return 0.0;
    const double idx = q * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    return v[lo] + (idx - lo) * (v[hi] - v[lo]);
  };

  std::printf("N=%d poses, seed=%u\n", kNumPoses, kSeed);
  std::printf("success rate = %d/%d = %.1f%%\n", n_success, kNumPoses,
              100.0 * n_success / kNumPoses);
  std::printf("median latency = %.2f ms  (p95 = %.2f ms)\n", median(times),
              pct(times, 0.95));
  std::printf("median pos err = %.3f m  (p95 = %.3f m)\n", median(pos_errs),
              pct(pos_errs, 0.95));
  std::printf("median yaw err = %.2f deg (p95 = %.2f deg)\n", median(yaw_errs),
              pct(yaw_errs, 0.95));
  std::printf("median score fraction = %.3f (threshold = %.2f)\n",
              median(score_fracs), p.min_score_fraction);
  std::printf("accepted (valid) = %d, false accepts = %d, false rejects = %d\n",
              n_valid, n_false_accept, n_false_reject);
  return 0;
}
