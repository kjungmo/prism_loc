#include "prism_loc_core/bbs.hpp"
#include <algorithm>
#include <functional>
namespace prism_loc_core {

BranchAndBoundMatcher::BranchAndBoundMatcher(const GridMap& grid, BbsParams params)
    : params_(params),
      width_(grid.width),
      height_(grid.height),
      resolution_(grid.resolution),
      origin_x_(grid.origin_x),
      origin_y_(grid.origin_y) {
  // Level 0: probability grid p = exp(-d^2 / 2 sigma^2) from the likelihood field.
  LikelihoodField lf(grid, 50, 4.0 * params_.sigma_hit + 1e-3);
  const int n = width_ * height_;
  blocked_.assign(n, 0);
  for (int i = 0; i < n; ++i)
    blocked_[i] = grid.data[i] >= params_.occupied_threshold ? 1 : 0;
  pyramid_.assign(params_.max_depth + 1, std::vector<float>(n, 0.0f));
  const double two_sigma2 = 2.0 * params_.sigma_hit * params_.sigma_hit;
  for (int y = 0; y < height_; ++y)
    for (int x = 0; x < width_; ++x) {
      double wx, wy;
      mapToWorld(grid, x, y, wx, wy);
      const double d = lf.distanceAt(wx, wy);
      pyramid_[0][y * width_ + x] = static_cast<float>(std::exp(-(d * d) / two_sigma2));
    }
  // Higher levels: max-pool doubling — pyramid_[h] = max over a 2^h x 2^h block.
  for (int h = 1; h <= params_.max_depth; ++h) {
    const int off = 1 << (h - 1);
    auto at = [&](int xx, int yy) -> float {
      if (xx >= width_ || yy >= height_) return 0.0f;
      return pyramid_[h - 1][yy * width_ + xx];
    };
    for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x) {
        float m = pyramid_[h - 1][y * width_ + x];
        m = std::max(m, at(x + off, y));
        m = std::max(m, at(x, y + off));
        m = std::max(m, at(x + off, y + off));
        pyramid_[h][y * width_ + x] = m;
      }
  }
}

double BranchAndBoundMatcher::scoreLevel(const std::vector<std::pair<int, int>>& ep_cells,
                                         int x_off, int y_off, int level) const {
  const std::vector<float>& g = pyramid_[level];
  // A node at this level covers the translation block [x_off, x_off + 2^level) x
  // [y_off, y_off + 2^level): pyramid_[level][cx, cy] already max-pools that block.
  // For the bound to stay admissible the anchor must be clamped into the grid (not
  // dropped) whenever the block still overlaps the grid — otherwise a coarse node
  // whose corner falls off-map under-counts cells that finer offsets inside it keep
  // in range, pruning the true optimum. Level 0 (exact leaf) keeps drop semantics.
  const int blk = 1 << level;
  double s = 0.0;
  for (const auto& c : ep_cells) {
    int cx = c.first + x_off;
    int cy = c.second + y_off;
    if (level == 0 || mutation_.coarse_drop_off_map) {
      if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) continue;
    } else {
      // Block entirely outside the grid contributes nothing.
      if (cx + blk <= 0 || cy + blk <= 0 || cx >= width_ || cy >= height_) continue;
      if (cx < 0) cx = 0;
      if (cy < 0) cy = 0;
      if (cx >= width_) cx = width_ - 1;
      if (cy >= height_) cy = height_ - 1;
    }
    s += g[cy * width_ + cx];
  }
  return level > 0 ? s * mutation_.coarse_scale : s;
}

bool BranchAndBoundMatcher::feasible(const Pose2D& center, int x_off, int y_off) const {
  // Same world->cell rule as worldToMap(), applied to the pose actually returned.
  const double wx = center.x + x_off * resolution_;
  const double wy = center.y + y_off * resolution_;
  const int mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  const int my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) return false;
  return blocked_[my * width_ + mx] == 0;
}

bool BranchAndBoundMatcher::coversMap(const Pose2D& center, int Lx, int Ly) const {
  // Every cell column/row must contain at least one searched robot position.
  const double x_lo = center.x - Lx * resolution_, x_hi = center.x + Lx * resolution_;
  const double y_lo = center.y - Ly * resolution_, y_hi = center.y + Ly * resolution_;
  return x_lo < origin_x_ + resolution_ && x_hi >= origin_x_ + (width_ - 1) * resolution_ &&
         y_lo < origin_y_ + resolution_ && y_hi >= origin_y_ + (height_ - 1) * resolution_;
}

Pose2D BranchAndBoundMatcher::globalCenter() const {
  return Pose2D{origin_x_ + (width_ / 2 + 0.5) * resolution_,
                origin_y_ + (height_ / 2 + 0.5) * resolution_, 0.0};
}

int BranchAndBoundMatcher::globalHalfWindowX() const {
  const int cap = std::max(1, static_cast<int>(std::lround(params_.max_linear_window / resolution_)));
  return std::max(1, std::min(width_ / 2, cap));
}

int BranchAndBoundMatcher::globalHalfWindowY() const {
  const int cap = std::max(1, static_cast<int>(std::lround(params_.max_linear_window / resolution_)));
  return std::max(1, std::min(height_ / 2, cap));
}

bool BranchAndBoundMatcher::globalWindowCoversMap() const {
  return coversMap(globalCenter(), globalHalfWindowX(), globalHalfWindowY());
}

BbsResult BranchAndBoundMatcher::match(const LaserScan2D& scan, const Pose2D& center) const {
  const int L = std::max(1, static_cast<int>(std::lround(params_.linear_window / resolution_)));
  return searchBest(scan, center, L, L, params_.angular_window, false);
}

BbsResult BranchAndBoundMatcher::matchExhaustive(const LaserScan2D& scan,
                                                 const Pose2D& center) const {
  const int L = std::max(1, static_cast<int>(std::lround(params_.linear_window / resolution_)));
  return searchBest(scan, center, L, L, params_.angular_window, true);
}

BbsResult BranchAndBoundMatcher::matchGlobal(const LaserScan2D& scan) const {
  return searchBest(scan, globalCenter(), globalHalfWindowX(), globalHalfWindowY(),
                    params_.angular_window, false);
}

std::vector<BbsResult> BranchAndBoundMatcher::matchTopK(const LaserScan2D& scan,
                                                        const Pose2D& center,
                                                        const BbsModeParams& modes) const {
  const int L = std::max(1, static_cast<int>(std::lround(params_.linear_window / resolution_)));
  return search(scan, center, L, L, params_.angular_window, false, modes);
}

std::vector<BbsResult> BranchAndBoundMatcher::matchGlobalTopK(const LaserScan2D& scan,
                                                              const BbsModeParams& modes) const {
  return search(scan, globalCenter(), globalHalfWindowX(), globalHalfWindowY(),
                params_.angular_window, false, modes);
}

BbsResult BranchAndBoundMatcher::matchLocal(const LaserScan2D& scan, const Pose2D& center,
                                            double linear_window, double angular_window) const {
  const int L = std::max(0, static_cast<int>(std::lround(linear_window / resolution_)));
  return searchBest(scan, center, L, L, angular_window, false, true);
}

BbsResult BranchAndBoundMatcher::searchBest(const LaserScan2D& scan, const Pose2D& center,
                                            int Lx, int Ly, double angular_window,
                                            bool exhaustive, bool prefer_center) const {
  std::vector<BbsResult> r = search(scan, center, Lx, Ly, angular_window, exhaustive,
                                    BbsModeParams{1, 0.0, 0.0}, prefer_center);
  if (!r.empty()) return r.front();
  BbsResult none;
  none.window_covers_map = coversMap(center, Lx, Ly);
  none.used_beams = 0;
  return none;
}

std::vector<BbsResult> BranchAndBoundMatcher::search(const LaserScan2D& scan,
                                                     const Pose2D& center, int Lx, int Ly,
                                                     double angular_window, bool exhaustive,
                                                     const BbsModeParams& modes,
                                                     bool prefer_center) const {
  std::vector<BbsResult> out;
  const bool covers = coversMap(center, Lx, Ly);
  const int ns = static_cast<int>(scan.ranges.size());
  if (ns == 0) return out;
  const int K = std::max(1, modes.k);

  // 1. Subsample valid endpoints in the sensor frame.
  std::vector<Eigen::Vector2d> eps;
  const int step = std::max(1, ns / std::max(1, params_.max_beams));
  for (int i = 0; i < ns; i += step) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
    const double a = scan.angle_min + i * scan.angle_increment;
    eps.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  if (eps.empty()) return out;

  // 2. Discrete rotations across the angular window.
  // Global/windowed search keeps the v0.1 grid (-w, -w + step, ...). The
  // centre-preferring local re-match uses a grid symmetric about 0 so the
  // centre pose itself is a candidate.
  std::vector<double> angles;
  if (prefer_center) {
    const int n = static_cast<int>(std::floor(angular_window / params_.angular_step + 1e-9));
    for (int k = -n; k <= n; ++k) angles.push_back(k * params_.angular_step);
  } else {
    for (double da = -angular_window; da <= angular_window + 1e-9; da += params_.angular_step)
      angles.push_back(da);
  }
  const int na = static_cast<int>(angles.size());

  // 3. Per-angle endpoint cells at the center translation (translation offsets add later).
  std::vector<std::vector<std::pair<int, int>>> ep_cells(na);
  for (int t = 0; t < na; ++t) {
    const Pose2D base{center.x, center.y, normalizeAngle(center.yaw + angles[t])};
    const Pose2D T = compose(base, scan.sensor_in_base);
    ep_cells[t].reserve(eps.size());
    for (const auto& e : eps) {
      const Eigen::Vector2d m = transformPoint(T, e);
      ep_cells[t].emplace_back(static_cast<int>(std::floor((m.x() - origin_x_) / resolution_)),
                               static_cast<int>(std::floor((m.y() - origin_y_) / resolution_)));
    }
  }

  // Kept modes, sorted by score descending. `threshold` is the score a leaf must
  // strictly exceed to change the kept set: the K-th kept score once K modes are
  // held, otherwise -1. With K = 1 this is exactly the single-best rule
  // (strictly-better replaces, ties keep the first leaf reached).
  // prefer_center (K = 1 only): ties on the best score are broken towards the
  // candidate closest to the centre, so on a flat score plateau (e.g. along a
  // featureless corridor) the result stays at the centre instead of drifting to
  // whichever tied leaf is reached first. Ties are then not pruned.
  struct Mode { Pose2D pose; double score; };
  std::vector<Mode> kept;
  double kept_dist = 0.0;
  const bool tie_break = prefer_center && K == 1;
  auto threshold = [&]() -> double {
    return static_cast<int>(kept.size()) >= K ? kept.back().score : -1.0;
  };
  auto centreDist = [&](int t, int xo, int yo) {
    const double a = angles[t] / params_.angular_step;
    return static_cast<double>(xo) * xo + static_cast<double>(yo) * yo + a * a;
  };
  auto improves = [&](int t, int xo, int yo, double s) {
    if (s > threshold()) return true;
    return tie_break && !kept.empty() && s == threshold() && centreDist(t, xo, yo) < kept_dist;
  };
  auto accept = [&](int t, int xo, int yo, double score) {
    const Pose2D p{center.x + xo * resolution_, center.y + yo * resolution_,
                   normalizeAngle(center.yaw + angles[t])};
    if (K == 1) {  // single best: no suppression needed
      if (kept.empty()) kept.push_back({p, score}); else kept[0] = {p, score};
      kept_dist = centreDist(t, xo, yo);
      return;
    }
    // Greedy NMS: a leaf joins the mode of any kept neighbour; it survives only if
    // it beats every neighbour, which it then replaces.
    bool dominated = false;
    std::vector<Mode> next;
    next.reserve(kept.size() + 1);
    for (const Mode& m : kept) {
      const bool near = std::hypot(m.pose.x - p.x, m.pose.y - p.y) <= modes.nms_xy &&
                        std::fabs(normalizeAngle(m.pose.yaw - p.yaw)) <= modes.nms_yaw;
      if (!near) { next.push_back(m); continue; }
      if (m.score >= score) { dominated = true; break; }
    }
    if (dominated) return;
    next.push_back({p, score});
    std::stable_sort(next.begin(), next.end(),
                     [](const Mode& a, const Mode& b) { return a.score > b.score; });
    if (static_cast<int>(next.size()) > K) next.resize(K);
    kept.swap(next);
  };

  if (exhaustive) {
    for (int t = 0; t < na; ++t)
      for (int xo = -Lx; xo <= Lx; ++xo)
        for (int yo = -Ly; yo <= Ly; ++yo) {
          if (!feasible(center, xo, yo)) continue;
          const double s = scoreLevel(ep_cells[t], xo, yo, 0);
          if (improves(t, xo, yo, s)) accept(t, xo, yo, s);
        }
  } else {
    // Search set: offsets in [-Lx, Lx] x [-Ly, Ly] exactly. Roots tile it from the
    // negative corner; any child whose block starts beyond +L is dropped, so no
    // offset outside the symmetric window is ever scored as a leaf. A node's
    // bound max-pools its whole 2^h block, which over-covers the in-window part
    // and therefore stays an upper bound on every in-window leaf below it.
    const int top = params_.max_depth;
    const int coarse = 1 << top;
    struct Cand { int t; int xo; int yo; int depth; double upper; };
    std::vector<Cand> roots;
    for (int t = 0; t < na; ++t)
      for (int xo = -Lx; xo <= Lx; xo += coarse)
        for (int yo = -Ly; yo <= Ly; yo += coarse)
          roots.push_back({t, xo, yo, top, scoreLevel(ep_cells[t], xo, yo, top)});

    std::function<void(std::vector<Cand>&)> branch = [&](std::vector<Cand>& cands) {
      std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.upper > b.upper; });
      for (const Cand& c : cands) {
        if (c.upper < threshold() || (c.upper == threshold() && !tie_break))
          break;  // admissible prune (sorted desc)
        if (c.depth == 0) {
          // Leaf upper == exact level-0 score. Infeasible robot cells (occupied or
          // off-map) are skipped: bounds above stay admissible for the feasible
          // subset because they bound every leaf, feasible or not.
          if (feasible(center, c.xo, c.yo) && improves(c.t, c.xo, c.yo, c.upper))
            accept(c.t, c.xo, c.yo, c.upper);
        } else {
          const int half = 1 << (c.depth - 1);
          std::vector<Cand> ch;
          ch.reserve(4);
          for (int dx : {0, half})
            for (int dy : {0, half}) {
              if (c.xo + dx > Lx || c.yo + dy > Ly) continue;  // outside the window
              ch.push_back({c.t, c.xo + dx, c.yo + dy, c.depth - 1,
                            scoreLevel(ep_cells[c.t], c.xo + dx, c.yo + dy, c.depth - 1)});
            }
          branch(ch);
        }
      }
    };
    branch(roots);
  }

  const double need = params_.min_score_fraction * static_cast<double>(eps.size());
  for (const Mode& m : kept) {
    BbsResult r;
    r.pose = m.pose;
    r.score = m.score;
    r.valid = m.score >= 0.0 && m.score >= need;
    r.window_covers_map = covers;
    r.used_beams = static_cast<int>(eps.size());
    r.found = true;
    out.push_back(r);
  }
  if (out.empty()) {  // endpoints exist but no feasible candidate in the window
    BbsResult r;
    r.window_covers_map = covers;
    r.used_beams = static_cast<int>(eps.size());
    out.push_back(r);
  }
  return out;
}

}  // namespace prism_loc_core
