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
    if (level == 0) {
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
  return s;
}

BbsResult BranchAndBoundMatcher::match(const LaserScan2D& scan, const Pose2D& center) const {
  BbsResult best;
  const int ns = static_cast<int>(scan.ranges.size());
  if (ns == 0) return best;

  // 1. Subsample valid endpoints in the sensor frame.
  std::vector<Eigen::Vector2d> eps;
  const int step = std::max(1, ns / std::max(1, params_.max_beams));
  for (int i = 0; i < ns; i += step) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
    const double a = scan.angle_min + i * scan.angle_increment;
    eps.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  if (eps.empty()) return best;

  // 2. Discrete rotations across the angular window.
  std::vector<double> angles;
  for (double da = -params_.angular_window; da <= params_.angular_window + 1e-9; da += params_.angular_step)
    angles.push_back(da);
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

  const int L = std::max(1, static_cast<int>(std::lround(params_.linear_window / resolution_)));
  const int top = params_.max_depth;
  const int coarse = 1 << top;

  struct Cand { int t; int xo; int yo; int depth; double upper; };
  std::vector<Cand> roots;
  for (int t = 0; t < na; ++t)
    for (int xo = -L; xo <= L; xo += coarse)
      for (int yo = -L; yo <= L; yo += coarse)
        roots.push_back({t, xo, yo, top, scoreLevel(ep_cells[t], xo, yo, top)});

  double best_score = -1.0;
  std::function<void(std::vector<Cand>&)> branch = [&](std::vector<Cand>& cands) {
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.upper > b.upper; });
    for (const Cand& c : cands) {
      if (c.upper <= best_score) break;  // admissible prune (sorted desc)
      if (c.depth == 0) {
        best_score = c.upper;            // leaf upper == exact level-0 score
        best.pose = Pose2D{center.x + c.xo * resolution_, center.y + c.yo * resolution_,
                           normalizeAngle(center.yaw + angles[c.t])};
        best.score = c.upper;
      } else {
        const int half = 1 << (c.depth - 1);
        std::vector<Cand> ch;
        ch.reserve(4);
        for (int dx : {0, half})
          for (int dy : {0, half})
            ch.push_back({c.t, c.xo + dx, c.yo + dy, c.depth - 1,
                          scoreLevel(ep_cells[c.t], c.xo + dx, c.yo + dy, c.depth - 1)});
        branch(ch);
      }
    }
  };
  branch(roots);

  best.valid = best_score >= params_.min_score_fraction * static_cast<double>(eps.size());
  return best;
}

}  // namespace prism_loc_core
