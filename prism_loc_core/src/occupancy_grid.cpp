#include "prism_loc_core/occupancy_grid.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace prism_loc_core {

bool worldToMap(const GridMap& g, double wx, double wy, int& mx, int& my) {
  if (g.resolution <= 0.0) return false;
  mx = static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
  my = static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
  return mx >= 0 && my >= 0 && mx < g.width && my < g.height;
}

void mapToWorld(const GridMap& g, int mx, int my, double& wx, double& wy) {
  wx = g.origin_x + (mx + 0.5) * g.resolution;
  wy = g.origin_y + (my + 0.5) * g.resolution;
}

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// 1D squared-distance transform of sampled function f (Felzenszwalb & Huttenlocher
// 2012). Infinite-aware: cells with f == +inf (free space) are NOT parabola sites,
// so a map with only a few occupied cells transforms correctly (the naive form
// computes inf - inf = NaN and poisons the whole row).
void edt1d(const std::vector<double>& f, std::vector<double>& d) {
  const int n = static_cast<int>(f.size());
  d.assign(n, kInf);
  std::vector<int> v(n, 0);
  std::vector<double> z(n + 1, 0.0);
  int k = -1;  // empty stack of lower-envelope parabolas
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(f[q])) continue;  // free cell: not a site
    double s = 0.0;
    while (k >= 0) {
      s = ((f[q] + 1.0 * q * q) - (f[v[k]] + 1.0 * v[k] * v[k])) /
          (2.0 * q - 2.0 * v[k]);
      if (s <= z[k]) { --k; } else { break; }
    }
    if (k < 0) {
      k = 0; v[0] = q; z[0] = -kInf; z[1] = kInf;
    } else {
      ++k; v[k] = q; z[k] = s; z[k + 1] = kInf;
    }
  }
  if (k < 0) return;  // no occupied sites in this line: distances stay +inf
  int j = 0;
  for (int q = 0; q < n; ++q) {
    while (j < k && z[j + 1] < q) ++j;
    const double dx = q - v[j];
    d[q] = dx * dx + f[v[j]];
  }
}
}  // namespace

LikelihoodField::LikelihoodField(const GridMap& grid, int occupied_threshold, double max_dist)
    : width_(grid.width),
      height_(grid.height),
      resolution_(grid.resolution),
      origin_x_(grid.origin_x),
      origin_y_(grid.origin_y),
      max_dist_(max_dist) {
  const int w = width_, h = height_;
  std::vector<double> f(static_cast<size_t>(w) * h, kInf);
  for (int i = 0; i < w * h; ++i) {
    if (grid.data[i] >= occupied_threshold) f[i] = 0.0;  // occupied => zero cost
  }
  // Transform along columns, then rows (separable).
  std::vector<double> col(h), dcol(h);
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) col[y] = f[y * w + x];
    edt1d(col, dcol);
    for (int y = 0; y < h; ++y) f[y * w + x] = dcol[y];
  }
  std::vector<double> row(w), drow(w);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) row[x] = f[y * w + x];
    edt1d(row, drow);
    for (int x = 0; x < w; ++x) f[y * w + x] = drow[x];
  }
  dist_.resize(static_cast<size_t>(w) * h);
  for (int i = 0; i < w * h; ++i) {
    const double meters = std::sqrt(f[i]) * resolution_;
    dist_[i] = std::min(meters, max_dist_);
  }
}

double LikelihoodField::distanceAt(double wx, double wy) const {
  if (resolution_ <= 0.0) return max_dist_;
  const int mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  const int my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) return max_dist_;
  return dist_[static_cast<size_t>(my) * width_ + mx];
}

}  // namespace prism_loc_core
