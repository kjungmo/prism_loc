#include "prism_loc_core/ground_height_map.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
namespace prism_loc_core {

namespace {
// Same key scheme as NdtMap, in 2D: two 21-bit cell indices with an offset of 2^20.
constexpr std::int64_t kOffset = 1 << 20;
constexpr std::int64_t kBits = 21;
constexpr std::int64_t kMaxIndex = kOffset - 3;  // |cell index| limit; leaves room for neighbours
constexpr int kSub = 4;                          // sub-columns per cell edge
constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();

template <typename T>
void require(bool ok, const char* name, T value, const char* rule) {
  if (ok) return;
  std::ostringstream os;
  os << "GroundHeightParams." << name << " = " << value << " (must be " << rule << ")";
  throw std::invalid_argument(os.str());
}
}  // namespace

void validate(const GroundHeightParams& p) {
  require(std::isfinite(p.resolution) && p.resolution > 0.0, "resolution", p.resolution, "finite and > 0");
  require(p.min_points >= 1, "min_points", p.min_points, ">= 1");
  require(std::isfinite(p.layer_gap) && p.layer_gap > 0.0, "layer_gap", p.layer_gap, "finite and > 0");
  require(std::isfinite(p.ground_quantile) && p.ground_quantile >= 0.0 && p.ground_quantile <= 1.0,
          "ground_quantile", p.ground_quantile, "in [0, 1]");
  require(std::isfinite(p.max_step) && p.max_step > 0.0, "max_step", p.max_step, "finite and > 0");
  require(std::isfinite(p.band) && p.band >= 0.0, "band", p.band, "finite and >= 0");
}

std::int64_t GroundHeightMap::keyOfCell(std::int64_t i, std::int64_t j) {
  return ((i + kOffset) << kBits) | (j + kOffset);
}

bool GroundHeightMap::cellIndex(double x, double y, std::int64_t& i, std::int64_t& j) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  const double fx = std::floor(x / params_.resolution), fy = std::floor(y / params_.resolution);
  if (!(std::fabs(fx) <= kMaxIndex) || !(std::fabs(fy) <= kMaxIndex)) return false;
  i = static_cast<std::int64_t>(fx);
  j = static_cast<std::int64_t>(fy);
  return true;
}

const std::vector<double>* GroundHeightMap::cell(std::int64_t i, std::int64_t j) const {
  const auto it = cells_.find(keyOfCell(i, j));
  return it == cells_.end() ? nullptr : &it->second;
}

bool GroundHeightMap::surround(double x, double y, std::int64_t& i0, std::int64_t& j0, double& tx,
                               double& ty) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  const double u = x / params_.resolution - 0.5, v = y / params_.resolution - 0.5;
  const double fu = std::floor(u), fv = std::floor(v);
  if (!(std::fabs(fu) <= kMaxIndex) || !(std::fabs(fv) <= kMaxIndex)) return false;
  i0 = static_cast<std::int64_t>(fu);
  j0 = static_cast<std::int64_t>(fv);
  tx = u - fu;
  ty = v - fv;
  return true;
}

namespace {
// Ground candidate of one group of z-sorted points (indices s[b, e) into pts).
double candidateOf(const std::vector<Eigen::Vector3d>& pts, const std::uint32_t* s, std::size_t b,
                   std::size_t e, double cx, double cy, const GroundHeightParams& prm) {
  const std::size_t n = e - b;
  const std::size_t qi = std::min(n - 1, static_cast<std::size_t>(std::floor(prm.ground_quantile * (n - 1))));
  const double zq = pts[s[b + qi]].z();
  // First point of the group with z >= zq (equal values before index qi count), then the
  // band up to zq + band; the lowest point of each sub-column.
  std::size_t k = b + qi;
  while (k > b && pts[s[k - 1]].z() >= zq) --k;
  std::array<int, kSub * kSub> seen{};
  std::array<double, kSub * kSub> px, py, pz;
  int m = 0;
  const double zmax = zq + prm.band;
  const double x0 = cx - 0.5 * prm.resolution, y0 = cy - 0.5 * prm.resolution;
  for (; k < e && pts[s[k]].z() <= zmax; ++k) {
    const Eigen::Vector3d& p = pts[s[k]];
    const int bx = std::clamp(static_cast<int>(std::floor((p.x() - x0) / prm.resolution * kSub)), 0, kSub - 1);
    const int by = std::clamp(static_cast<int>(std::floor((p.y() - y0) / prm.resolution * kSub)), 0, kSub - 1);
    const int bin = by * kSub + bx;
    if (seen[bin]) continue;  // z ascending: the first point of a sub-column is its lowest
    seen[bin] = 1;
    px[m] = p.x() - cx;
    py[m] = p.y() - cy;
    pz[m] = p.z();
    ++m;
  }
  if (m == 0) return zq;  // cannot happen (the quantile point is in the band); kept for safety
  double mx = 0, my = 0, mz = 0;
  for (int i = 0; i < m; ++i) { mx += px[i]; my += py[i]; mz += pz[i]; }
  mx /= m; my /= m; mz /= m;
  if (m < 3) return mz;
  double sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
  for (int i = 0; i < m; ++i) {
    const double dx = px[i] - mx, dy = py[i] - my, dz = pz[i] - mz;
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy; sxz += dx * dz; syz += dy * dz;
  }
  sxx /= m; sxy /= m; syy /= m; sxz /= m; syz /= m;
  const double half_tr = 0.5 * (sxx + syy);
  const double det = sxx * syy - sxy * sxy;
  const double min_eig = half_tr - std::sqrt(std::max(0.0, half_tr * half_tr - det));
  const double floor_sd = 0.1 * prm.resolution;
  if (!(min_eig >= floor_sd * floor_sd)) return mz;
  const double bx = (sxz * syy - syz * sxy) / det;
  const double by = (syz * sxx - sxz * sxy) / det;
  const double a = mz - bx * mx - by * my;  // plane value at the cell centre
  if (!std::isfinite(a) || std::fabs(a - mz) > prm.band) return mz;
  return a;
}
}  // namespace

GroundHeightMap::GroundHeightMap(const std::vector<Eigen::Vector3d>& points,
                                 const GroundHeightParams& params)
    : params_(params) {
  validate(params_);
  if (points.size() >= kNone) throw std::length_error("GroundHeightMap: too many points");
  // Pass 1: a dense id per occupied cell, and the id of each point. Pass 2: counting sort of
  // the point indices by cell id. 8 bytes per point in all; the points are not copied.
  std::unordered_map<std::int64_t, std::uint32_t> ids;
  std::vector<std::int64_t> keys;
  std::vector<std::uint32_t> cell_of(points.size(), kNone);
  std::vector<std::uint32_t> counts;
  for (std::size_t n = 0; n < points.size(); ++n) {
    const Eigen::Vector3d& p = points[n];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
    std::int64_t i, j;
    if (!cellIndex(p.x(), p.y(), i, j)) { ++skipped_out_of_range_; continue; }
    const auto ins = ids.emplace(keyOfCell(i, j), static_cast<std::uint32_t>(keys.size()));
    if (ins.second) { keys.push_back(ins.first->first); counts.push_back(0); }
    cell_of[n] = ins.first->second;
    ++counts[ins.first->second];
  }
  ids.clear();
  std::vector<std::size_t> start(keys.size() + 1, 0);
  for (std::size_t c = 0; c < keys.size(); ++c) start[c + 1] = start[c] + counts[c];
  std::vector<std::uint32_t> order(start.back());
  {
    std::vector<std::size_t> fill(start.begin(), start.end() - 1);
    for (std::size_t n = 0; n < points.size(); ++n)
      if (cell_of[n] != kNone) order[fill[cell_of[n]]++] = static_cast<std::uint32_t>(n);
  }
  std::vector<std::uint32_t>().swap(cell_of);
  const std::size_t min_pts = static_cast<std::size_t>(params_.min_points);
  // Each cell is handled on its own; the sort is a total order on (z, x, y), so equal keys
  // are equal points and the order of the input does not matter.
  for (std::size_t c = 0; c < keys.size(); ++c) {
    const std::size_t b0 = start[c], e0 = start[c + 1];
    if (e0 - b0 < min_pts) continue;
    std::uint32_t* s = order.data();
    std::sort(s + b0, s + e0, [&](std::uint32_t a, std::uint32_t b) {
      const Eigen::Vector3d &pa = points[a], &pb = points[b];
      if (pa.z() != pb.z()) return pa.z() < pb.z();
      if (pa.x() != pb.x()) return pa.x() < pb.x();
      return pa.y() < pb.y();
    });
    const std::int64_t i = (keys[c] >> kBits) - kOffset, j = (keys[c] & ((std::int64_t(1) << kBits) - 1)) - kOffset;
    const double cx = (static_cast<double>(i) + 0.5) * params_.resolution;
    const double cy = (static_cast<double>(j) + 0.5) * params_.resolution;
    std::vector<double> cand;
    std::size_t begin = b0;
    for (std::size_t k = b0 + 1; k <= e0; ++k) {
      if (k < e0 && points[s[k]].z() - points[s[k - 1]].z() <= params_.layer_gap) continue;
      if (k - begin >= min_pts) cand.push_back(candidateOf(points, s, begin, k, cx, cy, params_));
      begin = k;
    }
    if (cand.empty()) continue;
    std::sort(cand.begin(), cand.end());
    cand.shrink_to_fit();
    num_candidates_ += cand.size();
    cells_[keys[c]] = std::move(cand);
  }
}

namespace {
// Nearest to z of the ascending list v (tie: lower).
double nearestOf(const std::vector<double>& v, double z) {
  double best = v[0], best_d = std::fabs(best - z);
  for (std::size_t k = 1; k < v.size(); ++k) {
    const double d = std::fabs(v[k] - z);
    if (d < best_d) { best = v[k]; best_d = d; }
  }
  return best;
}
}  // namespace

std::optional<double> GroundHeightMap::heightAt(double x, double y, double z_hint) const {
  if (!std::isfinite(z_hint)) return std::nullopt;
  std::int64_t i0, j0;
  double tx, ty;
  if (!surround(x, y, i0, j0, tx, ty)) return std::nullopt;
  double sw = 0.0, swz = 0.0, ez = 0.0;
  int n = 0;
  for (int di = 0; di <= 1; ++di)
    for (int dj = 0; dj <= 1; ++dj) {
      const std::vector<double>* c = cell(i0 + di, j0 + dj);
      if (!c) continue;
      const double v = nearestOf(*c, z_hint);
      if (std::fabs(v - z_hint) > params_.max_step) continue;
      const double w = (di ? tx : 1.0 - tx) * (dj ? ty : 1.0 - ty);
      sw += w; swz += w * v; ez += v; ++n;
    }
  if (n == 0) return std::nullopt;
  if (sw < 1e-12) return ez / n;
  return swz / sw;
}

std::optional<double> GroundHeightMap::heightNearest(double x, double y, double z_guess) const {
  if (!std::isfinite(z_guess)) return std::nullopt;
  std::int64_t i0, j0;
  double tx, ty;
  if (!surround(x, y, i0, j0, tx, ty)) return std::nullopt;
  bool any = false;
  double anchor = 0.0, anchor_d = 0.0;
  for (int di = 0; di <= 1; ++di)
    for (int dj = 0; dj <= 1; ++dj) {
      const std::vector<double>* c = cell(i0 + di, j0 + dj);
      if (!c) continue;
      for (double v : *c) {
        const double d = std::fabs(v - z_guess);
        if (!any || d < anchor_d || (d == anchor_d && v < anchor)) { anchor = v; anchor_d = d; any = true; }
      }
    }
  if (!any) return std::nullopt;
  return heightAt(x, y, anchor);
}

std::size_t GroundHeightMap::approxBytes() const {
  auto alloc = [](std::size_t bytes) { return (bytes + 8 + 15) / 16 * 16; };  // malloc: header + 16 B granularity
  std::size_t total = cells_.bucket_count() * sizeof(void*);
  total += cells_.size() * alloc(sizeof(void*) + sizeof(std::int64_t) + sizeof(std::vector<double>));
  for (const auto& kv : cells_) total += alloc(kv.second.capacity() * sizeof(double));
  return total;
}

}  // namespace prism_loc_core
