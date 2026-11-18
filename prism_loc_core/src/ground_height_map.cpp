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

const std::vector<GroundHeightMap::Cand>* GroundHeightMap::cell(std::int64_t i, std::int64_t j) const {
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
struct SubCol { double x, y, v; };  // position (relative to the cell centre) and value

double medianOf(std::vector<double>& v) {  // sorts v; v is not empty
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// Least-squares plane through the sub-column values; false when fewer than 3 of them or
// their xy spread is degenerate (smaller eigenvalue of the covariance below (0.1 res)^2).
bool fitPlane(const std::vector<SubCol>& c, double res, double& a, double& bx, double& by) {
  const int m = static_cast<int>(c.size());
  if (m < 3) return false;
  double mx = 0, my = 0, mz = 0;
  for (const auto& p : c) { mx += p.x; my += p.y; mz += p.v; }
  mx /= m; my /= m; mz /= m;
  double sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
  for (const auto& p : c) {
    const double dx = p.x - mx, dy = p.y - my, dz = p.v - mz;
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy; sxz += dx * dz; syz += dy * dz;
  }
  sxx /= m; sxy /= m; syy /= m; sxz /= m; syz /= m;
  const double half_tr = 0.5 * (sxx + syy);
  const double det = sxx * syy - sxy * sxy;
  const double min_eig = half_tr - std::sqrt(std::max(0.0, half_tr * half_tr - det));
  const double floor_sd = 0.1 * res;
  if (!(min_eig >= floor_sd * floor_sd)) return false;
  bx = (sxz * syy - syz * sxy) / det;
  by = (syz * sxx - sxz * sxy) / det;
  a = mz - bx * mx - by * my;  // value at the cell centre
  return std::isfinite(a) && std::isfinite(bx) && std::isfinite(by);
}

double meanOf(const std::vector<SubCol>& c) {
  double s = 0;
  for (const auto& p : c) s += p.v;
  return s / static_cast<double>(c.size());
}

// Plane, or the mean with slope 0 when the fit is refused or extrapolates the centre by more
// than band from the mean.
void fitOrMean(const std::vector<SubCol>& c, const GroundHeightParams& prm, double& a, double& bx, double& by) {
  const double mean = meanOf(c);
  if (fitPlane(c, prm.resolution, a, bx, by) && std::fabs(a - mean) <= prm.band) return;
  a = mean; bx = 0.0; by = 0.0;
}

// Ground candidate of one group of z-sorted points (indices s[b, e) into pts) of the cell
// with centre (cx, cy).
// Returns false when the group is no ground: its sub-columns lie in a thin strip (the first
// fit is refused for poor xy spread) and the group is taller than 2 x band, i.e. the foot of
// a wall or pillar that spills into a cell without ground.
bool candidateOf(const std::vector<Eigen::Vector3d>& pts, const std::uint32_t* s, std::size_t b,
                 std::size_t e, double cx, double cy, const GroundHeightParams& prm, double& a,
                 double& bx, double& by) {
  std::array<std::vector<double>, kSub * kSub> zs;     // z of the points of each sub-column, ascending
  std::array<std::vector<std::pair<double, double>>, kSub * kSub> xy;
  const double x0 = cx - 0.5 * prm.resolution, y0 = cy - 0.5 * prm.resolution;
  std::array<double, kSub * kSub> zmax;
  zmax.fill(std::numeric_limits<double>::quiet_NaN());
  for (std::size_t k = b; k < e; ++k) {
    const Eigen::Vector3d& p = pts[s[k]];
    const int ix = std::clamp(static_cast<int>(std::floor((p.x() - x0) / prm.resolution * kSub)), 0, kSub - 1);
    const int iy = std::clamp(static_cast<int>(std::floor((p.y() - y0) / prm.resolution * kSub)), 0, kSub - 1);
    const int bin = iy * kSub + ix;
    if (std::isnan(zmax[bin])) zmax[bin] = p.z() + prm.band;  // z ascending: the first is the lowest
    if (p.z() > zmax[bin]) continue;
    zs[bin].push_back(p.z());
    xy[bin].emplace_back(p.x() - cx, p.y() - cy);
  }
  std::vector<SubCol> cols;
  for (int bin = 0; bin < kSub * kSub; ++bin) {
    if (zs[bin].empty()) continue;
    double mx = 0, my = 0;
    for (const auto& q : xy[bin]) { mx += q.first; my += q.second; }
    const double n = static_cast<double>(xy[bin].size());
    cols.push_back({mx / n, my / n, medianOf(zs[bin])});  // zs is ascending already
  }
  double pa, pbx, pby;
  const bool fitted = fitPlane(cols, prm.resolution, pa, pbx, pby);
  if (!fitted && pts[s[e - 1]].z() - pts[s[b]].z() > 2.0 * prm.band) return false;
  fitOrMean(cols, prm, a, bx, by);
  // The robust pass needs enough sub-columns to tell outliers from noise.
  if (!fitted || cols.size() < 8) return true;
  // One robust pass. Objects on the ground only lift sub-columns, but a strip of them at the
  // edge of the cell pulls the all-column plane towards itself. So: re-fit through the half
  // of the sub-columns that lie lowest above the first plane (when 3 or more well-spread
  // remain), judge every sub-column against that plane, drop those more than
  // max(0.05 m, 2.5 x MAD of the residuals) above it, and refit through the rest. When
  // nothing is dropped the first plane stays.
  auto residuals = [&](double qa, double qbx, double qby) {
    std::vector<double> r(cols.size());
    for (std::size_t i = 0; i < cols.size(); ++i) r[i] = cols[i].v - (qa + qbx * cols[i].x + qby * cols[i].y);
    return r;
  };
  std::vector<double> r0 = residuals(pa, pbx, pby);
  std::vector<double> tmp = r0;
  const double med0 = medianOf(tmp);
  std::vector<SubCol> lower;
  for (std::size_t i = 0; i < cols.size(); ++i)
    if (r0[i] <= med0) lower.push_back(cols[i]);
  double la = pa, lbx = pbx, lby = pby;
  if (!fitPlane(lower, prm.resolution, la, lbx, lby)) { la = pa; lbx = pbx; lby = pby; }
  const std::vector<double> r1 = residuals(la, lbx, lby);
  tmp = r1;
  const double med1 = medianOf(tmp);
  for (auto& t : tmp) t = std::fabs(t - med1);
  const double thr = std::max(0.05, 2.5 * medianOf(tmp));
  // the lower-half plane sits below the ground by about the noise: measure from the median residual
  std::vector<SubCol> kept;
  for (std::size_t i = 0; i < cols.size(); ++i)
    if (r1[i] - med1 <= thr) kept.push_back(cols[i]);
  if (kept.size() == cols.size() || kept.empty()) return true;
  fitOrMean(kept, prm, a, bx, by);
  return true;
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
    std::vector<Cand> cand;
    std::size_t begin = b0;
    for (std::size_t k = b0 + 1; k <= e0; ++k) {
      if (k < e0 && points[s[k]].z() - points[s[k - 1]].z() <= params_.layer_gap) continue;
      if (k - begin >= min_pts) {
        Cand c;
        if (candidateOf(points, s, begin, k, cx, cy, params_, c.a, c.bx, c.by)) cand.push_back(c);
      }
      begin = k;
    }
    if (cand.empty()) continue;
    std::sort(cand.begin(), cand.end(), [](const Cand& p, const Cand& q) { return p.a < q.a; });
    cand.shrink_to_fit();
    num_candidates_ += cand.size();
    cells_[keys[c]] = std::move(cand);
  }
}

std::optional<double> GroundHeightMap::heightAt(double x, double y, double z_hint) const {
  if (!std::isfinite(z_hint)) return std::nullopt;
  std::int64_t i0, j0;
  double tx, ty;
  if (!surround(x, y, i0, j0, tx, ty)) return std::nullopt;
  struct Kept { double v, w; };
  std::array<Kept, 4> kept;
  int n = 0;
  for (int di = 0; di <= 1; ++di)
    for (int dj = 0; dj <= 1; ++dj) {
      const std::vector<Cand>* c = cell(i0 + di, j0 + dj);
      if (!c) continue;
      const double cx = (static_cast<double>(i0 + di) + 0.5) * params_.resolution;
      const double cy = (static_cast<double>(j0 + dj) + 0.5) * params_.resolution;
      double best = 0.0, best_d = 0.0;
      bool have = false;
      for (const Cand& k : *c) {  // ascending a: a tie keeps the lower one
        const double v = k.a + k.bx * (x - cx) + k.by * (y - cy);
        const double d = std::fabs(v - z_hint);
        if (!have || d < best_d) { best = v; best_d = d; have = true; }
      }
      if (best_d > params_.max_step) continue;
      kept[n++] = {best, (di ? tx : 1.0 - tx) * (dj ? ty : 1.0 - ty)};
    }
  if (n == 0) return std::nullopt;
  // Reference: the value nearest to the hint (tie: higher weight). Others on another surface go.
  int ref = 0;
  for (int k = 1; k < n; ++k) {
    const double dk = std::fabs(kept[k].v - z_hint), dr = std::fabs(kept[ref].v - z_hint);
    if (dk < dr || (dk == dr && kept[k].w > kept[ref].w)) ref = k;
  }
  double sw = 0.0, swz = 0.0, ez = 0.0;
  int m = 0;
  for (int k = 0; k < n; ++k) {
    if (std::fabs(kept[k].v - kept[ref].v) > params_.layer_gap) continue;
    sw += kept[k].w; swz += kept[k].w * kept[k].v; ez += kept[k].v; ++m;
  }
  if (sw < 1e-12) return ez / m;
  return swz / sw;
}

std::optional<double> GroundHeightMap::heightNearest(double x, double y, double z_guess) const {
  if (!std::isfinite(z_guess)) return std::nullopt;
  std::int64_t i0, j0;
  double tx, ty;
  if (!surround(x, y, i0, j0, tx, ty)) return std::nullopt;
  bool any = false;
  double anchor = 0.0, anchor_d = 0.0;
  auto scan = [&](const std::vector<Cand>* c) {
    if (!c) return;
    for (const Cand& k : *c) {
      const double d = std::fabs(k.a - z_guess);
      if (!any || d < anchor_d || (d == anchor_d && k.a < anchor)) { anchor = k.a; anchor_d = d; any = true; }
    }
  };
  std::int64_t ci, cj;
  if (cellIndex(x, y, ci, cj)) scan(cell(ci, cj));
  if (!any)
    for (int di = 0; di <= 1; ++di)
      for (int dj = 0; dj <= 1; ++dj) scan(cell(i0 + di, j0 + dj));
  if (!any) return std::nullopt;
  return heightAt(x, y, anchor);
}

std::size_t GroundHeightMap::approxBytes() const {
  auto alloc = [](std::size_t bytes) { return (bytes + 8 + 15) / 16 * 16; };  // malloc: header + 16 B granularity
  std::size_t total = cells_.bucket_count() * sizeof(void*);
  total += cells_.size() * alloc(sizeof(void*) + sizeof(std::int64_t) + sizeof(std::vector<Cand>));
  for (const auto& kv : cells_) total += alloc(kv.second.capacity() * sizeof(Cand));
  return total;
}

}  // namespace prism_loc_core
