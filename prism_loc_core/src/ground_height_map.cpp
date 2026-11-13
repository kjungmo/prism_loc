#include "prism_loc_core/ground_height_map.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
namespace prism_loc_core {

namespace {
// Same key scheme as NdtMap, in 2D: two 21-bit cell indices with an offset of 2^20.
constexpr std::int64_t kOffset = 1 << 20;
constexpr std::int64_t kBits = 21;
constexpr std::int64_t kMaxIndex = kOffset - 2;  // |cell index| limit; leaves room for neighbours

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
}

std::int64_t GroundHeightMap::keyOfCell(std::int64_t i, std::int64_t j) const {
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

GroundHeightMap::GroundHeightMap(const std::vector<Eigen::Vector3d>& points,
                                 const GroundHeightParams& params)
    : params_(params) {
  validate(params_);
  std::unordered_map<std::int64_t, std::vector<double>> zs;
  for (const auto& p : points) {
    if (!std::isfinite(p.z())) continue;
    std::int64_t i, j;
    if (!cellIndex(p.x(), p.y(), i, j)) continue;
    zs[keyOfCell(i, j)].push_back(p.z());
  }
  const std::size_t min_pts = static_cast<std::size_t>(params_.min_points);
  // Each cell is handled on its own, from its sorted z values, so the order in which the
  // hash map is visited does not matter.
  for (auto& kv : zs) {
    std::vector<double>& z = kv.second;
    if (z.size() < min_pts) continue;
    std::sort(z.begin(), z.end());
    std::vector<double> cand;
    std::size_t begin = 0;
    for (std::size_t k = 1; k <= z.size(); ++k) {
      if (k < z.size() && z[k] - z[k - 1] <= params_.layer_gap) continue;
      const std::size_t n = k - begin;  // group = z[begin, k)
      if (n >= min_pts) {
        const std::size_t idx =
            std::min(n - 1, static_cast<std::size_t>(std::floor(params_.ground_quantile * (n - 1))));
        cand.push_back(z[begin + idx]);
      }
      begin = k;
    }
    if (cand.empty()) continue;
    num_candidates_ += cand.size();
    cells_[kv.first] = std::move(cand);  // ascending: the groups are in ascending z
  }
}

std::optional<double> GroundHeightMap::heightAt(double x, double y, double z_hint) const {
  if (!std::isfinite(z_hint)) return std::nullopt;
  std::int64_t ci, cj;
  if (!cellIndex(x, y, ci, cj)) return std::nullopt;
  std::array<double, 9> found;
  int n = 0;
  for (std::int64_t di = -1; di <= 1; ++di)
    for (std::int64_t dj = -1; dj <= 1; ++dj) {
      const std::vector<double>* c = cell(ci + di, cj + dj);
      if (!c) continue;
      double best = (*c)[0];
      double best_d = std::fabs(best - z_hint);
      for (std::size_t k = 1; k < c->size(); ++k) {
        const double d = std::fabs((*c)[k] - z_hint);
        if (d < best_d) { best = (*c)[k]; best_d = d; }  // strict: a tie keeps the lower one
      }
      if (best_d <= params_.max_step) found[n++] = best;
    }
  if (n == 0) return std::nullopt;
  std::sort(found.begin(), found.begin() + n);
  if (n % 2 == 1) return found[n / 2];
  return 0.5 * (found[n / 2 - 1] + found[n / 2]);
}

std::optional<double> GroundHeightMap::heightNearest(double x, double y, double z_guess) const {
  if (!std::isfinite(z_guess)) return std::nullopt;
  std::int64_t ci, cj;
  if (!cellIndex(x, y, ci, cj)) return std::nullopt;
  bool any = false;
  double anchor = 0.0, anchor_d = 0.0;
  for (std::int64_t di = -1; di <= 1; ++di)
    for (std::int64_t dj = -1; dj <= 1; ++dj) {
      const std::vector<double>* c = cell(ci + di, cj + dj);
      if (!c) continue;
      for (double v : *c) {  // ascending: strict < keeps the lowest of equals
        const double d = std::fabs(v - z_guess);
        if (!any || d < anchor_d || (d == anchor_d && v < anchor)) {
          anchor = v;
          anchor_d = d;
          any = true;
        }
      }
    }
  if (!any) return std::nullopt;
  return heightAt(x, y, anchor);
}

std::size_t GroundHeightMap::approxBytes() const {
  // node (key, vector header, next pointer) + bucket slot + the candidate array
  constexpr std::size_t kNode = sizeof(std::int64_t) + sizeof(std::vector<double>) + 2 * sizeof(void*);
  return cells_.size() * kNode + cells_.bucket_count() * sizeof(void*) + num_candidates_ * sizeof(double);
}

}  // namespace prism_loc_core
