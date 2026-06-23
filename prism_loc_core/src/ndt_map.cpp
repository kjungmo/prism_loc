#include "prism_loc_core/ndt_map.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
namespace prism_loc_core {

namespace {
constexpr std::int64_t kOffset = 1 << 20;  // ~1e6 voxel half-range per axis
constexpr std::int64_t kBits = 21;
}  // namespace

std::int64_t NdtMap::keyOf(const Eigen::Vector3d& p) const {
  const std::int64_t i = static_cast<std::int64_t>(std::floor(p.x() / resolution_)) + kOffset;
  const std::int64_t j = static_cast<std::int64_t>(std::floor(p.y() / resolution_)) + kOffset;
  const std::int64_t k = static_cast<std::int64_t>(std::floor(p.z() / resolution_)) + kOffset;
  return (i << (2 * kBits)) | (j << kBits) | k;
}

NdtMap::NdtMap(const std::vector<Eigen::Vector3d>& points, double resolution, int min_points)
    : resolution_(resolution) {
  struct Acc {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_sq = Eigen::Matrix3d::Zero();
    int n = 0;
  };
  std::unordered_map<std::int64_t, Acc> acc;
  for (const auto& p : points) {
    Acc& a = acc[keyOf(p)];
    a.sum += p;
    a.sum_sq += p * p.transpose();
    ++a.n;
  }
  for (auto& kv : acc) {
    const Acc& a = kv.second;
    if (a.n < min_points) continue;
    const Eigen::Vector3d mean = a.sum / a.n;
    Eigen::Matrix3d cov = a.sum_sq / a.n - mean * mean.transpose();
    // Regularize: inflate small eigenvalues to >= 0.001 * largest (Magnusson 2009).
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d evals = es.eigenvalues();
    const double max_ev = evals.maxCoeff();
    const double floor_ev = (max_ev > 0.0) ? 0.001 * max_ev : 1e-6;
    for (int d = 0; d < 3; ++d) evals[d] = std::max(evals[d], floor_ev);
    const Eigen::Matrix3d reg = es.eigenvectors() * evals.asDiagonal() * es.eigenvectors().transpose();
    voxels_[kv.first] = Voxel{mean, reg.inverse()};
  }
}

bool NdtMap::hasVoxel(const Eigen::Vector3d& p) const {
  return voxels_.find(keyOf(p)) != voxels_.end();
}

double NdtMap::score(const Eigen::Vector3d& p) const {
  auto it = voxels_.find(keyOf(p));
  if (it == voxels_.end()) return 0.0;
  const Eigen::Vector3d d = p - it->second.mean;
  const double m = d.transpose() * it->second.inv_cov * d;
  return std::exp(-0.5 * m);
}

}  // namespace prism_loc_core
