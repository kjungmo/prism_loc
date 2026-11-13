#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
namespace prism_loc_core {

class NdtMap {
 public:
  // min_sigma (m, a standard deviation) is an absolute floor on every cell's spread along
  // each principal axis, on top of the relative floor (0.001 x the largest eigenvalue). It
  // keeps a flat cell from scoring a point a few centimetres off its surface as ~0.
  // 0 (or a negative / NaN value) applies the relative floor only. min_sigma shapes score()
  // (the likelihood) only; matchScore() keeps the relative floor alone. With min_sigma > 0 the
  // map stores a second inverse covariance per cell (a separate map entry, about 100 bytes) for it.
  NdtMap(const std::vector<Eigen::Vector3d>& points, double resolution, int min_points = 6,
         double min_sigma = 0.0);
  double score(const Eigen::Vector3d& p) const;
  // Same as score() but with the relative covariance floor only, whatever min_sigma is.
  // The scan-to-map match diagnostic uses it so that min_sigma does not widen "on a surface".
  double matchScore(const Eigen::Vector3d& p) const;
  bool hasVoxel(const Eigen::Vector3d& p) const;
  std::size_t numVoxels() const { return voxels_.size(); }
  double resolution() const { return resolution_; }
  double minSigma() const { return min_sigma_; }

 private:
  struct Voxel {
    Eigen::Vector3d mean;
    Eigen::Matrix3d inv_cov;
  };
  std::int64_t keyOf(const Eigen::Vector3d& p) const;

  double resolution_;
  double min_sigma_;
  std::unordered_map<std::int64_t, Voxel> voxels_;
  // Inverse covariances without the min_sigma floor; filled only when min_sigma_ > 0.
  std::unordered_map<std::int64_t, Eigen::Matrix3d> match_inv_cov_;
};

}  // namespace prism_loc_core
