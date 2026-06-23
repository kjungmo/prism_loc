#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
namespace prism_loc_core {

class NdtMap {
 public:
  NdtMap(const std::vector<Eigen::Vector3d>& points, double resolution, int min_points = 6);
  double score(const Eigen::Vector3d& p) const;
  bool hasVoxel(const Eigen::Vector3d& p) const;
  std::size_t numVoxels() const { return voxels_.size(); }
  double resolution() const { return resolution_; }

 private:
  struct Voxel {
    Eigen::Vector3d mean;
    Eigen::Matrix3d inv_cov;
  };
  std::int64_t keyOf(const Eigen::Vector3d& p) const;

  double resolution_;
  std::unordered_map<std::int64_t, Voxel> voxels_;
};

}  // namespace prism_loc_core
