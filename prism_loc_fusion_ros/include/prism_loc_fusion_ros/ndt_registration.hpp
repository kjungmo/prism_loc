#pragma once
#include <memory>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace prism_loc_fusion_ros {
struct NdtResult {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  // Mean squared distance from the aligned source points to their nearest target point,
  // over the points that have one within fitness_max_range (all points when the range is
  // 0, which is PCL's getFitnessScore). 1e9 when no point is within the range.
  double fitness{1e9};
  // Share of the source points with a target point within fitness_max_range (1.0 when the
  // range is 0). A scan that mostly sees unmapped space has a low value.
  double inlier_fraction{0.0};
  int iterations{0};
  bool converged{false};
};

// Scan-to-map NDT. The map is turned into NDT cells and a kd-tree once, in setTarget();
// align() reuses both for every scan.
//
// Not thread safe: align() changes the PCL object it keeps. The fusion node calls it from
// one callback group (mutually exclusive), so there is never more than one call at a time.
class NdtRegistration {
 public:
  // fitness_max_range: distance in m; 0 = unlimited (the previous, PCL definition).
  NdtRegistration(double resolution, double step_size, double epsilon, int max_iter,
                  double fitness_max_range = 0.0);
  ~NdtRegistration();
  NdtRegistration(const NdtRegistration&) = delete;
  NdtRegistration& operator=(const NdtRegistration&) = delete;

  // Builds the NDT cells and the search tree of the target. Takes seconds for a map of
  // millions of points: call it once at start-up.
  void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target);
  bool hasTarget() const { return has_target_; }
  void setFitnessMaxRange(double range) { fitness_max_range_ = range; }
  double fitnessMaxRange() const { return fitness_max_range_; }
  // How many times a target was built (test seam: align() must not build one).
  int targetBuilds() const { return target_builds_; }

  NdtResult align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, const Eigen::Isometry3d& init);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  double fitness_max_range_;
  bool has_target_{false};
  int target_builds_{0};
};
}  // namespace prism_loc_fusion_ros
