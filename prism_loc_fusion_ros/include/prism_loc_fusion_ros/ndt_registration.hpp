#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace prism_loc_fusion_ros {
struct NdtResult {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double fitness{1e9};
  bool converged{false};
};
class NdtRegistration {
 public:
  NdtRegistration(double resolution, double step_size, double epsilon, int max_iter);
  void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target);
  bool hasTarget() const { return target_ && !target_->empty(); }
  NdtResult align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                  const Eigen::Isometry3d& init) const;
 private:
  double resolution_, step_size_, epsilon_;
  int max_iter_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
};
}  // namespace prism_loc_fusion_ros
