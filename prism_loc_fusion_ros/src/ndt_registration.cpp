#include "prism_loc_fusion_ros/ndt_registration.hpp"
#include <pcl/registration/ndt.h>
namespace prism_loc_fusion_ros {
NdtRegistration::NdtRegistration(double resolution, double step_size, double epsilon, int max_iter)
    : resolution_(resolution), step_size_(step_size), epsilon_(epsilon), max_iter_(max_iter) {}
void NdtRegistration::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) { target_ = target; }
NdtResult NdtRegistration::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                 const Eigen::Isometry3d& init) const {
  NdtResult r;
  if (!target_ || target_->empty() || !source || source->empty()) return r;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(static_cast<float>(resolution_));
  ndt.setStepSize(step_size_);
  ndt.setTransformationEpsilon(epsilon_);
  ndt.setMaximumIterations(max_iter_);
  ndt.setInputTarget(target_);
  ndt.setInputSource(source);
  pcl::PointCloud<pcl::PointXYZ> out;
  ndt.align(out, init.matrix().cast<float>());
  r.converged = ndt.hasConverged();
  r.fitness = ndt.getFitnessScore();
  r.pose.matrix() = ndt.getFinalTransformation().cast<double>();
  return r;
}
}  // namespace prism_loc_fusion_ros
