#include "prism_loc_fusion_ros/ndt_registration.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <pcl/registration/ndt.h>
namespace prism_loc_fusion_ros {

namespace {
using PclNdt = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
// Counts the calls that make PCL build its target (NDT cells and, at the next align, the search tree).
struct CountingNdt : PclNdt {
  int target_builds{0};
  void setInputTarget(const PointCloudTargetConstPtr& cloud) override {
    ++target_builds;
    PclNdt::setInputTarget(cloud);
  }
};
}  // namespace

struct NdtRegistration::Impl {
  CountingNdt ndt;
  pcl::PointCloud<pcl::PointXYZ> aligned;  // reused output buffer
};

NdtRegistration::NdtRegistration(double resolution, double step_size, double epsilon, int max_iter,
                                 double fitness_max_range)
    : impl_(new Impl), fitness_max_range_(fitness_max_range) {
  impl_->ndt.setResolution(static_cast<float>(resolution));
  impl_->ndt.setStepSize(step_size);
  impl_->ndt.setTransformationEpsilon(epsilon);
  impl_->ndt.setMaximumIterations(max_iter);
}

NdtRegistration::~NdtRegistration() = default;

int NdtRegistration::targetBuilds() const { return impl_->ndt.target_builds; }

void NdtRegistration::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
  has_target_ = false;
  if (!target || target->empty()) return;
  // setInputTarget builds the NDT cells. PCL builds the kd-tree of the target at the next
  // align, so align() once on a single point now: the first scan would otherwise wait for it.
  impl_->ndt.setInputTarget(target);
  pcl::PointCloud<pcl::PointXYZ>::Ptr probe(new pcl::PointCloud<pcl::PointXYZ>());
  probe->emplace_back(target->points.front());
  impl_->ndt.setInputSource(probe);
  impl_->ndt.align(impl_->aligned, Eigen::Matrix4f::Identity());
  has_target_ = true;
}

NdtResult NdtRegistration::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                 const Eigen::Isometry3d& init) {
  NdtResult r;
  if (!has_target_ || !source || source->empty()) return r;
  auto& ndt = impl_->ndt;
  ndt.setInputSource(source);
  ndt.align(impl_->aligned, init.matrix().cast<float>());
  r.converged = ndt.hasConverged();
  r.iterations = ndt.getFinalNumIteration();
  r.pose.matrix() = ndt.getFinalTransformation().cast<double>();

  // Fitness over the aligned points, using the search tree PCL built for the target.
  const auto tree = ndt.getSearchMethodTarget();
  const double range = fitness_max_range_;
  const double max_sq = range > 0.0 ? range * range : std::numeric_limits<double>::max();
  std::vector<int> idx(1);
  std::vector<float> sq(1);
  double sum = 0.0;
  std::size_t n_in = 0;
  for (const auto& p : impl_->aligned.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (tree->nearestKSearch(p, 1, idx, sq) < 1) continue;
    if (static_cast<double>(sq[0]) > max_sq) continue;
    sum += sq[0];
    ++n_in;
  }
  r.inlier_fraction = static_cast<double>(n_in) / static_cast<double>(impl_->aligned.size());
  if (n_in > 0) r.fitness = sum / static_cast<double>(n_in);
  return r;
}
}  // namespace prism_loc_fusion_ros
