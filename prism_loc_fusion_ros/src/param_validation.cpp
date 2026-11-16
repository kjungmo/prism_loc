#include "prism_loc_fusion_ros/param_validation.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace prism_loc_fusion_ros {
namespace {

template <typename T>
void require(bool ok, const char* name, T value, const char* rule) {
  if (ok) return;
  std::ostringstream os;
  os << "prism_loc_fusion: invalid parameter " << name << " = " << value << " (must be " << rule
     << ")";
  throw std::invalid_argument(os.str());
}

}  // namespace

void validateFusionParams(const FusionNodeParams& p, const prism_loc_fusion::EskfParams& ep) {
  require(p.transform_tolerance >= 0.0, "transform_tolerance", p.transform_tolerance, ">= 0");
  require(p.ndt_resolution > 0.0, "ndt_resolution", p.ndt_resolution, "> 0");
  require(p.ndt_step_size > 0.0, "ndt_step_size", p.ndt_step_size, "> 0");
  require(p.ndt_epsilon > 0.0, "ndt_epsilon", p.ndt_epsilon, "> 0");
  require(p.ndt_max_iter >= 1, "ndt_max_iter", p.ndt_max_iter, ">= 1");
  require(p.ndt_max_fitness > 0.0, "ndt_max_fitness", p.ndt_max_fitness, "> 0");
  require(std::isfinite(p.ndt_fitness_max_range) && p.ndt_fitness_max_range >= 0.0,
          "ndt_fitness_max_range", p.ndt_fitness_max_range, "finite and >= 0 (0 = unlimited)");
  require(p.ndt_min_inlier_fraction >= 0.0 && p.ndt_min_inlier_fraction <= 1.0,
          "ndt_min_inlier_fraction", p.ndt_min_inlier_fraction, "in [0, 1]");
  require(p.points_voxel_leaf > 0.0, "points_voxel_leaf", p.points_voxel_leaf, "> 0");
  require(std::isfinite(p.points_max_age) && p.points_max_age >= 0.0, "points_max_age",
          p.points_max_age, "finite and >= 0 (0 = off)");
  require(p.pose_pos_std > 0.0, "pose_pos_std", p.pose_pos_std, "> 0");
  require(p.pose_rot_std > 0.0, "pose_rot_std", p.pose_rot_std, "> 0");
  require(p.gnss_max_pos_cov > 0.0, "gnss_max_pos_cov", p.gnss_max_pos_cov, "> 0");
  require(p.imu_queue_depth >= 1, "imu_queue_depth", p.imu_queue_depth, ">= 1");
  require(p.startup_timeout_s > 0.0, "startup_timeout_s", p.startup_timeout_s, "> 0");
  require(p.input_timeout_s > 0.0, "input_timeout_s", p.input_timeout_s, "> 0");
  require(p.input_timeout_periods > 0.0, "input_timeout_periods", p.input_timeout_periods, "> 0");
  require(p.correction_timeout_s > 0.0, "correction_timeout_s", p.correction_timeout_s, "> 0");
  require(ep.sigma_acc > 0.0, "sigma_acc", ep.sigma_acc, "> 0");
  require(ep.sigma_gyro > 0.0, "sigma_gyro", ep.sigma_gyro, "> 0");
  require(ep.sigma_acc_bias >= 0.0, "sigma_acc_bias", ep.sigma_acc_bias, ">= 0");
  require(ep.sigma_gyro_bias >= 0.0, "sigma_gyro_bias", ep.sigma_gyro_bias, ">= 0");
}

std::string fitnessGateWarning(const FusionNodeParams& p) {
  const double r = p.ndt_fitness_max_range;
  if (!(r > 0.0) || !(p.ndt_max_fitness >= r * r)) return "";
  std::ostringstream os;
  os << "fusion3d: ndt_max_fitness " << p.ndt_max_fitness << " >= ndt_fitness_max_range^2 ("
     << r * r << "): the score only counts points within " << r
     << " m of the map, so no scan can exceed this limit and the fitness gate rejects nothing. "
        "Lower ndt_max_fitness (default 0.06), or set ndt_fitness_max_range to 0 to score every "
        "point (then 2.0 is the matching limit)";
  return os.str();
}

}  // namespace prism_loc_fusion_ros
