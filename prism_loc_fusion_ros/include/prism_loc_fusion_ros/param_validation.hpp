#pragma once
#include <string>
#include "prism_loc_fusion/types.hpp"

// Range checks run once at node construction; each throws std::invalid_argument naming
// the offending parameter. The shipped defaults all pass.
namespace prism_loc_fusion_ros {

struct FusionNodeParams {
  double transform_tolerance{0.1};
  double ndt_resolution{1.0}, ndt_step_size{0.1}, ndt_epsilon{1e-4};
  int ndt_max_iter{30};
  // ndt_max_fitness is a mean squared distance over the scan points that lie within
  // ndt_fitness_max_range of the map (0 = every point, PCL's score). Its default only fits
  // the default range: with range 0 the matching limit is 2.0.
  double ndt_max_fitness{0.06}, ndt_fitness_max_range{1.0}, ndt_min_inlier_fraction{0.5};
  double points_voxel_leaf{1.0};
  double points_max_age{0.5};  // s; a cloud older than this behind the latest IMU sample is dropped; 0 = off
  double pose_pos_std{0.1}, pose_rot_std{0.05};
  double gnss_max_pos_cov{25.0};
  int imu_queue_depth{5};
  double startup_timeout_s{30.0}, input_timeout_s{1.0}, input_timeout_periods{5.0};
  double correction_timeout_s{5.0};
};

void validateFusionParams(const FusionNodeParams& p, const prism_loc_fusion::EskfParams& ep);

// "" when the fitness limit can reject something. A limit of at least range^2 (range > 0)
// never rejects: no scan point counted by the score is farther than the range. Returns the
// text of a warning for the log in that case (a YAML that kept the old ndt_max_fitness 2.0).
// Also warns for range 0 with ndt_max_fitness below 1, which rejects almost every scan.
std::string fitnessGateWarning(const FusionNodeParams& p);

}  // namespace prism_loc_fusion_ros
