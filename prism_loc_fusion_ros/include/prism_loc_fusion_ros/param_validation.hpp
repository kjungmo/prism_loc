#pragma once
#include "prism_loc_fusion/types.hpp"

// Range checks run once at node construction; each throws std::invalid_argument naming
// the offending parameter. The shipped defaults all pass.
namespace prism_loc_fusion_ros {

struct FusionNodeParams {
  double transform_tolerance{0.1};
  double ndt_resolution{1.0}, ndt_step_size{0.1}, ndt_epsilon{0.01};
  int ndt_max_iter{30};
  double ndt_max_fitness{2.0}, points_voxel_leaf{0.5};
  double pose_pos_std{0.1}, pose_rot_std{0.05};
  double gnss_max_pos_cov{25.0};
  int imu_queue_depth{5};
  double startup_timeout_s{30.0}, input_timeout_s{1.0}, input_timeout_periods{5.0};
  double correction_timeout_s{5.0};
};

void validateFusionParams(const FusionNodeParams& p, const prism_loc_fusion::EskfParams& ep);

}  // namespace prism_loc_fusion_ros
