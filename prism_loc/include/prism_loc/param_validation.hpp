#pragma once
#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/relocalization.hpp"

// Range checks run once at node construction. Each throws std::invalid_argument naming
// the offending parameter, so a bad YAML aborts startup instead of running with a
// value the filter cannot use. The shipped defaults all pass.
namespace prism_loc {

struct NodeParams {
  double update_min_d{0.2}, update_min_a{0.2}, transform_tolerance{0.1};
  double startup_timeout_s{30.0}, input_timeout_s{1.0}, input_timeout_periods{5.0};
  double min_neff_fraction{0.005};
};

void validateNodeParams(const NodeParams& np);
void validateFilterParams(const prism_loc_core::ParticleFilterParams& pp,
                          const prism_loc_core::MotionParams& mp);
void validateLaserParams(const prism_loc_core::LaserParams& lp, double laser_min_range,
                         double laser_max_range);
void validateBbsParams(const prism_loc_core::BbsParams& bp,
                       const prism_loc_core::RelocVerifierParams& rv);
void validateNdtParams(double ndt_resolution, int voxel_min_points,
                       const prism_loc_core::NdtParams& np);

}  // namespace prism_loc
