#include "prism_loc/param_validation.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace prism_loc {
namespace {

template <typename T>
void require(bool ok, const char* name, T value, const char* rule) {
  if (ok) return;
  std::ostringstream os;
  os << "prism_loc: invalid parameter " << name << " = " << value << " (must be " << rule << ")";
  throw std::invalid_argument(os.str());
}

bool finite(double v) { return std::isfinite(v); }

}  // namespace

void validateNodeParams(const NodeParams& np) {
  require(finite(np.update_min_d) && np.update_min_d >= 0.0, "update_min_d", np.update_min_d, ">= 0");
  require(finite(np.update_min_a) && np.update_min_a >= 0.0, "update_min_a", np.update_min_a, ">= 0");
  require(finite(np.transform_tolerance) && np.transform_tolerance >= 0.0, "transform_tolerance",
          np.transform_tolerance, ">= 0");
  require(np.startup_timeout_s > 0.0, "startup_timeout_s", np.startup_timeout_s, "> 0");
  require(np.input_timeout_s > 0.0, "input_timeout_s", np.input_timeout_s, "> 0");
  require(np.input_timeout_periods > 0.0, "input_timeout_periods", np.input_timeout_periods, "> 0");
  require(np.min_neff_fraction >= 0.0 && np.min_neff_fraction <= 1.0, "min_neff_fraction",
          np.min_neff_fraction, "in [0, 1]");
}

void validateFilterParams(const prism_loc_core::ParticleFilterParams& pp,
                          const prism_loc_core::MotionParams& mp) {
  require(pp.min_particles >= 1, "min_particles", pp.min_particles, ">= 1");
  require(pp.max_particles >= pp.min_particles, "max_particles", pp.max_particles,
          ">= min_particles");
  require(pp.resample_threshold >= 0.0 && pp.resample_threshold <= 1.0, "resample_threshold",
          pp.resample_threshold, "in [0, 1]");
  require(pp.kld_err > 0.0, "kld_err", pp.kld_err, "> 0");
  require(pp.kld_z > 0.0, "kld_z", pp.kld_z, "> 0");
  require(pp.kld_bin_xy > 0.0, "kld_bin_xy", pp.kld_bin_xy, "> 0");
  require(pp.kld_bin_yaw > 0.0, "kld_bin_yaw", pp.kld_bin_yaw, "> 0");
  require(mp.alpha1 >= 0.0, "alpha1", mp.alpha1, ">= 0");
  require(mp.alpha2 >= 0.0, "alpha2", mp.alpha2, ">= 0");
  require(mp.alpha3 >= 0.0, "alpha3", mp.alpha3, ">= 0");
  require(mp.alpha4 >= 0.0, "alpha4", mp.alpha4, ">= 0");
}

void validateLaserParams(const prism_loc_core::LaserParams& lp, double laser_min_range,
                         double laser_max_range) {
  require(lp.max_beams >= 1, "max_beams", lp.max_beams, ">= 1");
  require(lp.z_hit >= 0.0, "z_hit", lp.z_hit, ">= 0");
  require(lp.z_rand >= 0.0, "z_rand", lp.z_rand, ">= 0");
  require(lp.z_hit + lp.z_rand > 0.0, "z_hit + z_rand", lp.z_hit + lp.z_rand, "> 0");
  require(lp.sigma_hit > 0.0, "sigma_hit", lp.sigma_hit, "> 0");
  require(lp.max_dist > 0.0, "likelihood_max_dist", lp.max_dist, "> 0");
  require(laser_min_range >= 0.0, "laser_min_range", laser_min_range, ">= 0");
  require(laser_max_range >= 0.0, "laser_max_range", laser_max_range, ">= 0");
  require(laser_max_range == 0.0 || laser_max_range > laser_min_range, "laser_max_range",
          laser_max_range, "0 (use the scan's range_max) or > laser_min_range");
}

void validateBbsParams(const prism_loc_core::BbsParams& bp,
                       const prism_loc_core::RelocVerifierParams& rv) {
  require(bp.linear_window > 0.0, "bbs_linear_window", bp.linear_window, "> 0");
  require(bp.max_linear_window > 0.0, "bbs_max_linear_window", bp.max_linear_window, "> 0");
  require(bp.angular_window > 0.0, "bbs_angular_window", bp.angular_window, "> 0");
  require(bp.angular_step > 0.0, "bbs_angular_step", bp.angular_step, "> 0");
  require(bp.max_depth >= 1, "bbs_max_depth", bp.max_depth, ">= 1");
  require(bp.max_beams >= 1, "bbs_max_beams", bp.max_beams, ">= 1");
  require(bp.min_score_fraction > 0.0 && bp.min_score_fraction <= 1.0, "bbs_min_score_fraction",
          bp.min_score_fraction, "in (0, 1]");
  require(rv.top_k >= 1, "bbs_verify_top_k", rv.top_k, ">= 1");
  require(rv.verify_scans >= 1, "bbs_verify_scans", rv.verify_scans, ">= 1");
  require(rv.evidence_gain > 0.0, "bbs_verify_evidence_gain", rv.evidence_gain, "> 0");
  require(rv.min_posterior > 0.0 && rv.min_posterior <= 1.0, "bbs_verify_min_posterior",
          rv.min_posterior, "in (0, 1]");
  require(rv.nms_xy >= 0.0, "bbs_verify_nms_xy", rv.nms_xy, ">= 0");
  require(rv.nms_yaw >= 0.0, "bbs_verify_nms_yaw", rv.nms_yaw, ">= 0");
  require(rv.track_linear_window >= 0.0, "bbs_verify_track_linear_window",
          rv.track_linear_window, ">= 0");
  require(rv.track_angular_window >= 0.0, "bbs_verify_track_angular_window",
          rv.track_angular_window, ">= 0");
}

void validateNdtParams(double ndt_resolution, int voxel_min_points,
                       const prism_loc_core::NdtParams& np) {
  require(ndt_resolution > 0.0, "ndt_resolution", ndt_resolution, "> 0");
  require(voxel_min_points >= 1, "voxel_min_points", voxel_min_points, ">= 1");
  require(np.max_points >= 1, "max_points", np.max_points, ">= 1");
}

}  // namespace prism_loc
