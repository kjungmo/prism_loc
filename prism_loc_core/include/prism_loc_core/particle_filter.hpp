#pragma once
#include <Eigen/Core>
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/rng.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/measurement_model.hpp"
namespace prism_loc_core {

struct ParticleFilterParams {
  int min_particles{500};
  int max_particles{2000};
  double resample_threshold{0.5};  // resample if Neff < threshold * N
  double kld_err{0.05};
  double kld_z{2.33};              // upper standard-normal quantile (~0.99)
  double kld_bin_xy{0.5};
  double kld_bin_yaw{0.17};
};

class ParticleFilter {
 public:
  ParticleFilter(ParticleFilterParams params, Rng& rng) : params_(params), rng_(rng) {}
  void initializeGaussian(const Pose2D& mean, const Pose2D& stddev, int n);
  void predict(const OdometryMotionModel& mm, const Pose2D& prev_odom, const Pose2D& cur_odom);
  void correct(const MeasurementModel& model);
  void resample();
  void resampleLowVariance(int n);
  Pose2D estimate(Eigen::Matrix<double, 6, 6>* cov = nullptr) const;
  double effectiveSampleSize() const;
  const ParticleSet& particles() const { return particles_; }
  ParticleSet& particles() { return particles_; }

 private:
  void normalizeWeights();
  ParticleFilterParams params_;
  Rng& rng_;
  ParticleSet particles_;
};

}  // namespace prism_loc_core
