#pragma once
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/rng.hpp"
namespace prism_loc_core {

struct MotionParams {
  double alpha1{0.2};
  double alpha2{0.2};
  double alpha3{0.2};
  double alpha4{0.2};
};

class OdometryMotionModel {
 public:
  explicit OdometryMotionModel(MotionParams params = {}) : p_(params) {}
  Pose2D sample(const Pose2D& particle, const Pose2D& prev_odom,
                const Pose2D& cur_odom, Rng& rng) const;
  void apply(ParticleSet& particles, const Pose2D& prev_odom,
             const Pose2D& cur_odom, Rng& rng) const;

 private:
  MotionParams p_;
};

}  // namespace prism_loc_core
