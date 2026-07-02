#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/types.hpp"
namespace prism_loc_fusion {
class Eskf {
 public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;
  explicit Eskf(const EskfParams& params = {}) : params_(params) { P_.setIdentity(); P_ *= 0.01; }
  void initialize(const NominalState& x0, const Mat15& P0);
  void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt);
  void updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas,
                  const Eigen::Matrix<double, 6, 6>& R);
  void updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R);
  const NominalState& state() const { return x_; }
  const Mat15& covariance() const { return P_; }
 private:
  void inject(const Vec15& dx);
  NominalState x_;
  Mat15 P_{Mat15::Identity()};
  EskfParams params_;
};
}  // namespace prism_loc_fusion
