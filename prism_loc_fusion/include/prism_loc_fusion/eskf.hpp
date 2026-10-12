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
  // Position fix of a point fixed to the body at `lever` (body frame), e.g. a GNSS
  // antenna: h(x) = p + R(q) lever.
  //
  // The attitude and the gyro bias are "consider" states of this update
  // (Schmidt-Kalman): the innovation covariance H P H^T carries the attitude uncertainty
  // through the lever arm, so the position is not trusted beyond what the attitude
  // allows, but the fix does not correct attitude or gyro bias; it corrects position,
  // velocity and accelerometer bias. Reason, measured: with a plain Kalman update the
  // heading (which a single antenna does not observe) and the vertical gyro bias drifted
  // by about 1 rad in a minute on GNSS and IMU alone, and with the wide attitude
  // covariance a filter starts from, the first fixes after a reseed tilted it.
  // A lever of exactly zero is updatePosition(), with identical arithmetic.
  void updatePositionLever(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& lever,
                           const Eigen::Matrix3d& R);
  // h(x) above, and its Jacobian with respect to the 15-dim error state. The attitude
  // error is local (q <- q * Exp(dtheta), as inject() applies it), so
  // dh/dp = I and dh/dtheta = -R(q) [lever]x.
  static Eigen::Vector3d leverArmPosition(const NominalState& x, const Eigen::Vector3d& lever);
  static Eigen::Matrix<double, 3, 15> leverArmJacobian(const Eigen::Quaterniond& q,
                                                       const Eigen::Vector3d& lever);
  const NominalState& state() const { return x_; }
  const Mat15& covariance() const { return P_; }
 private:
  void inject(const Vec15& dx);
  NominalState x_;
  Mat15 P_{Mat15::Identity()};
  EskfParams params_;
};
}  // namespace prism_loc_fusion
