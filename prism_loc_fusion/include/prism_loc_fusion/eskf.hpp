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
  // Full Jacobian (below), Joseph covariance, and a gain from which the states in
  // EskfParams::lever_consider are removed (Schmidt-Kalman "consider" states): by default
  // the heading and the vertical gyro bias. A single antenna does not separate the heading
  // from the position of the body, and a plain update there let heading and vertical bias
  // drift while the covariance shrank (measured, experiments/eskf_lever.cpp: the full update
  // is over-confident in most GNSS-and-IMU-only runs). Their uncertainty still enters the
  // innovation covariance H P H^T, so the position is not trusted beyond what the heading
  // allows. Roll and pitch, the horizontal gyro biases, position, velocity and the
  // accelerometer bias are corrected. That needs a roll and pitch variance of about 1e-4 rad^2
  // at the start (attitudeCovariance(), tiltVarianceFromAccel()): with 0.5 rad^2 the first
  // fixes after a start or a reseed tilt the filter.
  // Without NDT or another heading source the heading drifts with the vertical gyro bias.
  // A lever of exactly zero is updatePosition(), with identical arithmetic.
  void updatePositionLever(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& lever,
                           const Eigen::Matrix3d& R);
  // h(x) above, and its Jacobian with respect to the 15-dim error state. The attitude
  // error is local (q <- q * Exp(dtheta), as inject() applies it), so
  // dh/dp = I and dh/dtheta = -R(q) [lever]x.
  static Eigen::Vector3d leverArmPosition(const NominalState& x, const Eigen::Vector3d& lever);
  static Eigen::Matrix<double, 3, 15> leverArmJacobian(const Eigen::Quaterniond& q,
                                                       const Eigen::Vector3d& lever);
  // Covariance of the attitude error for an attitude q whose roll and pitch are known to
  // within var_tilt (rad^2, for example from the accelerometer) and whose heading is known
  // to var_heading: var_tilt about the two axes across the vertical, var_heading about the
  // vertical, no correlation. The error is local, so the vertical is R(q)^T z.
  static Eigen::Matrix3d attitudeCovariance(const Eigen::Quaterniond& q, double var_tilt,
                                            double var_heading);
  // Variance of the roll and pitch that follow from one accelerometer reading (specific
  // force, base frame): the accelerometer bias prior and the white noise over g, plus the
  // departure of the reading from g, which is acceleration the reading cannot tell from tilt.
  static double tiltVarianceFromAccel(const Eigen::Vector3d& specific_force, double var_acc_bias,
                                      const EskfParams& params);
  // Replace the attitude block of the covariance and drop the correlation of the attitude
  // with every other state. Valid only for a filter that has had no measurement since
  // initialize() (the node uses it for a lever arm found after the filter started).
  void replaceAttitudeCovariance(const Eigen::Matrix3d& P_att);
  const NominalState& state() const { return x_; }
  const Mat15& covariance() const { return P_; }
 private:
  void inject(const Vec15& dx);
  NominalState x_;
  Mat15 P_{Mat15::Identity()};
  EskfParams params_;
  double rate_{0.0};   // norm of the bias-corrected angular rate of the last predict()
};
}  // namespace prism_loc_fusion
