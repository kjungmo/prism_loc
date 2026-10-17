#include "prism_loc_fusion/eskf.hpp"
#include <algorithm>
#include <cmath>
#include "prism_loc_fusion/so3.hpp"
namespace prism_loc_fusion {

void Eskf::initialize(const NominalState& x0, const Mat15& P0) {
  x_ = x0; x_.q.normalize(); P_ = P0;
}

void Eskf::predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d a = accel - x_.ba;
  const Eigen::Vector3d w = gyro - x_.bg;
  rate_ = w.norm();
  const Eigen::Vector3d a_w = R * a + params_.gravity;

  // nominal integration
  x_.p += x_.v * dt + 0.5 * a_w * dt * dt;
  x_.v += a_w * dt;
  x_.q = (x_.q * so3Exp(w * dt)).normalized();

  // error-state transition (local orientation error)
  Mat15 F = Mat15::Identity();
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  F.block<3, 3>(0, 3) = I3 * dt;                                   // dp / dv
  F.block<3, 3>(3, 6) = -R * skew(a) * dt;                         // dv / dtheta
  F.block<3, 3>(3, 9) = -R * dt;                                   // dv / dba
  F.block<3, 3>(6, 6) = so3Exp(w * dt).toRotationMatrix().transpose();  // dtheta / dtheta
  F.block<3, 3>(6, 12) = -I3 * dt;                                 // dtheta / dbg

  // process noise
  Mat15 Q = Mat15::Zero();
  const double sa2 = params_.sigma_acc * params_.sigma_acc;
  const double sg2 = params_.sigma_gyro * params_.sigma_gyro;
  const double sba2 = params_.sigma_acc_bias * params_.sigma_acc_bias;
  const double sbg2 = params_.sigma_gyro_bias * params_.sigma_gyro_bias;
  Q.block<3, 3>(3, 3) = I3 * sa2 * dt * dt;
  Q.block<3, 3>(6, 6) = I3 * sg2 * dt * dt;
  Q.block<3, 3>(9, 9) = I3 * sba2 * dt;
  Q.block<3, 3>(12, 12) = I3 * sbg2 * dt;

  P_ = F * P_ * F.transpose() + Q;
}

void Eskf::updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R) {
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d y = p_meas - x_.p;
  const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
  const Vec15 dx = K * y;
  P_ = (Mat15::Identity() - K * H) * P_;
  inject(dx);
}

Eigen::Vector3d Eskf::leverArmPosition(const NominalState& x, const Eigen::Vector3d& lever) {
  return x.p + x.q.toRotationMatrix() * lever;
}

Eigen::Matrix<double, 3, 15> Eskf::leverArmJacobian(const Eigen::Quaterniond& q,
                                                    const Eigen::Vector3d& lever) {
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, 6) = -q.toRotationMatrix() * skew(lever);
  return H;
}

Eigen::Matrix3d Eskf::attitudeCovariance(const Eigen::Quaterniond& q, double var_tilt,
                                         double var_heading) {
  const Eigen::Vector3d up = q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d along = up * up.transpose();
  return var_tilt * (Eigen::Matrix3d::Identity() - along) + var_heading * along;
}

double Eskf::tiltVarianceFromAccel(const Eigen::Vector3d& f, double var_acc_bias,
                                   const EskfParams& params) {
  const double g = params.gravity.norm();
  const double off = (f.norm() - g) / g;
  return (var_acc_bias + params.sigma_acc * params.sigma_acc) / (g * g) + off * off;
}

void Eskf::replaceAttitudeCovariance(const Eigen::Matrix3d& P_att) {
  P_.block<3, 15>(6, 0).setZero();
  P_.block<15, 3>(0, 6).setZero();
  P_.block<3, 3>(6, 6) = P_att;
}

void Eskf::updatePositionLever(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& lever,
                               const Eigen::Matrix3d& R) {
  if (lever.isZero(0.0)) {  // the body origin itself: the plain position update, bit for bit
    updatePosition(p_meas, R);
    return;
  }
  unsigned consider = params_.lever_consider;
  if (params_.lever_motion_speed > 0.0 || params_.lever_motion_rate > 0.0) {
    // The speed counts only when it is clear of its own uncertainty: right after a start the
    // velocity estimate is far off and a standing robot would read as moving.
    const double sigma_v = std::sqrt(P_.block<3, 3>(3, 3).trace() / 3.0);
    const double speed = x_.v.norm();
    const bool moving = (params_.lever_motion_speed > 0.0 && speed > params_.lever_motion_speed && speed > 3.0 * sigma_v) ||
                        (params_.lever_motion_rate > 0.0 && rate_ > params_.lever_motion_rate);
    if (moving) consider = params_.lever_consider_moving;
  }
  const int iterations = std::max(1, params_.lever_iterations);
  const Eigen::Vector3d up = x_.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d along = up * up.transpose();
  const Eigen::Matrix3d across = Eigen::Matrix3d::Identity() - along;
  // Remove from the gain the parts that are not to be corrected.
  auto limit = [&](Eigen::Matrix<double, 15, 3>& K) {
    if (consider == kConsiderAttitudeAndGyroBias) {
      K.block<3, 3>(6, 0).setZero();
      K.block<3, 3>(12, 0).setZero();
      return;
    }
    if (consider & kConsiderHeading) K.block<3, 3>(6, 0) -= along * K.block<3, 3>(6, 0);
    if (consider & kConsiderTilt) K.block<3, 3>(6, 0) -= across * K.block<3, 3>(6, 0);
    if (consider & kConsiderGyroBiasVertical) K.block<3, 3>(12, 0) -= along * K.block<3, 3>(12, 0);
    if (consider & kConsiderGyroBiasHorizontal) K.block<3, 3>(12, 0) -= across * K.block<3, 3>(12, 0);
  };
  Vec15 dx = Vec15::Zero();
  Eigen::Matrix<double, 3, 15> H;
  Eigen::Matrix<double, 15, 3> K;
  for (int it = 0; it < iterations; ++it) {
    // Linearise at the state the previous pass arrived at (the first pass: at the estimate).
    NominalState xi = x_;
    if (it > 0) {
      xi.p += dx.segment<3>(0);
      xi.q = x_.q * so3Exp(dx.segment<3>(6));
    }
    H = leverArmJacobian(xi.q, lever);
    Eigen::Vector3d y = p_meas - leverArmPosition(xi, lever);
    if (it > 0) y += H * dx;
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    K = P_ * H.transpose() * S.inverse();
    limit(K);
    dx = K * y;
  }
  // Joseph form: valid for a gain that is not the optimal one.
  const Mat15 A = Mat15::Identity() - K * H;
  P_ = A * P_ * A.transpose() + K * R * K.transpose();
  inject(dx);
}

void Eskf::updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas,
                      const Eigen::Matrix<double, 6, 6>& R) {
  Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();   // position
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();   // orientation (local)
  Eigen::Matrix<double, 6, 1> y;
  y.head<3>() = p_meas - x_.p;
  y.tail<3>() = so3Log(x_.q.conjugate() * q_meas.normalized());
  const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * S.inverse();
  const Vec15 dx = K * y;
  P_ = (Mat15::Identity() - K * H) * P_;
  inject(dx);
}

void Eskf::inject(const Vec15& dx) {
  x_.p += dx.segment<3>(0);
  x_.v += dx.segment<3>(3);
  x_.q = (x_.q * so3Exp(dx.segment<3>(6))).normalized();
  x_.ba += dx.segment<3>(9);
  x_.bg += dx.segment<3>(12);
  if (params_.reset_jacobian) {
    Mat15 G = Mat15::Identity();
    G.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(0.5 * dx.segment<3>(6));
    P_ = G * P_ * G.transpose();
  }
}

}  // namespace prism_loc_fusion
