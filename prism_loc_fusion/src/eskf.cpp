#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/so3.hpp"
namespace prism_loc_fusion {

void Eskf::initialize(const NominalState& x0, const Mat15& P0) {
  x_ = x0; x_.q.normalize(); P_ = P0;
}

void Eskf::predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d a = accel - x_.ba;
  const Eigen::Vector3d w = gyro - x_.bg;
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
}

}  // namespace prism_loc_fusion
