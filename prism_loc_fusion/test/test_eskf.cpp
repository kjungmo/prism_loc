#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/so3.hpp"
using namespace prism_loc_fusion;

static Eskf::Mat15 initCov(double s) { return Eskf::Mat15::Identity() * s; }

TEST(Eskf, StaticImuStaysPut) {
  Eskf f;                                   // level, zero bias
  f.initialize(NominalState{}, initCov(0.01));
  const Eigen::Vector3d acc(0, 0, 9.81);    // specific force at rest (g cancels)
  for (int i = 0; i < 1000; ++i) f.predict(acc, Eigen::Vector3d::Zero(), 0.01);
  EXPECT_LT(f.state().p.norm(), 1e-6);
  EXPECT_LT(f.state().v.norm(), 1e-6);
}

TEST(Eskf, ConstantAccelKinematics) {
  Eskf f; f.initialize(NominalState{}, initCov(0.01));
  const Eigen::Vector3d acc(1.0, 0.0, 9.81);  // world a_w = [1,0,0]
  for (int i = 0; i < 100; ++i) f.predict(acc, Eigen::Vector3d::Zero(), 0.01);  // t=1s
  EXPECT_NEAR(f.state().v.x(), 1.0, 1e-6);
  EXPECT_NEAR(f.state().p.x(), 0.5, 0.02);
}

TEST(Eskf, PositionUpdatePullsToMeasurement) {
  Eskf f;
  NominalState x0; x0.p = Eigen::Vector3d(5, 5, 5);
  f.initialize(x0, initCov(1.0));
  for (int i = 0; i < 50; ++i)
    f.updatePosition(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.01);
  EXPECT_LT(f.state().p.norm(), 0.05);
}

TEST(Eskf, PoseUpdateCorrectsPositionAndYaw) {
  Eskf f; f.initialize(NominalState{}, initCov(1.0));
  const Eigen::Vector3d p_meas(1.0, 2.0, 0.0);
  const Eigen::Quaterniond q_meas = so3Exp(Eigen::Vector3d(0, 0, 0.5));
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  for (int i = 0; i < 50; ++i) f.updatePose(p_meas, q_meas, R);
  EXPECT_LT((f.state().p - p_meas).norm(), 0.05);
  EXPECT_NEAR(so3Log(f.state().q.conjugate() * q_meas).norm(), 0.0, 0.05);
}

// Optional ESKF reset Jacobian (Sola 2017, Sec. 7.2): after injecting the error,
// P <- G P G^T with G = blkdiag(I, I, I - [dtheta/2]x, I, I). Default off.
TEST(Eskf, ResetJacobianOptionTransformsCovarianceAfterInjection) {
  EskfParams off_p, on_p; on_p.reset_jacobian = true;
  Eskf off(off_p), on(on_p);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity() * 0.05;
  P0(0, 7) = P0(7, 0) = 0.01; P0(6, 8) = P0(8, 6) = -0.004;  // some cross terms
  off.initialize(NominalState{}, P0); on.initialize(NominalState{}, P0);
  const Eigen::Quaterniond q_meas = so3Exp(Eigen::Vector3d(0.1, -0.2, 0.3));
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  off.updatePose(Eigen::Vector3d(0.3, 0.1, 0.0), q_meas, R);
  on.updatePose(Eigen::Vector3d(0.3, 0.1, 0.0), q_meas, R);
  // Nominal state is identical; only the covariance is re-projected.
  EXPECT_LT((off.state().p - on.state().p).norm(), 1e-12);
  EXPECT_LT(so3Log(off.state().q.conjugate() * on.state().q).norm(), 1e-12);
  const Eigen::Vector3d dth = so3Log(off.state().q);   // injected from identity
  Eskf::Mat15 G = Eskf::Mat15::Identity();
  G.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(0.5 * dth);
  const Eskf::Mat15 expect = G * off.covariance() * G.transpose();
  EXPECT_LT((on.covariance() - expect).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_GT((on.covariance() - off.covariance()).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Eskf, ResetJacobianIsIdentityForPositionOnlyUpdate) {
  EskfParams on_p; on_p.reset_jacobian = true;
  Eskf off, on(on_p);
  off.initialize(NominalState{}, initCov(0.2)); on.initialize(NominalState{}, initCov(0.2));
  off.updatePosition(Eigen::Vector3d(1, 2, 3), Eigen::Matrix3d::Identity() * 0.01);
  on.updatePosition(Eigen::Vector3d(1, 2, 3), Eigen::Matrix3d::Identity() * 0.01);
  // Diagonal P0 => the position update injects no rotation => G = I.
  EXPECT_LT((on.covariance() - off.covariance()).cwiseAbs().maxCoeff(), 1e-15);
}
