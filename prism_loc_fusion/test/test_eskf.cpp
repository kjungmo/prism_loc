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
