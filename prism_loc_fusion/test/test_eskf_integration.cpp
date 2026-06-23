#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
using namespace prism_loc_fusion;

// Ground truth: constant velocity along +x, level, no rotation. The ESKF starts
// with the WRONG velocity (zero) and must recover it from GNSS position + NDT pose
// fixes, demonstrating IMU-prediction + measurement fusion (position innovations
// correct velocity through the predicted P(dp,dv) cross-covariance).
TEST(EskfIntegration, TracksMovingTargetFromWrongInit) {
  const double v0 = 1.0;                 // m/s
  const double imu_dt = 0.01;            // 100 Hz
  const int meas_every = 10;             // measurement at 10 Hz
  const int steps = 500;                 // 5 s
  Rng rng(2024);

  Eskf f;
  NominalState x0;                       // p=0, v=0 (WRONG: true v=1), q=I
  f.initialize(x0, Eskf::Mat15::Identity() * 0.5);

  const Eigen::Vector3d acc(0.0, 0.0, 9.81);   // a_w = 0 (constant velocity)
  const Eigen::Matrix3d Rpos = Eigen::Matrix3d::Identity() * (0.05 * 0.05);
  Eigen::Matrix<double, 6, 6> Rpose = Eigen::Matrix<double, 6, 6>::Identity();
  Rpose.topLeftCorner<3, 3>() *= (0.02 * 0.02);
  Rpose.bottomRightCorner<3, 3>() *= (0.01 * 0.01);

  for (int k = 1; k <= steps; ++k) {
    f.predict(acc + Eigen::Vector3d(rng.gaussian(0, 0.01), rng.gaussian(0, 0.01), rng.gaussian(0, 0.01)),
              Eigen::Vector3d(rng.gaussian(0, 0.001), rng.gaussian(0, 0.001), rng.gaussian(0, 0.001)),
              imu_dt);
    if (k % meas_every == 0) {
      const double t = k * imu_dt;
      const Eigen::Vector3d truth(v0 * t, 0.0, 0.0);
      f.updatePosition(truth + Eigen::Vector3d(rng.gaussian(0, 0.05), rng.gaussian(0, 0.05), rng.gaussian(0, 0.05)), Rpos);
      f.updatePose(truth + Eigen::Vector3d(rng.gaussian(0, 0.02), rng.gaussian(0, 0.02), rng.gaussian(0, 0.02)),
                   Eigen::Quaterniond::Identity(), Rpose);
    }
  }
  const Eigen::Vector3d truth_final(v0 * steps * imu_dt, 0.0, 0.0);  // [5,0,0]
  EXPECT_NEAR(f.state().p.x(), truth_final.x(), 0.30);
  EXPECT_LT(std::abs(f.state().p.y()), 0.30);
  EXPECT_NEAR(f.state().v.x(), v0, 0.30);
}
