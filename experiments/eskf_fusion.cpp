// eskf_fusion.cpp — HONEST synthetic ESKF fusion experiment for the PRISM-Loc paper.
//
// Drives the REAL prism_loc_fusion::Eskf (include/src under prism_loc_fusion/).
// A smooth 60 s 3-D figure-eight trajectory with gentle altitude change is
// generated analytically; from it we synthesize the IDEAL specific force and
// body angular rate, then corrupt them with the exact white-noise densities and
// constant biases the ESKF process model assumes (fusion3d.yaml defaults).
//
// Sensor cadence:  IMU 200 Hz  |  6-DoF pose (NDT) 10 Hz  |  position (GNSS) 1 Hz.
// A 10 s GNSS+pose dropout window (t in [25,35) s) exposes IMU-only drift and the
// subsequent covariance / error recovery once aiding returns.
//
// Every 0.5 s we log: time, position-error norm, the filter's own 3-sigma
// position bound (3*sqrt(trace P[0:3,0:3])), and which sensors were active.
//
// Everything is deterministic: a single fixed seed (SEED) drives all noise.
// Build (no ROS, no CMake):
//   g++ -O2 -std=c++17 \
//     -I prism_loc_fusion/include -I /usr/include/eigen3 \
//     prism_loc_fusion/src/*.cpp experiments/eskf_fusion.cpp -o /tmp/exp_eskf
//   /tmp/exp_eskf > docs/paper/data/eskf_track.csv

#include <cmath>
#include <cstdio>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
#include "prism_loc_fusion/so3.hpp"

using namespace prism_loc_fusion;

static constexpr std::uint64_t SEED = 20240517u;  // fixed for reproducibility (NOT time)

// ---- Analytic ground-truth trajectory ------------------------------------
// Horizontal figure-eight (lemniscate of Gerono style): x ~ sin, y ~ sin(2.).
// Vertical: one gentle up-down over the run.
struct Truth {
  Eigen::Vector3d p, v, a;   // world position, velocity, kinematic acceleration
  double yaw, yaw_rate;      // heading (body tracks velocity), yaw rate
};

static Truth truthAt(double t) {
  const double A = 8.0;                 // m, x amplitude
  const double B = 4.0;                 // m, y amplitude
  const double w = 2.0 * M_PI / 30.0;   // rad/s -> figure-eight period 30 s (2 loops in 60 s)
  const double Cz = 1.5;                // m, altitude amplitude
  const double wz = 2.0 * M_PI / 60.0;  // rad/s -> one gentle up-down over 60 s

  Truth T;
  T.p = Eigen::Vector3d(A * std::sin(w * t),
                        B * std::sin(2.0 * w * t),
                        Cz * std::sin(wz * t));
  T.v = Eigen::Vector3d(A * w * std::cos(w * t),
                        2.0 * B * w * std::cos(2.0 * w * t),
                        Cz * wz * std::cos(wz * t));
  T.a = Eigen::Vector3d(-A * w * w * std::sin(w * t),
                        -4.0 * B * w * w * std::sin(2.0 * w * t),
                        -Cz * wz * wz * std::sin(wz * t));

  const double vx = T.v.x(), vy = T.v.y();
  const double ax = T.a.x(), ay = T.a.y();
  T.yaw = std::atan2(vy, vx);
  const double sp = vx * vx + vy * vy;   // horizontal speed^2 (bounded away from 0)
  T.yaw_rate = (vx * ay - vy * ax) / sp;
  return T;
}

static Eigen::Quaterniond yawQuat(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

int main() {
  // ---- ESKF process parameters: fusion3d.yaml defaults ----
  EskfParams params;
  params.sigma_acc = 0.01;        // m/s^2   accel white-noise std
  params.sigma_gyro = 0.001;      // rad/s   gyro white-noise std
  params.sigma_acc_bias = 1e-4;   // m/s^3   accel bias random walk
  params.sigma_gyro_bias = 1e-5;  // rad/s^2 gyro bias random walk
  params.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  // ---- True constant IMU biases the filter must estimate (starts at zero) ----
  const Eigen::Vector3d ba_true(0.03, -0.02, 0.05);      // m/s^2
  const Eigen::Vector3d bg_true(0.002, -0.001, 0.003);   // rad/s

  // ---- Timing ----
  const double imu_dt = 1.0 / 200.0;   // 200 Hz
  const double duration = 60.0;        // s
  const int steps = static_cast<int>(std::llround(duration / imu_dt));  // 12000
  const int pose_every = 20;           // 200/20 = 10 Hz  (NDT 6-DoF pose)
  const int gnss_every = 200;          // 200/200 = 1 Hz  (GNSS position)
  const int log_every = 100;           // 200/100 -> log at 2 Hz (every 0.5 s)
  const double drop_t0 = 25.0, drop_t1 = 35.0;  // 10 s aiding blackout

  // ---- Measurement noise (injected == modeled: consistent tuning) ----
  const double ndt_pos_std = 0.02, ndt_rot_std = 0.01;   // m, rad  (NDT is tight)
  const double gnss_pos_std = 0.10;                       // m       (GNSS is coarse)
  const Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * (gnss_pos_std * gnss_pos_std);
  Eigen::Matrix<double, 6, 6> R_pose = Eigen::Matrix<double, 6, 6>::Identity();
  R_pose.topLeftCorner<3, 3>() *= (ndt_pos_std * ndt_pos_std);
  R_pose.bottomRightCorner<3, 3>() *= (ndt_rot_std * ndt_rot_std);

  Rng rng(SEED);

  // ---- Initialize filter at the true pose/velocity; biases unknown (0) ----
  const Truth T0 = truthAt(0.0);
  NominalState x0;
  x0.p = T0.p;
  x0.v = T0.v;
  x0.q = yawQuat(T0.yaw);
  x0.ba.setZero();
  x0.bg.setZero();
  Eskf::Mat15 P0 = Eskf::Mat15::Zero();
  P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.10 * 0.10);   // position
  P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (0.10 * 0.10);   // velocity
  P0.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (0.02 * 0.02);   // attitude
  P0.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (0.10 * 0.10);   // accel bias
  P0.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (0.02 * 0.02); // gyro bias

  Eskf f(params);
  f.initialize(x0, P0);

  // CSV header
  std::printf("time_s,err_norm_m,sigma3_m,gnss_active,pose_active\n");

  // Accumulators for windowed RMSE (before / during / after dropout), from logged rows.
  double se_before = 0, se_during = 0, se_after = 0;
  int n_before = 0, n_during = 0, n_after = 0;

  for (int k = 0; k <= steps; ++k) {
    const double t = k * imu_dt;

    if (k > 0) {
      // Synthesize IMU at the *previous->current* interval midpoint's truth.
      const Truth Tk = truthAt(t);
      const Eigen::Matrix3d R = yawQuat(Tk.yaw).toRotationMatrix();
      // Ideal specific force in body frame: f = R^T (a_world - g).
      const Eigen::Vector3d f_world = Tk.a - params.gravity;   // a - (0,0,-9.81) = a + (0,0,9.81)
      const Eigen::Vector3d f_body = R.transpose() * f_world;
      const Eigen::Vector3d w_body(0.0, 0.0, Tk.yaw_rate);     // level, only yaw changes

      Eigen::Vector3d accel = f_body + ba_true +
          Eigen::Vector3d(rng.gaussian(0, params.sigma_acc),
                          rng.gaussian(0, params.sigma_acc),
                          rng.gaussian(0, params.sigma_acc));
      Eigen::Vector3d gyro = w_body + bg_true +
          Eigen::Vector3d(rng.gaussian(0, params.sigma_gyro),
                          rng.gaussian(0, params.sigma_gyro),
                          rng.gaussian(0, params.sigma_gyro));

      f.predict(accel, gyro, imu_dt);
    }

    const bool aiding = !(t >= drop_t0 && t < drop_t1);
    bool gnss_on = false, pose_on = false;

    // 6-DoF pose update (NDT) at 10 Hz
    if (aiding && k > 0 && (k % pose_every == 0)) {
      const Truth Tk = truthAt(t);
      const Eigen::Vector3d p_meas = Tk.p +
          Eigen::Vector3d(rng.gaussian(0, ndt_pos_std),
                          rng.gaussian(0, ndt_pos_std),
                          rng.gaussian(0, ndt_pos_std));
      // small orientation noise around true yaw (local-frame rotation vector)
      const Eigen::Vector3d rot_noise(rng.gaussian(0, ndt_rot_std),
                                      rng.gaussian(0, ndt_rot_std),
                                      rng.gaussian(0, ndt_rot_std));
      const Eigen::Quaterniond q_meas = (yawQuat(Tk.yaw) * so3Exp(rot_noise)).normalized();
      f.updatePose(p_meas, q_meas, R_pose);
      pose_on = true;
    }

    // GNSS position update at 1 Hz
    if (aiding && k > 0 && (k % gnss_every == 0)) {
      const Truth Tk = truthAt(t);
      const Eigen::Vector3d p_meas = Tk.p +
          Eigen::Vector3d(rng.gaussian(0, gnss_pos_std),
                          rng.gaussian(0, gnss_pos_std),
                          rng.gaussian(0, gnss_pos_std));
      f.updatePosition(p_meas, R_gnss);
      gnss_on = true;
    }

    if (k % log_every == 0) {
      const Truth Tk = truthAt(t);
      const double err = (f.state().p - Tk.p).norm();
      const double trP = f.covariance().block<3, 3>(0, 0).trace();
      const double sigma3 = 3.0 * std::sqrt(trP);
      std::printf("%.3f,%.6f,%.6f,%d,%d\n", t, err, sigma3,
                  gnss_on ? 1 : 0, pose_on ? 1 : 0);

      if (t < drop_t0)            { se_before += err * err; ++n_before; }
      else if (t < drop_t1)       { se_during += err * err; ++n_during; }
      else                        { se_after  += err * err; ++n_after; }
    }
  }

  const double rmse_before = std::sqrt(se_before / n_before);
  const double rmse_during = std::sqrt(se_during / n_during);
  const double rmse_after = std::sqrt(se_after / n_after);
  std::fprintf(stderr,
      "RMSE before=%.4f m  during=%.4f m  after=%.4f m  "
      "(n=%d/%d/%d)\n",
      rmse_before, rmse_during, rmse_after, n_before, n_during, n_after);
  return 0;
}
