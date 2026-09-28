// eskf_nees.cpp -- Monte Carlo NEES consistency study for prism_loc_fusion::Eskf.
//
// HONEST SYNTHETIC EXPERIMENT. Drives the REAL Eskf over N = 100 independent
// runs of the same scenario as experiments/eskf_fusion.cpp (60 s 3-D
// figure-eight, IMU 200 Hz, 6-DoF pose 10 Hz, GNSS 1 Hz, both aiding streams
// withheld for t in [25, 35) s, fusion3d.yaml process noise, measurement noise
// equal to the filter's R). Differences from eskf_fusion.cpp, required for a
// NEES test: in every run the initial estimation error (position, velocity,
// attitude) AND the true constant IMU biases are drawn from N(0, P0), so the
// truth is a sample from the filter's own prior.
//
// Every 0.5 s (121 epochs, after any update at that instant) we compute the
// normalized estimation error squared eps = e^T P^-1 e for the full 15-D error
// state and for each 3-D block (p, v, theta, b_a, b_g), with
//   e = (p - p^, v - v^, Log(q^^-1 q), b_a - b_a^, b_g - b_g^),
// and average it over the N runs. Under consistency N * mean(eps) ~ chi2(N*dof);
// the two-sided 95% acceptance interval for mean(eps) is
//   [chi2_{0.025}(N dof) / N, chi2_{0.975}(N dof) / N]
// (Wilson-Hilferty approximation, relative error < 1e-3 at these dof; see
// Bar-Shalom et al. 2001, Sec. 5.4).
//
// Two filter variants see IDENTICAL sensor data and initial errors (common
// random numbers): "gi" = v0.1 behaviour (reset Jacobian G = I) and "rj" =
// EskfParams::reset_jacobian = true. Nothing is tuned on these runs: every
// parameter is the fusion3d.yaml / eskf_fusion.cpp value.
//
// Build & run from the repository root (writes docs/paper/data/eskf_nees.csv):
//   g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3
//     prism_loc_fusion/src/*.cpp experiments/eskf_nees.cpp -o /tmp/exp_nees
//   /tmp/exp_nees
// Fully deterministic (no wall-clock column).

#include <cmath>
#include <array>
#include <cstdio>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
#include "prism_loc_fusion/so3.hpp"

using namespace prism_loc_fusion;

static constexpr std::uint64_t kSeed = 20260929u;  // run k uses kSeed + k
static constexpr int kRuns = 100;

struct Truth { Eigen::Vector3d p, v, a; double yaw, yaw_rate; };

static Truth truthAt(double t) {  // identical to experiments/eskf_fusion.cpp
  const double A = 8.0, B = 4.0, w = 2.0 * M_PI / 30.0, Cz = 1.5, wz = 2.0 * M_PI / 60.0;
  Truth T;
  T.p = Eigen::Vector3d(A * std::sin(w * t), B * std::sin(2.0 * w * t), Cz * std::sin(wz * t));
  T.v = Eigen::Vector3d(A * w * std::cos(w * t), 2.0 * B * w * std::cos(2.0 * w * t),
                        Cz * wz * std::cos(wz * t));
  T.a = Eigen::Vector3d(-A * w * w * std::sin(w * t), -4.0 * B * w * w * std::sin(2.0 * w * t),
                        -Cz * wz * wz * std::sin(wz * t));
  const double vx = T.v.x(), vy = T.v.y(), ax = T.a.x(), ay = T.a.y();
  T.yaw = std::atan2(vy, vx);
  T.yaw_rate = (vx * ay - vy * ax) / (vx * vx + vy * vy);
  return T;
}

static Eigen::Quaterniond yawQuat(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

static double chi2q(double k, double z) {  // Wilson-Hilferty quantile
  const double c = 2.0 / (9.0 * k);
  const double b = 1.0 - c + z * std::sqrt(c);
  return k * b * b * b;
}

static Eigen::Vector3d gauss3(Rng& r, double s) {
  return Eigen::Vector3d(r.gaussian(0, s), r.gaussian(0, s), r.gaussian(0, s));
}

int main() {
  EskfParams params;  // fusion3d.yaml defaults
  params.sigma_acc = 0.01; params.sigma_gyro = 0.001;
  params.sigma_acc_bias = 1e-4; params.sigma_gyro_bias = 1e-5;
  params.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  const double imu_dt = 1.0 / 200.0, duration = 60.0;
  const int steps = static_cast<int>(std::llround(duration / imu_dt));
  const int pose_every = 20, gnss_every = 200, log_every = 100;
  const double drop_t0 = 25.0, drop_t1 = 35.0;
  const double ndt_pos_std = 0.02, ndt_rot_std = 0.01, gnss_pos_std = 0.10;
  const Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * (gnss_pos_std * gnss_pos_std);
  Eigen::Matrix<double, 6, 6> R_pose = Eigen::Matrix<double, 6, 6>::Identity();
  R_pose.topLeftCorner<3, 3>() *= ndt_pos_std * ndt_pos_std;
  R_pose.bottomRightCorner<3, 3>() *= ndt_rot_std * ndt_rot_std;
  const double s0_p = 0.10, s0_v = 0.10, s0_th = 0.02, s0_ba = 0.10, s0_bg = 0.02;
  Eskf::Mat15 P0 = Eskf::Mat15::Zero();
  P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * s0_p * s0_p;
  P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * s0_v * s0_v;
  P0.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * s0_th * s0_th;
  P0.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * s0_ba * s0_ba;
  P0.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * s0_bg * s0_bg;

  const int n_log = steps / log_every + 1;  // 121
  // sums[variant][epoch][block]: block 0..4 = p, v, th, ba, bg; 5 = full 15-D
  std::vector<std::vector<std::array<double, 6>>> sums(
      2, std::vector<std::array<double, 6>>(n_log, std::array<double, 6>{}));
  std::vector<int> aid(n_log, 0);

  for (int run = 0; run < kRuns; ++run) {
    Rng rng(kSeed + static_cast<std::uint64_t>(run));
    const Eigen::Vector3d ba_true = gauss3(rng, s0_ba), bg_true = gauss3(rng, s0_bg);
    const Truth T0 = truthAt(0.0);
    NominalState x0;
    x0.p = T0.p - gauss3(rng, s0_p);
    x0.v = T0.v - gauss3(rng, s0_v);
    x0.q = (yawQuat(T0.yaw) * so3Exp(-gauss3(rng, s0_th))).normalized();
    x0.ba.setZero(); x0.bg.setZero();

    EskfParams prj = params; prj.reset_jacobian = true;
    Eskf fl[2] = {Eskf(params), Eskf(prj)};
    for (auto& f : fl) f.initialize(x0, P0);

    for (int k = 0; k <= steps; ++k) {
      const double t = k * imu_dt;
      if (k > 0) {
        const Truth Tk = truthAt(t);
        const Eigen::Matrix3d R = yawQuat(Tk.yaw).toRotationMatrix();
        const Eigen::Vector3d f_body = R.transpose() * (Tk.a - params.gravity);
        const Eigen::Vector3d accel = f_body + ba_true + gauss3(rng, params.sigma_acc);
        const Eigen::Vector3d gyro =
            Eigen::Vector3d(0.0, 0.0, Tk.yaw_rate) + bg_true + gauss3(rng, params.sigma_gyro);
        for (auto& f : fl) f.predict(accel, gyro, imu_dt);
      }
      const bool aiding = !(t >= drop_t0 && t < drop_t1);
      if (aiding && k > 0 && k % pose_every == 0) {
        const Truth Tk = truthAt(t);
        const Eigen::Vector3d pm = Tk.p + gauss3(rng, ndt_pos_std);
        const Eigen::Quaterniond qm = (yawQuat(Tk.yaw) * so3Exp(gauss3(rng, ndt_rot_std))).normalized();
        for (auto& f : fl) f.updatePose(pm, qm, R_pose);
      }
      if (aiding && k > 0 && k % gnss_every == 0) {
        const Eigen::Vector3d pm = truthAt(t).p + gauss3(rng, gnss_pos_std);
        for (auto& f : fl) f.updatePosition(pm, R_gnss);
      }
      if (k % log_every == 0) {
        const int e = k / log_every;
        aid[e] = aiding ? 1 : 0;
        const Truth Tk = truthAt(t);
        for (int vi = 0; vi < 2; ++vi) {
          const NominalState& x = fl[vi].state();
          Eskf::Vec15 err;
          err.segment<3>(0) = Tk.p - x.p;
          err.segment<3>(3) = Tk.v - x.v;
          err.segment<3>(6) = so3Log(x.q.conjugate() * yawQuat(Tk.yaw));
          err.segment<3>(9) = ba_true - x.ba;
          err.segment<3>(12) = bg_true - x.bg;
          const Eskf::Mat15& P = fl[vi].covariance();
          for (int b = 0; b < 5; ++b) {
            const Eigen::Vector3d eb = err.segment<3>(3 * b);
            sums[vi][e][b] += eb.dot(P.block<3, 3>(3 * b, 3 * b).ldlt().solve(eb));
          }
          sums[vi][e][5] += err.dot(P.ldlt().solve(err));
        }
      }
    }
  }

  const double z = 1.959963985;
  const double lo15 = chi2q(15.0 * kRuns, -z) / kRuns, hi15 = chi2q(15.0 * kRuns, z) / kRuns;
  const double lo3 = chi2q(3.0 * kRuns, -z) / kRuns, hi3 = chi2q(3.0 * kRuns, z) / kRuns;
  FILE* f = std::fopen("docs/paper/data/eskf_nees.csv", "w");
  if (!f) { std::perror("open csv"); return 1; }
  std::fprintf(f, "time_s,aiding,runs,chi2_lo15,chi2_hi15,chi2_lo3,chi2_hi3");
  for (const char* v : {"gi", "rj"})
    for (const char* b : {"p", "v", "th", "ba", "bg", "all"}) std::fprintf(f, ",nees_%s_%s", b, v);
  std::fprintf(f, "\n");
  for (int e = 0; e < n_log; ++e) {
    std::fprintf(f, "%.3f,%d,%d,%.4f,%.4f,%.4f,%.4f", e * log_every * imu_dt, aid[e], kRuns,
                 lo15, hi15, lo3, hi3);
    for (int vi = 0; vi < 2; ++vi)
      for (int b = 0; b < 6; ++b) std::fprintf(f, ",%.5f", sums[vi][e][b] / kRuns);
    std::fprintf(f, "\n");
  }
  std::fclose(f);
  std::fprintf(stderr, "N=%d runs; 95%% band 15-D [%.3f, %.3f], 3-D [%.3f, %.3f]\n", kRuns, lo15,
               hi15, lo3, hi3);
  return 0;
}
