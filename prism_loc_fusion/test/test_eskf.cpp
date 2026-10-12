#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
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


// --- updatePositionLever: a position fix of a point off the body origin (GNSS antenna) ---

namespace {
// The error state as inject() applies it: additive, except the local attitude error.
NominalState boxplus(NominalState x, const Eskf::Vec15& dx) {
  x.p += dx.segment<3>(0);
  x.v += dx.segment<3>(3);
  x.q = (x.q * so3Exp(dx.segment<3>(6))).normalized();
  x.ba += dx.segment<3>(9);
  x.bg += dx.segment<3>(12);
  return x;
}
NominalState tiltedState() {
  NominalState x;
  x.p = Eigen::Vector3d(3.0, -2.0, 0.5);
  x.v = Eigen::Vector3d(0.4, 0.1, -0.2);
  x.q = so3Exp(Eigen::Vector3d(0.3, -0.5, 1.1));
  return x;
}
}  // namespace

// The analytic Jacobian against central finite differences of h(x [+] dx) in each of the
// 15 error-state directions, for a general attitude and lever arm.
TEST(EskfLever, JacobianMatchesFiniteDifferences) {
  const NominalState x = tiltedState();
  const Eigen::Vector3d lever(0.5, -0.2, 1.0);
  const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x.q, lever);
  const double eps = 1e-6;
  for (int i = 0; i < 15; ++i) {
    Eskf::Vec15 d = Eskf::Vec15::Zero();
    d[i] = eps;
    const Eigen::Vector3d num = (Eskf::leverArmPosition(boxplus(x, d), lever) -
                                 Eskf::leverArmPosition(boxplus(x, -d), lever)) / (2 * eps);
    EXPECT_LT((num - H.col(i)).norm(), 1e-8) << "error-state column " << i;
  }
  // the attitude block is not negligible: about |lever| metres per radian
  EXPECT_GT((H.block<3, 3>(0, 6).norm()), 1.0);
  // closed form at the identity attitude: dh/dtheta = -[lever]x
  const Eigen::Matrix<double, 3, 15> H0 = Eskf::leverArmJacobian(Eigen::Quaterniond::Identity(), lever);
  EXPECT_LT((H0.block<3, 3>(0, 6) + skew(lever)).norm(), 1e-15);
  EXPECT_LT((H0.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).norm(), 1e-15);
  EXPECT_EQ((H0.block<3, 3>(0, 3).norm() + H0.block<3, 6>(0, 9).norm()), 0.0);
}

// The gain the update applies: the Kalman gain with the attitude and gyro-bias rows zero.
Eigen::Matrix<double, 15, 3> considerGain(const NominalState& x, const Eskf::Mat15& P,
                                          const Eigen::Vector3d& lever, const Eigen::Matrix3d& R) {
  const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x.q, lever);
  Eigen::Matrix<double, 15, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  K.block<3, 3>(6, 0).setZero();
  K.block<3, 3>(12, 0).setZero();
  return K;
}

// One update ends in the state boxplus predicts from the correction K y, with the Joseph
// covariance for that gain.
TEST(EskfLever, UpdateIsTheConsiderGainWithJosephCovariance) {
  const NominalState x0 = tiltedState();
  Eskf::Mat15 P0 = initCov(0.04);
  P0(1, 8) = P0(8, 1) = 0.01;
  P0(4, 7) = P0(7, 4) = 0.006;
  P0(8, 14) = P0(14, 8) = 0.005;
  const Eigen::Vector3d lever(0.5, -0.2, 1.0), z(3.3, -1.7, 1.2);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  Eskf f;
  f.initialize(x0, P0);
  f.updatePositionLever(z, lever, R);
  const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x0.q, lever);
  const Eigen::Matrix<double, 15, 3> K = considerGain(x0, P0, lever, R);
  const Eskf::Vec15 dx = K * (z - Eskf::leverArmPosition(x0, lever));
  const NominalState expect = boxplus(x0, dx);
  EXPECT_LT((f.state().p - expect.p).norm(), 1e-12);
  EXPECT_LT((f.state().v - expect.v).norm(), 1e-12);
  EXPECT_LT((f.state().ba - expect.ba).norm(), 1e-12);
  EXPECT_GT((dx.segment<3>(0).norm()), 0.1);     // position, velocity and accelerometer bias moved
  EXPECT_GT((dx.segment<3>(3).norm()), 1e-3);
  EXPECT_TRUE(f.state().q.coeffs() == x0.q.coeffs());   // attitude and gyro bias: untouched, bit for bit
  EXPECT_TRUE(f.state().bg == x0.bg);
  const Eskf::Mat15 A = Eskf::Mat15::Identity() - K * H;
  const Eskf::Mat15 P1 = A * P0 * A.transpose() + K * R * K.transpose();
  EXPECT_LT((f.covariance() - P1).norm(), 1e-12);
  EXPECT_LT((f.covariance() - f.covariance().transpose()).norm(), 1e-15);
  // their variances are what they were
  for (int i : {6, 7, 8, 12, 13, 14}) EXPECT_NEAR(f.covariance()(i, i), P0(i, i), 1e-15) << i;
  // the simple form (I - K H) P is not valid for this gain and gives another covariance
  EXPECT_GT((P1 - A * P0).norm(), 1e-4);
}

// A lever of exactly zero takes the plain position update: same state and covariance
// bit for bit, also with cross-covariances that the consider gain would treat differently.
TEST(EskfLever, ZeroLeverIsThePlainPositionUpdateBitForBit) {
  Eskf a, b, c;
  Eskf::Mat15 P0 = initCov(0.3);
  P0(0, 8) = P0(8, 0) = 0.05;      // position correlated with the heading
  P0(1, 14) = P0(14, 1) = 0.01;    // and with the vertical gyro bias
  for (Eskf* f : {&a, &b, &c}) f->initialize(tiltedState(), P0);
  const Eigen::Vector3d z(2.5, -2.2, 0.9);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  a.updatePosition(z, R);
  b.updatePositionLever(z, Eigen::Vector3d::Zero(), R);
  auto same = [](const Eskf& x, const Eskf& y) {
    return x.state().p == y.state().p && x.state().v == y.state().v && x.state().q.coeffs() == y.state().q.coeffs() &&
           x.state().ba == y.state().ba && x.state().bg == y.state().bg && x.covariance() == y.covariance();
  };
  EXPECT_TRUE(same(a, b));
  // the smallest lever arm is not zero: the consider gain then leaves the attitude alone
  c.updatePositionLever(z, Eigen::Vector3d(1e-9, 0.0, 0.0), R);
  EXPECT_FALSE(same(a, c));
  EXPECT_NE(so3Log(a.state().q.conjugate() * c.state().q).norm(), 0.0);   // a turned the heading, c did not
}

// Closed form, and the sign of the Jacobian on the real filter. Level body heading along
// x, antenna 1 m ahead: the antenna's y is p_y + 1 m x heading. With variances a (p_y),
// b (heading), covariance c between them and fix variance r, the sideways innovation
// variance is a + 2c + b + r and the position gain (a + c) / (a + 2c + b + r). A Jacobian
// of the opposite sign would give (a - c) / (a - 2c + b + r).
TEST(EskfLever, PositionGainCountsTheAttitudeThroughTheLeverArm) {
  const double a = 0.01, b = 0.04, c = 0.004, r = 1e-4;
  NominalState x0;
  Eskf::Mat15 P0 = Eskf::Mat15::Identity() * 1e-9;
  P0(1, 1) = a;
  P0(8, 8) = b;
  P0(1, 8) = P0(8, 1) = c;
  P0(14, 14) = 1e-4;
  P0(8, 14) = P0(14, 8) = 1e-3;   // heading correlated with the vertical gyro bias
  Eskf f;
  f.initialize(x0, P0);
  const Eigen::Vector3d lever(1.0, 0.0, 0.0);
  f.updatePositionLever(Eigen::Vector3d(1.0, 0.3, 0.0), lever, Eigen::Matrix3d::Identity() * r);  // antenna seen 0.3 m to the left
  const double k = (a + c) / (a + 2 * c + b + r);
  EXPECT_NEAR(f.state().p.y(), 0.3 * k, 1e-7);
  EXPECT_GT(std::fabs(0.3 * k - 0.3 * (a - c) / (a - 2 * c + b + r)), 0.02);   // the two signs are far apart
  EXPECT_LT(0.3 * k, 0.1);                               // far from the whole 0.3 m: the heading could explain it
  EXPECT_TRUE(f.state().q.coeffs() == x0.q.coeffs());    // and the heading is not turned
  EXPECT_EQ(f.state().bg.z(), 0.0);
  EXPECT_NEAR(f.covariance()(8, 8), b, 1e-15);
  EXPECT_NEAR(f.covariance()(1, 1), (1 - k) * (1 - k) * a - 2 * (1 - k) * k * c + k * k * (b + r), 1e-9);  // Joseph, by hand
  // along the lever arm the attitude plays no part: the position takes a / (a + r) there
  Eskf g;
  Eskf::Mat15 Pg = P0;
  Pg(0, 0) = a;
  g.initialize(x0, Pg);
  g.updatePositionLever(Eigen::Vector3d(1.2, 0.0, 0.0), lever, Eigen::Matrix3d::Identity() * r);
  EXPECT_NEAR(g.state().p.x(), 0.2 * a / (a + r), 1e-7);
  // a plain Kalman update of the same state would have turned the heading by much
  const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x0.q, lever);
  const Eskf::Vec15 dx = P0 * H.transpose() * (H * P0 * H.transpose() + Eigen::Matrix3d::Identity() * r).inverse() *
                         Eigen::Vector3d(0.0, 0.3, 0.0);
  EXPECT_GT(std::fabs(dx[8]), 0.15);
}

namespace {
struct LeverRun {
  double xy, z, tilt, yaw, nees, sigma_xy;  // errors at the end; position NEES averaged over the run
};
// A robot standing (or turning on the spot at yaw_rate) for `seconds`, 100 Hz IMU with
// white noise (0.02 m/s^2, 0.002 rad/s), a 2 cm fix of the antenna at 5 Hz, and the
// covariance the node starts from. The first fix seeds the position as the node does.
LeverRun runLever(const Eigen::Vector3d& lever, double yaw_rate, double yaw0_err, unsigned seed,
                  double seconds = 60.0) {
  Rng rng(seed);
  EskfParams ep;
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) *= 0.5;
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  auto noise3 = [&rng](double s) { return Eigen::Vector3d(rng.gaussian(0, s), rng.gaussian(0, s), rng.gaussian(0, s)); };
  double yaw = 0.0;
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, yaw0_err));
  x0.p = lever + noise3(0.02) - x0.q * lever;   // truth base position is the origin
  f.initialize(x0, P0);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();  // the node floors z at 1 m^2
  const double dt = 0.01;
  const int n = static_cast<int>(seconds / dt);
  double nees = 0.0;
  int n_nees = 0;
  for (int k = 1; k <= n; ++k) {
    f.predict(Eigen::Vector3d(0, 0, 9.81) + noise3(0.02), Eigen::Vector3d(0, 0, yaw_rate) + noise3(0.002), dt);
    yaw += yaw_rate * dt;
    if (k % 20 == 0)
      f.updatePositionLever(so3Exp(Eigen::Vector3d(0, 0, yaw)) * lever + noise3(0.02), lever, R);
    if (k % 100 == 0 && k * dt > 10.0) {
      const Eigen::Vector3d e = f.state().p;
      nees += e.dot(f.covariance().block<3, 3>(0, 0).inverse() * e);
      ++n_nees;
    }
  }
  const Eigen::Vector3d att = so3Log(so3Exp(Eigen::Vector3d(0, 0, yaw)).conjugate() * f.state().q);
  return {f.state().p.head<2>().norm(), std::fabs(f.state().p.z()), att.head<2>().norm(), std::fabs(att.z()),
          nees / n_nees, std::sqrt(f.covariance()(0, 0) + f.covariance()(1, 1))};
}
}  // namespace

// GNSS and IMU alone for a minute, standing or turning on the spot, for the two antenna
// mounts the node check uses, with the heading 0.1 rad off at the start. The attitude
// stays where the IMU keeps it (the fix does not touch it), the height stays on the
// fixes, and the horizontal error is the lever arm times the heading error (which the
// fix cannot remove) and stays inside what the filter itself reports.
TEST(EskfLever, GnssOnlyStaysConsistentOverAMinute) {
  for (const Eigen::Vector3d& lever : {Eigen::Vector3d(0.5, 0.0, 1.0), Eigen::Vector3d(-0.3, 0.2, 1.0)}) {
    for (double yaw_rate : {0.0, 0.3}) {
      double nees_sum = 0.0;
      for (unsigned seed = 1; seed <= 8; ++seed) {
        const LeverRun r = runLever(lever, yaw_rate, 0.1, seed);
        EXPECT_LT(r.z, 0.1) << seed;
        EXPECT_LT(r.tilt, 0.02) << seed;
        EXPECT_NEAR(r.yaw, 0.1, 0.02) << seed;        // the 0.1 rad it started with, give or take the gyro
        EXPECT_LT(r.xy, lever.head<2>().norm() * 0.12 + 0.1) << seed;   // lever x heading error, plus noise
        EXPECT_LT(r.xy, 3.0 * r.sigma_xy) << seed;
        EXPECT_LT(r.sigma_xy, 1.2 * lever.head<2>().norm() * std::sqrt(0.5) + 0.05) << seed;
        nees_sum += r.nees;
      }
      EXPECT_LT(nees_sum / 8.0, 6.0) << yaw_rate;   // 3 degrees of freedom: 3 when consistent
      EXPECT_GT(nees_sum / 8.0, 0.2) << yaw_rate;
    }
  }
}
