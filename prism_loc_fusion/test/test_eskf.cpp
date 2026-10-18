#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
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

// The gain the update applies: the Kalman gain with the considered parts removed, written
// out independently of the library (projectors built from the vertical in the body frame).
Eigen::Matrix<double, 15, 3> considerGain(const NominalState& x, const Eskf::Mat15& P,
                                          const Eigen::Vector3d& lever, const Eigen::Matrix3d& R,
                                          unsigned consider) {
  const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x.q, lever);
  Eigen::Matrix<double, 15, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  const Eigen::Vector3d up = x.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d along = up * up.transpose(), across = Eigen::Matrix3d::Identity() - along;
  Eigen::Matrix3d keep_att = Eigen::Matrix3d::Identity(), keep_bg = Eigen::Matrix3d::Identity();
  if (consider & kConsiderHeading) keep_att -= along;
  if (consider & kConsiderTilt) keep_att -= across;
  if (consider & kConsiderGyroBiasVertical) keep_bg -= along;
  if (consider & kConsiderGyroBiasHorizontal) keep_bg -= across;
  K.block<3, 3>(6, 0) = keep_att * K.block<3, 3>(6, 0);
  K.block<3, 3>(12, 0) = keep_bg * K.block<3, 3>(12, 0);
  return K;
}

Eskf::Mat15 correlatedCov() {
  Eskf::Mat15 P0 = initCov(0.04);
  P0(1, 8) = P0(8, 1) = 0.01;
  P0(4, 7) = P0(7, 4) = 0.006;
  P0(8, 14) = P0(14, 8) = 0.005;
  P0(0, 6) = P0(6, 0) = 0.004;    // position correlated with a tilt
  P0(2, 13) = P0(13, 2) = 0.003;  // and with a horizontal gyro bias
  P0(6, 12) = P0(12, 6) = 0.002;
  return P0;
}

// One update ends in the state boxplus predicts from the correction K y, with the Joseph
// covariance for that gain: for the default parameters, and for every combination of
// consider states that LeverConsider offers.
TEST(EskfLever, UpdateIsTheConsiderGainWithJosephCovariance) {
  const NominalState x0 = tiltedState();
  const Eskf::Mat15 P0 = correlatedCov();
  const Eigen::Vector3d lever(0.5, -0.2, 1.0), z(3.3, -1.7, 1.2);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  struct Case { bool use_default; unsigned mask; };
  std::vector<Case> cases = {{true, kConsiderHeading | kConsiderGyroBiasVertical}};
  for (unsigned m = 0; m < 16; ++m) cases.push_back({false, m});
  for (const Case& c : cases) {
    EskfParams ep;
    if (!c.use_default) ep.lever_consider = c.mask;
    Eskf f(ep);
    f.initialize(x0, P0);
    f.updatePositionLever(z, lever, R);
    const Eigen::Matrix<double, 3, 15> H = Eskf::leverArmJacobian(x0.q, lever);
    const Eigen::Matrix<double, 15, 3> K = considerGain(x0, P0, lever, R, c.mask);
    const Eskf::Vec15 dx = K * (z - Eskf::leverArmPosition(x0, lever));
    const NominalState expect = boxplus(x0, dx);
    EXPECT_LT((f.state().p - expect.p).norm(), 1e-12) << c.mask;
    EXPECT_LT((f.state().v - expect.v).norm(), 1e-12) << c.mask;
    EXPECT_LT((f.state().ba - expect.ba).norm(), 1e-12) << c.mask;
    EXPECT_LT((f.state().bg - expect.bg).norm(), 1e-12) << c.mask;
    EXPECT_LT((f.state().q.coeffs() - expect.q.coeffs()).norm(), 1e-12) << c.mask;
    EXPECT_GT((dx.segment<3>(0).norm()), 0.1);     // position, velocity and accelerometer bias moved
    EXPECT_GT((dx.segment<3>(3).norm()), 1e-3);
    const Eskf::Mat15 A = Eskf::Mat15::Identity() - K * H;
    const Eskf::Mat15 P1 = A * P0 * A.transpose() + K * R * K.transpose();
    EXPECT_LT((f.covariance() - P1).norm(), 1e-12) << c.mask;
    EXPECT_LT((f.covariance() - f.covariance().transpose()).norm(), 1e-15) << c.mask;
    // the simple form (I - K H) P is not valid for a gain that is not the optimal one
    if (c.mask != 0u) { EXPECT_GT((P1 - A * P0).norm(), 1e-4) << c.mask; }
  }
}

// Under the default the heading and the vertical gyro bias are not corrected (exactly: the
// parts of the correction along the vertical are zero) and roll, pitch and the horizontal
// gyro biases are, on a body tilted so that the vertical is not a coordinate axis.
// With all attitude and gyro-bias states considered (the first version) none of them moves.
TEST(EskfLever, DefaultCorrectsTiltAndHorizontalGyroBiasButNotHeading) {
  const NominalState x0 = tiltedState();
  const Eskf::Mat15 P0 = correlatedCov();
  const Eigen::Vector3d lever(0.5, -0.2, 1.0), z(3.3, -1.7, 1.2);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  const Eigen::Vector3d up = x0.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  Eskf f;
  f.initialize(x0, P0);
  f.updatePositionLever(z, lever, R);
  const Eigen::Vector3d dtheta = so3Log(x0.q.conjugate() * f.state().q);
  const Eigen::Vector3d dbg = f.state().bg - x0.bg;
  EXPECT_LT(std::fabs(dtheta.dot(up)), 1e-14);                  // heading: none
  EXPECT_LT(std::fabs(dbg.dot(up)), 1e-14);                     // vertical gyro bias: none
  EXPECT_GT((dtheta - up * dtheta.dot(up)).norm(), 1e-3);       // roll and pitch: corrected
  EXPECT_GT((dbg - up * dbg.dot(up)).norm(), 1e-4);             // horizontal gyro bias: corrected
  EskfParams first;
  first.lever_consider = kConsiderAttitudeAndGyroBias;
  Eskf g(first);
  g.initialize(x0, P0);
  g.updatePositionLever(z, lever, R);
  EXPECT_TRUE(g.state().q.coeffs() == x0.q.coeffs());           // all considered: untouched, bit for bit
  EXPECT_TRUE(g.state().bg == x0.bg);
  for (int i : {6, 7, 8, 12, 13, 14}) EXPECT_NEAR(g.covariance()(i, i), P0(i, i), 1e-15) << i;
}

// The iterated update relinearises at each pass: with a heading 0.6 rad off, a lever of 1 m
// and a tight fix, three passes land the antenna closer to the fix than one pass, and
// lever_iterations 1 (the default) is the ordinary update bit for bit.
TEST(EskfLever, IteratedUpdateRelinearises) {
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, 0.6));   // the truth has heading 0
  Eskf::Mat15 P0 = initCov(1e-6);
  P0(8, 8) = 0.5;                              // the heading is the only uncertain state
  const Eigen::Vector3d lever(1.0, 0.0, 0.0);
  const Eigen::Vector3d fix = lever;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-6;
  EskfParams p1;
  p1.lever_consider = 0u;
  EskfParams p3 = p1;
  p3.lever_iterations = 3;
  EskfParams p0 = p1;   // iterations left at the default
  Eskf one(p1), three(p3), dflt(p0);
  for (Eskf* f : {&one, &three, &dflt}) f->initialize(x0, P0);
  for (Eskf* f : {&one, &three, &dflt}) f->updatePositionLever(fix, lever, R);
  const double r1 = (fix - Eskf::leverArmPosition(one.state(), lever)).norm();
  const double r3 = (fix - Eskf::leverArmPosition(three.state(), lever)).norm();
  EXPECT_GT(r1, 0.02);                         // one step of a 0.6 rad turn leaves a clear miss
  EXPECT_LT(r3, r1 / 10.0);
  EXPECT_TRUE(one.state().q.coeffs() == dflt.state().q.coeffs());
  EXPECT_TRUE(one.covariance() == dflt.covariance());
  EXPECT_LT(std::fabs(so3Log(x0.q.conjugate() * three.state().q).z() + 0.6), 0.02);   // the heading went to 0
}

TEST(EskfLever, AttitudeCovarianceAndTiltVariance) {
  // level: roll and pitch var_tilt, heading var_heading
  const Eigen::Matrix3d P = Eskf::attitudeCovariance(Eigen::Quaterniond::Identity(), 1e-4, 0.5);
  EXPECT_LT((P - Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal().toDenseMatrix()).norm(), 1e-15);
  // tilted: the heading variance sits on the vertical, as seen from the body
  const Eigen::Quaterniond q = tiltedState().q;
  const Eigen::Vector3d up = q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d Pt = Eskf::attitudeCovariance(q, 1e-4, 0.5);
  EXPECT_NEAR(up.dot(Pt * up), 0.5, 1e-14);
  EXPECT_NEAR(Pt.trace(), 0.5 + 2e-4, 1e-14);
  const Eigen::Vector3d across = up.cross(Eigen::Vector3d(0.3, -0.2, 0.9)).normalized();
  EXPECT_NEAR(across.dot(Pt * across), 1e-4, 1e-14);
  EXPECT_LT((Pt - Pt.transpose()).norm(), 1e-16);
  // tilt variance from one accelerometer reading: bias prior and noise over g^2 at rest ...
  EskfParams ep;
  ep.sigma_acc = 0.02;
  const double g = 9.81;
  const double rest = Eskf::tiltVarianceFromAccel(Eigen::Vector3d(0.0, 0.0, g), 1e-2, ep);
  EXPECT_NEAR(rest, (1e-2 + 4e-4) / (g * g), 1e-15);
  EXPECT_NEAR(rest, 1.08e-4, 2e-6);
  // ... and the departure of the reading from g counts as unexplained acceleration, up or down
  EXPECT_NEAR(Eskf::tiltVarianceFromAccel(Eigen::Vector3d(0.0, 0.0, 1.1 * g), 1e-2, ep) - rest, 0.01, 1e-12);
  EXPECT_NEAR(Eskf::tiltVarianceFromAccel(Eigen::Vector3d(0.0, 0.0, 0.9 * g), 1e-2, ep) - rest, 0.01, 1e-12);
  EXPECT_NEAR(Eskf::tiltVarianceFromAccel(Eigen::Vector3d(0.6 * g, 0.0, 0.8 * g), 1e-2, ep), rest, 1e-15);   // |f| = g, any direction
}

TEST(EskfLever, ReplaceAttitudeCovarianceDropsTheCorrelations) {
  Eskf f;
  const Eskf::Mat15 P0 = correlatedCov();
  f.initialize(tiltedState(), P0);
  const Eigen::Matrix3d Pa = Eigen::Vector3d(1e-4, 2e-4, 0.5).asDiagonal();
  f.replaceAttitudeCovariance(Pa);
  const Eskf::Mat15& P = f.covariance();
  EXPECT_TRUE((P.block<3, 3>(6, 6) == Pa));
  EXPECT_EQ((P.block<3, 6>(6, 0).norm() + P.block<3, 6>(6, 9).norm()), 0.0);   // the rest of the rows is zero
  EXPECT_EQ((P.block<6, 3>(0, 6).norm() + P.block<6, 3>(9, 6).norm()), 0.0);
  EXPECT_TRUE((P.block<6, 6>(0, 0) == P0.block<6, 6>(0, 0)));    // position and velocity untouched
  EXPECT_TRUE((P.block<6, 6>(9, 9) == P0.block<6, 6>(9, 9)));    // so are the biases
  EXPECT_TRUE((P.block<6, 6>(0, 9) == P0.block<6, 6>(0, 9)));
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
  // the smallest lever arm is not zero: the consider gain then leaves the heading alone
  c.updatePositionLever(z, Eigen::Vector3d(1e-9, 0.0, 0.0), R);
  EXPECT_FALSE(same(a, c));
  const Eigen::Vector3d up = tiltedState().q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  EXPECT_GT(std::fabs(so3Log(tiltedState().q.conjugate() * a.state().q).dot(up)), 1e-4);   // a turned the heading
  EXPECT_LT(std::fabs(so3Log(tiltedState().q.conjugate() * c.state().q).dot(up)), 1e-14);  // c did not
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
// white noise (0.02 m/s^2, 0.002 rad/s) and a constant gyro bias, a 2 cm fix of the
// antenna at 5 Hz, and the covariance the node starts from with an antenna off the origin:
// roll and pitch variance 1e-4 rad^2 (the accelerometer's accuracy), heading 0.5 rad^2.
// The first fix seeds the position as the node does. yaw is the signed heading error.
LeverRun runLever(const Eigen::Vector3d& lever, double yaw_rate, double yaw0_err, unsigned seed,
                  const Eigen::Vector3d& gyro_bias = Eigen::Vector3d::Zero(), double seconds = 60.0) {
  Rng rng(seed);
  EskfParams ep;
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
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
    f.predict(Eigen::Vector3d(0, 0, 9.81) + noise3(0.02), Eigen::Vector3d(0, 0, yaw_rate) + gyro_bias + noise3(0.002), dt);
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
  return {f.state().p.head<2>().norm(), std::fabs(f.state().p.z()), att.head<2>().norm(), att.z(),
          nees / n_nees, std::sqrt(f.covariance()(0, 0) + f.covariance()(1, 1))};
}
}  // namespace

// GNSS and IMU alone for a minute, standing or turning on the spot, for the two antenna
// mounts the node check uses, with the heading 0.1 rad off at the start, without a gyro
// bias and with two (the 0.003, -0.002, 0.002 rad/s of the review, and 0.5 deg/s on each
// axis). Thresholds fixed before the first run of this test:
//   height within 0.1 m and tilt within 0.03 rad (the fix corrects both, whatever the bias);
//   heading error = 0.1 rad + vertical gyro bias x 60 s, within 0.05 rad (the heading is a
//     consider state while the robot stands or turns on the spot: the fix does not touch it,
//     so it drifts with the bias);
//   horizontal error within 3 sigma of what the filter reports, and within 0.6 m;
//   position NEES, mean of 8 seeds, between 0.2 and 6 (3 degrees of freedom: 3 when consistent),
//     for the first two biases. With 0.5 deg/s the heading error reaches 0.6 rad, the arc the
//     antenna is on is no longer a straight line for the filter, and the NEES of the position
//     is 10 to 18 (measured, not asserted; experiments/eskf_lever.cpp has the grid).
// With the gyro bias the first version (attitude and gyro bias all considered) fails the
// height and tilt checks within 30 s: its tilt grows with the horizontal bias.
TEST(EskfLever, GnssOnlyStaysConsistentOverAMinute) {
  const Eigen::Vector3d biases[] = {Eigen::Vector3d::Zero(), Eigen::Vector3d(0.003, -0.002, 0.002),
                                    Eigen::Vector3d(1.0, -1.0, 1.0) * (0.5 * M_PI / 180.0)};
  for (const Eigen::Vector3d& lever : {Eigen::Vector3d(0.5, 0.0, 1.0), Eigen::Vector3d(-0.3, 0.2, 1.0)}) {
    for (double yaw_rate : {0.0, 0.3}) {
      for (const Eigen::Vector3d& bias : biases) {
        double nees_sum = 0.0;
        for (unsigned seed = 1; seed <= 8; ++seed) {
          const LeverRun r = runLever(lever, yaw_rate, 0.1, seed, bias);
          const std::string at = "seed " + std::to_string(seed) + " bias z " + std::to_string(bias.z());
          EXPECT_LT(r.z, 0.1) << at;
          EXPECT_LT(r.tilt, 0.03) << at;
          EXPECT_NEAR(r.yaw, 0.1 + bias.z() * 60.0, 0.05) << at;
          EXPECT_LT(r.xy, 0.6) << at;
          EXPECT_LT(r.xy, 3.0 * r.sigma_xy) << at;
          EXPECT_LT(r.sigma_xy, 1.2 * lever.head<2>().norm() * std::sqrt(0.5) + 0.05) << at;
          nees_sum += r.nees;
        }
        if (bias.z() < 0.005) {   // see above for the 0.5 deg/s case
          EXPECT_LT(nees_sum / 8.0, 6.0) << yaw_rate << " bias z " << bias.z();   // 3 degrees of freedom: 3 when consistent
          EXPECT_GT(nees_sum / 8.0, 0.2) << yaw_rate << " bias z " << bias.z();
        }
      }
    }
  }
}

// A lever of exactly zero is the plain position update whatever the consider states or the
// number of iterations are set to: the settings only act on an antenna off the origin.
TEST(EskfLever, ZeroLeverIgnoresTheSchemeSettings) {
  Eskf::Mat15 P0 = initCov(0.3);
  P0(0, 8) = P0(8, 0) = 0.05;
  P0(1, 14) = P0(14, 1) = 0.01;
  P0(2, 6) = P0(6, 2) = 0.02;
  const Eigen::Vector3d z(2.5, -2.2, 0.9);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  Eskf ref;
  ref.initialize(tiltedState(), P0);
  ref.updatePosition(z, R);
  for (unsigned mask = 0; mask < 16; ++mask) {
    for (int iterations : {1, 3}) {
      EskfParams ep;
      ep.lever_consider = mask;
      ep.lever_iterations = iterations;
      Eskf f(ep);
      f.initialize(tiltedState(), P0);
      f.updatePositionLever(z, Eigen::Vector3d::Zero(), R);
      EXPECT_TRUE(f.state().p == ref.state().p && f.state().v == ref.state().v &&
                  f.state().q.coeffs() == ref.state().q.coeffs() && f.state().ba == ref.state().ba &&
                  f.state().bg == ref.state().bg && f.covariance() == ref.covariance())
          << mask << " " << iterations;
    }
  }
}

// The motion gate: with a speed or rate over its threshold the update leaves out
// lever_consider_moving instead of lever_consider. Level body, so the heading is the
// attitude error about z; a position-heading covariance gives the fix a reason to turn it.
namespace {
double headingTurnedBy(EskfParams ep, double speed, double sigma_v, double rate, double dt = 0.01) {
  NominalState x0;
  x0.v = Eigen::Vector3d(speed, 0.0, 0.0);
  Eskf::Mat15 P0 = initCov(1e-4);
  P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
  P0(8, 8) = 0.3;
  P0(1, 8) = P0(8, 1) = 0.05;
  Eskf f(ep);
  f.initialize(x0, P0);
  if (rate > 0.0) {
    // predict() with an angular rate of `rate` about z; keep the state change small
    f.predict(Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d(0, 0, rate), dt);
  }
  const Eigen::Quaterniond q_before = f.state().q;
  const Eigen::Vector3d lever(1.0, 0.0, 0.5);
  f.updatePositionLever(f.state().p + f.state().q.toRotationMatrix() * lever + Eigen::Vector3d(0.0, 0.2, 0.0), lever,
                        Eigen::Matrix3d::Identity() * 1e-4);
  return std::fabs(so3Log(q_before.conjugate() * f.state().q).z());
}
}  // namespace

TEST(EskfLever, MotionGateLeavesTheHeadingOutOnlyWhileStanding) {
  EskfParams ep;
  ep.lever_consider = kConsiderHeading | kConsiderGyroBiasVertical;
  ep.lever_consider_moving = 0u;
  ep.lever_motion_speed = 0.3;
  ep.lever_motion_rate = 0.1;
  EXPECT_LT(headingTurnedBy(ep, 0.0, 0.01, 0.0), 1e-14);        // standing: heading not corrected
  EXPECT_GT(headingTurnedBy(ep, 0.5, 0.01, 0.0), 1e-3);         // moving: corrected
  EXPECT_LT(headingTurnedBy(ep, 0.29, 0.01, 0.0), 1e-14);       // just under the speed threshold
  EXPECT_LT(headingTurnedBy(ep, 0.3, 0.01, 0.0), 1e-14);        // exactly at it: not over
  EXPECT_GT(headingTurnedBy(ep, 0.31, 0.01, 0.0), 1e-3);        // just over it
  EXPECT_LT(headingTurnedBy(ep, 0.5, 0.2, 0.0), 1e-14);         // 0.5 m/s is not 3 sigma of a 0.2 m/s uncertainty
  EXPECT_GT(headingTurnedBy(ep, 0.7, 0.2, 0.0), 1e-3);          // 0.7 m/s is
  EXPECT_LT(headingTurnedBy(ep, 0.0, 0.01, 0.09), 1e-14);       // under the rate threshold
  EXPECT_LT(headingTurnedBy(ep, 0.0, 0.01, 0.1), 1e-14);        // exactly at it: not over
  EXPECT_GT(headingTurnedBy(ep, 0.0, 0.01, 0.11), 1e-3);        // over it
  EXPECT_GT(headingTurnedBy(ep, 0.0, 0.01, -0.11) + headingTurnedBy(ep, 0.0, 0.01, 0.11), 1e-3);
  EskfParams no_rate = ep;
  no_rate.lever_motion_rate = 0.0;                              // rate test off
  EXPECT_LT(headingTurnedBy(no_rate, 0.0, 0.01, 0.5), 1e-14);
  EXPECT_GT(headingTurnedBy(no_rate, 0.5, 0.01, 0.0), 1e-3);
  EskfParams no_speed = ep;
  no_speed.lever_motion_speed = 0.0;                            // speed test off
  EXPECT_LT(headingTurnedBy(no_speed, 5.0, 0.01, 0.0), 1e-14);
  EXPECT_GT(headingTurnedBy(no_speed, 0.0, 0.01, 0.5), 1e-3);
  EskfParams off = ep;
  off.lever_motion_speed = off.lever_motion_rate = 0.0;         // both off: the gate is off
  EXPECT_LT(headingTurnedBy(off, 5.0, 0.01, 0.5), 1e-14);
  EskfParams moving_considers = ep;
  moving_considers.lever_consider_moving = kConsiderHeading;     // the moving mask is the one used
  EXPECT_LT(headingTurnedBy(moving_considers, 0.5, 0.01, 0.0), 1e-14);
}

// --- the bound on the heading uncertainty and the release by acceleration
//     (lever_heading_sigma_max, lever_release_dv, lever_release_tau) ---

namespace {
// A fix that carries no information (R = 1e12): the update leaves x and P as the bound has left them.
void nullFix(Eskf& f, const Eigen::Vector3d& lever) {
  const Eigen::Vector3d fix = Eskf::leverArmPosition(f.state(), lever);
  f.updatePositionLever(fix, lever, Eigen::Matrix3d::Identity() * 1e12);
}
// Covariance with a heading variance var_h (about the vertical of the tilted body), correlated with
// the position and the vertical gyro bias.
Eskf::Mat15 headingCov(const Eigen::Quaterniond& q, double var_h) {
  Eskf::Mat15 P = initCov(1e-4);
  P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.04;
  P.block<3, 3>(6, 6) = Eskf::attitudeCovariance(q, 2e-4, var_h);
  const Eigen::Vector3d up = q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const double s = std::sqrt(var_h);
  P.block<3, 3>(0, 6) = 0.03 * s * Eigen::Vector3d(1.0, 0.5, -0.2) * up.transpose();      // position with heading
  P.block<3, 3>(6, 0) = P.block<3, 3>(0, 6).transpose();
  P.block<3, 3>(12, 6) = 2e-4 * s * up * up.transpose();                                   // vertical gyro bias with heading
  P.block<3, 3>(6, 12) = P.block<3, 3>(12, 6).transpose();
  return P;
}
}  // namespace

// Above the bound the heading error variable is scaled by c = bound / sigma: the heading variance
// by c^2, every cross-covariance of the heading by c, the tilt not at all, and P stays symmetric and
// positive definite. Checked on a tilted body (the vertical is not a coordinate axis) with an
// information-free fix, so that the update itself changes nothing.
TEST(EskfLever, HeadingBoundScalesTheHeadingAndItsCrossCovariances) {
  const NominalState x0 = tiltedState();
  const Eigen::Vector3d up = x0.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eskf::Mat15 P0 = headingCov(x0.q, 4.0);                     // sigma 2 rad
  EskfParams ep;
  ep.lever_heading_sigma_max = 0.7;
  Eskf f(ep);
  f.initialize(x0, P0);
  EXPECT_NEAR(f.headingSigma(), 2.0, 1e-12);
  nullFix(f, Eigen::Vector3d(0.5, -0.2, 1.0));
  EXPECT_TRUE(f.leverHeadingCapped());
  const double c = 0.7 / 2.0;
  Eskf::Mat15 D = Eskf::Mat15::Identity();
  D.block<3, 3>(6, 6) += (c - 1.0) * up * up.transpose();
  const Eskf::Mat15 expect = D * P0 * D.transpose();
  EXPECT_LT((f.covariance() - expect).norm(), 1e-9);                // the scaling, written out independently
  EXPECT_NEAR(f.headingSigma(), 0.7, 1e-9);
  EXPECT_GT((P0.block<3, 3>(0, 6).norm()), 0.01);                     // the cross-covariances are not zero to begin with
  EXPECT_LT((f.covariance().block<3, 3>(0, 6) - c * P0.block<3, 3>(0, 6)).norm(), 1e-9);     // position-heading
  EXPECT_LT((f.covariance().block<3, 3>(12, 6) - c * P0.block<3, 3>(12, 6)).norm(), 1e-9);   // gyro bias-heading
  const Eigen::Vector3d across = up.cross(Eigen::Vector3d(0.3, -0.2, 0.9)).normalized();
  EXPECT_NEAR(across.dot(f.covariance().block<3, 3>(6, 6) * across), across.dot(P0.block<3, 3>(6, 6) * across), 1e-9);   // tilt
  EXPECT_LT((f.covariance() - f.covariance().transpose()).norm(), 1e-12);
  Eigen::LLT<Eskf::Mat15> llt(f.covariance());
  EXPECT_EQ(llt.info(), Eigen::Success);
  EXPECT_TRUE(f.state().q.coeffs() == x0.q.coeffs());               // the bound does not touch the estimate
}

// Exactly at the bound, and just either side of it. The body is level, so the vertical is a
// coordinate axis and the variance read back is the one that was set.
TEST(EskfLever, HeadingBoundActsOnlyAboveIt) {
  const double bound = 0.7;
  const double at = bound * bound;
  struct Case { double var; bool capped; };
  for (const Case& c : {Case{std::nextafter(at, 0.0), false}, Case{at, false}, Case{std::nextafter(at, 1.0), true},
                        Case{0.5, true}, Case{0.3, false}, Case{1.0, true}}) {
    EskfParams ep;
    ep.lever_heading_sigma_max = bound;
    Eskf f(ep);
    Eskf::Mat15 P0 = initCov(1e-4);
    P0(8, 8) = c.var;
    f.initialize(NominalState(), P0);
    nullFix(f, Eigen::Vector3d(0.5, 0.0, 1.0));
    EXPECT_EQ(f.leverHeadingCapped(), c.capped) << c.var;
    if (c.capped) EXPECT_NEAR(f.covariance()(8, 8), at, 1e-9) << c.var;
    else EXPECT_NEAR(f.covariance()(8, 8), c.var, 1e-9) << c.var;
  }
  EskfParams off;                       // 0: no bound
  off.lever_heading_sigma_max = 0.0;
  Eskf g(off);
  Eskf::Mat15 P1 = initCov(1e-4);
  P1(8, 8) = 9.0;
  g.initialize(NominalState(), P1);
  nullFix(g, Eigen::Vector3d(0.5, 0.0, 1.0));
  EXPECT_FALSE(g.leverHeadingCapped());
  EXPECT_NEAR(g.covariance()(8, 8), 9.0, 1e-9);
  // the flag follows the last update: false before any, true after a capped one, false after one at the bound
  EskfParams ep;
  ep.lever_heading_sigma_max = bound;
  Eskf h(ep);
  Eskf::Mat15 P2 = initCov(1e-4);
  P2(8, 8) = 2.0;
  h.initialize(NominalState(), P2);
  EXPECT_FALSE(h.leverHeadingCapped());
  nullFix(h, Eigen::Vector3d(0.5, 0.0, 1.0));
  EXPECT_TRUE(h.leverHeadingCapped());
  nullFix(h, Eigen::Vector3d(0.5, 0.0, 1.0));
  EXPECT_FALSE(h.leverHeadingCapped());
  nullFix(h, Eigen::Vector3d::Zero());  // a zero lever is the plain update and leaves the flag as it was
  EXPECT_FALSE(h.leverHeadingCapped());
  Eskf::Mat15 P3 = initCov(1e-4);
  P3(8, 8) = 2.0;
  h.initialize(NominalState(), P3);
  nullFix(h, Eigen::Vector3d(0.5, 0.0, 1.0));
  nullFix(h, Eigen::Vector3d::Zero());
  EXPECT_TRUE(h.leverHeadingCapped());
}

namespace {
struct LongRun {
  double max_sigma_after_update{0.0}, sigma_end{0.0}, max_tilt{0.0}, max_z{0.0}, max_xy_beyond{-1e9};
  bool capped_at_end{false};
};
// A robot standing for `seconds` on GNSS and IMU alone (as runLever above), a 5 Hz fix of the antenna at
// `lever`, the filter started 0.1 rad off in heading with the node's covariance. Records after each lever
// update the heading sigma, and from 10 s on the largest tilt and height error and the horizontal error
// beyond the circle of radius |lever_xy| around the antenna (2 |lever_xy| sin(heading error / 2)).
LongRun runStanding(EskfParams ep, const Eigen::Vector3d& lever, const Eigen::Vector3d& gyro_bias, double seconds, unsigned seed) {
  Rng rng(seed);
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  auto noise3 = [&rng](double s) { return Eigen::Vector3d(rng.gaussian(0, s), rng.gaussian(0, s), rng.gaussian(0, s)); };
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, 0.1));
  x0.p = lever + noise3(0.02) - x0.q * lever;     // the truth base position is the origin, heading 0
  f.initialize(x0, P0);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  const double dt = 0.01;
  const int n = static_cast<int>(seconds / dt);
  LongRun out;
  for (int k = 1; k <= n; ++k) {
    f.predict(Eigen::Vector3d(0, 0, 9.81) + noise3(0.02), gyro_bias + noise3(0.002), dt);
    if (k % 20 == 0) {
      f.updatePositionLever(lever + noise3(0.02), lever, R);
      out.max_sigma_after_update = std::max(out.max_sigma_after_update, f.headingSigma());
    }
    if (k % 100 == 0 && k * dt >= 10.0) {
      const Eigen::Vector3d att = so3Log(f.state().q);
      out.max_tilt = std::max(out.max_tilt, att.head<2>().norm());
      out.max_z = std::max(out.max_z, std::fabs(f.state().p.z()));
      out.max_xy_beyond = std::max(out.max_xy_beyond, f.state().p.head<2>().norm() -
                                   (2.0 * lever.head<2>().norm() * std::sin(0.5 * std::min(std::fabs(att.z()), M_PI)) + 0.3));
    }
  }
  out.sigma_end = f.headingSigma();
  out.capped_at_end = f.leverHeadingCapped();
  return out;
}
}  // namespace

// The failure of the review (round 3): a robot that stands for 15 minutes on GNSS and IMU alone. Without
// the bound the heading variance grows with (vertical gyro bias sigma x t)^2, the lever term leaves its linear
// range and the tilt and height diverge; with it the heading sigma never exceeds the bound after an update,
// and the tilt, the height and the position stay within what a standing robot can have (two antenna mounts,
// vertical gyro bias 0 and 0.1 deg/s, seed 1; thresholds fixed before the first run of this test). One seed and
// 900 s: the core tests are also built without optimization in CI, where this test takes about a minute.
TEST(EskfLever, HeadingSigmaStaysAtTheBoundForFifteenMinutes) {
  int unbounded_failed = 0, runs = 0;
  for (const Eigen::Vector3d& lever : {Eigen::Vector3d(0.5, 0.0, 1.0), Eigen::Vector3d(-0.3, 0.2, 1.0)}) {
    for (double bias_dps : {0.0, 0.1}) {
      const Eigen::Vector3d bias(0.0, 0.0, bias_dps * M_PI / 180.0);
      const std::string at = "lever x " + std::to_string(lever.x()) + " bias " + std::to_string(bias_dps);
      const LongRun r = runStanding(EskfParams(), lever, bias, 900.0, 1);
      // The bound is on the sigma along the vertical of the estimate the update was linearised at; the update then
      // turns the estimate by its tilt correction (under 0.01 rad here), so the reading afterwards is the bound to second order.
      const double bound = EskfParams().lever_heading_sigma_max;
      EXPECT_LE(r.max_sigma_after_update, bound * (1.0 + 2e-3)) << at;
      EXPECT_NEAR(r.max_sigma_after_update, bound, bound * 2e-3) << at;     // and it does reach it: the bound is what holds it
      EXPECT_TRUE(r.capped_at_end) << at;
      EXPECT_LT(r.max_tilt, 0.03) << at;
      EXPECT_LT(r.max_z, 0.15) << at;
      EXPECT_LT(r.max_xy_beyond, 0.3) << at;      // circle + 0.6 m (the tangential wander of a consider filter reaches circle + 0.38 m)
      // The same without the bound (and without the release, which would correct the heading when a tilt error leaks into the
      // integral), 600 s: the failure of the review.
      EskfParams no_bound;
      no_bound.lever_heading_sigma_max = 0.0;
      no_bound.lever_release_dv = 0.0;
      const LongRun u = runStanding(no_bound, lever, bias, 600.0, 1);
      unbounded_failed += !(u.max_tilt < 0.03 && u.max_z < 0.15);    // false also for a NaN
      ++runs;
    }
  }
  EXPECT_GE(unbounded_failed, 3) << "of " << runs << " runs without the bound: the standing robot leans or its height runs";
}

namespace {
// A level filter that stands for 1 s, then accelerates along x at `a` m/s^2 for `accel_s`, then stands
// for `after_s` (100 Hz; the specific force of a level body at rest is (0, 0, 9.81)).
Eskf accelerated(EskfParams ep, double a, double accel_s, double after_s) {
  Eskf f(ep);
  Eskf::Mat15 P0 = initCov(1e-4);
  P0(8, 8) = 0.3;
  P0(1, 8) = P0(8, 1) = 0.05;       // the position is correlated with the heading, which gives the fix a reason to turn it
  f.initialize(NominalState(), P0);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  for (int k = 0; k < 100; ++k) f.predict(Eigen::Vector3d(0.0, 0.0, 9.81), gyro, 0.01);
  for (int k = 0; k < std::lround(accel_s * 100); ++k) f.predict(Eigen::Vector3d(a, 0.0, 9.81), gyro, 0.01);
  for (int k = 0; k < std::lround(after_s * 100); ++k) f.predict(Eigen::Vector3d(0.0, 0.0, 9.81), gyro, 0.01);
  return f;
}
double headingTurnedByFix(Eskf& f) {
  const Eigen::Quaterniond q_before = f.state().q;
  const Eigen::Vector3d lever(1.0, 0.0, 0.5);
  f.updatePositionLever(f.state().p + f.state().q.toRotationMatrix() * lever + Eigen::Vector3d(0.0, 0.2, 0.0), lever,
                        Eigen::Matrix3d::Identity() * 1e-4);
  return std::fabs(so3Log(q_before.conjugate() * f.state().q).z());
}
}  // namespace

// The integral of the horizontal specific force: closed form for a constant acceleration, decay after it,
// the threshold tested at the accumulated value (both sides), and no effect from a step of zero or negative
// length, a non-finite reading or a time constant of zero.
TEST(EskfLever, ReleaseFollowsTheAccelerationIntegral) {
  const double tau = 4.0, dt = 0.01, a = 1.0;
  EskfParams ep;
  ep.lever_release_tau = tau;
  ep.lever_release_dv = 1.0;
  const Eskf rest = accelerated(ep, a, 0.0, 0.0);
  EXPECT_EQ(rest.releaseAccumulator(), 0.0);              // standing: the specific force cancels gravity
  EXPECT_FALSE(rest.leverReleaseOpen());
  const Eskf f = accelerated(ep, a, 2.0, 0.0);
  const double r = std::exp(-dt / tau);
  double acc = 0.0;
  for (int k = 0; k < 200; ++k) acc = acc * r + a * dt;
  EXPECT_NEAR(f.releaseAccumulator(), acc, 1e-9);
  EXPECT_NEAR(acc, a * dt * (1.0 - std::pow(r, 200)) / (1.0 - r), 1e-9);         // the closed form
  EXPECT_NEAR(acc, a * tau * (1.0 - std::exp(-2.0 / tau)), 0.01 * acc);          // near its continuous limit, 1.57 m/s
  EXPECT_TRUE(f.leverReleaseOpen());                                              // over the threshold of 1.0
  // the threshold, either side of the accumulated value
  EskfParams just_below = ep, just_above = ep;
  just_below.lever_release_dv = acc - 1e-6;
  just_above.lever_release_dv = acc + 1e-6;
  EXPECT_TRUE(accelerated(just_below, a, 2.0, 0.0).leverReleaseOpen());
  EXPECT_FALSE(accelerated(just_above, a, 2.0, 0.0).leverReleaseOpen());
  EskfParams exactly = ep;
  exactly.lever_release_dv = f.releaseAccumulator();                              // exactly at the threshold: not over it
  EXPECT_FALSE(accelerated(exactly, a, 2.0, 0.0).leverReleaseOpen());
  EskfParams off = ep;
  off.lever_release_dv = 0.0;                                                     // 0 = off, whatever the accumulator says
  EXPECT_FALSE(accelerated(off, a, 2.0, 0.0).leverReleaseOpen());
  // decay: 1 s after the acceleration it is 1.57 x exp(-1 / 4) = 1.22 (open), 3 s after 0.74 (closed)
  EXPECT_NEAR(accelerated(ep, a, 2.0, 1.0).releaseAccumulator(), acc * std::exp(-1.0 / tau), 1e-9);
  EXPECT_TRUE(accelerated(ep, a, 2.0, 1.0).leverReleaseOpen());
  EXPECT_FALSE(accelerated(ep, a, 2.0, 3.0).leverReleaseOpen());
  // braking counts like accelerating (the magnitude of the horizontal part), and so does a sideways one
  EXPECT_NEAR(accelerated(ep, -a, 2.0, 0.0).releaseAccumulator(), acc, 1e-9);
  // a step of zero or negative length, and a time constant of zero, add nothing
  Eskf g = accelerated(ep, a, 2.0, 0.0);
  const double before = g.releaseAccumulator();
  g.predict(Eigen::Vector3d(a, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.0);
  g.predict(Eigen::Vector3d(a, 0.0, 9.81), Eigen::Vector3d::Zero(), -0.01);
  EXPECT_EQ(g.releaseAccumulator(), before);
  EskfParams no_tau = ep;
  no_tau.lever_release_tau = 0.0;
  EXPECT_EQ(accelerated(no_tau, a, 2.0, 0.0).releaseAccumulator(), 0.0);
  EXPECT_FALSE(accelerated(no_tau, a, 2.0, 0.0).leverReleaseOpen());
  // a non-finite reading does not make the accumulator non-finite (the state is gone anyway)
  Eskf h = accelerated(ep, a, 2.0, 0.0);
  h.predict(Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 9.81), Eigen::Vector3d::Zero(), 0.01);
  EXPECT_TRUE(std::isfinite(h.releaseAccumulator()));
}

// While the release is open the heading is corrected by the fix (lever_consider_moving), and while it is
// closed it is not, whatever the filter's speed is: a robot that has driven at a constant speed (speed
// 2 m/s, no acceleration) is closed, one that has just accelerated is open.
TEST(EskfLever, ReleaseLetsTheFixTurnTheHeadingOnlyWhileTheImuSeesAcceleration) {
  EskfParams ep;     // the defaults: bound 0.35, release at 1.0 m/s, no speed gate
  Eskf stands = accelerated(ep, 1.0, 0.0, 0.0), brief = accelerated(ep, 1.0, 0.5, 0.0), long_acc = accelerated(ep, 1.0, 2.0, 0.0);
  Eskf after = accelerated(ep, 1.0, 2.0, 3.0);
  EXPECT_LT(headingTurnedByFix(stands), 1e-14);
  EXPECT_LT(headingTurnedByFix(brief), 1e-14);          // 0.5 m/s of integral: under the threshold
  EXPECT_GT(headingTurnedByFix(long_acc), 1e-3);        // 1.57: over it
  Eskf after1 = accelerated(ep, 1.0, 2.0, 1.0);
  EXPECT_GT(headingTurnedByFix(after1), 1e-3);          // 1 s after: 1.22, still over it (the decay is exp(-t / 4 s))
  EXPECT_LT(headingTurnedByFix(after), 1e-14);          // 3 s after: decayed under it, though the filter's speed is 2 m/s
  EXPECT_GT(after.state().v.norm(), 1.0);
  EskfParams released_considers = ep;
  released_considers.lever_consider_moving = kConsiderHeading;     // the released mask is the one used
  Eskf c = accelerated(released_considers, 1.0, 2.0, 0.0);
  EXPECT_LT(headingTurnedByFix(c), 1e-14);
}

// The bound applies while the release is open too (the version before the honest release, lever_restore off): the full
// update still sees a heading sigma of at most the bound. With lever_restore (the default) the bound is not applied
// while the release is open: the full update sees the heading variance the filter holds (see
// HonestReleaseIsNotBounded).
TEST(EskfLever, BoundAlsoAppliesWhileReleased) {
  EskfParams ep;
  ep.lever_restore = false;
  Eskf f = accelerated(ep, 1.0, 2.0, 0.0);
  ASSERT_TRUE(f.leverReleaseOpen());
  Eskf::Mat15 P = f.covariance();
  P(8, 8) = 4.0;
  f.initialize(f.state(), P);
  nullFix(f, Eigen::Vector3d(0.5, 0.0, 1.0));
  EXPECT_TRUE(f.leverHeadingCapped());
  EXPECT_NEAR(f.headingSigma(), ep.lever_heading_sigma_max, 1e-9);
}

// The accumulator reads the specific force and nothing else: with it on (a time constant) or off (zero) the
// state and covariance are bit for bit the same after the same predictions and the same lever update, as long as
// the released mask equals the considered one.
TEST(EskfLever, AccumulatorDoesNotTouchStateOrCovariance) {
  EskfParams on, off;
  on.lever_release_tau = 4.0;
  off.lever_release_tau = 0.0;
  on.lever_consider_moving = off.lever_consider_moving = on.lever_consider;
  on.lever_restore = off.lever_restore = false;   // with it, an open release also lifts the bound: a difference in P
  Eskf a = accelerated(on, 1.0, 2.0, 0.5), b = accelerated(off, 1.0, 2.0, 0.5);
  EXPECT_GT(a.releaseAccumulator(), 1.0);
  EXPECT_EQ(b.releaseAccumulator(), 0.0);
  auto same = [](const Eskf& x, const Eskf& y) {
    return x.state().p == y.state().p && x.state().v == y.state().v && x.state().q.coeffs() == y.state().q.coeffs() &&
           x.state().ba == y.state().ba && x.state().bg == y.state().bg && x.covariance() == y.covariance();
  };
  EXPECT_TRUE(same(a, b));
  const Eigen::Vector3d lever(1.0, 0.0, 0.5);
  for (Eskf* f : {&a, &b}) f->updatePositionLever(f->state().p + lever + Eigen::Vector3d(0.0, 0.2, 0.0), lever, Eigen::Matrix3d::Identity() * 1e-4);
  EXPECT_TRUE(same(a, b));
}

// A lever of exactly zero is the plain position update whatever the bound and the release are set to, with a
// heading variance far over the bound and the release open.
TEST(EskfLever, ZeroLeverIgnoresTheBoundAndTheRelease) {
  const Eigen::Vector3d z(2.5, -2.2, 0.9);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  for (double bound : {0.0, 0.1, 0.7}) {
    for (double dv : {0.0, 1e-9, 1.0}) {
      EskfParams ep;
      ep.lever_heading_sigma_max = bound;
      ep.lever_release_dv = dv;
      Eskf::Mat15 P0 = headingCov(tiltedState().q, 9.0);
      Eskf ref, f(ep);
      ref.initialize(tiltedState(), P0);
      f.initialize(tiltedState(), P0);
      for (int k = 0; k < 200; ++k) {
        ref.predict(Eigen::Vector3d(1.0, 0.2, 9.81), Eigen::Vector3d(0.0, 0.0, 0.1), 0.01);
        f.predict(Eigen::Vector3d(1.0, 0.2, 9.81), Eigen::Vector3d(0.0, 0.0, 0.1), 0.01);
      }
      ref.updatePosition(z, R);
      f.updatePositionLever(z, Eigen::Vector3d::Zero(), R);
      EXPECT_TRUE(f.state().p == ref.state().p && f.state().v == ref.state().v && f.state().q.coeffs() == ref.state().q.coeffs() &&
                  f.state().ba == ref.state().ba && f.state().bg == ref.state().bg && f.covariance() == ref.covariance())
          << bound << " " << dv;
      EXPECT_FALSE(f.leverHeadingCapped());
    }
  }
}

// replaceAttitude(): the nominal attitude and its covariance are replaced, the correlations of the attitude
// with every other state dropped, position and velocity left where they were.
TEST(EskfLever, ReplaceAttitudeSetsTheAttitudeAndItsCovariance) {
  Eskf f;
  const Eskf::Mat15 P0 = correlatedCov();
  f.initialize(tiltedState(), P0);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 0.1, 1.0).normalized()));
  const Eigen::Matrix3d Pa = Eigen::Vector3d(1e-4, 2e-4, 0.5).asDiagonal();
  f.replaceAttitude(q, Pa);
  EXPECT_LT((f.state().q.coeffs() - q.coeffs()).norm(), 1e-15);
  EXPECT_TRUE((f.covariance().block<3, 3>(6, 6) == Pa));
  EXPECT_EQ((f.covariance().block<3, 6>(6, 0).norm() + f.covariance().block<3, 6>(6, 9).norm()), 0.0);
  EXPECT_TRUE(f.state().p == tiltedState().p);
  EXPECT_TRUE(f.state().v == tiltedState().v);
  EXPECT_TRUE((f.covariance().block<6, 6>(0, 0) == P0.block<6, 6>(0, 0)));
  // a quaternion that is not of unit length is normalized
  f.replaceAttitude(Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0), Pa);
  EXPECT_NEAR(f.state().q.norm(), 1.0, 1e-15);
}

// headingSigma() is sqrt(u^T P_theta u) with u = R^T z, on a tilted body.
TEST(EskfLever, HeadingSigmaIsTheSigmaAboutTheVertical) {
  const NominalState x0 = tiltedState();
  Eskf f;
  f.initialize(x0, headingCov(x0.q, 0.36));
  EXPECT_NEAR(f.headingSigma(), 0.6, 1e-12);
  Eskf g;
  g.initialize(x0, initCov(0.04));
  EXPECT_NEAR(g.headingSigma(), 0.2, 1e-12);
  // a covariance that is not usable reads as such: NaN stays NaN (not 0, which would read as certainty), a negative variance is 0
  Eskf::Mat15 bad = initCov(0.04);
  bad.block<3, 3>(6, 6).setConstant(std::numeric_limits<double>::quiet_NaN());
  Eskf h;
  h.initialize(NominalState(), bad);
  EXPECT_TRUE(std::isnan(h.headingSigma()));
  Eskf::Mat15 neg = initCov(0.04);
  neg(8, 8) = -1.0;
  Eskf k;
  k.initialize(NominalState(), neg);
  EXPECT_EQ(k.headingSigma(), 0.0);
}

// The defaults are the scheme the grid chose (experiments/eskf_lever.cpp, second pass; README): a bound of
// 0.35 rad on the heading sigma, release at 1.0 m/s of horizontal acceleration integral over a 4 s window,
// no speed gate, the heading and the vertical gyro bias considered. Pinned by value and by what they do: with
// 1 m/s^2 along x from rest the integral passes 1.0 m/s after 4 s x ln(4 / 3) = 1.15 s.
TEST(EskfLever, DefaultsAreTheChosenScheme) {
  const EskfParams d;
  EXPECT_EQ(d.lever_heading_sigma_max, 0.35);
  EXPECT_EQ(d.lever_release_dv, 1.0);
  EXPECT_EQ(d.lever_release_tau, 4.0);
  EXPECT_EQ(d.lever_motion_speed, 0.0);
  EXPECT_EQ(d.lever_motion_rate, 0.0);
  EXPECT_EQ(d.lever_consider, kConsiderHeading | kConsiderGyroBiasVertical);
  EXPECT_EQ(d.lever_consider_moving, 0u);
  EXPECT_EQ(d.lever_iterations, 1);
  EXPECT_TRUE(d.lever_align_span_guard);                                  // fifth review: the alignment refuses a window with a gap
  EXPECT_NEAR(d.lever_start_sigma, M_PI / std::sqrt(3.0), 1e-12);                 // a start with no heading source: the spread of an unknown heading
  Eskf f(d);
  f.initialize(NominalState(), initCov(0.01));
  int steps = 0;
  while (!f.leverReleaseOpen() && steps < 1000) {
    f.predict(Eigen::Vector3d(1.0, 0.0, 9.81), Eigen::Vector3d::Zero(), 0.01);
    ++steps;
  }
  EXPECT_NEAR(steps * 0.01, 4.0 * std::log(4.0 / 3.0), 0.02);
  EXPECT_GT(f.releaseAccumulator(), 1.0);
  EXPECT_LT(f.releaseAccumulator(), 1.0 + 0.01);                          // it opened at the first step over 1.0 m/s
}

// The integral counts the horizontal part of the specific force in the world frame, whatever the direction of the
// acceleration in the body: sideways like forward, a diagonal by its length, a vertical bump not at all, and a robot at
// rest on a slope (the body-frame reading has a horizontal part of g sin(tilt), the world frame none) not at all.
TEST(EskfLever, ReleaseCountsTheHorizontalAccelerationInTheWorldFrame) {
  auto run = [](const NominalState& x0, const Eigen::Vector3d& f, int steps) {
    Eskf e;
    e.initialize(x0, initCov(0.01));
    for (int k = 0; k < steps; ++k) e.predict(f, Eigen::Vector3d::Zero(), 0.01);
    return e.releaseAccumulator();
  };
  const NominalState level;
  const double along_x = run(level, Eigen::Vector3d(1.0, 0.0, 9.81), 200);
  EXPECT_GT(along_x, 1.5);
  EXPECT_NEAR(run(level, Eigen::Vector3d(0.0, 1.0, 9.81), 200), along_x, 1e-12);          // sideways
  EXPECT_NEAR(run(level, Eigen::Vector3d(0.0, -1.0, 9.81), 200), along_x, 1e-12);         // and braking
  EXPECT_NEAR(run(level, Eigen::Vector3d(0.6, 0.8, 9.81), 200), along_x, 1e-9);           // a diagonal: its length
  EXPECT_NEAR(run(level, Eigen::Vector3d(0.0, 0.0, 9.81 + 3.0), 200), 0.0, 1e-12);        // a vertical bump
  NominalState tilted;
  tilted.q = so3Exp(Eigen::Vector3d(0.3, -0.2, 0.7));                                      // at rest on a slope: f = R^T (0, 0, g)
  const Eigen::Vector3d f_rest = tilted.q.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, 9.81);
  EXPECT_GT(Eigen::Vector2d(f_rest.x(), f_rest.y()).norm(), 2.0);                          // the body-frame reading has a horizontal part
  EXPECT_NEAR(run(tilted, f_rest, 500), 0.0, 1e-9);                                        // the world-frame one has none
  EXPECT_GT(run(tilted, f_rest + tilted.q.toRotationMatrix().transpose() * Eigen::Vector3d(1.0, 0.0, 0.0), 200), 1.5);   // 1 m/s^2 north on it
}

// --- the honest release: the shadow of the heading variance, the kinematic alignment, the blind update
//     (lever_restore, lever_align_sigma, lever_align_dv, lever_blind) ---

// With the release open and the heading variance held by nothing (the filter's own, 4 rad^2), the bound is not applied
// and the full update sees all of it; the honest version of BoundAlsoAppliesWhileReleased.
TEST(EskfLever, HonestReleaseIsNotBounded) {
  EskfParams ep;
  Eskf f = accelerated(ep, 1.0, 2.0, 0.0);
  ASSERT_TRUE(f.leverReleaseOpen());
  Eskf::Mat15 P = f.covariance();
  P(8, 8) = 4.0;
  f.initialize(f.state(), P);
  // initialize() cleared the accumulator? No: the accumulator belongs to predict(); open stays true.
  ASSERT_TRUE(f.leverReleaseOpen());
  nullFix(f, Eigen::Vector3d(0.5, 0.0, 1.0));
  EXPECT_FALSE(f.leverHeadingCapped());
  EXPECT_NEAR(f.headingSigma(), 2.0, 1e-6);
  EXPECT_FALSE(f.leverBlind());
}

namespace {
struct GoRun {
  double heading_err_end{0.0}, tilt_end{0.0}, z_max{0.0}, sigma_capped{0.0}, sigma_honest{0.0};
  int aligned{0}, blind_updates{0};
  bool blind_when_standing{false};
};
// A robot that stands for `stand_s` on GNSS and IMU alone, its heading `yaw0` off from the start (the filter keeps 0.5 rad^2),
// with a vertical gyro bias, then accelerates at 0.5 m/s^2 along x for 3 s and cruises for `cruise_s`. Antenna at `lever`.
GoRun runStandGo(EskfParams ep, const Eigen::Vector3d& lever, double stand_s, double cruise_s, double yaw0, double bias_dps, unsigned seed,
                 bool unknown_start = false) {
  Rng rng(seed);
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  auto noise3 = [&rng](double s) { return Eigen::Vector3d(rng.gaussian(0, s), rng.gaussian(0, s), rng.gaussian(0, s)); };
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, yaw0));
  x0.p = lever + noise3(0.02) - x0.q * lever;
  f.initialize(x0, P0);
  if (unknown_start) f.markHeadingUnknown();
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  const Eigen::Vector3d bias(0.0, 0.0, bias_dps * M_PI / 180.0);
  const double dt = 0.01;
  GoRun out;
  const int n_stand = static_cast<int>(stand_s / dt), n = n_stand + static_cast<int>((3.0 + cruise_s) / dt);
  for (int k = 1; k <= n; ++k) {
    const double t = k * dt - stand_s;   // time since the start
    double a = 0.0, x = 0.0;
    if (t > 0.0 && t <= 3.0) { a = 0.5; x = 0.25 * t * t; }
    else if (t > 3.0) { x = 2.25 + 1.5 * (t - 3.0); }
    f.predict(Eigen::Vector3d(a, 0, 9.81) + noise3(0.02), bias + noise3(0.002), dt);
    if (k % 20 == 0) {
      f.updatePositionLever(Eigen::Vector3d(x, 0, 0) + lever + noise3(0.02), lever, R);
      out.aligned += f.leverAligned() ? 1 : 0;
      if (k <= n_stand && f.leverBlind()) out.blind_when_standing = true;
      out.blind_updates += f.leverBlind() ? 1 : 0;
    }
    if (k == n_stand) { out.sigma_capped = f.headingSigma(); out.sigma_honest = f.headingSigmaHonest(); }
    if (k > n_stand) out.z_max = std::max(out.z_max, std::fabs(f.state().p.z()));
  }
  const Eigen::Vector3d att = so3Log(f.state().q);
  out.heading_err_end = std::fabs(att.z());       // truth yaw is 0 after the start (straight line)
  out.tilt_end = att.head<2>().norm();
  return out;
}
}  // namespace

// The failure of the fourth review: a robot that stands for minutes on GNSS and IMU and then starts. The heading was off by
// the integrated vertical gyro bias (1.2 rad here), the bound held the filter at 0.35 rad, and the full update at the release
// turned the heading the wrong way and tilted the filter. Now: the filter reports the honest spread while it stands, finds the heading from the
// first 0.5 m/s^2 of the start, and ends the start with a small heading error, a level attitude and the right height.
// The control is the previous behaviour (lever_restore off): wrong heading, tilt, height.
TEST(EskfLever, StartAfterALongStandFindsTheHeading) {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  const double bias_dps = 0.1;
  // 0.1 deg/s on the vertical axis for 600 s: 1.05 rad of heading; start estimate off by the same, plus the 0.1 rad of the start
  const GoRun now = runStandGo(EskfParams(), lever, 600.0, 15.0, 0.1, bias_dps, 3);
  EXPECT_NEAR(now.sigma_capped, EskfParams().lever_heading_sigma_max, 1e-3);   // the bound holds the covariance
  EXPECT_GT(now.sigma_honest, 3.0);                                            // the shadow does not
  EXPECT_GE(now.aligned, 1);
  EXPECT_LT(now.heading_err_end, 0.35);
  EXPECT_LT(now.tilt_end, 0.03);
  EXPECT_LT(now.z_max, 0.3);
  EskfParams old;
  old.lever_restore = false;
  const GoRun before = runStandGo(old, lever, 600.0, 15.0, 0.1, bias_dps, 3);
  EXPECT_EQ(before.aligned, 0);
  EXPECT_GT(before.heading_err_end + before.tilt_end * 10.0 + before.z_max, now.heading_err_end + now.tilt_end * 10.0 + now.z_max);
}

// The same on every antenna mount, with the filter started 1.5 rad off in heading and no gyro bias: the alignment finds
// any heading, not only one near the filter's.
TEST(EskfLever, AlignmentFindsAnyHeading) {
  for (const Eigen::Vector3d& lever : {Eigen::Vector3d(0.5, 0.0, 1.0), Eigen::Vector3d(-0.3, 0.2, 1.0), Eigen::Vector3d(0.0, 0.0, 1.5)}) {
    for (double yaw0 : {-2.8, -1.5, 0.1, 2.0, 3.1}) {
      const GoRun r = runStandGo(EskfParams(), lever, 300.0, 10.0, yaw0, 0.0, 5);
      const std::string at = "lever x " + std::to_string(lever.x()) + " yaw0 " + std::to_string(yaw0);
      EXPECT_GE(r.aligned, 1) << at;
      EXPECT_LT(r.heading_err_end, 0.35) << at;
      // The 1.5 m mast has no horizontal arm, so the lever term tells it nothing about the heading, and its vertical arm
      // amplifies a tilt error: it ends the 10 s with up to 0.055 rad (the others under 0.03).
      EXPECT_LT(r.tilt_end, lever.head<2>().norm() > 0.0 ? 0.03 : 0.06) << at;
      EXPECT_LT(r.z_max, 0.3) << at;
    }
  }
}

// Standing is not a change of velocity: no alignment, and once the honest spread is over lever_align_sigma the update is blind
// (the attitude columns of the lever Jacobian are left out). Before that (a short stand) it is not blind.
TEST(EskfLever, NoAlignmentWithoutAChangeOfVelocityAndBlindWhileTheHeadingIsUnknown) {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  const GoRun short_stand = runStandGo(EskfParams(), lever, 20.0, 0.0, 0.1, 0.0, 7);   // spread 0.5 rad^2 + the bias prior: still under 1 rad at 20 s? it is not
  (void)short_stand;
  const GoRun long_stand = runStandGo(EskfParams(), lever, 300.0, 0.0, 0.1, 0.0, 7);
  EXPECT_TRUE(long_stand.blind_when_standing);
  EXPECT_GT(long_stand.sigma_honest, 1.0);
  // no start (cruise 0 still has the 3 s of acceleration); count alignments that happened while standing
  EskfParams ep;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  NominalState x0;
  x0.p = Eigen::Vector3d(0.0, 0.0, 0.0);
  f.initialize(x0, P0);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  int aligned = 0;
  for (int k = 1; k <= 20000; ++k) {
    f.predict(Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d(0, 0, 0.002), 0.01);
    if (k % 20 == 0) {
      f.updatePositionLever(x0.q * lever, lever, R);
      aligned += f.leverAligned() ? 1 : 0;
    }
  }
  EXPECT_EQ(aligned, 0);
  EXPECT_TRUE(f.leverBlind());
  EXPECT_GT(f.headingSigmaHonest(), 1.0);
  EXPECT_NEAR(f.headingSigma(), ep.lever_heading_sigma_max, 1e-3);
}

// lever_restore off is the version before: none of the new states is ever reached, and the honest heading sigma is the
// filter's.
TEST(EskfLever, RestoreOffNeverAlignsOrGoesBlind) {
  EskfParams ep;
  ep.lever_restore = false;
  const GoRun r = runStandGo(ep, Eigen::Vector3d(0.5, 0.0, 1.0), 300.0, 5.0, 0.1, 0.1, 3);
  EXPECT_EQ(r.aligned, 0);
  EXPECT_EQ(r.blind_updates, 0);
  EXPECT_DOUBLE_EQ(r.sigma_honest, r.sigma_capped);
}

// --- the fifth review: an outage across the start, and the default boot with a heading that is only a guess ---

namespace {
struct GapRun { int aligned_after{0}, aligned_total{0}; double heading_err_end{0.0}, tilt_max{0.0}, z_max{0.0}, sigma_end{0.0}; };
// Stands `stand_s` on GNSS and IMU alone with a vertical gyro bias, the filter 0.1 rad off in heading; the fixes stop `gap_before` s
// before the robot starts and come back `gap_s` s after they stopped. The robot runs the start-stop cycle (0.5 m/s^2 for 3 s, 20 s at
// 1.5 m/s, 0.5 m/s^2 of braking for 3 s, still) all through the outage and for 60 s after the fixes return.
GapRun runGap(EskfParams ep, const Eigen::Vector3d& lever, double stand_s, double gap_before, double gap_s, double bias_dps, unsigned seed) {
  Rng rng(seed);
  ep.sigma_acc = 0.02;
  ep.sigma_gyro = 0.002;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  auto noise3 = [&rng](double s) { return Eigen::Vector3d(rng.gaussian(0, s), rng.gaussian(0, s), rng.gaussian(0, s)); };
  NominalState x0;
  x0.q = so3Exp(Eigen::Vector3d(0, 0, 0.1));
  x0.p = lever + noise3(0.02) - x0.q * lever;
  f.initialize(x0, P0);
  const Eigen::Matrix3d R = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();
  const Eigen::Vector3d bias(0.0, 0.0, bias_dps * M_PI / 180.0);
  const double dt = 0.01;
  GapRun out;
  const double t_back = gap_s - gap_before;   // time since the start at which the fixes return
  const int n_stand = static_cast<int>(stand_s / dt), n = n_stand + static_cast<int>((t_back + 60.0) / dt);
  for (int k = 1; k <= n; ++k) {
    const double t = k * dt - stand_s;
    double a = 0.0, x = 0.0;
    if (t > 0.0) {
      const double cyc = std::floor(t / 40.0), r = t - 40.0 * cyc;
      if (r < 6.0) { x = 0.0; }
      else if (r < 9.0) { const double u = r - 6.0; x = 0.25 * u * u; a = 0.5; }
      else if (r < 29.0) { x = 2.25 + 1.5 * (r - 9.0); }
      else if (r < 32.0) { const double u = r - 29.0; x = 32.25 + 1.5 * u - 0.25 * u * u; a = -0.5; }
      else { x = 34.5; }
      x += 34.5 * cyc;
    }
    f.predict(Eigen::Vector3d(a, 0, 9.81) + noise3(0.02), bias + noise3(0.002), dt);
    const bool in_gap = t >= -gap_before && t < t_back;
    if (k % 20 == 0 && !in_gap) {
      f.updatePositionLever(Eigen::Vector3d(x, 0, 0) + lever + noise3(0.02), lever, R);
      if (f.leverAligned()) { ++out.aligned_total; if (t >= t_back) ++out.aligned_after; }
    }
    if (t >= t_back + 2.0) {   // after the first fixes
      const Eigen::Vector3d att = so3Log(f.state().q);
      out.tilt_max = std::max(out.tilt_max, att.head<2>().norm());
      out.z_max = std::max(out.z_max, std::fabs(f.state().p.z()));
    }
  }
  out.heading_err_end = std::fabs(so3Log(f.state().q).z());
  out.sigma_end = f.headingSigmaHonest();
  return out;
}
}  // namespace

// The fifth review: the fixes stop 10 s before the start and come back 120 s later, while the robot runs the start-stop cycle (the
// fixes return during a braking). The alignment used the last fix before the outage and the IMU sum over the whole outage, and set the
// heading from them 0.8 s after the fixes returned (heading 0.3 rad off, locked at 0.35 rad of spread, tilt 0.7 rad, height -43 m in the
// reviewer's simulation). Now a gap anywhere between the two spans refuses the alignment.
TEST(EskfLever, OutageAcrossTheStartDoesNotAlignOnTheWholeOutage) {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  for (unsigned seed : {3u, 4u, 5u}) {
    const GapRun r = runGap(EskfParams(), lever, 300.0, 10.0, 120.0, 0.2, seed);
    EXPECT_LT(r.tilt_max, 0.1) << "seed " << seed;
    EXPECT_LT(r.z_max, 0.5) << "seed " << seed;
  }
}

// A real alignment still works when the outage is short: the window and both spans are free of gaps once the fixes have been back
// for the window and a span.
TEST(EskfLever, AlignmentWorksAgainAfterAnOutage) {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  const GapRun r = runGap(EskfParams(), lever, 300.0, 10.0, 12.0, 0.2, 3);   // the fixes return 2 s after the start
  EXPECT_GE(r.aligned_total, 1);
  EXPECT_LT(r.heading_err_end, 0.35);
}

// The default boot: GNSS and IMU, no /initialpose, initial_yaw 2 rad off, a stand of 20 s, then the start. The filter's own start
// spread is 0.71 rad, so the first start used to run a full update with a heading 2 rad off. With markHeadingUnknown() (the node
// calls it when there is no heading source) the first start finds the heading.
TEST(EskfLever, StartWithAGuessedHeadingFindsItAtTheFirstStart) {
  const Eigen::Vector3d lever(0.5, 0.0, 1.0);
  EskfParams ep;
  const GoRun marked = runStandGo(ep, lever, 20.0, 10.0, 2.0, 0.1, 3, true);
  EXPECT_GE(marked.aligned, 1);
  EXPECT_LT(marked.heading_err_end, 0.35);
  EXPECT_LT(marked.tilt_end, 0.03);
  EXPECT_LT(marked.z_max, 0.3);
  const GoRun plain = runStandGo(ep, lever, 20.0, 10.0, 2.0, 0.1, 3, false);
  EXPECT_GT(plain.z_max + plain.tilt_end * 10.0 + plain.heading_err_end, marked.z_max + marked.tilt_end * 10.0 + marked.heading_err_end);
}

// markHeadingUnknown() touches neither x nor P; it holds the honest spread at lever_start_sigma, and does nothing when that is 0
// or under lever_align_sigma, or when lever_restore is off.
TEST(EskfLever, MarkHeadingUnknownOnlySetsTheShadow) {
  EskfParams ep;
  Eskf f(ep);
  Eskf::Mat15 P0 = Eskf::Mat15::Identity();
  P0.block<3, 3>(6, 6) = Eigen::Vector3d(1e-4, 1e-4, 0.5).asDiagonal();
  NominalState x0;
  f.initialize(x0, P0);
  const Eskf::Mat15 P_before = f.covariance();
  const Eigen::Quaterniond q_before = f.state().q;
  EXPECT_NEAR(f.headingSigmaHonest(), std::sqrt(0.5), 1e-9);
  ASSERT_TRUE(f.markHeadingUnknown());
  EXPECT_EQ((f.covariance() - P_before).norm(), 0.0);
  EXPECT_EQ(f.state().q.coeffs(), q_before.coeffs());
  EXPECT_NEAR(f.headingSigmaHonest(), ep.lever_start_sigma, 1e-9);
  EXPECT_GT(f.headingSigmaHonest(), ep.lever_align_sigma);
  for (int variant = 0; variant < 3; ++variant) {
    EskfParams q;
    if (variant == 0) q.lever_start_sigma = 0.0;
    if (variant == 1) q.lever_start_sigma = 0.9 * q.lever_align_sigma;
    if (variant == 2) q.lever_restore = false;
    Eskf g(q);
    g.initialize(x0, P0);
    EXPECT_FALSE(g.markHeadingUnknown()) << variant;
    EXPECT_NEAR(g.headingSigmaHonest(), std::sqrt(0.5), 1e-9) << variant;
  }
}
