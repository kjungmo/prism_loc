#include <gtest/gtest.h>
#include <cmath>
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
