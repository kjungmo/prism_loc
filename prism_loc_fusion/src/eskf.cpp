#include "prism_loc_fusion/eskf.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include "prism_loc_fusion/so3.hpp"
namespace prism_loc_fusion {

void Eskf::initialize(const NominalState& x0, const Mat15& P0) {
  x_ = x0; x_.q.normalize(); P_ = P0;
  shadow_valid_ = false;
  fixes_.clear();
  unknown_pending_ = false;
  rate_unsupported_ = false;
  rate_degraded_ = false;
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

  // Shadow of the heading: the same prediction on (heading, vertical gyro bias) alone.
  if (shadow_valid_) {
    s_hh_ += -2.0 * dt * s_hb_ + dt * dt * s_bb_ + sg2 * dt * dt;
    s_hb_ += -dt * s_bb_;
    s_bb_ += sbg2 * dt;
  }

  // Recent horizontal acceleration, for the release of the lever-arm consider states. It
  // reads a_w and changes neither the state nor P.
  if (dt > 0.0 && params_.lever_release_tau > 0.0) {
    const double a_h = std::hypot(a_w.x(), a_w.y());
    if (std::isfinite(a_h)) dv_acc_ = dv_acc_ * std::exp(-dt / params_.lever_release_tau) + a_h * dt;
    if (std::isfinite(a_h)) {   // the kinematic alignment's clock and sums (alignHeading, spreadAfterOutage)
      clock_ += dt;
      cum_a_ += a_w.head<2>() * dt;
      cum_p_ += cum_a_ * dt;
      rate_max_ = std::max(rate_max_, rate_);
    }
  }
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
  shadow_valid_ = false;
  P_.block<3, 15>(6, 0).setZero();
  P_.block<15, 3>(0, 6).setZero();
  P_.block<3, 3>(6, 6) = P_att;
}

void Eskf::replaceAttitude(const Eigen::Quaterniond& q, const Eigen::Matrix3d& P_att) {
  x_.q = q.normalized();
  shadow_valid_ = false;
  replaceAttitudeCovariance(P_att);
}

void Eskf::syncShadow(const Eigen::Vector3d& up) {
  s_hh_ = up.dot(P_.block<3, 3>(6, 6) * up);
  s_hb_ = up.dot(P_.block<3, 3>(6, 12) * up);
  s_bb_ = up.dot(P_.block<3, 3>(12, 12) * up);
  shadow_valid_ = true;
  shadow_held_ = false;
}

bool Eskf::markHeadingUnknown() {
  if (!(params_.lever_start_sigma > 0.0) || !(params_.lever_align_sigma > 0.0) ||
      !(params_.lever_start_sigma > params_.lever_align_sigma))
    return false;
  const Eigen::Vector3d up = x_.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  s_hh_ = params_.lever_start_sigma * params_.lever_start_sigma;
  s_hb_ = 0.0;
  s_bb_ = up.dot(P_.block<3, 3>(12, 12) * up);
  shadow_valid_ = true;
  shadow_held_ = true;
  unknown_pending_ = true;
  return true;
}

double Eskf::fixPeriod() const {
  const size_t n = fixes_.size();
  if (n < 3) return 0.0;
  const size_t m = std::min<size_t>(n - 1, 15);
  std::array<double, 15> d;
  for (size_t i = 0; i < m; ++i) d[i] = fixes_[n - 1 - i].t - fixes_[n - 2 - i].t;
  std::nth_element(d.begin(), d.begin() + m / 2, d.begin() + m);
  return d[m / 2];
}

// Give the heading the variance `var`, independent of every other state. The heading error variable
// is projected out of P (P <- D P D^T, D = I - u u^T on the attitude block: the filter then has no
// heading, and keeps P positive) and a new heading of variance `var` is added. What the bound
// scaled, and the covariance the heading had built with position, velocity and the biases, is gone.
void Eskf::setHeadingVariance(const Eigen::Vector3d& up, double var) {
  const Eigen::Matrix3d along = up * up.transpose();
  Mat15 D = Mat15::Identity();
  D.block<3, 3>(6, 6) -= along;
  P_ = D * P_ * D.transpose();
  P_.block<3, 3>(6, 6) += var * along;
  s_hh_ = var;
  s_hb_ = 0.0;
  shadow_held_ = false;
}

// Lengths of the alignment (seconds), from the measured fix period T: the span of one velocity, the window between the two
// velocities, the longest interval that is not a gap, and the history that is checked for gaps and fast turns.
Eskf::Span Eskf::alignSpans(double T) const {
  Span r;
  r.span = std::max(params_.lever_align_span, 2.0 * T);
  r.window = std::max(params_.lever_align_window, 1.5 * r.span);
  r.gap = std::max(params_.lever_align_gap, 2.5 * T);
  r.history = r.window + std::max(1.0, r.span + std::max(0.2, 0.5 * T));
  return r;
}

// Drop the fixes that no alignment can read any more. The alignment looks back at most the history plus the span of the older
// velocity from the newest fix. The buffer keeps the last kMinFixes fixes (what the versions before kept: up to 22 Hz that covers
// twice the history, and an outage of the fixes leaves the fixes from before it in place) and, above that, the fixes of twice that
// time: at 100 Hz a count of 256 is 2.6 s, shorter than the 4 s window of the alignment. kMaxFixes bounds the memory of a
// receiver that sends faster than 400 Hz.
void Eskf::trimFixes() {
  constexpr size_t kMinFixes = 256, kMaxFixes = 8192;
  const double T = fixPeriod();
  if (T > 0.0) {
    const Span sp = alignSpans(T);
    const double keep = 2.0 * (sp.history + 2.0 * sp.span + 2.0 * T);
    while (fixes_.size() > kMinFixes && fixes_.back().t - fixes_.front().t > keep) fixes_.pop_front();
  } else {
    while (fixes_.size() > kMinFixes) fixes_.pop_front();
  }
  while (fixes_.size() > kMaxFixes) fixes_.pop_front();
}

// Kinematic alignment. The antenna fixes give the velocity of the antenna without the filter, and
// so without its heading: for a robot that translates, the antenna moves as the body does. Over
// the last lever_align_window seconds, the change of that velocity is compared with the change of
// velocity that the IMU gave when resolved with the filter's heading (cum_a_, the sum of the
// world-frame horizontal acceleration that predict() kept). If the heading is wrong by e, the two
// differ by a rotation about the vertical of e, whatever e is. The heading is rotated by that
// angle, about the antenna so that the antenna position does not move, and gets the variance
// lever_heading_sigma_max^2. Both changes of velocity must be over lever_align_dv, or nothing is
// done and false is returned.
// The spans are seconds (lever_align_span): the velocity is taken over max(span, 2 T), and the
// window, the gap limit and the history scale with the measured fix period T.
bool Eskf::alignHeading(const Eigen::Vector3d& lever, const Eigen::Vector3d& up) {
  const size_t n = fixes_.size();
  if (n < 6) return false;
  const double T = fixPeriod();
  if (!(T > 0.0)) return false;
  if (params_.lever_align_period_max > 0.0 && T > params_.lever_align_period_max) return false;
  const Span sp = alignSpans(T);
  const double span = sp.span, window = sp.window, gap = sp.gap, slack = std::max(1.0, 1.5 * T), history = sp.history;
  // Index of the first fix of the span that ends at fix k; fixes_.size() if the history is too short.
  auto span_begin = [&](size_t k) -> size_t {
    for (size_t j = k; j-- > 0;)
      if (k - j >= 2 && fixes_[k].t - fixes_[j].t >= span - 0.5 * T) return j;
    return fixes_.size();
  };
  // Velocity of the antenna and the IMU sum at the middle of a span that ends at fix k.
  struct Vel { Eigen::Vector2d v, s; double t; bool ok; };
  auto sum_at = [&](double tm) {   // cum_a_ at time tm, linear between the fixes
    for (size_t i = 1; i < fixes_.size(); ++i)
      if (fixes_[i].t >= tm) {
        const Fix& a = fixes_[i - 1]; const Fix& b = fixes_[i];
        const double w = b.t > a.t ? (tm - a.t) / (b.t - a.t) : 1.0;
        return Eigen::Vector2d(a.s + w * (b.s - a.s));
      }
    return Eigen::Vector2d(fixes_.back().s);
  };
  auto vel_at = [&](size_t k) {
    const size_t j = span_begin(k);
    Vel r{Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), 0.0, false};
    if (j >= fixes_.size()) return r;
    const Fix& e = fixes_[k]; const Fix& b = fixes_[j];
    r.t = 0.5 * (e.t + b.t);
    const double dt = e.t - b.t;
    if (dt > 0.5 * span && dt < std::max(2.0, 2.5 * span)) {
      r.v = (e.p - b.p) / dt; r.s = sum_at(r.t); r.ok = true;
    }
    return r;
  };
  // A gap in the fixes (longer than the gap limit) or a body that turns fast (the antenna moves by |rate| |t_xy| without the body moving) in
  // the window: the antenna's change of velocity is then not the body's. Nothing is done.
  const double t_xy = lever.head<2>().norm();
  const Fix& newest = fixes_[n - 1];
  for (size_t i = n - 1; i > 0; --i) {
    if (newest.t - fixes_[i - 1].t > history) break;   // older than the window and its first span
    if (fixes_[i].t - fixes_[i - 1].t > gap) return false;
    if (fixes_[i].rate_max * t_xy > 0.5 * params_.lever_align_dv) return false;
  }
  const Vel now = vel_at(n - 1);
  size_t i0 = n - 1;   // the span that ends the window earlier
  while (i0 > 0 && fixes_[n - 1].t - fixes_[i0].t < window) --i0;
  if (span_begin(i0) >= fixes_.size() || span_begin(i0) == 0) return false;   // the span of the older velocity must start after the first fix
  // The span that ends the window earlier must be as free of gaps and fast turns as the window, and
  // the two spans must lie within the window: after an outage the walk back above stops at the last
  // fix before it, and the change of velocity would then be taken over the whole outage.
  if (fixes_[n - 1].t - fixes_[i0].t > window + slack) return false;
  for (size_t i = i0; i > span_begin(i0); --i)
    if (fixes_[i].t - fixes_[i - 1].t > gap ||
        fixes_[i].rate_max * t_xy > 0.5 * params_.lever_align_dv) return false;
  const Vel before = vel_at(i0);
  if (!now.ok || !before.ok) return false;
  const Eigen::Vector2d a = now.s - before.s, v = now.v - before.v;
  if (!(a.norm() > params_.lever_align_dv && v.norm() > params_.lever_align_dv)) return false;
  const double delta = std::atan2(a.x() * v.y() - a.y() * v.x(), a.dot(v));
  if (!std::isfinite(delta)) return false;
  const Eigen::Matrix3d Rz = Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d R_old = x_.q.toRotationMatrix();
  x_.q = (Eigen::Quaterniond(Rz) * x_.q).normalized();
  x_.p += R_old * lever - x_.q.toRotationMatrix() * lever;   // the antenna stays where it was
  // The velocity the filter integrated with the wrong heading is wrong too: take the antenna's, carried to now by
  // the IMU sum in the new heading, and let the update below trust it only as far as the fixes allow.
  const Eigen::Vector2d v_now = now.v + Rz.topLeftCorner<2, 2>() * (cum_a_ - now.s);
  x_.v.head<2>() = v_now;
  {
    Mat15 D = Mat15::Identity();   // project the horizontal velocity out of P, then give it a variance of its own
    D(3, 3) = 0.0;
    D(4, 4) = 0.0;
    P_ = D * P_ * D.transpose();
    P_(3, 3) += params_.lever_align_vel_std * params_.lever_align_vel_std;
    P_(4, 4) += params_.lever_align_vel_std * params_.lever_align_vel_std;
  }
  const double sig = params_.lever_heading_sigma_max > 0.0 ? params_.lever_heading_sigma_max : kLeverHeadingSigmaMax;
  setHeadingVariance(up, sig * sig);
  // The body origin is known only up to the circle of the lever around the antenna, to the heading's spread.
  const Eigen::Vector3d tw = x_.q.toRotationMatrix() * lever;
  const double var_p = tw.head<2>().squaredNorm() * sig * sig;
  P_(0, 0) += var_p;
  P_(1, 1) += var_p;
  // The position that predict() dead-reckoned through the blind stretch is not the antenna's: seat it at the newest fix, so that the
  // update that follows does not see the whole drift as an innovation and spend it on the heading and the gyro bias.
  x_.p.head<2>() = newest.p - (x_.q.toRotationMatrix() * lever).head<2>();
  fixes_.clear();
  lever_aligned_ = true;
  return true;
}

// The first lever update after an outage of the fixes, with the heading unknown (EskfParams::lever_outage_spread). predict()
// dead-reckoned the acceleration in the filter's heading. If that is off by e, the velocity gained over the outage is dS rotated
// by e (dS: the IMU sum, cum_a_), and the position gained from the acceleration is D rotated by e (D: the integral of dS over the
// outage, cum_p_). So the velocity and position are uncertain by e |dS| and e |D|, fully correlated, whatever P said with the
// heading held at its bound. The horizontal position and velocity block of P gets min(heading spread^2, 2) / 2 times
// [[|D|^2, D.dS], [D.dS, |dS|^2]] on each axis (2 rad^2: a heading uniform on the circle). The block is positive semidefinite
// (Cauchy-Schwarz); the fixes that follow correct the velocity quickly, instead of explaining its error as a tilt or an
// accelerometer bias.
void Eskf::spreadAfterOutage() {
  const size_t n = fixes_.size();
  if (n < 2) return;
  const Fix& a = fixes_[n - 2];
  const Fix& b = fixes_[n - 1];
  const double T = fixPeriod();
  const double gap_limit = std::max(params_.lever_align_gap, 2.5 * T);
  const double gap = b.t - a.t;
  if (!(gap > gap_limit)) return;
  const double k = params_.lever_outage_spread * 0.5 * std::min(s_hh_, 2.0);
  const Eigen::Vector2d dS = b.s - a.s;
  const Eigen::Vector2d D = (b.d - a.d) - a.s * gap;
  const double pp = k * D.squaredNorm(), pv = k * D.dot(dS), vv = k * dS.squaredNorm();
  if (!std::isfinite(pp + pv + vv)) return;
  for (int i = 0; i < 2; ++i) {
    P_(i, i) += pp;
    P_(i, 3 + i) += pv;
    P_(3 + i, i) += pv;
    P_(3 + i, 3 + i) += vv;
  }
}

double Eskf::headingSigmaHonest() const {
  const double s = headingSigma();
  if (!shadow_valid_ || !(s_hh_ > s * s)) return s;
  return std::sqrt(s_hh_);
}

Eskf::Mat15 Eskf::covarianceHonest(const Eigen::Vector3d& lever) const {
  Mat15 P = P_;
  if (!shadow_valid_ || !shadow_held_) return P;
  const Eigen::Vector3d up = x_.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const double var = up.dot(P_.block<3, 3>(6, 6) * up);
  if (!(s_hh_ > var)) return P;
  P.block<3, 3>(6, 6) += (s_hh_ - var) * up * up.transpose();
  const double extra = lever.head<2>().squaredNorm() * (std::min(s_hh_, 1.0) - std::min(var, 1.0));
  P(0, 0) += extra;
  P(1, 1) += extra;
  return P;
}

double Eskf::headingSigma() const {
  const Eigen::Vector3d u = x_.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const double var = u.dot(P_.block<3, 3>(6, 6) * u);
  return var > 0.0 ? std::sqrt(var) : (std::isnan(var) ? var : 0.0);   // a NaN covariance reads as NaN, not as 0
}

void Eskf::updatePositionLever(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& lever,
                               const Eigen::Matrix3d& R) {
  if (lever.isZero(0.0)) {  // the body origin itself: the plain position update, bit for bit
    updatePosition(p_meas, R);
    return;
  }
  // The antenna fix and the IMU sums at its time, for alignHeading() and spreadAfterOutage().
  fixes_.push_back({clock_, cum_a_, p_meas.head<2>(), rate_max_, cum_p_});
  rate_max_ = 0.0;
  trimFixes();
  unsigned consider = params_.lever_consider;
  const Eigen::Vector3d up = x_.q.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d along = up * up.transpose();
  // Release: while the IMU measures horizontal acceleration the heading is observable.
  bool open = leverReleaseOpen();
  bool blind = false;   // the heading is unknown: the antenna says where the circle of the lever is, nothing about roll and pitch
  if (open) consider = 0u;   // released: the full update, heading and vertical gyro bias included
  lever_aligned_ = false;
  lever_blind_ = false;
  if (!shadow_valid_) syncShadow(up);
  if (fixes_.size() >= 6) {   // enough intervals to know the fix rate
    const double period = fixPeriod();
    rate_unsupported_ = params_.lever_align_period_max > 0.0 && period > params_.lever_align_period_max;
    rate_degraded_ = !rate_unsupported_ && params_.lever_align_period_ok > 0.0 && period > params_.lever_align_period_ok;
    if (unknown_pending_) {
      unknown_pending_ = false;
      if (rate_unsupported_) {   // the alignment cannot run at this rate: the start is not marked unknown
        shadow_held_ = false;
        syncShadow(up);
      }
    }
  }
  // The bound kept the heading variance below the honest one (held_back), and the honest spread is over
  // the range of the lever term (wide): re-acquire the heading from the motion (kinematic alignment) as
  // soon as the robot has changed its velocity enough, and not with the linearised update.
  bool held_back = shadow_held_;   // the bound has cut the heading variance since the shadow was set from P
  const bool wide = params_.lever_align_sigma > 0.0 && std::sqrt(s_hh_) > params_.lever_align_sigma;
  if (held_back && wide) {
    if (alignHeading(lever, up)) held_back = false;
    else {   // the motion does not give the heading (yet): stay closed, and do not let the lever term tell roll and pitch what a heading error is
      if (params_.lever_outage_spread > 0.0) spreadAfterOutage();   // after an outage the dead-reckoned velocity and position are as uncertain as the heading
      blind = true;
      if (open) {
        open = false;
        consider = params_.lever_consider;
      }
    }
  }
  if (open && held_back) {   // put the honest variance back
    setHeadingVariance(up, s_hh_);
  }
  // Keep the heading uncertainty this update sees inside the range where -R[t]x is a fair
  // linear model of the lever term (lever_heading_sigma_max). The bound is not applied while the
  // release is open: the heading is then estimated with its honest variance.
  lever_capped_ = false;
  if (params_.lever_heading_sigma_max > 0.0 && !open) {
    const double var_h = up.dot(P_.block<3, 3>(6, 6) * up);
    const double cap2 = params_.lever_heading_sigma_max * params_.lever_heading_sigma_max;
    if (var_h > cap2) {
      const double c = std::sqrt(cap2 / var_h);
      Mat15 D = Mat15::Identity();
      D.block<3, 3>(6, 6) += (c - 1.0) * along;
      P_ = D * P_ * D.transpose();
      lever_capped_ = true;
      shadow_held_ = true;
    }
  }
  // Remove from the gain the parts that are not to be corrected.
  auto limit = [&](Eigen::Matrix<double, 15, 3>& K) {
    if (consider & kConsiderHeading) K.block<3, 3>(6, 0) -= along * K.block<3, 3>(6, 0);
    if (consider & kConsiderGyroBiasVertical) K.block<3, 3>(12, 0) -= along * K.block<3, 3>(12, 0);
  };
  lever_blind_ = blind;
  Eigen::Matrix3d R_eff = R;
  if (blind) {
    R_eff(0, 0) += lever.head<2>().squaredNorm();
    R_eff(1, 1) += lever.head<2>().squaredNorm();
  }
  Eigen::Matrix<double, 3, 15> H = leverArmJacobian(x_.q, lever);
  const Eigen::Vector3d y = p_meas - leverArmPosition(x_, lever);
  if (blind) H.block<3, 3>(0, 6).setZero();
  const Eigen::Matrix3d S = H * P_ * H.transpose() + R_eff;
  Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
  limit(K);
  const Vec15 dx = K * y;
  // Joseph form: valid for a gain that is not the optimal one.
  const Mat15 A = Mat15::Identity() - K * H;
  P_ = A * P_ * A.transpose() + K * R_eff * K.transpose();
  inject(dx);
  if (open) syncShadow(up);   // the heading was estimated: the filter's covariance is the honest one again
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
  shadow_valid_ = false;   // the pose fix corrects the heading by its own covariance
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
