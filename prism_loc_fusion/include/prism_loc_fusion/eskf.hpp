#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>
#include "prism_loc_fusion/types.hpp"
namespace prism_loc_fusion {
class Eskf {
 public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;
  explicit Eskf(const EskfParams& params = {}) : params_(params) { P_.setIdentity(); P_ *= 0.01; }
  void initialize(const NominalState& x0, const Mat15& P0);
  void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt);
  void updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas,
                  const Eigen::Matrix<double, 6, 6>& R);
  void updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R);
  // Position fix of a point fixed to the body at `lever` (body frame), e.g. a GNSS antenna:
  // h(x) = p + R(q) lever. The update uses the full Jacobian and the Joseph covariance, and a gain
  // from which the states in EskfParams::lever_consider are removed (Schmidt-Kalman "consider"
  // states): by default the heading and the vertical gyro bias, which one antenna and an IMU do not
  // observe while the robot stands. Their uncertainty still enters the innovation covariance, so
  // the position is not trusted beyond what the heading allows. Roll and pitch, the horizontal
  // gyro biases, position, velocity and the accelerometer bias are corrected. That needs a roll
  // and pitch variance of about 1e-4 rad^2 at the start (attitudeCovariance(),
  // tiltVarianceFromAccel()): with 0.5 rad^2 the first fixes tilt the filter.
  //
  // The rules that decide how much of the heading the update sees (README.md, "GNSS antenna off
  // base_frame"):
  //  - Bound: the heading standard deviation seen by the update is limited to
  //    EskfParams::lever_heading_sigma_max by scaling the heading error variable (P <- D P D^T).
  //  - Release: while releaseAccumulator() is over EskfParams::lever_release_dv, the mask is
  //    lever_consider_moving and the bound is not applied. The honest heading variance is put back.
  //  - Honest heading: the filter keeps the spread that the bound hides (headingSigmaHonest(),
  //    covarianceHonest()), re-finds the heading from a change of velocity (alignHeading()), and
  //    while it cannot, leaves roll and pitch out of the lever term (leverBlind()).
  //  - Unknown start (markHeadingUnknown()): a start whose heading is a guess holds the spread of
  //    an unknown heading, so that the first start or stop aligns it.
  //  - Outage (spreadAfterOutage()): after an outage with the heading unknown, the dead-reckoned
  //    velocity and position get the spread that the heading implies.
  // A lever of exactly zero is updatePosition(), with identical arithmetic.
  void updatePositionLever(const Eigen::Vector3d& p_meas, const Eigen::Vector3d& lever,
                           const Eigen::Matrix3d& R);
  // h(x) above, and its Jacobian with respect to the 15-dim error state. The attitude
  // error is local (q <- q * Exp(dtheta), as inject() applies it), so
  // dh/dp = I and dh/dtheta = -R(q) [lever]x.
  static Eigen::Vector3d leverArmPosition(const NominalState& x, const Eigen::Vector3d& lever);
  static Eigen::Matrix<double, 3, 15> leverArmJacobian(const Eigen::Quaterniond& q,
                                                       const Eigen::Vector3d& lever);
  // Covariance of the attitude error for an attitude q whose roll and pitch are known to
  // within var_tilt (rad^2, for example from the accelerometer) and whose heading is known
  // to var_heading: var_tilt about the two axes across the vertical, var_heading about the
  // vertical, no correlation. The error is local, so the vertical is R(q)^T z.
  static Eigen::Matrix3d attitudeCovariance(const Eigen::Quaterniond& q, double var_tilt,
                                            double var_heading);
  // Variance of the roll and pitch that follow from one accelerometer reading (specific
  // force, base frame): the accelerometer bias prior and the white noise over g, plus the
  // departure of the reading from g, which is acceleration the reading cannot tell from tilt.
  static double tiltVarianceFromAccel(const Eigen::Vector3d& specific_force, double var_acc_bias,
                                      const EskfParams& params);
  // Replace the attitude block of the covariance and drop the correlation of the attitude
  // with every other state. Valid only for a filter that has had no measurement since
  // initialize() (the node uses it for a lever arm found after the filter started).
  void replaceAttitudeCovariance(const Eigen::Matrix3d& P_att);
  // The same, with the attitude itself replaced (the node takes roll and pitch from the
  // accelerometer at a reseed). The position stays where it is.
  void replaceAttitude(const Eigen::Quaterniond& q, const Eigen::Matrix3d& P_att);
  // Heading standard deviation (rad): sqrt(u^T P_theta u), u = R^T z. This is what the filter's
  // covariance holds, which the bound (lever_heading_sigma_max) keeps small while the robot
  // stands; see headingSigmaHonest().
  double headingSigma() const;
  // The heading of the state just given to initialize() is a guess (no /initialpose, no lidar): hold its spread at
  // EskfParams::lever_start_sigma in the shadow, so that the first lever updates align the heading from the motion or run blind and do
  // not trust the start covariance. Does nothing if lever_start_sigma or lever_align_sigma is 0. Touches neither x nor P.
  // Returns true if it marked the shadow.
  bool markHeadingUnknown();
  // Median of the last 15 intervals between lever-arm fixes (s), 0 until 3 fixes have come. The alignment scales its span, window and gap
  // limit with it (EskfParams::lever_align_span).
  double fixPeriod() const;
  // True once the filter has seen enough fixes to know that their period is over EskfParams::lever_align_period_max (a rate under
  // 0.8 Hz): the heading alignment does not run, and a start marked by markHeadingUnknown() has been taken back. Stays true until
  // initialize().
  bool leverUnsupportedRate() const { return rate_unsupported_; }
  // True once the period is over EskfParams::lever_align_period_ok (1 Hz) and not over lever_align_period_max: the alignment runs and
  // finds the heading less often.
  bool leverDegradedRate() const { return rate_degraded_; }
  // Heading standard deviation (rad) without the bound: the larger of headingSigma() and the
  // shadow of the heading variance that predict() carries while the bound holds the heading back.
  // The same as headingSigma() when nothing is held back.
  double headingSigmaHonest() const;
  // True if the last updatePositionLever() restored the honest heading variance (the first
  // update after a release that follows a held-back stretch), and if it re-acquired the heading
  // from the motion first (lever_align_sigma).
  bool leverRestored() const { return lever_restored_; }
  bool leverAligned() const { return lever_aligned_; }
  // True if the last updatePositionLever() took the heading as unknown: held back, over
  // lever_align_sigma, and not alignable from the motion.
  bool leverBlind() const { return lever_blind_; }
  // True if the last updatePositionLever() had to scale the heading down to
  // lever_heading_sigma_max. False before the first one and after one that did not.
  bool leverHeadingCapped() const { return lever_capped_; }
  // Decaying integral of the horizontal specific force (m/s) and whether it is above
  // lever_release_dv (the consider states are then released).
  double releaseAccumulator() const { return dv_acc_; }
  bool leverReleaseOpen() const { return params_.lever_release_dv > 0.0 && dv_acc_ > params_.lever_release_dv; }
  const NominalState& state() const { return x_; }
  const Mat15& covariance() const { return P_; }
  // The covariance to publish for a filter with a GNSS antenna at `lever`: P, with the heading variance the bound has held back
  // put back (the shadow, headingSigmaHonest()) and the position of the body origin made as uncertain as that heading makes it
  // (up to the circle of radius |lever_xy| around the antenna: |lever_xy|^2 times the heading variance, at most 1, on each
  // horizontal axis). Equal to covariance() when nothing is held back.
  Mat15 covarianceHonest(const Eigen::Vector3d& lever) const;
 private:
  void inject(const Vec15& dx);
  NominalState x_;
  Mat15 P_{Mat15::Identity()};
  EskfParams params_;
  double rate_{0.0};   // norm of the bias-corrected angular rate of the last predict()
  double dv_acc_{0.0};       // see lever_release_dv; touches neither x nor P
  bool lever_capped_{false};
  bool lever_restored_{false}, lever_aligned_{false}, lever_blind_{false};
  // Shadow of the heading: covariance of (heading, vertical gyro bias), valid from the first
  // lever update after a reset; see EskfParams (the honest heading).
  bool shadow_valid_{false};
  bool shadow_held_{false};   // the bound has cut the heading variance since the shadow was last set from P
  double s_hh_{0.0}, s_hb_{0.0}, s_bb_{0.0};
  void syncShadow(const Eigen::Vector3d& up);
  void setHeadingVariance(const Eigen::Vector3d& up, double var);
  struct Span { double span, window, gap, history; };   // seconds, see alignSpans()
  Span alignSpans(double fix_period) const;
  void trimFixes();
  bool alignHeading(const Eigen::Vector3d& lever, const Eigen::Vector3d& up);
  void spreadAfterOutage();
  bool unknown_pending_{false};     // markHeadingUnknown() has marked the start and the fix rate is not yet known to be supported
  bool rate_unsupported_{false}, rate_degraded_{false};
  struct Fix { double t; Eigen::Vector2d s, p; double rate_max; Eigen::Vector2d d; };   // clock_, cum_a_, the antenna fix, the largest body rate since the previous fix and cum_p_, of one lever update
  std::deque<Fix> fixes_;
  double clock_{0.0};
  double rate_max_{0.0};
  Eigen::Vector2d cum_a_{Eigen::Vector2d::Zero()};   // sum of the world-frame horizontal acceleration since the start, see alignHeading()
  Eigen::Vector2d cum_p_{Eigen::Vector2d::Zero()};   // its integral over time (cum_a_ dt), see spreadAfterOutage()
};
}  // namespace prism_loc_fusion
