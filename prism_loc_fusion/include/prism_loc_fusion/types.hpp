#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_fusion {
struct NominalState {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
};
enum LeverConsider : unsigned {
  kConsiderHeading = 1u,            // attitude error about the vertical
  kConsiderGyroBiasVertical = 4u,   // gyro bias component along the vertical
};
// The default bound (rad) on the heading standard deviation that a lever update sees, and the variance an aligned heading gets
// when the bound is off.
constexpr double kLeverHeadingSigmaMax = 0.35;
struct EskfParams {
  double sigma_acc{1e-2};
  double sigma_gyro{1e-3};
  double sigma_acc_bias{1e-4};
  double sigma_gyro_bias{1e-5};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.81)};
  // Apply the ESKF reset Jacobian G = blkdiag(I, I, I - [dtheta/2]x, I, I) to P
  // after each error injection (Sola 2017, Sec. 7.2). Off = G = I (v0.1 behaviour).
  bool reset_jacobian{false};
  // Lever-arm update (Eskf::updatePositionLever), a GNSS antenna off the body origin. The rules are
  // in README.md ("GNSS antenna off base_frame"); the parameters are:
  //
  // lever_consider: the error states that the update counts in its innovation covariance but
  // does not correct (Schmidt-Kalman "consider" states), a mask of LeverConsider. One antenna and
  // an IMU do not observe the heading and the vertical gyro bias while the robot stands, turns on
  // the spot or travels without accelerating. While the release is open the update is the full
  // one (no consider state).
  unsigned lever_consider{kConsiderHeading | kConsiderGyroBiasVertical};
  // Bound (rad) on the heading standard deviation that one update sees; 0 = off. When
  // sqrt(u^T P_theta u), u = R^T z, is over the bound, the heading error variable is scaled by
  // c = bound / sigma (P <- D P D^T, D = I + (c - 1) u u^T on the attitude block): the heading
  // variance by c^2, every heading cross-covariance by c, the tilt untouched. Without it the
  // heading variance of a standing robot grows without limit and the lever term -R[t]x leaves
  // its linear range.
  double lever_heading_sigma_max{kLeverHeadingSigmaMax};
  // Release: while the decaying integral of the horizontal specific force in the world frame,
  // dv = dv exp(-dt / lever_release_tau) + |a_xy| dt (m/s), is over lever_release_dv, the update is
  // the full one and the bound is not applied. 0 = never.
  double lever_release_dv{1.0};
  double lever_release_tau{4.0};
  // Honest heading. The bound hides the heading spread, which keeps growing while the robot
  // stands. The filter keeps that spread (headingSigmaHonest()). While it is over lever_align_sigma
  // (rad) and the bound has cut it, an update first tries the kinematic alignment
  // (Eskf::alignHeading): the heading is rotated by the angle between the change of velocity that
  // the IMU gave and the one that the antenna fixes gave, if both are over lever_align_dv (m/s).
  // The aligned heading gets the variance lever_heading_sigma_max^2 and the horizontal velocity
  // lever_align_vel_std (m/s) of spread. If the alignment finds nothing, the update leaves the
  // attitude columns out of the lever Jacobian and counts the circle of the lever in R (blind).
  // lever_align_sigma 0 = no alignment, no blind update.
  double lever_align_sigma{1.0};
  double lever_align_dv{0.25};
  double lever_align_vel_std{0.15};
  // The alignment measures time. The antenna velocity is taken over max(lever_align_span, 2 T)
  // seconds and at least 2 fixes, T being the measured fix period (the median of the last 15
  // intervals). The window between the two velocities is max(lever_align_window, 1.5 span), a gap
  // is an interval over max(lever_align_gap, 2.5 T), and the alignment refuses a window with a
  // gap.
  double lever_align_span{0.8};
  double lever_align_window{3.0};
  double lever_align_gap{1.0};
  // Fix rate of the alignment, as a fix period in s. Up to lever_align_period_ok (1.05 s: 1 Hz with
  // 5 % for the jitter of the stamps) the alignment is supported. Between that and
  // lever_align_period_max (1.25 s: 0.8 Hz) it runs but finds the heading less often
  // (leverDegradedRate()). Over lever_align_period_max it does nothing, a start marked unknown is
  // taken back, and leverUnsupportedRate() is true. 0 = no limit (period_max) or no degraded band (period_ok).
  double lever_align_period_ok{1.05};
  double lever_align_period_max{1.25};
  // A start with a lever arm whose heading has no source (initial_yaw only) holds this spread
  // (rad) in the shadow, so that the first start or stop aligns the heading. Default pi / sqrt(3),
  // a heading uniform on the circle. 0 = off.
  double lever_start_sigma{1.8137993642342178};
  // After an outage of the fixes with the heading unknown, the velocity and position that the
  // filter dead-reckoned are as uncertain as the heading. The first update after the outage adds
  // s min(heading spread^2, 2) / 2 times [[|D|^2, D.dS], [D.dS, |dS|^2]] to the horizontal
  // position and velocity covariance (Eskf::spreadAfterOutage), dS and D being the velocity and
  // position that the IMU summed over the outage; s is this factor. 0 = off.
  double lever_outage_spread{0.1};
};
}  // namespace prism_loc_fusion
