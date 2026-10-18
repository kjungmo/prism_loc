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
  kConsiderTilt = 2u,               // attitude error about the two horizontal axes
  kConsiderGyroBiasVertical = 4u,   // gyro bias component along the vertical
  kConsiderGyroBiasHorizontal = 8u, // gyro bias components across the vertical
  kConsiderAttitudeAndGyroBias = 15u,
};
struct EskfParams {
  double sigma_acc{1e-2};
  double sigma_gyro{1e-3};
  double sigma_acc_bias{1e-4};
  double sigma_gyro_bias{1e-5};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.81)};
  // Apply the ESKF reset Jacobian G = blkdiag(I, I, I - [dtheta/2]x, I, I) to P
  // after each error injection (Sola 2017, Sec. 7.2). Off = G = I (v0.1 behaviour).
  bool reset_jacobian{false};
  // Lever-arm update (Eskf::updatePositionLever), a GNSS antenna off the body origin.
  // lever_consider: the error states the update does NOT correct but only counts in its
  // innovation covariance (Schmidt-Kalman "consider" states), a bit mask of LeverConsider.
  // Default: the heading and the vertical gyro bias, because one antenna with an IMU does not
  // observe them while the robot stands, turns on the spot or travels without accelerating
  // (README, "GNSS antenna off base_frame"). Roll, pitch, the horizontal gyro biases, position,
  // velocity and the accelerometer bias are always corrected. The node does not set these.
  //
  // lever_consider_moving is the mask used instead while the robot is observably moving:
  // (a) lever_release_dv below, the default way; (b) a speed gate (the filter's speed above
  // lever_motion_speed, m/s, and clear of 3 times its own uncertainty) or a body-rate gate
  // (lever_motion_rate, rad/s), both off by default (0): the speed gate of the previous
  // version, kept for experiments/eskf_lever.cpp (scheme S6vb). A threshold of 0 switches that
  // test off.
  unsigned lever_consider{kConsiderHeading | kConsiderGyroBiasVertical};
  double lever_motion_speed{0.0};
  double lever_motion_rate{0.0};
  unsigned lever_consider_moving{0u};
  // Gauss-Newton iterations of the lever-arm update (1 = the ordinary one-step update;
  // more is the iterated update of Bell and Cathey 1993, relinearised at each pass).
  int lever_iterations{1};
  // Bound on the heading standard deviation (rad) that a lever-arm update sees, 0 = off.
  // When sqrt(u^T P_theta u), u = R^T z, is over the bound, the heading error variable is
  // scaled by c = bound / sigma before the update (P <- D P D^T, D = I + (c - 1) u u^T on the
  // attitude block): the heading variance by c^2, every heading cross-covariance by c, the
  // tilt untouched, P stays positive. Without it the heading variance of a robot that stands
  // grows as (vertical gyro bias sigma x t)^2 without bound, the lever term -R[t]x leaves its
  // linear range and the tilt and height diverge (README, study 6).
  double lever_heading_sigma_max{0.35};
  // Release of the consider states (lever_consider_moving instead of lever_consider) while
  // the IMU measures horizontal acceleration: a decaying integral of the horizontal part of
  // the world-frame specific force, dv = dv exp(-dt / lever_release_tau) + |a_xy| dt (m/s),
  // above lever_release_dv. A single antenna with an IMU observes the heading only then
  // (a turn at speed, a start, a stop); 0 = off. Independent of the speed estimate.
  double lever_release_dv{1.0};
  double lever_release_tau{4.0};
  // Honest heading (README, "What one antenna and an IMU can observe", points 5 to 7; experiments/eskf_lever.cpp, third pass).
  // The bound above hides the true heading variance from the update, and the vertical gyro bias keeps turning the true heading while
  // the robot stands: the error is about 1.1 rad after 600 s at 0.002 rad/s, far over the bound (0.35 rad). A full update at the
  // release that trusts the bounded sigma turned the heading the wrong way and tilted the filter. With lever_restore on:
  //  - the filter carries the spread it holds (the 2 x 2 covariance of heading and vertical gyro bias, never bounded): headingSigmaHonest();
  //  - while that spread (rad) is over lever_align_sigma and the bound has cut it, a lever update tries the kinematic alignment
  //    (Eskf::alignHeading): the heading is rotated by the angle between the change of velocity the IMU gave and the one the antenna
  //    fixes gave over the last lever_align_window s, if both are over lever_align_dv (m/s), the fixes have no gap over lever_align_gap
  //    (s) and the body rate stays under 0.5 lever_align_dv / |t_xy|; the heading gets the variance lever_heading_sigma_max^2 and the
  //    horizontal velocity lever_align_vel_std (m/s) of spread. 0 = never;
  //  - if the spread is under lever_align_sigma the release puts it back and the bound is not applied while the release is open;
  //  - lever_blind: if the spread is over lever_align_sigma and the alignment finds nothing, the update leaves the attitude columns
  //    out of the lever Jacobian (as a filter without a lever arm does) and counts the circle of the lever, |t_xy|^2 on each
  //    horizontal axis, in R: the antenna says where the circle is and tells roll and pitch nothing. The release stays closed.
  // lever_restore off is the behaviour of the version before, bit for bit (the other settings are then not used).
  bool lever_restore{true};
  double lever_align_sigma{1.0};
  double lever_align_dv{0.25};
  double lever_align_window{3.0};
  double lever_align_gap{1.0};
  bool lever_blind{true};
  double lever_align_vel_std{0.15};
};
}  // namespace prism_loc_fusion
