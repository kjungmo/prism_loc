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
  // The alignment refuses a span of fixes with a gap (lever_align_gap) or a window longer than lever_align_window + 1 s anywhere between
  // the first fix of the older velocity and now. false is the version before (fifth review, finding 1), kept for the scheme S8o of
  // experiments/eskf_lever.cpp, which shows what it did: after an outage across a start the alignment compared the IMU sum of the outage with
  // the velocity from before it.
  bool lever_align_span_guard{true};
  // A start with a lever arm whose heading has no source (initial_yaw only: no /initialpose, no lidar): the heading is a guess, and
  // the filter's start value of 0.71 rad of spread is far too narrow for it. markHeadingUnknown() then sets the spread the shadow
  // holds to this value (rad), over lever_align_sigma, so that the first start or stop finds the heading by the kinematic alignment
  // and a robot that cannot be aligned runs the blind update, instead of a full update with a heading 1 to 3 rad off (README, study 6,
  // fifth pass). The default is the spread of a heading that is uniform on the circle, pi / sqrt(3). 0 = off (the version before).
  double lever_start_sigma{1.8137993642342178};
  // The alignment measures time, not fixes (round 7, finding 1: with a span of 4 fixes it worked only at 5 to just under 10 Hz).
  //  - lever_align_span: the antenna velocity is taken over at least this many seconds and at least 2 fixes (the span of 4 fixes at 5 Hz,
  //    0.8 s, is what the earlier rounds had). The measured fix period T (the median of the last 15 intervals between fixes) scales the
  //    rest: the span is max(lever_align_span, 2 T), the window max(lever_align_window, 1.5 span), a gap is an interval over
  //    max(lever_align_gap, 2.5 T). 0 = the span of 4 fixes and the limits of the earlier rounds (the scheme S8k of experiments/eskf_lever.cpp).
  //  - lever_align_norm: a rotation about the vertical keeps the norm, so the change of velocity from the IMU (a) and the one from the fixes (v)
  //    could be required to have about the same norm: the alignment is refused if ||a| - |v|| > lever_align_norm max(|a|, |v|). OFF by default (0): the
  //    sixth review proposed it with 0.3, and it was measured (experiments/eskf_lever.cpp, schemes S8n, S8v; README, "Sixth pass"): with the spans in seconds it
  //    removes no alignment of the fix-rate suite that the spans do not, and it makes the node scenario gnss_gap_across_start fail more often (13 of 300 seeds of
  //    experiments/eskf_gap_across_start.cpp with the outage spread at 1 against 4 without the test, 4 against 1 with the spread at 0.1, 2 against 0 with no spread):
  //    a refused marginal alignment is followed by a later one on a window that the tilt and bias corrections after the outage have polluted. 0.3 gives the reviewer's test.
  //  - lever_align_period_max: the slowest fix rate the alignment supports, as a fix period in s (1.25 s: 0.8 Hz). If the measured period is
  //    longer, the alignment does nothing, and markHeadingUnknown() is taken back after the first fixes (the start spread is that of the
  //    filter's own covariance, the version before round 6) and leverUnsupportedRate() is true. 0 = no limit.
  double lever_align_span{0.8};
  double lever_align_norm{0.0};
  double lever_align_period_max{1.25};
  // After an outage of the fixes (an interval over the gap limit) with the heading unknown (the shadow of the heading over lever_align_sigma and the alignment
  // not possible), the velocity and position the filter dead-reckoned through the outage are as uncertain as the heading is: a velocity change dS (the IMU sum
  // over the outage, in the filter's heading) turned by the heading error is off by up to 2 |dS|. P holds the heading bounded (lever_heading_sigma_max), so it
  // said much less, and the first fixes after the outage corrected the velocity far too slowly (round 7, finding 2: a robot that starts during a short outage
  // had a position error of 3 m for 20 s, and the mast diverged). With this on, the first lever update after such an outage adds to the horizontal position and
  // velocity covariance s min(heading spread^2, 2) / 2 times [[|D|^2, D.dS], [D.dS, |dS|^2]] on each axis, with dS the velocity change the IMU summed over the outage, D
  // the position it gave (Eskf::spreadAfterOutage), and s this factor. 0 = off, the version before (the scheme S8w of experiments/eskf_lever.cpp). The factor is a
  // tenth, not 1: the full variance gave the same gain in the short outages (hgap) but made the mast worse in the long ones and the node scenario
  // gnss_gap_across_start fail in 4 of 300 seeds of the core replica (1 of 300 at a tenth, 0 without the spread); 0.02 to 1 were measured (README, "Sixth pass") and a
  // tenth is where the gain in the short outages is complete.
  double lever_outage_spread{0.1};
};
}  // namespace prism_loc_fusion
