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
  // While the body is moving (the filter's speed above lever_motion_speed, m/s, and clear of
  // 3 times its own uncertainty, or its body rate above lever_motion_rate, rad/s) the mask is
  // lever_consider_moving. A threshold of 0 switches that test off; both 0 switch the gate off.
  // Defaults (chosen in experiments/eskf_lever.cpp, README study 6): standing or turning on
  // the spot, the heading and the vertical gyro bias are left out, because one antenna does not
  // separate them from the position of the body; travelling faster than 0.6 m/s nothing is
  // left out, because an antenna that travels does show them (lower thresholds, 0.3 and 0.15 m/s,
  // diverged more often on a standing robot). The body rate test is off: with
  // it the full update ran during spins on the spot, where it was over-confident.
  // Roll, pitch, the horizontal gyro biases, position, velocity and the accelerometer bias are
  // always corrected. The node does not set these.
  unsigned lever_consider{kConsiderHeading | kConsiderGyroBiasVertical};
  double lever_motion_speed{0.6};
  double lever_motion_rate{0.0};
  unsigned lever_consider_moving{0u};
  // Gauss-Newton iterations of the lever-arm update (1 = the ordinary one-step update;
  // more is the iterated update of Bell and Cathey 1993, relinearised at each pass).
  int lever_iterations{1};
};
}  // namespace prism_loc_fusion
