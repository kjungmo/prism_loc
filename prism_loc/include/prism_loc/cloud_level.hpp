#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc {

// Level a cloud with the roll and pitch of the static base -> sensor rotation (the node's
// use_sensor_tilt option). ROS-free.
//
// The model (prism_loc_core::Ndt3DModel) treats the sensor frame as level: it places the
// cloud with the planar pose (x, y, yaw) of base -> sensor. A tilted mount has
//   R_base_sensor = Rz(yaw) * R_tilt,   R_tilt = Ry(pitch) * Rx(roll)
// (tf2 getRPY convention: R = Rz(yaw) Ry(pitch) Rx(roll)). So the model needs
//   points_level = R_tilt * points_sensor,
// and then Rz(yaw) (applied by the model through sensor_in_base) completes the full rotation.
// The translation of the transform is not used here: x, y go through sensor_in_base and the
// height comes from base_height.
// This is a static mount correction. The roll and pitch of the robot body on a slope are not
// part of it.
inline Eigen::Matrix3d tiltRotation(double roll, double pitch) {
  return (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// The same tilt from the full rotation and the yaw the model actually uses (the node takes
// that yaw from the quaternion with atan2(2(wz + xy), 1 - 2(y^2 + z^2)), not from getRPY):
//   R_tilt = Rz(-yaw) * R_base_sensor,
// so that Rz(yaw) * R_tilt == R_base_sensor exactly, whatever the roll/pitch convention and
// even at gimbal lock where the decomposition is not unique.
inline Eigen::Matrix3d tiltFromRotation(const Eigen::Matrix3d& R_base_sensor, double yaw) {
  return Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_base_sensor;
}

// Rotation matrix of a quaternion (x, y, z, w), normalised first; the identity when the
// quaternion is zero or not finite.
inline Eigen::Matrix3d rotationFromQuaternion(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  const double n = q.norm();
  if (!(n > 0.0) || !std::isfinite(n)) return Eigen::Matrix3d::Identity();
  q.coeffs() /= n;
  return q.toRotationMatrix();
}

// points[i] = R_tilt * points[i], in place (R_tilt from tiltFromRotation).
inline void levelCloud(const Eigen::Matrix3d& R_tilt, std::vector<Eigen::Vector3d>& points) {
  for (auto& p : points) p = R_tilt * p;
}

// points[i] = tiltRotation(roll, pitch) * points[i], in place. Zero roll and pitch leave
// the points untouched.
inline void levelCloud(double roll, double pitch, std::vector<Eigen::Vector3d>& points) {
  if (roll == 0.0 && pitch == 0.0) return;
  const Eigen::Matrix3d R = tiltRotation(roll, pitch);
  for (auto& p : points) p = R * p;
}

}  // namespace prism_loc
