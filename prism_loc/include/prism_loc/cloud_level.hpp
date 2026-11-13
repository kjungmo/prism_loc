#pragma once
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

// points[i] = tiltRotation(roll, pitch) * points[i], in place. Zero roll and pitch leave
// the points untouched.
inline void levelCloud(double roll, double pitch, std::vector<Eigen::Vector3d>& points) {
  if (roll == 0.0 && pitch == 0.0) return;
  const Eigen::Matrix3d R = tiltRotation(roll, pitch);
  for (auto& p : points) p = R * p;
}

}  // namespace prism_loc
