#pragma once
#include <Eigen/Geometry>
namespace prism_loc_fusion_ros {

// Mount of one sensor on the robot: T is base_frame <- sensor frame (a point p_s in the
// sensor frame is T * p_s in base_frame), as tf2 returns for
// lookupTransform(base_frame, sensor_frame).
//
// identity is true when the sensor frame IS base_frame, or its transform is exactly the
// identity matrix. Every function below then returns its input untouched (no arithmetic),
// so a robot whose data is already expressed in base_frame gets bit-identical values.
struct Extrinsic {
  bool identity{true};
  Eigen::Isometry3d T{Eigen::Isometry3d::Identity()};
};

inline Extrinsic makeExtrinsic(const Eigen::Isometry3d& base_from_sensor) {
  Extrinsic e;
  e.identity = (base_from_sensor.matrix().array() == Eigen::Matrix4d::Identity().array()).all();
  e.T = base_from_sensor;
  return e;
}

// LiDAR. Registration aligns a cloud expressed in the sensor frame, so it starts from and
// returns the pose of the sensor: T_map_sensor = T_map_base * T_base_sensor.
inline Eigen::Isometry3d sensorPoseFromBasePose(const Extrinsic& e,
                                                const Eigen::Isometry3d& map_from_base) {
  if (e.identity) return map_from_base;
  return map_from_base * e.T;
}
// T_map_base = T_map_sensor * inverse(T_base_sensor).
inline Eigen::Isometry3d basePoseFromSensorPose(const Extrinsic& e,
                                                const Eigen::Isometry3d& map_from_sensor) {
  if (e.identity) return map_from_sensor;
  return map_from_sensor * e.T.inverse();
}

// IMU. Angular velocity and specific force measured in the IMU frame, expressed in
// base_frame: only the rotation of the mount is applied. The translation (lever arm) is
// NOT compensated: the specific force is that at the IMU, not at the base_frame origin.
inline Eigen::Vector3d vectorInBase(const Extrinsic& e, const Eigen::Vector3d& v_sensor) {
  if (e.identity) return v_sensor;
  return e.T.linear() * v_sensor;
}

// GNSS. The fix is the position of the antenna; the antenna sits at e.T.translation() in
// base_frame, so p_base = p_antenna - R_map_base * t_base_antenna. Used to seed the
// filter's position from the first fix (later fixes are fused at the lever arm, see
// Eskf::updatePositionLever). The antenna frame's rotation is not used.
inline Eigen::Vector3d basePositionFromAntenna(const Extrinsic& e,
                                               const Eigen::Quaterniond& q_map_base,
                                               const Eigen::Vector3d& p_antenna) {
  if (e.identity) return p_antenna;
  return p_antenna - q_map_base * e.T.translation();
}

}  // namespace prism_loc_fusion_ros
