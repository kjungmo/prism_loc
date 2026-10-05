#pragma once
#include <Eigen/Geometry>
namespace prism_loc_fusion_ros {
inline Eigen::Isometry3d computeMapToOdom(const Eigen::Isometry3d& map_base,
                                          const Eigen::Isometry3d& odom_base) {
  return map_base * odom_base.inverse();
}
// nav_msgs/Odometry carries its twist in child_frame_id (the body frame); the ESKF keeps
// velocity in the world frame. q_wb rotates body vectors into the world frame.
inline Eigen::Vector3d bodyVelocity(const Eigen::Quaterniond& q_wb, const Eigen::Vector3d& v_w) {
  return q_wb.conjugate() * v_w;
}
}  // namespace prism_loc_fusion_ros
