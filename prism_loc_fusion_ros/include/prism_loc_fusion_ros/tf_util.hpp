#pragma once
#include <Eigen/Geometry>
namespace prism_loc_fusion_ros {
inline Eigen::Isometry3d computeMapToOdom(const Eigen::Isometry3d& map_base,
                                          const Eigen::Isometry3d& odom_base) {
  return map_base * odom_base.inverse();
}
}  // namespace prism_loc_fusion_ros
