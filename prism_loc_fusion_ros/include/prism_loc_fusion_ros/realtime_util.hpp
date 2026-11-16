#pragma once
#include <cmath>
#include <Eigen/Geometry>
namespace prism_loc_fusion_ros {

// A scan is registered from the pose the filter had when the scan arrived (map_from_guess),
// while the IMU keeps moving the state. When the result z (pose of the body in the map, from
// the registration) is applied, the state is map_from_now. z belongs to the guess, so the
// motion since then is added to it, in the body frame of the guess:
//   z' = z * guess^-1 * now
// With now == guess this is z.
inline Eigen::Isometry3d compensateForMotion(const Eigen::Isometry3d& z,
                                             const Eigen::Isometry3d& map_from_guess,
                                             const Eigen::Isometry3d& map_from_now) {
  return z * map_from_guess.inverse() * map_from_now;
}

// True when a cloud stamped cloud_stamp_s is more than max_age_s older than the latest IMU
// sample (imu_stamp_s). False when max_age_s <= 0 (check off), when either stamp is 0 or not
// finite (a driver that sets no stamp, or no IMU sample yet), or when the cloud is newer.
inline bool cloudIsStale(double cloud_stamp_s, double imu_stamp_s, double max_age_s) {
  if (!(max_age_s > 0.0)) return false;
  if (!std::isfinite(cloud_stamp_s) || !std::isfinite(imu_stamp_s)) return false;
  if (cloud_stamp_s == 0.0 || imu_stamp_s == 0.0) return false;
  return imu_stamp_s - cloud_stamp_s > max_age_s;
}

}  // namespace prism_loc_fusion_ros
