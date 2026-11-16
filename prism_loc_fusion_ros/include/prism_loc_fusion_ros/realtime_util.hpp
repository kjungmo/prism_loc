#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
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

// The time a scan's registration stands for: the header stamp plus points_time_offset. A
// driver that stamps the start of a sweep and does not deskew gives a registration that fits
// the pose at about the middle of the sweep (+ half the sweep period). A zero stamp (a driver
// that sets none) stays 0, so the checks that treat 0 as unstamped still see it.
inline double cloudTime(double header_stamp_s, double offset_s) {
  if (header_stamp_s == 0.0) return 0.0;
  return header_stamp_s + offset_s;
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


// ---- Pose history -------------------------------------------------------------------------
// The node records, after every IMU prediction, the filter state and a "dead-reckoned" pose: the
// chain of the prediction increments only (advanceDeadReckoning). A scan is registered from the
// state at its own stamp (interpolatePose), and the motion since then is taken from the
// dead-reckoned pose, so that GNSS and lidar updates applied meanwhile do not count as motion.
struct PoseSample {
  double t{0.0};  // IMU stamp, s
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d dr_p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond dr_q{Eigen::Quaterniond::Identity()};
};
struct PoseAtTime {
  Eigen::Isometry3d state{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d dr{Eigen::Isometry3d::Identity()};
};

// Appends s. Drops samples older than s.t - keep_s. When s is not newer than the newest sample
// (a bag played again, a clock reset) the history starts over with s.
inline void recordPose(std::deque<PoseSample>& h, const PoseSample& s, double keep_s) {
  if (!h.empty() && !(s.t > h.back().t)) h.clear();
  h.push_back(s);
  while (h.size() > 1 && h.front().t < s.t - keep_s) h.pop_front();
}

// The state and the dead-reckoned pose at time t: exact at a sample, linear in the position and
// spherical in the attitude between two. False (out untouched) for an empty history or a t
// outside [oldest, newest].
inline bool interpolatePose(const std::deque<PoseSample>& h, double t, PoseAtTime& out) {
  if (h.empty() || !(t >= h.front().t) || !(t <= h.back().t)) return false;
  const auto hi = std::lower_bound(h.begin(), h.end(), t,
                                   [](const PoseSample& s, double v) { return s.t < v; });
  PoseSample a = *hi, b = *hi;
  if (hi != h.begin() && hi->t != t) a = *(hi - 1);
  const double span = b.t - a.t;
  const double w = span > 0.0 ? (t - a.t) / span : 0.0;
  auto lerp = [w](const Eigen::Vector3d& x, const Eigen::Vector3d& y) { return (1.0 - w) * x + w * y; };
  out.state = Eigen::Isometry3d::Identity();
  out.state.translation() = lerp(a.p, b.p);
  out.state.linear() = a.q.slerp(w, b.q).normalized().toRotationMatrix();
  out.dr = Eigen::Isometry3d::Identity();
  out.dr.translation() = lerp(a.dr_p, b.dr_p);
  out.dr.linear() = a.dr_q.slerp(w, b.dr_q).normalized().toRotationMatrix();
  return true;
}

// One IMU prediction moved the state from `before` to `after` (both in the map): chain its
// increment, in the body frame, onto the dead-reckoned pose. Whatever changed the state between
// two predictions (a GNSS or lidar update) is in `before` already and is not in the increment.
inline Eigen::Isometry3d advanceDeadReckoning(const Eigen::Isometry3d& dr_prev,
                                              const Eigen::Isometry3d& before,
                                              const Eigen::Isometry3d& after) {
  return dr_prev * before.inverse() * after;
}

// ---- Decisions ---------------------------------------------------------------------------
enum class NdtVerdict { kAccept, kNotConverged, kFitness, kInliers };

// First failing check wins: converged, fitness at most max_fitness, inlier share at least
// min_inlier_fraction. A NaN fails.
inline NdtVerdict ndtVerdict(bool converged, double fitness, double inlier_fraction,
                             double max_fitness, double min_inlier_fraction) {
  if (!converged) return NdtVerdict::kNotConverged;
  if (!(fitness <= max_fitness)) return NdtVerdict::kFitness;
  if (!(inlier_fraction >= min_inlier_fraction)) return NdtVerdict::kInliers;
  return NdtVerdict::kAccept;
}
inline const char* ndtVerdictName(NdtVerdict v) {
  switch (v) {
    case NdtVerdict::kAccept: return "accepted";
    case NdtVerdict::kNotConverged: return "not converged";
    case NdtVerdict::kFitness: return "fitness";
    case NdtVerdict::kInliers: return "inliers";
  }
  return "";
}

// A result registered from a state that has been replaced since (generation changed) or from a
// filter that is gone is not applied.
inline bool discardAlignResult(std::uint64_t generation_at_guess, std::uint64_t generation_now,
                               bool filter_initialized) {
  return generation_at_guess != generation_now || !filter_initialized;
}

// Clouds keep being dropped as stale although the align is fast (or none ran): the lidar clock is
// behind the IMU clock, the registration is not too slow. `align_ms` is the last align time.
inline bool clockOffsetSuspected(bool any_align, double align_ms, double max_age_s,
                                 long consecutive_drops) {
  if (consecutive_drops < 3) return false;
  return !any_align || align_ms < 0.5 * 1000.0 * max_age_s;
}

}  // namespace prism_loc_fusion_ros
