#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
namespace prism_loc {

// ndt3d health for /diagnostics. ROS-free so the rules can be unit-tested.

// Scan-to-map match of the last few filter updates. After every update the node reports
// the share of the scan points that lie on a mapped surface at the published pose
// (prism_loc_core::NdtMatchStats::fraction) and warns while the mean of the last kWindow
// values is below min_match_fraction. The update forced by a reseed (/initialpose,
// set_initial_pose) is not judged, as for n_eff: it empties the window instead.
//   state                               low()    mean()
//   no update since the last reseed     false    0 (reported as n/a)
//   mean of the last <= 5 >= threshold  false    the mean
//   mean of the last <= 5 <  threshold  true     the mean
//   threshold 0                         false    (check off: the mean is never negative)
// A reseed also makes the pose "not verified": a wrong initial pose can match well for
// several updates before the match collapses (measured: OK for 8 to 14 updates in a row
// while 1.4 to 2.3 m off). verified(n) is false until n updates have been judged since
// the last reseed; the node reports WARN "pose not verified" meanwhile (unless the match
// is low, which is reported instead).
//   updates judged since the reseed   verified(n)
//   fewer than n                      false
//   n or more                         true
//   n = 0                             true (check off)
// The effective particle count is no health signal for ndt3d: with the NDT likelihood one
// particle takes all the weight at every update of a healthy run (n_eff 1 of 500), and
// when no scan point falls in a map cell all particles weigh the same (n_eff = N).
constexpr double kDefaultMinMatchFraction = 0.65;  // default of the min_match_fraction parameter
constexpr int kDefaultMatchVerifyUpdates = 15;     // default of the match_verify_updates parameter

// In-code default of min_neff_fraction: the laser2d check stays at 0.005; for ndt3d it
// is off (0), min_match_fraction being its check.
inline double defaultMinNeffFraction(const std::string& backend) {
  return backend == "ndt3d" ? 0.0 : 0.005;
}

class MatchMonitor {
 public:
  static constexpr std::size_t kWindow = 5;

  // One filter update. reseeded: the update was forced by a reseed; its fraction is not
  // judged and everything before it is forgotten.
  void update(bool reseeded, double fraction) {
    if (reseeded) {
      count_ = 0;
      next_ = 0;
      since_reseed_ = 0;
      return;
    }
    values_[next_] = fraction;
    next_ = (next_ + 1) % kWindow;
    if (count_ < kWindow) ++count_;
    if (since_reseed_ < kMaxCounted) ++since_reseed_;
  }
  // Updates judged since the last reseed (saturates at kMaxCounted).
  std::size_t sinceReseed() const { return since_reseed_; }
  bool verified(std::size_t needed) const { return since_reseed_ >= needed; }
  static constexpr std::size_t kMaxCounted = 1000000;
  std::size_t size() const { return count_; }
  double mean() const {
    if (count_ == 0) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 0; i < count_; ++i) sum += values_[i];
    return sum / static_cast<double>(count_);
  }
  bool low(double threshold) const { return count_ > 0 && mean() < threshold; }

 private:
  std::array<double, kWindow> values_{};
  std::size_t count_{0}, next_{0}, since_reseed_{0};
};

// ndt3d is planar: of the transform base_frame -> cloud frame it uses x, y and yaw only.
// The sensor is placed level, at height base_height in the map frame; the transform's z,
// roll and pitch are ignored. This reports which ignored parts are not negligible: a tilt
// over angle_tol (always a mismatch: the node warns), or a z that differs from base_height
// by more than height_tol. The latter is not an error by itself: base_height is a height
// in the map frame and z one above base_frame, so they differ on every robot whose
// base_frame does not ride at map z = 0 (the node only informs). A value that is not
// finite counts as not negligible.
struct IgnoredSensorTf {
  bool tilt{false};
  bool height{false};
  bool any() const { return tilt || height; }
};
constexpr double kSensorTiltTol = 0.01;    // rad
constexpr double kSensorHeightTol = 0.05;  // m
inline IgnoredSensorTf ignoredSensorTf(double roll, double pitch, double z, double base_height,
                                       double angle_tol = kSensorTiltTol,
                                       double height_tol = kSensorHeightTol) {
  IgnoredSensorTf r;
  r.tilt = !(std::fabs(roll) <= angle_tol && std::fabs(pitch) <= angle_tol);
  r.height = !(std::fabs(z - base_height) <= height_tol);
  return r;
}

}  // namespace prism_loc
