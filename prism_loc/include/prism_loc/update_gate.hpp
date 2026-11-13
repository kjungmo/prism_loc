#pragma once
#include <cmath>
#include <limits>
namespace prism_loc {

// When a scan or cloud triggers a filter update. ROS-free so it can be unit-tested and
// called from an offline harness; times are passed as doubles in seconds (message stamps,
// never the wall clock), so the same inputs always give the same answer.
//
// Rules, in this order:
//   1. forced (a reseed)                                   -> update, reason Forced
//   2. moved >= min_d  or  turned >= min_a                 -> update, reason Motion
//      (the original gate; with min_period_s = 0 it is the only one besides Forced)
//   3. min_period_s > 0, and the stamp time since the last update is finite and
//      >= min_period_s, and the robot moved at all (>= kStandingFraction of min_d or
//      of min_a)                                           -> update, reason Time
//   4. otherwise                                           -> no update
//
// "Moved at all" exists because scoring one view again adds no information: the motion
// model adds noise in proportion to motion, so with zero motion each correct + resample
// only narrows the particle set (see test_update_gate_filter). The rule: no timed update
// while the odometry motion SINCE THE LAST UPDATE is below kStandingFraction of both
// thresholds. That is not "the robot stands": odometry that drifts while it stands (a gyro
// bias, lidar-odometry jitter) or a creeping robot accumulates that motion and does trigger
// timed updates on a nearly unchanged view, which narrows the set without new information.
// A threshold of 0 makes the fraction 0, so every scan may update, as today.
enum class UpdateReason { None, Motion, Time, Forced };

struct UpdateGate {
  static constexpr double kStandingFraction = 0.05;

  double min_d{0.2};         // m: translation that triggers an update
  double min_a{0.2};         // rad: rotation that triggers an update
  double min_period_s{0.0};  // s: time-based trigger; 0 = off

  struct Decision {
    bool update{false};
    UpdateReason reason{UpdateReason::None};
  };

  // moved_distance (m) and turned_angle (rad, absolute) are measured since the last
  // update. seconds_since_last_update is the stamp time since the last update; pass NaN
  // when there was none yet (it never triggers a timed update, nor does a negative or
  // infinite value). A NaN motion never passes a comparison, so it never triggers either.
  Decision decide(bool forced, double moved_distance, double turned_angle,
                  double seconds_since_last_update) const {
    if (forced) return {true, UpdateReason::Forced};
    if (moved_distance >= min_d || turned_angle >= min_a) return {true, UpdateReason::Motion};
    if (min_period_s > 0.0 && std::isfinite(seconds_since_last_update) &&
        seconds_since_last_update >= min_period_s &&
        (moved_distance >= kStandingFraction * min_d ||
         turned_angle >= kStandingFraction * min_a))
      return {true, UpdateReason::Time};
    return {};
  }
};

// Stamp of the last update, and the elapsed time since it. A stamp that goes backwards (a
// looping bag, a sim-time reset) moves the stored stamp back to it, so the timed trigger
// restarts one period after the jump instead of waiting for the old time to come round.
// A repeated stamp (a stuck sim clock) gives elapsed 0. Non-finite stamps are ignored.
class UpdateStamp {
 public:
  // NaN until the first mark(), and for a non-finite `now_s`. Any backwards stamp
  // re-anchors, so a few ms of out-of-order jitter makes the next timed update that much
  // early.
  double elapsed(double now_s) {
    if (!have_ || !std::isfinite(now_s)) return std::numeric_limits<double>::quiet_NaN();
    if (now_s < last_s_) last_s_ = now_s;
    return now_s - last_s_;
  }
  void mark(double now_s) {
    if (!std::isfinite(now_s)) return;
    have_ = true;
    last_s_ = now_s;
  }
  double last() const { return last_s_; }

 private:
  bool have_{false};
  double last_s_{0.0};
};

}  // namespace prism_loc

