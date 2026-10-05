#pragma once
#include <cmath>
#include <cstdint>
namespace prism_loc {

// When to stop waiting for a TF lookup at a message stamp. ROS-free so it can be
// unit-tested; steady time is passed in seconds and ROS time in integer nanoseconds.
//
// The rule: 0.1 s of ROS time (plus up to 30 ms when a stepped clock reaches the budget
// in one jump), capped at 1 s of steady time. Limits: a /clock that changes less often
// than once per second of wall time counts as frozen and is not waited on; below 0.1x
// the 1 s cap ends the wait before 0.1 s of ROS time; the callback in which a clock
// stops can hold up to the cap. "Frozen" is judged across waits (TfWaitRule remembers
// when the ROS clock last changed), so a coarse /clock (e.g. 10 Hz of sim time at 0.5x,
// one step per 0.2 s of wall time) still gets its full budget.
//
// Step grace: a coarse clock usually reaches the budget in one step, published in the
// same instant as the transform stamped at that step, which may still be in flight on
// /tf. When the clock moved by step_min_s (30 ms) or more between two polls, the lookup
// is retried for step_grace_s (30 ms) more of steady time. The node polls every 10 ms
// (as tf2_ros does), so 30 ms gives at least two more polls even with scheduling
// jitter. A live clock or smooth playback below 3x moves about 10 ms or less per poll
// and stops at the budget, so a live robot behaves exactly as with tf2_ros.

// Parameters plus the "when did the ROS clock last change" state shared by all waits.
class TfWaitRule {
 public:
  double budget_s{0.1};        // ROS time to wait for a late transform
  double frozen_after_s{1.0};  // clock unchanged this long (steady): frozen, do not wait
  double hard_cap_s{1.0};      // steady time no wait exceeds
  double step_min_s{0.03};     // ROS time moved between two polls that counts as a step
  double step_grace_s{0.03};   // extra steady time after a stepped clock hits the budget

  void observe(double steady_s, std::int64_t ros_ns) {
    if (!seen_ || ros_ns != last_ros_ns_) {
      seen_ = true;
      last_ros_ns_ = ros_ns;
      changed_steady_s_ = steady_s;
    }
  }
  // True when the ROS clock has not changed for frozen_after_s of steady time.
  bool frozen(double steady_s) const {
    return seen_ && steady_s - changed_steady_s_ >= frozen_after_s;
  }

 private:
  bool seen_{false};
  std::int64_t last_ros_ns_{0};
  double changed_steady_s_{0.0};
};

// One wait. stop() is called after each failed lookup with the current steady and ROS
// time; it also feeds the rule's clock state.
class TfWait {
 public:
  TfWait(TfWaitRule& rule, double steady_s, std::int64_t ros_ns)
      : rule_(rule), steady0_(steady_s), ros0_ns_(ros_ns), last_ros_ns_(ros_ns) {
    rule_.observe(steady_s, ros_ns);
  }

  bool stop(double steady_s, std::int64_t ros_ns) {
    rule_.observe(steady_s, ros_ns);
    if (ros_ns < last_ros_ns_) return true;  // clock jumped backward (a looping bag)
    const std::int64_t moved = ros_ns - last_ros_ns_;
    last_ros_ns_ = ros_ns;
    if (steady_s - steady0_ >= rule_.hard_cap_s) return true;
    if (ros_ns - ros0_ns_ >= ns(rule_.budget_s)) {
      if (grace_from_ < 0.0) {
        if (moved < ns(rule_.step_min_s)) return true;  // smooth clock: stop at the budget
        grace_from_ = steady_s;                          // stepped past it: brief grace
      }
      return steady_s - grace_from_ >= rule_.step_grace_s;
    }
    return rule_.frozen(steady_s);
  }

 private:
  static std::int64_t ns(double s) { return static_cast<std::int64_t>(std::llround(s * 1e9)); }
  TfWaitRule& rule_;
  double steady0_;
  std::int64_t ros0_ns_, last_ros_ns_;
  double grace_from_{-1.0};
};

}  // namespace prism_loc
