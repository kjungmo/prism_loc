#pragma once
namespace prism_loc {

// When to stop waiting for a TF lookup at a message stamp: 0.1 s of ROS time, capped at
// 1 s of steady time; a clock unchanged for 1 s is treated as frozen and no longer
// waited on. "Frozen" is decided across waits, not within one: the rule remembers the
// steady time at which the ROS clock was last seen to change, so a /clock that ticks
// coarsely (e.g. 10 Hz of sim time at 0.5x, one step per 0.2 s of wall time) still gets
// its full 0.1 s of ROS time. A clock that has just stopped can hold one wait for up to
// the 1 s cap before it is recognised as frozen; after that waits return at once. A
// clock that jumps backward (a looping bag) ends the wait. On a live robot (ROS time =
// wall time) this is a plain 0.1 s wait. ROS-free so it can be unit-tested.
class TfWaitRule {
 public:
  double budget_s{0.1};
  double frozen_after_s{1.0};
  double hard_cap_s{1.0};

  // Record the ROS clock (seconds) as read at steady time steady_s (seconds, any epoch).
  void observe(double steady_s, double ros_s) {
    if (!seen_ || ros_s != last_ros_s_) {
      last_ros_s_ = ros_s;
      last_change_steady_s_ = steady_s;
      seen_ = true;
    }
  }

  // True when the ROS clock has not changed for frozen_after_s of steady time.
  bool frozen(double steady_s) const {
    return seen_ && steady_s - last_change_steady_s_ >= frozen_after_s;
  }

  // steady_elapsed_s / ros_elapsed_s: time since this wait began on each clock (take
  // ros_elapsed_s from an integer-nanosecond difference, as tf2_ros does, so a clock
  // that steps by exactly the budget is compared exactly);
  // steady_now_s: the current steady time (same epoch as observe()).
  bool stop(double steady_elapsed_s, double ros_elapsed_s, double steady_now_s) const {
    if (ros_elapsed_s >= budget_s) return true;
    if (ros_elapsed_s < 0.0) return true;  // clock jumped backward
    if (frozen(steady_now_s)) return true;
    return steady_elapsed_s >= hard_cap_s;
  }

 private:
  bool seen_{false};
  double last_ros_s_{0.0};
  double last_change_steady_s_{0.0};
};

}  // namespace prism_loc
