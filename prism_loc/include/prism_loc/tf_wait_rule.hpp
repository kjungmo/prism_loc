#pragma once
namespace prism_loc {

// When to stop waiting for a TF lookup at a message stamp. The budget is ROS time, so a
// scan slightly ahead of its odometry resolves the same way at playback rates down to
// 0.1x; below that the 1 s steady cap shortens it. Two bounds keep a bad clock from
// holding the callback (and the node's mutex): a ROS clock that has not moved at all
// after frozen_after_s of steady time (use_sim_time without /clock) gives up, and
// nothing waits longer than hard_cap_s of steady time (a clock that started, then
// froze). On a live robot (ROS time = wall time) this is a plain budget_s wait.
// ROS-free so it can be unit-tested.
struct TfWaitRule {
  double budget_s{0.1};
  double frozen_after_s{0.1};
  double hard_cap_s{1.0};

  // steady_elapsed_s: steady time since the wait began; ros_elapsed_s: ROS clock now
  // minus the ROS clock when it began (<= 0 means it has not advanced).
  bool stop(double steady_elapsed_s, double ros_elapsed_s) const {
    if (ros_elapsed_s >= budget_s) return true;
    if (ros_elapsed_s <= 0.0 && steady_elapsed_s >= frozen_after_s) return true;
    return steady_elapsed_s >= hard_cap_s;
  }
};

}  // namespace prism_loc
