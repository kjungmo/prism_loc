#pragma once
#include <chrono>
#include <string>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include "prism_loc/tf_wait_rule.hpp"

namespace prism_loc {

// Looks up target <- source at `stamp`, polling the buffer until the transform resolves
// or `rule` says to stop: 0.1 s of ROS time, capped at 1 s of steady time; a clock
// unchanged for 1 s is treated as frozen and no longer waited on (see TfWaitRule; the
// rule object carries the "last changed" state across calls, so pass the same one each
// time). tf2_ros::Buffer's own timeout loops on the ROS clock alone and never ends under
// use_sim_time without /clock.
// Throws tf2::TransformException like tf2_ros::Buffer::lookupTransform.
inline geometry_msgs::msg::TransformStamped lookupTransformWait(
    const tf2_ros::Buffer& buffer, const std::string& target, const std::string& source,
    const tf2::TimePoint& stamp, rclcpp::Clock& ros_clock, TfWaitRule& rule) {
  auto steady_now = [] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  const double steady0 = steady_now();
  const rclcpp::Time ros0 = ros_clock.now();
  rule.observe(steady0, ros0.seconds());
  while (!buffer.canTransform(target, source, stamp, nullptr)) {
    const double s = steady_now();
    const rclcpp::Time r = ros_clock.now();
    rule.observe(s, r.seconds());
    if (rule.stop(s - steady0, 1e-9 * static_cast<double>((r - ros0).nanoseconds()), s)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // tf2_ros polls every 10 ms
  }
  return buffer.lookupTransform(target, source, stamp);
}

}  // namespace prism_loc
