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

// Looks up target <- source at `stamp`, polling the buffer every 10 ms (as tf2_ros does)
// until the transform resolves or the rule says to stop: 0.1 s of ROS time (plus up to
// 30 ms when the clock moved 30 ms or more in the poll that reaches the budget), capped
// at 1 s of steady time; see TfWaitRule / TfWait for the limits. Pass the same rule object every time:
// it carries the "last changed" state of the ROS clock across calls. tf2_ros::Buffer's
// own timeout loops on the ROS clock alone and never ends under use_sim_time without
// /clock. Throws tf2::TransformException like tf2_ros::Buffer::lookupTransform.
inline geometry_msgs::msg::TransformStamped lookupTransformWait(
    const tf2_ros::Buffer& buffer, const std::string& target, const std::string& source,
    const tf2::TimePoint& stamp, rclcpp::Clock& ros_clock, TfWaitRule& rule) {
  auto steady_now = [] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  TfWait wait(rule, steady_now(), ros_clock.now().nanoseconds());
  while (!buffer.canTransform(target, source, stamp, nullptr)) {
    if (wait.stop(steady_now(), ros_clock.now().nanoseconds())) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return buffer.lookupTransform(target, source, stamp);
}

}  // namespace prism_loc
