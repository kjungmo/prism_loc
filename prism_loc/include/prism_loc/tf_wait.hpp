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
// or `rule` says to stop (see TfWaitRule: 0.1 s of ROS time, bounded on the steady clock
// so a frozen or stalled ROS clock cannot hang the node). tf2_ros::Buffer's own timeout
// loops on the ROS clock alone and never ends under use_sim_time without /clock.
// Throws tf2::TransformException like tf2_ros::Buffer::lookupTransform.
inline geometry_msgs::msg::TransformStamped lookupTransformWait(
    const tf2_ros::Buffer& buffer, const std::string& target, const std::string& source,
    const tf2::TimePoint& stamp, rclcpp::Clock& ros_clock, const TfWaitRule& rule = {}) {
  const auto steady0 = std::chrono::steady_clock::now();
  const rclcpp::Time ros0 = ros_clock.now();
  while (!buffer.canTransform(target, source, stamp, nullptr)) {
    const double steady_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - steady0).count();
    if (rule.stop(steady_s, (ros_clock.now() - ros0).seconds())) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return buffer.lookupTransform(target, source, stamp);
}

}  // namespace prism_loc
