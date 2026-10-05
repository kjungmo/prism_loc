#pragma once
#include <chrono>
#include <string>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>

namespace prism_loc {

// Looks up target <- source at `stamp`, waiting at most `timeout_s` of steady (wall) time
// for the transform to arrive. tf2_ros::Buffer's own timeout is measured on the node
// clock, which never advances under use_sim_time without /clock: the wait then never
// ends and the executor stalls with the node's mutex held. Polling the buffer here keeps
// the wait bounded whatever the ROS clock does, with the same tolerance semantics (a
// scan slightly ahead of the latest odometry still resolves once odometry catches up).
// Throws tf2::TransformException like tf2_ros::Buffer::lookupTransform.
inline geometry_msgs::msg::TransformStamped lookupTransformSteady(
    const tf2_ros::Buffer& buffer, const std::string& target, const std::string& source,
    const tf2::TimePoint& stamp, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(timeout_s));
  while (!buffer.canTransform(target, source, stamp, nullptr) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return buffer.lookupTransform(target, source, stamp);
}

}  // namespace prism_loc
