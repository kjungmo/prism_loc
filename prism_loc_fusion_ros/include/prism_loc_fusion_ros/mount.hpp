#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <Eigen/Geometry>
#include "prism_loc_fusion_ros/extrinsic.hpp"
namespace prism_loc_fusion_ros {

// Looks up base_frame <- frame, latest available. Returns false and fills `error` when
// the transform is not available. ROS-free so that Mount can be unit-tested; the node
// passes a lambda around tf2_ros::Buffer::lookupTransform(base_frame, frame, TimePointZero).
using MountLookup = std::function<bool(const std::string& frame, Eigen::Isometry3d& base_from_sensor,
                                       std::string& error)>;

// Which mount (base_frame <- sensor frame) applies to the messages of one sensor.
//
// The sensor frame is override_frame when set, else the message's header.frame_id; one
// leading "/" is dropped (tf2 has no such frames, older drivers still send them).
//   frame                         result of resolve()   describe()
//   empty                         false (not used)      missing (empty frame_id)
//   base_frame                    true, identity        identity (frame <base>)
//   other, transform found        true, that transform  <base> -> <frame> xyz ... rpy ...
//   other, never found            false (not used)      missing (frame <frame>)
//   other, found, then lost       true, the one found   <base> -> <frame> xyz ... rpy ...
// A transform that is not finite counts as not found. A found transform is used for
// kRefreshS before it is looked up again (a mount is rigid: the latest transform is
// taken and nothing is waited for), and is kept when a later lookup fails, because tf2
// empties its buffer when time jumps back. A different frame starts from scratch.
class Mount {
 public:
  static constexpr double kRefreshS = 1.0;
  std::string override_frame;  // points_frame / imu_frame / gnss_frame parameter

  // True when the message may be used, with ext() as its mount. steady_s is a steady
  // clock in seconds. *changed is set when a transform from TF is new or differs from the
  // one in use (for a log line).
  bool resolve(const std::string& msg_frame, const std::string& base_frame, double steady_s,
               const MountLookup& lookup, bool* changed = nullptr) {
    if (changed) *changed = false;
    seen_ = true;
    const std::string frame = unslashed(override_frame.empty() ? msg_frame : override_frame);
    if (frame.empty() || frame == unslashed(base_frame)) {
      frame_ = frame;
      ext_ = Extrinsic{};
      source_ = frame.empty() ? Source::kNone : Source::kBaseFrame;  // base_frame: no TF, nothing applied
      error_.clear();
      return source_ != Source::kNone;
    }
    const bool same = source_ == Source::kTf && frame_ == frame;
    if (same && steady_s - looked_up_s_ < kRefreshS) return true;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    std::string error;
    bool ok = lookup(frame, T, error);
    if (ok && !T.matrix().allFinite()) {
      ok = false;
      error = "the transform is not finite";
    }
    if (!ok) {
      if (same) {  // keep the mount already found
        looked_up_s_ = steady_s;
        return true;
      }
      frame_ = frame;
      source_ = Source::kNone;
      error_ = error;
      return false;
    }
    if (changed) *changed = !same || (T.matrix().array() != ext_.T.matrix().array()).any();
    frame_ = frame;
    ext_ = makeExtrinsic(T);
    source_ = Source::kTf;
    error_.clear();
    looked_up_s_ = steady_s;
    return true;
  }

  bool seen() const { return seen_; }          // resolve() has been called
  bool resolved() const { return source_ != Source::kNone; }  // the last message could be used
  const Extrinsic& ext() const { return ext_; }
  const std::string& frame() const { return frame_; }

  // What is in use, for /diagnostics and the log.
  std::string describe(const std::string& base_frame) const {
    if (!seen_) return "no message yet";
    if (source_ == Source::kNone)
      return frame_.empty() ? std::string("missing (empty frame_id)") : "missing (frame " + frame_ + ")";
    if (source_ == Source::kBaseFrame) return "identity (frame " + frame_ + ")";
    const Eigen::Vector3d& p = ext_.T.translation();
    const Eigen::Matrix3d& R = ext_.T.linear();
    // Fixed-axis roll, pitch, yaw as tf2 prints them; rounded first so -0.000 never shows.
    auto r3 = [](double v) { return std::round(v * 1000.0) / 1000.0 + 0.0; };
    char b[160];
    std::snprintf(b, sizeof(b), " xyz %.3f %.3f %.3f rpy %.3f %.3f %.3f", r3(p.x()), r3(p.y()), r3(p.z()),
                  r3(std::atan2(R(2, 1), R(2, 2))), r3(std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))))),
                  r3(std::atan2(R(1, 0), R(0, 0))));
    return base_frame + " -> " + frame_ + b;
  }

  // Why the last message was not used ("" when it was), naming the topic it came from and
  // the parameter that overrides its frame.
  std::string problem(const std::string& base_frame, const std::string& topic,
                      const std::string& param) const {
    if (!seen_ || source_ != Source::kNone) return "";
    if (frame_.empty())
      return "empty header.frame_id on " + topic + ": set " + param + " to the sensor's frame (to '" +
             base_frame + "' if the data is already expressed in it)";
    // tf2's reason on one line, bounded (it can carry a source location on a second line)
    std::string why = error_.substr(0, error_.find('\n'));
    if (why.size() > kMaxReason) why = why.substr(0, kMaxReason) + "...";
    return "no transform " + base_frame + " -> " + frame_ + " (frame of " + topic +
           "): publish it on TF, or set " + param + " [" + why + "]";
  }
  static constexpr std::size_t kMaxReason = 240;

 private:
  static std::string unslashed(const std::string& f) {
    return !f.empty() && f[0] == '/' ? f.substr(1) : f;
  }
  // Where ext_ comes from: nowhere (the last message is not usable), the frame being
  // base_frame itself, or a transform found on TF.
  enum class Source { kNone, kBaseFrame, kTf };
  std::string frame_;  // sensor frame of the last message
  Extrinsic ext_;      // valid unless source_ is kNone
  Source source_{Source::kNone};
  bool seen_{false};
  std::string error_;        // lookup error of the last unresolved message
  double looked_up_s_{0.0};  // steady time of the last lookup of frame_
};

}  // namespace prism_loc_fusion_ros
