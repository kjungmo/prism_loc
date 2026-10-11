#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include "prism_loc_fusion_ros/mount.hpp"
using namespace prism_loc_fusion_ros;

namespace {
// A stand-in for TF: frames it knows, and how often it was asked.
struct FakeTf {
  std::map<std::string, Eigen::Isometry3d> frames;
  int calls{0};
  MountLookup lookup() {
    return [this](const std::string& frame, Eigen::Isometry3d& T, std::string& error) {
      ++calls;
      auto it = frames.find(frame);
      if (it == frames.end()) {
        error = "\"" + frame + "\" does not exist";
        return false;
      }
      T = it->second;
      return true;
    };
  }
};
Eigen::Isometry3d mountAt(double x, double y, double z, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}
const char* kBase = "base_link";
}  // namespace

TEST(Mount, BeforeAnyMessage) {
  Mount m;
  EXPECT_FALSE(m.seen());
  EXPECT_FALSE(m.resolved());
  EXPECT_EQ(m.describe(kBase), "no message yet");
  EXPECT_EQ(m.problem(kBase, "/points", "points_frame"), "");
}

// Data already in base_frame: usable, the identity, and TF is never asked.
TEST(Mount, BaseFrameIsTheIdentityAndNeedsNoTf) {
  FakeTf tf;
  Mount m;
  bool changed = true;
  EXPECT_TRUE(m.resolve("base_link", kBase, 10.0, tf.lookup(), &changed));
  EXPECT_FALSE(changed);
  EXPECT_EQ(tf.calls, 0);
  EXPECT_TRUE(m.seen());
  EXPECT_TRUE(m.resolved());
  EXPECT_TRUE(m.ext().identity);
  EXPECT_EQ(m.describe(kBase), "identity (frame base_link)");
  EXPECT_EQ(m.problem(kBase, "/points", "points_frame"), "");
  // the name is compared with base_frame as given, not with a fixed "base_link"
  EXPECT_TRUE(m.resolve("chassis", "chassis", 10.1, tf.lookup()));
  EXPECT_EQ(m.describe("chassis"), "identity (frame chassis)");
  // ... and a frame is the identity only for as long as it is base_frame
  EXPECT_FALSE(m.resolve("chassis", kBase, 10.15, tf.lookup()));
  EXPECT_FALSE(m.resolve("base_link", "chassis", 10.2, tf.lookup()));
  EXPECT_EQ(tf.calls, 2);
}

TEST(Mount, EmptyFrameIsNotUsed) {
  FakeTf tf;
  Mount m;
  EXPECT_FALSE(m.resolve("", kBase, 1.0, tf.lookup()));
  EXPECT_EQ(tf.calls, 0);
  EXPECT_TRUE(m.seen());
  EXPECT_FALSE(m.resolved());
  EXPECT_EQ(m.describe(kBase), "missing (empty frame_id)");
  EXPECT_EQ(m.problem(kBase, "/scan3d", "points_frame"),
            "empty header.frame_id on /scan3d: set points_frame to the sensor's frame (to 'base_link' if "
            "the data is already expressed in it)");
}

TEST(Mount, FrameWithoutTransformIsNotUsedUntilTheTransformAppears) {
  FakeTf tf;
  Mount m;
  bool changed = true;
  EXPECT_FALSE(m.resolve("lidar", kBase, 1.0, tf.lookup(), &changed));
  EXPECT_FALSE(changed);
  EXPECT_FALSE(m.resolved());
  EXPECT_EQ(m.describe(kBase), "missing (frame lidar)");
  EXPECT_EQ(m.problem(kBase, "/points", "points_frame"),
            "no transform base_link -> lidar (frame of /points): publish it on TF, or set points_frame "
            "[\"lidar\" does not exist]");
  // asked again on every message while missing (no waiting, no back-off)
  EXPECT_FALSE(m.resolve("lidar", kBase, 1.01, tf.lookup()));
  EXPECT_EQ(tf.calls, 2);
  // the transform appears: used from the next message on, and nothing stale remains
  tf.frames["lidar"] = mountAt(0.2, 0.0, 0.5, 0.0);
  EXPECT_TRUE(m.resolve("lidar", kBase, 1.02, tf.lookup(), &changed));
  EXPECT_TRUE(changed);
  EXPECT_TRUE(m.resolved());
  EXPECT_FALSE(m.ext().identity);
  EXPECT_EQ(m.ext().T.translation(), Eigen::Vector3d(0.2, 0.0, 0.5));
  EXPECT_EQ(m.problem(kBase, "/points", "points_frame"), "");
  EXPECT_EQ(m.describe(kBase), "base_link -> lidar xyz 0.200 0.000 0.500 rpy 0.000 0.000 0.000");
}

// A found mount is used for kRefreshS without asking TF, then read again.
TEST(Mount, FoundMountIsCachedForOneSecondThenReadAgain) {
  FakeTf tf;
  tf.frames["lidar"] = mountAt(0.2, 0.0, 0.5, 0.0);
  Mount m;
  bool changed = false;
  EXPECT_TRUE(m.resolve("lidar", kBase, 5.0, tf.lookup(), &changed));
  EXPECT_TRUE(changed);
  EXPECT_EQ(tf.calls, 1);
  tf.frames["lidar"] = mountAt(0.3, 0.0, 0.5, 0.0);  // recalibrated
  EXPECT_TRUE(m.resolve("lidar", kBase, 5.0 + Mount::kRefreshS - 0.001, tf.lookup(), &changed));
  EXPECT_FALSE(changed);
  EXPECT_EQ(tf.calls, 1);                             // just inside the second: not asked
  EXPECT_EQ(m.ext().T.translation().x(), 0.2);
  EXPECT_TRUE(m.resolve("lidar", kBase, 5.0 + Mount::kRefreshS, tf.lookup(), &changed));
  EXPECT_TRUE(changed);                               // at one second: asked, and the new value is taken
  EXPECT_EQ(tf.calls, 2);
  EXPECT_EQ(m.ext().T.translation().x(), 0.3);
  // read again a second later, unchanged: no "changed"
  EXPECT_TRUE(m.resolve("lidar", kBase, 7.5, tf.lookup(), &changed));
  EXPECT_FALSE(changed);
  EXPECT_EQ(tf.calls, 3);
}

// tf2 empties its buffer when time jumps back; the mount found before is kept.
TEST(Mount, FoundMountIsKeptWhenTfLosesIt) {
  FakeTf tf;
  tf.frames["imu"] = mountAt(0.0, 0.1, 0.2, 1.0);
  Mount m;
  EXPECT_TRUE(m.resolve("imu", kBase, 1.0, tf.lookup()));
  tf.frames.clear();
  bool changed = true;
  EXPECT_TRUE(m.resolve("imu", kBase, 3.0, tf.lookup(), &changed));
  EXPECT_FALSE(changed);
  EXPECT_EQ(tf.calls, 2);
  EXPECT_TRUE(m.resolved());
  EXPECT_EQ(m.ext().T.translation(), Eigen::Vector3d(0.0, 0.1, 0.2));
  EXPECT_EQ(m.problem(kBase, "/imu", "imu_frame"), "");
  // the failed lookup also counts as a lookup: not asked again within the second
  EXPECT_TRUE(m.resolve("imu", kBase, 3.5, tf.lookup()));
  EXPECT_EQ(tf.calls, 2);
  // but another frame that TF does not know is not covered by the kept mount
  EXPECT_FALSE(m.resolve("imu2", kBase, 3.6, tf.lookup()));
  EXPECT_EQ(m.describe(kBase), "missing (frame imu2)");
  // and going back to the first frame starts from scratch
  EXPECT_FALSE(m.resolve("imu", kBase, 3.7, tf.lookup()));
}

TEST(Mount, NonFiniteTransformCountsAsMissing) {
  FakeTf tf;
  Eigen::Isometry3d bad = mountAt(0.2, 0.0, 0.5, 0.0);
  bad.translation().y() = std::numeric_limits<double>::quiet_NaN();
  tf.frames["lidar"] = bad;
  Mount m;
  EXPECT_FALSE(m.resolve("lidar", kBase, 1.0, tf.lookup()));
  EXPECT_EQ(m.problem(kBase, "/points", "points_frame"),
            "no transform base_link -> lidar (frame of /points): publish it on TF, or set points_frame "
            "[the transform is not finite]");
  bad.translation().y() = std::numeric_limits<double>::infinity();
  tf.frames["lidar"] = bad;
  EXPECT_FALSE(m.resolve("lidar", kBase, 1.1, tf.lookup()));
}

// The parameter replaces whatever the message says, including an empty frame_id.
TEST(Mount, OverrideFrameReplacesTheMessageFrame) {
  FakeTf tf;
  tf.frames["lidar"] = mountAt(0.2, 0.0, 0.5, 0.0);
  Mount to_base;
  to_base.override_frame = "base_link";
  EXPECT_TRUE(to_base.resolve("", kBase, 1.0, tf.lookup()));
  EXPECT_TRUE(to_base.resolve("not_on_tf", kBase, 1.0, tf.lookup()));
  EXPECT_TRUE(to_base.ext().identity);
  EXPECT_EQ(to_base.describe(kBase), "identity (frame base_link)");
  EXPECT_EQ(tf.calls, 0);
  Mount to_lidar;
  to_lidar.override_frame = "lidar";
  EXPECT_TRUE(to_lidar.resolve("", kBase, 1.0, tf.lookup()));
  EXPECT_EQ(to_lidar.frame(), "lidar");
  EXPECT_EQ(to_lidar.ext().T.translation().z(), 0.5);
}

// An identity transform on TF takes the same no-arithmetic path as base_frame itself.
TEST(Mount, IdentityTransformOnTfIsTheIdentity) {
  FakeTf tf;
  tf.frames["lidar"] = Eigen::Isometry3d::Identity();
  Mount m;
  bool changed = false;
  EXPECT_TRUE(m.resolve("lidar", kBase, 1.0, tf.lookup(), &changed));
  EXPECT_TRUE(changed);  // a mount found on TF is new even when it equals the default
  EXPECT_TRUE(m.ext().identity);
  EXPECT_EQ(m.describe(kBase), "base_link -> lidar xyz 0.000 0.000 0.000 rpy 0.000 0.000 0.000");
}

// States in sequence on one mount: missing -> found -> base_frame -> empty -> found again.
TEST(Mount, LeavesNoStaleStateAcrossTransitions) {
  FakeTf tf;
  Mount m;
  EXPECT_FALSE(m.resolve("gnss", kBase, 1.0, tf.lookup()));
  tf.frames["gnss"] = mountAt(-0.3, 0.2, 1.0, 0.0);
  EXPECT_TRUE(m.resolve("gnss", kBase, 2.0, tf.lookup()));
  EXPECT_FALSE(m.ext().identity);
  EXPECT_TRUE(m.resolve("base_link", kBase, 3.0, tf.lookup()));
  EXPECT_TRUE(m.ext().identity);  // not the antenna offset of the previous message
  EXPECT_EQ(m.ext().T.translation(), Eigen::Vector3d::Zero());
  EXPECT_FALSE(m.resolve("", kBase, 4.0, tf.lookup()));
  EXPECT_EQ(m.describe(kBase), "missing (empty frame_id)");
  bool changed = false;
  EXPECT_TRUE(m.resolve("gnss", kBase, 4.1, tf.lookup(), &changed));
  EXPECT_TRUE(changed);
  EXPECT_EQ(m.ext().T.translation(), Eigen::Vector3d(-0.3, 0.2, 1.0));
  EXPECT_EQ(m.problem(kBase, "/gnss", "gnss_frame"), "");
}

TEST(Mount, DescribeRoundsAndNeverPrintsMinusZero) {
  FakeTf tf;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(-0.0004, 0.0004, -1.2346);
  T.linear() = (Eigen::AngleAxisd(-2.5, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
  tf.frames["cam"] = T;
  Mount m;
  EXPECT_TRUE(m.resolve("cam", kBase, 1.0, tf.lookup()));
  EXPECT_EQ(m.describe(kBase), "base_link -> cam xyz 0.000 0.000 -1.235 rpy -0.300 0.200 -2.500");
}
