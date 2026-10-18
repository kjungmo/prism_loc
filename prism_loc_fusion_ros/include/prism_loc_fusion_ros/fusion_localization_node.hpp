#pragma once
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/geodetic.hpp"
#include "prism_loc_fusion_ros/mount.hpp"
#include "prism_loc_fusion_ros/ndt_registration.hpp"
namespace prism_loc_fusion_ros {
class FusionLocalizationNode : public rclcpp::Node {
 public:
  explicit FusionLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
 private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onWatchdog();
  void onDiagnostics();
  void onTfWatch(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  bool tryInitialize();
  using Mat15 = prism_loc_fusion::Eskf::Mat15;
  void seedTilt(Mat15& P0, const Eigen::Quaterniond& q) const;
  void publish(const rclcpp::Time& stamp);

  // Mounts of the sensors on base_frame (see mount.hpp). resolveMount() returns false
  // when the message must not be used, and logs a mount that is new or has changed.
  bool resolveMount(Mount& m, const std::string& msg_frame, const std::string& topic);
  Mount points_mount_, imu_mount_, gnss_mount_;
  MountLookup mount_lookup_;

  std::string global_frame_, odom_frame_, base_frame_;
  std::string imu_topic_, points_topic_, gnss_topic_;
  double ndt_max_fitness_, points_voxel_leaf_, transform_tolerance_;
  double pose_pos_std_, pose_rot_std_, gnss_max_pos_cov_, initial_yaw_{0.0};
  int gnss_min_status_{0};
  bool map_to_base_fallback_{true};

  std::unique_ptr<prism_loc_fusion::Eskf> eskf_;
  prism_loc_fusion::EskfParams eskf_params_;
  // Starting covariance of the attitude (see tryInitialize): with a GNSS antenna off the
  // origin of base_frame the roll and pitch start at the accelerometer's accuracy.
  Eigen::Vector3d last_acc_{Eigen::Vector3d::Zero()};   // latest specific force, base_frame
  bool last_acc_valid_{false};
  bool tilt_seed_pending_{false};    // started before the antenna mount was known
  bool attitude_from_pose_{false};   // the attitude to start from is the one of /initialpose (roll and pitch: from the accelerometer)
  // Where the heading came from last (/diagnostics): the lidar (an accepted NDT correction), /initialpose, the
  // motion (the kinematic alignment of a lever-arm filter at a start or a stop, Eskf::alignHeading), or neither
  // (the initial_yaw parameter, then the IMU alone). The lever-arm update bounds the heading
  // uncertainty it uses (EskfParams::lever_heading_sigma_max); capped_active_ is true while every lever update
  // since capped_since_wall_ was bounded.
  enum class HeadingSource { kNone, kInitialPose, kLidar, kMotion };
  HeadingSource heading_source_{HeadingSource::kNone};
  bool initial_yaw_known_{false};
  bool capped_active_{false};
  long ndt_since_init_{0};           // NDT corrections since the filter started
  prism_loc_fusion::GeodeticConverter geo_;
  std::unique_ptr<NdtRegistration> ndt_;

  bool filter_init_{false}, have_attitude_{false}, have_position_{false}, last_imu_valid_{false};
  // Startup watchdog: track whether each required input has been seen at least once.
  bool imu_seen_{false}, points_seen_{false}, gnss_seen_{false}, initialpose_seen_{false};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // /diagnostics (1 Hz wall timer) and steady-clock bookkeeping. Throttled warnings use
  // steady_clock_ so they keep firing when the ROS clock is stuck (use_sim_time, no /clock).
  using Steady = std::chrono::steady_clock;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  double startup_timeout_s_{30.0}, input_timeout_s_{1.0}, input_timeout_periods_{5.0};
  double correction_timeout_s_{5.0};
  Steady::time_point start_wall_, last_imu_wall_, last_points_wall_, last_ndt_wall_, last_gnss_wall_;
  Steady::time_point init_wall_, heading_source_wall_, capped_since_wall_;
  bool any_ndt_{false}, any_gnss_fix_{false};
  double imu_period_s_{0.0};   // smoothed IMU stamp interval (gaps excluded)
  long imu_count_{0}, imu_gaps_{0}, imu_gaps_reported_{0}, ndt_rejected_{0};
  std::string tf_child_{"none"};  // child frame of the last broadcast TF
  long odom_tf_missing_{0};       // publishes without odom->base since the last status
  // Who publishes odom->base_link is watched on /tf directly: while this node broadcasts
  // map->base_link, lookups of odom->base_link fail intermittently (base_link's parent
  // flips), so they cannot tell whether odometry exists.
  bool ext_odom_seen_{false};     // odom->base_link seen on /tf at least once
  bool fallback_ever_{false};     // map->base_link broadcast at least once
  bool two_parents_{false};       // odom->base_link on /tf while map->base_link was broadcast
  Steady::time_point last_ext_odom_wall_, last_fallback_wall_, two_parents_wall_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_watch_sub_;
  rclcpp::Time last_ros_now_{0, 0, RCL_ROS_TIME};
  int ros_clock_stuck_ticks_{0};
  rclcpp::Time last_imu_time_;
  Eigen::Quaterniond init_attitude_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d init_position_{Eigen::Vector3d::Zero()};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
};
}  // namespace prism_loc_fusion_ros
