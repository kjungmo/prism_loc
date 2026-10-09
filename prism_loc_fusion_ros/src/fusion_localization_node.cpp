#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
#include "prism_loc_fusion_ros/tf_util.hpp"
#include "prism_loc_fusion_ros/param_validation.hpp"
#include "prism_loc_fusion/so3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include <stdexcept>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/time.h>
namespace prism_loc_fusion_ros {
using prism_loc_fusion::NominalState;
using Mat15 = prism_loc_fusion::Eskf::Mat15;

static Eigen::Quaterniond attitudeFromAccel(const Eigen::Vector3d& f, double yaw) {
  const double roll = std::atan2(f.y(), f.z());
  const double pitch = std::atan2(-f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
}

FusionLocalizationNode::FusionLocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("prism_loc_fusion", options) {
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  ndt_max_fitness_ = declare_parameter<double>("ndt_max_fitness", 2.0);
  points_voxel_leaf_ = declare_parameter<double>("points_voxel_leaf", 0.5);
  pose_pos_std_ = declare_parameter<double>("pose_pos_std", 0.1);
  pose_rot_std_ = declare_parameter<double>("pose_rot_std", 0.05);
  gnss_min_status_ = declare_parameter<int>("gnss_min_status", 0);
  gnss_max_pos_cov_ = declare_parameter<double>("gnss_max_pos_cov", 25.0);
  initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);

  prism_loc_fusion::EskfParams ep;
  ep.sigma_acc = declare_parameter<double>("sigma_acc", 1e-2);
  ep.sigma_gyro = declare_parameter<double>("sigma_gyro", 1e-3);
  ep.sigma_acc_bias = declare_parameter<double>("sigma_acc_bias", 1e-4);
  ep.sigma_gyro_bias = declare_parameter<double>("sigma_gyro_bias", 1e-5);
  ep.reset_jacobian = declare_parameter<bool>("reset_jacobian", false);
  eskf_ = std::make_unique<prism_loc_fusion::Eskf>(ep);

  // datum_* are declared unconditionally so YAML values always bind; used only when
  // use_datum is true.
  const bool use_datum = declare_parameter<bool>("use_datum", false);
  const prism_loc_fusion::GeoPoint datum{declare_parameter<double>("datum_lat", 0.0),
                                         declare_parameter<double>("datum_lon", 0.0),
                                         declare_parameter<double>("datum_alt", 0.0)};
  if (use_datum) geo_.setDatum(datum);

  // When odom->base is unavailable, publish map->base_link instead of nothing. Set false
  // when another node (wheel odometry, an EKF) owns odom->base_link: if it starts after
  // this node, base_link would otherwise get two parents.
  map_to_base_fallback_ = declare_parameter<bool>("map_to_base_fallback", true);
  FusionNodeParams vp;
  vp.transform_tolerance = transform_tolerance_;
  vp.ndt_max_fitness = ndt_max_fitness_;
  vp.points_voxel_leaf = points_voxel_leaf_;
  vp.pose_pos_std = pose_pos_std_;
  vp.pose_rot_std = pose_rot_std_;
  vp.gnss_max_pos_cov = gnss_max_pos_cov_;
  // IMU messages queue while a point cloud is being registered on the same executor.
  // The default keeps SensorDataQoS's depth (5); raise it (e.g. 200) when /diagnostics
  // reports IMU gaps, so the queue holds one NDT align's worth of samples.
  vp.imu_queue_depth = declare_parameter<int>("imu_queue_depth", vp.imu_queue_depth);
  // /diagnostics: startup grace, IMU silence threshold (larger of input_timeout_s and
  // input_timeout_periods x the IMU period) and the age of the last accepted NDT
  // correction that turns the status WARN.
  startup_timeout_s_ = vp.startup_timeout_s =
      declare_parameter<double>("startup_timeout_s", vp.startup_timeout_s);
  input_timeout_s_ = vp.input_timeout_s = declare_parameter<double>("input_timeout_s", vp.input_timeout_s);
  input_timeout_periods_ = vp.input_timeout_periods =
      declare_parameter<double>("input_timeout_periods", vp.input_timeout_periods);
  correction_timeout_s_ = vp.correction_timeout_s =
      declare_parameter<double>("correction_timeout_s", vp.correction_timeout_s);

  const std::string pcd = declare_parameter<std::string>("map_pcd_path", "");
  vp.ndt_resolution = declare_parameter<double>("ndt_resolution", 1.0);
  vp.ndt_step_size = declare_parameter<double>("ndt_step_size", 0.1);
  vp.ndt_epsilon = declare_parameter<double>("ndt_epsilon", 0.01);
  vp.ndt_max_iter = declare_parameter<int>("ndt_max_iter", 30);
  validateFusionParams(vp, ep);
  ndt_ = std::make_unique<NdtRegistration>(vp.ndt_resolution, vp.ndt_step_size, vp.ndt_epsilon,
                                           vp.ndt_max_iter);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcd.empty()) {
    RCLCPP_ERROR(get_logger(), "fusion3d: map_pcd_path is empty");
    throw std::runtime_error(
        "fusion3d: map_pcd_path is empty - set map_pcd_path to a valid .pcd map file "
        "(e.g. ros2 launch ... map_pcd_path:=/abs/path/to/map.pcd)");
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *map) < 0 || map->empty()) {
    RCLCPP_ERROR(get_logger(), "fusion3d: failed to load PCD map: %s", pcd.c_str());
    throw std::runtime_error(
        "fusion3d: could not load a non-empty PCD map from '" + pcd +
        "' - check the file exists, is readable, and is a valid non-empty point cloud");
  }
  ndt_->setTarget(map);
  RCLCPP_INFO(get_logger(), "fusion3d: NDT map loaded (%zu pts)", map->size());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);

  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  points_topic_ = declare_parameter<std::string>("points_topic", "/points");
  gnss_topic_ = declare_parameter<std::string>("gnss_topic", "/gnss");
  // IMU/points/GNSS drivers commonly publish BEST_EFFORT, so match with SensorDataQoS.
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(vp.imu_queue_depth)),
      std::bind(&FusionLocalizationNode::onImu, this, std::placeholders::_1));
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onPoints, this, std::placeholders::_1));
  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onGnss, this, std::placeholders::_1));
  initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&FusionLocalizationNode::onInitialPose, this, std::placeholders::_1));
  // Startup watchdog: warn every 10 s about required inputs that have gone silent.
  watchdog_timer_ = create_wall_timer(std::chrono::seconds(10),
                                      std::bind(&FusionLocalizationNode::onWatchdog, this));
  tf_watch_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      std::bind(&FusionLocalizationNode::onTfWatch, this, std::placeholders::_1));
  // Wall timer, so /diagnostics keeps flowing when inputs stop or the ROS clock is stuck.
  start_wall_ = last_imu_wall_ = last_points_wall_ = last_ndt_wall_ = last_gnss_wall_ = Steady::now();
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&FusionLocalizationNode::onDiagnostics, this));
  RCLCPP_INFO(get_logger(), "prism_loc_fusion (fusion3d) up");
}

void FusionLocalizationNode::onWatchdog() {
  std::lock_guard<std::mutex> lk(mutex_);
  bool all_seen = true;
  if (!imu_seen_) {
    all_seen = false;
    RCLCPP_WARN(get_logger(),
                "no messages on %s (%zu publishers) - check imu_topic and the IMU driver",
                imu_topic_.c_str(), count_publishers(imu_topic_));
  }
  if (!points_seen_) {
    all_seen = false;
    RCLCPP_WARN(get_logger(),
                "no messages on %s (%zu publishers) - check points_topic and the LiDAR driver",
                points_topic_.c_str(), count_publishers(points_topic_));
  }
  // GNSS is optional once a pose has come from /initialpose.
  if (!gnss_seen_ && !initialpose_seen_) {
    all_seen = false;
    RCLCPP_WARN(get_logger(),
                "no messages on %s (%zu publishers) - check gnss_topic and the GNSS driver, "
                "or use /initialpose instead of GNSS for the initial fix",
                gnss_topic_.c_str(), count_publishers(gnss_topic_));
  }
  if (all_seen) watchdog_timer_->cancel();
}

// Notes odom->base_link transforms published by any node (diagnostics only).
void FusionLocalizationNode::onTfWatch(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto& t : msg->transforms) {
    if (t.header.frame_id != odom_frame_ || t.child_frame_id != base_frame_) continue;
    std::lock_guard<std::mutex> lk(mutex_);
    const auto now_w = Steady::now();
    ext_odom_seen_ = true;
    last_ext_odom_wall_ = now_w;
    if (fallback_ever_ && now_w - last_fallback_wall_ < std::chrono::seconds(1)) {
      two_parents_ = true;
      two_parents_wall_ = now_w;
    }
    return;
  }
}

// 1 Hz wall timer. Only reads state under the mutex; no TF lookups or registration here.
void FusionLocalizationNode::onDiagnostics() {
  std::lock_guard<std::mutex> lk(mutex_);
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  const auto now_w = Steady::now();
  auto secs = [&now_w](Steady::time_point t) {
    return std::chrono::duration<double>(now_w - t).count();
  };
  const double since_start = secs(start_wall_);
  const double since_imu = secs(last_imu_wall_);
  char buf[320];
  std::vector<std::string> errors, warnings;

  // Errors: the fused pose cannot be propagated or was never initialized.
  const double imu_timeout = std::max(input_timeout_s_, input_timeout_periods_ * imu_period_s_);
  if (!imu_seen_ && since_start > startup_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "no IMU on %s after %.0f s", imu_topic_.c_str(), since_start);
    errors.emplace_back(buf);
  } else if (imu_seen_ && since_imu > imu_timeout) {
    std::snprintf(buf, sizeof(buf), "IMU stopped: none on %s for %.1f s", imu_topic_.c_str(),
                  since_imu);
    errors.emplace_back(buf);
  }
  if (!filter_init_ && since_start > startup_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "not initialized: need IMU attitude and a position fix "
                  "(GNSS on %s or /initialpose)", gnss_topic_.c_str());
    errors.emplace_back(buf);
  }

  // Warnings: the pose is dead-reckoning on the IMU or the TF tree is incomplete.
  if (filter_init_) {
    const double since_ndt = any_ndt_ ? secs(last_ndt_wall_) : since_start;
    if (since_ndt > std::max(correction_timeout_s_, startup_timeout_s_ * (any_ndt_ ? 0.0 : 1.0))) {
      std::snprintf(buf, sizeof(buf), "no accepted NDT correction for %.1f s (%ld rejected; "
                    "points on %s %s)", since_ndt, ndt_rejected_, points_topic_.c_str(),
                    points_seen_ ? "arriving" : "never received");
      warnings.emplace_back(buf);
    }
  }
  // A robot without odometry runs on the fallback by design: that is OK. Warn when the
  // fallback is off (no TF at all), when odometry was seen and then lost, and for a
  // while after odometry appeared next to the fallback (two parents for base_link).
  const bool fallback_active = fallback_ever_ && secs(last_fallback_wall_) < 1.5;
  if (ext_odom_seen_ && secs(last_ext_odom_wall_) > 1.0) {
    std::snprintf(buf, sizeof(buf), "lost %s->%s: none on /tf for %.1f s (odometry stopped?)%s",
                  odom_frame_.c_str(), base_frame_.c_str(), secs(last_ext_odom_wall_),
                  fallback_active ? "; publishing map->base_link instead (map_to_base_fallback)" : "");
    warnings.emplace_back(buf);
  } else if (!ext_odom_seen_ && odom_tf_missing_ > 0 && !map_to_base_fallback_) {
    std::snprintf(buf, sizeof(buf), "no %s->%s TF: not broadcasting TF (map_to_base_fallback is "
                  "false)", odom_frame_.c_str(), base_frame_.c_str());
    warnings.emplace_back(buf);
  }
  if (two_parents_ && secs(two_parents_wall_) < 30.0) {
    std::snprintf(buf, sizeof(buf), "%s->%s published while map->%s was being broadcast: %s had "
                  "two parents; set map_to_base_fallback false when another node owns %s->%s",
                  odom_frame_.c_str(), base_frame_.c_str(), base_frame_.c_str(),
                  base_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    warnings.emplace_back(buf);
  }
  if (imu_gaps_ > imu_gaps_reported_) {
    std::snprintf(buf, sizeof(buf), "%ld IMU gap(s) in the last second (samples lost; %ld total)",
                  imu_gaps_ - imu_gaps_reported_, imu_gaps_);
    warnings.emplace_back(buf);
  }
  const rclcpp::Time ros_now = now();
  const bool sim_time = get_parameter("use_sim_time").as_bool();
  ros_clock_stuck_ticks_ =
      (sim_time && ros_now.nanoseconds() == last_ros_now_.nanoseconds()) ? ros_clock_stuck_ticks_ + 1
                                                                         : 0;
  last_ros_now_ = ros_now;
  if (ros_clock_stuck_ticks_ >= 2) {
    std::snprintf(buf, sizeof(buf), "use_sim_time is true but the ROS clock has not advanced for "
                  "%d s (is /clock published?)", ros_clock_stuck_ticks_);
    warnings.emplace_back(buf);
  }

  DS st;
  st.name = std::string(get_name()) + ": fusion";
  st.hardware_id = get_namespace();
  std::string msg;
  for (const auto& e : errors) msg += (msg.empty() ? "" : "; ") + e;
  for (const auto& w : warnings) msg += (msg.empty() ? "" : "; ") + w;
  if (!errors.empty()) st.level = DS::ERROR;
  else if (!warnings.empty()) st.level = DS::WARN;
  else st.level = DS::OK;
  if (msg.empty()) msg = filter_init_ ? "localizing" : "starting: waiting for IMU and a position fix";
  st.message = msg;
  auto kv = [&st](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; st.values.push_back(p);
  };
  auto num = [](double v, const char* fmt) {
    char b[64];
    std::snprintf(b, sizeof(b), fmt, v);
    return std::string(b);
  };
  kv("filter_initialized", filter_init_ ? "true" : "false");
  kv("imu_rate_hz", num(imu_period_s_ > 0.0 ? 1.0 / imu_period_s_ : 0.0, "%.1f"));
  kv("seconds_since_last_imu", num(imu_seen_ ? since_imu : since_start, "%.2f"));
  kv("imu_gaps_total", std::to_string(imu_gaps_));
  kv("seconds_since_last_points", num(points_seen_ ? secs(last_points_wall_) : since_start, "%.2f"));
  kv("seconds_since_ndt_correction", any_ndt_ ? num(secs(last_ndt_wall_), "%.2f") : "never");
  kv("seconds_since_gnss_correction", any_gnss_fix_ ? num(secs(last_gnss_wall_), "%.2f") : "never");
  kv("ndt_rejected_total", std::to_string(ndt_rejected_));
  kv("covariance_trace_position",
     filter_init_ ? num(eskf_->covariance().block<3, 3>(0, 0).trace(), "%.6f") : "n/a");
  kv("tf_child_frame", tf_child_);
  kv("map_to_base_fallback_active", fallback_active ? "true" : "false");
  kv("use_sim_time", sim_time ? "true" : "false");
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = ros_now;
  arr.status.push_back(st);
  diag_pub_->publish(arr);
  imu_gaps_reported_ = imu_gaps_;
  odom_tf_missing_ = 0;
}

bool FusionLocalizationNode::tryInitialize() {
  if (filter_init_) return true;
  if (!(have_attitude_ && have_position_)) return false;
  NominalState x0;
  x0.p = init_position_;
  x0.q = init_attitude_;
  Mat15 P0 = Mat15::Identity();
  P0.block<3, 3>(0, 0) *= 1.0;
  P0.block<3, 3>(3, 3) *= 1.0;
  P0.block<3, 3>(6, 6) *= 0.5;
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  eskf_->initialize(x0, P0);
  filter_init_ = true;
  RCLCPP_INFO(get_logger(), "fusion3d: initialized at (%.2f, %.2f, %.2f)", x0.p.x(), x0.p.y(), x0.p.z());
  return true;
}

void FusionLocalizationNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  imu_seen_ = true;
  last_imu_wall_ = Steady::now();
  const Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  if (!have_attitude_) { init_attitude_ = attitudeFromAccel(acc, initial_yaw_); have_attitude_ = true; }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (!last_imu_valid_) { last_imu_time_ = stamp; last_imu_valid_ = true; return; }
  const double dt = (stamp - last_imu_time_).seconds();
  last_imu_time_ = stamp;
  // Gap counter: an interval well beyond the usual IMU period means samples were lost
  // (driver, transport or a full queue); intervals over 0.5 s also skip prediction.
  if (dt > 0.0) {
    if (imu_count_ > 20 && (dt > 3.0 * imu_period_s_ || dt > 0.5)) ++imu_gaps_;
    else imu_period_s_ = imu_count_ == 0 ? dt : 0.95 * imu_period_s_ + 0.05 * dt;
    ++imu_count_;
  }
  if (!tryInitialize()) return;
  if (dt <= 0.0 || dt > 0.5) return;
  eskf_->predict(acc, gyro, dt);
  publish(stamp);
}

void FusionLocalizationNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  points_seen_ = true;
  last_points_wall_ = Steady::now();
  if (!ndt_ || !ndt_->hasTarget()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onPoints: NDT target map not set - map_pcd_path failed to load; "
                         "check map_pcd_path");
    return;
  }
  if (!filter_init_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onPoints: waiting for filter initialization - need IMU attitude and a "
                         "position fix (GNSS or /initialpose); check imu_topic/gnss_topic or "
                         "publish to /initialpose");
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  const float leaf = static_cast<float>(points_voxel_leaf_);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*src);
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess.translation() = eskf_->state().p;
  guess.linear() = eskf_->state().q.toRotationMatrix();
  NdtResult r = ndt_->align(src, guess);
  if (!r.converged || r.fitness > ndt_max_fitness_) {
    ++ndt_rejected_;
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "fusion3d: NDT rejected (conv=%d fit=%.3f)",
                         static_cast<int>(r.converged), r.fitness);
    return;
  }
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
  R.topLeftCorner<3, 3>() *= pose_pos_std_ * pose_pos_std_;
  R.bottomRightCorner<3, 3>() *= pose_rot_std_ * pose_rot_std_;
  eskf_->updatePose(r.pose.translation(), Eigen::Quaterniond(r.pose.linear()), R);
  any_ndt_ = true;
  last_ndt_wall_ = Steady::now();
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  gnss_seen_ = true;
  if (msg->status.status < gnss_min_status_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onGnss: fix rejected - status %d < gnss_min_status %d",
                         static_cast<int>(msg->status.status), gnss_min_status_);
    return;
  }
  if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onGnss: fix rejected - NaN latitude/longitude (lat=%.6f lon=%.6f)",
                         msg->latitude, msg->longitude);
    return;
  }
  if (msg->position_covariance[0] > gnss_max_pos_cov_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onGnss: fix rejected - position covariance %.2f > gnss_max_pos_cov %.2f",
                         msg->position_covariance[0], gnss_max_pos_cov_);
    return;
  }
  const prism_loc_fusion::GeoPoint gp{msg->latitude, msg->longitude, msg->altitude};
  if (!geo_.hasDatum()) geo_.setDatum(gp);
  const Eigen::Vector3d enu = geo_.toEnu(gp);
  if (!have_position_) { init_position_ = enu; have_position_ = true; }
  if (!filter_init_) { tryInitialize(); return; }
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0, 0) = std::max(msg->position_covariance[0], 1e-4);
  R(1, 1) = std::max(msg->position_covariance[4], 1e-4);
  R(2, 2) = std::max(msg->position_covariance[8], 1.0);
  eskf_->updatePosition(enu, R);
  any_gnss_fix_ = true;
  last_gnss_wall_ = Steady::now();
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  init_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  const auto& o = msg->pose.pose.orientation;
  init_attitude_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
  have_position_ = have_attitude_ = true;
  initialpose_seen_ = true;
  filter_init_ = false;
  RCLCPP_INFO(get_logger(), "fusion3d: initialpose set");
}

void FusionLocalizationNode::publish(const rclcpp::Time& stamp) {
  if (!filter_init_) return;
  const auto& x = eskf_->state();
  Eigen::Isometry3d map_base = Eigen::Isometry3d::Identity();
  map_base.translation() = x.p;
  map_base.linear() = x.q.toRotationMatrix();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
  tf.header.frame_id = global_frame_;
  Eigen::Isometry3d out_tf;
  std::string child;
  bool send_tf = true;
  try {
    // Prefer odom->base at the measurement stamp; publish() runs at IMU rate,
    // so never block — fall back to the latest buffered transform when the
    // odometry publisher has not caught up to the stamp yet.
    const auto t = tf_buffer_->canTransform(odom_frame_, base_frame_, stamp)
                       ? tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp)
                       : tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
    odom_base.translation() = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    odom_base.linear() = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                                            t.transform.rotation.y, t.transform.rotation.z).toRotationMatrix();
    out_tf = computeMapToOdom(map_base, odom_base);
    child = odom_frame_;
  } catch (const std::exception&) {
    ++odom_tf_missing_;
    if (map_to_base_fallback_) {
      out_tf = map_base;
      child = base_frame_;
      fallback_ever_ = true;
      last_fallback_wall_ = Steady::now();
      if (ext_odom_seen_) {
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                             "fusion3d: %s->%s lost; publishing %s->%s (map_to_base_fallback)",
                             odom_frame_.c_str(), base_frame_.c_str(), global_frame_.c_str(),
                             base_frame_.c_str());
      } else {
        RCLCPP_INFO_ONCE(get_logger(),
                         "fusion3d: no %s->%s; publishing %s->%s (map_to_base_fallback)",
                         odom_frame_.c_str(), base_frame_.c_str(), global_frame_.c_str(),
                         base_frame_.c_str());
      }
    } else {
      send_tf = false;
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                           "fusion3d: no %s->%s; not broadcasting TF (map_to_base_fallback is "
                           "false), pose topics still publish",
                           odom_frame_.c_str(), base_frame_.c_str());
    }
  }
  tf_child_ = send_tf ? child : "none";
  if (send_tf) {
    tf.child_frame_id = child;
    tf.transform.translation.x = out_tf.translation().x();
    tf.transform.translation.y = out_tf.translation().y();
    tf.transform.translation.z = out_tf.translation().z();
    Eigen::Quaterniond oq(out_tf.linear());
    tf.transform.rotation.x = oq.x(); tf.transform.rotation.y = oq.y();
    tf.transform.rotation.z = oq.z(); tf.transform.rotation.w = oq.w();
    tf_broadcaster_->sendTransform(tf);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped ps;
  ps.header.stamp = stamp; ps.header.frame_id = global_frame_;
  ps.pose.pose.position.x = x.p.x(); ps.pose.pose.position.y = x.p.y(); ps.pose.pose.position.z = x.p.z();
  ps.pose.pose.orientation.x = x.q.x(); ps.pose.pose.orientation.y = x.q.y();
  ps.pose.pose.orientation.z = x.q.z(); ps.pose.pose.orientation.w = x.q.w();
  const auto& P = eskf_->covariance();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      ps.pose.covariance[i * 6 + j] = P(i, j);
      ps.pose.covariance[(i + 3) * 6 + (j + 3)] = P(6 + i, 6 + j);
    }
  pose_pub_->publish(ps);

  nav_msgs::msg::Odometry od;
  od.header.stamp = stamp; od.header.frame_id = global_frame_; od.child_frame_id = base_frame_;
  od.pose = ps.pose;
  od.twist.twist.linear.x = x.v.x(); od.twist.twist.linear.y = x.v.y(); od.twist.twist.linear.z = x.v.z();
  odom_pub_->publish(od);
}

}  // namespace prism_loc_fusion_ros
