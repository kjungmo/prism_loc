#include "prism_loc/localization_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>
#include "prism_loc/param_validation.hpp"
#include "prism_loc/tf_wait.hpp"
#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>
namespace prism_loc {
using prism_loc_core::Pose2D;

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("prism_loc", options) {
  backend_ = declare_parameter<std::string>("backend", "laser2d");
  if (backend_ != "laser2d" && backend_ != "ndt3d")
    throw std::invalid_argument("prism_loc: invalid parameter backend = " + backend_ +
                                " (must be \"laser2d\" or \"ndt3d\")");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  update_min_d_ = declare_parameter<double>("update_min_d", 0.2);
  update_min_a_ = declare_parameter<double>("update_min_a", 0.2);
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  tf_broadcast_ = declare_parameter<bool>("tf_broadcast", true);
  // /diagnostics: grace before missing inputs count as errors (drivers can be slow to
  // start), and the scan/cloud silence that turns the status ERROR: the larger of
  // input_timeout_s and input_timeout_periods x the observed input period.
  NodeParams np;
  np.update_min_d = update_min_d_;
  np.update_min_a = update_min_a_;
  np.transform_tolerance = transform_tolerance_;
  startup_timeout_s_ = np.startup_timeout_s =
      declare_parameter<double>("startup_timeout_s", np.startup_timeout_s);
  input_timeout_s_ = np.input_timeout_s = declare_parameter<double>("input_timeout_s", np.input_timeout_s);
  input_timeout_periods_ = np.input_timeout_periods =
      declare_parameter<double>("input_timeout_periods", np.input_timeout_periods);
  // Warn when the effective sample size of the last correction falls below this share of
  // the particle count (0 disables the check).
  min_neff_fraction_ = np.min_neff_fraction =
      declare_parameter<double>("min_neff_fraction", np.min_neff_fraction);
  validateNodeParams(np);

  prism_loc_core::ParticleFilterParams pp;
  pp.min_particles = declare_parameter<int>("min_particles", 500);
  pp.max_particles = declare_parameter<int>("max_particles", 2000);
  pp.resample_threshold = declare_parameter<double>("resample_threshold", 0.5);
  // KLD-sampling controls (defaults mirror ParticleFilterParams in prism_loc_core).
  pp.kld_err = declare_parameter<double>("kld_err", 0.05);
  pp.kld_z = declare_parameter<double>("kld_z", 2.33);
  pp.kld_bin_xy = declare_parameter<double>("kld_bin_xy", 0.5);
  pp.kld_bin_yaw = declare_parameter<double>("kld_bin_yaw", 0.17);

  prism_loc_core::MotionParams mp;
  mp.alpha1 = declare_parameter<double>("alpha1", 0.2);
  mp.alpha2 = declare_parameter<double>("alpha2", 0.2);
  mp.alpha3 = declare_parameter<double>("alpha3", 0.2);
  mp.alpha4 = declare_parameter<double>("alpha4", 0.2);

  validateFilterParams(pp, mp);
  rng_ = std::make_unique<prism_loc_core::Rng>(declare_parameter<int>("seed", 42));
  pf_ = std::make_unique<prism_loc_core::ParticleFilter>(pp, *rng_);
  motion_ = std::make_unique<prism_loc_core::OdometryMotionModel>(mp);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
  cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("~/particle_cloud", 10);
  initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&LocalizationNode::onInitialPose, this, std::placeholders::_1));

  if (backend_ == "laser2d") {
    // Declare the laser-model params so YAML overrides bind (read again in onMap).
    prism_loc_core::LaserParams lp;
    lp.max_beams = declare_parameter<int>("max_beams", 60);
    lp.z_hit = declare_parameter<double>("z_hit", 0.5);
    lp.z_rand = declare_parameter<double>("z_rand", 0.5);
    lp.sigma_hit = declare_parameter<double>("sigma_hit", 0.2);
    lp.max_dist = declare_parameter<double>("likelihood_max_dist", 2.0);
    laser_min_range_ = declare_parameter<double>("laser_min_range", 0.0);
    laser_max_range_ = declare_parameter<double>("laser_max_range", 0.0);
    validateLaserParams(lp, laser_min_range_, laser_max_range_);
    try_global_localization_ = declare_parameter<bool>("try_global_localization", false);
    // bbs_global_window=true: the search window is sized from the map extent
    // (capped per axis at bbs_max_linear_window); false: ±bbs_linear_window
    // about the map centre.
    bbs_global_window_ = declare_parameter<bool>("bbs_global_window", true);
    bbs_params_.linear_window = declare_parameter<double>("bbs_linear_window", 10.0);
    bbs_params_.max_linear_window = declare_parameter<double>("bbs_max_linear_window", 50.0);
    bbs_params_.angular_window = declare_parameter<double>("bbs_angular_window", M_PI);
    bbs_params_.angular_step = declare_parameter<double>("bbs_angular_step", 0.0175);
    bbs_params_.max_depth = declare_parameter<int>("bbs_max_depth", 6);
    bbs_params_.max_beams = declare_parameter<int>("bbs_max_beams", 120);
    bbs_params_.min_score_fraction = declare_parameter<double>("bbs_min_score_fraction", 0.4);
    bbs_params_.sigma_hit = get_parameter_or("sigma_hit", 0.2);
    // Relocalization verification: keep the top-K distinct BBS modes of the first
    // scan and commit only when one holds >= bbs_verify_min_posterior of the
    // posterior after bbs_verify_scans scans (odometry-chained); otherwise report
    // ambiguity and retry. top_k = 1 and scans = 1 restore the single-scan rule.
    const prism_loc_core::RelocVerifierParams rv;
    reloc_params_.top_k = declare_parameter<int>("bbs_verify_top_k", rv.top_k);
    reloc_params_.verify_scans = declare_parameter<int>("bbs_verify_scans", rv.verify_scans);
    reloc_params_.evidence_gain =
        declare_parameter<double>("bbs_verify_evidence_gain", rv.evidence_gain);
    reloc_params_.min_posterior =
        declare_parameter<double>("bbs_verify_min_posterior", rv.min_posterior);
    reloc_params_.nms_xy = declare_parameter<double>("bbs_verify_nms_xy", rv.nms_xy);
    reloc_params_.nms_yaw = declare_parameter<double>("bbs_verify_nms_yaw", rv.nms_yaw);
    reloc_params_.track_linear_window =
        declare_parameter<double>("bbs_verify_track_linear_window", rv.track_linear_window);
    reloc_params_.track_angular_window =
        declare_parameter<double>("bbs_verify_track_angular_window", rv.track_angular_window);
    validateBbsParams(bbs_params_, reloc_params_);
    global_loc_srv_ = create_service<std_srvs::srv::Empty>(
        "~/global_localization",
        std::bind(&LocalizationNode::onGlobalLocalization, this,
                  std::placeholders::_1, std::placeholders::_2));
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    // /map must stay transient_local+reliable to latch a one-shot map from map_server.
    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        std::bind(&LocalizationNode::onMap, this, std::placeholders::_1));
    // Sensor scans commonly arrive BEST_EFFORT, so match with SensorDataQoS.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationNode::onScan, this, std::placeholders::_1));
  } else {  // ndt3d
    const std::string pcd = declare_parameter<std::string>("map_pcd_path", "");
    const double res = declare_parameter<double>("ndt_resolution", 1.0);
    const int minpts = declare_parameter<int>("voxel_min_points", 5);
    prism_loc_core::NdtParams np3;
    np3.max_points = declare_parameter<int>("max_points", 500);
    np3.base_height = declare_parameter<double>("base_height", 0.0);
    validateNdtParams(res, minpts, np3);
    if (pcd.empty()) {
      RCLCPP_ERROR(get_logger(), "ndt3d: map_pcd_path is empty");
      throw std::runtime_error(
          "ndt3d: map_pcd_path is empty - set map_pcd_path to a valid .pcd map file "
          "(e.g. ros2 launch ... map_pcd_path:=/abs/path/to/map.pcd)");
    }
    auto pts = loadPcd(pcd);
    if (pts.empty()) {
      RCLCPP_ERROR(get_logger(), "ndt3d: failed to load PCD or empty: %s", pcd.c_str());
      throw std::runtime_error(
          "ndt3d: could not load a non-empty PCD map from '" + pcd +
          "' - check the file exists, is readable, and is a valid non-empty point cloud");
    }
    ndt_map_ = std::make_shared<prism_loc_core::NdtMap>(pts, res, minpts);
    ndt_model_ = std::make_shared<prism_loc_core::Ndt3DModel>(ndt_map_, np3);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "ndt3d: NDT map built (%zu voxels)", ndt_map_->numVoxels());
    points_topic_ = declare_parameter<std::string>("points_topic", "/points");
    // Sensor point clouds commonly arrive BEST_EFFORT, so match with SensorDataQoS.
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationNode::onPoints, this, std::placeholders::_1));
  }

  // initial_pose_* are declared unconditionally so YAML values always bind (and show up in
  // `ros2 param list`); they are used only when set_initial_pose is true.
  const bool set_initial_pose = declare_parameter<bool>("set_initial_pose", false);
  const Pose2D ip{declare_parameter<double>("initial_pose_x", 0.0),
                  declare_parameter<double>("initial_pose_y", 0.0),
                  declare_parameter<double>("initial_pose_yaw", 0.0)};
  if (set_initial_pose) {
    pf_->initializeGaussian(ip, Pose2D{0.5, 0.5, 0.25}, pp.max_particles / 2);
    filter_init_ = true; force_update_ = true;
  }
  // Startup watchdog: warn every 10 s about required inputs that have gone silent.
  watchdog_timer_ = create_wall_timer(std::chrono::seconds(10),
                                      std::bind(&LocalizationNode::onWatchdog, this));
  // Wall timer, so /diagnostics keeps flowing (and turns ERROR) when input stops or the
  // ROS clock is stuck.
  start_wall_ = last_input_wall_ = last_update_wall_ = Steady::now();
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&LocalizationNode::onDiagnostics, this));
  RCLCPP_INFO(get_logger(), "prism_loc up: backend=%s", backend_.c_str());
}

void LocalizationNode::onWatchdog() {
  std::lock_guard<std::mutex> lk(mutex_);
  bool all_seen = true;
  if (backend_ == "laser2d") {
    if (!map_seen_) {
      all_seen = false;
      RCLCPP_WARN(get_logger(),
                  "no messages on %s (%zu publishers) - check map_topic and that a map "
                  "server is publishing an OccupancyGrid (transient_local)",
                  map_topic_.c_str(), count_publishers(map_topic_));
    }
    if (!scan_seen_) {
      all_seen = false;
      RCLCPP_WARN(get_logger(),
                  "no messages on %s (%zu publishers) - check scan_topic and the sensor driver",
                  scan_topic_.c_str(), count_publishers(scan_topic_));
    }
  } else {
    if (!points_seen_) {
      all_seen = false;
      RCLCPP_WARN(get_logger(),
                  "no messages on %s (%zu publishers) - check points_topic and the sensor driver",
                  points_topic_.c_str(), count_publishers(points_topic_));
    }
  }
  if (all_seen) watchdog_timer_->cancel();
}

// Called with mutex_ held for every scan/cloud: tracks arrival on the steady clock so the
// silence threshold scales with the sensor's own rate.
void LocalizationNode::noteInput() {
  const auto now_w = Steady::now();
  if (any_input_) {
    const double dt = std::chrono::duration<double>(now_w - last_input_wall_).count();
    input_period_s_ = input_count_ < 2 ? dt : 0.9 * input_period_s_ + 0.1 * dt;
  }
  any_input_ = true;
  ++input_count_;
  last_input_wall_ = now_w;
}

// 1 Hz wall timer. Only reads state under the mutex; no TF lookups or filter work here.
void LocalizationNode::onDiagnostics() {
  std::lock_guard<std::mutex> lk(mutex_);
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  const auto now_w = Steady::now();
  auto secs = [&now_w](Steady::time_point t) {
    return std::chrono::duration<double>(now_w - t).count();
  };
  const double since_start = secs(start_wall_);
  const double since_input = any_input_ ? secs(last_input_wall_) : since_start;
  const bool laser = backend_ == "laser2d";
  const std::string& input_topic = laser ? scan_topic_ : points_topic_;
  const char* input_name = laser ? "scans" : "point clouds";
  char buf[320];
  std::vector<std::string> errors, warnings;

  // Errors: no localization output is possible.
  if (laser && !map_ready_ && since_start > startup_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "no map on %s after %.0f s", map_topic_.c_str(), since_start);
    errors.emplace_back(buf);
  }
  const double input_timeout =
      std::max(input_timeout_s_, input_timeout_periods_ * input_period_s_);
  if (!any_input_ && since_start > startup_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "no %s on %s after %.0f s", input_name, input_topic.c_str(),
                  since_start);
    errors.emplace_back(buf);
  } else if (any_input_ && since_input > input_timeout) {
    std::snprintf(buf, sizeof(buf), "%s stopped: none on %s for %.1f s (expected every %.2f s)",
                  input_name, input_topic.c_str(), since_input, input_period_s_);
    errors.emplace_back(buf);
  }
  if (!filter_init_ && since_start > startup_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "no pose seed: publish /initialpose%s (relocalization: %s)",
                  laser ? ", enable try_global_localization or call ~/global_localization" : "",
                  reloc_state_.c_str());
    errors.emplace_back(buf);
  }

  // Warnings: output is degraded or about to stop.
  if (odom_tf_failures_ > 0) {
    std::snprintf(buf, sizeof(buf), "odom TF %s->%s lookup failed %ld time(s) in the last second: %s",
                  odom_frame_.c_str(), base_frame_.c_str(), odom_tf_failures_,
                  last_tf_error_.substr(0, 120).c_str());
    warnings.emplace_back(buf);
  }
  if (sensor_tf_failures_ > 0) {
    std::snprintf(buf, sizeof(buf), "sensor TF lookup failed %ld time(s) in the last second",
                  sensor_tf_failures_);
    warnings.emplace_back(buf);
  }
  const bool reloc_running = relocalize_requested_ || (!filter_init_ && try_global_localization_);
  if (reloc_running && reloc_state_ != "idle" && reloc_state_ != "accepted")
    warnings.emplace_back("relocalization " + reloc_state_);
  if (min_neff_fraction_ > 0.0 && any_update_ && last_n_ > 0 &&
      last_neff_ < min_neff_fraction_ * static_cast<double>(last_n_)) {
    std::snprintf(buf, sizeof(buf), "low effective particle count %.0f of %zu at the last update",
                  last_neff_, last_n_);
    warnings.emplace_back(buf);
  }
  if (zero_stamp_inputs_ > 0) {
    std::snprintf(buf, sizeof(buf), "%ld %s with a zero header stamp dropped in the last second",
                  zero_stamp_inputs_, input_name);
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
  st.name = std::string(get_name()) + ": localization";
  st.hardware_id = get_namespace();
  std::string msg;
  for (const auto& e : errors) msg += (msg.empty() ? "" : "; ") + e;
  for (const auto& w : warnings) msg += (msg.empty() ? "" : "; ") + w;
  if (!errors.empty()) st.level = DS::ERROR;
  else if (!warnings.empty()) st.level = DS::WARN;
  else st.level = DS::OK;
  if (msg.empty()) {
    if (filter_init_ && have_map_odom_) msg = "localizing";
    else if (!filter_init_) msg = "starting: waiting for a pose seed";
    else msg = "starting: waiting for inputs";
  }
  st.message = msg;
  auto kv = [&st](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; st.values.push_back(p);
  };
  auto num = [](double v, const char* fmt) {
    char b[64];
    std::snprintf(b, sizeof(b), fmt, v);
    return std::string(b);
  };
  kv("backend", backend_);
  kv("map_received", map_ready_ ? "true" : "false");
  kv("filter_initialized", filter_init_ ? "true" : "false");
  kv("particles", std::to_string(pf_->particles().size()));
  kv("n_eff", last_n_ > 0 ? num(last_neff_, "%.1f") : std::string("n/a"));
  kv("covariance_trace_xy", num(last_cov_(0, 0) + last_cov_(1, 1), "%.6f"));
  kv("covariance_yaw", num(last_cov_(5, 5), "%.6f"));
  kv("input_rate_hz", num(input_period_s_ > 0.0 ? 1.0 / input_period_s_ : 0.0, "%.2f"));
  kv("seconds_since_last_input", num(since_input, "%.2f"));
  kv("seconds_since_last_update", num(any_update_ ? secs(last_update_wall_) : since_start, "%.2f"));
  kv("odom_tf_failures", std::to_string(odom_tf_failures_));
  kv("relocalization", reloc_state_);
  kv("use_sim_time", sim_time ? "true" : "false");
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = ros_now;
  arr.status.push_back(st);
  diag_pub_->publish(arr);
  odom_tf_failures_ = sensor_tf_failures_ = zero_stamp_inputs_ = 0;  // per-status counters
}

void LocalizationNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  map_seen_ = true;
  auto grid = fromOccupancyGrid(*msg);
  prism_loc_core::LaserParams lp;
  lp.max_beams = get_parameter_or("max_beams", 60);
  lp.z_hit = get_parameter_or("z_hit", 0.5);
  lp.z_rand = get_parameter_or("z_rand", 0.5);
  lp.sigma_hit = get_parameter_or("sigma_hit", 0.2);
  lp.max_dist = get_parameter_or("likelihood_max_dist", 2.0);
  laser_model_ = std::make_shared<prism_loc_core::Laser2DLikelihoodField>(grid, lp);
  grid_ = std::make_shared<prism_loc_core::GridMap>(grid);
  if (try_global_localization_ || relocalize_requested_) makeBbsMatcher();
  map_ready_ = true;
  RCLCPP_INFO(get_logger(), "laser2d: map received (%dx%d)", grid.width, grid.height);
}

bool LocalizationNode::lookupOdom(const rclcpp::Time& stamp, Pose2D& odom_base) {
  try {
    auto tf = lookupTransformWait(*tf_buffer_, odom_frame_, base_frame_,
                                  tf2_ros::fromRclcpp(stamp), *get_clock(), tf_wait_rule_);
    odom_base = toPose2D(tf.transform);
    return true;
  } catch (const std::exception& e) {
    ++odom_tf_failures_;
    last_tf_error_ = e.what();
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "odom TF: %s", e.what());
    return false;
  }
}

bool LocalizationNode::lookupSensor(const std::string& sensor_frame, Pose2D& sensor_in_base) {
  try {
    auto tf = tf_buffer_->lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero);
    sensor_in_base = toPose2D(tf.transform);
    return true;
  } catch (const std::exception& e) {
    ++sensor_tf_failures_;
    last_tf_error_ = e.what();
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "sensor TF: %s", e.what());
    return false;
  }
}

void LocalizationNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  scan_seen_ = true;
  noteInput();
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    // A zero stamp would make the odom lookup take the latest transform and the
    // broadcast map->odom TF land at t = transform_tolerance: drop the scan instead.
    ++zero_stamp_inputs_;
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onScan: dropping scan with a zero header stamp (frame '%s') - fix the "
                         "driver's timestamps",
                         msg->header.frame_id.c_str());
    return;
  }
  if (!map_ready_ || !laser_model_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onScan: waiting for a map on '%s' - none received yet; start a map "
                         "server or check map_topic",
                         map_topic_.c_str());
    return;
  }
  Pose2D sib;
  if (!lookupSensor(msg->header.frame_id, sib)) return;
  const auto scan = fromLaserScan(*msg, sib, laser_min_range_, laser_max_range_);

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (bbs_matcher_ && (relocalize_requested_ || (!filter_init_ && try_global_localization_)))
    runRelocalization(scan, stamp);

  if (!filter_init_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onScan: waiting for an initial pose - publish to /initialpose "
                         "(RViz '2D Pose Estimate'), or enable try_global_localization / call "
                         "the ~/global_localization service to relocalize");
    return;
  }
  laser_model_->setScan(scan);
  runUpdate(stamp);
}

// While verification is pending the existing particle set (if any) keeps
// tracking untouched; it is replaced only when one hypothesis is accepted.
void LocalizationNode::runRelocalization(const prism_loc_core::LaserScan2D& scan,
                                         const rclcpp::Time& stamp) {
  using prism_loc_core::RelocStatus;
  prism_loc_core::Pose2D odom;
  const bool need_odom = reloc_params_.verify_scans > 1;
  if (need_odom && !lookupOdom(stamp, odom)) return;  // scans must be odometry-chained
  if (!reloc_verifier_)
    reloc_verifier_ =
        std::make_unique<prism_loc_core::RelocalizationVerifier>(*bbs_matcher_, reloc_params_);
  RelocStatus st;
  if (!reloc_active_) {
    st = bbs_global_window_ ? reloc_verifier_->start(scan)
                            : reloc_verifier_->start(scan, bbs_center_);
  } else {
    st = reloc_verifier_->update(
        prism_loc_core::compose(prism_loc_core::inverse(reloc_last_odom_), odom), scan);
  }
  reloc_last_odom_ = odom;
  reloc_active_ = st == RelocStatus::kPending;
  switch (st) {
    case RelocStatus::kPending:
      reloc_state_ = "pending";
      RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 1000,
                           "global localization: verifying %zu hypotheses (%d/%d scans, best "
                           "posterior %.3f) - keep the robot moving",
                           reloc_verifier_->hypotheses().size(), reloc_verifier_->scansUsed(),
                           reloc_params_.verify_scans, reloc_verifier_->bestPosterior());
      break;
    case RelocStatus::kAccepted: {
      const prism_loc_core::Pose2D p = reloc_verifier_->pose();
      const int n = static_cast<int>(pf_->particles().size());
      pf_->initializeGaussian(p, Pose2D{0.2, 0.2, 0.1}, n > 0 ? n : 2000);
      filter_init_ = true;
      force_update_ = true;
      have_last_odom_ = false;
      relocalize_requested_ = false;
      reloc_state_ = "accepted";
      RCLCPP_INFO(get_logger(),
                  "global localization: seeded at (%.2f, %.2f, %.2f), posterior %.3f over %d "
                  "scans",
                  p.x, p.y, p.yaw, reloc_verifier_->bestPosterior(), reloc_verifier_->scansUsed());
      break;
    }
    case RelocStatus::kAmbiguous:
      reloc_state_ = "ambiguous";
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                  "global localization: AMBIGUOUS - best of %zu hypotheses holds only %.3f "
                  "(< %.3f) after %d scans; not committing, retrying (move the robot to a "
                  "more distinctive place)",
                  reloc_verifier_->hypotheses().size(), reloc_verifier_->bestPosterior(),
                  reloc_params_.min_posterior, reloc_verifier_->scansUsed());
      break;
    default:
      reloc_state_ = "no candidate";
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000,
                           "global localization: no confident pose this attempt");
      break;
  }
}

void LocalizationNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  points_seen_ = true;
  noteInput();
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    ++zero_stamp_inputs_;
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onPoints: dropping cloud with a zero header stamp (frame '%s') - fix "
                         "the driver's timestamps",
                         msg->header.frame_id.c_str());
    return;
  }
  if (!map_ready_ || !ndt_model_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onPoints: NDT map not ready - map_pcd_path failed to load; "
                         "check map_pcd_path");
    return;
  }
  if (!filter_init_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
                         "onPoints: waiting for an initial pose - publish to /initialpose, "
                         "or set set_initial_pose:=true with initial_pose_x/y/yaw");
    return;
  }
  Pose2D sib;
  if (!lookupSensor(msg->header.frame_id, sib)) return;
  ndt_model_->setCloud(fromPointCloud2(*msg), sib);
  runUpdate(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void LocalizationNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  Pose2D mean = toPose2D(msg->pose.pose);
  const double sx = std::sqrt(std::max(msg->pose.covariance[0], 0.25));
  const double sy = std::sqrt(std::max(msg->pose.covariance[7], 0.25));
  const double sa = std::sqrt(std::max(msg->pose.covariance[35], 0.0685));
  const int n = pf_->particles().empty() ? 2000 : static_cast<int>(pf_->particles().size());
  pf_->initializeGaussian(mean, Pose2D{sx, sy, sa}, std::max(n, 500));
  filter_init_ = true; force_update_ = true; have_last_odom_ = false;
  reloc_active_ = false;  // a manual pose abandons any verification in progress
  reloc_state_ = "idle";
  RCLCPP_INFO(get_logger(), "initialpose: (%.2f, %.2f, %.2f)", mean.x, mean.y, mean.yaw);
}

void LocalizationNode::makeBbsMatcher() {
  if (bbs_matcher_ || !grid_) return;
  bbs_matcher_ = std::make_shared<prism_loc_core::BranchAndBoundMatcher>(*grid_, bbs_params_);
  bbs_center_ = prism_loc_core::Pose2D{
      grid_->origin_x + 0.5 * grid_->width * grid_->resolution,
      grid_->origin_y + 0.5 * grid_->height * grid_->resolution, 0.0};
  if (bbs_global_window_) {
    const double hx = bbs_matcher_->globalHalfWindowX() * grid_->resolution;
    const double hy = bbs_matcher_->globalHalfWindowY() * grid_->resolution;
    if (bbs_matcher_->globalWindowCoversMap()) {
      RCLCPP_INFO(get_logger(),
                  "laser2d: BBS global-localization matcher ready (window +/-%.1f x +/-%.1f m "
                  "covers the whole map)", hx, hy);
    } else {
      RCLCPP_WARN(get_logger(),
                  "laser2d: BBS window capped at +/-%.1f x +/-%.1f m by bbs_max_linear_window; "
                  "relocalization is NOT global on this %.1f x %.1f m map", hx, hy,
                  grid_->width * grid_->resolution, grid_->height * grid_->resolution);
    }
  } else {
    RCLCPP_INFO(get_logger(), "laser2d: BBS matcher ready (+/-%.1f m about the map centre)",
                bbs_params_.linear_window);
  }
}

void LocalizationNode::onGlobalLocalization(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  std::lock_guard<std::mutex> lk(mutex_);
  relocalize_requested_ = true;
  reloc_active_ = false;  // start a fresh verification from the next scan
  if (!bbs_matcher_ && !grid_) {
    RCLCPP_WARN(get_logger(),
                "global localization requested before any map arrived - it will run once the "
                "map and a scan are received");
    return;
  }
  makeBbsMatcher();
  RCLCPP_INFO(get_logger(), "global localization requested via service");
}

void LocalizationNode::runUpdate(const rclcpp::Time& stamp) {
  Pose2D odom_base;
  if (!lookupOdom(stamp, odom_base)) return;
  if (!have_last_odom_) { last_odom_ = odom_base; have_last_odom_ = true; }

  const double dd = std::hypot(odom_base.x - last_odom_.x, odom_base.y - last_odom_.y);
  const double da = std::fabs(prism_loc_core::normalizeAngle(odom_base.yaw - last_odom_.yaw));
  const bool moved = dd >= update_min_d_ || da >= update_min_a_;

  if (force_update_ || moved) {
    pf_->predict(*motion_, last_odom_, odom_base);
    if (backend_ == "laser2d") pf_->correct(*laser_model_);
    else pf_->correct(*ndt_model_);
    if (force_update_) {
      // The update forced by a reseed (/initialpose, set_initial_pose, an accepted
      // relocalization) scores a fresh Gaussian cloud against one scan: its n_eff is
      // always tiny and says nothing about tracking health. Judge n_eff again only
      // after a normal motion-triggered update.
      last_n_ = 0;
    } else {
      last_neff_ = pf_->effectiveSampleSize();  // before resampling resets the weights
      last_n_ = pf_->particles().size();
    }
    pf_->resample();
    last_odom_ = odom_base;
    force_update_ = false;
    const Pose2D base_in_map = pf_->estimate(&last_cov_);
    map_odom_ = prism_loc_core::compose(base_in_map, prism_loc_core::inverse(odom_base));
    have_map_odom_ = true;
    any_update_ = true;
    last_update_wall_ = Steady::now();
  }
  if (have_map_odom_) publish(stamp, odom_base);
}

void LocalizationNode::publish(const rclcpp::Time& stamp, const Pose2D& odom_base) {
  // map -> odom TF
  if (tf_broadcast_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
    tf.header.frame_id = global_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_odom_.x;
    tf.transform.translation.y = map_odom_.y;
    tf2::Quaternion q; q.setRPY(0, 0, map_odom_.yaw);
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
  // pose with covariance (map frame): T_map_base = map_odom_ (+) odom_base
  const Pose2D base_in_map = prism_loc_core::compose(map_odom_, odom_base);
  geometry_msgs::msg::PoseWithCovarianceStamped ps;
  ps.header.stamp = stamp; ps.header.frame_id = global_frame_;
  ps.pose.pose.position.x = base_in_map.x;
  ps.pose.pose.position.y = base_in_map.y;
  tf2::Quaternion qp; qp.setRPY(0, 0, base_in_map.yaw);
  ps.pose.pose.orientation.x = qp.x(); ps.pose.pose.orientation.y = qp.y();
  ps.pose.pose.orientation.z = qp.z(); ps.pose.pose.orientation.w = qp.w();
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) ps.pose.covariance[r * 6 + c] = last_cov_(r, c);
  pose_pub_->publish(ps);
  // particle cloud
  nav2_msgs::msg::ParticleCloud pc;
  pc.header.stamp = stamp; pc.header.frame_id = global_frame_;
  pc.particles.reserve(pf_->particles().size());
  for (const auto& part : pf_->particles()) {
    nav2_msgs::msg::Particle q;
    q.pose.position.x = part.pose.x; q.pose.position.y = part.pose.y;
    tf2::Quaternion qq; qq.setRPY(0, 0, part.pose.yaw);
    q.pose.orientation.x = qq.x(); q.pose.orientation.y = qq.y();
    q.pose.orientation.z = qq.z(); q.pose.orientation.w = qq.w();
    q.weight = part.weight;
    pc.particles.push_back(q);
  }
  cloud_pub_->publish(pc);
}

}  // namespace prism_loc
