#include "prism_loc/localization_node.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>
namespace prism_loc {
using prism_loc_core::Pose2D;

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("prism_loc", options) {
  backend_ = declare_parameter<std::string>("backend", "laser2d");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  update_min_d_ = declare_parameter<double>("update_min_d", 0.2);
  update_min_a_ = declare_parameter<double>("update_min_a", 0.2);
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  tf_broadcast_ = declare_parameter<bool>("tf_broadcast", true);

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
    declare_parameter<int>("max_beams", 60);
    declare_parameter<double>("z_hit", 0.5);
    declare_parameter<double>("z_rand", 0.5);
    declare_parameter<double>("sigma_hit", 0.2);
    declare_parameter<double>("likelihood_max_dist", 2.0);
    laser_min_range_ = declare_parameter<double>("laser_min_range", 0.0);
    laser_max_range_ = declare_parameter<double>("laser_max_range", 0.0);
    try_global_localization_ = declare_parameter<bool>("try_global_localization", false);
    bbs_params_.linear_window = declare_parameter<double>("bbs_linear_window", 10.0);
    bbs_params_.angular_window = declare_parameter<double>("bbs_angular_window", M_PI);
    bbs_params_.angular_step = declare_parameter<double>("bbs_angular_step", 0.0175);
    bbs_params_.max_depth = declare_parameter<int>("bbs_max_depth", 6);
    bbs_params_.max_beams = declare_parameter<int>("bbs_max_beams", 120);
    bbs_params_.min_score_fraction = declare_parameter<double>("bbs_min_score_fraction", 0.4);
    bbs_params_.sigma_hit = get_parameter_or("sigma_hit", 0.2);
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
    prism_loc_core::NdtParams np;
    np.max_points = declare_parameter<int>("max_points", 500);
    np.base_height = declare_parameter<double>("base_height", 0.0);
    ndt_model_ = std::make_shared<prism_loc_core::Ndt3DModel>(ndt_map_, np);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "ndt3d: NDT map built (%zu voxels)", ndt_map_->numVoxels());
    points_topic_ = declare_parameter<std::string>("points_topic", "/points");
    // Sensor point clouds commonly arrive BEST_EFFORT, so match with SensorDataQoS.
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationNode::onPoints, this, std::placeholders::_1));
  }

  if (declare_parameter<bool>("set_initial_pose", false)) {
    Pose2D ip{declare_parameter<double>("initial_pose_x", 0.0),
              declare_parameter<double>("initial_pose_y", 0.0),
              declare_parameter<double>("initial_pose_yaw", 0.0)};
    pf_->initializeGaussian(ip, Pose2D{0.5, 0.5, 0.25}, pp.max_particles / 2);
    filter_init_ = true; force_update_ = true;
  }
  // Startup watchdog: warn every 10 s about required inputs that have gone silent.
  watchdog_timer_ = create_wall_timer(std::chrono::seconds(10),
                                      std::bind(&LocalizationNode::onWatchdog, this));
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
  if (try_global_localization_) {
    bbs_matcher_ = std::make_shared<prism_loc_core::BranchAndBoundMatcher>(grid, bbs_params_);
    bbs_center_ = prism_loc_core::Pose2D{
        grid.origin_x + 0.5 * grid.width * grid.resolution,
        grid.origin_y + 0.5 * grid.height * grid.resolution, 0.0};
    RCLCPP_INFO(get_logger(), "laser2d: BBS global-localization matcher ready");
  }
  map_ready_ = true;
  RCLCPP_INFO(get_logger(), "laser2d: map received (%dx%d)", grid.width, grid.height);
}

bool LocalizationNode::lookupOdom(const rclcpp::Time& stamp, Pose2D& odom_base) {
  try {
    auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp,
                                          rclcpp::Duration::from_seconds(0.1));
    odom_base = toPose2D(tf.transform);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom TF: %s", e.what());
    return false;
  }
}

bool LocalizationNode::lookupSensor(const std::string& sensor_frame, Pose2D& sensor_in_base) {
  try {
    auto tf = tf_buffer_->lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero);
    sensor_in_base = toPose2D(tf.transform);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "sensor TF: %s", e.what());
    return false;
  }
}

void LocalizationNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  scan_seen_ = true;
  if (!map_ready_ || !laser_model_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onScan: waiting for a map on '%s' - none received yet; start a map "
                         "server or check map_topic",
                         map_topic_.c_str());
    return;
  }
  Pose2D sib;
  if (!lookupSensor(msg->header.frame_id, sib)) return;
  const auto scan = fromLaserScan(*msg, sib, laser_min_range_, laser_max_range_);

  if (bbs_matcher_ && (relocalize_requested_ || (!filter_init_ && try_global_localization_))) {
    const prism_loc_core::BbsResult r = bbs_matcher_->match(scan, bbs_center_);
    if (r.valid) {
      const int n = static_cast<int>(pf_->particles().size());
      pf_->initializeGaussian(r.pose, Pose2D{0.2, 0.2, 0.1}, n > 0 ? n : 2000);
      filter_init_ = true;
      force_update_ = true;
      have_last_odom_ = false;
      relocalize_requested_ = false;
      RCLCPP_INFO(get_logger(), "global localization: seeded at (%.2f, %.2f, %.2f) score %.1f",
                  r.pose.x, r.pose.y, r.pose.yaw, r.score);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "global localization: no confident pose this scan");
    }
  }

  if (!filter_init_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onScan: waiting for an initial pose - publish to /initialpose "
                         "(RViz '2D Pose Estimate'), or enable try_global_localization / call "
                         "the ~/global_localization service to relocalize");
    return;
  }
  laser_model_->setScan(scan);
  runUpdate(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void LocalizationNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  points_seen_ = true;
  if (!map_ready_ || !ndt_model_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "onPoints: NDT map not ready - map_pcd_path failed to load; "
                         "check map_pcd_path");
    return;
  }
  if (!filter_init_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
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
  RCLCPP_INFO(get_logger(), "initialpose: (%.2f, %.2f, %.2f)", mean.x, mean.y, mean.yaw);
}

void LocalizationNode::onGlobalLocalization(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  std::lock_guard<std::mutex> lk(mutex_);
  relocalize_requested_ = true;
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
    pf_->resample();
    last_odom_ = odom_base;
    force_update_ = false;
    const Pose2D base_in_map = pf_->estimate(&last_cov_);
    map_odom_ = prism_loc_core::compose(base_in_map, prism_loc_core::inverse(odom_base));
    have_map_odom_ = true;
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
