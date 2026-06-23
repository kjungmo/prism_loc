#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
#include "prism_loc_fusion_ros/tf_util.hpp"
#include "prism_loc_fusion/so3.hpp"
#include <algorithm>
#include <cmath>
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
  eskf_ = std::make_unique<prism_loc_fusion::Eskf>(ep);

  if (declare_parameter<bool>("use_datum", false)) {
    geo_.setDatum({declare_parameter<double>("datum_lat", 0.0),
                   declare_parameter<double>("datum_lon", 0.0),
                   declare_parameter<double>("datum_alt", 0.0)});
  }

  const std::string pcd = declare_parameter<std::string>("map_pcd_path", "");
  ndt_ = std::make_unique<NdtRegistration>(
      declare_parameter<double>("ndt_resolution", 1.0),
      declare_parameter<double>("ndt_step_size", 0.1),
      declare_parameter<double>("ndt_epsilon", 0.01),
      declare_parameter<int>("ndt_max_iter", 30));
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcd.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *map) < 0 || map->empty()) {
    RCLCPP_ERROR(get_logger(), "fusion3d: failed to load PCD map: %s", pcd.c_str());
  } else {
    ndt_->setTarget(map);
    RCLCPP_INFO(get_logger(), "fusion3d: NDT map loaded (%zu pts)", map->size());
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onImu, this, std::placeholders::_1));
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("points_topic", "/points"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onPoints, this, std::placeholders::_1));
  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      declare_parameter<std::string>("gnss_topic", "/gnss"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onGnss, this, std::placeholders::_1));
  initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&FusionLocalizationNode::onInitialPose, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "prism_loc_fusion (fusion3d) up");
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
  const Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  if (!have_attitude_) { init_attitude_ = attitudeFromAccel(acc, initial_yaw_); have_attitude_ = true; }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (!last_imu_valid_) { last_imu_time_ = stamp; last_imu_valid_ = true; return; }
  const double dt = (stamp - last_imu_time_).seconds();
  last_imu_time_ = stamp;
  if (!tryInitialize()) return;
  if (dt <= 0.0 || dt > 0.5) return;
  eskf_->predict(acc, gyro, dt);
  publish(stamp);
}

void FusionLocalizationNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (!filter_init_ || !ndt_ || !ndt_->hasTarget()) return;
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "fusion3d: NDT rejected (conv=%d fit=%.3f)",
                         static_cast<int>(r.converged), r.fitness);
    return;
  }
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
  R.topLeftCorner<3, 3>() *= pose_pos_std_ * pose_pos_std_;
  R.bottomRightCorner<3, 3>() *= pose_rot_std_ * pose_rot_std_;
  eskf_->updatePose(r.pose.translation(), Eigen::Quaterniond(r.pose.linear()), R);
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (msg->status.status < gnss_min_status_) return;
  if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;
  if (msg->position_covariance[0] > gnss_max_pos_cov_) return;
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
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  init_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  const auto& o = msg->pose.pose.orientation;
  init_attitude_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
  have_position_ = have_attitude_ = true;
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
  try {
    auto t = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
    odom_base.translation() = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    odom_base.linear() = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                                            t.transform.rotation.y, t.transform.rotation.z).toRotationMatrix();
    out_tf = computeMapToOdom(map_base, odom_base);
    child = odom_frame_;
  } catch (const std::exception&) {
    out_tf = map_base;
    child = base_frame_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "fusion3d: no %s->%s; publishing %s->%s",
                         odom_frame_.c_str(), base_frame_.c_str(), global_frame_.c_str(), base_frame_.c_str());
  }
  tf.child_frame_id = child;
  tf.transform.translation.x = out_tf.translation().x();
  tf.transform.translation.y = out_tf.translation().y();
  tf.transform.translation.z = out_tf.translation().z();
  Eigen::Quaterniond oq(out_tf.linear());
  tf.transform.rotation.x = oq.x(); tf.transform.rotation.y = oq.y();
  tf.transform.rotation.z = oq.z(); tf.transform.rotation.w = oq.w();
  tf_broadcaster_->sendTransform(tf);

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
