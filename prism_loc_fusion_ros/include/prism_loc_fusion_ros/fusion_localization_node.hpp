#pragma once
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
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/geodetic.hpp"
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
  bool tryInitialize();
  void publish(const rclcpp::Time& stamp);

  std::string global_frame_, odom_frame_, base_frame_;
  double ndt_max_fitness_, points_voxel_leaf_, transform_tolerance_;
  double pose_pos_std_, pose_rot_std_, gnss_max_pos_cov_, initial_yaw_{0.0};
  int gnss_min_status_{0};

  std::unique_ptr<prism_loc_fusion::Eskf> eskf_;
  prism_loc_fusion::GeodeticConverter geo_;
  std::unique_ptr<NdtRegistration> ndt_;

  bool filter_init_{false}, have_attitude_{false}, have_position_{false}, last_imu_valid_{false};
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
