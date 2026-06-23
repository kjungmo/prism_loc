#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/ndt_map.hpp"

namespace prism_loc {

prism_loc_core::GridMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg);
prism_loc_core::LaserScan2D fromLaserScan(const sensor_msgs::msg::LaserScan& msg,
                                          const prism_loc_core::Pose2D& sensor_in_base);
std::vector<Eigen::Vector3d> fromPointCloud2(const sensor_msgs::msg::PointCloud2& msg);
std::vector<Eigen::Vector3d> loadPcd(const std::string& path);
prism_loc_core::Pose2D toPose2D(const geometry_msgs::msg::Transform& t);
prism_loc_core::Pose2D toPose2D(const geometry_msgs::msg::Pose& p);

class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void runUpdate(const rclcpp::Time& stamp);
  bool lookupOdom(const rclcpp::Time& stamp, prism_loc_core::Pose2D& odom_base);
  bool lookupSensor(const std::string& sensor_frame, prism_loc_core::Pose2D& sensor_in_base);
  void publish(const rclcpp::Time& stamp, const prism_loc_core::Pose2D& odom_base);

  std::string backend_, global_frame_, odom_frame_, base_frame_;
  double update_min_d_{0.2}, update_min_a_{0.2}, transform_tolerance_{0.1};
  bool tf_broadcast_{true};

  std::unique_ptr<prism_loc_core::Rng> rng_;
  std::unique_ptr<prism_loc_core::ParticleFilter> pf_;
  std::unique_ptr<prism_loc_core::OdometryMotionModel> motion_;
  std::shared_ptr<prism_loc_core::Laser2DLikelihoodField> laser_model_;
  std::shared_ptr<prism_loc_core::Ndt3DModel> ndt_model_;
  std::shared_ptr<prism_loc_core::NdtMap> ndt_map_;

  bool map_ready_{false}, filter_init_{false}, force_update_{false};
  bool have_last_odom_{false}, have_map_odom_{false};
  prism_loc_core::Pose2D last_odom_, map_odom_;
  Eigen::Matrix<double, 6, 6> last_cov_ = Eigen::Matrix<double, 6, 6>::Zero();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
};

}  // namespace prism_loc
