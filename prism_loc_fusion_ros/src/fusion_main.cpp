#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    // Two threads: the points callback group registers a scan (tens of milliseconds to
    // seconds) while the other group keeps taking the IMU, GNSS and the timers.
    auto node = std::make_shared<prism_loc_fusion_ros::FusionLocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("prism_loc_fusion"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
