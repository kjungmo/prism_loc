#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<prism_loc_fusion_ros::FusionLocalizationNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("prism_loc_fusion"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
