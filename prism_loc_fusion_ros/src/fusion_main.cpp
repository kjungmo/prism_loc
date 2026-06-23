#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prism_loc_fusion_ros::FusionLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
