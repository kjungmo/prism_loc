#include "prism_loc/localization_node.hpp"
#include <cmath>
namespace prism_loc {

static double quatYaw(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

prism_loc_core::Pose2D toPose2D(const geometry_msgs::msg::Transform& t) {
  return {t.translation.x, t.translation.y,
          quatYaw(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)};
}
prism_loc_core::Pose2D toPose2D(const geometry_msgs::msg::Pose& p) {
  return {p.position.x, p.position.y,
          quatYaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)};
}

prism_loc_core::GridMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg) {
  prism_loc_core::GridMap g;
  g.width = static_cast<int>(msg.info.width);
  g.height = static_cast<int>(msg.info.height);
  g.resolution = msg.info.resolution;
  g.origin_x = msg.info.origin.position.x;
  g.origin_y = msg.info.origin.position.y;
  g.data.assign(msg.data.begin(), msg.data.end());
  return g;
}

prism_loc_core::LaserScan2D fromLaserScan(const sensor_msgs::msg::LaserScan& msg,
                                          const prism_loc_core::Pose2D& sensor_in_base) {
  prism_loc_core::LaserScan2D s;
  s.ranges = msg.ranges;
  s.angle_min = msg.angle_min;
  s.angle_increment = msg.angle_increment;
  s.range_min = msg.range_min;
  s.range_max = msg.range_max;
  s.sensor_in_base = sensor_in_base;
  return s;
}

}  // namespace prism_loc
