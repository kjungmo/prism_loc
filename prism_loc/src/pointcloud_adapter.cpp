#include "prism_loc/localization_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
namespace prism_loc {

std::vector<Eigen::Vector3d> fromPointCloud2(const sensor_msgs::msg::PointCloud2& msg) {
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(msg.width * msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    const float x = *ix, y = *iy, z = *iz;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      pts.emplace_back(x, y, z);
  }
  return pts;
}

std::vector<Eigen::Vector3d> loadPcd(const std::string& path) {
  std::vector<Eigen::Vector3d> pts;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) < 0) return pts;
  pts.reserve(cloud.size());
  for (const auto& p : cloud)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      pts.emplace_back(p.x, p.y, p.z);
  return pts;
}

}  // namespace prism_loc
