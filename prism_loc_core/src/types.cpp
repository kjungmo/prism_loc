#include "prism_loc_core/types.hpp"
#include <cmath>
namespace prism_loc_core {

double normalizeAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

Pose2D compose(const Pose2D& a, const Pose2D& b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return Pose2D{a.x + c * b.x - s * b.y,
                a.y + s * b.x + c * b.y,
                normalizeAngle(a.yaw + b.yaw)};
}

Pose2D inverse(const Pose2D& a) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return Pose2D{-(c * a.x + s * a.y),
                -(-s * a.x + c * a.y),
                normalizeAngle(-a.yaw)};
}

Eigen::Vector2d transformPoint(const Pose2D& T, const Eigen::Vector2d& p) {
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return Eigen::Vector2d(T.x + c * p.x() - s * p.y(),
                         T.y + s * p.x() + c * p.y());
}

Eigen::Isometry3d toIsometry3(const Pose2D& p, double z) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  iso.translation() = Eigen::Vector3d(p.x, p.y, z);
  return iso;
}

}  // namespace prism_loc_core
