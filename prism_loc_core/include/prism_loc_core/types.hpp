#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_core {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Particle {
  Pose2D pose;
  double weight{0.0};
};
using ParticleSet = std::vector<Particle>;

double normalizeAngle(double a);
Pose2D compose(const Pose2D& a, const Pose2D& b);
Pose2D inverse(const Pose2D& a);
Eigen::Vector2d transformPoint(const Pose2D& T, const Eigen::Vector2d& p);
Eigen::Isometry3d toIsometry3(const Pose2D& p, double z = 0.0);

}  // namespace prism_loc_core
