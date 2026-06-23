#include "prism_loc_fusion/so3.hpp"
#include <algorithm>
#include <cmath>
namespace prism_loc_fusion {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return m;
}

Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-8) {
    Eigen::Quaterniond q(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z());
    q.normalize();
    return q;
  }
  const Eigen::Vector3d axis = phi / theta;
  const double h = 0.5 * theta, s = std::sin(h);
  return Eigen::Quaterniond(std::cos(h), axis.x() * s, axis.y() * s, axis.z() * s);
}

Eigen::Vector3d so3Log(const Eigen::Quaterniond& q_in) {
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;  // shortest path
  const Eigen::Vector3d v(q.x(), q.y(), q.z());
  const double n = v.norm();
  if (n < 1e-8) return 2.0 * v;
  const double w = std::min(1.0, std::max(-1.0, q.w()));
  const double theta = 2.0 * std::atan2(n, w);
  return (theta / n) * v;
}

}  // namespace prism_loc_fusion
