#include "prism_loc_core/motion_model.hpp"
#include <cmath>
namespace prism_loc_core {

Pose2D OdometryMotionModel::sample(const Pose2D& particle, const Pose2D& prev_odom,
                                   const Pose2D& cur_odom, Rng& rng) const {
  const double dx = cur_odom.x - prev_odom.x;
  const double dy = cur_odom.y - prev_odom.y;
  const double trans = std::hypot(dx, dy);
  double rot1, rot2;
  if (trans < 1e-3) {                       // pure-rotation guard
    rot1 = 0.0;
    rot2 = normalizeAngle(cur_odom.yaw - prev_odom.yaw);
  } else {
    rot1 = normalizeAngle(std::atan2(dy, dx) - prev_odom.yaw);
    rot2 = normalizeAngle(cur_odom.yaw - prev_odom.yaw - rot1);
  }
  auto sample_var = [&](double var) { return rng.gaussian(0.0, std::sqrt(std::max(var, 0.0))); };
  const double r1 = rot1 - sample_var(p_.alpha1 * rot1 * rot1 + p_.alpha2 * trans * trans);
  const double tr = trans - sample_var(p_.alpha3 * trans * trans +
                                       p_.alpha4 * (rot1 * rot1 + rot2 * rot2));
  const double r2 = rot2 - sample_var(p_.alpha1 * rot2 * rot2 + p_.alpha2 * trans * trans);
  Pose2D out;
  out.x = particle.x + tr * std::cos(particle.yaw + r1);
  out.y = particle.y + tr * std::sin(particle.yaw + r1);
  out.yaw = normalizeAngle(particle.yaw + r1 + r2);
  return out;
}

void OdometryMotionModel::apply(ParticleSet& particles, const Pose2D& prev_odom,
                                const Pose2D& cur_odom, Rng& rng) const {
  for (auto& part : particles) part.pose = sample(part.pose, prev_odom, cur_odom, rng);
}

}  // namespace prism_loc_core
