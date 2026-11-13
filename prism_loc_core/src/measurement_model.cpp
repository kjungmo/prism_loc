#include "prism_loc_core/measurement_model.hpp"
#include <algorithm>
#include <cmath>
namespace prism_loc_core {

Laser2DLikelihoodField::Laser2DLikelihoodField(const GridMap& map, LaserParams params)
    : field_(map, 50, params.max_dist), params_(params) {}

double Laser2DLikelihoodField::logLikelihood(const Pose2D& base_in_map) const {
  const Pose2D T = compose(base_in_map, scan_.sensor_in_base);  // sensor in map
  const int n = static_cast<int>(scan_.ranges.size());
  if (n == 0) return 0.0;
  const int step = std::max(1, n / std::max(1, params_.max_beams));
  const double z_rand_term = params_.z_rand / std::max(scan_.range_max, 1e-6);
  const double two_sigma2 = 2.0 * params_.sigma_hit * params_.sigma_hit;
  double log_l = 0.0;
  for (int i = 0; i < n; i += step) {
    const double r = scan_.ranges[i];
    if (!std::isfinite(r) || r <= scan_.range_min || r >= scan_.range_max) continue;
    const double a = scan_.angle_min + i * scan_.angle_increment;
    const Eigen::Vector2d ep_sensor(r * std::cos(a), r * std::sin(a));
    const Eigen::Vector2d ep_map = transformPoint(T, ep_sensor);
    const double d = field_.distanceAt(ep_map.x(), ep_map.y());
    const double p = params_.z_hit * std::exp(-(d * d) / two_sigma2) + z_rand_term;
    log_l += std::log(std::max(p, 1e-12));
  }
  return log_l;
}

// --- appended to prism_loc_core/src/measurement_model.cpp ---
Ndt3DModel::Ndt3DModel(std::shared_ptr<const NdtMap> map, NdtParams params)
    : map_(std::move(map)), params_(params) {}

void Ndt3DModel::setCloud(const std::vector<Eigen::Vector3d>& pts_sensor,
                          const Pose2D& sensor_in_base) {
  pts_sensor_ = pts_sensor;
  sensor_in_base_ = sensor_in_base;
}

std::optional<double> Ndt3DModel::groundHeightAt(const Pose2D& p) const {
  if (!ground_) return std::nullopt;
  return ground_->heightAt(p.x, p.y, height_hint_);
}

std::optional<double> Ndt3DModel::groundHeightNearest(const Pose2D& p, double z_guess) const {
  if (!ground_) return std::nullopt;
  return ground_->heightNearest(p.x, p.y, z_guess);
}

bool Ndt3DModel::seedGround(const Pose2D& at, double z_guess) {
  ground_misses_ = 0;
  last_follow_ = GroundFollow::NoGround;
  const std::optional<double> g = groundHeightNearest(at, z_guess);
  height_hint_ = g ? *g : z_guess;
  return g.has_value();
}

GroundFollow Ndt3DModel::followGround(const Pose2D& estimate) {
  if (!ground_) return last_follow_ = GroundFollow::NoGround;
  if (const auto g = groundHeightAt(estimate)) {
    height_hint_ = *g;
    ground_misses_ = 0;
    return last_follow_ = GroundFollow::Followed;
  }
  if (ground_misses_ < 1000000000) ++ground_misses_;
  if (ground_misses_ < std::max(1, params_.ground_reanchor_after)) return last_follow_ = GroundFollow::Kept;
  if (const auto g = groundHeightNearest(estimate, height_hint_)) {
    height_hint_ = *g;
    ground_misses_ = 0;
    return last_follow_ = GroundFollow::Reanchored;
  }
  return last_follow_ = GroundFollow::NoGround;
}

double Ndt3DModel::cloudHeightAt(const Pose2D& p) const {
  if (!ground_) return params_.base_height;
  // No ground cell for this particle: it keeps the height of the current estimate.
  const std::optional<double> g = ground_->heightAt(p.x, p.y, height_hint_);
  return (g ? *g : height_hint_) + params_.base_height;
}

Eigen::Isometry3d Ndt3DModel::sensorInMap(const Pose2D& base_in_map) const {
  const Eigen::Isometry3d T_map_base = toIsometry3(base_in_map, cloudHeightAt(base_in_map));
  const Eigen::Isometry3d T_base_sensor = toIsometry3(sensor_in_base_, 0.0);
  return T_map_base * T_base_sensor;
}

double Ndt3DModel::logLikelihood(const Pose2D& base_in_map) const {
  if (!map_ || pts_sensor_.empty()) return 0.0;
  const Eigen::Isometry3d T = sensorInMap(base_in_map);
  const int n = static_cast<int>(pts_sensor_.size());
  const int step = std::max(1, n / std::max(1, params_.max_points));
  double log_l = 0.0;
  int evaluated = 0;
  for (int i = 0; i < n; i += step) {
    const Eigen::Vector3d pm = T * pts_sensor_[i];
    const double s = map_->score(pm);                 // 0 if empty voxel
    log_l += std::log(s + params_.score_floor);
    ++evaluated;
  }
  if (params_.effective_points > 0.0) return log_l / evaluated * params_.effective_points;
  return log_l;
}

NdtMatchStats Ndt3DModel::matchStats(const Pose2D& base_in_map) const {
  NdtMatchStats st;
  if (!map_ || pts_sensor_.empty()) return st;
  const Eigen::Isometry3d T = sensorInMap(base_in_map);
  const int n = static_cast<int>(pts_sensor_.size());
  const int step = std::max(1, n / std::max(1, params_.max_points));
  const double min_score = std::exp(-0.5 * kNdtMatchSigma * kNdtMatchSigma);
  for (int i = 0; i < n; i += step) {
    ++st.points;
    if (map_->matchScore(T * pts_sensor_[i]) >= min_score) ++st.matched;
  }
  return st;
}

}  // namespace prism_loc_core
