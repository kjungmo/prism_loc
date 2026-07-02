#include "prism_loc_core/particle_filter.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
namespace prism_loc_core {

void ParticleFilter::initializeGaussian(const Pose2D& mean, const Pose2D& stddev, int n) {
  particles_.resize(n);
  const double w = 1.0 / std::max(1, n);
  for (auto& p : particles_) {
    p.pose.x = mean.x + rng_.gaussian(0.0, stddev.x);
    p.pose.y = mean.y + rng_.gaussian(0.0, stddev.y);
    p.pose.yaw = normalizeAngle(mean.yaw + rng_.gaussian(0.0, stddev.yaw));
    p.weight = w;
  }
}

void ParticleFilter::predict(const OdometryMotionModel& mm, const Pose2D& prev_odom,
                             const Pose2D& cur_odom) {
  mm.apply(particles_, prev_odom, cur_odom, rng_);
}

void ParticleFilter::normalizeWeights() {
  double sum = 0.0;
  for (const auto& p : particles_) sum += p.weight;
  if (sum <= 0.0) {
    const double w = 1.0 / std::max<size_t>(1, particles_.size());
    for (auto& p : particles_) p.weight = w;
    return;
  }
  for (auto& p : particles_) p.weight /= sum;
}

void ParticleFilter::correct(const MeasurementModel& model) {
  if (particles_.empty()) return;
  std::vector<double> logw(particles_.size());
  double max_log = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < particles_.size(); ++i) {
    logw[i] = model.logLikelihood(particles_[i].pose);
    max_log = std::max(max_log, logw[i]);
  }
  for (size_t i = 0; i < particles_.size(); ++i)
    particles_[i].weight = std::exp(logw[i] - max_log);
  normalizeWeights();
}

double ParticleFilter::effectiveSampleSize() const {
  double sum = 0.0, sum_sq = 0.0;
  for (const auto& p : particles_) { sum += p.weight; sum_sq += p.weight * p.weight; }
  if (sum_sq <= 0.0) return 0.0;
  return (sum * sum) / sum_sq;
}

void ParticleFilter::resampleLowVariance(int n) {
  if (particles_.empty() || n <= 0) return;
  std::vector<double> cum(particles_.size());
  double acc = 0.0;
  for (size_t i = 0; i < particles_.size(); ++i) { acc += particles_[i].weight; cum[i] = acc; }
  if (acc <= 0.0) return;
  ParticleSet out;
  out.reserve(n);
  const double step = acc / n;
  double u = rng_.uniform(0.0, step);
  size_t idx = 0;
  for (int m = 0; m < n; ++m) {
    const double target = u + m * step;
    while (idx + 1 < cum.size() && cum[idx] < target) ++idx;
    Particle p = particles_[idx];
    p.weight = 1.0 / n;
    out.push_back(p);
  }
  particles_ = std::move(out);
}

void ParticleFilter::resample() {
  if (particles_.empty()) return;
  const double n = static_cast<double>(particles_.size());
  if (effectiveSampleSize() >= params_.resample_threshold * n) return;  // healthy; skip

  // KLD-adaptive sampling (Fox): draw proportional-to-weight samples until the
  // KL bound — derived from the number of occupied bins — is met.
  std::vector<double> cum(particles_.size());
  double acc = 0.0;
  for (size_t i = 0; i < particles_.size(); ++i) { acc += particles_[i].weight; cum[i] = acc; }
  if (acc <= 0.0) return;

  auto draw = [&]() -> const Particle& {
    const double t = rng_.uniform(0.0, acc);
    auto it = std::lower_bound(cum.begin(), cum.end(), t);
    size_t i = static_cast<size_t>(it - cum.begin());
    if (i >= particles_.size()) i = particles_.size() - 1;
    return particles_[i];
  };

  std::unordered_set<std::int64_t> bins;
  ParticleSet out;
  out.reserve(params_.min_particles);
  int k = 0;
  double mx = static_cast<double>(params_.min_particles);
  const int max_n = params_.max_particles;
  while (static_cast<int>(out.size()) < max_n) {
    const Particle& s = draw();
    Particle np = s;
    np.weight = 1.0;  // normalized after the loop
    out.push_back(np);

    const std::int64_t bx = static_cast<std::int64_t>(std::floor(s.pose.x / params_.kld_bin_xy));
    const std::int64_t by = static_cast<std::int64_t>(std::floor(s.pose.y / params_.kld_bin_xy));
    const std::int64_t bt = static_cast<std::int64_t>(std::floor(s.pose.yaw / params_.kld_bin_yaw));
    const std::int64_t key = ((bx + (1 << 20)) << 42) | ((by + (1 << 20)) << 21) | (bt + (1 << 20));
    if (bins.insert(key).second) {
      ++k;
      if (k > 1) {
        const double kk = k - 1.0;
        const double t1 = 2.0 / (9.0 * kk);
        mx = (kk / (2.0 * params_.kld_err)) *
             std::pow(1.0 - t1 + std::sqrt(t1) * params_.kld_z, 3.0);
      }
    }
    if (static_cast<int>(out.size()) >= std::max<double>(mx, params_.min_particles)) break;
  }
  const double w = 1.0 / std::max<size_t>(1, out.size());
  for (auto& p : out) p.weight = w;
  particles_ = std::move(out);
}

Pose2D ParticleFilter::estimate(Eigen::Matrix<double, 6, 6>* cov) const {
  Pose2D e;
  if (particles_.empty()) return e;
  double sw = 0.0, mx = 0.0, my = 0.0, sc = 0.0, ss = 0.0;
  for (const auto& p : particles_) {
    sw += p.weight;
    mx += p.weight * p.pose.x;
    my += p.weight * p.pose.y;
    sc += p.weight * std::cos(p.pose.yaw);
    ss += p.weight * std::sin(p.pose.yaw);
  }
  if (sw <= 0.0) sw = 1.0;
  e.x = mx / sw; e.y = my / sw;
  e.yaw = std::atan2(ss, sc);
  if (cov) {
    cov->setZero();
    double cxx = 0, cxy = 0, cyy = 0, ctt = 0;
    for (const auto& p : particles_) {
      const double dx = p.pose.x - e.x, dy = p.pose.y - e.y;
      const double dt = normalizeAngle(p.pose.yaw - e.yaw);
      cxx += p.weight * dx * dx; cxy += p.weight * dx * dy;
      cyy += p.weight * dy * dy; ctt += p.weight * dt * dt;
    }
    (*cov)(0, 0) = cxx / sw; (*cov)(0, 1) = cxy / sw;
    (*cov)(1, 0) = cxy / sw; (*cov)(1, 1) = cyy / sw;
    (*cov)(5, 5) = ctt / sw;
    (*cov)(2, 2) = 1e6; (*cov)(3, 3) = 1e6; (*cov)(4, 4) = 1e6;  // z,roll,pitch unobserved
  }
  return e;
}

}  // namespace prism_loc_core
