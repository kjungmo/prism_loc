#include "prism_loc_core/relocalization.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace prism_loc_core {

const char* toString(RelocStatus s) {
  switch (s) {
    case RelocStatus::kIdle: return "idle";
    case RelocStatus::kPending: return "pending";
    case RelocStatus::kAccepted: return "accepted";
    case RelocStatus::kAmbiguous: return "ambiguous";
    case RelocStatus::kNoCandidate: return "no-candidate";
  }
  return "?";
}

RelocalizationVerifier::RelocalizationVerifier(const BranchAndBoundMatcher& matcher,
                                               RelocVerifierParams params)
    : matcher_(matcher), params_(params) {}

void RelocalizationVerifier::reset() {
  hyps_.clear();
  status_ = RelocStatus::kIdle;
  best_pose_ = Pose2D{};
  best_posterior_ = 0.0;
  scans_ = 0;
}

RelocStatus RelocalizationVerifier::start(const LaserScan2D& scan) {
  return seed(matcher_.matchGlobalTopK(
      scan, BbsModeParams{std::max(1, params_.top_k), params_.nms_xy, params_.nms_yaw}));
}

RelocStatus RelocalizationVerifier::start(const LaserScan2D& scan, const Pose2D& center) {
  return seed(matcher_.matchTopK(
      scan, center, BbsModeParams{std::max(1, params_.top_k), params_.nms_xy, params_.nms_yaw}));
}

RelocStatus RelocalizationVerifier::seed(const std::vector<BbsResult>& modes) {
  reset();
  for (const BbsResult& r : modes) {
    if (!r.found || !r.valid || r.used_beams <= 0) continue;
    RelocHypothesis h;
    h.pose = r.pose;
    const double f = r.score / r.used_beams;
    h.log_weight = params_.evidence_gain * f;
    h.fraction_sum = f;
    h.scans = 1;
    hyps_.push_back(h);
  }
  if (hyps_.empty()) {
    status_ = RelocStatus::kNoCandidate;
    if (!modes.empty()) best_pose_ = modes.front().pose;
    return status_;
  }
  scans_ = 1;
  status_ = RelocStatus::kPending;
  normalizeAndDecide();
  return status_;
}

RelocStatus RelocalizationVerifier::update(const Pose2D& odom_delta, const LaserScan2D& scan) {
  if (status_ != RelocStatus::kPending) return status_;
  bool informative = false;
  for (RelocHypothesis& h : hyps_) {
    if (!h.alive) continue;
    const Pose2D pred = compose(h.pose, odom_delta);
    const BbsResult r = matcher_.matchLocal(scan, pred, params_.track_linear_window,
                                            params_.track_angular_window);
    if (r.used_beams <= 0) { h.pose = pred; continue; }  // empty scan: no evidence
    informative = true;
    if (!r.found) { h.alive = false; h.pose = pred; continue; }
    const double f = r.score / r.used_beams;
    h.pose = r.pose;
    h.log_weight += params_.evidence_gain * f;
    h.fraction_sum += f;
    h.scans += 1;
  }
  if (informative) ++scans_;
  normalizeAndDecide();
  return status_;
}

void RelocalizationVerifier::normalizeAndDecide() {
  double mx = -std::numeric_limits<double>::infinity();
  for (const RelocHypothesis& h : hyps_)
    if (h.alive) mx = std::max(mx, h.log_weight);
  double z = 0.0;
  for (RelocHypothesis& h : hyps_) {
    h.posterior = h.alive ? std::exp(h.log_weight - mx) : 0.0;
    z += h.posterior;
  }
  int best = -1;
  for (size_t i = 0; i < hyps_.size(); ++i) {
    if (z > 0.0) hyps_[i].posterior /= z;
    if (hyps_[i].alive && (best < 0 || hyps_[i].posterior > hyps_[best].posterior))
      best = static_cast<int>(i);
  }
  if (best < 0) {  // every hypothesis died
    status_ = RelocStatus::kNoCandidate;
    best_posterior_ = 0.0;
    return;
  }
  best_pose_ = hyps_[best].pose;
  best_posterior_ = hyps_[best].posterior;
  if (scans_ < std::max(1, params_.verify_scans)) return;  // still pending
  const double mean_f = hyps_[best].fraction_sum / std::max(1, hyps_[best].scans);
  if (best_posterior_ < params_.min_posterior)
    status_ = RelocStatus::kAmbiguous;
  else if (mean_f >= matcher_.params().min_score_fraction)
    status_ = RelocStatus::kAccepted;
  else
    status_ = RelocStatus::kNoCandidate;
}

}  // namespace prism_loc_core
