#pragma once
#include <vector>
#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/types.hpp"
namespace prism_loc_core {

// Multi-hypothesis, multi-scan verification of a global relocalization.
//
// start(): the first scan's global branch-and-bound search keeps up to top_k
// distinct modes (BbsModeParams NMS) that pass the single-scan gate
// (score >= min_score_fraction * |E|). Each becomes a hypothesis with a uniform
// prior.
// update(): every hypothesis is moved by the odometry increment and re-matched
// locally (+-track_linear_window, +-track_angular_window) against the new scan;
// a hypothesis whose local window holds no feasible pose dies.
// Evidence: each scan adds evidence_gain * f to a hypothesis' log-weight, where
// f = score / |E| is its per-beam probability-grid score. The posterior mass of
// hypothesis k is softmax over the live hypotheses' log-weights.
// Decision after verify_scans scans (the first included): kAccepted when the
// best mass >= min_posterior and its mean f over the scans passes
// min_score_fraction; kAmbiguous when no mass reaches min_posterior; otherwise
// kNoCandidate. top_k = 1, verify_scans = 1 is the single-scan rule exactly.
// Defaults chosen by experiments/select_verify_params.py on the calibration run
// (seed 20261105, disjoint from the paper's study seed); see that script.
struct RelocVerifierParams {
  int top_k{8};
  double nms_xy{1.0};
  double nms_yaw{0.35};
  int verify_scans{9};
  double evidence_gain{40.0};
  double min_posterior{0.9};
  double track_linear_window{0.3};
  double track_angular_window{0.1};
  // Motion gate for update(): a scan counts as verification evidence only once the
  // odometry has moved at least min_translation (m) or min_rotation (rad) since the
  // last counted scan; uncounted scans only accumulate the odometry increment. A
  // stationary robot otherwise re-scores near-identical scans and the posterior grows
  // from repetition alone. 0 disables a criterion; both 0 (default) counts every scan.
  double min_translation{0.0};
  double min_rotation{0.0};
};

enum class RelocStatus { kIdle, kPending, kAccepted, kAmbiguous, kNoCandidate };

const char* toString(RelocStatus s);

struct RelocHypothesis {
  Pose2D pose;
  double log_weight{0.0};
  double fraction_sum{0.0};
  int scans{0};
  double posterior{0.0};
  bool alive{true};
};

class RelocalizationVerifier {
 public:
  // min_score_fraction is taken from the matcher's BbsParams.
  RelocalizationVerifier(const BranchAndBoundMatcher& matcher, RelocVerifierParams params);

  // Global (map-sized) search for the first scan's hypotheses.
  RelocStatus start(const LaserScan2D& scan);
  // Same, but over the matcher's +-linear_window about `center`.
  RelocStatus start(const LaserScan2D& scan, const Pose2D& center);
  // odom_delta: motion since the previous scan, in the previous base frame.
  RelocStatus update(const Pose2D& odom_delta, const LaserScan2D& scan);
  void reset();

  RelocStatus status() const { return status_; }
  // Scans given to update() that the motion gate did not count.
  int scansSkipped() const { return skipped_; }
  // Tracked pose of the most probable hypothesis (valid after start()).
  const Pose2D& pose() const { return best_pose_; }
  double bestPosterior() const { return best_posterior_; }
  int scansUsed() const { return scans_; }
  const std::vector<RelocHypothesis>& hypotheses() const { return hyps_; }

 private:
  RelocStatus seed(const std::vector<BbsResult>& modes);
  void normalizeAndDecide();
  const BranchAndBoundMatcher& matcher_;
  RelocVerifierParams params_;
  std::vector<RelocHypothesis> hyps_;
  RelocStatus status_{RelocStatus::kIdle};
  Pose2D best_pose_;
  double best_posterior_{0.0};
  int scans_{0};
  int skipped_{0};
  Pose2D pending_delta_;  // odometry since the last counted scan
};

}  // namespace prism_loc_core
