#pragma once
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/measurement_model.hpp"
namespace prism_loc_core {

struct BbsParams {
  double linear_window{10.0};        // match(): ± metres searched around center (symmetric)
  double max_linear_window{50.0};    // matchGlobal(): cap on the per-axis ± half-window (m)
  double angular_window{M_PI};       // ± radians
  double angular_step{0.0175};       // ~1 deg
  double sigma_hit{0.2};             // probability-grid falloff
  double min_score_fraction{0.4};    // valid if best_score >= frac * used_beams
  int max_depth{6};                  // pyramid levels (coarsest block = 2^max_depth cells)
  int max_beams{120};                // scan subsample
  int occupied_threshold{50};        // robot cell value >= this is infeasible
};

struct BbsResult {
  Pose2D pose;
  double score{0.0};
  bool valid{false};
  // True when the searched translation set contains every cell of the map
  // (always the case for matchGlobal unless max_linear_window caps it).
  bool window_covers_map{false};
};

// Test-only perturbation of the coarse-level (level > 0) bounds, used by the
// negative-control tests to show that the branch-and-bound vs exhaustive
// agreement check detects an inadmissible bound. The default is the identity;
// production code never sets it.
struct BoundMutationForTesting {
  double coarse_scale{1.0};         // multiply every coarse-level bound
  bool coarse_drop_off_map{false};  // coarse levels drop off-map cells like leaves
};

// Branch-and-bound correlative scan matcher (Hess et al. 2016) over the set
//   Theta x {-Lx..Lx} x {-Ly..Ly}   (cell offsets from a center pose),
// restricted to FEASIBLE candidates: the robot position must lie in an on-map
// cell whose occupancy value is below occupied_threshold (unknown cells, value
// -1, are feasible). The returned pose maximizes the level-0 score over the
// feasible set exactly (ties: first leaf reached in best-first order).
class BranchAndBoundMatcher {
 public:
  BranchAndBoundMatcher(const GridMap& grid, BbsParams params);

  // Symmetric ±linear_window search about `center`.
  BbsResult match(const LaserScan2D& scan, const Pose2D& center) const;

  // Map-sized search: center = centre of cell (width/2, height/2), per-axis
  // half-window = ceil-to-cover the whole grid, capped by max_linear_window.
  BbsResult matchGlobal(const LaserScan2D& scan) const;

  // Reference O(|Theta| (2Lx+1)(2Ly+1)) exhaustive search over the same feasible
  // set as match(); used to verify branch-and-bound optimality.
  BbsResult matchExhaustive(const LaserScan2D& scan, const Pose2D& center) const;

  // The center and per-axis half-windows (cells) matchGlobal() uses.
  Pose2D globalCenter() const;
  int globalHalfWindowX() const;
  int globalHalfWindowY() const;
  bool globalWindowCoversMap() const;

  void setBoundMutationForTesting(BoundMutationForTesting m) { mutation_ = m; }

 private:
  BbsResult search(const LaserScan2D& scan, const Pose2D& center, int Lx, int Ly,
                   bool exhaustive) const;
  double scoreLevel(const std::vector<std::pair<int, int>>& ep_cells,
                    int x_off, int y_off, int level) const;
  bool feasible(const Pose2D& center, int x_off, int y_off) const;
  bool coversMap(const Pose2D& center, int Lx, int Ly) const;
  BbsParams params_;
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<std::uint8_t> blocked_;          // 1 = occupied cell
  std::vector<std::vector<float>> pyramid_;  // pyramid_[level][y*width_ + x]
  BoundMutationForTesting mutation_;
};

}  // namespace prism_loc_core
