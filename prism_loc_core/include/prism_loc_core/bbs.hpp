#pragma once
#include <cmath>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/measurement_model.hpp"
namespace prism_loc_core {

struct BbsParams {
  double linear_window{10.0};        // ± metres searched around center
  double angular_window{M_PI};       // ± radians
  double angular_step{0.0175};       // ~1 deg
  double sigma_hit{0.2};             // probability-grid falloff
  double min_score_fraction{0.4};    // valid if best_score >= frac * used_beams
  int max_depth{6};                  // pyramid levels (coarsest block = 2^max_depth cells)
  int max_beams{120};                // scan subsample
};

struct BbsResult {
  Pose2D pose;
  double score{0.0};
  bool valid{false};
};

class BranchAndBoundMatcher {
 public:
  BranchAndBoundMatcher(const GridMap& grid, BbsParams params);
  BbsResult match(const LaserScan2D& scan, const Pose2D& center) const;

 private:
  double scoreLevel(const std::vector<std::pair<int, int>>& ep_cells,
                    int x_off, int y_off, int level) const;
  BbsParams params_;
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<std::vector<float>> pyramid_;  // pyramid_[level][y*width_ + x]
};

}  // namespace prism_loc_core
