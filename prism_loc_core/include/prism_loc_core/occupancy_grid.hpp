#pragma once
#include <cstdint>
#include <vector>
#include "prism_loc_core/types.hpp"
namespace prism_loc_core {

struct GridMap {
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::int8_t> data;  // row-major, [y*width + x], 0..100, -1 unknown
};

bool worldToMap(const GridMap& g, double wx, double wy, int& mx, int& my);
void mapToWorld(const GridMap& g, int mx, int my, double& wx, double& wy);

class LikelihoodField {
 public:
  LikelihoodField(const GridMap& grid, int occupied_threshold = 50, double max_dist = 2.0);
  double distanceAt(double wx, double wy) const;
  double maxDist() const { return max_dist_; }

 private:
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double max_dist_;
  std::vector<double> dist_;  // metric distance per cell, [y*width + x]
};

}  // namespace prism_loc_core
