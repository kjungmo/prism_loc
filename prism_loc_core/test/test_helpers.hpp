#pragma once
#include <cmath>
#include <vector>
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/types.hpp"
namespace prism_loc_core { namespace test {

// Rectangular room: occupied border cells, free interior. res metres/cell.
inline GridMap makeRoomGrid(int w, int h, double res) {
  GridMap g; g.width = w; g.height = h; g.resolution = res;
  g.origin_x = 0.0; g.origin_y = 0.0; g.data.assign(w * h, 0);
  for (int x = 0; x < w; ++x) { g.data[x] = 100; g.data[(h - 1) * w + x] = 100; }
  for (int y = 0; y < h; ++y) { g.data[y * w] = 100; g.data[y * w + (w - 1)] = 100; }
  return g;
}

// Grid-march raycast producing a synthetic scan from base_in_map.
inline LaserScan2D raycastScan(const GridMap& g, const Pose2D& base_in_map,
                               const Pose2D& sensor_in_base, int n_beams, double max_range) {
  LaserScan2D scan;
  scan.angle_min = -M_PI; scan.angle_increment = 2.0 * M_PI / n_beams;
  scan.range_min = 0.0; scan.range_max = max_range;
  scan.sensor_in_base = sensor_in_base;
  const Pose2D T = compose(base_in_map, sensor_in_base);  // sensor in map
  const double step = g.resolution * 0.5;
  scan.ranges.resize(n_beams);
  for (int b = 0; b < n_beams; ++b) {
    const double a = scan.angle_min + b * scan.angle_increment;
    const double dirx = std::cos(T.yaw + a), diry = std::sin(T.yaw + a);
    float r = max_range;
    for (double t = step; t <= max_range; t += step) {
      int mx, my;
      if (!worldToMap(g, T.x + dirx * t, T.y + diry * t, mx, my)) { r = static_cast<float>(t); break; }
      if (g.data[my * g.width + mx] >= 50) { r = static_cast<float>(t); break; }
    }
    scan.ranges[b] = r;
  }
  return scan;
}

}}  // namespace prism_loc_core::test
