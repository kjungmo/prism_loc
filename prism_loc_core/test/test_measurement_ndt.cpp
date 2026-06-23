#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/ndt_map.hpp"
using namespace prism_loc_core;

// Build a "wall" of points (a plane x=3) as the map; observe it from a known pose.
static std::vector<Eigen::Vector3d> wallCloud() {
  std::vector<Eigen::Vector3d> pts;
  for (double y = -2; y <= 2; y += 0.05)
    for (double z = 0; z <= 1.0; z += 0.05)
      pts.emplace_back(3.0, y, z);
  return pts;
}

TEST(NdtModel, AlignedOutscoresTranslated) {
  auto map_pts = wallCloud();
  auto map = std::make_shared<NdtMap>(map_pts, /*res=*/0.5, /*min_points=*/3);
  Ndt3DModel model(map, NdtParams{});

  // Robot at origin, sensor = base. The observed cloud (in sensor frame) equals
  // the map points seen from truth pose {0,0,0}: identical to map coords.
  Pose2D truth{0,0,0}, sensor_in_base{0,0,0};
  model.setCloud(map_pts, sensor_in_base);

  const double aligned = model.logLikelihood(truth);
  const double shifted = model.logLikelihood(Pose2D{0.8, 0.0, 0.0});
  EXPECT_GT(aligned, shifted);
}
