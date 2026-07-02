#include <gtest/gtest.h>
#include "prism_loc_core/measurement_model.hpp"
#include "test_helpers.hpp"
using namespace prism_loc_core;

TEST(LaserModel, TruthOutscoresDisplaced) {
  GridMap g = test::makeRoomGrid(40, 40, 0.1);     // 4m x 4m room
  Pose2D truth{2.0, 2.0, 0.3};                     // centre-ish
  Pose2D sensor_in_base{0,0,0};
  LaserScan2D scan = test::raycastScan(g, truth, sensor_in_base, 180, 8.0);

  Laser2DLikelihoodField model(g, LaserParams{});
  model.setScan(scan);
  const double at_truth = model.logLikelihood(truth);
  const double displaced = model.logLikelihood(Pose2D{2.5, 1.6, 0.3});
  EXPECT_GT(at_truth, displaced);
}
