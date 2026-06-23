#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "test_helpers.hpp"
using namespace prism_loc_core;

TEST(Integration, Laser2dConvergesToTruth) {
  GridMap g = test::makeRoomGrid(60, 60, 0.1);   // 6m x 6m room
  Pose2D truth{3.0, 3.0, 0.4};
  Pose2D sensor_in_base{0,0,0};
  LaserScan2D scan = test::raycastScan(g, truth, sensor_in_base, 180, 10.0);

  Laser2DLikelihoodField model(g, LaserParams{});
  model.setScan(scan);

  Rng rng(2024);
  ParticleFilterParams pp; pp.min_particles = 800; pp.max_particles = 3000;
  ParticleFilter pf(pp, rng);
  pf.initializeGaussian(truth, Pose2D{0.5, 0.5, 0.3}, 2000);  // spread around truth

  OdometryMotionModel mm(MotionParams{0.05,0.05,0.05,0.05});
  const Pose2D still{0,0,0};
  // Zero odom delta each step: convergence is driven by measurement weighting +
  // KLD resampling concentrating the initial 2000-particle spread onto the truth.
  for (int it = 0; it < 30; ++it) {
    pf.predict(mm, still, still);
    pf.correct(model);
    pf.resample();
  }
  Pose2D e = pf.estimate();
  EXPECT_NEAR(e.x, truth.x, 0.20);
  EXPECT_NEAR(e.y, truth.y, 0.20);
  EXPECT_NEAR(normalizeAngle(e.yaw - truth.yaw), 0.0, 0.15);
}

TEST(Integration, Ndt3dConvergesToTruth) {
  std::vector<Eigen::Vector3d> map_pts;
  for (double y = -3; y <= 3; y += 0.05) for (double z = 0; z <= 1.2; z += 0.05) {
    map_pts.emplace_back(3.0, y, z);     // wall x=+3
    map_pts.emplace_back(-3.0, y, z);    // wall x=-3
  }
  for (double x = -3; x <= 3; x += 0.05) for (double z = 0; z <= 1.2; z += 0.05) {
    map_pts.emplace_back(x, 3.0, z);     // wall y=+3
    map_pts.emplace_back(x, -3.0, z);    // wall y=-3
  }
  auto map = std::make_shared<NdtMap>(map_pts, 0.5, 4);
  Ndt3DModel model(map, NdtParams{});
  Pose2D truth{0.3, -0.2, 0.2}, sensor_in_base{0,0,0};

  // Observed cloud in sensor frame = map points expressed in truth's sensor frame.
  std::vector<Eigen::Vector3d> obs;
  Eigen::Isometry3d T_map_sensor = toIsometry3(truth, 0.0);
  Eigen::Isometry3d T_sensor_map = T_map_sensor.inverse();
  for (const auto& p : map_pts) obs.push_back(T_sensor_map * p);
  model.setCloud(obs, sensor_in_base);

  Rng rng(99);
  ParticleFilterParams pp; pp.min_particles = 800; pp.max_particles = 3000;
  ParticleFilter pf(pp, rng);
  pf.initializeGaussian(Pose2D{0,0,0}, Pose2D{0.6,0.6,0.4}, 2000);
  OdometryMotionModel mm(MotionParams{0.05,0.05,0.05,0.05});
  const Pose2D still{0,0,0};
  for (int it = 0; it < 30; ++it) { pf.predict(mm, still, still); pf.correct(model); pf.resample(); }
  Pose2D e = pf.estimate();
  EXPECT_NEAR(e.x, truth.x, 0.30);
  EXPECT_NEAR(e.y, truth.y, 0.30);
}
