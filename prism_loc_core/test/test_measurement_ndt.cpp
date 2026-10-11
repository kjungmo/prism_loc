#include <gtest/gtest.h>
#include <cmath>
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

// --- matchStats: the share of scan points that lie on a mapped surface (a diagnostic) ---

// The cloud is the map seen from the true pose: every point matches. Moved off the wall
// by 2 cm (the wall is a few millimetres thick in the NDT map) or by 0.8 m (cells without
// a Gaussian): none does.
TEST(NdtModel, MatchFractionIsOneOnTheMapAndZeroOffIt) {
  auto map_pts = wallCloud();
  auto map = std::make_shared<NdtMap>(map_pts, /*res=*/0.5, /*min_points=*/3);
  Ndt3DModel model(map, NdtParams{});
  model.setCloud(map_pts, Pose2D{0, 0, 0});
  const NdtMatchStats on = model.matchStats(Pose2D{0, 0, 0});
  ASSERT_EQ(map_pts.size(), 1600u);
  EXPECT_EQ(on.points, 534);  // 1600 points, max_points 500: every 3rd
  EXPECT_EQ(on.matched, on.points);
  EXPECT_DOUBLE_EQ(on.fraction(), 1.0);
  const NdtMatchStats near = model.matchStats(Pose2D{0.02, 0, 0});
  EXPECT_EQ(near.points, 534);
  EXPECT_EQ(near.matched, 0);
  EXPECT_DOUBLE_EQ(near.fraction(), 0.0);
  EXPECT_EQ(model.matchStats(Pose2D{0.8, 0, 0}).matched, 0);
}

// One map cell with a known Gaussian: six points at +-0.1 m along each axis around
// (0.5, 0.5, 0.5) give a covariance of (0.1^2 / 3) I, sigma = 0.0577 m. A point counts as
// matched up to kNdtMatchSigma = 3 sigma from the mean, in any direction.
TEST(NdtModel, MatchThresholdIsThreeSigma) {
  const Eigen::Vector3d c(0.5, 0.5, 0.5);
  std::vector<Eigen::Vector3d> cell;
  for (int ax = 0; ax < 3; ++ax)
    for (double s : {-0.1, 0.1}) {
      Eigen::Vector3d p = c;
      p[ax] += s;
      cell.push_back(p);
    }
  auto map = std::make_shared<NdtMap>(cell, /*res=*/1.0, /*min_points=*/5);
  Ndt3DModel model(map, NdtParams{});
  const double sigma = 0.1 / std::sqrt(3.0);
  ASSERT_DOUBLE_EQ(kNdtMatchSigma, 3.0);
  const Eigen::Vector3d diag = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  for (const Eigen::Vector3d& dir : {Eigen::Vector3d(Eigen::Vector3d::UnitX()), Eigen::Vector3d(Eigen::Vector3d::UnitZ()), diag}) {
    model.setCloud({Eigen::Vector3d(c + 2.99 * sigma * dir)}, Pose2D{0, 0, 0});
    EXPECT_EQ(model.matchStats(Pose2D{0, 0, 0}).matched, 1);
    model.setCloud({Eigen::Vector3d(c + 3.01 * sigma * dir)}, Pose2D{0, 0, 0});
    EXPECT_EQ(model.matchStats(Pose2D{0, 0, 0}).matched, 0);
    EXPECT_EQ(model.matchStats(Pose2D{0, 0, 0}).points, 1);
  }
  // the mean itself, and a point in a neighbouring cell that has no Gaussian
  model.setCloud({c, Eigen::Vector3d(1.5, 0.5, 0.5)}, Pose2D{0, 0, 0});
  const NdtMatchStats st = model.matchStats(Pose2D{0, 0, 0});
  EXPECT_EQ(st.points, 2);
  EXPECT_EQ(st.matched, 1);
  EXPECT_DOUBLE_EQ(st.fraction(), 0.5);
}

// matchStats looks at exactly the points logLikelihood scores: off the map every scored
// point contributes log(score_floor), so the log-likelihood counts them.
TEST(NdtModel, MatchStatsScoresTheSameSubsampleAsLogLikelihood) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  std::vector<Eigen::Vector3d> far;
  for (int i = 0; i < 1000; ++i) far.emplace_back(50.0 + 0.01 * i, 60.0, 5.0);
  for (int max_points : {300, 1000, 5000, 1}) {
    NdtParams p;
    p.max_points = max_points;
    Ndt3DModel model(map, p);
    model.setCloud(far, Pose2D{0, 0, 0});
    const NdtMatchStats st = model.matchStats(Pose2D{0, 0, 0});
    EXPECT_EQ(st.matched, 0);
    EXPECT_NEAR(model.logLikelihood(Pose2D{0, 0, 0}), st.points * std::log(p.score_floor), 1e-6) << max_points;
  }
  NdtParams p;
  p.max_points = 300;  // 1000 / 300 -> every 3rd point: 334
  Ndt3DModel model(map, p);
  model.setCloud(far, Pose2D{0, 0, 0});
  EXPECT_EQ(model.matchStats(Pose2D{0, 0, 0}).points, 334);
}

// No cloud yet, or an empty one: nothing evaluated, and the fraction is 0, not NaN.
TEST(NdtModel, MatchStatsOfAnEmptyCloud) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  Ndt3DModel model(map, NdtParams{});
  NdtMatchStats st = model.matchStats(Pose2D{0, 0, 0});
  EXPECT_EQ(st.points, 0);
  EXPECT_EQ(st.matched, 0);
  EXPECT_EQ(st.fraction(), 0.0);
  model.setCloud({}, Pose2D{0, 0, 0});
  st = model.matchStats(Pose2D{1, 2, 0.3});
  EXPECT_EQ(st.points, 0);
  EXPECT_EQ(st.fraction(), 0.0);
}

// The points are placed as logLikelihood places them: the sensor pose in base_link is
// planar (x, y, yaw) and the sensor sits at base_height in the map. A floor at map
// z = 0.25 seen from a lidar 0.7 m above it matches with base_height 0.95 (the lidar's z in
// the map) and not with base_height 0.25.
TEST(NdtModel, MatchStatsUsesSensorPoseAndBaseHeight) {
  std::vector<Eigen::Vector3d> floor;
  for (double x = 0.02; x < 4.0; x += 0.05)  // off the cell borders: the cloud goes through a round trip
    for (double y = 0.02; y < 4.0; y += 0.05) floor.emplace_back(x, y, 0.25);
  auto map = std::make_shared<NdtMap>(floor, 1.0, 5);
  const Pose2D base{1.0, 0.5, 0.2}, sensor_in_base{0.3, 0.1, 0.4};
  const Eigen::Isometry3d map_from_sensor = toIsometry3(base, 0.95) * toIsometry3(sensor_in_base, 0.0);
  std::vector<Eigen::Vector3d> cloud;
  for (const auto& p : floor) cloud.push_back(map_from_sensor.inverse() * p);
  NdtParams right;
  right.base_height = 0.95;
  right.max_points = 100000;
  Ndt3DModel model(map, right);
  model.setCloud(cloud, sensor_in_base);
  const NdtMatchStats st = model.matchStats(base);
  EXPECT_EQ(st.points, static_cast<int>(floor.size()));
  EXPECT_EQ(st.matched, st.points);
  // the sensor offset matters: without it the same cloud lands elsewhere on the floor,
  // still on it (a floor matches anywhere) ...
  model.setCloud(cloud, Pose2D{0, 0, 0});
  EXPECT_GT(model.matchStats(base).fraction(), 0.5);
  // ... but the height does: 0.7 m under the lidar there is no floor in the map
  NdtParams wrong = right;
  wrong.base_height = 0.25;
  Ndt3DModel low(map, wrong);
  low.setCloud(cloud, sensor_in_base);
  EXPECT_EQ(low.matchStats(base).matched, 0);
}
