#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "prism_loc_core/ground_height_map.hpp"
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

// --- effective_points: the log-likelihood as the mean per-point log score times a fixed count ---

// Half the cloud on the wall, half far off it, at the identity pose with sensor = base.
static std::vector<Eigen::Vector3d> halfOnWall(int n) {
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < n; ++i)
    pts.emplace_back(i % 2 == 0 ? Eigen::Vector3d(3.0, -1.0 + 0.002 * i, 0.5)
                                : Eigen::Vector3d(50.0 + 0.01 * i, 60.0, 5.0));
  return pts;
}

TEST(NdtModel, EffectivePointsZeroIsThePlainSum) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  const auto cloud = halfOnWall(600);  // max_points 500 < 600: step 1, every point scored
  Ndt3DModel model(map, NdtParams{});
  model.setCloud(cloud, Pose2D{0, 0, 0});
  double expect = 0.0;
  for (const auto& p : cloud) expect += std::log(map->score(p) + NdtParams{}.score_floor);
  EXPECT_DOUBLE_EQ(model.logLikelihood(Pose2D{0, 0, 0}), expect);
}

TEST(NdtModel, EffectivePointsScalesTheMeanPerPointLogScore) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  const auto cloud = halfOnWall(600);
  NdtParams plain;
  Ndt3DModel m0(map, plain);
  m0.setCloud(cloud, Pose2D{0, 0, 0});
  const double sum = m0.logLikelihood(Pose2D{0, 0, 0});
  NdtParams p;
  p.effective_points = 10.0;
  Ndt3DModel m(map, p);
  m.setCloud(cloud, Pose2D{0, 0, 0});
  EXPECT_NEAR(m.logLikelihood(Pose2D{0, 0, 0}), sum / 600.0 * 10.0, 1e-9);
  // The spread between poses shrinks by effective_points / points_evaluated.
  const double off0 = m0.logLikelihood(Pose2D{0, 0, 0}) - m0.logLikelihood(Pose2D{0.8, 0, 0});
  const double off = m.logLikelihood(Pose2D{0, 0, 0}) - m.logLikelihood(Pose2D{0.8, 0, 0});
  EXPECT_NEAR(off, off0 * 10.0 / 600.0, 1e-9);
}

// The subsample holds max_points .. 2*max_points-1 points depending on the cloud size
// (integer step). The tempered value does not depend on that count.
TEST(NdtModel, EffectivePointsRemovesDependenceOnSubsampleSize) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  NdtParams p;
  p.max_points = 100;
  p.effective_points = 10.0;
  // Same per-point scores in every cloud: all far off the map.
  std::vector<double> values;
  for (int n : {100, 199, 200, 299, 1000}) {
    std::vector<Eigen::Vector3d> far;
    for (int i = 0; i < n; ++i) far.emplace_back(50.0 + 0.01 * i, 60.0, 5.0);
    Ndt3DModel model(map, p);
    model.setCloud(far, Pose2D{0, 0, 0});
    values.push_back(model.logLikelihood(Pose2D{0, 0, 0}));
  }
  for (double v : values) EXPECT_NEAR(v, 10.0 * std::log(p.score_floor), 1e-9);
}

TEST(NdtModel, EffectivePointsEdgeCases) {
  auto map = std::make_shared<NdtMap>(wallCloud(), 0.5, 3);
  NdtParams p;
  p.effective_points = 10.0;
  Ndt3DModel model(map, p);
  EXPECT_EQ(model.logLikelihood(Pose2D{0, 0, 0}), 0.0);  // no cloud yet
  model.setCloud({}, Pose2D{0, 0, 0});
  EXPECT_EQ(model.logLikelihood(Pose2D{0, 0, 0}), 0.0);  // empty cloud
  model.setCloud({Eigen::Vector3d(3.0, 0.0, 0.5)}, Pose2D{0, 0, 0});  // fewer points than max_points
  const double one = model.logLikelihood(Pose2D{0, 0, 0});
  EXPECT_TRUE(std::isfinite(one));
  EXPECT_NEAR(one, 10.0 * std::log(map->score(Eigen::Vector3d(3.0, 0.0, 0.5)) + p.score_floor), 1e-9);
}

// --- min_sigma shapes only the likelihood: matchStats judges the cell without it ---

TEST(NdtModel, MatchStatsIgnoresMinSigma) {
  const auto wall = wallCloud();
  auto plain = std::make_shared<NdtMap>(wall, 0.5, 3);
  auto floored = std::make_shared<NdtMap>(wall, 0.5, 3, /*min_sigma=*/0.10);
  Ndt3DModel mp(plain, NdtParams{}), mf(floored, NdtParams{});
  mp.setCloud(wall, Pose2D{0, 0, 0});
  mf.setCloud(wall, Pose2D{0, 0, 0});
  for (double dx : {0.0, 0.005, 0.02, 0.1, 0.25, 0.8}) {
    const NdtMatchStats a = mp.matchStats(Pose2D{dx, 0, 0});
    const NdtMatchStats b = mf.matchStats(Pose2D{dx, 0, 0});
    EXPECT_EQ(a.points, b.points) << dx;
    EXPECT_EQ(a.matched, b.matched) << dx;
  }
  EXPECT_EQ(mf.matchStats(Pose2D{0.1, 0, 0}).matched, 0);  // 0.1 m off the plane: not on it
}

TEST(NdtModel, MinSigmaChangesTheLikelihood) {
  const auto wall = wallCloud();
  auto plain = std::make_shared<NdtMap>(wall, 0.5, 3);
  auto floored = std::make_shared<NdtMap>(wall, 0.5, 3, 0.10);
  Ndt3DModel mp(plain, NdtParams{}), mf(floored, NdtParams{});
  mp.setCloud(wall, Pose2D{0, 0, 0});
  mf.setCloud(wall, Pose2D{0, 0, 0});
  EXPECT_NE(mp.logLikelihood(Pose2D{0.1, 0, 0}), mf.logLikelihood(Pose2D{0.1, 0, 0}));
  EXPECT_GT(mf.logLikelihood(Pose2D{0.1, 0, 0}), mp.logLikelihood(Pose2D{0.1, 0, 0}));
}

// --- ground height from the map: the cloud is placed at ground(x, y) + base_height ---

namespace {
constexpr double kSlope = 0.2;  // ground z = kSlope * x

// Map of a ramp (z = 0.2 x for x in [0, 30], y in [-8, 8]) with a wall at x = 30, a wall across
// it at x = 13.5 (so that x is observable) and walls at y = +-6 (1.5 m high above the
// ground). 0.1 m spacing.
std::vector<Eigen::Vector3d> rampMap() {
  std::vector<Eigen::Vector3d> pts;
  for (double x = 0.05; x < 30.0; x += 0.1)
    for (double y = -8.0 + 0.05; y < 8.0; y += 0.1) pts.emplace_back(x, y, kSlope * x);
  for (double x = 0.05; x < 30.0; x += 0.1)
    for (double h = 0.1; h <= 1.5; h += 0.1)
      for (double y : {-6.0, 6.0}) pts.emplace_back(x, y, kSlope * x + h);
  for (double y = -8.0 + 0.05; y < 8.0; y += 0.1)
    for (double h = 0.1; h <= 3.0; h += 0.1) pts.emplace_back(29.95, y, kSlope * 29.95 + h);
  for (double y = -8.0 + 0.05; y < 8.0; y += 0.1)
    for (double h = 0.1; h <= 1.5; h += 0.1) pts.emplace_back(13.5, y, kSlope * 13.5 + h);
  return pts;
}

// The map points within 10 m of the robot, as seen from base_in_map with the level sensor
// base_height above the local ground.
std::vector<Eigen::Vector3d> viewFrom(const std::vector<Eigen::Vector3d>& map, const Pose2D& base,
                                      double base_height) {
  const Eigen::Isometry3d T = toIsometry3(base, kSlope * base.x + base_height);
  std::vector<Eigen::Vector3d> cloud;
  for (const auto& p : map)
    if (std::hypot(p.x() - base.x, p.y() - base.y) < 10.0) cloud.push_back(T.inverse() * p);
  return cloud;
}

struct Ramp {
  std::vector<Eigen::Vector3d> pts = rampMap();
  std::shared_ptr<NdtMap> ndt = std::make_shared<NdtMap>(pts, 1.0, 5);
  std::shared_ptr<GroundHeightMap> ground = std::make_shared<GroundHeightMap>(pts);
  static NdtParams params(double base_height) {
    NdtParams p;
    p.base_height = base_height;
    p.max_points = 1 << 20;  // score every point
    return p;
  }
};
}  // namespace

// Moving up the ramp, a constant height puts the cloud ever further from the map and the
// match falls; with the ground map it stays on the map, mid-cell as well as on cell borders.
TEST(NdtGround, ConstantHeightDegradesUpTheRampGroundMapDoesNot) {
  Ramp r;
  const double bh = 1.0;  // cloud frame 1 m above the local ground
  for (double x : {0.0, 1.0, 2.5, 5.5, 10.5, 20.5}) {
    const Pose2D truth{x, 0.0, 0.0};
    const auto cloud = viewFrom(r.pts, truth, bh);
    ASSERT_GT(cloud.size(), 5000u);
    Ndt3DModel constant(r.ndt, Ramp::params(bh));  // right at x = 0 (ground at 0), wrong above
    constant.setCloud(cloud, Pose2D{0, 0, 0});
    Ndt3DModel grounded(r.ndt, Ramp::params(bh));
    grounded.setGroundMap(r.ground);
    grounded.setHeightHint(kSlope * x);
    grounded.setCloud(cloud, Pose2D{0, 0, 0});
    const double fc = constant.matchStats(truth).fraction();
    if (x == 0.0) {
      EXPECT_GT(fc, 0.9);  // the ground is at z = 0 here: the constant height is right
      continue;            // (and the grid, at the map's edge, has two cells to average)
    }
    const double fg = grounded.matchStats(truth).fraction();
    EXPECT_GT(fg, 0.9) << "x = " << x;
    EXPECT_LT(fc, 0.3) << "x = " << x;
    EXPECT_GT(grounded.logLikelihood(truth), constant.logLikelihood(truth)) << "x = " << x;
  }
}

// The likelihood along x peaks at the truth when the truth is in the middle of a cell, scanned
// over one cell either side in 5 cm steps: the ground height seen by the particles is
// continuous, so no false peak sits at a cell edge. With and without the NDT min_sigma.
TEST(NdtGround, LikelihoodPeaksAtTheTruthMidCellOnTheRamp) {
  Ramp r;
  for (double min_sigma : {0.0, 0.1}) {
   for (double tx : {10.5, 10.95, 10.05}) {  // a cell centre, close to a cell edge on either side
    const Pose2D truth{tx, 0.0, 0.0};
    const auto cloud = viewFrom(r.pts, truth, 1.0);
    auto ndt = std::make_shared<NdtMap>(r.pts, 1.0, 5, min_sigma);
    Ndt3DModel m(ndt, Ramp::params(1.0));
    m.setGroundMap(r.ground);
    m.setHeightHint(kSlope * truth.x);
    m.setCloud(cloud, Pose2D{0, 0, 0});
    double best = -1e300, best_x = 0.0;
    for (int k = -20; k <= 20; ++k) {
      const double x = truth.x + 0.05 * k;
      const double ll = m.logLikelihood(Pose2D{x, truth.y, 0.0});
      if (ll > best) { best = ll; best_x = x; }
    }
    EXPECT_NEAR(best_x, truth.x, 0.05) << "min_sigma " << min_sigma << " x " << tx;
   }
  }
}

// A particle one metre up the ramp from the truth sees the cloud placed on its own
// ground, not the robot's: the likelihood still peaks at the true pose.
TEST(NdtGround, LikelihoodPeaksAtTruthOnTheRamp) {
  Ramp r;
  const Pose2D truth{12.5, 1.3, 0.0};  // mid-cell
  const auto cloud = viewFrom(r.pts, truth, 1.0);
  Ndt3DModel m(r.ndt, Ramp::params(1.0));
  m.setGroundMap(r.ground);
  m.setHeightHint(kSlope * truth.x);
  m.setCloud(cloud, Pose2D{0, 0, 0});
  const double at = m.logLikelihood(truth);
  for (const Pose2D& p : {Pose2D{13.5, 1.3, 0.0}, Pose2D{11.5, 1.3, 0.0}, Pose2D{12.5, 2.3, 0.0},
                          Pose2D{12.5, 1.3, 0.1}, Pose2D{12.3, 1.3, 0.0}, Pose2D{12.7, 1.3, 0.0}})
    EXPECT_GT(at, m.logLikelihood(p));
}

// logLikelihood and matchStats place the cloud with the same transform (sensorInMap).
TEST(NdtGround, LikelihoodAndMatchStatsPlaceTheCloudAlike) {
  Ramp r;
  const Pose2D base{8.3, -0.7, 0.4}, sib{0.3, 0.1, 0.2};
  NdtParams p = Ramp::params(1.0);
  p.max_points = 700;  // a subsample, so the two must agree on that too
  Ndt3DModel m(r.ndt, p);
  m.setGroundMap(r.ground);
  m.setHeightHint(kSlope * 8.3);
  const auto cloud = viewFrom(r.pts, Pose2D{8.0, 0.0, 0.0}, 1.0);
  m.setCloud(cloud, sib);
  for (const Pose2D& pose : {base, Pose2D{9.1, 0.2, 0.0}, Pose2D{30.0, 30.0, 1.0}}) {
    const Eigen::Isometry3d T = m.sensorInMap(pose);
    const int step = std::max(1, static_cast<int>(cloud.size()) / p.max_points);
    double ll = 0.0;
    int n = 0, matched = 0;
    const double min_score = std::exp(-0.5 * kNdtMatchSigma * kNdtMatchSigma);
    for (size_t i = 0; i < cloud.size(); i += step) {
      ll += std::log(r.ndt->score(T * cloud[i]) + p.score_floor);
      matched += r.ndt->matchScore(T * cloud[i]) >= min_score;
      ++n;
    }
    EXPECT_DOUBLE_EQ(m.logLikelihood(pose), ll);
    const NdtMatchStats st = m.matchStats(pose);
    EXPECT_EQ(st.points, n);
    EXPECT_EQ(st.matched, matched);
    EXPECT_DOUBLE_EQ(T.translation().z(), m.cloudHeightAt(pose));
  }
}

// No ground map (never set, or set to null): the model is the one of before, bit for bit,
// and a height hint is ignored.
TEST(NdtGround, NullGroundMapIsTheConstantHeightModel) {
  const auto map_pts = wallCloud();
  auto map = std::make_shared<NdtMap>(map_pts, 0.5, 3);
  NdtParams p;
  p.base_height = 0.4;
  Ndt3DModel plain(map, p), nulled(map, p), hinted(map, p);
  nulled.setGroundMap(nullptr);
  hinted.setGroundMap(nullptr);
  hinted.setHeightHint(7.0);
  const Pose2D sib{0.1, 0.0, 0.0};
  for (Ndt3DModel* m : {&plain, &nulled, &hinted}) m->setCloud(map_pts, sib);
  EXPECT_FALSE(hinted.hasGroundMap());
  for (const Pose2D& pose : {Pose2D{0, 0, 0}, Pose2D{0.3, -0.2, 0.05}, Pose2D{4, 4, 1}}) {
    const double ref = plain.logLikelihood(pose);
    EXPECT_EQ(nulled.logLikelihood(pose), ref);
    EXPECT_EQ(hinted.logLikelihood(pose), ref);
    EXPECT_EQ(hinted.matchStats(pose).matched, plain.matchStats(pose).matched);
    // and it is what the formula says
    const Eigen::Isometry3d T = toIsometry3(pose, 0.4) * toIsometry3(sib, 0.0);
    double ll = 0.0;
    const int step = std::max(1, static_cast<int>(map_pts.size()) / p.max_points);
    for (size_t i = 0; i < map_pts.size(); i += step) ll += std::log(map->score(T * map_pts[i]) + p.score_floor);
    EXPECT_DOUBLE_EQ(ref, ll);
    EXPECT_EQ(hinted.cloudHeightAt(pose), 0.4);
    EXPECT_FALSE(hinted.groundHeightAt(pose).has_value());
  }
}

// A particle with no ground cell keeps the height of the estimate: hint + base_height.
TEST(NdtGround, ParticleWithoutGroundCellUsesTheHint) {
  Ramp r;
  // ground map of the first 10 m of the ramp only
  std::vector<Eigen::Vector3d> head;
  for (const auto& p : r.pts)
    if (p.x() < 10.0) head.push_back(p);
  auto partial = std::make_shared<GroundHeightMap>(head);
  const Pose2D far{20.0, 0.0, 0.0};
  const auto cloud = viewFrom(r.pts, far, 1.0);
  Ndt3DModel g(r.ndt, Ramp::params(1.0));
  g.setGroundMap(partial);
  g.setHeightHint(3.9);
  g.setCloud(cloud, Pose2D{0, 0, 0});
  EXPECT_FALSE(g.groundHeightAt(far).has_value());
  EXPECT_DOUBLE_EQ(g.cloudHeightAt(far), 3.9 + 1.0);
  Ndt3DModel c(r.ndt, Ramp::params(4.9));  // the constant model at that height
  c.setCloud(cloud, Pose2D{0, 0, 0});
  EXPECT_EQ(g.logLikelihood(far), c.logLikelihood(far));
  EXPECT_EQ(g.matchStats(far).matched, c.matchStats(far).matched);
  // inside the mapped part the map answers instead
  EXPECT_DOUBLE_EQ(g.cloudHeightAt(Pose2D{5.0, 0.0, 0.0}), 3.9 + 1.0);  // ground ~1 m: 2.9 m off the hint, rejected
  g.setHeightHint(1.1);
  EXPECT_NEAR(g.cloudHeightAt(Pose2D{5.0, 0.0, 0.0}), kSlope * 5.0 + 1.0, 0.25);
}

// Two levels 3 m apart over the same xy: the hint picks the level; a hint between them
// (rejected by max_step) falls back to the hint.
TEST(NdtGround, TwoLevelsFollowTheHint) {
  std::vector<Eigen::Vector3d> pts;
  for (double x = 0.05; x < 10.0; x += 0.1)
    for (double y = 0.05; y < 10.0; y += 0.1) {
      pts.emplace_back(x, y, 0.0);
      pts.emplace_back(x, y, 3.0);
    }
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  auto ground = std::make_shared<GroundHeightMap>(pts);
  Ndt3DModel m(map, Ramp::params(0.8));
  m.setGroundMap(ground);
  const Pose2D at{5.0, 5.0, 0.0};
  m.setHeightHint(0.1);
  EXPECT_DOUBLE_EQ(m.cloudHeightAt(at), 0.0 + 0.8);
  m.setHeightHint(2.9);
  EXPECT_DOUBLE_EQ(m.cloudHeightAt(at), 3.0 + 0.8);
  m.setHeightHint(1.5);  // 1.5 m from both: no candidate within max_step
  EXPECT_FALSE(m.groundHeightAt(at).has_value());
  EXPECT_DOUBLE_EQ(m.cloudHeightAt(at), 1.5 + 0.8);
  // the reseed query picks the nearest level whatever the guess
  EXPECT_DOUBLE_EQ(*m.groundHeightNearest(at, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(*m.groundHeightNearest(at, 2.0), 3.0);
  EXPECT_FALSE(m.groundHeightNearest(Pose2D{50.0, 50.0, 0.0}, 0.0).has_value());
  Ndt3DModel none(map, NdtParams{});
  EXPECT_FALSE(none.groundHeightNearest(at, 0.0).has_value());
}

// --- followGround: the hint follows the estimate, and re-anchors when it has lost the ground ---

namespace {
// Flat ground at `z` over x in [x0, x1), y in [0, 10).
void flat(std::vector<Eigen::Vector3d>& pts, double x0, double x1, double z) {
  for (double x = x0 + 0.05; x < x1; x += 0.1)
    for (double y = 0.05; y < 10.0; y += 0.1) pts.emplace_back(x, y, z);
}
}  // namespace

TEST(NdtGround, FollowGroundFollowsTheEstimateOnARamp) {
  Ramp r;
  Ndt3DModel m(r.ndt, Ramp::params(1.0));
  m.setGroundMap(r.ground);
  EXPECT_TRUE(m.seedGround(Pose2D{5.5, 0.0, 0.0}, 0.0));
  EXPECT_NEAR(m.heightHint(), kSlope * 5.5, 0.02);
  for (double x = 5.5; x < 25.0; x += 0.2) {
    EXPECT_EQ(m.followGround(Pose2D{x, 0.0, 0.0}), GroundFollow::Followed);
    EXPECT_NEAR(m.heightHint(), kSlope * x, 0.02) << x;  // within 2 cm: a wall crosses the ramp at x = 13.5
    EXPECT_EQ(m.groundMisses(), 0);
  }
  EXPECT_EQ(m.lastGroundFollow(), GroundFollow::Followed);
}

// A stretch without ground cells while the ground rises 0.6 m (more than ground_max_step):
// Kept for four updates, NoGround while no cell answers, Reanchored at the first cell on the
// far side, then Followed again.
TEST(NdtGround, FollowGroundReanchorsAfterAStretchWithoutGround) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 10.0, 0.0);
  flat(pts, 14.0, 30.0, 0.6);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  Ndt3DModel m(map, Ramp::params(1.0));
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  ASSERT_TRUE(m.seedGround(Pose2D{5.0, 5.0, 0.0}, 0.0));
  std::vector<GroundFollow> seen;
  for (double x = 8.0; x < 20.0; x += 0.5) {
    seen.push_back(m.followGround(Pose2D{x, 5.0, 0.0}));
    if (seen.back() == GroundFollow::Kept) { EXPECT_LT(m.groundMisses(), 5); }
  }
  int reanchored = 0;
  size_t first_miss = seen.size();
  for (size_t i = 0; i < seen.size(); ++i) {
    if (seen[i] != GroundFollow::Followed && first_miss == seen.size()) first_miss = i;
    reanchored += seen[i] == GroundFollow::Reanchored;
  }
  ASSERT_LT(first_miss, seen.size());
  for (size_t i = 0; i < 4; ++i) EXPECT_EQ(seen[first_miss + i], GroundFollow::Kept) << i;
  EXPECT_EQ(seen[first_miss + 4], GroundFollow::NoGround);  // fifth miss: nothing to anchor to yet
  EXPECT_EQ(reanchored, 1);
  EXPECT_EQ(seen.back(), GroundFollow::Followed);
  EXPECT_NEAR(m.heightHint(), 0.6, 1e-9);
  EXPECT_EQ(m.groundMisses(), 0);
  // while the hint was held, it stayed where it was
  Ndt3DModel held(map, Ramp::params(1.0));
  held.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  held.seedGround(Pose2D{5.0, 5.0, 0.0}, 0.0);
  held.followGround(Pose2D{12.0, 5.0, 0.0});
  EXPECT_EQ(held.lastGroundFollow(), GroundFollow::Kept);
  EXPECT_EQ(held.heightHint(), 0.0);
}

// A seed with no ground cell near it (hint = the guess 0) while the map ground is at z = 50:
// held for four updates, then the first estimate with a cell takes the level.
TEST(NdtGround, SeedWithoutGroundThenReanchorsWhereTheGroundIs) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 30.0, 50.0);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  Ndt3DModel m(map, Ramp::params(1.0));
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  EXPECT_FALSE(m.seedGround(Pose2D{-40.0, 5.0, 0.0}, 0.0));
  EXPECT_EQ(m.heightHint(), 0.0);
  for (int k = 0; k < 4; ++k) EXPECT_EQ(m.followGround(Pose2D{10.0 + k, 5.0, 0.0}), GroundFollow::Kept);
  EXPECT_EQ(m.groundMisses(), 4);
  EXPECT_EQ(m.followGround(Pose2D{14.0, 5.0, 0.0}), GroundFollow::Reanchored);
  EXPECT_NEAR(m.heightHint(), 50.0, 1e-9);
  EXPECT_EQ(m.followGround(Pose2D{14.5, 5.0, 0.0}), GroundFollow::Followed);
  // a seed on the mapped ground with the guess 0 (RViz) takes the level at once
  EXPECT_TRUE(m.seedGround(Pose2D{10.0, 5.0, 0.0}, 0.0));
  EXPECT_NEAR(m.heightHint(), 50.0, 1e-9);
}

// Two levels (0 and 3): the estimate stops getting answers because the hint (1.2) sits between
// them; re-anchoring takes the level nearest to the stale hint, here the lower one. That can be
// the wrong level; holding the hint for ever would leave every particle at one fixed height.
TEST(NdtGround, ReanchorPicksTheLevelNearestTheStaleHint) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 10.0, 0.0);
  flat(pts, 0.0, 10.0, 3.0);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  Ndt3DModel m(map, Ramp::params(1.0));
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  m.setHeightHint(1.2);
  for (int k = 0; k < 4; ++k) EXPECT_EQ(m.followGround(Pose2D{5.0, 5.0, 0.0}), GroundFollow::Kept);
  EXPECT_EQ(m.followGround(Pose2D{5.0, 5.0, 0.0}), GroundFollow::Reanchored);
  EXPECT_NEAR(m.heightHint(), 0.0, 1e-9);
  m.setHeightHint(1.8);
  for (int k = 0; k < 4; ++k) m.followGround(Pose2D{5.0, 5.0, 0.0});
  EXPECT_EQ(m.followGround(Pose2D{5.0, 5.0, 0.0}), GroundFollow::Reanchored);
  EXPECT_NEAR(m.heightHint(), 3.0, 1e-9);
}

TEST(NdtGround, ReseedResetsTheMissCounterAndNoMapMeansNoGround) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 10.0, 0.0);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  NdtParams p = Ramp::params(1.0);
  p.ground_reanchor_after = 3;
  Ndt3DModel m(map, p);
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  m.seedGround(Pose2D{5.0, 5.0, 0.0}, 0.0);
  m.followGround(Pose2D{50.0, 5.0, 0.0});
  m.followGround(Pose2D{50.0, 5.0, 0.0});
  EXPECT_EQ(m.groundMisses(), 2);
  m.seedGround(Pose2D{5.0, 5.0, 0.0}, 0.0);
  EXPECT_EQ(m.groundMisses(), 0);
  EXPECT_EQ(m.followGround(Pose2D{50.0, 5.0, 0.0}), GroundFollow::Kept);  // the limit is 3: still held
  EXPECT_EQ(m.followGround(Pose2D{50.0, 5.0, 0.0}), GroundFollow::Kept);
  EXPECT_EQ(m.followGround(Pose2D{50.0, 5.0, 0.0}), GroundFollow::NoGround);  // third miss, no cell anywhere near
  Ndt3DModel none(map, p);
  none.setHeightHint(2.0);
  EXPECT_EQ(none.followGround(Pose2D{5.0, 5.0, 0.0}), GroundFollow::NoGround);
  EXPECT_EQ(none.heightHint(), 2.0);
  EXPECT_FALSE(none.seedGround(Pose2D{5.0, 5.0, 0.0}, 7.0));  // no map: only the hint is set
  EXPECT_EQ(none.heightHint(), 7.0);
}

// Re-anchoring must not jump floors: the upper floor (3.3 m) has a gap; a floor at 0 runs under
// all of it. After the miss limit the nearest level to the hint is the lower floor, 3.3 m
// away, more than ground_reanchor_max_jump: the hint is held, and the upper floor is
// followed again after the gap.
TEST(NdtGround, ReanchorDoesNotJumpFloors) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 30.0, 0.0);
  flat(pts, 0.0, 10.0, 3.3);
  flat(pts, 14.0, 30.0, 3.3);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  Ndt3DModel m(map, Ramp::params(1.0));
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  ASSERT_TRUE(m.seedGround(Pose2D{5.0, 5.0, 0.0}, 3.3));
  EXPECT_NEAR(m.heightHint(), 3.3, 1e-9);
  int no_ground = 0;
  for (double x = 8.0; x < 20.0; x += 0.25) {
    const GroundFollow r = m.followGround(Pose2D{x, 5.0, 0.0});
    EXPECT_NE(r, GroundFollow::Reanchored) << x;
    no_ground += r == GroundFollow::NoGround;
    if (r != GroundFollow::Followed) { EXPECT_NEAR(m.heightHint(), 3.3, 1e-6) << x; }  // held
  }
  EXPECT_GT(no_ground, 0);  // the limit was reached and the lower floor was refused
  EXPECT_EQ(m.lastGroundFollow(), GroundFollow::Followed);
  EXPECT_NEAR(m.heightHint(), 3.3, 1e-9);
}

// A seed that found no ground cell has only a guess for a hint: it takes the first ground it
// meets, however far. A seed that found its level does not (above); the state reads Seeded.
TEST(NdtGround, SeedState) {
  std::vector<Eigen::Vector3d> pts;
  flat(pts, 0.0, 30.0, 50.0);
  auto map = std::make_shared<NdtMap>(pts, 1.0, 5);
  Ndt3DModel m(map, Ramp::params(1.0));
  m.setGroundMap(std::make_shared<GroundHeightMap>(pts));
  m.seedGround(Pose2D{-40.0, 5.0, 0.0}, 0.0);
  EXPECT_EQ(m.lastGroundFollow(), GroundFollow::Seeded);
  for (int k = 0; k < 4; ++k) m.followGround(Pose2D{10.0, 5.0, 0.0});
  EXPECT_EQ(m.followGround(Pose2D{10.0, 5.0, 0.0}), GroundFollow::Reanchored);  // 50 m away: accepted
  EXPECT_NEAR(m.heightHint(), 50.0, 1e-9);
  // the same distance after a seed that found a cell: refused
  m.seedGround(Pose2D{10.0, 5.0, 0.0}, 50.0);
  EXPECT_EQ(m.lastGroundFollow(), GroundFollow::Seeded);
  m.setHeightHint(0.0);  // a hint 50 m from the only level (e.g. a stale one)
  for (int k = 0; k < 6; ++k) EXPECT_NE(m.followGround(Pose2D{10.0, 5.0, 0.0}), GroundFollow::Reanchored);
  EXPECT_EQ(m.lastGroundFollow(), GroundFollow::NoGround);
  EXPECT_EQ(m.heightHint(), 0.0);
}
