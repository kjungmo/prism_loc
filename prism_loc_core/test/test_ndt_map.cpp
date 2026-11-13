#include <gtest/gtest.h>
#include <vector>
#include "prism_loc_core/ndt_map.hpp"
#include "prism_loc_core/rng.hpp"
using namespace prism_loc_core;

TEST(NdtMap, RecoversVoxelMean) {
  Rng rng(11);
  std::vector<Eigen::Vector3d> pts;
  const Eigen::Vector3d center(0.4, 0.4, 0.4);  // all inside voxel [0,1)^3 at res=1
  for (int i = 0; i < 400; ++i)
    pts.emplace_back(center + Eigen::Vector3d(rng.gaussian(0, 0.05),
                                              rng.gaussian(0, 0.05),
                                              rng.gaussian(0, 0.05)));
  NdtMap m(pts, /*resolution=*/1.0, /*min_points=*/6);
  EXPECT_EQ(m.numVoxels(), 1u);
  EXPECT_TRUE(m.hasVoxel(center));
  // Score is maximal near the mean, smaller far away (but in-voxel).
  EXPECT_GT(m.score(center), m.score(Eigen::Vector3d(0.9, 0.9, 0.9)));
  EXPECT_GT(m.score(center), 0.5);
}

TEST(NdtMap, EmptyVoxelScoresZero) {
  std::vector<Eigen::Vector3d> pts(20, Eigen::Vector3d(0.5, 0.5, 0.5));
  NdtMap m(pts, 1.0, 6);
  EXPECT_FALSE(m.hasVoxel(Eigen::Vector3d(50.0, 50.0, 50.0)));
  EXPECT_DOUBLE_EQ(m.score(Eigen::Vector3d(50.0, 50.0, 50.0)), 0.0);
}

TEST(NdtMap, SparseVoxelDropped) {
  std::vector<Eigen::Vector3d> pts = {{0.5,0.5,0.5},{0.5,0.5,0.5},{0.5,0.5,0.5}};
  NdtMap m(pts, 1.0, /*min_points=*/6);  // only 3 points -> below threshold
  EXPECT_EQ(m.numVoxels(), 0u);
}

// Points on the plane z = 0.5 with about 1 mm of noise: the cell is nearly flat, so without
// an absolute floor a point 0.1 m off the plane scores about 0.
static std::vector<Eigen::Vector3d> thinPlane() {
  Rng rng(21);
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 400; ++i)
    pts.emplace_back(rng.uniform(0.1, 0.9), rng.uniform(0.1, 0.9), 0.5 + rng.gaussian(0, 0.001));
  return pts;
}

TEST(NdtMap, MinSigmaFloorsThinCell) {
  const auto pts = thinPlane();
  NdtMap legacy(pts, 1.0, 6);
  NdtMap floored(pts, 1.0, 6, /*min_sigma=*/0.05);
  const Eigen::Vector3d off(0.5, 0.5, 0.6);  // 0.1 m off the plane
  EXPECT_LT(legacy.score(off), 1e-6);
  EXPECT_GT(floored.score(off), 0.1);  // 0.1 m is 2 sigma: exp(-2) = 0.135
  EXPECT_DOUBLE_EQ(floored.minSigma(), 0.05);
  EXPECT_DOUBLE_EQ(legacy.minSigma(), 0.0);
}

// In the plane the covariance is far above the floor: the score is unchanged.
TEST(NdtMap, MinSigmaLeavesInPlaneScoreUnchanged) {
  const auto pts = thinPlane();
  NdtMap legacy(pts, 1.0, 6);
  NdtMap floored(pts, 1.0, 6, 0.05);
  for (const Eigen::Vector3d& q : {Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(0.3, 0.7, 0.5),
                                   Eigen::Vector3d(0.8, 0.2, 0.5)})
    EXPECT_NEAR(floored.score(q), legacy.score(q), 0.02) << q.transpose();
}

TEST(NdtMap, MinSigmaZeroIsTheDefault) {
  const auto pts = thinPlane();
  NdtMap a(pts, 1.0, 6);
  NdtMap b(pts, 1.0, 6, 0.0);
  for (double dz : {-0.05, 0.0, 0.003, 0.1, 0.3}) {
    const Eigen::Vector3d q(0.4, 0.6, 0.5 + dz);
    EXPECT_EQ(a.score(q), b.score(q));
  }
}
