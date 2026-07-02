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
