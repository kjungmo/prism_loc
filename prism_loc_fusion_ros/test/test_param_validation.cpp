#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include "prism_loc_fusion_ros/param_validation.hpp"

using prism_loc_fusion_ros::FusionNodeParams;
using prism_loc_fusion_ros::validateFusionParams;

template <typename F>
static void expectRejects(F f, const std::string& name) {
  try {
    f();
    FAIL() << "expected std::invalid_argument for " << name;
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find(name), std::string::npos) << e.what();
  }
}

TEST(FusionParamValidation, ShippedDefaultsPass) {
  EXPECT_NO_THROW(validateFusionParams(FusionNodeParams{}, prism_loc_fusion::EskfParams{}));
}

TEST(FusionParamValidation, RangesRejected) {
  FusionNodeParams p;
  p.points_voxel_leaf = 0.0;
  expectRejects([&] { validateFusionParams(p, prism_loc_fusion::EskfParams{}); },
                "points_voxel_leaf");
  p = {};
  p.ndt_max_iter = 0;
  expectRejects([&] { validateFusionParams(p, prism_loc_fusion::EskfParams{}); }, "ndt_max_iter");
  p = {};
  p.imu_queue_depth = 0;
  expectRejects([&] { validateFusionParams(p, prism_loc_fusion::EskfParams{}); },
                "imu_queue_depth");
  prism_loc_fusion::EskfParams ep;
  ep.sigma_gyro = -1e-3;
  expectRejects([&] { validateFusionParams(FusionNodeParams{}, ep); }, "sigma_gyro");
}

TEST(FusionParamValidation, NewDefaults) {
  const FusionNodeParams p;
  EXPECT_DOUBLE_EQ(p.ndt_max_fitness, 0.06);
  EXPECT_DOUBLE_EQ(p.ndt_fitness_max_range, 1.0);
  EXPECT_DOUBLE_EQ(p.ndt_min_inlier_fraction, 0.5);
  EXPECT_DOUBLE_EQ(p.ndt_epsilon, 1e-4);
  EXPECT_DOUBLE_EQ(p.points_voxel_leaf, 1.0);
  EXPECT_DOUBLE_EQ(p.points_max_age, 0.5);
  EXPECT_DOUBLE_EQ(p.points_time_offset, 0.0);
}

TEST(FusionParamValidation, FitnessRangeInlierFractionAndAgeRanges) {
  const prism_loc_fusion::EskfParams ep;
  FusionNodeParams p;
  p.ndt_fitness_max_range = -0.1;
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_fitness_max_range");
  p.ndt_fitness_max_range = std::numeric_limits<double>::infinity();
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_fitness_max_range");
  p.ndt_fitness_max_range = std::nan("");
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_fitness_max_range");
  p = {};
  p.ndt_min_inlier_fraction = 1.1;
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_min_inlier_fraction");
  p.ndt_min_inlier_fraction = -0.1;
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_min_inlier_fraction");
  p.ndt_min_inlier_fraction = std::nan("");
  expectRejects([&] { validateFusionParams(p, ep); }, "ndt_min_inlier_fraction");
  p = {};
  p.points_max_age = -1.0;
  expectRejects([&] { validateFusionParams(p, ep); }, "points_max_age");
  p.points_max_age = std::nan("");
  expectRejects([&] { validateFusionParams(p, ep); }, "points_max_age");
  p = {};
  p.points_time_offset = 1.5;
  expectRejects([&] { validateFusionParams(p, ep); }, "points_time_offset");
  p.points_time_offset = std::nan("");
  expectRejects([&] { validateFusionParams(p, ep); }, "points_time_offset");
  p.points_time_offset = -0.05;
  EXPECT_NO_THROW(validateFusionParams(p, ep));
  p = {};
  p.ndt_fitness_max_range = 0.0;  // unlimited
  p.ndt_min_inlier_fraction = 0.0;
  p.points_max_age = 0.0;  // off
  EXPECT_NO_THROW(validateFusionParams(p, ep));
  p.ndt_min_inlier_fraction = 1.0;
  EXPECT_NO_THROW(validateFusionParams(p, ep));
}

TEST(FusionParamValidation, WarnsWhenTheFitnessLimitGatesNothing) {
  FusionNodeParams p;
  EXPECT_TRUE(fitnessGateWarning(p).empty());  // 0.06 < 1.0^2
  p.ndt_max_fitness = 2.0;                     // a YAML that kept the old default
  const std::string w = fitnessGateWarning(p);
  EXPECT_NE(w.find("ndt_max_fitness"), std::string::npos) << w;
  EXPECT_NE(w.find("ndt_fitness_max_range"), std::string::npos) << w;
  p.ndt_max_fitness = 1.0;                     // equal to range^2: still gates nothing
  EXPECT_FALSE(fitnessGateWarning(p).empty());
  p.ndt_fitness_max_range = 0.0;               // unlimited score: 2.0 is the matching value
  p.ndt_max_fitness = 2.0;
  EXPECT_TRUE(fitnessGateWarning(p).empty());
}

TEST(FusionParamValidation, WarnsWhenTheUnlimitedScoreIsGivenALimitBelowOne) {
  FusionNodeParams p;
  p.ndt_fitness_max_range = 0.0;
  p.ndt_max_fitness = 0.06;  // the new default with the old (unlimited) score
  const std::string w = fitnessGateWarning(p);
  EXPECT_NE(w.find("ndt_fitness_max_range"), std::string::npos) << w;
  EXPECT_NE(w.find("rejects"), std::string::npos) << w;
  p.ndt_max_fitness = 1.0;
  EXPECT_TRUE(fitnessGateWarning(p).empty());
}
