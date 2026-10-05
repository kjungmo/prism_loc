#include <gtest/gtest.h>
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
