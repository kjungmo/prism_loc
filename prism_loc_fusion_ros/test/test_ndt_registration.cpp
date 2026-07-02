#include <gtest/gtest.h>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "prism_loc_fusion_ros/ndt_registration.hpp"
using namespace prism_loc_fusion_ros;

static pcl::PointCloud<pcl::PointXYZ>::Ptr boxCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>());
  for (double x = -5; x <= 5; x += 0.2) for (double z = 0; z <= 3; z += 0.2) {
    c->emplace_back(x, 5.0, z); c->emplace_back(x, -5.0, z);
  }
  for (double y = -5; y <= 5; y += 0.2) for (double z = 0; z <= 3; z += 0.2) {
    c->emplace_back(5.0, y, z); c->emplace_back(-5.0, y, z);
  }
  for (double x = -5; x <= 5; x += 0.2) for (double y = -5; y <= 5; y += 0.2)
    c->emplace_back(x, y, 0.0);
  return c;
}
TEST(NdtRegistration, RecoversTranslation) {
  auto target = boxCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
  const double shift = 0.4;
  for (const auto& p : *target) source->emplace_back(p.x + shift, p.y, p.z);  // shifted +x
  NdtRegistration reg(2.0, 0.1, 1e-3, 50);
  reg.setTarget(target);
  NdtResult r = reg.align(source, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(r.pose.translation().x(), -shift, 0.1);  // source->target ≈ -shift
  EXPECT_LT(std::abs(r.pose.translation().y()), 0.1);
}
