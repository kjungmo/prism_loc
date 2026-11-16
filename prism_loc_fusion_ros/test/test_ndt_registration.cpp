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

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud::Ptr shifted(const Cloud::Ptr& c, double dx, double dy = 0.0) {
  Cloud::Ptr out(new Cloud());
  for (const auto& p : *c) out->emplace_back(p.x + dx, p.y + dy, p.z);
  return out;
}

// The room plus an extra wall far outside it: `fraction` of the returned points are on it.
Cloud::Ptr withFarWall(const Cloud::Ptr& room, double fraction) {
  Cloud::Ptr out(new Cloud(*room));
  const std::size_t extra = static_cast<std::size_t>(room->size() * fraction / (1.0 - fraction));
  for (std::size_t i = 0; i < extra; ++i)
    out->emplace_back(-5.0 + 10.0 * double(i) / double(extra), 40.0, 1.0);
  return out;
}

void expectSamePose(const NdtResult& a, const NdtResult& b) {
  EXPECT_TRUE(a.pose.matrix().isApprox(b.pose.matrix(), 1e-12));
  EXPECT_DOUBLE_EQ(a.fitness, b.fitness);
  EXPECT_DOUBLE_EQ(a.inlier_fraction, b.inlier_fraction);
  EXPECT_EQ(a.iterations, b.iterations);
  EXPECT_EQ(a.converged, b.converged);
}
}  // namespace

TEST(NdtRegistration, ReusedObjectGivesTheResultsOfFreshObjects) {
  auto target = boxCloud();
  auto s1 = shifted(target, 0.4);
  auto s2 = shifted(target, -0.3, 0.2);
  NdtRegistration reused(2.0, 0.1, 1e-4, 50, 1.0);
  reused.setTarget(target);
  const NdtResult a1 = reused.align(s1, Eigen::Isometry3d::Identity());
  const NdtResult a2 = reused.align(s2, Eigen::Isometry3d::Identity());
  const NdtResult a1_again = reused.align(s1, Eigen::Isometry3d::Identity());
  NdtRegistration fresh1(2.0, 0.1, 1e-4, 50, 1.0), fresh2(2.0, 0.1, 1e-4, 50, 1.0);
  fresh1.setTarget(target);
  fresh2.setTarget(target);
  expectSamePose(a1, fresh1.align(s1, Eigen::Isometry3d::Identity()));
  expectSamePose(a2, fresh2.align(s2, Eigen::Isometry3d::Identity()));
  expectSamePose(a1, a1_again);  // the earlier align leaves no state behind
}

TEST(NdtRegistration, TargetIsBuiltOncePerSetTargetNotPerAlign) {
  auto target = boxCloud();
  NdtRegistration reg(2.0, 0.1, 1e-3, 30);
  EXPECT_EQ(reg.targetBuilds(), 0);
  reg.setTarget(target);
  EXPECT_EQ(reg.targetBuilds(), 1);
  EXPECT_TRUE(reg.hasTarget());
  for (int i = 0; i < 3; ++i) reg.align(shifted(target, 0.2), Eigen::Isometry3d::Identity());
  EXPECT_EQ(reg.targetBuilds(), 1);
  reg.setTarget(target);
  EXPECT_EQ(reg.targetBuilds(), 2);
}

TEST(NdtRegistration, DistantPointsSpoilTheUnlimitedScoreOnly) {
  auto room = boxCloud();
  auto source = withFarWall(room, 0.3);
  NdtRegistration limited(2.0, 0.1, 1e-4, 30, 1.0);
  limited.setTarget(room);
  const NdtResult r = limited.align(source, Eigen::Isometry3d::Identity());  // the true pose
  EXPECT_TRUE(r.converged);
  EXPECT_LT(r.fitness, 0.06);
  EXPECT_NEAR(r.inlier_fraction, 0.7, 0.01);
  EXPECT_LT(r.pose.translation().norm(), 0.05);
  EXPECT_GE(r.iterations, 1);

  NdtRegistration unlimited(2.0, 0.1, 1e-4, 30);  // range 0: PCL's score over every point
  unlimited.setTarget(room);
  const NdtResult u = unlimited.align(source, Eigen::Isometry3d::Identity());
  EXPECT_GT(u.fitness, 2.0);
  EXPECT_DOUBLE_EQ(u.inlier_fraction, 1.0);
}

TEST(NdtRegistration, UnlimitedFitnessIsTheMeanSquaredNearestNeighbourDistance) {
  auto room = boxCloud();
  auto source = withFarWall(shifted(room, 0.15), 0.3);
  NdtRegistration reg(2.0, 0.1, 1e-4, 30);
  reg.setTarget(room);
  const NdtResult r = reg.align(source, Eigen::Isometry3d::Identity());
  double sum = 0.0;
  for (const auto& p : *source) {
    const Eigen::Vector3d q = r.pose * Eigen::Vector3d(p.x, p.y, p.z);
    double best = 1e18;
    for (const auto& t : *room)
      best = std::min(best, (Eigen::Vector3d(t.x, t.y, t.z) - q).squaredNorm());
    sum += best;
  }
  // The cloud PCL scores is the float transform of the source; the pose here is the same
  // transform in double.
  EXPECT_NEAR(r.fitness, sum / source->size(), 1e-3 * r.fitness + 1e-6);
}

TEST(NdtRegistration, WrongPoseScoresWorseWithinTheRange) {
  auto room = boxCloud();
  auto source = withFarWall(shifted(room, 0.5), 0.3);
  // One iteration of at most 0.01 m: the result stays near the wrong start.
  NdtRegistration stuck(2.0, 0.01, 1e-4, 1, 1.0);
  stuck.setTarget(room);
  const NdtResult wrong = stuck.align(source, Eigen::Isometry3d::Identity());
  NdtRegistration good(2.0, 0.1, 1e-4, 30, 1.0);
  good.setTarget(room);
  Eigen::Isometry3d near_truth = Eigen::Isometry3d::Identity();
  near_truth.translation().x() = -0.5;
  const NdtResult right = good.align(source, near_truth);
  EXPECT_GT(wrong.fitness, right.fitness + 0.02);
  EXPECT_LT(right.fitness, 0.06);
}

TEST(NdtRegistration, EmptyInputsGiveANotConvergedResult) {
  NdtRegistration reg(2.0, 0.1, 1e-3, 30, 1.0);
  Cloud::Ptr empty(new Cloud());
  EXPECT_FALSE(reg.hasTarget());
  EXPECT_FALSE(reg.align(boxCloud(), Eigen::Isometry3d::Identity()).converged);  // no target
  reg.setTarget(boxCloud());
  EXPECT_FALSE(reg.align(empty, Eigen::Isometry3d::Identity()).converged);
  EXPECT_FALSE(reg.align(nullptr, Eigen::Isometry3d::Identity()).converged);
}
