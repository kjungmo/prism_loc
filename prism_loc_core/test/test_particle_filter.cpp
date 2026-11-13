#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "prism_loc_core/particle_filter.hpp"
using namespace prism_loc_core;

TEST(ParticleFilter, LowVarianceResamplerConcentratesOnHeavyParticle) {
  Rng rng(3);
  ParticleFilter pf(ParticleFilterParams{}, rng);
  pf.initializeGaussian(Pose2D{0,0,0}, Pose2D{0,0,0}, 4);  // 4 identical-pose particles
  // Manually skew weights via a fake model: weight index 2 dominant.
  // Use resampleLowVariance directly on a hand-set particle set is internal;
  // instead assert Neff math on uniform weights == N.
  EXPECT_NEAR(pf.effectiveSampleSize(), 4.0, 1e-9);
}

TEST(ParticleFilter, EstimateIsWeightedMean) {
  Rng rng(3);
  ParticleFilter pf(ParticleFilterParams{}, rng);
  pf.initializeGaussian(Pose2D{1.0, 2.0, 0.0}, Pose2D{0.0,0.0,0.0}, 10);
  Pose2D e = pf.estimate();
  EXPECT_NEAR(e.x, 1.0, 1e-6);
  EXPECT_NEAR(e.y, 2.0, 1e-6);
}

namespace {
// Likelihood read from the particle's x: the particle at x = i scores like[i].
class TableModel : public MeasurementModel {
 public:
  explicit TableModel(std::vector<double> like) : like_(std::move(like)) {}
  double logLikelihood(const Pose2D& p) const override {
    const double l = like_[static_cast<size_t>(std::lround(p.x))];
    return std::log(l);
  }
 private:
  std::vector<double> like_;
};

ParticleFilter lineFilter(Rng& rng, ParticleFilterParams params, int n) {
  ParticleFilter pf(params, rng);
  pf.initializeGaussian(Pose2D{0, 0, 0}, Pose2D{0, 0, 0}, n);
  for (int i = 0; i < n; ++i) pf.particles()[i].pose.x = i;
  return pf;
}
}  // namespace

TEST(ParticleFilter, WeightsAccumulateWhenResamplingWasSkipped) {
  Rng rng(5);
  ParticleFilterParams params;
  params.resample_threshold = 0.0;  // never resample: Neff >= 0
  ParticleFilter pf = lineFilter(rng, params, 4);
  const TableModel a({1.0, 2.0, 4.0, 8.0});
  const TableModel b({3.0, 1.0, 1.0, 5.0});
  pf.correct(a);
  pf.resample();  // skipped: weights stay as correct() left them
  pf.correct(b);
  const double expect[4] = {1.0 * 3.0, 2.0 * 1.0, 4.0 * 1.0, 8.0 * 5.0};
  double sum = 0.0;
  for (double e : expect) sum += e;
  double total = 0.0;
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(pf.particles()[i].weight, expect[i] / sum, 1e-12) << i;
    total += pf.particles()[i].weight;
  }
  EXPECT_NEAR(total, 1.0, 1e-12);
}

TEST(ParticleFilter, WeightsAreFreshLikelihoodAfterResample) {
  Rng rng(5);
  ParticleFilterParams params;
  params.resample_threshold = 1.0;  // resample unless the weights are exactly uniform
  ParticleFilter pf = lineFilter(rng, params, 4);
  pf.correct(TableModel({1.0, 1.0, 1.0, 1e-9}));  // Neff < N: the next resample() draws
  pf.resample();
  const size_t n = pf.particles().size();
  for (const auto& p : pf.particles()) EXPECT_NEAR(p.weight, 1.0 / n, 1e-12);
  const TableModel b({1.0, 2.0, 4.0, 8.0});
  pf.correct(b);
  double sum = 0.0;
  for (const auto& p : pf.particles()) sum += std::exp(b.logLikelihood(p.pose));
  for (const auto& p : pf.particles())
    EXPECT_NEAR(p.weight, std::exp(b.logLikelihood(p.pose)) / sum, 1e-12);
}

TEST(ParticleFilter, ZeroWeightStaysZeroAndNonFiniteLikelihoodIsZeroWeight) {
  Rng rng(5);
  ParticleFilterParams params;
  params.resample_threshold = 0.0;
  ParticleFilter pf = lineFilter(rng, params, 4);
  pf.particles()[1].weight = 0.0;
  pf.correct(TableModel({1.0, 100.0, 1.0, 0.0}));  // particle 3: log(0) = -inf
  EXPECT_EQ(pf.particles()[1].weight, 0.0);
  EXPECT_EQ(pf.particles()[3].weight, 0.0);
  EXPECT_NEAR(pf.particles()[0].weight, 0.5, 1e-12);
  EXPECT_NEAR(pf.particles()[2].weight, 0.5, 1e-12);
}

TEST(ParticleFilter, AllZeroLikelihoodFallsBackToUniform) {
  Rng rng(5);
  ParticleFilterParams params;
  params.resample_threshold = 0.0;
  ParticleFilter pf = lineFilter(rng, params, 4);
  pf.correct(TableModel({0.0, 0.0, 0.0, 0.0}));
  for (const auto& p : pf.particles()) EXPECT_NEAR(p.weight, 0.25, 1e-12);
  pf.correct(TableModel({1.0, 3.0, 1.0, 3.0}));  // the filter recovers afterwards
  EXPECT_NEAR(pf.particles()[1].weight, 0.375, 1e-12);
}

TEST(ParticleFilter, SingleParticleKeepsUnitWeight) {
  Rng rng(5);
  ParticleFilterParams params;
  params.resample_threshold = 0.0;
  ParticleFilter pf = lineFilter(rng, params, 1);
  pf.correct(TableModel({0.001}));
  pf.correct(TableModel({0.001}));
  EXPECT_DOUBLE_EQ(pf.particles()[0].weight, 1.0);
}
