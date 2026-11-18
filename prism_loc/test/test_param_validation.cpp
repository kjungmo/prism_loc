#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <string>
#include "prism_loc/param_validation.hpp"

using namespace prism_loc;

// The message must name the parameter so the operator can find it in the YAML.
template <typename F>
static void expectRejects(F f, const std::string& name) {
  try {
    f();
    FAIL() << "expected std::invalid_argument for " << name;
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find(name), std::string::npos) << e.what();
  }
}

TEST(ParamValidation, ShippedDefaultsPass) {
  EXPECT_NO_THROW(validateNodeParams(NodeParams{}));
  EXPECT_NO_THROW(validateFilterParams(prism_loc_core::ParticleFilterParams{},
                                       prism_loc_core::MotionParams{}));
  EXPECT_NO_THROW(validateLaserParams(prism_loc_core::LaserParams{}, 0.0, 0.0));
  EXPECT_NO_THROW(validateBbsParams(prism_loc_core::BbsParams{},
                                    prism_loc_core::RelocVerifierParams{}));
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, prism_loc_core::NdtParams{}));
}

TEST(ParamValidation, ParticleBoundsOrdered) {
  prism_loc_core::ParticleFilterParams pp;
  pp.min_particles = 3000;
  pp.max_particles = 2000;
  expectRejects([&] { validateFilterParams(pp, prism_loc_core::MotionParams{}); },
                "max_particles");
  pp.min_particles = 0;
  expectRejects([&] { validateFilterParams(pp, prism_loc_core::MotionParams{}); },
                "min_particles");
}

TEST(ParamValidation, FilterRangesRejected) {
  prism_loc_core::ParticleFilterParams pp;
  pp.resample_threshold = 1.5;
  expectRejects([&] { validateFilterParams(pp, prism_loc_core::MotionParams{}); },
                "resample_threshold");
  pp = {};
  pp.kld_bin_xy = 0.0;
  expectRejects([&] { validateFilterParams(pp, prism_loc_core::MotionParams{}); }, "kld_bin_xy");
  prism_loc_core::MotionParams mp;
  mp.alpha3 = -0.1;
  expectRejects([&] { validateFilterParams(prism_loc_core::ParticleFilterParams{}, mp); },
                "alpha3");
}

TEST(ParamValidation, LaserRangesRejected) {
  prism_loc_core::LaserParams lp;
  lp.sigma_hit = 0.0;
  expectRejects([&] { validateLaserParams(lp, 0.0, 0.0); }, "sigma_hit");
  lp = {};
  lp.z_hit = 0.0;
  lp.z_rand = 0.0;
  expectRejects([&] { validateLaserParams(lp, 0.0, 0.0); }, "z_hit + z_rand");
  expectRejects([&] { validateLaserParams(prism_loc_core::LaserParams{}, 5.0, 2.0); },
                "laser_max_range");
  EXPECT_NO_THROW(validateLaserParams(prism_loc_core::LaserParams{}, 0.2, 12.0));
}

TEST(ParamValidation, NodeAndVerifierRangesRejected) {
  NodeParams np;
  np.transform_tolerance = -0.1;
  expectRejects([&] { validateNodeParams(np); }, "transform_tolerance");
  np = {};
  np.input_timeout_s = 0.0;
  expectRejects([&] { validateNodeParams(np); }, "input_timeout_s");
  prism_loc_core::RelocVerifierParams rv;
  rv.min_posterior = 0.0;
  expectRejects([&] { validateBbsParams(prism_loc_core::BbsParams{}, rv); },
                "bbs_verify_min_posterior");
  rv = {};
  rv.verify_scans = 0;
  expectRejects([&] { validateBbsParams(prism_loc_core::BbsParams{}, rv); }, "bbs_verify_scans");
  rv = {};
  rv.min_translation = -0.3;
  expectRejects([&] { validateBbsParams(prism_loc_core::BbsParams{}, rv); },
                "bbs_verify_min_translation");
  expectRejects([&] { validateNdtParams(0.0, 5, prism_loc_core::NdtParams{}); }, "ndt_resolution");
}

// min_match_fraction is a share: [0, 1], with 0 (check off) and 1 allowed.
TEST(ParamValidation, MinMatchFractionRange) {
  const prism_loc_core::NdtParams np;
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.0));
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 1.0));
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, prism_loc::kDefaultMinMatchFraction));
  expectRejects([&] { validateNdtParams(1.0, 5, np, -0.01); }, "min_match_fraction");
  expectRejects([&] { validateNdtParams(1.0, 5, np, 1.01); }, "min_match_fraction");
  EXPECT_DOUBLE_EQ(prism_loc::kDefaultMinMatchFraction, 0.65);
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 0));
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 1000));
  expectRejects([&] { validateNdtParams(1.0, 5, np, 0.65, -1); }, "match_verify_updates");
}

// ndt_effective_points and ndt_min_sigma: finite and >= 0; 0 keeps the previous behaviour.
TEST(ParamValidation, NdtLikelihoodShapingRanges) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  prism_loc_core::NdtParams np;
  np.effective_points = prism_loc::kDefaultNdtEffectivePoints;
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 15, prism_loc::kDefaultNdtMinSigma));
  np.effective_points = 10.0;
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 15, 0.05));
  EXPECT_DOUBLE_EQ(prism_loc::kDefaultNdtEffectivePoints, 0.0);
  EXPECT_DOUBLE_EQ(prism_loc::kDefaultNdtMinSigma, 0.0);
  np.effective_points = 0.0;
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 15, 0.0));
  for (double bad : {-0.5, inf, -inf, nan}) {
    np.effective_points = bad;
    expectRejects([&] { validateNdtParams(1.0, 5, np); }, "ndt_effective_points");
    np.effective_points = 10.0;
    expectRejects([&] { validateNdtParams(1.0, 5, np, 0.65, 15, bad); }, "ndt_min_sigma");
  }
}

// Caps that keep extreme finite values from overflowing the likelihood or the cell covariance:
// ndt_min_sigma <= ndt_resolution, ndt_effective_points <= max_points.
TEST(ParamValidation, NdtLikelihoodShapingCaps) {
  prism_loc_core::NdtParams np;  // max_points 500
  np.effective_points = 500.0;
  EXPECT_NO_THROW(validateNdtParams(1.0, 5, np, 0.65, 15, 1.0));
  np.effective_points = 500.5;
  expectRejects([&] { validateNdtParams(1.0, 5, np); }, "ndt_effective_points");
  np.effective_points = 1e300;
  expectRejects([&] { validateNdtParams(1.0, 5, np); }, "ndt_effective_points");
  np.effective_points = 10.0;
  // The message names both parameters and the remedy.
  np.max_points = 5;
  expectRejects([&] { validateNdtParams(1.0, 5, np); }, "max_points = 5");
  expectRejects([&] { validateNdtParams(1.0, 5, np); }, "ndt_effective_points = 10");
  expectRejects([&] { validateNdtParams(1.0, 5, np); }, "lower ndt_effective_points or raise max_points");
  np.max_points = 500;
  expectRejects([&] { validateNdtParams(1.0, 5, np, 0.65, 15, 1.01); }, "ndt_min_sigma");
  expectRejects([&] { validateNdtParams(0.05, 5, np, 0.65, 15, 0.10); }, "ndt_min_sigma");
  expectRejects([&] { validateNdtParams(1.0, 5, np, 0.65, 15, 1e200); }, "ndt_min_sigma");
}

// ground_* parameters (ground_height_from_map): the shipped defaults pass; each key is
// named in the message of its own rejection.
TEST(ParamValidation, GroundHeightParams) {
  prism_loc_core::GroundHeightParams gp;
  EXPECT_NO_THROW(validateGroundParams(gp));
  EXPECT_DOUBLE_EQ(gp.resolution, 1.0);
  EXPECT_DOUBLE_EQ(gp.layer_gap, 0.2);
  EXPECT_DOUBLE_EQ(gp.max_step, 0.5);
  EXPECT_EQ(gp.min_points, 5);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto with = [&](auto setter) {
    prism_loc_core::GroundHeightParams p;
    setter(p);
    return p;
  };
  for (double bad : {0.0, -1.0, nan, std::numeric_limits<double>::infinity()}) {
    expectRejects([&] { validateGroundParams(with([&](auto& p) { p.resolution = bad; })); }, "ground_resolution");
    expectRejects([&] { validateGroundParams(with([&](auto& p) { p.layer_gap = bad; })); }, "ground_layer_gap");
    expectRejects([&] { validateGroundParams(with([&](auto& p) { p.max_step = bad; })); }, "ground_max_step");
  }
  expectRejects([&] { validateGroundParams(with([](auto& p) { p.min_points = 0; })); }, "ground_min_points");
  EXPECT_NO_THROW(validateGroundParams(with([](auto& p) { p.min_points = 1; })));
}
