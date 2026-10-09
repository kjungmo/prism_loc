#include <gtest/gtest.h>
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
  expectRejects([&] { validateNdtParams(0.0, 5, prism_loc_core::NdtParams{}); }, "ndt_resolution");
}
