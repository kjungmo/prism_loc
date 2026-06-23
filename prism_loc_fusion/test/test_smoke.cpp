#include <gtest/gtest.h>
#include "prism_loc_fusion/version.hpp"
TEST(Smoke, Version) { EXPECT_STREQ(PRISM_LOC_FUSION_VERSION, "0.1.0"); }
