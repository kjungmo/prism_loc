#include <gtest/gtest.h>
#include "prism_loc_core/version.hpp"

TEST(Smoke, VersionDefined) {
  EXPECT_STREQ(PRISM_LOC_CORE_VERSION, "0.1.0");
}
