#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/geodetic.hpp"
using namespace prism_loc_fusion;

TEST(Geodetic, DatumMapsToZero) {
  GeodeticConverter g; g.setDatum({37.0, 127.0, 0.0});
  Eigen::Vector3d e = g.toEnu({37.0, 127.0, 0.0});
  EXPECT_LT(e.norm(), 1e-6);
}
TEST(Geodetic, AltitudeIsUp) {
  GeodeticConverter g; g.setDatum({37.0, 127.0, 0.0});
  Eigen::Vector3d e = g.toEnu({37.0, 127.0, 10.0});
  EXPECT_NEAR(e.x(), 0.0, 1e-3);
  EXPECT_NEAR(e.y(), 0.0, 1e-3);
  EXPECT_NEAR(e.z(), 10.0, 1e-2);
}
TEST(Geodetic, EastAndNorthSignsAndScale) {
  const double lat0 = 37.0;
  GeodeticConverter g; g.setDatum({lat0, 127.0, 0.0});
  const double dlon = 0.001;  // deg
  Eigen::Vector3d e = g.toEnu({lat0, 127.0 + dlon, 0.0});
  EXPECT_GT(e.x(), 0.0);                 // east
  EXPECT_LT(std::abs(e.y()), 0.5);       // ~no north
  EXPECT_LT(std::abs(e.z()), 0.5);       // ~no up
  const double expected_east = 6378137.0 * std::cos(lat0 * M_PI / 180.0) * (dlon * M_PI / 180.0);
  EXPECT_NEAR(e.x(), expected_east, 0.02 * expected_east);
  Eigen::Vector3d n = g.toEnu({lat0 + dlon, 127.0, 0.0});
  EXPECT_GT(n.y(), 0.0);                 // north
  EXPECT_LT(std::abs(n.x()), 0.5);
}
