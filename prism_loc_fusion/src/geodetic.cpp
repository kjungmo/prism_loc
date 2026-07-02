#include "prism_loc_fusion/geodetic.hpp"
#include <cmath>
namespace prism_loc_fusion {
namespace {
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

Eigen::Vector3d GeodeticConverter::llaToEcef(const GeoPoint& p) {
  const double lat = p.lat_deg * kDeg2Rad, lon = p.lon_deg * kDeg2Rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);
  const double N = kA / std::sqrt(1.0 - kE2 * slat * slat);
  return Eigen::Vector3d((N + p.alt_m) * clat * clon,
                         (N + p.alt_m) * clat * slon,
                         (N * (1.0 - kE2) + p.alt_m) * slat);
}

void GeodeticConverter::setDatum(const GeoPoint& d) {
  datum_ = d; has_datum_ = true; ecef0_ = llaToEcef(d);
  const double lat = d.lat_deg * kDeg2Rad, lon = d.lon_deg * kDeg2Rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);
  R_ecef_to_enu_ <<        -slon,         clon,   0.0,
                    -slat * clon, -slat * slon,  clat,
                     clat * clon,  clat * slon,  slat;
}

Eigen::Vector3d GeodeticConverter::toEnu(const GeoPoint& p) const {
  if (!has_datum_) return Eigen::Vector3d::Zero();
  return R_ecef_to_enu_ * (llaToEcef(p) - ecef0_);
}

}  // namespace prism_loc_fusion
