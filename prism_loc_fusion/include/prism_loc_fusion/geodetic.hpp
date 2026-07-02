#pragma once
#include <Eigen/Core>
namespace prism_loc_fusion {
struct GeoPoint { double lat_deg{0.0}; double lon_deg{0.0}; double alt_m{0.0}; };
class GeodeticConverter {
 public:
  void setDatum(const GeoPoint& d);
  bool hasDatum() const { return has_datum_; }
  GeoPoint datum() const { return datum_; }
  Eigen::Vector3d toEnu(const GeoPoint& p) const;
 private:
  static Eigen::Vector3d llaToEcef(const GeoPoint& p);
  bool has_datum_{false};
  GeoPoint datum_;
  Eigen::Vector3d ecef0_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_ecef_to_enu_{Eigen::Matrix3d::Identity()};
};
}  // namespace prism_loc_fusion
