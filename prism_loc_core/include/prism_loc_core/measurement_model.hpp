#pragma once
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/ndt_map.hpp"
namespace prism_loc_core {

class MeasurementModel {
 public:
  virtual ~MeasurementModel() = default;
  // Unnormalized log-likelihood of the current observation given base_in_map.
  virtual double logLikelihood(const Pose2D& base_in_map) const = 0;
};

struct LaserScan2D {
  std::vector<float> ranges;
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{100.0};
  Pose2D sensor_in_base;
};

struct LaserParams {
  int max_beams{60};
  double z_hit{0.5};
  double z_rand{0.5};
  double sigma_hit{0.2};
  double max_dist{2.0};
};

class Laser2DLikelihoodField : public MeasurementModel {
 public:
  Laser2DLikelihoodField(const GridMap& map, LaserParams params);
  void setScan(const LaserScan2D& scan) { scan_ = scan; }
  double logLikelihood(const Pose2D& base_in_map) const override;

 private:
  LikelihoodField field_;
  LaserParams params_;
  LaserScan2D scan_;
};

struct NdtParams {
  int max_points{500};
  double base_height{0.0};
  double score_floor{1e-6};
};

class Ndt3DModel : public MeasurementModel {
 public:
  Ndt3DModel(std::shared_ptr<const NdtMap> map, NdtParams params);
  // cloud points expressed in the sensor frame
  void setCloud(const std::vector<Eigen::Vector3d>& pts_sensor, const Pose2D& sensor_in_base);
  double logLikelihood(const Pose2D& base_in_map) const override;

 private:
  std::shared_ptr<const NdtMap> map_;
  NdtParams params_;
  std::vector<Eigen::Vector3d> pts_sensor_;
  Pose2D sensor_in_base_;
};

}  // namespace prism_loc_core
