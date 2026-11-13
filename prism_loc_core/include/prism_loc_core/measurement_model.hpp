#pragma once
#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Core>
#include "prism_loc_core/types.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/ndt_map.hpp"
#include "prism_loc_core/ground_height_map.hpp"
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
  // Height of the cloud frame (the sensor, level) in the map frame. Without a ground map
  // (Ndt3DModel::setGroundMap) it is a constant map z. With one it is the height of the
  // cloud frame above the local ground: the cloud sits at ground(x, y) + base_height.
  double base_height{0.0};
  double score_floor{1e-6};
  // 0: the log-likelihood is the sum of log(score + score_floor) over the evaluated points.
  // > 0: it is the mean of those per-point terms times this count, i.e. the cloud counts as
  // this many independent points however many were evaluated (a 3D scan's neighbouring
  // points are strongly correlated, so the plain sum is overconfident).
  double effective_points{0.0};
};
// The NDT map's min_sigma (NdtMap constructor) is the other likelihood shaping knob. It is not
// part of NdtParams, and matchStats() does not use it.

// How well the current cloud fits the map at one pose. A diagnostic: the filter does not
// use it, and computing it changes nothing in the model. It scores with NdtMap::matchScore,
// i.e. without the map's min_sigma floor, so "on a surface" does not depend on min_sigma.
struct NdtMatchStats {
  int points{0};   // points evaluated: the subsample logLikelihood() scores
  int matched{0};  // of those, within kNdtMatchSigma standard deviations of the map cell they fall in
  // Share of the evaluated points that lie on a mapped surface; 0 when no point was evaluated.
  double fraction() const { return points > 0 ? static_cast<double>(matched) / points : 0.0; }
};
// A point counts as matched when its Mahalanobis distance to the Gaussian of its cell is
// at most this (NdtMap::score >= exp(-0.5 * 3^2)); a point in a cell without a Gaussian
// (score 0) never does.
constexpr double kNdtMatchSigma = 3.0;

class Ndt3DModel : public MeasurementModel {
 public:
  Ndt3DModel(std::shared_ptr<const NdtMap> map, NdtParams params);
  // cloud points expressed in the sensor frame
  void setCloud(const std::vector<Eigen::Vector3d>& pts_sensor, const Pose2D& sensor_in_base);
  double logLikelihood(const Pose2D& base_in_map) const override;
  // Scores the same points as logLikelihood(base_in_map), placed the same way.
  NdtMatchStats matchStats(const Pose2D& base_in_map) const;

  // --- optional: height of each particle from a ground height grid of the map ---
  // Without a ground map (the default, or null) every particle's cloud sits at the constant
  // map z base_height and the model is exactly the one without this feature.
  // With one, the cloud of a particle at (x, y) sits at map z = ground(x, y) + base_height,
  // where ground() is GroundHeightMap::heightAt(x, y, height hint). When that has no answer
  // (no ground cell near (x, y), or none within max_step of the hint) the particle keeps the
  // height of the current estimate: hint + base_height. logLikelihood() and matchStats()
  // both place the cloud with sensorInMap(), so they cannot differ.
  void setGroundMap(std::shared_ptr<const GroundHeightMap> ground) { ground_ = std::move(ground); }
  bool hasGroundMap() const { return ground_ != nullptr; }
  // Ground height (map z) under the current estimate; the hint of every particle's query in
  // the next evaluations. The caller sets it after each filter update and at a reseed.
  // Ignored without a ground map.
  void setHeightHint(double z_ground) { height_hint_ = z_ground; }
  double heightHint() const { return height_hint_; }
  // Ground height under base_in_map with the current hint; nullopt without a ground map or
  // when the grid has no answer there.
  std::optional<double> groundHeightAt(const Pose2D& base_in_map) const;
  // Reseed query: GroundHeightMap::heightNearest, the level nearest to z_guess with no
  // limit on the distance. nullopt without a ground map or when the grid has no cell there.
  std::optional<double> groundHeightNearest(const Pose2D& base_in_map, double z_guess) const;
  // Map z of the cloud frame for a particle at base_in_map (what the next evaluation uses).
  double cloudHeightAt(const Pose2D& base_in_map) const;
  // Pose of the (level) cloud frame in the map for a particle at base_in_map.
  Eigen::Isometry3d sensorInMap(const Pose2D& base_in_map) const;

 private:
  std::shared_ptr<const NdtMap> map_;
  NdtParams params_;
  std::vector<Eigen::Vector3d> pts_sensor_;
  Pose2D sensor_in_base_;
  std::shared_ptr<const GroundHeightMap> ground_;
  double height_hint_{0.0};
};

}  // namespace prism_loc_core
