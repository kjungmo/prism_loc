#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
namespace prism_loc_core {

struct GroundHeightParams {
  double resolution{1.0};      // m: edge of an xy cell; > 0
  int min_points{5};           // a cell, and a vertical group inside it, needs this many points; >= 1
  double layer_gap{1.5};       // m: z gap that splits a cell's points into separate vertical groups
                               // (head-room between two drivable levels); > 0
  double ground_quantile{0.05};  // in [0, 1]: which low z of a group is its ground candidate
  double max_step{0.5};        // m: heightAt() accepts candidates within this of the hint; > 0
};

// Throws std::invalid_argument naming the field when a value is out of range or not finite.
void validate(const GroundHeightParams& p);

// Ground height of the map as a function of (x, y), built once from the map points (the
// points the NdtMap gets). A hash grid of xy cells; each cell stores a short ascending
// list of "ground candidates", one per vertical group of points, so a multi-level map
// (a ramp, a bridge, two floors) keeps every level.
//
// Build, per cell holding at least min_points points: sort the z values; start a new
// group wherever two consecutive z differ by more than layer_gap; a group with at least
// min_points points gives one candidate, the value at sorted index
// floor(ground_quantile * (n - 1)) of that group (n = its size). A low quantile instead of
// the minimum ignores a few points below the surface; a wall standing on the ground adds
// points above it in the same group, so the low quantile is still the ground. Points that
// are not finite or lie beyond about +-1e6 cells are skipped. The result depends on the
// set of points only, not on their order.
//
// Ground under a ceiling or an overhead structure: it forms its own group (when the gap is
// larger than layer_gap) and its own candidate; heightAt()'s hint keeps it from being used.
//
// The object is immutable after construction: queries are const and thread safe.
class GroundHeightMap {
 public:
  explicit GroundHeightMap(const std::vector<Eigen::Vector3d>& points,
                           const GroundHeightParams& params = GroundHeightParams{});

  // Ground height under (x, y) on the level that holds z_hint (the ground height of the
  // current estimate, or any z near the level wanted).
  // In each of the 3x3 cells around (x, y), take the candidate nearest to z_hint (a tie
  // goes to the lower one); keep those within max_step of z_hint; return their median
  // (mean of the two middle values when there is an even number). nullopt when no cell
  // has such a candidate, or when an argument is not finite. One hash lookup per cell.
  std::optional<double> heightAt(double x, double y, double z_hint) const;

  // Same, for a reseed where the level is only roughly known: z_guess may be far from
  // every candidate (RViz sends z = 0).
  // 1. In each of the 3x3 cells, take the candidate nearest to z_guess (tie: lower).
  // 2. Among those values take the one nearest to z_guess (tie: lower), the anchor.
  // 3. Return heightAt(x, y, anchor), i.e. the median of the step 1 values within
  //    max_step of the anchor.
  // No limit on the distance between z_guess and the anchor. nullopt only when the 3x3
  // cells hold no candidate at all, or when an argument is not finite.
  std::optional<double> heightNearest(double x, double y, double z_guess) const;

  std::size_t numCells() const { return cells_.size(); }       // cells with >= 1 candidate
  std::size_t numCandidates() const { return num_candidates_; }
  // Rough heap use of the grid in bytes (entries, candidate vectors, hash buckets).
  std::size_t approxBytes() const;
  const GroundHeightParams& params() const { return params_; }

 private:
  std::int64_t keyOfCell(std::int64_t i, std::int64_t j) const;
  // Candidates of a cell, ascending; nullptr when the cell has none or i, j are out of range.
  const std::vector<double>* cell(std::int64_t i, std::int64_t j) const;
  bool cellIndex(double x, double y, std::int64_t& i, std::int64_t& j) const;

  GroundHeightParams params_;
  std::unordered_map<std::int64_t, std::vector<double>> cells_;
  std::size_t num_candidates_{0};
};

}  // namespace prism_loc_core
