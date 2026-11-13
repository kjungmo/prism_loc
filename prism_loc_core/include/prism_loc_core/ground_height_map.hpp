#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
namespace prism_loc_core {

struct GroundHeightParams {
  double resolution{1.0};        // m: edge of an xy cell; finite and > 0
  int min_points{5};             // a cell, and a vertical group inside it, needs this many points; >= 1
  double layer_gap{0.2};         // m: smallest empty vertical gap that separates two surfaces in one
                                 // cell (floor and ceiling underside, two floors); finite and > 0
  double max_step{0.5};          // m: a candidate counts only within this of the hint; finite and > 0
  double band{0.3};              // m: height above the lowest point of a sub-column that is looked at;
                                 // finite and >= 0. Core field only (not a node parameter)
};

// Throws std::invalid_argument naming the field when a value is out of range or not finite.
void validate(const GroundHeightParams& p);

// Ground height of the map as a function of (x, y), built once from the map points (the
// points the NdtMap gets). A hash grid of xy cells; each cell stores a short list of
// "ground candidates", one per vertical group of points, so a map with several surfaces
// over the same xy (two floors, a floor below a ceiling) keeps each of them. A candidate is
// a plane: height a at the cell centre and slope (bx, by).
//
// Build, per cell that holds at least min_points points:
//  1. Sort the cell's points by z. Start a new group wherever two consecutive z differ by
//     more than layer_gap (an empty vertical gap: the surfaces are separate). A group with
//     fewer than min_points points gives no candidate (a few stray points are dropped).
//  2. Split the cell into 4 x 4 sub-columns. In each sub-column of the group that has
//     points: s_low = its lowest z; the points with s_low <= z <= s_low + band; the
//     sub-column value v = the median z of those points, at the position of their centroid
//     (x, y).
//  3. Fit the least-squares plane through the sub-column values. One robust pass: drop the
//     sub-columns whose value lies more than max(0.05 m, 2.5 x MAD of the residuals) ABOVE
//     the plane (walls and objects only bias upward; the MAD is about the median residual),
//     and refit when at least 3 well-spread sub-columns remain. Median instead of minimum
//     keeps the value unbiased on noisy ground at any density.
//  4. The fit is refused, and the mean of the (kept) sub-column values is stored with slope
//     0, when fewer than 3 sub-columns are used, or the smaller eigenvalue of their 2 x 2 xy
//     covariance is below (0.1 * resolution)^2 (they lie on a line or in a corner), or the
//     plane's value at the centre differs from the mean by more than band.
// Points that are not finite are skipped. Points beyond about +-1e6 cells of the origin are
// skipped too and counted in numSkippedOutOfRange(). The result depends on the set of
// points only, not on their order (every comparison is a total order on the values).
// Build memory: 8 bytes per map point on top of the input (a cell id and an index each).
//
// heightAt() blends the planes of the four cells around the query with bilinear weights, so
// the height is continuous and a constant slope is exact with any subset of cells present.
// Edges of the mapped ground: a query between the centre of an edge cell and the edge uses
// that cell's plane alone (it extends half a cell); where the edge cell's fit was refused
// (a thin strip of points) its slope is 0, and the error there is up to about
// slope x resolution / 2. Remaining errors elsewhere: curvature of the ground inside a cell,
// and objects that cover most of a cell's sub-columns up to the band.
//
// The object is immutable after construction: queries are const and thread safe.
class GroundHeightMap {
 public:
  explicit GroundHeightMap(const std::vector<Eigen::Vector3d>& points,
                           const GroundHeightParams& params = GroundHeightParams{});

  // Ground height under (x, y) on the level that holds z_hint (the ground height of the
  // current estimate, or any z near the level wanted).
  // For each of the four cells whose centres surround (x, y): evaluate the plane of each of
  // its candidates at (x, y), take the one nearest to z_hint (a tie goes to the lower one)
  // and keep it only when within max_step of z_hint. Of the kept values take the one nearest
  // to z_hint (tie: the higher bilinear weight) as the reference and drop those that differ
  // from it by more than layer_gap (another surface). Return the bilinear interpolation of
  // the rest, weights renormalised (equal weights when they are all 0, i.e. (x, y) is the
  // centre of an empty cell). nullopt when nothing is kept, or an argument is not finite.
  std::optional<double> heightAt(double x, double y, double z_hint) const;

  // Same, for a reseed where the level is only roughly known: z_guess may be far from every
  // candidate (RViz sends z = 0).
  // 1. Candidates: those of the cell that contains (x, y) when it has any, otherwise those of
  //    the four cells around (x, y). Their heights are the plane values at the cell centres.
  // 2. The anchor is the candidate nearest to z_guess (tie: lower).
  // 3. Return heightAt(x, y, anchor).
  // No limit on the distance between z_guess and the anchor. nullopt when there is no
  // candidate, or an argument is not finite.
  std::optional<double> heightNearest(double x, double y, double z_guess) const;

  std::size_t numCells() const { return cells_.size(); }       // cells with >= 1 candidate
  std::size_t numCandidates() const { return num_candidates_; }
  std::size_t numSkippedOutOfRange() const { return skipped_out_of_range_; }
  // Estimate of the heap held by the grid after the build, in bytes: per cell a hash node
  // (next pointer, key, vector header: 40 bytes) plus the allocator's 16 byte granularity
  // and header, the candidate array likewise, and the bucket array.
  std::size_t approxBytes() const;
  const GroundHeightParams& params() const { return params_; }

 private:
  static std::int64_t keyOfCell(std::int64_t i, std::int64_t j);
  struct Cand { double a, bx, by; };  // plane: z = a + bx (x - cx) + by (y - cy), (cx, cy) the cell centre
  const std::vector<Cand>* cell(std::int64_t i, std::int64_t j) const;
  bool cellIndex(double x, double y, std::int64_t& i, std::int64_t& j) const;
  // the four surrounding cells and their weights; false when out of range
  bool surround(double x, double y, std::int64_t& i0, std::int64_t& j0, double& tx, double& ty) const;

  GroundHeightParams params_;
  std::unordered_map<std::int64_t, std::vector<Cand>> cells_;
  std::size_t num_candidates_{0};
  std::size_t skipped_out_of_range_{0};
};

}  // namespace prism_loc_core
