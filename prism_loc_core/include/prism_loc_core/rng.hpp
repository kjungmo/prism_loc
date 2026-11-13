#pragma once
#include <cstdint>
#include <random>
namespace prism_loc_core {
class Rng {
 public:
  explicit Rng(std::uint64_t seed = 42u) : gen_(seed) {}
  void seed(std::uint64_t s) { gen_.seed(s); }
  double uniform(double lo = 0.0, double hi = 1.0) {
    return std::uniform_real_distribution<double>(lo, hi)(gen_);
  }
  double gaussian(double mean = 0.0, double stddev = 1.0) {
    // mean + stddev * N(0, 1): the same draws and the same value as
    // std::normal_distribution(mean, stddev), and defined for stddev == 0 (a zero motion
    // gives a zero noise spread), which the standard distribution does not allow.
    return mean + stddev * std::normal_distribution<double>(0.0, 1.0)(gen_);
  }
  std::mt19937_64& engine() { return gen_; }
 private:
  std::mt19937_64 gen_;
};
}  // namespace prism_loc_core
