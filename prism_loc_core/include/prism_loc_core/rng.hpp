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
    return std::normal_distribution<double>(mean, stddev)(gen_);
  }
  std::mt19937_64& engine() { return gen_; }
 private:
  std::mt19937_64 gen_;
};
}  // namespace prism_loc_core
