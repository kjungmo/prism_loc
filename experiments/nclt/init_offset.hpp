// init_offset.hpp -- the initial-pose error of experiments/nclt/INIT_PROTOCOL.md, shared by the
// NCLT drivers. For (seed, segment start, d, a): the planar offset of length d in a direction
// uniform in [0, 2 pi) and the yaw offset +a or -a, both drawn from
// std::mt19937_64(seed * 1000003 + segment_start_s): first draw the direction
// (uniform_real_distribution(0, 2 pi)), then the sign (bernoulli_distribution(0.5): true = +).
#pragma once
#include <cmath>
#include <cstdint>
#include <random>

struct InitOffset { double dx, dy, dyaw; };

inline InitOffset initOffset(int seed, double segment_start_s, double d_m, double a_deg) {
  std::mt19937_64 g(static_cast<std::uint64_t>(seed) * 1000003ULL + static_cast<std::uint64_t>(std::llround(segment_start_s)));
  const double dir = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(g);
  const bool plus = std::bernoulli_distribution(0.5)(g);
  return InitOffset{d_m * std::cos(dir), d_m * std::sin(dir), (plus ? 1.0 : -1.0) * a_deg * M_PI / 180.0};
}
