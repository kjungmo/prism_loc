// amcl_offsets.cpp -- prints the INIT_PROTOCOL.md offset (dx dy dyaw) for every (segment, level L1-L6, seed)
// so that amcl_run_nclt.py uses exactly init_offset.hpp. Build: g++ -O2 -std=c++17 amcl_offsets.cpp -o amcl_offsets
#include <cstdio>
#include "init_offset.hpp"
int main() {
  const double D[7] = {0, 2, 5, 10, 2, 2, 10}, A[7] = {0, 15, 15, 15, 45, 90, 90};
  for (int seg : {400, 1400, 2400, 3400})
    for (int lv = 0; lv < 7; ++lv)
      for (int seed : {42, 1, 2}) {
        const InitOffset o = initOffset(seed, seg, D[lv], A[lv]);
        std::printf("%d %d %d %.9f %.9f %.9f\n", seg, lv, seed, o.dx, o.dy, o.dyaw);
      }
}
