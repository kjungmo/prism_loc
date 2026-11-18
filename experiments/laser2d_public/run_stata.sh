#!/usr/bin/env bash
# run_stata.sh DATA_DIR WORK_DIR OUT_DIR
# laser2d + BBS relocalization on the public MIT Stata Center dataset. DATA_DIR holds the dataset's
# *.gt.laser.scans and *.gt.laser.poses files of the sessions 2012-04-06-11-28-12 (map),
# 2012-04-02-10-54-41 (long query), 2012-04-04-07-15-50 (short query) and 2012-01-18-09-09-07
# (older map), all "part1_floor2". Run from the repository root. Needs python3 + numpy, g++, Eigen.
set -euo pipefail
DATA=$1; WORK=$2; OUT=$3
mkdir -p "$WORK" "$OUT"
BIN=$WORK/mit_stata_laser2d
g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3 prism_loc_core/src/*.cpp \
  experiments/laser2d_public/mit_stata_laser2d.cpp -o "$BIN"
S=_part1_floor2.gt.laser
MAPS=2012-04-06-11-28-12; MAPO=2012-01-18-09-09-07; QL=2012-04-02-10-54-41; QS=2012-04-04-07-15-50
for m in $MAPS $MAPO; do
  [ -f "$WORK/map_$m.bin" ] || python3 experiments/laser2d_public/stata_prepare.py \
    "$DATA/$m$S.scans" "$DATA/$m$S.poses" "$WORK/map_$m.bin" | tee "$OUT/map_$m.txt"
done
run() { # mode map query out [params...]
  local mode=$1 map=$2 q=$3 out=$4; shift 4
  "$BIN" "$mode" "$WORK/map_$map.bin" "$DATA/$q$S.scans" "$DATA/$q$S.poses" "$out" "$@" 2>>"$OUT/stderr.log"
}
# 1. tracking: seeded at the ground truth, 3 seeds
for s in 42 1 2; do
  run track $MAPS $QL "$OUT/track_map0406_q0402_s$s" --param seed=$s --param stride=10
  run track $MAPS $QS "$OUT/track_map0406_q0404_s$s" --param seed=$s --param stride=10
  run track $MAPO $QS "$OUT/track_map0118_q0404_s$s" --param seed=$s --param stride=10   # older map
  run track $MAPO $QL "$OUT/track_map0118_q0402_s$s" --param seed=$s --param stride=10
done
# sanity: map built from the very session that is tracked (optimistic)
run track $MAPS $MAPS "$OUT/track_same_session_0406" --param stride=10
run track $MAPO $MAPO "$OUT/track_same_session_0118" --param stride=10
# 2. global relocalization, 50 queries over the run (window widened to cover the whole 101 m map)
run reloc $MAPS $QL "$OUT/reloc_map0406_q0402" --param bbs_max_linear_window=60
run reloc $MAPO $QL "$OUT/reloc_map0118_q0402" --param bbs_max_linear_window=60
# 3. initial-pose offset levels
INIT_STARTS=200,500,800,1100 run init $MAPS $QL "$OUT/init_map0406_q0402" --param bbs_max_linear_window=60
