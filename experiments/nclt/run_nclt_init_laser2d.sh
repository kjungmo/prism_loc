#!/usr/bin/env bash
# run_nclt_init_laser2d.sh DATA_DIR OUT_DIR [JOBS]
# The initialization-robustness protocol (INIT_PROTOCOL.md) for laser2d on NCLT projected to 2D:
# 4 segments (400, 1400, 2400, 3400 s) x 8 levels (L0-L6, U) x 3 seeds (42, 1, 2), the first
# 180 s of each. DATA_DIR as in run_nclt.sh; OUT_DIR/map and OUT_DIR/query are reused when
# present (the prepared inputs of run_nclt.sh). Runs: OUT_DIR/runs_laser2d/laser2d_<L>_t<seg>_s<seed>.csv.
# Run from the repository root. Needs python3 + numpy, g++, Eigen.
set -euo pipefail
DATA=$1; OUT=$2; JOBS=${3:-4}
MAP_S=2012-01-08; QUERY_S=2012-02-04
mkdir -p "$OUT" "$OUT/runs_laser2d"
BIN=$OUT/nclt_laser2d
g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3 prism_loc_core/src/*.cpp \
  experiments/nclt/nclt_laser2d.cpp -o "$BIN"
[ -f "$OUT/map/map.bin" ] || python3 experiments/nclt/nclt_prepare.py map "$DATA/$MAP_S" \
  "$DATA/groundtruth_$MAP_S.csv" "$OUT/map"
[ -f "$OUT/query/scans.bin" ] || python3 experiments/nclt/nclt_prepare.py query "$DATA/$QUERY_S" \
  "$DATA/groundtruth_$QUERY_S.csv" "$DATA/$QUERY_S/odometry_mu_100hz.csv" "$OUT/query"
jobs_file=$OUT/jobs_laser2d.txt
: > "$jobs_file"
for s in 400 1400 2400 3400; do
  for lv in 0 1 2 3 4 5 6 7; do
    name=L$lv; [ "$lv" = 7 ] && name=U
    for seed in 42 1 2; do
      tag=$OUT/runs_laser2d/laser2d_${name}_t${s}_s${seed}
      [ -f "$tag.csv" ] && continue
      echo "$BIN $OUT/map $OUT/query $tag --param start_s=$s --param duration_s=180 --param level=$lv --param seed=$seed" >> "$jobs_file"
    done
  done
done
xargs -P "$JOBS" -I{} sh -c '{} 2>/dev/null' < "$jobs_file"
for lv in L0 L1 L2 L3 L4 L5 L6 U; do
  echo "== laser2d $lv"
  python3 experiments/nclt/nclt_init_score.py "$OUT"/runs_laser2d/laser2d_${lv}_t*_s*.csv | tail -1
done
