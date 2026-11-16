#!/usr/bin/env bash
# run_nclt.sh DATA_DIR OUT_DIR [JOBS]
# ndt3d on the public NCLT dataset: a prior map from session 2012-01-08, localization in session
# 2012-02-04 (four 1000 s segments, three seeds), for the likelihood, ground-height and motion
# noise settings below. DATA_DIR holds the extracted sessions (DATA_DIR/2012-01-08/velodyne_sync,
# ...), groundtruth_<date>.csv and <date>/odometry_mu_100hz.csv from the sensor archive.
# Run from the repository root. Needs python3 + numpy, g++, Eigen.
set -euo pipefail
DATA=$1; OUT=$2; JOBS=${3:-4}
MAP_S=2012-01-08; QUERY_S=2012-02-04
mkdir -p "$OUT"
BIN=$OUT/nclt_ndt3d
g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3 prism_loc_core/src/*.cpp \
  experiments/nclt/nclt_ndt3d.cpp -o "$BIN"
[ -f "$OUT/map/map.bin" ] || python3 experiments/nclt/nclt_prepare.py map "$DATA/$MAP_S" \
  "$DATA/groundtruth_$MAP_S.csv" "$OUT/map"
[ -f "$OUT/query/scans.bin" ] || python3 experiments/nclt/nclt_prepare.py query "$DATA/$QUERY_S" \
  "$DATA/groundtruth_$QUERY_S.csv" "$DATA/$QUERY_S/odometry_mu_100hz.csv" "$OUT/query"

# name | parameters
CONFIGS=(
  "default|"
  "ep10|--param effective_points=10"
  "ep5_ms10|--param effective_points=5 --param min_sigma=0.10"
  "ground|--param ground=1"
  "ground_ep10|--param ground=1 --param effective_points=10"
  "ground_ep5_ms10_a05|--param ground=1 --param effective_points=5 --param min_sigma=0.10 --param alpha1=0.05 --param alpha2=0.05 --param alpha3=0.05 --param alpha4=0.05"
)
SEGMENTS="0 1200 2400 3600"
SEEDS="42 1 2"
jobs_file=$OUT/jobs.txt
: > "$jobs_file"
for c in "${CONFIGS[@]}"; do
  name=${c%%|*}; params=${c#*|}
  for s in $SEGMENTS; do for seed in $SEEDS; do
    tag=$OUT/runs/${name}_t${s}_s${seed}
    [ -f "$tag.csv" ] && continue
    echo "$BIN $OUT/map $OUT/query $tag --param start_s=$s --param duration_s=1000 --param seed=$seed $params" >> "$jobs_file"
  done; done
done
mkdir -p "$OUT/runs"
xargs -P "$JOBS" -I{} sh -c '{} 2>/dev/null' < "$jobs_file"
for c in "${CONFIGS[@]}"; do
  name=${c%%|*}
  echo "== $name"
  python3 experiments/nclt/nclt_score.py "$OUT"/runs/${name}_t*_s*.csv | tail -1
done
