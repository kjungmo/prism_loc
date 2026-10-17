#!/usr/bin/env bash
# Regenerate the lever-arm study (experiments/eskf_lever.cpp): build, run both suites,
# merge the shards in configuration order, write the tables.
#
#   experiments/run_eskf_lever.sh OUT_DIR [JOBS]       (from the repository root; JOBS default 4)
#
# Writes OUT_DIR/g60_{runs,agg}.csv, OUT_DIR/g300_{runs,agg}.csv and OUT_DIR/report.md.
# The committed copy is experiments/results/eskf_lever/ (the same files, gzipped). About
# 2 CPU hours in all (1440 configurations of 60 s with 10 seeds, 288 of 300 s with 6 seeds,
# 10 schemes each), 30 minutes with 4 jobs. Needs g++, Eigen 3 headers and python3.
set -euo pipefail
OUT=${1:?usage: run_eskf_lever.sh OUT_DIR [JOBS]}
JOBS=${2:-4}
EIGEN=${EIGEN_INCLUDE:-/usr/include/eigen3}
mkdir -p "$OUT"
g++ -O2 -std=c++17 -I prism_loc_fusion/include -I "$EIGEN" \
    prism_loc_fusion/src/*.cpp experiments/eskf_lever.cpp -o "$OUT/eskf_lever"
for suite in g60 g300; do
  pids=()
  for ((s = 0; s < JOBS; ++s)); do
    "$OUT/eskf_lever" "$suite" "$s" "$JOBS" "$OUT/${suite}_s$s" &
    pids+=($!)
  done
  for p in "${pids[@]}"; do wait "$p"; done
  for kind in runs agg; do
    head -1 "$OUT/${suite}_s0_$kind.csv" > "$OUT/${suite}_$kind.csv"
    for ((s = 0; s < JOBS; ++s)); do tail -n +2 "$OUT/${suite}_s${s}_$kind.csv"; done \
      | sort -t, -k1,1n -s >> "$OUT/${suite}_$kind.csv"
    rm -f "$OUT"/${suite}_s*_"$kind".csv
  done
done
python3 experiments/eskf_lever_report.py "$OUT/g60" "$OUT/g300" > "$OUT/report.md"
echo "done: $OUT/report.md"
