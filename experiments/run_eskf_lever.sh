#!/usr/bin/env bash
# Run the lever-arm study (experiments/eskf_lever.cpp) with the current core: build, run the suites, merge the shards in
# configuration order, write the tables.
#
#   experiments/run_eskf_lever.sh OUT_DIR [JOBS] [SUITES...]   (from the repository root; JOBS default 4)
#
# Any subset of the suites works: the suite CSVs are written first, and eskf_lever_report.py leaves out the tables and the quoted numbers
# of the suites that are not there. The report of a subset is not the committed report.
#
# SUITES default: h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hrate hratex. The first-pass suites g60 and g300 are
# run only when named. Writes OUT_DIR/<suite>_{runs,agg}.csv and OUT_DIR/report.md. The committed copy is experiments/results/eskf_lever/
# (the same files, gzipped).
#
# Schemes of this build: S1s, S2s and S8u (the code that ships; EskfParams defaults); `ignore` in the first-pass suites. LEVER_ONLY=S8u
# runs just one scheme. The schemes of the earlier passes (S0 to S8x) differ from S8u by switches that the core no longer has; their rows
# in the committed CSVs were run with this harness and the core of commit 2011666 (git show 2011666:experiments/eskf_lever.cpp). The
# S8u rows of the cores of round 8 and round 9 are in the CSVs as S8u@0ef1add and S8u@b921395 (the suites where a later round changed a row). To check that this build reproduces the committed rows of S1s, S2s
# and S8u, run a suite into OUT_DIR and compare:
#   LEVER_ONLY=S2s,S8u experiments/run_eskf_lever.sh /tmp/lever 4 hrate
#   python3 experiments/compare_rows.py experiments/results/eskf_lever/hrate /tmp/lever/hrate S2s,S8u
# Size of the suites (configurations x seeds): g60 1,920 x 10; g300 384 x 6; h300 2,160 x 2; h600 96 x 8; h900 48 x 8; h1800 16 x 8; hndt 192 x 2;
# hslope 288 x 4; hyaw 120 x 4; hvib 40 x 4; hsg and hsgv 384 x 4; hgap 832 x 4; hboot 192 x 4; hrate 816 x 4; hratex 544 x 4.
# About 5 minutes per scheme on 4 jobs for hrate, 10 for hratex (the 50 and 100 Hz cells are the slow ones). Needs g++, Eigen 3 headers and
# python3. The committed results are from the system g++ 9.4 (std::normal_distribution is implementation defined: another standard
# library draws other numbers).
set -euo pipefail
OUT=${1:?usage: run_eskf_lever.sh OUT_DIR [JOBS] [SUITES...]}
JOBS=${2:-4}
shift $(( $# < 2 ? $# : 2 ))
SUITES=("$@")
[ ${#SUITES[@]} -gt 0 ] || SUITES=(h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hrate hratex)
EIGEN=${EIGEN_INCLUDE:-/usr/include/eigen3}
mkdir -p "$OUT"
g++ -O2 -std=c++17 -I prism_loc_fusion/include -I "$EIGEN" \
    prism_loc_fusion/src/*.cpp experiments/eskf_lever.cpp -o "$OUT/eskf_lever"
for suite in "${SUITES[@]}"; do
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
prefixes=()
for suite in "${SUITES[@]}"; do prefixes+=("$OUT/$suite"); done
python3 experiments/eskf_lever_report.py "${prefixes[@]}" > "$OUT/report.md"
echo "done: $OUT/report.md"
