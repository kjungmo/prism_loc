#!/usr/bin/env bash
# Regenerate the lever-arm study (experiments/eskf_lever.cpp): build, run the suites, merge
# the shards in configuration order, write the tables.
#
#   experiments/run_eskf_lever.sh OUT_DIR [JOBS] [SUITES...]   (from the repository root; JOBS default 4)
#
# Any subset of the suites works: the suite CSVs are written first, and eskf_lever_report.py leaves out the tables and the quoted numbers
# of the suites that are not there. The report of a subset is not the committed report.
#
# SUITES default: g60 g300 h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hrate. Writes OUT_DIR/<suite>_{runs,agg}.csv
# and OUT_DIR/report.md. The committed copy is experiments/results/eskf_lever/ (the same files,
# gzipped). Size of the suites (configurations x seeds x schemes):
#   g60    1,920 configurations of 60 s, 10 seeds, 15 schemes   (first pass)
#   g300     384 configurations of 300 s, 6 seeds, 15 schemes   (first pass)
#   h300   2,160 configurations of 300 s, 2 seeds, 13 schemes   (second pass, see the top of eskf_lever.cpp)
#   h600      96 configurations of 600 s, 8 seeds, 13 schemes
#   h900      48 configurations of 900 s, 8 seeds, 13 schemes
#   h1800     16 configurations of 1,800 s, 8 seeds, 13 schemes
#   hndt     192 configurations of 300 s with NDT, 2 seeds, 13 schemes
#   hslope   288 configurations of 60 s, 4 seeds, 13 schemes
#   hyaw     120 configurations of 300 s, 4 seeds, 13 schemes
#   hvib      40 configurations of 300 s, 4 seeds, 13 schemes
#   hsg      384 configurations of 460 to 1,960 s (standing, then moving), 4 seeds, 6 schemes   (third pass)
#   hsgv     the same on other data (the check of the schemes developed on hsg)
#   hgap     192 configurations of 460 to 760 s (standing, then moving, the fixes out for 30 s or 120 s across the start), 4 seeds   (fifth pass: S8o, S8, S8u; round 7 adds more cells and S8k, S8w, below)
#   hboot    192 configurations of 166 to 460 s (a wrong initial_yaw, standing, then moving), 4 seeds, 4 schemes (S8o, S8, S8k, S8u)
#   hgap     (round 7) the 192 configurations above and 640 more: outages of 1.5 to 12 s that begin just before or in the middle of the first acceleration after a stand; 5 schemes (S8o, S8, S8k, S8w, S8u)
#   hrate    (round 7) 816 configurations of 166 to 1,060 s at fix rates of 0.5 to 20 Hz, 4 seeds, 7 schemes (S2s, S6vb, S8, S8k, S8t, S8n, S8u)
# The second-pass suites run the 13 schemes of the second pass. The rows of S8 and S8u for them (the schemes of the third and the fifth pass; S8u is what the code ships)
# are not part of a default run; they are made with
#   LEVER_ONLY=S8,S8u "$OUT/eskf_lever" SUITE SHARD NSHARDS PREFIX    for each second-pass suite, then
#   python3 experiments/merge_round7.py MAIN_DIR SUP_DIR OUT_DIR     (round 7: keeps the committed S8u rows as S8k, appends the new S8u rows; prints how many of the re-run rows of S8 equal the committed ones)
#   python3 experiments/merge_rows.py --replace S8,S8u MAIN_DIR SUP_DIR OUT_DIR     (the rounds before; appends the rows to those of their configuration; a scheme named in --replace
#   takes the place of its old rows, and the script prints how many of the new rows equal the old ones)
# About 2 CPU hours for the first pass and 2.5 CPU hours for the second (13 schemes); with 4 jobs the
# committed results took 57 minutes for the 11 schemes of the first run and 13 more for the two added. Needs g++, Eigen 3 headers
# and python3. The committed results are from the system g++ 9.4 (std::normal_distribution is
# implementation defined: another standard library draws other numbers).
set -euo pipefail
OUT=${1:?usage: run_eskf_lever.sh OUT_DIR [JOBS] [SUITES...]}
JOBS=${2:-4}
shift $(( $# < 2 ? $# : 2 ))
SUITES=("$@")
[ ${#SUITES[@]} -gt 0 ] || SUITES=(g60 g300 h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hrate)
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
