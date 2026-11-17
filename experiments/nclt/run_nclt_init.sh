#!/usr/bin/env bash
# run_nclt_init.sh OUT_DIR JOBS: the initialization-robustness protocol (INIT_PROTOCOL.md) for
# ndt3d (best setting) and the 6-DoF scan-to-map NDT tracker on the prepared NCLT data in
# OUT_DIR/map and OUT_DIR/query (run run_nclt.sh first). First 180 s of each segment.
set -euo pipefail
OUT=$1; JOBS=${2:-4}; PCL_VER=${PCL_VER:-1.12}
PK="pcl_registration-$PCL_VER pcl_filters-$PCL_VER pcl_common-$PCL_VER pcl_kdtree-$PCL_VER pcl_search-$PCL_VER eigen3"
g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3 prism_loc_core/src/*.cpp \
  experiments/nclt/nclt_ndt3d.cpp -o "$OUT/nclt_ndt3d"
g++ -O2 -std=c++17 experiments/nclt/nclt_ndt_tracker.cpp -o "$OUT/nclt_ndt_tracker" $(pkg-config --cflags --libs $PK)
BEST="--param ground=1 --param effective_points=5 --param min_sigma=0.10 --param alpha1=0.05 --param alpha2=0.05 --param alpha3=0.05 --param alpha4=0.05"
LEVELS="L0:0:0 L1:2:15 L2:5:15 L3:10:15 L4:2:45 L5:2:90 L6:10:90"
mkdir -p "$OUT/init"
jobs=$OUT/init/jobs.txt; : > "$jobs"
for lv in $LEVELS; do IFS=: read -r name d a <<< "$lv"
  for s in 400 1400 2400 3400; do for seed in 42 1 2; do
    common="--param start_s=$s --param duration_s=180 --param seed=$seed --param init_d=$d --param init_a=$a"
    [ -f "$OUT/init/ndt3d_${name}_t${s}_s${seed}.csv" ] || echo "$OUT/nclt_ndt3d $OUT/map $OUT/query $OUT/init/ndt3d_${name}_t${s}_s${seed} $common $BEST" >> "$jobs"
    [ -f "$OUT/init/tracker_${name}_t${s}_s${seed}.csv" ] || echo "$OUT/nclt_ndt_tracker $OUT/map $OUT/query $OUT/init/tracker_${name}_t${s}_s${seed} $common" >> "$jobs"
  done; done
done
xargs -P "$JOBS" -I{} sh -c '{} > /dev/null 2>&1' < "$jobs"
for m in ndt3d tracker; do for lv in $LEVELS; do name=${lv%%:*}
  printf "%-8s %-3s " $m $name; python3 experiments/nclt/nclt_init_score.py "$OUT"/init/${m}_${name}_t*_s*.csv | tail -1
done; done
