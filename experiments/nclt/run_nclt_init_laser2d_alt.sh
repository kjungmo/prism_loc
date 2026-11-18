#!/usr/bin/env bash
# run_nclt_init_laser2d_alt.sh BIN MAP_DIR QUERY_DIR OUT_DIR [JOBS]
# INIT_PROTOCOL.md for laser2d with the pre-registered alternative height band of INIT_PROTOCOL_AMCL.md
# (map 1.5-3.0 m above the local ground, scan 0.85-2.35 m above the sensor plane). All other settings as
# run_nclt_init_laser2d.sh. BIN = nclt_laser2d built as in its header. CSV per run in OUT_DIR.
set -euo pipefail
BIN=$1; MAP=$2; QUERY=$3; OUT=$4; JOBS=${5:-3}
mkdir -p "$OUT"
jobs_file=$OUT/jobs.txt; : > "$jobs_file"
for s in 400 1400 2400 3400; do
  for lv in 0 1 2 3 4 5 6 7; do
    name=L$lv; [ "$lv" = 7 ] && name=U
    for seed in 42 1 2; do
      tag=$OUT/laser2d_alt_${name}_t${s}_s${seed}
      [ -f "$tag.csv" ] && continue
      echo "$BIN $MAP $QUERY $tag --param start_s=$s --param duration_s=180 --param level=$lv --param seed=$seed --param band_lo=1.5 --param band_hi=3.0 --param scan_z_lo=0.85 --param scan_z_hi=2.35" >> "$jobs_file"
    done
  done
done
xargs -P "$JOBS" -I{} sh -c '{} 2>/dev/null' < "$jobs_file"
