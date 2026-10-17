# Synthetic experiments for the PRISM-Loc paper

These three standalone drivers produce every number and every data point shown
in the paper's synthetic-evaluation section (`docs/paper/sections/validation.tex`)
and its figures (`docs/paper/figures/fig_mcl.tex`, `fig_kld.tex`, `fig_bbs.tex`,
`fig_eskf.tex`). They compile the **actual estimator cores**
(`prism_loc_core/src/*.cpp`, `prism_loc_fusion/src/*.cpp`) directly with `g++`
and Eigen — no ROS, no CMake, no colcon workspace.

All data are **synthetic**: worlds, laser scans, odometry, and inertial streams
are generated inside the drivers. All randomness derives from **fixed
compile-time seeds** (never wall-clock), so the CSVs are bit-for-bit
reproducible — with the single exception of the wall-clock query-latency column
in the BBS experiment, which is machine-dependent.

Prerequisites: a C++17 compiler and Eigen 3 headers (Debian/Ubuntu:
`sudo apt install g++ libeigen3-dev`; headers land in `/usr/include/eigen3`).
Run all commands from the repository root.

## 1. `mcl_tracking.cpp` — MCL pose tracking + KLD adaptation

Drives the real `prism_loc_core::ParticleFilter` + `Laser2DLikelihoodField`
with KLD-adaptive resampling, in the exact predict → correct → resample order
used by `localization_node.cpp`, around a 65 s closed oval loop at 10 Hz in a
12 m × 10 m occupancy grid. Parameters mirror `prism_loc/params/laser2d.yaml`.
Three fixed seeds: 101, 202, 303.

```sh
g++ -O2 -std=c++17 \
  -I prism_loc_core/include -I prism_loc_core/test -I /usr/include/eigen3 \
  prism_loc_core/src/*.cpp experiments/mcl_tracking.cpp -o /tmp/exp_mcl
/tmp/exp_mcl
```

Output CSVs (written to `docs/paper/data/` relative to the working directory —
run the drivers from the repo root):

- `docs/paper/data/mcl_tracking.csv` — time, position error, yaw error per seed
- `docs/paper/data/mcl_particles.csv` — KLD particle count per seed

Feeds `fig_mcl.tex` and `fig_kld.tex`.

## 2. `bbs_relocalization.cpp` — kidnapped-robot global relocalization

Runs the real `prism_loc_core::BranchAndBoundMatcher` (with the
`laser2d.yaml` `bbs_*` defaults) over N = 100 kidnapped-robot queries in the
asymmetric 6 m × 6 m room from the BBS unit test. Scans are corrupted with a
per-query clutter fraction drawn uniformly in [0, 0.8]. Fixed seed: 20240607.
Success criterion: position error < 0.5 m AND yaw error < 10°.

```sh
g++ -O2 -std=c++17 \
  -I prism_loc_core/include -I prism_loc_core/test -I /usr/include/eigen3 \
  prism_loc_core/src/*.cpp experiments/bbs_relocalization.cpp -o /tmp/exp_bbs
/tmp/exp_bbs
```

Output CSV (same absolute-path caveat as above — the `std::fopen` path near the
bottom of the driver):

- `docs/paper/data/bbs_relocalization.csv` — true/recovered pose, score,
  score fraction, position/yaw error, wall-clock query time, success flag

The `time_ms` column is wall-clock and machine-dependent; every other column is
deterministic. Feeds `fig_bbs.tex`.

## 3. `eskf_fusion.cpp` — ESKF fusion under a 10 s aiding dropout

Drives the real `prism_loc_fusion::Eskf` over a synthetic 60 s 3-D
figure-eight with IMU at 200 Hz, NDT-like 6-DoF pose updates at 10 Hz, and
GNSS-like position updates at 1 Hz (`fusion3d.yaml` noise defaults; fixed true
IMU biases the filter must estimate). Both aiding streams are withheld for
t ∈ [25, 35) s. Fixed seed: 20240517. Logs at 2 Hz to stdout.

```sh
g++ -O2 -std=c++17 \
  -I prism_loc_fusion/include -I /usr/include/eigen3 \
  prism_loc_fusion/src/*.cpp experiments/eskf_fusion.cpp -o /tmp/exp_eskf
/tmp/exp_eskf > docs/paper/data/eskf_track.csv
```

Output CSV:

- `docs/paper/data/eskf_track.csv` — time, position-error norm, the filter's
  own 3-sigma position bound, per-sensor activity flags

Feeds `fig_eskf.tex`.

## Regenerating the paper

After (re-)running the drivers:

```sh
cd docs/paper && tectonic main.tex
```

The figure `.tex` files read the CSVs via pgfplots `table` directives with
paths relative to `docs/paper/`, so no further wiring is needed.

## 4. `bbs_largemap.cpp` — relocalization on large maps, v0.1 window vs map-sized window

Runs the **v0.1** matcher (verbatim copy under `experiments/legacy/`, centred on
the map midpoint with the v0.1 default ±10 m window, which actually searched
−10 m … +15.5 m) and the **current** `BranchAndBoundMatcher::matchGlobal()`
(symmetric map-sized window, occupied/off-map candidates rejected) on the
identical scans. Worlds: 20, 40 and 80 m square warehouse-like grids at
0.1 m/cell with repeated shelf rows and random pillars. Kidnapped poses are
stratified into inside (`in`) and outside (`out`) the v0.1 searched set, 60
each (the 20 m map lies entirely inside it). 180-beam, 30 m scans with the same
U[0, 0.8] clutter model as study 2. Fixed seed: 20260928.

```sh
g++ -O2 -std=c++17 -I prism_loc_core/include -I prism_loc_core/test -I experiments \
  -I /usr/include/eigen3 prism_loc_core/src/*.cpp experiments/legacy/bbs_v01.cpp \
  experiments/bbs_largemap.cpp -o /tmp/exp_bbs_large
/tmp/exp_bbs_large          # ~7 min single-threaded; writes docs/paper/data/bbs_largemap.csv
```

One row per (query, method); `time_ms` is wall-clock, all other columns are
deterministic.

## 5. `eskf_nees.cpp` — Monte Carlo NEES consistency of the ESKF

N = 100 runs of the study-3 scenario, with the initial error and the true IMU
biases drawn from the filter's own prior N(0, P0) in every run; the true biases
then random-walk exactly as the filter's process model assumes. Logs, every
0.5 s, the run-averaged NEES of the full 15-D error state and of each 3-D block,
with the two-sided 95 % chi-square acceptance interval, for the v0.1 filter
(reset Jacobian G = I, suffix `_gi`) and with `EskfParams::reset_jacobian`
(suffix `_rj`) on identical data. Seeds: 20260929 + run index (sensor noise and
initial draws), 20261029 + run index (bias random walk).

```sh
g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3 \
  prism_loc_fusion/src/*.cpp experiments/eskf_nees.cpp -o /tmp/exp_nees
/tmp/exp_nees               # writes docs/paper/data/eskf_nees.csv (deterministic)
```

## `bbs_verify.cpp` + `select_verify_params.py` — multi-scan relocalization verification

Compares the single-scan acceptance rule with `RelocalizationVerifier`
(top-K BBS modes of the first scan, re-matched over M odometry-chained scans,
committed only when one hypothesis holds enough posterior mass) on the maps,
poses and first scans of `bbs_largemap.cpp` (seed 20260928; its `single` rows
reproduce that study's `new` rows). The robot then drives a short random
trajectory (0.3 m steps) with noisy odometry, one cluttered scan per step.

The verifier defaults were chosen on a separate calibration run (seed
20261105: different maps and poses) by `select_verify_params.py`, which
replays the decision rule over a (K, M, gain, posterior) grid and picks the
configuration with the fewest false accepts, then the most correct accepts.

```sh
g++ -O2 -std=c++17 \
  -I prism_loc_core/include -I prism_loc_core/test -I /usr/include/eigen3 \
  prism_loc_core/src/*.cpp experiments/bbs_verify.cpp -pthread -o /tmp/exp_bbs_verify
/tmp/exp_bbs_verify calib 10                          # bbs_verify_calib.csv
python3 experiments/select_verify_params.py --check   # bbs_verify_selection.csv
/tmp/exp_bbs_verify study 3                           # bbs_verify.csv
```

The second argument is the worker-thread count; it changes only `time_ms`.
## 6. `eskf_lever.cpp` — how to fuse a GNSS fix taken off the body origin

Drives the real `prism_loc_fusion::Eskf` (no copy, no ROS) with a simulated IMU, a simulated
GNSS antenna at a lever arm, and optionally simulated NDT pose fixes, and runs 15 ways of
applying the antenna fix on identical data (common random numbers). The schemes are listed
at the top of `eskf_lever.cpp`: `ignore` (the offset not modelled, as on `main` before the
lever arm), `S0` (attitude and gyro bias are consider states, the first version of the lever
arm), `S1` (heading and vertical gyro bias), `S3` (heading), `S2` (none: the full update),
`S4` and `S5` (S2 and S1 iterated three times), `S6` (S1 while the robot stands, the full
update while it moves, with a speed test, a rate test or both; `S6va`, `S6v`, `S6vb` are the
speed test alone at 0.15, 0.3 and 0.6 m/s). A trailing `s` means that the roll/pitch variance
at start and at every reseed comes from the accelerometer instead of the 0.5 rad² of the heading.
**`S6vb` is what the node runs** (the defaults of `EskfParams`), chosen because it had the
fewest diverged runs of the 2,304 configurations below; the speed threshold of 0.6 m/s is the
best of the three tested, not an optimum (driving was simulated at 1.0 to 1.7 m/s only).

The grid (`eskf_lever.cpp`, top comment): duration 60 s (1,920 configurations, 10 seeds) and
300 s (384 configurations, 6 seeds); gyro bias 0, 0.05, 0.1, 0.2, 0.5 deg/s on each axis (300 s:
0, 0.1, 0.5) with a random sign per axis; accelerometer bias 0 and 0.1 m/s²; standing, turning
on the spot with pauses, spinning without pause, driving a planar lemniscate; antenna A
(0.5, 0, 1.0), B (-0.3, 0.2, 1.0) and Z (0, 0, 0, the zero lever: every scheme runs
`updatePosition`, and the results are identical in all 6,400 runs, which the report checks);
without and with NDT fixes; without and with a GNSS dropout (10 s in 60 s runs, 30 s in 300 s
runs); without and with a reseed at half time; simulated IMU noise equal to what the filter
assumes, and twice what `fusion3d.yaml` assumes. The thresholds of "diverged" and the NEES bands are fixed in the file before the final
run; the exploratory passes before it (the first one with the dropout inside the divergence test,
then a shorter list of schemes, the motion gates, the spin motion) are not part of the result.
`std::normal_distribution` is implementation defined: the committed results are from GCC 9.4 /
libstdc++, and the 13 schemes that the last two passes had in common gave identical rows.

```sh
experiments/run_eskf_lever.sh /tmp/lever 4      # about 2 CPU hours: builds, runs both suites, writes report.md
python3 experiments/eskf_lever_report.py experiments/results/eskf_lever/g60 experiments/results/eskf_lever/g300
```

The committed results are `experiments/results/eskf_lever/` (`g60_runs.csv.gz`, `g60_agg.csv.gz`,
`g300_*`: one row per run and scheme, and per configuration and scheme with the NEES bands) and
`report.md`, the output of the second command. The tables in `README.md` come from there.

Not done: a real robot; slopes; GNSS multipath or jumps; driving slower than 1 m/s; an
observability-constrained update or first-estimates Jacobians (Huang, Mourikis and Roumeliotis
2010), the usual remedy for an update that gains information along an unobservable direction,
which was not tried. The scheme that the grid supports is a gate on the speed with two fixed
settings; it is a design choice that the data favour, not a proof.

## 7. `ndt3d_replay.cpp` — rates of the ndt3d status rule

Replays synthetic clouds through the update loop of `prism_loc::LocalizationNode::runUpdate`
for the `ndt3d` backend and measures how often the `/diagnostics` status rule (`match_fraction`,
the mean of the last 5, `min_match_fraction`, the "not verified" state) is right. The driver
`ndt3d_replay.cpp` links the real `prism_loc_core` sources (`ParticleFilter`, `Ndt3DModel`,
`NdtMap`, `OdometryMotionModel`) and the real `prism_loc::MatchMonitor` (`ndt_status.hpp`); only
the order of the loop (skip an update until the robot has moved 0.2 m or 0.2 rad or a reseed
forces one; predict; correct; resample; estimate; `matchStats`; `MatchMonitor::update`) is a copy
of the node's, and the node-level counterpart is the `ndt3d_*` scenarios of
`scripts/synthetic_e2e.py`. The inputs come from `ndt3d_replay_gen.py`, which imports the world,
the trajectory and the odometry error model from `scripts/synthetic_e2e.py` (so it needs the
ROS Python environment: that file imports `rclpy`) and adds the case's perturbation (map noise, moving
columns, boxes added or removed, range noise, a kidnap, other initial poses, a pitched lidar, a
cropped prior map). `ndt3d_replay_batch.py` holds the case list (52 cases) and the seeds (4 per
case: generator seeds 7, 21, 22, 23 and filter seeds 42, 1, 2, 3, fixed in the file), and
`ndt3d_replay_rates.py` turns the per-update CSVs into the tables of the README.

```sh
g++ -O2 -std=c++17 -I prism_loc_core/include -I prism_loc/include -I /usr/include/eigen3 \
  prism_loc_core/src/*.cpp experiments/ndt3d_replay.cpp -o /tmp/ndt3d_replay
python3 experiments/ndt3d_replay_batch.py /tmp/ndt3d_replay /tmp/replay_res '' --jobs 3   # 208 runs, about an hour
python3 experiments/ndt3d_replay_rates.py /tmp/replay_res > /tmp/rates.md                 # threshold 0.65, 15 updates
```

The raw output of the committed pass (one CSV row per filter update, `.sens` files with the
sensitivity probe) is `experiments/results/ndt3d_replay/replay_res.tar.gz` and its tables are
`rates.md` next to it; the README table, `PARAMS.md` and the PR text are copied from there.
The `time_ms` columns are wall-clock and machine dependent; every other column is deterministic
for one compiler and standard library (this pass: GCC 14.3 / libstdc++, the `integ_humble`
environment). The first pass of the replay (a scratch copy of the driver that produced the
earlier table) and this one agree bit for bit on the healthy base case (4 runs) and within 2 percentage
points on every group of the table; the changes are listed in the PR text.

What the replay does not cover: real lidars and maps, slopes, the TF and clock handling of the
node, and the `/diagnostics` table of the README ("Through `/diagnostics` on the node"), which
came from an integration harness outside this repository.
