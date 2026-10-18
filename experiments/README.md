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
`S6vb` is what the node ran in the round that this pass belongs to (round 3); it was chosen because
it had the fewest diverged runs of the 2,304 configurations below, and the second pass below
(round 4) replaces it: the node now runs `S7a35`. The speed threshold of 0.6 m/s was the best of the
three tested, not an optimum (driving was simulated at 1.0 to 1.7 m/s only).

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
experiments/run_eskf_lever.sh /tmp/lever 4      # builds, runs all ten suites (first pass about 2 CPU hours), writes report.md
python3 experiments/eskf_lever_report.py experiments/results/eskf_lever/g60 experiments/results/eskf_lever/g300  # the first pass alone
```

The committed results are `experiments/results/eskf_lever/` (`<suite>_runs.csv.gz`, `<suite>_agg.csv.gz`
for `g60`, `g300` and the second-pass suites: one row per run and scheme, and per configuration and scheme with the
NEES bands) and `report.md`, the output of `eskf_lever_report.py` on all of them. The tables in `README.md` come from
there, and its last section ("Numbers quoted in README.md and the PR text") prints every number that `README.md`,
`SPEC_fusion.md` and the PR text quote and that no table prints as such. Every column of the files of the previous
round has the same values in `g60` and `g300` here (the second pass added columns and suites; both were rerun with the final code
and compared column by column).

Not done: a real robot; slopes; GNSS multipath or jumps; driving slower than 1 m/s; an
observability-constrained update or first-estimates Jacobians (Huang, Mourikis and Roumeliotis
2010), the usual remedy for an update that gains information along an unobservable direction,
which was not tried. The scheme that the grid supports is a gate on the speed with two fixed
settings; it is a design choice that the data favour, not a proof.

### Second pass (round 4): what one antenna and an IMU observe

The first pass chose `S6vb`, a gate on the filter's speed. The third review round showed what that
leaves open: a robot that stands for minutes on GNSS and IMU alone leans and its height runs
(the reviewer's runs of 900 s standing: 32 of 32 failed; 192 of 192 in this grid). The second pass looks at that case, on a grid that is declared in
the top comment of `eskf_lever.cpp`, with its failure criteria and its selection rule, before the
first run of any `h` suite.

Schemes (all with the roll/pitch variance from the accelerometer at start and reseed, `s`):
`S0`, `S1s`, `S2s`, `S6vb` as in the first pass; `S7` = `S1s` + a bound on the heading standard
deviation that the lever update sees (0.7 rad, `lever_heading_sigma_max`) and the attitude of a reseed from
the accelerometer at the heading of the message; `S7a` = `S7` + the consider states released while
the IMU measures horizontal acceleration (`lever_release_dv` 1.0 m/s over a 4 s decaying window);
`S7a35`, `S7a50` = the bound at 0.35 and 0.5 rad; `S7r05`, `S7r20` = the release threshold at 0.5
and 2.0 m/s; `S7n` = `S7a` with the level attitude of `/initialpose` (an ablation, for the slope);
`S7a35r05`, `S7a35r20` = `S7a35` with the release threshold at 0.5 and 2.0 m/s (added after the
first results, when the bound of 0.35 rad was ahead of 0.5 and 0.7: the thresholds had been swept at
0.7 only). The grid: motions stand, line (1.5 m/s), start-stop (a 40 s cycle with 0.5 m/s² for 3 s),
turn, drive, spin; 300 s (2 seeds, `h300`), 600 s standing (8 seeds, `h600`), 900 s and 1,800 s standing and line
(8 seeds, `h900`, `h1800`); gyro bias 0, 0.05, 0.1, 0.2, 0.5 deg/s, accelerometer bias 0, 0.1, 0.3 m/s²,
dropouts of 30 s and 120 s, a reseed at half time; antennas A, B, Z and a 1.5 m mast M (0, 0, 1.5); NDT on
(`hndt`); a reseed at 30 s of 60 s on a body tilted by 0.05 and 0.1 rad (`hslope`); a start heading 0.5, 1
and 2 rad off (`hyaw`); accelerometer white noise of 0.02 to 0.4 m/s² per axis (`hvib`). The four antennas of a configuration share one data set, so the zero-lever row
is the paired reference of the lever-arm rows.

Failures are counted by kind, apart: `tz` (tilt over 0.05 rad or height over 0.5 m), `circ` (the horizontal error
outside the circle of radius `|t_xy|` around the antenna plus 0.3 m), `xy` (over 1 m), `heading` (over 0.5 rad).
The selection rule is in the file: the candidates `S7`, `S7a`, `S7a35`, `S7a50`, `S7r05`, `S7r20`, `S7a35r05`, `S7a35r20`
are ordered by E (runs with `tz` or `circ` in the cells of a standing, line, start-stop or turning robot at up to 0.2 deg/s),
then by X (`tz` beyond the zero-lever row in the other cells), the NEES conditions, the heading failures against `S6vb` while driving,
the median of the largest error, and the number of parameters. The first look at `h300` showed that the zero-lever filter
fails too in some of the cells of E (an accelerometer bias of 0.3 m/s², dropouts of 120 s), so that E > 0 for every candidate; the
sentence that says the order is then applied with E as its first key, and the column E' (the runs of E that the zero-lever row of
the same data survives), were added to the declaration and the report at that point. The criteria and the cells did not change.

Result on the GNSS-and-IMU-only rows with a lever arm of `h300`, `h600`, `h900`, `h1800` (4,200 runs per scheme: 2,448 in the cells of E,
1,752 in the others; the zero-lever reference has 1,400 runs and `tz` or `circ` in 178 of its 816 runs in the cells of E):

| scheme | bound rad | release m/s | E: `tz` or `circ` in the cells of E | E': of these, runs the zero-lever row of the same data survives | X: `tz` beyond the zero-lever row, other cells | median largest error in the cells of E, m |
|---|---|---|---|---|---|---|
| `S7a35r05` | 0.35 | 0.5 | 451 | 45 | 30 | 0.135 |
| **`S7a35`** | **0.35** | **1.0** | **454** | **50** | **35** | **0.143** |
| `S7a50` | 0.5 | 1.0 | 489 | 79 | 55 | 0.195 |
| `S7a35r20` | 0.35 | 2.0 | 502 | 92 | 89 | 0.170 |
| `S7r05` | 0.7 | 0.5 | 931 | 489 | 51 | 0.253 |
| `S7a` | 0.7 | 1.0 | 942 | 494 | 67 | 0.259 |
| `S7` | 0.7 | never | 1,071 | 614 | 274 | 0.306 |
| `S7r20` | 0.7 | 2.0 | 1,072 | 615 | 129 | 0.304 |
| `S6vb` (round 3) | none | speed gate | 1,280 | 801 | 319 | 0.328 |
| `S2s` (full update) | none | always | 532 | 56 | 16 | 0.161 |
| `S1s` | none | never | 1,930 | 1,413 | 671 | 0.867 |
| `S0` | | | 1,515 | 1,028 | 829 | 0.400 |

No candidate has E = 0: the zero-lever filter fails in 178 of its 816 runs of those cells too (an accelerometer bias of 0.3 m/s² and the
120 s dropouts), so the order was applied with E as its first key (the sentence above). It puts `S7a35r05` first, 3 runs of E and 5 of X
ahead of `S7a35`, which two seeds per configuration cannot tell apart. The code ships `S7a35`, a deviation from the declared order
for the reason in the last table: the release at 0.5 m/s is open in 65 % of the updates of a standing robot at an accelerometer noise of
0.1 m/s² per axis, 1.0 m/s from 0.2 m/s² (0.56) and 2.0 m/s from 0.4 m/s² (0.58); the integral of `|a_xy|` of white noise settles at
about 5 σ, so the threshold has a noise floor (the `hvib` table in `report.md` has the counts of `tz`, `circ` and `xy` too: none at 1.0 m/s). `S2s`, the full update, has as few `tz` as `S7a35` (939 of 4,200 runs against 880) but locks
the heading wrong while the robot stands (heading over 0.5 rad in 2,509 runs against 1,599) and its position NEES is inside its band in
18 % of the standing configurations (31 % for `S7a35`).


```sh
experiments/run_eskf_lever.sh /tmp/lever 4   # builds, runs the ten suites, writes report.md (first pass 2 CPU hours, second pass about 2.5 CPU hours;
                                             # the committed results took 57 minutes of wall time with 4 jobs for 11 schemes, 13 more for the two added ones)
LEVER_ONLY=S7a35r05,S7a35r20 ...            # run only these schemes (how the two added schemes were run; a full run gives the same rows)
```

Not done in this pass: a real robot; slopes under motion (the tilt is constant); a bound below 0.35 rad (the smallest tested);
release thresholds between the three tested; a heading start error with a bound at 0.35 rad other than the three tested; an
observability-constrained update or first-estimates Jacobians (above).

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
