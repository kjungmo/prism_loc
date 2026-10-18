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

### Third pass (round 5): standing for minutes, then moving

The fourth review found that the second pass had moved the round-3 failure instead of removing it. No motion of the grid stood longer than
14 s before it accelerated, and the filter's start values (heading sigma 0.7 rad against the error) never had a drifted heading under a bounded sigma.
A robot on GNSS and IMU with a lever arm that stood for 600 s with a vertical gyro bias of 0.002 rad/s (the review's bias) had a heading
error of 1.1 rad, the bound held the sigma at 0.35 rad, and the release at the first start ran a full update on a heading about 3 sigma off:
the heading jumped the wrong way, tilt and height diverged (antenna A: +1.3 m and 0.15 rad; antenna B: +2.5 m, -2.6 m, +2.5 m, 0.2 rad), and the filter then reported a heading
sigma of 0.004 rad with the bound's WARN cleared. The design memo was not adequate for this: its prototype never stood long enough
before accelerating, and it took the bounded covariance as the covariance of the release.

New suites `hsg` (the schemes below were developed on these runs) and `hsgv` (the same configurations on other data, run once after the schemes were fixed):
stand 300, 600, 900 or 1,800 s at gyro bias 0 to 0.2 deg/s, then `sgstart` (the start-stop cycle), `sgdrive` (a start, then a circle of 6 m radius at 1.5 m/s) or `sgturn` (turn on the spot) for 160 s;
antennas A, B, M and Z; 4 seeds; 1,152 lever-arm runs and 384 zero-lever runs per scheme and suite. The bar is written in the header of `eskf_lever.cpp`:
in each motion the lever-arm rows have no more `tz` than the zero-lever rows of the same data, and none has |z| over 1 m or tilt over 0.3 rad in `sgstart` and `sgdrive` with the antennas that have a horizontal arm.

What was found, in the order it was found (every step is a scheme in `eskf_lever.cpp`; the exploratory passes between them are not part of the result):
1. **The failure reproduces** (`S7a35`, hsg): `tz` in 375 of 1,152 lever-arm runs, against 324 for the zero-lever runs of the same data; |z| over 1 m in 271.
2. **Putting the shadow variance back at the release is not enough** (`S8r`: 428). Standing for 600 s, the filter's own heading spread is 6 rad (the 0.01 rad/s prior of the vertical gyro bias times 600 s); a linearised full update at that spread
   tilts the filter too. The honest variance is the right one to report, and the wrong one to linearise at.
3. **Kinematic alignment** (`S8a`: 78). Over the last 3 s the antenna fixes give the change of velocity of the antenna, which does not depend on the filter's heading; the IMU, resolved with the filter's heading,
   gives the same change rotated by the heading error. The angle between them is the heading correction, for any error. Three things had to be right: a covariance that stays positive (projecting the heading out of `P`
   instead of zeroing its entries: a first version made `P_vv` indefinite and the filter blew up); the restore must act only after the bound has cut the variance (a first version acted at every update and
   removed the cross-covariances the full update needs); and the velocity the filter integrated with the wrong heading must be replaced by the antenna's.
   A change of velocity of 0.25 m/s starts it (at the stage of `S8a`, 0.4 m/s gave `tz` 102 and 42 runs with |z| over 1 m against 86 and 16 at 0.25 m/s: the first 1.5 s of motion with a heading 2 rad off already tilt the filter).
   A window with a gap of more than 1 s in the fixes, or with a body rate over `0.5 · 0.25 m/s / |t_xy|`, is not used (without the guard a GNSS dropout together with a spin or a turn made `S8` lose the heading, 2.7 to 3 rad at the end, in dozens of runs of the second pass).
   Taking back the roll, pitch and accelerometer bias corrections of the window (tried, not kept) changed nothing.
4. **Blind update** (`S8`: 26). While the heading is held back, over 1 rad of spread and not alignable (a turn on the spot, or a stand), the attitude columns of the lever Jacobian are left out and the circle of the lever is put in `R`:
   the antenna says where the circle is and tells roll and pitch nothing. It removes the turn failure (`sgturn`: 22 of 384 runs with `tz` for `S8a`, 0 for `S8`) and the rest of the standing failures.

Results (`report.md`, third pass; lever-arm runs, 1,152 per scheme and suite):

| | `S7a35` (the version before) | `S8r` | `S8a` | `S8` (shipped) | zero lever, same data |
|---|---|---|---|---|---|
| hsg: `tz` | 375 | 428 | 78 | **26** | 324 |
| hsg: |z| over 1 m / tilt over 0.3 rad | 271 / 161 | 292 / 182 | 15 / 5 | **1 / 0** | |
| hsg: `tz` where the zero-lever run has none | 215 | 249 | 49 | 12 | |
| hsgv (other data): `tz` | 381 | 431 | 83 | **21** | 366 |
| hsgv: |z| over 1 m / tilt over 0.3 rad | 279 / 157 | 290 / 189 | 21 / 6 | **0 / 0** | |
| hsgv: the bar (`sgstart`, `sgdrive`, `sgturn`) | no, no, no | no, no, no | yes, yes, no | **yes, yes, yes** | |
| hsgv: heading error over 0.5 rad at the end, `sgstart` | 138 of 384 | 138 | 0 | **0** | |

What is left, in hsg: 12 runs with `tz` where the zero-lever run has none (11 with the 1.5 m mast M, whose vertical arm amplifies a tilt error of 0.05 rad), one run with |z| over 1 m
(mast, 0.2 deg/s, stand 900 s); `sgturn` ends with the heading over 0.5 rad in 267 of 384 runs (nothing in a turn on the spot tells the heading; the zero-lever filter has the same: 106 of 128 runs) and the position of `base_link` over 1 m from the truth in 35 (`xy`, the circle of the lever).

The second pass, with the rows of `S8` added (`LEVER_ONLY=S8`, `merge_rows.py`): the clean envelope has 0 `tz` for both schemes; all cells 883 `tz` for `S8` against 880; 213 runs over 1 m against 156 (of the 78 runs that
are over 1 m for `S8` and not for `S7a35`, 59 are antenna A at 0.5 deg/s, where the heading is 2.7 rad off and the circle of the lever is 1.0 m: 1.00 to 1.05 m against 0.96 to 0.99 m); spinning 300 s at 0.5 deg/s: `tz` 0 of 24 (2 for `S7a35`), `circ` 2 (0); with the lidar `tz` 0 for both;
reseed on a slope: `tz` 2, 2 and 1 of 144 for 0, 0.05 and 0.1 rad (0, 0, 0), tilt at the end over 0.02 rad in 0 (the ablation `S7n`: 41 and 60).
**Worse than `S7a35`:** a start heading 1 rad or 2 rad off (suite `hyaw`, `startstop`, 24 runs each): recovered (heading error under 0.2 rad at the end) in 14 and 8 runs (21 and 3 for `S7a35`); `tz` 19 and 24 (16 and 24). The alignment starts at a spread of 1 rad, and the filter
starts at 0.71 rad. A trial with the threshold at 0.6 rad (`S8s`, `LEVER_ONLY=S8s`, not part of the committed results) recovers all 24 runs at both errors (`tz` 1 and 1), but has `tz` 41 against 26 in hsg and 12 against 5 in the reseed-on-a-slope suite
(an alignment on a normal start adds its own error). The shipped threshold is 1.0 rad; a start heading off by more than about 0.5 rad followed by a short stand stays outside what was measured to work.

```sh
experiments/run_eskf_lever.sh /tmp/lever 4   # builds, runs the twelve suites, writes report.md (first pass 2 CPU hours, second pass about 2.5 CPU hours;
                                             # the committed results took 57 minutes of wall time with 4 jobs for 11 schemes, 13 more for the two added ones;
                                             # hsg and hsgv: 6 schemes, about 25 minutes each with 11 jobs)
LEVER_ONLY=S7a35r05,S7a35r20 ...            # run only these schemes (how the two added schemes were run; a full run gives the same rows)
```

Not done in this pass: a real robot; slopes under motion (the tilt is constant); a bound below 0.35 rad (the smallest tested);
release thresholds between the three tested; a heading start error with a bound at 0.35 rad other than the three tested; an
observability-constrained update or first-estimates Jacobians (above).
