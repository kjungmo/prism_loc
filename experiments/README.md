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

| | `S7a35` (the version before) | `S8r` | `S8a` | `S8` (shipped in round 5; the fifth round replaced it with `S8u`) | `S8u` (fifth round) | zero lever, same data (each run once) |
|---|---|---|---|---|---|---|
| hsg: `tz` | 375 | 428 | 81 | **26** | 49 | 108 of 384 |
| hsg: |z| over 1 m / tilt over 0.3 rad | 271 / 161 | 292 / 182 | 18 / 6 | **1 / 0** | 3 / 0 | 35 / 1 |
| hsg: `tz` where the zero-lever run has none | 215 | 249 | 52 | 12 | 24 | |
| hsgv (other data): `tz` | 381 | 431 | 83 | **21** | 52 | 122 of 384 |
| hsgv: |z| over 1 m / tilt over 0.3 rad | 279 / 157 | 290 / 189 | 21 / 6 | **0 / 0** | 4 / 0 | 47 / 5 |
| hsgv: the bar (`sgstart`, `sgdrive`, `sgturn`) | no, no, no | no, no, no | yes, yes, no | **yes, yes, yes** | yes, yes, yes | |
| hsgv: heading error over 0.5 rad at the end, `sgstart` | 138 of 384 | 138 | 0 | **0** | 0 | 5 of 128 |

What is left, in hsg: 12 runs with `tz` where the zero-lever run has none (11 with the 1.5 m mast M, whose vertical arm amplifies a tilt error of 0.05 rad), one run with |z| over 1 m
(mast, 0.2 deg/s, stand 900 s); `sgturn` ends with the heading over 0.5 rad in 266 of 384 runs of hsg and 258 of hsgv for `S8` (nothing in a turn on the spot tells the heading; the zero-lever filter has the same: 106 of 128 runs in hsg, 97 of 128 in hsgv) and the position of `base_link` over 1 m from the truth in 33 (hsg) and 35 (hsgv) of the 384 (`xy`, the circle of the lever).
The report prints these counts (section "Numbers quoted in README.md and experiments/README.md" under each suite). The zero-lever column counts each run once: 108 of 384 runs in hsg, 122 in hsgv. An earlier version of this table counted each zero-lever run once for every lever-arm antenna that shares its data, three times (324 and 366), and called it 'zero lever, same data'.

The second pass, with the rows of `S8` added (`LEVER_ONLY=S8`, `merge_rows.py`): the clean envelope has 0 `tz` for both schemes; all cells 880 `tz` for `S8` against 880 (the round before printed 883 for `S8`: the rows of h300 were rerun with the fifth round's alignment check, which changed 21 of 4,320 rows, all in dropout cells); 213 runs over 1 m against 156 (of the 65 runs that
are over 1 m for `S8` and not for `S7a35`, 59 are antenna A at 0.5 deg/s, where the heading is 2.7 rad off and the circle of the lever is 1.0 m: 1.00 to 1.29 m against 0.65 to 1.00 m; 8 runs are over 1 m for `S7a35` only; the round before said 78 and 1.00 to 1.05 m, which no script printed); spinning 300 s at 0.5 deg/s: `tz` 0 of 24 (2 for `S7a35`), `circ` 2 (0); with the lidar `tz` 0 for both;
reseed on a slope: `tz` 2, 2 and 1 of 144 for 0, 0.05 and 0.1 rad (0, 0, 0), tilt at the end over 0.02 rad in 0 (the ablation `S7n`: 41 and 60).
**Worse than `S7a35`:** a start heading 1 rad or 2 rad off (suite `hyaw`, `startstop`, 24 runs each): recovered (heading error under 0.2 rad at the end) in 14 and 8 runs (21 and 3 for `S7a35`); `tz` 19 and 24 (16 and 24). The alignment starts at a spread of 1 rad, and the filter
starts at 0.71 rad. A trial with the threshold at 0.6 rad (`S8s`, `LEVER_ONLY=S8s`, not part of the committed results) recovers all 24 runs at both errors (`tz` 1 and 1), but has `tz` 41 against 26 in hsg and 12 against 5 in the reseed-on-a-slope suite
(an alignment on a normal start adds its own error). The shipped threshold is 1.0 rad. The fifth round replaced the trial with `S8u` (below): the heading of a start without a heading source is held as unknown, which lets the alignment act at the first start whatever the stand.

### Fifth pass (round 6): an outage across the start, and a boot with a guessed heading

(Round 7 renamed the scheme `S8u` of this pass to `S8k`; its rows are bit for bit the same, and the name `S8u` now stands for the code of round 7, sixth pass below. Everything in this section that says `S8u` is `S8k`.)

The fifth review found two failures of `S8` that the first four passes did not have a motion for. `eskf_lever.cpp` has two new suites, `hgap` and `hboot` (the header of the file says how they were written), and two new schemes: `S8o`, `S8` without the new check of the alignment
(`EskfParams::lever_align_span_guard = false`, the version of the round before), and `S8u`, `S8` + the heading of a start without a heading source held as unknown (`Eskf::markHeadingUnknown()`, `lever_start_sigma` = π/√3 rad).
Both suites have 576 lever-arm runs and 192 zero-lever runs per scheme (4 seeds, antennas A, B, M, Z, GNSS and IMU only).

1. **An outage across a start** (`hgap`: stand 300 or 600 s, the fixes stop 10 s before the robot starts the start-stop cycle or the circle and come back 30 s or 120 s later; gyro bias 0, 0.1, 0.2 deg/s; accelerometer bias 0, 0.1 m/s²).
   The alignment looked for a gap in the fixes only in the last 4 s. The walk back to the fix 3 s earlier then stopped at the last fix before the outage, and the change of velocity was taken from the velocity before the outage and the IMU sum of the whole outage. A reviewer's trace (stand 600 s, 120 s outage, configuration 100, seed 1) aligned 0.8 s after
   the fixes came back, with a heading error of 2.95 rad and a spread of 0.35 rad, locked it at 0.08 rad after the release, and ended at a tilt of 0.73 rad and a height of −43 m. The fix checks the gaps over the first fix of the older velocity to now and refuses a window longer than 4 s.
2. **The default boot** (`hboot`: `initial_yaw` 0.5, 1, 2 and 3 rad off, no `/initialpose`, stand 6, 30, 60, 90, 120 or 300 s, then the start-stop cycle or the circle). The shadow of the heading starts at the 0.5 rad² of `P0`, 0.71 rad, which is under the 1 rad at which the alignment acts; the first start then
   runs a full update with the heading 1 to 3 rad off. `S8` aligns only after about 70 s of standing, when the shadow has grown past 1 rad.

**The design memo was not adequate for finding 2.** `PR7_OBSERVABILITY_DESIGN.md` (§4) started the heading at the filter's own covariance and held that a start is unchanged by the bound (σ_max = √0.5, "the start value"). That is right when `initial_yaw` is within about 0.5 rad, and wrong for the default boot of a robot without a heading source,
where `initial_yaw` is a parameter that the deployer cannot set to the heading of a robot that boots facing a random way. The reviewer's prototype (`S8u`) and the data below are the evidence for the change.

Results (`report.md`, fifth pass; counts of runs):

| | `S8o` | `S8` | `S8u` (now `S8k`) | zero lever, same data (each run once) |
|---|---|---|---|---|
| hgap, 120 s: `tz` (of 288) | 68 | **11** | 12 | 11 of 96 |
| hgap, 120 s: |z| over 1 m / tilt over 0.3 rad | 36 / 7 | **2 / 0** | 4 / 1 | |
| hgap, 120 s: `tz` where the zero-lever run has none | 58 | 11 | 11 | |
| hgap, 30 s: `tz` (of 288) | 19 | 12 | 13 | 18 of 96 |
| hgap, both: |z| over 1 m / tilt over 0.3 rad (of 576) | 38 / 7 | 4 / 0 | 6 / 1 | |
| hboot: `tz` (of 576) | 219 | 219 | **77** | 112 of 192 |
| hboot: |z| over 1 m / tilt over 0.3 rad | 160 / 104 | 160 / 104 | **3 / 0** | |
| hboot: heading error over 0.5 rad at the end | 162 | 162 | **11** | |
| hboot: antenna A / B / M, `tz` (of 192 each) | 58 / 72 / 89 | 58 / 72 / 89 | **1 / 0 / 76** | |
| hboot, `initial_yaw` 1 rad, stand 6 to 60 s: `tz` (of 72) | 54 | 54 | **0** | 8 of 24 |
| hboot, `initial_yaw` 2 and 3 rad, stand 6 to 60 s: `tz` (of 144) | 144 | 144 | **34** | 45 of 48 |
| hboot, `initial_yaw` 2 and 3 rad, stand 90 to 300 s: `tz` (of 144) | 13 | 13 | 41 | 43 of 48 |

`S8` and `S8o` give the same rows in hboot (no gap in the fixes). The antennas A and B have 1 run of 384 with `tz` under `S8u` (tilt 0.051 rad). The mast M, whose lever has no horizontal part, fails like the zero-lever filter (76 of 192 against 112 of 192; largest height error 1.34 m, largest tilt 0.13 rad):
a wrong heading does not enter its lever term, but after the alignment at the first start the horizontal accelerometer-bias estimate jumps to 0.4 to 0.7 m/s² for some seconds and the height drifts by up to 1 m (a trace of configuration 187, seed 0). A try to leave the blind update off for an antenna with a horizontal arm under 0.1 m (a mast) did not help (not kept).
With a stand of 90 s or more at 2 or 3 rad, `S8` is better than `S8u` in the mast (13 against 41 `tz` of 144): by then `S8`'s shadow has grown past 1 rad, and it has had 90 s of updates with the full Jacobian before the first start; `S8u` is blind from the first update.

What `S8u` costs on the earlier suites, with the same code and the 0.1 rad start error they use (`LEVER_ONLY=S8,S8u`; the rows of `S8` are bit for bit those of the round before wherever the data has no gap in the fixes, and 21 of the 4,320 `S8` rows of h300 changed, all in dropout cells):
- second pass (4,200 runs): `tz` 962 against 880, runs over 1 m 249 against 213, the clean envelope 0 `tz` for both; 117 runs gain `tz` and 35 lose it, 104 of the 117 are drive and start-stop runs that begin moving within 6 s of the start, 83 of the 117 have an accelerometer bias of 0.3 m/s²; the zero-lever run has `tz` in 24 of the 117.
- third pass: hsg 49 `tz` against 26, hsgv 52 against 21; the bar holds; of the 24 runs with `tz` where the zero-lever run has none in hsg 23 are the mast (22 of 22 in hsgv).
- a reseed on a slope (`hslope`): 7 and 4 runs with `tz` of 144 at 0.05 and 0.1 rad, against 2 and 1; with the lidar (`hndt`): 0 for both.
- a start 1 and 2 rad off (`hyaw`, start-stop): recovered in 24 and 23 of 24 runs (14 and 8 for `S8`), `tz` 0 and 4 of 24 (19 and 24). Drive from rest at t = 0 (the robot already moves at 1.7 m/s when the filter starts): `tz` 24 of 24 at 2 rad for every scheme, over 1 m in 18 runs (9 for `S8`).
The costs are in the cells where the heading was right and the filter was told to doubt it. A robot whose heading at the start is right to about 0.3 rad has two ways to keep the numbers of `S8`: `/initialpose`, and `initial_yaw_known: true`.

Not done in this pass: a start-up heading between 0 and 0.5 rad off with a stand of 90 s or more (the grid has 0.5 rad as its smallest); the alignment with a lever arm smaller than 0.1 m other than the 1.5 m mast; a real robot.

### Sixth pass (round 7): the fix rate, and a short outage at the first start

The sixth review found two defects of `S8u` that the first five passes had no cell for. `eskf_lever.cpp` has a new suite, `hrate`, and new cells in `hgap` (the header of the file says how they were written). In this round the scheme `S8u` of the fifth pass was renamed `S8k` (its rows are bit for bit the same:
the harness reproduces all 1,536 `S8u` rows of `hsg` and `hsgv` and the 768 of `hboot` as `S8k`), and the name `S8u` went to the code of this round. Schemes added: `S8t` (the spans in seconds only), `S8n` (the test of the norms only), `S8w` (`S8t` + the limit of the fix period), `S8v` (`S8w` + the spread after an outage + the test of the norms),
`S8u` (`S8v` without the test of the norms: what the code ships), `S8x` (`S8u` with `lever_outage_spread` and `lever_align_norm` from the environment, `LEVER_SPREAD` and `LEVER_NORM`, for the sweep). The harness has a fix period per configuration (`Config::gper`; the IMU stays at 100 Hz) and an outage start per configuration (`Config::drop_off`).
`merge_round7.py` keeps the committed `S8u` rows as `S8k` and adds the new rows; the rows of `S8` that were run again (`hslope`, `hsg`, `hsgv`) are identical to the committed ones (printed). The first pass and the other rows of the second pass do not use the alignment and are not run again.

1. **The fix rate** (`hrate`: the boot cells and the stand-then-go cells at fix periods of 2, 1, 0.5, 0.2, 0.1 and 0.05 s; the IMU data of a cell is the same at every rate; 136 configurations per rate, 4 seeds, antennas A, B, M, Z).
   The alignment took each of its two velocities over 4 fixes: 4 s at 1 Hz, so the span never passed its own check (`dt < 2 s`) and a robot that booted with an unknown heading ran blind for good; 0.4 s at 10 Hz, where the check `dt > 0.4 s` was decided by the rounding of the clock, and a span that passed had the fix noise of 0.07 m/s per velocity,
   which gave alignments at constant speed. A trace of the round before at 10 Hz (reviewer's configuration 16): the first start was not aligned, and an alignment came 7 s after it at constant speed, with the heading 0.9 rad off and a spread of 0.09 rad.
   Fix: the spans are seconds (`lever_align_span` 0.8 s and at least 2 fixes), and the window, the gap limit and the history scale with the measured fix period (the median of the last 15 intervals); under about 0.8 Hz the alignment does nothing, the start is not marked as unknown, and `/diagnostics` WARNs.
   Results (antennas A and B, runs with `tz` / heading over 0.5 rad at the end; boot 144 runs and stand-then-go 128 runs per rate):

   | fix rate | boot `S8k` | boot `S8u` | stand-then-go `S8k` | stand-then-go `S8u` |
   |---|---|---|---|---|
   | 0.5 Hz | 48 / 89 | 54 / 83 | 53 / 64 | 51 / 64 |
   | 1 Hz | 36 / 89 | **0 / 0** | 41 / 64 | **0 / 0** |
   | 2 Hz | 5 / 5 | **0 / 0** | 1 / 0 | 1 / 0 |
   | 5 Hz | 0 / 1 | 0 / 1 | 0 / 0 | 0 / 0 |
   | 10 Hz | 31 / 62 | **0 / 0** | 39 / 64 | **0 / 0** |
   | 20 Hz | 38 / 89 | **0 / 0** | 40 / 64 | **0 / 0** |

   `S8t` (the spans only) has the counts of `S8u` within 1 run at 1 to 20 Hz; at 0.5 Hz it has 32 and 17 runs with `tz` (against 54 and 51) and 38 and 28 with the heading over 0.5 rad: better, not reliable, and the limit stays at 0.8 Hz.
   **The test of the norms of the two changes of velocity** (the review's proposal, `lever_align_norm` 0.3; `S8n` alone, `S8v` with the rest): it changes 1 to 9 of the 544 rows of a rate from 1 to 20 Hz and no count of the table; `S8n` (the test without the spans) differs from `S8k` in 0 to 33 of the 544 rows of a rate and has the counts of `S8k` within 3 runs (10 Hz, boot: `tz` 32 against 31), so the test alone does not repair the rates either. With the spans in seconds there is no alignment to remove. In the outage cells it makes things worse:
   the node scenario `gnss_gap_across_start` on the core (`eskf_gap_across_start.cpp`, 300 seeds) fails in 2 runs with the test alone (0 without), in 4 with the test and the spread at a tenth (1 without the test), in 13 with the test and the whole spread (4 without). A refused marginal alignment is followed by one on a window that the corrections after the outage have polluted.
   **The design memo had no fix-rate axis** (`PR7_OBSERVABILITY_DESIGN.md`, §6 runs the grid at the 5 Hz of `kGnssPeriod`, and its node checks send 5 Hz); the release by acceleration (§4b) reads the IMU and does not depend on the rate, but the alignment of the rounds 4 to 6 is not in the memo and measures the fixes. The memo needs the axis, and `hrate` is it.
2. **A short outage at the first start** (`hgap`, 640 new configurations: the fixes stop 1 s before or in the middle of the first acceleration after a stand of 300 or 900 s, for 1.5, 3, 5 or 12 s; gyro bias 0 or 0.2 deg/s; sgstart at 5 and 7.5 s, sgdrive at −1, 1.5 and 5 s). The reviewer's probe (`rgap`) had these cells for sgstart and sgdrive at 5 s.
   The failure: antenna A kept a horizontal error of 3 m for about 20 s (configuration 756, seed 0 of `hgap`: at the return of the fixes the velocity was off by 1.9 m/s, the alignment came 22 s later), antenna B 2.6 m, and the mast, which has no horizontal arm to absorb the error, diverged to a tilt of 3 rad and a height of 1,600 m (configuration 675, seed 0: 2,278 m; 16 of 640 mast runs).
   The cause: with the heading unknown and held at its bound in `P`, `predict` dead-reckoned the acceleration of the outage in a heading that was off by up to 3 rad; the velocity and the position it gained are the true ones rotated by that error, and `P` said they were good. The first fixes explained the difference as a tilt and an accelerometer bias.
   **The design memo was not adequate here either.** §4a shrinks the heading error variable (`P ← D P Dᵀ`) so that the lever term stays in its linear range, and §7 says what the filter then reports; neither says what the shrink does to the velocity and position that depend on the heading during dead reckoning. The honest heading spread (the shadow of rounds 5 and 6) is reported but never reaches `P`.
   The change: at the first lever update after an outage, with the heading unknown and no alignment possible, `Eskf::spreadAfterOutage()` adds to the horizontal position and velocity block of `P` the matrix `s · min(σ², 2)/2 · [[|D|², D·dS], [D·dS, |dS|²]]` on each axis (`dS`: the velocity the IMU summed over the outage, `D`: its integral; `s` = `lever_outage_spread`). Evidence for `s` (hgap, `S8x`, no test of the norms; short outages, antennas A and B / mast; old cells = the 30 s and 120 s outages of the fifth pass):

   | `lever_outage_spread` | short, A and B: `tz` / over 1.5 m | short, mast: `tz` / height over 1 m / diverged | long, A and B: `tz` (30 s, 120 s of 192) | long, mast: `tz` (of 96, 96) | `gnss_gap_across_start` replica: failing seeds of 300 |
   |---|---|---|---|---|---|
   | 0 (`S8k`) | 83 / 76 | 154 / 59 / 16 | 6, 4 | 7, 8 | 0 |
   | 0.02 | 43 / 59 | 138 / 22 / 1 | 6, 3 | 10, 8 | 1 |
   | 0.05 | 38 / 39 | 134 / 18 / 0 | 5, 3 | 10, 9 | 1 |
   | **0.1 (`S8u`)** | **32 / 34** | **132 / 16 / 0** | **4, 2** | **8, 9** | **1** |
   | 0.25 | 35 / 36 | 134 / 14 / 0 | 5, 2 | 8, 12 | 2 |
   | 0.5 | 34 / 41 | 133 / 15 / 0 | 5, 2 | 9, 17 | 3 |
   | 1 | 36 / 41 | 132 / 14 / 0 | 5, 1 | 9, 21 | 4 |

   A tenth gives the whole gain of the short outages and the smallest cost in the long ones (the mast at 120 s: 9 of 96 against 8 without the spread and 21 with the whole of it). The final runs of `S8u` (`hgap`, 1,280 runs of A and B, 640 of M, 640 zero-lever): A and B `tz` 32, over 1.5 m 34, height over 1 m 8; mast `tz` 132, height over 1 m 16, diverged 0, largest height error 2.7 m (`S8k`: 2,310 m); zero lever `tz` 226, height over 1 m 84.
3. **What is left** (all listed in the README under "Sixth pass"): the mast has `tz` in 132 of 640 runs of the short outages (the zero lever 226); antenna A on a circle that starts during the outage never finds its heading, because a body rate of 0.25 rad/s with an arm of 0.5 m is the limit of the alignment's rate rule (`0.125 m/s / |t_xy|`; 32 of 128 runs over 1.5 m; antenna B 0 of 128);
   an alignment from a weak change of velocity right after an outage can be wrong by a radian under a spread of 0.35 rad (the IMU sum is polluted by the tilt and bias corrections that follow the outage), and the next release can turn the heading the wrong way (4 of 300 seeds of the replica with the whole spread). An honest spread after such an alignment was not tried.

On the earlier suites `S8u` equals `S8k` where the data has no outage: all 1,536 rows of `hsg` and of `hsgv`, `hndt`, `hvib`, `h900`, `h1800`; `hboot` has the same counts (`tz` 77); the second pass has 962 `tz` in 4,200 runs for both and 248 and 249 runs over 1 m; the changes are in the dropout cells (`h300`, `h600`, `hslope`, `hyaw`). Counts: `report.md`, sixth pass.

```sh
experiments/run_eskf_lever.sh /tmp/lever 4   # builds, runs the fifteen suites, writes report.md (first pass 2 CPU hours, second pass about 2.5 CPU hours;
                                             # the committed results took 57 minutes of wall time with 4 jobs for 11 schemes, 13 more for the two added ones;
                                             # hsg and hsgv: 6 schemes, about 25 minutes each with 11 jobs)
LEVER_ONLY=S7a35r05,S7a35r20 ...            # run only these schemes (how the two added schemes were run; a full run gives the same rows)
LEVER_ONLY=S8x LEVER_SPREAD=0.1 LEVER_NORM=0 eskf_lever hgap 0 4 /tmp/sweep   # the sweep of the sixth pass: lever_outage_spread and lever_align_norm from the environment
python3 experiments/merge_round7.py MAIN_DIR SUP_DIR OUT_DIR                  # round 7: the committed S8u rows become S8k, the re-run S8u and S8v rows are added
g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3 prism_loc_fusion/src/*.cpp experiments/eskf_gap_across_start.cpp -o /tmp/gap && NSEED=300 /tmp/gap   # gnss_gap_across_start on the core
# round 7 on 4 jobs: hrate about 12 minutes, hgap about 14, hsg and hsgv about 3 each (S8, S8u, S8v), the second-pass suites S8u and S8v together about 6
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
