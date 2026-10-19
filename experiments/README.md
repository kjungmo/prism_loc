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

`eskf_lever.cpp` drives the real `prism_loc_fusion::Eskf` (no copy, no ROS) with a simulated IMU, a simulated GNSS antenna at a lever arm, and
optionally simulated NDT pose fixes. It runs the `ignore` comparator and three schemes on identical data (common random numbers). The schemes
are built from `EskfParams` at the top of the file:

| Scheme | What it is |
|---|---|
| `ignore` | the antenna offset is not modelled (what `main` did before the lever arm) |
| `S1s` | heading and vertical gyro bias are consider states; no bound, no release, no alignment |
| `S2s` | no consider state: the full update of every state |
| `S8u` | the code that ships: the `EskfParams` defaults, and a start with a guessed heading marked unknown |

A trailing `s` means that the roll and pitch variance at the start and at every reseed comes from the accelerometer (about 1e-4 rad²), not the
0.5 rad² of the heading. The antennas are A = (0.5, 0, 1.0), B = (−0.3, 0.2, 1.0), M = (0, 0, 1.5) (a mast: no horizontal arm) and Z = (0, 0, 0)
(the zero lever: every scheme runs `updatePosition`, so its rows are the same in all schemes, which the report checks).

**Which core the rows come from.** The rows of `S1s`, `S2s` and `S8u` in the committed CSVs are written by the harness and the core of this tree
(round 10); `compare_rows.py` reproduces them bit for bit ("Reproducing"). The other schemes in the CSVs (`S0` to `S8x`, `S6vb`, `S7a35` and
the rest) differed from `S8u` by switches that the core no longer has: a speed and rate gate, an iterated update, other consider masks,
`lever_restore`, `lever_blind`, the span guard, the span of 4 fixes, the test of the norms. (Round 10 cut two more rules of the core, the fast-turn guard
of the alignment and the putting back of the heading variance at the release; no scheme of the earlier passes isolates either.) Their rows were written by the harness and the core
of commit `2011666` (`git show 2011666:experiments/eskf_lever.cpp`). They stay as they are and `report.md` counts them in one frozen table.
Three more labels name old `S8u` rows: `S8u@2011666` (in `hratex`), `S8u@0ef1add`, the `S8u` of the core of round 8, kept in the suites where
round 9 changed a row, and `S8u@b921395`, the `S8u` of the core of round 9 (with the fast-turn guard and the variance restore), kept in the suites where
round 10 changed a row. `S8u` is always the code that ships; its meaning moved with the core, and the labels keep the older meanings. The per-round narrative of the passes, with the tables as they were quoted, is in
[`results/eskf_lever/HISTORY.md`](results/eskf_lever/HISTORY.md).

**Suites** (configurations × seeds; each run has GNSS and IMU only unless it says NDT):

| Suite | Size | What it runs |
|---|---|---|
| `g60`, `g300` | 1,920 × 10, 384 × 6 (A, B and Z only) | the first pass: 60 s and 300 s, gyro bias 0 to 0.5 deg/s, standing, turning, spinning, driving, NDT on and off, dropout, reseed (schemes of `2011666` only) |
| `h300`, `h600`, `h900`, `h1800` | 2,160 × 2, 96 × 8, 48 × 8, 16 × 8 | the observability grid: 300 to 1,800 s, standing, line, start-stop, turn, spin, drive |
| `hndt`, `hslope`, `hyaw`, `hvib` | 192 × 2, 288 × 4, 120 × 4, 40 × 4 | NDT fixes on; a reseed on a sloped mount; a start heading 1 to 2 rad off; accelerometer noise |
| `hsg`, `hsgv` | 384 × 4 each | stand 300 to 1,800 s, then start, circle or turn on the spot for 160 s (`hsgv`: the same configurations on other data) |
| `hgap` | 832 × 4 | the fixes stop before the first start: long (30 s, 120 s) and short (1.5 to 12 s) outages |
| `hboot` | 192 × 4 | `initial_yaw` 0.5 to 3 rad off, no `/initialpose`, a stand of 6 to 300 s, then the start |
| `hrate`, `hratex` | 816 × 4, 544 × 4 | the boot and stand-then-go cells at fix rates of 0.5 to 100 Hz; the IMU data of a cell is the same at every rate |
| `hphase` | 5,440 × 4 | the cells of `hrate` at 1, 2, 5 and 10 Hz with the fixes at ten phases of the fix period after the whole seconds on which the motion starts |
| `hgapsweep` | 832 × 4 per value, 7 values | the sweep of `lever_outage_spread` on the cells of `hgap` (scheme `S8x_spread<value>`, frozen) |

A run has *diverged* if at any 1 s epoch from 10 s on the tilt error passes 0.05 rad, `|z|` passes 0.5 m, the horizontal error passes 1 m or the heading
error passes 0.5 rad. The report also counts the failures apart: `tz` (tilt over 0.05 rad or `|z|` over 0.5 m, the failure a deployer sees),
`circ` (the horizontal error beyond the circle of radius `|t_xy|` around the antenna that a standing robot cannot leave, plus 0.3 m), `head` and
`xy`. `std::normal_distribution` is implementation defined: the committed results are from the system g++ 9.4 / libstdc++, and another standard
library draws other numbers.

```sh
experiments/run_eskf_lever.sh /tmp/lever 4          # builds the harness from this tree, runs the default suites with S1s, S2s and S8u, writes report.md
experiments/run_eskf_lever.sh /tmp/lever 4 hphase   # the phase suite (not in the default list)
```

The committed results are `experiments/results/eskf_lever/` (`<suite>_runs.csv.gz`: one row per run and scheme; `<suite>_agg.csv.gz`: one row per
configuration and scheme with the NEES bands) and `report.md`, the output of `eskf_lever_report.py` on all of them. The report prints the tables of
`S1s`, `S2s` and `S8u` per suite, the comparison of round 10 with round 9 and of round 9 with round 8, the counts that `README.md` and this file quote about
the live schemes ("Numbers quoted"), a block of frozen cells (the counts about retired schemes that the two texts quote and that the committed rows hold), and the frozen table of the
retired schemes. The counts of the removed switches that are not in the frozen cells (the iterated update, the rate gates, the consider masks, the cells of E of the release
study, the slope reseed) are in [`HISTORY.md`](results/eskf_lever/HISTORY.md) and in the report at commit `0ef1add` (`git show 0ef1add:experiments/results/eskf_lever/report.md`).

### The shipped design, and the failure behind each rule

The core carries only the rules that a measured failure needs (the seventh review, round 8). `EskfParams` and `eskf.cpp` have no switch that exists to replay an older scheme.

*Rules that stay, and the measured failure behind each* (counts of runs with `tz` unless the cell says otherwise; the scheme names are those of the passes above):

| Rule | Suite | Without the rule | With it |
|---|---|---|---|
| consider states (heading, vertical gyro bias) | `hsg`, 1,152 runs | 408 (`S2s`, the plain full update) | 49 (`S8u@0ef1add`, all rules); 1 (`S8u@b921395` and `S8u`) |
| bound 0.35 rad | `h900`, `h1800`, standing, 192 runs | 192 (`S6vb`) | 0 (`S7a35`) |
| release 1.0 m/s | `h300` to `h1800`, 2,448 runs of the cells of E: `tz` or `circ` (HISTORY.md, second pass; not in the committed report) | 1,071 (`S7`, bound 0.7 rad, never released) | 942 (`S7a`, released); 454 with the bound at 0.35 rad (`S7a35`) |
| shadow and alignment | `hsg`, 1,152 runs | 375 (`S7a35`) | 81 (`S8a`) |
| blind update | `hsg`, 1,152 runs | 81 (`S8a`) | 26 (`S8`) |
| span guard of the alignment | `hgap`, outage of 120 s, 288 runs | 68 (`S8o`) | 11 (`S8`) |
| unknown start | `hboot`, 576 runs | 219 (`S8`) | 77 (`S8k`, `S8u@0ef1add`); 0 (`S8u@b921395` and `S8u`) |
| spans in seconds and the rate limit | `hrate`, boot cells at 1, 10, 20 Hz, antennas A and B, 144 runs each | 36, 31, 38 (`S8k`) | 0, 0, 0 (`S8u@0ef1add`, `S8u@b921395` and `S8u`) |
| outage spread | `hgap`, short outages, 1,280 runs of A and B; 640 of the mast | 83; mast `tz` 154, diverged 16 (`S8k`) | 32; mast `tz` 132, diverged 0 (`S8u@0ef1add`, the same core era); 31 and 35 (`S8u@b921395`); 1 and 34 (`S8u`, which has the other changes of rounds 9 and 10 as well) |
| fix history by time | `hratex`, 100 Hz, antennas A and B, 272 runs | 71 (`S8u@2011666`) | 0 (`S8u@0ef1add`); 1 (`S8u@b921395` and `S8u`: a tight circle of antenna A) |

The counts in the rows above that name a retired scheme (`S6vb`, `S7a35`, `S8`, `S8a`, `S8k`, `S8o`, `S8u@...`) are printed from the committed rows in the block "Frozen cells
quoted in README.md and experiments/README.md" of `report.md`. The two release counts of the cells of E are the exception: see HISTORY.md.

*Switches removed, and why* (each is a scheme or a field that no shipped configuration used; the counts in this table are in HISTORY.md and in the report at commit `0ef1add`, except `S7a35` and `S8` in the `lever_restore` row, which are in the frozen cells):

| Removed | Scheme that had it | Measured | Reason |
|---|---|---|---|
| `lever_motion_speed`, `lever_motion_rate` (gates on speed and body rate) | `S6s`, `S6t`, `S6v`, `S6va`, `S6vb` | `S6vb` 1,882 of 4,200 runs with `tz`, the release 880; the rate gates diverge in 28.8 % of the spin runs | the release replaced them |
| `lever_iterations` (iterated update) | `S4s`, `S5s` | iterating `S1s`: 953 against 954 of 15,104 runs diverged; iterating `S2s`: 2,418 against 1,651 | no gain |
| tilt and horizontal-bias consider masks | `S0`, `S0s` | 2,813 of 15,104 runs diverged, `S1s` 954 | the heading and the vertical gyro bias are the unobservable states |
| `lever_restore` = false | `S7a35` | 375 `tz` in `hsg`, `S8` 26 (`S7a35` also lacks the shadow, the alignment and the blind update, so the row does not isolate the restore) | the switch went in round 8; the restore itself went in round 10 (see below) |
| fast-turn guard of the alignment (refuse a window with a body rate over `0.5 lever_align_dv / ‖t_xy‖`) | none (it never had a switch) | the mutant without it passes all 54 gtests; the rows of `S8u` without it and without the variance restore are in "Round 10" below | removed in round 10: no test needs it, and the cells it was meant for are better without it |
| putting the honest heading variance back into `P` at the release (`if (open && held_back) setHeadingVariance(...)`) | none (the switch `lever_restore` went in round 8; this line stayed) | the mutant without it passes all 54 gtests; its only ablation, `S7a35`, lacks the shadow, the alignment and the blind update as well | removed in round 10: nothing measured needs it (see "Round 10" below) |
| `lever_blind`, `lever_align_span_guard` = false | `S8a`, `S8o` | see the first table | the rules stay; the switch goes |
| `lever_align_span` = 0 (the span of 4 fixes) | `S8k` | see the first table (`hrate`) | the spans in seconds replace it |
| `lever_align_norm` (test of the norms of the two changes of velocity, 0.3) | `S8n`, `S8v` | `hrate`: 0 to 33 of 544 rows differ per rate and the counts stay within 3 runs, but 1 boot run at 1 Hz ends with the heading over 0.5 rad under `S8v` (configuration 148, seed 3), 0 under `S8u@0ef1add`; replica `gnss_gap_across_start`: 2, 4, 13 failing seeds of 300 with the test at spread 0, 0.1, 1, against 0, 1, 4 without | measured harmful in the node scenario |

### Round 9: the position that the alignment hands to the next update

The ninth review (PR #7: one major and three minor findings) found a defect that the 1 Hz tail of the node boot scenario had hidden. The three minor
findings (the texts that contradict each other, the PR texts, the maintainability of the study) are answered in the texts and files they name. The
major finding: the defect, the fix and the effect:

**The defect** (the review's trace of 17 node runs of `gnss_boot_wrong_yaw_1hz` on `0ef1add`, 2 of them bad: 3.42 m and 2.76 m high, heading
+0.51 and +0.39 rad; the same failure when the recorded IMU and GNSS messages were replayed into the core). `alignHeading` rotates the attitude about
the antenna and resets the velocity, and the position it left in the state was the one that `predict` had dead-reckoned through the blind stretch
with the wrong heading. The update that follows the alignment then saw an innovation of 2.67 m and spent it on the heading (−0.44 rad) and the
vertical gyro bias (+0.00995 rad/s, one sigma of its prior), and later releases with a heading error of 0.9 rad tilted the filter. A second defect
made it worse at 1 Hz: the check on the window (`fixes_[n-1].t − fixes_[i0].t > window + slack`) had `window = 3 T` and `slack = T` for `T ≥ 1 s`,
so the limit was exactly `4 T` and a stamp jitter of under a millisecond decided whether the alignment ran at the fix 2 s after the start
(rejected in both bad runs: 4.0027 > 4.0023) or one fix later, when the release was already open (accumulator 1.073 to 1.080 against 1.0). In
the review's core harness at 1 Hz (stand 20 s, start-stop, heading 2 rad off, every tick phase) the release accumulator at the alignment stood at
0.68 to 0.99, so the core passed on a margin of 0.008 m/s.

**The fix** (`eskf.cpp`, `alignHeading`): after the rotation, the horizontal position is seated at the newest fix,
`x_xy ← fix_xy − (R t)_xy`; and the allowance on the window is `max(1 s, 1.5 T)` instead of `max(1 s, T)`, so that no stamp jitter decides the
check. The zero lever never reaches `alignHeading`; its bit identity is unchanged (sha256 `dbe91c30…`). The review found that seating the position
alone, or the allowance alone, removes both bad node replays; both are in. The covariance of the position is not touched beyond the existing
`|t_xy|² σ²`.

**Tests** (`prism_loc_fusion/test/test_eskf.cpp`): `OneHertzFixesWithStampJitterAlignAtTheStartAndKeepTheFilterLevel` (40 seeds, 1 Hz, stamps late by
0 to 20 ms, a start at 1 m/s², heading 2 rad off: every run aligns at the fix 2 s after the start and ends level, at the right height, with the
heading found), `AlignmentSeatsThePositionAtTheNewestFix` (20 seeds, the heading and the vertical gyro bias right after the aligned update) and
`CovarianceHonestPutsTheHeldBackHeadingSpreadIntoThePublishedCovariance`. Mutants: without the seat, `AlignmentSeatsThePositionAtTheNewestFix` fails;
with the allowance back at `max(1, T)` or at `T + 0.5 ms`, the 1 Hz test fails; without both, both fail. Neither change alone fails the end state of
the 1 Hz test, which is why it also asserts the time of the first alignment.

**Effect on the suites.** The `S8u` rows of every suite were run again with the new core (`LEVER_ONLY=S8u`, 15 suites); `S1s` and `S2s` rows are
bit for bit those of the committed CSVs ("Reproducing"). The old `S8u` rows stay as `S8u@0ef1add` in the suites where any row changed. Runs with
`tz`, round 8 → round 9 (the section "Round 9 against round 8" of `report.md` has the other columns):

| Suite and cell | Runs | Round 8 | Round 9 | Zero lever |
|---|---|---|---|---|
| `hboot`, mast | 192 | 76 | 0 | 112 |
| `hboot`, A and B | 384 | 1 | 0 | |
| `hsg`, mast | 384 | 48 | 0 | 108 |
| `hsgv`, mast | 384 | 52 | 0 | 122 |
| `hgap`, short outages, mast | 640 | 132 | 35 | 226 |
| `hgap`, short outages, hardest cell of the mast (stand 900 s, 0.2 deg/s) | 160 | 123 | 26 | 61 |
| `hgap`, short outages, A and B | 1,280 | 32 | 31 | |
| `hgap`, long outages, mast | 192 | 17 | 10 | 29 |
| `hrate` and `hratex`, 1 to 100 Hz, mast | 136 per rate | 30 to 43 | 0 to 4 | |
| `hphase`, 1 Hz, A and B (ten phases) | 2,720 | 26 | 10 | |
| `hphase`, 2 Hz, A and B | 2,720 | 10 | 3 | |
| `hphase`, 1 Hz, mast | 1,360 | 436 | 35 | |
| `h300`, A and B | 2,160 | 599 | 597 | |
| `hslope`, A and B | 576 | 11 | 13 | |

Antennas A and B are unchanged within two runs everywhere else. One more run fails than before in `hratex` at 100 Hz (1 of 272 against 0; a
stand-then-go circle of antenna A, the tight circle that the alignment refuses, heading 0.59 rad off at the end) and one more in `hrate` at 2 Hz
(2 against 1). The remaining `tz` at 1 Hz (10 of 2,720 over ten phases, largest height error 1.42 m, 2 runs over 1 m, phase 0: 0) is spread over the start-stop and circle
motions of antennas A and B. The stream phase was not the cause of the tail: the review's tick-level sweep in the core (every tick phase 0 to 99, 800 runs
per antenna) gave `tz` in 1 of 800.

**Node figures.** `scripts/fusion_extrinsic_check.py` on the final head, one ROS domain per run, every run in [`results/eskf_lever/node_checks.txt`](results/eskf_lever/node_checks.txt), block "Round 9 head". The default run has 36 scenarios and 249 checks and passes (24 minutes). The repeated scenarios:

| Scenario | Runs | Largest height error | Tilt error | Heading error after the cycle | Bound (height / tilt / heading) |
|---|---|---|---|---|---|
| `gnss_boot_wrong_yaw` (A, 5 Hz) | 6 | 0.101 to 0.120 m | 0.0048 to 0.0091 rad | +0.157 to +0.219 rad | 1.0 m / 0.1 rad / 0.5 rad |
| `gnss_boot_wrong_yaw_b` (B, 5 Hz) | 6 | 0.080 to 0.090 m | 0.0058 to 0.0072 rad | +0.037 to +0.056 rad | same |
| `gnss_boot_wrong_yaw_10hz` | 6 | 0.048 to 0.062 m | 0.0121 to 0.0133 rad | −0.090 to −0.107 rad | same |
| `gnss_boot_wrong_yaw_50hz` | 30 | 29 runs 0.049 to 0.068 m; 1 run 0.219 m | 0.0058 to 0.0110 rad; 0.0349 rad | up to 0.014 rad; −0.994 rad | same |
| `gnss_boot_wrong_yaw_1hz` | 30 | 0.056 to 0.087 m | 0.0042 to 0.0071 rad | −0.067 to +0.006 rad | same (was 5.0 m / 0.3 rad / 1.0 rad) |
| `gnss_stand_go` (A) | 6 | 0.012 to 0.033 m | 0.0030 to 0.0082 rad | | 0.25 m / 0.03 rad |
| `gnss_stand_go_b` (B) | 6 | 0.035 to 0.061 m | 0.0093 to 0.0140 rad | | same |

Before round 9, 3 of 30 runs of `gnss_boot_wrong_yaw_1hz` ended 2.7 to 3.3 m high (10 %); the review's 17 recorded runs had 2 bad ones. Now 0 of 30 (a 10 % rate would give none in 30 runs with a chance of 4 %). The long scenarios pass: `gnss_stand_long` 0.133 m, `gnss_gap_across_start` 0.259 m (0.0323 rad) and `_b` 0.072 m (0.0101 rad). At 50 Hz one run of 54 on this build (6 + 24 + 24, at three to six node runs at a time) ended with the heading 0.99 rad off after the cycle (0.219 m high, tilt 0.035 rad; the rig counted 1 status with IMU gaps); the build of `0ef1add` has 0 of 24 under the same load, so the run is not shown to come from round 9, and it is not explained or reproduced. At sixteen parallel runs the rig fails every run on both builds (the rig streams in real time and the machine cannot keep up), so those runs say nothing about the filter.  The zero-lever poses are bit for bit those of `main` (lockstep feed of 1,284 poses, sha256 `dbe91c30…`), and the five experiment CSVs of the paper rebuilt from this tree have the hashes of `main`.

**What the old text got wrong.** Three claims are rewritten in `README.md`. The 1.5 m mast "gains little": in the suites that start from a stand or
an outage it now ends well under the zero-lever filter (table above), and over all 108 slices of the live tables it has more runs with `tz` than the
zero lever in 4 (`h300` drive and turn, and two cells at 0.5 Hz, which is outside the supported rate; `report.md`, last line of "Numbers
quoted"; it was 5 slices at round 9). The 1 Hz tail was blamed on the phase between the IMU and GNSS streams of the node rig; the core sweep above and the node repeats show
that the check on the window and the stale position caused it. `BOOT_Z_TOL_1HZ` in `scripts/fusion_extrinsic_check.py` returns to the 1.0 m of the
other rates, and "1 to 100 Hz" is supported on the node evidence below.

### Round 10: two rules cut, the numbers made traceable

The tenth review (PR #7: one major and four minor findings) measured two rules of the core that no test needs and that no table named. Both are cut. The other
findings are answered in the texts and files they name (the frozen cells of `report.md`, the limit on the horizontal arm in `README.md`, the SIGTERM handler of the node check).

**The cuts** (`eskf.cpp`, `eskf.hpp`, `types.hpp`):
1. The fast-turn guard of the alignment: the two checks `fixes_[i].rate_max * t_xy > 0.5 * lever_align_dv` in `alignHeading`, with `Fix::rate_max`, `rate_max_` and `rate_`.
2. The variance restore at the release: `if (open && held_back) setHeadingVariance(up, s_hh_)`. While the release is open the heading is estimated with the variance `P` holds. The shadow
   keeps the honest spread as before.

*Why.* The mutant without each rule passes all 54 gtests of `prism_loc_fusion` (the tests are unchanged: 54 cases, all pass). The review reran the `S8u` rows without them and found the guard
worse in `h300`, `hgap`, `hyaw` and `hslope`, and the restore worth at most 2 runs in `h300`. This tree reran the rows with both cut (below). The limit "a tight circle that starts during an outage" of
round 9 was caused by the guard: antenna A has the heading off by over 0.5 rad in 0 of 128 runs now (64 before), and none of the runs passes 1.5 m (32 before).

*What `S8u` means now.* `S8u` is the code that ships, so its rows moved with the core. The rows of the core of round 9 (with the guard and the restore) stay as `S8u@b921395`
in the suites where any row changed; the rows of the core of round 8 stay as `S8u@0ef1add`. The scheme that this round measures has no name of its own in the CSVs: it is `S8u`.
No scheme of the earlier passes isolates the guard or the restore (`S7a35` lacks the shadow, the alignment and the blind update as well), so the table below is the only measurement of either.

**Effect on the suites** (`LEVER_ONLY=S8u`, 15 suites, same configurations and seeds; runs with `tz`; the section "Round 10 against round 9" of `report.md` has the other columns):

| Suite and cell | Runs | Round 9 | Round 10 |
|---|---|---|---|
| `h300`, A and B | 2,160 | 597 | 565 |
| `hslope`, A and B | 576 | 13 | 3 |
| `hyaw`, A and B | 240 | 27 | 1 |
| `hgap`, A and B (all outages) | 1,664 | 37 | 1 |
| `hgap`, mast | 832 | 45 | 42 |
| `hrate`, A and B | 1,632 | 107 | 100 |
| `hrate`, mast | 816 | 73 | 66 |
| `hratex`, mast | 544 | 8 | 4 |
| `hphase`, 1 s, mast | 1,360 | 35 | 18 |
| `hphase`, 0.5 s, mast | 1,360 | 13 | 3 |
| `hphase`, 0.2 s, mast | 1,360 | 2 | 6 |
| `hphase`, A and B, four periods | 10,880 | 13 | 16 |

The heading error over 0.5 rad and the horizontal error over 1.5 m fall where `tz` falls: `hgap`, A and B, head 137 to 10 and `xy>1.5` 51 to 2; `hyaw`, A and B, `xy>1.5` 4 to 0.
Four cells get worse, all small: `h300`, A and B, head 643 to 656 and `xy>1.5` 15 to 18; `hphase`, A and B, 0.2 s, head 0 to 4; 0.1 s, head 0 to 3; `hphase`, mast, 0.2 s, `tz` 2 to 6.
`hsg`, `hsgv`, `hboot` and the mast in `h300`, `h600` to `h1800`, `hndt` and `hvib` are unchanged within one run. Antennas A and B in `h300` by motion: drive 121 to 75 of 360,
start and stop 146 to 148, turn 63 to 74, spin 73 to 74.

**Numbers made traceable.** `report.md` has a block "Frozen cells quoted in README.md and experiments/README.md", read from the committed rows. It holds the counts about retired schemes
that the two texts quote (rules 2 to 8, the rate limit, the fix history, the observability grid). The counts of the removed switches (the iterated update, the rate gates, the consider masks,
the cells of E, the slope reseed) are not in it; they are in HISTORY.md and in the report at commit `0ef1add`. Rule 8 is quoted as `S8k` against `S8u@0ef1add` (cores of the same era): the
earlier text set `S8k` against the `S8u` of round 9, which mixed the effect of the position seat into the effect of the spread.

**SIGTERM.** `scripts/fusion_extrinsic_check.py` had no handler: a SIGTERM (an outer `timeout`) ended the script before its `finally` stopped the node, and the node stayed in its own
session (the review found 8 such nodes about 2 h old). The handler is installed after `rclpy.init()`, which installs its own. A run under `timeout -s TERM 25` ends with `rc=124` and no
`fusion_node_main` left; a first version of the handler, installed before `rclpy.init()`, left the node running.

**Node figures.** `scripts/fusion_extrinsic_check.py` on the head of round 10 (code commit `252b34e`), one ROS domain per run, every run in [`results/eskf_lever/node_checks.txt`](results/eskf_lever/node_checks.txt), block "Round 10 head". The default run has 36 scenarios and 249 checks and passes (24 minutes). The repeated scenarios:

| Scenario | Runs | Largest height error | Tilt error | Heading error after the cycle | Bound (height / tilt / heading) |
|---|---|---|---|---|---|
| `gnss_boot_wrong_yaw` (A, 5 Hz) | 6 | 0.101 to 0.123 m | 0.0056 to 0.0080 rad | +0.169 to +0.210 rad | 1.0 m / 0.1 rad / 0.5 rad |
| `gnss_boot_wrong_yaw_b` (B, 5 Hz) | 6 | 0.078 to 0.096 m | 0.0028 to 0.0042 rad | +0.041 to +0.046 rad | same |
| `gnss_boot_wrong_yaw_10hz` | 6 | 0.045 to 0.053 m | 0.0105 to 0.0150 rad | −0.101 to −0.087 rad | same |
| `gnss_boot_wrong_yaw_50hz` | 30 | 0.052 to 0.068 m | 0.0064 to 0.0123 rad | −0.003 to +0.016 rad | same |
| `gnss_boot_wrong_yaw_1hz` | 30 | 0.053 to 0.079 m | 0.0042 to 0.0080 rad | −0.058 to −0.029 rad | same (was 5.0 m / 0.3 rad / 1.0 rad) |
| `gnss_stand_go` (A) | 6 | 0.016 to 0.028 m | 0.0031 to 0.0037 rad | | 0.25 m / 0.03 rad |
| `gnss_stand_go_b` (B) | 6 | 0.023 to 0.052 m | 0.0100 to 0.0115 rad | | same |

Of the 30 runs at 50 Hz, 6 ran with six other node runs on the machine and 24 three at a time; none ended with the heading off. The round-9 build had 1 of 54 with the heading 0.99 rad off; the cause of that run is still not known, and 0 of 30 does not show that it is gone (the chance of 0 of 30 at a rate of 1 in 54 is 57 %). The 1 Hz scenario ends 0.053 to 0.079 m high in 30 of 30 runs. The long scenarios pass: `gnss_stand_long` 0.138 m, `gnss_gap_across_start` 0.219 m (0.0302 rad) and `_b` 0.058 m (0.0105 rad). The zero-lever poses are bit for bit those of `main` (lockstep feed of 1,284 poses, sha256 `dbe91c30…`), and the five experiment CSVs of the paper rebuilt from this tree have the hashes of `main`.

### Reproducing

`LEVER_ONLY=S1s,S2s,S8u` on a suite, then `experiments/compare_rows.py`. The rows of `S8u` that the current core writes equal the committed rows bit for bit in all
15 suites that carry them (`runs` and `agg` files): in round 10 the `S8u` rows of every suite were run again, replace the committed ones, and are the committed ones now
(`S8u@b921395` keeps the rows of round 9 where round 10 changed any row; `S8u@0ef1add` keeps those of round 8). The rows of `S1s` and `S2s` were compared in round 9 on all
suites that carry them. In round 10 they were run again on the suites below and are equal to the committed rows; on the other suites they were not run again. The round-10 cuts
cannot reach them: `S1s` and `S2s` have no bound, no shadow and no unknown start, so the alignment and the release that the cuts touch do not run. `S1s` is in the first eight suites only.

Rows are the `runs` and `agg` rows of the schemes named.

| Suite | Schemes compared | Rows | Equal |
|---|---|---|---|
| `h600` | S1s,S2s,S8u | 2,592 | 2,592 |
| `h900` | S1s,S2s,S8u | 1,296 | 1,296 |
| `h1800` | S1s,S2s,S8u | 432 | 432 |
| `hvib` | S1s,S2s,S8u | 600 | 600 |
| `hyaw` | S1s,S2s,S8u | 1,800 | 1,800 |
| `hsg` | S2s,S8u | 3,840 | 3,840 |
| `hrate` | S2s,S8u | 8,160 | 8,160 |
| `h300` | S8u | 6,480 | 6,480 |
| `hndt` | S8u | 576 | 576 |
| `hslope` | S8u | 1,440 | 1,440 |
| `hsgv` | S8u | 1,920 | 1,920 |
| `hgap` | S8u | 4,160 | 4,160 |
| `hboot` | S8u | 960 | 960 |
| `hratex` | S8u | 2,720 | 2,720 |
| `hphase` | S8u | 27,200 | 27,200 |
| all | | 64,176 | 64,176 |

The first-pass suites `g60` and `g300` use schemes of the removed switches and are not rebuilt; their rows are those of `2011666`. The rows of `hratex` that the committed CSVs hold under the scheme name `S8u@2011666` are from the core of that commit (its 100 Hz cells differ from `S8u` because that core kept 256 fixes, 2.56 s at 100 Hz; the alignment needs 4 s). `gap_across_start.txt` is reproduced as well.

#### Commands

```sh
experiments/run_eskf_lever.sh /tmp/lever 4          # builds the harness from this tree, runs the default suites with S1s, S2s and S8u, writes report.md
experiments/run_eskf_lever.sh /tmp/lever 4 hphase   # the phase suite (not in the default list)
LEVER_ONLY=S2s,S8u experiments/run_eskf_lever.sh /tmp/lever 4 hrate      # one scheme set on one suite, then:
python3 experiments/compare_rows.py experiments/results/eskf_lever/hrate /tmp/lever/hrate S2s,S8u
g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3 prism_loc_fusion/src/*.cpp experiments/eskf_gap_across_start.cpp -o /tmp/gap && NSEED=300 /tmp/gap   # gnss_gap_across_start on the core (the sweep of lever_outage_spread)
python3 experiments/eskf_lever_report.py experiments/results/eskf_lever/{g60,g300,h300,h600,h900,h1800,hndt,hslope,hyaw,hvib,hsg,hsgv,hgap,hboot,hrate,hratex,hphase,hgapsweep} > report.md
```

The committed `hgapsweep` rows, the `S8x` rows and the first-pass suites need the harness of commit `2011666` (`git show 2011666:experiments/eskf_lever.cpp`, built against the core of that commit). `results/eskf_lever/gap_across_start.txt` is the output of the replica command above on this core (300 seeds); `gap_across_start_2011666.txt` is that of the core with the test of the norms, and stays.

Not done: a real robot; slopes under motion (the tilt is constant); a bound below 0.35 rad (the smallest tested); release thresholds between the three tested; a heading start error with a bound at 0.35 rad other than the three tested; an observability-constrained update or first-estimates Jacobians (Huang, Mourikis and Roumeliotis 2010), the usual remedy for an update that gains information along an unobservable direction, which was not tried.
