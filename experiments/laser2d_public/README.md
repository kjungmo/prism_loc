# laser2d and BBS relocalization on the MIT Stata Center dataset

The laser2d backend (likelihood-field MCL, KLD resampling) and its global relocalization (branch-and-bound
scan matching, BBS, with the multi-scan verifier) on a public 2D LiDAR dataset with ground truth. The driver
`mit_stata_laser2d.cpp` runs the loop of `LocalizationNode` without ROS, with `laser2d.yaml` defaults.
Scores are in `../results/laser2d_public/`.

## Dataset and licence

- MIT Stata Center Data Set (Fallon, Johannsson, Kaess, Leonard, IJRR 2013), https://projects.csail.mit.edu/stata/
- Licence, quoted from the dataset's download page (https://projects.csail.mit.edu/stata/downloads.html): "This work is
  licensed under a Creative Commons Attribution 3.0 Unported License" (http://creativecommons.org/licenses/by/3.0/deed.en_US).
- The project page states the dataset is inactive and "less consistent than we would have liked" (robot hardware and
  calibration changes, a broken inter-computer time synchronisation).
- Used files: the dataset's extracted laser scans (`*.gt.laser.scans`: Hokuyo UTM-30LX, 1040 beams over 260 deg, 20 Hz,
  30 m) and ground-truth laser poses (`*.gt.laser.poses`, "to within 2-3 cm" from floor-plan-constrained scan matching),
  floor 2, in the Drive folder `ground_truth_poses` linked from the downloads page. About 0.6 GB in total, kept outside
  the repository.
- Not used: the ROS bags (36-60 GB each). They hold the wheel odometry; see "Odometry" below.

## Sessions, map, and what is optimistic

| role | session (part1, floor 2) | duration | path |
|---|---|---|---|
| map | 2012-04-06-11-28-12 | 1406 s | 621 m |
| long query | 2012-04-02-10-54-41 (4 days before the map) | 1403 s | 650 m |
| short query | 2012-04-04-07-15-50 | 119 s | 56 m |
| older map | 2012-01-18-09-09-07 (3 months earlier) | 2058 s | 768 m |

- The map session and the query sessions are different days, so map and evaluation are different sessions.
- Map (`stata_prepare.py`): scan endpoints placed at the ground-truth poses of the map session, counted per 0.05 m
  cell without ray casting. A cell with at least 5 endpoints (every 3rd scan, range 0.1-25 m) is occupied, every other cell
  is free. Free and unknown behave the same for the likelihood field and BBS.
- The dataset's own floor-plan occupancy maps (`FP_*.pgm`) have no origin or resolution file. I could not register them to
  the ground-truth frame (no match peak for any scale, rotation or flip tried), so they are not used.
- The ground truth of all sessions is in one frame, so a map from one session can be scored against another's truth.
- The map is built from ground-truth poses. A SLAM map would carry its own drift. This is optimistic for the map.

## Odometry (read this first)

The scan files carry no odometry. The odometry is synthesized from ground-truth increments between consecutive scans:
translation scale 1.03, rotation gain 1.03, 1% Gaussian noise on each increment, and a constant yaw bias of 0.02 deg/s
(about 0.5 deg per 25 s). The values are fixed, not tuned, and are not the robot's real wheel odometry. Dead
reckoning with it ends 16-24 m off after 1400 s. Another odometry model would change the numbers.

## Method (mirrors `localization_node.cpp`)

- Update gate: a scan updates the filter once the odometry has moved 0.2 m or turned 0.2 rad (or the seed forces it).
  `predict` -> `correct` -> `resample` (KLD, 500-2000 particles, alpha1-4 = 0.2, 60 beams, sigma_hit 0.2).
- Seed: the ground-truth pose at the first scan with the `/initialpose` floor spread (0.5 m, 0.5 m, 0.2617 rad, 2000
  particles).
- Error: the planar distance of `map_odom (+) odom_base` to the ground truth at every scan.
- Relocalization: `RelocalizationVerifier` with the node's defaults (top-8 modes, 9 odometry-chained scans, posterior 0.9).
  Query k of 50 starts at scan `100 + (N-2*100)*k/49` of the long query session, no prior. The window cap
  `bbs_max_linear_window` is 60 m instead of 50 m, so the search covers the whole 101 m x 68 m map (the default would
  leave 0.9 m at the map edges, and the node warns that it is then not global). Correct: within 0.5 m and 10 deg of the
  ground truth at the scan the verifier decided on (the synthetic criterion of the paper).
- Offset levels L0-L6 and U: `../nclt/INIT_PROTOCOL.md`, offset drawn from the seed, spread `max(0.5, d)` m and `max(15, a)`
  deg, seeds 42, 1, 2, four start times (200, 500, 800, 1100 s) on the long query session, 180 s per run, converged =
  planar error < 1 m for >= 10 s starting within 120 s. U: the verifier from the start scan (retrying from the next scan
  when not accepted), then tracking from the accepted pose.

## Reproduce

Download the `*.gt.laser.scans` and `*.gt.laser.poses` of the four sessions above into one directory, then from the
repository root:

    experiments/laser2d_public/run_stata.sh DATA_DIR WORK_DIR experiments/results/laser2d_public
    python3 experiments/laser2d_public/stata_score.py track experiments/results/laser2d_public/track_*.csv

It takes about 25 minutes on one core. Needs python3 + numpy, g++, Eigen.

## Results

### Tracking, seeded at the ground truth (3 seeds, 42 / 1 / 2)

Shares are over the logged rows (every 10th scan plus every update). "Excursion": error over 1 m for more than 2 s.

| map -> query | seed | RMSE (m) | median (m) | max (m) | > 0.3 m | > 1 m | excursions (s) |
|---|---|---|---|---|---|---|---|
| Apr 6 -> Apr 4 (119 s) | 42 | 0.078 | 0.047 | 0.22 | 0.000 | 0.000 | 0 |
| | 1 | 0.069 | 0.051 | 0.23 | 0.000 | 0.000 | 0 |
| | 2 | 0.077 | 0.049 | 0.21 | 0.000 | 0.000 | 0 |
| Apr 6 -> Apr 2 (1403 s) | 42 | 12.24 | 0.103 | 33.95 | 0.378 | 0.363 | 3 (518) |
| | 1 | 0.94 | 0.065 | 5.55 | 0.109 | 0.074 | 5 (118) |
| | 2 | 13.77 | 0.120 | 51.41 | 0.382 | 0.363 | 4 (515) |
| Jan 18 -> Apr 4 (119 s) | 42 | 29.52 | 20.79 | 53.51 | 0.883 | 0.879 | 1 (91) |
| | 1 | 0.57 | 0.175 | 1.58 | 0.352 | 0.105 | 2 (9) |
| | 2 | 22.66 | 13.93 | 41.95 | 1.000 | 0.875 | 2 (91) |
| Jan 18 -> Apr 2 (1403 s) | 42 | 31.13 | 25.91 | 57.50 | 0.967 | 0.963 | 1 (1306) |
| | 1 | 14.07 | 2.88 | 44.79 | 0.590 | 0.555 | 5 (722) |
| | 2 | 28.48 | 19.99 | 71.26 | 0.965 | 0.960 | 1 (1301) |
| same session as map (optimistic): Jan 18 | 42 | 0.062 | 0.024 | 0.70 | 0.005 | 0.000 | 0 |
| same session as map (optimistic): Apr 6 | 42 | 0.070 | 0.021 | 0.66 | 0.012 | 0.000 | 0 |

- With a map that is a different session, tracking is centimetre-level while it holds (median 0.05-0.12 m) and loses the
  pose in some runs. In two of three seeds on the 1403 s query the filter is lost for about 500 s in total. The runs
  that lose it end up tens of metres off and do not recover (the filter has no relocalization trigger while tracking).
- The three-month-old map (Jan 18) fails on both April queries. Checked with the Apr 6 scans placed at their ground truth:
  the share of endpoints within 0.15 m of an occupied cell of the Jan 18 map is 0.77 on average and 0.13-0.38 in one
  stretch of the run (scans 1800-2200), which suggests the environment changed between January and April. I did not
  confirm the cause. Same-session runs show the filter and the driver themselves are fine.
- With exact odometry (no scale error, noise or bias) the Jan 18 map -> Apr 6 session run (not in the tables) still failed
  (RMSE 14 m), so the synthetic odometry alone is not the cause of the January-map failures. I did not run exact odometry
  for the April maps.

### Global relocalization (50 queries, long query session, no prior)

| map | correct | wrong accepted | rejected | time, first scan (BBS) | accepted-correct error (median) |
|---|---|---|---|---|---|
| Apr 6 (4 days) | 43 | 7 | 0 | median 1.59 s, p90 1.90 s, max 2.10 s | 0.051 m / 0.47 deg |
| Jan 18 (3 months) | 30 | 8 | 12 (ambiguous) | median 1.33 s, p90 1.59 s, max 2.33 s | 0.060 m / 0.54 deg |

- The single-scan rule (first scan, accept if score >= 0.4) on the same queries: Apr 6 map 43 / 7 / 0, Jan 18 map 31 / 19 / 0
  (correct / wrong accepted / rejected). The verifier cut the wrong acceptances from 19 to 8 with the older map, and
  changed nothing with the Apr 6 map.
- The verifier changes nothing with the Apr 6 map because it sees 9 scans in 0.45 s of driving, so the extra evidence is
  almost the same scene. The wrong acceptances of the Apr 6 map run have posterior 0.92-1.00. Six are 24-41 m off, one is a near miss at 0.54 m (the criterion is 0.5 m).
- Time is one core, 2038x1364 / 1997x1412 cells at 0.05 m, 361 yaw steps, wall-clock, machine dependent (WSL2).

### Initial-pose offset levels (Apr 6 map, long query session; 4 starts x 3 seeds = 12 runs per level)

| level | offset | converged | median convergence time (s) | median RMSE after convergence (m) |
|---|---|---|---|---|
| L0 | 0 m, 0 deg | 12/12 | 0.0 | 0.48 |
| L1 | 2 m, 15 deg | 11/12 | 0.0 | 0.84 |
| L2 | 5 m, 15 deg | 11/12 | 0.0 | 0.16 |
| L3 | 10 m, 15 deg | 6/12 | 0.3 | 0.89 |
| L4 | 2 m, 45 deg | 11/12 | 0.0 | 0.18 |
| L5 | 2 m, 90 deg | 9/12 | 0.0 | 0.15 |
| L6 | 10 m, 90 deg | 2/12 | 4.7 | 0.88 |
| U (BBS, then tracking) | unknown | 9/12 | 0.4 | 0.89 |

- Convergence time 0.0 means the error was already under 1 m at the start and stayed so for 10 s. The RMSE after
  convergence includes later losses within the 180 s, which can be why L0 shows 0.48 m. I did not check where in the run the error is large.
- U fails 3 of 3 at the start time 500 s (the verifier accepted a pose, the tracker never got under 1 m) and succeeds at the
  other three start times (3/3 each). Compute time to the accepted pose: 1.1-1.9 s at starts 200 and 800, 7-8 s at 1100, 11.5-11.8 s at 500
  (the verifier retried after non-accepted attempts).
- Per level, per start time and seed rows: `init_map0406_q0402.csv`.

## AMCL baseline

AMCL baseline not run: the machine has ROS 1 noetic only, without the `amcl` and `map_server` packages (not in
`/opt/ros/noetic`), and ROS 2 `nav2_amcl` is not installed. Nothing was installed system-wide.

## Not measured, and limits

- Real wheel odometry (it is in the multi-gigabyte bags). All numbers use the synthetic odometry above.
- The floor-plan map of the dataset (frame unknown, see above). Maps are built from ground-truth poses of another session.
- Only floor 2, only the building's one wing and corridors that the map session covers.
- Single odometry-noise realization per seed; 3 seeds, 4 start times, 50 queries: counts are small, differences of one or
  two runs are not significant.
- The wrong BBS acceptances were not inspected for map symmetry (the ones listed in `reloc_*.csv` are far from the truth).
