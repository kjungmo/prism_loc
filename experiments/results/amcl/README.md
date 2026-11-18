# AMCL baseline for the initialization-robustness protocol

Protocol: `experiments/nclt/INIT_PROTOCOL.md` plus the addendum `experiments/nclt/INIT_PROTOCOL_AMCL.md`
(committed before the first AMCL run). AMCL = ROS 2 Humble `nav2_amcl` 1.1.20, unmodified, driven offline in lock
step with simulated time by `experiments/nclt/amcl_run.py`. Same 2D map, scans, odometry, ground truth and offsets as
laser2d; the spread given is the same. Scorer unchanged (`nclt_init_score.py`; summaries by `amcl_score.py`).
Each run: 899-901 scans (NCLT) or 3578-3587 scans (Stata) sent and as many map->odom TFs received, 0 timeouts
(`*.log` per run). AMCL cannot be seeded, so the 3 seeds differ only in the offset (and Stata odometry noise).

## NCLT, 2D projection, 4 segments x 3 seeds = 12 runs per level

Cells: converged / 12, median convergence time (s), median RMSE after convergence (m).
Sources: ndt3d and tracker `experiments/results/nclt/score_init.txt` (branch exp/nclt-ndt3d-bench 5dc53b7, "ndt3d"
= its best setting); laser2d `experiments/results/nclt/score_init_laser2d.txt` (375ada4); laser2d alt and AMCL:
this branch (`nclt_laser2d_alt/`, `nclt_pri/`).

| level | ndt3d | 6-DoF tracker | laser2d | laser2d alt band | AMCL |
|---|---|---|---|---|---|
| L0 | 12, 0.0, 0.303 | 12, 0.0, 0.148 | 12, 0.0, 0.399 | 12, 0.0, 0.555 | 6, 9.4, 23.808 |
| L1 | 12, 0.0, 0.294 | 12, 1.9, 0.151 | 12, 0.1, 0.389 | 12, 0.0, 0.557 | 4, 15.8, 1.968 |
| L2 | 12, 0.0, 0.310 | 12, 3.5, 0.151 | 12, 1.2, 0.413 | 12, 0.2, 0.453 | 3, 98.8, 1.850 |
| L3 | 12, 0.8, 0.344 | 11, 9.8, 0.166 | 9, 4.0, 0.359 | 8, 2.6, 0.516 | 3, 98.0, 2.168 |
| L4 | 12, 0.0, 0.309 | 4, 34.3, 0.169 | 12, 0.4, 0.382 | 12, 0.1, 0.426 | 5, 18.8, 2.266 |
| L5 | 12, 0.0, 0.307 | 0, -, - | 12, 1.2, 0.372 | 12, 0.1, 0.487 | 5, 17.0, 2.287 |
| L6 | 11, 8.8, 0.331 | 0, -, - | 6, 3.9, 5.456 | 6, 5.5, 31.998 | 3, 97.8, 1.906 |
| U | not supported | not supported | 12, 3.5, 0.387 | 9, 1.6, 0.598 | 0, -, - (U-20k particles: 0, -, -) |

Reading notes:
- RMSE after convergence includes later losses inside the 180 s. Runs with RMSE > 5 m after convergence: AMCL L0 3,
  L1 1, L4 2, L5 2; laser2d alt L0 2, L1 3, L2 3, L3 1, L4 3, L5 2, L6 3, U 4. "Converged" does not mean "stayed".
- ndt3d converged 11/12 at L6 (not 12/12); 12/12 at L0-L5.
- Segment 3400 s carries most of the laser2d losses (original band, as in its score file); the alt band changed
  U from 12/12 to 9/12 and L3 from 9 to 8, and left L6 at 6/12.
- AMCL fails on the NCLT projection even at L0 (4 distinct runs; seeds are identical there): segment 400 s tracks
  (RMSE 1.7 m), 2400 s converges at 0.2 s and is lost later (RMSE 46 m), 1400 s and 3400 s never converge (error
  up to 150-170 m). Checks that this is not a driver fault:
  1. On MIT Stata, with the same driver, AMCL tracks (table below).
  2. At the ground-truth pose the NCLT scans fit the map (endpoint hit fraction 0.95 on segment 400 s; 0.65
     with angles mirrored or 180 degrees turned; frame, origin, resolution and scan direction are right).
  3. With the odometry replaced by the ground truth (segment 1400 s, L0) AMCL still fails (RMSE 99.6 m).
  4. With 5000-20000 particles (`diag/amcl_L0_t1400_20kparticles.csv`) it fails the same way (RMSE 99.3 m).
  The scene (open campus, sparse height-band returns) has little 2D structure at the start of segment 1400 s (a
  2 m shift changes the hit fraction by 0.01), and AMCL's tempered likelihood (sum of cubed beam probabilities)
  is weaker than laser2d's product of beam probabilities. I did not test other AMCL sensor settings (z_hit,
  sigma_hit, beams); different settings could change the NCLT numbers.

## MIT Stata (Apr 6 map, long query Apr 2, 4 starts x 3 seeds), converged / 12, median conv. time (s), median RMSE (m)

laser2d row: `experiments/results/laser2d_public/init_map0406_q0402.csv` (7fa3cd8, cherry-picked here).
AMCL: `stata/amcl_*.csv`, scored with `nclt_init_score.py`.

| level | laser2d | AMCL |
|---|---|---|
| L0 | 12, 0.0, 0.480 | 12, 0.0, 0.993 |
| L1 | 11, 0.0, 0.837 | 12, 3.2, 1.078 |
| L2 | 11, 0.0, 0.164 | 12, 15.5, 1.290 |
| L3 | 6, 0.3, 0.893 | 9, 18.7, 1.660 |
| L4 | 11, 0.0, 0.175 | 12, 5.8, 1.054 |
| L5 | 9, 0.0, 0.151 | 12, 13.3, 1.119 |
| L6 | 2, 4.7, 0.875 | 2, 24.4, 1.495 |
| U | 9, 0.4, 0.892 (BBS) | 3, 49.3, 1.464 (U-20k particles: 4, 7.1, 1.396) |

Tracking seeded at the ground truth (spread 0.5 m, 0.5 m, 0.26 rad), synthetic odometry, per seed:

| query | method | RMSE (m) | median (m) | max (m) | share > 1 m | excursions (s) |
|---|---|---|---|---|---|---|
| Apr 4 (119 s) | laser2d s42/1/2 | 0.078, 0.069, 0.077 | 0.047-0.051 | 0.21-0.23 | 0 | 0 |
| Apr 4 (119 s) | AMCL s42/1/2 | 0.161, 0.175, 0.154 | 0.064, 0.062, 0.066 | 0.47, 0.58, 0.55 | 0 | 0 |
| Apr 2 (1403 s) | laser2d s42/1/2 | 12.24, 0.94, 13.77 | 0.103, 0.065, 0.120 | 33.95, 5.55, 51.41 | 0.363, 0.074, 0.363 | 3 (518), 5 (118), 4 (515) |
| Apr 2 (1403 s) | AMCL s42/1/2 | 1.33, 1.24, 2.03 | 0.145, 0.140, 0.151 | 10.48, 10.24, 16.19 | 0.146, 0.141, 0.165 | 10 (203), 10 (196), 10 (228) |

AMCL is about twice laser2d's error while tracking (median 0.14-0.15 vs 0.07-0.12 m on the long query) but its
losses are shorter and it does not end tens of metres off in two of three seeds.

## What was not measured

- AMCL on the alternative band (the optional extra row), other AMCL parameter sets, Nav2's default gate
  (0.25 m) and the other Stata map (Jan 18).
- AMCL runs are not independent filter realisations (no seed); laser2d alt and AMCL counts are small (12 runs).
- Wall time per AMCL run 25 s (180 s NCLT segment) to 8 min (1403 s Stata run) on one core each, 3-4 in parallel.

## Files

`nclt_pri/` NCLT AMCL runs (`amcl_<level>_t<segment>_s<seed>.csv`); `nclt_laser2d_alt/` laser2d with the alternative
band; `stata/` AMCL Stata tracking (`track_*`) and levels (`amcl_<level>_st<start>_s<seed>.csv`); `diag/` the 20k
particle check. Drivers: `experiments/nclt/amcl_run.py`, `amcl_run_nclt.py`, `amcl_prepare_nclt.py`, `amcl_offsets.cpp`,
`amcl_score.py`, `run_nclt_init_laser2d_alt.sh`, `nclt_laser2d.cpp --dump`; `experiments/laser2d_public/amcl_run_stata.py`,
`amcl_prepare_stata.py`, `mit_stata_laser2d.cpp` mode `dump`.
