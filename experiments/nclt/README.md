# ndt3d on the public NCLT dataset

The University of Michigan North Campus Long-Term dataset (NCLT,
http://robots.engin.umich.edu/nclt/; Carlevaris-Bianco, Ushani and Eustice, IJRR 2016) drives the
same campus many times over a year with a Velodyne HDL-32E, wheel odometry and a ground truth from
SLAM. This experiment builds a prior map from one session and localizes another session in it
with the ndt3d update loop of `prism_loc::LocalizationNode`, driven offline on `prism_loc_core`.

- Prior map: session 2012-01-08, every 5th scan placed with the ground truth, 0.2 m voxels
  (31.6 M points).
- Query: session 2012-02-04 (four weeks later), its wheel odometry (`odometry_mu_100hz.csv`) as the
  motion input and its ground truth for scoring. Four segments of 1000 s starting at 400, 1400,
  2400 and 3400 s (the first ~250 s of the session are outside the area the map covers), three
  seeds each: 12 runs per setting.
- Seed: the ground-truth pose at the first scan, spread as the node's `/initialpose` floor
  (0.5 m, 0.5 m, 0.2617 rad, 2000 particles). Node defaults otherwise (`ndt3d.yaml`).
- Score (`nclt_score.py`): planar error against the ground truth at every scan; an excursion is a
  planar error over 1 m for more than 2 s.

## Run

```sh
# DATA_DIR: 2012-01-08/velodyne_sync, 2012-02-04/velodyne_sync (from the <date>_vel.tar.gz
# archives), groundtruth_<date>.csv, 2012-02-04/odometry_mu_100hz.csv (from <date>_sen.tar.gz).
bash experiments/nclt/run_nclt.sh DATA_DIR OUT_DIR 6
```

About 2.5 min per run on one core; about 1 GB of memory per run.

## Result

12 runs per setting (`experiments/results/nclt/score.txt` has every run). Likelihood settings
are `ndt_effective_points` / `ndt_min_sigma`; "ground" is `ground_height_from_map`; "alpha 0.05" sets
`alpha1..4` to 0.05 instead of the default 0.2.

| setting | runs with an excursion | excursions (total) | max error | median planar RMSE | median N_eff / N |
|---|---|---|---|---|---|
| defaults (0 / 0, constant height) | 9 of 12 | 40 (1354.6 s) | 354.9 m | 0.536 m | 0.20 % |
| 10 / 0 | 7 of 12 | 27 (1837.1 s) | 247.5 m | 2.145 m | 6.04 % |
| 5 / 0.10 | 8 of 12 | 26 (1872.9 s) | 346.2 m | 2.372 m | 10.82 % |
| ground | 10 of 12 | 17 (89.0 s) | 8.35 m | 0.360 m | 0.20 % |
| ground, 10 / 0 | 4 of 12 | 5 (16.0 s) | 4.81 m | 0.312 m | 5.52 % |
| ground, 5 / 0.10, alpha 0.05 | 3 of 12 | 3 (11.4 s) | 1.50 m | 0.263 m | 20.57 % |

- The session climbs and descends about 18 m. With the constant `base_height` the segments that
  start at 1400 and 3400 s are lost (tens to hundreds of metres) whatever the likelihood setting;
  with the ground-height grid the largest error of any run is 8.35 m (one run; 4.81 m and 1.50 m
  with the likelihood settings added).
- On the flat segment (2400 s) the settings without the ground grid all track; the likelihood
  settings lower the RMSE there from 0.27-0.28 m (defaults) to 0.23-0.24 m. The ground grid alone
  has one 40 s excursion there (one seed); with the likelihood settings added it has none.
- With the ground grid, the tempered likelihood removes most excursions, and with the cell floor
  and lower motion noise the remaining three are 3.2-4.8 s long and under 1.5 m.
- `update_min_period` (time-based updates) and fusion3d were not run here.

## Baseline: scan-to-map NDT tracker

`nclt_ndt_tracker.cpp` is the common alternative to a particle filter: predict the 6-DoF pose with
the wheel-odometry increment, register every scan to the map with PCL's NDT (resolution 1 m,
step 0.1, epsilon 1e-4, 30 iterations, 1 m scan voxels; map cropped to 150 m around the segment's
path) and take the result when PCL reports convergence. Seeded at the ground-truth pose; one run
per segment (deterministic). Scores in `experiments/results/nclt/score_ndt_tracker.txt`.

| method | runs with an excursion | max error | median planar RMSE |
|---|---|---|---|
| scan-to-map NDT tracker | 0 of 4 | 0.62 m | 0.153 m |
| ndt3d, best setting above | 3 of 12 | 1.50 m | 0.263 m |

The tracker is more accurate here. It estimates a 6-DoF pose and registers every scan with all
its points, while ndt3d estimates a planar pose and corrects every 0.2 m of travel with a
500-point subsample. Not measured here: recovery from a wrong initial pose (the tracker starts
at the ground truth; ndt3d from particles spread 0.5 m / 15 degrees around it).

## laser2d on NCLT projected to 2D

`nclt_laser2d.cpp` runs the laser2d update loop of `LocalizationNode` (likelihood field, `laser2d.yaml`
defaults: 60 beams, sigma 0.2 m, alpha 0.2, 500-2000 particles) and its BBS global localization on the
same NCLT map and query as above, projected to 2D. It is for the initialization-robustness protocol
(`INIT_PROTOCOL.md`); no results yet.

- Map: occupied cells (0.1 m) hold a map point 1.0-2.5 m above the local ground (lowest map point of
  the 1 m cell); the grid covers the bounding box of the segment's 180 s ground-truth path plus 80 m.
- Scan: the cloud is levelled with the odometry roll and pitch; points with z 0.8-2.3 m in the body
  frame become a 360 degree scan of 720 bins (0.5 degree, nearest range per bin, 0.5-40 m).
- The two bands were picked among 16 combinations by the L0 error (seed 42) on the four segments,
  so L0 is not an independent test of them. Lower bands let ground returns and low vegetation in and
  the filter lost the pose. With the chosen bands L0 (seed 42, 180 s) tracked the segments at 400,
  1400 and 2400 s (median error 0.31, 0.18, 0.13 m) but not the one at 3400 s (median 6.1 m, 90th
  percentile 11.2 m): the descent there is where the 2D projection fails.
- Seed: levels L0-L6 start at the ground truth moved by `initOffset(seed, segment_start_s, d, a)` from
  `init_offset.hpp` (direction uniform in [0, 2 pi) and yaw sign from `std::mt19937_64(seed * 1000003 +
  segment_start_s)`; first draw the direction, second `bernoulli_distribution(0.5)` for the sign),
  Gaussian with sigma `max(0.5, d)` m and `max(15, a)` degrees, 2000 particles, filter seed = run seed.
- Level U (`--param level=7`): as the node, `RelocalizationVerifier` (laser2d.yaml defaults: 8 modes, 9
  odometry-chained scans, posterior 0.9) starts with a map-wide BBS search of the first scan and is
  fed every following scan; on accept the filter is seeded at (0.2 m, 0.2 m, 0.1 rad); on ambiguous
  it starts over. The node's 50 m window cap is raised to cover the cropped map (it would not be
  global on this 215 x 323 m grid). Rows before the accept have `nan` estimate and error, which the
  scorer counts as not below 1 m.
- Run: `bash experiments/nclt/run_nclt_init_laser2d.sh DATA_DIR OUT_DIR JOBS` (96 runs, 180 s each),
  scored per level with `nclt_init_score.py`. One run takes 4 s (L0-L6, 230 MB peak) to 24 s (U, 410
  MB peak) on one core, about 3 s of it reading the 31.6 M-point map.
