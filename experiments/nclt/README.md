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
- On the flat segment (2400 s) every setting tracks; the likelihood settings lower the RMSE there
  from 0.27-0.28 m (defaults) to 0.23-0.24 m.
- With the ground grid, the tempered likelihood removes most excursions, and with the cell floor
  and lower motion noise the remaining three are 3.2-4.8 s long and under 1.5 m.
- `update_min_period` (time-based updates) and fusion3d were not run here.
