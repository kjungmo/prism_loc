# Addendum: AMCL baseline and a second height band (fixed before any AMCL run)

Written and committed before the first AMCL run. It adds to `INIT_PROTOCOL.md`; nothing there
changes (levels, seeds, segments, map, first 180 s, convergence criterion, scorer).

Why: the first protocol compared ndt3d (a particle filter that is told the initial spread) with a
6-DoF scan-to-map tracker (which cannot use a spread). A fair baseline for "told the spread" is
the standard MCL that accepts the same initial covariance: `nav2_amcl`.

## How AMCL is driven

- ROS 2 Humble `nav2_amcl` 1.1.20 and `nav2_map_server` 1.1.20 (RoboStack binaries, installed only
  into the micromamba env `ros2_humble`), unmodified.
- Offline, in lock step, with simulated time. One Python (`rclpy`) driver per run starts a fresh
  `map_server` + `amcl` + `lifecycle_manager`, publishes `/clock`, the `odom -> base_link` TF and
  one `LaserScan` per scan, and after each scan waits (up to 2 s wall clock) for AMCL to publish its
  `map -> odom` TF, which AMCL does once per processed scan. The driver then logs
  `map->odom (+) odom->base` against the ground truth, as the laser2d driver does with
  `compose(map_odom, odom_base)`.
- Why not a rosbag: playback has no back-pressure, so AMCL could drop scans, and the result would
  depend on machine load. Lock step gives AMCL every scan, once, in order.
- Inputs are exactly those of laser2d: the 2D grid map (0.1 m cells, occupied = 100, everything else
  free) and the 720-bin 360 degree scans produced by `nclt_laser2d.cpp` (new option `--dump`, which
  writes the map and scans it would use and exits), the same odometry (`odom.csv`), the same
  ground truth. The scan frame is `base_link` (sensor at the base, as in laser2d).
- Stata: the Apr 6 map and the dataset scans as in `experiments/laser2d_public`; the synthetic
  odometry of that driver (same seeds); the same start times and offset formula.
- Unobserved bins (infinite range) are sent as `range_max`, which AMCL skips.

## AMCL parameters

Nav2 Humble defaults, except the ones below (each is a match to prism_loc's `laser2d.yaml`).

| parameter | value | note |
|---|---|---|
| `laser_model_type` | `likelihood_field` | default |
| `min_particles`, `max_particles` | 500, 2000 | prism_loc uses 500-2000 (Nav2 default 500-2000) |
| `update_min_d`, `update_min_a` | 0.2, 0.2 | prism_loc's gate (Nav2 default 0.25, 0.2) |
| `max_beams` | 60 | same as default and as laser2d |
| `sigma_hit`, `z_hit`, `z_rand`, `laser_likelihood_max_dist` | 0.2, 0.5, 0.5, 2.0 | defaults, equal to laser2d |
| `alpha1..alpha5` | 0.2 (alpha5 unused) | defaults, equal to laser2d |
| `laser_min_range`, `laser_max_range` | -1 (take the scan's) | scan range 0.5-40 m on NCLT, 0.1-30 m on Stata |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | default |
| `recovery_alpha_slow/fast` | 0.0 | default (no random-particle injection) |
| `set_initial_pose` | false | the pose arrives on `/initialpose` |

Known differences from laser2d that I do not remove: AMCL's KLD bins (fixed 0.5 m, 10 degrees,
`pf_z` 0.99, `pf_err` 0.05) and its resampling rule (every `resample_interval` = 1 update, no N_eff
threshold) are AMCL's own. AMCL cannot be seeded: it calls `drand48` without a seed, so each fresh
process draws the same sequence. The three seeds therefore differ in the initial offset (and, on
Stata, in the odometry noise) but not in the filter's random draws. AMCL results with three seeds are
three offsets, not three independent filter realisations.

## Initial pose and level U

- Levels L0-L6: the offset pose of `init_offset.hpp` (NCLT) or of the Stata driver's init mode
  (Stata) published once on `/initialpose` in the `map` frame with covariance
  xx = yy = max(0.5, d)^2 m^2 and yaw variance (max(15, a) deg)^2, i.e. the spread laser2d and
  ndt3d are given. AMCL draws a Gaussian cloud with `max_particles`, as laser2d does with 2000.
- Level U: AMCL's own global localization, the `reinitialize_global_localization` service: particles
  uniform over the free cells of the same map. On the cropped NCLT map every cell that holds no
  height-band point counts as free (the map has no unknown state), so the cloud also covers
  unmapped area; this is the same grid BBS searches. Two settings are run and both are reported:
  U (500-2000 particles, comparable to laser2d) and U-20k (2000-20000 particles, a generous
  variant). The pose is scored from the first scan on; AMCL has no accept step.

## Measures

Unchanged: `nclt_init_score.py` on one CSV per run (`utime, err_xy`), converged = planar error under
1 m for at least 10 s starting within 120 s, RMSE from the convergence time to 180 s. Note that the
RMSE after convergence includes any later loss within the 180 s (laser2d's segment at 3400 s has
RMSEs of 7-68 m after a convergence at 0 s for this reason); the convergence count does not mean
"stays converged".

## Sanity checks before trusting a number

- AMCL at L0 on every segment must track (a failure at L0 everywhere means the driver, frames, time or
  map origin is wrong, and is debugged, not reported).
- Map origin and resolution, scan angle direction and the `odom -> base_link` frame are checked
  by comparing the first scan's likelihood-field fit at the ground-truth pose, and the count of scans
  AMCL processed against the count sent (reported per run).

## Honest note on the 2D projection of laser2d

The height band of the 2D projection (map points 1.0-2.5 m above the local ground; scan points
0.8-2.3 m above the sensor plane) was chosen after seeing L0 results: 16 combinations were compared
by the L0 error of seed 42 on the four segments, the same data the experiment scores
(`nclt_laser2d.cpp`, header). L0 is not an independent test of that choice, and the choice favours
the 2D methods (laser2d and AMCL, which share it) over ndt3d and the tracker, which use 3D data.

Pre-registered alternative band (one, not tuned, chosen to be the same geometry shifted up so that
the map band and the scan band cover the same heights above the ground): map points 1.5-3.0 m above
the local ground, scan points 0.85-2.35 m above the sensor plane (the ground is near -0.65 m
there, so these are 1.5-3.0 m above ground). laser2d is run with it on all levels, segments and
seeds (96 runs), with all other settings unchanged. Reported next to the original band, whatever
the result. If time permits AMCL is also run on the alternative band, as an extra labelled row.
