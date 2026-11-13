# Parameters

Complete reference for every ROS 2 parameter declared by the `prism_loc` nodes,
one table per backend, maintained one-to-one against the `declare_parameter`
calls in the node sources. Defaults shown are the in-code defaults; the shipped YAML files under
`prism_loc/params/` and `prism_loc_fusion_ros/params/` override a subset of these.

Units column legend: `str` = string, `bool` = boolean, `count` = dimensionless
integer, `unitless`/`fraction`/`weight`/`score`/`enum` = dimensionless double,
otherwise an SI unit.

Every parameter is declared unconditionally (also `initial_pose_*` and `datum_*`,
which are used only when their switch is on), so `ros2 param list` shows it and a
YAML value always binds; `scripts/check_param_binding.py` checks this in CI for the
three shipped YAML files. Values are range-checked at startup: an out-of-range value
(for example `min_particles > max_particles`, a non-positive `sigma_hit`) aborts the
node with an error naming the parameter.

## laser2d backend (`prism_loc` node, `backend: "laser2d"`)

| name | type | default | unit | description |
|------|------|---------|------|-------------|
| backend | str | `"laser2d"` | str | Backend selector (`laser2d` \| `ndt3d`). |
| global_frame | str | `"map"` | str | Fixed map frame the `map->odom` TF is broadcast in. |
| odom_frame | str | `"odom"` | str | Odometry frame (child of map in the TF tree). |
| base_frame | str | `"base_link"` | str | Robot body frame the pose estimate refers to. |
| update_min_d | double | `0.2` | m | Min translation before a filter update runs. |
| update_min_a | double | `0.2` | rad | Min rotation before a filter update runs. |
| transform_tolerance | double | `0.1` | s | Future-dating added to the `map->odom` TF stamp. |
| tf_broadcast | bool | `true` | bool | Whether to broadcast the `map->odom` transform. |
| seed | int | `42` | count | RNG seed for the particle filter. |
| min_particles | int | `500` | count | Lower bound on the particle set size. |
| max_particles | int | `2000` | count | Upper bound on the particle set size. |
| resample_threshold | double | `0.5` | fraction | Resample when Neff < threshold * N. A correction multiplies each particle's weight onto the weight it had before, so the weights carry over while resampling is skipped. `0` never resamples: the weights then accumulate for the whole run. |
| kld_err | double | `0.05` | fraction | KLD max error bound between true/estimated distribution. |
| kld_z | double | `2.33` | unitless | Upper standard-normal quantile for KLD (~0.99). |
| kld_bin_xy | double | `0.5` | m | KLD histogram bin size in x and y. |
| kld_bin_yaw | double | `0.17` | rad | KLD histogram bin size in yaw (~10 deg). |
| alpha1 | double | `0.2` | unitless | Odometry noise: rotation from rotation. |
| alpha2 | double | `0.2` | unitless | Odometry noise: rotation from translation. |
| alpha3 | double | `0.2` | unitless | Odometry noise: translation from translation. |
| alpha4 | double | `0.2` | unitless | Odometry noise: translation from rotation. |
| scan_topic | str | `"/scan"` | str | LaserScan input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| map_topic | str | `"/map"` | str | OccupancyGrid map topic (transient_local, reliable). |
| max_beams | int | `60` | count | Laser beams subsampled per likelihood evaluation. |
| z_hit | double | `0.5` | weight | Likelihood-field mixture weight for a true hit. |
| z_rand | double | `0.5` | weight | Likelihood-field mixture weight for random noise. |
| sigma_hit | double | `0.2` | m | Std-dev of the Gaussian around obstacles in the field. |
| likelihood_max_dist | double | `2.0` | m | Max distance the likelihood field is computed out to. |
| laser_min_range | double | `0.0` | m | Min accepted beam range; `0.0` = use `LaserScan.range_min`. |
| laser_max_range | double | `0.0` | m | Max accepted beam range; `0.0` = use `LaserScan.range_max`. |
| set_initial_pose | bool | `false` | bool | Seed a Gaussian at `initial_pose_*` on startup. |
| initial_pose_x | double | `0.0` | m | Initial pose x (when `set_initial_pose` is true). |
| initial_pose_y | double | `0.0` | m | Initial pose y (when `set_initial_pose` is true). |
| initial_pose_yaw | double | `0.0` | rad | Initial pose yaw (when `set_initial_pose` is true). |
| try_global_localization | bool | `false` | bool | Run BBS global localization until first converged fix. |
| bbs_global_window | bool | `true` | - | Size the BBS search window from the map extent so relocalization covers the whole map (symmetric, capped by `bbs_max_linear_window`). Candidates whose robot cell is occupied or off-map are never returned. |
| bbs_max_linear_window | double | `50.0` | m | Per-axis cap on the ± half-window used when `bbs_global_window` is true (compute bound); the node warns when the cap leaves part of the map unsearched. |
| bbs_linear_window | double | `10.0` | m | ± translation search window around the map center, used only when `bbs_global_window` is false. |
| bbs_angular_window | double | `M_PI` (~3.14159) | rad | ± yaw search window (full circle by default). |
| bbs_angular_step | double | `0.0175` | rad | Yaw search resolution (~1 deg). |
| bbs_max_depth | int | `6` | count | Branch-and-bound pyramid levels. |
| bbs_max_beams | int | `120` | count | Laser beams subsampled for BBS scoring. |
| bbs_min_score_fraction | double | `0.4` | fraction | Min score/used-beams for a BBS match to be valid. |
| bbs_verify_top_k | int | `8` | count | Relocalization keeps up to this many distinct BBS modes of the first scan as hypotheses (`1` with `bbs_verify_scans: 1` restores the single-scan rule). |
| bbs_verify_scans | int | `9` | count | Odometry-chained scans (first included) over which the hypotheses are re-matched before a decision; the existing particle set keeps tracking meanwhile. Needs odom TF. |
| bbs_verify_evidence_gain | double | `40.0` | 1/fraction | Per-scan log-weight increment = gain x score fraction of the hypothesis' local re-match. |
| bbs_verify_min_posterior | double | `0.9` | probability | The best hypothesis is committed only if its posterior mass reaches this; otherwise the node reports AMBIGUOUS and retries instead of re-seeding. |
| bbs_verify_nms_xy | double | `1.0` | m | Two BBS leaves within this distance (and `bbs_verify_nms_yaw`) belong to the same mode. |
| bbs_verify_nms_yaw | double | `0.35` | rad | Yaw radius of a mode. |
| bbs_verify_track_linear_window | double | `0.3` | m | ± per-axis window of each hypothesis' local re-match per scan. |
| bbs_verify_track_angular_window | double | `0.1` | rad | ± yaw window of the local re-match. |
| bbs_verify_min_translation | double | `0.0` | m | Motion gate: a scan counts toward `bbs_verify_scans` only once odometry has moved this far since the last counted scan (or turned `bbs_verify_min_rotation`); skipped scans only carry their odometry forward. `0` disables the criterion. Recommended `0.3` (see README, Global localization). |
| bbs_verify_min_rotation | double | `0.0` | rad | Motion gate, rotation criterion (an in-place turn this large counts). `0` disables it. Recommended `0.3`. With both gate values `0` (default) every scan counts, as in v0.1. |
| startup_timeout_s | double | `30.0` | s | `/diagnostics` grace after startup before a missing map, missing input or missing pose seed is reported as ERROR (drivers can be slow to start). |
| input_timeout_s | double | `1.0` | s | `/diagnostics` reports the scans/clouds as stopped (ERROR) after `max(input_timeout_s, input_timeout_periods x observed input period)` of silence. |
| input_timeout_periods | double | `5.0` | count | Input periods of silence (period measured on the steady clock) before input counts as stopped. |
| min_neff_fraction | double | `0.005` | fraction | `/diagnostics` WARNs when the effective sample size of the last correction falls below this share of the particle count (`0` disables). The effective sample size is that of the weights after the last correction, carried over since the last resampling (a correction multiplies onto the previous weights when resampling is skipped), not that of one correction alone. The laser2d range quoted before this change, healthy tracking 0.024 to 0.026 at the lowest in two runs of the synthetic e2e `track` scenario (not asserted), was measured with weights that each correction overwrote; it has not been re-measured. Not judged on the update forced by a reseed (`/initialpose`, `set_initial_pose`, accepted relocalization). |

## ndt3d backend (`prism_loc` node, `backend: "ndt3d"`)

| name | type | default | unit | description |
|------|------|---------|------|-------------|
| backend | str | `"laser2d"` | str | Backend selector; set to `"ndt3d"` to use this backend. |
| global_frame | str | `"map"` | str | Fixed map frame the `map->odom` TF is broadcast in. |
| odom_frame | str | `"odom"` | str | Odometry frame (child of map in the TF tree). |
| base_frame | str | `"base_link"` | str | Robot body frame the pose estimate refers to. |
| update_min_d | double | `0.2` | m | Min translation before a filter update runs. |
| update_min_a | double | `0.2` | rad | Min rotation before a filter update runs. |
| transform_tolerance | double | `0.1` | s | Future-dating added to the `map->odom` TF stamp. |
| tf_broadcast | bool | `true` | bool | Whether to broadcast the `map->odom` transform. |
| seed | int | `42` | count | RNG seed for the particle filter. |
| min_particles | int | `500` | count | Lower bound on the particle set size. |
| max_particles | int | `2000` | count | Upper bound on the particle set size. |
| resample_threshold | double | `0.5` | fraction | Resample when Neff < threshold * N. A correction multiplies each particle's weight onto the weight it had before, so the weights carry over while resampling is skipped. `0` never resamples: the weights then accumulate for the whole run. |
| kld_err | double | `0.05` | fraction | KLD max error bound between true/estimated distribution. |
| kld_z | double | `2.33` | unitless | Upper standard-normal quantile for KLD (~0.99). |
| kld_bin_xy | double | `0.5` | m | KLD histogram bin size in x and y. |
| kld_bin_yaw | double | `0.17` | rad | KLD histogram bin size in yaw (~10 deg). |
| alpha1 | double | `0.2` | unitless | Odometry noise: rotation from rotation. |
| alpha2 | double | `0.2` | unitless | Odometry noise: rotation from translation. |
| alpha3 | double | `0.2` | unitless | Odometry noise: translation from translation. |
| alpha4 | double | `0.2` | unitless | Odometry noise: translation from rotation. |
| points_topic | str | `"/points"` | str | PointCloud2 input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| map_pcd_path | str | `""` | str | Absolute path to the `.pcd` map. Required; empty aborts startup. |
| ndt_resolution | double | `1.0` | m | NDT voxel (cell) edge length for the map. |
| voxel_min_points | int | `5` | count | Min points per voxel to form a valid NDT cell. |
| max_points | int | `500` | count | Scan points subsampled per measurement update. Must be >= `ndt_effective_points` unless that is 0 (the default). |
| base_height | double | `0.0` | m | Height of the lidar (the frame of the clouds) for the model. What it is depends on `ground_height_from_map`. Off (the default): z of the lidar in the map frame, i.e. for a map whose ground plane is z = 0, the lidar's height above the ground; it is not the height of the robot or of `base_frame`. On: the height of the lidar above the local ground, and the model adds the map's ground height under each particle to it. In both modes ndt3d is planar: of the transform `base_frame -> cloud frame` it uses x, y and yaw only; the transform's z is ignored, and so are its roll and pitch unless `use_sensor_tilt` is on; the lidar is taken to be level at this height. At the first cloud of a frame the node WARNs once if that transform has a roll or pitch over 0.01 rad (an INFO instead when `use_sensor_tilt` is on), and logs an INFO if its z differs from `base_height` by more than 0.05 m (not a fault: with the option off the two agree only when `base_frame` rides at map z = 0; with it on, only when `base_frame` rides on the ground). A `base_height` 5 cm wrong or a lidar pitched 0.02 rad already brings `match_fraction` under 0.65 on an exact map. |
| ground_height_from_map | bool | `false` | bool | Off (the default): every particle's cloud sits at the constant map z `base_height`, the previous behaviour. On: the node builds a ground height grid from the map points at startup and each particle's cloud sits at map z = ground(x, y) + `base_height`, so the model follows a ground whose height changes along the route. The ground height under a particle is looked up on the level nearest to the height under the current estimate (the hint) and interpolated between cell centres (see `ground_resolution`); a particle with no usable ground answer keeps the height of the estimate. After each filter update the hint moves to the ground under the new estimate. When the estimate gets no answer for 5 updates in a row, the hint re-anchors to the nearest level if that is within 1.5 m of it (both are core constants, not parameters; on a map with several levels over the same x, y the nearest level can be the wrong one, but holding the hint for ever is worse). A level further than 1.5 m is refused and the height is held (a hint on an upper floor does not jump to the floor below a gap), except right after a seed that found no ground cell, whose hint is only a guess. `/diagnostics` and the node log (throttled) WARN `no map ground under the estimate for N updates: height held at ...` meanwhile. A change of ground height of more than 1.5 m across a stretch without mapped ground, on a map with one level, needs a reseed: send `/initialpose` (its z selects the level, and a reseed has no distance limit). The published pose and the TF stay planar (z is not published). `/diagnostics` reports `ground_height_at_estimate` (the ground height the grid gave at the estimate after the last update, on the hint's level; `n/a` when it gave none, the held hint is not reported there), `ground_cloud_map_z` (the map z of the cloud frame that the last update used, before the hint moved), `ground_cell_at_estimate`, `ground_follow_state` (followed, kept, reanchored, no_ground, seeded), `ground_follow_misses` and `ground_particles_without_cell` (of the current particles, those the grid does not answer for against the current hint; counted by the 1 Hz diagnostics timer, not per update). Needs map points on the ground under the robot's path. After `/initialpose` or `set_initial_pose` the hint starts at the level nearest to the z of the initial pose at the seeded x, y (RViz sends z = 0, which picks the level nearest to 0). Cost: four hash lookups per particle per update, and one grid built at startup. Limits (measured on synthetic maps with 1 m cells and the defaults): it needs dense ground: with about 30 or more ground points per cell the error is around 1-2 cm (at most 0.08-0.10 m next to a hole); with 10 points per cell up to 0.07 m on flat ground, 0.15 m on a 0.2 slope and 0.17-0.20 m next to a hole; with 6 points per cell up to 0.36 m next to a hole (`ground_min_points` 5 is the minimum for a ground height, not for a good one). Vegetation or other scatter within 0.3 m above the ground lifts it: +0.13 to +0.15 m where no ground is mapped under it, +0.03 m when the scatter is twice as dense as the ground, +0.11 m when ten times. A wall foot or a vehicle side inside a cell that also has ground lifts the ground near it: about 0.10, 0.05 and 0.04 m at 0.1, 0.3 and 0.5 m from a wall, up to 0.12 m next to a vehicle with no ground mapped under it. A kerb (0.15 m step) inside a cell: about -0.04 m on the low side 0.25-0.5 m from the step, about -0.10 m at the step. Points up to `ground_layer_gap` below the ground pull it down by up to 0.04 m; points further below are a separate surface and have no effect. At the centre of a cell without a ground height the value depends on the direction of approach (jumps up to about 2 cm on sparse noisy ground). |
| ground_resolution | double | `1.0` | m | Edge of an xy cell of the ground grid (ndt3d, with `ground_height_from_map`). Must be finite and > 0. Per cell and per surface the grid stores a plane: its 16 sub-columns (4 x 4) each give a value, the median z of their points within 0.3 m above their lowest point (core field `GroundHeightParams::band`, not a parameter), at the centroid of those points; a least-squares plane is fitted through the values; with 8 or more sub-columns, those more than max(0.05 m, 2.5 x MAD of the residuals) above the plane (judged from a plane through the lower half; walls, pillars and objects only lift) are dropped and the plane is refitted (fewer than 8: the plain fit); the plane's value at the cell centre and its slope are stored. If fewer than 3 well-spread sub-columns remain the mean of their values is stored with slope 0. A query blends the planes of the four cells around it with bilinear weights, so a constant slope is followed exactly, also across missing cells. Remaining errors: curvature of the ground inside a cell; objects covering about half the sub-columns of a cell or more; and at the edge of the mapped ground, where the plane of the last cell answers alone from its centre outwards, up to half a cell beyond the cell's border (no answer further out) and, if that cell had too few well-spread points for a plane, its slope is 0 (an error of up to about slope x resolution / 2). A group of points in a thin strip that is taller than 0.6 m (the foot of a wall in a cell without ground) gives no ground height. A smaller value follows curvature better and leaves fewer points per cell (see `ground_min_points`). |
| ground_layer_gap | double | `0.2` | m | Smallest empty vertical gap that separates two surfaces in one cell. Between two consecutive points of a cell (sorted by z) a gap larger than this starts a separate surface, e.g. a floor and the underside of a ceiling above it, or two floors. Each surface gets its own plane and the hint selects one (`ground_max_step` limits); a blend never mixes two surfaces that differ by more than this. Must be finite and > 0. Set it above the largest vertical gap between neighbouring ground points inside a cell (about slope x point spacing), or a steep, sparse ground splits into groups with too few points. Sparse points above the ground (a wall sampled every 0.2 m or more in z) can form surfaces of their own; the hint, being the ground of the estimate, still selects the ground. A wall sampled more densely than that is one surface with the ground: its sub-columns are dropped as outliers. |
| ground_max_step | double | `0.5` | m | A surface counts for a particle only if its ground height is within this of the hint. Must be finite and > 0. Larger values follow steeper changes between two updates and risk a jump to another level; a particle with no surface within this keeps the height of the estimate. |
| ground_min_points | int | `5` | count | Points a cell, and a surface (vertical group) inside it, need to give a ground height. Must be >= 1. |
| use_sensor_tilt | bool | `false` | bool | Off (the default): the roll and pitch of `base_frame -> cloud frame` are ignored and the node WARNs at startup when they exceed 0.01 rad, the previous behaviour. On: each cloud is rotated by the roll and pitch of that transform before matching, so the model's level-sensor assumption holds for a tilted mount (with R the transform's rotation and yaw the yaw the model uses, points are multiplied by Rz(-yaw) R, which is Ry(pitch) Rx(roll) for R = Rz(yaw) Ry(pitch) Rx(roll); the yaw and x, y still go through the transform as before). The transform's z is still not used (`base_height` is). This is a static mount correction: the roll and pitch of the robot body on a slope are not handled. Reported in `/diagnostics` as `use_sensor_tilt`. |
| ndt_effective_points | double | `0.0` | count | The scan counts as this many independent points in the likelihood: the log-likelihood of a particle is the mean of `log(score + score_floor)` over the `max_points` subsample times this value. It makes the likelihood independent of how many points the subsample holds, and sets how sharply one scan separates the particles. Larger values weigh a scan more. `0` (the default) is the plain sum over the subsample, the previous behaviour. On a real outdoor recording, values around 5 to 10, alone or with `ndt_min_sigma` 0.05 to 0.10 m, reduced position excursions. Cost, measured in the repository's synthetic end-to-end world (`scripts/synthetic_e2e.py`, one machine): with `10` the filter converges more slowly after a reseed, and the end-to-end limits leave little margin, which is why it is off by default. `ndt3d_track`, largest position error from 10 s after `/initialpose`: 0.05 to 0.29 m in five runs with `0`; 0.08 to 0.54 m in four runs with `10` (test limit 0.5 m, exceeded in one run). `ndt3d_strict`, from 6 s after `/initialpose`: 0.04 to 0.15 m in five runs with `0`; 0.41 to 0.47 m in four runs with `10` (limit 0.5 m). After 30 s the largest error is 0.02 to 0.03 m with either value. A cloud with fewer points than this value is still weighted as that many points. Must be in `[0, max_points]`. |
| ndt_min_sigma | double | `0.0` | m | Absolute floor on the standard deviation of every NDT cell along each of its principal axes, on top of the relative floor (0.001 x the largest eigenvalue). A flat cell, such as a wall or the ground, otherwise gets a spread of millimetres and scores a point a few centimetres off its surface as about 0. `0` restores the previous behaviour (relative floor only). The scoring of a pose for `match_fraction` ignores this floor; the estimated pose, and so the reported `match_fraction`, can change. Off by default. The floor widens the likelihood around thin map surfaces, which reduced position excursions on a real outdoor recording when used together with `ndt_effective_points`; using it without `ndt_effective_points` is not recommended. On a map whose surfaces are thinner than the floor it lets the estimate move a few centimetres, which lowers `match_fraction` at the tracked pose (one run each of the `ndt3d_track` scenario of `scripts/synthetic_e2e.py` with `ndt_effective_points: 10`, `match_fraction` in the statuses after t = 15 s, the 5-update mean: `ndt_min_sigma` 0 gave min 0.77 and median 0.81; 0.05 gave min 0.67 and median 0.70; 0.10 gave min 0.62 and median 0.66, so at 0.10 m the default `min_match_fraction` 0.65 WARNs on that healthy run), so re-measure `min_match_fraction` on a healthy run when you enable it. Values around 0.05 to 0.10 m are the range to try. Must be in `[0, ndt_resolution]`. |
| set_initial_pose | bool | `false` | bool | Seed a Gaussian at `initial_pose_*` on startup. |
| initial_pose_x | double | `0.0` | m | Initial pose x (when `set_initial_pose` is true). |
| initial_pose_y | double | `0.0` | m | Initial pose y (when `set_initial_pose` is true). |
| initial_pose_yaw | double | `0.0` | rad | Initial pose yaw (when `set_initial_pose` is true). |
| initial_pose_z | double | `0.0` | m | Initial pose z (when `set_initial_pose` is true). Used only with `ground_height_from_map`: on a multi-level map it selects the level the ground height starts on (the one nearest to this z). |
| startup_timeout_s | double | `30.0` | s | `/diagnostics` grace after startup before a missing map, missing input or missing pose seed is reported as ERROR (drivers can be slow to start). |
| input_timeout_s | double | `1.0` | s | `/diagnostics` reports the scans/clouds as stopped (ERROR) after `max(input_timeout_s, input_timeout_periods x observed input period)` of silence. |
| input_timeout_periods | double | `5.0` | count | Input periods of silence (period measured on the steady clock) before input counts as stopped. |
| min_neff_fraction | double | `0.0` | fraction | The laser2d check on the effective sample size (`/diagnostics` WARNs when it falls below this share of the particle count), off by default for ndt3d: the in-code default is `0.0` when `backend` is `ndt3d` and `0.005` otherwise. Measured with the ndt3d model and the plain likelihood, the defaults `ndt_effective_points: 0` and `ndt_min_sigma: 0` (4 healthy runs of the 3D synthetic world, pose error at most 0.17 m after t = 30 s): one particle takes nearly all the weight at every update, n_eff 1-2 of 500 (share 0.002-0.004); with the pose off the map every particle weighs the same and n_eff equals the particle count. It does not tell good tracking from bad; `min_match_fraction` does. |
| min_match_fraction | double | `0.65` | fraction | `/diagnostics` WARNs `low scan-to-map match` while the share of scan points that lie on mapped surfaces, averaged over the last 5 filter updates, is below this (`0` disables). A point counts when it is within 3 standard deviations of the NDT cell it falls in, at the published pose; the points are the `max_points` subsample the filter scores. Reported as `match_fraction`. Not judged on the update forced by a reseed. The figures below were measured with the defaults (`ndt_effective_points: 0`, `ndt_min_sigma: 0`, the plain likelihood); the scoring of a pose ignores `ndt_min_sigma`, but the estimated pose, and so the reported `match_fraction`, can change with it. Measured with the ndt3d model on the 3D synthetic world (README, ndt3d status): healthy 0.73-0.89 with the exact prior map (0.94-1.0 with 2 or 5 cm of noise on it), lost up to 0.78 on the node (0.77 in the replay of `experiments/ndt3d_replay.cpp`). OK is not proof of a correct pose; the measured miss and false-alarm rates are in the README. Set it per site, just under the lowest `match_fraction` of a known-good lap (0.03 to 0.05 below it; `ros2 topic echo /diagnostics | grep -A1 match_fraction`): lower gives lost poses room, higher alarms on a healthy robot. |
| match_verify_updates | int | `15` | count | After a reseed (`/initialpose`, `set_initial_pose`) `/diagnostics` WARNs `pose not verified since the last reseed` until this many filter updates have been judged (about 3 m of travel at `update_min_d` 0.2 m; a standing robot stays unverified). A pose 3.6 m off matched above the threshold for its first 9 to 15 judged updates (replay, 12 runs). `0` disables; nothing is reported when `min_match_fraction` is `0`. |

## fusion3d backend (`prism_loc_fusion` node)

| name | type | default | unit | description |
|------|------|---------|------|-------------|
| global_frame | str | `"map"` | str | Fixed map frame the fused pose is expressed in. |
| odom_frame | str | `"odom"` | str | Odometry frame (child of map in the TF tree). |
| base_frame | str | `"base_link"` | str | Robot body frame the pose estimate refers to. Inputs measured in another frame need the transform `base_frame -> frame` on TF (README, Sensor frames). |
| points_frame | str | `""` | str | Frame the point clouds are expressed in. Empty: each cloud's `header.frame_id`. Equal to `base_frame`: used as it is. Any other frame: the full transform `base_frame -> frame` is looked up on TF and the lidar pose from NDT is converted to the pose of `base_frame`; without it the cloud is not used. |
| imu_frame | str | `""` | str | Frame of the IMU. Empty: each message's `header.frame_id`. Any frame other than `base_frame` needs `base_frame -> frame` on TF; only its rotation is applied (the lever arm is not compensated). Without it the IMU is not used. |
| gnss_frame | str | `""` | str | Frame of the GNSS antenna. Empty: each fix's `header.frame_id`. Any frame other than `base_frame` needs `base_frame -> frame` on TF; only its translation (the lever arm) is used: the fix is fused as the position of the antenna and corrects position, velocity, accelerometer bias, roll, pitch and the horizontal gyro biases; the heading and the vertical gyro bias are corrected only while the IMU has measured horizontal acceleration (a start, a stop, a turn at speed), the heading uncertainty the fix sees is bounded at 0.35 rad, and without lidar fixes the heading drifts with the vertical gyro bias; `/diagnostics` reports it (`heading_source`, `heading_sigma_rad`, `heading_capped`, `lever_release`, `gnss_only_s`, and a WARN after 30 s; README, Sensor frames). Without it the fix is not used. The bound (0.35 rad) and the release (1.0 m/s over a 4 s window) are `EskfParams` defaults, not node parameters. |
| transform_tolerance | double | `0.1` | s | Future-dating added to the `map->odom` TF stamp. |
| imu_topic | str | `"/imu"` | str | sensor_msgs/Imu input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| points_topic | str | `"/points"` | str | PointCloud2 input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| gnss_topic | str | `"/gnss"` | str | sensor_msgs/NavSatFix input topic (subscribed BEST_EFFORT / SensorDataQoS). With a GNSS antenna off `base_frame`, the heading of a start without `/initialpose` or lidar is found from a start or a stop of the robot, which needs fixes at **1 Hz or faster** (supported: 1 to 100 Hz, measured: `experiments/README.md`, suites `hrate` and `hratex`). From 0.8 Hz to 1 Hz (a median interval of 1.05 s to 1.25 s) it finds the heading less often and `/diagnostics` WARNs `"GNSS fixes come every N s (slower than 1 Hz) ..."` (`heading_align_rate: degraded`). Under 0.8 Hz the filter does not try, does not mark the start as unknown, and `/diagnostics` WARNs `"... (slower than 0.8 Hz) ..."` (`heading_align_rate: unsupported`, `gnss_fix_period_s`): give `/initialpose` or use the lidar. |
| map_pcd_path | str | `""` | str | Absolute path to the `.pcd` map. Required; empty aborts startup. |
| ndt_resolution | double | `1.0` | m | NDT voxel (cell) edge length for the map. |
| ndt_step_size | double | `0.1` | m | NDT line-search maximum step length. |
| ndt_epsilon | double | `0.01` | m | NDT convergence threshold on the transform delta. |
| ndt_max_iter | int | `30` | count | NDT max optimization iterations per align. |
| ndt_max_fitness | double | `2.0` | score | Reject NDT results whose fitness exceeds this. |
| points_voxel_leaf | double | `0.5` | m | Voxel-grid downsample leaf size for incoming scans. |
| pose_pos_std | double | `0.1` | m | Assumed std-dev of the NDT position measurement. |
| pose_rot_std | double | `0.05` | rad | Assumed std-dev of the NDT orientation measurement. |
| sigma_acc | double | `0.01` | m/s^2 | Accelerometer white-noise std-dev (ESKF process). |
| sigma_gyro | double | `0.001` | rad/s | Gyroscope white-noise std-dev (ESKF process). |
| sigma_acc_bias | double | `0.0001` | m/s^3 | Accelerometer bias random-walk std-dev. |
| sigma_gyro_bias | double | `0.00001` | rad/s^2 | Gyroscope bias random-walk std-dev. |
| reset_jacobian | bool | `false` | - | Apply the ESKF reset Jacobian G = blkdiag(I, I, I - [dθ/2]×, I, I) to the covariance after each error injection (Solà 2017 §7.2). Off reproduces v0.1 (G = I). |
| use_datum | bool | `false` | bool | Use a fixed ENU datum instead of the first GNSS fix. |
| datum_lat | double | `0.0` | deg | Datum latitude (when `use_datum` is true). |
| datum_lon | double | `0.0` | deg | Datum longitude (when `use_datum` is true). |
| datum_alt | double | `0.0` | m | Datum altitude (when `use_datum` is true). |
| gnss_min_status | int | `0` | enum | Min `NavSatFix.status.status` accepted (>= this value). |
| gnss_max_pos_cov | double | `25.0` | m^2 | Reject GNSS fixes whose x position variance exceeds this. |
| initial_yaw | double | `0.0` | rad | Assumed initial heading when seeding attitude from accel. With a GNSS antenna off `base_frame` and no `/initialpose` it is a guess: the filter starts with the spread of an unknown heading (`heading_sigma_rad` 1.8, `heading_source` `none`) until the first start or stop of 0.25 m/s or more finds the heading (`motion`) or the lidar does. That needs GNSS fixes at 1 Hz or faster (see `gnss_topic`); with slower fixes the start is not marked as unknown and the heading stays a guess. |
| initial_yaw_known | bool | `false` | | `true` says `initial_yaw` is the true heading to within about 0.3 rad (a dock, a surveyed start). The filter then starts with its own heading spread (0.71 rad) as it did before; with a wrong `initial_yaw` and this set, the first start can tilt the filter and run the height away. `/initialpose` has the same effect for one start and needs no parameter. |
| map_to_base_fallback | bool | `true` | bool | While `odom_frame -> base_frame` is unavailable, broadcast `map -> base_link` instead of `map -> odom`. Set `false` when wheel odometry or an EKF owns `odom -> base_link`: if that node starts after this one, `base_link` would otherwise have two parents in the TF tree. With `false` no TF is broadcast until odometry appears; the pose topics still publish. |
| imu_queue_depth | int | `5` | count | Depth of the best-effort IMU subscription queue (`5` = the `SensorDataQoS` depth used before this parameter existed). IMU messages wait in it while a point cloud is being registered on the same executor; raise it (recommended `200`) when `/diagnostics` reports IMU gaps. |
| startup_timeout_s | double | `30.0` | s | `/diagnostics` grace after startup before a missing IMU or a filter that never initialized is reported as ERROR. |
| input_timeout_s | double | `1.0` | s | `/diagnostics` reports the IMU as stopped (ERROR) after `max(input_timeout_s, input_timeout_periods x IMU period)` of silence. |
| input_timeout_periods | double | `5.0` | count | IMU periods of silence before the IMU counts as stopped. |
| correction_timeout_s | double | `5.0` | s | `/diagnostics` WARNs when no NDT correction has been accepted for this long (the pose is then dead-reckoning on the IMU). |

## Launch arguments

Passed on the `ros2 launch` command line (e.g. `map:=/abs/map.yaml`), not as
node parameters. `use_sim_time` is forwarded to every node.

| name | backend(s) | default | description |
|------|-----------|---------|-------------|
| map | laser2d | (required) | Path to the map `.yaml` for `map_server` (OccupancyGrid). |
| map_pcd_path | ndt3d | (required) | Path to the prior map `.pcd`. |
| map_pcd_path | fusion3d | `""` | Path to the prior map `.pcd` (empty aborts node startup). |
| use_sim_time | laser2d, ndt3d, fusion3d | `false` | Use the `/clock` topic instead of the system clock. |
| rviz | laser2d, ndt3d | `true` | Launch RViz with the bundled config. |
| rviz | fusion3d | `false` | Launch RViz with the bundled config. |
