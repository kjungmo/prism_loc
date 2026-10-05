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
| resample_threshold | double | `0.5` | fraction | Resample when Neff < threshold * N. |
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
| min_neff_fraction | double | `0.005` | fraction | `/diagnostics` WARNs when the effective sample size of the last correction falls below this share of the particle count (`0` disables). Healthy tracking measured >= 0.025 in the synthetic e2e `track` runs (not asserted). Not judged on the update forced by a reseed (`/initialpose`, `set_initial_pose`, accepted relocalization). |

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
| resample_threshold | double | `0.5` | fraction | Resample when Neff < threshold * N. |
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
| max_points | int | `500` | count | Scan points subsampled per measurement update. |
| base_height | double | `0.0` | m | Assumed sensor/base height above the map ground plane. |
| set_initial_pose | bool | `false` | bool | Seed a Gaussian at `initial_pose_*` on startup. |
| initial_pose_x | double | `0.0` | m | Initial pose x (when `set_initial_pose` is true). |
| initial_pose_y | double | `0.0` | m | Initial pose y (when `set_initial_pose` is true). |
| initial_pose_yaw | double | `0.0` | rad | Initial pose yaw (when `set_initial_pose` is true). |
| startup_timeout_s | double | `30.0` | s | `/diagnostics` grace after startup before a missing map, missing input or missing pose seed is reported as ERROR (drivers can be slow to start). |
| input_timeout_s | double | `1.0` | s | `/diagnostics` reports the scans/clouds as stopped (ERROR) after `max(input_timeout_s, input_timeout_periods x observed input period)` of silence. |
| input_timeout_periods | double | `5.0` | count | Input periods of silence (period measured on the steady clock) before input counts as stopped. |
| min_neff_fraction | double | `0.005` | fraction | `/diagnostics` WARNs when the effective sample size of the last correction falls below this share of the particle count (`0` disables). Healthy tracking measured >= 0.025 in the synthetic e2e `track` runs (not asserted). Not judged on the update forced by a reseed (`/initialpose`, `set_initial_pose`, accepted relocalization). |

## fusion3d backend (`prism_loc_fusion` node)

| name | type | default | unit | description |
|------|------|---------|------|-------------|
| global_frame | str | `"map"` | str | Fixed map frame the fused pose is expressed in. |
| odom_frame | str | `"odom"` | str | Odometry frame (child of map in the TF tree). |
| base_frame | str | `"base_link"` | str | Robot body frame the pose estimate refers to. |
| transform_tolerance | double | `0.1` | s | Future-dating added to the `map->odom` TF stamp. |
| imu_topic | str | `"/imu"` | str | sensor_msgs/Imu input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| points_topic | str | `"/points"` | str | PointCloud2 input topic (subscribed BEST_EFFORT / SensorDataQoS). |
| gnss_topic | str | `"/gnss"` | str | sensor_msgs/NavSatFix input topic (subscribed BEST_EFFORT / SensorDataQoS). |
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
| initial_yaw | double | `0.0` | rad | Assumed initial heading when seeding attitude from accel. |
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
