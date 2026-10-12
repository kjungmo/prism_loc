<div align="center">

# PRISM-Loc

**A pluggable LiDAR localization stack for ROS 2 Humble — three backends over
shared middleware-free estimator cores, one standard `map → odom` contract.**

[![CI](https://github.com/kjungmo/prism_loc/actions/workflows/ci.yml/badge.svg)](https://github.com/kjungmo/prism_loc/actions/workflows/ci.yml)
[![License](https://img.shields.io/github/license/kjungmo/prism_loc?color=blue)](LICENSE)
![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
[![Paper](https://img.shields.io/badge/Paper-PDF-b31b1b)](docs/paper/main.pdf)
[![Sponsor](https://img.shields.io/github/sponsors/kjungmo?logo=githubsponsors&color=ea4aaa)](https://github.com/sponsors/kjungmo)

[Overview](#overview) &nbsp;·&nbsp; [Paper](#-paper) &nbsp;·&nbsp;
[Install](#-prerequisites) &nbsp;·&nbsp; [Quick start](#-quick-start) &nbsp;·&nbsp;
[Evaluation](#-evaluation-synthetic) &nbsp;·&nbsp; [Docs](#-documentation) &nbsp;·&nbsp;
[Roadmap](#-roadmap)

<img src="docs/assets/architecture.png" width="540" alt="PRISM-Loc architecture: two middleware-free estimator cores (MCL and ESKF) under two thin ROS 2 node packages sharing one topic/TF contract"/>

</div>

`prism_loc` localizes a mobile robot inside a prior map from a **2D LiDAR**
(`LaserScan` vs `OccupancyGrid`), a **3D LiDAR** (`PointCloud2` vs a `.pcd` map),
or a **3D LiDAR fused with IMU and RTK-GNSS**. Pick a backend; the output contract
— `map→odom` TF, pose with covariance, `/initialpose` seeding — follows the
`nav2_amcl` / `hdl_localization` / `robot_localization` conventions ROS users
already know.

## 📢 News

- **2026-07** — `v0.1` released: `laser2d`, `ndt3d`, and `fusion3d` backends,
  branch-and-bound global relocalization, and a systems-paper draft with a
  fully reproducible synthetic evaluation
  ([`docs/paper/main.pdf`](docs/paper/main.pdf)).

## Overview

| Backend | Sensors | Map | Estimator | State | Global relocalization |
|---|---|---|---|---|---|
| **`laser2d`** | 2D LiDAR + wheel odometry (TF) | `OccupancyGrid` (`map.yaml`) | likelihood-field MCL, KLD-adaptive | planar (x, y, yaw) | ✅ branch-and-bound (BBS), param or on-demand service |
| **`ndt3d`** | 3D LiDAR + wheel odometry (TF) | `.pcd` point cloud | NDT-MCL (same particle-filter core) | planar (x, y, yaw) | — (`/initialpose`) |
| **`fusion3d`** | 3D LiDAR + IMU + RTK-GNSS | `.pcd` point cloud | 15-state error-state Kalman filter | full 6-DoF + IMU biases | auto-init from IMU attitude + first valid RTK fix |

**Why "PRISM"** — **P**article-filter · **R**TK · **I**nertial ·
**S**can-matching: the four ingredients this stack fuses to localize against a
prior map. The name also nods to the optics: a prism splits one beam into its
components, just as `prism_loc` splits one estimator core into multiple
sensor/observation paths — grounded in **NDT-MCL** (Saarinen et al., IROS 2013),
which showed a 3D NDT map can serve as the measurement model inside the very
same particle filter that 2D AMCL uses.

**Design principle** — all filter mathematics live in two pure C++17 + Eigen
libraries (`prism_loc_core`, `prism_loc_fusion`) with **no ROS and no PCL**,
unit-tested deterministically with gtest and a seedable RNG. `rclcpp`, `tf2`,
and PCL appear only in the two thin node packages (`prism_loc`,
`prism_loc_fusion_ros`). CI builds both paths: the bare-`cmake` cores and the
full `colcon` workspace in a `ros:humble` container. It then runs the nodes over
real ROS topics: each shipped launch file starts headless on generated maps and
[`scripts/check_param_binding.py`](scripts/check_param_binding.py) fails if any
YAML key is not declared and applied, and
[`scripts/synthetic_e2e.py`](scripts/synthetic_e2e.py) drives `laser2d` on a
synthetic world (ray-cast scans, drifting odometry) and asserts position error
≤ 0.15 m and yaw error ≤ 0.05 rad after 10 s, a `map→odom` stamp lead of
0–0.15 s, and no TF gap over 0.5 s; after a 7 s scan gap, `map→odom` resumes
within 1 s and the position error is ≤ 0.15 m from 15 s after the gap
(re-convergence measured 3–12 s; a 3 s bound is not met by the current filter).
It also asserts poses at a 0.1x simulated clock and with a coarse (10 Hz,
0.5x) simulated clock, with scans stamped ahead of odometry, no low-n_eff warning after `/initialpose` on a standing robot, and the
`/diagnostics` status for stopped scans, missing odometry, zero-stamped scans and a
stuck ROS clock. With `ndt3d.yaml` the same script drives `ndt3d` in the 3D version of
that world (16-ring lidar, prior map sampled from the same surfaces) and asserts position
error ≤ 0.5 m after 10 s and ≤ 0.25 m after 30 s, yaw error ≤ 0.05 rad, an OK status
throughout healthy tracking, and the `low scan-to-map match` WARN after a kidnap, for a
pose off the map and for a wrong `base_height`. This is a ROS-path check on synthetic
data, not a field result.
[`scripts/fusion_extrinsic_check.py`](scripts/fusion_extrinsic_check.py) feeds `fusion3d`
a robot that stands, or turns on the spot, with its lidar, IMU and GNSS antenna mounted
off `base_link`, and asserts the pose of `base_link` within 0.05 m and 0.02 rad, the
`/diagnostics` status when a mount is missing from TF, and, on GNSS and IMU alone with
sensor noise, height, tilt and heading within 0.15 m, 0.03 rad and 0.05 rad and a
horizontal error within twice the reported sigma (see
[Sensor frames](#sensor-frames-fusion3d)). CI runs it on an optimized build of the two
fusion packages.
GoogleTest cases: 50 in `prism_loc_core`, 20 in `prism_loc_fusion`, 29 in
`prism_loc`, 24 in `prism_loc_fusion_ros` (123 in total).

Run the same checks locally from a built workspace (`rosdep install` as below,
plus `sudo apt-get install python3-numpy python3-yaml`):
```bash
source install/setup.bash
bash src/prism_loc/scripts/launch_checks.sh laser2d ndt3d fusion3d
python3 src/prism_loc/scripts/synthetic_e2e.py install/prism_loc/share/prism_loc/params/laser2d.yaml
python3 src/prism_loc/scripts/synthetic_e2e.py install/prism_loc/share/prism_loc/params/ndt3d.yaml   # needs a Release build
python3 src/prism_loc/scripts/fusion_extrinsic_check.py install/prism_loc_fusion_ros/share/prism_loc_fusion_ros/params/fusion3d.yaml
```

**TF waits.** A lookup of `odom→base_link` at a scan's stamp waits for odometry to
catch up: 0.1 s of ROS time (plus up to 30 ms when the clock moved 30 ms or more in
the poll that reaches the budget), capped at 1 s of steady time. Limits: a `/clock` that changes less often
than once per second of wall time counts as frozen and is not waited on; below 0.1x
the 1 s cap ends the wait before 0.1 s of ROS time; the callback in which a clock
stops can hold up to the cap. Whether the clock is frozen is judged across waits, so
a coarse `/clock` (for example 10 Hz of sim time at 0.5x) still gets its full 0.1 s.
The buffer is polled every 10 ms as in `tf2_ros`; a live clock (about 10 ms per poll)
gets the 30 ms grace only when that poll is delayed by 30 ms or more, so a live robot
otherwise behaves as with `tf2_ros`. Smooth playback at about 3x or faster moves 30 ms
or more per poll and always gets the grace.

## 📄 Paper

The architecture, estimator cores, interface contract, and a reproducible
synthetic evaluation are described in the systems-paper draft (LaTeX sources +
PDF under [`docs/paper/`](docs/paper/)):

> **PRISM-Loc: Three LiDAR Localization Backends Behind One ROS 2 Contract,
> with Middleware-Free Estimator Cores** — [PDF](docs/paper/main.pdf)

The paper describes commit cdb81bf; later commits add tests and the `/diagnostics`
status, and with default parameters leave the estimates unchanged except that scans
and clouds with a zero header stamp are now dropped; fusion3d's `~/odometry` linear
twist is now expressed in `base_link` (it was the world frame); and fusion3d now applies
the mount of a sensor whose frame is not `base_link` (it used every input as if it were
measured at `base_link`; inputs already in `base_link` give the same values as before, see
[Sensor frames](#sensor-frames-fusion3d)).

If `prism_loc` is useful in your research, please cite it
(see also [`CITATION.cff`](CITATION.cff)):

```bibtex
@unpublished{kang2026prismloc,
  author = {Kang, Jung Mo},
  title  = {{PRISM-Loc}: Three {LiDAR} Localization Backends Behind One
            {ROS}~2 Contract, with Middleware-Free Estimator Cores},
  year   = {2026},
  note   = {Draft manuscript, available at
            \url{https://github.com/kjungmo/prism_loc} under docs/paper/},
}
```

## ⚙️ Prerequisites

**Core-only path (no ROS)** — builds/tests the two estimator libraries by
themselves. This is exactly what CI installs in
[`.github/workflows/ci.yml`](.github/workflows/ci.yml):
```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends cmake g++ libeigen3-dev libgtest-dev
```

**Full ROS path** — needed for the launch files / RViz / TF:
1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
   (`ros-humble-desktop` — includes `rviz2`).
2. From the workspace root (with this repo checked out at
   `<ws>/src/prism_loc`), resolve the ROS package dependencies declared in each
   `package.xml` (pulls in `nav2_map_server`, `nav2_lifecycle_manager`,
   `rviz2`, PCL, ...):
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## 🚀 Quick start

```bash
# 1. Estimator cores — build & test with the plain system toolchain (no ROS):
cmake -S prism_loc_core   -B build/core   -DPRISM_LOC_CORE_BUILD_TESTS=ON
cmake -S prism_loc_fusion -B build/fusion -DPRISM_LOC_FUSION_BUILD_TESTS=ON
cmake --build build/core   -j && ( cd build/core   && ctest --output-on-failure )
cmake --build build/fusion -j && ( cd build/fusion && ctest --output-on-failure )

# 2. Full ROS 2 Humble build (workspace):
#    place this repo at <ws>/src/prism_loc, then:
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
#    For ndt3d or fusion3d on a robot add  --cmake-args -DCMAKE_BUILD_TYPE=Release :
#    built without optimization (-O0), one ndt3d filter update or one fusion3d NDT
#    registration took over a second here, and the node could not follow its inputs.
colcon test --packages-select prism_loc_core prism_loc prism_loc_fusion prism_loc_fusion_ros

# 3. Run — 2D LiDAR (MCL):
ros2 launch prism_loc laser2d.launch.py map:=/path/to/map.yaml
#    3D LiDAR (NDT-MCL):
ros2 launch prism_loc ndt3d.launch.py map_pcd_path:=/path/to/map.pcd
#    3D LiDAR + IMU + RTK-GNSS (ESKF fusion):
ros2 launch prism_loc_fusion_ros fusion3d.launch.py map_pcd_path:=/path/to/map.pcd
#    laser2d/ndt3d: click "2D Pose Estimate" in RViz to seed; fusion3d
#    auto-initializes once it has IMU attitude + a valid RTK fix
#    (or use /initialpose).
```

All three launch files accept `use_sim_time` (default `false`). Leave it
`false` against a real robot; set it `true` only when replaying a bag /
running in simulation, where pose/TF are stamped from a `/clock` topic
instead of the wall clock:
```bash
ros2 launch prism_loc laser2d.launch.py map:=/path/to/map.yaml use_sim_time:=true
```

### Verify it's working

After launching, check the node came up and is actually publishing pose/TF:
```bash
ros2 topic echo /prism_loc/pose --once
ros2 run tf2_ros tf2_echo map odom
```
(`fusion3d` runs as node `prism_loc_fusion`, so its pose topic is
`/prism_loc_fusion/pose` instead.)

Expected startup log lines (`RCLCPP_INFO`, from
[`prism_loc/src/localization_node.cpp`](prism_loc/src/localization_node.cpp) and
[`prism_loc_fusion_ros/src/fusion_localization_node.cpp`](prism_loc_fusion_ros/src/fusion_localization_node.cpp)):
- laser2d: `"prism_loc up: backend=laser2d"` then, once the map arrives,
  `"laser2d: map received (%dx%d)"` (plus
  `"laser2d: BBS global-localization matcher ready"` if
  `try_global_localization:=true`)
- ndt3d: `"ndt3d: NDT map built (%zu voxels)"` then `"prism_loc up: backend=ndt3d"`
- fusion3d: `"fusion3d: NDT map loaded (%zu pts)"` then
  `"prism_loc_fusion (fusion3d) up"`, then, once IMU+position priors land,
  `"fusion3d: initialized at (%.2f, %.2f, %.2f)"`

If any of these don't show up, or the pose/TF commands above hang or print
nothing, see [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

### Runtime status (`/diagnostics`)

Both nodes publish a `diagnostic_msgs/DiagnosticArray` on `/diagnostics` once per
second from a wall-clock timer, so the status keeps coming when inputs stop or the
ROS clock is stuck. Status names are `prism_loc: localization` and
`prism_loc_fusion: fusion` (hardware_id = node namespace); watch them with
`ros2 topic echo /diagnostics` or `rqt_robot_monitor`.

| Level | `prism_loc` (laser2d / ndt3d) | `prism_loc_fusion` (fusion3d) |
|---|---|---|
| ERROR | no map, no scans/clouds, or no pose seed after `startup_timeout_s` (30 s); scans/clouds stopped for `max(input_timeout_s, input_timeout_periods x observed period)` | no IMU or no initialization after `startup_timeout_s`; IMU stopped; IMU not used after `startup_timeout_s` because its mount is unknown (empty `frame_id`, or no transform `base_link -> <imu frame>`) |
| WARN | odom or sensor TF lookups failing; relocalization pending, ambiguous or without candidate; effective particle count below `min_neff_fraction` (laser2d; off by default for ndt3d); ndt3d: scan-to-map match below `min_match_fraction`; zero-stamped scans dropped; `use_sim_time` on while the ROS clock does not advance | no accepted NDT correction for `correction_timeout_s`; `odom->base_link` lost after being seen, or appearing while the map->base_link fallback was broadcast (two parents); fallback off and no odometry (no TF); IMU gaps; ROS clock not advancing under `use_sim_time`; point clouds or GNSS fixes not used because their mount is unknown (the IMU likewise, within `startup_timeout_s`) |

Values carried for monitors: `n_eff`, `particles`, `covariance_trace_xy`,
`covariance_yaw`, `input_rate_hz`, `seconds_since_last_input`,
`seconds_since_last_update`, `relocalization`, `relocalization_scans_used`,
`relocalization_scans_skipped` (laser2d/ndt3d), `match_fraction` (ndt3d only); `imu_rate_hz`,
`imu_gaps_total`, `seconds_since_ndt_correction`, `seconds_since_gnss_correction`,
`ndt_rejected_total`, `covariance_trace_position`, `tf_child_frame`,
`map_to_base_fallback_active`, `points_mount`, `imu_mount`, `gnss_mount` (fusion3d; each
mount reads `identity (frame base_link)`, `base_link -> lidar xyz ... rpy ...`,
`missing (frame lidar)`, `missing (empty frame_id)` or `no message yet`). A fusion3d robot without odometry that
runs on the `map→base_link` fallback by design reports OK with
`map_to_base_fallback_active=true`; it WARNs only when `odom→base_link` was seen and
then lost, or when odometry appears while the fallback was being broadcast (two
parents for `base_link`).

**ndt3d status.** The effective particle count is not a health signal for `ndt3d`: with
the NDT likelihood one particle takes nearly all the weight at every update of a healthy
run, and when no scan point falls in a map cell all particles weigh the same. `ndt3d`
therefore reports `match_fraction`: the share of the scored scan points that lie within
3 standard deviations of the NDT cell they fall in, at the published pose, averaged over
the last 5 filter updates (it is not judged on the update forced by `/initialpose`). The
status WARNs `low scan-to-map match` while it is below `min_match_fraction` (0.65).
Measured with the `ndt3d` model and the shipped parameters on the 3D synthetic world
that `scripts/synthetic_e2e.py` uses (75 s runs made outside CI; statuses after t = 10 s):

| Run | `match_fraction` | n_eff / particles | Status |
|---|---|---|---|
| healthy, 4 seeds (largest pose error after 10 s 0.09 – 0.43 m, after 30 s ≤ 0.17 m) | 0.747 – 0.883 | 0.002 – 0.004 | OK in 260 of 260 |
| healthy, prior map with 2 cm of noise | 0.972 – 0.998 | 0.002 – 0.004 | OK in 65 of 65 |
| kidnapped (2 runs; pose error 2 – 8 m), from 5 s after the kidnap | 0.331 – 0.626 | 0.002 – 0.007 | WARN in 70 of 70; first WARN at most 2.0 s after the kidnap; OK in 50 of 50 before it |
| initial pose 3.6 m and 1 rad off | 0.229 – 0.561 | 0.002 – 0.005 | WARN in 65 of 65 |
| pose off the map (no overlap) | 0.000 | 1.000 | WARN in 65 of 65 |
| `base_height` 0.5 m too low (pose error still ≤ 0.24 m) | 0.345 – 0.443 | 0.002 – 0.002 | WARN in 65 of 65 |

Limits: these are 11 runs of one synthetic world; a pose error under about half
an NDT cell is not detected (an initial pose 1.3 m and 0.35 rad off stayed more than
0.3 m off for 15 s, up to 0.47 m after t = 10 s, with `match_fraction`
0.763 – 0.851 and status OK); the value is that of
the last filter update, so it does not change while the robot stands still; and objects
that are not in the map lower the healthy value, so read `match_fraction` on a known-good
run at your site and set `min_match_fraction` below it.

Status names follow the `diagnostic_updater` convention: `<node name>: ...` with the
namespace as `hardware_id`. In a multi-robot setup either namespace or remap
`/diagnostics` per robot or give each node a unique name, or the statuses of
different robots carry the same name.

## 📊 Evaluation (synthetic)

The paper ships a **fully reproducible synthetic evaluation**: three
closed-loop experiments compile the *actual* estimator cores directly with
`g++` (no ROS, no CMake), drive them through end-to-end localization
scenarios with fixed seeds, and write the CSVs committed under
[`docs/paper/data/`](docs/paper/data/). Every number and figure below is
recomputable from those drivers ([`experiments/`](experiments/)); a committed
guard script ([`docs/paper/check_numbers.py`](docs/paper/check_numbers.py))
asserts the quoted statistics still trace to the CSVs. This is a correctness
and behavior demonstration on synthetic data — **not** a field-accuracy
benchmark; evaluation on public datasets is on the [roadmap](#-roadmap).

| | |
|:---:|:---:|
| <img src="docs/assets/eval_bbs.png" alt="BBS global relocalization: position error vs score fraction, 100 kidnapped-robot queries"/> | <img src="docs/assets/eval_eskf.png" alt="ESKF fusion: position error and 3-sigma bound across a 10 s aiding dropout"/> |
| **Global relocalization (BBS)** — 100 kidnapped-robot queries: **97 % success**, median error **0.037 m / 0.48°**, zero false accepts at the score threshold, median full-map latency 37.65 ms. | **ESKF under aiding dropout** — position RMSE **0.016 m → 0.125 m → 0.009 m** before/during/after a 10 s IMU-only blackout; the filter's 3σ bound stays above the true error throughout. |

<div align="center">
<img src="docs/assets/eval_mcl.png" width="470" alt="MCL tracking: position and yaw error over a 65 s closed loop, three seeds"/>

**MCL tracking** — 65 s closed loop, three seeds: steady-state mean position
error **0.105 m**, mean yaw error **1.8°**; KLD resampling shrinks the
particle set 2000 → 500 (4× fewer likelihood evaluations) after convergence.
</div>

## 🗺️ Building a map

**laser2d** consumes a standard Nav2 2D map (`map.yaml` + `.pgm`). Build one
with [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) while
driving/teleoperating the robot through the space, then save it with
`nav2_map_server`'s saver once the map looks complete:
```bash
ros2 run nav2_map_server map_saver_cli -f /path/to/map
#   -> writes /path/to/map.yaml + /path/to/map.pgm; point laser2d.launch.py's
#      map:= argument at the .yaml.
```

**ndt3d** / **fusion3d** consume a single prior `.pcd` point cloud
(`map_pcd_path`). `prism_loc` itself is localization-only — it does not ship a
mapping node — so build the map with a separate 3D SLAM/registration pipeline,
then hand the result to `prism_loc`:
1. Drive the robot through the space once, recording the raw cloud (and TF)
   to a bag: `ros2 bag record -o mapping_bag /points /tf /tf_static`.
2. Post-process the bag with a 3D SLAM/scan-registration pipeline (e.g.
   a LiDAR-inertial odometry package or PCL registration) to produce
   one globally-consistent point cloud.
3. Save that merged, downsampled cloud as a single named `.pcd` file (e.g.
   `pcl::io::savePCDFileBinary("/path/to/map.pcd", merged_cloud)`).
4. Point `map_pcd_path:=/path/to/map.pcd` at it for `ndt3d.launch.py` /
   `fusion3d.launch.py`.

### Global localization: counting only scans taken after the robot moved

Verification (`bbs_verify_scans`, default 9) multiplies the evidence of every
scan it counts. The shipped values were selected in
[`experiments/bbs_verify.cpp`](experiments/bbs_verify.cpp) with **0.3 m** of travel
between scans, but by default the node counts every incoming scan. A stationary
robot then feeds near-identical scans: no new information arrives, yet the
posterior of whichever hypothesis the first scan slightly preferred grows until it
is committed (the unit test
`RelocVerifierMotionGate.StationaryRepeatsCommitToTheWrongRoomWithoutGate`
reproduces a commit to the wrong one of two aliased rooms). Setting

```yaml
bbs_verify_min_translation: 0.3   # m, the step the defaults were selected with
bbs_verify_min_rotation: 0.3      # rad, so in-place turns also count
```

makes a scan count only after that much odometry motion since the last counted
one; the verifier then stays pending while the robot stands still and decides on
the move. Meanwhile `/diagnostics` WARNs `"relocalization pending: waiting for motion
(N scans skipped)"` and reports `relocalization_scans_used` /
`relocalization_scans_skipped`. The code default is `0.0` and the shipped YAML does
not set these keys (every scan counts, the v0.1 behaviour) until this is adopted as
the default.

## 🔌 Interface

| Backend | Package | Input | Output |
|---|---|---|---|
| **laser2d** | `prism_loc` | `/scan`, `/map`, `/initialpose`, TF `odom→base` | `/tf` `map→odom`, `~/pose`, `~/particle_cloud` |
| **ndt3d** | `prism_loc` | `/points`, `map.pcd`, `/initialpose`, TF `odom→base` | same |
| **fusion3d** | `prism_loc_fusion_ros` | `/points`, `/imu`, `/gnss` (NavSatFix), `map.pcd`, `/initialpose`, TF `base→sensor` for every input not in `base_link` | `/tf` `map→odom`, `~/pose`, `~/odometry` |

All backends also publish `/diagnostics` (see [Runtime status](#runtime-status-diagnostics)).
`fusion3d` broadcasts `map→base_link` instead of `map→odom` while no
`odom→base_link` transform exists (`map_to_base_fallback`, default `true`). If wheel
odometry or an EKF publishes `odom→base_link` and may start after `fusion3d`, set
`map_to_base_fallback: false`; otherwise `base_link` briefly has two parents in the
TF tree. `~/odometry` (`nav_msgs/Odometry`, `frame_id` = `map`, `child_frame_id` =
`base_link`) carries its linear twist in `base_link`, as the message definition
specifies; the angular twist is not filled.

### Sensor frames (`fusion3d`)

The pose `fusion3d` estimates and publishes is that of `base_frame` (`base_link`). Each
input is taken to be measured in the frame its `header.frame_id` names; `points_frame`,
`imu_frame` and `gnss_frame` replace that name when set (for a driver that leaves
`frame_id` empty or wrong).

| Input | Frame is `base_frame` | Any other frame | What is applied |
|---|---|---|---|
| `/points` | used as it is, no TF needed | transform `base_frame → frame` must be on TF | all 6 DoF: registration starts from and returns the pose of the lidar, `T_map_base = T_map_lidar · T_base_lidar⁻¹` |
| `/imu` | used as it is, no TF needed | transform must be on TF | rotation only: angular velocity and specific force are rotated into `base_frame` |
| `/gnss` | used as it is, no TF needed | transform must be on TF | translation only (the lever arm `t`): the fix is fused as the position of the antenna, `p + R·t`; the attitude uncertainty enters its innovation covariance through `∂/∂θ = −R[t]×` |

- **Not compensated.** The IMU's lever arm: the specific force is the one at the IMU, which
  differs from the one at the `base_frame` origin by `α × r + ω × (ω × r)` (0.3 m/s² for
  r = 0.3 m at 1 rad/s); put `base_frame` near the IMU if that matters. The lidar's
  measurement noise (`pose_pos_std`) is used unchanged for the pose of `base_frame`: with
  a lever arm `t`, an error `δ` in the registered attitude adds about `|t|·δ` of position
  error that the filter does not model.
- **GNSS antenna off `base_frame`.** A fix corrects position, velocity and accelerometer
  bias. It does not correct attitude or gyro bias (they are "consider" states of this
  update: their uncertainty limits how far the fix is trusted, through the lever arm). A
  single antenna does not observe the heading, and correcting attitude from it made the
  heading drift and, right after a reseed, tilted the filter. Attitude is left to the IMU
  and the lidar. What follows: the position of `base_frame` is known to about the
  horizontal lever arm times the heading error, and the reported covariance says so.
  Measured on the node with GNSS and IMU only (noise 0.02 m/s², 0.002 rad/s, 0.02 m; 30 s;
  antennas at (0.5, 0, 1.0) and (−0.3, 0.2, 1.0); standing and turning; 16 runs, heading
  exact at the start): height error ≤ 0.02 m, tilt ≤ 0.003 rad, heading ≤ 0.002 rad,
  horizontal error ≤ 0.03 m. In the pairing runs with a 0.1 rad heading error the
  horizontal error was 0.08 – 0.09 m for a 0.5 m lever arm. Seeding from the first fix
  uses the attitude the filter starts with.
- **Missing mount.** With an empty frame name or no transform, that input is not used
  (nothing is assumed): the log says so every 5 s and `/diagnostics` reports
  `point clouds not used: ...`, `GNSS not used: ...` (WARN) or `IMU not used: ...` (WARN
  within `startup_timeout_s`, ERROR after: without the IMU there is no pose). The status
  clears with the first message after the transform appears.
- **Lookup.** The latest available transform is used (a mount is rigid; nothing is waited
  for), read again once per second, and kept if TF later loses it. A sensor on a moving
  joint is not supported.
- **Unchanged case.** Data already in `base_frame` (the frame name equals `base_frame`, or
  the transform on TF is exactly the identity) goes through no arithmetic at all; the
  values are bit-identical to those of earlier versions
  (`prism_loc_fusion_ros/test/test_extrinsic.cpp`).
- **Changed case.** Earlier versions ignored `frame_id` and used every input as if it were
  measured at `base_frame`. A robot whose frames differ from `base_frame` and that relied
  on that gets the old behaviour with `points_frame`, `imu_frame`, `gnss_frame` set to
  the value of `base_frame`.

## 📚 Documentation

| Document | Contents |
|---|---|
| [`docs/paper/main.pdf`](docs/paper/main.pdf) | Systems paper: architecture, estimator cores, interface contract, synthetic evaluation |
| [`PARAMS.md`](PARAMS.md) | Every `laser2d`/`ndt3d`/`fusion3d` parameter, its default, and its meaning |
| [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) | Symptom → cause → fix for the common failure modes, with the `/diagnostics` message each one produces |
| [`SPEC.md`](SPEC.md) / [`SPEC_fusion.md`](SPEC_fusion.md) | Full design specifications |
| [`experiments/README.md`](experiments/README.md) | How to rebuild and rerun the synthetic evaluation |

## 🧭 Roadmap

`v0.1` ships 2D likelihood-field MCL and 3D NDT-MCL (planar x, y, yaw), plus a
3D LiDAR + IMU + RTK-GNSS error-state Kalman fusion (`fusion3d`) with full
6-DoF state and IMU-bias estimation. The `laser2d` backend self-initializes
via branch-and-bound (BBS) global localization with no `/initialpose` (param
`try_global_localization`, on-demand service `~/global_localization`): the
top-K distinct BBS modes of the first scan are verified over several
odometry-chained scans, and the pose is committed only when one hypothesis
dominates the posterior; otherwise the node reports the place as ambiguous
instead of re-seeding (`bbs_verify_*` params). Planned:

- [ ] Quantitative field evaluation (ATE/RPE) on public datasets
- [ ] 3D global localization for `ndt3d` / `fusion3d`
- [ ] Tight LiDAR–IMU time-offset estimation
- [ ] Sample map + bag pair for a known-good first run

## 🙏 Acknowledgements

`prism_loc` claims no new estimator theory — it stands on well-known work and
conventions: **MCL / AMCL** (Dellaert, Fox, Thrun et al.; `nav2_amcl`),
**NDT & NDT-MCL** (Biber & Straßer; Magnusson; Saarinen et al.),
**branch-and-bound scan matching** (Olson; Hess et al., Cartographer),
**error-state Kalman filtering** (Solà), and the ROS interface patterns of
[`robot_localization`](https://github.com/cra-ros-pkg/robot_localization) and
[`hdl_localization`](https://github.com/koide3/hdl_localization). See the
paper's Related Work section for the full positioning.

## 💛 Sponsor

If `prism_loc` saves you time, consider
[sponsoring](https://github.com/sponsors/kjungmo). Sponsorship funds
maintenance, new features, and faster issue response. Backers will be
acknowledged here — thank you.

## License

Apache-2.0.
