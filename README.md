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
stuck ROS clock. This is a ROS-path check on synthetic data, not a field result.
[`scripts/fusion_extrinsic_check.py`](scripts/fusion_extrinsic_check.py) feeds `fusion3d`
a robot that stands, or turns on the spot, with its lidar, IMU and GNSS antenna mounted
off `base_link`, and asserts the pose of `base_link` within 0.05 m and 0.02 rad, the
`/diagnostics` status when a mount is missing from TF, and, on GNSS and IMU alone with
sensor noise, height, tilt and heading within 0.15 m, 0.03 rad and 0.05 rad and a
horizontal error within twice the reported sigma; with a constant gyro bias (45 s), across a
reseed (also with the body on a slope), from a start before the antenna mount is known, over a
start and a stop, through a gap in the fixes, and from a boot with `initial_yaw` 2 rad off (also with the fixes at 1 Hz and 10 Hz, and with fixes too slow to align), height and tilt within the same
bounds, heading within 0.05 rad plus the vertical bias times the time, and a horizontal error
within 3 sigma, and the heading values and the WARN on `/diagnostics` (see
[Sensor frames](#sensor-frames-fusion3d)). CI runs it on an optimized build
of the two fusion packages; the whole script takes about 22 minutes. `--long` adds 600 s of
standing (`gnss_stand_long`, 10 more minutes, not in CI), the same followed by a start and a stop (`gnss_stand_go`, `_b`) and the outage across a start (`gnss_gap_across_start`, `_b`, 8 minutes each).
GoogleTest cases: 45 in `prism_loc_core`, 55 in `prism_loc_fusion`, 18 in
`prism_loc`, 24 in `prism_loc_fusion_ros` (142 in total).

Run the same checks locally from a built workspace (`rosdep install` as below,
plus `sudo apt-get install python3-numpy python3-yaml`):
```bash
source install/setup.bash
bash src/prism_loc/scripts/launch_checks.sh laser2d ndt3d fusion3d
python3 src/prism_loc/scripts/synthetic_e2e.py install/prism_loc/share/prism_loc/params/laser2d.yaml
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
| WARN | odom or sensor TF lookups failing; relocalization pending, ambiguous or without candidate; effective particle count below `min_neff_fraction`; zero-stamped scans dropped; `use_sim_time` on while the ROS clock does not advance | no accepted NDT correction for `correction_timeout_s`; `odom->base_link` lost after being seen, or appearing while the map->base_link fallback was broadcast (two parents); fallback off and no odometry (no TF); IMU gaps; ROS clock not advancing under `use_sim_time`; point clouds or GNSS fixes not used because their mount is unknown (the IMU likewise, within `startup_timeout_s`) |

Values carried for monitors: `n_eff`, `particles`, `covariance_trace_xy`,
`covariance_yaw`, `input_rate_hz`, `seconds_since_last_input`,
`seconds_since_last_update`, `relocalization`, `relocalization_scans_used`,
`relocalization_scans_skipped` (laser2d/ndt3d); `imu_rate_hz`,
`imu_gaps_total`, `seconds_since_ndt_correction`, `seconds_since_gnss_correction`,
`ndt_rejected_total`, `covariance_trace_position`, `tf_child_frame`,
`map_to_base_fallback_active`, `points_mount`, `imu_mount`, `gnss_mount` (fusion3d; each
mount reads `identity (frame base_link)`, `base_link -> lidar xyz ... rpy ...`,
`missing (frame lidar)`, `missing (empty frame_id)` or `no message yet`). A fusion3d robot without odometry that
runs on the `map→base_link` fallback by design reports OK with
`map_to_base_fallback_active=true`; it WARNs only when `odom→base_link` was seen and
then lost, or when odometry appears while the fallback was being broadcast (two
parents for `base_link`).

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
- **GNSS antenna off `base_frame`.** The fix is fused as the position of the antenna,
  `p + R·t`, with the full Jacobian. Position, velocity, accelerometer bias, roll, pitch and the
  two horizontal gyro biases are always corrected. With a lever arm the roll/pitch variance at
  start and at every reseed is the accelerometer's (about 1e-4 rad²), not the 0.5 rad² of the
  heading, and a start from `/initialpose` takes roll and pitch from the accelerometer at the
  heading of the message (RViz sends a level attitude; on a slope that is wrong by the slope).
  With the antenna at `base_frame` nothing changes. **Supported GNSS fix rate: 1 to 20 Hz** for the heading
  alignment of a start or a stop (points 6 and 8 below; measured at 1, 2, 5, 10 and 20 Hz, "Sixth pass"). Under about 0.8 Hz the
  filter says so on `/diagnostics` and leaves the start heading as a guess.

  **What one antenna and an IMU can observe.** A zero lever arm needs no table: the position is
  the position of `base_link`. With a lever arm `t` (‖t_xy‖ is the length of its horizontal part):

  | The robot | position of `base_link` | roll, pitch | heading | vertical gyro bias |
  |---|---|---|---|---|
  | stands, creeps, or drives straight at constant speed | only to a circle of radius ‖t_xy‖ around the antenna; the point on it is the heading | yes | **no** | no |
  | accelerates, brakes, or turns at speed | yes | yes | yes, through the IMU (a horizontal acceleration resolved with a wrong heading mispredicts the velocity) | with the heading |
  | spins on the spot | yes, through the sweep of the antenna | yes | weakly | weakly |

  The filter does what the table says:
  1. **Consider states.** The heading and the vertical gyro bias are counted in the innovation
     covariance but not corrected (a plain update there trades the heading for the position of
     the body and its covariance shrinks along a direction that was never observed).
  2. **Bound.** The heading standard deviation that an update sees is at most 0.35 rad
     (`EskfParams::lever_heading_sigma_max`). Without it the vertical gyro bias is never
     corrected while the robot stands, the heading variance grows as (bias sigma · t)² without
     limit, the lever term `−R[t]×` leaves its linear range, and after minutes the tilt and the
     height diverge (the previous round's `S6vb`: 192 of 192 runs of 900 s and 1,800 s standing).
     The bound scales the heading error variable, `P ← D P Dᵀ` with `D = I + (c − 1) u uᵀ`
     on the attitude block (`u` is the vertical in the body frame, `c` = bound / sigma): the heading
     variance by `c²`, every heading cross-covariance by `c`, the tilt not at all, `P` stays positive.
  3. **Release.** While the IMU has measured horizontal acceleration the update is the full one,
     heading and vertical gyro bias included: the filter keeps a decaying integral of the
     horizontal specific force in the world frame, `dv ← dv·e^(−dt/4 s) + |a_xy|·dt`, and
     releases above `dv` = 1.0 m/s (`lever_release_dv`; a start at 1 m/s² passes it after 1.15 s).
     A straight drive at constant speed or a spin has `dv` = 0 and stays closed; the speed estimate
     is not used (the previous version's gate on 0.6 m/s is off, `lever_motion_speed` = 0).
     While the release is open the bound is not applied (the version before applied it; points 5 to 7 are what replaces it).
  4. **Reseed.** `/initialpose` with a lever arm: roll and pitch from the accelerometer at the
     heading of the message (a message that comes before the first IMU sample waits for it).
  5. **The spread the bound hides.** The bound keeps the update in the linear range of `−R[t]×`, but the true heading
     keeps turning with the vertical gyro bias while the robot stands (1.05 rad after 600 s at 0.1 deg/s). The filter
     therefore also carries the unbounded spread of the heading and of the vertical gyro bias (a 2 × 2 covariance, `headingSigmaHonest()`;
     it is 6 rad after 600 s with the shipped 0.01 rad/s prior). It is reported as `heading_sigma_rad` and published in the pose
     covariance (heading variance, and the position variance of the circle of the lever), while `heading_sigma_used_rad`
     is the bounded one the update uses. The version before released the heading at the first start with the bounded
     0.35 rad: a full update on a heading that was 3 sigma off turned it the wrong way and tilted the filter (fourth review).
  6. **Start after a stand: alignment.** While the heading is held back and its spread is over 1 rad, a lever update does not
     linearise at that spread. Over the last 3 s the antenna fixes give the change of velocity of the antenna (it does not depend on the heading);
     the IMU, resolved with the filter's heading, gives the same change rotated by the heading error. When both are over
     0.25 m/s, the filter rotates its heading by that angle (any error, up to ±π), about the antenna, sets the horizontal velocity to the antenna's,
     and gives the heading 0.35 rad of spread (`heading_source: motion`). A start or a stop does it; a constant speed, a creep and a turn on the spot do not.
     The velocity of the antenna is taken over a span of seconds, not of fixes (`lever_align_span`: 0.8 s and at least 2 fixes). The window, the longest interval that is not a gap and the history that is checked scale with the measured fix period
     (`gnss_fix_period_s`, the median of the last 15 intervals): from 5 Hz up they are 3 s, 1 s and 4 s, as before; at 1 Hz a span of 2 s, a window of 3 s and a gap of 2.5 s. **Supported fix rate: 1 to 20 Hz** (sixth pass below).
     (A rotation keeps the norm, so the two changes could also be required to have the same norm, `lever_align_norm`; the sixth review proposed 30 %. It is off: it removed no alignment that the spans did not, and it made one node scenario worse, see the sixth pass.)
     Under about 0.8 Hz (a median interval over 1.25 s, `lever_align_period_max`) the alignment does nothing, the start is not marked as unknown (point 8), and `/diagnostics` WARNs (`heading_align_rate: unsupported`).
     A gap in the fixes (an interval over 1 s, or over 2.5 fix periods if that is more) anywhere from the first fix of the older velocity to now, a window of more than 4 s between the two velocities (the window plus 1 s), or a body rate over `0.125 m/s / |t_xy|` (a faster turn moves the antenna without the body), stops it.
     (The round before looked for a gap only in the last 4 s. After an outage across a start it compared the IMU sum of the whole outage with the velocity from before the outage and set the heading from that.
     The round before that took the velocity over 4 fixes: 4 s at 1 Hz, where the alignment never worked and a start with an unknown heading ran blind for good, and 0.4 s at 10 Hz, where a check of the span against the rounding of the clock decided
     whether it worked, and the noise of the fixes gave alignments at constant speed.)
  7. **Blind.** While the heading is held back, over 1 rad of spread, and not alignable (standing, turning on the spot), the update
     leaves roll and pitch out of the lever term (a filter without a lever arm does the same) and counts the circle of the lever
     as noise on the two horizontal axes: `lever_blind: true`. The antenna then says where the circle is and tells roll and pitch nothing.
     The release stays closed.
  8. **Start with a guessed heading.** With a lever arm and no heading source (no `/initialpose`, no lidar correction yet, `initial_yaw_known` false) the heading is the `initial_yaw` parameter, a guess. The filter then holds the spread of an unknown
     heading, π/√3 = 1.81 rad (`EskfParams::lever_start_sigma`, `Eskf::markHeadingUnknown()`), in the shadow of point 5. That is over the 1 rad of point 6, so the first start or stop of 0.25 m/s or more finds the heading, and until then the update is blind (point 7).
     The round before started with the 0.71 rad of the filter's own covariance: nothing aligned, and the first start ran a full update with a heading 1 to 3 rad off. `/initialpose`, the lidar and `initial_yaw_known: true` (a heading right to about 0.3 rad) start as the round before did.
     The mark needs the alignment, and the alignment needs fixes at 1 Hz or faster (point 6): if the median interval of the first fixes is over 1.25 s, the filter takes the mark back (`Eskf::leverUnsupportedRate()`) and starts with its own spread, as the round before did.

  9. **An outage with the heading unknown.** While the heading is unknown and the fixes stop, `predict` dead-reckons the acceleration in the filter's heading. If that is off by `e`, the velocity it gains is the sum `dS` of the IMU turned by `e`, and the position it gains
     is `D` (the integral of `dS`) turned by `e`: the error is `e |dS|` and `e |D|`, and the bound of point 2, which holds the heading at 0.35 rad in `P`, left both out. The first fixes after the outage then explained the velocity error as a tilt and an accelerometer bias.
     At the first lever update after an outage (`lever_outage_spread`, a tenth), if the alignment finds nothing, the horizontal position and velocity block of `P` gets `0.1 · min(σ², 2)/2 · [[|D|², D·dS], [D·dS, |dS|²]]` on each axis (σ: the heading spread of point 5), so that the fixes that follow correct the velocity within a few updates.
     Nothing changes when the heading is known, when there was no outage, or when the alignment succeeded at that update.

  **What you see on `/diagnostics`** (new values; none changes an existing one):

  | State | `heading_source` | `heading_sigma_rad` | `heading_capped` | `lever_release` | Status |
  |---|---|---|---|---|---|
  | filter not initialized | `none` | `n/a` | `false` | `n/a` | as before |
  | antenna at `base_frame` (no lever arm) | the last source | the filter's | `false` | `n/a` | as before, never the heading WARN |
  | lever arm, less than 30 s since the start, no `/initialpose`, `initial_yaw_known` false | `none` | 1.8 and growing; `heading_sigma_used_rad` 0.35 | `true` | `closed` | OK; `lever_blind` `true` |
  | lever arm, less than 30 s since the last `/initialpose`, or `initial_yaw_known` true, GNSS and IMU only | `initialpose` or `none` | 0.71 and growing; `heading_sigma_used_rad` 0.35 | `true` | `closed` | OK |
  | lever arm, 30 s or more with the bound active and no heading source | `none`, `initialpose` or `lidar` (the last one) | grows (the spread held, 6 rad after 600 s); `heading_sigma_used_rad` 0.35 | `true` | `closed` | **WARN** `heading not observed: GNSS and IMU only for N s, heading spread X rad, ...` |
  | start or stop of 0.25 m/s or more (the alignment) | `motion` | falls to 0.35 and below | `false` | `closed` or `open` | no heading WARN |
  | turn at speed (release open) | unchanged | falls below 0.35 | `false` | `open` | no heading WARN |
  | lidar corrections accepted | `lidar` | about 0.03 | `false` | `closed`, `open` while accelerating | OK |
  | fixes stop for more than 2 s | unchanged | unchanged | `false` | unchanged | the heading WARN clears; its 30 s start again with the first fix |
  | lever arm, the fixes come slower than 0.8 Hz (median interval over 1.25 s) | as without the alignment | the filter's own, not 1.8 | as before | as before | **WARN** `GNSS fixes come every N s (slower than 0.80 Hz) ...`; `heading_align_rate` `unsupported` |

  `gnss_fix_period_s` is the median interval of the last lever-arm fixes (`n/a` before the third), and `heading_align_rate` is `supported` or `unsupported` (`n/a` before the sixth fix and for a zero lever).
  `heading_source` is the last of `lidar` (an accepted NDT correction), `initialpose` and `motion` (the alignment of a start or a stop), `none`
  if none has happened (the `initial_yaw` parameter and the IMU alone); `lever_blind` is `true` while the update leaves roll and pitch out of the lever term (point 7 above); `gnss_only_s` is the
  seconds since that source (or since the start). `heading_capped` is true while every lever-arm
  update of the last stretch had to scale the heading down. The WARN is the filter saying what the
  table above says: the heading is not estimated. It does not say that the pose is wrong.

  **What is not observable, in numbers.** With GNSS and IMU only, on a robot that stands, creeps or
  drives straight at constant speed, the heading is not estimated. It drifts with the vertical
  gyro bias (0.1 rad per minute for each 0.1 deg/s). The filter uses 0.35 rad for the update, and reports
  the spread it holds (`heading_sigma_rad`) and publishes it in the pose covariance. The position of `base_link` can be wrong by up to `2·|t_xy|`
  (1.0 m for a 0.5 m horizontal arm) in either direction. Roll, pitch, height, velocity, the
  accelerometer bias and the horizontal gyro biases stay estimated and bounded. The heading comes
  back from the lidar, from `/initialpose`, or from driving with a start or a stop that changes the speed by 0.25 m/s or more
  (point 6; a turn at speed also opens the release). **A constant speed does not do it, and neither does a turn on the spot.** After minutes of standing the heading
  may be off by 1 rad and more under a small reported sigma of the update; read `heading_sigma_rad`, not `heading_sigma_used_rad`.
  Spinning for minutes on the spot with 0.5 deg/s and no heading source, a start with `initial_yaw_known: true` (or from `/initialpose`) whose heading is 1 rad or more off, and 120 s without fixes on a biased IMU are outside what
  was measured to work (see the counts below). With `initial_yaw_known: false`, the default, the first start finds a heading that is off by any angle (fifth pass below). That costs something when `initial_yaw` was right:
  a start within seconds of boot is aligned from its first 0.25 m/s of velocity change instead of trusting the heading, and the alignment adds its own error (second and third pass, `S8u` against `S8`). A 1.5 m mast with no horizontal arm gains little: on a boot with a wrong heading it ends at the level of the zero-lever filter (fifth pass), and in an outage across the first start the round before made it diverge, which the zero-lever filter does not (sixth pass: fixed; it still has more `tz` than the arms A and B).

  Measured on the real filter core in a synthetic experiment (study 6 of
  [`experiments/README.md`](experiments/README.md), `experiments/eskf_lever.cpp`; two passes, every
  run counted, diverged or not). A run **fails by kind** (counted apart): *tz*, the tilt exceeded
  0.05 rad or the height 0.5 m at a 1 s epoch from 10 s on; *circ*, the horizontal error left
  the circle of radius `|t_xy|` around the antenna by more than 0.3 m; *xy*, over 1 m;
  *heading*, over 0.5 rad. (A GNSS dropout and 5 s after it, and 5 s after a reseed, are not
  tested.) **Second pass**, GNSS and IMU only, antennas A (0.5, 0, 1.0), B (−0.3, 0.2, 1.0) and a
  1.5 m mast M (0, 0, 1.5), 4,200 runs per scheme of 300 s to 1,800 s (standing, driving a straight
  line, start and stop, turning with pauses, a planar lemniscate, spinning; gyro bias 0 to
  0.5 deg/s and accelerometer bias 0 to 0.3 m/s² per axis; dropouts of 30 s and 120 s; a reseed),
  and the zero-lever filter (1,400 runs of the same data) as the reference:

  | | previous round (speed gate at 0.6 m/s, `S6vb`) | this version (`S7a35`) | zero lever, same data |
  |---|---|---|---|
  | tz, all cells | 1,882 of 4,200 (44.8 %) | 880 of 4,200 (21.0 %) | 321 of 1,400 (22.9 %) |
  | of these, tz in the zero-lever run of the same data too | 857 | 782 | |
  | tz only with the lever arm | 1,025 | 98 | |
  | clean envelope: stand, line, start-stop, turn; gyro bias up to 0.2 deg/s; accelerometer bias up to 0.1 m/s²; no dropout (300 to 1,800 s): tz | 400 of 912 | **0 of 912** | 3 of 304 |
  | standing 900 s and 1,800 s, no dropout, every bias of the grid: tz | 192 of 192 | **0 of 192** | |
  | median of the largest horizontal error, clean envelope | 0.71 m | 0.14 m | 0.04 m |
  | heading error over 0.5 rad, clean envelope | 666 of 912 | 375 of 912 | 261 of 304 |

  Reading, with the counts above and in
  [`report.md`](experiments/results/eskf_lever/report.md):
  - The failure of the third review round (tilt, then height, diverge after minutes of standing)
    is gone for a robot that keeps standing: 0 of 192 standing runs of 900 s and 1,800 s, where the previous round had 192 of 192.
    It was moved, not removed: the first start after minutes of standing failed (third pass below); that failure is what points 5 to 7 above are for.
    The 880 runs that still have tz are in cells where the zero-lever filter fails too (an
    accelerometer bias of 0.3 m/s²: 574 of them; a GNSS dropout: 684; three have neither: two spins
    and one start-stop, all at 0.5 deg/s) and 782 of them fail in the zero-lever run of the
    same data. Of the 98 that do not, 88 have a GNSS dropout, 7 an accelerometer bias of 0.3 m/s²
    and 3 neither; all 98 are runs of 300 s.
  - The heading is not estimated while the robot stands, and this was not fixed: in the clean envelope
    the heading error is over 0.5 rad in 375 of 912 runs (zero lever: 261 of 304; the previous round:
    666 of 912). The table above names it.
  - Spinning on the spot at 0.5 deg/s for 300 s (no dropout, accelerometer bias up to 0.1 m/s²): tz in
    2 of 24 runs (the previous round 15 of 24, zero lever 0 of 8).
  - The position of `base_link` stays near the antenna: median largest error 0.14 m in all
    cells (0.30 m before), 156 of 4,200 runs over 1 m (894 before). Of these 124 are on the 0.5 m
    arm A and 48 are runs of 900 s or 1,800 s without a dropout, where a heading error of 1.5 rad
    or more puts `base_link` 1 m from where it is; 86 have a GNSS dropout (the zero-lever filter:
    7 of 1,400 runs over 1 m, 7 of its 816 with a dropout).
  - With the lidar (NDT fixes, 288 runs per scheme, all motions, a reseed, a dropout) tz is 0 for
    every scheme but the first version (9, 3.1 %).
  - A reseed on a slope (body tilted 0.05 and 0.1 rad, 144 runs per tilt): the previous round left a tilt
    error of more than 0.02 rad in 43 and 68 runs; this version in 0 and 0 (taking the level attitude of the
    message, the ablation `S7n`, 41 and 60).
  - Consistency (NEES at the last epoch, 95 % band): the position NEES of standing configurations is
    inside its band in 112 of 366 configurations (59 before); the tilt NEES is above its band (over-confident)
    in 684 of 1,740 configurations and below it in 303; the heading NEES above its band in 491.
    The heading and the horizontal covariance are optimistic once the true error is over the
    bound, by construction (above).
  - Choice of the bound and the release (second pass; the criterion is written in `eskf_lever.cpp` before
    the first run): bounds 0.35, 0.5 and 0.7 rad, release thresholds 0.5, 1.0 and 2.0 m/s. With a bound of
    0.7 rad the horizontal error leaves the circle in 928 of the 4,200 runs (0.5 rad: 238; 0.35 rad: 147).
    A threshold of 2.0 m/s opens too late for the simulated start-stop (a start at 0.5 m/s² for 3 s reaches 1.06 m/s)
    and recovers a heading that starts 0.5 rad off in 3 of 24 runs of that motion (1.0 m/s: 24 of 24; 0.5 m/s: 24 of 24). The declared
    order puts 0.5 m/s ahead of 1.0 m/s by 3 of 2,448 runs (451 against 454 runs with `tz` or `circ`), which two
    seeds per configuration cannot tell apart; I kept 1.0 m/s because of the next point, a deviation from the order that the
    report states. Nothing was tried below 0.35 rad.
  - **The release has a noise floor.** The integral of `|a_xy|` of white accelerometer noise `σ` per axis settles at about
    `1.25·σ·4 s` = 5σ. On a standing robot with `σ` = 0.05 m/s² the release stays closed at both thresholds; at 0.1 m/s² the
    threshold of 0.5 m/s is open in 65 % of the updates and 1.0 m/s in 0 %; at 0.2 m/s² 1.0 m/s is open in 56 % and
    at 0.4 m/s² in 99 % (suite `hvib`). An open release on a robot that does not accelerate did no harm in these runs
    (`tz` 0, `circ` 0 at 1.0 m/s) because the bound stays on, but the heading is then corrected from data that does not
    show it. A vibrating platform with more than about 0.15 m/s² of accelerometer noise per axis needs a larger
    `lever_release_dv` (a compile-time default; not a node parameter).
  - No bound, no release (`S1s`, consider states only): tz in 2,767 of 4,200 runs; the previous version in 1,882.

  **Third pass: standing for minutes, then moving** (`experiments/README.md`, "Third pass"; suites `hsg`, and `hsgv`
  on other data). Per suite: 1,152 runs with a lever arm per scheme (antennas A, B, M; 384 per motion) and 384 zero-lever runs on the same data (128 per motion).
  Each run stands 300 to 1,800 s at a gyro bias of 0 to 0.2 deg/s, then makes a start and a stop, a start and a circle, or a turn on the spot, for 160 s.
  The last column counts each zero-lever run once. (An earlier version of this table counted it once per lever-arm antenna, three times, under the heading "zero lever": 324, 366, 171 / 195 / 0.
  Paired with the lever-arm runs, as the bar compares them, the same runs are 324 and 366 of 1,152.)

  | | the version before the fifth round (`S8`) | this version (`S8u`) | zero lever, same data, each run once |
  |---|---|---|---|
  | hsg: runs with `tz` (tilt over 0.05 rad or height over 0.5 m) | 26 | **49** | 108 of 384 |
  | hsg: height over 1 m / tilt over 0.3 rad | 1 / 0 | **3 / 0** | 35 / 1 |
  | hsg: `tz` where the zero-lever run has none (of 1,152) | 12 | 24 | |
  | hsgv (other data): runs with `tz` | 21 | **52** | 122 of 384 |
  | hsgv: height over 1 m / tilt over 0.3 rad | 0 / 0 | **4 / 0** | 47 / 5 |
  | hsgv: `tz` where the zero-lever run has none | 8 | 22 | |
  | hsgv: start and stop, heading error over 0.5 rad at the end | 0 of 384 | **0 of 384** | 5 of 128 |
  | hsgv: `tz` in start-stop / circle / turn on the spot | 14 / 7 / 0 | **27 / 25 / 0** | 57 / 65 / 0 |

  The two versions before (`S7a35`: 375 `tz` in hsg, 381 in hsgv, 271 and 279 runs with the height over 1 m) are in [`report.md`](experiments/results/eskf_lever/report.md) and `experiments/README.md`.
  What `S8u` costs here: `tz` doubles (26 to 49, 21 to 52), and of the 24 runs with `tz` where the zero-lever run has none, 23 in hsg (22 of 22 in hsgv) are the 1.5 m mast M, whose vertical arm amplifies a tilt error of 0.05 rad.
  The bar (no more `tz` than the zero-lever runs of the same data in each motion; no run with the height over 1 m or the tilt over 0.3 rad in the start and the circle with antennas A and B) still holds.
  What is left: after a turn on the spot nothing tells the heading (it is over 0.5 rad off at the end in 266 of 384 runs of hsg, 258 of hsgv for `S8`, 269 and 268 for `S8u`; the zero-lever filter has 106 of 128 in hsg and 97 of 128 in hsgv)
  and the position of `base_link` is over 1 m from the truth in 33 (hsg) and 35 (hsgv) of the 384 turn runs for `S8` (the circle of the lever).
  All counts, per motion, stand and antenna: [`report.md`](experiments/results/eskf_lever/report.md), "Numbers quoted in README.md and experiments/README.md" under each suite.
  The second pass with `S8` and `S8u` added (`LEVER_ONLY=S8,S8u`, `merge_rows.py`; the rows of `S8` are bit for bit the ones of the round before wherever the data has no gap in the fixes: 21 of 4,320 rows of h300 changed, all in dropout cells):
  `S8` has 880 `tz` in 4,200 runs (`S7a35`: 880), 213 runs over 1 m (156); `S8u` has 962 `tz` and 248 runs over 1 m. The clean envelope has 0 `tz` for all three.
  Of the 65 runs over 1 m that `S8` has and `S7a35` has not, 59 are antenna A at 0.5 deg/s (1.00 to 1.29 m against 0.65 to 1.00 m: the circle of the lever, 1.0 m, with the heading 2.7 rad off).
  `S8u` has 37 runs over 1 m that `S8` has not (and 2 that only `S8` has; 10 of the 37 are antenna A at 0.5 deg/s, 1.00 to 1.90 m against 0.08 to 1.00 m under `S8`) and 82 more `tz`: 117 runs gain `tz` and 35 lose it. 104 of the 117 are drive and start-stop
  runs that begin moving within 6 s of the start (a start whose `initial_yaw` was right, 0.1 rad off, is aligned from a change of velocity, and the alignment adds its own error); 83 of the 117 have an accelerometer bias of 0.3 m/s²
  (the zero-lever run has `tz` in 24 of the 117).
  A reseed on a slope: 7 and 4 runs with `tz` of 144 at 0.05 and 0.1 rad for `S8u` (2 and 1 for `S8`). With the lidar: 0 `tz` for both.
  **`initial_yaw_known: true`, or a start from `/initialpose`, gives the `S8` numbers** for a robot whose heading at the start is right to about 0.3 rad.

  **Fifth pass: an outage across the start, and a boot with a guessed heading** (`experiments/README.md`, "Fifth pass"; suites `hgap` and `hboot`, written after the fifth review's probes and run once as committed;
  576 lever-arm runs and 192 zero-lever runs per scheme and suite; `S8o` is `S8` without the new check of the alignment, `S8` is the round before with it, `S8k` is the `S8u` of that round and the next one, `S8u` is this version, which differs only after the sixth review, below):

  | | `S8o` | `S8` | `S8k` | `S8u` | zero lever, same data |
  |---|---|---|---|---|---|
  | hgap, fixes out for 120 s from 10 s before the start: runs with `tz` (of 288) | 68 | **11** | 12 | 11 | 11 of 96 |
  | hgap, 120 s: height over 1 m / tilt over 0.3 rad | 36 / 7 | **2 / 0** | 4 / 1 | 3 / 0 | |
  | hgap, 120 s: antenna A / B / M with `tz` (of 96 each) | 26 / 24 / 18 | **3 / 0 / 8** | 4 / 0 / 8 | 2 / 0 / 9 | |
  | hgap, 30 s: runs with `tz` (of 288) | 19 | **12** | 13 | 12 | 18 of 96 |
  | hboot, `initial_yaw` 0.5 to 3 rad off, no `/initialpose`, stand 6 to 300 s: runs with `tz` (of 576) | 219 | 219 | **77** | **77** | 112 of 192 |
  | hboot: height over 1 m / tilt over 0.3 rad | 160 / 104 | 160 / 104 | **3 / 0** | **3 / 0** | |
  | hboot: heading error over 0.5 rad at the end | 162 | 162 | **11** | **11** | |
  | hboot: antenna A / B / M with `tz` (of 192 each) | 58 / 72 / 89 | 58 / 72 / 89 | **1 / 0 / 76** | **1 / 0 / 76** | |
  | hboot, `initial_yaw` 2 and 3 rad off, stand 6 to 60 s: runs with `tz` (of 144 lever-arm runs) | 144 | 144 | **34** | **34** | 45 of 48 |

  The first failure: after an outage across a start the alignment compared the IMU sum of the whole outage with the velocity from before it; the new check refuses a gap between the two spans. The second: a robot on GNSS and IMU with a lever arm, no `/initialpose`
  and an `initial_yaw` that is off by 1 rad or more, which starts within about 60 s, ran its first update with the heading it did not know; now the heading starts as unknown (point 8). What is left in hboot: the mast M (the lever has no horizontal part, so a wrong heading does not enter the lever term; it has `tz` in 76 of 192 runs against 112 of 192 for the zero-lever filter,
  with the largest height error 1.34 m and the largest tilt 0.13 rad), and 1 run of antenna A (tilt 0.051 rad). With `initial_yaw_known: true` and a wrong `initial_yaw`, the old failure is back (the `S8` column).
  On the real node (`scripts/fusion_extrinsic_check.py --scenario gnss_gap_across_start` and `gnss_boot_wrong_yaw`, each for antenna A and B): the outage across a start ends with a height error of 0.024 m and 0.065 m (the reviewer's run before the fix: +6.0 m, tilt 0.20 rad, heading 1.72 rad off 2 s after the fixes returned); the boot with `initial_yaw` 2 rad off ends the 80 s cycle with a height error of 0.091 m (antenna A) and 0.057 m (antenna B) and a tilt of 0.023 and 0.007 rad in the run of this round, and of 0.139 m and 0.163 m (tilt 0.010 and 0.012 rad) in the run of the round before
  (before the fix: +8.6 m and 0.49 rad, +25.9 m and 0.87 rad). One run does not fix the figure: antenna B ranged from 0.057 m to 0.696 m in height and from 0.006 to 0.047 rad in tilt over 7 runs (the 6 runs of the round before, 0.066 to 0.696 m, and this round's), the larger values under load.
  The check allows a height error of 1.0 m (it was 0.5 m until that range was seen; the failure it is for is 8 to 26 m) and a tilt of 0.1 rad.
  `heading_source` `motion` came 8.0 s after the start of the path (2 s after the first acceleration), the heading error after the cycle −0.063 and −0.121 rad (the check allows 0.5 rad).
  The same boot with the fixes at 1 Hz (`gnss_boot_wrong_yaw_1hz`): `motion` after 10.0 s, height error 0.041 m, tilt 0.008 rad, heading error +0.289 rad; at 10 Hz: 8.0 s, 0.061 m, 0.003 rad, +0.056 rad (the round before, at 1 Hz: `motion` never, +1.6 m, heading 2.2 rad off).
  With fixes every 2.5 s (`gnss_slow_rate`) the status WARNs `GNSS fixes come every 2.7 s (slower than 0.80 Hz) ...` after 40 s and `heading_align_rate` reads `unsupported`.

  On the real node (`gnss_stand_go` and `gnss_stand_go_b`; 600 s of standing with the review's gyro bias, then a start and a stop; antenna A and B in three runs: largest height error 0.062 and 0.091 m, then 0.099 and 0.038 m, then 0.066 and 0.140 m (the check allows 0.15 m); tilt 0.004 and 0.007 rad, then 0.004 and 0.008 rad, then 0.005 and 0.008 rad; heading error after the stop +0.03 and −0.04 rad, then 0.000 and −0.067 rad, then +0.011 and −0.085 rad; the reviewer's run gave +0.141 and +0.199 rad, so the heading after the stop varies by a few tenths of a radian from run to run and the check allows 0.25 rad; `heading_source: motion` 2.0 and 1.0 s after the start; the round before had +1.32 m and 0.15 rad (antenna A) and ±2.5 m and 0.2 rad (antenna B)).
  The default run of the script (34 scenarios, 241 checks, 22 minutes) and the long scenarios (`gnss_stand_long`, `gnss_stand_go`, `_b`, `gnss_gap_across_start`, `_b`) pass; the zero-lever poses are bit for bit those of `main` (lockstep feed of 1,284 poses, sha256 `dbe91c30...25f25`). The raw output of the node runs is in [`experiments/results/eskf_lever/node_checks.txt`](experiments/results/eskf_lever/node_checks.txt).

  **Sixth pass: the fix rate, and a short outage at the first start** (`experiments/README.md`, "Sixth pass"; suite `hrate` and the short-outage cells of `hgap`, written after the sixth review's probes and run once as committed;
  `S8k` is the `S8u` of the two rounds before, bit for bit, `S8u` is this version).

  *The fix rate.* The alignment took its two velocities over 4 fixes, so its span was 4 s at 1 Hz and 0.4 s at 10 Hz. At 1 Hz the span never passed its own check and a robot that booted with an unknown heading ran blind for good;
  at 10 Hz the check of 0.4 s against the rounding of the clock decided whether it worked, and the noise of the fixes gave alignments at constant speed. The span, the window and the gap limit are now seconds scaled by the measured fix period
  (point 6), and `/diagnostics` says when the rate is out of range (`heading_align_rate`). Antennas A and B (a horizontal arm), runs with `tz` / heading error over 0.5 rad at the end, the same IMU data at every rate, 4 seeds:

  | fix rate | boot (144 runs): `S8k` | boot: `S8u` | stand-then-go (128 runs): `S8k` | stand-then-go: `S8u` |
  |---|---|---|---|---|
  | 0.5 Hz (not supported) | 48 / 89 | 54 / 83 | 53 / 64 | 51 / 64 |
  | 1 Hz | 36 / 89 | **0 / 0** | 41 / 64 | **0 / 0** |
  | 2 Hz | 5 / 5 | **0 / 0** | 1 / 0 | 1 / 0 |
  | 5 Hz (the rate of every earlier suite) | 0 / 1 | 0 / 1 | 0 / 0 | 0 / 0 |
  | 10 Hz | 31 / 62 | **0 / 0** | 39 / 64 | **0 / 0** |
  | 20 Hz | 38 / 89 | **0 / 0** | 40 / 64 | **0 / 0** |

  *Boot*: no `/initialpose`, `initial_yaw` 0.1, 0.3 or 2 rad off, a stand of 6, 30 or 300 s, then the start-stop cycle or the circle, gyro and accelerometer bias 0.1; *stand-then-go*: 300 or 900 s of standing, gyro bias 0 or 0.2 deg/s, accelerometer bias 0 or 0.1, then the same motions.
  The mast M (no horizontal arm) is at the level of the zero-lever filter at every rate (boot, 72 runs: `tz` 24, 15, 12, 14, 16 at 1, 2, 5, 10, 20 Hz against 28, 24, 23, 29, 27 for the zero lever; `S8k`: 26, 21, 12, 21, 22).
  Under about 0.8 Hz the filter does nothing new: the start is not marked as unknown and the heading stays the guess, as in the round before. What the alignment would do there without the limit (`S8t`, spans in seconds only, 0.5 Hz): 32 and 17 runs with `tz`, 38 and 28 with the heading over 0.5 rad,
  which is better than 54 and 51 but not a rate to promise. **Supported: 1 to 20 Hz.**
  The test of the norms that the review proposed (`S8n`: only the test; `S8v`: `S8u` with it) changes 1 to 9 of the 544 rows of a rate from 1 to 20 Hz (0 at 0.5 Hz) and no count of the table: with the spans in seconds it removes no alignment that they do not. It is off (`lever_align_norm` 0, the unit test sets 0.3);
  it also made the node scenario `gnss_gap_across_start` worse (below).
  The node: `gnss_boot_wrong_yaw_1hz` and `_10hz` (new, in the default run) and `gnss_slow_rate`; the round before fails the 1 Hz scenario (`heading_source` `motion` never, +1.6 m, heading 2.2 rad off after the cycle), the new build passes all three.

  *A short outage at the first start.* The fixes stop 1 s before the first acceleration after a stand (or in the middle of it) for 1.5, 3, 5 or 12 s; 1,280 runs of antennas A and B, 640 of the mast M, 640 zero-lever runs on the same data (stand 300 or 900 s, gyro bias 0 or 0.2 deg/s):

  | | `S8k` | `S8w` (spans in seconds, no spread) | `S8u` | zero lever |
  |---|---|---|---|---|
  | antennas A and B: `tz` / over 1.5 m horizontal / height over 1 m (of 1,280) | 83 / 76 / 21 | 83 / 76 / 21 | **32 / 34 / 8** | |
  | mast M: `tz` / height over 1 m / diverged (of 640) | 154 / 59 / 16 | 154 / 59 / 16 | **132 / 16 / 0** | 226 / 84 / 0 |
  | mast M: largest height error | 2,310 m | 2,310 m | **2.7 m** | 10.2 m |
  | A and B, the outage ends before the circle starts or does not cover it (1,024): `tz` / over 1.5 m | 35 / 34 | | **2 / 2** | |

  The cause: with the heading unknown, `predict` dead-reckons the acceleration of the outage in a heading that is off by up to 3 rad, so the velocity and position it gains are rotated by that error. The bound of point 2 holds the heading at 0.35 rad in `P`, and with it exactly that term, so `P` said
  the velocity was good when it was off by up to twice the velocity change. The first fixes after the outage then explained the error as a tilt and an accelerometer bias: antenna A kept a horizontal error of 3 m for 20 s (a trace of configuration 756, seed 0 of `hgap`: a velocity error of 1.9 m/s at the return of the fixes, 20 s to correct), and the mast, which has no horizontal arm
  to absorb it, ended at a tilt of 3 rad and a height of 1,600 m (16 of 640 runs; `hgap` configuration 675, seed 0: stand 900 s, 0.2 deg/s, an outage of 1.5 s, height error 2,278 m). Point 9 above is the fix. A tenth of the variance, not the whole of it, is what ships: the whole made the node scenario `gnss_gap_across_start` fail in 4 of 300 seeds (1 of 300 at a tenth, 0 without the spread; `experiments/eskf_gap_across_start.cpp`).
  **This contradicts what this README said before** ("the mast fails like the zero-lever filter"): in an outage across the first start the mast of the round before diverged where the zero-lever filter did not. Now it does not diverge, and it has fewer runs with `tz` than the zero-lever filter (132 against 226 of 640) and fewer with the height over 1 m (16 against 84).
  What is left, and listed as limits: (1) the mast still has `tz` in 132 of 640 runs (the height error stays under 3 m); with the long outages of `hgap` (30 and 120 s) it has 17 of 192 against 15 for `S8k` and 29 for the zero lever. (2) Antenna A (0.5 m arm) on a circle of radius 6 m that begins during the outage: the alignment refuses a body rate over
  `0.125 m/s / |t_xy|`, and the circle turns at 0.25 rad/s, so with A it never finds the heading (32 of 128 runs over 1.5 m and 64 of 128 with the heading over 0.5 rad at the end; antenna B, arm 0.36 m: 0 over 1.5 m, 11 of 128 with the heading over 0.5 rad). (3) After an outage the first alignment can be wrong by a radian (the IMU sum is polluted by the tilt corrections that follow the outage, about 0.2 m/s over the window of 3 s);
  the spread of 0.35 rad that follows it is then far too narrow, and the next release (a stop) can turn the heading the wrong way. This is the mechanism of the failures of the replica (a trace with the test of the norms and the whole spread on: an alignment 4.8 s after the fixes returned left the heading 1.6 rad off, and the release at the stop that followed gave an accelerometer bias of 0.8 m/s²). Not done: an honest spread after an alignment from a weak change of velocity.

  **First pass** (15,104 runs with a lever arm, 60 s and 300 s; unchanged from the previous
  round, rerun with the second pass):

  | | all runs | with lidar fixes | GNSS and IMU only | 60 s | 300 s |
  |---|---|---|---|---|---|
  | first version of the lever arm (attitude and gyro bias all consider states, `S0`) | 2,813 of 15,104 (18.6 %) | 43 of 7,552 (0.6 %) | 2,770 of 7,552 (36.7 %) | 17.3 % | 26.2 % |
  | previous round (`S6vb`) | 760 of 15,104 (5.0 %) | 0 of 7,552 | 760 of 7,552 (10.1 %) | 2.3 % | 20.2 % |

  With the offset ignored, as before the lever arm, every GNSS-and-IMU-only run diverged
  (the antenna is 0.4 to 1.1 m from `base_frame`). With the antenna at `base_frame` nothing
  changed: 1,265 of 3,200 such GNSS-and-IMU-only runs diverge in the same grid, with the same
  plain update as on `main`. All tables, every run, every scheme, and the numbers quoted here:
  [`experiments/results/eskf_lever/report.md`](experiments/results/eskf_lever/report.md).
  The node check `scripts/fusion_extrinsic_check.py` adds a constant gyro bias (0.003, −0.002,
  0.002 rad/s, and 0.5 deg/s on each axis), a reseed, a start before the antenna mount is known,
  a body on a slope, a start and a stop, a gap in the fixes, and (not in the default run: 10
  minutes) 600 s of standing, and 600 s of standing followed by a start and a stop (antennas A and B).
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
