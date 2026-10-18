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
error ≤ 0.5 m after 10 s and ≤ 0.25 m after 30 s, yaw error ≤ 0.05 rad, the `pose not
verified` WARN after `/initialpose` and an OK status from then on in healthy tracking,
and the `low scan-to-map match` WARN after a kidnap, for a pose off the map and for a
wrong `base_height`. This is a ROS-path check on synthetic
data, not a field result.
[`scripts/fusion_extrinsic_check.py`](scripts/fusion_extrinsic_check.py) feeds `fusion3d`
a robot that stands, or turns on the spot, with its lidar, IMU and GNSS antenna mounted
off `base_link`, and asserts the pose of `base_link` within 0.05 m and 0.02 rad, the
`/diagnostics` status when a mount is missing from TF, and, on GNSS and IMU alone with
sensor noise, height, tilt and heading within 0.15 m, 0.03 rad and 0.05 rad and a
horizontal error within twice the reported sigma; with a constant gyro bias (45 s), across a
reseed (also with the body on a slope), from a start before the antenna mount is known, over a
start and a stop and through a gap in the fixes, height and tilt within the same
bounds, heading within 0.05 rad plus the vertical bias times the time, and a horizontal error
within 3 sigma, and the heading values and the WARN on `/diagnostics` (see
[Sensor frames](#sensor-frames-fusion3d)). CI runs it on an optimized build
of the two fusion packages; the whole script takes about 15 minutes. `--long` adds 600 s of
standing (`gnss_stand_long`, 10 more minutes, not in CI).
GoogleTest cases: 50 in `prism_loc_core`, 43 in `prism_loc_fusion`, 29 in
`prism_loc`, 24 in `prism_loc_fusion_ros` (146 in total).

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
| WARN | odom or sensor TF lookups failing; relocalization pending, ambiguous or without candidate; effective particle count below `min_neff_fraction` (laser2d; off by default for ndt3d); ndt3d: scan-to-map match below `min_match_fraction`, or pose not verified since the last reseed; zero-stamped scans dropped; `use_sim_time` on while the ROS clock does not advance | no accepted NDT correction for `correction_timeout_s`; `odom->base_link` lost after being seen, or appearing while the map->base_link fallback was broadcast (two parents); fallback off and no odometry (no TF); IMU gaps; ROS clock not advancing under `use_sim_time`; point clouds or GNSS fixes not used because their mount is unknown (the IMU likewise, within `startup_timeout_s`) |

Values carried for monitors: `n_eff`, `particles`, `covariance_trace_xy`,
`covariance_yaw`, `input_rate_hz`, `seconds_since_last_input`,
`seconds_since_last_update`, `relocalization`, `relocalization_scans_used`,
`relocalization_scans_skipped` (laser2d/ndt3d), `match_fraction`, `match_updates_since_reseed` (ndt3d only); `imu_rate_hz`,
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
the last 5 filter updates. Three states follow from it:

| Status | When |
|---|---|
| WARN `low scan-to-map match: ...` | `match_fraction` below `min_match_fraction` (0.65) |
| WARN `pose not verified since the last reseed: N of 15 ...` | after `/initialpose` (or `set_initial_pose`), until `match_verify_updates` (15) filter updates have been judged, about 3 m of travel; a robot that stands still stays here. A wrong initial pose can match well for its first updates |
| OK `localizing` | otherwise |

**OK is not proof that the pose is right.** The signal is a classifier with these measured
rates, as shares of all filter updates of the runs named (the offline replay
[`experiments/ndt3d_replay.cpp`](experiments/ndt3d_replay.cpp): the real filter core and
`MatchMonitor` driven by synthetic clouds of the world `scripts/synthetic_e2e.py` uses, 4 seeds
per case; no part of a run left out unless stated). Regenerate them with
`experiments/ndt3d_replay_batch.py` and `experiments/ndt3d_replay_rates.py`
([`experiments/README.md`](experiments/README.md), study 7). The raw per-update output is in
`experiments/results/ndt3d_replay/`, and `rates.md` there has the same tables for threshold 0.70,
without the "not verified" state, and one row for every case of the batch, also the ones not in
this table (a cropped prior map, a pitched lidar, other `base_height` values, a second
`/initialpose` after a kidnap).

| Runs | OK | not verified | low match | OK while > 1 m off | OK while > 0.3 m off | low match while < 0.15 m off |
|---|---|---|---|---|---|---|
| healthy, exact prior map; 0.5x to 3x speed, sparser lidars (24 runs) | 89.5 % | 8.8 % | 1.6 % | 0 | 2.3 % | 0.1 % |
| healthy, 2 or 5 cm of noise on the prior map (8) | 87.8 % | 12.2 % | 0 | 0 | 0.2 % | 0 |
| healthy, scene differs from the exact map: 6 or 15 moving columns, boxes added or removed, 3 or 5 cm range noise (24) | 74.7 % | 12.1 % | 13.2 % | 0 | 3.2 % | **11.6 %** |
| the same with 2 cm of noise on the prior map (24) | 87.7 % | 12.2 % | 0.1 % | 0 | 0.8 % | 0.1 % |
| kidnapped, exact map, every update after the kidnap (8) | 8.4 % | 0 | 91.6 % | **2.7 %**, at most 4 updates in a row | 7.0 % | 0 |
| kidnapped, 2 or 5 cm map noise (12) | 24.1 % | 0 | 75.9 % | **6.8 %**, up to 15 updates in a row (7 s); the longest in time 8 s (14 updates) | 21.5 % | 0 |
| `/initialpose` 3.6 m and 1 rad off, exact map, every update (4) | 0.8 % | 8.9 % | 90.2 % | 0 (8.5 % without the "not verified" state) | 0.8 % | 0 |
| the same, 2 or 5 cm map noise (8) | 58.3 % | 10.5 % | 31.2 % | 0 (10.1 % without it) | 9.8 % | 0.2 % |
| `/initialpose` 0.3 to 1.0 m off (28) | 86.8 % | 12.2 % | 1.0 % | 0 | 4.7 % | 0 |
| pose off the map (4) | 0 | 2.0 % | 98.0 % | 0 | 0 | 0 |
| `base_height` 5 or 10 cm wrong, pose fine (8) | 0 | 0.8 % | 99.2 % | 0 | 0 | 82.2 % (a true mismatch, not of x and y) |

What these numbers say:
- **Missed.** A kidnapped robot reads OK in 3 to 7 % of the updates, for up to 8 s at a
  time on a thick (noisy) map, where lost poses match better (a single update up to 0.77;
  0.73 on the exact map) and healthy ones too (0.88 to 1.0). An offset of up to about one NDT
  cell is not detected at all: with `/initialpose` 0.3 to 1.0 m off, 7 of 28 runs were still
  0.5 to 0.87 m off for up to 6.7 s after the status had turned OK.
- **False alarms.** On an exact (thin) map, objects that are not in the map and range noise
  of a few centimetres bring a healthy run under 0.65: 15 moving columns 35 % of the
  updates, 5 boxes added and 1 removed 21 %, 5 cm range noise 20 %, 2 boxes added 3 %,
  6 columns 1 %, 3 cm range noise 0. With 2 cm of noise on the map all of these are at
  most 0.5 %.
- **The default.** At 0.70 the kidnapped misses fall to 0.9 % (exact map) and 4.6 % (noisy
  map), and the false alarms of the third row rise from 13 % to 41 %. 0.65 is kept;
  it has to be set per site.
- **Setting it.** Drive a known-good lap, read the value
  (`ros2 topic echo /diagnostics | grep -A1 match_fraction`), take its minimum and set
  `min_match_fraction` 0.03 to 0.05 below that minimum, not lower: lost poses in the
  replay reached 0.77 (0.78 on the node, below), so a threshold far below the healthy
  minimum gives the misses room.
- **A wrong initial pose** (3.6 m and 1 rad off) stayed above the threshold for the first 9 to 15
  judged updates, while 0.8 to 2.5 m off; that is why the "not verified" state exists.
- The value is that of the last filter update: it does not change while the robot stands.

Through `/diagnostics` on the node (pairing harness without strata, 75 s per run, every
status from t = 2 s on). **These runs came from an integration harness outside this repository
(it starts the node and a simulated robot and reads `/diagnostics`); no script in this repository
regenerates this table, and the replay above is the committed counterpart**:

| Runs | statuses | OK | not verified | low match | OK while > 1 m off | OK while > 0.3 m off | `match_fraction` |
|---|---|---|---|---|---|---|---|
| healthy, exact map, 4 seeds | 292 | 260 | 32 | 0 | 0 | 8 | 0.732 – 0.888 |
| healthy, 2 cm and 5 cm map noise | 146 | 130 | 16 | 0 | 0 | 0 | 0.944 – 1.000 |
| kidnapped at 40 s / 30 s, exact map (2 runs), statuses after the kidnap | 80 | 1 | 0 | 79 | 1 | 1 | 0.329 – 0.719 |
| kidnapped at 30 s, 5 cm map noise, statuses after the kidnap | 45 | 2 | 0 | 43 | 2 | 2 | 0.351 – 0.782 |
| `/initialpose` 3.6 m and 1 rad off, exact map | 73 | 0 | 3 | 70 | 0 | 0 | 0.233 – 0.677 |
| the same, 2 cm and 5 cm map noise | 146 | 80 | 14 | 52 | 0 | 22 | 0.232 – 1.000 |
| `/initialpose` 0.5 m, 1.0 m, and 1.3 m with 0.35 rad off | 219 | 194 | 25 | 0 | 0 | 18 | 0.730 – 0.876 |
| pose off the map | 73 | 0 | 0 | 73 | 0 | 0 | 0.000 – 0.000 |
| `base_height` 0.5 m too low | 73 | 0 | 0 | 73 | 0 | 0 | 0.328 – 0.462 |

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
  With the antenna at `base_frame` nothing changes.

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
     A gap of more than 1 s in the fixes in the window, or a body rate over `0.125 m/s / |t_xy|` (a faster turn moves the antenna without the body), stops it.
  7. **Blind.** While the heading is held back, over 1 rad of spread, and not alignable (standing, turning on the spot), the update
     leaves roll and pitch out of the lever term (a filter without a lever arm does the same) and counts the circle of the lever
     as noise on the two horizontal axes: `lever_blind: true`. The antenna then says where the circle is and tells roll and pitch nothing.
     The release stays closed.

  **What you see on `/diagnostics`** (new values; none changes an existing one):

  | State | `heading_source` | `heading_sigma_rad` | `heading_capped` | `lever_release` | Status |
  |---|---|---|---|---|---|
  | filter not initialized | `none` | `n/a` | `false` | `n/a` | as before |
  | antenna at `base_frame` (no lever arm) | the last source | the filter's | `false` | `n/a` | as before, never the heading WARN |
  | lever arm, less than 30 s since the start or the last `/initialpose`, GNSS and IMU only | `none` or `initialpose` | 0.35 | `true` | `closed` | OK |
  | lever arm, 30 s or more with the bound active and no heading source | `none`, `initialpose` or `lidar` (the last one) | grows (the spread held, 6 rad after 600 s); `heading_sigma_used_rad` 0.35 | `true` | `closed` | **WARN** `heading not observed: GNSS and IMU only for N s, heading spread X rad, ...` |
  | start or stop of 0.25 m/s or more (the alignment) | `motion` | falls to 0.35 and below | `false` | `closed` or `open` | no heading WARN |
  | turn at speed (release open) | unchanged | falls below 0.35 | `false` | `open` | no heading WARN |
  | lidar corrections accepted | `lidar` | about 0.03 | `false` | `closed`, `open` while accelerating | OK |
  | fixes stop for more than 2 s | unchanged | unchanged | `false` | unchanged | the heading WARN clears; its 30 s start again with the first fix |

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
  Spinning for minutes on the spot with 0.5 deg/s and no heading source, a start heading off by
  1 rad or more followed by a short stand and a start (the alignment begins at 1 rad of spread, the filter starts at 0.71 rad; see the third pass below), and 120 s without fixes on a biased IMU are outside what
  was measured to work (see the counts below).

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
  on other data). 1,152 lever-arm runs and 384 zero-lever runs per scheme and suite: stand 300 to 1,800 s at a gyro bias of 0 to 0.2 deg/s, then a start and a
  stop, a start and a circle, or a turn on the spot, for 160 s:

  | | the version before (`S7a35`) | this version (`S8`) | zero lever, same data |
  |---|---|---|---|
  | hsg: runs with `tz` (tilt over 0.05 rad or height over 0.5 m) | 375 | **26** | 324 |
  | hsg: height over 1 m / tilt over 0.3 rad | 271 / 161 | **1 / 0** | |
  | hsgv (other data): runs with `tz` | 381 | **21** | 366 |
  | hsgv: height over 1 m / tilt over 0.3 rad | 279 / 157 | **0 / 0** | |
  | hsgv: start and stop, heading error over 0.5 rad at the end | 138 of 384 | **0 of 384** | |
  | hsgv: `tz` in start-stop / circle / turn on the spot | 184 / 172 / 25 | **14 / 7 / 0** | 171 / 195 / 0 |

  What is left: runs with `tz` where the zero-lever run has none, 12 of 1,152 in hsg (11 with the 1.5 m mast, whose vertical arm amplifies a tilt of 0.05 rad), 8 in hsgv;
  after a turn on the spot nothing tells the heading (the heading is over 0.5 rad off at the end in 267 of 384 runs, the zero-lever filter has the same); a start heading 1 or 2 rad off followed by a start
  recovers in 14 and 8 of 24 runs (21 and 3 before). The second pass with `S8` added: the clean envelope has 0 runs with `tz` as before, 213 runs of 4,200 are over 1 m from the truth (156 before; 59 of the new ones are antenna A at 0.5 deg/s, where
  the circle of the lever is 1.0 m). All counts: [`report.md`](experiments/results/eskf_lever/report.md).
  On the real node (`scripts/fusion_extrinsic_check.py --scenario gnss_stand_go` and `gnss_stand_go_b`; 600 s of standing with the review's gyro bias, then a start and a stop):
  largest height error 0.062 m and 0.091 m, tilt 0.004 and 0.007 rad, heading error after the stop +0.03 and −0.04 rad, `heading_source: motion` 2.0 s after the start; the version before had +1.32 m and 0.15 rad
  (antenna A) and ±2.5 m and 0.2 rad (antenna B). The default run of the script (216 checks) and `gnss_stand_long` pass; the zero-lever poses are bit for bit those of `main`.

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
