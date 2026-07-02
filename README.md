# prism_loc

[![CI](https://github.com/kjungmo/prism_loc/actions/workflows/ci.yml/badge.svg)](https://github.com/kjungmo/prism_loc/actions/workflows/ci.yml) [![License](https://img.shields.io/github/license/kjungmo/prism_loc?color=blue)](LICENSE) ![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white) [![Sponsor](https://img.shields.io/github/sponsors/kjungmo?logo=githubsponsors&color=ea4aaa)](https://github.com/sponsors/kjungmo)

**A pluggable LiDAR localization stack for ROS 2 Humble — three backends over a
shared estimator family: 2D MCL, 3D NDT-MCL, and a 3D LiDAR + IMU + RTK-GNSS
error-state fusion — all with one standard `map → odom` output.**

## 💛 Sponsor
If prism_loc saves you time, consider [sponsoring](https://github.com/sponsors/kjungmo).
Sponsorship funds maintenance, new features, and faster issue response. Backers will
be acknowledged here — thank you.

`prism_loc` localizes a mobile robot inside a prior map from a **2D LiDAR**
(`LaserScan` vs `OccupancyGrid`), a **3D LiDAR** (`PointCloud2` vs a `.pcd` map),
or a **3D LiDAR fused with IMU and RTK-GNSS**. Pick a backend; the output contract
— `map→odom` TF, pose with covariance, `/initialpose` seeding — follows the
`nav2_amcl` / `hdl_localization` / `robot_localization` conventions ROS users
already know.

## Why "PRISM"

**PRISM** = **P**article-filter · **R**TK · **I**nertial · **S**can-matching — the
four ingredients this stack fuses to localize against a prior map:

- **P**article-filter — the Monte Carlo Localization core (`laser2d` / `ndt3d` backends).
- **R**TK — RTK-GNSS absolute position fixes (the `fusion3d` backend).
- **I**nertial — IMU-driven prediction in the error-state Kalman filter (`fusion3d`).
- **S**can-matching — 2D likelihood-field and 3D NDT registration against the prior map.

The name also nods to the optics: a prism splits one beam of light into its
components, just as `prism_loc` splits one estimator core into multiple
sensor/observation paths — grounded in **NDT-MCL** (Saarinen et al., IROS 2013),
which showed a 3D NDT map can be the measurement model inside the very same
particle filter that 2D AMCL uses.

## Architecture

Two estimator cores share one job — track pose against a prior map — and are both
pure C++17 + Eigen, unit-tested with gtest **without ROS**. rclcpp and PCL live
only in the two ROS node packages.

```
estimator cores  (pure C++17 + Eigen, no ROS/PCL, gtest-tested):

  prism_loc_core — Particle filter (MCL)        prism_loc_fusion — Error-state KF (ESKF)
    OdometryMotionModel ─► ParticleFilter         IMU predict ─► 15-state ESKF
      KLD resample  ├─ Laser2DLikelihoodField                  ◄─ NDT 6-DoF pose update
                    └─ Ndt3DModel                              ◄─ RTK-GNSS position update
    └─► pose + 6×6 covariance                     └─► pose + velocity (+ WGS84 LLA→ENU)

ROS 2 nodes  (rclcpp / tf2 / PCL here only):

  prism_loc                backend = laser2d | ndt3d
  prism_loc_fusion_ros     backend = fusion3d
    in :  /scan | /points | /imu | /gnss(NavSatFix) ,  /map | map.pcd ,  /initialpose ,  TF odom→base
    out:  /tf (map→odom) ,  ~/pose ,  ~/particle_cloud | ~/odometry
```

## Prerequisites

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

## Quick start

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

## Building a map

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
   LIO-SAM, FAST-LIO2, or an offline ICP/NDT accumulation script) to produce
   one globally-consistent point cloud.
3. Save that merged, downsampled cloud as a single named `.pcd` file (e.g.
   `pcl::io::savePCDFileBinary("/path/to/map.pcd", merged_cloud)`).
4. Point `map_pcd_path:=/path/to/map.pcd` at it for `ndt3d.launch.py` /
   `fusion3d.launch.py`.

## Interface

| Backend | Package | Input | Output |
|---|---|---|---|
| **laser2d** | `prism_loc` | `/scan`, `/map`, `/initialpose`, TF `odom→base` | `/tf` `map→odom`, `~/pose`, `~/particle_cloud` |
| **ndt3d** | `prism_loc` | `/points`, `map.pcd`, `/initialpose`, TF `odom→base` | same |
| **fusion3d** | `prism_loc_fusion_ros` | `/points`, `/imu`, `/gnss` (NavSatFix), `map.pcd`, `/initialpose` | `/tf` `map→odom`, `~/pose`, `~/odometry` |

See [`SPEC.md`](SPEC.md) for the full design,
[`PARAMS.md`](PARAMS.md) for the full parameter reference (every
`laser2d`/`ndt3d`/`fusion3d` parameter, its default, and its meaning), and
[`docs/superpowers/plans/`](docs/superpowers/plans/) for the implementation plan.

Something not working? See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

## Status

v0.1 — three backends: 2D likelihood-field MCL and 3D NDT-MCL (planar x, y, yaw),
plus a 3D LiDAR + IMU + RTK-GNSS error-state Kalman fusion (`fusion3d`) with full
6-DoF state and IMU-bias estimation. The `laser2d` backend **self-initializes via
branch-and-bound (BBS) global localization** — recovering pose from a single scan
with no `/initialpose` (param `try_global_localization`, on-demand service
`~/global_localization`). 3D global localization and tight LiDAR-IMU time-offset
estimation are roadmap items.

## Paper

A systems-paper draft describing the architecture — *PRISM-Loc: Three LiDAR
Localization Backends Behind One ROS 2 Contract, with Middleware-Free Estimator
Cores* — lives in [`docs/paper/`](docs/paper/) (LaTeX sources +
[`main.pdf`](docs/paper/main.pdf)). If prism_loc is useful in your research,
please cite it via [`CITATION.cff`](CITATION.cff).

## License

Apache-2.0.
