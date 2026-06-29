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

## Quick start

```bash
# 1. Estimator cores — build & test with the plain system toolchain (no ROS):
cmake -S prism_loc_core   -B build/core   -DPRISM_LOC_CORE_BUILD_TESTS=ON
cmake -S prism_loc_fusion -B build/fusion -DPRISM_LOC_FUSION_BUILD_TESTS=ON
cmake --build build/core   -j && ( cd build/core   && ctest --output-on-failure )
cmake --build build/fusion -j && ( cd build/fusion && ctest --output-on-failure )

# 2. Full ROS 2 Humble build (workspace):
#    place this repo at <ws>/src/prism_loc, then:
colcon build --symlink-install
colcon test --packages-select prism_loc_core prism_loc prism_loc_fusion prism_loc_fusion_ros

# 3. Run — 2D LiDAR (MCL):
ros2 launch prism_loc laser2d.launch.py map:=/path/to/map.yaml
#    3D LiDAR (NDT-MCL):
ros2 launch prism_loc ndt3d.launch.py map_pcd_path:=/path/to/map.pcd
#    3D LiDAR + IMU + RTK-GNSS (ESKF fusion):
ros2 launch prism_loc_fusion_ros fusion3d.launch.py map_pcd_path:=/path/to/map.pcd
#    laser2d/ndt3d: click "2D Pose Estimate" in RViz to seed; fusion3d
#    auto-initializes from the first RTK fix (or use /initialpose).
```

## Interface

| Backend | Package | Input | Output |
|---|---|---|---|
| **laser2d** | `prism_loc` | `/scan`, `/map`, `/initialpose`, TF `odom→base` | `/tf` `map→odom`, `~/pose`, `~/particle_cloud` |
| **ndt3d** | `prism_loc` | `/points`, `map.pcd`, `/initialpose`, TF `odom→base` | same |
| **fusion3d** | `prism_loc_fusion_ros` | `/points`, `/imu`, `/gnss` (NavSatFix), `map.pcd`, `/initialpose` | `/tf` `map→odom`, `~/pose`, `~/odometry` |

See [`SPEC.md`](SPEC.md) for the full design and
[`docs/superpowers/plans/`](docs/superpowers/plans/) for the implementation plan.

## Status

v0.1 — three backends: 2D likelihood-field MCL and 3D NDT-MCL (planar x, y, yaw),
plus a 3D LiDAR + IMU + RTK-GNSS error-state Kalman fusion (`fusion3d`) with full
6-DoF state and IMU-bias estimation. The `laser2d` backend **self-initializes via
branch-and-bound (BBS) global localization** — recovering pose from a single scan
with no `/initialpose` (param `try_global_localization`, on-demand service
`~/global_localization`). 3D global localization and tight LiDAR-IMU time-offset
estimation are roadmap items.

## License

Apache-2.0.
