# prism_loc

**A pluggable Monte Carlo Localization tool for ROS 2 Humble — one particle
filter, two LiDAR observation models (2D occupancy-grid likelihood field _or_ 3D
NDT point cloud), one standard `map → odom` output.**

`prism_loc` localizes a mobile robot inside a prior map using **either** a 2D
LiDAR (`sensor_msgs/LaserScan` against a `nav_msgs/OccupancyGrid`) **or** a 3D
LiDAR (`sensor_msgs/PointCloud2` against a `.pcd` map). Pick the backend with one
parameter; the rest of the interface — TF, pose, particle cloud, `/initialpose`
seeding — is identical and matches the `nav2_amcl` / `hdl_localization`
conventions ROS users already know.

## Why "prism"

A prism splits one beam of light into its components. `prism_loc` splits one
estimator core into a 2D and a 3D observation path — grounded in **NDT-MCL**
(Saarinen et al., IROS 2013), which showed a 3D NDT map can be the measurement
model inside the very same particle filter that 2D AMCL uses.

## Architecture

```
            ┌──────────────── prism_loc_core (no ROS, no PCL — Eigen only) ───────────────┐
            │  OdometryMotionModel ──► ParticleFilter ◄── MeasurementModel (interface)     │
            │                              │ KLD resample        ├── Laser2DLikelihoodField │
            │                              ▼                     └── Ndt3DModel             │
            │                       pose + 6×6 covariance                                   │
            └────────────────────────────────┬───────────────────────────────────────────┘
                                              │
                    prism_loc (ROS 2 node) ───┘  subs: /scan|/points, /map|map.pcd, /initialpose, TF
                                                  pubs: /tf (map→odom), ~/pose, ~/particle_cloud
```

The core is pure C++17 + Eigen and unit-tested with gtest **without ROS**. ROS
and PCL live only in the node package.

## Quick start

```bash
# 1. Core algorithms — builds & tests with the plain system toolchain (no ROS):
cmake -S prism_loc_core -B build/core -DPRISM_LOC_CORE_BUILD_TESTS=ON
cmake --build build/core -j && ctest --test-dir build/core --output-on-failure

# 2. Full ROS 2 Humble build (workspace):
#    place this repo at <ws>/src/prism_loc, then:
colcon build --symlink-install
colcon test --packages-select prism_loc_core prism_loc

# 3. Run (2D LiDAR):
ros2 launch prism_loc laser2d.launch.py map:=/path/to/map.yaml
#    Run (3D LiDAR):
ros2 launch prism_loc ndt3d.launch.py map_pcd_path:=/path/to/map.pcd
#    Then click "2D Pose Estimate" in RViz to seed the filter.
```

## Interface

| | Input | Output |
|---|---|---|
| **laser2d** | `/scan`, `/map`, `/initialpose`, TF `odom→base` | `/tf` `map→odom`, `~/pose`, `~/particle_cloud` |
| **ndt3d** | `/points`, `map.pcd`, `/initialpose`, TF `odom→base` | same |

See [`SPEC.md`](SPEC.md) for the full design and
[`docs/superpowers/plans/`](docs/superpowers/plans/) for the implementation plan.

## Status

v0.1 — 2D likelihood-field MCL and 3D NDT-MCL backends, planar (x, y, yaw)
estimation. Global/kidnapped-robot recovery and full 6-DoF are roadmap items.

## License

Apache-2.0.
