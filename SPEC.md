# prism_loc — Specification

A real, open-source ROS 2 **Humble** localization tool for mobile robots that
takes **either a 2D LiDAR scan or a 3D LiDAR point cloud** as input and tracks
the robot's pose inside a prior map. One Monte Carlo Localization (particle
filter) estimator core drives **two pluggable observation models**, selected at
runtime by a single parameter.

License: **Apache-2.0**. No proprietary dependencies.

---

## 1. Why this design

The research corpus (`~/kangj/loops/localization-tool-.../resources.md`, 144
sources) makes the unifying insight explicit:

- **MCL** (Fox et al., AAAI-99) and **Adaptive MCL / KLD-sampling** (Fox) are the
  canonical particle-filter localizers against a known map — the math behind
  `nav2_amcl`.
- **NDT-MCL** (Saarinen et al., IROS 2013) proves that a **3D NDT map can be used
  as the observation model *inside the same particle filter*** to reach
  industrial-AGV precision. So 3D LiDAR localization is *also* MCL — just with a
  different measurement model.
- **Beluga / beluga_amcl** (Ekumen-OS) demonstrates a clean, ground-up, modular
  C++17 MCL library whose ROS node reaches interface parity with `nav2_amcl`.

Therefore the architecture is **one estimator core + two observation models**:

| Backend | Map | Input topic | Observation model | Lineage |
|---|---|---|---|---|
| `laser2d` | `nav_msgs/OccupancyGrid` | `/scan` (`sensor_msgs/LaserScan`) | likelihood field (Probabilistic Robotics §6.4) | AMCL |
| `ndt3d` | `.pcd` point cloud | `/points` (`sensor_msgs/PointCloud2`) | NDT voxel-Gaussian score (Biber 2003; Magnusson 2009) | NDT-MCL |

Both produce the **identical output contract** (below). This is not a hack: it is
NDT-MCL sitting next to likelihood-field MCL, sharing the particle filter,
motion model, KLD-adaptive resampling, and pose/covariance estimation.

---

## 2. I/O contract (the "commonly used" ROS 2 localization interface)

Modeled exactly on `nav2_amcl` / `hdl_localization` / `robot_localization`
conventions and **REP-105** (Nav2 "Setting Up Transformations").

### Inputs (subscriptions / params)
| Name | Type | Backend | Notes |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | laser2d | planar scan |
| `/points` | `sensor_msgs/PointCloud2` | ndt3d | 3D cloud |
| `/map` | `nav_msgs/OccupancyGrid` (latched/transient_local) | laser2d | prior 2D map from `map_server` |
| `map.pcd` | file path param `map_pcd_path` | ndt3d | prior 3D map (PCD) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | both | RViz "2D Pose Estimate" |
| odometry | **TF** `odom → base_link` | both | consumed via tf2, *not* a topic |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | both | sensor → base_link extrinsics |

### Outputs (publications)
| Name | Type | Notes |
|---|---|---|
| `/tf` | `map → odom` (`geometry_msgs/TransformStamped`) | **the** localization output per REP-105. We publish `map→odom` so wheel odom keeps `odom→base_link`. |
| `~/pose` (`/prism_loc/pose`) | `geometry_msgs/PoseWithCovarianceStamped` | weighted-mean pose + 6×6 covariance |
| `~/particle_cloud` | `nav2_msgs/ParticleCloud` | particles for RViz |

### TF math (critical, REP-105)
The filter estimates `T_map_base` (map → base_link). We must publish
`map → odom`, so:

```
T_map_odom = T_map_base * inverse(T_odom_base)
```

where `T_odom_base` is read from tf2 at the scan stamp. Publishing
`map→base_link` directly is a known bug (breaks the TF tree); we never do it.

---

## 3. Algorithm

### 3.1 State & particle
2D-planar pose is sufficient for both backends on a ground robot (`ndt3d` tracks
`x,y,z` but the filter samples `x,y,yaw`; `z,roll,pitch` are taken from the TF
height/attitude prior and held — documented limitation; full 6-DoF is future
work). Each particle: `Pose2D{ x, y, yaw }` + `weight`.

### 3.2 Motion model — odometry sample model
`sample_motion_model_odometry` (Probabilistic Robotics §5.4). Given previous and
current `odom→base` poses, decompose into `δ_rot1, δ_trans, δ_rot2`, perturb each
with noise `α1..α4`, and apply to every particle. Drives prediction for **both**
backends.

### 3.3a Observation model — `laser2d` likelihood field
1. From `OccupancyGrid`, compute a **likelihood field**: a 2D distance transform
   giving distance to the nearest occupied cell per cell (two-pass Felzenszwalb
   EDT, or Chamfer). Precomputed once on map receipt.
2. Per particle, for a subsample of `max_beams` scan rays: transform the beam
   endpoint into the map, look up distance `d`, score
   `p = z_hit * exp(-d²/(2 σ_hit²)) + z_rand/z_max`.
3. Particle weight `*= Π p` (sum of log-likelihoods for numerical stability).

### 3.3b Observation model — `ndt3d` NDT score
1. From the PCD map, build an **NDT voxel grid**: bucket points into voxels of
   `ndt_resolution`, store per-voxel mean `μ` and covariance `Σ` (≥`min_points`
   per voxel, regularized). Built once on map load.
2. Per particle, for a subsample of `max_points` cloud points: transform point
   into map, find its voxel, score `p = exp(-½ (x-μ)ᵀ Σ⁻¹ (x-μ))` (NDT score,
   Biber 2003). Skip points whose voxel is empty.
3. Particle weight `*= Π p` (log-domain accumulation).

### 3.4 Resampling — KLD-adaptive low-variance
- **Low-variance / systematic resampler** (Probabilistic Robotics §4.3).
- **KLD-sampling** (Fox): choose the resampled set size from the number of
  occupied bins so particle count adapts between `min_particles` and
  `max_particles`. Resample only when `N_eff < resample_threshold * N`.

### 3.5 Pose & covariance estimate
Weighted mean of `x,y`; circular weighted mean of `yaw`; weighted sample
covariance → 6×6 (fill `x,y,yaw`; large constant variance on `z,roll,pitch`).

### 3.6 Global / initial pose
- `/initialpose`: re-seed particles ~ Gaussian around the given pose+covariance.
- If no initial pose and `set_initial_pose` param given, seed from params.
- **laser2d global localization (implemented):** branch-and-bound (BBS) correlative
  scan matching (Hess et al., ICRA 2016) recovers pose from a single scan against the
  occupancy grid — param `try_global_localization`, service `~/global_localization`
  (`std_srvs/Empty`). 3D global recovery (Scan Context / 3D-BBS) remains future work.

---

## 4. Module / file structure

Two ament packages in one repo. **All algorithmic code lives in `prism_loc_core`,
which has NO ROS and NO PCL dependency** — only Eigen + gtest — so it builds and
unit-tests with the plain system toolchain (gcc 9.4, system Eigen3, system
gtest). The ROS/PCL surface is confined to `prism_loc`.

```
prism_loc_core/                         # pure C++17 + Eigen, no ROS, no PCL
  include/prism_loc_core/
    types.hpp            # Pose2D, Particle, Eigen typedefs, helpers
    occupancy_grid.hpp   # GridMap struct + likelihood-field (distance transform)
    ndt_map.hpp          # NdtMap: voxel → {mean, cov, inv_cov}
    motion_model.hpp     # OdometryMotionModel
    measurement_model.hpp# MeasurementModel interface + Laser2D + Ndt3D
    particle_filter.hpp  # ParticleFilter: predict/correct/resample/estimate
    rng.hpp              # seedable RNG wrapper (deterministic tests)
  src/                   # one .cpp per header that needs out-of-line defs
  test/                  # gtest per unit + one integration test

prism_loc/                              # ROS 2 Humble node
  include/prism_loc/localization_node.hpp
  src/localization_node.cpp   # subs/pubs/TF, backend select, calls core
  src/laser_adapter.cpp       # LaserScan + OccupancyGrid -> core types
  src/pointcloud_adapter.cpp  # PointCloud2 + PCD(PCL) -> core types
  src/main.cpp
  launch/laser2d.launch.py
  launch/ndt3d.launch.py
  params/laser2d.yaml
  params/ndt3d.yaml
  rviz/prism_loc.rviz
  test/test_tf_math.cpp       # map->odom composition unit test (gtest, no rclcpp)
```

---

## 5. Parameters (ROS params, namespaced)

Common: `backend` (`laser2d`|`ndt3d`), `global_frame`(map), `odom_frame`(odom),
`base_frame`(base_link), `min_particles`(500), `max_particles`(2000),
`update_min_d`(0.2 m), `update_min_a`(0.2 rad), `resample_threshold`(0.5),
`transform_tolerance`(0.1 s), `tf_broadcast`(true), `alpha1..alpha4`(0.2),
`set_initial_pose`/`initial_pose_{x,y,yaw}`.

laser2d: `scan_topic`(/scan), `max_beams`(60), `z_hit`(0.5), `z_rand`(0.5),
`sigma_hit`(0.2 m), `laser_max_range`(100), `laser_min_range`(0).

ndt3d: `points_topic`(/points), `map_pcd_path`, `ndt_resolution`(1.0 m),
`max_points`(500), `voxel_min_points`(5), `points_voxel_leaf`(0.5 m, downsample).

---

## 6. Test strategy (TDD)

Every algorithmic claim has a deterministic gtest (seeded RNG, synthetic
fixtures) in `prism_loc_core/test`, runnable with **no ROS**:

- **rng**: same seed → same sequence; different seed → different.
- **occupancy_grid**: a single occupied cell yields exact Euclidean distances in
  the likelihood field; world↔grid index round-trips.
- **motion_model**: zero odom delta + zero noise → particle unchanged; pure
  translation moves along heading; mean of many noisy samples ≈ commanded delta.
- **measurement_model laser2d**: a particle at ground truth outscores a displaced
  particle for a synthetic scan against a synthetic grid.
- **measurement_model ndt3d**: against a synthetic planar/box cloud, the
  aligned particle outscores a translated particle; empty voxels are skipped.
- **ndt_map**: voxel mean/cov of a known Gaussian blob recovers μ,Σ within tol.
- **particle_filter**: (a) `N_eff` math; (b) low-variance resampler reproduces a
  known multiset for fixed weights; (c) **integration** — seed particles around a
  ground-truth pose in a synthetic grid/cloud, feed a sequence of odom+scan
  steps, assert the weighted-mean pose converges to ground truth within tol.
- `prism_loc/test/test_tf_math`: `T_map_odom = T_map_base * inv(T_odom_base)`
  round-trips a known pair.

CI-equivalent gates: `prism_loc_core` builds + all ctest green with system
toolchain; `prism_loc` builds clean under colcon in the `ros2_humble` env.

---

## 7. Build & run

**Core only (no ROS):**
```bash
cmake -S prism_loc/prism_loc_core -B build/core -DPRISM_LOC_CORE_BUILD_TESTS=ON
cmake --build build/core -j
( cd build/core && ctest --output-on-failure )   # cd-in form: portable across ctest versions
```

**Full ROS 2 build (Humble via micromamba; clean env to avoid noetic leak):**
```bash
env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH \
  /home/cona/.local/bin/micromamba run -n ros2_humble bash -c '
    cd /home/cona/kangj/prism_loc_ws &&
    colcon build --symlink-install &&
    colcon test --packages-select prism_loc_core prism_loc &&
    colcon test-result --verbose'
```

**Run (laser2d):**
```bash
ros2 launch prism_loc laser2d.launch.py map:=/path/to/map.yaml
# then in RViz: "2D Pose Estimate" to seed /initialpose
```

---

## 8. Global constraints (verbatim into the plan)

- ROS 2 **Humble**, **C++17**, **Apache-2.0**.
- `prism_loc_core`: **no rclcpp, no PCL** — Eigen3 + gtest only; must build with
  system gcc/cmake outside any ROS env.
- Localization output is the **`map → odom`** TF (REP-105), never `map→base_link`.
- Numerically stable weighting: accumulate **log-likelihoods**, normalize before
  exp; never multiply raw probabilities in a loop.
- Deterministic tests: all randomness goes through the seedable `Rng` wrapper; no
  bare `rand()`/`std::random_device` in library code paths under test.
- No "Generated with Claude" / Co-Authored-By footers anywhere (user rule).
- Frequent commits, one per task; TDD (failing test first).

---

## 9. Key references (from the research loop)

- Fox et al., *Monte Carlo Localization* (AAAI-99); Fox, *KLD-Sampling*.
- Thrun/Burgard/Fox, *Probabilistic Robotics* — motion, likelihood-field,
  low-variance resampling.
- Biber & Strasser, *NDT* (IROS 2003); Magnusson, *3D-NDT* (thesis 2009).
- Saarinen et al., *NDT-MCL* (IROS 2013) — NDT-as-observation-model in MCL.
- Ekumen-OS *Beluga / beluga_amcl* — modular C++ MCL reference.
- Nav2 *Setting Up Transformations* (REP-105 TF chain); `nav2_amcl` config docs.
- Koide *hdl_localization*; at-wat *mcl_3dl*; CATEC *amcl3d* — 3D MCL/NDT lineage.
