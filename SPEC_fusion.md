# prism_loc — Fusion backend (`fusion3d`) Specification

A tightly-structured **error-state Kalman filter (ESKF)** that fuses **3D LiDAR
point cloud + IMU + RTK-GNSS** to localize a mobile robot in a prior PCD map.
Option **B** from the design discussion: the `hdl_localization` (IMU prediction +
NDT correction) pattern extended with a GNSS position update and map
georeferencing, in the spirit of `robot_localization` (EKF + `navsat`).

Delivered as **two new standalone ament packages** that sit beside `prism_loc_core`
and `prism_loc` in the repo; neither existing package is modified. License **Apache-2.0**.

---

## 1. Why ESKF (not the particle filter)

The PF core is excellent for a single observation model but scales badly when the
state must include velocity and IMU biases. Tightly-coupled IMU with bias and
gravity estimation is the home turf of Kalman filters. Corpus support:
`hdl_localization` (UKF: IMU predict + NDT correct), `robot_localization` (EKF
fusing IMU + a pose source + GPS via `navsat_transform`), Barfoot's *State
Estimation for Robotics* (Lie-group ESKF). We use an **error-state EKF** (Solà,
*Quaternion kinematics for the error-state KF*): the nominal state integrates IMU;
the 15-dim error state is Kalman-updated by LiDAR-NDT pose and GNSS position, then
injected and reset.

## 2. State

**Nominal** `x` (16 params): `p∈R³` (map), `v∈R³` (map), `q` (unit quat, body→map),
`b_a∈R³` (accel bias), `b_g∈R³` (gyro bias).
**Error** `δx∈R¹⁵`, ordered **`[δp(0:3) δv(3:6) δθ(6:9) δb_a(9:12) δb_g(12:15)]`**;
`δθ` is the so(3) tangent. Covariance `P∈R¹⁵ˣ¹⁵`.

**Assumption (documented):** map is **gravity-aligned, z-up**, so gravity is a
fixed constant `g=[0,0,−9.81]` in map (not estimated). Standard for ground-robot
prior maps.

## 3. Sensors → ESKF operations

| Sensor | ROS type | Rate | ESKF role |
|---|---|---|---|
| IMU | `sensor_msgs/Imu` (`/imu`) | 100–400 Hz | **predict**: nominal integration + `P = F P Fᵀ + Q` |
| 3D LiDAR | `sensor_msgs/PointCloud2` (`/points`) | ~10 Hz | **updatePose**: PCL NDT registers live cloud → PCD map (init = current ESKF pose) → 6-DoF pose measurement |
| RTK-GNSS | `sensor_msgs/NavSatFix` (`/gnss`) | 1–10 Hz | **updatePosition**: lat/lon/alt → ENU (datum) → 3-DoF position, gated by fix status/covariance |
| `/initialpose` | `PoseWithCovarianceStamped` | — | optional manual seed (else auto-init, §6) |

## 4. ESKF math

**Predict** (IMU `a_m, ω_m`, step `dt`): `R=q.toRotationMatrix()`, `a=a_m−b_a`,
`ω=ω_m−b_g`, `a_w=R·a+g`:
- `p += v·dt + ½·a_w·dt²`; `v += a_w·dt`; `q = (q ⊗ Exp(ω·dt)).normalized()`.
- `F=I₁₅`, blocks: `F(δp,δv)=I·dt`, `F(δv,δθ)=−R·skew(a)·dt`, `F(δv,δb_a)=−R·dt`,
  `F(δθ,δθ)=Exp(ω·dt)ᵀ`, `F(δθ,δb_g)=−I·dt`.
- `Q`: `Q(δv)=σ_a²·dt²·I`, `Q(δθ)=σ_g²·dt²·I`, `Q(δb_a)=σ_{ba}²·dt·I`,
  `Q(δb_g)=σ_{bg}²·dt·I`. `P = F·P·Fᵀ + Q`.

**updatePosition** (`p_z`, `R₃`): `H=[I₃ 0…]`; `y=p_z−p`; `K=P Hᵀ(H P Hᵀ+R)⁻¹`;
`δx=K y`; `P=(I−K H)P`; inject+reset.

**updatePose** (`p_z`, `q_z`, `R₆`): `H` selects `δp` (rows 0:3 ↔ cols 0:3 = I₃),
`δθ` (rows 3:6 ↔ cols 6:9 = I₃); residual `y[0:3]=p_z−p`,
`y[3:6]=Log(q.conjugate()·q_z)`; Kalman update; inject+reset.

**inject+reset:** `p+=δp; v+=δv; q=(q⊗Exp(δθ)).normalized(); b_a+=δb_a; b_g+=δb_g;`
error mean zeroed (covariance-reset Jacobian omitted — acceptable first-order).

so(3) helpers: `skew`, `Exp(φ)` (rotation-vector→quat, small-angle safe),
`Log(q)` (quat→rotation-vector).

## 5. Georeferencing (RTK → map)

`GeodeticConverter` (pure math, no GeographicLib dep): WGS84 geodetic→ECEF→ENU
about a **datum** `(lat₀,lon₀,h₀)`. ENU is the map's local Cartesian frame.
Datum from a param, or **auto-set from the first RTK-fixed sample**. **Fix
gating:** accept GNSS only when `status.status ≥ gnss_min_status`; reject/inflate
when `position_covariance[0]` exceeds `gnss_max_pos_cov`. `R₃` from
`position_covariance`.

## 6. Initialization

- **Manual:** `/initialpose` sets `p`(+yaw); roll/pitch from first IMU gravity; large `P`.
- **Auto:** first RTK-fixed sample sets datum + `p`; first IMU sets level attitude;
  **yaw unobservable from single-antenna RTK** — held with large covariance until
  LiDAR-NDT pose updates resolve it (documented).

## 7. Package structure (two new standalone ament packages)

`prism_loc_fusion` is pure C++17 + Eigen — **no ROS, no PCL** — so the ESKF and
geodetic math build and unit-test with the system toolchain. PCL NDT and ROS I/O
live in `prism_loc_fusion_ros`.

```
prism_loc_fusion/                       # pure C++17 + Eigen, no ROS/PCL
  package.xml  CMakeLists.txt
  include/prism_loc_fusion/{so3,types,geodetic,eskf}.hpp
  src/{so3,geodetic,eskf}.cpp
  test/{test_smoke,test_so3,test_geodetic,test_eskf,test_eskf_integration}.cpp

prism_loc_fusion_ros/                   # ROS 2 Humble node, depends prism_loc_fusion + PCL
  package.xml  CMakeLists.txt
  include/prism_loc_fusion_ros/{ndt_registration,fusion_localization_node}.hpp
  src/{ndt_registration,fusion_localization_node,fusion_main}.cpp
  launch/fusion3d.launch.py
  params/fusion3d.yaml
  test/{test_tf_compose,test_ndt_registration}.cpp
```

## 8. I/O contract (fusion3d node)

**In:** `/imu` (Imu), `/points` (PointCloud2), `/gnss` (NavSatFix), `/initialpose`
(optional), TF `odom→base` (optional). **Out:** `/tf` `map→odom` (REP-105; falls
back to `map→base_link` with a warning if no `odom→base`), `~/pose`
(PoseWithCovarianceStamped, 6×6 from ESKF P), `~/odometry` (`nav_msgs/Odometry`,
velocity).

## 9. Parameters (fusion3d)

`map_pcd_path`, `ndt_resolution`(1.0), `ndt_step_size`(0.1), `ndt_epsilon`(0.01),
`ndt_max_iter`(30), `points_voxel_leaf`(0.5), `ndt_max_fitness`(2.0),
`imu_topic`/`points_topic`/`gnss_topic`, `global_frame`/`odom_frame`/`base_frame`,
`sigma_acc`/`sigma_gyro`/`sigma_acc_bias`/`sigma_gyro_bias`,
`pose_pos_std`/`pose_rot_std`, `gnss_min_status`(0), `gnss_max_pos_cov`(25.0),
`use_datum`/`datum_lat`/`datum_lon`/`datum_alt`, `transform_tolerance`(0.1).

## 10. Test strategy (TDD; core needs no ROS)

`prism_loc_fusion/test` (system toolchain, seedable RNG):
- **so3:** `Exp(0)=I`; `Log(Exp(φ))=φ`; `skew(a)·b=a×b`.
- **geodetic:** datum `(lat₀,lon₀,0)`; +δlon → ENU east ≈ `R_E·cos(lat₀)·Δλ`;
  +δlat → north; +δalt → up; datum → ENU≈0.
- **eskf predict static:** level IMU specific force `[0,0,+9.81]`, zero gyro →
  `v,p`≈0 over 1000 steps.
- **eskf predict constant accel:** `a_m=[ax,ay,az+9.81]` → `v=a_w·t`, `p=½a_w·t²`.
- **updatePosition:** wrong init `p`; repeated GNSS at truth → `p`→truth.
- **updatePose:** feed `(p_z,q_z)` → pose → measurement.
- **integration:** synthetic straight+turn trajectory emitting IMU (bias+noise) +
  periodic NDT pose + GNSS; ESKF tracks ground truth ≤0.3 m, ≤0.1 rad.

`prism_loc_fusion_ros/test`: `test_tf_compose` (map→odom = map→base · inv(odom→base),
SE(3)); `test_ndt_registration` (register a cloud onto a translated copy → recover
the shift within a few cm, via PCL NDT).

CI gates: `prism_loc_fusion` plain-CMake + ctest green; `prism_loc_fusion_ros`
colcon build + test green in `ros2_humble`.

## 11. Global constraints

- ROS 2 **Humble**, **C++17**, **Apache-2.0**.
- `prism_loc_fusion`: **no rclcpp, no PCL** — Eigen3 + gtest only; builds with
  system gcc/cmake outside any ROS env.
- Output TF is **`map→odom`** (REP-105), `map→base_link` only as documented fallback.
- Quaternions kept normalized; error injected via `q ⊗ Exp(δθ)`.
- Deterministic tests via seedable RNG; no bare `rand()`.
- No "Generated with Claude"/Co-Authored-By footers. TDD; one commit per task.
- Build env: core via system toolchain; ROS via
  `env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH micromamba run -n ros2_humble`.

## 12. Key references (corpus)

hdl_localization (UKF IMU+NDT); robot_localization + navsat_transform (EKF+GPS);
Barfoot, *State Estimation for Robotics* (ESKF/Lie groups); Solà, *Quaternion
kinematics for the error-state KF*; LIO-SAM / FAST_LIO_LOCALIZATION (IMU+LiDAR+GNSS
lineage); PCL NDT (registration engine); LiDAR_IMU_Init (extrinsics).
