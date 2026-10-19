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
| RTK-GNSS | `sensor_msgs/NavSatFix` (`/gnss`) | 1–10 Hz (the lever-arm alignment, §8.1, is supported at 1–100 Hz) | **updatePosition** (**updatePositionLever** when the antenna is off `base_frame`, §8.1): lat/lon/alt → ENU (datum) → 3-DoF position, gated by fix status/covariance |
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

**updatePositionLever** (`p_z`, lever `t`, `R₃`), for a fix of a point at `t` in the body
frame (GNSS antenna off `base_frame`): `h = p + R·t`; `H=[I₃ 0 −R[t]× 0 0]` (local attitude
error, `q ← q⊗Exp(δθ)`); `y=p_z−h`; `K=P Hᵀ(H P Hᵀ+R)⁻¹`. Two parts of `K` are removed
before the update, so that they are *consider* states (Schmidt-Kalman: counted in
`H P Hᵀ`, not corrected): the heading, `δθ` along the vertical `u = Rᵀ ẑ` (body frame), and
the vertical gyro bias, the part of `δb_g` along `u`. With `A = I − uuᵀ` the gain rows
become `K_θ ← A K_θ` and `K_bg ← A K_bg`; roll and pitch (`δθ` across `u`), the two
horizontal gyro bias components, `δp`, `δv` and `δb_a` are corrected. Joseph form,
`P=(I−K H)P(I−K H)ᵀ+K R Kᵀ`; inject+reset. The states left out are `EskfParams::lever_consider`
(a mask of `LeverConsider`; default heading and vertical gyro bias; the node does not
set it). A zero lever is `updatePosition`, bit for bit, whatever these are set to.

*Bound* (`lever_heading_sigma_max`, default 0.35 rad, 0 = off). With `u = Rᵀẑ` and
`σ² = uᵀ P_θθ u`, before the gain is formed: if `σ² > σ_max²`, `c = σ_max/σ`,
`D = I₁₅` with its attitude block `I + (c − 1) u uᵀ`, and `P ← D P Dᵀ`. The heading variance
is scaled by `c²`, every cross-covariance of the heading with another state by `c`, the
tilt (across `u`) not at all, and `P` stays symmetric and positive. The estimate is not
touched. `leverHeadingCapped()` tells whether the last lever update scaled. Equal to the bound
is not above it.

*Release* (`lever_release_dv` 1.0 m/s, `lever_release_tau` 4 s, 0 = off). `predict` keeps
`dv ← dv·e^(−dt/τ) + |(R(f − b_a) + g)_xy|·dt` (the horizontal part of the specific force in the
world frame, a step with `dt ≤ 0` or a non-finite value adds nothing). It reads the sensor and the
estimated attitude and changes neither `x` nor `P`. While `dv > lever_release_dv` the update is
the full one (heading and vertical gyro bias included). The
bound is not applied while the release is open (the honest heading below). The release reads the IMU, not the filter's speed estimate.

*Why the bound.* The vertical gyro bias is not corrected while the robot stands, so the
heading variance grows as `(σ_bgz·t)²` (the heading
sigma is 0.71 rad after 10 s and 2.0 rad after 190 s, with the shipped 0.01 rad/s prior). The lever term `−R[t]×` is a good linear model of the antenna
position only for a heading error of a fraction of a radian; at a heading sigma of 2 rad the
update moves tilt and the vertical bias by a lot (a corrected tilt next to an unbounded heading in
the same Jacobian), and after minutes the tilt and the height diverge. The bound keeps the
variance the update is fed in range. What it costs: once the true heading error is over the
bound, the published heading and horizontal covariance are optimistic by up to
`|t_xy|·|heading error|`, at most `2|t_xy|` (README, "GNSS antenna off `base_frame`", limits).

*Honest heading at the release.* The bound hides the
true heading variance from the update, and the vertical gyro bias keeps turning the true heading while the robot stands
(1.05 rad after 600 s at 0.1 deg/s; the filter's own spread is then 6 rad with the 0.01 rad/s prior). A full update at the
release that trusts the bounded 0.35 rad turns the heading the wrong way and tilts the filter. So:
1. *Shadow.* `predict` also carries the 2 x 2 covariance of (heading, vertical gyro bias) with the same prediction, never
   bounded; it is set from `P` at the first lever update after a reset and after every update that estimates the heading (the
   release open, the alignment). `headingSigmaHonest()` reads it; `covarianceHonest(lever)` is `P` with the heading variance
   put back and `|t_xy|²·(min(σ_honest², 1) − min(σ², 1))` added to `P_xx`, `P_yy` (what the node publishes).
2. *Kinematic alignment* (`lever_align_sigma` 1 rad, `lever_align_dv` 0.25 m/s, `lever_align_window` 3 s). When the bound has cut the
   variance (a lever update scaled it since the shadow was set) and the shadow sigma is over `lever_align_sigma`, each lever
   update tries `alignHeading`: from the antenna fixes of the last 3 s it forms the antenna's change of velocity (position
   differences over spans of seconds, no filter state involved: `lever_align_span` 0.8 s and at least 2 fixes) and compares it with the change of velocity the IMU gave in the filter's
   heading (the sum of the world-frame horizontal acceleration that `predict` kept). If both are over `lever_align_dv`, the heading
   is rotated by the angle between them (`atan2`, any error up to ±π), about the antenna so that the antenna does not move; the
   horizontal velocity is set to the antenna's, carried to now by the IMU sum (`lever_align_vel_std` 0.15 m/s); the heading
   variance is `lever_heading_sigma_max²`, independent of every other state (`P ← D P Dᵀ`, `D = I − uuᵀ` on the attitude block, so `P`
   stays positive); `P_xx`, `P_yy` gain `|t_xy|²σ²`; the horizontal position is seated at the newest fix (`x_xy ← fix_xy − (R t)_xy`), because the position that `predict` dead-reckoned through the blind stretch is not the antenna's, and the update that follows would spend the whole difference on the heading and the vertical gyro bias. It does nothing with a gap of more than `lever_align_gap` (1 s; 2.5 fix periods if that is more) between two fixes anywhere from the first fix of the older velocity to now,
   a window of more than `lever_align_window` + `max(1 s, 1.5 T)` between the two velocities (after an outage across a start the change of velocity would be taken over the whole outage; the allowance is over one fix period `T` so that the stamp jitter of a 1 Hz stream does not decide the check), or a body rate over `0.5 lever_align_dv/|t_xy|` in the window (a fast turn moves the antenna without the body). `leverAligned()` is true for that update.
3. *Release.* If the release is open and the variance was held back but the shadow is under `lever_align_sigma`, the shadow is put
   back (`P ← D P Dᵀ` as above, then the shadow variance) and the bound is not applied while the release is open.
4. *Blind.* If the variance was held back, the shadow is over `lever_align_sigma` and the alignment finds nothing, the
   update leaves the attitude columns of `H` out (`H_θ = 0`, a filter without a lever arm) and adds `|t_xy|²` to `R_xx`, `R_yy`: the
   antenna says where the circle of the lever is and tells roll and pitch nothing. The release stays closed. Without it a turn on
   the spot after a long stand tilts the filter (without it 25 of 384 runs of suite hsg, with it 0).
5. *Unknown start* (`lever_start_sigma` 1.81 rad = π/√3, 0 = off). With a lever arm and no heading source (no `/initialpose`, no lidar,
   `initial_yaw_known` false) the node calls `Eskf::markHeadingUnknown()` after `initialize()`: the shadow variance is set to `lever_start_sigma²`, held back, valid. `x` and `P` are not touched.
   The shadow is over `lever_align_sigma`, so the first start or stop aligns the heading (item 2) and an unalignable robot runs the blind update (item 4). A pose fix from the lidar invalidates the shadow as before.
6. *Fix rate.* The alignment measures time, not fixes. `Eskf::fixPeriod()` is the median of the last 15 intervals between lever-arm
   fixes, `T`. The velocity of the antenna is taken over `max(lever_align_span, 2 T)` seconds (at least 2 intervals), the window is `max(lever_align_window, 1.5 span)`, a gap is an
   interval over `max(lever_align_gap, 2.5 T)`, and the history that is checked for gaps and fast turns is the window plus a span. At 5 Hz and above these are 0.8 s, 3 s, 1 s;
   at 1 Hz 2 s, 3 s, 2.5 s. The fixes are kept for twice the history plus two spans (by time, not by count).
   **Supported fix rate: 1 to 100 Hz** (measured: experiments/README.md, suites `hrate` and `hratex`). If the median interval is over `lever_align_period_ok` (1.05 s, a rate under 1 Hz) and not over `lever_align_period_max` (1.25 s, 0.8 Hz), the alignment runs and
   finds the heading less often (`leverDegradedRate()`, `heading_align_rate: degraded`). If it is over `lever_align_period_max`, `alignHeading` does nothing,
   `leverUnsupportedRate()` is true after the first 6 fixes, and a start marked by `markHeadingUnknown()` is taken back (the shadow is set from `P` again, because an unknown start that
   cannot be aligned would run blind for good). The node WARNs on `/diagnostics` in both cases (`GNSS fixes come every N s ...`, `heading_align_rate`, `gnss_fix_period_s`).
7. *Outage spread* (`lever_outage_spread`, 0 = off). While the heading is unknown (the shadow over `lever_align_sigma`) and the fixes are out (an interval over the gap limit of item 6),
   `predict` dead-reckons the acceleration in the filter's heading. If that is off by `e`, the velocity it gained over the outage is `dS` rotated by `e` (`dS`: the IMU sum `cum_a_` over the outage), and the position it gained
   from the acceleration is `D` rotated by `e` (`D`: the integral of `dS` over time, `cum_p_`); the error is `e |dS|` and `e |D|`, fully correlated. `P` does not say so: the bound holds the heading at 0.35 rad, so `P` is cut in
   exactly the part that the honest heading would feed into the velocity and the position. At the first lever update after such an outage, if the alignment finds nothing, `spreadAfterOutage()` adds to the horizontal position and
   velocity block of `P` (`k` = `lever_outage_spread` (0.1) · min(shadow variance, 2 rad²) / 2) the matrix `k [[|D|², D·dS], [D·dS, |dS|²]]` on each axis (positive semidefinite, Cauchy-Schwarz). The fixes that follow then correct the velocity within a few
   updates, instead of explaining its error as a tilt and an accelerometer bias (a mast, with no horizontal arm to absorb it, diverged to a tilt of 3 rad and a height of 1,600 m in 16 of 640 runs). Touches nothing when
   the heading is known, when no outage has happened, or when the alignment succeeded at that update.

*Why the release.* A horizontal specific force resolved with a wrong heading mispredicts the
velocity (`F(δv, δθ) = −R[a]×`), so a start, a stop or a turn at speed shows the heading to one
antenna and an IMU, as it does without a lever arm. A straight drive at constant speed does not
(`dv` = 0), and a speed gate on the filter's own estimate opens on dead-reckoned speed;
the integral reads the IMU. Spinning on the spot has `dv` = 0 and stays closed: the antenna
sweep confounds the heading with the position of one fix, and the first pass of the study
measured the full update over-confident there (a release on the body rate diverged in 28.8 % of the spin runs of the study).

*Why the heading and the vertical gyro bias.* A single antenna does not separate the
heading from the position of the body: `p + R·t` stays put if the heading turns and the
position moves around the antenna. A plain update (`lever_consider` 0) lets the filter
trade one for the other and the vertical gyro bias with them; its covariance then shrinks
along a direction it has not observed (it is over-confident). Roll, pitch and the
horizontal gyro bias *are* observable from the antenna and the IMU (the tilt moves the
antenna by `|t|` per radian and tilts gravity into the velocity), and leaving them out
(the first version) let a constant gyro bias tilt the filter and the height diverge.
Measured in `experiments/eskf_lever.cpp` (README, study 6).

*Start of the filter.* With an antenna off `base_frame` the roll/pitch variance starts at
`(σ_ba,prior² + σ_a²)/g² + ((|f|−g)/g)²` from the first accelerometer sample `f`
(`Eskf::tiltVarianceFromAccel`, about 1e-4 rad² at rest) and the heading variance at 0.5 rad²
(`Eskf::attitudeCovariance`), at the start and before every reseed. With the antenna at
`base_frame` the starting covariance is what it always was (0.5 rad² on all three axes).

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

- **Manual:** `/initialpose` sets `p`(+yaw); roll/pitch from the latest IMU gravity reading (a level
  attitude in the message is not used when the GNSS antenna has a lever arm: a robot on a slope
  would start tilted by the slope with a tilt variance of 1e-4 rad²); large `P`.
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
(optional), TF `odom→base` (optional), TF `base→sensor frame` for every input that is
not expressed in `base_frame` (§8.1). **Out:** `/tf` `map→odom` (REP-105; falls
back to `map→base_link` with a warning if no `odom→base`), `~/pose`
(PoseWithCovarianceStamped, 6×6 from ESKF P), `~/odometry` (`nav_msgs/Odometry`,
linear velocity expressed in `base_link`, the message's `child_frame_id`).

### 8.1 Frames

The state is the pose of `base_frame`. Every input carries a frame (`header.frame_id`, or
the `points_frame` / `imu_frame` / `gnss_frame` parameter when set). If that frame is
`base_frame` the input is used as it is. Otherwise `T_bs` = TF `base_frame → frame`
(latest available, re-read once per second, kept once found) is applied:

- **LiDAR, 6 DoF.** NDT registers the cloud in its own frame, so the initial guess is
  `T_map_base · T_bs` and the measurement fed to `updatePose` is `T_map_sensor · T_bs⁻¹`.
- **IMU, rotation.** `ω_b = R_bs ω_s`, `f_b = R_bs f_s`. The lever-arm terms
  `α × r + ω × (ω × r)` are not compensated.
- **GNSS, translation.** The fix is fused with `updatePositionLever` (§4) at the lever
  arm `t_bs`; with `t_bs = 0` or a fix in `base_frame`, with `updatePosition` as before.
  The first fix seeds the position as `p_antenna − R(q₀) t_bs`, `q₀` being the attitude
  the filter will start from. With `t_bs ≠ 0` the roll/pitch variance at start and before
  every reseed comes from the accelerometer (§4, `Eskf::tiltVarianceFromAccel`); if the mount
  is not yet known when the filter starts (a start from `/initialpose`), the attitude
  covariance and the attitude are replaced at the first fix (roll and pitch from the accelerometer
  at the heading the filter has), unless the lidar has corrected the filter since the start;
  a fix at a lever arm that arrives before the first IMU sample (a start from `/initialpose`) is held back until it.

`/diagnostics` adds, after the existing values: `heading_source` (`lidar`, `initialpose`, `motion` or
`none`: the last heading source; `motion` = the kinematic alignment), `heading_sigma_rad` (the spread the filter holds about the vertical,
`headingSigmaHonest()`: it grows while the bound hides it), `heading_sigma_used_rad` (the bounded one the update uses),
`lever_blind` (`true`, `false`, `n/a`),
`heading_capped` (`true` while the stretch of lever updates that ended now was bounded, only with
fixes younger than 2 s), `lever_release` (`open`, `closed`, `n/a` without a lever arm) and
`gnss_only_s` (seconds since the last heading source or since the start), `gnss_fix_period_s` (the median interval of the last lever-arm
fixes, `n/a` before the third) and `heading_align_rate` (`supported`, `degraded`, `unsupported`; `n/a` before the sixth fix and for a zero
lever). With a lever arm, the bound
active for 30 s and no heading source for 30 s the status is WARN `heading not observed: ...`
(README, "What you see on /diagnostics").

An input whose frame is empty or has no transform is not used, and `/diagnostics` says so
(WARN; ERROR for the IMU once `startup_timeout_s` has passed). Earlier versions ignored
the frames and used every input as measured at `base_frame`.

## 9. Parameters (fusion3d)

`map_pcd_path`, `ndt_resolution`(1.0), `ndt_step_size`(0.1), `ndt_epsilon`(0.01),
`ndt_max_iter`(30), `points_voxel_leaf`(0.5), `ndt_max_fitness`(2.0),
`imu_topic`/`points_topic`/`gnss_topic`, `global_frame`/`odom_frame`/`base_frame`,
`points_frame`/`imu_frame`/`gnss_frame`(empty = `header.frame_id`),
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
- **updatePositionLever:** Jacobian against finite differences; the update equals the
  consider gain with the Joseph covariance for every mask of `LeverConsider` (against a gain
  written out independently); the default leaves out exactly the parts of the heading and the
  vertical gyro bias along the vertical and corrects the rest; the first version's mask leaves
  attitude and gyro bias untouched bit for bit; the motion gate (speed and rate thresholds, both
  sides, the 3σ rule, each test switched off by 0, the mask used while moving); the iterated
  update relinearises; `attitudeCovariance`, `tiltVarianceFromAccel`,
  `replaceAttitudeCovariance`; closed-form position gain with a position–heading covariance
  (fixes the sign of the Jacobian); zero lever equals `updatePosition` bit for bit, for every
  setting of mask and iterations; GNSS-only for a minute (standing, turning, two lever arms, three
  gyro biases up to 0.5 deg/s, 8 seeds, heading 0.1 rad off): height ≤ 0.1 m, tilt ≤ 0.03 rad,
  heading = 0.1 rad + vertical bias × 60 s within 0.05 rad, horizontal error within 3σ and 0.6 m,
  position NEES between 0.2 and 6 (for the two smaller biases).
- **integration:** synthetic straight+turn trajectory emitting IMU (bias+noise) +
  periodic NDT pose + GNSS; ESKF tracks ground truth ≤0.3 m, ≤0.1 rad.

`prism_loc_fusion_ros/test`: `test_tf_compose` (map→odom = map→base · inv(odom→base),
SE(3)); `test_extrinsic` (sensor mounts of §8.1 against closed forms; the identity mount
returns its input bit for bit); `test_mount` (which mount applies to a message: every
state and transition of the table in `mount.hpp`); `test_ndt_registration` (register a cloud onto a translated copy → recover
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
- Build env: core via system toolchain; ROS packages via a ROS 2 Humble
  environment (`colcon build`).

## 12. Key references (corpus)

hdl_localization (UKF IMU+NDT); robot_localization + navsat_transform (EKF+GPS);
Barfoot, *State Estimation for Robotics* (ESKF/Lie groups); Solà, *Quaternion
kinematics for the error-state KF*; LIO-SAM / FAST_LIO_LOCALIZATION (IMU+LiDAR+GNSS
lineage); PCL NDT (registration engine); LiDAR_IMU_Init (extrinsics).
