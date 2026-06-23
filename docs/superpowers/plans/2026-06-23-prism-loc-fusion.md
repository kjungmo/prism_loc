# prism_loc fusion backend (`fusion3d`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Add a 3D LiDAR + IMU + RTK-GNSS localization backend to prism_loc via an error-state Kalman filter that fuses IMU prediction, PCL-NDT pose correction, and GNSS position correction, publishing `map→odom`.

**Architecture:** A new pure-C++/Eigen package `prism_loc_fusion` holds the 15-state ESKF + so(3) ops + WGS84 geodetic→ENU conversion (no ROS/PCL, unit-tested with the system toolchain). A new ROS 2 package `prism_loc_fusion_ros` runs the `fusion3d` node: IMU→predict, PCL `NormalDistributionsTransform`→pose update, NavSatFix→ENU→position update. Neither existing package (`prism_loc_core`, `prism_loc`) is modified.

**Tech Stack:** C++17, Eigen3, GoogleTest, ament_cmake, rclcpp, tf2_ros, sensor_msgs/nav_msgs/geometry_msgs, PCL (registration/filters/io), pcl_conversions.

## Global Constraints

- ROS 2 **Humble**, C++ **17**, **Apache-2.0** (`<license>Apache-2.0</license>`).
- `prism_loc_fusion`: **no rclcpp, no PCL** — Eigen3 + GoogleTest only; builds with system gcc/cmake outside any ROS env.
- ESKF uses a **single consistent local (body/right) orientation-error convention**: nominal `q` is body→map; error injected as `q ⊗ Exp(δθ)`; pose residual `δθ = Log(qᵀ·q_meas)`; predict block `F(δθ,δθ)=Exp(ω·dt)ᵀ`, `F(δv,δθ)=−R·skew(a)·dt`. Do not mix in a global-error block.
- Quaternions kept normalized after every integration/injection.
- Output TF is **`map→odom`** (REP-105): `T_map_odom = T_map_base · inv(T_odom_base)`; broadcast `map→base_link` only when no `odom→base` exists (with a warning).
- All randomness via the package `Rng` (seedable mt19937_64). Deterministic tests.
- No "Generated with Claude"/Co-Authored-By lines. TDD: failing test → see fail → implement → see pass → commit. One commit per task.
- Dev location: `/home/cona/kangj/prism_loc_fusion_dev/{prism_loc_fusion,prism_loc_fusion_ros}`. Core build: plain CMake (system toolchain). ROS build: workspace `/home/cona/kangj/prism_loc_fusion_ws` with both packages symlinked into `src/`, built in a clean env: `env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH /home/cona/.local/bin/micromamba run -n ros2_humble bash -c '<cmd>'`.

## File Structure

```
prism_loc_fusion/                        # Tasks 0–4 (no ROS, no PCL)
  package.xml  CMakeLists.txt
  include/prism_loc_fusion/{version,rng,so3,types,geodetic,eskf}.hpp
  src/{so3,geodetic,eskf}.cpp
  test/{test_smoke,test_so3,test_geodetic,test_eskf,test_eskf_integration}.cpp
prism_loc_fusion_ros/                    # Tasks 5–8 (ROS 2 node)
  package.xml  CMakeLists.txt
  include/prism_loc_fusion_ros/{tf_util,ndt_registration,fusion_localization_node}.hpp
  src/{ndt_registration,fusion_localization_node,fusion_main}.cpp
  launch/fusion3d.launch.py  params/fusion3d.yaml
  test/{test_tf_compose,test_ndt_registration}.cpp
```

---

## Task 0: `prism_loc_fusion` package skeleton + smoke test

**Files:** Create `prism_loc_fusion/{package.xml,CMakeLists.txt,include/prism_loc_fusion/version.hpp,test/test_smoke.cpp}`.

**Interfaces:** Produces a buildable `prism_loc_fusion` library + ctest harness gated by `-DPRISM_LOC_FUSION_BUILD_TESTS=ON`; macro `PRISM_LOC_FUSION_VERSION="0.1.0"`.

- [ ] **Step 1: smoke test** — `prism_loc_fusion/test/test_smoke.cpp`:
```cpp
#include <gtest/gtest.h>
#include "prism_loc_fusion/version.hpp"
TEST(Smoke, Version) { EXPECT_STREQ(PRISM_LOC_FUSION_VERSION, "0.1.0"); }
```

- [ ] **Step 2: version header** — `prism_loc_fusion/include/prism_loc_fusion/version.hpp`:
```cpp
#pragma once
#define PRISM_LOC_FUSION_VERSION "0.1.0"
```

- [ ] **Step 3: `package.xml`**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>prism_loc_fusion</name>
  <version>0.1.0</version>
  <description>Pure C++/Eigen error-state Kalman filter + geodetic core for prism_loc fusion (no ROS, no PCL).</description>
  <maintainer email="kangjmo91@gmail.com">Kang Jung Mo</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>eigen</depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 4: `CMakeLists.txt`** (dual-mode ament / plain-CTest, same pattern as prism_loc_core):
```cmake
cmake_minimum_required(VERSION 3.16)
project(prism_loc_fusion VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

add_library(prism_loc_fusion
  src/so3.cpp
  src/geodetic.cpp
  src/eskf.cpp)
target_include_directories(prism_loc_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(prism_loc_fusion PUBLIC Eigen3::Eigen)
target_compile_features(prism_loc_fusion PUBLIC cxx_std_17)

option(PRISM_LOC_FUSION_BUILD_TESTS "Build prism_loc_fusion tests" OFF)
if(BUILD_TESTING OR PRISM_LOC_FUSION_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  set(PRISM_LOC_FUSION_TESTS
    test/test_smoke.cpp
    test/test_so3.cpp
    test/test_geodetic.cpp
    test/test_eskf.cpp
    test/test_eskf_integration.cpp)
  add_executable(prism_loc_fusion_tests ${PRISM_LOC_FUSION_TESTS})
  target_link_libraries(prism_loc_fusion_tests prism_loc_fusion GTest::gtest GTest::gtest_main)
  add_test(NAME prism_loc_fusion_tests COMMAND prism_loc_fusion_tests)
endif()

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS prism_loc_fusion EXPORT export_prism_loc_fusion
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
  ament_export_targets(export_prism_loc_fusion HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
```

- [ ] **Step 5:** Until later tasks land, create empty-but-valid stub `.cpp` for `src/{so3,geodetic,eskf}.cpp` (each `#include`-ing its header) and a `#pragma once` placeholder header for `{so3,geodetic,eskf}.hpp` so the library links. Each real task replaces the stub.

- [ ] **Step 6: build + run smoke**:
```bash
cd /home/cona/kangj/prism_loc_fusion_dev
cmake -S prism_loc_fusion -B build/fusion -DPRISM_LOC_FUSION_BUILD_TESTS=ON
cmake --build build/fusion -j
( cd build/fusion && ctest --output-on-failure )
```
Expected: builds; `Smoke.Version` passes.

- [ ] **Step 7: commit** — `git ... -m "feat(fusion): package skeleton + smoke test"` *(commit only after integration into the prism_loc repo; during dev in the standalone dir, skip git)*.

---

## Task 1: `so3.hpp` (skew/Exp/Log) + `rng.hpp` + `types.hpp`

**Files:** Create `include/prism_loc_fusion/{so3,rng,types}.hpp`, `src/so3.cpp`, `test/test_so3.cpp`.

**Interfaces:**
- `Eigen::Matrix3d skew(const Eigen::Vector3d&)`
- `Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi)` (rotation-vector→quat, small-angle safe)
- `Eigen::Vector3d so3Log(const Eigen::Quaterniond& q)` (quat→rotation-vector)
- `class Rng` (uniform/gaussian/seed)
- `struct NominalState { Eigen::Vector3d p,v; Eigen::Quaterniond q; Eigen::Vector3d ba,bg; }`
- `struct EskfParams { double sigma_acc,sigma_gyro,sigma_acc_bias,sigma_gyro_bias; Eigen::Vector3d gravity; }`

- [ ] **Step 1: failing test** — `prism_loc_fusion/test/test_so3.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/so3.hpp"
using namespace prism_loc_fusion;

TEST(So3, SkewIsCross) {
  Eigen::Vector3d a(1, 2, 3), b(-4, 5, 6);
  EXPECT_TRUE((skew(a) * b).isApprox(a.cross(b), 1e-12));
}
TEST(So3, ExpZeroIsIdentity) {
  Eigen::Quaterniond q = so3Exp(Eigen::Vector3d::Zero());
  EXPECT_NEAR(q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}
TEST(So3, LogExpRoundTrip) {
  for (const Eigen::Vector3d phi : {Eigen::Vector3d(0.01, -0.02, 0.005),
                                    Eigen::Vector3d(0.3, 0.4, -0.5),
                                    Eigen::Vector3d(1.2, 0.0, 0.0)}) {
    Eigen::Vector3d out = so3Log(so3Exp(phi));
    EXPECT_TRUE(out.isApprox(phi, 1e-9)) << "phi=" << phi.transpose() << " out=" << out.transpose();
  }
}
TEST(So3, ExpMatchesAngleAxis) {
  Eigen::Vector3d phi(0.0, 0.0, M_PI / 3);
  Eigen::Matrix3d R = so3Exp(phi).toRotationMatrix();
  Eigen::Matrix3d Rref(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(R.isApprox(Rref, 1e-9));
}
```

- [ ] **Step 2: run → FAIL** (add `test/test_so3.cpp` already in CMake list; build).

- [ ] **Step 3: `rng.hpp`**:
```cpp
#pragma once
#include <cstdint>
#include <random>
namespace prism_loc_fusion {
class Rng {
 public:
  explicit Rng(std::uint64_t seed = 42u) : gen_(seed) {}
  void seed(std::uint64_t s) { gen_.seed(s); }
  double uniform(double lo = 0.0, double hi = 1.0) { return std::uniform_real_distribution<double>(lo, hi)(gen_); }
  double gaussian(double m = 0.0, double s = 1.0) { return std::normal_distribution<double>(m, s)(gen_); }
  std::mt19937_64& engine() { return gen_; }
 private:
  std::mt19937_64 gen_;
};
}  // namespace prism_loc_fusion
```

- [ ] **Step 4: `types.hpp`**:
```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_fusion {
struct NominalState {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
};
struct EskfParams {
  double sigma_acc{1e-2};
  double sigma_gyro{1e-3};
  double sigma_acc_bias{1e-4};
  double sigma_gyro_bias{1e-5};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.81)};
};
}  // namespace prism_loc_fusion
```

- [ ] **Step 5: `so3.hpp`**:
```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace prism_loc_fusion {
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi);
Eigen::Vector3d so3Log(const Eigen::Quaterniond& q);
}  // namespace prism_loc_fusion
```

- [ ] **Step 6: `so3.cpp`**:
```cpp
#include "prism_loc_fusion/so3.hpp"
#include <algorithm>
#include <cmath>
namespace prism_loc_fusion {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return m;
}

Eigen::Quaterniond so3Exp(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-8) {
    Eigen::Quaterniond q(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z());
    q.normalize();
    return q;
  }
  const Eigen::Vector3d axis = phi / theta;
  const double h = 0.5 * theta, s = std::sin(h);
  return Eigen::Quaterniond(std::cos(h), axis.x() * s, axis.y() * s, axis.z() * s);
}

Eigen::Vector3d so3Log(const Eigen::Quaterniond& q_in) {
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;  // shortest path
  const Eigen::Vector3d v(q.x(), q.y(), q.z());
  const double n = v.norm();
  if (n < 1e-8) return 2.0 * v;
  const double w = std::min(1.0, std::max(-1.0, q.w()));
  const double theta = 2.0 * std::atan2(n, w);
  return (theta / n) * v;
}

}  // namespace prism_loc_fusion
```

- [ ] **Step 7: run → PASS.** Commit-equivalent: `feat(fusion): so(3) ops, RNG, ESKF state types`.

---

## Task 2: `geodetic.hpp` — WGS84 LLA→ENU

**Files:** Create `include/prism_loc_fusion/geodetic.hpp`, `src/geodetic.cpp`, `test/test_geodetic.cpp`.

**Interfaces:**
- `struct GeoPoint { double lat_deg, lon_deg, alt_m; }`
- `class GeodeticConverter { void setDatum(const GeoPoint&); bool hasDatum() const; GeoPoint datum() const; Eigen::Vector3d toEnu(const GeoPoint&) const; }` — ENU metres (E,N,U) about the datum.

- [ ] **Step 1: failing test** — `prism_loc_fusion/test/test_geodetic.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/geodetic.hpp"
using namespace prism_loc_fusion;

TEST(Geodetic, DatumMapsToZero) {
  GeodeticConverter g; g.setDatum({37.0, 127.0, 0.0});
  Eigen::Vector3d e = g.toEnu({37.0, 127.0, 0.0});
  EXPECT_LT(e.norm(), 1e-6);
}
TEST(Geodetic, AltitudeIsUp) {
  GeodeticConverter g; g.setDatum({37.0, 127.0, 0.0});
  Eigen::Vector3d e = g.toEnu({37.0, 127.0, 10.0});
  EXPECT_NEAR(e.x(), 0.0, 1e-3);
  EXPECT_NEAR(e.y(), 0.0, 1e-3);
  EXPECT_NEAR(e.z(), 10.0, 1e-2);
}
TEST(Geodetic, EastAndNorthSignsAndScale) {
  const double lat0 = 37.0;
  GeodeticConverter g; g.setDatum({lat0, 127.0, 0.0});
  const double dlon = 0.001;  // deg
  Eigen::Vector3d e = g.toEnu({lat0, 127.0 + dlon, 0.0});
  EXPECT_GT(e.x(), 0.0);                 // east
  EXPECT_LT(std::abs(e.y()), 0.5);       // ~no north
  EXPECT_LT(std::abs(e.z()), 0.5);       // ~no up
  const double expected_east = 6378137.0 * std::cos(lat0 * M_PI / 180.0) * (dlon * M_PI / 180.0);
  EXPECT_NEAR(e.x(), expected_east, 0.02 * expected_east);
  Eigen::Vector3d n = g.toEnu({lat0 + dlon, 127.0, 0.0});
  EXPECT_GT(n.y(), 0.0);                 // north
  EXPECT_LT(std::abs(n.x()), 0.5);
}
```

- [ ] **Step 2: run → FAIL.**

- [ ] **Step 3: `geodetic.hpp`**:
```cpp
#pragma once
#include <Eigen/Core>
namespace prism_loc_fusion {
struct GeoPoint { double lat_deg{0.0}; double lon_deg{0.0}; double alt_m{0.0}; };
class GeodeticConverter {
 public:
  void setDatum(const GeoPoint& d);
  bool hasDatum() const { return has_datum_; }
  GeoPoint datum() const { return datum_; }
  Eigen::Vector3d toEnu(const GeoPoint& p) const;
 private:
  static Eigen::Vector3d llaToEcef(const GeoPoint& p);
  bool has_datum_{false};
  GeoPoint datum_;
  Eigen::Vector3d ecef0_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_ecef_to_enu_{Eigen::Matrix3d::Identity()};
};
}  // namespace prism_loc_fusion
```

- [ ] **Step 4: `geodetic.cpp`**:
```cpp
#include "prism_loc_fusion/geodetic.hpp"
#include <cmath>
namespace prism_loc_fusion {
namespace {
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

Eigen::Vector3d GeodeticConverter::llaToEcef(const GeoPoint& p) {
  const double lat = p.lat_deg * kDeg2Rad, lon = p.lon_deg * kDeg2Rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);
  const double N = kA / std::sqrt(1.0 - kE2 * slat * slat);
  return Eigen::Vector3d((N + p.alt_m) * clat * clon,
                         (N + p.alt_m) * clat * slon,
                         (N * (1.0 - kE2) + p.alt_m) * slat);
}

void GeodeticConverter::setDatum(const GeoPoint& d) {
  datum_ = d; has_datum_ = true; ecef0_ = llaToEcef(d);
  const double lat = d.lat_deg * kDeg2Rad, lon = d.lon_deg * kDeg2Rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);
  R_ecef_to_enu_ <<        -slon,         clon,   0.0,
                    -slat * clon, -slat * slon,  clat,
                     clat * clon,  clat * slon,  slat;
}

Eigen::Vector3d GeodeticConverter::toEnu(const GeoPoint& p) const {
  if (!has_datum_) return Eigen::Vector3d::Zero();
  return R_ecef_to_enu_ * (llaToEcef(p) - ecef0_);
}

}  // namespace prism_loc_fusion
```

- [ ] **Step 5: run → PASS.** Commit-equivalent: `feat(fusion): WGS84 geodetic LLA->ENU converter`.

---

## Task 3: `eskf.hpp` — the error-state Kalman filter

**Files:** Create `include/prism_loc_fusion/eskf.hpp`, `src/eskf.cpp`, `test/test_eskf.cpp`.

**Interfaces:**
- `class Eskf` with `Mat15 = Matrix<double,15,15>`, `Vec15 = Matrix<double,15,1>`:
  - `explicit Eskf(const EskfParams& = {})`
  - `void initialize(const NominalState& x0, const Mat15& P0)`
  - `void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt)`
  - `void updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas, const Eigen::Matrix<double,6,6>& R)`
  - `void updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R)`
  - `const NominalState& state() const`, `const Mat15& covariance() const`
- Error order `[δp δv δθ δba δbg]`. Local orientation error (per Global Constraints).

- [ ] **Step 1: failing test** — `prism_loc_fusion/test/test_eskf.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/so3.hpp"
using namespace prism_loc_fusion;

static Eskf::Mat15 initCov(double s) { return Eskf::Mat15::Identity() * s; }

TEST(Eskf, StaticImuStaysPut) {
  Eskf f;                                   // level, zero bias
  f.initialize(NominalState{}, initCov(0.01));
  const Eigen::Vector3d acc(0, 0, 9.81);    // specific force at rest (g cancels)
  for (int i = 0; i < 1000; ++i) f.predict(acc, Eigen::Vector3d::Zero(), 0.01);
  EXPECT_LT(f.state().p.norm(), 1e-6);
  EXPECT_LT(f.state().v.norm(), 1e-6);
}

TEST(Eskf, ConstantAccelKinematics) {
  Eskf f; f.initialize(NominalState{}, initCov(0.01));
  const Eigen::Vector3d acc(1.0, 0.0, 9.81);  // world a_w = [1,0,0]
  for (int i = 0; i < 100; ++i) f.predict(acc, Eigen::Vector3d::Zero(), 0.01);  // t=1s
  EXPECT_NEAR(f.state().v.x(), 1.0, 1e-6);
  EXPECT_NEAR(f.state().p.x(), 0.5, 0.02);
}

TEST(Eskf, PositionUpdatePullsToMeasurement) {
  Eskf f;
  NominalState x0; x0.p = Eigen::Vector3d(5, 5, 5);
  f.initialize(x0, initCov(1.0));
  for (int i = 0; i < 50; ++i)
    f.updatePosition(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.01);
  EXPECT_LT(f.state().p.norm(), 0.05);
}

TEST(Eskf, PoseUpdateCorrectsPositionAndYaw) {
  Eskf f; f.initialize(NominalState{}, initCov(1.0));
  const Eigen::Vector3d p_meas(1.0, 2.0, 0.0);
  const Eigen::Quaterniond q_meas = so3Exp(Eigen::Vector3d(0, 0, 0.5));
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  for (int i = 0; i < 50; ++i) f.updatePose(p_meas, q_meas, R);
  EXPECT_LT((f.state().p - p_meas).norm(), 0.05);
  EXPECT_NEAR(so3Log(f.state().q.conjugate() * q_meas).norm(), 0.0, 0.05);
}
```

- [ ] **Step 2: run → FAIL.**

- [ ] **Step 3: `eskf.hpp`**:
```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/types.hpp"
namespace prism_loc_fusion {
class Eskf {
 public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;
  explicit Eskf(const EskfParams& params = {}) : params_(params) { P_.setIdentity(); P_ *= 0.01; }
  void initialize(const NominalState& x0, const Mat15& P0);
  void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt);
  void updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas,
                  const Eigen::Matrix<double, 6, 6>& R);
  void updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R);
  const NominalState& state() const { return x_; }
  const Mat15& covariance() const { return P_; }
 private:
  void inject(const Vec15& dx);
  NominalState x_;
  Mat15 P_{Mat15::Identity()};
  EskfParams params_;
};
}  // namespace prism_loc_fusion
```

- [ ] **Step 4: `eskf.cpp`** (local-error convention throughout):
```cpp
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/so3.hpp"
namespace prism_loc_fusion {

void Eskf::initialize(const NominalState& x0, const Mat15& P0) {
  x_ = x0; x_.q.normalize(); P_ = P0;
}

void Eskf::predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d a = accel - x_.ba;
  const Eigen::Vector3d w = gyro - x_.bg;
  const Eigen::Vector3d a_w = R * a + params_.gravity;

  // nominal integration
  x_.p += x_.v * dt + 0.5 * a_w * dt * dt;
  x_.v += a_w * dt;
  x_.q = (x_.q * so3Exp(w * dt)).normalized();

  // error-state transition (local orientation error)
  Mat15 F = Mat15::Identity();
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  F.block<3, 3>(0, 3) = I3 * dt;                                   // dp / dv
  F.block<3, 3>(3, 6) = -R * skew(a) * dt;                         // dv / dtheta
  F.block<3, 3>(3, 9) = -R * dt;                                   // dv / dba
  F.block<3, 3>(6, 6) = so3Exp(w * dt).toRotationMatrix().transpose();  // dtheta / dtheta
  F.block<3, 3>(6, 12) = -I3 * dt;                                 // dtheta / dbg

  // process noise
  Mat15 Q = Mat15::Zero();
  const double sa2 = params_.sigma_acc * params_.sigma_acc;
  const double sg2 = params_.sigma_gyro * params_.sigma_gyro;
  const double sba2 = params_.sigma_acc_bias * params_.sigma_acc_bias;
  const double sbg2 = params_.sigma_gyro_bias * params_.sigma_gyro_bias;
  Q.block<3, 3>(3, 3) = I3 * sa2 * dt * dt;
  Q.block<3, 3>(6, 6) = I3 * sg2 * dt * dt;
  Q.block<3, 3>(9, 9) = I3 * sba2 * dt;
  Q.block<3, 3>(12, 12) = I3 * sbg2 * dt;

  P_ = F * P_ * F.transpose() + Q;
}

void Eskf::updatePosition(const Eigen::Vector3d& p_meas, const Eigen::Matrix3d& R) {
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d y = p_meas - x_.p;
  const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
  const Vec15 dx = K * y;
  P_ = (Mat15::Identity() - K * H) * P_;
  inject(dx);
}

void Eskf::updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas,
                      const Eigen::Matrix<double, 6, 6>& R) {
  Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();   // position
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();   // orientation (local)
  Eigen::Matrix<double, 6, 1> y;
  y.head<3>() = p_meas - x_.p;
  y.tail<3>() = so3Log(x_.q.conjugate() * q_meas.normalized());
  const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * S.inverse();
  const Vec15 dx = K * y;
  P_ = (Mat15::Identity() - K * H) * P_;
  inject(dx);
}

void Eskf::inject(const Vec15& dx) {
  x_.p += dx.segment<3>(0);
  x_.v += dx.segment<3>(3);
  x_.q = (x_.q * so3Exp(dx.segment<3>(6))).normalized();
  x_.ba += dx.segment<3>(9);
  x_.bg += dx.segment<3>(12);
}

}  // namespace prism_loc_fusion
```

- [ ] **Step 5: run → PASS** (`Eskf.*`). Commit-equivalent: `feat(fusion): 15-state error-state Kalman filter`.

---

## Task 4: ESKF fusion integration test

**Files:** Create `test/test_eskf_integration.cpp`.

**Interfaces:** none new; exercises predict + updatePose + updatePosition together on a synthetic trajectory.

- [ ] **Step 1: failing test** — `prism_loc_fusion/test/test_eskf_integration.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
using namespace prism_loc_fusion;

// Ground truth: constant velocity along +x, level, no rotation. The ESKF starts
// with the WRONG velocity (zero) and must recover it from GNSS position + NDT pose
// fixes, demonstrating IMU-prediction + measurement fusion (position innovations
// correct velocity through the predicted P(dp,dv) cross-covariance).
TEST(EskfIntegration, TracksMovingTargetFromWrongInit) {
  const double v0 = 1.0;                 // m/s
  const double imu_dt = 0.01;            // 100 Hz
  const int meas_every = 10;             // measurement at 10 Hz
  const int steps = 500;                 // 5 s
  Rng rng(2024);

  Eskf f;
  NominalState x0;                       // p=0, v=0 (WRONG: true v=1), q=I
  f.initialize(x0, Eskf::Mat15::Identity() * 0.5);

  const Eigen::Vector3d acc(0.0, 0.0, 9.81);   // a_w = 0 (constant velocity)
  const Eigen::Matrix3d Rpos = Eigen::Matrix3d::Identity() * (0.05 * 0.05);
  Eigen::Matrix<double, 6, 6> Rpose = Eigen::Matrix<double, 6, 6>::Identity();
  Rpose.topLeftCorner<3, 3>() *= (0.02 * 0.02);
  Rpose.bottomRightCorner<3, 3>() *= (0.01 * 0.01);

  for (int k = 1; k <= steps; ++k) {
    f.predict(acc + Eigen::Vector3d(rng.gaussian(0, 0.01), rng.gaussian(0, 0.01), rng.gaussian(0, 0.01)),
              Eigen::Vector3d(rng.gaussian(0, 0.001), rng.gaussian(0, 0.001), rng.gaussian(0, 0.001)),
              imu_dt);
    if (k % meas_every == 0) {
      const double t = k * imu_dt;
      const Eigen::Vector3d truth(v0 * t, 0.0, 0.0);
      f.updatePosition(truth + Eigen::Vector3d(rng.gaussian(0, 0.05), rng.gaussian(0, 0.05), rng.gaussian(0, 0.05)), Rpos);
      f.updatePose(truth + Eigen::Vector3d(rng.gaussian(0, 0.02), rng.gaussian(0, 0.02), rng.gaussian(0, 0.02)),
                   Eigen::Quaterniond::Identity(), Rpose);
    }
  }
  const Eigen::Vector3d truth_final(v0 * steps * imu_dt, 0.0, 0.0);  // [5,0,0]
  EXPECT_NEAR(f.state().p.x(), truth_final.x(), 0.30);
  EXPECT_LT(std::abs(f.state().p.y()), 0.30);
  EXPECT_NEAR(f.state().v.x(), v0, 0.30);
}
```

- [ ] **Step 2: run → FAIL** (compile until present), then **PASS** once the prior tasks are in. If the velocity estimate lags, raise `meas_every` rate or `steps`, or lower measurement noise — do NOT widen tolerances past 0.30. Commit-equivalent: `test(fusion): IMU+GNSS+NDT fusion integration`.

---

## Task 5: `prism_loc_fusion_ros` skeleton + SE(3) TF-math test

**Files:** Create `prism_loc_fusion_ros/{package.xml,CMakeLists.txt,include/prism_loc_fusion_ros/tf_util.hpp,test/test_tf_compose.cpp}`.

**Interfaces:** `Eigen::Isometry3d computeMapToOdom(const Eigen::Isometry3d& map_base, const Eigen::Isometry3d& odom_base)` = `map_base · odom_base⁻¹`.

- [ ] **Step 1: `tf_util.hpp`**:
```cpp
#pragma once
#include <Eigen/Geometry>
namespace prism_loc_fusion_ros {
inline Eigen::Isometry3d computeMapToOdom(const Eigen::Isometry3d& map_base,
                                          const Eigen::Isometry3d& odom_base) {
  return map_base * odom_base.inverse();
}
}  // namespace prism_loc_fusion_ros
```

- [ ] **Step 2: failing test** — `prism_loc_fusion_ros/test/test_tf_compose.cpp`:
```cpp
#include <gtest/gtest.h>
#include "prism_loc_fusion_ros/tf_util.hpp"
using namespace prism_loc_fusion_ros;
TEST(TfCompose, MapOdomRoundTrip) {
  Eigen::Isometry3d map_base = Eigen::Isometry3d::Identity();
  map_base.translate(Eigen::Vector3d(2, -1, 0.5));
  map_base.rotate(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
  odom_base.translate(Eigen::Vector3d(5, 5, 0));
  odom_base.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  Eigen::Isometry3d map_odom = computeMapToOdom(map_base, odom_base);
  EXPECT_TRUE((map_odom * odom_base).isApprox(map_base, 1e-9));
}
```

- [ ] **Step 3: `package.xml`**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>prism_loc_fusion_ros</name>
  <version>0.1.0</version>
  <description>ROS 2 Humble ESKF fusion node (3D LiDAR NDT + IMU + RTK-GNSS) for prism_loc.</description>
  <maintainer email="kangjmo91@gmail.com">Kang Jung Mo</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>prism_loc_fusion</depend>
  <depend>rclcpp</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>eigen</depend>
  <exec_depend>rviz2</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 4: `CMakeLists.txt`**:
```cmake
cmake_minimum_required(VERSION 3.16)
project(prism_loc_fusion_ros VERSION 0.1.0 LANGUAGES CXX)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(prism_loc_fusion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(deps prism_loc_fusion rclcpp tf2 tf2_ros sensor_msgs nav_msgs geometry_msgs pcl_conversions)

add_library(fusion_node SHARED
  src/ndt_registration.cpp
  src/fusion_localization_node.cpp)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(fusion_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(fusion_node ${deps})

add_executable(fusion_node_main src/fusion_main.cpp)
target_link_libraries(fusion_node_main fusion_node)
ament_target_dependencies(fusion_node_main rclcpp)

install(TARGETS fusion_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS fusion_node_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_compose test/test_tf_compose.cpp)
  target_include_directories(test_tf_compose PRIVATE include)
  target_link_libraries(test_tf_compose Eigen3::Eigen)
  ament_add_gtest(test_ndt_registration test/test_ndt_registration.cpp)
  target_link_libraries(test_ndt_registration fusion_node)
  ament_target_dependencies(test_ndt_registration ${deps})
endif()

ament_package()
```

> The implementer creates `src/{ndt_registration,fusion_localization_node,fusion_main}.cpp`, `include/prism_loc_fusion_ros/{ndt_registration,fusion_localization_node}.hpp`, and `test/test_ndt_registration.cpp` as minimal valid stubs now (so this CMakeLists builds), filled in Tasks 6–7. **MANDATORY before first colcon build:** `touch prism_loc_fusion_ros/launch/.gitkeep prism_loc_fusion_ros/params/.gitkeep` (the `install(DIRECTORY launch params)` configure-fails on missing dirs); Task 8 adds the real files.

- [ ] **Step 5: build + test** (workspace: both packages symlinked into `prism_loc_fusion_ws/src`):
```bash
mkdir -p /home/cona/kangj/prism_loc_fusion_ws/src
ln -sfn /home/cona/kangj/prism_loc_fusion_dev/prism_loc_fusion     /home/cona/kangj/prism_loc_fusion_ws/src/prism_loc_fusion
ln -sfn /home/cona/kangj/prism_loc_fusion_dev/prism_loc_fusion_ros /home/cona/kangj/prism_loc_fusion_ws/src/prism_loc_fusion_ros
env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH /home/cona/.local/bin/micromamba run -n ros2_humble \
  bash -c 'cd /home/cona/kangj/prism_loc_fusion_ws && colcon build --symlink-install && \
           colcon test --packages-select prism_loc_fusion_ros --ctest-args -R test_tf_compose && colcon test-result --verbose'
```
Expected: builds; `test_tf_compose` passes.

---

## Task 6: `ndt_registration` — PCL NDT wrapper

**Files:** Create `include/prism_loc_fusion_ros/ndt_registration.hpp`, `src/ndt_registration.cpp`, `test/test_ndt_registration.cpp`.

**Interfaces:**
- `struct NdtResult { Eigen::Isometry3d pose; double fitness; bool converged; }`
- `class NdtRegistration { NdtRegistration(double res,double step,double eps,int max_iter); void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr&); bool hasTarget() const; NdtResult align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, const Eigen::Isometry3d& init) const; }`

- [ ] **Step 1: failing test** — `prism_loc_fusion_ros/test/test_ndt_registration.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "prism_loc_fusion_ros/ndt_registration.hpp"
using namespace prism_loc_fusion_ros;

static pcl::PointCloud<pcl::PointXYZ>::Ptr boxCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>());
  for (double x = -5; x <= 5; x += 0.2) for (double z = 0; z <= 3; z += 0.2) {
    c->emplace_back(x, 5.0, z); c->emplace_back(x, -5.0, z);
  }
  for (double y = -5; y <= 5; y += 0.2) for (double z = 0; z <= 3; z += 0.2) {
    c->emplace_back(5.0, y, z); c->emplace_back(-5.0, y, z);
  }
  for (double x = -5; x <= 5; x += 0.2) for (double y = -5; y <= 5; y += 0.2)
    c->emplace_back(x, y, 0.0);
  return c;
}
TEST(NdtRegistration, RecoversTranslation) {
  auto target = boxCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
  const double shift = 0.4;
  for (const auto& p : *target) source->emplace_back(p.x + shift, p.y, p.z);  // shifted +x
  NdtRegistration reg(1.0, 0.1, 0.01, 50);
  reg.setTarget(target);
  NdtResult r = reg.align(source, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(r.pose.translation().x(), -shift, 0.1);  // source->target ≈ -shift
  EXPECT_LT(std::abs(r.pose.translation().y()), 0.1);
}
```

- [ ] **Step 2: run → FAIL.**

- [ ] **Step 3: `ndt_registration.hpp`**:
```cpp
#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace prism_loc_fusion_ros {
struct NdtResult {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double fitness{1e9};
  bool converged{false};
};
class NdtRegistration {
 public:
  NdtRegistration(double resolution, double step_size, double epsilon, int max_iter);
  void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target);
  bool hasTarget() const { return target_ && !target_->empty(); }
  NdtResult align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                  const Eigen::Isometry3d& init) const;
 private:
  double resolution_, step_size_, epsilon_;
  int max_iter_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
};
}  // namespace prism_loc_fusion_ros
```

- [ ] **Step 4: `ndt_registration.cpp`**:
```cpp
#include "prism_loc_fusion_ros/ndt_registration.hpp"
#include <pcl/registration/ndt.h>
namespace prism_loc_fusion_ros {
NdtRegistration::NdtRegistration(double resolution, double step_size, double epsilon, int max_iter)
    : resolution_(resolution), step_size_(step_size), epsilon_(epsilon), max_iter_(max_iter) {}
void NdtRegistration::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) { target_ = target; }
NdtResult NdtRegistration::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                 const Eigen::Isometry3d& init) const {
  NdtResult r;
  if (!target_ || target_->empty() || !source || source->empty()) return r;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(static_cast<float>(resolution_));
  ndt.setStepSize(step_size_);
  ndt.setTransformationEpsilon(epsilon_);
  ndt.setMaximumIterations(max_iter_);
  ndt.setInputTarget(target_);
  ndt.setInputSource(source);
  pcl::PointCloud<pcl::PointXYZ> out;
  ndt.align(out, init.matrix().cast<float>());
  r.converged = ndt.hasConverged();
  r.fitness = ndt.getFitnessScore();
  r.pose.matrix() = ndt.getFinalTransformation().cast<double>();
  return r;
}
}  // namespace prism_loc_fusion_ros
```

- [ ] **Step 5: run → PASS.** If NDT misses the basin, increase `max_iter`/cloud density or shrink `shift` — keep the recovered-translation assertion at ≤0.1 m.

---

## Task 7: `FusionLocalizationNode` — subs, ESKF wiring, NDT, GNSS, TF

**Files:** Create `include/prism_loc_fusion_ros/fusion_localization_node.hpp`, `src/fusion_localization_node.cpp`, `src/fusion_main.cpp`.

**Interfaces:** `FusionLocalizationNode : rclcpp::Node` constructible from `NodeOptions`; subscribes `/imu`,`/points`,`/gnss`,`/initialpose`; publishes `map→odom` TF (or `map→base_link` fallback), `~/pose`, `~/odometry`.

- [ ] **Step 1: `fusion_localization_node.hpp`**:
```cpp
#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/geodetic.hpp"
#include "prism_loc_fusion_ros/ndt_registration.hpp"
namespace prism_loc_fusion_ros {
class FusionLocalizationNode : public rclcpp::Node {
 public:
  explicit FusionLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
 private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool tryInitialize();
  void publish(const rclcpp::Time& stamp);

  std::string global_frame_, odom_frame_, base_frame_;
  double ndt_max_fitness_, points_voxel_leaf_, transform_tolerance_;
  double pose_pos_std_, pose_rot_std_, gnss_max_pos_cov_, initial_yaw_{0.0};
  int gnss_min_status_{0};

  std::unique_ptr<prism_loc_fusion::Eskf> eskf_;
  prism_loc_fusion::GeodeticConverter geo_;
  std::unique_ptr<NdtRegistration> ndt_;

  bool filter_init_{false}, have_attitude_{false}, have_position_{false}, last_imu_valid_{false};
  rclcpp::Time last_imu_time_;
  Eigen::Quaterniond init_attitude_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d init_position_{Eigen::Vector3d::Zero()};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
};
}  // namespace prism_loc_fusion_ros
```

- [ ] **Step 2: `fusion_localization_node.cpp`**:
```cpp
#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
#include "prism_loc_fusion_ros/tf_util.hpp"
#include "prism_loc_fusion/so3.hpp"
#include <algorithm>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/time.h>
namespace prism_loc_fusion_ros {
using prism_loc_fusion::NominalState;
using Mat15 = prism_loc_fusion::Eskf::Mat15;

static Eigen::Quaterniond attitudeFromAccel(const Eigen::Vector3d& f, double yaw) {
  const double roll = std::atan2(f.y(), f.z());
  const double pitch = std::atan2(-f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
}

FusionLocalizationNode::FusionLocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("prism_loc_fusion", options) {
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  ndt_max_fitness_ = declare_parameter<double>("ndt_max_fitness", 2.0);
  points_voxel_leaf_ = declare_parameter<double>("points_voxel_leaf", 0.5);
  pose_pos_std_ = declare_parameter<double>("pose_pos_std", 0.1);
  pose_rot_std_ = declare_parameter<double>("pose_rot_std", 0.05);
  gnss_min_status_ = declare_parameter<int>("gnss_min_status", 0);
  gnss_max_pos_cov_ = declare_parameter<double>("gnss_max_pos_cov", 25.0);
  initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);

  prism_loc_fusion::EskfParams ep;
  ep.sigma_acc = declare_parameter<double>("sigma_acc", 1e-2);
  ep.sigma_gyro = declare_parameter<double>("sigma_gyro", 1e-3);
  ep.sigma_acc_bias = declare_parameter<double>("sigma_acc_bias", 1e-4);
  ep.sigma_gyro_bias = declare_parameter<double>("sigma_gyro_bias", 1e-5);
  eskf_ = std::make_unique<prism_loc_fusion::Eskf>(ep);

  if (declare_parameter<bool>("use_datum", false)) {
    geo_.setDatum({declare_parameter<double>("datum_lat", 0.0),
                   declare_parameter<double>("datum_lon", 0.0),
                   declare_parameter<double>("datum_alt", 0.0)});
  }

  const std::string pcd = declare_parameter<std::string>("map_pcd_path", "");
  ndt_ = std::make_unique<NdtRegistration>(
      declare_parameter<double>("ndt_resolution", 1.0),
      declare_parameter<double>("ndt_step_size", 0.1),
      declare_parameter<double>("ndt_epsilon", 0.01),
      declare_parameter<int>("ndt_max_iter", 30));
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcd.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *map) < 0 || map->empty()) {
    RCLCPP_ERROR(get_logger(), "fusion3d: failed to load PCD map: %s", pcd.c_str());
  } else {
    ndt_->setTarget(map);
    RCLCPP_INFO(get_logger(), "fusion3d: NDT map loaded (%zu pts)", map->size());
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onImu, this, std::placeholders::_1));
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("points_topic", "/points"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onPoints, this, std::placeholders::_1));
  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      declare_parameter<std::string>("gnss_topic", "/gnss"), rclcpp::SensorDataQoS(),
      std::bind(&FusionLocalizationNode::onGnss, this, std::placeholders::_1));
  initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&FusionLocalizationNode::onInitialPose, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "prism_loc_fusion (fusion3d) up");
}

bool FusionLocalizationNode::tryInitialize() {
  if (filter_init_) return true;
  if (!(have_attitude_ && have_position_)) return false;
  NominalState x0;
  x0.p = init_position_;
  x0.q = init_attitude_;
  Mat15 P0 = Mat15::Identity();
  P0.block<3, 3>(0, 0) *= 1.0;
  P0.block<3, 3>(3, 3) *= 1.0;
  P0.block<3, 3>(6, 6) *= 0.5;
  P0.block<3, 3>(9, 9) *= 1e-2;
  P0.block<3, 3>(12, 12) *= 1e-4;
  eskf_->initialize(x0, P0);
  filter_init_ = true;
  RCLCPP_INFO(get_logger(), "fusion3d: initialized at (%.2f, %.2f, %.2f)", x0.p.x(), x0.p.y(), x0.p.z());
  return true;
}

void FusionLocalizationNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  const Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  if (!have_attitude_) { init_attitude_ = attitudeFromAccel(acc, initial_yaw_); have_attitude_ = true; }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (!last_imu_valid_) { last_imu_time_ = stamp; last_imu_valid_ = true; return; }
  const double dt = (stamp - last_imu_time_).seconds();
  last_imu_time_ = stamp;
  if (!tryInitialize()) return;
  if (dt <= 0.0 || dt > 0.5) return;
  eskf_->predict(acc, gyro, dt);
  publish(stamp);
}

void FusionLocalizationNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (!filter_init_ || !ndt_ || !ndt_->hasTarget()) return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  const float leaf = static_cast<float>(points_voxel_leaf_);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*src);
  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess.translation() = eskf_->state().p;
  guess.linear() = eskf_->state().q.toRotationMatrix();
  NdtResult r = ndt_->align(src, guess);
  if (!r.converged || r.fitness > ndt_max_fitness_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "fusion3d: NDT rejected (conv=%d fit=%.3f)",
                         static_cast<int>(r.converged), r.fitness);
    return;
  }
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
  R.topLeftCorner<3, 3>() *= pose_pos_std_ * pose_pos_std_;
  R.bottomRightCorner<3, 3>() *= pose_rot_std_ * pose_rot_std_;
  eskf_->updatePose(r.pose.translation(), Eigen::Quaterniond(r.pose.linear()), R);
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (msg->status.status < gnss_min_status_) return;
  if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;
  if (msg->position_covariance[0] > gnss_max_pos_cov_) return;
  const prism_loc_fusion::GeoPoint gp{msg->latitude, msg->longitude, msg->altitude};
  if (!geo_.hasDatum()) geo_.setDatum(gp);
  const Eigen::Vector3d enu = geo_.toEnu(gp);
  if (!have_position_) { init_position_ = enu; have_position_ = true; }
  if (!filter_init_) { tryInitialize(); return; }
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0, 0) = std::max(msg->position_covariance[0], 1e-4);
  R(1, 1) = std::max(msg->position_covariance[4], 1e-4);
  R(2, 2) = std::max(msg->position_covariance[8], 1.0);
  eskf_->updatePosition(enu, R);
  publish(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void FusionLocalizationNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  init_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  const auto& o = msg->pose.pose.orientation;
  init_attitude_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
  have_position_ = have_attitude_ = true;
  filter_init_ = false;
  RCLCPP_INFO(get_logger(), "fusion3d: initialpose set");
}

void FusionLocalizationNode::publish(const rclcpp::Time& stamp) {
  if (!filter_init_) return;
  const auto& x = eskf_->state();
  Eigen::Isometry3d map_base = Eigen::Isometry3d::Identity();
  map_base.translation() = x.p;
  map_base.linear() = x.q.toRotationMatrix();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
  tf.header.frame_id = global_frame_;
  Eigen::Isometry3d out_tf;
  std::string child;
  try {
    auto t = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
    odom_base.translation() = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    odom_base.linear() = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x,
                                            t.transform.rotation.y, t.transform.rotation.z).toRotationMatrix();
    out_tf = computeMapToOdom(map_base, odom_base);
    child = odom_frame_;
  } catch (const std::exception&) {
    out_tf = map_base;
    child = base_frame_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "fusion3d: no %s->%s; publishing %s->%s",
                         odom_frame_.c_str(), base_frame_.c_str(), global_frame_.c_str(), base_frame_.c_str());
  }
  tf.child_frame_id = child;
  tf.transform.translation.x = out_tf.translation().x();
  tf.transform.translation.y = out_tf.translation().y();
  tf.transform.translation.z = out_tf.translation().z();
  Eigen::Quaterniond oq(out_tf.linear());
  tf.transform.rotation.x = oq.x(); tf.transform.rotation.y = oq.y();
  tf.transform.rotation.z = oq.z(); tf.transform.rotation.w = oq.w();
  tf_broadcaster_->sendTransform(tf);

  geometry_msgs::msg::PoseWithCovarianceStamped ps;
  ps.header.stamp = stamp; ps.header.frame_id = global_frame_;
  ps.pose.pose.position.x = x.p.x(); ps.pose.pose.position.y = x.p.y(); ps.pose.pose.position.z = x.p.z();
  ps.pose.pose.orientation.x = x.q.x(); ps.pose.pose.orientation.y = x.q.y();
  ps.pose.pose.orientation.z = x.q.z(); ps.pose.pose.orientation.w = x.q.w();
  const auto& P = eskf_->covariance();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      ps.pose.covariance[i * 6 + j] = P(i, j);
      ps.pose.covariance[(i + 3) * 6 + (j + 3)] = P(6 + i, 6 + j);
    }
  pose_pub_->publish(ps);

  nav_msgs::msg::Odometry od;
  od.header.stamp = stamp; od.header.frame_id = global_frame_; od.child_frame_id = base_frame_;
  od.pose = ps.pose;
  od.twist.twist.linear.x = x.v.x(); od.twist.twist.linear.y = x.v.y(); od.twist.twist.linear.z = x.v.z();
  odom_pub_->publish(od);
}

}  // namespace prism_loc_fusion_ros
```

- [ ] **Step 3: `fusion_main.cpp`**:
```cpp
#include "prism_loc_fusion_ros/fusion_localization_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prism_loc_fusion_ros::FusionLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: build (colcon).** Expected: builds clean, gtests still pass.

---

## Task 8: Launch, params + final build/run gate

**Files:** Create `prism_loc_fusion_ros/launch/fusion3d.launch.py`, `prism_loc_fusion_ros/params/fusion3d.yaml`.

- [ ] **Step 1: `params/fusion3d.yaml`**:
```yaml
/**:
  ros__parameters:
    global_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    imu_topic: "/imu"
    points_topic: "/points"
    gnss_topic: "/gnss"
    map_pcd_path: ""
    ndt_resolution: 1.0
    ndt_step_size: 0.1
    ndt_epsilon: 0.01
    ndt_max_iter: 30
    ndt_max_fitness: 2.0
    points_voxel_leaf: 0.5
    pose_pos_std: 0.1
    pose_rot_std: 0.05
    sigma_acc: 0.01
    sigma_gyro: 0.001
    sigma_acc_bias: 0.0001
    sigma_gyro_bias: 0.00001
    use_datum: false
    datum_lat: 0.0
    datum_lon: 0.0
    datum_alt: 0.0
    gnss_min_status: 0
    gnss_max_pos_cov: 25.0
    initial_yaw: 0.0
    transform_tolerance: 0.1
```

- [ ] **Step 2: `launch/fusion3d.launch.py`**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('prism_loc_fusion_ros')
    params = os.path.join(pkg, 'params', 'fusion3d.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd_path', default_value='', description='prior map .pcd'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='prism_loc_fusion_ros', executable='fusion_node_main', name='prism_loc_fusion',
             output='screen',
             parameters=[params, {'map_pcd_path': LaunchConfiguration('map_pcd_path'), 'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
```

- [ ] **Step 3: full clean build + test**:
```bash
env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH /home/cona/.local/bin/micromamba run -n ros2_humble \
  bash -c 'cd /home/cona/kangj/prism_loc_fusion_ws && colcon build --symlink-install && \
           colcon test --packages-select prism_loc_fusion prism_loc_fusion_ros && colcon test-result --verbose'
```
Expected: both packages build; all gtests pass; 0 errors.

- [ ] **Step 4: node smoke** (starts, no PCD needed for the error path):
```bash
env -u ROS_DISTRO -u ROS_VERSION -u ROS_PACKAGE_PATH /home/cona/.local/bin/micromamba run -n ros2_humble \
  bash -c 'source /home/cona/kangj/prism_loc_fusion_ws/install/setup.bash && \
           timeout 5 ros2 run prism_loc_fusion_ros fusion_node_main || true'
```
Expected: logs "prism_loc_fusion (fusion3d) up" then exits on timeout.

---

## Self-Review

**Spec coverage:** SPEC §3 sensors → IMU predict (T3,T7), NDT pose update (T6,T7), GNSS position update (T2,T7); §4 ESKF math (T3); §5 georeferencing (T2,T7); §6 init (T7); §8 I/O (T7); §10 tests (every bullet has a gtest, T1–T7). **Placeholders:** none (Apache LICENSE inherited from the repo root after integration; no per-package LICENSE needed). **Type consistency:** `Eskf` method signatures match between header (T3) and node calls (T7); `NdtResult`/`NdtRegistration` match (T6→T7); `GeodeticConverter` match (T2→T7); local-error convention is consistent across predict/update/inject (T3).

**Known v1 limitations (documented):** gravity-aligned-map assumption; single-antenna RTK → yaw initialized only via LiDAR/initialpose; covariance-reset Jacobian omitted (first-order); no tight LiDAR-IMU time-offset estimation.

## Execution Handoff

Implement via **workflow-orchestration**, models by difficulty:
- **opus** — Tasks 3 (ESKF), 7 (fusion node), and build-failure repair.
- **sonnet** — Tasks 1, 2, 6 (so3/geodetic/NDT wrapper).
- **haiku** — Tasks 0, 5 skeletons, 8 launch/params.
Sequence: core (0→1→2→3→4) gates `prism_loc_fusion`; ros (5→6→7→8) gates `prism_loc_fusion_ros`. Build gates: core via plain CMake (system toolchain); ros via colcon in `ros2_humble` (clean env).


