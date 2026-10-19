// eskf_lever.cpp -- how to fuse a GNSS fix that is taken at a lever arm from base_link.
//
// SYNTHETIC EXPERIMENT. Drives the REAL prism_loc_fusion::Eskf (no copy, no ROS) with a simulated IMU, a simulated GNSS
// antenna at a lever arm from the body origin, and optionally simulated NDT pose fixes. Every scheme sees the identical data
// of a run (common random numbers).
//
// Schemes (this file builds them from the parameters of EskfParams):
//   ignore  the antenna offset is not modelled (what main did before the lever arm): the fix is a position of the body origin
//   S1s     consider states = heading and vertical gyro bias; no bound, no release, no alignment (EskfParams with those rules off)
//   S2s     no consider state: the full update of every state; the same rules off
//   S8u     the code that ships: the EskfParams defaults, and a start with a guessed heading is marked unknown
// The "s" in a name: the roll and pitch variance at the start and at every reseed is taken from the accelerometer
// (Eskf::tiltVarianceFromAccel, about 1e-4 rad^2) instead of 0.5 rad^2. A reseed takes roll and pitch from the accelerometer.
// The schemes of the earlier passes (S0, S4s to S8o, S8k, S8t, S8n, S8v, S8w, S8x), which differ from S8u by switches that the
// core no longer has, were run with this file and the core at commit 2011666. Their rows stay in the committed CSVs; the S1s,
// S2s and S8u rows of the committed CSVs are reproduced bit for bit by this file (experiments/README.md, "Reproducing"). The S8u
// rows of the core before round 9 (0ef1add) are kept in the CSVs as S8u@0ef1add where round 9 changed any row.
//
// Run = one (configuration, seed) pair. A run has DIVERGED if at any 1 s epoch from t = 10 s on: tilt error > 0.05 rad, or
// |z error| > 0.5 m, or horizontal error > 1.0 m, or heading error > 0.5 rad. Epochs from the start of a GNSS dropout to 5 s
// after its end, and the 5 s after a reseed, are not tested. The failures are also counted apart (columns of the runs file;
// experiments/eskf_lever_report.py applies them):
//   tz     tilt > 0.05 rad or |z| > 0.5 m at an epoch outside those windows (the failure a deployer sees: the robot leans, the height runs)
//   circ   max over epochs of  xy - (2 |t_xy| sin(|heading error| / 2) + 0.3 m)  > 0: the horizontal error of base_link beyond the
//          circle of radius |t_xy| around the antenna that a standing robot cannot leave, plus 0.3 m
//   head   heading error > 0.5 rad
//   xy     horizontal error > 1 m
// NEES at 1 s epochs from t = 10 s on, for position (3), attitude (3), the whole 15-state error, tilt (2) and heading (1). The NEES of a
// configuration is the mean over its seeds; its two-sided 95 % band is [chi2_.025(S dof), chi2_.975(S dof)] / S (Wilson-Hilferty).
//
// Simulation: IMU 100 Hz; GNSS fix every Config::gper s (default 0.2 s), noise 0.02 m per axis; NDT pose fix at 5 Hz, offset 0.1 s;
// gyro bias 0 to 0.5 deg/s per axis, accelerometer bias 0 to 0.3 m/s^2 per axis (signs drawn per run); heading error at the start
// 0.1 rad unless the suite says otherwise; antennas A = (0.5, 0, 1.0), B = (-0.3, 0.2, 1.0), Z = (0, 0, 0) (zero lever), M = (0, 0, 1.5) (mast).
// Motions: stand | turn (0.3 rad/s for 4 s of every 5 s) | drive (planar lemniscate) | spin (0.3 rad/s) | line (1.5 m/s) | startstop
// (40 s cycle: still 6 s, 0 -> 1.5 m/s in 3 s, 20 s cruise, brake in 3 s, still 8 s) | sgstart, sgturn, sgdrive (stand for stand_s, then
// startstop, turn, or 0 -> 1.5 m/s and a circle of radius 6 m).
//
// Suites: g60 g300 smoke h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hrate hratex hphase hsmoke.
//   hrate   boot and stand-then-go cells at fix periods of 2, 1, 0.5, 0.2, 0.1 and 0.05 s (0.5 to 20 Hz); the IMU stays at 100 Hz and
//           the data of a cell is the same at every rate. Column noise = fix<period>s.
//   hratex  the same cells at fix periods of 1.2, 1.1, 0.02 and 0.01 s (0.83, 0.91, 50 and 100 Hz), on the data of the hrate cells.
//   hphase  the same cells at 1, 2, 5 and 10 Hz with the fixes 0, 1/10, ..., 9/10 of a fix period after the whole seconds, on which the motion starts
//           (hrate has phase 0 only). Column noise = p<period>i<tenths of the period>.
// Environment: LEVER_ONLY=name,name runs just these schemes; LEVER_TAG=text is appended to every scheme name in the output (to
// label a run against another core build).
//
// Build and run from the repository root (see experiments/README.md):
//   g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3 \
//       prism_loc_fusion/src/*.cpp experiments/eskf_lever.cpp -o /tmp/exp_lever
//   /tmp/exp_lever hrate 0 1 out/lever_hrate      # suite, shard, number of shards, output prefix
// Writes <prefix>_runs.csv (one row per run and scheme) and <prefix>_agg.csv (one row per configuration and scheme).
// Deterministic for one compiler and standard library (std::normal_distribution is implementation defined); the committed results
// are from the system g++ 9.4.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "prism_loc_fusion/eskf.hpp"
#include "prism_loc_fusion/rng.hpp"
#include "prism_loc_fusion/so3.hpp"

using namespace prism_loc_fusion;
using Vec3 = Eigen::Vector3d;

namespace {

// ---- pre-declared constants ----------------------------------------------------------
constexpr double kDt = 0.01;                 // 100 Hz IMU
constexpr double kGnssPeriod = 0.2;          // 5 Hz fix (Config::gper overrides it, suite hrate)
constexpr double kNdtPeriod = 0.2;           // 5 Hz pose fix, offset 0.1 s from the fix
constexpr double kGnssStd = 0.02;            // simulated antenna noise, per axis
constexpr double kSimAcc = 0.02, kSimGyro = 0.002;   // simulated IMU white noise
constexpr double kNdtPos = 0.1, kNdtRot = 0.05;      // NDT std the node assumes (pose_pos_std/rot_std)
constexpr double kYawStart = 0.1;            // heading error at start, rad (sign drawn per run)
constexpr double kTiltDiv = 0.05, kZDiv = 0.5, kXyDiv = 1.0, kHeadDiv = 0.5;
constexpr double kEpochFrom = 10.0;
constexpr std::uint64_t kSeed = 20261018u;
constexpr std::uint64_t kSeedV = 20271019u;   // suite hsgv: the configurations of hsg on other data (a check of the schemes developed on hsg)

const Vec3 kAntenna[4] = {Vec3(0.5, 0.0, 1.0), Vec3(-0.3, 0.2, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 1.5)};
const char* kAntennaName[4] = {"A", "B", "Z", "M"};
const char* kMotionName[9] = {"stand", "turn", "drive", "spin", "line", "startstop", "sgstart", "sgturn", "sgdrive"};

struct Variant {
  const char* name;
  bool ignore;       // do not model the lever arm (main's behaviour)
  unsigned consider;
  bool seeded;       // roll and pitch variance from the accelerometer at the start and at every reseed
  double cap{0.0}, rel_dv{0.0};   // EskfParams::lever_heading_sigma_max (rad), lever_release_dv (m/s); 0 = off
  bool reseed_accel{false};       // a reseed takes roll and pitch from the accelerometer
  double align{0.0};              // EskfParams::lever_align_sigma, rad (0 = no alignment, no blind update)
  double start_sigma{0.0};        // EskfParams::lever_start_sigma, rad (0 = a start with a guessed heading is not marked unknown)
  double period_max{0.0};         // EskfParams::lever_align_period_max, s
  double outage_spread{0.0};      // EskfParams::lever_outage_spread
};
const EskfParams kShipped;
const Variant kVariants[] = {
    {"ignore", true, 0, false},
    {"S1s", false, kConsiderHeading | kConsiderGyroBiasVertical, true},
    {"S2s", false, 0, true},
    {"S8u", false, kShipped.lever_consider, true, kShipped.lever_heading_sigma_max, kShipped.lever_release_dv, true, kShipped.lever_align_sigma,
     kShipped.lever_start_sigma, kShipped.lever_align_period_max, kShipped.lever_outage_spread},
};
constexpr int kNumVariants = sizeof(kVariants) / sizeof(kVariants[0]);

struct Config {
  int id, dur, motion, ant;
  double gbias_dps, abias;
  bool ndt, drop, reseed, matched;
  int nseed;
  // second pass (h suites); the first-pass suites leave the defaults
  int data_id{-1};         // seed of the data: that of the antenna-A row of the configuration (-1: id)
  double drop_s{-1.0};     // dropout length, s (-1: the first pass's 10 s of 60 s runs, 30 s of 300 s runs)
  double slope{0.0};       // body tilt, rad: pitch = slope, roll = -slope / 2
  double yaw0{0.1};        // heading error at the start, rad (sign drawn per run); the first pass: kYawStart
  double sim_acc{0.02};    // accelerometer white noise of the simulated IMU, m/s^2 per axis (kSimAcc); suite hvib: more
  bool hsuite{false};
  double stand_s{0.0};     // third pass (suite hsg, motions sgstart/sgturn/sgdrive): the robot stands for this long, then moves
  double gper{0.2};        // round 7: period of the GNSS fixes, s (the IMU stays at 100 Hz); suite hrate. The default is the 5 Hz of every earlier suite.
  double drop_off{-10.0};  // the outage of a run with a stand begins at stand_s + drop_off (the earlier suites: 10 s before the start of the motion cycle)
  int gphase{0};           // suite hphase: the fixes come this many IMU ticks later than the multiples of the fix period (the start of a motion is at a whole second)
  int gphase_idx{0};       // suite hphase: the phase in tenths of the fix period
};

struct Truth { Vec3 p, v, a; double yaw, wz; };

Truth truthAt(int motion, double t) {
  Truth T{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 0.0, 0.0};
  if (motion == 1) {  // turn on the spot: 0.3 rad/s for 4 s of every 5 s
    const double cycle = std::floor(t / 5.0), r = t - 5.0 * cycle;
    T.yaw = 0.3 * (4.0 * cycle + std::min(r, 4.0));
    T.wz = r < 4.0 ? 0.3 : 0.0;
  } else if (motion == 3) {  // spin on the spot, 0.3 rad/s without pause
    T.yaw = 0.3 * t;
    T.wz = 0.3;
  } else if (motion == 4) {  // straight line, 1.5 m/s along x
    T.p = Vec3(1.5 * t, 0.0, 0.0);
    T.v = Vec3(1.5, 0.0, 0.0);
  } else if (motion == 5) {  // start-stop along x: 40 s cycle, 0 -> 1.5 m/s in 3 s (0.5 m/s^2), 20 s cruise, brake in 3 s
    const double cycle = std::floor(t / 40.0), r = t - 40.0 * cycle;
    double x, v, a;
    if (r < 6.0) { x = 0.0; v = 0.0; a = 0.0; }
    else if (r < 9.0) { const double u = r - 6.0; x = 0.25 * u * u; v = 0.5 * u; a = 0.5; }
    else if (r < 29.0) { x = 2.25 + 1.5 * (r - 9.0); v = 1.5; a = 0.0; }
    else if (r < 32.0) { const double u = r - 29.0; x = 32.25 + 1.5 * u - 0.25 * u * u; v = 1.5 - 0.5 * u; a = -0.5; }
    else { x = 34.5; v = 0.0; a = 0.0; }
    T.p = Vec3(34.5 * cycle + x, 0.0, 0.0);
    T.v = Vec3(v, 0.0, 0.0);
    T.a = Vec3(a, 0.0, 0.0);
  } else if (motion == 6) {  // circle: 0 -> 1.5 m/s along x in 3 s, then a circle of radius 6 m at 1.5 m/s (a = 0.375 m/s^2), left turn
    const double R = 6.0, V = 1.5;
    if (t < 3.0) {
      T.p = Vec3(0.25 * t * t, 0.0, 0.0); T.v = Vec3(0.5 * t, 0.0, 0.0); T.a = Vec3(0.5, 0.0, 0.0);
    } else {
      const double phi = (V / R) * (t - 3.0), x0 = 2.25;
      T.p = Vec3(x0 + R * std::sin(phi), R * (1.0 - std::cos(phi)), 0.0);
      T.v = Vec3(V * std::cos(phi), V * std::sin(phi), 0.0);
      T.a = Vec3(-V * V / R * std::sin(phi), V * V / R * std::cos(phi), 0.0);
      T.yaw = phi; T.wz = V / R;
    }
  } else if (motion == 2) {  // planar lemniscate
    const double A = 8.0, B = 4.0, w = 2.0 * M_PI / 30.0;
    T.p = Vec3(A * std::sin(w * t), B * std::sin(2.0 * w * t), 0.0);
    T.v = Vec3(A * w * std::cos(w * t), 2.0 * B * w * std::cos(2.0 * w * t), 0.0);
    T.a = Vec3(-A * w * w * std::sin(w * t), -4.0 * B * w * w * std::sin(2.0 * w * t), 0.0);
    T.yaw = std::atan2(T.v.y(), T.v.x());
    T.wz = (T.v.x() * T.a.y() - T.v.y() * T.a.x()) / (T.v.x() * T.v.x() + T.v.y() * T.v.y());
  }
  return T;
}

// Truth of a configuration: the motions sg* stand at the origin (heading 0) for stand_s seconds and then
// start the motion they name (startstop, turn, circle) from rest.
Truth truthOf(const Config& c, double t) {
  if (c.motion < 6) return truthAt(c.motion, t);
  if (t < c.stand_s) return Truth{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 0.0, 0.0};
  static const int kBase[3] = {5, 1, 6};
  return truthAt(kBase[c.motion - 6], t - c.stand_s);
}

Eigen::Matrix3d rotZ(double yaw) { return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix(); }
Eigen::Quaterniond quatZ(double yaw) { return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3::UnitZ())); }
// Constant tilt of the body against the horizontal (the mount of a robot on a slope): identity for 0.
Eigen::Matrix3d tiltOf(double slope) {
  if (slope == 0.0) return Eigen::Matrix3d::Identity();
  return (Eigen::AngleAxisd(slope, Vec3::UnitY()) * Eigen::AngleAxisd(-0.5 * slope, Vec3::UnitX())).toRotationMatrix();
}
// Attitude of the body: heading about the vertical, then the tilt (for 0 tilt: the heading rotation itself).
Eigen::Matrix3d bodyRot(const Config& c, double yaw) {
  return c.slope == 0.0 ? rotZ(yaw) : Eigen::Matrix3d(rotZ(yaw) * tiltOf(c.slope));
}
Eigen::Quaterniond bodyQuat(const Config& c, double yaw) {
  return c.slope == 0.0 ? quatZ(yaw) : Eigen::Quaterniond(bodyRot(c, yaw));
}

// The node's attitude from one accelerometer sample (fusion_localization_node.cpp).
Eigen::Quaterniond attitudeFromAccel(const Vec3& f, double yaw) {
  const double roll = std::atan2(f.y(), f.z());
  const double pitch = std::atan2(-f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3::UnitX())).normalized();
}

Vec3 gauss3(Rng& r, double s) { return Vec3(r.gaussian(0, s), r.gaussian(0, s), r.gaussian(0, s)); }
Vec3 signs3(Rng& r) {
  return Vec3(r.uniform() < 0.5 ? -1.0 : 1.0, r.uniform() < 0.5 ? -1.0 : 1.0, r.uniform() < 0.5 ? -1.0 : 1.0);
}

double chi2q(double k, double z) {  // Wilson-Hilferty quantile
  const double c = 2.0 / (9.0 * k);
  const double b = 1.0 - c + z * std::sqrt(c);
  return k * b * b * b;
}

// ---- one run's data (identical for every scheme) -------------------------------------
struct Data {
  std::vector<Vec3> f, w;           // IMU sample k = 0..N
  std::vector<Vec3> gnss_noise;     // per fix
  std::vector<Vec3> ndt_p, ndt_r;   // per pose fix
  Vec3 ba, bg;
  double yaw_err;
  Vec3 reseed_p_off; double reseed_yaw_err;
};

Data makeData(const Config& c, std::uint64_t seed) {
  Rng rng(seed);
  Data d;
  const Eigen::Matrix3d Rt = tiltOf(c.slope);
  const int n = static_cast<int>(std::llround(c.dur / kDt));
  d.ba = c.abias * signs3(rng);
  d.bg = (c.gbias_dps * M_PI / 180.0) * signs3(rng);
  d.yaw_err = c.yaw0 * (rng.uniform() < 0.5 ? -1.0 : 1.0);
  d.reseed_p_off = Vec3(0.04, -0.03, 0.0);
  d.reseed_yaw_err = 0.01;
  d.f.resize(n + 1); d.w.resize(n + 1);
  for (int k = 0; k <= n; ++k) {
    const Truth T = truthOf(c, k * kDt);
    if (c.slope == 0.0) {   // the first pass: exactly the arithmetic it had
      d.f[k] = rotZ(T.yaw).transpose() * (T.a + Vec3(0, 0, 9.81)) + d.ba + gauss3(rng, c.sim_acc);
      d.w[k] = Vec3(0, 0, T.wz) + d.bg + gauss3(rng, kSimGyro);
    } else {
      d.f[k] = bodyRot(c, T.yaw).transpose() * (T.a + Vec3(0, 0, 9.81)) + d.ba + gauss3(rng, c.sim_acc);
      d.w[k] = Rt.transpose() * Vec3(0, 0, T.wz) + d.bg + gauss3(rng, kSimGyro);
    }
  }
  const int ngnss = static_cast<int>(c.dur / c.gper) + 1;
  for (int i = 0; i < ngnss; ++i) {
    d.gnss_noise.push_back(gauss3(rng, kGnssStd));
    d.ndt_p.push_back(gauss3(rng, kNdtPos));
    d.ndt_r.push_back(gauss3(rng, kNdtRot));
  }
  return d;
}

// ---- one scheme on one run -------------------------------------------------------------
struct RunOut {
  bool diverged{false};
  double t_div{-1}, drop_xy{0}, max_xy{0}, max_z{0}, max_tilt{0}, max_head{0}, end_xy{0}, end_z{0}, end_tilt{0}, end_head{0};
  double max_circ{-1e9}, max_sigh{0}, cap_frac{0}, rel_frac{0};   // second pass
  std::vector<std::array<double, 5>> nees;   // per epoch: position, attitude, all 15, tilt (2 dof), heading (1 dof)
};

RunOut runScheme(const Config& c, const Variant& v, const Data& d) {
  EskfParams ep;
  ep.sigma_acc = c.matched ? c.sim_acc : 0.01;   // matched: the filter is told the noise of the simulated IMU
  ep.sigma_gyro = c.matched ? kSimGyro : 0.001;
  ep.sigma_acc_bias = 1e-4;
  ep.sigma_gyro_bias = 1e-5;
  ep.lever_consider = v.consider;
  ep.lever_heading_sigma_max = v.cap;
  ep.lever_release_dv = v.rel_dv;
  ep.lever_align_sigma = v.align;
  ep.lever_start_sigma = v.start_sigma;
  ep.lever_align_period_max = v.period_max;
  ep.lever_outage_spread = v.outage_spread;
  Eskf f(ep);
  const Vec3 lever = kAntenna[c.ant];
  const bool lever_on = !v.ignore;
  const Eigen::Matrix3d Rg = Eigen::Vector3d(4e-4, 4e-4, 1.0).asDiagonal();  // the node floors z at 1 m^2
  Eigen::Matrix<double, 6, 6> Rn = Eigen::Matrix<double, 6, 6>::Identity();
  Rn.topLeftCorner<3, 3>() *= kNdtPos * kNdtPos;
  Rn.bottomRightCorner<3, 3>() *= kNdtRot * kNdtRot;

  auto covariance = [&](const Eigen::Quaterniond& q, const Vec3& accel) {
    Eskf::Mat15 P0 = Eskf::Mat15::Identity();
    P0.block<3, 3>(6, 6) *= 0.5;
    P0.block<3, 3>(9, 9) *= 1e-2;
    P0.block<3, 3>(12, 12) *= 1e-4;
    if (v.seeded && !lever.isZero(0.0))
      P0.block<3, 3>(6, 6) = Eskf::attitudeCovariance(q, Eskf::tiltVarianceFromAccel(accel, 1e-2, ep), 0.5);
    return P0;
  };

  // Initialise as the node does: attitude from one accelerometer sample, position from the
  // first fix moved from the antenna to the body origin.
  const int n = static_cast<int>(d.f.size()) - 1;
  const Truth T0 = truthOf(c, 0.0);
  NominalState x0;
  x0.q = attitudeFromAccel(d.f[0], T0.yaw + d.yaw_err);
  const Vec3 fix0 = T0.p + bodyRot(c, T0.yaw) * lever + d.gnss_noise[0];
  x0.p = lever_on ? Vec3(fix0 - x0.q * lever) : fix0;
  x0.v = Vec3::Zero();
  f.initialize(x0, covariance(x0.q, d.f[0]));
  if (!lever.isZero(0.0) && v.start_sigma > 0.0) f.markHeadingUnknown();

  RunOut out;
  const int reseed_k = c.reseed ? n / 2 : -1;
  // Suites with a stand (hsg, hgap): an outage begins 10 s before the robot starts.
  const double drop0 = c.stand_s > 0.0 ? c.stand_s + c.drop_off : (c.dur == 60 ? 30.0 : (c.dur <= 300 ? 120.0 : 0.4 * c.dur));
  const double drop1 = drop0 + (c.drop_s >= 0.0 ? c.drop_s : (c.dur == 60 ? 10.0 : 30.0));
  const double tsq = lever.head<2>().norm();       // horizontal lever arm, for the circle criterion
  long n_lever = 0, n_capped = 0, n_open = 0;
  const int gnss_every = static_cast<int>(std::llround(c.gper / kDt));
  Eigen::Matrix<double, 15, 1> e;
  for (int k = 1; k <= n; ++k) {
    const double t = k * kDt;
    if (k == reseed_k) {
      const Truth Tr = truthOf(c, t);
      NominalState xr;
      xr.p = Tr.p + d.reseed_p_off;
      xr.v = Vec3::Zero();   // /initialpose carries no velocity: the node starts at rest
      xr.q = quatZ(Tr.yaw + d.reseed_yaw_err);
      // The scheme's rule for the attitude of a reseed: as /initialpose sends it (level), or
      // roll and pitch from the accelerometer at the /initialpose heading (a lever arm only).
      if (v.reseed_accel && !lever.isZero(0.0)) xr.q = attitudeFromAccel(d.f[k], Tr.yaw + d.reseed_yaw_err);
      f.initialize(xr, covariance(xr.q, d.f[k]));
    }
    f.predict(d.f[k], d.w[k], kDt);
    const int i = k / gnss_every;
    if ((k + c.gphase) % gnss_every == 0 && !(c.drop && t >= drop0 && t < drop1)) {
      const Truth T = truthOf(c, t);
      const Vec3 fix = T.p + bodyRot(c, T.yaw) * lever + d.gnss_noise[i];
      if (lever_on) {
        if (!lever.isZero(0.0)) { ++n_lever; if (f.leverReleaseOpen()) ++n_open; }
        f.updatePositionLever(fix, lever, Rg);
        if (!lever.isZero(0.0) && f.leverHeadingCapped()) ++n_capped;
      } else f.updatePosition(fix, Rg);
    }
    if (c.ndt && (k + c.gphase) % gnss_every == gnss_every / 2) {
      const Truth T = truthOf(c, t);
      f.updatePose(T.p + d.ndt_p[i], bodyQuat(c, T.yaw) * so3Exp(d.ndt_r[i]), Rn);
    }
    if (k % 100 == 0 && t >= kEpochFrom) {
      const Truth T = truthOf(c, t);
      const NominalState& x = f.state();
      const Eigen::Quaterniond qt = bodyQuat(c, T.yaw);
      e.segment<3>(0) = T.p - x.p;
      e.segment<3>(3) = T.v - x.v;
      e.segment<3>(6) = so3Log(x.q.conjugate() * qt);
      e.segment<3>(9) = d.ba - x.ba;
      e.segment<3>(12) = d.bg - x.bg;
      const Eskf::Mat15& P = f.covariance();
      const double np = e.head<3>().dot(P.block<3, 3>(0, 0).inverse() * e.head<3>());
      const double na = e.segment<3>(6).dot(P.block<3, 3>(6, 6).inverse() * e.segment<3>(6));
      const double nall = e.dot(P.ldlt().solve(e));
      // tilt (2 dof, the plane across the vertical) and heading (1 dof, along it), local error
      const Vec3 u = x.q.toRotationMatrix().transpose() * Vec3::UnitZ();
      const Vec3 b1 = (std::fabs(u.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY()).cross(u).normalized();
      const Vec3 b2 = u.cross(b1);
      Eigen::Matrix<double, 3, 2> Bt; Bt.col(0) = b1; Bt.col(1) = b2;
      const Eigen::Vector2d et = Bt.transpose() * e.segment<3>(6);
      const Eigen::Matrix2d Pt = Bt.transpose() * P.block<3, 3>(6, 6) * Bt;
      const double nt = et.dot(Pt.inverse() * et);
      const double eh = u.dot(e.segment<3>(6)), vh = u.dot(P.block<3, 3>(6, 6) * u);
      const double nh = vh > 0.0 ? eh * eh / vh : 0.0;
      out.max_sigh = std::max(out.max_sigh, std::sqrt(std::max(vh, 0.0)));
      out.nees.push_back({np, na, nall, nt, nh});
      const Vec3 rot = so3Log(qt * x.q.conjugate());   // error about world axes
      const double xy = e.head<2>().norm(), z = std::fabs(e.z()), tilt = rot.head<2>().norm(), head = std::fabs(rot.z());
      const bool in_drop = c.drop && t >= drop0 && t < drop1 + 5.0;
      const bool after_reseed = c.reseed && k >= reseed_k && t < reseed_k * kDt + 5.0;
      if (in_drop) out.drop_xy = std::max(out.drop_xy, xy);
      out.end_xy = xy; out.end_z = z; out.end_tilt = tilt; out.end_head = head;
      if (!(in_drop || after_reseed)) {
        out.max_xy = std::max(out.max_xy, xy); out.max_z = std::max(out.max_z, z);
        out.max_tilt = std::max(out.max_tilt, tilt); out.max_head = std::max(out.max_head, head);
        out.max_circ = std::max(out.max_circ, xy - (2.0 * tsq * std::sin(0.5 * std::min(head, M_PI)) + 0.3));
        if (!out.diverged && (tilt > kTiltDiv || z > kZDiv || xy > kXyDiv || head > kHeadDiv)) {
          out.diverged = true; out.t_div = t;
        }
      }
    }
  }
  out.cap_frac = n_lever ? double(n_capped) / n_lever : 0.0;
  out.rel_frac = n_lever ? double(n_open) / n_lever : 0.0;
  return out;
}

std::vector<Config> suite(const std::string& name) {
  std::vector<Config> cs;
  int id = 0;
  auto add = [&](int dur, int motion, int ant, double gb, double ab, bool ndt, bool drop, bool reseed, bool matched, int nseed) {
    cs.push_back({id++, dur, motion, ant, gb, ab, ndt, drop, reseed, matched, nseed});
  };
  // second pass: the four antennas innermost, all on the data of the antenna-A row; drop_s 0 = none
  auto addH = [&](int dur, int motion, double gb, double ab, bool ndt, double drop_s, bool reseed, double slope, int nseed, double yaw0 = kYawStart, double sim_acc = kSimAcc) {
    const int first = id;
    for (int ant = 0; ant < 4; ++ant) {
      Config c{id++, dur, motion, ant, gb, ab, ndt, drop_s > 0.0, reseed, true, nseed};
      c.data_id = first; c.drop_s = drop_s > 0.0 ? drop_s : -1.0; c.slope = slope; c.yaw0 = yaw0; c.sim_acc = sim_acc; c.hsuite = true;
      cs.push_back(c);
    }
  };
  if (name == "g60") {
    for (double gb : {0.0, 0.05, 0.1, 0.2, 0.5})
      for (double ab : {0.0, 0.1})
        for (int motion = 0; motion < 4; ++motion)
          for (int ant = 0; ant < 3; ++ant)
            for (bool ndt : {false, true})
              for (bool drop : {false, true})
                for (bool reseed : {false, true})
                  for (bool matched : {true, false}) add(60, motion, ant, gb, ab, ndt, drop, reseed, matched, 10);
  } else if (name == "g300") {
    for (double gb : {0.0, 0.1, 0.5})
      for (double ab : {0.0, 0.1})
        for (int motion = 0; motion < 4; ++motion)
          for (int ant = 0; ant < 2; ++ant)
            for (bool ndt : {false, true})
              for (bool drop : {false, true})
                for (bool reseed : {false, true}) add(300, motion, ant, gb, ab, ndt, drop, reseed, true, 6);
  } else if (name == "smoke") {
    add(60, 0, 0, 0.1, 0.0, false, false, false, true, 4);
    add(60, 1, 1, 0.5, 0.1, true, true, true, false, 4);
  } else if (name == "h300") {      // 540 groups of 4 antennas, 2 seeds
    for (double gb : {0.0, 0.05, 0.1, 0.2, 0.5})
      for (double ab : {0.0, 0.1, 0.3})
        for (int motion : {0, 4, 5, 1, 2, 3})
          for (double drop_s : {0.0, 30.0, 120.0})
            for (bool reseed : {false, true}) addH(300, motion, gb, ab, false, drop_s, reseed, 0.0, 2);
  } else if (name == "h600") {      // standing, 600 s, 8 seeds
    for (double gb : {0.0, 0.1, 0.5})
      for (double ab : {0.0, 0.1})
        for (double drop_s : {0.0, 120.0})
          for (bool reseed : {false, true}) addH(600, 0, gb, ab, false, drop_s, reseed, 0.0, 8);
  } else if (name == "h900") {      // standing and line, 900 s, 8 seeds
    for (int motion : {0, 4})
      for (double gb : {0.0, 0.1, 0.2})
        for (double ab : {0.0, 0.1}) addH(900, motion, gb, ab, false, 0.0, false, 0.0, 8);
  } else if (name == "h1800") {     // standing and line, 1,800 s, 8 seeds
    for (int motion : {0, 4})
      for (double gb : {0.0, 0.5}) addH(1800, motion, gb, 0.1, false, 0.0, false, 0.0, 8);
  } else if (name == "hndt") {      // NDT on, 300 s, all motions, with a reseed and a dropout
    for (double gb : {0.0, 0.5})
      for (int motion = 0; motion < 6; ++motion)
        for (double drop_s : {0.0, 30.0})
          for (bool reseed : {false, true}) addH(300, motion, gb, 0.1, true, drop_s, reseed, 0.0, 2);
  } else if (name == "hslope") {    // reseed at 30 s of 60 s, level or on a tilted mount, NDT off and on
    for (double slope : {0.0, 0.05, 0.1})
      for (int motion : {0, 1, 2})
        for (double gb : {0.0, 0.1})
          for (double ab : {0.0, 0.1})
            for (bool ndt : {false, true}) addH(60, motion, gb, ab, ndt, 0.0, true, slope, 4);
  } else if (name == "hyaw") {      // a wrong heading at the start (0.5, 1, 2 rad): does a scheme recover it?
    for (double yaw0 : {0.5, 1.0, 2.0})
      for (int motion : {0, 4, 5, 1, 2})
        for (double gb : {0.0, 0.1}) addH(300, motion, gb, 0.1, false, 0.0, false, 0.0, 4, yaw0);
  } else if (name == "hvib") {      // a standing or straight-driving robot with more accelerometer noise (vibration): does the release open?
    for (double sa : {0.02, 0.05, 0.1, 0.2, 0.4})
      for (int motion : {0, 4})
        addH(300, motion, 0.1, 0.1, false, 0.0, false, 0.0, 4, kYawStart, sa);
  } else if (name == "hsg" || name == "hsgv") {       // third pass: stand for minutes, then start (startstop, turn on the spot, circle); 160 s of motion, 4 seeds
    for (int stand : {300, 600, 900, 1800})
      for (double gb : {0.0, 0.05, 0.1, 0.2})
        for (double ab : {0.0, 0.1})
          for (int motion : {6, 7, 8}) {
            const int first = id;
            addH(stand + 160, motion, gb, ab, false, 0.0, false, 0.0, 4);
            for (int i = first; i < id; ++i) cs[i].stand_s = stand;
          }
  } else if (name == "hgap") {   // fifth pass: an outage of the fixes that spans the start after a stand (30 s and 120 s, from 10 s before the start)
    for (int stand : {300, 600})
      for (double gb : {0.0, 0.1, 0.2})
        for (double ab : {0.0, 0.1})
          for (int motion : {6, 8})
            for (double drop_s : {30.0, 120.0}) {
              const int first = id;
              addH(stand + 160, motion, gb, ab, false, drop_s, false, 0.0, 4);
              for (int i = first; i < id; ++i) cs[i].stand_s = stand;
            }
    // Round 7 (finding 2): an outage of 1.5 to 12 s that begins just before or during the first acceleration after a long stand (the fifth round
    // tested only 30 s and 120 s outages that begin 10 s before the start). The outage begins drop_off s after the start of the stand's end: sgstart
    // accelerates from 6 s on (5 s: 1 s before, 7.5 s: in the middle of the acceleration), sgdrive from 0 s (-1 s, 1.5 s, and 5 s: on the circle).
    for (int stand : {300, 900})
      for (double gb : {0.0, 0.2})
        for (double ab : {0.0, 0.1})
          for (int motion : {6, 8})
            for (double off : (motion == 6 ? std::vector<double>{5.0, 7.5} : std::vector<double>{-1.0, 1.5, 5.0}))
              for (double drop_s : {1.5, 3.0, 5.0, 12.0}) {
                const int first = id;
                addH(stand + 160, motion, gb, ab, false, drop_s, false, 0.0, 4);
                for (int i = first; i < id; ++i) { cs[i].stand_s = stand; cs[i].drop_off = off; }
              }
  } else if (name == "hboot") {   // fifth pass: the default boot, a wrong initial_yaw (0.5 to 3 rad), no /initialpose, a short or long stand, then the start
    for (int stand : {6, 30, 60, 90, 120, 300})
      for (double yaw0 : {0.5, 1.0, 2.0, 3.0})
        for (int motion : {6, 8}) {
          const int first = id;
          addH(stand + 160, motion, 0.1, 0.1, false, 0.0, false, 0.0, 4, yaw0);
          for (int i = first; i < id; ++i) cs[i].stand_s = stand;
        }
  } else if (name == "hrate" || name == "hratex") {   // boot and stand-then-go cells at several fix rates; the data (IMU) of a cell is the same at every rate
    std::vector<int> cell_first;
    int rate_i = 0;
    // hratex lists the periods of hrate first, so that the data of its cells are those of the hrate cells (the ids of hrate), then keeps its own
    const std::vector<double> periods = name == "hrate" ? std::vector<double>{2.0, 1.0, 0.5, 0.2, 0.1, 0.05} : std::vector<double>{2.0, 1.2, 1.1, 0.02, 0.01};
    for (double gper : periods) {
      int cell = 0;
      auto grp = [&](int stand, int motion, double gb, double ab, double yaw0) {
        const int first = id;
        addH(stand + 160, motion, gb, ab, false, 0.0, false, 0.0, 4, yaw0);
        if (rate_i == 0) cell_first.push_back(first);
        for (int i = first; i < id; ++i) { cs[i].stand_s = stand; cs[i].gper = gper; cs[i].data_id = cell_first[cell] + (i - first); }
        ++cell;
      };
      for (int stand : {6, 30, 300}) for (double y : {0.1, 0.3, 2.0}) for (int m : {6, 8}) grp(stand, m, 0.1, 0.1, y);
      for (int stand : {300, 900}) for (double gb : {0.0, 0.2}) for (double ab : {0.0, 0.1}) for (int m : {6, 8}) grp(stand, m, gb, ab, 0.1);
      ++rate_i;
    }
    if (name == "hratex") cs.erase(cs.begin(), cs.begin() + cell_first.size() * 4);   // the 2 s cells were only there to fix the data ids
  } else if (name == "hphase") {   // the cells of hrate at 1, 2, 5 and 10 Hz with the fixes at ten phases of the fix period to the start of the motion
    std::vector<int> cell_first;
    bool first_block = true;
    for (double gper : {1.0, 0.5, 0.2, 0.1}) {
      const int period_ticks = static_cast<int>(std::llround(gper / kDt));
      for (int idx = 0; idx < 10; ++idx) {
        int cell = 0;
        auto grp = [&](int stand, int motion, double gb, double ab, double yaw0) {
          const int first = id;
          addH(stand + 160, motion, gb, ab, false, 0.0, false, 0.0, 4, yaw0);
          if (first_block) cell_first.push_back(first);
          for (int i = first; i < id; ++i) {
            cs[i].stand_s = stand; cs[i].gper = gper; cs[i].gphase = idx * period_ticks / 10; cs[i].gphase_idx = idx;
            cs[i].data_id = cell_first[cell] + (i - first);
          }
          ++cell;
        };
        for (int stand : {6, 30, 300}) for (double y : {0.1, 0.3, 2.0}) for (int m : {6, 8}) grp(stand, m, 0.1, 0.1, y);
        for (int stand : {300, 900}) for (double gb : {0.0, 0.2}) for (double ab : {0.0, 0.1}) for (int m : {6, 8}) grp(stand, m, gb, ab, 0.1);
        first_block = false;
      }
    }
  } else if (name == "hsmoke") {
    addH(60, 0, 0.1, 0.0, false, 0.0, false, 0.0, 2);
    addH(60, 5, 0.5, 0.1, true, 10.0, true, 0.05, 2);
  }
  return cs;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5) { std::fprintf(stderr, "usage: %s g60|g300|smoke|h300|h600|h900|h1800|hndt|hslope|hyaw|hvib|hsg|hsgv|hgap|hboot|hrate|hratex|hphase|hsmoke SHARD NSHARDS OUT_PREFIX\n", argv[0]); return 2; }
  const std::string sname = argv[1];
  const char* tag = std::getenv("LEVER_TAG") ? std::getenv("LEVER_TAG") : "";
  const int shard = std::atoi(argv[2]), nshards = std::atoi(argv[3]);
  const std::string prefix = argv[4];
  const std::vector<Config> cs = suite(sname);
  std::FILE* fr = std::fopen((prefix + "_runs.csv").c_str(), "w");
  std::FILE* fa = std::fopen((prefix + "_agg.csv").c_str(), "w");
  if (!fr || !fa) { std::perror("open"); return 1; }
  const char* cfg_hdr = "cfg,dur,motion,antenna,gyro_bias_dps,acc_bias,ndt,drop,reseed,noise,drop_s,slope,yaw0";
  std::fprintf(fr, "%s,scheme,seed,diverged,t_div,drop_xy,max_xy,max_z,max_tilt,max_head,end_xy,end_z,end_tilt,end_head,"
                   "max_circ,max_sigh,cap_frac,rel_frac\n", cfg_hdr);
  std::fprintf(fa, "%s,scheme,nseed,nepoch,nees_pos_end,nees_att_end,nees_all_end,nees_pos_mean,nees_att_mean,nees_all_mean,"
                   "pos_lo,pos_hi,att_lo,att_hi,all_lo,all_hi,frac_pos_in,frac_pos_above,frac_att_in,frac_att_above,frac_all_in,frac_all_above,"
                   "nees_tilt_end,nees_tilt_mean,tilt_lo,tilt_hi,frac_tilt_in,frac_tilt_above,"
                   "nees_head_end,nees_head_mean,head_lo,head_hi,frac_head_in,frac_head_above\n",
               cfg_hdr);
  for (const Config& c : cs) {
    if (c.id % nshards != shard) continue;
    std::vector<Data> data;
    const unsigned data_id = c.data_id >= 0 ? c.data_id : c.id;
    for (int s = 0; s < c.nseed; ++s) data.push_back(makeData(c, (sname == "hsgv" ? kSeedV : kSeed) + 1000u * data_id + s));
    char cfg[200];
    char noise[32];
    if (c.sim_acc != kSimAcc) std::snprintf(noise, sizeof noise, "vib%g", c.sim_acc);   // accelerometer noise other than the default: told to the filter
    else std::snprintf(noise, sizeof noise, "%s", c.matched ? "matched" : "yaml");
    char noise2[48];
    if (sname == "hphase") std::snprintf(noise2, sizeof noise2, "p%gi%d", c.gper, c.gphase_idx);   // suite hphase: fix period and phase (tenths of the period)
    else if (c.gper != kGnssPeriod) std::snprintf(noise2, sizeof noise2, "fix%gs", c.gper);   // a fix rate other than 5 Hz (suite hrate)
    else if (c.stand_s > 0.0 && c.drop && c.drop_off != -10.0) std::snprintf(noise2, sizeof noise2, "off%g", c.drop_off);   // an outage that begins other than 10 s before the start (suite hgap, round 7)
    else std::snprintf(noise2, sizeof noise2, "%s", noise);
    std::snprintf(cfg, sizeof cfg, "%d,%d,%s,%s,%g,%g,%d,%d,%d,%s,%g,%g,%g", c.id, c.dur, kMotionName[c.motion], kAntennaName[c.ant],
                  c.gbias_dps, c.abias, c.ndt, c.drop, c.reseed, noise2, c.drop_s, c.slope, c.yaw0);
    const Variant* variants = kVariants;
    const int nvar = kNumVariants;
    for (int vi = 0; vi < nvar; ++vi) {
      // LEVER_ONLY=name,name: run just these schemes. Without it: ignore in the first-pass suites (g*, smoke), S1s, S2s, S8u everywhere.
      if (const char* only = std::getenv("LEVER_ONLY")) {
        if (!std::strstr((std::string(",") + only + ",").c_str(), (std::string(",") + variants[vi].name + ",").c_str())) continue;
      } else if (c.hsuite && variants[vi].ignore) continue;
      std::vector<RunOut> outs;
      for (int s = 0; s < c.nseed; ++s) outs.push_back(runScheme(c, variants[vi], data[s]));
      for (int s = 0; s < c.nseed; ++s) {
        const RunOut& o = outs[s];
        std::fprintf(fr, "%s,%s%s,%d,%d,%.1f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f\n", cfg, variants[vi].name, tag, s, o.diverged ? 1 : 0,
                     o.t_div, o.drop_xy, o.max_xy, o.max_z, o.max_tilt, o.max_head, o.end_xy, o.end_z, o.end_tilt, o.end_head,
                     o.max_circ, o.max_sigh, o.cap_frac, o.rel_frac);
      }
      // seed-mean NEES at each epoch, against the chi-square band of S seeds
      const int ne = static_cast<int>(outs[0].nees.size());
      double end[5] = {0, 0, 0, 0, 0}, mean[5] = {0, 0, 0, 0, 0}, lo[5], hi[5], in[5] = {0, 0, 0, 0, 0}, above[5] = {0, 0, 0, 0, 0};
      const double dof[5] = {3, 3, 15, 2, 1};
      for (int b = 0; b < 5; ++b) {
        lo[b] = chi2q(dof[b] * c.nseed, -1.959964) / c.nseed;
        hi[b] = chi2q(dof[b] * c.nseed, 1.959964) / c.nseed;
        for (int k = 0; k < ne; ++k) {
          double m = 0;
          for (int s = 0; s < c.nseed; ++s) m += outs[s].nees[k][b];
          m /= c.nseed;
          mean[b] += m / ne;
          if (m >= lo[b] && m <= hi[b]) in[b] += 1.0 / ne;
          if (m > hi[b]) above[b] += 1.0 / ne;
          if (k == ne - 1) end[b] = m;
        }
      }
      std::fprintf(fa, "%s,%s,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                   cfg, (std::string(variants[vi].name) + tag).c_str(), c.nseed, ne, end[0], end[1], end[2], mean[0], mean[1], mean[2], lo[0], hi[0], lo[1], hi[1],
                   lo[2], hi[2], in[0], above[0], in[1], above[1], in[2], above[2],
                   end[3], mean[3], lo[3], hi[3], in[3], above[3], end[4], mean[4], lo[4], hi[4], in[4], above[4]);
    }
    std::fflush(fr); std::fflush(fa);
  }
  std::fclose(fr); std::fclose(fa);
  return 0;
}
