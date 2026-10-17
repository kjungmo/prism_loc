// eskf_lever.cpp -- how to fuse a GNSS fix that is taken at a lever arm from base_link.
//
// SYNTHETIC EXPERIMENT. Drives the REAL prism_loc_fusion::Eskf (no copy, no ROS) with a
// simulated IMU, a simulated GNSS antenna at a lever arm from the body origin, and
// optionally simulated NDT pose fixes, and compares ways to apply the antenna fix:
//
//   ignore    the antenna offset is not modelled (what main did before the lever arm):
//             the fix is a position of the body origin, the filter starts at the fix
//   S0        consider states = attitude and gyro bias (the first version of the lever arm)
//   S1        consider states = heading and the vertical gyro bias only
//   S3        consider state  = heading only
//   S2        none: the full Jacobian (a plain Kalman update of every state)
//   S4        S2, iterated three times (Bell and Cathey 1993)
//   S5        S1, iterated three times
//   S6        S1 while the robot stands, the full update (no consider state) while it moves:
//             the heading and the vertical gyro bias are observable from a travelling antenna.
//             Moving = the filter's speed is over 0.3 m/s and over 3 times its own uncertainty
//             (the velocity estimate is far off right after a start), or its body rate is over
//             0.1 rad/s (S6s; S6t: 0.1 m/s and 0.05 rad/s). S6v: the speed test alone (0.3 m/s,
//             no body-rate test); S6va and S6vb: its threshold halved and doubled (0.15, 0.6 m/s).
//             S6vb (0.6 m/s) had the fewest divergences and is what the node runs (EskfParams
//             defaults). The speed gate of S6v and S6vb is the only difference between them.
//   <name>s   the same scheme, with the roll/pitch variance at start (and at every
//             reseed) taken from the accelerometer (Eskf::tiltVarianceFromAccel,
//             about 1e-4 rad^2) instead of the 0.5 rad^2 of the heading
//
// Grid (one "configuration" = one row of the grid; every configuration is run with
// NSEED seeds and every scheme sees the identical data, common random numbers):
//   duration       60 s (suite g60), 300 s (suite g300)
//   gyro bias      0, 0.05, 0.1, 0.2, 0.5 deg/s on each axis, sign of each axis drawn per run
//   accel bias     0 and 0.1 m/s^2 on each axis, sign drawn per run
//   motion         stand | turn (0.3 rad/s for 4 s, still 1 s, repeated) | drive (planar
//                  lemniscate, 8 m x 4 m, period 30 s) | spin (0.3 rad/s without pause)
//   antenna        A = (0.5, 0, 1.0)  B = (-0.3, 0.2, 1.0)  Z = (0, 0, 0) (the zero lever)
//   NDT            off | on (6-DoF pose fix at 5 Hz, as accurate as the filter assumes)
//   GNSS dropout   none | no fix for 10 s from t = 30 s (60 s runs) / 30 s from 120 s (300 s)
//   reseed         none | /initialpose at half time: truth + (0.04, -0.03) m, 0.01 rad,
//                  level, biases and covariance reset as the node does
//   noise          matched (the filter is told the noise of the simulated IMU) |
//                  yaml (fusion3d.yaml values, half of the simulated noise)
//
// Declared before the final run (the numbers below are the pre-declared thresholds):
//   a run has DIVERGED if at any 1 s epoch from t = 10 s on: tilt error > 0.05 rad, or
//   |z error| > 0.5 m, or horizontal error > 1.0 m, or heading error > 0.5 rad.
//   Epochs from the start of a GNSS dropout to 5 s after its end, and the 5 s after a
//   reseed, are not tested (and not in the max_* columns): without NDT, 10 s of dead
//   reckoning with a biased IMU drifts by metres in every scheme (an exploratory first
//   pass that tested them counted 494 of its 499 divergences of the best scheme there).
//   That pass, and three more before this one (a smaller list of schemes; the motion gates;
//   the spin motion), were exploratory and are not part of the result: this file and the
//   grid in it are what was run last, once, with these thresholds.
//   The largest horizontal error in the dropout window is reported apart (drop_xy).
//   NEES at 1 s epochs from t = 10 s on, for position (3), attitude (3) and the whole
//   15-state error (the truth of every state, including the biases, is known). The
//   NEES of a configuration is the mean over its seeds; its two-sided 95 % band is
//   [chi2_.025(S dof), chi2_.975(S dof)] / S (Wilson-Hilferty).
//
// Build and run from the repository root (see experiments/README.md, study 6):
//   g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3
//       prism_loc_fusion/src/*.cpp experiments/eskf_lever.cpp -o /tmp/exp_lever
//   /tmp/exp_lever g60 0 1 out/lever_g60      # suite, shard, number of shards, output prefix
// Writes <prefix>_runs.csv (one row per run and scheme) and <prefix>_agg.csv (one row per
// configuration and scheme). Deterministic for one compiler and standard library
// (std::normal_distribution is implementation defined).

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
constexpr double kGnssPeriod = 0.2;          // 5 Hz fix
constexpr double kNdtPeriod = 0.2;           // 5 Hz pose fix, offset 0.1 s from the fix
constexpr double kGnssStd = 0.02;            // simulated antenna noise, per axis
constexpr double kSimAcc = 0.02, kSimGyro = 0.002;   // simulated IMU white noise
constexpr double kNdtPos = 0.1, kNdtRot = 0.05;      // NDT std the node assumes (pose_pos_std/rot_std)
constexpr double kYawStart = 0.1;            // heading error at start, rad (sign drawn per run)
constexpr double kTiltDiv = 0.05, kZDiv = 0.5, kXyDiv = 1.0, kHeadDiv = 0.5;
constexpr double kEpochFrom = 10.0;
constexpr std::uint64_t kSeed = 20261018u;

const Vec3 kAntenna[3] = {Vec3(0.5, 0.0, 1.0), Vec3(-0.3, 0.2, 1.0), Vec3(0.0, 0.0, 0.0)};
const char* kAntennaName[3] = {"A", "B", "Z"};
const char* kMotionName[4] = {"stand", "turn", "drive", "spin"};

struct Variant {
  const char* name;
  bool ignore;       // do not model the lever arm (main's behaviour)
  unsigned consider;
  int iterations;
  bool seeded;
  double gate_speed{0.0}, gate_rate{0.0};   // motion gate (EskfParams::lever_motion_*); 0 = none
  unsigned consider_moving{0u};
};
const Variant kVariants[] = {
    {"ignore", true, 0, 1, false},
    {"S0", false, kConsiderAttitudeAndGyroBias, 1, false},
    {"S0s", false, kConsiderAttitudeAndGyroBias, 1, true},
    {"S1", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, false},
    {"S1s", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true},
    {"S2", false, 0, 1, false},
    {"S2s", false, 0, 1, true},
    {"S3s", false, kConsiderHeading, 1, true},
    {"S4s", false, 0, 3, true},
    {"S5s", false, kConsiderHeading | kConsiderGyroBiasVertical, 3, true},
    {"S6s", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.3, 0.1, 0u},
    {"S6t", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.1, 0.05, 0u},
    {"S6v", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.3, 0.0, 0u},
    {"S6va", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.15, 0.0, 0u},
    {"S6vb", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.6, 0.0, 0u},
};
constexpr int kNumVariants = sizeof(kVariants) / sizeof(kVariants[0]);

struct Config {
  int id, dur, motion, ant;
  double gbias_dps, abias;
  bool ndt, drop, reseed, matched;
  int nseed;
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

Eigen::Matrix3d rotZ(double yaw) { return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix(); }
Eigen::Quaterniond quatZ(double yaw) { return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3::UnitZ())); }

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
  const int n = static_cast<int>(std::llround(c.dur / kDt));
  d.ba = c.abias * signs3(rng);
  d.bg = (c.gbias_dps * M_PI / 180.0) * signs3(rng);
  d.yaw_err = kYawStart * (rng.uniform() < 0.5 ? -1.0 : 1.0);
  d.reseed_p_off = Vec3(0.04, -0.03, 0.0);
  d.reseed_yaw_err = 0.01;
  d.f.resize(n + 1); d.w.resize(n + 1);
  for (int k = 0; k <= n; ++k) {
    const Truth T = truthAt(c.motion, k * kDt);
    d.f[k] = rotZ(T.yaw).transpose() * (T.a + Vec3(0, 0, 9.81)) + d.ba + gauss3(rng, kSimAcc);
    d.w[k] = Vec3(0, 0, T.wz) + d.bg + gauss3(rng, kSimGyro);
  }
  const int ngnss = static_cast<int>(c.dur / kGnssPeriod) + 1;
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
  std::vector<std::array<double, 3>> nees;   // per epoch: position, attitude, all 15
};

RunOut runScheme(const Config& c, const Variant& v, const Data& d) {
  EskfParams ep;
  ep.sigma_acc = c.matched ? kSimAcc : 0.01;
  ep.sigma_gyro = c.matched ? kSimGyro : 0.001;
  ep.sigma_acc_bias = 1e-4;
  ep.sigma_gyro_bias = 1e-5;
  ep.lever_consider = v.consider;
  ep.lever_iterations = v.iterations;
  ep.lever_motion_speed = v.gate_speed;
  ep.lever_motion_rate = v.gate_rate;
  ep.lever_consider_moving = v.consider_moving;
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
  const Truth T0 = truthAt(c.motion, 0.0);
  NominalState x0;
  x0.q = attitudeFromAccel(d.f[0], T0.yaw + d.yaw_err);
  const Vec3 fix0 = T0.p + rotZ(T0.yaw) * lever + d.gnss_noise[0];
  x0.p = lever_on ? Vec3(fix0 - x0.q * lever) : fix0;
  x0.v = Vec3::Zero();
  f.initialize(x0, covariance(x0.q, d.f[0]));

  RunOut out;
  const int reseed_k = c.reseed ? n / 2 : -1;
  const double drop0 = c.dur == 60 ? 30.0 : 120.0, drop1 = drop0 + (c.dur == 60 ? 10.0 : 30.0);
  const int gnss_every = static_cast<int>(std::llround(kGnssPeriod / kDt));
  Eigen::Matrix<double, 15, 1> e;
  for (int k = 1; k <= n; ++k) {
    const double t = k * kDt;
    if (k == reseed_k) {
      const Truth Tr = truthAt(c.motion, t);
      NominalState xr;
      xr.p = Tr.p + d.reseed_p_off;
      xr.v = Vec3::Zero();   // /initialpose carries no velocity: the node starts at rest
      xr.q = quatZ(Tr.yaw + d.reseed_yaw_err);
      f.initialize(xr, covariance(xr.q, d.f[k]));
    }
    f.predict(d.f[k], d.w[k], kDt);
    const int i = k / gnss_every;
    if (k % gnss_every == 0 && !(c.drop && t >= drop0 && t < drop1)) {
      const Truth T = truthAt(c.motion, t);
      const Vec3 fix = T.p + rotZ(T.yaw) * lever + d.gnss_noise[i];
      if (lever_on) f.updatePositionLever(fix, lever, Rg);
      else f.updatePosition(fix, Rg);
    }
    if (c.ndt && k % gnss_every == gnss_every / 2) {
      const Truth T = truthAt(c.motion, t);
      f.updatePose(T.p + d.ndt_p[i], quatZ(T.yaw) * so3Exp(d.ndt_r[i]), Rn);
    }
    if (k % 100 == 0 && t >= kEpochFrom) {
      const Truth T = truthAt(c.motion, t);
      const NominalState& x = f.state();
      const Eigen::Quaterniond qt = quatZ(T.yaw);
      e.segment<3>(0) = T.p - x.p;
      e.segment<3>(3) = T.v - x.v;
      e.segment<3>(6) = so3Log(x.q.conjugate() * qt);
      e.segment<3>(9) = d.ba - x.ba;
      e.segment<3>(12) = d.bg - x.bg;
      const Eskf::Mat15& P = f.covariance();
      const double np = e.head<3>().dot(P.block<3, 3>(0, 0).inverse() * e.head<3>());
      const double na = e.segment<3>(6).dot(P.block<3, 3>(6, 6).inverse() * e.segment<3>(6));
      const double nall = e.dot(P.ldlt().solve(e));
      out.nees.push_back({np, na, nall});
      const Vec3 rot = so3Log(qt * x.q.conjugate());   // error about world axes
      const double xy = e.head<2>().norm(), z = std::fabs(e.z()), tilt = rot.head<2>().norm(), head = std::fabs(rot.z());
      const bool in_drop = c.drop && t >= drop0 && t < drop1 + 5.0;
      const bool after_reseed = c.reseed && k >= reseed_k && t < reseed_k * kDt + 5.0;
      if (in_drop) out.drop_xy = std::max(out.drop_xy, xy);
      out.end_xy = xy; out.end_z = z; out.end_tilt = tilt; out.end_head = head;
      if (!(in_drop || after_reseed)) {
        out.max_xy = std::max(out.max_xy, xy); out.max_z = std::max(out.max_z, z);
        out.max_tilt = std::max(out.max_tilt, tilt); out.max_head = std::max(out.max_head, head);
        if (!out.diverged && (tilt > kTiltDiv || z > kZDiv || xy > kXyDiv || head > kHeadDiv)) {
          out.diverged = true; out.t_div = t;
        }
      }
    }
  }
  return out;
}

std::vector<Config> suite(const std::string& name) {
  std::vector<Config> cs;
  int id = 0;
  auto add = [&](int dur, int motion, int ant, double gb, double ab, bool ndt, bool drop, bool reseed, bool matched, int nseed) {
    cs.push_back({id++, dur, motion, ant, gb, ab, ndt, drop, reseed, matched, nseed});
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
  }
  return cs;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5) { std::fprintf(stderr, "usage: %s g60|g300|smoke SHARD NSHARDS OUT_PREFIX\n", argv[0]); return 2; }
  const std::string sname = argv[1];
  const int shard = std::atoi(argv[2]), nshards = std::atoi(argv[3]);
  const std::string prefix = argv[4];
  const std::vector<Config> cs = suite(sname);
  std::FILE* fr = std::fopen((prefix + "_runs.csv").c_str(), "w");
  std::FILE* fa = std::fopen((prefix + "_agg.csv").c_str(), "w");
  if (!fr || !fa) { std::perror("open"); return 1; }
  const char* cfg_hdr = "cfg,dur,motion,antenna,gyro_bias_dps,acc_bias,ndt,drop,reseed,noise";
  std::fprintf(fr, "%s,scheme,seed,diverged,t_div,drop_xy,max_xy,max_z,max_tilt,max_head,end_xy,end_z,end_tilt,end_head\n", cfg_hdr);
  std::fprintf(fa, "%s,scheme,nseed,nepoch,nees_pos_end,nees_att_end,nees_all_end,nees_pos_mean,nees_att_mean,nees_all_mean,"
                   "pos_lo,pos_hi,att_lo,att_hi,all_lo,all_hi,frac_pos_in,frac_pos_above,frac_att_in,frac_att_above,frac_all_in,frac_all_above\n",
               cfg_hdr);
  for (const Config& c : cs) {
    if (c.id % nshards != shard) continue;
    std::vector<Data> data;
    for (int s = 0; s < c.nseed; ++s) data.push_back(makeData(c, kSeed + 1000u * c.id + s));
    char cfg[160];
    std::snprintf(cfg, sizeof cfg, "%d,%d,%s,%s,%g,%g,%d,%d,%d,%s", c.id, c.dur, kMotionName[c.motion], kAntennaName[c.ant],
                  c.gbias_dps, c.abias, c.ndt, c.drop, c.reseed, c.matched ? "matched" : "yaml");
    for (int vi = 0; vi < kNumVariants; ++vi) {
      std::vector<RunOut> outs;
      for (int s = 0; s < c.nseed; ++s) outs.push_back(runScheme(c, kVariants[vi], data[s]));
      for (int s = 0; s < c.nseed; ++s) {
        const RunOut& o = outs[s];
        std::fprintf(fr, "%s,%s,%d,%d,%.1f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.5f,%.5f\n", cfg, kVariants[vi].name, s, o.diverged ? 1 : 0,
                     o.t_div, o.drop_xy, o.max_xy, o.max_z, o.max_tilt, o.max_head, o.end_xy, o.end_z, o.end_tilt, o.end_head);
      }
      // seed-mean NEES at each epoch, against the chi-square band of S seeds
      const int ne = static_cast<int>(outs[0].nees.size());
      double end[3] = {0, 0, 0}, mean[3] = {0, 0, 0}, lo[3], hi[3], in[3] = {0, 0, 0}, above[3] = {0, 0, 0};
      const double dof[3] = {3, 3, 15};
      for (int b = 0; b < 3; ++b) {
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
      std::fprintf(fa, "%s,%s,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                   cfg, kVariants[vi].name, c.nseed, ne, end[0], end[1], end[2], mean[0], mean[1], mean[2], lo[0], hi[0], lo[1], hi[1],
                   lo[2], hi[2], in[0], above[0], in[1], above[1], in[2], above[2]);
    }
    std::fflush(fr); std::fflush(fa);
  }
  std::fclose(fr); std::fclose(fa);
  return 0;
}
