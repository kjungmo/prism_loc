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
//   S7        S1s + a bound on the heading standard deviation the lever update sees
//             (EskfParams::lever_heading_sigma_max = 0.7 rad), nothing released, and the
//             attitude of a reseed taken from the accelerometer at the /initialpose heading
//   S7a       S7 + the consider states released while the IMU measures horizontal
//             acceleration (lever_release_dv = 1.0 m/s over a 4 s decaying window); the
//             speed gate is off. S7a35, S7a50: the bound 0.35 and 0.5 rad. S7r05, S7r20: the
//             release threshold 0.5 and 2.0 m/s. S7n: S7a, the reseed attitude as /initialpose
//             sends it (level), for the sloped-reseed suite.
//   S7a35r05, S7a35r20: S7a35 with the release threshold 0.5 and 2.0 m/s (added after the first results, when the bound of
//             0.35 rad was ahead of 0.5 and 0.7 rad: the thresholds had been swept at 0.7 only)
//   Suites g60 and g300 (study 6, first pass) run the 15 schemes above S7; the suites h* (study 6,
//   second pass: the observability grid below) run S0, S1s, S2s, S6vb and the S7 family.
//
//   S8r, S8a, S8   (third pass, below) S7a35 + the honest heading at the release; S8 is what the code ships.
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
// Second pass (suites h300, h600, h900, h1800, hndt, hslope): the observability grid. It was
// written, with the failure criteria below, BEFORE the first run of any h suite; the exploratory
// prototype runs that led to the S7 family (a copy of this file with the S7 schemes, 8 suites of
// 300 to 1,800 s) are not part of the result. What is added to the first pass:
//   motions        line (1.5 m/s along x, heading constant, no acceleration), startstop (a
//                  40 s cycle: still 6 s, 0 -> 1.5 m/s in 3 s, 20 s at 1.5 m/s, 1.5 -> 0 in
//                  3 s, still 8 s; heading constant), and stand, turn, drive, spin as before
//   duration       300 s (suite h300, all motions, 2 seeds), 600 s (h600, standing, 8 seeds),
//                  900 s (h900, standing and line, 8 seeds), 1,800 s (h1800, standing and line, 8 seeds)
//   gyro bias      0, 0.05, 0.1, 0.2, 0.5 deg/s (h300); accelerometer bias 0, 0.1, 0.3 m/s^2
//   GNSS dropout   none, 30 s, 120 s (at 40 % of the run for 600 s and more, at 120 s of 300)
//   reseed         none, level (/initialpose), and on a tilted mount: suite hslope, 60 s, the body
//                  tilted by 0.05 and 0.1 rad (pitch, roll half of it, opposite sign) from the start;
//                  /initialpose at 30 s sends a level attitude, as RViz does. A reseed with no IMU
//                  sample yet waits for the first one in the node (tilt_seed_pending_) and then does
//                  what a reseed with a sample does; the simulation has the sample at every reseed,
//                  the node check scenarios gnss_pose_first* cover the waiting path.
//   antennas       A, B, Z (zero lever) and M, a 1.5 m mast (0, 0, 1.5): no horizontal lever arm
//   NDT            off (h300 ... h1800, hslope) and on (hndt, hslope)
//   vibration      hvib: 300 s, standing and line, gyro bias 0.1 deg/s, accelerometer bias 0.1, accelerometer white noise 0.02 (as
//                  everywhere else), 0.05, 0.1, 0.2 and 0.4 m/s^2 per axis, told to the filter (column noise = vib<value>): how often the
//                  release opens on a robot that does not accelerate (added with the release thresholds, after the first results)
//   start heading  0.1 rad off at the start (all suites but hyaw); hyaw: 0.5, 1 and 2 rad off, 300 s, to
//                  see whether a bound on the heading uncertainty keeps a scheme from recovering a
//                  heading that is far off (reported, not a criterion: the heading is declared unobserved
//                  while the robot stands; driving with acceleration releases it)
//   data           the IMU and fix noise of a run is the same for every antenna of a configuration
//                  (the data seed is that of the antenna-A row), so the zero-lever row is the
//                  paired reference of the lever rows
// Per run the failures are counted apart (columns of the runs file):
//   tz       tilt > 0.05 rad or |z| > 0.5 m at an epoch outside the dropout and 5 s after the
//            reseed windows (the failure a deployer sees: the robot leans and the height runs)
//   circ     max over epochs of  xy - (2 |t_xy| sin(|heading error| / 2) + 0.3 m)  > 0: the
//            horizontal error of base_link beyond the circle of radius |t_xy| around the
//            antenna that a standing robot cannot leave, plus 0.3 m
//   head     heading error > 0.5 rad
//   xy       horizontal error > 1 m
// Selection among the candidates S7, S7a, S7a35, S7a50, S7r05, S7r20, S7a35r05, S7a35r20 (S7n is an ablation of the reseed
// rule, not a candidate; S0, S1s, S2s, S6vb are comparators), on the GNSS-and-IMU-only rows with a lever
// arm (A, B, M). "Envelope cells" = stand, line, startstop and turn at gyro bias <= 0.2 deg/s, every
// duration. experiments/eskf_lever_report.py prints every line of this and the verdict. The order:
//   1. E = number of runs with tz or circ in the envelope cells. A candidate with E > 0 is excluded.
//   2. X = sum over the other cells (drive, spin, 0.5 deg/s) and the lever antennas of
//      max(0, tz of the scheme - tz of the zero-lever row of the same data). Smaller is better.
//   3. C = (position NEES in the 95 % band in a smaller share of the standing configurations than S1s)
//      + (tilt NEES, 2 dof, below its band in more than 20 % of the configurations). The heading NEES
//      (1 dof) is reported apart: it is optimistic once the true error exceeds the bound, by construction.
//      Smaller is better.
//   4. H = 1 if the scheme releases the heading and has more head failures than S6vb in the drive cells.
//   5. median of max xy over the envelope cells, smaller is better.
//   6. number of parameters changed from the defaults (the bound, the release threshold).
// If every candidate has E > 0 (the grid includes cells in which the zero-lever filter fails too: the accelerometer bias of
// 0.3 m/s^2, the 120 s dropouts), the order above is applied to all of them with E as its first key, smaller is better; the
// report also prints E', the runs of E that the zero-lever row of the same data survives. This sentence was added to the
// declaration after a first look at the h300 suite had shown that the reference fails in those cells; the criteria themselves
// and the cells were not changed.
// After the first results two things were added that this declaration does not cover: the schemes S7a35r05 and S7a35r20, and the
// suite hvib (accelerometer noise). With them the order above puts S7a35r05 (release at 0.5 m/s) ahead of S7a35 (1.0 m/s) by 3 runs of
// E; hvib shows the release at 0.5 m/s opening on a standing robot at an accelerometer noise of 0.1 m/s^2 per axis, so the code ships
// S7a35, and eskf_lever_report.py says that this is a deviation from the order.
// head failures are reported against the zero-lever row and are not a criterion for exclusion: a scheme
// that declares the heading unobserved while the robot stands does not estimate it. The chosen scheme
// is the first by the order (1 to 6) and the report lists which of 2, 3, 4 it fails.
//
// Third pass (suites hsg and hsgv): standing, then moving. The fourth review found that a robot on GNSS and IMU that stands for minutes
// and then starts tilts the filter and runs the height away (S7a35: the bound holds the heading sigma at 0.35 rad while the true error,
// 1.1 rad after 600 s at a vertical gyro bias of 0.002 rad/s, keeps growing; the release at the start runs a full update on a heading
// about 3 sigma off). No motion of the second pass stood longer than 14 s before it accelerated. The suite and the bar below were
// written after the reviewer's probe (a copy of this file with a stand-then-go motion) had shown the failure, and before the first run of
// the suite as committed; the schemes S8r, S8a and S8 were developed on its runs (several exploratory passes, not part of the result;
// they found a variance that went indefinite, a restore that fired at every update, a rate and a dropout in which the alignment must not act).
// hsgv is the same configurations on other data (kSeedV), run once after the schemes were fixed.
//   motions        sgstart (stand, then the startstop cycle from rest), sgdrive (stand, then 0 -> 1.5 m/s in 3 s, then a circle of radius 6 m at
//                  1.5 m/s), sgturn (stand, then the turn-on-the-spot cycle, 0.3 rad/s for 4 s of every 5 s); 160 s of motion
//   standing       300, 600, 900, 1,800 s; gyro bias 0, 0.05, 0.1, 0.2 deg/s per axis (random signs); accelerometer bias 0, 0.1 m/s^2;
//                  antennas A, B, M, Z; 4 seeds; GNSS and IMU only, no dropout, no reseed
//   schemes        S2s, S6vb, S7a35 (the comparators), S8r (S7a35 + the shadow variance put back at the release), S8a (+ the kinematic
//                  alignment from a change of velocity of 0.25 m/s), S8 (+ the blind update: lever_blind)
//   the bar        (1) in each motion the lever-arm rows have no more tz than the zero-lever rows of the same data (summed over pairs); (2) no
//                  lever-arm run with |z| > 1 m or tilt > 0.3 rad in sgstart or sgdrive with the antennas A and B (they have a horizontal arm);
//                  reported besides: the excess (tz where the paired zero-lever run has none), circ, xy, heading at the end.
//   regression     S8 is also run on every cell of the second pass (LEVER_ONLY=S8 and experiments/merge_rows.py), so that the second-pass tables
//                  show it next to S7a35.
// Fifth pass (suites hgap and hboot; the fifth review round). Two failures of S8 that the reviewer's probes (copies of this file with the two suites) showed:
//   hgap   the GNSS fixes stop 10 s before the robot starts (the start-stop cycle, or the circle) and come back after 30 s or 120 s: the kinematic
//          alignment took the last fix before the outage and the IMU sum of the whole outage as one change of velocity. Stand 300 or 600 s, gyro bias 0, 0.1, 0.2 deg/s,
//          accelerometer bias 0, 0.1 m/s^2, antennas A, B, M, Z, 4 seeds. S8o is S8 without the check of the two spans (EskfParams::lever_align_span_guard = false).
//   hboot  the default boot of a robot on GNSS and IMU with a lever arm: no /initialpose, initial_yaw 0.5, 1, 2 or 3 rad off (the harness's yaw0), a stand of 6, 30, 60, 90, 120 or
//          300 s, then the start-stop cycle or the circle, gyro bias 0.1 deg/s, accelerometer bias 0.1 m/s^2. S8 starts with the 0.71 rad of heading spread of its own covariance;
//          S8u (what the code ships) holds the spread of an unknown heading, pi / sqrt(3) rad, in the shadow at the first start (Eskf::markHeadingUnknown), so that the first start aligns.
//   Like the suites hsg and hsgv they were written after the probes had shown the failures and run once as committed. No bar was declared before the probes: the reviewer's prototype of S8u
//   came first. S8u was chosen because it removes the failure on hboot, and then run on every earlier suite (LEVER_ONLY=S8,S8u) to see what it costs; the costs are counts in
//   README.md ("Fifth pass") and report.md, not a criterion that decided. The mast M (no horizontal arm) is the cell where S8u does not help.
// Build and run from the repository root (see experiments/README.md, study 6):
//   g++ -O2 -std=c++17 -I prism_loc_fusion/include -I /usr/include/eigen3
//       prism_loc_fusion/src/*.cpp experiments/eskf_lever.cpp -o /tmp/exp_lever
//   /tmp/exp_lever g60 0 1 out/lever_g60      # suite, shard, number of shards, output prefix
// (suites: g60 g300 smoke h300 h600 h900 h1800 hndt hslope hyaw hvib hsg hsgv hgap hboot hsmoke)
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
constexpr std::uint64_t kSeedV = 20271019u;   // suite hsgv: the configurations of hsg on other data (a check of the schemes developed on hsg)

const Vec3 kAntenna[4] = {Vec3(0.5, 0.0, 1.0), Vec3(-0.3, 0.2, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 1.5)};
const char* kAntennaName[4] = {"A", "B", "Z", "M"};
const char* kMotionName[9] = {"stand", "turn", "drive", "spin", "line", "startstop", "sgstart", "sgturn", "sgdrive"};

struct Variant {
  const char* name;
  bool ignore;       // do not model the lever arm (main's behaviour)
  unsigned consider;
  int iterations;
  bool seeded;
  double gate_speed{0.0}, gate_rate{0.0};   // motion gate (EskfParams::lever_motion_*); 0 = none
  unsigned consider_moving{0u};
  double cap{0.0}, rel_dv{0.0};             // EskfParams::lever_heading_sigma_max (rad), lever_release_dv (m/s); 0 = off
  bool reseed_accel{false};                 // a reseed takes roll and pitch from the accelerometer
  bool restore{false};                      // EskfParams::lever_restore (the honest heading at a release); off for every scheme of the earlier passes
  double align{0.0};                        // EskfParams::lever_align_sigma, rad
  double align_dv{0.25};                    // EskfParams::lever_align_dv, m/s
  bool blind{false};                        // EskfParams::lever_blind
  double start_sigma{0.0};                  // EskfParams::lever_start_sigma, rad: the heading of the first start is a guess (0 = trusted, as before)
  bool span_guard{true};                    // EskfParams::lever_align_span_guard (false: S8o, the version before the fifth review)
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
constexpr int kNumFirstPass = sizeof(kVariants) / sizeof(kVariants[0]);
// Second pass (suites h*): the reference schemes and the S7 family.
const Variant kVariantsH[] = {
    {"S0", false, kConsiderAttitudeAndGyroBias, 1, false},
    {"S1s", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true},
    {"S2s", false, 0, 1, true},
    {"S6vb", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.6, 0.0, 0u},
    {"S7", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.7, 0.0, true},
    {"S7a", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.7, 1.0, true},
    {"S7a35", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true},
    {"S7a50", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.5, 1.0, true},
    {"S7r05", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.7, 0.5, true},
    {"S7r20", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.7, 2.0, true},
    {"S7n", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.7, 1.0, false},
    // Added after the first results (the bound of 0.35 rad was ahead of 0.5 and 0.7): the release thresholds at that bound
    {"S7a35r05", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 0.5, true},
    {"S7a35r20", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 2.0, true},
    // Third pass (suite hsg): S7a35 + the honest heading at the release. S8r: the shadow variance is put back at the release, and
    // that is all. S8a: and a heading over 1 rad of spread is first re-acquired by the kinematic alignment (Eskf::alignHeading,
    // from a change of velocity of 0.25 m/s) instead of a linearised update. S8: and, while the heading is unknown and not alignable,
    // the lever Jacobian leaves out the attitude columns (lever_blind). S8 is what the code ships (EskfParams defaults).
    {"S8r", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 0.0},
    {"S8a", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 1.0},
    {"S8", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 1.0, 0.25, true},
    // S8s: S8 with lever_align_sigma 0.6 rad (the alignment also at a start from a stand of a few seconds): a trial, run with LEVER_ONLY=S8s
    // S8u (fifth pass): S8 + a start whose heading is a guess (initial_yaw, no /initialpose) holds the spread of an unknown heading,
    // pi / sqrt(3) rad, in the shadow (Eskf::markHeadingUnknown). S8u is what the code ships; S8 is the version before. The harness marks only the
    // first start of a run, a reseed (/initialpose) is a heading source and is not marked.
    // S8o: S8 with the span guard of the alignment off (EskfParams::lever_align_span_guard = false), the version of the fourth review round's fix before
    // the fifth review: what an outage across a start did to it (suite hgap).
    {"S8o", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 1.0, 0.25, true, 0.0, false},
    {"S8u", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 1.0, 0.25, true, 1.8137993642342178},
    {"S8s", false, kConsiderHeading | kConsiderGyroBiasVertical, 1, true, 0.0, 0.0, 0u, 0.35, 1.0, true, true, 0.6, 0.25, true},
};
constexpr int kNumH = sizeof(kVariantsH) / sizeof(kVariantsH[0]);

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
  ep.lever_iterations = v.iterations;
  ep.lever_motion_speed = v.gate_speed;
  ep.lever_motion_rate = v.gate_rate;
  ep.lever_consider_moving = v.consider_moving;
  ep.lever_heading_sigma_max = v.cap;
  ep.lever_release_dv = v.rel_dv;
  ep.lever_restore = v.restore;
  ep.lever_align_sigma = v.align;
  ep.lever_align_dv = v.align_dv;
  ep.lever_blind = v.blind;
  ep.lever_start_sigma = v.start_sigma;
  ep.lever_align_span_guard = v.span_guard;
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
  const double drop0 = c.stand_s > 0.0 ? c.stand_s - 10.0 : (c.dur == 60 ? 30.0 : (c.dur <= 300 ? 120.0 : 0.4 * c.dur));
  const double drop1 = drop0 + (c.drop_s >= 0.0 ? c.drop_s : (c.dur == 60 ? 10.0 : 30.0));
  const double tsq = lever.head<2>().norm();       // horizontal lever arm, for the circle criterion
  long n_lever = 0, n_capped = 0, n_open = 0;
  const int gnss_every = static_cast<int>(std::llround(kGnssPeriod / kDt));
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
    if (k % gnss_every == 0 && !(c.drop && t >= drop0 && t < drop1)) {
      const Truth T = truthOf(c, t);
      const Vec3 fix = T.p + bodyRot(c, T.yaw) * lever + d.gnss_noise[i];
      if (lever_on) {
        if (!lever.isZero(0.0)) { ++n_lever; if (f.leverReleaseOpen()) ++n_open; }
        f.updatePositionLever(fix, lever, Rg);
        if (!lever.isZero(0.0) && f.leverHeadingCapped()) ++n_capped;
      } else f.updatePosition(fix, Rg);
    }
    if (c.ndt && k % gnss_every == gnss_every / 2) {
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
  } else if (name == "hboot") {   // fifth pass: the default boot, a wrong initial_yaw (0.5 to 3 rad), no /initialpose, a short or long stand, then the start
    for (int stand : {6, 30, 60, 90, 120, 300})
      for (double yaw0 : {0.5, 1.0, 2.0, 3.0})
        for (int motion : {6, 8}) {
          const int first = id;
          addH(stand + 160, motion, 0.1, 0.1, false, 0.0, false, 0.0, 4, yaw0);
          for (int i = first; i < id; ++i) cs[i].stand_s = stand;
        }
  } else if (name == "hsmoke") {
    addH(60, 0, 0.1, 0.0, false, 0.0, false, 0.0, 2);
    addH(60, 5, 0.5, 0.1, true, 10.0, true, 0.05, 2);
  }
  return cs;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5) { std::fprintf(stderr, "usage: %s g60|g300|smoke|h300|h600|h900|h1800|hndt|hslope|hyaw|hvib|hsg|hsgv|hgap|hboot|hsmoke SHARD NSHARDS OUT_PREFIX\n", argv[0]); return 2; }
  const std::string sname = argv[1];
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
    std::snprintf(cfg, sizeof cfg, "%d,%d,%s,%s,%g,%g,%d,%d,%d,%s,%g,%g,%g", c.id, c.dur, kMotionName[c.motion], kAntennaName[c.ant],
                  c.gbias_dps, c.abias, c.ndt, c.drop, c.reseed, noise, c.drop_s, c.slope, c.yaw0);
    const Variant* variants = c.hsuite ? kVariantsH : kVariants;
    const int nvar = c.hsuite ? kNumH : kNumFirstPass;
    for (int vi = 0; vi < nvar; ++vi) {
      // LEVER_ONLY=name,name: run just these schemes (the rows of the others are what a full run writes; rerun to extend a result)
      if (const char* only = std::getenv("LEVER_ONLY")) {
        if (!std::strstr((std::string(",") + only + ",").c_str(), (std::string(",") + variants[vi].name + ",").c_str())) continue;
      } else if (c.hsuite) {
        // Without LEVER_ONLY: the second pass runs the schemes of the second pass (S0 ... S7a35r20), the third pass (suites hsg, hsgv) the
        // comparators S2s, S6vb, S7a35 and the S8 family (S8r, S8a, S8, S8u), the fifth pass (suites hgap, hboot) S8o, S8 and S8u.
        // The S8 and S8u rows of the second-pass suites are added with LEVER_ONLY=S8,S8u.
        const bool third = variants[vi].name[0] == 'S' && variants[vi].name[1] == '8';
        const std::string nm = variants[vi].name;
        if (nm == "S8s") continue;   // the trial of a lower alignment threshold (README, study 6): only with LEVER_ONLY=S8s
        const bool sg_set = third || nm == "S2s" || nm == "S6vb" || nm == "S7a35";
        if (nm == "S8o" && sname != "hgap" && sname != "hboot") continue;   // S8o is for the fifth-pass suites (and LEVER_ONLY=S8o)
        if (sname == "hgap" || sname == "hboot") { if (nm != "S8o" && nm != "S8" && nm != "S8u") continue; }   // fifth pass: S8o, S8, S8u
        else if (c.stand_s > 0.0 ? !sg_set : third) continue;
      }
      std::vector<RunOut> outs;
      for (int s = 0; s < c.nseed; ++s) outs.push_back(runScheme(c, variants[vi], data[s]));
      for (int s = 0; s < c.nseed; ++s) {
        const RunOut& o = outs[s];
        std::fprintf(fr, "%s,%s,%d,%d,%.1f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f\n", cfg, variants[vi].name, s, o.diverged ? 1 : 0,
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
                   cfg, variants[vi].name, c.nseed, ne, end[0], end[1], end[2], mean[0], mean[1], mean[2], lo[0], hi[0], lo[1], hi[1],
                   lo[2], hi[2], in[0], above[0], in[1], above[1], in[2], above[2],
                   end[3], mean[3], lo[3], hi[3], in[3], above[3], end[4], mean[4], lo[4], hi[4], in[4], above[4]);
    }
    std::fflush(fr); std::fflush(fa);
  }
  std::fclose(fr); std::fclose(fa);
  return 0;
}
