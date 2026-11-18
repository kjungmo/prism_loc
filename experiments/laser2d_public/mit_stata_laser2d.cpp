// mit_stata_laser2d.cpp -- the laser2d loop of prism_loc::LocalizationNode (likelihood-field MCL with
// KLD resampling, BBS global relocalization with the multi-scan verifier) on the public MIT Stata
// Center dataset (http://projects.csail.mit.edu/stata/, CC BY 3.0), driven offline without ROS.
//
// Inputs (see README.md): the occupancy map built from the dataset's as-built floor plan (map.bin,
// from stata_prepare.py), and one session's laser scans and ground-truth laser poses (the dataset's
// *.gt.laser.scans / *.gt.laser.poses). The dataset's scan files carry no odometry: the odometry is
// synthesized from ground-truth increments with a fixed error model (README.md, "Odometry").
//
//   mit_stata_laser2d MODE MAP_BIN SCANS POSES OUT_PREFIX [--param key=value]...
//
//   MODE track   MCL seeded at the ground truth (the node's /initialpose floor spread), whole run.
//   MODE reloc   N global relocalization queries (verifier from the scan at the query time, no prior).
//   MODE init    initial-pose offset levels L0-L6 and U at several start times (INIT_PROTOCOL.md).
//
// Node parameters are laser2d.yaml's defaults. Single thread, fixed seeds.
//
// Build (repository root):
//   g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3 prism_loc_core/src/*.cpp \
//       experiments/laser2d_public/mit_stata_laser2d.cpp -o /tmp/mit_stata_laser2d
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "prism_loc_core/bbs.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/occupancy_grid.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/relocalization.hpp"
#include "prism_loc_core/rng.hpp"
#include "prism_loc_core/types.hpp"
using namespace prism_loc_core;

namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point a) { return std::chrono::duration<double, std::milli>(Clock::now() - a).count(); }

std::map<std::string, double> g_par = {
    // laser2d.yaml defaults
    {"min_particles", 500}, {"max_particles", 2000}, {"resample_threshold", 0.5}, {"kld_err", 0.05},
    {"kld_z", 2.33}, {"kld_bin_xy", 0.5}, {"kld_bin_yaw", 0.17}, {"update_min_d", 0.2}, {"update_min_a", 0.2},
    {"alpha1", 0.2}, {"alpha2", 0.2}, {"alpha3", 0.2}, {"alpha4", 0.2}, {"max_beams", 60}, {"z_hit", 0.5},
    {"z_rand", 0.5}, {"sigma_hit", 0.2}, {"likelihood_max_dist", 2.0}, {"bbs_max_linear_window", 50.0},
    {"bbs_angular_step", 0.0175}, {"bbs_max_depth", 6}, {"bbs_max_beams", 120}, {"bbs_min_score_fraction", 0.4},
    {"bbs_verify_top_k", 8}, {"bbs_verify_scans", 9}, {"bbs_verify_evidence_gain", 40.0},
    {"bbs_verify_min_posterior", 0.9}, {"bbs_verify_nms_xy", 1.0}, {"bbs_verify_nms_yaw", 0.35},
    {"bbs_verify_track_linear_window", 0.3}, {"bbs_verify_track_angular_window", 0.1},
    // sensor (Hokuyo UTM-30LX) and synthetic odometry (README.md)
    {"range_min", 0.1}, {"range_max", 30.0},
    {"odom_trans_scale", 1.03}, {"odom_rot_gain", 1.03}, {"odom_noise", 0.01}, {"odom_yaw_bias_deg_s", 0.02},
    // experiment
    {"seed", 42}, {"queries", 50}, {"run_s", 180}, {"stride", 1}};
double P(const char* k) { return g_par.at(k); }

struct Scan { std::int64_t t; std::vector<float> r; double amin, ainc; };
struct GtPose { std::int64_t t; Pose2D p; };

GridMap readMap(const std::string& path) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) throw std::runtime_error("cannot read " + path);
  GridMap g;
  std::int32_t wh[2];
  double d[3];
  if (std::fread(wh, 4, 2, f) != 2 || std::fread(d, 8, 3, f) != 3) throw std::runtime_error("bad map");
  g.width = wh[0]; g.height = wh[1]; g.resolution = d[0]; g.origin_x = d[1]; g.origin_y = d[2];
  g.data.resize(static_cast<size_t>(g.width) * g.height);
  if (std::fread(g.data.data(), 1, g.data.size(), f) != g.data.size()) throw std::runtime_error("bad map data");
  std::fclose(f);
  return g;
}

// Scans: "t_us,angle_min,angle_inc,count[,ranges...]"; a row without ranges is skipped
// (its pose row is kept so rows stay aligned).
void readData(const std::string& scans, const std::string& poses, std::vector<Scan>& S, std::vector<GtPose>& G) {
  std::ifstream fp(poses), fs(scans);
  if (!fp || !fs) throw std::runtime_error("cannot read scans/poses");
  std::string a, b;
  while (std::getline(fp, a) && std::getline(fs, b)) {
    std::int64_t tp = 0; double x, y, th;
    if (std::sscanf(a.c_str(), "%ld,%lf,%lf,%lf", &tp, &x, &y, &th) != 4) continue;
    Scan s; int n = 0;
    const char* c = b.c_str(); char* e;
    s.t = std::strtoll(c, &e, 10); c = e + 1;
    s.amin = std::strtod(c, &e); c = e + 1;
    s.ainc = std::strtod(c, &e); c = e + 1;
    n = static_cast<int>(std::strtol(c, &e, 10)); c = e;
    if (n <= 0 || s.t != tp) continue;
    s.r.resize(n);
    for (int i = 0; i < n; ++i) { c = e + 1; s.r[i] = static_cast<float>(std::strtod(c, &e)); }
    S.push_back(std::move(s));
    G.push_back({tp, Pose2D{x, y, th}});
  }
}

LaserScan2D toScan(const Scan& s) {
  LaserScan2D l;
  l.ranges = s.r; l.angle_min = s.amin; l.angle_increment = s.ainc;
  l.range_min = P("range_min"); l.range_max = P("range_max");
  l.sensor_in_base = Pose2D{0, 0, 0};  // the dataset's ground truth is the laser pose: base = laser
  return l;
}

// Synthetic odometry: the odom frame starts at the ground-truth pose of scan 0 and integrates the
// ground-truth increment of every scan step, in the previous body frame, with a translation scale
// error, a rotation gain, a constant yaw bias and Gaussian noise (all fixed, not tuned).
std::vector<Pose2D> makeOdom(const std::vector<GtPose>& G, unsigned seed) {
  std::mt19937 rng(seed * 7919u + 11u);
  std::normal_distribution<double> nd(0.0, 1.0);
  std::vector<Pose2D> odom(G.size());
  odom[0] = G[0].p;
  for (size_t i = 1; i < G.size(); ++i) {
    const Pose2D d = compose(inverse(G[i - 1].p), G[i].p);
    const double dt = (G[i].t - G[i - 1].t) * 1e-6;
    const double tr = std::hypot(d.x, d.y), ang = std::atan2(d.y, d.x);
    const double n = P("odom_noise");
    const double t2 = tr * P("odom_trans_scale") * (1.0 + n * nd(rng));
    const double yaw2 = d.yaw * P("odom_rot_gain") * (1.0 + n * nd(rng)) + P("odom_yaw_bias_deg_s") * M_PI / 180.0 * dt;
    Pose2D inc{t2 * std::cos(ang), t2 * std::sin(ang), yaw2};
    odom[i] = compose(odom[i - 1], inc);
    odom[i].yaw = normalizeAngle(odom[i].yaw);
  }
  return odom;
}

ParticleFilterParams pfParams() {
  ParticleFilterParams pp;
  pp.min_particles = static_cast<int>(P("min_particles")); pp.max_particles = static_cast<int>(P("max_particles"));
  pp.resample_threshold = P("resample_threshold"); pp.kld_err = P("kld_err"); pp.kld_z = P("kld_z");
  pp.kld_bin_xy = P("kld_bin_xy"); pp.kld_bin_yaw = P("kld_bin_yaw");
  return pp;
}
BbsParams bbsParams() {
  BbsParams b;
  b.max_linear_window = P("bbs_max_linear_window"); b.angular_window = M_PI; b.angular_step = P("bbs_angular_step");
  b.max_depth = static_cast<int>(P("bbs_max_depth")); b.max_beams = static_cast<int>(P("bbs_max_beams"));
  b.min_score_fraction = P("bbs_min_score_fraction"); b.sigma_hit = P("sigma_hit");
  return b;
}
RelocVerifierParams rvParams() {
  RelocVerifierParams r;
  r.top_k = static_cast<int>(P("bbs_verify_top_k")); r.verify_scans = static_cast<int>(P("bbs_verify_scans"));
  r.evidence_gain = P("bbs_verify_evidence_gain"); r.min_posterior = P("bbs_verify_min_posterior");
  r.nms_xy = P("bbs_verify_nms_xy"); r.nms_yaw = P("bbs_verify_nms_yaw");
  r.track_linear_window = P("bbs_verify_track_linear_window"); r.track_angular_window = P("bbs_verify_track_angular_window");
  return r;
}

// The node's runUpdate()/onScan() state for the laser2d backend.
struct Node {
  const GridMap& map;
  Rng rng;
  ParticleFilter pf;
  OdometryMotionModel motion;
  Laser2DLikelihoodField laser;
  bool filter_init{false}, force_update{false}, have_last_odom{false};
  Pose2D last_odom, map_odom;
  bool have_map_odom{false};
  Node(const GridMap& m, unsigned seed)
      : map(m), rng(seed), pf(pfParams(), rng),
        motion(MotionParams{P("alpha1"), P("alpha2"), P("alpha3"), P("alpha4")}),
        laser(m, LaserParams{static_cast<int>(P("max_beams")), P("z_hit"), P("z_rand"), P("sigma_hit"), P("likelihood_max_dist")}) {}
  // onInitialPose with the pose covariance floor: sigma = 0.5 m, 0.5 m, sqrt(0.0685) rad.
  void initialPose(const Pose2D& mean, double sx, double sy, double sa) {
    const int n = pf.particles().empty() ? 2000 : static_cast<int>(pf.particles().size());
    pf.initializeGaussian(mean, Pose2D{std::sqrt(std::max(sx * sx, 0.25)), std::sqrt(std::max(sy * sy, 0.25)),
                                       std::sqrt(std::max(sa * sa, 0.0685))}, std::max(n, 500));
    filter_init = true; force_update = true; have_last_odom = false;
  }
  // The accepted-relocalization seed.
  void seedAccepted(const Pose2D& p) {
    const int n = static_cast<int>(pf.particles().size());
    pf.initializeGaussian(p, Pose2D{0.2, 0.2, 0.1}, n > 0 ? n : 2000);
    filter_init = true; force_update = true; have_last_odom = false;
  }
  // onScan -> runUpdate. Returns true when the filter updated.
  bool onScan(const LaserScan2D& scan, const Pose2D& odom_base) {
    if (!filter_init) return false;
    laser.setScan(scan);
    if (!have_last_odom) { last_odom = odom_base; have_last_odom = true; }
    const double dd = std::hypot(odom_base.x - last_odom.x, odom_base.y - last_odom.y);
    const double da = std::fabs(normalizeAngle(odom_base.yaw - last_odom.yaw));
    const bool moved = dd >= P("update_min_d") || da >= P("update_min_a");
    if (!(force_update || moved)) return false;
    pf.predict(motion, last_odom, odom_base);
    pf.correct(laser);
    pf.resample();
    last_odom = odom_base; force_update = false;
    const Pose2D base_in_map = pf.estimate();
    map_odom = compose(base_in_map, inverse(odom_base));
    have_map_odom = true;
    return true;
  }
  Pose2D poseNow(const Pose2D& odom_base) const { return compose(map_odom, odom_base); }
};

struct Err { double pos, yaw_deg; };
Err err(const Pose2D& e, const Pose2D& t) {
  return {std::hypot(e.x - t.x, e.y - t.y), std::fabs(normalizeAngle(e.yaw - t.yaw)) * 180.0 / M_PI};
}

// ---- MODE track ----
int runTrack(const GridMap& map, const std::vector<Scan>& S, const std::vector<GtPose>& G, const std::string& out) {
  const unsigned seed = static_cast<unsigned>(P("seed"));
  const auto odom = makeOdom(G, seed);
  Node node(map, seed);
  node.initialPose(G[0].p, 0.0, 0.0, 0.0);  // the floor spread, centred on the ground truth
  FILE* f = std::fopen((out + ".csv").c_str(), "w");
  std::fprintf(f, "utime,x,y,yaw,gt_x,gt_y,gt_yaw,err_xy,err_yaw_deg,updated,particles,odom_err_xy\n");
  const int stride = static_cast<int>(P("stride"));
  for (size_t i = 0; i < S.size(); ++i) {
    const bool upd = node.onScan(toScan(S[i]), odom[i]);
    if (!node.have_map_odom) continue;
    const Pose2D e = node.poseNow(odom[i]);
    const Err er = err(e, G[i].p);
    if (i % stride == 0 || upd)
      std::fprintf(f, "%ld,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%.3f,%d,%zu,%.3f\n", static_cast<long>(S[i].t), e.x, e.y,
                   e.yaw, G[i].p.x, G[i].p.y, G[i].p.yaw, er.pos, er.yaw_deg, upd ? 1 : 0, node.pf.particles().size(),
                   std::hypot(odom[i].x - G[i].p.x, odom[i].y - G[i].p.y));
  }
  std::fclose(f);
  return 0;
}

// Runs the node's relocalization path from scan index i0: start() on the scan, update() on the
// following scans with the odometry increment, until the verifier leaves kPending.
struct RelocOut { RelocStatus st; Pose2D pose; int scans; size_t hyps; double post; double ms_start, ms_total; int last_i; };
RelocOut relocalize(const BranchAndBoundMatcher& bbs, const std::vector<Scan>& S, const std::vector<Pose2D>& odom, size_t i0) {
  RelocalizationVerifier ver(bbs, rvParams());
  RelocOut o{};
  const auto t0 = Clock::now();
  RelocStatus st = ver.start(toScan(S[i0]));
  o.ms_start = msSince(t0);
  size_t last = i0;
  size_t i = i0;
  while (st == RelocStatus::kPending && i + 1 < S.size()) {
    ++i;
    st = ver.update(compose(inverse(odom[last]), odom[i]), toScan(S[i]));
    last = i;
  }
  o.ms_total = msSince(t0);
  o.st = st; o.pose = ver.pose(); o.scans = ver.scansUsed(); o.hyps = ver.hypotheses().size();
  o.post = ver.bestPosterior(); o.last_i = static_cast<int>(i);
  return o;
}
const char* stName(RelocStatus s) { return toString(s); }

// ---- MODE reloc ----
int runReloc(const GridMap& map, const std::vector<Scan>& S, const std::vector<GtPose>& G, const std::string& out) {
  const unsigned seed = static_cast<unsigned>(P("seed"));
  const auto odom = makeOdom(G, seed);
  BranchAndBoundMatcher bbs(map, bbsParams());
  std::fprintf(stderr, "bbs window covers map: %d (half window %d x %d cells)\n", bbs.globalWindowCoversMap() ? 1 : 0,
               bbs.globalHalfWindowX(), bbs.globalHalfWindowY());
  const int N = static_cast<int>(P("queries"));
  FILE* f = std::fopen((out + ".csv").c_str(), "w");
  std::fprintf(f, "query,scan_index,t_s,status,x,y,yaw,gt_x,gt_y,gt_yaw,err_xy,err_yaw_deg,correct,scans_used,hypotheses,posterior,ms_first_scan,ms_total,single_valid,single_err_xy,single_err_yaw_deg,single_correct\n");
  const size_t margin = 100;  // keep room for the verification scans
  for (int q = 0; q < N; ++q) {
    const size_t i0 = margin + (S.size() - 2 * margin) * q / std::max(1, N - 1);
    const RelocOut o = relocalize(bbs, S, odom, std::min(i0, S.size() - 60));
    const Err er = err(o.pose, G[o.last_i].p);
    const bool correct = er.pos < 0.5 && er.yaw_deg < 10.0;
    // The v0.2 single-scan rule on the same first scan, for reference.
    const BbsResult single = bbs.matchGlobal(toScan(S[i0]));
    const Err es = err(single.pose, G[i0].p);
    std::fprintf(f, "%d,%zu,%.1f,%s,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.3f,%.2f,%d,%d,%zu,%.4f,%.1f,%.1f,%d,%.3f,%.2f,%d\n", q, i0,
                 (S[i0].t - S[0].t) * 1e-6, stName(o.st), o.pose.x, o.pose.y, o.pose.yaw, G[o.last_i].p.x, G[o.last_i].p.y,
                 G[o.last_i].p.yaw, er.pos, er.yaw_deg, correct ? 1 : 0, o.scans, o.hyps, o.post, o.ms_start, o.ms_total,
                 single.valid ? 1 : 0, es.pos, es.yaw_deg, (single.valid && es.pos < 0.5 && es.yaw_deg < 10.0) ? 1 : 0);
    std::fflush(f);
  }
  std::fclose(f);
  return 0;
}

// ---- MODE init ----
// Levels as experiments/nclt/INIT_PROTOCOL.md. U: BBS verifier from the start scan, then tracking.
struct Level { const char* name; double d, a_deg; };
int runInit(const GridMap& map, const std::vector<Scan>& S, const std::vector<GtPose>& G, const std::string& out) {
  const Level levels[] = {{"L0", 0, 0}, {"L1", 2, 15}, {"L2", 5, 15}, {"L3", 10, 15}, {"L4", 2, 45}, {"L5", 2, 90}, {"L6", 10, 90}, {"U", -1, -1}};
  const unsigned seeds[] = {42, 1, 2};
  std::vector<double> starts;  // start times (s from the first scan)
  const char* env = std::getenv("INIT_STARTS");
  std::stringstream ss(env ? env : "400,800,1200,1600");
  for (std::string tok; std::getline(ss, tok, ',');) starts.push_back(std::atof(tok.c_str()));
  const double run_s = P("run_s");
  BranchAndBoundMatcher bbs(map, bbsParams());
  FILE* f = std::fopen((out + ".csv").c_str(), "w");
  std::fprintf(f, "start_s,level,seed,converged,conv_time_s,rmse_after,note,final_err_xy,reloc_status,reloc_ms\n");
  for (double st : starts) {
    size_t i0 = 0;
    while (i0 < S.size() && (S[i0].t - S[0].t) * 1e-6 < st) ++i0;
    if (i0 >= S.size()) continue;
    size_t i1 = i0;
    while (i1 < S.size() && (S[i1].t - S[i0].t) * 1e-6 < run_s) ++i1;
    for (const Level& lv : levels) {
      for (unsigned seed : seeds) {
        std::mt19937 rng(seed * 100003u + static_cast<unsigned>(st) * 31u + 7u);
        std::uniform_real_distribution<double> ang(-M_PI, M_PI);
        const double dir = ang(rng);
        const double sgn = (ang(rng) > 0) ? 1.0 : -1.0;  // same draw for every level at (start, seed)
        const auto odom = makeOdom(G, seed);
        Node node(map, seed);
        std::string status = "-"; double reloc_ms = 0;
        size_t first = i0;
        if (lv.d < 0) {
          // Level U: global localization. The verifier decides after verify_scans scans; if it is
          // not accepted the node retries from the next scan (a new start()).
          const auto t0 = Clock::now();
          size_t i = i0;
          bool accepted = false;
          while (i + 60 < i1) {
            const RelocOut o = relocalize(bbs, S, odom, i);
            status = stName(o.st);
            if (o.st == RelocStatus::kAccepted) {
              node.seedAccepted(o.pose);
              node.last_odom = odom[o.last_i];
              first = static_cast<size_t>(o.last_i);
              accepted = true;
              break;
            }
            i = static_cast<size_t>(o.last_i) + 1;
          }
          reloc_ms = msSince(t0);
          if (!accepted) first = i1;  // never seeded within the run
        } else {
          const Pose2D g = G[i0].p;
          Pose2D start{g.x + lv.d * std::cos(dir), g.y + lv.d * std::sin(dir), normalizeAngle(g.yaw + sgn * lv.a_deg * M_PI / 180.0)};
          if (lv.d == 0) start = g;
          // The spread the user would give for that guess: sigma max(0.5, d) m, max(15 deg, a) rad.
          const double sxy = std::max(0.5, lv.d);
          const double sa = std::max(15.0, lv.a_deg) * M_PI / 180.0;
          node.initialPose(start, sxy, sxy, sa);
        }
        // Run and score.
        std::vector<double> t, e;
        for (size_t i = first; i < i1; ++i) {
          node.onScan(toScan(S[i]), odom[i]);
          if (!node.have_map_odom) continue;
          t.push_back((S[i].t - S[i0].t) * 1e-6);
          e.push_back(err(node.poseNow(odom[i]), G[i].p).pos);
        }
        // Converged: planar error < 1 m for >= 10 s continuously, starting within the first 120 s.
        bool conv = false; double ct = -1, rm = -1;
        for (size_t k = 0; k < t.size() && t[k] <= 120.0 && !conv; ++k) {
          if (e[k] >= 1.0) continue;
          size_t m = k;
          while (m < t.size() && e[m] < 1.0) ++m;
          if (t[m - 1] - t[k] >= 10.0) { conv = true; ct = t[k]; }
          else k = m;
        }
        if (conv) {
          double s2 = 0; int n = 0;
          for (size_t k = 0; k < t.size(); ++k) if (t[k] >= ct) { s2 += e[k] * e[k]; ++n; }
          rm = std::sqrt(s2 / std::max(1, n));
        }
        std::fprintf(f, "%.0f,%s,%u,%d,%.1f,%.3f,%s,%.3f,%s,%.0f\n", st, lv.name, seed, conv ? 1 : 0, ct, rm, "",
                     e.empty() ? -1.0 : e.back(), status.c_str(), reloc_ms);
        std::fflush(f);
      }
    }
  }
  std::fclose(f);
  return 0;
}

// ---- MODE dump ----
// For the AMCL baseline (experiments/nclt/INIT_PROTOCOL_AMCL.md): writes the map in map_server format
// (OUT.pgm, OUT.yaml), the synthetic odometry of each seed (OUT_odom_s<seed>.csv: utime,x,y,yaw) and the
// initial poses of the init mode (OUT_jobs.csv: start_s,level,seed,i0,i1,x,y,yaw,sx,sy,sa), computed by the
// same code as runInit, so that AMCL starts from exactly the poses laser2d did.
int runDump(const GridMap& map, const std::vector<Scan>& S, const std::vector<GtPose>& G, const std::string& out) {
  {
    FILE* pg = std::fopen((out + ".pgm").c_str(), "wb");
    std::fprintf(pg, "P5\n%d %d\n255\n", map.width, map.height);
    std::vector<unsigned char> row(static_cast<size_t>(map.width));
    for (int r = 0; r < map.height; ++r) {
      const int gy = map.height - 1 - r;
      for (int c = 0; c < map.width; ++c) row[static_cast<size_t>(c)] = map.data[static_cast<size_t>(gy) * map.width + c] > 50 ? 0 : 254;
      std::fwrite(row.data(), 1, row.size(), pg);
    }
    std::fclose(pg);
    const std::string base = out.substr(out.find_last_of('/') + 1);
    FILE* ym = std::fopen((out + ".yaml").c_str(), "w");
    std::fprintf(ym, "image: %s.pgm\nmode: trinary\nresolution: %.4f\norigin: [%.4f, %.4f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
                 base.c_str(), map.resolution, map.origin_x, map.origin_y);
    std::fclose(ym);
  }
  for (unsigned seed : {42u, 1u, 2u}) {
    const auto odom = makeOdom(G, seed);
    FILE* f = std::fopen((out + "_odom_s" + std::to_string(seed) + ".csv").c_str(), "w");
    for (size_t i = 0; i < odom.size(); ++i)
      std::fprintf(f, "%ld,%.6f,%.6f,%.8f\n", static_cast<long>(S[i].t), odom[i].x, odom[i].y, odom[i].yaw);
    std::fclose(f);
  }
  const Level levels[] = {{"L0", 0, 0}, {"L1", 2, 15}, {"L2", 5, 15}, {"L3", 10, 15}, {"L4", 2, 45}, {"L5", 2, 90}, {"L6", 10, 90}};
  std::vector<double> starts;
  const char* env = std::getenv("INIT_STARTS");
  std::stringstream ss(env ? env : "400,800,1200,1600");
  for (std::string tok; std::getline(ss, tok, ',');) starts.push_back(std::atof(tok.c_str()));
  const double run_s = P("run_s");
  FILE* f = std::fopen((out + "_jobs.csv").c_str(), "w");
  std::fprintf(f, "start_s,level,seed,i0,i1,x,y,yaw,sx,sy,sa\n");
  for (double st : starts) {
    size_t i0 = 0;
    while (i0 < S.size() && (S[i0].t - S[0].t) * 1e-6 < st) ++i0;
    if (i0 >= S.size()) continue;
    size_t i1 = i0;
    while (i1 < S.size() && (S[i1].t - S[i0].t) * 1e-6 < run_s) ++i1;
    for (const Level& lv : levels)
      for (unsigned seed : {42u, 1u, 2u}) {
        std::mt19937 rng(seed * 100003u + static_cast<unsigned>(st) * 31u + 7u);
        std::uniform_real_distribution<double> ang(-M_PI, M_PI);
        const double dir = ang(rng);
        const double sgn = (ang(rng) > 0) ? 1.0 : -1.0;
        const Pose2D g = G[i0].p;
        Pose2D start{g.x + lv.d * std::cos(dir), g.y + lv.d * std::sin(dir), normalizeAngle(g.yaw + sgn * lv.a_deg * M_PI / 180.0)};
        if (lv.d == 0) start = g;
        const double sxy = std::max(0.5, lv.d), sa = std::max(15.0, lv.a_deg) * M_PI / 180.0;
        std::fprintf(f, "%.0f,%s,%u,%zu,%zu,%.6f,%.6f,%.8f,%.4f,%.4f,%.6f\n", st, lv.name, seed, i0, i1, start.x, start.y, start.yaw, sxy, sxy, sa);
      }
  }
  std::fclose(f);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 6) {
    std::fprintf(stderr, "usage: %s track|reloc|init MAP_BIN SCANS POSES OUT_PREFIX [--param key=value]...\n", argv[0]);
    return 2;
  }
  for (int i = 6; i < argc; ++i) {
    if (std::strcmp(argv[i], "--param") == 0 && i + 1 < argc) {
      const std::string kv = argv[++i];
      const auto eq = kv.find('=');
      g_par[kv.substr(0, eq)] = std::atof(kv.c_str() + eq + 1);
    }
  }
  try {
    const GridMap map = readMap(argv[2]);
    std::vector<Scan> S; std::vector<GtPose> G;
    readData(argv[3], argv[4], S, G);
    std::fprintf(stderr, "map %dx%d @ %.3f m, %zu scans (%.0f s)\n", map.width, map.height, map.resolution, S.size(),
                 (S.back().t - S.front().t) * 1e-6);
    const std::string mode = argv[1];
    if (mode == "track") return runTrack(map, S, G, argv[5]);
    if (mode == "reloc") return runReloc(map, S, G, argv[5]);
    if (mode == "init") return runInit(map, S, G, argv[5]);
    if (mode == "dump") return runDump(map, S, G, argv[5]);
    std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
    return 2;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
