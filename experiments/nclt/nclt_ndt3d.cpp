// nclt_ndt3d.cpp -- the ndt3d update loop of prism_loc::LocalizationNode on the public NCLT
// dataset (http://robots.engin.umich.edu/nclt/), driven offline. Inputs come from
// nclt_prepare.py: a prior map built from one session (map.bin) and the scans, wheel odometry
// and ground truth of another session (scans.bin, odom.csv, gt.csv), all in a z-up frame.
//
// The loop follows LocalizationNode::runUpdate: skip a scan until the odometry has moved
// update_min_d or turned update_min_a since the last update (or the seed forces it); predict
// with the odometry delta; correct; resample; estimate. The seed is the ground-truth pose at the
// first scan with a ground-truth row, spread like the node's /initialpose floor (0.5 m, 0.5 m,
// 0.2617 rad, 2000 particles). base_height is the ground-truth z of the body at the seed (a
// constant, as the node's parameter), or, with --param ground=1, the height of the body above
// the map's ground there (the node's ground_height_from_map). The cloud frame is the body frame
// (NCLT's velodyne_sync points), not levelled unless --param level=1 (odometry roll and pitch).
//
//   nclt_ndt3d MAP_DIR QUERY_DIR OUT_PREFIX [--param key=value]...
//
// Writes OUT_PREFIX.csv: one row per scan with an estimate (utime, estimate x y yaw, ground truth
// x y yaw, planar error, updated, particles, n_eff, match fraction), and OUT_PREFIX.json with the
// parameters. Single thread; fixed seeds.
//
// Build (repository root):
//   g++ -O2 -std=c++17 -I prism_loc_core/include -I /usr/include/eigen3
//       prism_loc_core/src/*.cpp experiments/nclt/nclt_ndt3d.cpp -o /tmp/nclt_ndt3d
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "prism_loc_core/ground_height_map.hpp"
#include "prism_loc_core/measurement_model.hpp"
#include "prism_loc_core/motion_model.hpp"
#include "prism_loc_core/ndt_map.hpp"
#include "prism_loc_core/particle_filter.hpp"
#include "prism_loc_core/rng.hpp"
#include "prism_loc_core/types.hpp"
#include "init_offset.hpp"
using namespace prism_loc_core;

namespace {

struct Row { std::int64_t t; double x, y, z, roll, pitch, yaw; };

std::vector<Row> readPoses(const std::string& path) {
  std::vector<Row> out;
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read " + path);
  std::string line;
  while (std::getline(f, line)) {
    Row r{};
    if (std::sscanf(line.c_str(), "%ld,%lf,%lf,%lf,%lf,%lf,%lf", &r.t, &r.x, &r.y, &r.z, &r.roll, &r.pitch, &r.yaw) == 7)
      out.push_back(r);
  }
  return out;
}

// Pose at utime t: linear in x, y, z, shortest-arc in the angles; false outside or across a
// gap of more than 0.5 s.
bool poseAt(const std::vector<Row>& v, std::int64_t t, Row& out) {
  auto it = std::lower_bound(v.begin(), v.end(), t, [](const Row& r, std::int64_t x) { return r.t < x; });
  if (it == v.begin() || it == v.end()) return false;
  const Row& b = *it;
  const Row& a = *(it - 1);
  if (b.t - a.t > 500000) return false;
  const double u = double(t - a.t) / double(b.t - a.t);
  auto ang = [u](double p, double q) { return normalizeAngle(p + u * normalizeAngle(q - p)); };
  out = Row{t, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.z + u * (b.z - a.z),
            ang(a.roll, b.roll), ang(a.pitch, b.pitch), ang(a.yaw, b.yaw)};
  return true;
}

struct Params {
  // prism_loc/params/ndt3d.yaml and the node's declare_parameter defaults.
  double ndt_resolution = 1.0;
  int voxel_min_points = 5;
  int max_points = 500;
  int min_particles = 500, max_particles = 2000;
  double resample_threshold = 0.5, kld_err = 0.05, kld_z = 2.33, kld_bin_xy = 0.5, kld_bin_yaw = 0.17;
  double update_min_d = 0.2, update_min_a = 0.2;
  double alpha1 = 0.2, alpha2 = 0.2, alpha3 = 0.2, alpha4 = 0.2;
  double effective_points = 0.0, min_sigma = 0.0;
  int ground = 0;
  double ground_resolution = 1.0, ground_layer_gap = 0.2, ground_max_step = 0.5;
  int ground_min_points = 5;
  int level = 0;
  int seed = 42;
  double start_s = 0.0;      // skip this much of the session before seeding
  double duration_s = 0.0;   // 0 = to the end
  // INIT_PROTOCOL.md: start moved by init_d (m) / init_a (deg), spread max(0.5, d) m, max(15, a) deg.
  double init_d = 0.0, init_a = 0.0;
};

bool setParam(Params& p, const std::string& k, double v) {
  std::map<std::string, double*> d = {
      {"ndt_resolution", &p.ndt_resolution}, {"resample_threshold", &p.resample_threshold},
      {"kld_err", &p.kld_err}, {"kld_z", &p.kld_z}, {"kld_bin_xy", &p.kld_bin_xy}, {"kld_bin_yaw", &p.kld_bin_yaw},
      {"update_min_d", &p.update_min_d}, {"update_min_a", &p.update_min_a}, {"alpha1", &p.alpha1},
      {"alpha2", &p.alpha2}, {"alpha3", &p.alpha3}, {"alpha4", &p.alpha4},
      {"effective_points", &p.effective_points}, {"min_sigma", &p.min_sigma},
      {"ground_resolution", &p.ground_resolution}, {"ground_layer_gap", &p.ground_layer_gap},
      {"ground_max_step", &p.ground_max_step}, {"start_s", &p.start_s}, {"duration_s", &p.duration_s},
      {"init_d", &p.init_d}, {"init_a", &p.init_a}};
  std::map<std::string, int*> i = {
      {"voxel_min_points", &p.voxel_min_points}, {"max_points", &p.max_points},
      {"min_particles", &p.min_particles}, {"max_particles", &p.max_particles}, {"ground", &p.ground},
      {"ground_min_points", &p.ground_min_points}, {"level", &p.level}, {"seed", &p.seed}};
  if (d.count(k)) { *d[k] = v; return true; }
  if (i.count(k)) { *i[k] = static_cast<int>(v); return true; }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: nclt_ndt3d MAP_DIR QUERY_DIR OUT_PREFIX [--param key=value]...\n");
    return 2;
  }
  const std::string map_dir = argv[1], query_dir = argv[2], prefix = argv[3];
  Params prm;
  for (int k = 4; k < argc; ++k) {
    if (std::strcmp(argv[k], "--param") != 0 || k + 1 >= argc) { std::fprintf(stderr, "bad argument %s\n", argv[k]); return 2; }
    const std::string kv = argv[++k];
    const auto eq = kv.find('=');
    if (eq == std::string::npos || !setParam(prm, kv.substr(0, eq), std::stod(kv.substr(eq + 1)))) {
      std::fprintf(stderr, "unknown --param %s\n", kv.c_str());
      return 2;
    }
  }

  std::vector<Eigen::Vector3d> map_pts;
  {
    std::ifstream f(map_dir + "/map.bin", std::ios::binary);
    if (!f) { std::fprintf(stderr, "cannot read %s/map.bin\n", map_dir.c_str()); return 2; }
    float p[3];
    while (f.read(reinterpret_cast<char*>(p), sizeof p)) map_pts.emplace_back(p[0], p[1], p[2]);
  }
  auto ndt_map = std::make_shared<NdtMap>(map_pts, prm.ndt_resolution, prm.voxel_min_points, prm.min_sigma);
  std::shared_ptr<GroundHeightMap> ground;
  if (prm.ground) {
    GroundHeightParams gp;
    gp.resolution = prm.ground_resolution;
    gp.layer_gap = prm.ground_layer_gap;
    gp.max_step = prm.ground_max_step;
    gp.min_points = prm.ground_min_points;
    ground = std::make_shared<GroundHeightMap>(map_pts, gp);
  }
  std::fprintf(stderr, "map %zu points, %zu NDT cells%s\n", map_pts.size(), ndt_map->numVoxels(),
               ground ? (", ground cells " + std::to_string(ground->numCells())).c_str() : "");
  map_pts.clear();
  map_pts.shrink_to_fit();

  const std::vector<Row> odom = readPoses(query_dir + "/odom.csv");
  const std::vector<Row> gt = readPoses(query_dir + "/gt.csv");
  std::ifstream scans(query_dir + "/scans.bin", std::ios::binary);
  if (!scans || odom.empty() || gt.empty()) { std::fprintf(stderr, "missing query inputs\n"); return 2; }

  ParticleFilterParams pp;
  pp.min_particles = prm.min_particles; pp.max_particles = prm.max_particles;
  pp.resample_threshold = prm.resample_threshold; pp.kld_err = prm.kld_err; pp.kld_z = prm.kld_z;
  pp.kld_bin_xy = prm.kld_bin_xy; pp.kld_bin_yaw = prm.kld_bin_yaw;
  Rng rng(static_cast<std::uint64_t>(prm.seed));
  ParticleFilter pf(pp, rng);
  OdometryMotionModel motion(MotionParams{prm.alpha1, prm.alpha2, prm.alpha3, prm.alpha4});
  std::unique_ptr<Ndt3DModel> model;

  std::FILE* out = std::fopen((prefix + ".csv").c_str(), "w");
  if (!out) { std::fprintf(stderr, "cannot write %s.csv\n", prefix.c_str()); return 2; }
  std::fprintf(out, "utime,x,y,yaw,gt_x,gt_y,gt_yaw,err_xy,updated,particles,neff,match_fraction\n");

  bool init = false, force = false;
  Pose2D last_odom{}, map_odom{};
  std::int64_t t_first = 0, t_seed = 0;
  long n_scans = 0, n_updates = 0;
  std::vector<Eigen::Vector3d> cloud;
  std::vector<float> buf;
  while (true) {
    std::int64_t t = 0;
    std::int32_t n = 0;
    if (!scans.read(reinterpret_cast<char*>(&t), sizeof t) || !scans.read(reinterpret_cast<char*>(&n), sizeof n)) break;
    buf.resize(static_cast<size_t>(n) * 3);
    if (!scans.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size() * sizeof(float)))) break;
    if (t_first == 0) t_first = t;
    if (double(t - t_first) * 1e-6 < prm.start_s) continue;
    if (init && prm.duration_s > 0 && double(t - t_seed) * 1e-6 > prm.duration_s) break;
    Row o{}, g{};
    if (!poseAt(odom, t, o)) continue;
    const bool have_gt = poseAt(gt, t, g);
    const Pose2D odom_base{o.x, o.y, o.yaw};
    if (!init) {
      if (!have_gt) continue;
      NdtParams np;
      np.max_points = prm.max_points;
      np.base_height = g.z;
      np.effective_points = prm.effective_points;
      const InitOffset off = initOffset(prm.seed, prm.start_s, prm.init_d, prm.init_a);
      const Pose2D seed{g.x + off.dx, g.y + off.dy, normalizeAngle(g.yaw + off.dyaw)};
      const bool offset_seed = prm.init_d > 0.0 || prm.init_a > 0.0;
      if (ground) {
        // base_height is a property of the robot (body above the ground under it, at the true pose);
        // the ground hint starts on the level of that ground at the (possibly offset) seed.
        const auto g0 = ground->heightNearest(g.x, g.y, g.z - 0.5);
        if (!g0) { std::fprintf(stderr, "no map ground under the start\n"); return 2; }
        np.base_height = g.z - *g0;
        model = std::make_unique<Ndt3DModel>(ndt_map, np);
        model->setGroundMap(ground);
        model->seedGround(seed, *g0);
      } else {
        model = std::make_unique<Ndt3DModel>(ndt_map, np);
      }
      const double sxy = std::max(0.5, prm.init_d), syaw = offset_seed ? std::max(15.0, prm.init_a) * M_PI / 180.0 : 0.2617;
      pf.initializeGaussian(seed, Pose2D{sxy, sxy, syaw}, 2000);
      init = force = true;
      last_odom = odom_base;
      t_seed = t;
    }
    cloud.clear();
    cloud.reserve(static_cast<size_t>(n));
    const Eigen::Matrix3d lev = prm.level
        ? Eigen::Matrix3d(Eigen::AngleAxisd(o.pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(o.roll, Eigen::Vector3d::UnitX()))
        : Eigen::Matrix3d::Identity();
    for (std::int32_t k = 0; k < n; ++k) cloud.emplace_back(lev * Eigen::Vector3d(buf[3 * k], buf[3 * k + 1], buf[3 * k + 2]));
    model->setCloud(cloud, Pose2D{0.0, 0.0, 0.0});
    ++n_scans;
    const double dd = std::hypot(odom_base.x - last_odom.x, odom_base.y - last_odom.y);
    const double da = std::fabs(normalizeAngle(odom_base.yaw - last_odom.yaw));
    bool updated = false;
    double neff = 0.0, mf = -1.0;
    size_t np_before = 0;
    if (force || dd >= prm.update_min_d || da >= prm.update_min_a) {
      pf.predict(motion, last_odom, odom_base);
      pf.correct(*model);
      neff = pf.effectiveSampleSize();
      np_before = pf.particles().size();
      pf.resample();
      last_odom = odom_base;
      force = false;
      const Pose2D est = pf.estimate(nullptr);
      mf = model->matchStats(est).fraction();
      if (ground) model->followGround(est);
      map_odom = compose(est, inverse(odom_base));
      updated = true;
      ++n_updates;
    }
    const Pose2D e = compose(map_odom, odom_base);
    if (have_gt) {
      std::fprintf(out, "%ld,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%d,%zu,%.2f,%.3f\n", t, e.x, e.y, e.yaw, g.x, g.y, g.yaw,
                   std::hypot(e.x - g.x, e.y - g.y), updated ? 1 : 0, np_before, neff, mf);
    }
  }
  std::fclose(out);
  std::FILE* js = std::fopen((prefix + ".json").c_str(), "w");
  std::fprintf(js, "{\"scans\": %ld, \"updates\": %ld, \"seed\": %d, \"effective_points\": %g, \"min_sigma\": %g, "
               "\"ground\": %d, \"level\": %d, \"alpha\": [%g, %g, %g, %g], \"start_s\": %g, \"duration_s\": %g}\n",
               n_scans, n_updates, prm.seed, prm.effective_points, prm.min_sigma, prm.ground, prm.level, prm.alpha1,
               prm.alpha2, prm.alpha3, prm.alpha4, prm.start_s, prm.duration_s);
  std::fclose(js);
  std::fprintf(stderr, "scans %ld updates %ld\n", n_scans, n_updates);
  return 0;
}
