// nclt_ndt_tracker.cpp -- baseline for the NCLT experiment: the common scan-to-map NDT tracker.
// Every scan: predict the 6-DoF pose with the wheel-odometry increment since the previous scan,
// register the scan to the prior map with PCL's NormalDistributionsTransform from that
// prediction, and take the result when PCL reports convergence (otherwise keep the prediction).
// No particle filter, no gating, no recovery: the scheme of NDT scan matchers in common ROS
// localization stacks, reduced to its core. Inputs and outputs as nclt_ndt3d (nclt_prepare.py).
//
//   nclt_ndt_tracker MAP_DIR QUERY_DIR OUT_PREFIX [--param key=value]...
//     ndt_resolution 1.0, step_size 0.1, epsilon 1e-4, max_iter 30, leaf 1.0 (scan voxel),
//     crop_m 150 (map points within this of the segment's ground-truth path; 0 = all),
//     start_s 0, duration_s 0 (0 = to the end).
// The seed is the ground-truth pose (6-DoF) at the first scan after start_s.
//
// Build (repository root):
//   g++ -O2 -std=c++17 experiments/nclt/nclt_ndt_tracker.cpp -o /tmp/nclt_ndt_tracker
//       $(pkg-config --cflags --libs pcl_registration-1.12 pcl_filters-1.12 pcl_common-1.12
//         pcl_kdtree-1.12 pcl_search-1.12 eigen3)   (or the PCL version installed)
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "init_offset.hpp"

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct Row { std::int64_t t; double x, y, z, roll, pitch, yaw; };

std::vector<Row> readPoses(const std::string& path) {
  std::vector<Row> out;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    Row r{};
    if (std::sscanf(line.c_str(), "%ld,%lf,%lf,%lf,%lf,%lf,%lf", &r.t, &r.x, &r.y, &r.z, &r.roll, &r.pitch, &r.yaw) == 7)
      out.push_back(r);
  }
  return out;
}

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

bool poseAt(const std::vector<Row>& v, std::int64_t t, Row& out) {
  auto it = std::lower_bound(v.begin(), v.end(), t, [](const Row& r, std::int64_t x) { return r.t < x; });
  if (it == v.begin() || it == v.end()) return false;
  const Row& b = *it;
  const Row& a = *(it - 1);
  if (b.t - a.t > 500000) return false;
  const double u = double(t - a.t) / double(b.t - a.t);
  auto ang = [u](double p, double q) { return wrap(p + u * wrap(q - p)); };
  out = Row{t, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.z + u * (b.z - a.z),
            ang(a.roll, b.roll), ang(a.pitch, b.pitch), ang(a.yaw, b.yaw)};
  return true;
}

Eigen::Isometry3d iso(const Row& r) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(r.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(r.pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(r.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(r.x, r.y, r.z);
  return T;
}
double yawOf(const Eigen::Isometry3d& T) { return std::atan2(T.linear()(1, 0), T.linear()(0, 0)); }
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) { std::fprintf(stderr, "usage: nclt_ndt_tracker MAP_DIR QUERY_DIR OUT_PREFIX [--param k=v]...\n"); return 2; }
  const std::string map_dir = argv[1], query_dir = argv[2], prefix = argv[3];
  std::map<std::string, double> p = {{"ndt_resolution", 1.0}, {"step_size", 0.1}, {"epsilon", 1e-4}, {"max_iter", 30},
                                     {"leaf", 1.0}, {"crop_m", 150.0}, {"start_s", 0.0}, {"duration_s", 0.0},
                                     {"seed", 42}, {"init_d", 0.0}, {"init_a", 0.0}};
  for (int k = 4; k < argc; ++k) {
    if (std::strcmp(argv[k], "--param") != 0 || k + 1 >= argc) { std::fprintf(stderr, "bad argument %s\n", argv[k]); return 2; }
    const std::string kv = argv[++k];
    const auto eq = kv.find('=');
    if (eq == std::string::npos || !p.count(kv.substr(0, eq))) { std::fprintf(stderr, "unknown --param %s\n", kv.c_str()); return 2; }
    p[kv.substr(0, eq)] = std::stod(kv.substr(eq + 1));
  }
  const std::vector<Row> odom = readPoses(query_dir + "/odom.csv");
  const std::vector<Row> gt = readPoses(query_dir + "/gt.csv");

  // First pass over the scans: the segment's stamps, to crop the map around its ground-truth path.
  std::vector<std::int64_t> stamps;
  {
    std::ifstream s(query_dir + "/scans.bin", std::ios::binary);
    std::int64_t t; std::int32_t n;
    while (s.read(reinterpret_cast<char*>(&t), 8) && s.read(reinterpret_cast<char*>(&n), 4)) {
      stamps.push_back(t);
      s.seekg(static_cast<std::streamoff>(n) * 12, std::ios::cur);
    }
  }
  if (stamps.empty()) return 2;
  const std::int64_t t0 = stamps.front();
  const std::int64_t t_begin = t0 + static_cast<std::int64_t>(p["start_s"] * 1e6);
  const std::int64_t t_end = p["duration_s"] > 0 ? t_begin + static_cast<std::int64_t>(p["duration_s"] * 1e6) : stamps.back();
  std::vector<Eigen::Vector2d> path;
  for (std::int64_t t : stamps) {
    Row g{};
    if (t >= t_begin && t <= t_end && poseAt(gt, t, g)) path.emplace_back(g.x, g.y);
  }
  Cloud::Ptr map(new Cloud);
  {
    std::ifstream f(map_dir + "/map.bin", std::ios::binary);
    const double c = p["crop_m"];
    // Coarse occupancy of the path on a c-metre grid, then a 3x3 neighbourhood test per map point.
    std::map<std::pair<long, long>, bool> near;
    if (c > 0) for (const auto& q : path) near[{std::lround(std::floor(q.x() / c)), std::lround(std::floor(q.y() / c))}] = true;
    float v[3];
    while (f.read(reinterpret_cast<char*>(v), 12)) {
      if (c > 0) {
        const long cx = std::lround(std::floor(v[0] / c)), cy = std::lround(std::floor(v[1] / c));
        bool ok = false;
        for (int i = -1; i <= 1 && !ok; ++i) for (int j = -1; j <= 1 && !ok; ++j) ok = near.count({cx + i, cy + j}) > 0;
        if (!ok) continue;
      }
      map->emplace_back(v[0], v[1], v[2]);
    }
  }
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(static_cast<float>(p["ndt_resolution"]));
  ndt.setStepSize(p["step_size"]);
  ndt.setTransformationEpsilon(p["epsilon"]);
  ndt.setMaximumIterations(static_cast<int>(p["max_iter"]));
  ndt.setInputTarget(map);
  std::fprintf(stderr, "map %zu points after the crop\n", map->size());

  std::FILE* out = std::fopen((prefix + ".csv").c_str(), "w");
  std::fprintf(out, "utime,x,y,yaw,gt_x,gt_y,gt_yaw,err_xy,updated,particles,neff,match_fraction,iterations,align_ms\n");
  std::ifstream s(query_dir + "/scans.bin", std::ios::binary);
  bool init = false;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity(), T_odom_prev = Eigen::Isometry3d::Identity();
  long n_scans = 0, n_conv = 0;
  std::vector<float> buf;
  while (true) {
    std::int64_t t; std::int32_t n;
    if (!s.read(reinterpret_cast<char*>(&t), 8) || !s.read(reinterpret_cast<char*>(&n), 4)) break;
    buf.resize(static_cast<size_t>(n) * 3);
    s.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size() * 4));
    if (t < t_begin) continue;
    if (t > t_end) break;
    Row o{}, g{};
    if (!poseAt(odom, t, o)) continue;
    const bool have_gt = poseAt(gt, t, g);
    const Eigen::Isometry3d T_odom = iso(o);
    if (!init) {
      if (!have_gt) continue;
      const InitOffset off = initOffset(static_cast<int>(p["seed"]), p["start_s"], p["init_d"], p["init_a"]);
      Row s = g;
      s.x += off.dx; s.y += off.dy; s.yaw = wrap(g.yaw + off.dyaw);  // INIT_PROTOCOL.md
      T = iso(s);
      T_odom_prev = T_odom;
      init = true;
    }
    T = T * (T_odom_prev.inverse() * T_odom);  // prediction
    T_odom_prev = T_odom;
    Cloud::Ptr raw(new Cloud), src(new Cloud);
    raw->reserve(static_cast<size_t>(n));
    for (std::int32_t k = 0; k < n; ++k) raw->emplace_back(buf[3 * k], buf[3 * k + 1], buf[3 * k + 2]);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw);
    const float leaf = static_cast<float>(p["leaf"]);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*src);
    ndt.setInputSource(src);
    Cloud aligned;
    const auto a0 = std::chrono::steady_clock::now();
    ndt.align(aligned, T.matrix().cast<float>());
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count();
    const bool conv = ndt.hasConverged();
    if (conv) { T.matrix() = ndt.getFinalTransformation().cast<double>(); ++n_conv; }
    ++n_scans;
    if (have_gt)
      std::fprintf(out, "%ld,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%d,1,1,-1,%d,%.1f\n", t, T.translation().x(), T.translation().y(),
                   yawOf(T), g.x, g.y, g.yaw, std::hypot(T.translation().x() - g.x, T.translation().y() - g.y), conv ? 1 : 0,
                   ndt.getFinalNumIteration(), ms);
  }
  std::fclose(out);
  std::FILE* js = std::fopen((prefix + ".json").c_str(), "w");
  std::fprintf(js, "{\"scans\": %ld, \"converged\": %ld, \"map_points\": %zu}\n", n_scans, n_conv, map->size());
  std::fclose(js);
  std::fprintf(stderr, "scans %ld converged %ld\n", n_scans, n_conv);
  return 0;
}
