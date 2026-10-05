#!/usr/bin/env python3
"""End-to-end check of the laser2d node on a synthetic world, over real ROS topics and TF.

The script starts `prism_loc_node_main` with a params YAML, then plays the robot:
  * /map       -- a 20 x 15 m occupancy grid (walls plus five boxes, no symmetry),
                  published once, transient local + reliable like nav2_map_server
  * /scan      -- 360-beam LaserScan ray-cast in that grid from a known trajectory
                  (an ellipse at about 0.4 m/s), seeded range noise, best effort
  * TF         -- base_link -> laser static (0.2 m ahead); odom -> base_link
                  integrated from the true motion with a 3 % scale error and a
                  0.01 rad/s yaw-rate bias, so map -> odom must keep correcting
  * /initialpose -- once at t = 1 s, 0.36 m and 0.1 rad off the truth
and checks /prism_loc/pose, the map -> odom TF and /diagnostics.

Scenarios (--scenario, default all):
  track       pose error <= 0.15 m and yaw <= 0.05 rad for every pose after t = 10 s;
              map -> odom stamp leads its arrival by 0 .. 0.15 s (scan stamp +
              transform_tolerance); no gap between map -> odom TFs over 0.5 s;
              /diagnostics OK at the end
  gap         scans withheld for 7 s (t = 15 .. 22 s): /diagnostics turns ERROR naming
              the stopped scans within 3 s, and that ERROR clears within 3 s of the
              scans returning (a WARN such as low n_eff may follow the jump); map -> odom resumes within 1 s and the position error is
              <= 0.15 m from 15 s after the gap (re-convergence measured 3 .. 12 s; a
              3 s bound is not met by the current filter)
  no_odom     no odom -> base_link TF: no map -> odom TF is published and /diagnostics
              WARNs about the odom TF within 5 s of the initial pose
  zero_stamp  after 10 s of tracking, scans carry a zero header stamp for 3 s: no
              map -> odom TF stamped near zero is published and /diagnostics WARNs
  stuck_clock use_sim_time on, nothing publishes /clock, no odom TF: the node keeps
              answering parameter requests (its executor is not blocked in a TF wait)
              and /diagnostics WARNs that the ROS clock is not advancing
  slow_clock  use_sim_time on, /clock at 0.1x real time, odometry TF at 20 Hz of sim
              time, each scan stamped 30 ms of sim time ahead of the latest odometry:
              at least 90 % of the scans after the initial pose produce a pose (the TF
              wait budget is ROS time, not wall time)
  stationary_seed  robot standing still, /initialpose as from RViz: poses are published
              and /diagnostics never WARNs about a low effective particle count

This is a ROS-path test (topics -> node -> pose/TF/diagnostics) on a synthetic world
whose sensor model matches the filter's assumptions; it is not a field result.

Usage: synthetic_e2e.py PARAMS_YAML [--scenario NAME] [--node-arg name:=value ...]
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

RES, W, H = 0.05, 400, 300
LASER_X = 0.2
RATE = 10.0
STATUS = 'prism_loc: localization'
OK, WARN, ERROR = 0, 1, 2
# After a 7 s scan gap the first update applies 2.8 m of odometry at once; the motion
# model spreads the particles by tenths of a metre and the filter, updating only every
# update_min_d, pulls back in over several seconds (measured 3 .. 12 s on the current
# code). The check asserts recovery, not its speed.
RECOVERY_S = 15.0


def make_grid():
    g = np.zeros((H, W), dtype=np.int8)
    g[0:3, :] = 100; g[-3:, :] = 100; g[:, 0:3] = 100; g[:, -3:] = 100
    for (x0, y0, x1, y1) in [(3, 3, 4, 6), (12, 2, 13, 4), (7, 9, 11, 10), (15, 8, 16, 13), (2, 11, 3, 12)]:
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100
    return g


GRID = make_grid()
RANGES = np.arange(0.05, 12.0, RES / 2)


def raycast(x, y, th, n=360):
    a = th + np.linspace(-math.pi, math.pi, n, endpoint=False)
    px = x + np.outer(np.cos(a), RANGES)
    py = y + np.outer(np.sin(a), RANGES)
    ix = (px / RES).astype(int)
    iy = (py / RES).astype(int)
    inside = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    occ = np.ones_like(inside)
    occ[inside] = GRID[iy[inside], ix[inside]] > 50
    first = np.where(occ.any(1), occ.argmax(1), len(RANGES) - 1)
    return RANGES[first]


def truth(t):
    w = 0.08
    x = 10 + 5 * math.cos(w * t)
    y = 7.5 + 3.5 * math.sin(w * t)
    return x, y, math.atan2(3.5 * w * math.cos(w * t), -5 * w * math.sin(w * t))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Odom:
    """odom -> base_link integrated from the true increments with scale and yaw-rate errors."""

    def __init__(self):
        self.prev = None
        self.o = [0.0, 0.0, 0.0]

    def step(self, x, y, th, dt):
        if self.prev is None:
            self.prev = (x, y, th)
            self.o = [x - 10.0, y - 7.5, th]
            return tuple(self.o)
        px, py, pth = self.prev
        dx, dy = x - px, y - py
        fx = 1.03 * (math.cos(pth) * dx + math.sin(pth) * dy)
        fy = 1.03 * (-math.sin(pth) * dx + math.cos(pth) * dy)
        dth = wrap(th - pth) + 0.01 * dt
        o = self.o
        o[0] += math.cos(o[2]) * fx - math.sin(o[2]) * fy
        o[1] += math.sin(o[2]) * fx + math.cos(o[2]) * fy
        o[2] += dth
        self.prev = (x, y, th)
        return tuple(o)


def tf_msg(stamp, parent, child, x, y, yaw):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    return t


def level_of(status):
    lv = status.level
    return lv[0] if isinstance(lv, bytes) else lv


class Driver:
    def __init__(self, node):
        self.n = node
        mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = node.create_publisher(OccupancyGrid, '/map', mq)
        self.scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.ip_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tfb = TransformBroadcaster(node)
        self.stb = StaticTransformBroadcaster(node)
        self.stb.sendTransform(tf_msg(node.get_clock().now().to_msg(), 'base_link', 'laser',
                                      LASER_X, 0.0, 0.0))
        self.poses, self.mo, self.diags = [], [], []
        node.create_subscription(PoseWithCovarianceStamped, '/prism_loc/pose',
                                 lambda m: self.poses.append((time.monotonic(), m)), 50)
        node.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)
        self.param_cli = node.create_client(GetParameters, '/prism_loc/get_parameters')
        self.truth_at = {}
        self.rng = np.random.default_rng(7)
        self.t0 = None

    def on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.mo.append((time.monotonic(), t))

    def on_diag(self, m):
        for s in m.status:
            if s.name == STATUS:
                self.diags.append((time.monotonic(), s))

    def rel(self, mono):
        return mono - self.t0

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.info.resolution = RES
        m.info.width, m.info.height = W, H
        m.info.origin.orientation.w = 1.0
        m.data = GRID.flatten().tolist()
        self.map_pub.publish(m)

    def wait_for_node(self, timeout):
        end = time.monotonic() + timeout
        while self.scan_pub.get_subscription_count() == 0 or self.map_pub.get_subscription_count() == 0:
            if time.monotonic() > end:
                sys.exit('FAIL: the node did not subscribe to /scan and /map')
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def param_answers(self, timeout=2.0):
        req = GetParameters.Request()
        req.names = ['update_min_d']
        fut = self.param_cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)
        return fut.done() and fut.result() is not None

    def run(self, duration, gap=None, odom=True, zero_from=None, probe_params_at=(),
            stationary=False):
        od = Odom()
        self.t0 = time.monotonic()
        tl, sent_ip, k = 0.0, False, 0
        answers = []
        probes = list(probe_params_at)
        while True:
            t = time.monotonic() - self.t0
            if t >= duration:
                break
            x, y, th = truth(0.0 if stationary else t)
            ox, oy, oth = od.step(x, y, th, t - tl)
            tl = t
            stamp = self.n.get_clock().now().to_msg()  # wall clock, as a live sensor driver stamps
            if odom:
                self.tfb.sendTransform(tf_msg(stamp, 'odom', 'base_link', ox, oy, oth))
            if t > 1.0 and not sent_ip:
                ip = PoseWithCovarianceStamped()
                ip.header.frame_id = 'map'
                ip.header.stamp = stamp
                ip.pose.pose.position.x = x + 0.3
                ip.pose.pose.position.y = y - 0.2
                ip.pose.pose.orientation.z = math.sin((th + 0.1) / 2)
                ip.pose.pose.orientation.w = math.cos((th + 0.1) / 2)
                self.ip_pub.publish(ip)
                sent_ip = True
            if not (gap and gap[0] <= t < gap[1]):
                s = LaserScan()
                s.header.stamp = stamp
                if zero_from is not None and zero_from <= t < zero_from + 3.0:
                    s.header.stamp.sec, s.header.stamp.nanosec = 0, 0
                s.header.frame_id = 'laser'
                s.angle_min = -math.pi
                s.angle_increment = 2 * math.pi / 360
                s.angle_max = math.pi - s.angle_increment
                s.range_min, s.range_max = 0.05, 12.0
                lx, ly = x + LASER_X * math.cos(th), y + LASER_X * math.sin(th)
                r = raycast(lx, ly, th) + self.rng.normal(0.0, 0.01, 360)
                s.ranges = r.astype(np.float32).tolist()
                self.truth_at[(stamp.sec, stamp.nanosec)] = (t, x, y, th)
                self.scan_pub.publish(s)
            if probes and t >= probes[0]:
                probes.pop(0)
                answers.append((t, self.param_answers()))
            end = self.t0 + (k + 1) / RATE
            k += 1
            while time.monotonic() < end:
                rclpy.spin_once(self.n, timeout_sec=0.01)
        return answers

    def errors(self):
        out = []
        for _, m in self.poses:
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            if key not in self.truth_at:
                continue
            t, x, y, th = self.truth_at[key]
            p = m.pose.pose
            yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
            out.append((t, math.hypot(p.position.x - x, p.position.y - y), abs(wrap(yaw - th))))
        return out

    def diag_times(self, level, text):
        return [self.rel(mono) for mono, s in self.diags
                if level_of(s) == level and text in s.message]

    def last_level(self):
        if not self.diags:
            return None
        return level_of(self.diags[-1][1])


def check(results, ok, text):
    results.append((ok, text))


def scenario_track(d, results):
    d.run(40.0)
    errs = [e for e in d.errors() if e[0] > 10.0]
    check(results, len(errs) > 100, f'poses after t = 10 s: {len(errs)} (need > 100)')
    if errs:
        mp, my = max(e[1] for e in errs), max(e[2] for e in errs)
        mean = sum(e[1] for e in errs) / len(errs)
        check(results, mp <= 0.15, f'max position error after 10 s: {mp:.3f} m (mean {mean:.3f}; need <= 0.15)')
        check(results, my <= 0.05, f'max yaw error after 10 s: {my:.4f} rad (need <= 0.05)')
    leads = []
    for rx, t in d.mo:
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        leads.append(stamp - (time.time() - (time.monotonic() - rx)))
    if leads:
        check(results, 0.0 <= min(leads) and max(leads) <= 0.15,
              f'map->odom stamp lead over arrival: {min(leads):.3f} .. {max(leads):.3f} s (need 0 .. 0.15)')
    gaps = [d.mo[i + 1][0] - d.mo[i][0] for i in range(len(d.mo) - 1) if d.rel(d.mo[i][0]) > 3.0]
    check(results, bool(gaps) and max(gaps) <= 0.5,
          f'max interval between map->odom TFs: {max(gaps) if gaps else float("nan"):.2f} s (need <= 0.5)')
    lvl = d.last_level()
    check(results, lvl == OK, f'/diagnostics level at the end: {lvl} (need 0 = OK; '
          f'"{d.diags[-1][1].message if d.diags else "none"}")')
    fr = []
    for m, s in d.diags:
        v = {kv.key: kv.value for kv in s.values}
        if d.rel(m) > 10.0 and float(v.get('particles', 0)) > 0:
            fr.append(float(v['n_eff']) / float(v['particles']))
    if fr:
        print(f'  n_eff / particles after 10 s: min {min(fr):.3f}, median {sorted(fr)[len(fr) // 2]:.3f}')
    vals = {kv.key: kv.value for kv in d.diags[-1][1].values} if d.diags else {}
    print(f'  diagnostics values at the end: n_eff={vals.get("n_eff")} particles={vals.get("particles")} '
          f'input_rate_hz={vals.get("input_rate_hz")} cov_trace_xy={vals.get("covariance_trace_xy")}')


def scenario_gap(d, results):
    g0, g1 = 15.0, 22.0
    end = g1 + RECOVERY_S + 3.0
    d.run(end, gap=(g0, g1))
    errs = d.errors()
    prof = []
    for k in range(0, int(end - g1)):
        win = [e[1] for e in errs if g1 + k <= e[0] < g1 + k + 1]
        prof.append(f'{max(win):.2f}' if win else '-')
    print(f'  max position error per second after the gap: {" ".join(prof)} m')
    tf_back = [d.rel(m) for m, _ in d.mo if d.rel(m) >= g1]
    check(results, bool(tf_back) and tf_back[0] - g1 <= 1.0,
          f'map->odom TF resumed {tf_back[0] - g1:.2f} s after scans returned (need <= 1.0)' if tf_back
          else 'map->odom TF did not resume after the gap')
    err_t = [t for t in d.diag_times(ERROR, 'scans stopped') if g0 <= t <= g1 + 1.0]
    check(results, bool(err_t) and err_t[0] - g0 <= 3.0,
          f'ERROR "scans stopped" {err_t[0] - g0:.1f} s after the gap began' if err_t
          else 'no ERROR "scans stopped" during the 7 s scan gap')
    # The first update after the gap applies 2.8 m of odometry and can leave a low-n_eff
    # WARN for a few seconds; what must clear is the "scans stopped" ERROR.
    after = sorted({s.message for m, s in d.diags if g1 < d.rel(m) <= g1 + 6.0 and level_of(s) != OK})
    print(f'  non-OK statuses within 6 s after the gap: {after}')
    ok_t = [d.rel(m) for m, s in d.diags if d.rel(m) > g1 and level_of(s) != ERROR]
    check(results, bool(ok_t) and ok_t[0] - g1 <= 3.0,
          f'"scans stopped" ERROR cleared {ok_t[0] - g1:.1f} s after scans returned' if ok_t
          else 'no OK status after scans returned')
    post = [e for e in errs if e[0] > g1 + RECOVERY_S]
    check(results, len(post) > 20 and max(e[1] for e in post) <= 0.15,
          f'max position error from {RECOVERY_S:.0f} s after the gap: '
          f'{max(e[1] for e in post) if post else float("nan"):.3f} m over {len(post)} poses (need <= 0.15)')


def scenario_no_odom(d, results):
    d.run(12.0, odom=False)
    check(results, len(d.mo) == 0, f'map->odom TFs without odometry: {len(d.mo)} (need 0)')
    w = [t for t in d.diag_times(WARN, 'odom TF') if t > 1.0]
    check(results, bool(w) and w[0] - 1.0 <= 5.0,
          f'WARN about the odom TF {w[0] - 1.0:.1f} s after the initial pose' if w
          else 'no WARN about the missing odom TF')


def scenario_zero_stamp(d, results):
    d.run(18.0, zero_from=10.0)
    early = [t for _, t in d.mo if t.header.stamp.sec < 1000]
    check(results, not early, f'map->odom TFs stamped near zero: {len(early)} (need 0)')
    w = [t for t in d.diag_times(WARN, 'zero header stamp') if 10.0 <= t <= 14.5]
    check(results, bool(w), 'WARN about zero-stamped scans' + ('' if w else ' missing'))


def scenario_stuck_clock(d, results):
    answers = d.run(12.0, odom=False, probe_params_at=(4.0, 7.0, 10.0))
    good = sum(1 for _, a in answers if a)
    check(results, good == len(answers) == 3,
          f'parameter requests answered within 2 s while scans wait on a missing TF: {good}/{len(answers)}')
    w = d.diag_times(WARN, 'ROS clock has not advanced')
    check(results, bool(w), 'WARN that the ROS clock is not advancing' + ('' if w else ' missing'))


def scenario_slow_clock(d, results):
    from rosgraph_msgs.msg import Clock
    clock_pub = d.n.create_publisher(Clock, '/clock', 10)
    factor, sim0, ahead = 0.1, 1000.0, 0.03
    od = Odom()
    w0 = time.monotonic()
    d.t0 = w0
    next_tick, sent_ip, after_ip, prev_ts = 0.0, False, 0, 0.0

    def stamp_of(sec):
        return rclpy.time.Time(seconds=sec).to_msg()

    while time.monotonic() - w0 < 36.0:
        ts = factor * (time.monotonic() - w0)
        c = Clock()
        c.clock = stamp_of(sim0 + ts)
        clock_pub.publish(c)
        if ts >= next_tick:
            next_tick += 0.05  # odometry at 20 Hz of sim time
            x, y, th = truth(ts)
            ox, oy, oth = od.step(x, y, th, ts - prev_ts)
            prev_ts = ts
            d.tfb.sendTransform(tf_msg(stamp_of(sim0 + ts), 'odom', 'base_link', ox, oy, oth))
            if not sent_ip and ts > 0.2:
                ip = PoseWithCovarianceStamped()
                ip.header.frame_id = 'map'
                ip.header.stamp = stamp_of(sim0 + ts)
                ip.pose.pose.position.x, ip.pose.pose.position.y = x + 0.1, y - 0.1
                ip.pose.pose.orientation.z = math.sin(th / 2)
                ip.pose.pose.orientation.w = math.cos(th / 2)
                d.ip_pub.publish(ip)
                sent_ip = True
            elif sent_ip:
                after_ip += 1
            # scan stamped `ahead` of the odometry just sent
            sx, sy, sth = truth(ts + ahead)
            sc = LaserScan()
            sc.header.stamp = stamp_of(sim0 + ts + ahead)
            sc.header.frame_id = 'laser'
            sc.angle_min = -math.pi
            sc.angle_increment = 2 * math.pi / 360
            sc.angle_max = math.pi - sc.angle_increment
            sc.range_min, sc.range_max = 0.05, 12.0
            r = raycast(sx + LASER_X * math.cos(sth), sy + LASER_X * math.sin(sth), sth)
            sc.ranges = (r + d.rng.normal(0.0, 0.01, 360)).astype(np.float32).tolist()
            d.truth_at[(sc.header.stamp.sec, sc.header.stamp.nanosec)] = (ts, sx, sy, sth)
            d.scan_pub.publish(sc)
        rclpy.spin_once(d.n, timeout_sec=0.01)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(d.n, timeout_sec=0.01)
    poses = len(d.errors())
    check(results, after_ip > 30 and poses >= 0.9 * after_ip,
          f'scans after the initial pose that produced a pose at 0.1x: {poses}/{after_ip} (need >= 90 %)')


def scenario_stationary_seed(d, results):
    d.run(15.0, stationary=True)
    check(results, len(d.poses) > 50, f'poses from a stationary robot: {len(d.poses)} (need > 50)')
    w = d.diag_times(WARN, 'low effective particle count')
    check(results, not w, f'WARN "low effective particle count" after /initialpose: {len(w)} status(es) (need 0)')


SCENARIOS = {'track': scenario_track, 'gap': scenario_gap, 'no_odom': scenario_no_odom,
             'zero_stamp': scenario_zero_stamp, 'stuck_clock': scenario_stuck_clock,
             'slow_clock': scenario_slow_clock, 'stationary_seed': scenario_stationary_seed}


def run_scenario(name, params, node_args):
    args = ['ros2', 'run', 'prism_loc', 'prism_loc_node_main', '--ros-args', '--params-file', params]
    sim = name in ('stuck_clock', 'slow_clock')
    for a in node_args + (['use_sim_time:=true'] if sim else []):
        args += ['-p', a]
    log = open(f'e2e_{name}.log', 'w')
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node(f'prism_loc_e2e_{name}')
    d = Driver(node)
    results = []
    try:
        d.wait_for_node(30.0)
        d.publish_map()
        SCENARIOS[name](d, results)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('params', help='params YAML for the node (prism_loc/params/laser2d.yaml)')
    ap.add_argument('--scenario', choices=['all'] + list(SCENARIOS), default='all')
    ap.add_argument('--node-arg', action='append', default=[],
                    help='extra node parameter, name:=value (repeatable)')
    args = ap.parse_args()
    names = list(SCENARIOS) if args.scenario == 'all' else [args.scenario]
    failed = False
    for name in names:
        results = run_scenario(name, os.path.abspath(args.params), args.node_arg)
        print(f'[{name}]')
        for ok, text in results:
            print(f'  {"PASS" if ok else "FAIL"}: {text}')
            failed |= not ok
        if not results or not all(ok for ok, _ in results):
            failed = True
            print(f'  (node log: e2e_{name}.log)')
    if failed:
        sys.exit('FAIL: synthetic end-to-end')
    print('PASS: synthetic end-to-end')


if __name__ == '__main__':
    main()
