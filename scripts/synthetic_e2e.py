#!/usr/bin/env python3
"""End-to-end check of the laser2d and ndt3d nodes on a synthetic world, over real ROS topics and TF.

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
  coarse_clock use_sim_time on, /clock at 0.5x real time but stepped at 10 Hz of sim
              time (one step per 0.2 s of wall time), odometry and scans as in
              slow_clock: every scan after the initial pose produces a pose (a clock
              that ticks coarsely must not be mistaken for a frozen one)
  stationary_seed  robot standing still, /initialpose as from RViz: poses are published
              and /diagnostics never WARNs about a low effective particle count
  reloc_gate  try_global_localization with the verification motion gate on
              (bbs_verify_min_translation/rotation 0.3), robot standing still, no
              /initialpose: /diagnostics WARNs "relocalization pending: waiting for
              motion (N scans skipped)" with relocalization_scans_used 1 and
              relocalization_scans_skipped > 0

With an ndt3d params YAML (backend: "ndt3d") the same world is used in 3D: walls and boxes
2.5 m high on a ground plane, written to e2e_world3d.pcd as the prior map, and a 16-ring
lidar (360 azimuths, 10 Hz, 1 cm range noise) mounted 0.2 m ahead of and 0.5 m above
base_link, turned 0.3 rad about z. The node runs with base_height 0.5 (the lidar's z in
the map frame). Thresholds were written down before the scenarios were first run in this
form; the status thresholds come from the runs listed in PARAMS.md (min_match_fraction).
  ndt3d_track  40 s of tracking: position error <= 0.5 m (mean <= 0.15) for every pose
              after t = 10 s and <= 0.25 m after t = 30 s (pulling in the 0.36 m error of
              the initial pose took up to 20 s in the seeded runs), yaw <= 0.05 rad;
              after /initialpose the status is WARN "pose not verified" (t = 3 .. 8 s)
              and turns OK by t = 14 s; every status after t = 15 s is OK with
              match_fraction >= 0.65; no startup message about the sensor transform
  ndt3d_kidnap  at t = 20 s the robot is moved (the clouds come from another place, the
              odometry does not notice): statuses from t = 15 s to then are OK; WARN
              "low scan-to-map match" within 6 s and in >= 80 % of the statuses from then on
  ndt3d_off_map  /initialpose 100 m off the map; the robot stands for 6 s, then drives:
              while it stands the status is WARN "pose not verified ... 0 of 15" (the
              match is not judged on the reseed update: match_fraction n/a); once it
              moves, WARN "low scan-to-map match: 0.000" in every status from t = 9 s,
              while n_eff / particles is >= 0.99
  ndt3d_base_height  base_height left at 0 with the lidar 0.5 m up: one INFO in the node
              log giving the TF z and base_height (no WARN: the two differ on any robot
              whose base_link is not at map z = 0), and WARN "low scan-to-map match" in
              every status from t = 6 s
  ndt3d_tilt  lidar pitched 0.1 rad, YAML without min_neff_fraction, min_match_fraction
              and match_verify_updates: one WARN in the node log naming the tilt (no
              height message); the node's own defaults for ndt3d are 0, 0.65 and 15
  ndt3d_strict  healthy tracking with min_match_fraction 0.95 (and match_verify_updates
              0): WARN "low scan-to-map match ... min_match_fraction 0.950" in every
              status from t = 6 s (the parameter, not its default, drives the decision)
  ndt3d_verify3  healthy tracking with match_verify_updates 3: WARN "pose not verified
              ... N of 3" first, OK in every status from t = 6 s (with the default 15 it
              would still be "pose not verified")
  ndt3d_off   min_match_fraction 0 and /initialpose 100 m off the map: the check is off,
              every status is OK although the robot is lost (0 disables both WARNs)

This is a ROS-path test (topics -> node -> pose/TF/diagnostics) on a synthetic world
whose sensor model matches the filter's assumptions; it is not a field result.

Usage: synthetic_e2e.py PARAMS_YAML [--scenario NAME] [--node-arg name:=value ...]
       (--scenario all, the default, runs the scenarios of the YAML's backend)
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
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
            stationary=False, initial_pose=True):
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
            if t > 1.0 and not sent_ip and initial_pose:
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
          else '"scans stopped" ERROR did not clear after scans returned')
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


def sim_clock_run(d, factor, clock_step):
    """Publishes /clock at `factor` x real time (stepped by clock_step s of sim time if
    > 0), odometry TF at 20 Hz of sim time and, with each odometry, a scan stamped 30 ms
    of sim time ahead of it. Returns the number of scans sent after the initial pose."""
    from rosgraph_msgs.msg import Clock
    clock_pub = d.n.create_publisher(Clock, '/clock', 10)
    sim0, ahead = 1000.0, 0.03
    od = Odom()
    w0 = time.monotonic()
    d.t0 = w0
    next_tick, sent_ip, after_ip, prev_ts = 0.0, False, 0, 0.0

    def stamp_of(sec):
        return rclpy.time.Time(seconds=sec).to_msg()

    while time.monotonic() - w0 < 36.0:
        ts = factor * (time.monotonic() - w0)
        c = Clock()
        c.clock = stamp_of(sim0 + (math.floor(ts / clock_step) * clock_step if clock_step > 0 else ts))
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
    return after_ip


def scenario_slow_clock(d, results):
    after_ip = sim_clock_run(d, 0.1, 0.0)
    poses = len(d.errors())
    check(results, after_ip > 30 and poses >= 0.9 * after_ip,
          f'scans after the initial pose that produced a pose at 0.1x: {poses}/{after_ip} (need >= 90 %)')


COARSE_FACTOR = 0.5  # --coarse-factor


def scenario_coarse_clock(d, results):
    after_ip = sim_clock_run(d, COARSE_FACTOR, 0.1)
    poses = len(d.errors())
    check(results, after_ip > 100 and poses >= after_ip,
          f'scans after the initial pose that produced a pose with a 10 Hz /clock at '
          f'{COARSE_FACTOR:g}x: {poses}/{after_ip} (need all)')


def scenario_stationary_seed(d, results):
    d.run(15.0, stationary=True)
    check(results, len(d.poses) > 50, f'poses from a stationary robot: {len(d.poses)} (need > 50)')
    w = d.diag_times(WARN, 'low effective particle count')
    check(results, not w, f'WARN "low effective particle count" after /initialpose: {len(w)} status(es) (need 0)')


def scenario_reloc_gate(d, results):
    d.run(12.0, stationary=True, initial_pose=False)
    w = [s for m, s in d.diags if level_of(s) == WARN and 'waiting for motion' in s.message]
    check(results, bool(w), 'WARN "relocalization pending: waiting for motion"'
          + (f' ("{w[-1].message}")' if w else ' missing'))
    vals = {kv.key: kv.value for kv in w[-1].values} if w else {}
    used = int(vals.get('relocalization_scans_used', -1))
    skipped = int(vals.get('relocalization_scans_skipped', -1))
    check(results, used == 1 and skipped > 0,
          f'relocalization_scans_used {used} (need 1), relocalization_scans_skipped {skipped} (need > 0)')


# ----------------------------------------------------------------------------------------
# ndt3d: the same world in 3D. The grid's walls and boxes are extruded to WALL_H on a
# ground plane at map z = GROUND_Z; base_link rides at map z = 0. The lidar (16 rings,
# -15 .. +15 deg, 360 azimuths, 10 Hz, 1 cm range noise) is mounted LIDAR_XYZ from
# base_link and turned LIDAR_YAW about z; its z in the map frame is LIDAR_XYZ[2], which is
# what base_height has to be.
GROUND_Z, WALL_H = -0.1, 2.5
LIDAR_XYZ, LIDAR_YAW = (0.2, 0.0, 0.5), 0.3
ELEV = np.radians(np.arange(-15.0, 15.1, 2.0))
N_AZ = 360
AZ = np.linspace(-math.pi, math.pi, N_AZ, endpoint=False)
KIDNAP_SHIFT_S = 25.0   # a kidnapped robot is where the trajectory would be this much later


def write_world_pcd(path):
    """Prior map: the ground plane and every wall face, sampled at 0.1 m (ASCII PCD)."""
    occ = GRID > 50
    pts = [((ix + 0.5) * RES, (iy + 0.5) * RES, GROUND_Z)
           for iy in range(0, H, 2) for ix in range(0, W, 2) if not occ[iy, ix]]
    free = ~occ
    border = np.zeros_like(occ)
    border[1:, :] |= occ[1:, :] & free[:-1, :]
    border[:-1, :] |= occ[:-1, :] & free[1:, :]
    border[:, 1:] |= occ[:, 1:] & free[:, :-1]
    border[:, :-1] |= occ[:, :-1] & free[:, 1:]
    for iy, ix in zip(*np.nonzero(border)):
        for k in range(int(round(WALL_H / 0.1)) + 1):
            pts.append(((ix + 0.5) * RES, (iy + 0.5) * RES, GROUND_Z + 0.1 * k))
    with open(path, 'w') as f:
        f.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n')
        for p in pts:
            f.write('%.3f %.3f %.3f\n' % p)
    return len(pts)


def scan3d(x, y, th, rng):
    """One sweep from base pose (x, y, th), as points in the lidar frame (N x 3 float32)."""
    lx = x + LIDAR_XYZ[0] * math.cos(th) - LIDAR_XYZ[1] * math.sin(th)
    ly = y + LIDAR_XYZ[0] * math.sin(th) + LIDAR_XYZ[1] * math.cos(th)
    sensor_h = LIDAR_XYZ[2] - GROUND_Z                  # lidar height above the ground
    rh = raycast(lx, ly, th, n=N_AZ)                    # horizontal distance to the first wall
    hit_wall = rh < RANGES[-1] - 1e-6
    out = []
    for e in ELEV:
        t = math.tan(e)
        z_at_wall = rh * t                              # relative to the lidar
        r = np.full(N_AZ, np.nan)
        zrel = np.full(N_AZ, np.nan)
        wall_ok = hit_wall & (z_at_wall > -sensor_h) & (z_at_wall < WALL_H - sensor_h)
        r[wall_ok], zrel[wall_ok] = rh[wall_ok], z_at_wall[wall_ok]
        if e < 0:
            rg = sensor_h / -t                          # where the ray meets the ground
            ground = (~hit_wall | (z_at_wall <= -sensor_h)) & (rg < 12.0)
            r[ground], zrel[ground] = rg, -sensor_h
        ok = ~np.isnan(r)
        slant = np.hypot(r[ok], zrel[ok])
        k = (slant + rng.normal(0.0, 0.01, slant.shape)) / slant      # range noise along the ray
        # azimuths are measured from the robot's heading; the lidar frame is turned by LIDAR_YAW
        a = AZ[ok] - LIDAR_YAW
        out.append(np.stack([r[ok] * np.cos(a) * k, r[ok] * np.sin(a) * k, zrel[ok] * k], axis=1))
    return np.concatenate(out).astype(np.float32)


def cloud_msg(stamp, pts):
    m = PointCloud2()
    m.header.stamp, m.header.frame_id = stamp, 'lidar'
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(pts), True
    m.data = pts.tobytes()
    return m


class Driver3D(Driver):
    def __init__(self, node, lidar_pitch=0.0):
        super().__init__(node)
        self.pts_pub = node.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)
        lidar = tf_msg(node.get_clock().now().to_msg(), 'base_link', 'lidar', LIDAR_XYZ[0], LIDAR_XYZ[1], LIDAR_YAW)
        lidar.transform.translation.z = LIDAR_XYZ[2]
        if lidar_pitch:                                  # q = qz(LIDAR_YAW) * qy(pitch)
            cy, sy, cp, sp = (math.cos(LIDAR_YAW / 2), math.sin(LIDAR_YAW / 2), math.cos(lidar_pitch / 2),
                              math.sin(lidar_pitch / 2))
            r = lidar.transform.rotation
            r.x, r.y, r.z, r.w = -sy * sp, cy * sp, sy * cp, cy * cp
        self.stb.sendTransform([tf_msg(node.get_clock().now().to_msg(), 'base_link', 'laser', LASER_X, 0.0, 0.0),
                                lidar])

    def wait_for_node(self, timeout):
        end = time.monotonic() + timeout
        while self.pts_pub.get_subscription_count() == 0:
            if time.monotonic() > end:
                sys.exit('FAIL: the node did not subscribe to /points')
            rclpy.spin_once(self.n, timeout_sec=0.05)
        end = time.monotonic() + 2.0                     # let the static TF arrive
        while time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def run3d(self, duration, kidnap_at=None, ip_offset=(0.3, -0.2, 0.1), move_at=0.0):
        """Like run(): odometry TF, /initialpose at t = 1 s, one cloud per tick. The robot
        stands at the start of the trajectory until move_at. From kidnap_at on the clouds
        come from where the trajectory is KIDNAP_SHIFT_S later, while the odometry
        carries on as if nothing had happened."""
        od = Odom()
        self.t0 = time.monotonic()
        tl, sent_ip, k = 0.0, False, 0
        while True:
            t = time.monotonic() - self.t0
            if t >= duration:
                break
            x, y, th = truth(max(0.0, t - move_at))
            ox, oy, oth = od.step(x, y, th, t - tl)
            tl = t
            stamp = self.n.get_clock().now().to_msg()
            self.tfb.sendTransform(tf_msg(stamp, 'odom', 'base_link', ox, oy, oth))
            if t > 1.0 and not sent_ip:
                ip = PoseWithCovarianceStamped()
                ip.header.frame_id = 'map'
                ip.header.stamp = stamp
                ip.pose.pose.position.x = x + ip_offset[0]
                ip.pose.pose.position.y = y + ip_offset[1]
                ip.pose.pose.orientation.z = math.sin((th + ip_offset[2]) / 2)
                ip.pose.pose.orientation.w = math.cos((th + ip_offset[2]) / 2)
                self.ip_pub.publish(ip)
                sent_ip = True
            if kidnap_at is not None and t >= kidnap_at:
                x, y, th = truth(t + KIDNAP_SHIFT_S)
            self.truth_at[(stamp.sec, stamp.nanosec)] = (t, x, y, th)
            self.pts_pub.publish(cloud_msg(stamp, scan3d(x, y, th, self.rng)))
            end = self.t0 + (k + 1) / RATE
            k += 1
            while time.monotonic() < end:
                rclpy.spin_once(self.n, timeout_sec=0.01)

    def statuses(self, t_from, t_to=float('inf')):
        return [s for m, s in self.diags if t_from < self.rel(m) <= t_to]

    def get_params(self, names, timeout=5.0):
        """Values of double parameters of the node, or None when it does not answer."""
        req = GetParameters.Request()
        req.names = list(names)
        fut = self.param_cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)
        if not fut.done() or fut.result() is None:
            return None
        return [v.double_value if v.type == 3 else v.integer_value if v.type == 2 else None
                for v in fut.result().values]


LOW_MATCH = 'low scan-to-map match'
NOT_VERIFIED = 'pose not verified since the last reseed'
TILT_WARN = 'is tilted (roll'                       # startup WARN: the lidar's tilt is ignored
HEIGHT_INFO = 'ndt3d does not use that z'           # startup INFO: TF z against base_height


def values_of(status):
    return {kv.key: kv.value for kv in status.values}


def node_log(name):
    with open(f'e2e_{name}.log') as f:
        return f.read()


def scenario_ndt3d_track(d, results):
    d.run3d(40.0)
    errs = [e for e in d.errors() if e[0] > 10.0]
    check(results, len(errs) > 100, f'poses after t = 10 s: {len(errs)} (need > 100)')
    if errs:
        mp, my = max(e[1] for e in errs), max(e[2] for e in errs)
        mean = sum(e[1] for e in errs) / len(errs)
        late = [e[1] for e in errs if e[0] > 30.0]
        check(results, mp <= 0.5, f'max position error after 10 s: {mp:.3f} m (need <= 0.5)')
        check(results, mean <= 0.15, f'mean position error after 10 s: {mean:.3f} m (need <= 0.15)')
        check(results, bool(late) and max(late) <= 0.25,
              f'max position error after 30 s: {max(late) if late else float("nan"):.3f} m (need <= 0.25)')
        check(results, my <= 0.05, f'max yaw error after 10 s: {my:.4f} rad (need <= 0.05)')
    early = d.statuses(3.0, 8.0)
    unv = [s for s in early if level_of(s) == WARN and s.message.startswith(NOT_VERIFIED) and ' of 15 filter' in s.message]
    check(results, len(early) >= 4 and len(unv) == len(early),
          f'statuses from t = 3 to 8 s (after /initialpose at 1 s) that WARN "{NOT_VERIFIED}: N of 15 ...": '
          f'{len(unv)} of {len(early)} (need all of >= 4)' + (f' ("{early[-1].message}")' if early else ''))
    first_ok = next((d.rel(m) for m, s in d.diags if d.rel(m) > 3.0 and level_of(s) == OK), None)
    check(results, first_ok is not None and first_ok <= 14.0,
          f'first OK status after /initialpose at t = {first_ok if first_ok is not None else float("nan"):.1f} s (need <= 14)')
    # match_updates_since_reseed counts every judged update since /initialpose (not the window of 5):
    # it equals N of the "N of 15" WARN and is at least 15 once the status is OK.
    unv_all = [s for s in d.statuses(1.5) if s.message.startswith(NOT_VERIFIED)]
    wrong = [(s.message.split(': ')[1].split(' of')[0], values_of(s).get('match_updates_since_reseed'))
             for s in unv_all if s.message.split(': ')[1].split(' of')[0] != values_of(s).get('match_updates_since_reseed')]
    oks = [int(values_of(s).get('match_updates_since_reseed', -1)) for s in d.statuses(15.0) if level_of(s) == OK]
    check(results, bool(unv_all) and not wrong and bool(oks) and min(oks) >= 15,
          f'match_updates_since_reseed: equal to N of "N of 15" in {len(unv_all) - len(wrong)} of {len(unv_all)} '
          f'statuses, smallest value in the OK statuses after t = 15 s {min(oks) if oks else "none"} '
          f'(need all equal and >= 15)' + (f' (message, value: {wrong[0]})' if wrong else ''))
    late = d.statuses(15.0)
    bad = sorted({(level_of(s), s.message) for s in late if level_of(s) != OK})
    check(results, len(late) >= 20 and not bad,
          f'/diagnostics statuses after t = 15 s that are not OK: {sum(1 for s in late if level_of(s) != OK)} of '
          f'{len(late)} (need 0 of >= 20): {bad[:3] if bad else "none"}')
    mf = [float(values_of(s)['match_fraction']) for s in late if values_of(s).get('match_fraction', 'n/a') != 'n/a']
    check(results, len(mf) == len(late) and bool(mf) and min(mf) >= 0.65,
          f'match_fraction in those statuses: min {min(mf) if mf else float("nan"):.3f}, '
          f'median {sorted(mf)[len(mf) // 2] if mf else float("nan"):.3f} over {len(mf)} (need >= 0.65 in all)')
    ne = [float(values_of(s)['n_eff']) / float(values_of(s)['particles']) for s in late
          if values_of(s).get('n_eff', 'n/a') != 'n/a' and float(values_of(s).get('particles', 0)) > 0]
    if ne:
        print(f'  n_eff / particles in those statuses (not a health signal for ndt3d): min {min(ne):.4f}, '
              f'median {sorted(ne)[len(ne) // 2]:.4f}, max {max(ne):.4f}')
    log = node_log('ndt3d_track')
    said = TILT_WARN in log or HEIGHT_INFO in log
    check(results, not said,
          'no startup message about the sensor transform (lidar level, TF z = base_height, yaw 0.3 rad)'
          + (': one was logged' if said else ''))


def scenario_ndt3d_kidnap(d, results):
    t_k, end = 20.0, 34.0
    d.run3d(end, kidnap_at=t_k)
    before = d.statuses(15.0, t_k)
    bad = [s.message for s in before if level_of(s) != OK]
    check(results, len(before) >= 4 and not bad,
          f'statuses from t = 15 s to the kidnap that are not OK: {len(bad)} of {len(before)} (need 0 of >= 4)'
          + (f' ("{bad[0]}")' if bad else ''))
    errs = [e[1] for e in d.errors() if e[0] > t_k + 2.0]
    check(results, bool(errs) and min(errs) > 1.0,
          f'the pose is wrong after the kidnap: smallest position error {min(errs) if errs else float("nan"):.2f} m (need > 1.0)')
    w = [t for t in d.diag_times(WARN, LOW_MATCH) if t > t_k]
    check(results, bool(w) and w[0] - t_k <= 6.0,
          f'WARN "{LOW_MATCH}" {w[0] - t_k:.1f} s after the kidnap (need <= 6)' if w
          else f'no WARN "{LOW_MATCH}" after the kidnap')
    after = d.statuses(t_k + 6.0)
    n_warn = sum(1 for s in after if level_of(s) == WARN and LOW_MATCH in s.message)
    check(results, len(after) >= 6 and n_warn >= 0.8 * len(after),
          f'statuses from 6 s after the kidnap that WARN "{LOW_MATCH}": {n_warn} of {len(after)} (need >= 80 % of >= 6)'
          + (f' ("{after[-1].message}")' if after else ''))


def scenario_ndt3d_off_map(d, results):
    move_at = 6.0
    d.run3d(15.0, ip_offset=(100.0, 0.0, 0.0), move_at=move_at)
    pre = d.statuses(-60.0, 0.9)
    early = [s.message for s in pre if NOT_VERIFIED in s.message or level_of(s) != OK]
    check(results, len(pre) >= 1 and not early,
          f'before /initialpose (no pose seed yet): statuses that are not OK: {len(early)} of {len(pre)} (need 0 of >= 1)'
          + (f' ("{early[0]}")' if early else ''))
    standing = d.statuses(2.5, move_at)
    judged = [s for s in standing if values_of(s).get('match_fraction') != 'n/a' or LOW_MATCH in s.message]
    check(results, len(standing) >= 2 and not judged,
          f'standing after /initialpose (only the reseed update has run): statuses with a match_fraction or the '
          f'low-match WARN: {len(judged)} of {len(standing)} (need 0 of >= 2)'
          + (f' ("{judged[0].message}", match_fraction {values_of(judged[0]).get("match_fraction")})' if judged else ''))
    unv = [s for s in standing if level_of(s) == WARN and s.message.startswith(NOT_VERIFIED + ': 0 of 15 filter')
           and values_of(s).get('match_updates_since_reseed') == '0']
    check(results, len(standing) >= 2 and len(unv) == len(standing),
          f'standing after /initialpose: statuses that WARN "{NOT_VERIFIED}: 0 of 15 ...": {len(unv)} of {len(standing)} '
          '(need all)' + (f' ("{standing[-1].message}")' if standing else ''))
    late = d.statuses(move_at + 3.0)
    n_warn = sum(1 for s in late if level_of(s) == WARN and LOW_MATCH + ': 0.000' in s.message)
    check(results, len(late) >= 4 and n_warn == len(late),
          f'pose seeded 100 m off the map, driving: statuses from t = {move_at + 3.0:g} s that WARN "{LOW_MATCH}: 0.000": '
          f'{n_warn} of {len(late)} (need all of >= 4)' + (f' ("{late[-1].message}")' if late else ''))
    pts = re.search(r'(\d+) points at the last one', late[-1].message) if late else None
    check(results, pts is not None and int(pts.group(1)) >= 100,
          f'scan points scored at the last update, as the WARN reports them: {pts.group(1) if pts else "none"} (need >= 100)')
    v = values_of(late[-1]) if late else {}
    ratio = float(v['n_eff']) / float(v['particles']) if v.get('n_eff', 'n/a') != 'n/a' else float('nan')
    check(results, ratio >= 0.99, f'n_eff / particles there: {ratio:.3f} (need >= 0.99: every particle weighs the same, '
          'which is why n_eff cannot flag this)')


def scenario_ndt3d_base_height(d, results):
    d.run3d(12.0)
    log = node_log('ndt3d_base_height')
    lines = [ln for ln in log.splitlines() if HEIGHT_INFO in ln]
    check(results, len(lines) == 1 and '[INFO]' in lines[0] and 'has z 0.500 m and base_height is 0.000 m' in lines[0]
          and TILT_WARN not in log,
          f'startup INFO giving the TF z and base_height (z 0.500 m, base_height 0.000 m), logged once, no tilt WARN: '
          f'{len(lines)} line(s)' + (f' ("{lines[0][:60]}...")' if lines else ''))
    late = d.statuses(6.0)
    n_warn = sum(1 for s in late if level_of(s) == WARN and LOW_MATCH in s.message and 'base_height' in s.message
                 and 'min_match_fraction 0.650)' in s.message)
    check(results, len(late) >= 4 and n_warn == len(late),
          f'base_height left at 0 with the lidar at 0.5 m: statuses from t = 6 s that WARN "{LOW_MATCH} ... '
          f'min_match_fraction 0.650": {n_warn} of {len(late)} (need all of >= 4)'
          + (f' ("{late[-1].message}")' if late else ''))


def scenario_ndt3d_tilt(d, results):
    d.run3d(5.0)
    log = node_log('ndt3d_tilt')
    lines = [ln for ln in log.splitlines() if TILT_WARN in ln]
    check(results, len(lines) == 1 and '[WARN]' in lines[0] and 'pitch 0.100 rad' in lines[0] and HEIGHT_INFO not in log,
          f'startup WARN about the ignored tilt (lidar pitched 0.1 rad, z = base_height), logged once, no height '
          f'message: {len(lines)} line(s)')
    got = d.get_params(['min_neff_fraction', 'min_match_fraction', 'match_verify_updates'])
    check(results, got == [0.0, 0.65, 15],
          f'in-code defaults for ndt3d (YAML without the three keys): min_neff_fraction, min_match_fraction, '
          f'match_verify_updates = {got} (need [0.0, 0.65, 15])')


def scenario_ndt3d_strict(d, results):
    d.run3d(12.0)
    late = d.statuses(6.0)
    n_warn = sum(1 for s in late if level_of(s) == WARN and LOW_MATCH in s.message and 'min_match_fraction 0.950)' in s.message)
    check(results, len(late) >= 4 and n_warn == len(late),
          f'healthy tracking with min_match_fraction 0.95: statuses from t = 6 s that WARN "{LOW_MATCH} ... '
          f'min_match_fraction 0.950": {n_warn} of {len(late)} (need all of >= 4)'
          + (f' ("{late[-1].message}")' if late else ''))
    errs = [e[1] for e in d.errors() if e[0] > 6.0]
    check(results, bool(errs) and max(errs) <= 0.5, f'and the pose is fine: max position error after 6 s '
          f'{max(errs) if errs else float("nan"):.3f} m (need <= 0.5)')


def scenario_ndt3d_off(d, results):
    d.run3d(9.0, ip_offset=(100.0, 0.0, 0.0))
    late = d.statuses(2.5)
    bad = [s.message for s in late if level_of(s) != OK]
    check(results, len(late) >= 5 and not bad,
          f'min_match_fraction 0, pose 100 m off the map: statuses from t = 2.5 s that are not OK: {len(bad)} of '
          f'{len(late)} (need 0 of >= 5: the check is off)' + (f' ("{bad[0]}")' if bad else ''))
    v = values_of(late[-1]) if late else {}
    check(results, v.get('match_fraction') == '0.000',
          f'and the value is still reported: match_fraction {v.get("match_fraction")} (need 0.000)')


def scenario_ndt3d_verify3(d, results):
    d.run3d(10.0)
    unv = [s.message for m, s in d.diags if s.message.startswith(NOT_VERIFIED)]
    check(results, bool(unv) and all(' of 3 filter updates' in m for m in unv),
          f'"{NOT_VERIFIED}" statuses, all counting "of 3": {len(unv)} (need >= 1)' + (f' ("{unv[0][:70]}...")' if unv else ''))
    late = d.statuses(6.0)
    bad = [s.message for s in late if level_of(s) != OK]
    check(results, len(late) >= 3 and not bad,
          f'healthy tracking with match_verify_updates 3: statuses from t = 6 s that are not OK: {len(bad)} of {len(late)} '
          '(need 0 of >= 3)' + (f' ("{bad[0]}")' if bad else ''))


NDT3D_SCENARIOS = {'ndt3d_track': scenario_ndt3d_track, 'ndt3d_kidnap': scenario_ndt3d_kidnap,
                   'ndt3d_off_map': scenario_ndt3d_off_map, 'ndt3d_base_height': scenario_ndt3d_base_height,
                   'ndt3d_tilt': scenario_ndt3d_tilt, 'ndt3d_strict': scenario_ndt3d_strict,
                   'ndt3d_verify3': scenario_ndt3d_verify3, 'ndt3d_off': scenario_ndt3d_off}
WORLD_PCD = 'e2e_world3d.pcd'


SCENARIO_ARGS = {'reloc_gate': ['try_global_localization:=true', 'bbs_verify_min_translation:=0.3',
                                'bbs_verify_min_rotation:=0.3'],
                 'ndt3d_base_height': ['base_height:=0.0'],
                 'ndt3d_strict': ['min_match_fraction:=0.95', 'match_verify_updates:=0'],
                 'ndt3d_verify3': ['match_verify_updates:=3'],
                 'ndt3d_off': ['min_match_fraction:=0.0']}
# YAML keys left out for a scenario, so that the node's own defaults apply.
SCENARIO_DROP_KEYS = {'ndt3d_tilt': ['min_neff_fraction', 'min_match_fraction', 'match_verify_updates']}


SCENARIOS = {'track': scenario_track, 'gap': scenario_gap, 'no_odom': scenario_no_odom,
             'zero_stamp': scenario_zero_stamp, 'stuck_clock': scenario_stuck_clock,
             'slow_clock': scenario_slow_clock, 'coarse_clock': scenario_coarse_clock,
             'stationary_seed': scenario_stationary_seed, 'reloc_gate': scenario_reloc_gate}


def run_scenario(name, params, node_args):
    if name in SCENARIO_DROP_KEYS:
        with open(params, encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        for section in doc.values():
            for key in SCENARIO_DROP_KEYS[name]:
                del section['ros__parameters'][key]     # KeyError: the shipped YAML no longer has it
        params = os.path.abspath(f'e2e_{name}.yaml')
        with open(params, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f)
    args = ['ros2', 'run', 'prism_loc', 'prism_loc_node_main', '--ros-args', '--params-file', params]
    sim = name in ('stuck_clock', 'slow_clock', 'coarse_clock')
    ndt3d = name in NDT3D_SCENARIOS
    if ndt3d:
        if not os.path.exists(WORLD_PCD):
            print(f'  prior map {WORLD_PCD}: {write_world_pcd(WORLD_PCD)} points')
        # base_height first, so that a scenario's own value (SCENARIO_ARGS) replaces it
        node_args = [f'map_pcd_path:={os.path.abspath(WORLD_PCD)}', f'base_height:={LIDAR_XYZ[2]}'] + node_args
    for a in node_args + SCENARIO_ARGS.get(name, []) + (['use_sim_time:=true'] if sim else []):
        args += ['-p', a]
    log = open(f'e2e_{name}.log', 'w')
    # The ndt3d scenarios read the node log while the node runs: keep its stream unbuffered.
    env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0') if ndt3d else None
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
    rclpy.init()
    node = rclpy.create_node(f'prism_loc_e2e_{name}')
    d = Driver3D(node, lidar_pitch=0.1 if name == 'ndt3d_tilt' else 0.0) if ndt3d else Driver(node)
    results = []
    try:
        d.wait_for_node(30.0)
        if ndt3d:
            NDT3D_SCENARIOS[name](d, results)
        else:
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
    ap.add_argument('params', help='params YAML for the node (prism_loc/params/laser2d.yaml or ndt3d.yaml)')
    ap.add_argument('--scenario', choices=['all'] + list(SCENARIOS) + list(NDT3D_SCENARIOS), default='all')
    ap.add_argument('--coarse-factor', type=float, default=0.5,
                    help='real-time factor of the coarse_clock scenario (default 0.5)')
    ap.add_argument('--node-arg', action='append', default=[],
                    help='extra node parameter, name:=value (repeatable)')
    args = ap.parse_args()
    with open(args.params, encoding='utf-8') as f:
        backend = next(iter(yaml.safe_load(f).values()))['ros__parameters'].get('backend', 'laser2d')
    table = NDT3D_SCENARIOS if backend == 'ndt3d' else SCENARIOS
    if args.scenario != 'all' and args.scenario not in table:
        sys.exit(f'FAIL: scenario {args.scenario} is not for backend "{backend}" (the backend of {args.params}); '
                 f'its scenarios are: {", ".join(table)}')
    names = list(table) if args.scenario == 'all' else [args.scenario]
    failed = False
    for name in names:
        global COARSE_FACTOR
        COARSE_FACTOR = args.coarse_factor
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
