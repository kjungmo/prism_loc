#!/usr/bin/env python3
"""Check that fusion3d honours the mounts of its sensors on base_link, over real topics and TF.

A robot stands still at a known pose in a 12 x 9 m room (floor, four 2.5 m walls, three
boxes; surfaces sampled at random with 5 cm of noise, written to fusion_extrinsic_map.pcd).
`fusion_node_main` is started once per scenario and fed synthetic sensors built from that
truth, each expressed in its own frame:
  * /points  other random samples of the same surfaces as seen from a lidar mounted at
             LIDAR on base_link (1 Hz)
  * /imu     the specific force of a level, resting robot as measured by an IMU mounted
             with the rotation IMU on base_link (100 Hz)
  * /gnss    the position of an antenna mounted at ANTENNA on base_link (5 Hz, datum fixed)
The mounts are published as static TF (base_link -> lidar / imu / gnss).

Scenarios (--scenario, default all). Thresholds were written before the first run: the
pose of base_link within POS_TOL of the truth on each axis and within ANG_TOL in roll,
pitch and yaw. Every mount is 4 x POS_TOL or 10 x ANG_TOL away from the identity at least,
so a mount that is ignored or applied the wrong way round fails.
  lidar        cloud in the lidar frame, IMU in base_link, /initialpose, no GNSS: pose of
               base_link (not of the lidar) within tolerance, status OK, the mount reported
  imu          IMU in its rotated frame, GNSS in base_link, no cloud, no /initialpose; the
               robot turns on the spot at 0.3 rad/s for 3 s: the attitude taken from
               gravity is level, the yaw follows the turn and the position holds
  gnss         antenna frame, IMU in base_link, no cloud, no /initialpose; the robot turns
               on the spot as in "imu": the first pose (initialized from the first fix)
               and the pose after the turn are within tolerance of base_link (the
               antenna is 0.36 m away horizontally and 1 m above)
  gnss_missing antenna frame without a transform, /initialpose: WARN "GNSS not used", no
               GNSS correction, and the pose stays where /initialpose put it
  gnss_stand_a, gnss_stand_b, gnss_turn_a, gnss_turn_b
               GNSS and IMU only, with sensor noise (accelerometer 0.02 m/s^2, gyro
               0.002 rad/s, fix 0.02 m), 30 s, antenna at (0.5, 0, 1.0) (a) or
               (-0.3, 0.2, 1.0) (b), standing or turning on the spot at 0.3 rad/s (4 s
               of turning, 1 s still, repeated). At t = 10, 15, .. 30 s: height within 0.15 m, tilt within 0.03 rad, heading
               within 0.05 rad of the truth, and the horizontal error within twice the
               sigma the node reports (sqrt of covariance_trace_position). The fix does
               not observe the heading here, so the horizontal error is not small: it
               is bounded by what the filter says it is.
  gnss_bias_stand_a, gnss_bias_turn_b, gnss_bias_big_turn_a
               GNSS and IMU only as above, 45 s, with a constant gyro bias on every axis: the
               bias of the review (0.003, -0.002, 0.002 rad/s) standing with antenna A and
               turning with antenna B, and 0.5 deg/s on each axis (+, -, +) turning with A. At
               t = 15, 30, 45 s: height within 0.15 m, tilt within 0.03 rad, heading within
               0.05 rad + vertical bias x t of the truth (the heading is not corrected by the
               fix, so it drifts with the bias), horizontal error within 3 x the node's sigma
  The gnss_bias_*, gnss_reseed, gnss_late_pose, gnss_after_ndt, gnss_reseed_before_fix and gnss_pose_first
               scenarios also assert, in the published pose covariance 0.25 to 0.6 s after the first fix and
               after each /initialpose: roll/pitch variance at most 2e-3 rad^2 (it starts at about 1e-4 from the
               accelerometer; the heading's 0.5 rad^2 would fail); without lidar clouds the heading variance
               between 0.3 and 0.7 rad^2 (one antenna does not observe it); with clouds the heading variance at
               most 0.05 rad^2 and the position covariance trace at most 0.06 m^2.
  gnss_reseed  antenna A, standing, the review's bias; /initialpose at t = 15 s (0.04 m,
               -0.03 m, 0.01 rad off); the checks above at t = 30, 45 s (heading within 0.06 rad
               + vertical bias x time since the reseed)
  gnss_late_pose  antenna A, standing, the review's bias; /initialpose and the IMU for 1 s
               before the first fix, so that the filter starts before the antenna mount is known;
               the checks above at t = 15, 30, 45 s after the first fix
  gnss_after_ndt  antenna A, standing, the review's bias; /initialpose, the IMU and lidar clouds for 8 s
               before the first fix (the lidar corrects the filter, the antenna mount is not known
               yet); then the fixes join: at t = 15, 30 s after the first fix, height within 0.15 m,
               tilt within 0.03 rad, heading within 0.1 rad, horizontal error within 3 x the node's
               sigma, and the position covariance trace within 0.06 m^2 (a heading variance of 0.5 rad^2
               on a 0.5 m lever arm alone would add 0.125)
  gnss_reseed_before_fix  antenna A, standing, the review's bias; /initialpose and lidar clouds for 4 s,
               a second /initialpose, 1.5 s of the IMU alone, then the fixes: the checks of
               gnss_late_pose at t = 15, 30, 45 s
  gnss_pose_first  antenna A, standing, the review's bias; /initialpose and GNSS fixes for 1 s before the
               first IMU sample (the filter starts without an accelerometer reading): the checks of
               gnss_late_pose at t = 15, 30, 45 s
  gnss_pose_first_wait, gnss_pose_first_base
               /initialpose and fixes for 1 s before the first IMU sample, then the IMU: with the
               antenna at A (_wait) no pose is published during that second (the fixes wait for the
               first accelerometer sample, which seeds the roll/pitch covariance) and poses follow
               within 1 s of the first IMU sample; with the antenna at base_link (_base, no TF) the
               fixes are used as before and at least 3 poses are published during that second
  slash        frame_id "/base_link" on the IMU and the fix, "/lidar" on the cloud (with
               base_link -> lidar on TF): one leading slash is dropped, pose within
               tolerance
  missing      cloud frame without a transform: WARN "point clouds not used" naming
               base_link -> lidar, no correction applied; after the transform is
               published the warning is gone, the status is OK and the pose is right
  empty_frame  cloud with an empty frame_id: WARN "point clouds not used" naming points_frame
  override     cloud, IMU and fix expressed in base_link under frames that are not on TF,
               with points_frame, imu_frame and gnss_frame set to base_link: used as they
               are, pose within tolerance, status OK
  imu_missing  IMU frame without a transform (startup_timeout_s 10): WARN "IMU not used"
               first, ERROR after the startup grace and no pose; once the transform is
               published the ERROR is gone and the pose holds. Then the IMU switches to
               a frame without a transform: ERROR "IMU not used" (the only error now)
               and no new pose; back on the first frame the ERROR is gone again

    fusion_extrinsic_check.py PARAMS_YAML [--scenario NAME] [--tolerate-imu-gaps]

--tolerate-imu-gaps accepts a WARN that reports nothing but "IMU gap(s)" where the status
has to be OK (never "IMU stopped", never an ERROR). CI passes it: on a two-core runner
this script's own 100 Hz loop can stall for a few samples. Without it the status must be OK.
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
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

STATUS = 'prism_loc_fusion: fusion'
OK, WARN, ERROR = 0, 1, 2
POS_TOL, ANG_TOL = 0.05, 0.02                       # m per axis, rad per angle
ROOM = np.array([0.31, 0.23, 0.37])                 # map position of the room's floor centre
BASE_XYZ, BASE_YAW = ROOM + np.array([1.0, -0.5, 0.3]), 0.6  # truth: base_link in the map, level
LIDAR = ((0.2, 0.1, 0.5), (0.0, 0.2, 1.0))          # xyz, rpy of base_link -> lidar
IMU = ((0.1, 0.0, 0.2), (0.3, -0.2, 1.2))           # base_link -> imu (only the rotation matters)
ANTENNA = ((-0.3, 0.2, 1.0), (0.0, 0.0, 0.0))       # base_link -> gnss (only the translation matters)
ANTENNA_A = ((0.5, 0.0, 1.0), (0.0, 0.0, 0.0))     # the two antenna mounts of the noisy GNSS-only scenarios
ANTENNA_B = ANTENNA
NOISE = {'acc': 0.02, 'gyro': 0.002, 'gnss': 0.02}  # m/s^2, rad/s, m (white, per axis)
DATUM = (37.0, 127.0, 0.0)
TOLERATE_IMU_GAPS = False                           # --tolerate-imu-gaps
# The room is built for PCL NDT at the shipped 1 m cells: surfaces 5 cm thick (a thinner
# wall attracts the registration only from within a few centimetres), the floor sampled at
# a fifth of its area (a large flat floor pulls the registration toward the cell lattice),
# and the room placed so that no wall or floor lies on a cell border.
SURF_NOISE, FLOOR_SHARE = 0.05, 0.2
# Kept small on purpose: the node registers a cloud on the thread that also takes the IMU,
# so a large map or a fast cloud stream starves the IMU on a slow machine.
MAP_POINTS, CLOUD_POINTS = 20000, 3000   # and one cloud per second
IP_OFFSET = (0.04, -0.03, 0.01)                     # /initialpose off the truth: x, y (m), yaw (rad)


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rpy_quat(x, y, z, w):
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


R_BASE = rot_rpy(0.0, 0.0, BASE_YAW)


def level_of(st):
    return st.level[0] if isinstance(st.level, bytes) else st.level


def surfaces(rng, n):
    """n points sampled at random on the room's surfaces (map frame), with SURF_NOISE."""
    # each surface: origin, edge u, edge v
    quads = [((-6, -4.5, 0), (12, 0, 0), (0, 9, 0)),                                  # floor
             ((-6, -4.5, 0), (12, 0, 0), (0, 0, 2.5)), ((-6, 4.5, 0), (12, 0, 0), (0, 0, 2.5)),
             ((-6, -4.5, 0), (0, 9, 0), (0, 0, 2.5)), ((6, -4.5, 0), (0, 9, 0), (0, 0, 2.5))]
    for (x0, y0, x1, y1, h) in [(3.0, 1.5, 4.0, 3.0, 1.5), (-4.0, -3.0, -2.5, -2.0, 2.0), (-1.0, 2.5, 0.5, 3.5, 1.0)]:
        quads += [((x0, y0, 0), (x1 - x0, 0, 0), (0, 0, h)), ((x0, y1, 0), (x1 - x0, 0, 0), (0, 0, h)),
                  ((x0, y0, 0), (0, y1 - y0, 0), (0, 0, h)), ((x1, y0, 0), (0, y1 - y0, 0), (0, 0, h)),
                  ((x0, y0, h), (x1 - x0, 0, 0), (0, y1 - y0, 0))]
    q = np.array(quads, dtype=float)
    area = np.linalg.norm(np.cross(q[:, 1], q[:, 2]), axis=1)
    area[0] *= FLOOR_SHARE
    which = rng.choice(len(q), size=n, p=area / area.sum())
    a, b = rng.random(n), rng.random(n)
    pts = q[which, 0] + a[:, None] * q[which, 1] + b[:, None] * q[which, 2]
    return ROOM + pts + rng.normal(0.0, SURF_NOISE, pts.shape)


def write_pcd(path, pts):
    with open(path, 'w') as f:
        f.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n')
        for p in pts:
            f.write('%.3f %.3f %.3f\n' % tuple(p))


def static_tf(node, child, mount):
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = 'base_link', child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = mount[0]
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
     t.transform.rotation.w) = quat_rpy(*mount[1])
    return t


class Rig:
    def __init__(self, node):
        self.n = node
        self.imu_pub = node.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.pts_pub = node.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)
        self.gnss_pub = node.create_publisher(NavSatFix, '/gnss', qos_profile_sensor_data)
        self.ip_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.stb = StaticTransformBroadcaster(node)
        self.statics = []
        self.diags, self.poses = [], []
        node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 20)
        node.create_subscription(PoseWithCovarianceStamped, '/prism_loc_fusion/pose',
                                 lambda m: self.poses.append((time.monotonic(), m)), 200)
        rng = np.random.default_rng(5)
        # clouds are prepared up front (sampling one takes long enough to stall the 100 Hz IMU)
        self.clouds = [(surfaces(rng, CLOUD_POINTS) - BASE_XYZ) @ R_BASE for _ in range(5)]   # in base_link
        self.sent_clouds = 0
        self.turned = 0.0        # yaw the robot has turned so far (rad), as the gyro reported it
        self.noise = None        # NOISE to add white noise to the IMU and the fixes
        self.gyro_bias = np.zeros(3)   # constant gyro bias, in the IMU frame (rad/s)
        self.noise_rng = np.random.default_rng(17)
        self.last_imu_s = None   # stamp of the previous IMU sample
        self.t0 = time.monotonic()

    def on_diag(self, m):
        self.diags.extend((time.monotonic(), s) for s in m.status if s.name == STATUS)

    def add_static(self, child, mount):
        self.statics.append(static_tf(self.n, child, mount))
        self.stb.sendTransform(self.statics)

    def initial_pose(self):
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = self.n.get_clock().now().to_msg()
        ip.pose.pose.position.x = BASE_XYZ[0] + IP_OFFSET[0]
        ip.pose.pose.position.y = BASE_XYZ[1] + IP_OFFSET[1]
        ip.pose.pose.position.z = BASE_XYZ[2]
        ip.pose.pose.orientation.z = math.sin((BASE_YAW + IP_OFFSET[2]) / 2)
        ip.pose.pose.orientation.w = math.cos((BASE_YAW + IP_OFFSET[2]) / 2)
        self.ip_pub.publish(ip)

    def send_imu(self, frame, mount, yaw_rate=0.0):
        m = Imu()
        now = self.n.get_clock().now()
        m.header.stamp = now.to_msg()
        m.header.frame_id = frame
        f = np.array([0.0, 0.0, 9.81])               # specific force of a level robot turning on the spot, in base_link
        w = np.array([0.0, 0.0, yaw_rate])
        if mount is not None:
            f = rot_rpy(*mount[1]).T @ f             # ... as the mounted IMU measures them
            w = rot_rpy(*mount[1]).T @ w
        w = w + self.gyro_bias
        if self.noise:
            f = f + self.noise_rng.normal(0.0, self.noise['acc'], 3)
            w = w + self.noise_rng.normal(0.0, self.noise['gyro'], 3)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = f
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
        now_s = now.nanoseconds * 1e-9               # the filter integrates each sample over the time since the previous one
        if self.last_imu_s is not None:
            self.turned += yaw_rate * (now_s - self.last_imu_s)
        self.last_imu_s = now_s
        m.orientation_covariance[0] = -1.0
        self.imu_pub.publish(m)

    def send_cloud(self, frame, mount):
        pts = self.clouds[self.sent_clouds % len(self.clouds)]
        self.sent_clouds += 1
        if mount is not None:                         # express the points in the lidar frame
            pts = (pts - np.array(mount[0])) @ rot_rpy(*mount[1])
        pts = pts.astype(np.float32)
        m = PointCloud2()
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, c in enumerate('xyz')]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(pts), True
        m.data = pts.tobytes()
        self.pts_pub.publish(m)

    def send_gnss(self, frame, mount):
        p = BASE_XYZ if mount is None else \
            BASE_XYZ + rot_rpy(0.0, 0.0, BASE_YAW + self.turned) @ np.array(mount[0])
        if self.noise:
            p = p + self.noise_rng.normal(0.0, self.noise['gnss'], 3)
        a, f = 6378137.0, 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        sl = math.sin(math.radians(DATUM[0]))
        r_n = a / math.sqrt(1.0 - e2 * sl * sl)
        r_m = a * (1.0 - e2) / (1.0 - e2 * sl * sl) ** 1.5
        m = NavSatFix()
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.status.status = 2
        m.latitude = DATUM[0] + math.degrees(p[1] / r_m)
        m.longitude = DATUM[1] + math.degrees(p[0] / (r_n * math.cos(math.radians(DATUM[0]))))
        m.altitude = DATUM[2] + p[2]
        m.position_covariance = [4e-4, 0.0, 0.0, 0.0, 4e-4, 0.0, 0.0, 0.0, 1e-2]
        m.position_covariance_type = 2
        self.gnss_pub.publish(m)

    def stream(self, seconds, imu=('base_link', None), cloud=None, gnss=None, yaw_rate=0.0):
        """100 Hz IMU; every 20th tick (5 Hz) a fix and every 100th (1 Hz) a cloud, when given as
        (frame, mount)."""
        end = time.monotonic() + seconds
        k = 0
        while time.monotonic() < end:
            if imu is not None:
                self.send_imu(*imu, yaw_rate=yaw_rate)
            if k % 100 == 0 and cloud is not None:
                self.send_cloud(*cloud)
            if k % 20 == 0 and gnss is not None:
                self.send_gnss(*gnss)
            k += 1
            tick = time.monotonic() + 0.01
            while time.monotonic() < tick:
                rclpy.spin_once(self.n, timeout_sec=0.002)

    def now(self):
        return time.monotonic()

    def last_status(self):
        return self.diags[-1][1] if self.diags else None

    def values(self):
        st = self.last_status()
        return {kv.key: kv.value for kv in st.values} if st else {}

    def statuses(self, t_from, t_to=float('inf')):
        return [s for t, s in self.diags if t_from <= t <= t_to]

    def pose_error(self, last_s=3.0):
        """Mean pose over the last 3 s against the truth: (dx, dy, dz, roll, pitch, dyaw, n)."""
        t1 = time.monotonic()
        ms = [m for t, m in self.poses if t >= t1 - last_s]
        if not ms:
            return None
        out = np.zeros(6)
        for m in ms:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            r, pi, y = rpy_quat(q.x, q.y, q.z, q.w)
            dyaw = (y - BASE_YAW - self.turned + math.pi) % (2 * math.pi) - math.pi
            out += np.array([p.x - BASE_XYZ[0], p.y - BASE_XYZ[1], p.z - BASE_XYZ[2], r, pi, dyaw])
        return tuple(out / len(ms)) + (len(ms),)


def check_pose(rig, results, what, axes='xyzrpw'):
    e = rig.pose_error()
    if e is None:
        results.append((False, f'{what}: no pose published in the last 3 s'))
        return
    names = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'w': 5}
    ok = all(abs(e[names[a]]) <= (POS_TOL if names[a] < 3 else ANG_TOL) for a in axes)
    results.append((ok, f'{what}: base_link pose error x {e[0]:+.3f} y {e[1]:+.3f} z {e[2]:+.3f} m, roll {e[3]:+.4f} '
                        f'pitch {e[4]:+.4f} yaw {e[5]:+.4f} rad over {e[6]} poses (checked "{axes}": need <= {POS_TOL} m, '
                        f'<= {ANG_TOL} rad)'))


def clean(st):
    """True for an OK status. With --tolerate-imu-gaps also for a WARN that reports nothing
    but IMU gaps (this script's own 100 Hz loop stalling); "IMU stopped" never passes."""
    if level_of(st) == OK:
        return True
    rest = re.sub(r'\d+ IMU gap\(s\) in the last second \(samples lost; \d+ total\)', '', st.message)
    return TOLERATE_IMU_GAPS and level_of(st) == WARN and not rest.strip('; ')


def check_clean(rig, results, what):
    st = rig.last_status()
    results.append((st is not None and clean(st),
                    f'{what}: /diagnostics level {level_of(st) if st else None} ("{st.message if st else ""}"; '
                    'need OK' + (', or a WARN about IMU gaps only' if TOLERATE_IMU_GAPS else '') + ')'))


def scenario_lidar(rig, results):
    rig.add_static('lidar', LIDAR)
    rig.stream(1.0)
    rig.initial_pose()
    rig.stream(7.0, cloud=('lidar', LIDAR))
    check_pose(rig, results, 'lidar mounted at (0.2, 0.1, 0.5), rpy (0, 0.2, 1.0)')
    check_clean(rig, results, 'lidar mounted')
    v = rig.values()
    results.append((v.get('points_mount', '').startswith('base_link -> lidar xyz 0.200 0.100 0.500 rpy 0.000 0.200 1.000'),
                    f'points_mount reported: "{v.get("points_mount")}"'))
    results.append((v.get('seconds_since_ndt_correction', 'never') != 'never' and v.get('ndt_rejected_total') == '0',
                    f'NDT corrections accepted (seconds_since_ndt_correction {v.get("seconds_since_ndt_correction")}, '
                    f'ndt_rejected_total {v.get("ndt_rejected_total")}; need a number and 0)'))
    results.append((v.get('imu_mount') == 'identity (frame base_link)', f'imu_mount reported: "{v.get("imu_mount")}"'))


def scenario_imu(rig, results):
    rig.add_static('imu', IMU)
    rig.stream(2.0, imu=('imu', IMU), gnss=('base_link', None))
    rig.stream(3.0, imu=('imu', IMU), gnss=('base_link', None), yaw_rate=0.3)
    rig.stream(3.5, imu=('imu', IMU), gnss=('base_link', None))
    results.append((0.8 < rig.turned < 1.0, f'IMU mounted: the robot turned {rig.turned:.3f} rad (need 0.8 .. 1.0)'))
    check_pose(rig, results, 'IMU mounted with rpy (0.3, -0.2, 1.2), after the turn')
    check_clean(rig, results, 'IMU mounted')
    v = rig.values()
    results.append((v.get('imu_mount', '').startswith('base_link -> imu xyz 0.100 0.000 0.200 rpy 0.300 -0.200 1.200'),
                    f'imu_mount reported: "{v.get("imu_mount")}"'))


def scenario_gnss(rig, results):
    rig.add_static('gnss', ANTENNA)
    rig.stream(2.0, gnss=('gnss', ANTENNA))
    if rig.poses:
        p = rig.poses[0][1].pose.pose.position
        e0 = (p.x - BASE_XYZ[0], p.y - BASE_XYZ[1], p.z - BASE_XYZ[2])
        results.append((all(abs(c) <= POS_TOL for c in e0),
                        f'antenna mounted: first pose (from the first fix) off base_link by x {e0[0]:+.3f} y {e0[1]:+.3f} '
                        f'z {e0[2]:+.3f} m (need <= {POS_TOL})'))
    else:
        results.append((False, 'antenna mounted: no pose after 2 s of IMU and GNSS'))
    rig.stream(3.0, gnss=('gnss', ANTENNA), yaw_rate=0.3)
    rig.stream(3.5, gnss=('gnss', ANTENNA))
    results.append((0.8 < rig.turned < 1.0, f'antenna mounted: the robot turned {rig.turned:.3f} rad (need 0.8 .. 1.0)'))
    check_pose(rig, results, 'antenna mounted at (-0.3, 0.2, 1.0), after the turn')
    check_clean(rig, results, 'antenna mounted')
    v = rig.values()
    results.append((v.get('gnss_mount', '').startswith('base_link -> gnss xyz -0.300 0.200 1.000'),
                    f'gnss_mount reported: "{v.get("gnss_mount")}"'))
    results.append((v.get('seconds_since_gnss_correction', 'never') != 'never',
                    f'GNSS corrections applied (seconds_since_gnss_correction {v.get("seconds_since_gnss_correction")})'))


def scenario_gnss_missing(rig, results):
    rig.stream(1.0)
    rig.initial_pose()
    t1 = rig.now()
    rig.stream(5.0, gnss=('gnss', ANTENNA))
    late = rig.statuses(t1 + 2.0)
    w = [s for s in late if level_of(s) == WARN and 'GNSS not used: no transform base_link -> gnss' in s.message
         and 'gnss_frame' in s.message]
    results.append((bool(late) and len(w) == len(late),
                    f'no transform for the antenna frame: {len(w)} of {len(late)} statuses WARN "GNSS not used: no transform '
                    f'base_link -> gnss ..." ("{late[-1].message if late else ""}")'))
    v = rig.values()
    results.append((v.get('gnss_mount') == 'missing (frame gnss)' and v.get('seconds_since_gnss_correction') == 'never',
                    f'no transform: gnss_mount "{v.get("gnss_mount")}", seconds_since_gnss_correction '
                    f'{v.get("seconds_since_gnss_correction")} (need "missing (frame gnss)" and never)'))
    e = rig.pose_error()
    ok = e is not None and abs(e[0] - IP_OFFSET[0]) <= POS_TOL and abs(e[1] - IP_OFFSET[1]) <= POS_TOL and abs(e[2]) <= POS_TOL
    results.append((ok, 'no transform: the pose stays at /initialpose, error '
                        + (f'x {e[0]:+.3f} y {e[1]:+.3f} z {e[2]:+.3f} m (need {IP_OFFSET[0]:+.2f}, {IP_OFFSET[1]:+.2f}, 0 '
                           f'within {POS_TOL} m; the antenna is 0.36 m and 1 m away)' if e else 'n/a: no pose')))


def gnss_only_noisy(rig, results, mount, yaw_rate):
    rig.noise = NOISE
    rig.add_static('gnss', mount)
    lever_xy = math.hypot(mount[0][0], mount[0][1])
    rig.stream(5.0, gnss=('gnss', mount), yaw_rate=yaw_rate)
    worst = {'z': 0.0, 'tilt': 0.0, 'yaw': 0.0, 'xy_over_sigma': 0.0, 'xy': 0.0}
    n = 0
    for _ in range(5):                                # t = 10, 15, .. 30 s
        rig.stream(4.0, gnss=('gnss', mount), yaw_rate=yaw_rate)
        rig.stream(1.0, gnss=('gnss', mount))         # hold still for the second the pose is read in
        e = rig.pose_error(1.0)
        trace = rig.values().get('covariance_trace_position', 'n/a')
        if e is None or trace == 'n/a':
            continue
        n += 1
        xy = math.hypot(e[0], e[1])
        worst['z'] = max(worst['z'], abs(e[2]))
        worst['tilt'] = max(worst['tilt'], math.hypot(e[3], e[4]))
        worst['yaw'] = max(worst['yaw'], abs(e[5]))
        worst['xy'] = max(worst['xy'], xy)
        worst['xy_over_sigma'] = max(worst['xy_over_sigma'], xy / math.sqrt(float(trace)))
    what = f'antenna {mount[0]}, {"turning" if yaw_rate else "standing"}, 30 s with noise'
    results.append((n == 5, f'{what}: checkpoints with a pose and a covariance: {n} (need 5)'))
    results.append((n == 5 and worst['z'] <= 0.15, f'{what}: largest height error {worst["z"]:.3f} m (need <= 0.15)'))
    results.append((n == 5 and worst['tilt'] <= 0.03, f'{what}: largest tilt error {worst["tilt"]:.4f} rad (need <= 0.03)'))
    results.append((n == 5 and worst['yaw'] <= 0.05, f'{what}: largest heading error {worst["yaw"]:.4f} rad (need <= 0.05)'))
    results.append((n == 5 and worst['xy_over_sigma'] <= 2.0,
                    f'{what}: largest horizontal error {worst["xy"]:.3f} m, at most {worst["xy_over_sigma"]:.2f} x the sigma the node '
                    f'reports (need <= 2; the heading is not observed, the horizontal lever arm is {lever_xy:.2f} m)'))
    v = rig.values()
    results.append((v.get('gnss_mount', '').startswith('base_link -> gnss xyz'), f'{what}: gnss_mount "{v.get("gnss_mount")}"'))


GYRO_BIAS = np.array([0.003, -0.002, 0.002])             # rad/s, the bias of the review of PR 7
GYRO_BIAS_BIG = np.array([1.0, -1.0, 1.0]) * math.radians(0.5)   # 0.5 deg/s on each axis


def gnss_bias_run(rig, results, mount, yaw_rate, bias, what, checkpoints, reseed_at=None, late_pose=False,
                  prelude=None, cloud=None, max_trace=None, heading_allowed=None):
    """GNSS and IMU only with a constant gyro bias. checkpoints: seconds after the first fix at
    which the pose is read (after streaming `yaw_rate` for 4 s of every 5 s, still for the last
    second). The thresholds are those of the docstring. prelude(rig) runs before the first fix; cloud
    (frame, mount) adds lidar clouds all along; max_trace bounds the position covariance trace at every
    checkpoint; heading_allowed replaces 0.05 rad + bias x time as the heading bound (a lidar corrects it)."""
    rig.noise = NOISE
    rig.gyro_bias = np.array(bias)
    rig.add_static('gnss', mount)
    if late_pose:                                     # /initialpose and the IMU before the first fix
        rig.stream(1.0)
        rig.initial_pose()
        rig.stream(1.0)
    if prelude is not None:
        prelude(rig)
    t_reseed = None
    marks = [time.monotonic()]                        # starts of the filter's covariance: the first fix, each reseed
    worst = {'z': 0.0, 'tilt': 0.0, 'yaw_excess': -1e9, 'xy_over_sigma': 0.0, 'xy': 0.0, 'yaw': 0.0}
    n = 0
    t_nominal = 0.0                                   # seconds streamed since the first fix
    for cp in checkpoints:
        while t_nominal < cp - 1e-6:                  # 5 s cycles: 4 s turning, 1 s still
            if reseed_at is not None and t_reseed is None and t_nominal >= reseed_at - 1e-6:
                rig.initial_pose()
                marks.append(time.monotonic())
                t_reseed = t_nominal
            rig.stream(4.0, gnss=('gnss', mount), cloud=cloud, yaw_rate=yaw_rate)
            rig.stream(1.0, gnss=('gnss', mount), cloud=cloud)
            t_nominal += 5.0
        e = rig.pose_error(1.0)
        trace = rig.values().get('covariance_trace_position', 'n/a')
        if e is None or trace == 'n/a':
            continue
        n += 1
        since = t_nominal - (t_reseed if t_reseed is not None else 0.0)
        xy = math.hypot(e[0], e[1])
        worst['z'] = max(worst['z'], abs(e[2]))
        worst['tilt'] = max(worst['tilt'], math.hypot(e[3], e[4]))
        worst['yaw'] = max(worst['yaw'], abs(e[5]))
        allowed = heading_allowed if heading_allowed is not None else (0.06 if t_reseed is not None else 0.05) + abs(bias[2]) * since
        worst['yaw_excess'] = max(worst['yaw_excess'], abs(e[5]) - allowed)
        worst['trace'] = max(worst.get('trace', 0.0), float(trace))
        worst['xy'] = max(worst['xy'], xy)
        worst['xy_over_sigma'] = max(worst['xy_over_sigma'], xy / math.sqrt(float(trace)))
    results.append((n == len(checkpoints), f'{what}: checkpoints with a pose and a covariance: {n} (need {len(checkpoints)})'))
    ok = n == len(checkpoints)
    # Roll/pitch variance of the published pose 0.25 to 0.6 s after each start: with an antenna off the origin it
    # starts at the accelerometer's accuracy (about 1e-4 rad^2), not at the 0.5 rad^2 of the heading.
    win = [m for t, m in rig.poses if any(mk + 0.25 <= t <= mk + 0.6 for mk in marks)]
    cov = [max(m.pose.covariance[21], m.pose.covariance[28]) for m in win]
    results.append((bool(cov) and max(cov) <= 2e-3,
                    f'{what}: largest roll/pitch variance 0.25 to 0.6 s after a start: '
                    f'{max(cov) if cov else float("nan"):.2e} rad^2 over {len(cov)} poses (need <= 2e-3)'))
    if cloud is None:   # nothing but the IMU and the antenna: the heading is not observed and keeps its 0.5 rad^2
        yv = [m.pose.covariance[35] for m in win]
        results.append((bool(yv) and 0.3 <= min(yv) and max(yv) <= 0.7,
                        f'{what}: heading variance 0.25 to 0.6 s after a start: '
                        f'{min(yv) if yv else float("nan"):.3f} to {max(yv) if yv else float("nan"):.3f} rad^2 (need 0.3 to 0.7)'))
    else:               # with the lidar the heading is observed: its variance must not go back to the 0.5 rad^2 of a start
        yv = [m.pose.covariance[35] for m in win]
        results.append((bool(yv) and max(yv) <= 0.05,
                        f'{what}: heading variance 0.25 to 0.6 s after a start: '
                        f'{max(yv) if yv else float("nan"):.4f} rad^2 (need <= 0.05)'))
    if max_trace is not None:
        tr = [m.pose.covariance[0] + m.pose.covariance[7] + m.pose.covariance[14] for m in win]
        results.append((bool(tr) and max(tr) <= max_trace,
                        f'{what}: position covariance trace 0.25 to 0.6 s after a start: '
                        f'{max(tr) if tr else float("nan"):.4f} m^2 (need <= {max_trace})'))
    results.append((ok and worst['z'] <= 0.15, f'{what}: largest height error {worst["z"]:.3f} m (need <= 0.15)'))
    results.append((ok and worst['tilt'] <= 0.03, f'{what}: largest tilt error {worst["tilt"]:.4f} rad (need <= 0.03)'))
    results.append((ok and worst['yaw_excess'] <= 0.0,
                    f'{what}: largest heading error {worst["yaw"]:.4f} rad, at most {worst["yaw_excess"]:+.4f} rad over the '
                    + (f'allowed {heading_allowed} rad' if heading_allowed is not None else
                       f'allowed 0.05 rad + {abs(bias[2]):.4f} rad/s x time') + ' (need <= 0)'))
    results.append((ok and worst['xy_over_sigma'] <= 3.0,
                    f'{what}: largest horizontal error {worst["xy"]:.3f} m, at most {worst["xy_over_sigma"]:.2f} x the sigma the node '
                    f'reports (need <= 3)'))
    if max_trace is not None:
        results.append((ok and worst.get('trace', 1e9) <= max_trace,
                        f'{what}: largest position covariance trace {worst.get("trace", float("nan")):.4f} m^2 (need <= {max_trace})'))
    v = rig.values()
    results.append((v.get('gnss_mount', '').startswith('base_link -> gnss xyz'), f'{what}: gnss_mount "{v.get("gnss_mount")}"'))


def scenario_slash(rig, results):
    rig.add_static('lidar', LIDAR)
    rig.stream(1.0, imu=('/base_link', None))
    rig.initial_pose()
    rig.stream(7.0, imu=('/base_link', None), cloud=('/lidar', LIDAR), gnss=('/base_link', None))
    check_pose(rig, results, 'frame_id "/base_link" (IMU, fix) and "/lidar" (cloud)')
    check_clean(rig, results, 'leading slashes')
    v = rig.values()
    mounts = [v.get(k, '') for k in ('points_mount', 'imu_mount', 'gnss_mount')]
    results.append((mounts[0].startswith('base_link -> lidar xyz 0.200 0.100 0.500') and
                    mounts[1:] == ['identity (frame base_link)'] * 2 and v.get('ndt_rejected_total') == '0',
                    f'leading slashes: points_mount, imu_mount, gnss_mount {mounts}, ndt_rejected_total {v.get("ndt_rejected_total")}'))


def scenario_missing(rig, results):
    rig.stream(1.0)
    rig.initial_pose()
    t1 = rig.now()
    rig.stream(5.0, cloud=('lidar', LIDAR))
    late = rig.statuses(t1 + 2.0)
    w = [s for s in late if level_of(s) == WARN and 'point clouds not used' in s.message
         and 'base_link -> lidar' in s.message and 'points_frame' in s.message]
    results.append((bool(late) and len(w) == len(late),
                    f'no transform for the cloud frame: {len(w)} of {len(late)} statuses WARN "point clouds not used: '
                    f'no transform base_link -> lidar ..." ("{late[-1].message if late else ""}")'))
    v = rig.values()
    results.append((v.get('points_mount') == 'missing (frame lidar)' and v.get('seconds_since_ndt_correction') == 'never',
                    f'no transform: points_mount "{v.get("points_mount")}", seconds_since_ndt_correction '
                    f'{v.get("seconds_since_ndt_correction")} (need "missing (frame lidar)" and never)'))
    rig.add_static('lidar', LIDAR)
    t2 = rig.now()
    rig.stream(6.0, cloud=('lidar', LIDAR))
    after = rig.statuses(t2 + 3.0)
    stale = [s for s in after if not clean(s)]
    results.append((bool(after) and not stale,
                    f'after the transform is published: {len(stale)} of {len(after)} statuses not clean from 3 s on '
                    f'("{stale[-1].message if stale else "localizing"}")'))
    check_pose(rig, results, 'after the transform is published')


def scenario_empty_frame(rig, results):
    rig.stream(1.0)
    rig.initial_pose()
    t1 = rig.now()
    rig.stream(4.0, cloud=('', None))
    late = rig.statuses(t1 + 2.0)
    w = [s for s in late if level_of(s) == WARN and 'point clouds not used: empty header.frame_id on /points' in s.message
         and 'points_frame' in s.message]
    results.append((bool(late) and len(w) == len(late),
                    f'empty frame_id: {len(w)} of {len(late)} statuses WARN "point clouds not used: empty header.frame_id" '
                    f'("{late[-1].message if late else ""}")'))
    v = rig.values()
    results.append((v.get('points_mount') == 'missing (empty frame_id)' and v.get('seconds_since_ndt_correction') == 'never',
                    f'empty frame_id: points_mount "{v.get("points_mount")}", seconds_since_ndt_correction '
                    f'{v.get("seconds_since_ndt_correction")}'))


def scenario_override(rig, results):
    rig.stream(1.0, imu=('imu_not_on_tf', None))
    rig.initial_pose()
    rig.stream(7.0, imu=('imu_not_on_tf', None), cloud=('not_on_tf', None), gnss=('', None))
    check_pose(rig, results, '*_frame:=base_link, inputs expressed in base_link')
    check_clean(rig, results, '*_frame:=base_link')
    v = rig.values()
    mounts = [v.get(k) for k in ('points_mount', 'imu_mount', 'gnss_mount')]
    results.append((mounts == ['identity (frame base_link)'] * 3 and v.get('ndt_rejected_total') == '0'
                    and v.get('seconds_since_gnss_correction', 'never') != 'never',
                    f'*_frame:=base_link: points_mount, imu_mount, gnss_mount {mounts}, ndt_rejected_total '
                    f'{v.get("ndt_rejected_total")}, seconds_since_gnss_correction {v.get("seconds_since_gnss_correction")}'))


def v_mount(rig):
    return rig.values().get('imu_mount')


def scenario_imu_missing(rig, results):
    # startup_timeout_s is 10 s here and the node has been up for a few seconds at t0:
    # statuses up to t0 + 3 s are inside the grace, those from t0 + 11 s on are past it.
    t0 = rig.t0
    rig.stream(1.0, imu=('imu', IMU))
    rig.initial_pose()
    rig.stream(12.5, imu=('imu', IMU))
    early = [s for t, s in rig.diags if t - t0 <= 3.0 and 'IMU not used' in s.message]
    results.append((bool(early) and all(level_of(s) == WARN for s in early),
                    f'IMU frame without a transform, within the startup grace: {len(early)} statuses name it, levels '
                    f'{sorted({level_of(s) for s in early})} (need WARN = 1)'))
    late = [s for t, s in rig.diags if t - t0 >= 11.0]
    bad = [s for s in late if level_of(s) == ERROR and 'IMU not used: no transform base_link -> imu' in s.message
           and 'imu_frame' in s.message]
    results.append((bool(late) and len(bad) == len(late),
                    f'after the startup grace: {len(bad)} of {len(late)} statuses ERROR "IMU not used: no transform '
                    f'base_link -> imu ..." ("{late[-1].message if late else ""}")'))
    results.append((len(rig.poses) == 0, f'poses published while the IMU is not used: {len(rig.poses)} (need 0)'))
    rig.add_static('imu', IMU)
    t2 = rig.now()
    rig.stream(5.0, imu=('imu', IMU))
    # No cloud is sent in this scenario, so "no accepted NDT correction" (WARN) remains.
    after = rig.statuses(t2 + 3.0)
    stale = [s for s in after if level_of(s) == ERROR or 'IMU not used' in s.message]
    results.append((bool(after) and not stale,
                    f'after the IMU transform is published: {len(stale)} of {len(after)} statuses still ERROR or '
                    f'"IMU not used" from 3 s on ("{after[-1].message if after else ""}")'))
    # /initialpose is IP_OFFSET off and nothing corrects it here: the pose must stay there.
    e = rig.pose_error()
    ok = e is not None and abs(e[0] - IP_OFFSET[0]) <= POS_TOL and abs(e[1] - IP_OFFSET[1]) <= POS_TOL \
        and abs(e[2]) <= POS_TOL and abs(e[3]) <= ANG_TOL and abs(e[4]) <= ANG_TOL
    results.append((ok, 'after the IMU transform is published: pose holds at the initial pose, error '
                        + (f'x {e[0]:+.3f} y {e[1]:+.3f} z {e[2]:+.3f} m, roll {e[3]:+.4f} pitch {e[4]:+.4f} rad '
                           f'(need {IP_OFFSET[0]:+.2f}, {IP_OFFSET[1]:+.2f}, 0 within {POS_TOL} m; 0 within {ANG_TOL} rad)'
                           if e else 'n/a: no pose')))
    # The filter runs and the startup grace is over. The IMU now reports a frame nobody
    # publishes: its samples must be left out, and that is the only error there is.
    t3 = rig.now()
    rig.stream(4.0, imu=('imu_b', IMU))
    during = rig.statuses(t3 + 2.0)
    bad = [s for s in during if level_of(s) == ERROR and s.message.startswith('IMU not used: no transform base_link -> imu_b')]
    results.append((bool(during) and len(bad) == len(during),
                    f'IMU switches to a frame without a transform: {len(bad)} of {len(during)} statuses ERROR starting '
                    f'"IMU not used: no transform base_link -> imu_b" ("{during[-1].message if during else ""}")'))
    fresh = [t for t, _ in rig.poses if t > t3 + 1.0]
    results.append((not fresh, f'poses published while the IMU is on that frame (after 1 s): {len(fresh)} (need 0)'))
    t4 = rig.now()
    rig.stream(4.0, imu=('imu', IMU))
    back = rig.statuses(t4 + 2.0)
    stale = [s for s in back if level_of(s) == ERROR or 'IMU not used' in s.message]
    results.append((bool(back) and not stale and v_mount(rig) == 'base_link -> imu xyz 0.100 0.000 0.200 rpy 0.300 -0.200 1.200',
                    f'back on the first frame: {len(stale)} of {len(back)} statuses still ERROR or "IMU not used"; '
                    f'imu_mount "{v_mount(rig)}"'))


def prelude_ndt(rig):
    """/initialpose, then the IMU and lidar clouds alone: the filter starts before any fix and the lidar corrects it."""
    rig.stream(1.0)
    rig.initial_pose()
    rig.stream(8.0, cloud=('base_link', None))


def prelude_two_poses(rig):
    """/initialpose and clouds, a second /initialpose, then the IMU alone for 1.5 s: nothing has corrected the new start."""
    rig.stream(1.0)
    rig.initial_pose()
    rig.stream(4.0, cloud=('base_link', None))
    rig.initial_pose()
    rig.stream(1.5)


def prelude_pose_first(rig):
    """/initialpose and fixes, no IMU yet: the filter starts at the first fix with no accelerometer sample."""
    rig.initial_pose()
    rig.stream(1.0, imu=None, gnss=('gnss', ANTENNA_A))


def scenario_pose_first_wait(rig, results):
    rig.add_static('gnss', ANTENNA_A)
    rig.initial_pose()
    n0 = len(rig.poses)
    rig.stream(1.0, imu=None, gnss=('gnss', ANTENNA_A))
    n1 = len(rig.poses)
    results.append((n1 == n0, f'antenna A, /initialpose and fixes before the first IMU sample: {n1 - n0} poses in 1 s (need 0)'))
    rig.stream(1.0, gnss=('gnss', ANTENNA_A))
    n2 = len(rig.poses)
    results.append((n2 - n1 >= 50, f'... and {n2 - n1} poses in the next second with the IMU (need >= 50)'))


def scenario_pose_first_base(rig, results):
    rig.initial_pose()
    n0 = len(rig.poses)
    rig.stream(1.0, imu=None, gnss=('base_link', None))
    n1 = len(rig.poses)
    results.append((n1 - n0 >= 3, f'antenna at base_link, /initialpose and fixes before the first IMU sample: {n1 - n0} poses in 1 s (need >= 3)'))


SCENARIOS = {'lidar': scenario_lidar, 'imu': scenario_imu, 'gnss': scenario_gnss, 'missing': scenario_missing,
             'empty_frame': scenario_empty_frame, 'override': scenario_override, 'imu_missing': scenario_imu_missing,
             'gnss_missing': scenario_gnss_missing, 'slash': scenario_slash,
             'gnss_stand_a': lambda r, res: gnss_only_noisy(r, res, ANTENNA_A, 0.0),
             'gnss_stand_b': lambda r, res: gnss_only_noisy(r, res, ANTENNA_B, 0.0),
             'gnss_turn_a': lambda r, res: gnss_only_noisy(r, res, ANTENNA_A, 0.3),
             'gnss_turn_b': lambda r, res: gnss_only_noisy(r, res, ANTENNA_B, 0.3),
             'gnss_bias_stand_a': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias 0.003 -0.002 0.002 rad/s, 45 s', [15, 30, 45]),
             'gnss_bias_turn_b': lambda r, res: gnss_bias_run(r, res, ANTENNA_B, 0.3, GYRO_BIAS, 'antenna B, turning, gyro bias 0.003 -0.002 0.002 rad/s, 45 s', [15, 30, 45]),
             'gnss_bias_big_turn_a': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.3, GYRO_BIAS_BIG, 'antenna A, turning, gyro bias 0.5 deg/s per axis, 45 s', [15, 30, 45]),
             'gnss_reseed': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias, /initialpose at 15 s', [30, 45], reseed_at=15.0),
             'gnss_after_ndt': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias, lidar before the first fix', [15, 30], prelude=prelude_ndt, cloud=('base_link', None), max_trace=0.06, heading_allowed=0.1),
             'gnss_reseed_before_fix': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias, /initialpose twice before the first fix', [15, 30, 45], prelude=prelude_two_poses),
             'gnss_pose_first': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias, /initialpose and fixes before the first IMU sample', [15, 30, 45], prelude=prelude_pose_first),
             'gnss_pose_first_wait': scenario_pose_first_wait,
             'gnss_pose_first_base': scenario_pose_first_base,
             'gnss_late_pose': lambda r, res: gnss_bias_run(r, res, ANTENNA_A, 0.0, GYRO_BIAS, 'antenna A, standing, gyro bias, /initialpose before the first fix', [15, 30, 45], late_pose=True)}
SCENARIO_ARGS = {'imu': [f'initial_yaw:={BASE_YAW}'], 'gnss': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_stand_a': [f'initial_yaw:={BASE_YAW}'], 'gnss_stand_b': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_turn_a': [f'initial_yaw:={BASE_YAW}'], 'gnss_turn_b': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_bias_stand_a': [f'initial_yaw:={BASE_YAW}'], 'gnss_bias_turn_b': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_bias_big_turn_a': [f'initial_yaw:={BASE_YAW}'], 'gnss_reseed': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_late_pose': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_pose_first': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_pose_first_wait': [f'initial_yaw:={BASE_YAW}'], 'gnss_pose_first_base': [f'initial_yaw:={BASE_YAW}'],
                 'gnss_after_ndt': [f'initial_yaw:={BASE_YAW}'], 'gnss_reseed_before_fix': [f'initial_yaw:={BASE_YAW}'],
                 'override': ['points_frame:=base_link', 'imu_frame:=base_link', 'gnss_frame:=base_link'],
                 'imu_missing': ['startup_timeout_s:=10.0']}


def run_scenario(name, params, pcd):
    args = ['ros2', 'run', 'prism_loc_fusion_ros', 'fusion_node_main', '--ros-args', '--params-file', params,
            '-p', f'map_pcd_path:={pcd}', '-p', 'use_datum:=true', '-p', f'datum_lat:={DATUM[0]}',
            '-p', f'datum_lon:={DATUM[1]}', '-p', f'datum_alt:={DATUM[2]}', '-p', 'imu_queue_depth:=200']
    for a in SCENARIO_ARGS.get(name, []):
        args += ['-p', a]
    log = open(f'fusion_extrinsic_{name}.log', 'w')
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node(f'prism_loc_fusion_extrinsic_check_{name}')
    rig = Rig(node)
    results = []
    try:
        end = time.monotonic() + 30
        while (rig.imu_pub.get_subscription_count() == 0 or rig.pts_pub.get_subscription_count() == 0) \
                and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rig.imu_pub.get_subscription_count() == 0:
            results.append((False, 'the node did not subscribe to /imu'))
        else:
            rig.t0 = time.monotonic()
            SCENARIOS[name](rig, results)
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
    ap.add_argument('params', help='prism_loc_fusion_ros/params/fusion3d.yaml')
    ap.add_argument('--scenario', choices=['all'] + list(SCENARIOS), default='all')
    ap.add_argument('--tolerate-imu-gaps', action='store_true',
                    help='where the status must be OK, also accept a WARN about IMU gaps only')
    args = ap.parse_args()
    global TOLERATE_IMU_GAPS
    TOLERATE_IMU_GAPS = args.tolerate_imu_gaps
    pcd = os.path.abspath('fusion_extrinsic_map.pcd')
    write_pcd(pcd, surfaces(np.random.default_rng(1), MAP_POINTS))
    failed = False
    for name in (list(SCENARIOS) if args.scenario == 'all' else [args.scenario]):
        results = run_scenario(name, os.path.abspath(args.params), pcd)
        print(f'[{name}]')
        for ok, text in results:
            print(f'  {"PASS" if ok else "FAIL"}: {text}')
        if not results or not all(ok for ok, _ in results):
            failed = True
            print(f'  (node log: fusion_extrinsic_{name}.log)')
    if failed:
        sys.exit('FAIL: fusion3d sensor mounts')
    print('PASS: fusion3d sensor mounts')


if __name__ == '__main__':
    main()
