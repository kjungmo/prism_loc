#!/usr/bin/env python3
"""amcl_run.py: one nav2_amcl run, driven offline in lock step (INIT_PROTOCOL_AMCL.md).

Run inside the ROS 2 Humble env, e.g.
  micromamba run -n ros2_humble python3 amcl_run.py --spec SEG.npz --map SEG.yaml --i0 0 --i1 900 \
      --init pose --pose X Y YAW --cov SX SY SA --out RUN.csv --domain 11

SEG.npz: t (N, int64 us), odom (N, 3: x y yaw), gt (N, 3), ranges (N, B float32; inf = no return),
amin, ainc, rmin, rmax (scalars). The driver starts map_server + amcl + lifecycle_manager, then for
scan i in [i0, i1) publishes /clock, TF odom->base_link, /scan (frame base_link) and waits for the
map->odom TF that AMCL publishes once per processed scan. Output CSV: utime, x, y, yaw (map->odom
(+) odom->base), gt_x, gt_y, gt_yaw, err_xy, updated (an /amcl_pose arrived), timeout.
--init pose: /initialpose = (X, Y, YAW) with covariance (SX^2, SY^2, SA^2).
--init global: the reinitialize_global_localization service (uniform particles over free cells).
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--spec', required=True)
ap.add_argument('--map', required=True)
ap.add_argument('--i0', type=int, default=0)
ap.add_argument('--i1', type=int, default=-1)
ap.add_argument('--init', choices=['pose', 'global'], required=True)
ap.add_argument('--pose', type=float, nargs=3)
ap.add_argument('--cov', type=float, nargs=3)
ap.add_argument('--out', required=True)
ap.add_argument('--domain', type=int, default=0)
ap.add_argument('--min_particles', type=int, default=500)
ap.add_argument('--max_particles', type=int, default=2000)
ap.add_argument('--wait_s', type=float, default=2.0)
ap.add_argument('--keep_log', action='store_true')
args = ap.parse_args()
os.environ['ROS_DOMAIN_ID'] = str(args.domain)
os.environ.pop('ROS_LOCALHOST_ONLY', None)

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_srvs.srv import Empty, Trigger  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

spec = np.load(args.spec)
T, ODOM, GT, RNG = spec['t'], spec['odom'], spec['gt'], spec['ranges']
AMIN, AINC, RMIN, RMAX = float(spec['amin']), float(spec['ainc']), float(spec['rmin']), float(spec['rmax'])
i0, i1 = args.i0, (args.i1 if args.i1 > 0 else len(T))


def stamp(us):
    return Time(sec=int(us // 1_000_000), nanosec=int((us % 1_000_000) * 1000))


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def q_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


# ---- parameters (INIT_PROTOCOL_AMCL.md); everything not listed is the Nav2 Humble default ----
params = f"""
amcl:
  ros__parameters:
    use_sim_time: true
    min_particles: {args.min_particles}
    max_particles: {args.max_particles}
    update_min_d: 0.2
    update_min_a: 0.2
    max_beams: 60
    laser_model_type: "likelihood_field"
    sigma_hit: 0.2
    z_hit: 0.5
    z_rand: 0.5
    laser_likelihood_max_dist: 2.0
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    scan_topic: scan
    set_initial_pose: false
    tf_broadcast: true
    transform_tolerance: 1.0
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "{os.path.abspath(args.map)}"
lifecycle_manager_loc:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names: ["map_server", "amcl"]
    bond_timeout: 0.0
"""
tmpd = tempfile.mkdtemp(prefix='amcl_run_')
pfile = os.path.join(tmpd, 'params.yaml')
open(pfile, 'w').write(params)
logf = open(os.path.join(tmpd, 'nodes.log'), 'w')
procs = []


def spawn(pkg, exe, *extra):
    p = subprocess.Popen(['ros2', 'run', pkg, exe, '--ros-args', '--params-file', pfile, *extra],
                         stdout=logf, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    procs.append(p)


def cleanup():
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass
    time.sleep(1.0)
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass


class Drv(Node):
    def __init__(self):
        super().__init__('amcl_lockstep_driver')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.n_tf = 0
        self.map_odom = None
        self.n_pose = 0
        self.cv = threading.Condition()
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.active_cli = self.create_client(Trigger, '/lifecycle_manager_loc/is_active')
        self.glob_cli = self.create_client(Empty, '/reinitialize_global_localization')

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                with self.cv:
                    self.map_odom = (tr.transform.translation.x, tr.transform.translation.y, q_yaw(tr.transform.rotation))
                    self.n_tf += 1
                    self.cv.notify_all()

    def on_pose(self, msg):
        self.n_pose += 1

    def clock(self, us):
        self.clock_pub.publish(Clock(clock=stamp(us)))


def main():
    rclpy.init()
    d = Drv()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    ex.add_node(d)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        spawn('nav2_map_server', 'map_server')
        spawn('nav2_amcl', 'amcl')
        spawn('nav2_lifecycle_manager', 'lifecycle_manager', '-r', '__node:=lifecycle_manager_loc')
        t_us = int(T[i0])
        # clock runs from start-up on (a fixed time before the first scan) until the nodes are active
        t_boot = t_us - 5_000_000
        t0 = time.time()
        active = False
        while time.time() - t0 < 90:
            d.clock(t_boot)
            if d.active_cli.service_is_ready():
                f = d.active_cli.call_async(Trigger.Request())
                t1 = time.time()
                while not f.done() and time.time() - t1 < 2:
                    time.sleep(0.02)
                if f.done() and f.result() is not None and f.result().success:
                    active = True
                    break
            time.sleep(0.3)
        if not active:
            raise RuntimeError('amcl did not become active: see ' + os.path.join(tmpd, 'nodes.log'))
        # subscribers matched?
        t1 = time.time()
        while d.init_pub.get_subscription_count() < 1 or d.scan_pub.get_subscription_count() < 1:
            d.clock(t_boot)
            if time.time() - t1 > 20:
                raise RuntimeError('amcl subscriptions missing')
            time.sleep(0.1)
        time.sleep(1.0)

        def pub_tf(i, us):
            tr = TransformStamped()
            tr.header.stamp = stamp(us)
            tr.header.frame_id = 'odom'
            tr.child_frame_id = 'base_link'
            tr.transform.translation.x, tr.transform.translation.y = float(ODOM[i, 0]), float(ODOM[i, 1])
            qx, qy, qz, qw = yaw_q(float(ODOM[i, 2]))
            tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = qx, qy, qz, qw
            d.tf_pub.publish(TFMessage(transforms=[tr]))

        # the odom->base TF at the start time must exist before the initial pose is handled
        d.clock(t_us)
        pub_tf(i0, t_us)
        time.sleep(0.3)
        if args.init == 'pose':
            m = PoseWithCovarianceStamped()
            m.header.stamp = stamp(t_us)
            m.header.frame_id = 'map'
            x, y, yaw = args.pose
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            qx, qy, qz, qw = yaw_q(yaw)
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
            cov = [0.0] * 36
            cov[0], cov[7], cov[35] = args.cov[0] ** 2, args.cov[1] ** 2, args.cov[2] ** 2
            m.pose.covariance = cov
            d.init_pub.publish(m)
            time.sleep(0.5)
        else:
            f = None
            t1 = time.time()
            while not d.glob_cli.service_is_ready() and time.time() - t1 < 10:
                time.sleep(0.1)
            f = d.glob_cli.call_async(Empty.Request())
            t1 = time.time()
            while not f.done() and time.time() - t1 < 30:
                time.sleep(0.05)
            if not f.done():
                raise RuntimeError('global localization service did not answer')
            time.sleep(0.3)

        out = open(args.out, 'w')
        out.write('utime,x,y,yaw,gt_x,gt_y,gt_yaw,err_xy,updated,timeout\n')
        n_timeout = 0
        ranges_clean = None
        for i in range(i0, i1):
            us = int(T[i])
            r = RNG[i].astype(np.float32).copy()
            r[~np.isfinite(r)] = RMAX
            sc = LaserScan()
            sc.header.stamp = stamp(us)
            sc.header.frame_id = 'base_link'
            sc.angle_min, sc.angle_max = AMIN, AMIN + AINC * (len(r) - 1)
            sc.angle_increment = AINC
            sc.range_min, sc.range_max = RMIN, RMAX
            sc.ranges = r.tolist()
            with d.cv:
                n_before, p_before = d.n_tf, d.n_pose
            d.clock(us)
            pub_tf(i, us)
            d.scan_pub.publish(sc)
            to = 0
            with d.cv:
                if not d.cv.wait_for(lambda: d.n_tf > n_before, timeout=args.wait_s):
                    to = 1
                    n_timeout += 1
                mo = d.map_odom
            time.sleep(0.002)
            upd = 1 if d.n_pose > p_before else 0
            if mo is None:
                out.write('%d,nan,nan,nan,%.4f,%.4f,%.5f,nan,0,%d\n' % (us, GT[i, 0], GT[i, 1], GT[i, 2], to))
                continue
            e = compose(mo, (ODOM[i, 0], ODOM[i, 1], ODOM[i, 2]))
            g = GT[i]
            if np.isfinite(g[0]):
                out.write('%d,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%d,%d\n' % (
                    us, e[0], e[1], e[2], g[0], g[1], g[2], math.hypot(e[0] - g[0], e[1] - g[1]), upd, to))
        out.close()
        print('scans %d timeouts %d amcl_pose %d tf %d' % (i1 - i0, n_timeout, d.n_pose, d.n_tf), file=sys.stderr)
    finally:
        cleanup()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if not args.keep_log:
            try:
                import shutil
                shutil.rmtree(tmpd)
            except Exception:
                pass


main()
