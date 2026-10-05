#!/usr/bin/env python3
"""Check fusion3d's /diagnostics around the map->base_link fallback, over real topics.

Starts `fusion_node_main` with a params YAML and a PCD map, feeds a level IMU at 100 Hz
and one /initialpose, and checks the "prism_loc_fusion: fusion" status:
  1. no odometry at all (a robot that runs on the fallback by design): OK with
     map_to_base_fallback_active=true, and map->base_link is broadcast
  2. an odom->base_link publisher appears: WARN naming the two parents of base_link
  3. that publisher stops again: WARN that odom->base_link is old (the buffer still
     holds it) or lost

    fusion_fallback_check.py PARAMS_YAML MAP_PCD
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

STATUS = 'prism_loc_fusion: fusion'


def level_of(st):
    return st.level[0] if isinstance(st.level, bytes) else st.level


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    params, pcd = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    log = open('fusion_fallback.log', 'w')
    proc = subprocess.Popen(['ros2', 'run', 'prism_loc_fusion_ros', 'fusion_node_main', '--ros-args',
                             '--params-file', params, '-p', f'map_pcd_path:={pcd}'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    n = rclpy.create_node('prism_loc_fusion_fallback_check')
    imu_pub = n.create_publisher(Imu, '/imu', qos_profile_sensor_data)
    ip_pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    tfb = TransformBroadcaster(n)
    diags, tf_children = [], set()
    n.create_subscription(DiagnosticArray, '/diagnostics',
                          lambda m: diags.extend((time.monotonic(), s) for s in m.status if s.name == STATUS), 10)
    n.create_subscription(TFMessage, '/tf',
                          lambda m: tf_children.update((t.header.frame_id, t.child_frame_id) for t in m.transforms
                                                       if t.header.frame_id == 'map'), 100)
    results = []

    def stream(seconds, odom):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = n.get_clock().now().to_msg()
            m = Imu()
            m.header.stamp = now
            m.header.frame_id = 'base_link'
            m.linear_acceleration.z = 9.81
            m.orientation.w = 1.0
            imu_pub.publish(m)
            if odom:
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
                t.transform.rotation.w = 1.0
                tfb.sendTransform(t)
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.01)

    def latest_after(t0):
        later = [s for t, s in diags if t > t0]
        return later[-1] if later else None

    try:
        end = time.monotonic() + 30
        while imu_pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        stream(1.0, odom=False)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = 'map'
        ip.header.stamp = n.get_clock().now().to_msg()
        ip.pose.pose.orientation.w = 1.0
        ip_pub.publish(ip)
        t1 = time.monotonic()
        stream(5.0, odom=False)
        st = latest_after(t1 + 2.0)
        vals = {kv.key: kv.value for kv in st.values} if st else {}
        results.append((st is not None and level_of(st) == 0 and vals.get('map_to_base_fallback_active') == 'true',
                        f'no odometry: level {level_of(st) if st else None}, '
                        f'map_to_base_fallback_active={vals.get("map_to_base_fallback_active")} '
                        f'("{st.message if st else ""}"; need 0 and true)'))
        results.append((('map', 'base_link') in tf_children, f'map->base_link broadcast: {sorted(tf_children)}'))
        t2 = time.monotonic()
        stream(3.0, odom=True)
        w = [s for t, s in diags if t > t2 and level_of(s) == 1 and 'two parents' in s.message]
        results.append((bool(w), 'odometry appears next to the fallback: WARN about two parents'
                        + ('' if w else ' missing')))
        t3 = time.monotonic()
        stream(3.0, odom=False)
        w = [s for t, s in diags if t > t3 and level_of(s) == 1
             and (' old ' in s.message or 'lost' in s.message)]
        results.append((bool(w), 'odometry stops after being seen: WARN "old"/"lost"'
                        + (f' ("{w[0].message}")' if w else ' missing')))
    finally:
        n.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()
    for ok, text in results:
        print(f'  {"PASS" if ok else "FAIL"}: {text}')
    if not results or not all(ok for ok, _ in results):
        sys.exit('FAIL: fusion3d fallback diagnostics (node log: fusion_fallback.log)')
    print('PASS: fusion3d fallback diagnostics')


if __name__ == '__main__':
    main()
