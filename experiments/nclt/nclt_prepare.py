#!/usr/bin/env python3
"""Prepare the public NCLT dataset (http://robots.engin.umich.edu/nclt/) for nclt_ndt3d.

NCLT conventions (devkit `project_vel_to_cam.py`, `read_vel_sync.py`): the points of
`velodyne_sync/<utime>.bin` are in the robot body frame (x forward, y right, z down), stored as
3 x uint16 (scale 0.005 m, offset -100 m) + uint8 intensity + uint8 laser per point. Ground
truth and wheel odometry rows are `utime, x, y, z, roll, pitch, heading` of the body in a
z-down frame, rotation R = Rz(h) Ry(p) Rx(r). Everything is converted here to a z-up frame
with F = diag(1, -1, -1): p_up = F p, R_up = F R F.

  nclt_prepare.py map   SESSION_DIR GT_CSV OUT_DIR [--stride 5] [--voxel 0.2] [--range 60]
      map.bin: float32 x y z (z-up world frame): every stride-th scan placed with the ground
      truth, range-cropped, voxel-downsampled.
  nclt_prepare.py query SESSION_DIR GT_CSV ODOM_CSV OUT_DIR [--voxel 0.2] [--range 60]
      scans.bin: per scan int64 utime, int32 n, n x float32 x y z (body frame, z-up),
      voxel-downsampled; odom.csv and gt.csv: utime x y z roll pitch yaw (z-up).
SESSION_DIR holds velodyne_sync/. Rows with NaN are dropped."""
import argparse
import glob
import os
import struct
import sys

import numpy as np

F = np.diag([1.0, -1.0, -1.0])


def rot(r, p, h):
    sr, cr, sp, cp, sh, ch = np.sin(r), np.cos(r), np.sin(p), np.cos(p), np.sin(h), np.cos(h)
    return np.array([[ch * cp, -sh * cr + ch * sp * sr, sh * sr + ch * sp * cr],
                     [sh * cp, ch * cr + sh * sp * sr, -ch * sr + sh * sp * cr],
                     [-sp, cp * sr, cp * cr]])


def rpy(R):
    return np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(np.clip(R[2, 0], -1, 1)), np.arctan2(R[1, 0], R[0, 0])


def load_poses(path):
    a = np.genfromtxt(path, delimiter=',')
    a = a[np.all(np.isfinite(a), axis=1)]
    a = a[np.argsort(a[:, 0])]
    out = np.empty_like(a)
    for k, row in enumerate(a):
        R = F @ rot(*row[4:7]) @ F
        out[k, 0] = row[0]
        out[k, 1:4] = F @ row[1:4]
        out[k, 4:7] = rpy(R)
    return out


def pose_at(poses, t):
    """z-up pose (p, R) at utime t, linear in position and nearest in attitude; None outside."""
    i = np.searchsorted(poses[:, 0], t)
    if i <= 0 or i >= len(poses) or poses[i, 0] - poses[i - 1, 0] > 0.5e6:
        return None
    a, b = poses[i - 1], poses[i]
    u = (t - a[0]) / (b[0] - a[0])
    q = a if u < 0.5 else b
    return (1 - u) * a[1:4] + u * b[1:4], rot(*q[4:7])


def read_sync(path, max_range):
    raw = np.fromfile(path, dtype=np.uint8)
    n = len(raw) // 8
    raw = raw[:n * 8].reshape(n, 8)
    xyz = raw[:, :6].copy().view('<u2').reshape(n, 3).astype(np.float64) * 0.005 - 100.0
    r = np.linalg.norm(xyz, axis=1)
    return (xyz[(r > 1.5) & (r < max_range)] @ F.T)  # body frame, z-up


def voxel(points, size):
    """One point (the first) per cubic cell of edge `size`."""
    if len(points) == 0:
        return points
    k = np.floor(points / size).astype(np.int64) + (1 << 20)
    key = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    _, idx = np.unique(key, return_index=True)
    return points[np.sort(idx)]


def sync_files(session):
    files = sorted(glob.glob(os.path.join(session, 'velodyne_sync', '*.bin')),
                   key=lambda f: int(os.path.basename(f)[:-4]))
    if not files:
        sys.exit('no velodyne_sync/*.bin under ' + session)
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('role', choices=['map', 'query'])
    ap.add_argument('session')
    ap.add_argument('gt')
    ap.add_argument('rest', nargs='+')
    ap.add_argument('--stride', type=int, default=5)
    ap.add_argument('--voxel', type=float, default=0.2)
    ap.add_argument('--range', type=float, default=60.0)
    a = ap.parse_args()
    gt = load_poses(a.gt)
    files = sync_files(a.session)
    if a.role == 'map':
        out = a.rest[0]
        os.makedirs(out, exist_ok=True)
        chunks, used = [], 0
        acc = np.zeros((0, 3))
        for k, f in enumerate(files[::a.stride]):
            t = int(os.path.basename(f)[:-4])
            ps = pose_at(gt, t)
            if ps is None:
                continue
            p, R = ps
            acc = np.vstack([acc, read_sync(f, a.range) @ R.T + p])
            used += 1
            if len(acc) > 4_000_000:
                chunks.append(voxel(acc, a.voxel))
                acc = np.zeros((0, 3))
        chunks.append(voxel(acc, a.voxel))
        m = voxel(np.vstack(chunks), a.voxel).astype(np.float32)
        m.tofile(os.path.join(out, 'map.bin'))
        print(f'map: {used} scans, {len(m)} points (voxel {a.voxel} m) -> {out}/map.bin')
    else:
        odom_csv, out = a.rest[0], a.rest[1]
        os.makedirs(out, exist_ok=True)
        odom = load_poses(odom_csv)
        np.savetxt(os.path.join(out, 'odom.csv'), odom, fmt='%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f')
        np.savetxt(os.path.join(out, 'gt.csv'), gt, fmt='%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f')
        n_scans = 0
        with open(os.path.join(out, 'scans.bin'), 'wb') as o:
            for f in files:
                t = int(os.path.basename(f)[:-4])
                pts = voxel(read_sync(f, a.range), a.voxel).astype(np.float32)
                o.write(struct.pack('<qi', t, len(pts)))
                o.write(pts.tobytes())
                n_scans += 1
        print(f'query: {n_scans} scans, {len(odom)} odometry rows, {len(gt)} ground-truth rows -> {out}')


if __name__ == '__main__':
    main()
