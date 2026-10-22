#!/usr/bin/env python3
"""Map and cloud sequence for the offline ndt3d replay (experiments/ndt3d_replay.cpp).

World, trajectory, odometry error model and lidar geometry are those of scripts/synthetic_e2e.py
(imported from there, so the two cannot drift apart); this file adds the perturbations of the
replay cases (map noise, moving columns, added or removed boxes, range noise, a kidnap, other
initial poses, pitched lidar, cropped prior map). Needs the ROS Python environment, because
scripts/synthetic_e2e.py imports rclpy at module level.

    ndt3d_replay_gen.py OUT_DIR [--seed N] [case options ...]    # writes OUT_DIR/map.bin, frames.bin

Binary layout: map.bin = int32 n, n x 3 float64 (map points). frames.bin = 3 float64 (lidar x, y,
yaw on base_link), then per frame: 7 float64 (t, odom x, y, yaw, truth x, y, yaw), int32 initialpose
flag, 3 float64 (initialpose x, y, yaw), int32 n points, n x 3 float32 (cloud in the lidar frame).
"""
import argparse
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import synthetic_e2e as pe  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('out')
ap.add_argument('--seconds', type=float, default=75.0)
ap.add_argument('--seed', type=int, default=7)
ap.add_argument('--speed', type=float, default=1.0)
ap.add_argument('--map-noise', type=float, default=0.0)
ap.add_argument('--kidnap', type=float, default=-1.0)
ap.add_argument('--kidnap-shift', type=float, default=25.0)
ap.add_argument('--init-offset', type=float, nargs=3, default=[0.3, -0.2, 0.1])
ap.add_argument('--map-crop-x', type=float, default=None, help='prior map keeps only points with x < this')
ap.add_argument('--add-boxes', default='', help='x0,y0,x1,y1;... present in the scans, not in the map')
ap.add_argument('--drop-boxes', default='', help='indices of world boxes absent from the scans (still in the map)')
ap.add_argument('--dyn', type=int, default=0, help='moving 0.5 m columns in the scans')
ap.add_argument('--ring-step', type=int, default=1)
ap.add_argument('--az', type=int, default=360)
ap.add_argument('--pitch', type=float, default=0.0, help='true lidar pitch (rad), ignored by ndt3d')
ap.add_argument('--range-noise', type=float, default=0.01)
ap.add_argument('--stand', type=float, default=0.0, help='robot stands until this time')
ap.add_argument('--ip2', type=float, nargs=4, default=None, help='second /initialpose: t dx dy dth (relative to truth)')
args = ap.parse_args()

RES, W, H = pe.RES, pe.W, pe.H
GROUND_Z, WALL_H = pe.GROUND_Z, pe.WALL_H
LX, LY, LZ = pe.LIDAR_XYZ
LYAW = pe.LIDAR_YAW
BOXES = [(3, 3, 4, 6), (12, 2, 13, 4), (7, 9, 11, 10), (15, 8, 16, 13), (2, 11, 3, 12)]
ELEV = np.radians(np.arange(-15.0, 15.1, 2.0))[::args.ring_step]
N_AZ = args.az
AZ = np.linspace(-math.pi, math.pi, N_AZ, endpoint=False)
RANGES = pe.RANGES


def grid_with(boxes):
    g = np.zeros((H, W), dtype=np.int8)
    g[0:3, :] = 100; g[-3:, :] = 100; g[:, 0:3] = 100; g[:, -3:] = 100
    for (x0, y0, x1, y1) in boxes:
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100
    return g


MAP_GRID = grid_with(BOXES)
assert (MAP_GRID == pe.GRID).all()
drop = [int(i) for i in args.drop_boxes.split(',') if i != '']
extra = [tuple(float(v) for v in b.split(',')) for b in args.add_boxes.split(';') if b]
SCAN_GRID = grid_with([b for i, b in enumerate(BOXES) if i not in drop] + extra)


def raycast(grid, x, y, th):
    a = th + AZ
    px = x + np.outer(np.cos(a), RANGES)
    py = y + np.outer(np.sin(a), RANGES)
    ix = (px / RES).astype(int)
    iy = (py / RES).astype(int)
    inside = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    occ = np.ones_like(inside)
    occ[inside] = grid[iy[inside], ix[inside]] > 50
    first = np.where(occ.any(1), occ.argmax(1), len(RANGES) - 1)
    return RANGES[first]


def map_points():
    occ = MAP_GRID > 50
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
    p = np.array(pts, dtype=np.float64)
    if args.map_noise > 0:
        p = p + np.random.default_rng(11).normal(0.0, args.map_noise, p.shape)
    if args.map_crop_x is not None:
        p = p[p[:, 0] < args.map_crop_x]
    return np.round(p, 3).astype(np.float32).astype(np.float64)   # as the ASCII PCD -> pcl float path


cp, sp = math.cos(args.pitch), math.sin(args.pitch)
R_PITCH = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])   # lidar axes in the level frame


def scan3d(grid, x, y, th, rng):
    lx = x + LX * math.cos(th) - LY * math.sin(th)
    ly = y + LX * math.sin(th) + LY * math.cos(th)
    sensor_h = LZ - GROUND_Z
    rh = raycast(grid, lx, ly, th)
    hit_wall = rh < RANGES[-1] - 1e-6
    out = []
    for e in ELEV:
        t = math.tan(e)
        z_at_wall = rh * t
        r = np.full(N_AZ, np.nan)
        zrel = np.full(N_AZ, np.nan)
        wall_ok = hit_wall & (z_at_wall > -sensor_h) & (z_at_wall < WALL_H - sensor_h)
        r[wall_ok], zrel[wall_ok] = rh[wall_ok], z_at_wall[wall_ok]
        if e < 0:
            rg = sensor_h / -t
            ground = (~hit_wall | (z_at_wall <= -sensor_h)) & (rg < 12.0)
            r[ground], zrel[ground] = rg, -sensor_h
        ok = ~np.isnan(r)
        slant = np.hypot(r[ok], zrel[ok])
        k = (slant + rng.normal(0.0, args.range_noise, slant.shape)) / slant
        a = AZ[ok] - LYAW
        out.append(np.stack([r[ok] * np.cos(a) * k, r[ok] * np.sin(a) * k, zrel[ok] * k], axis=1))
    pts = np.concatenate(out)
    if args.pitch:
        pts = pts @ R_PITCH        # level-frame coords -> tilted lidar frame coords (R^T p)
    return pts.astype(np.float32)


def truth(t):
    return pe.truth(args.speed * max(0.0, t - args.stand))


rng = np.random.default_rng(args.seed)
drng = np.random.default_rng(1000 + args.seed)
dyn = [(drng.uniform(4, 16), drng.uniform(3, 12), drng.uniform(0.5, 2.0), drng.uniform(0.2, 0.6) * drng.choice([-1, 1]),
        drng.uniform(0, 6.28)) for _ in range(args.dyn)]

os.makedirs(args.out, exist_ok=True)
mp = map_points()
with open(os.path.join(args.out, 'map.bin'), 'wb') as f:
    f.write(struct.pack('<i', len(mp)))
    f.write(mp.astype('<f8').tobytes())

od = pe.Odom()
npts = []
with open(os.path.join(args.out, 'frames.bin'), 'wb') as f:
    f.write(struct.pack('<3d', LX, LY, LYAW))
    tl, sent_ip, sent_ip2 = 0.0, False, False
    n = int(args.seconds * pe.RATE)
    for k in range(n):
        t = k / pe.RATE
        x, y, th = truth(t)
        ox, oy, oth = od.step(x, y, th, t - tl)
        tl = t
        ipf, ip = 0, (0.0, 0.0, 0.0)
        if t > 1.0 and not sent_ip:
            ipf, ip, sent_ip = 1, (x + args.init_offset[0], y + args.init_offset[1], th + args.init_offset[2]), True
        if args.kidnap >= 0 and t >= args.kidnap:
            x, y, th = truth(t + args.kidnap_shift)
        if args.ip2 is not None and t >= args.ip2[0] and not sent_ip2:
            ipf, ip, sent_ip2 = 1, (x + args.ip2[1], y + args.ip2[2], th + args.ip2[3]), True
        grid = SCAN_GRID
        if dyn:
            grid = SCAN_GRID.copy()
            for (cx, cy, r, w, ph) in dyn:
                px, py = cx + r * math.cos(w * t + ph), cy + r * math.sin(w * t + ph)
                if math.hypot(px - x, py - y) < 0.9:
                    continue
                grid[max(0, int((py - 0.25) / RES)):int((py + 0.25) / RES), max(0, int((px - 0.25) / RES)):int((px + 0.25) / RES)] = 100
        pts = scan3d(grid, x, y, th, rng)
        npts.append(len(pts))
        f.write(struct.pack('<7d', t, ox, oy, oth, x, y, th))
        f.write(struct.pack('<i', ipf))
        f.write(struct.pack('<3d', *ip))
        f.write(struct.pack('<i', len(pts)))
        f.write(pts.astype('<f4').tobytes())
print(f'{args.out}: map {len(mp)} pts, {n} frames, cloud pts min {min(npts)} max {max(npts)}')
