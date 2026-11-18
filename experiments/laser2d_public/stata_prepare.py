#!/usr/bin/env python3
"""stata_prepare.py SCANS POSES OUT_MAP.bin [--res 0.05] [--min-hits 5] [--stride 3] [--max-range 25]

Builds the prior occupancy map for mit_stata_laser2d from one MIT Stata Center session: every
STRIDE-th scan is placed at its ground-truth laser pose and its valid endpoints are counted per
cell (no ray casting). A cell with at least MIN_HITS endpoints is occupied (100), every other cell
is free (0). Free and unknown are the same for the likelihood field and for BBS (only occupied
cells matter), so no separate unknown state is written.

Map file: int32 width, int32 height, float64 resolution, origin_x, origin_y, then width*height int8
(row-major, row 0 at origin_y). Needs numpy.
"""
import argparse
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('scans'); ap.add_argument('poses'); ap.add_argument('out')
ap.add_argument('--res', type=float, default=0.05)
ap.add_argument('--min-hits', type=int, default=5)
ap.add_argument('--stride', type=int, default=3)
ap.add_argument('--max-range', type=float, default=25.0)
ap.add_argument('--margin', type=float, default=3.0)
a = ap.parse_args()

P = np.loadtxt(a.poses, delimiter=',')
pts = []
with open(a.scans) as f:
    for i, line in enumerate(f):
        if i % a.stride:
            continue
        t = line.rstrip().split(',')
        n = int(t[3])
        if n <= 0:
            continue
        r = np.asarray(t[4:4 + n], dtype=np.float64)
        ang = float(t[1]) + float(t[2]) * np.arange(n)
        ok = (r > 0.1) & (r < a.max_range)
        x, y, th = P[i, 1:4]
        c, s = np.cos(th), np.sin(th)
        px, py = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        pts.append(np.c_[x + c * px - s * py, y + s * px + c * py].astype(np.float32))
pts = np.vstack(pts)
lo = pts.min(0) - a.margin
hi = pts.max(0) + a.margin
W = int(np.ceil((hi[0] - lo[0]) / a.res)); H = int(np.ceil((hi[1] - lo[1]) / a.res))
ix = ((pts[:, 0] - lo[0]) / a.res).astype(np.int64); iy = ((pts[:, 1] - lo[1]) / a.res).astype(np.int64)
cnt = np.bincount(iy * W + ix, minlength=W * H).reshape(H, W)
grid = np.where(cnt >= a.min_hits, 100, 0).astype(np.int8)
with open(a.out, 'wb') as f:
    np.array([W, H], dtype=np.int32).tofile(f)
    np.array([a.res, lo[0], lo[1]], dtype=np.float64).tofile(f)
    grid.tofile(f)
print('map %dx%d at %.3f m, origin (%.2f, %.2f), %d occupied cells, from %d endpoints' % (W, H, a.res, lo[0], lo[1], int((grid == 100).sum()), len(pts)))
