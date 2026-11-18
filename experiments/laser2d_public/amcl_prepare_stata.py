#!/usr/bin/env python3
"""amcl_prepare_stata.py DATA_DIR WORK_DIR: MIT Stata inputs of amcl_run.py.
Needs the output of `mit_stata_laser2d dump` (WORK_DIR/q{l,s}_*) for the long (ql) and short (qs) query
sessions of run_stata.sh. Writes WORK_DIR/{ql,qs}_s<seed>.npz (scans, synthetic odometry of that seed, truth)."""
import os, sys
import numpy as np

data, work = sys.argv[1], sys.argv[2]
S = '_part1_floor2.gt.laser'
for tag, sess in (('ql', '2012-04-02-10-54-41'), ('qs', '2012-04-04-07-15-50')):
    ts, rs, gts, amin, ainc = [], [], [], None, None
    with open(os.path.join(data, sess + S + '.poses')) as fp, open(os.path.join(data, sess + S + '.scans')) as fs:
        for a, b in zip(fp, fs):
            pa = a.split(',')
            if len(pa) < 4:
                continue
            pb = b.rstrip().split(',')
            n = int(pb[3])
            if n <= 0 or int(pb[0]) != int(pa[0]):
                continue
            ts.append(int(pa[0])); gts.append([float(pa[1]), float(pa[2]), float(pa[3])])
            rs.append(np.asarray(pb[4:4 + n], dtype=np.float32))
            amin, ainc = float(pb[1]), float(pb[2])
    R = np.stack(rs); T = np.asarray(ts, dtype=np.int64); GT = np.asarray(gts)
    for seed in (42, 1, 2):
        o = np.loadtxt(os.path.join(work, '%s_odom_s%d.csv' % (tag, seed)), delimiter=',')
        assert len(o) == len(T) and np.all(o[:, 0].astype(np.int64) == T)
        np.savez(os.path.join(work, '%s_s%d.npz' % (tag, seed)), t=T, odom=o[:, 1:4], gt=GT, ranges=R,
                 amin=amin, ainc=ainc, rmin=0.1, rmax=30.0)
        print(tag, seed, R.shape)
