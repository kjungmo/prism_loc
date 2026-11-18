#!/usr/bin/env python3
"""amcl_score.py levels RUN_DIR PREFIX [LEVELS]  |  track RUN.csv...

levels: per level, converged runs / total, median convergence time, median RMSE after convergence, with
the unchanged measure() of nclt_init_score.py, over RUN_DIR/PREFIX_<level>_*.csv. Also prints the number of
runs whose RMSE after convergence is over 5 m (convergence that was lost again later).
track: the measures of stata_score.py track (RMSE, median, max, share over 0.3 m and 1 m, excursions)."""
import csv, glob, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nclt_init_score import measure


def levels(d, prefix, lv):
    for L in lv:
        fs = sorted(glob.glob(os.path.join(d, '%s_%s_*.csv' % (prefix, L))))
        res = [measure(f) for f in fs]
        conv = [r for r in res if r[0]]
        lost = sum(1 for r in conv if r[2] > 5.0)
        print('%-5s converged %2d/%-2d  median conv time %s s  median RMSE after conv %s m  (runs with RMSE > 5 m after conv: %d)' % (
            L, len(conv), len(res), '%.1f' % np.median([r[1] for r in conv]) if conv else '-',
            '%.3f' % np.median([r[2] for r in conv]) if conv else '-', lost))


def track(paths):
    for p in paths:
        r = [x for x in csv.DictReader(open(p)) if x['err_xy'] != 'nan']
        t = np.array([int(x['utime']) for x in r]) * 1e-6
        e = np.array([float(x['err_xy']) for x in r])
        yaw = np.array([float(x['yaw']) - float(x['gt_yaw']) for x in r]); yaw = np.abs(np.arctan2(np.sin(yaw), np.cos(yaw))) * 180 / np.pi
        ev, tot, st = 0, 0.0, None
        for k in range(len(t)):
            bad = e[k] > 1.0
            if bad and st is None: st = t[k]
            if (not bad or k == len(t) - 1) and st is not None:
                if t[k] - st > 2.0: ev += 1; tot += t[k] - st
                st = None
        print('%-32s rows %6d (%5.0f s) rmse %.3f m  p50 %.3f  max %.2f  >0.3m %.3f  >1m %.4f  yaw rmse %.2f deg  excursions %d (%.1f s)' % (
            os.path.basename(p), len(e), t[-1] - t[0], np.sqrt(np.mean(e ** 2)), np.median(e), e.max(), np.mean(e > .3), np.mean(e > 1),
            np.sqrt(np.mean(yaw ** 2)), ev, tot))


if __name__ == '__main__':
    if sys.argv[1] == 'levels':
        levels(sys.argv[2], sys.argv[3], (sys.argv[4] if len(sys.argv) > 4 else 'L0,L1,L2,L3,L4,L5,L6,U,U20k').split(','))
    else:
        track(sys.argv[2:])
