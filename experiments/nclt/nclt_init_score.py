#!/usr/bin/env python3
"""nclt_init_score.py RUN.csv [...]: the measures of INIT_PROTOCOL.md for runs of the NCLT drivers.

Per run (columns utime, err_xy): converged = planar error below 1 m continuously for at least
10 s, starting within the first 120 s of the run; convergence time = start of the first such
window (s from the first row); RMSE after convergence = planar RMSE from that time to 180 s.
Prints one line per run, then a summary line: converged / runs, median convergence time,
median RMSE after convergence (over the converged runs)."""
import csv
import sys

import numpy as np


def measure(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return False, float('nan'), float('nan')
    t = (np.array([int(r['utime']) for r in rows]) - int(rows[0]['utime'])) * 1e-6
    e = np.array([float(r['err_xy']) for r in rows])
    good = e < 1.0
    start = None
    for k in range(len(t)):
        if good[k]:
            if start is None:
                start = k
            if t[k] - t[start] >= 10.0 and t[start] <= 120.0:
                tc = t[start]
                m = (t >= tc) & (t <= 180.0)
                return True, float(tc), float(np.sqrt(np.mean(e[m] ** 2)))
        else:
            start = None
    return False, float('nan'), float('nan')


def main():
    res = []
    for p in sys.argv[1:]:
        c, tc, r = measure(p)
        res.append((c, tc, r))
        print("%-60s converged %d conv_time_s %6.1f rmse_after %.3f" % (p, c, tc, r))
    if len(res) > 1:
        conv = [x for x in res if x[0]]
        print("summary: converged %d / %d, median convergence time %s s, median RMSE after convergence %s m" % (
            len(conv), len(res), "%.1f" % np.median([x[1] for x in conv]) if conv else "-",
            "%.3f" % np.median([x[2] for x in conv]) if conv else "-"))


if __name__ == '__main__':
    main()
