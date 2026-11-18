#!/usr/bin/env python3
"""nclt_score.py RUN.csv [...]: planar accuracy of nclt_ndt3d runs against the NCLT ground truth.

Per run: planar RMSE and max error, share of scans over 0.3 m and 1 m, excursions (planar
error over 1 m for more than 2 s; count and total seconds), and at the filter updates the
median of n_eff / particles. With several runs, a last line pools them."""
import csv
import sys

import numpy as np


def score(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([int(r['utime']) for r in rows]) * 1e-6
    e = np.array([float(r['err_xy']) for r in rows])
    upd = [r for r in rows if r['updated'] == '1' and int(r['particles']) > 0]
    neff = np.array([float(r['neff']) / int(r['particles']) for r in upd]) if upd else np.array([np.nan])
    ev, tot, start = 0, 0.0, None
    for k in range(len(t)):
        bad = e[k] > 1.0
        if bad and start is None:
            start = t[k]
        if (not bad or k == len(t) - 1) and start is not None:
            if t[k] - start > 2.0:
                ev += 1
                tot += t[k] - start
            start = None
    return dict(run=path, scans=len(e), duration_s=t[-1] - t[0] if len(t) else 0, rmse=float(np.sqrt(np.mean(e ** 2))),
                max=float(e.max()), over03=float(np.mean(e > 0.3)), over1=float(np.mean(e > 1.0)), excursions=ev,
                excursion_s=tot, neff_frac_p50=float(np.median(neff)))


def main():
    res = [score(p) for p in sys.argv[1:]]
    for r in res:
        print("%-50s scans %6d (%5.0f s) rmse %.3f max %7.2f >0.3m %.4f >1m %.4f excursions %d (%.1f s) neff/N %.2f%%" % (
            r['run'], r['scans'], r['duration_s'], r['rmse'], r['max'], r['over03'], r['over1'], r['excursions'],
            r['excursion_s'], 100 * r['neff_frac_p50']))
    if len(res) > 1:
        print("pooled %d runs: rmse median %.3f, max %.2f, excursions %d (%.1f s), runs with an excursion %d, neff/N median %.2f%%" % (
            len(res), np.median([r['rmse'] for r in res]), max(r['max'] for r in res), sum(r['excursions'] for r in res),
            sum(r['excursion_s'] for r in res), sum(r['excursions'] > 0 for r in res),
            100 * np.median([r['neff_frac_p50'] for r in res])))


if __name__ == '__main__':
    main()
