#!/usr/bin/env python3
"""stata_score.py track RUN.csv...   |  reloc RELOC.csv  |  init INIT.csv
Scores the outputs of mit_stata_laser2d. Needs numpy."""
import csv, sys
import numpy as np


def track(paths):
    for p in paths:
        r = list(csv.DictReader(open(p)))
        t = np.array([int(x['utime']) for x in r]) * 1e-6
        e = np.array([float(x['err_xy']) for x in r])
        ey = np.array([float(x['err_yaw_deg']) for x in r])
        ev, tot, st = 0, 0.0, None
        for k in range(len(t)):
            bad = e[k] > 1.0
            if bad and st is None: st = t[k]
            if (not bad or k == len(t) - 1) and st is not None:
                if t[k] - st > 2.0: ev += 1; tot += t[k] - st
                st = None
        oe = np.array([float(x['odom_err_xy']) for x in r])
        print('%-40s rows %6d (%5.0f s) rmse %.3f m  p50 %.3f  max %.2f  >0.3m %.3f  >1m %.4f  yaw rmse %.2f deg  excursions %d (%.1f s)  [odometry alone: final %.1f m, max %.1f m]' % (
            p.split('/')[-1], len(e), t[-1] - t[0], np.sqrt(np.mean(e ** 2)), np.median(e), e.max(), np.mean(e > .3), np.mean(e > 1),
            np.sqrt(np.mean(ey ** 2)), ev, tot, oe[-1], oe.max()))


def reloc(p):
    r = list(csv.DictReader(open(p)))
    n = len(r)
    acc = [x for x in r if x['status'] == 'accepted']
    ok = [x for x in acc if x['correct'] == '1']
    amb = sum(x['status'] == 'ambiguous' for x in r)
    nc = sum(x['status'] == 'no candidate' for x in r)
    print('verifier (node defaults), %d queries: correct %d, wrong accepted %d, rejected %d (ambiguous %d, no candidate %d)' % (
        n, len(ok), len(acc) - len(ok), amb + nc, amb, nc))
    s_acc = [x for x in r if x['single_valid'] == '1']
    s_ok = [x for x in s_acc if x['single_correct'] == '1']
    print('single-scan rule: correct %d, wrong accepted %d, rejected %d' % (len(s_ok), len(s_acc) - len(s_ok), n - len(s_acc)))
    a = np.array([float(x['ms_first_scan']) for x in r]); b = np.array([float(x['ms_total']) for x in r])
    print('time first scan (global BBS): median %.0f ms, p90 %.0f, max %.0f; verifier total (9 scans): median %.0f ms, max %.0f' % (
        np.median(a), np.percentile(a, 90), a.max(), np.median(b), b.max()))
    if ok:
        print('accepted-correct pose error: median %.3f m / %.2f deg' % (np.median([float(x['err_xy']) for x in ok]), np.median([float(x['err_yaw_deg']) for x in ok])))


def init(p):
    r = list(csv.DictReader(open(p)))
    lv = []
    for x in r:
        if x['level'] not in lv: lv.append(x['level'])
    print('level  converged  median conv time (s)  median RMSE after conv (m)')
    for l in lv:
        rr = [x for x in r if x['level'] == l]
        c = [x for x in rr if x['converged'] == '1']
        print('%-5s %3d/%-3d   %s   %s' % (l, len(c), len(rr),
              '%6.1f' % np.median([float(x['conv_time_s']) for x in c]) if c else '   -  ',
              '%.3f' % np.median([float(x['rmse_after']) for x in c]) if c else '  -  '))


if __name__ == '__main__':
    m = sys.argv[1]
    {'track': lambda: track(sys.argv[2:]), 'reloc': lambda: reloc(sys.argv[2]), 'init': lambda: init(sys.argv[2])}[m]()
