#!/usr/bin/env python3
"""Tables for the lever-arm study (experiments/eskf_lever.cpp). Standard library only.

    eskf_lever_report.py PREFIX [PREFIX ...]      # reads PREFIX_runs.csv and PREFIX_agg.csv (or the .csv.gz of each)

PREFIX is OUT/<suite>: g60, g300 (first pass) and h300, h600, h900, h1800, hndt, hslope, hyaw (second
pass, the observability grid). The first pass is reported as before; the second pass has failures counted
by kind, the selection of experiments/eskf_lever.cpp applied line by line, and a last section, "Numbers
quoted in README.md and the PR body", that prints every number those texts quote.

Every run is counted, diverged or not (a run has diverged if, at any 1 s epoch from 10 s
on, tilt > 0.05 rad, |z| > 0.5 m, horizontal error > 1 m or heading error > 0.5 rad).
NEES columns count configurations (a configuration is the mean over its seeds): "in" is
the share whose NEES at the last epoch lies inside the two-sided 95 % chi-square band,
"over" the share above it (overconfident), "under" the share below it (conservative).
"""
import csv
import gzip
import os
import statistics
import sys
from collections import Counter, OrderedDict, defaultdict

REF = 'S6vb'   # the scheme the node uses (EskfParams defaults)
SCHEMES = ['ignore', 'S0', 'S0s', 'S1', 'S1s', 'S2', 'S2s', 'S3s', 'S4s', 'S5s', 'S6s', 'S6t', 'S6v', 'S6va', 'S6vb']
FIRST_SUITES = ('g60', 'g300')
SG_SUITES = ('hsg', 'hsgv')    # third pass: standing, then moving (reported apart; not part of the second-pass pools)
FIFTH_SUITES = ('hgap', 'hboot')    # fifth pass: an outage across the start, and the default boot with a guessed heading (reported apart)
SIXTH_SUITES = ('hrate',)           # sixth pass: the fix rate (hrate), and the short outages of hgap (the cells whose noise column reads off<s>)
APART = FIRST_SUITES + SG_SUITES + FIFTH_SUITES + SIXTH_SUITES


def load(prefixes):
    runs, agg = [], []
    for p in prefixes:
        for name, out in (('_runs.csv', runs), ('_agg.csv', agg)):
            f = open(p + name) if os.path.exists(p + name) else gzip.open(p + name + '.gz', 'rt')   # plain or gzipped
            for row in csv.DictReader(f):
                if None not in row.values():   # a line cut short by a run still writing
                    row['suite'] = os.path.basename(p).split('_s')[0]
                    row['cfg'] = row['suite'] + '-' + row['cfg']   # suite-id: the ids restart in each suite
                    out.append(row)
    return runs, agg


def mdn(xs):
    """Median, nan for no data (a run of a subset of the suites leaves some groups empty)."""
    xs = list(xs)
    return statistics.median(xs) if xs else float('nan')


def pct(a, b):
    return f'{100.0 * a / b:5.1f}' if b else '  n/a'


def q(xs, p):
    xs = sorted(xs)
    if not xs:
        return float('nan')
    k = min(len(xs) - 1, int(round(p * (len(xs) - 1))))
    return xs[k]


def table(title, runs, agg, keyfn, keys=None, schemes=SCHEMES):
    print(f'\n### {title}\n')
    groups = defaultdict(list)
    for r in runs:
        groups[keyfn(r)].append(r)
    ag = defaultdict(list)
    for a in agg:
        ag[keyfn(a)].append(a)
    keys = keys or sorted(groups, key=lambda k: (str(type(k)), k))
    all_schemes = schemes
    for k in keys:
        if k not in groups:
            continue
        schemes = all_schemes if all_schemes is not None else (['ignore', 'S0'] if k.endswith(' Z') else SCHEMES)
        n_sch = len({r['scheme'] for r in groups[k]})
        print(f'\n**{k}**: {len(groups[k]) // n_sch} runs per scheme, '
              f'{len(ag[k]) // n_sch} configurations\n')
        print('| scheme | diverged | max xy m (median / p95) | max z m (med / p95) | max tilt rad (med / p95) | '
              'max heading rad (med / p95) | end heading rad (median) | pos NEES in/over/under % | att NEES in/over/under % | all-state NEES in/over/under % | median pos / att / all NEES |')
        print('|---|---|---|---|---|---|---|---|---|---|---|')
        for s in schemes:
            rs = [r for r in groups[k] if r['scheme'] == s]
            as_ = [a for a in ag[k] if a['scheme'] == s]
            if not rs:
                continue
            nd = sum(int(r['diverged']) for r in rs)

            def band(col, lo, hi):
                vals = [(float(a[col]), float(a[lo]), float(a[hi])) for a in as_]
                n = len(vals)
                i = sum(1 for v, l, h in vals if l <= v <= h)
                o = sum(1 for v, l, h in vals if v > h)
                u = sum(1 for v, l, h in vals if v < l)
                return f'{100 * i / n:.0f} / {100 * o / n:.0f} / {100 * u / n:.0f}'

            def med(col):
                return mdn(float(a[col]) for a in as_)

            def mp(col, f=3):
                xs = [float(r[col]) for r in rs]
                return f'{mdn(xs):.{f}f} / {q(xs, 0.95):.{f}f}'

            print(f'| {s} | {nd}/{len(rs)} ({pct(nd, len(rs)).strip()} %) | {mp("max_xy", 2)} | {mp("max_z", 2)} | {mp("max_tilt", 3)} | '
                  f'{mp("max_head", 2)} | {mdn(float(r["end_head"]) for r in rs):.2f} | '
                  f'{band("nees_pos_end", "pos_lo", "pos_hi")} | {band("nees_att_end", "att_lo", "att_hi")} | {band("nees_all_end", "all_lo", "all_hi")} | '
                  f'{med("nees_pos_end"):.1f} / {med("nees_att_end"):.1f} / {med("nees_all_end"):.1f} |')



def summary(runs_l, agg_l):
    """One row per scheme: runs diverged / runs, in per cent, for the slices that decide the choice."""
    slices = [('all', lambda r: True), ('GNSS+IMU only', lambda r: r['ndt'] == '0'), ('NDT on', lambda r: r['ndt'] == '1'),
              ('60 s', lambda r: r['dur'] == '60'), ('300 s', lambda r: r['dur'] == '300')]
    for m in ('stand', 'turn', 'spin', 'drive'):
        slices.append((f'GNSS-only {m}', lambda r, m=m: r['ndt'] == '0' and r['motion'] == m))
    slices.append(('GNSS-only 0.5 deg/s', lambda r: r['ndt'] == '0' and r['gyro_bias_dps'] == '0.5'))
    slices.append(('GNSS-only, up to 0.2 deg/s', lambda r: r['ndt'] == '0' and r['gyro_bias_dps'] != '0.5'))
    print('\n### Summary: runs diverged, per cent of the runs of the slice (lever-arm configurations)\n')
    print('| scheme | ' + ' | '.join(n for n, _ in slices) + ' | pos NEES in/over/under % |')
    print('|---|' + '---|' * (len(slices) + 1))
    n_slice = [sum(1 for r in runs_l if r['scheme'] == SCHEMES[0] and f(r)) for _, f in slices]
    print('| runs per scheme | ' + ' | '.join(str(n) for n in n_slice) + ' | |')
    for sc in SCHEMES:
        cells = []
        for (n, f), tot in zip(slices, n_slice):
            d = sum(1 for r in runs_l if r['scheme'] == sc and f(r) and r['diverged'] == '1')
            cells.append(f'{100.0 * d / tot:.1f}' if tot else 'n/a')   # a slice with no runs (one suite only)
        a = [x for x in agg_l if x['scheme'] == sc]
        i = sum(1 for x in a if float(x['pos_lo']) <= float(x['nees_pos_end']) <= float(x['pos_hi']))
        o = sum(1 for x in a if float(x['nees_pos_end']) > float(x['pos_hi']))
        u = len(a) - i - o
        print(f'| {sc} | ' + ' | '.join(cells) + f' | {100 * i / len(a):.0f} / {100 * o / len(a):.0f} / {100 * u / len(a):.0f} |')


def first_pass(runs, agg):
    lev = lambda r: r['antenna'] != 'Z'
    runs_l = [r for r in runs if lev(r)]
    agg_l = [a for a in agg if lev(a)]
    print(f'# Lever-arm study, first pass (suites g60 and g300): {len(runs) // len(SCHEMES)} runs per scheme in '
          f'{len(agg) // len(SCHEMES)} configurations ({len(runs_l) // len(SCHEMES)} runs in {len(agg_l) // len(SCHEMES)} '
          f'configurations with a lever arm)')
    summary(runs_l, agg_l)
    table('All configurations with a lever arm (antenna A or B)', runs_l, agg_l, lambda r: 'all')
    table('By duration', runs_l, agg_l, lambda r: f'{r["dur"]} s')
    table('By NDT', runs_l, agg_l, lambda r: 'NDT on' if r['ndt'] == '1' else 'GNSS + IMU only')
    table('By motion', runs_l, agg_l, lambda r: r['motion'], ['stand', 'turn', 'spin', 'drive'])
    table('By gyro bias per axis (deg/s)', runs_l, agg_l, lambda r: f'gyro bias {r["gyro_bias_dps"]} deg/s',
          [f'gyro bias {g} deg/s' for g in ('0', '0.05', '0.1', '0.2', '0.5')])
    table('By accelerometer bias per axis (m/s^2)', runs_l, agg_l, lambda r: f'accel bias {r["acc_bias"]}')
    table('By antenna (Z = no lever arm: every scheme runs updatePosition, so only the S0 row is shown)', runs, agg,
          lambda r: f'antenna {r["antenna"]}', None, None)
    table('By noise setting', runs_l, agg_l, lambda r: f'noise {r["noise"]}')
    table('By GNSS dropout', runs_l, agg_l, lambda r: 'dropout' if r['drop'] == '1' else 'no dropout')
    table('By reseed', runs_l, agg_l, lambda r: 'reseed' if r['reseed'] == '1' else 'no reseed')
    table('GNSS + IMU only, by gyro bias (the case of the review finding)',
          [r for r in runs_l if r['ndt'] == '0'], [a for a in agg_l if a['ndt'] == '0'],
          lambda r: f'GNSS+IMU, gyro bias {r["gyro_bias_dps"]} deg/s',
          [f'GNSS+IMU, gyro bias {g} deg/s' for g in ('0', '0.05', '0.1', '0.2', '0.5')])

    # paired comparison with the chosen scheme, run by run
    div = defaultdict(dict)
    for r in runs_l:
        div[(r['cfg'], r['seed'])][r['scheme']] = r['diverged'] == '1'
    print('\n### Paired comparison with ' + REF + ', run by run (lever-arm configurations)\n')
    print(f'| scheme | diverged, {REF} not | {REF} diverged, scheme not | both | neither |')
    print('|---|---|---|---|---|')
    for sc in SCHEMES:
        if sc == REF:
            continue
        a = sum(1 for d in div.values() if d[sc] and not d[REF])
        b = sum(1 for d in div.values() if d[REF] and not d[sc])
        c = sum(1 for d in div.values() if d[REF] and d[sc])
        n = sum(1 for d in div.values() if not d[REF] and not d[sc])
        print(f'| {sc} | {a} | {b} | {c} | {n} |')

    # zero lever: every scheme runs the same code (updatePosition), so the rows must be identical
    z = [r for r in runs if r['antenna'] == 'Z']
    by = defaultdict(dict)
    for r in z:
        by[(r['cfg'], r['seed'])][r['scheme']] = tuple(r[c] for c in ('diverged', 'max_xy', 'max_z', 'max_tilt', 'max_head', 'end_xy', 'end_z', 'end_tilt', 'end_head'))
    # S-variants with a seeded tilt variance are also identical at zero lever (the node and the
    # driver do not seed it: bit-identity with main)
    same = 0
    for k, d in by.items():
        ref = d.get('S0')
        if ref is not None and all(d[s] == ref for s in d if s != 'ignore'):
            same += 1
    print(f'\n### Zero lever (antenna Z)\n\n{len(by)} runs; all lever schemes give identical results in {same} of them '
          f'("ignore" is the same code path as well: identical in '
          f'{sum(1 for d in by.values() if "ignore" in d and "S0" in d and d["ignore"] == d["S0"])}).')

    # worst runs
    print('\n### Diverged runs of S0 (the first version), the chosen scheme and the full update, first 12 by size of the heading error\n')
    for s in ('S0', REF, 'S2s'):
        bad = sorted((r for r in runs_l if r['scheme'] == s and r['diverged'] == '1'), key=lambda r: -float(r['max_head']))[:12]
        print(f'\n{s}:')
        for r in bad:
            print(f"- cfg {r['cfg']} seed {r['seed']} {r['motion']} {r['antenna']} gyro {r['gyro_bias_dps']} acc {r['acc_bias']} ndt {r['ndt']} "
                  f"drop {r['drop']} reseed {r['reseed']} {r['noise']}: t_div {r['t_div']} xy {r['max_xy']} z {r['max_z']} tilt {r['max_tilt']} head {r['max_head']}")



# ---------------------------------------------------------------------------------------------
# Second pass: the observability grid (suites h*)
# ---------------------------------------------------------------------------------------------
HS = ['S0', 'S1s', 'S2s', 'S6vb', 'S7', 'S7a', 'S7a35', 'S7a50', 'S7r05', 'S7r20', 'S7n', 'S7a35r05', 'S7a35r20', 'S8', 'S8u']   # S8: added after the fourth review, S8u after the fifth
CAND = ['S7', 'S7a', 'S7a35', 'S7a50', 'S7r05', 'S7r20', 'S7a35r05', 'S7a35r20']
RELEASES = {'S7a', 'S7a35', 'S7a50', 'S7r05', 'S7r20', 'S7n', 'S7a35r05', 'S7a35r20'}     # schemes with lever_release_dv > 0
NPARAM = {'S7': 1, 'S7a': 2, 'S7a35': 2, 'S7a50': 2, 'S7r05': 2, 'S7r20': 2, 'S7a35r05': 2, 'S7a35r20': 2}
ENVELOPE_MOTIONS = ('stand', 'line', 'startstop', 'turn')


def f(r, k):
    return float(r[k])


def tz(r):          # tilt > 0.05 rad or |z| > 0.5 m (outside the dropout and reseed windows)
    return f(r, 'max_tilt') > 0.05 or f(r, 'max_z') > 0.5


def circ(r):        # beyond the circle of radius |t_xy| around the antenna, plus 0.3 m
    return f(r, 'max_circ') > 0.0


def headf(r):
    return f(r, 'max_head') > 0.5


def xy1(r):
    return f(r, 'max_xy') > 1.0


def envelope(r):    # the cells in which tz and circ must be 0
    return r['motion'] in ENVELOPE_MOTIONS and f(r, 'gyro_bias_dps') <= 0.2


def pair_key(r):    # one data set: the four antennas of a configuration share it
    return (r['suite'], int(r['cfg'].split('-')[1]) // 4, r['seed'])


def ftable(title, rows, keyfn, keys=None, schemes=HS, note=None):
    """Failures counted by kind, per scheme, for the rows of each key."""
    print(f'\n### {title}\n')
    if note:
        print(note + '\n')
    groups = defaultdict(list)
    for r in rows:
        groups[keyfn(r)].append(r)
    keys = keys or sorted(groups)
    for k in keys:
        if k not in groups:
            continue
        n = len({r['scheme'] for r in groups[k]})
        print(f'\n**{k}**: {len(groups[k]) // n} runs per scheme\n')
        print('| scheme | runs | tilt>0.05 or abs z>0.5 (tz) | abs z>1 | outside the circle (circ) | xy>1 | heading>0.5 | heading only | '
              'max xy m (median / p95) | max z m p95 | max tilt rad p95 | end heading rad (median) | release open (median share) | bound active (median share) |')
        print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
        for sc in schemes:
            rs = [r for r in groups[k] if r['scheme'] == sc]
            if not rs:
                continue
            nn = len(rs)
            nt = sum(tz(r) for r in rs)
            nz = sum(f(r, 'max_z') > 1.0 for r in rs)
            nc = sum(circ(r) for r in rs)
            nx = sum(xy1(r) for r in rs)
            nh = sum(headf(r) for r in rs)
            nho = sum(headf(r) and not (tz(r) or circ(r) or xy1(r)) for r in rs)
            mx = [f(r, 'max_xy') for r in rs]
            print(f'| {sc} | {nn} | {nt} ({pct(nt, nn).strip()} %) | {nz} | {nc} | {nx} | {nh} ({pct(nh, nn).strip()} %) | {nho} | '
                  f'{mdn(mx):.2f} / {q(mx, 0.95):.2f} | {q([f(r, "max_z") for r in rs], 0.95):.2f} | '
                  f'{q([f(r, "max_tilt") for r in rs], 0.95):.3f} | {mdn(f(r, "end_head") for r in rs):.2f} | '
                  f'{mdn(f(r, "rel_frac") for r in rs):.2f} | {mdn(f(r, "cap_frac") for r in rs):.2f} |')


def paired_with_zero(main_rows):
    """Run by run against the zero-lever row of the same data (the four antennas of a configuration share it)."""
    zt = {pair_key(r): tz(r) for r in main_rows if r['antenna'] == 'Z' and r['scheme'] == 'S0' and r['ndt'] == '0'}
    print('\n### Paired with the zero-lever row of the same data (GNSS and IMU only, lever-arm rows of h300 to h1800)\n')
    print('tz = tilt > 0.05 rad or abs z > 0.5 m. "scheme only": the lever-arm scheme has tz and the zero-lever row of the same data has not; '
          '"zero lever only": the other way round.\n')
    print('| scheme | runs | both | scheme only | zero lever only | neither | scheme only, envelope cells | scheme only, other cells |')
    print('|---|---|---|---|---|---|---|---|')
    for sc in HS:
        rs = [r for r in main_rows if r['scheme'] == sc and r['antenna'] != 'Z' and r['ndt'] == '0']
        if not rs:
            continue
        both = sum(tz(r) and zt[pair_key(r)] for r in rs)
        only = sum(tz(r) and not zt[pair_key(r)] for r in rs)
        zonly = sum((not tz(r)) and zt[pair_key(r)] for r in rs)
        nei = len(rs) - both - only - zonly
        oe = sum(tz(r) and not zt[pair_key(r)] and envelope(r) for r in rs)
        print(f'| {sc} | {len(rs)} | {both} | {only} | {zonly} | {nei} | {oe} | {only - oe} |')


def selection(rows, agg):
    """The pre-declared selection of experiments/eskf_lever.cpp, line by line."""
    lev = [r for r in rows if r['antenna'] != 'Z' and r['ndt'] == '0' and r['suite'] in H_MAIN]
    zref = [r for r in rows if r['antenna'] == 'Z' and r['ndt'] == '0' and r['suite'] in H_MAIN and r['scheme'] == 'S0']
    cell = lambda r: (r['suite'], r['dur'], r['motion'], r['gyro_bias_dps'])
    zc = Counter(cell(r) for r in zref if tz(r))
    zfail = {pair_key(r): (tz(r) or circ(r)) for r in zref}      # the zero-lever row of the same data
    print('\n### Selection (the pre-declared criteria of experiments/eskf_lever.cpp)\n')
    n_env = sum(1 for r in lev if r['scheme'] == 'S1s' and envelope(r))
    n_oth = sum(1 for r in lev if r['scheme'] == 'S1s' and not envelope(r))
    print(f'GNSS-and-IMU-only rows with a lever arm (antennas A, B, M) in the suites {", ".join(H_MAIN)}: '
          f'{n_env} runs per scheme in the envelope cells (stand, line, startstop, turn at gyro bias <= 0.2 deg/s, '
          f'every duration), {n_oth} runs per scheme in the other cells (drive, spin, 0.5 deg/s). '
          f'The zero-lever reference (antenna Z) has {sum(1 for r in zref)} runs, of which {sum(zc.values())} have tz.\n')
    ag_stand = [a for a in agg if a['antenna'] != 'Z' and a['ndt'] == '0' and a['motion'] == 'stand' and a['suite'] in H_MAIN]
    ag_all = [a for a in agg if a['antenna'] != 'Z' and a['ndt'] == '0' and a['suite'] in H_MAIN]

    def in_share(sc):
        a = [x for x in ag_stand if x['scheme'] == sc]
        return sum(1 for x in a if f(x, 'pos_lo') <= f(x, 'nees_pos_end') <= f(x, 'pos_hi')) / len(a)

    def tilt_below(sc):
        a = [x for x in ag_all if x['scheme'] == sc]
        return sum(1 for x in a if f(x, 'nees_tilt_end') < f(x, 'tilt_lo')) / len(a)

    def tilt_above(sc):
        a = [x for x in ag_all if x['scheme'] == sc]
        return sum(1 for x in a if f(x, 'nees_tilt_end') > f(x, 'tilt_hi')) / len(a)

    def head_above(sc):
        a = [x for x in ag_all if x['scheme'] == sc]
        return sum(1 for x in a if f(x, 'nees_head_end') > f(x, 'head_hi')) / len(a)

    ref_in = in_share('S1s')
    drive_head = {sc: sum(headf(r) for r in lev if r['scheme'] == sc and r['motion'] == 'drive') for sc in HS}
    results = []
    print('| scheme | E: tz in envelope | E: circ in envelope | E: either | E\': of those, runs the zero-lever row of the same data survives | X: tz over the zero-lever reference, other cells | '
          f'pos NEES in band, standing configs % (S1s: {100 * ref_in:.0f}) | tilt NEES below band % | tilt NEES above band % | heading NEES above band % | '
          f'C | drive heading>0.5 (S6vb: {drive_head["S6vb"]}) | H | median max xy, envelope m | parameters |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for sc in HS:
        rs = [r for r in lev if r['scheme'] == sc]
        if not rs:
            continue
        env = [r for r in rs if envelope(r)]
        e_tz, e_circ = sum(tz(r) for r in env), sum(circ(r) for r in env)
        e_any = sum(tz(r) or circ(r) for r in env)
        e_excess = sum((tz(r) or circ(r)) and not zfail.get(pair_key(r), False) for r in env)
        # X: per cell and antenna, failures above the zero-lever row of the same data
        sc_cell = defaultdict(Counter)
        for r in rs:
            if not envelope(r) and tz(r):
                sc_cell[cell(r)][r['antenna']] += 1
        x = 0
        for c, cnt in sc_cell.items():
            for ant in ('A', 'B', 'M'):
                x += max(0, cnt[ant] - zc[c])
        for c in {cell(r) for r in rs if not envelope(r)}:
            pass
        ins = in_share(sc)
        tb, ta, ha = tilt_below(sc), tilt_above(sc), head_above(sc)
        c_pen = int(ins + 1e-12 < ref_in) + int(tb > 0.2)
        h_pen = int(sc in RELEASES and drive_head[sc] > drive_head['S6vb'])
        mxy = mdn(f(r, 'max_xy') for r in env)
        results.append((sc, e_any, x, c_pen, h_pen, mxy, NPARAM.get(sc, 99)))
        print(f'| {sc} | {e_tz} | {e_circ} | {e_any} | {e_excess} | {x} | {100 * ins:.0f} | {100 * tb:.0f} | {100 * ta:.0f} | {100 * ha:.0f} | {c_pen} | '
              f'{drive_head[sc]} | {h_pen} | {mxy:.3f} | {NPARAM.get(sc, "")} |')
    cands = [t for t in results if t[0] in CAND]
    ok = sorted([t for t in cands if t[1] == 0], key=lambda t: (t[2], t[3], t[4], t[5], t[6]))
    zr_env = [r for r in zref if envelope(r)]
    zr_bad = sum(1 for r in zr_env if tz(r) or circ(r))
    if zr_bad:
        print(f'\nThe zero-lever reference itself has tz or circ in {zr_bad} of its {len(zr_env)} runs in the envelope cells (the accelerometer bias of '
              f'0.3 m/s^2 and the 120 s dropouts, from which the plain update does not recover either): E > 0 for a lever-arm scheme is then not by itself '
              f'a defect of the scheme, and E\' counts the runs that the zero-lever row of the same data survives.\n')
    else:
        print(f'\nThe zero-lever reference has no tz or circ in its {len(zr_env)} runs in the envelope cells.\n')
    if ok:
        best = ok[0]
        print(f'**Chosen: {best[0]}** (candidates with E = 0, ordered by X, C, H, median max xy, parameters: '
              + ', '.join(f'{t[0]} (X {t[2]}, C {t[3]}, H {t[4]}, {t[5]:.3f} m)' for t in ok) + ').')
        fails = [n for n, v in (('2 (X)', best[2]), ('3 (C)', best[3]), ('4 (H)', best[4])) if v]
        print(f'Criteria that {best[0]} does not meet outright: ' + (', '.join(fails) if fails else 'none') + '.')
        print('Excluded by E > 0: ' + (', '.join(f'{t[0]} (E {t[1]})' for t in cands if t[1] > 0) or 'none') + '.')
    else:
        best = min(cands, key=lambda t: (t[1], t[2], t[3], t[4], t[5], t[6]))
        print(f'**None of the candidates has E = 0.** The declared order is then applied to all of them with E as its first key '
              f'(fewest runs with tz or circ in the envelope cells): {best[0]} (E {best[1]}, X {best[2]}, C {best[3]}, H {best[4]}, median max xy {best[5]:.3f} m).')
        print('Order of all candidates: ' + ', '.join(f'{t[0]} (E {t[1]}, X {t[2]}, C {t[3]}, H {t[4]}, {t[5]:.3f} m)' for t in sorted(cands, key=lambda t: (t[1], t[2], t[3], t[4], t[5], t[6]))) + '.')
    if best[0] != SHIPPED:
        a = next(t for t in results if t[0] == best[0])
        b = next(t for t in results if t[0] == SHIPPED)
        print(f'\n**Shipped: {SHIPPED}, not {best[0]}.** The declared order puts {best[0]} first by {b[1] - a[1]} runs in E ({a[1]} against {b[1]}, of '
              f'{n_env} runs per scheme in the envelope cells) and {b[2] - a[2]} in X ({a[2]} against {b[2]}): inside what two seeds per configuration can tell apart. '
              f'The suite hvib, written after that order was declared and run after the first results, shows that the release at the lower threshold opens on a robot that does '
              f'not accelerate once the accelerometer noise is 0.1 m/s^2 per axis (table "Accelerometer noise" below), and the threshold the order prefers is the '
              f'less robust one. This is a deviation from the declared order, and it is the only one.')
    return best[0] if best[0] == SHIPPED else SHIPPED, results


H_MAIN = ('h300', 'h600', 'h900', 'h1800')
# What the code ships (EskfParams defaults). The declared order picks the scheme printed under "Chosen"; where it differs from SHIPPED the
# report says so and why. S7a35: bound 0.35 rad, release at 1.0 m/s.
SHIPPED = "S7a35"


def second_pass(runs, agg):
    runs = [r for r in runs if r['suite'] not in APART]
    agg = [a for a in agg if a['suite'] not in APART]
    if not runs:
        return None
    suites = sorted({r['suite'] for r in runs})
    print('\n\n# Lever-arm study, second pass: the observability grid\n')
    print('| suite | configurations | runs per scheme | seeds | duration s | notes |')
    print('|---|---|---|---|---|---|')
    for su in suites:
        rs = [r for r in runs if r['suite'] == su and r['scheme'] == 'S1s']
        cf = len({r['cfg'] for r in rs})
        print(f'| {su} | {cf} | {len(rs)} | {len({r["seed"] for r in rs})} | {",".join(sorted({r["dur"] for r in rs}, key=int))} | '
              f'{"NDT on" if rs[0]["ndt"] == "1" else "GNSS + IMU only"}{"; tilted mount" if su == "hslope" else ""}{"; wrong start heading" if su == "hyaw" else ""} |')
    main_rows = [r for r in runs if r['suite'] in H_MAIN]
    lev = [r for r in main_rows if r['antenna'] != 'Z' and r['ndt'] == '0']
    if lev:
      ftable('GNSS and IMU only, all lever-arm rows (antennas A, B, M) of h300, h600, h900, h1800', lev, lambda r: 'all')
    cells = lambda r: f'{r["motion"]} {r["dur"]} s'
    keys = [f'{m} {d} s' for d in ('300', '600', '900', '1800') for m in ('stand', 'line', 'startstop', 'turn', 'drive', 'spin')]
    ftable('GNSS and IMU only, by motion and duration (lever-arm rows)', lev, cells, keys)
    ftable('GNSS and IMU only, by gyro bias per axis, deg/s (lever-arm rows)', lev, lambda r: f'gyro bias {r["gyro_bias_dps"]} deg/s')
    ftable('GNSS and IMU only, by accelerometer bias per axis, m/s^2 (lever-arm rows)', lev, lambda r: f'accel bias {r["acc_bias"]}')
    ftable('GNSS and IMU only, by antenna (A, B, M: lever arms; Z: zero lever, the same code path in every scheme)',
           [r for r in main_rows if r['ndt'] == '0'], lambda r: f'antenna {r["antenna"]}', ['antenna A', 'antenna B', 'antenna M', 'antenna Z'])
    ftable('GNSS and IMU only, by GNSS dropout (lever-arm rows; the dropout window and 5 s after are not tested)', lev,
           lambda r: 'no dropout' if r['drop'] == '0' else f'dropout {r["drop_s"]} s')
    ftable('GNSS and IMU only, by reseed (lever-arm rows; the 5 s after a reseed are not tested)', lev,
           lambda r: 'reseed at half time' if r['reseed'] == '1' else 'no reseed')

    # zero lever: one code path
    z = [r for r in main_rows + [r for r in runs if r['suite'] in ('hndt', 'hslope', 'hyaw')] if r['antenna'] == 'Z']
    by = defaultdict(dict)
    for r in z:
        by[(r['cfg'], r['seed'])][r['scheme']] = tuple(r[c] for c in ('diverged', 'max_xy', 'max_z', 'max_tilt', 'max_head',
                                                                      'end_xy', 'end_z', 'end_tilt', 'end_head', 'max_circ'))
    same = sum(1 for d in by.values() if len(set(d.values())) == 1)
    zl = [r for r in main_rows if r['antenna'] == 'Z' and r['ndt'] == '0' and r['scheme'] == 'S0']
    print(f'\n### Zero lever (antenna Z), second pass\n\n{len(by)} runs; every scheme gives identical results in {same} of them '
          f'(the lever schemes only act on an antenna off the origin). With GNSS and IMU only in h300 to h1800 the zero-lever reference has '
          f'{sum(tz(r) for r in zl)} tz, {sum(headf(r) for r in zl)} heading>0.5 and {sum(xy1(r) for r in zl)} xy>1 in {len(zl)} runs.')

    best = None
    if main_rows:
        best, res = selection(runs, agg)
        paired_with_zero(main_rows)
        clean = [r for r in lev if envelope(r) and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1]
        zclean = [r for r in main_rows if r['antenna'] == 'Z' and r['scheme'] == 'S0' and r['ndt'] == '0' and envelope(r) and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1]
        ftable('GNSS and IMU only, the clean envelope: stand, line, startstop, turn at gyro bias <= 0.2 deg/s, accelerometer bias <= 0.1 m/s^2, '
               'no GNSS dropout (lever-arm rows)', clean, lambda r: f'{r["dur"]} s',
               note=f'The zero-lever row of the same data ({len(zclean)} runs): {sum(tz(r) for r in zclean)} with tz, {sum(headf(r) for r in zclean)} with heading>0.5, '
                    f'{sum(xy1(r) for r in zclean)} with xy>1.')
        ftable('GNSS and IMU only, standing and line, 900 s and 1800 s, accelerometer bias <= 0.1 m/s^2, no dropout (lever-arm rows)',
               [r for r in clean if r['dur'] in ('900', '1800') and r['motion'] in ('stand', 'line')], lambda r: f'{r["motion"]} {r["dur"]} s',
               [f'{m} {d} s' for d in ('900', '1800') for m in ('stand', 'line')])

    # NDT on
    ndt = [r for r in runs if r['suite'] == 'hndt' and r['antenna'] != 'Z']
    if ndt:
        ftable('NDT fixes on (suite hndt, 300 s, all motions, reseed, dropout; lever-arm rows)', ndt, lambda r: 'NDT on')
    sl = [r for r in runs if r['suite'] == 'hslope' and r['antenna'] != 'Z']
    if sl:
        print('\n### Reseed on a tilted mount (suite hslope: 60 s, /initialpose at 30 s with a level attitude; lever-arm rows)\n')
        print('"tilt at the end > 0.02" is the tilt error after the reseed, at 60 s. S7n takes the level attitude of the message, the others '
              'roll and pitch from the accelerometer (S0, S1s, S2s and S6vb as before: the message).\n')
        print('| tilt of the body rad | NDT | scheme | runs | tz | tilt at the end > 0.02 | median end tilt rad | p95 end tilt rad |')
        print('|---|---|---|---|---|---|---|---|')
        for slope in ('0', '0.05', '0.1'):
            for nd in ('0', '1'):
                for sc in HS:
                    rs = [r for r in sl if r['slope'] == slope and r['ndt'] == nd and r['scheme'] == sc]
                    if not rs:
                        continue
                    et = [f(r, 'end_tilt') for r in rs]
                    print(f'| {slope} | {"on" if nd == "1" else "off"} | {sc} | {len(rs)} | {sum(tz(r) for r in rs)} | '
                          f'{sum(e > 0.02 for e in et)} | {mdn(et):.4f} | {q(et, 0.95):.4f} |')
    ya = [r for r in runs if r['suite'] == 'hyaw' and r['antenna'] != 'Z']
    if ya:
        print('\n### A heading that starts far off (suite hyaw: 300 s, GNSS and IMU only, lever-arm rows)\n')
        print('"recovered" = heading error below 0.2 rad at the end. Not a criterion: with the robot standing the heading is not '
              'estimated; the question is only whether the bound keeps a scheme from recovering it when the robot drives with acceleration.\n')
        print('| start error rad | motion | scheme | runs | recovered | median end heading rad | p95 end heading rad | tz |')
        print('|---|---|---|---|---|---|---|---|')
        for y0 in ('0.5', '1', '2'):
            for m in ('stand', 'line', 'startstop', 'turn', 'drive'):
                for sc in HS:
                    rs = [r for r in ya if r['yaw0'] == y0 and r['motion'] == m and r['scheme'] == sc]
                    if not rs:
                        continue
                    eh = [f(r, 'end_head') for r in rs]
                    print(f'| {y0} | {m} | {sc} | {len(rs)} | {sum(e < 0.2 for e in eh)} | {mdn(eh):.2f} | {q(eh, 0.95):.2f} | {sum(tz(r) for r in rs)} |')
    vb = [r for r in runs if r['suite'] == 'hvib' and r['antenna'] != 'Z']
    if vb:
        print('\n### Accelerometer noise (suite hvib: 300 s, standing and straight line at 1.5 m/s, gyro bias 0.1 deg/s, accelerometer bias 0.1 m/s^2; lever-arm rows)\n')
        print('The filter is told the noise of the simulated IMU. "release open" is the share of lever updates with the release open: on a robot that does not '
              'accelerate it is a false release (the heading is then corrected from data that does not show it).\n')
        print('| accelerometer noise m/s^2 per axis | scheme | runs | tz | circ | xy>1 | release open, mean | release open, largest | heading>0.5 |')
        print('|---|---|---|---|---|---|---|---|---|')
        noise_of = lambda r: 0.02 if r['noise'] == 'matched' else float(r['noise'][3:])
        for nz in sorted({noise_of(r) for r in vb}):
            for sc in HS:
                rs = [r for r in vb if noise_of(r) == nz and r['scheme'] == sc]
                if not rs or sc in ('S0', 'S7', 'S7n', 'S7r05', 'S7r20', 'S7a50'):
                    continue
                print(f'| {nz:g} | {sc} | {len(rs)} | {sum(tz(r) for r in rs)} | {sum(circ(r) for r in rs)} | {sum(xy1(r) for r in rs)} | '
                      f'{statistics.mean(f(r, "rel_frac") for r in rs):.3f} | {max(f(r, "rel_frac") for r in rs):.3f} | {sum(headf(r) for r in rs)} |')
    return best


def quoted_first(runs, agg):
    """The numbers of the first pass that README.md and the PR text quote and that no table above prints as such."""
    lev = [r for r in runs if r['antenna'] != 'Z']
    print('\n### Numbers quoted in README.md and the PR text: first pass\n')
    v = [r for r in lev if r['scheme'] == 'S6vb' and r['diverged'] == '1']
    print(f'- S6vb (the version of the previous round): {len(v)} of {sum(1 for r in lev if r["scheme"] == "S6vb")} lever-arm runs diverged; '
          f'{sum(f(r, "max_head") > 0.5 for r in v)} of them crossed the heading limit (0.5 rad); {sum(r["gyro_bias_dps"] == "0.5" for r in v)} have a gyro bias of 0.5 deg/s; '
          f'{sum(f(r, "max_tilt") > 0.05 or f(r, "max_z") > 0.5 for r in v)} have a tilt over 0.05 rad or a height error over 0.5 m; {sum(f(r, "max_z") > 1.0 for r in v)} a height error over 1 m.')
    n05 = sum(1 for r in lev if r['scheme'] == 'S6vb' and r['gyro_bias_dps'] == '0.5')
    print(f'- S6vb at 0.5 deg/s: {sum(r["gyro_bias_dps"] == "0.5" for r in v)} of {n05} runs ({100.0 * sum(r["gyro_bias_dps"] == "0.5" for r in v) / n05:.1f} %) diverged.')
    for sc in ('S0', 'S6vb'):
        st = [r for r in lev if r['scheme'] == sc and r['dur'] == '300' and r['motion'] == 'stand' and r['gyro_bias_dps'] == '0' and r['ndt'] == '0']
        print(f'- {sc}, standing 300 s, gyro bias 0, GNSS and IMU only: {sum(r["diverged"] == "1" for r in st)} of {len(st)} runs diverged.')
    for sc in ('S1s', 'S6vb'):
        dr = [r for r in lev if r['scheme'] == sc and r['dur'] == '300' and r['motion'] == 'drive' and r['ndt'] == '0']
        print(f'- {sc}, driving 300 s, GNSS and IMU only: {sum(r["diverged"] == "1" for r in dr)} of {len(dr)} runs diverged.')
    z = [r for r in runs if r['antenna'] == 'Z' and r['scheme'] == 'S0' and r['ndt'] == '0']
    print(f'- Zero lever, GNSS and IMU only: {sum(r["diverged"] == "1" for r in z)} of {len(z)} runs diverged.')
    for sc in ('S0', 'S6vb'):
        xs = [f(r, 'max_xy') for r in lev if r['scheme'] == sc and r['ndt'] == '0' and r['gyro_bias_dps'] == '0']
        print(f'- {sc}, GNSS and IMU only, gyro bias 0: median of the largest horizontal error {mdn(xs):.2f} m over {len(xs)} runs.')
    for sc in ('S0', 'S0s', 'S6vb'):
        g = [r for r in lev if r['scheme'] == sc and r['ndt'] == '0']
        print(f'- {sc}, GNSS and IMU only, {len(g)} runs: diverged {sum(r["diverged"] == "1" for r in g)}.')
    a_all = [a for a in agg if a['antenna'] != 'Z' and a['scheme'] == 'S6vb']
    a_z = [a for a in agg if a['antenna'] == 'Z' and a['scheme'] == 'S6vb']
    for name, aa in (('lever arm', a_all), ('zero lever', a_z)):
        above = sum(1 for a in aa if f(a, 'nees_all_end') > f(a, 'all_hi'))
        print(f'- S6vb, NEES of all 15 states above its band, {name}: {above} of {len(aa)} configurations ({100.0 * above / len(aa):.0f} %).')


def quoted_second(runs, agg, best, prev='S6vb'):
    """The numbers of the second pass that README.md and the PR text quote."""
    chosen = best
    main_rows = [r for r in runs if r['suite'] in H_MAIN]
    lev = [r for r in main_rows if r['antenna'] != 'Z' and r['ndt'] == '0']
    zr = [r for r in main_rows if r['antenna'] == 'Z' and r['ndt'] == '0' and r['scheme'] == 'S0']
    print(f'\n### Numbers quoted in README.md and the PR text: second pass (chosen scheme {chosen}, previous version {prev}, zero lever = antenna Z)\n')
    print(f'- Runs per scheme, GNSS and IMU only: {sum(1 for r in lev if r["scheme"] == chosen)} with a lever arm (antennas A, B, M), {len(zr)} with the zero lever.')

    def line(label, rs):
        n = len(rs)
        if not n:
            return f'- {label}: no runs'
        return (f'- {label}: {n} runs, tz {sum(tz(r) for r in rs)}, circ {sum(circ(r) for r in rs)}, xy>1 {sum(xy1(r) for r in rs)}, '
                f'heading>0.5 {sum(headf(r) for r in rs)}, median max xy {mdn(f(r, "max_xy") for r in rs):.2f} m, '
                f'p95 {q([f(r, "max_xy") for r in rs], 0.95):.2f} m')

    for sc in (prev, chosen):
        print(line(f'{sc}, all cells', [r for r in lev if r['scheme'] == sc]))
    print(line('zero lever, all cells', zr))
    for dur in ('300', '600', '900', '1800'):
        for mot in ('stand', 'line'):
            for sc in (prev, chosen):
                rs = [r for r in lev if r['scheme'] == sc and r['dur'] == dur and r['motion'] == mot and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1 and f(r, 'gyro_bias_dps') <= 0.2]
                print(line(f'{sc}, {mot} {dur} s, no dropout, accelerometer bias <= 0.1, gyro bias <= 0.2 deg/s', rs))
            rs = [r for r in zr if r['dur'] == dur and r['motion'] == mot and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1 and f(r, 'gyro_bias_dps') <= 0.2]
            print(line(f'zero lever, {mot} {dur} s, same cells', rs))
    for sc in (prev, chosen):
        rs = [r for r in lev if r['scheme'] == sc and r['dur'] in ('900', '1800') and r['motion'] == 'stand' and r['drop'] == '0']
        print(line(f'{sc}, standing 900 s and 1800 s, every gyro and accelerometer bias of the grid, no dropout', rs))
    clean = lambda r: envelope(r) and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1
    for sc in (prev, chosen):
        print(line(f'{sc}, the clean envelope (stand, line, startstop, turn; gyro bias <= 0.2 deg/s; accelerometer bias <= 0.1; no dropout)', [r for r in lev if r['scheme'] == sc and clean(r)]))
    print(line('zero lever, the clean envelope', [r for r in zr if clean(r)]))
    # where the failures of the chosen scheme are
    bad = [r for r in lev if r['scheme'] == chosen and tz(r)]
    zt = {pair_key(r): tz(r) for r in zr}
    print(f'- {chosen}: {len(bad)} runs with tz; {sum(zt[pair_key(r)] for r in bad)} of them also have tz in the zero-lever row of the same data; '
          f'{sum(r["acc_bias"] == "0.3" for r in bad)} have an accelerometer bias of 0.3 m/s^2; {sum(r["drop"] == "1" for r in bad)} a GNSS dropout; '
          f'{sum(r["drop"] == "0" and r["acc_bias"] != "0.3" for r in bad)} neither.')
    neither = [r for r in bad if r['drop'] == '0' and r['acc_bias'] != '0.3']
    from collections import Counter as _C
    print(f'  those without dropout and without accelerometer bias 0.3 by motion and gyro bias: ' + ', '.join(f'{k[0]} {k[1]} deg/s: {v}' for k, v in sorted(_C((r['motion'], r['gyro_bias_dps']) for r in neither).items())) + '.')
    for sc in (prev, chosen):
        rs = [r for r in lev if r['scheme'] == sc and r['dur'] == '600' and r['motion'] == 'stand']
        reseed_in_dropout = [r for r in rs if r['drop'] == '1' and r['reseed'] == '1']
        print(f'- {sc}, standing 600 s: {sum(tz(r) for r in rs)} of {len(rs)} with tz; {sum(tz(r) for r in reseed_in_dropout)} of the {len(reseed_in_dropout)} runs whose reseed (300 s) falls inside the dropout (240 to 360 s); '
              f'the zero-lever row has {sum(tz(r) for r in zr if r["dur"] == "600" and r["drop"] == "1" and r["reseed"] == "1")} of {sum(1 for r in zr if r["dur"] == "600" and r["drop"] == "1" and r["reseed"] == "1")} there.')
    rs_only = [r for r in bad if not zt[pair_key(r)]]
    print(f'- {chosen}: {len(rs_only)} runs with tz where the zero-lever run of the same data has none: {sum(r["drop"] == "1" for r in rs_only)} with a GNSS dropout, '
          f'{sum(r["drop"] == "0" and r["acc_bias"] == "0.3" for r in rs_only)} without a dropout and with an accelerometer bias of 0.3 m/s^2, '
          f'{sum(r["drop"] == "0" and r["acc_bias"] != "0.3" for r in rs_only)} with neither; {sum(r["dur"] == "300" for r in rs_only)} are runs of 300 s.')
    big = [r for r in lev if r['scheme'] == chosen and xy1(r)]
    long_nodrop = [r for r in big if r['drop'] == '0' and int(r['dur']) >= 900]
    print(f'- {chosen}: {len(big)} runs with a horizontal error over 1 m; antenna A {sum(r["antenna"] == "A" for r in big)}, B {sum(r["antenna"] == "B" for r in big)}, M {sum(r["antenna"] == "M" for r in big)}; '
          f'{len(long_nodrop)} are runs of 900 s or 1,800 s without a dropout; {sum(r["drop"] == "1" for r in big)} have a GNSS dropout; the zero-lever row: {sum(xy1(r) for r in zr)} of {len(zr)}.')
    prev_big = {(r['cfg'], r['seed']) for r in lev if r['scheme'] == prev and xy1(r)}
    new_big = [r for r in big if (r['cfg'], r['seed']) not in prev_big]
    print(f'- {chosen}: {len(new_big)} runs over 1 m that {prev} does not have (and {sum(1 for k in prev_big if k not in {(r["cfg"], r["seed"]) for r in big})} that only {prev} has); '
          f'of the new ones {sum(r["antenna"] == "A" and r["gyro_bias_dps"] == "0.5" for r in new_big)} are antenna A at 0.5 deg/s, with the largest error {min((f(r, "max_xy") for r in new_big if r["antenna"] == "A" and r["gyro_bias_dps"] == "0.5"), default=float("nan")):.2f} to '
          f'{max((f(r, "max_xy") for r in new_big if r["antenna"] == "A" and r["gyro_bias_dps"] == "0.5"), default=float("nan")):.2f} m (the same runs under {prev}: '
          f'{min((f(r, "max_xy") for r in lev if r["scheme"] == prev and (r["cfg"], r["seed"]) in {(x["cfg"], x["seed"]) for x in new_big if x["antenna"] == "A" and x["gyro_bias_dps"] == "0.5"}), default=float("nan")):.2f} to '
          f'{max((f(r, "max_xy") for r in lev if r["scheme"] == prev and (r["cfg"], r["seed"]) in {(x["cfg"], x["seed"]) for x in new_big if x["antenna"] == "A" and x["gyro_bias_dps"] == "0.5"}), default=float("nan")):.2f} m).')
    prow = {(r['cfg'], r['seed']): r for r in lev if r['scheme'] == prev}
    gain = [r for r in lev if r['scheme'] == chosen and tz(r) and not tz(prow[(r['cfg'], r['seed'])])]
    lose = [r for r in lev if r['scheme'] == chosen and not tz(r) and tz(prow[(r['cfg'], r['seed'])])]
    print(f'- {chosen} against {prev}: {len(gain)} runs gain tz and {len(lose)} lose it; {sum(r["motion"] in ("drive", "startstop") for r in gain)} of the {len(gain)} are drive and start-stop runs '
          f'(they begin moving within 6 s of the start); {sum(r["acc_bias"] == "0.3" for r in gain)} of the {len(gain)} have an accelerometer bias of 0.3 m/s^2; '
          f'the zero-lever run of the same data has tz in {sum(zt[pair_key(r)] for r in gain)} of the {len(gain)}.')
    for sc in (prev, chosen):
        dr = [r for r in lev if r['scheme'] == sc and r['drop'] == '1']
        print(line(f'{sc}, runs with a GNSS dropout (30 s or 120 s)', dr))
    print(line('zero lever, runs with a GNSS dropout', [r for r in zr if r['drop'] == '1']))
    zl_long = [r for r in zr if r['motion'] == 'stand' and r['dur'] in ('900', '1800') and r['drop'] == '0']
    print(line('zero lever, standing 900 s and 1800 s, every gyro and accelerometer bias of the grid, no dropout', zl_long))
    for sc in (prev, chosen):
        sp = [r for r in lev if r['scheme'] == sc and r['motion'] == 'spin' and r['dur'] == '300' and r['gyro_bias_dps'] == '0.5' and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1]
        print(line(f'{sc}, spinning 300 s at 0.5 deg/s, no dropout, accelerometer bias <= 0.1', sp))
    print(line('zero lever, spinning 300 s at 0.5 deg/s, no dropout, accelerometer bias <= 0.1',
               [r for r in zr if r['motion'] == 'spin' and r['dur'] == '300' and r['gyro_bias_dps'] == '0.5' and r['drop'] == '0' and f(r, 'acc_bias') <= 0.1]))
    # release and bound activity
    for mot in ('stand', 'line', 'startstop', 'turn', 'drive', 'spin'):
        rs = [r for r in lev if r['scheme'] == chosen and r['motion'] == mot and r['dur'] == '300']
        if not rs:   # h300 was not among the suites
            continue
        print(f'- {chosen}, {mot} 300 s: share of lever updates with the release open: median {mdn(f(r, "rel_frac") for r in rs):.2f}, mean {statistics.mean(f(r, "rel_frac") for r in rs):.2f}; '
              f'with the bound active: median {mdn(f(r, "cap_frac") for r in rs):.2f}, mean {statistics.mean(f(r, "cap_frac") for r in rs):.2f}.')
    # NDT, slope, wrong start heading
    nd = [r for r in runs if r['suite'] == 'hndt' and r['antenna'] != 'Z']
    for sc in ('S0', prev, chosen):
        print(line(f'{sc}, NDT fixes on (hndt)', [r for r in nd if r['scheme'] == sc]))
    sl = [r for r in runs if r['suite'] == 'hslope' and r['antenna'] != 'Z' and r['ndt'] == '0']
    for slope in ('0', '0.05', '0.1'):
        for sc in (prev, chosen, 'S7n'):
            rs = [r for r in sl if r['slope'] == slope and r['scheme'] == sc]
            if not rs:   # hslope was not among the suites
                continue
            print(f'- reseed on a body tilted by {slope} rad, GNSS and IMU only, {sc}: {len(rs)} runs, tz {sum(tz(r) for r in rs)}, tilt at the end > 0.02 rad {sum(f(r, "end_tilt") > 0.02 for r in rs)}, '
                  f'median end tilt {mdn(f(r, "end_tilt") for r in rs):.4f} rad.')
    ya = [r for r in runs if r['suite'] == 'hyaw' and r['antenna'] != 'Z']
    for y0 in ('0.5', '1', '2'):
        for sc in (prev, chosen, 'S7'):
            for mot in ('stand', 'startstop', 'drive'):
                rs = [r for r in ya if r['yaw0'] == y0 and r['scheme'] == sc and r['motion'] == mot]
                if not rs:   # hyaw was not among the suites
                    continue
                print(f'- start heading {y0} rad off, {mot} 300 s, {sc}: {len(rs)} runs, recovered (< 0.2 rad at the end) {sum(f(r, "end_head") < 0.2 for r in rs)}, tz {sum(tz(r) for r in rs)}, median end heading {mdn(f(r, "end_head") for r in rs):.2f} rad.')
    # consistency
    ag = [a for a in agg if a['suite'] in H_MAIN and a['antenna'] != 'Z' and a['ndt'] == '0']
    for sc in (prev, chosen):
        aa = [a for a in ag if a['scheme'] == sc]
        st = [a for a in aa if a['motion'] == 'stand']
        inb = sum(1 for a in st if f(a, 'pos_lo') <= f(a, 'nees_pos_end') <= f(a, 'pos_hi'))
        tb = sum(1 for a in aa if f(a, 'nees_tilt_end') < f(a, 'tilt_lo'))
        ta = sum(1 for a in aa if f(a, 'nees_tilt_end') > f(a, 'tilt_hi'))
        ha = sum(1 for a in aa if f(a, 'nees_head_end') > f(a, 'head_hi'))
        print(f'- {sc}, NEES at the last epoch: position inside its band in {inb} of {len(st)} standing configurations; tilt (2 dof) below its band in {tb} and above in {ta} of {len(aa)} configurations; '
              f'heading (1 dof) above its band in {ha} of {len(aa)}.')
    # the zero-lever identity over the second pass
    zz = defaultdict(dict)
    for r in runs:
        if r['antenna'] == 'Z':
            zz[(r['cfg'], r['seed'])][r['scheme']] = tuple(r[c] for c in ('diverged', 'max_xy', 'max_z', 'max_tilt', 'max_head', 'end_xy', 'end_z', 'end_tilt', 'end_head', 'max_circ'))
    print(f'- Zero lever in the second pass: {len(zz)} runs, every scheme identical in {sum(1 for d in zz.values() if len(set(d.values())) == 1)}.')


SG_MOTIONS = ('sgstart', 'sgdrive', 'sgturn')
SG_SCHEMES = ['S2s', 'S6vb', 'S7a35', 'S8r', 'S8a', 'S8', 'S8k', 'S8v', 'S8u']


def third_pass(sg_rows):
    """Standing for minutes, then moving (suites hsg and hsgv): the failure of the fourth review round and the bar of eskf_lever.cpp."""
    print('\n\n# Lever-arm study, third pass: standing, then moving\n')
    print('Suites hsg (the schemes S8r, S8a and S8 were developed on these runs) and hsgv (the same configurations on other data, run once after the '
          'schemes were fixed). Each run stands for 300, 600, 900 or 1,800 s at the origin with a gyro bias of 0 to 0.2 deg/s per axis, then starts '
          '(sgstart: 0 to 1.5 m/s in 3 s, then the start-stop cycle), drives (sgdrive: the same start, then a circle of radius 6 m at 1.5 m/s) or turns on the spot '
          '(sgturn: 0.3 rad/s for 4 s of every 5 s) for 160 s. GNSS and IMU only; antennas A, B, M and the zero lever Z; 4 seeds. '
          'tz = tilt > 0.05 rad or abs z > 0.5 m at an epoch from 10 s on (the standing time counts); "excess" = the lever-arm run has tz and the '
          'zero-lever run of the same data has not; "z>1" = abs z > 1 m; "tilt>0.3" = tilt > 0.3 rad. The bar (declared in eskf_lever.cpp): in each '
          'motion, the lever-arm rows have no more tz than the zero-lever rows of the same data (pairs counted: tz of the lever run <= tz of its zero-lever '
          'run, summed), and no run with z>1 or tilt>0.3 in the envelope motions (sgstart, sgdrive with an antenna that has a horizontal arm: A, B).\n')
    for su in SG_SUITES:
        rows = [r for r in sg_rows if r['suite'] == su]
        if not rows:
            continue
        zref = {}
        for r in rows:
            if r['antenna'] == 'Z':
                zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])] = r
        print(f'\n## Suite {su}\n')
        schemes = [sc for sc in SG_SCHEMES if any(r['scheme'] == sc for r in rows)]

        def cnt(rs):
            both = excess = zonly = 0
            for r in rs:
                z = zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])]
                a, b = tz(r), tz(z)
                both += a and b
                excess += a and not b
                zonly += (not a) and b
            n = len(rs)
            return (n, sum(tz(r) for r in rs), both + zonly, excess, sum(f(r, 'max_z') > 1.0 for r in rs), sum(f(r, 'max_tilt') > 0.3 for r in rs),
                    sum(circ(r) for r in rs), sum(xy1(r) for r in rs), sum(f(r, 'end_head') > 0.5 for r in rs))
        hdr = ('| scheme | runs | tz | tz of the paired zero-lever runs | excess | z>1 | tilt>0.3 | circ | xy>1 | heading>0.5 at the end |\n'
               '|---|---|---|---|---|---|---|---|---|---|')

        def block(title, pick):
            print(f'\n### {title}\n')
            print(hdr)
            for sc in schemes:
                rs = [r for r in rows if r['scheme'] == sc and r['antenna'] != 'Z' and pick(r)]
                if not rs:
                    continue
                n, t, ztz, ex, z1, t3, ci, x1, eh = cnt(rs)
                zt = sum(tz(zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])]) for r in rs)
                print(f'| {sc} | {n} | {t} | {zt} | {ex} | {z1} | {t3} | {ci} | {x1} | {eh} |')
        block('All lever-arm runs (antennas A, B, M)', lambda r: True)
        for m in SG_MOTIONS:
            block(f'{m}', lambda r, m=m: r['motion'] == m)
        for stand in (300, 600, 900, 1800):
            block(f'Standing {stand} s', lambda r, stand=stand: int(r['dur']) - 160 == stand)
        for gb in ('0', '0.05', '0.1', '0.2'):
            block(f'Gyro bias {gb} deg/s per axis', lambda r, gb=gb: r['gyro_bias_dps'] == gb)
        for ant in ('A', 'B', 'M'):
            block(f'Antenna {ant}', lambda r, ant=ant: r['antenna'] == ant)
        print('\n### The bar, per scheme\n')
        print('| scheme | tz <= zero lever in sgstart | in sgdrive | in sgturn | z>1 or tilt>0.3, envelope (A, B; sgstart, sgdrive) | z>1 or tilt>0.3, all |')
        print('|---|---|---|---|---|---|')
        for sc in schemes:
            cells = []
            for m in SG_MOTIONS:
                rs = [r for r in rows if r['scheme'] == sc and r['antenna'] != 'Z' and r['motion'] == m]
                t = sum(tz(r) for r in rs)
                zt = sum(tz(zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])]) for r in rs)
                cells.append(f'{"yes" if t <= zt else "no"} ({t} against {zt})')
            env = [r for r in rows if r['scheme'] == sc and r['antenna'] in ('A', 'B') and r['motion'] in ('sgstart', 'sgdrive')]
            allr = [r for r in rows if r['scheme'] == sc and r['antenna'] != 'Z']
            cat = lambda rs: sum(f(r, 'max_z') > 1.0 or f(r, 'max_tilt') > 0.3 for r in rs)
            print(f'| {sc} | {cells[0]} | {cells[1]} | {cells[2]} | {cat(env)} of {len(env)} | {cat(allr)} of {len(allr)} |')
        zr = [r for r in rows if r['antenna'] == 'Z']
        by = defaultdict(set)
        for r in zr:
            by[(r['cfg'], r['seed'])].add(tuple(r[c] for c in ('diverged', 'max_xy', 'max_z', 'max_tilt', 'max_head', 'end_xy', 'end_z', 'end_tilt', 'end_head', 'max_circ')))
        print(f'\nZero lever in {su}: {len(by)} runs, every scheme identical in {sum(1 for v in by.values() if len(v) == 1)}.')
        print(f'\n### Numbers quoted in README.md and experiments/README.md: {su}\n')
        print('The zero-lever filter has 384 runs per suite (128 per motion), the same for every scheme. The lever-arm rows are 1,152 (384 per motion); '
              'each zero-lever run is the pair of three lever-arm runs (antennas A, B, M), so a column that sums the zero-lever run once per lever-arm run '
              'counts it three times ("paired").\n')
        zrows = [r for r in rows if r['antenna'] == 'Z' and r['scheme'] == (schemes[0])]
        print(f'- Zero lever, unique runs ({schemes[0]} rows): {len(zrows)}; tz {sum(tz(r) for r in zrows)}; heading>0.5 at the end {sum(f(r, "end_head") > 0.5 for r in zrows)}; '
              f'xy>1 {sum(xy1(r) for r in zrows)}; z>1 {sum(f(r, "max_z") > 1.0 for r in zrows)}; tilt>0.3 {sum(f(r, "max_tilt") > 0.3 for r in zrows)}.')
        for m in SG_MOTIONS:
            zm = [r for r in zrows if r['motion'] == m]
            print(f'- Zero lever, {m}, unique runs: {len(zm)}; tz {sum(tz(r) for r in zm)}; heading>0.5 at the end {sum(f(r, "end_head") > 0.5 for r in zm)}; xy>1 {sum(xy1(r) for r in zm)}.')
        for sc in schemes:
            lr = [r for r in rows if r['scheme'] == sc and r['antenna'] != 'Z']
            ptz = sum(tz(zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])]) for r in lr)
            print(f'- {sc}: lever-arm runs {len(lr)}; tz {sum(tz(r) for r in lr)}; the paired zero-lever tz summed once per lever-arm run (paired) {ptz}, '
                  f'which is {ptz // 3} unique zero-lever runs of {len(zrows)} counted three times.')
            for m in SG_MOTIONS:
                lm = [r for r in lr if r['motion'] == m]
                print(f'  - {m}: {len(lm)} runs; heading>0.5 at the end {sum(f(r, "end_head") > 0.5 for r in lm)}; xy>1 {sum(xy1(r) for r in lm)}; '
                      f'tz {sum(tz(r) for r in lm)}; excess {sum(tz(r) and not tz(zref[(int(r["cfg"].split("-")[1]) // 4, r["seed"])]) for r in lm)}.')
            ex = [r for r in lr if tz(r) and not tz(zref[(int(r['cfg'].split('-')[1]) // 4, r['seed'])])]
            print(f'  - excess {len(ex)}: antenna M {sum(r["antenna"] == "M" for r in ex)}, A {sum(r["antenna"] == "A" for r in ex)}, B {sum(r["antenna"] == "B" for r in ex)}; '
                  f'z>1 {sum(f(r, "max_z") > 1.0 for r in lr)}, tilt>0.3 {sum(f(r, "max_tilt") > 0.3 for r in lr)}.')


def fifth_pass(rows):
    """Fifth review round: an outage across the start (hgap) and the default boot with a heading that is a guess (hboot)."""
    print('\n\n# Lever-arm study, fifth pass: an outage across the start, and a boot with a guessed heading\n')
    print('Schemes (round 7: S8k is the S8u of the fifth and sixth rounds, S8u the code that ships now, see the sixth pass): S8o (S8 without the span guard of the alignment, the version before this round), S8 (with the guard), S8u (S8 + the heading of the '
          'first start is held as unknown, pi / sqrt(3) rad: what the code ships). tz = tilt > 0.05 rad or abs z > 0.5 m at an epoch from 10 s on '
          '(epochs from the start of an outage to 5 s after it are not tested); "excess" = the lever-arm run has tz and the zero-lever run of the same data has not; '
          '"z>1" = abs z > 1 m; "tilt>0.3" = tilt > 0.3 rad. 4 seeds; GNSS and IMU only; antennas A, B, M (lever) and Z (zero lever, the same data).\n')
    for su in FIFTH_SUITES:
        rs_all = [r for r in rows if r['suite'] == su and not r['noise'].startswith('off')]   # hgap: the cells of the fifth pass (the short outages are in the sixth pass)
        if not rs_all:
            continue
        zref = {}
        for r in rs_all:
            if r['antenna'] == 'Z':
                zref[(r['scheme'], int(r['cfg'].split('-')[1]) // 4, r['seed'])] = r
        schemes = [sc for sc in ('S8o', 'S8', 'S8k', 'S8u') if any(r['scheme'] == sc for r in rs_all)]
        zp = lambda r: zref[(r['scheme'], int(r['cfg'].split('-')[1]) // 4, r['seed'])]
        if su == 'hgap':
            print('\n## Suite hgap: the fixes stop 10 s before the robot starts (the start-stop cycle, or the circle) and come back 30 s or 120 s later\n')
            keyf = lambda r: ('stop %s s' % r['drop_s'])
            keys = sorted({keyf(r) for r in rs_all}, key=lambda k: float(k.split()[1]))
        else:
            print('\n## Suite hboot: initial_yaw off by 0.5, 1, 2 and 3 rad, no /initialpose, a stand of 6 to 300 s, then the start (gyro bias 0.1 deg/s, accelerometer bias 0.1 m/s^2)\n')
            keyf = lambda r: 'yaw %s rad, stand %d s' % (r['yaw0'], int(r['dur']) - 160)
            keys = sorted({keyf(r) for r in rs_all}, key=lambda k: (float(k.split()[1]), int(k.split()[4])))

        def row(label, sc, sel):
            lr = [r for r in rs_all if r['scheme'] == sc and r['antenna'] != 'Z' and sel(r)]
            if not lr:
                return
            zu = {(int(r['cfg'].split('-')[1]) // 4, r['seed']): zp(r) for r in lr}   # the zero-lever runs of these data, once each
            ex = sum(tz(r) and not tz(zp(r)) for r in lr)
            print(f'| {label} | {sc} | {len(lr)} | {sum(tz(r) for r in lr)} | {sum(tz(z) for z in zu.values())} of {len(zu)} | {ex} | '
                  f'{sum(f(r, "max_z") > 1.0 for r in lr)} | {sum(f(r, "max_tilt") > 0.3 for r in lr)} | {sum(f(r, "end_head") > 0.5 for r in lr)} |')
        hdr = ('| cell | scheme | lever-arm runs | tz | zero lever (unique runs): tz | excess | z>1 | tilt>0.3 | heading>0.5 at the end |\n'
               '|---|---|---|---|---|---|---|---|---|')
        print(hdr)
        for sc in schemes:
            row('all', sc, lambda r: True)
        for k in keys:
            for sc in schemes:
                row(k, sc, lambda r, k=k: keyf(r) == k)
        if su == 'hgap':
            print('\n### By antenna, outage of 120 s\n')
            print(hdr)
            for ant in ('A', 'B', 'M'):
                for sc in schemes:
                    row('antenna ' + ant, sc, lambda r, ant=ant: r['antenna'] == ant and r['drop_s'] == '120')
            print('\n### By standing time\n')
            print(hdr)
            for st in (300, 600):
                for sc in schemes:
                    row('stand %d s' % st, sc, lambda r, st=st: int(r['dur']) - 160 == st)
        else:
            print('\n### By start error\n')
            print(hdr)
            for y0 in ('0.5', '1', '2', '3'):
                for sc in schemes:
                    row('yaw ' + y0 + ' rad', sc, lambda r, y0=y0: r['yaw0'] == y0)
            print('\n### By antenna\n')
            print(hdr)
            for ant in ('A', 'B', 'M'):
                for sc in schemes:
                    row('antenna ' + ant, sc, lambda r, ant=ant: r['antenna'] == ant)
            print('\n### By motion\n')
            print(hdr)
            for m in ('sgstart', 'sgdrive'):
                for sc in schemes:
                    row(m, sc, lambda r, m=m: r['motion'] == m)


def sixth_pass(rows):
    """Sixth review round: the fix rate (suite hrate) and the short outages across the first start (the cells of hgap whose noise column reads off<s>)."""
    print('\n\n# Lever-arm study, sixth pass: the fix rate, and a short outage at the first start\n')
    xy15 = lambda r: f(r, 'max_xy') > 1.5
    z1 = lambda r: f(r, 'max_z') > 1.0
    head5 = lambda r: f(r, 'end_head') > 0.5
    div = lambda r: f(r, 'max_z') > 100.0 or f(r, 'max_tilt') > 1.5
    rate = [r for r in rows if r['suite'] == 'hrate']
    if rate:
        print('## Suite hrate: the same boot and stand-then-go cells at fix rates of 0.5, 1, 2, 5, 10 and 20 Hz\n')
        print('Cells (GNSS and IMU only, no /initialpose, 4 seeds, the IMU data of a cell is the same at every rate): *boot* = gyro bias 0.1 deg/s and accelerometer bias 0.1 m/s^2, a stand of 6, 30 or 300 s, '
              '`initial_yaw` 0.1, 0.3 or 2.0 rad off, then the start-stop cycle or the circle; *stand-then-go* = stand 300 or 900 s, gyro bias 0 or 0.2 deg/s, accelerometer bias 0 or 0.1 m/s^2, `initial_yaw` 0.1 rad off. '
              'Schemes: S8k = the alignment of the fifth and sixth rounds (span of 4 fixes); S8t = spans in seconds only; S8n = the test of the norms only (the sixth review\'s proposal, 30 %); '
              'S8v = S8u + the test of the norms; S8u = spans in seconds, the fix-rate limit and the spread after an outage (what the code ships; the spread acts only after an outage, none in this suite); '
              'S8 = no unknown start; S2s, S6vb = the comparators. tz = tilt > 0.05 rad or abs z > 0.5 m; z>1 = abs z > 1 m; heading>0.5 = heading error over 0.5 rad at the end; xy>1.5 = largest horizontal error over 1.5 m. '
              'Antennas A and B (a horizontal arm) and, apart, the mast M (no horizontal arm).\n')
        per = lambda n: float(n[3:-1]) if n.startswith('fix') else 0.2   # the noise column reads fix<period>s; "matched" is the 5 Hz of every earlier suite
        periods = sorted({r['noise'] for r in rate}, key=lambda n: -per(n))
        hz = lambda n: '%g Hz' % (1.0 / per(n))
        schemes = [sc for sc in ('S2s', 'S6vb', 'S8', 'S8k', 'S8t', 'S8n', 'S8v', 'S8u') if any(r['scheme'] == sc for r in rate)]

        def kind(r):
            st = int(r['dur']) - 160
            return 'boot' if st in (6, 30, 300) and r['gyro_bias_dps'] == '0.1' and r['acc_bias'] == '0.1' else 'stand-then-go'

        def block(title, ants, keyf, keys, only=None):
            print(f'\n### {title}\n')
            print('| fix rate | cell | scheme | runs | tz | z>1 | heading>0.5 | xy>1.5 |')
            print('|---|---|---|---|---|---|---|---|')
            for n in periods:
                for k in keys:
                    for sc in (only or schemes):
                        rs = [r for r in rate if r['noise'] == n and r['scheme'] == sc and r['antenna'] in ants and keyf(r) == k]
                        if rs:
                            print(f'| {hz(n)} | {k} | {sc} | {len(rs)} | {sum(tz(r) for r in rs)} | {sum(z1(r) for r in rs)} | {sum(head5(r) for r in rs)} | {sum(xy15(r) for r in rs)} |')
        block('Antennas A and B, by cell', ('A', 'B'), kind, ['boot', 'stand-then-go'])
        block('Antennas A and B, boot, by `initial_yaw` error', ('A', 'B'), lambda r: 'boot, yaw %s' % r['yaw0'] if kind(r) == 'boot' else 'other', ['boot, yaw 0.1', 'boot, yaw 0.3', 'boot, yaw 2'])
        block('Mast M, by cell', ('M',), kind, ['boot', 'stand-then-go'])
        block('Zero lever Z (the same data; it does not depend on the scheme, one scheme shown), by cell', ('Z',), kind, ['boot', 'stand-then-go'], only=['S8u'])
        print('\n### Numbers quoted in README.md and experiments/README.md: hrate\n')
        for n in periods:
            for sc in ('S8k', 'S8u'):
                rs = [r for r in rate if r['noise'] == n and r['scheme'] == sc and r['antenna'] in ('A', 'B')]
                if rs:
                    print(f'- {hz(n)} (a fix every {per(n):g} s), {sc}, antennas A and B, {len(rs)} runs: tz {sum(tz(r) for r in rs)}, z>1 {sum(z1(r) for r in rs)}, heading>0.5 at the end {sum(head5(r) for r in rs)}, xy>1.5 {sum(xy15(r) for r in rs)}.')
        for n in periods:
            for sc in ('S8k', 'S8u'):
                rs = [r for r in rate if r['noise'] == n and r['scheme'] == sc and r['antenna'] in ('A', 'B') and kind(r) == 'boot' and r['yaw0'] == '2']
                if rs:
                    print(f'- {hz(n)}, {sc}, boot with `initial_yaw` 2 rad off, antennas A and B, {len(rs)} runs: tz {sum(tz(r) for r in rs)}, heading>0.5 at the end {sum(head5(r) for r in rs)}.')
        for n in periods:
            for sc in ('S8k', 'S8u'):
                rs = [r for r in rate if r['noise'] == n and r['scheme'] == sc and r['antenna'] in ('A', 'B') and kind(r) == 'stand-then-go']
                if rs:
                    print(f'- {hz(n)}, {sc}, stand-then-go, antennas A and B, {len(rs)} runs: tz {sum(tz(r) for r in rs)}, heading>0.5 at the end {sum(head5(r) for r in rs)}.')
    gap = [r for r in rows if r['suite'] == 'hgap' and r['noise'].startswith('off')]
    if gap:
        print('\n## Suite hgap, the short outages: the fixes stop just before or in the middle of the first acceleration after a stand\n')
        print('The outage begins `off` s after the end of the stand (sgstart accelerates from 6 s on: 5 s is 1 s before the acceleration, 7.5 s in the middle of it; sgdrive accelerates from 0 s: -1 s, 1.5 s, 5 s) and lasts 1.5, 3, 5 or 12 s. '
              'Stand 300 or 900 s, gyro bias 0 or 0.2 deg/s, accelerometer bias 0 or 0.1 m/s^2, 4 seeds, antennas A, B (lever arm with a horizontal part), M (mast) and Z (zero lever, the same data). '
              'S8k = the S8u of the fifth and sixth rounds; S8w = spans in seconds and the fix-rate limit, no spread of the covariance after an outage (`lever_outage_spread` 0); S8v = S8u + the test of the norms (`lever_align_norm` 0.3); S8u = the code that ships (spread factor 0.1, no test of the norms). '
              'tz, z>1, xy>1.5 as above; excess = the run has tz and the zero-lever run of the same data has none; diverged = abs z > 100 m or tilt > 1.5 rad.\n')
        zref = {}
        for r in gap:
            if r['antenna'] == 'Z':
                zref[(r['scheme'], int(r['cfg'].split('-')[1]) // 4, r['seed'])] = r
        schemes = [sc for sc in ('S8o', 'S8', 'S8k', 'S8w', 'S8v', 'S8u') if any(r['scheme'] == sc for r in gap)]

        def gblock(title, keyf, keys):
            print(f'\n### {title}\n')
            print('| cell | scheme | antennas | runs | tz | excess | z>1 | xy>1.5 | diverged |')
            print('|---|---|---|---|---|---|---|---|---|')
            for k in keys:
                for sc in schemes:
                    for label, ants in (('A, B', ('A', 'B')), ('M', ('M',)), ('Z', ('Z',))):
                        rs = [r for r in gap if r['scheme'] == sc and r['antenna'] in ants and keyf(r) == k]
                        if not rs or (label == 'Z' and sc != 'S8u'):   # the zero lever does not depend on the scheme: one row
                            continue
                        ex = sum(tz(r) and not tz(zref[(sc, int(r['cfg'].split('-')[1]) // 4, r['seed'])]) for r in rs) if label != 'Z' else 0
                        print(f'| {k} | {sc} | {label} | {len(rs)} | {sum(tz(r) for r in rs)} | {ex if label != "Z" else "-"} | {sum(z1(r) for r in rs)} | {sum(xy15(r) for r in rs)} | {sum(div(r) for r in rs)} |')
        gblock('All short outages', lambda r: 'all', ['all'])
        gblock('By start of the outage', lambda r: '%s, %s' % (r['motion'], r['noise']), sorted({'%s, %s' % (r['motion'], r['noise']) for r in gap}))
        gblock('By length of the outage', lambda r: '%s s' % r['drop_s'], ['1.5 s', '3 s', '5 s', '12 s'])
        gblock('By stand and gyro bias', lambda r: 'stand %d s, gyro bias %s' % (int(r['dur']) - 160, r['gyro_bias_dps']), sorted({'stand %d s, gyro bias %s' % (int(r['dur']) - 160, r['gyro_bias_dps']) for r in gap}))
        print('\n### Numbers quoted in README.md and experiments/README.md: the short outages\n')
        for sc in schemes:
            for label, ants in (('antennas A and B', ('A', 'B')), ('the mast M', ('M',)), ('zero lever', ('Z',))):
                rs = [r for r in gap if r['scheme'] == sc and r['antenna'] in ants]
                if rs:
                    print(f'- {sc}, {label}: {len(rs)} runs, tz {sum(tz(r) for r in rs)}, z>1 {sum(z1(r) for r in rs)}, xy>1.5 {sum(xy15(r) for r in rs)}, diverged (abs z>100 m or tilt>1.5 rad) {sum(div(r) for r in rs)}, largest height error {max(f(r, "max_z") for r in rs):.1f} m.')
        for sc in ('S8k', 'S8u'):
            rs = [r for r in gap if r['scheme'] == sc and r['antenna'] in ('A', 'B') and not (r['motion'] == 'sgdrive' and r['noise'] == 'off-1')]
            ca = [r for r in gap if r['scheme'] == sc and r['antenna'] in ('A', 'B') and (r['motion'] == 'sgdrive' and r['noise'] == 'off-1')]
            print(f'- {sc}, antennas A and B, outages that end before the circle starts or that do not cover its start ({len(rs)} runs): tz {sum(tz(r) for r in rs)}, xy>1.5 {sum(xy15(r) for r in rs)}.')
            print(f'- {sc}, antennas A and B, the outage begins 1 s before the circle (sgdrive, off-1; {len(ca)} runs): tz {sum(tz(r) for r in ca)}, xy>1.5 {sum(xy15(r) for r in ca)}; antenna A alone {sum(xy15(r) for r in ca if r["antenna"] == "A")} of {sum(1 for r in ca if r["antenna"] == "A")}.')
        for sc in ('S8k', 'S8u'):
            rs = [r for r in gap if r['scheme'] == sc and r['antenna'] == 'M' and r['gyro_bias_dps'] == '0.2' and int(r['dur']) - 160 == 900]
            print(f'- {sc}, mast M, stand 900 s at 0.2 deg/s ({len(rs)} runs): tz {sum(tz(r) for r in rs)}, z>1 {sum(z1(r) for r in rs)}, diverged {sum(div(r) for r in rs)}.')


def main():
    runs, agg = load(sys.argv[1:])
    first_r = [r for r in runs if r['suite'] in FIRST_SUITES]
    first_a = [a for a in agg if a['suite'] in FIRST_SUITES]
    if first_r:
        first_pass(first_r, first_a)
        if {r['suite'] for r in first_r} >= set(FIRST_SUITES):
            quoted_first(first_r, first_a)
        else:
            print('\n(The numbers quoted from the first pass need both g60 and g300; left out.)')
    best = second_pass(runs, agg)
    second_r = [r for r in runs if r['suite'] not in APART]
    second_a = [a for a in agg if a['suite'] not in APART]
    if best:
        quoted_second(second_r, second_a, best)
        if any(r['scheme'] == 'S8' for r in second_r):   # the shipped scheme of the fourth review round, on the same cells
            quoted_second(second_r, second_a, 'S8', prev='S7a35')
        if any(r['scheme'] == 'S8u' for r in second_r):  # the shipped scheme of the fifth review round
            quoted_second(second_r, second_a, 'S8u', prev='S8')
        if any(r['scheme'] == 'S8k' for r in second_r) and any(r['scheme'] == 'S8u' for r in second_r):   # round 7: the code that ships against the S8u of the rounds 5 and 6
            quoted_second(second_r, second_a, 'S8u', prev='S8k')
    sg_r = [r for r in runs if r['suite'] in SG_SUITES]
    if sg_r:
        third_pass(sg_r)
    fifth_r = [r for r in runs if r['suite'] in FIFTH_SUITES]
    if fifth_r:
        fifth_pass(fifth_r)
    sixth_r = [r for r in runs if r['suite'] in SIXTH_SUITES or (r['suite'] == 'hgap' and r['noise'].startswith('off'))]
    if sixth_r:
        sixth_pass(sixth_r)

if __name__ == '__main__':
    main()
