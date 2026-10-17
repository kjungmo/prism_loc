#!/usr/bin/env python3
"""Tables for the lever-arm study (experiments/eskf_lever.cpp). Standard library only.

    eskf_lever_report.py PREFIX [PREFIX ...]      # reads PREFIX_runs.csv and PREFIX_agg.csv (or the .csv.gz of each)

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
from collections import OrderedDict, defaultdict

REF = 'S6vb'   # the scheme the node uses (EskfParams defaults)
SCHEMES = ['ignore', 'S0', 'S0s', 'S1', 'S1s', 'S2', 'S2s', 'S3s', 'S4s', 'S5s', 'S6s', 'S6t', 'S6v', 'S6va', 'S6vb']


def load(prefixes):
    runs, agg = [], []
    for p in prefixes:
        for name, out in (('_runs.csv', runs), ('_agg.csv', agg)):
            f = open(p + name) if os.path.exists(p + name) else gzip.open(p + name + '.gz', 'rt')   # plain or gzipped
            for row in csv.DictReader(f):
                if None not in row.values():   # a line cut short by a run still writing
                    row['cfg'] = os.path.basename(p).split('_s')[0] + '-' + row['cfg']   # suite-id: the ids restart in each suite
                    out.append(row)
    return runs, agg


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
                return statistics.median(float(a[col]) for a in as_)

            def mp(col, f=3):
                xs = [float(r[col]) for r in rs]
                return f'{statistics.median(xs):.{f}f} / {q(xs, 0.95):.{f}f}'

            print(f'| {s} | {nd}/{len(rs)} ({pct(nd, len(rs)).strip()} %) | {mp("max_xy", 2)} | {mp("max_z", 2)} | {mp("max_tilt", 3)} | '
                  f'{mp("max_head", 2)} | {statistics.median(float(r["end_head"]) for r in rs):.2f} | '
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
            cells.append(f'{100.0 * d / tot:.1f}')
        a = [x for x in agg_l if x['scheme'] == sc]
        i = sum(1 for x in a if float(x['pos_lo']) <= float(x['nees_pos_end']) <= float(x['pos_hi']))
        o = sum(1 for x in a if float(x['nees_pos_end']) > float(x['pos_hi']))
        u = len(a) - i - o
        print(f'| {sc} | ' + ' | '.join(cells) + f' | {100 * i / len(a):.0f} / {100 * o / len(a):.0f} / {100 * u / len(a):.0f} |')


def main():
    runs, agg = load(sys.argv[1:])
    lev = lambda r: r['antenna'] != 'Z'
    runs_l = [r for r in runs if lev(r)]
    agg_l = [a for a in agg if lev(a)]
    print(f'# Lever-arm study: {len(runs) // len(SCHEMES)} runs per scheme in {len(agg) // len(SCHEMES)} configurations '
          f'({len(runs_l) // len(SCHEMES)} runs in {len(agg_l) // len(SCHEMES)} configurations with a lever arm)')
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


if __name__ == '__main__':
    main()
