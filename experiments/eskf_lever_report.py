#!/usr/bin/env python3
"""Tables for the lever-arm study (experiments/eskf_lever.cpp). Standard library only.

    eskf_lever_report.py PREFIX [PREFIX ...]      # reads PREFIX_runs.csv and PREFIX_agg.csv (or the .csv.gz of each)

PREFIX is OUT/<suite>. Two parts:

  1. The live tables: S1s, S2s and S8u, the schemes that this tree's harness writes (EskfParams defaults; S8u is the code that ships),
     per suite, with the zero lever ("Z") of the same data beside them, and under the heading "Numbers quoted in README.md and
     experiments/README.md" every count that the two texts quote about them, the comparison of round 10 with round 9 and of round 9 with
     round 8, and a block of frozen cells (the counts about retired schemes that README.md and experiments/README.md quote, read from the
     committed rows). A run of a subset of the suites leaves out the tables of the others. The counts of the removed switches that are not in
     the frozen cells (the iterated update, the rate gates, the consider masks, the cells of E, the slope reseed) are in HISTORY.md and in
     the report at commit 0ef1add.
  2. One frozen ablation table: every other scheme in the CSVs (S0 to S8x, S6vb, ignore, S8u@2011666 ...), per suite, read from the committed
     rows as they are. Those rows were written by the harness and the core of commit 2011666 (see experiments/README.md); this tree
     cannot regenerate them. The earlier tables of each pass (by gyro bias, by antenna, selection criteria, NEES bands, the paired
     comparison, the sweeps) are in the history of this file (git log -- experiments/eskf_lever_report.py) and in
     experiments/results/eskf_lever/HISTORY.md.

Words used in the tables: tz = tilt > 0.05 rad or |z| > 0.5 m at an epoch (outside the dropout and reseed windows); z>1 = |z| > 1 m;
circ = the position is beyond the circle of radius |t_xy| around the antenna, plus 0.3 m; xy>1 and xy>1.5 = the horizontal error passes
1 m or 1.5 m; head = the heading error at the end passes 0.5 rad; excess = the run has tz and the zero-lever run of the same data has
none. Antennas: A (0.5, 0, 1.0) and B (-0.3, 0.2, 1.0) have a horizontal arm; M is the mast (no horizontal arm); Z is the zero lever.
"""
import csv
import gzip
import os
import sys
from collections import defaultdict

LIVE = ('S1s', 'S2s', 'S8u')
OLD8 = 'S8u@0ef1add'   # the S8u rows of the core of round 8, kept where round 9 changed any row
OLD9 = 'S8u@b921395'   # the S8u rows of the core of round 9 (fast-turn guard, variance restore), kept where round 10 changed any row
ABM = ('A', 'B', 'M')
ARMS = (('A, B', ('A', 'B')), ('M (mast)', ('M',)), ('Z (zero lever)', ('Z',)))
SUITE_ORDER = ('g60', 'g300', 'h300', 'h600', 'h900', 'h1800', 'hndt', 'hslope', 'hyaw', 'hvib', 'hsg', 'hsgv', 'hgap', 'hboot', 'hrate', 'hratex',
               'hphase', 'hgapsweep')


def load(prefixes):
    runs = []
    for p in prefixes:
        f = open(p + '_runs.csv') if os.path.exists(p + '_runs.csv') else gzip.open(p + '_runs.csv.gz', 'rt')
        suite = os.path.basename(p)
        for row in csv.DictReader(f):
            if None not in row.values():   # a line cut short by a run still writing
                row['suite'] = suite
                row['gid'] = int(row['cfg']) // 4   # the four antennas of a configuration share their data
                runs.append(row)
    return runs


def num(r, k):
    return float(r[k])


def tz(r):
    return num(r, 'max_tilt') > 0.05 or num(r, 'max_z') > 0.5


def z1(r):
    return num(r, 'max_z') > 1.0


def tilt3(r):
    return num(r, 'max_tilt') > 0.3


def circ(r):
    return num(r, 'max_circ') > 0.0


def xy1(r):
    return num(r, 'max_xy') > 1.0


def xy15(r):
    return num(r, 'max_xy') > 1.5


def head(r):
    return num(r, 'end_head') > 0.5


def stand_s(r):
    return int(r['dur']) - 160   # the suites with a stand: 160 s of motion after it


def fix_period(r):
    n = r['noise']
    return float(n[3:-1]) if n.startswith('fix') else 0.2   # the noise column reads fix<period>s; "matched" is the 5 Hz of every earlier suite


def hz(r):
    return '%.3g Hz' % (1.0 / fix_period(r))


def kind(r):   # the cells of hrate: boot (gyro bias 0.1 deg/s and accelerometer bias 0.1 m/s^2, stand of 6, 30 or 300 s) or stand-then-go
    return 'boot' if stand_s(r) in (6, 30, 300) and r['gyro_bias_dps'] == '0.1' and r['acc_bias'] == '0.1' else 'stand-then-go'


class Zero:
    """The zero-lever run of the same data (the four antennas of a configuration share it), for the excess column."""

    def __init__(self, rows):
        self.z = {}
        self.differs = 0
        same = {}
        for r in rows:
            if r['antenna'] == 'Z':
                self.z.setdefault((r['suite'], r['gid'], r['seed']), r)
                k = (r['suite'], r['cfg'], r['seed'])   # the same configuration and seed under another scheme: the same run
                if k in same and any(same[k][c] != r[c] for c in ('max_xy', 'max_z', 'max_tilt', 'end_head')):
                    self.differs += 1
                same.setdefault(k, r)

    def of(self, r):
        return self.z.get((r['suite'], r['gid'], r['seed']))

    def excess(self, rs):
        n = 0
        for r in rs:
            z = self.of(r)
            n += tz(r) and z is not None and not tz(z)
        return n


def cells(rs, zero):
    n = len(rs)
    return (f'{n} | {sum(map(tz, rs))} | {sum(map(z1, rs))} | {sum(map(tilt3, rs))} | {sum(map(circ, rs))} | {sum(map(xy1, rs))} | {sum(map(xy15, rs))} | '
            f'{sum(map(head, rs))} | {sum(r["diverged"] == "1" for r in rs)} | {zero.excess(rs) if rs and rs[0]["antenna"] != "Z" else "-"}')


MASTZ = {}   # (table, slice) -> {'M': tz of the mast under S8u, 'Z': tz of the zero lever}, filled by live_table, read by quoted

HDR = '| slice | antennas | scheme | runs | tz | z>1 | tilt>0.3 | circ | xy>1 | xy>1.5 | head | diverged | excess |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|'


def live_table(title, rows, keyfn, keys, zero, note=None):
    """One row per slice, antenna class and live scheme. The zero lever does not depend on the scheme: one row (the first live scheme present)."""
    print(f'\n### {title}\n')
    if note:
        print(note + '\n')
    print(HDR)
    schemes = [s for s in LIVE if any(r['scheme'] == s for r in rows)]
    for k in keys:
        sel = [r for r in rows if keyfn(r) == k]
        for label, ants in ARMS:
            for sc in (schemes if ants != ('Z',) else schemes[:1]):
                rs = [r for r in sel if r['scheme'] == sc and r['antenna'] in ants]
                if rs:
                    print(f'| {k} | {label} | {sc if ants != ("Z",) else "(any)"} | {cells(rs, zero)} |')
                    if ants == ('M',) and sc == 'S8u':
                        MASTZ.setdefault((title, k), {})['M'] = sum(map(tz, rs))
                    if ants == ('Z',):
                        MASTZ.setdefault((title, k), {})['Z'] = sum(map(tz, rs))


def keys_of(rows, keyfn, order=None):
    ks = {keyfn(r) for r in rows}
    return sorted(ks, key=order) if order else sorted(ks)


def pooled(rows, zero):
    """The observability grid (h300 to h1800, GNSS and IMU only) and its small companions: by motion and duration, and the other suites in one row each."""
    for su in ('h300', 'h600', 'h900', 'h1800', 'hndt', 'hslope', 'hyaw', 'hvib'):
        rs = [r for r in rows if r['suite'] == su]
        if not rs:
            continue
        if su == 'hndt':
            by = lambda r: 'NDT fixes on'
        elif su == 'hyaw':
            by = lambda r: f'yaw0 {r["yaw0"]} rad'
        elif su == 'hvib':
            by = lambda r: f'noise {r["noise"]}'
        elif su == 'hslope':
            by = lambda r: f'{r["motion"]}'
        else:
            by = lambda r: f'{r["motion"]}, GNSS and IMU only' if r['ndt'] == '0' else f'{r["motion"]}, NDT on'
        live_table(f'Suite {su}', rs, by, keys_of(rs, by), zero)


def stand_then_move(rows, zero):
    """hsg and hsgv: stand 300 to 1,800 s, then start, circle or turn on the spot for 160 s (the bar of eskf_lever.cpp)."""
    for su in ('hsg', 'hsgv'):
        rs = [r for r in rows if r['suite'] == su]
        if not rs:
            continue
        print(f'\n## Suite {su}: stand for minutes, then move (sgstart, sgdrive, sgturn)\n')
        live_table(f'{su} by motion', rs, lambda r: r['motion'], keys_of(rs, lambda r: r['motion']), zero)
        live_table(f'{su} by standing time', rs, lambda r: f'stand {stand_s(r)} s', keys_of(rs, lambda r: f'stand {stand_s(r)} s', lambda k: int(k.split()[1])), zero)
        live_table(f'{su} by gyro bias', rs, lambda r: f'gyro bias {r["gyro_bias_dps"]} deg/s', keys_of(rs, lambda r: f'gyro bias {r["gyro_bias_dps"]} deg/s', lambda k: float(k.split()[2])), zero)


def outages(rows, zero):
    """hgap: the fixes stop before the first start (long: 30 s and 120 s from 10 s before the start; short: 1.5 to 12 s)."""
    gap = [r for r in rows if r['suite'] == 'hgap']
    if not gap:
        return
    longr = [r for r in gap if not r['noise'].startswith('off')]
    short = [r for r in gap if r['noise'].startswith('off')]
    print('\n## Suite hgap: an outage of the fixes at the first start\n')
    for name, rs in (('long outages (30 s and 120 s, from 10 s before the start)', longr), ('short outages (1.5 to 12 s, just before or in the middle of the first acceleration)', short)):
        if not rs:
            continue
        by = lambda r: f'outage {r["drop_s"]} s'
        live_table(f'hgap, {name}, by length', rs, by, keys_of(rs, by, lambda k: float(k.split()[1])), zero)
        by = lambda r: f'{r["motion"]}, {r["noise"]}'
        if rs is short:
            live_table(f'hgap, {name}, by start of the outage', rs, by, keys_of(rs, by), zero)
        by = lambda r: f'stand {stand_s(r)} s, gyro bias {r["gyro_bias_dps"]} deg/s'
        live_table(f'hgap, {name}, by stand and gyro bias', rs, by, keys_of(rs, by), zero)


def boot(rows, zero):
    bt = [r for r in rows if r['suite'] == 'hboot']
    if not bt:
        return
    print('\n## Suite hboot: initial_yaw off by 0.5 to 3 rad, no /initialpose, a stand of 6 to 300 s, then the start\n')
    for title, by in (('hboot by start error', lambda r: f'yaw0 {r["yaw0"]} rad'), ('hboot by motion', lambda r: r['motion']),
                      ('hboot by standing time', lambda r: f'stand {stand_s(r)} s')):
        live_table(title, bt, by, keys_of(bt, by), zero)


def rates(rows, zero):
    rt = [r for r in rows if r['suite'] in ('hrate', 'hratex')]
    if not rt:
        return
    print('\n## Suites hrate and hratex: boot and stand-then-go cells at fix rates of 0.5 to 100 Hz (the IMU data of a cell is the same at every rate)\n')
    by = lambda r: f'{hz(r)}, {kind(r)}'
    order = lambda k: (-float(k.split(' Hz')[0]), k)
    live_table('Antennas by fix rate and cell', rt, by, keys_of(rt, by, order), zero)
    boot2 = [r for r in rt if kind(r) == 'boot' and r['yaw0'] == '2']
    by = lambda r: f'{hz(r)}, boot, initial_yaw 2 rad off'
    live_table('The boot with initial_yaw 2 rad off', boot2, by, keys_of(boot2, by, order), zero)


def phases(rows, zero):
    ph = [r for r in rows if r['suite'] == 'hphase']
    if not ph:
        return
    print('\n## Suite hphase: the cells of hrate at 1, 2, 5 and 10 Hz with the fixes at ten phases of the fix period after the whole seconds on which the motion starts\n')
    print('Antennas A and B, scheme S8u; the same IMU data at every phase of a cell; boot = 144 runs per phase, stand-then-go = 128.\n')
    print('| fix period | phase (tenths) | boot: tz / head / z>1 (runs) | stand-then-go: tz / head / z>1 (runs) | largest height error m |\n|---|---|---|---|---|')
    ab = [r for r in ph if r['scheme'] == 'S8u' and r['antenna'] in ('A', 'B')]
    periods = sorted({r['noise'].split('i')[0][1:] for r in ab}, key=lambda x: -float(x))
    for per in periods:
        for idx in range(10):
            out, zmax = [], 0.0
            for kd in ('boot', 'stand-then-go'):
                rs = [r for r in ab if r['noise'] == f'p{per}i{idx}' and kind(r) == kd]
                out.append(f'{sum(map(tz, rs))} / {sum(map(head, rs))} / {sum(map(z1, rs))} ({len(rs)})')
                zmax = max([zmax] + [num(r, 'max_z') for r in rs])
            print(f'| {per} s | {idx} | {out[0]} | {out[1]} | {zmax:.2f} |')


def rounds(rows):
    """Round 10 against round 9 and round 9 against round 8. Round 10 cut the fast-turn guard of the alignment and the variance restore at
    the release; round 9 seated the position at the alignment and allowed more stamp jitter in the window check. In a suite where a round
    changed no row, nothing was renamed: the S8u rows are then the rows of the earlier core as well."""
    has9 = {r['suite'] for r in rows if r['scheme'] == OLD9}
    for title, old, name_new in (('Round 10 against round 9: S8u now, and ' + OLD9 + ' (the core of round 9: fast-turn guard and variance restore)', OLD9, lambda su: 'S8u'),
                                 ('Round 9 against round 8: ' + OLD9 + ' (or S8u where round 10 changed no row), and ' + OLD8 + ' (the core of round 8)', OLD8,
                                  lambda su: OLD9 if su in has9 else 'S8u')):
        olds = {r['suite'] for r in rows if r['scheme'] == old}
        if not olds:
            continue
        newlab = 'now' if old == OLD9 else 'round 9'
        oldlab = 'round 9' if old == OLD9 else 'round 8'
        print(f'\n## {title}, by suite\n')
        print(f'Same configurations, same seeds, same data; only the core differs. `{old}` rows are the committed rows of the older core.\n')
        print(f'| suite | antennas | runs | tz {oldlab} / {newlab} | z>1 {oldlab} / {newlab} | head {oldlab} / {newlab} | xy>1.5 {oldlab} / {newlab} | circ {oldlab} / {newlab} |\n|---|---|---|---|---|---|---|---|')
        for su in SUITE_ORDER:
            if su not in olds:
                continue
            for label, ants in ARMS[:2]:
                a = [r for r in rows if r['suite'] == su and r['scheme'] == old and r['antenna'] in ants]
                b = [r for r in rows if r['suite'] == su and r['scheme'] == name_new(su) and r['antenna'] in ants]
                cell = lambda f: f'{sum(map(f, a))} / {sum(map(f, b))}'
                print(f'| {su} | {label} | {len(b)} | {cell(tz)} | {cell(z1)} | {cell(head)} | {cell(xy15)} | {cell(circ)} |')
        # by fix rate and by phase, antennas A and B and the mast
        for t2, suites, keyf in (('hrate and hratex by fix rate', ('hrate', 'hratex'), hz), ('hphase by fix period', ('hphase',), lambda r: r['noise'].split('i')[0][1:] + ' s')):
            sel = [r for r in rows if r['suite'] in suites and r['antenna'] in ('A', 'B', 'M')]
            if not any(r['scheme'] == old for r in sel):
                continue
            print(f'\n### {t2} ({oldlab} against {newlab})\n')
            print(f'| fix | antennas | runs | tz {oldlab} / {newlab} | head {oldlab} / {newlab} | xy>1.5 {oldlab} / {newlab} |\n|---|---|---|---|---|---|')
            keys = sorted({keyf(r) for r in sel}, key=lambda k: -float(k.split()[0]))
            for k in keys:
                for label, ants in ARMS[:2]:
                    a = [r for r in sel if keyf(r) == k and r['scheme'] == old and r['antenna'] in ants]
                    b = [r for r in sel if keyf(r) == k and r['scheme'] == name_new(r['suite']) and r['antenna'] in ants]
                    if a and b:
                        cell = lambda f: f'{sum(map(f, a))} / {sum(map(f, b))}'
                        print(f'| {k} | {label} | {len(b)} | {cell(tz)} | {cell(head)} | {cell(xy15)} |')


def quoted(rows, zero):
    """Every count that README.md and experiments/README.md quote about S1s, S2s and S8u (the retired schemes: the frozen table)."""
    print('\n## Numbers quoted in README.md and experiments/README.md\n')
    S = lambda su, sc, ants, pick=lambda r: True: [r for r in rows if r['suite'] == su and r['scheme'] == sc and r['antenna'] in ants and pick(r)]
    n = lambda rs, pred: sum(map(pred, rs))
    div = lambda r: num(r, 'max_z') > 100.0 or num(r, 'max_tilt') > 1.5
    # Fix rates, antennas A and B
    for su in ('hrate', 'hratex'):
        rs_all = S(su, 'S8u', ('A', 'B'))
        for per in sorted({fix_period(r) for r in rs_all}, reverse=True):
            rs = [r for r in rs_all if fix_period(r) == per]
            b2 = [r for r in rs if kind(r) == 'boot' and r['yaw0'] == '2']
            sg = [r for r in rs if kind(r) == 'stand-then-go']
            print(f'- {su}, {hz(rs[0])} (a fix every {per:g} s), S8u, antennas A and B, {len(rs)} runs: tz {n(rs, tz)}, z>1 {n(rs, z1)}, head {n(rs, head)}, xy>1.5 {n(rs, xy15)}; '
                  f'boot with initial_yaw 2 rad off ({len(b2)} runs): tz {n(b2, tz)}, head {n(b2, head)}; stand-then-go ({len(sg)}): tz {n(sg, tz)}, head {n(sg, head)}.')
    # Stand, then move
    for su in ('hsg', 'hsgv'):
        for sc in ('S2s', 'S8u'):
            lr = S(su, sc, ('A', 'B', 'M'))
            if not lr:
                continue
            print(f'- {su}, {sc}, antennas A, B, M: {len(lr)} runs, tz {n(lr, tz)}, excess {zero.excess(lr)}, z>1 {n(lr, z1)}, tilt>0.3 {n(lr, tilt3)}, head {n(lr, head)}, xy>1 {n(lr, xy1)}.')
            for m in ('sgstart', 'sgdrive', 'sgturn'):
                rm = [r for r in lr if r['motion'] == m]
                print(f'  - {m}: {len(rm)} runs; head {n(rm, head)}; xy>1 {n(rm, xy1)}; tz {n(rm, tz)}; excess {zero.excess(rm)}.')
        zr = S(su, 'S8u', ('Z',))
        if zr:
            print(f'- {su}, zero lever ({len(zr)} unique runs): tz {n(zr, tz)}, head {n(zr, head)}, xy>1 {n(zr, xy1)}; turning on the spot: head {n([r for r in zr if r["motion"] == "sgturn"], head)} of '
                  f'{len([r for r in zr if r["motion"] == "sgturn"])}.')
    # Boot
    for sc in ('S8u',):
        lr = S('hboot', sc, ('A', 'B', 'M'))
        if lr:
            for label, ants in (('A and B', ('A', 'B')), ('the mast M', ('M',)), ('zero lever', ('Z',))):
                rs = S('hboot', sc, ants)
                print(f'- hboot, {sc}, {label}: {len(rs)} runs, tz {n(rs, tz)}, head {n(rs, head)}, z>1 {n(rs, z1)}.')
    # Outages
    for su_kind, pick in (('short', lambda r: r['noise'].startswith('off')), ('long', lambda r: not r['noise'].startswith('off'))):
        for label, ants in (('antennas A and B', ('A', 'B')), ('the mast M', ('M',)), ('zero lever', ('Z',))):
            rs = S('hgap', 'S8u', ants, pick)
            if rs:
                print(f'- hgap, {su_kind} outages, S8u, {label}: {len(rs)} runs, tz {n(rs, tz)}, z>1 {n(rs, z1)}, xy>1.5 {n(rs, xy15)}, diverged (|z| > 100 m or tilt > 1.5 rad) {n(rs, div)}, '
                      f'largest height error {max(num(r, "max_z") for r in rs):.2f} m.')
    hard = lambda r: r['gyro_bias_dps'] == '0.2' and stand_s(r) == 900
    rs = S('hgap', 'S8u', ('M',), lambda r: r['noise'].startswith('off') and hard(r))
    zs = S('hgap', 'S8u', ('Z',), lambda r: r['noise'].startswith('off') and hard(r))
    if rs:
        print(f'- hgap, short outages, S8u, mast M, stand 900 s at 0.2 deg/s ({len(rs)} runs): tz {n(rs, tz)}, z>1 {n(rs, z1)}, diverged {n(rs, div)}; the zero lever on the same data has tz in '
              f'{n(zs, tz)} of {len(zs)}; excess {zero.excess(rs)}.')
    circle = S('hgap', 'S8u', ('A', 'B'), lambda r: r['motion'] == 'sgdrive' and r['noise'] == 'off-1')
    for ant in ('A', 'B'):
        ca = [r for r in circle if r['antenna'] == ant]
        if ca:
            print(f'- hgap, S8u, antenna {ant}, the outage begins 1 s before the circle ({len(ca)} runs): head {n(ca, head)}, xy>1.5 {n(ca, xy15)}.')
    # Phase of the fixes
    ph = S('hphase', 'S8u', ('A', 'B'))
    for per in sorted({r['noise'].split('i')[0][1:] for r in ph}, key=lambda x: -float(x)):
        rs = [r for r in ph if r['noise'].startswith(f'p{per}i')]
        p0 = [r for r in rs if r['noise'] == f'p{per}i0']
        worst = max((sum(map(tz, [r for r in rs if r['noise'] == f'p{per}i{i}'])) for i in range(10)))
        print(f'- hphase, {per} s, S8u, antennas A and B: {len(rs)} runs over 10 phases, tz {n(rs, tz)}, head {n(rs, head)}, z>1 {n(rs, z1)}, largest height error '
              f'{max(num(r, "max_z") for r in rs):.2f} m; phase 0: tz {n(p0, tz)}; the worst phase has tz {worst}.')


def frozen_cells(rows):
    """The counts that README.md and experiments/README.md quote about the retired schemes (and the S8u rows of earlier cores), read from the
    committed rows. The counts of the removed switches (the iterated update, the rate gates, the consider masks, the cells of E of the release
    study, the slope reseed) are not here: they are in HISTORY.md, and in the report at commit 0ef1add (git show 0ef1add:experiments/results/eskf_lever/report.md)."""
    print('\n### Frozen cells quoted in README.md and experiments/README.md\n')
    print('Read from the committed rows of the schemes named; none of them can be regenerated by this tree (`S8u@0ef1add` and `S8u@b921395` are the S8u rows of the cores of rounds 8 and 9). '
          'Antennas A, B and M unless the line says otherwise; `tz` unless it says otherwise.\n')
    n = lambda rs, pred: sum(map(pred, rs))
    div = lambda r: num(r, 'max_z') > 100.0 or num(r, 'max_tilt') > 1.5
    S = lambda su, sc, ants, pick=lambda r: True: [r for r in rows if r['suite'] == su and r['scheme'] == sc and r['antenna'] in ants and pick(r)]
    have = lambda su, sc: any(r['suite'] == su and r['scheme'] == sc for r in rows)

    def line(rule, su, sc, rs, pred=tz, what='tz'):
        if rs:
            print(f'- {rule}: {su}, {sc}: {n(rs, pred)} of {len(rs)} runs with {what}.')
    for sc in ('S6vb', 'S7a35'):   # rule 2, the bound
        rs = [r for su in ('h900', 'h1800') for r in S(su, sc, ABM, lambda r: r['motion'] == 'stand')]
        line('Rule 2, bound (standing 900 s and 1,800 s)', 'h900 and h1800', sc, rs)
    for sc in ('S6vb', 'S7a35'):   # rule 3, the release
        rs = [r for su in ('h300', 'h600', 'h900', 'h1800') for r in S(su, sc, ABM, lambda r: r['ndt'] == '0')]
        line('Rule 3, release (GNSS and IMU only)', 'h300 to h1800', sc, rs)
    for rule, sc in (('Rule 5, shadow and alignment, without', 'S7a35'), ('Rule 5 and rule 6, with the shadow and the alignment, without the blind update', 'S8a'),
                     ('Rule 6, with the blind update', 'S8')):
        line(rule, 'hsg', sc, S('hsg', sc, ABM))
    for sc in ('S8o', 'S8'):   # the span guard
        line('Rule 5, span guard, outage of 120 s', 'hgap', sc, S('hgap', sc, ABM, lambda r: r['drop_s'] == '120'))
    for sc in ('S8', 'S8k', OLD8, OLD9, 'S8u'):   # rule 7
        line('Rule 7, unknown start', 'hboot', sc, S('hboot', sc, ABM))
    for sc in ('S8k', OLD8, OLD9, 'S8u'):   # rule 8, short outages at the first start
        for label, ants in (('antennas A and B', 'AB'), ('the mast M', 'M')):
            rs = S('hgap', sc, ants, lambda r: r['noise'].startswith('off'))
            if rs:
                print(f'- Rule 8, outage spread, short outages: hgap, {sc}, {label}: {n(rs, tz)} of {len(rs)} runs with tz, {n(rs, div)} diverged.')
    for sc in ('S8k', OLD8, OLD9, 'S8u'):   # the rate limit
        parts = []
        for hzv in (1, 10, 20):
            rs = S('hrate', sc, 'AB', lambda r: abs(1 / fix_period(r) - hzv) < 1e-6 and kind(r) == 'boot')
            if rs:
                parts.append(f'{hzv} Hz {n(rs, tz)} of {len(rs)}')
        if parts:
            print(f'- Spans in seconds and the rate limit, boot cells, antennas A and B, {sc}: ' + ', '.join(parts) + '.')
    # Round 10 against round 9, the cells that the texts quote one by one (README.md and experiments/README.md, the limits and the round 10 section)
    if have('h300', OLD9):
        parts = []
        for mo in ('drive', 'startstop', 'turn', 'spin'):
            a9, a10 = S('h300', OLD9, 'AB', lambda r: r['motion'] == mo), S('h300', 'S8u', 'AB', lambda r: r['motion'] == mo)
            parts.append(f'{mo} {n(a9, tz)} of {len(a9)} to {n(a10, tz)} of {len(a10)}')
        print('- Round 10 against round 9, h300, antennas A and B, tz per motion, ' + OLD9 + ' to S8u: ' + ', '.join(parts) + '.')
    if have('hgap', OLD9):
        for ant in ('A', 'B'):
            a9 = S('hgap', OLD9, ant, lambda r: r['motion'] == 'sgdrive' and r['noise'] == 'off-1')
            a10 = S('hgap', 'S8u', ant, lambda r: r['motion'] == 'sgdrive' and r['noise'] == 'off-1')
            if a9 and a10:
                print(f'- Round 10 against round 9, the tight circle (hgap, sgdrive, noise off-1), antenna {ant}: head {n(a9, head)} of {len(a9)} to {n(a10, head)} of {len(a10)}, '
                      f'xy>1.5 {n(a9, xy15)} to {n(a10, xy15)}, tz {n(a9, tz)} to {n(a10, tz)}.')
    # The reseed on a slope: /initialpose takes roll and pitch from the accelerometer (S7n kept the level attitude of the message)
    for sc in ('S7n', 'S8u'):
        rs = S('hslope', sc, ABM, lambda r: r['slope'] == '0.1')
        if rs:
            print(f'- Reseed on a slope of 0.1 rad, level attitude of the message (S7n) against the accelerometer: hslope, {sc}: {n(rs, lambda r: num(r, "end_tilt") > 0.02)} of {len(rs)} runs '
                  'with a tilt over 0.02 rad at the end.')
    # The observability grid with a lever arm (README.md, limits): runs with tz, and runs with tz whose zero-lever run has none (excess)
    zero = Zero(rows)
    grid = [r for r in rows if r['suite'] in ('h300', 'h600', 'h900', 'h1800') and r['ndt'] == '0' and r['antenna'] in ABM]
    has9 = {r['suite'] for r in grid if r['scheme'] == OLD9}
    for label, pick in (('S7a35 (round 6)', lambda r: r['scheme'] == 'S7a35'),
                        ('round 9 (' + OLD9 + ' where round 10 changed a row, S8u elsewhere)', lambda r: r['scheme'] == (OLD9 if r['suite'] in has9 else 'S8u')),
                        ('S8u (shipped)', lambda r: r['scheme'] == 'S8u')):
        rs = [r for r in grid if pick(r)]
        if rs:
            ab = [r for r in rs if r['antenna'] in ('A', 'B')]
            m = [r for r in rs if r['antenna'] == 'M']
            print(f'- Observability grid, h300 to h1800, GNSS and IMU only, {label}: {len(rs)} runs, tz {n(rs, tz)}, excess {zero.excess(rs)}; antennas A and B {len(ab)} runs, tz {n(ab, tz)}, '
                  f'excess {zero.excess(ab)}; the mast {len(m)} runs, tz {n(m, tz)}, excess {zero.excess(m)}.')
    sh = [r for r in grid if r['scheme'] == 'S8u']
    ex = [r for r in sh if tz(r) and zero.of(r) is not None and not tz(zero.of(r))]
    print(f'- Observability grid, S8u: of the {len(ex)} excess runs, {sum(r["acc_bias"] == "0.3" for r in ex)} are in cells with an accelerometer bias of 0.3 m/s^2.')
    for sc in ('S8u@2011666', OLD8, OLD9, 'S8u'):   # the fix history by time
        rs = S('hratex', sc, 'AB', lambda r: abs(1 / fix_period(r) - 100) < 1e-6)
        line('Fix history by time, 100 Hz, antennas A and B', 'hratex', sc, rs)


def mast_summary():
    both = {k: v for k, v in MASTZ.items() if 'M' in v and 'Z' in v}
    worse = sorted(k for k, v in both.items() if v['M'] > v['Z'])
    print(f'- The mast (S8u) against the zero lever, over the {len(both)} slices of the tables above that carry both: the mast has more runs with tz than the zero lever in {len(worse)}'
          + (': ' + '; '.join(f'{t.replace("### ", "")}, {k} (mast {both[(t, k)]["M"]}, zero lever {both[(t, k)]["Z"]})' for t, k in worse) + '.' if worse else '.'))


def frozen(rows):
    """One table: every scheme that is not S1s, S2s or S8u, per suite, from the committed rows. The runs are the lever-arm runs (antennas A, B, M; A and B in g60, g300)."""
    print('\n## Frozen ablation: the retired schemes, read from the committed CSVs\n')
    print('Rows written by the harness and the core of commit `2011666`, except `S8u@2011666` (the S8u of that commit, in `hratex`) `S8u@0ef1add` (the S8u of the core of round 8, in the suites where round 9 changed a row) and `S8u@b921395` (the S8u of the core of round 9, in the suites where round 10 changed a row); '
          'not regenerated by this tree. '
          'Lever-arm runs only (antennas A, B, M); `ignore` and the zero lever Z: see the live tables. A scheme that a suite does not carry has no row.\n')
    print('| suite | scheme | runs | tz | z>1 | circ | xy>1 | head | diverged |\n|---|---|---|---|---|---|---|---|---|')
    for su in SUITE_ORDER:
        rs_su = [r for r in rows if r['suite'] == su and r['antenna'] != 'Z']
        names = sorted({r['scheme'] for r in rs_su if r['scheme'] not in LIVE}, key=lambda s: (s != 'ignore', s))
        for sc in names:
            rs = [r for r in rs_su if r['scheme'] == sc]
            print(f'| {su} | {sc} | {len(rs)} | {sum(map(tz, rs))} | {sum(map(z1, rs))} | {sum(map(circ, rs))} | {sum(map(xy1, rs))} | {sum(map(head, rs))} | '
                  f'{sum(r["diverged"] == "1" for r in rs)} |')


def main():
    prefixes = [a for a in sys.argv[1:] if not a.endswith('.txt')]   # the replica outputs (.txt) are committed files, not tables
    if not prefixes:
        print(__doc__)
        return 2
    rows = load(prefixes)
    zero = Zero(rows)
    live = [r for r in rows if r['scheme'] in LIVE]
    print('# Lever-arm study: tables of the schemes that this tree writes (S1s, S2s, S8u) and the frozen ablation of the others\n')
    print(f'{len(rows)} runs read from {len(prefixes)} suites; {len(live)} of them are S1s, S2s or S8u. '
          'Zero lever: the runs of one data set are identical in every scheme in all cases but ' + str(zero.differs) + '.')
    pooled(live, zero)
    stand_then_move(live, zero)
    outages(live, zero)
    boot(live, zero)
    rates(live, zero)
    phases(live, zero)
    rounds(rows)
    quoted(live, zero)
    frozen_cells(rows)
    mast_summary()
    frozen(rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
