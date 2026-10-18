#!/usr/bin/env python3
"""r7_merge.py COMMITTED_DIR FINAL_DIR OUT_DIR : the result files of round 7.
For the suites whose rows of S8u were run again (h300 ... hvib, hsg, hsgv): the rows of the committed S8u (the code of the rounds 5 and 6, the scheme S8k of round 7, bit for bit,
which this script checks on the rows of the suites where S8k was run too) are kept under the name S8k, and the new rows of S8u take the place of the old ones. The rows of S8 that were run
again (hslope, hsg, hsgv) must equal the committed ones (printed). hgap, hboot, hrate come whole from FINAL_DIR. The first pass is not touched."""
import csv
import gzip
import os
import sys

committed, final, out = sys.argv[1:4]
os.makedirs(out, exist_ok=True)
SECOND = ['hslope', 'hyaw', 'hndt', 'hvib', 'h1800', 'h900', 'h600', 'h300', 'hsg', 'hsgv']
WHOLE = ['hboot', 'hgap', 'hrate']


def read(path):
    if os.path.exists(path):
        return list(csv.reader(open(path)))
    return list(csv.reader(gzip.open(path + '.gz', 'rt')))


def write(path, rows):
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh, lineterminator='\n')
        for r in rows:
            w.writerow(r)


for suite in SECOND:
    for kind in ('runs', 'agg'):
        a = read(f'{committed}/{suite}_{kind}.csv')
        b = read(f'{final}/{suite}/{suite}_{kind}.csv')
        assert a[0] == b[0], (suite, kind)
        sc = a[0].index('scheme')
        seed = a[0].index('seed') if 'seed' in a[0] else None
        key = lambda row: (row[0], row[sc], row[seed] if seed is not None else '')
        old_u = {key(r): r for r in a[1:] if r[sc] == 'S8u'}
        old_8 = {key(r): r for r in a[1:] if r[sc] == 'S8'}
        new = [r for r in b[1:] if r[sc] in ('S8u', 'S8v')]   # S8v: S8u + the test of the norms (the sixth review's proposal)
        new8 = [r for r in b[1:] if r[sc] == 'S8']
        same8 = sum(old_8.get(key(r)) == r for r in new8)
        by, order = {}, []
        for r in a[1:]:
            if r[sc] == 'S8u':
                continue
            by.setdefault(r[0], []).append(r)
            if r[0] not in order:
                order.append(r[0])
        # S8k = the committed S8u, renamed (placed where S8u was)
        k_rows = []
        for r in a[1:]:
            if r[sc] == 'S8u':
                k = list(r)
                k[sc] = 'S8k'
                by.setdefault(r[0], []).append(k)
                k_rows.append(k)
        for r in new:
            by.setdefault(r[0], []).append(r)
            if r[0] not in order:
                order.append(r[0])
        write(f'{out}/{suite}_{kind}.csv', [a[0]] + [r for c in sorted(by, key=int) for r in by[c]])
        changed = sum(1 for r in new if r[sc] == 'S8u' and old_u.get(key(r)) is not None and [x for i, x in enumerate(old_u[key(r)]) if i != sc] != [x for i, x in enumerate(r) if i != sc])
        print(f'{suite} {kind}: S8k {len(k_rows)} rows (the committed S8u), S8u and S8v {len(new)} new rows, {changed} differ from the old S8u'
              + (f'; S8 re-run: {same8} of {len(new8)} rows identical to the committed ones' if new8 else ''))
for suite in WHOLE:
    for kind in ('runs', 'agg'):
        rows = read(f'{final}/{suite}/{suite}_{kind}.csv')
        write(f'{out}/{suite}_{kind}.csv', rows)
        print(f'{suite} {kind}: {len(rows) - 1} rows')
