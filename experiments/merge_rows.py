#!/usr/bin/env python3
"""merge_rows.py [--replace S8,S8u] MAIN_DIR SUP_DIR OUT_DIR [SUITE ...]

Append the rows of schemes that were run later (LEVER_ONLY=S8 ...) to the rows of their configuration, as a full run writes them: the
order within a configuration is the order of the scheme list. MAIN_DIR has SUITE_{runs,agg}.csv or .csv.gz, SUP_DIR has SUITE/SUITE_{runs,agg}.csv
(or SUITE_{runs,agg}.csv) with only the added rows. Writes OUT_DIR/SUITE_{runs,agg}.csv. A scheme already in MAIN_DIR is an error,
unless it is named in --replace: its rows in MAIN_DIR are then dropped and the rows of SUP_DIR take their place (a scheme whose code changed
and that was run again). For each replaced scheme the script prints how many of the new rows are identical, field by field, to the ones they
replace: where the change in the code cannot act (the data has no gap in the fixes, for the alignment guard of the fifth round) the count is all of them.
"""
import csv
import gzip
import os
import sys

DEFAULT = ['hslope', 'hyaw', 'hndt', 'hvib', 'h1800', 'h900', 'h600', 'h300']


def read(path):
    if os.path.exists(path):
        return list(csv.reader(open(path)))
    return list(csv.reader(gzip.open(path + '.gz', 'rt')))


def main():
    args = sys.argv[1:]
    replace = set()
    if args and args[0] == '--replace':
        replace = set(args[1].split(','))
        args = args[2:]
    main_dir, sup_dir, out = args[:3]
    suites = args[3:] or DEFAULT
    os.makedirs(out, exist_ok=True)
    for suite in suites:
        for kind in ('runs', 'agg'):
            a = read(f'{main_dir}/{suite}_{kind}.csv')
            sub = f'{sup_dir}/{suite}/{suite}_{kind}.csv'
            b = read(sub if os.path.exists(sub) else f'{sup_dir}/{suite}_{kind}.csv')
            assert a[0] == b[0], (suite, kind)
            scheme = a[0].index('scheme')
            seed = a[0].index('seed') if 'seed' in a[0] else None
            key = lambda row: (row[0], row[scheme], row[seed] if seed is not None else '')
            dropped = {key(r): r for r in a[1:] if r[scheme] in replace}
            a = [a[0]] + [r for r in a[1:] if r[scheme] not in replace]
            have = {r[scheme] for r in a[1:]}
            assert not any(r[scheme] in have for r in b[1:]), f'{suite}: a scheme to add is already there'
            by, order = {}, []
            for row in a[1:]:
                if row[0] not in by:
                    by[row[0]] = []
                    order.append(row[0])
                by[row[0]].append(row)
            for row in b[1:]:
                if row[0] not in by:   # a configuration whose other schemes were all replaced
                    by[row[0]] = []
                    order.append(row[0])
                by[row[0]].append(row)
            with open(f'{out}/{suite}_{kind}.csv', 'w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(a[0])
                for cfg in sorted(order, key=int):
                    for row in by[cfg]:
                        w.writerow(row)
            msg = f'{suite} {kind} {len(a) - 1} + {len(b) - 1}'
            if dropped:
                same = {}
                for r in b[1:]:
                    if r[scheme] in replace:
                        same.setdefault(r[scheme], [0, 0])
                        same[r[scheme]][1] += 1
                        same[r[scheme]][0] += dropped.get(key(r)) == r
                had = {k[1] for k in dropped}
                msg += '; ' + ', '.join((f'{k}: {v[0]} of {v[1]} rows identical to the rows they replace' if k in had else f'{k}: {v[1]} rows added')
                                        for k, v in sorted(same.items()))
            print(msg)


if __name__ == '__main__':
    main()
