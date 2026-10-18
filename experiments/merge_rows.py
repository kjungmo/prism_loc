#!/usr/bin/env python3
"""merge_rows.py MAIN_DIR SUP_DIR OUT_DIR [SUITE ...]

Append the rows of schemes that were run later (LEVER_ONLY=S8 ...) to the rows of their configuration, as a full run writes them: the
order within a configuration is the order of the scheme list. MAIN_DIR has SUITE_{runs,agg}.csv or .csv.gz, SUP_DIR has SUITE/SUITE_{runs,agg}.csv
(or SUITE_{runs,agg}.csv) with only the added rows. Writes OUT_DIR/SUITE_{runs,agg}.csv. A scheme already in MAIN_DIR is an error.
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
    main_dir, sup_dir, out = sys.argv[1:4]
    suites = sys.argv[4:] or DEFAULT
    os.makedirs(out, exist_ok=True)
    for suite in suites:
        for kind in ('runs', 'agg'):
            a = read(f'{main_dir}/{suite}_{kind}.csv')
            sub = f'{sup_dir}/{suite}/{suite}_{kind}.csv'
            b = read(sub if os.path.exists(sub) else f'{sup_dir}/{suite}_{kind}.csv')
            assert a[0] == b[0], (suite, kind)
            scheme = a[0].index('scheme')
            have = {r[scheme] for r in a[1:]}
            assert not any(r[scheme] in have for r in b[1:]), f'{suite}: a scheme to add is already there'
            by, order = {}, []
            for row in a[1:]:
                if row[0] not in by:
                    by[row[0]] = []
                    order.append(row[0])
                by[row[0]].append(row)
            for row in b[1:]:
                by[row[0]].append(row)
            with open(f'{out}/{suite}_{kind}.csv', 'w', newline='') as f:
                w = csv.writer(f, lineterminator='\n')
                w.writerow(a[0])
                for cfg in sorted(order, key=int):
                    for row in by[cfg]:
                        w.writerow(row)
            print(suite, kind, len(a) - 1, '+', len(b) - 1)


if __name__ == '__main__':
    main()
