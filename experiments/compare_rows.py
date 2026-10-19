#!/usr/bin/env python3
"""Compare the rows of one or more schemes of a new run with the committed rows of the same suite. Standard library only.

    compare_rows.py COMMITTED_PREFIX NEW_PREFIX SCHEME[,SCHEME...] [KIND...]

PREFIX is <dir>/<suite>; the files are PREFIX_runs.csv and PREFIX_agg.csv (or the .csv.gz of each). KIND is runs, agg or both
(default both). Rows are matched on (configuration, scheme, seed) for runs and (configuration, scheme) for agg, and compared in
every column. Prints how many rows of the new run exist in the committed file and how many of those are identical. Exit status 1
if any matched row differs or no row matched.

The committed rows of S1s, S2s and S8u come from the core at commit 2011666 (with the switches that the core no longer has);
the rows that the current core writes must equal them wherever the data is the same.
"""
import csv
import gzip
import os
import sys


def rows(prefix, kind):
    name = prefix + '_' + kind + '.csv'
    f = open(name) if os.path.exists(name) else gzip.open(name + '.gz', 'rt')
    out = {}
    for r in csv.DictReader(f):
        if None in r.values():
            continue
        key = (r['cfg'], r['scheme']) + ((r['seed'],) if kind == 'runs' else ())
        out[key] = r
    return out


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    old_p, new_p, schemes = sys.argv[1], sys.argv[2], set(sys.argv[3].split(','))
    kinds = [k for k in sys.argv[4:] if k in ('runs', 'agg')] or ['runs', 'agg']
    status = 0
    for kind in kinds:
        old, new = rows(old_p, kind), rows(new_p, kind)
        for sc in sorted(schemes):
            keys = [k for k in new if k[1] == sc]
            matched = [k for k in keys if k in old]
            same = sum(1 for k in matched if all(old[k][c] == new[k][c] for c in new[k]))
            print(f'{os.path.basename(old_p)} {kind} {sc}: {len(keys)} new rows, {len(matched)} in the committed file, {same} identical')
            if not matched or same != len(matched):
                status = 1
                bad = [k for k in matched if any(old[k][c] != new[k][c] for c in new[k])][:3]
                for k in bad:
                    print('  differs:', k, [(c, old[k][c], new[k][c]) for c in new[k] if old[k][c] != new[k][c]][:4])
    return status


if __name__ == '__main__':
    sys.exit(main())
