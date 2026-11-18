#!/usr/bin/env python3
"""amcl_run_stata.py WORK_DIR OUT_DIR [JOBS]: nav2_amcl on MIT Stata (INIT_PROTOCOL_AMCL.md).

WORK_DIR from `mit_stata_laser2d dump` + amcl_prepare_stata.py (ql_/qs_ files). Runs
 - tracking: AMCL seeded at the ground truth (spread 0.5 m, 0.5 m, 0.26173 rad) on the long (ql, 1403 s) and
   short (qs, 119 s) query, 3 seeds -> OUT_DIR/track_<q>_s<seed>.csv
 - init levels: L0-L6 at the poses of ql_jobs.csv (laser2d's), U and U20k (global localization), 4 starts x
   3 seeds -> OUT_DIR/amcl_<L>_st<start>_s<seed>.csv (180 s each)
Existing CSVs are skipped. Needs `micromamba run -n ros2_humble`."""
import csv, os, queue, subprocess, sys, threading

here = os.path.dirname(os.path.abspath(__file__))
run_py = os.path.join(here, '..', 'nclt', 'amcl_run.py')
work, out = sys.argv[1], sys.argv[2]
njobs = int(sys.argv[3]) if len(sys.argv) > 3 else 3
os.makedirs(out, exist_ok=True)
base = ['micromamba', 'run', '-n', 'ros2_humble', 'python3', run_py]
jobs = []


def add(tag, cmd):
    if not os.path.exists(tag + '.csv'):
        jobs.append((tag, cmd + ['--out', tag + '.csv.part']))


for q in ('qs', 'ql'):
    for seed in (42, 1, 2):
        import numpy as np
        g = np.load(os.path.join(work, '%s_s%d.npz' % (q, seed)))['gt'][0]
        add(os.path.join(out, 'track_%s_s%d' % (q, seed)),
            base + ['--spec', os.path.join(work, '%s_s%d.npz' % (q, seed)), '--map', os.path.join(work, q + '.yaml'),
                    '--init', 'pose', '--pose', repr(g[0]), repr(g[1]), repr(g[2]), '--cov', '0.5', '0.5', '0.26173', '--wait_s', '5'])
starts = sorted({int(r['start_s']) for r in csv.DictReader(open(os.path.join(work, 'ql_jobs.csv')))})
rows = list(csv.DictReader(open(os.path.join(work, 'ql_jobs.csv'))))
for r in rows:
    spec = os.path.join(work, 'ql_s%s.npz' % r['seed'])
    c = base + ['--spec', spec, '--map', os.path.join(work, 'ql.yaml'), '--i0', r['i0'], '--i1', r['i1'], '--init', 'pose',
                '--pose', r['x'], r['y'], r['yaw'], '--cov', r['sx'], r['sy'], r['sa'], '--wait_s', '5']
    add(os.path.join(out, 'amcl_%s_st%s_s%s' % (r['level'], r['start_s'], r['seed'])), c)
for st in starts:
    r0 = [r for r in rows if int(r['start_s']) == st][0]
    for seed in (42, 1, 2):
        spec = os.path.join(work, 'ql_s%d.npz' % seed)
        c = base + ['--spec', spec, '--map', os.path.join(work, 'ql.yaml'), '--i0', r0['i0'], '--i1', r0['i1'], '--init', 'global', '--wait_s', '5']
        add(os.path.join(out, 'amcl_U_st%d_s%d' % (st, seed)), c)
        add(os.path.join(out, 'amcl_U20k_st%d_s%d' % (st, seed)), c + ['--min_particles', '2000', '--max_particles', '20000'])
print(len(jobs), 'runs', file=sys.stderr)
q = queue.Queue()
for j in sorted(jobs, key=lambda j: 'track_ql' in j[0]):  # long tracking runs last
    q.put(j)


def worker(w):
    while True:
        try:
            tag, cmd = q.get_nowait()
        except queue.Empty:
            return
        r = subprocess.run(cmd + ['--domain', str(40 + w)], capture_output=True, text=True)
        last = (r.stderr.strip().splitlines() or [''])[-1]
        if r.returncode == 0 and os.path.exists(tag + '.csv.part'):
            os.rename(tag + '.csv.part', tag + '.csv')
            open(tag + '.log', 'w').write(last + '\n')
        else:
            open(tag + '.err', 'w').write(r.stderr[-2000:])
        print(os.path.basename(tag), r.returncode, last, flush=True)


ts = [threading.Thread(target=worker, args=(w,)) for w in range(njobs)]
[t.start() for t in ts]
[t.join() for t in ts]
