#!/usr/bin/env python3
"""amcl_run_nclt.py WORK_DIR OUT_DIR BAND [JOBS] [LEVELS]: INIT_PROTOCOL.md on NCLT with nav2_amcl.

WORK_DIR holds seg<S>_<BAND>.npz/.yaml (nclt_laser2d --dump + amcl_prepare_nclt.py), BAND = pri | alt.
Runs 4 segments x levels x 3 seeds (LEVELS default L0..L6,U,U20k); CSV per run in OUT_DIR as
amcl_<level>_t<seg>_s<seed>.csv (skipped if present). Needs `micromamba run -n ros2_humble` for amcl_run.py."""
import os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
work, out, band = sys.argv[1], sys.argv[2], sys.argv[3]
njobs = int(sys.argv[4]) if len(sys.argv) > 4 else 3
levels = sys.argv[5].split(',') if len(sys.argv) > 5 else ['L0', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'U', 'U20k']
os.makedirs(out, exist_ok=True)
D = [0, 2, 5, 10, 2, 2, 10]; A = [0, 15, 15, 15, 45, 90, 90]
offs = {}
for line in open(os.path.join(here, 'amcl_offsets_nclt.txt')):
    s, lv, sd, dx, dy, dyaw = line.split()
    offs[(int(s), int(lv), int(sd))] = (float(dx), float(dy), float(dyaw))
jobs = []
for seg in (400, 1400, 2400, 3400):
    g0 = np.load(os.path.join(work, 'seg%d_%s.npz' % (seg, band)))['gt'][0]
    for L in levels:
        for seed in (42, 1, 2):
            tag = os.path.join(out, 'amcl_%s_t%d_s%d' % (L, seg, seed))
            if os.path.exists(tag + '.csv'):
                continue
            cmd = ['micromamba', 'run', '-n', 'ros2_humble', 'python3', os.path.join(here, 'amcl_run.py'),
                   '--spec', os.path.join(work, 'seg%d_%s.npz' % (seg, band)), '--map', os.path.join(work, 'seg%d_%s.yaml' % (seg, band)),
                   '--out', tag + '.csv.part']
            if L.startswith('U'):
                cmd += ['--init', 'global']
                if L == 'U20k':
                    cmd += ['--min_particles', '2000', '--max_particles', '20000']
            else:
                lv = int(L[1])
                dx, dy, dyaw = offs[(seg, lv, seed)]
                cmd += ['--init', 'pose', '--pose', repr(g0[0] + dx), repr(g0[1] + dy), repr(float(np.arctan2(np.sin(g0[2] + dyaw), np.cos(g0[2] + dyaw)))),
                        '--cov', repr(max(0.5, D[lv])), repr(max(0.5, D[lv])), repr(max(15.0, A[lv]) * np.pi / 180)]
            jobs.append((tag, cmd))
print(len(jobs), 'runs', file=sys.stderr)


def run(k_job):
    k, (tag, cmd) = k_job
    env = dict(os.environ)
    c = cmd + ['--domain', str(20 + k % njobs)]
    r = subprocess.run(c, env=env, capture_output=True, text=True)
    last = (r.stderr.strip().splitlines() or [''])[-1]
    if r.returncode == 0 and os.path.exists(tag + '.csv.part'):
        os.rename(tag + '.csv.part', tag + '.csv')
        open(tag + '.log', 'w').write(last + '\n')
    print(os.path.basename(tag), r.returncode, last, flush=True)


# worker-affine domain ids: run k uses 20 + (k % njobs); keep at most njobs runs alive at once, each in its own domain
import itertools, queue, threading
q = queue.Queue()
for k, j in enumerate(jobs):
    q.put(j)


def worker(w):
    while True:
        try:
            tag, cmd = q.get_nowait()
        except queue.Empty:
            return
        r = subprocess.run(cmd + ['--domain', str(20 + w)], capture_output=True, text=True)
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
