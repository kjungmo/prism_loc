#!/usr/bin/env python3
"""Run the replay cases of the ndt3d status study: every case with every seed.

    ndt3d_replay_batch.py REPLAY_BINARY OUT_DIR [CASE_NAME_FILTER] [--jobs N]

For each (case, seed index): ndt3d_replay_gen.py writes the world and the cloud sequence with the
generator seed SEEDS[i][0], ndt3d_replay runs the filter with the particle-filter seed SEEDS[i][1],
and OUT_DIR/<case>_<i>.csv (one row per filter update) and <case>_<i>.sens keep the result; the
generated inputs are deleted. A finished (case, seed) is skipped. The case list below is fixed
before the runs; the rates are computed from the CSVs by ndt3d_replay_rates.py. Needs the ROS
Python environment (see ndt3d_replay_gen.py)."""
import os
import subprocess
import sys
from multiprocessing import Pool

CH = '--add-boxes 9,5,10,6;12,5.5,13,6.5;6.5,7,7.5,8;17,3,18,5;8,12,9,13 --drop-boxes 1'
CASES = {
    # name: (gen args, sim args)
    'H_base': ('', ''),
    'H_speed0.5': ('--speed 0.5 --seconds 120', ''),
    'H_speed2': ('--speed 2', ''),
    'H_speed3': ('--speed 3', ''),
    'H_sparse_4x90': ('--ring-step 4 --az 90', ''),
    'H_sparse_8x180': ('--ring-step 2 --az 180', ''),
    'H_mapnoise2': ('--map-noise 0.02', ''),
    'H_mapnoise5': ('--map-noise 0.05', ''),
    'H_range3': ('--range-noise 0.03', ''),
    'H_range3_mapnoise2': ('--range-noise 0.03 --map-noise 0.02', ''),
    'H_dyn6': ('--dyn 6', ''),
    'H_dyn15': ('--dyn 15', ''),
    'H_chg5add1drop': (CH, ''),
    'H_chg5add1drop_mapnoise2': (CH + ' --map-noise 0.02', ''),
    'H_chg2add': ('--add-boxes 9,5,10,6;12,5.5,13,6.5', ''),
    'H_chg2add_mapnoise2': ('--add-boxes 9,5,10,6;12,5.5,13,6.5 --map-noise 0.02', ''),
    'H_range5': ('--range-noise 0.05', ''),
    'H_range5_mapnoise2': ('--range-noise 0.05 --map-noise 0.02', ''),
    'H_dyn6_mapnoise2': ('--dyn 6 --map-noise 0.02', ''),
    'H_dyn15_mapnoise2': ('--dyn 15 --map-noise 0.02', ''),
    'H_edge_crop14': ('--map-crop-x 14', ''),
    'H_edge_crop10': ('--map-crop-x 10', ''),
    'H_edge_crop7': ('--map-crop-x 7', ''),
    'H_pitch0.02': ('--pitch 0.02', ''),
    'H_pitch0.05': ('--pitch 0.05', ''),
    'B_height-0.05': ('', '--base-height 0.45'),
    'B_height-0.1': ('', '--base-height 0.4'),
    'B_height-0.2': ('', '--base-height 0.3'),
    'B_height-0.5': ('', '--base-height 0.0'),
    'B_height-0.1_mapnoise2': ('--map-noise 0.02', '--base-height 0.4'),
    'B_height-0.2_mapnoise2': ('--map-noise 0.02', '--base-height 0.3'),
    'B_height-0.5_mapnoise2': ('--map-noise 0.02', '--base-height 0.0'),
    'B_height-0.5_mapnoise5': ('--map-noise 0.05', '--base-height 0.0'),
    'K_kidnap': ('--kidnap 30', ''),
    'K_kidnap_mapnoise2': ('--kidnap 30 --map-noise 0.02', ''),
    'K_kidnap_mapnoise5': ('--kidnap 30 --map-noise 0.05', ''),
    'K_kidnap_shift8': ('--kidnap 30 --kidnap-shift 8', ''),
    'K_kidnap_shift8_mapnoise2': ('--kidnap 30 --kidnap-shift 8 --map-noise 0.02', ''),
    'W_init_3_2_1': ('--init-offset 3 2 1', ''),
    'W_init_3_2_1_mapnoise2': ('--init-offset 3 2 1 --map-noise 0.02', ''),
    'W_init_3_2_1_mapnoise5': ('--init-offset 3 2 1 --map-noise 0.05', ''),
    'W_init_0.3': ('--init-offset 0.3 0 0', ''),
    'W_init_0.5': ('--init-offset 0.5 0 0', ''),
    'W_init_1.0': ('--init-offset 1.0 0 0', ''),
    'W_init_1.0y': ('--init-offset 0 1.0 0', ''),
    'W_init_5deg': ('--init-offset 0 0 0.087', ''),
    'W_init_1.0_mapnoise2': ('--init-offset 1.0 0 0 --map-noise 0.02', ''),
    'W_init_mid': ('--init-offset 1.0 -0.8 0.35', ''),
    'W_init_mid_mapnoise2': ('--init-offset 1.0 -0.8 0.35 --map-noise 0.02', ''),
    'O_offmap': ('--init-offset 100 0 0 --seconds 30', ''),
    'R_reinit_after_kidnap': ('--kidnap 30 --ip2 45 0.3 -0.2 0.1', ''),
    'R_reinit_wrong_after_kidnap': ('--kidnap 30 --ip2 45 3 2 1', ''),
}
SEEDS = [(7, 42), (21, 1), (22, 2), (23, 3)]


HERE = os.path.dirname(os.path.abspath(__file__))
BIN = OUT = None


def _init(binary, out):
    global BIN, OUT
    BIN, OUT = binary, out


def run(job):
    name, i = job
    g, s = CASES[name]
    ns, ps = SEEDS[i]
    d = f'{OUT}/_work_{name}_{i}'
    out = f'{OUT}/{name}_{i}.csv'
    if os.path.exists(out):
        return name, i, 'cached'
    subprocess.run(['python3', os.path.join(HERE, 'ndt3d_replay_gen.py'), d, '--seed', str(ns)] + g.split(), check=True, stdout=subprocess.DEVNULL)
    r = subprocess.run([BIN, f'{d}/map.bin', f'{d}/frames.bin', out + '.tmp', '--seed', str(ps), '--sens'] + s.split(),
                       check=True, capture_output=True, text=True)
    open(f'{OUT}/{name}_{i}.sens', 'w').write(r.stdout)
    os.rename(out + '.tmp', out)
    os.remove(f'{d}/frames.bin'); os.remove(f'{d}/map.bin'); os.rmdir(d)
    return name, i, 'ok'


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    njobs = int(sys.argv[sys.argv.index('--jobs') + 1]) if '--jobs' in sys.argv else 3
    if '--jobs' in sys.argv:
        args.remove(sys.argv[sys.argv.index('--jobs') + 1])
    BIN, OUT = os.path.abspath(args[0]), os.path.abspath(args[1])
    os.makedirs(OUT, exist_ok=True)
    flt = args[2] if len(args) > 2 else ''
    jobs = [(n, i) for n in CASES if flt in n for i in range(len(SEEDS))]
    with Pool(njobs, initializer=_init, initargs=(BIN, OUT)) as p:
        for n, i, st in p.imap_unordered(run, jobs):
            print(n, i, st, flush=True)
