#!/usr/bin/env python3
"""amcl_prepare_nclt.py WORK_DIR: turns the --dump files of nclt_laser2d (SEG_pri / SEG_alt) into the
.npz inputs of amcl_run.py (scan layout: 720 bins over 360 degrees from -pi, range 0.5-40 m)."""
import glob, os, sys
import numpy as np

B = 720
rec = np.dtype([('t', '<i8'), ('v', '<f8', 6), ('r', '<f4', B)])
for f in sorted(glob.glob(os.path.join(sys.argv[1], 'seg*_scans.bin'))):
    a = np.fromfile(f, dtype=rec)
    out = f.replace('_scans.bin', '.npz')
    np.savez(out, t=a['t'], odom=a['v'][:, :3], gt=a['v'][:, 3:], ranges=a['r'],
             amin=-np.pi, ainc=2 * np.pi / B, rmin=0.5, rmax=40.0)
    print(out, len(a))
