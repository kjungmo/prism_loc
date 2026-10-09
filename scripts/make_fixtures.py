#!/usr/bin/env python3
"""Write small synthetic maps for headless launch tests.

    make_fixtures.py OUT_DIR

OUT_DIR/map.pgm + map.yaml -- a 10 x 10 m walled room at 0.05 m (nav2_map_server format)
OUT_DIR/map.pcd            -- an ASCII point cloud of a 10 x 10 m floor with two 2 m walls
"""
import os
import sys


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    n = 200
    rows = []
    for r in range(n):
        rows.append(bytes(0 if r < 3 or r >= n - 3 or c < 3 or c >= n - 3 else 254 for c in range(n)))
    with open(os.path.join(out, "map.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (n, n))
        for row in rows:
            f.write(row)
    with open(os.path.join(out, "map.yaml"), "w") as f:
        f.write("image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    pts = [(i * 0.1 - 5, j * 0.1 - 5, 0.0) for i in range(100) for j in range(100)]
    for i in range(100):
        for k in range(20):
            pts.append((i * 0.1 - 5, 5.0, k * 0.1))
            pts.append((5.0, i * 0.1 - 5, k * 0.1))
    with open(os.path.join(out, "map.pcd"), "w") as f:
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n")
        for p in pts:
            f.write("%.3f %.3f %.3f\n" % p)
    print(f"wrote {out}/map.yaml, map.pgm, map.pcd ({len(pts)} points)")


if __name__ == "__main__":
    main()
