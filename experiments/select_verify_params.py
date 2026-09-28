#!/usr/bin/env python3
"""Choose the relocalization-verification defaults on the CALIBRATION run.

Reads docs/paper/data/bbs_verify_calib.csv (written by
`experiments/bbs_verify.cpp calib`, seed 20261105, disjoint from the study seed
20260928), replays RelocalizationVerifier's decision rule for every
(top_k, verify_scans, evidence_gain, min_posterior) in the grid below, writes
the per-config summary to docs/paper/data/bbs_verify_selection.csv and prints
the selected configuration.

Selection rule (fixed before looking at the study data):
  1. fewest false accepts over all calibration queries (pooled over sizes);
  2. then most correct accepts;
  3. then fewest scans, then smallest K, then smallest evidence_gain, then
     largest min_posterior.

Replay of the decision rule (mirrors prism_loc_core/src/relocalization.cpp):
  hypotheses = the first K logged modes (all passed the single-scan gate);
  log-weight = gain * sum of per-scan score fractions over the first M scans
  while alive; posterior = softmax over hypotheses alive at scan M-1; best =
  first maximum; accepted iff posterior >= min_posterior and the best
  hypothesis' mean fraction >= 0.4; ambiguous iff posterior < min_posterior;
  no-candidate if no hypothesis (or none alive).
Note: the calibration logs the modes of a top-8 search; the modes of a top-K
search are taken as its first K entries (greedy NMS makes this approximate for
lower-ranked modes; the study itself runs the real verifier).

With --check, also asserts that RelocVerifierParams' defaults in
prism_loc_core/include/prism_loc_core/relocalization.hpp equal the selection.
"""
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CALIB = ROOT / "docs/paper/data/bbs_verify_calib.csv"
OUT = ROOT / "docs/paper/data/bbs_verify_selection.csv"
HDR = ROOT / "prism_loc_core/include/prism_loc_core/relocalization.hpp"

KS = [1, 2, 3, 4, 6, 8]
MS = list(range(1, 13))
GAINS = [10.0, 20.0, 40.0, 80.0, 160.0]
TAUS = [0.9, 0.95, 0.99, 0.999]
RHO = 0.4

# query -> hyp -> list over scans of (alive, frac, ok)
queries = defaultdict(lambda: defaultdict(dict))
for r in csv.DictReader(open(CALIB)):
    q = (r["map_m"], int(r["idx"]))
    h = int(r["hyp"])
    if h < 0:
        queries[q]  # touch: query with no hypothesis
        continue
    ok = float(r["pos_err_m"]) < 0.5 and float(r["yaw_err_deg"]) < 10.0
    queries[q][h][int(r["scan"])] = (int(r["alive"]), float(r["frac"]), ok)


def decide(hyps, K, M, gain, tau):
    cand = [hyps[h] for h in sorted(hyps) if h < K]
    if not cand:
        return "no-candidate", False
    lw, alive_end, mean_f, ok_end = [], [], [], []
    for tr in cand:
        s, n, alive = 0.0, 0, True
        for t in range(M):
            a, f, _ = tr.get(t, (0, 0.0, False))
            if not a:
                alive = False
                break
            s += f
            n += 1
        lw.append(gain * s)
        alive_end.append(alive)
        mean_f.append(s / max(1, n))
        ok_end.append(tr.get(M - 1, (0, 0.0, False))[2])
    live = [i for i in range(len(cand)) if alive_end[i]]
    if not live:
        return "no-candidate", False
    mx = max(lw[i] for i in live)
    z = sum(math.exp(lw[i] - mx) for i in live)
    best = None
    for i in live:
        p = math.exp(lw[i] - mx) / z
        if best is None or p > best[1]:
            best = (i, p)
    i, p = best
    if p < tau:
        return "ambiguous", False
    if mean_f[i] >= RHO:
        return "accepted", ok_end[i]
    return "no-candidate", False


rows = []
for K in KS:
    for M in MS:
        for gain in GAINS:
            for tau in TAUS:
                acc = fa = amb = noc = 0
                for q, hyps in queries.items():
                    st, ok = decide(hyps, K, M, gain, tau)
                    if st == "accepted":
                        acc += ok
                        fa += not ok
                    elif st == "ambiguous":
                        amb += 1
                    else:
                        noc += 1
                rows.append(dict(top_k=K, verify_scans=M, evidence_gain=gain,
                                 min_posterior=tau, n=len(queries), correct=acc,
                                 false_accept=fa, ambiguous=amb, no_candidate=noc))

with open(OUT, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

best = min(rows, key=lambda r: (r["false_accept"], -r["correct"], r["verify_scans"],
                                r["top_k"], r["evidence_gain"], -r["min_posterior"]))
single = next(r for r in rows if r["top_k"] == 1 and r["verify_scans"] == 1
              and r["min_posterior"] == TAUS[0] and r["evidence_gain"] == GAINS[0])
print(f"calibration queries: {len(queries)}")
print("single-scan rule (K=1, M=1): correct={correct} false_accept={false_accept}".format(**single))
print("selected: top_k={top_k} verify_scans={verify_scans} evidence_gain={evidence_gain} "
      "min_posterior={min_posterior} -> correct={correct} false_accept={false_accept} "
      "ambiguous={ambiguous} no_candidate={no_candidate}".format(**best))

if "--check" in sys.argv:
    src = HDR.read_text()

    def val(name):
        m = re.search(rf"\b{name}\{{([0-9.eE+-]+)\}}", src)
        return float(m.group(1)) if m else None

    want = dict(top_k=best["top_k"], verify_scans=best["verify_scans"],
                evidence_gain=best["evidence_gain"], min_posterior=best["min_posterior"])
    bad = {k: (val(k), v) for k, v in want.items() if val(k) != float(v)}
    if bad:
        print(f"MISMATCH header defaults vs selection: {bad}")
        sys.exit(1)
    print("header defaults match the selection")
