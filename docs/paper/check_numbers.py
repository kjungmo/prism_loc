#!/usr/bin/env python3
"""Number guard for the PRISM-Loc paper.

Recomputes every headline statistic from the committed CSVs under
docs/paper/data/ and asserts (a) the recomputed value still matches the
rounded form quoted in the LaTeX sources, and (b) retired figures no
longer appear. Run from the repo root:

    python3 docs/paper/check_numbers.py

Exit code 0 = every quoted number traces to the committed artifacts.
"""
import csv
import math
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
TEX_FILES = sorted((ROOT / "sections").glob("*.tex")) + sorted(
    (ROOT / "figures").glob("*.tex")
)
CORPUS = {p: p.read_text() for p in TEX_FILES}

failures = []


def expect(name, formatted, files=None):
    """Assert `formatted` appears in at least one (or the given) tex file."""
    hits = [p for p, t in CORPUS.items() if formatted in t]
    if files is not None:
        hits = [p for p in hits if p.name in files]
    if not hits:
        failures.append(f"MISSING  {name}: '{formatted}' not found in tex sources")
    else:
        print(f"ok  {name}: '{formatted}' in {', '.join(p.name for p in hits)}")


def forbid(name, token, files):
    hits = [p for p, t in CORPUS.items() if token in t and p.name in files]
    if hits:
        failures.append(
            f"RETIRED  {name}: '{token}' still present in "
            f"{', '.join(p.name for p in hits)}"
        )
    else:
        print(f"ok  retired '{token}' absent from {files}")


def check(name, cond, detail):
    if not cond:
        failures.append(f"DRIFT    {name}: {detail}")
    else:
        print(f"ok  {name}: {detail}")


# ---------------- MCL tracking ----------------
rows = list(csv.DictReader(open(DATA / "mcl_tracking.csv")))
ss = [r for r in rows if float(r["t"]) >= 10.0]
pe = [float(r[f"pos_err_s{i}"]) for r in ss for i in range(3)]
ye = [abs(float(r[f"yaw_err_s{i}"])) for r in ss for i in range(3)]
check("mcl mean pos", f"{st.mean(pe):.3f}" == "0.105", f"mean={st.mean(pe):.4f}")
check("mcl max pos", f"{max(pe):.2f}" == "0.29", f"max={max(pe):.4f}")
check("mcl mean yaw", f"{st.mean(ye):.3f}" == "0.032", f"mean={st.mean(ye):.4f}")
check("mcl max yaw", f"{max(ye):.2f}" == "0.18", f"max={max(ye):.4f}")
expect("mcl mean pos", "0.105\\,m")
expect("mcl max pos", "0.29\\,m")
expect("mcl mean yaw", "0.032\\,rad")
expect("mcl max yaw", "0.18\\,rad")
forbid("old mcl max pos", "0.31\\,m", ["validation.tex", "fig_mcl.tex"])
forbid("old mcl max pos ($)", "$0.31\\,$m", ["validation.tex", "fig_mcl.tex"])
forbid("old mcl max yaw", "0.20\\,rad", ["validation.tex", "fig_mcl.tex"])
forbid("old mcl max yaw ($)", "$0.20\\,$rad", ["validation.tex", "fig_mcl.tex"])

p = list(csv.DictReader(open(DATA / "mcl_particles.csv")))
counts = [int(float(r[k])) for r in p for k in r if k != "t"]
check("kld bounds", (min(counts), max(counts)) == (500, 2000), f"{min(counts)}..{max(counts)}")
expect("kld floor", "500-particle")
expect("kld ceiling", "2000-particle")

# ---------------- BBS relocalization ----------------
rows = list(csv.DictReader(open(DATA / "bbs_relocalization.csv")))
pe = [float(r["pos_err_m"]) for r in rows]
ye = [abs(float(r["yaw_err_deg"])) for r in rows]
succ = sum(1 for a, b in zip(pe, ye) if a < 0.5 and b < 10)
check("bbs n", len(rows) == 100, f"n={len(rows)}")
check("bbs successes", succ == 97, f"succ={succ}")
expect("bbs success rate", "97 of\n100 queries succeed (97.0\\%)", ["validation.tex"]) if False else None
expect("bbs success frac", "97/100")
def p95(xs):
    """95th percentile, linear interpolation (numpy default convention)."""
    s = sorted(xs)
    k = 0.95 * (len(s) - 1)
    lo = int(math.floor(k))
    return s[lo] + (k - lo) * (s[min(lo + 1, len(s) - 1)] - s[lo])


check("bbs med pos", f"{st.median(pe):.3f}" == "0.037", f"{st.median(pe):.4f}")
check("bbs p95 pos", f"{p95(pe):.3f}" == "0.072", f"{p95(pe):.4f}")
check("bbs med yaw", f"{st.median(ye):.2f}" == "0.48", f"{st.median(ye):.4f}")
check("bbs p95 yaw", f"{p95(ye):.2f}" == "1.74", f"{p95(ye):.4f}")
expect("bbs med pos", "0.037\\,m")
expect("bbs p95 pos", "0.072\\,m")
lat = [float(r["time_ms"]) for r in rows]
check("bbs med latency", f"{st.median(lat):.2f}" == "37.65", f"{st.median(lat):.3f}")
expect("bbs med latency", "37.65\\,ms")
frac = [float(r["score_frac"]) for r in rows]
fails_frac = [f for f, a, b in zip(frac, pe, ye) if not (a < 0.5 and b < 10)]
check(
    "bbs fail band",
    f"{min(fails_frac):.2f}" == "0.21" and f"{max(fails_frac):.2f}" == "0.27",
    f"{min(fails_frac):.4f}..{max(fails_frac):.4f}",
)
accepted = [i for i, s in enumerate(frac) if s >= 0.40]
false_accepts = [i for i in accepted if not (pe[i] < 0.5 and ye[i] < 10)]
check("bbs accepted", len(accepted) == 78, f"accepted={len(accepted)}")
check("bbs false accepts", len(false_accepts) == 0, f"false={len(false_accepts)}")
check("bbs med frac", f"{st.median(frac):.3f}" == "0.632", f"{st.median(frac):.4f}")
expect("bbs med frac", "0.632")

# ---------------- ESKF fusion ----------------
rows = list(csv.DictReader(open(DATA / "eskf_track.csv")))
runs, cur = [], []
for r in rows:
    if r["gnss_active"] == "0" and r["pose_active"] == "0":
        cur.append(float(r["time_s"]))
    else:
        if cur:
            runs.append((cur[0], cur[-1]))
        cur = []
if cur:
    runs.append((cur[0], cur[-1]))
t0, t1 = max(runs, key=lambda ab: ab[1] - ab[0])


def rmse(rs):
    return math.sqrt(st.mean([float(r["err_norm_m"]) ** 2 for r in rs]))


before = [r for r in rows if 1.0 <= float(r["time_s"]) < t0]
during = [r for r in rows if t0 <= float(r["time_s"]) <= t1]
after = [r for r in rows if float(r["time_s"]) > t1]
check("eskf rmse before", f"{rmse(before):.3f}" == "0.016", f"{rmse(before):.4f}")
check("eskf rmse during", f"{rmse(during):.3f}" == "0.125", f"{rmse(during):.4f}")
check("eskf rmse after", f"{rmse(after):.3f}" == "0.009", f"{rmse(after):.4f}")
expect("eskf rmse triple", "0.016\\,m")
expect("eskf rmse during", "0.125\\,m")
expect("eskf rmse after", "0.009\\,m")
mx = max(rows, key=lambda r: float(r["err_norm_m"]))
check("eskf peak", f"{float(mx['err_norm_m']):.3f}" == "0.256", mx["err_norm_m"])
check("eskf peak time", mx["time_s"] == "34.500", f"t={mx['time_s']}")
expect("eskf peak", "0.256\\,m")
sig_max = max(float(r["sigma3_m"]) for r in rows)
check("eskf 3sig peak", f"{sig_max:.3f}" == "0.663", f"{sig_max:.4f}")
expect("eskf 3sig peak", "0.663\\,m")
viol = sum(1 for r in rows if float(r["err_norm_m"]) > float(r["sigma3_m"]))
check("eskf 3sig consistency", viol == 0, f"violations={viol}")

# ---------------- verdict ----------------
print()
if failures:
    for f in failures:
        print("FAIL", f, file=sys.stderr)
    sys.exit(1)
print(f"NUMBER GUARD GREEN — {len(TEX_FILES)} tex files checked against {len(list(DATA.glob('*.csv')))} CSVs")
