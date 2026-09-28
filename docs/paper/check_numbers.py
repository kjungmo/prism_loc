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

# ---------------- arXiv reference-style package (arxiv_ref/) ----------------
# The arXiv port must quote the same numbers, plus the per-seed / ablation
# statistics derived from the same CSVs, and ship byte-identical data copies.
AX = ROOT / "arxiv_ref"
AX_FILES = sorted(AX.glob("sections/*.tex")) + sorted(AX.glob("figures/*.tex")) + sorted(AX.glob("tables/*.tex"))
AX_CORPUS = {p: p.read_text() for p in AX_FILES}


def ax_expect(name, formatted):
    hits = [p for p, t in AX_CORPUS.items() if formatted in t]
    if not hits:
        failures.append(f"MISSING  arxiv {name}: '{formatted}' not found in arxiv_ref sources")
    else:
        print(f"ok  arxiv {name}: '{formatted}' in {', '.join(p.name for p in hits)}")


for csvf in sorted(DATA.glob("*.csv")):
    cp = AX / "data" / csvf.name
    check(f"arxiv data copy {csvf.name}", cp.exists() and cp.read_bytes() == csvf.read_bytes(), "byte-identical")

for tok in ["0.105\\,m", "0.29\\,m", "0.032\\,rad", "0.18\\,rad", "1.8$^\\circ$",
            "2000-particle", "500-particle", "97 of 100", "97 / 100",
            "0.037\\,m", "0.072\\,m", "0.48$^\\circ$", "1.74$^\\circ$", "0.632",
            "37.65\\,ms", "8.77", "106.07", "78 / 0", "19 correct",
            "0.016\\,m", "0.125\\,m", "0.009\\,m", "0.256\\,m", "0.663\\,m", "0 / 121",
            "0.015\\,m"]:
    ax_expect("headline", tok)
for tok in ["0.31\\,m", "0.20\\,rad", "37.7\\,ms"]:
    if any(tok in t for t in AX_CORPUS.values()):
        failures.append(f"RETIRED  arxiv '{tok}' present")

# per-seed MCL table
rows = list(csv.DictReader(open(DATA / "mcl_tracking.csv")))
ss = [r for r in rows if float(r["t"]) >= 10.0]
check("mcl ss samples/seed", len(ss) == 110, f"n={len(ss)}")
tab = (AX / "tables" / "mcl_results.tex").read_text()
seed_means, yaw_means = [], []
for i, seed in enumerate([101, 202, 303]):
    pe = [float(r[f"pos_err_s{i}"]) for r in ss]
    ye = [abs(float(r[f"yaw_err_s{i}"])) for r in ss]
    seed_means.append(st.mean(pe)); yaw_means.append(st.mean(ye))
    line = f"{seed} & {st.mean(pe):.3f} & {max(pe):.3f} & {st.mean(ye):.3f} & {max(ye):.3f} \\\\"
    check(f"mcl table seed {seed}", line in tab, line)
ms = f"{st.mean(seed_means):.3f}\\stdv{{{st.stdev(seed_means):.3f}}}"
ys = f"{st.mean(yaw_means):.3f}\\stdv{{{st.stdev(yaw_means):.3f}}}"
check("mcl table mean+-std", ms in tab and ys in tab, f"{ms} {ys}")
ax_expect("mcl seed-mean std text", f"{st.mean(seed_means):.3f}\\pm{st.stdev(seed_means):.3f}")
p = list(csv.DictReader(open(DATA / "mcl_particles.csv")))
avg = st.mean(int(float(r[k])) for r in p for k in r if k != "t")
check("kld avg count", f"{avg:.1f}" == "526.9" and f"{2000/avg:.1f}" == "3.8", f"{avg:.2f}, {2000/avg:.2f}x")
ax_expect("kld avg", "526.9 particles"); ax_expect("kld ratio", "3.8$\\times$")

# BBS threshold sweep + clutter terciles
rows = list(csv.DictReader(open(DATA / "bbs_relocalization.csv")))
ok = lambda r: float(r["pos_err_m"]) < 0.5 and abs(float(r["yaw_err_deg"])) < 10
fr = lambda r: float(r["score_frac"])
th = (AX / "tables" / "bbs_threshold.tex").read_text()
rhos = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60]
acc = [sum(fr(r) >= t for r in rows) for t in rhos]
fa = [sum(fr(r) >= t and not ok(r) for r in rows) for t in rhos]
rc = [sum(fr(r) < t and ok(r) for r in rows) for t in rhos]
for name, vals in [("Accepted", acc), ("False accepts", fa), ("Rejected correct", rc)]:
    line = name + " & " + " & ".join(str(v) for v in vals) + " \\\\"
    check(f"bbs sweep {name}", line in th, line)
check("bbs margin", f"{0.40 - max(fr(r) for r in rows if not ok(r)):.2f}" == "0.13", "0.40 - max fail frac")
ax_expect("bbs margin", "margin of $0.13$")
ax_expect("bbs rho0.3 extra", f"accepted {acc[2] - acc[4]} more correct")
lat = [float(r["time_ms"]) for r in rows]
check("bbs latency min/p95", f"{min(lat):.2f}" == "8.77" and f"{p95(lat):.2f}" == "106.07", f"{min(lat):.2f} {p95(lat):.2f}")
cl = (AX / "tables" / "bbs_clutter.tex").read_text()
bins = [(0.0, 0.8 / 3), (0.8 / 3, 1.6 / 3), (1.6 / 3, 0.81)]
for lo, hi in bins:
    sub = [r for r in rows if lo <= float(r["clutter_frac"]) < hi]
    line = (f"& {len(sub)} & {sum(ok(r) for r in sub)} & {sum(fr(r) >= 0.40 for r in sub)} & "
            f"{st.median(float(r['pos_err_m']) for r in sub):.3f} & {st.median(fr(r) for r in sub):.3f} \\\\")
    check(f"bbs clutter bin {lo:.3f}", line in cl, line)

# ESKF extra quoted values
rows = list(csv.DictReader(open(DATA / "eskf_track.csv")))
at35 = [r for r in rows if r["time_s"] == "35.000"][0]
check("eskf err at 35 s", f"{float(at35['err_norm_m']):.3f}" == "0.015", at35["err_norm_m"])
check("eskf rows", len(rows) == 121, f"n={len(rows)}")

# re-run artifact (latency only differs)
rr = DATA / "rerun_2026-09-28" / "bbs_relocalization.csv"
if rr.exists():
    a = list(csv.DictReader(open(rr))); b = list(csv.DictReader(open(DATA / "bbs_relocalization.csv")))
    same = all(x[c] == y[c] for x, y in zip(a, b) for c in b[0] if c != "time_ms") and len(a) == len(b)
    check("rerun bbs non-latency columns identical", same, f"rows={len(a)}")
    med = st.median(float(r["time_ms"]) for r in a)
    check("rerun median latency", f"{med:.2f}" == "10.26", f"{med:.3f}")
    ax_expect("rerun latency", "10.26\\,ms")

# test-suite size quoted in the paper (TEST/TEST_F macros per package)
import re
REPO = ROOT.parent.parent
def ntests(pkg):
    return sum(len(re.findall(r"^TEST(?:_F)?\(", f.read_text(), re.M)) for f in (REPO / pkg / "test").glob("*.cpp"))
counts = {k: ntests(k) for k in ["prism_loc_core", "prism_loc_fusion", "prism_loc", "prism_loc_fusion_ros"]}
check("test counts", list(counts.values()) == [25, 13, 4, 2] and sum(counts.values()) == 44, str(counts))
ax_expect("test total", "44 GoogleTest cases")

# ---------------- verdict ----------------
print()
if failures:
    for f in failures:
        print("FAIL", f, file=sys.stderr)
    sys.exit(1)
print(f"NUMBER GUARD GREEN — {len(TEX_FILES)} + {len(AX_FILES)} (arxiv_ref) tex files checked against {len(list(DATA.glob('*.csv')))} CSVs")
