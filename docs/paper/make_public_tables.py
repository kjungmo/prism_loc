#!/usr/bin/env python3
"""Statistics and LaTeX tables of the public-dataset evaluation (NCLT, MIT Stata Center).

Everything is read from the committed result files under experiments/results/ and scored with
experiments/nclt/nclt_init_score.py (the unchanged scorer of the protocol):

    python3 docs/paper/make_public_tables.py          # write the tables
    python3 docs/paper/make_public_tables.py --print  # also print the derived statistics

check_numbers.py imports this module, fails if the committed tables differ from what the result
files produce, and checks every number quoted in the text against `derive()`.
"""
import csv
import re
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
RES = REPO / "experiments" / "results"
TABLES = ROOT / "arxiv_ref" / "tables"
sys.path.insert(0, str(REPO / "experiments" / "nclt"))
from nclt_init_score import measure  # noqa: E402

LEVELS = ["L0", "L1", "L2", "L3", "L4", "L5", "L6", "U"]
OFFSET = {"L0": "0 / 0", "L1": "2 / 15", "L2": "5 / 15", "L3": "10 / 15", "L4": "2 / 45",
          "L5": "2 / 90", "L6": "10 / 90", "U": "unknown"}


def summarize(results):
    """results: list of (converged, conv_time, rmse). Returns (converged, n, median time, median rmse)."""
    conv = [r for r in results if r[0]]
    return (len(conv), len(results),
            st.median([r[1] for r in conv]) if conv else None,
            st.median([r[2] for r in conv]) if conv else None)


def _round(x, nd):
    # Python's round() on the statistics.median value, as the scorer prints with %.nf
    return None if x is None else float("%.*f" % (nd, x))


def cell(c):
    n, tot, t, r = c
    return (n, tot, _round(t, 1), _round(r, 3))


def parse_score_file(path):
    """Summary lines of a score_init*.txt: {(method, level): cell}."""
    out, cur = {}, None
    for line in Path(path).read_text().splitlines():
        m = re.match(r"== (\S+) (\S+)$", line)
        if m:
            cur = (m.group(1), m.group(2))
            continue
        m = re.match(r"summary: converged (\d+) / (\d+), median convergence time (\S+) s, "
                     r"median RMSE after convergence (\S+) m", line)
        if m:
            f = lambda s: None if s == "-" else float(s)
            out[cur] = (int(m.group(1)), int(m.group(2)), f(m.group(3)), f(m.group(4)))
    return out


def score_dir(d, pattern):
    files = sorted(Path(d).glob(pattern))
    return cell(summarize([measure(str(f)) for f in files])), len(files)


def nclt():
    """NCLT initialization-robustness cells per method and level."""
    s3d = parse_score_file(RES / "nclt" / "score_init.txt")
    sl2 = parse_score_file(RES / "nclt" / "score_init_laser2d.txt")
    out = {"ndt3d": {}, "tracker": {}, "laser2d": {}, "laser2d_alt": {}, "amcl": {}, "amcl_u20k": None}
    for L in LEVELS:
        if ("ndt3d", L) in s3d:
            out["ndt3d"][L] = s3d[("ndt3d", L)]
            out["tracker"][L] = s3d[("tracker", L)]
        out["laser2d"][L] = sl2[("laser2d", L)]
        c, n = score_dir(RES / "amcl" / "nclt_laser2d_alt", f"laser2d_alt_{L}_t*_s*.csv")
        assert n == 12, (L, n)
        out["laser2d_alt"][L] = c
        c, n = score_dir(RES / "amcl" / "nclt_pri", f"amcl_{L}_t*_s*.csv")
        assert n == 12, (L, n)
        out["amcl"][L] = c
    c, n = score_dir(RES / "amcl" / "nclt_pri", "amcl_U20k_t*_s*.csv")
    assert n == 12
    out["amcl_u20k"] = c
    return out


def stata():
    """MIT Stata Center initialization cells: laser2d from its committed CSV, AMCL from run CSVs."""
    rows = list(csv.DictReader(open(RES / "laser2d_public" / "init_map0406_q0402.csv")))
    out = {"laser2d": {}, "amcl": {}, "amcl_u20k": None}
    for L in LEVELS:
        x = [r for r in rows if r["level"] == L]
        assert len(x) == 12, (L, len(x))
        res = [(r["converged"] == "1",
                float(r["conv_time_s"]) if r["converged"] == "1" else float("nan"),
                float(r["rmse_after"]) if r["converged"] == "1" else float("nan")) for r in x]
        out["laser2d"][L] = cell(summarize(res))
        c, n = score_dir(RES / "amcl" / "stata", f"amcl_{L}_st*_s*.csv")
        assert n == 12, (L, n)
        out["amcl"][L] = c
    c, n = score_dir(RES / "amcl" / "stata", "amcl_U20k_st*_s*.csv")
    assert n == 12
    out["amcl_u20k"] = c
    return out


def reloc():
    """BBS relocalization counts, 50 queries, from the scorer outputs."""
    out = {}
    for tag in ["0406", "0118"]:
        t = (RES / "laser2d_public" / f"score_reloc_map{tag}_q0402.txt").read_text()
        m = re.search(r"verifier .*?: correct (\d+), wrong accepted (\d+), rejected (\d+)", t)
        s = re.search(r"single-scan rule: correct (\d+), wrong accepted (\d+), rejected (\d+)", t)
        tm = re.search(r"time first scan \(global BBS\): median (\d+) ms", t)
        out[tag] = {"ver": tuple(map(int, m.groups())), "single": tuple(map(int, s.groups())),
                    "ms_first": int(tm.group(1))}
    return out


def track(path):
    r = [x for x in csv.DictReader(open(path)) if x["err_xy"] != "nan"]
    t = [int(x["utime"]) * 1e-6 for x in r]
    e = [float(x["err_xy"]) for x in r]
    ev, tot, s0 = 0, 0.0, None
    for k in range(len(t)):
        bad = e[k] > 1.0
        if bad and s0 is None:
            s0 = t[k]
        if (not bad or k == len(t) - 1) and s0 is not None:
            if t[k] - s0 > 2.0:
                ev += 1
                tot += t[k] - s0
            s0 = None
    return {"rmse": (sum(v * v for v in e) / len(e)) ** 0.5, "median": st.median(e), "max": max(e),
            "exc": ev, "exc_s": tot, "share1": sum(v > 1 for v in e) / len(e)}


def stata_track():
    out = {}
    for seed in ["42", "1", "2"]:
        out[("laser2d", "long", seed)] = track(RES / "laser2d_public" / f"track_map0406_q0402_s{seed}.csv")
        out[("laser2d", "short", seed)] = track(RES / "laser2d_public" / f"track_map0406_q0404_s{seed}.csv")
        out[("amcl", "long", seed)] = track(RES / "amcl" / "stata" / f"track_ql_s{seed}.csv")
        out[("amcl", "short", seed)] = track(RES / "amcl" / "stata" / f"track_qs_s{seed}.csv")
    return out


def nclt_elevation():
    """ndt3d on NCLT, constant base height versus ground grid (scores of nclt_score.py)."""
    t = (RES / "nclt" / "score.txt").read_text().splitlines()
    sections, cur = {}, None
    for line in t:
        m = re.match(r"== (\S+)$", line)
        if m:
            cur = m.group(1)
            sections[cur] = {"runs": [], "pooled": None}
        elif line.startswith("pooled") and cur:
            sections[cur]["pooled"] = line
        elif cur and line.strip():
            m = re.match(r"\S+_t(\d+)_s(\d+)\.csv .* rmse (\S+) max\s+(\S+) ", line)
            sections[cur]["runs"].append((int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))))
    out = {}
    for k, s in sections.items():
        m = re.match(r"pooled 12 runs: rmse median (\S+), max (\S+), excursions (\d+) \((\S+) s\), "
                     r"runs with an excursion (\d+)", s["pooled"])
        out[k] = {"rmse_med": float(m.group(1)), "max": float(m.group(2)), "exc": int(m.group(3)),
                  "runs_exc": int(m.group(5)), "runs": s["runs"]}
    return out


def derive():
    D = {"nclt": nclt(), "stata": stata(), "reloc": reloc(), "track": stata_track(), "elev": nclt_elevation()}
    tr = {}
    for p in ["400", "1400", "2400", "3400"]:
        f = RES / "nclt" / "score_ndt_tracker.txt"
    txt = (RES / "nclt" / "score_ndt_tracker.txt").read_text()
    m = re.search(r"pooled 4 runs: rmse median (\S+), max (\S+), excursions 0 .* runs with an excursion (\d+)", txt)
    D["tracker_excursion"] = (float(m.group(1)), float(m.group(2)), int(m.group(3)))
    return D


# ------------------------------------------------------------------ LaTeX
def fc(c, t=True, r=True):
    n, tot, ti, rm = c
    s = f"{n}/{tot}"
    if n == 0:
        return s
    extra = []
    if t:
        extra.append(f"{ti:.1f}")
    if r:
        extra.append(f"{rm:.2f}")
    return s + " (" + ", ".join(extra) + ")" if extra else s


def table_nclt_3d(D):
    N = D["nclt"]
    rows = []
    for L in LEVELS[:-1]:
        rows.append(f"{L} & {OFFSET[L]} & {fc(N['ndt3d'][L])} & {fc(N['tracker'][L])} \\\\")
    return "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{\textbf{NCLT: recovery from a wrong initial pose, \code{ndt3d} against a 6-DoF scan-to-map NDT tracker.} Cells: runs converged out of 12 (median convergence time in s, median planar RMSE after convergence in m; the last two are over the converged runs). \code{ndt3d} runs with the ground grid and the tempered-likelihood settings of \Cref{sec:public}. The particle filter is given the offset pose with the spread $(\max(0.5,d)\,\mathrm{m},\ \max(15,a)^\circ)$; the tracker takes the offset pose as its start and cannot use a spread. Level U (unknown pose) is not supported by either method.}\label{tab:nclt3d}",
        r"\small",
        r"\begin{tabular}{@{}llcc@{}}",
        r"\toprule",
        r"Level & Offset $d$\,(m) / $a$\,($^\circ$) & \code{ndt3d} & 6-DoF tracker \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        ""])


def table_nclt_2d(D):
    N = D["nclt"]
    rows = []
    for L in LEVELS:
        a = fc(N["amcl"][L])
        if L == "U":
            a += f"; U-20k: {fc(N['amcl_u20k'], False, False)}"
        rows.append(f"{L} & {OFFSET[L]} & {fc(N['laser2d'][L])} & {fc(N['laser2d_alt'][L])} & {a} \\\\")
    return "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{\textbf{NCLT projected to 2D: \code{laser2d} and \code{nav2\_amcl}.} Same cells and protocol as \Cref{tab:nclt3d}. \code{laser2d} uses the height band chosen after seeing L0 results (``band 1''), and a band fixed in advance (``band 2''). AMCL runs with Nav2 defaults on band 1. At level U, \code{laser2d} runs branch-and-bound relocalization and AMCL its own global localization (500--2000 particles; U-20k: 2000--20000). AMCL fails at L0, so it is not used as a baseline on this dataset.}\label{tab:nclt2d}",
        r"\small",
        r"\begin{tabular}{@{}llccc@{}}",
        r"\toprule",
        r"Level & Offset & \code{laser2d} band 1 & \code{laser2d} band 2 & AMCL \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        ""])


def table_stata(D):
    S = D["stata"]
    rows = []
    for L in LEVELS:
        a = fc(S["amcl"][L])
        if L == "U":
            a += f"; U-20k: {fc(S['amcl_u20k'])}"
        rows.append(f"{L} & {OFFSET[L]} & {fc(S['laser2d'][L])} & {a} \\\\")
    return "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{\textbf{MIT Stata Center: \code{laser2d} and \code{nav2\_amcl}.} Map: session 2012-04-06; queries: four start times of session 2012-04-02, three seeds, 180\,s each. Cells as in \Cref{tab:nclt3d}. Odometry is synthesized from the ground truth plus fixed errors (\Cref{sec:public}). At level U, \code{laser2d} runs branch-and-bound relocalization with multi-scan verification and then tracks; AMCL runs its own global localization (U-20k: 2000--20000 particles).}\label{tab:stata}",
        r"\small",
        r"\begin{tabular}{@{}llcc@{}}",
        r"\toprule",
        r"Level & Offset & \code{laser2d} & AMCL \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        ""])


def table_elev(D):
    E = D["elev"]
    names = [("default", "constant base height (\\code{ndt3d} defaults)"), ("ground", "ground grid"),
             ("ground_ep5_ms10_a05", "ground grid, tempered likelihood, lower motion noise")]
    rows = []
    for k, n in names:
        e = E[k]
        rows.append(f"{n} & {e['runs_exc']}/12 & {e['exc']} & {e['max']:.2f} & {e['rmse_med']:.3f} \\\\")
    return "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{\textbf{NCLT, tracking from the true start pose: effect of the ground grid on \code{ndt3d}.} Four segments of 1000\,s, three seeds each. An excursion is a planar error above 1\,m for more than 2\,s. The session climbs and descends about 18\,m.}\label{tab:elev}",
        r"\small",
        r"\begin{tabular}{@{}>{\raggedright\arraybackslash}p{5.4cm}cccc@{}}",
        r"\toprule",
        r"Height model & Runs with excursion & Excursions & Max error (m) & Median RMSE (m) \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        ""])


def table_reloc(D):
    R = D["reloc"]
    rows = []
    for tag, name in [("0406", "2012-04-06 (4 days before the query)"), ("0118", "2012-01-18 (3 months before)")]:
        v, s1 = R[tag]["ver"], R[tag]["single"]
        rows.append(f"{name} & {v[0]} / {v[1]} / {v[2]} & {s1[0]} / {s1[1]} / {s1[2]} \\\\")
    return "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{\textbf{MIT Stata Center: global relocalization of \code{laser2d} from a single unknown pose.} 50 queries spread over the 1403\,s query session (2012-04-02), no prior. Cells: correct / wrong accepted / rejected. Correct means within 0.5\,m and $10^\circ$ of the ground truth at the scan on which the method decided. ``Verifier'' is the multi-scan verifier of \Cref{sec:verify} with node defaults; ``single scan'' accepts the first scan's best match if its score fraction is at least 0.4.}\label{tab:statareloc}",
        r"\small",
        r"\begin{tabular}{@{}lcc@{}}",
        r"\toprule",
        r"Map session & Verifier & Single scan \\",
        r"\midrule",
        *rows,
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
        ""])


def generate():
    D = derive()
    return {TABLES / "nclt_3d.tex": table_nclt_3d(D), TABLES / "nclt_2d.tex": table_nclt_2d(D),
            TABLES / "stata_init.tex": table_stata(D), TABLES / "nclt_elev.tex": table_elev(D),
            TABLES / "stata_reloc.tex": table_reloc(D)}


if __name__ == "__main__":
    for p, text in generate().items():
        p.write_text(text)
        print("wrote", p.relative_to(REPO))
    if "--print" in sys.argv:
        import pprint
        pprint.pprint(derive(), width=140)
