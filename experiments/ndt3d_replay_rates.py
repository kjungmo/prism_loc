#!/usr/bin/env python3
"""Rates of the ndt3d status rule from the replay CSVs (ndt3d_replay_batch.py). Standard library only.

    ndt3d_replay_rates.py RES_DIR [THR [N]]      # defaults 0.65 and 15 (the shipped min_match_fraction, match_verify_updates)

Prints, as Markdown, every number the README, PARAMS.md and the PR text quote from the replay:
  1. the group table (README, "classifier" table);
  2. the same for the threshold 0.70 (the "default" bullet);
  3. the per-case false-alarm rates (changed scenes) and the mean-of-5 ranges (missed / setting bullets);
  4. the runs that read OK while off by more than 0.5 m after an /initialpose 0.3 to 1.0 m off;
  5. every case of the batch, one row per case (4 runs each), so that no run is left out of view.

Rule, for each filter update: low match if the mean of the last 5 judged fractions is below THR (the
update forced by a reseed is not judged); else "not verified" while fewer than N updates have been
judged since the reseed; else OK. Columns are shares of ALL updates of the runs named (no update left
out; the kidnap groups count from the kidnap, t >= 30 s). "OK > 1 m" = OK while the estimate is more
than 1 m off the truth; "max run" = the longest run of such consecutive updates; "low < 0.15 m" = low match
while the estimate is within 0.15 m (a false alarm).
"""
import csv
import glob
import os
import sys

res = sys.argv[1]
THR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.65
N = int(sys.argv[3]) if len(sys.argv) > 3 else 15

GROUPS = [
    ('healthy, exact map; 0.5x to 3x speed, sparser lidars', ['H_base', 'H_speed0.5', 'H_speed2', 'H_speed3', 'H_sparse_4x90', 'H_sparse_8x180'], 0),
    ('healthy, 2 or 5 cm noise on the prior map', ['H_mapnoise2', 'H_mapnoise5'], 0),
    ('healthy, scene differs from the exact map', ['H_dyn6', 'H_dyn15', 'H_chg5add1drop', 'H_chg2add', 'H_range3', 'H_range5'], 0),
    ('the same with 2 cm of noise on the prior map', ['H_dyn6_mapnoise2', 'H_dyn15_mapnoise2', 'H_chg5add1drop_mapnoise2',
                                                      'H_chg2add_mapnoise2', 'H_range3_mapnoise2', 'H_range5_mapnoise2'], 0),
    ('kidnapped, exact map, every update after the kidnap', ['K_kidnap', 'K_kidnap_shift8'], 30),
    ('kidnapped, 2 or 5 cm map noise', ['K_kidnap_mapnoise2', 'K_kidnap_mapnoise5', 'K_kidnap_shift8_mapnoise2'], 30),
    ('/initialpose 3.6 m and 1 rad off, exact map', ['W_init_3_2_1'], 0),
    ('the same, 2 or 5 cm map noise', ['W_init_3_2_1_mapnoise2', 'W_init_3_2_1_mapnoise5'], 0),
    ('/initialpose 0.3 to 1.0 m off', ['W_init_0.3', 'W_init_0.5', 'W_init_1.0', 'W_init_1.0y', 'W_init_5deg', 'W_init_mid',
                                       'W_init_1.0_mapnoise2'], 0),
    ('pose off the map', ['O_offmap'], 0),
    ('base_height 5 or 10 cm wrong, pose fine', ['B_height-0.05', 'B_height-0.1'], 0),
]


def runs_of(case):
    return sorted(glob.glob(f'{res}/{case}_[0-9].csv'))


def classify(path, thr, n_verify, t0=0.0):
    """[(t, err, state, forced, mean5)] for the updates from t0; state in 'ok', 'unv', 'low'."""
    out, since = [], 0
    for r in csv.DictReader(open(path)):
        forced = r['forced'] == '1'
        since = 0 if forced else since + 1
        if float(r['t']) < t0:
            continue
        m = float(r['mean5'])
        if not forced and m < thr:
            st = 'low'
        elif since < n_verify:
            st = 'unv'
        else:
            st = 'ok'
        out.append((float(r['t']), float(r['err']), st, forced, m))
    return out


def stats(cases, t0, thr, n_verify):
    n = ok = unv = low = ok1 = ok03 = lowgood = maxrun = 0
    for c in cases:
        for f in runs_of(c):
            run = 0
            for t, e, st, forced, m in classify(f, thr, n_verify, t0):
                n += 1
                if st == 'low':
                    low += 1
                    lowgood += e < 0.15
                    run = 0
                elif st == 'unv':
                    unv += 1
                    run = 0
                else:
                    ok += 1
                    ok1 += e > 1.0
                    ok03 += e > 0.3
                    run = run + 1 if e > 1.0 else 0
                    maxrun = max(maxrun, run)
    return n, ok, unv, low, ok1, ok03, lowgood, maxrun


def pc(x, n):
    return f'{100.0 * x / n:.1f} %' if n else 'n/a'


def table(thr, n_verify):
    print('| Runs | updates | OK | not verified | low match | OK while > 1 m off (max in a row) | OK while > 0.3 m off | low match while < 0.15 m off |')
    print('|---|---|---|---|---|---|---|---|')
    for name, cases, t0 in GROUPS:
        k = sum(len(runs_of(c)) for c in cases)
        n, ok, unv, low, ok1, ok03, lowgood, maxrun = stats(cases, t0, thr, n_verify)
        if not n:
            continue
        print(f'| {name} ({k} runs) | {n} | {pc(ok, n)} | {pc(unv, n)} | {pc(low, n)} | {pc(ok1, n)}'
              f'{f" ({maxrun})" if ok1 else ""} | {pc(ok03, n)} | {pc(lowgood, n)} |')


print(f'# ndt3d replay: rates of the status rule (threshold {THR}, verification {N} updates)\n')
print(f'## 1. Groups, threshold {THR}\n')
table(THR, N)
if THR != 0.70:
    print('\n## 2. The same groups at threshold 0.70\n')
    table(0.70, N)
    print('\nWithout the "not verified" state (verification 0), threshold %s:\n' % THR)
    table(THR, 0)

print('\n## 3. False alarms and mean-of-5 ranges\n')
print('Low match per changed-scene case on the exact map (share of the updates of the 4 runs):\n')
print('| Case | updates | low match | low match while < 0.15 m off |')
print('|---|---|---|---|')
for c in ['H_dyn15', 'H_chg5add1drop', 'H_range5', 'H_chg2add', 'H_dyn6', 'H_range3',
          'H_dyn15_mapnoise2', 'H_chg5add1drop_mapnoise2', 'H_range5_mapnoise2', 'H_chg2add_mapnoise2', 'H_dyn6_mapnoise2', 'H_range3_mapnoise2']:
    n, ok, unv, low, ok1, ok03, lowgood, maxrun = stats([c], 0, THR, N)
    print(f'| {c} | {n} | {pc(low, n)} | {pc(lowgood, n)} |')


def ranges(cases, t0, lost_only):
    """(min, max, count) of the single-update fraction and of the mean of 5, over the updates that are
    not the forced one (and, with lost_only, only those more than 1 m off)."""
    fr, m5 = [], []
    for c in cases:
        for f in runs_of(c):
            for r in csv.DictReader(open(f)):
                if r['forced'] == '1' or float(r['t']) < t0:
                    continue
                if lost_only and float(r['err']) <= 1.0:
                    continue
                fr.append(float(r['frac']))
                m5.append(float(r['mean5']))
    return ((min(fr), max(fr)), (min(m5), max(m5)), len(fr)) if fr else None


print('\n`match_fraction` of single updates, and its mean of 5 (the value the rule uses; just after a kidnap the window still holds\n'
      'healthy values, which is why its maximum is higher):\n')
print('| Updates | count | single update, min - max | mean of 5, min - max |')
print('|---|---|---|---|')
for name, cases, t0, lost in [
        ('kidnapped, exact map, estimate > 1 m off', ['K_kidnap', 'K_kidnap_shift8'], 30, True),
        ('kidnapped, 2 or 5 cm map noise, estimate > 1 m off', ['K_kidnap_mapnoise2', 'K_kidnap_mapnoise5', 'K_kidnap_shift8_mapnoise2'], 30, True),
        ('healthy, exact map (6 cases)', GROUPS[0][1], 0, False),
        ('healthy, 2 or 5 cm map noise', ['H_mapnoise2', 'H_mapnoise5'], 0, False),
        ('/initialpose 3.6 m off, estimate > 1 m off, exact map', ['W_init_3_2_1'], 0, True),
        ('/initialpose 3.6 m off, estimate > 1 m off, 2 or 5 cm map noise', ['W_init_3_2_1_mapnoise2', 'W_init_3_2_1_mapnoise5'], 0, True)]:
    r = ranges(cases, t0, lost)
    print(f'| {name} | {r[2]} | {r[0][0]:.3f} - {r[0][1]:.3f} | {r[1][0]:.3f} - {r[1][1]:.3f} |' if r else f'| {name} | 0 | | |')

print('\nKidnapped, longest stretch of OK updates while > 1 m off, in seconds (updates are about 0.6 s apart):\n')
print('| Case | runs with such an update | longest stretch (updates) | longest stretch (s) |')
print('|---|---|---|---|')
for c in ['K_kidnap', 'K_kidnap_shift8', 'K_kidnap_mapnoise2', 'K_kidnap_mapnoise5', 'K_kidnap_shift8_mapnoise2']:
    k = best = 0
    best_s = 0.0
    for f in runs_of(c):
        rows = classify(f, THR, N, 30)
        hit, run, t_start = False, 0, 0.0
        for t, e, st, forced, m in rows:
            if st == 'ok' and e > 1.0:
                if run == 0:
                    t_start = t
                run += 1
                hit = True
                if run > best:
                    best, best_s = run, t - t_start
            else:
                run = 0
        k += hit
    print(f'| {c} | {k} of {len(runs_of(c))} | {best} | {best_s:.1f} |')

print('\n## 4. /initialpose 0.3 to 1.0 m off: OK while off by more than 0.5 m\n')
print('| Case | runs | runs with an OK update > 0.5 m off | OK updates > 0.5 m off | largest error of those (m) | longest stretch (s) |')
print('|---|---|---|---|---|---|')
tot_runs = tot_hit = 0
for c in GROUPS[8][1]:
    hit = cnt = 0
    big = 0.0
    longest = 0.0
    for f in runs_of(c):
        rows = classify(f, THR, N, 0)
        h, run_t0, prev = False, None, None
        for t, e, st, forced, m in rows:
            if st == 'ok' and e > 0.5:
                h = True
                cnt += 1
                big = max(big, e)
                if run_t0 is None:
                    run_t0 = t
                longest = max(longest, t - run_t0)
            else:
                run_t0 = None
        hit += h
    tot_runs += len(runs_of(c))
    tot_hit += hit
    print(f'| {c} | {len(runs_of(c))} | {hit} | {cnt} | {big:.2f} | {longest:.1f} |')
print(f'| all | {tot_runs} | {tot_hit} | | | |')

print('\n## 4b. /initialpose 3.6 m and 1 rad off: judged updates before the first low match\n')
print('| Case | seed | judged updates with the match not yet low | estimate error over those (m) |')
print('|---|---|---|---|')
firsts, lo_e, hi_e = [], 9.9, 0.0
for c in ['W_init_3_2_1', 'W_init_3_2_1_mapnoise2', 'W_init_3_2_1_mapnoise5']:
    for f in runs_of(c):
        judged, errs, first = 0, [], None
        for r in csv.DictReader(open(f)):
            if r['forced'] == '1':
                judged, errs = 0, []
                continue
            judged += 1
            if float(r['mean5']) < THR:
                first = judged - 1
                break
            errs.append(float(r['err']))
        firsts.append(first if first is not None else judged)
        if errs:
            lo_e, hi_e = min(lo_e, min(errs)), max(hi_e, max(errs))
        print(f'| {c} | {f[-5]} | {first if first is not None else "never (" + str(judged) + " judged)"} | '
              f'{min(errs):.2f} - {max(errs):.2f} |' if errs else f'| {c} | {f[-5]} | {first} | |')
print(f'\nAll 12 runs: {min(firsts)} to {max(firsts)} judged updates, estimate {lo_e:.2f} to {hi_e:.2f} m off.')

print('\n## 5. Every case of the batch (4 runs each)\n')
print('| Case | runs | updates | OK | not verified | low match | OK while > 1 m off | OK while > 0.3 m off | low match while < 0.15 m off |')
print('|---|---|---|---|---|---|---|---|---|')
cases = sorted({os.path.basename(f).rsplit('_', 1)[0] for f in glob.glob(f'{res}/*_[0-9].csv')})
for c in cases:
    t0 = 30 if c.startswith('K_') else 0
    n, ok, unv, low, ok1, ok03, lowgood, maxrun = stats([c], t0, THR, N)
    print(f'| {c}{" (from the kidnap)" if t0 else ""} | {len(runs_of(c))} | {n} | {pc(ok, n)} | {pc(unv, n)} | {pc(low, n)} | '
          f'{pc(ok1, n)} | {pc(ok03, n)} | {pc(lowgood, n)} |')
