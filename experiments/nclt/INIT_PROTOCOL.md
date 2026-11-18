# Initialization robustness on NCLT: protocol

Fixed before any run of this experiment. Data, map and segments as in `README.md` (map 2012-01-08;
query 2012-02-04, segments starting at 400, 1400, 2400 and 3400 s).

## Initial-pose error levels

The initial pose given to a method is the ground-truth pose at the first scan of the segment,
moved by a planar offset of length `d` in a direction drawn from the seed, and turned by `+a` or
`-a` (sign drawn from the seed). Height, roll and pitch stay at the ground truth for the 6-DoF
baseline.

| level | d (m) | a (deg) |
|---|---|---|
| L0 | 0 | 0 |
| L1 | 2 | 15 |
| L2 | 5 | 15 |
| L3 | 10 | 15 |
| L4 | 2 | 45 |
| L5 | 2 | 90 |
| L6 | 10 | 90 |
| U | unknown | unknown |

- Seeds 42, 1, 2 (they draw the offset direction and the yaw sign, and seed the particle filter).
- The offset is the same for every method at a given (segment, level, seed).

## What each method is told

- **ndt3d** (particle filter): the offset pose as the mean, with the spread the user would give
  for that guess: standard deviation `max(0.5, d)` m in x and y, `max(15, a)` deg in yaw, 2000
  particles. Best setting of `README.md`: ground grid, `ndt_effective_points` 5, `ndt_min_sigma`
  0.10, `alpha1..4` 0.05.
- **baseline** (6-DoF scan-to-map NDT tracker): the offset pose as its start; it has no notion of
  spread.
- **laser2d** (if run; NCLT projected to 2D): as ndt3d (offset mean and spread). Level U: global
  localization by branch-and-bound scan matching (BBS), then tracking.
- Level U for ndt3d and the baseline: not supported (no 3D global localization); reported as
  "not supported", not as a failure of a run.

## Measures (first 180 s of each segment)

- **Converged**: the planar error is below 1 m continuously for at least 10 s, starting within
  the first 120 s.
- **Convergence time**: the start of the first such 10 s window.
- **RMSE after convergence**: planar RMSE from the convergence time to 180 s.
- **Failure**: not converged within 120 s.
- Reported per method and level: converged runs / 12, median convergence time, median RMSE
  after convergence.

## Decision rule (given by the user)

If ndt3d recovers from wrong initial poses where the baseline does not, keep direction A (systems
paper + NCLT evaluation). If there is no difference, narrow to C (laser2d + BBS at the centre, 3D
backends as experimental features).
