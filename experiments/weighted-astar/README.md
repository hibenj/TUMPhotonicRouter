# Weighted A* (bounded-suboptimal search)

Status: not adopted (2026-11-09). Twenty-seven-cell tables at weights 1.05
and 1.10: four cells fail at either weight, the remaining lengths move by
up to 0.7 %, and the total wall is 0.94 at 1.05 and 1.13 at 1.10.
Patch `patches/0001-weighted-astar-experiment-switch.patch` against
`ec84bb2`: `PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT=<w>` multiplies the kernel's
admissible heuristic by `w` (`search/astar/heuristic.rs`,
`SearchHeuristic::weight`, read once per process; unset = 1.0, which changes
nothing). The branch `exp/weighted-astar` carries the switch for the table.

## Idea

Research item 6 of the algorithm pass. The kernel instrumentation
(2026-10-29) showed the heavy searches spending 200 to 6,000 expansions per
route cell with cost over h(start) of 1.17 to 2.02: the routes are nearly
straight, the heuristic sees the geometry but not the bend and crossing
pricing, and the exact A* spends its time proving optimality. Weighted A*
with factor `w` finds a route whose cost is at most `w` times the optimum
and expands far fewer states when the heuristic is informative. The price
is paid in length (and possibly crossings), which the paper table pins.

## Measurement

Nine-cell gate at weights 1.05, 1.10 and 1.25 (`tables/gate_weights.md`):

- Crossings identical to the pins and 0 verifier errors at every weight.
- Lengths: at 1.05, Benes unchanged (8x8 lidar-pure -0.03 um), ADEPT 8x8
  +0.19 / +0.13 / +0.10 %; at 1.10, ADEPT lidar-pure +0.72 %, contribution
  1 -0.02 %, contribution 2 +0.10 %; at 1.25, Benes 4x4 lidar-pure -0.06 %,
  8x8 lidar-pure +0.16 %, ADEPT +0.54 / +0.01 / +0.10 %.
- Routing loop over the nine cells: 30.9 s at 1.0, 28.9 s (0.94) at 1.05,
  28.4 s (0.92) at 1.10, 26.0 s (0.84) at 1.25. Benes 8x8 lidar-pure 13.9
  to 10.1 s at 1.25 (0.73); ADEPT 8x8 lidar-pure gets slower at 1.10 and
  1.25 (6.3 to 6.9 s, 119 to 122 attempts: the suboptimal routes cost
  retries).

Reading so far: the trade is real but small on the gate cells (-8 % time
for +0.1 to +0.7 % length at 1.10); whether the heavy cells, where the
exact search spends its minutes, gain more is what the 27-cell table says.

## Result

Two 27-cell tables on cda-server-4 (commit `69ab70d`, 2026-11-09 14:04 to
16:00, both weights concurrently on the shared machine, so walls are
indicative only): `tables/weight_1_05.json`, `tables/weight_1_10.json`,
rendered against the budget-schedule table of the same tree without the
switch (`tables/reference_1377ad6.json`) in `tables/table.md`.

- Four cells fail at both weights. ADEPT 16x16 lidar-pure ends with four
  verifier errors: the grid-to-port endpoint correction finds no
  realizable centerline for one net and a neighbouring net crosses its own
  path (64 crossings against 63 pinned). ADEPT 64x64 lidar-pure runs ten
  negotiation rounds without converging (2,482 s at 1.05 and 3,075 s at
  1.10 against 1,457 s and three rounds for the exact search). ADEPT 64x64
  contribution 2 leaves 4 (1.05) and 15 (1.10) nets unrouted after ten
  rounds where the exact search needs one round; ADEPT 128x128
  contribution 2 does not converge either (eight global rip-ups, ten
  rounds). The suboptimal routes are the cause, not the speed: a
  bounded-suboptimal route takes a different shape near the ports and
  through the preplaced crossing grids, and the repair machinery that
  follows (endpoint correction, rip-up) was tuned on optimal routes.
- Over the 23 cells that finish: 5 identical to the reference at either
  weight, geometric mean length 1.0002 (1.05) and 1.0006 (1.10), extremes
  ADEPT 8x8 lidar-pure +0.72 % and ADEPT 16x16 contribution 1 -0.31 % at
  1.10; Benes 32x32 gains 2 to 6 crossings at either weight.
- Wall: 0.94 in total at 1.05 and 1.13 at 1.10. The gains sit on the medium
  cells (Benes 16x16 contribution 1 0.58 / 0.50, ADEPT 32x32 contribution 1
  0.77 / 0.53, ADEPT 64x64 contribution 1 0.59 at 1.05); the heavy
  lidar-pure cells get slower (Benes 32x32 lidar-pure 1.24 / 1.33, ADEPT
  64x64 contribution 1 1.69 at 1.10) because the suboptimal routes cost
  retries and rip-up rounds, the same effect the gate saw on ADEPT 8x8.

## Decision

Not adopted. A fixed heuristic weight is not a usable default: it fails
four paper cells and does not pay for itself on the heavy cells. The
switch stays on the branch `exp/weighted-astar` (69ab70d) and as the patch
here; the main tree carries no weight. What the table does say for the
algorithm pass: the exact search's time on the heavy cells is not slack a
weight can remove, the search needs a tighter admissible heuristic
(`experiments/heuristic-tightness/`, research item 2), not a looser one.

## Reproduce

```bash
git checkout ec84bb2
git apply experiments/weighted-astar/patches/0001-weighted-astar-experiment-switch.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT=1.10 scripts/results/gate_short.sh
# the 27-cell tables
PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT=1.10 RESULTS_ROOT=$PWD/results_wastar_110 scripts/results/reproduce_paper.sh all
.venv/bin/python experiments/record_table.py results_wastar_110 experiments/weighted-astar/tables/weight_1_10.json --commit 69ab70d --machine "<machine>" --switch PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT=1.10
.venv/bin/python experiments/table.py experiments/weighted-astar/tables/reference_1377ad6.json experiments/weighted-astar/tables/weight_1_10.json
```
