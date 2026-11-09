# Weighted A* (bounded-suboptimal search)

Status: measured on the nine gate cells (2026-11-09); 27-cell table at
weights 1.05 and 1.10 pending; decision open (not exact: lengths move).
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

(filled in from the 27-cell table)

## Reproduce

```bash
git checkout ec84bb2
git apply experiments/weighted-astar/patches/0001-weighted-astar-experiment-switch.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT=1.10 scripts/results/gate_short.sh
```
