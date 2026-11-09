# Heuristic tightness: where the kernel's pops go

Status: measured (2026-11-09); the measurement for research item 2 (an
obstacle- and crossing-aware heuristic). Instrumentation only, on branch
`exp/f-profile` (`patches/0001-kernel-f-value-profile.patch` against
`95f1974`), nothing in the main tree.

## Idea

The pattern-pass anatomy (`../pattern-pass/`) put 85 % of the table's
search time into the 1,572 searches above 100 k expansions. A* must pop
every state with f < C* (C* the optimal cost); the pops at f = C* are
tie-breaking, and a tighter admissible heuristic removes pops below C*
only where it lifts their f above C*. So three numbers per search decide
what a better heuristic can do: the share of pops below C* (below 0.99,
0.95, 0.90 C*), the ratio h(start)/C* of the current heuristic, and the
ratio a candidate heuristic would reach. The candidate measured here is the
2D relaxation of research item 2: an 8-connected Dijkstra over the window's
cells from the source to the target, static obstacles as walls, cells
blocked by committed routes passable, with the cheapest crossing price
charged when the path enters a route-blocked region from outside
(`exp_two_d_lower_bound` in the patch). It ignores bends, angles,
footprints and crossing legality, so it is a lower bound on C*; it is
computed once per search and reported, not used.

## What was built

`PHOTONIC_ROUTER_EXP_F_PROFILE=1` makes the kernel keep the f-score of every
pop and, when a search succeeds, print one `exp_f_profile` line: pops,
pops below C* and below 0.999 / 0.99 / 0.95 / 0.90 C*, h(start)/C*, the 2D
length-only bound over C*, the 2D priced bound over C*, and C*.
`scripts/f_profile_summary.py` groups the lines of a run log by search size
and reports pop-weighted shares.

## Measurement

Workstation, Benes 16x16 lidar-pure, ADEPT 16x16 lidar-pure and Benes 8x8
lidar-pure (the nine-cell gate has no search above 1 M expansions; the
16x16 cells have 14 and 2, the 8x8 cell 2). Summaries in `tables/`, one per
cell, grouped by search size with pop-weighted shares. The routes and the
f-distribution are those of the main tree: the instrumentation only
records. The bend weight of the paper cells is 12 per 45 degrees and the
crossing price 200 per crossing (lidar-pure has no planned crossings).

The 2D relaxation went through three versions before it saw the crossings,
and the record keeps them because each says something about the geometry:
a zero-width walker over the cells (1) slips between a committed route's
end and the component it ends on, which a route with a footprint cannot,
so cells touching a static obstacle are closed except within two cells of
the terminals; and (2) slips diagonally through the 8-connected chain of a
45-degree route's core cells, so a diagonal step whose two orthogonal
neighbours are route cells counts as a crossing. Without (2) the priced
bound equalled the length bound on every search: the routes did not
separate source and target in the model at all. With both rules the "walls"
variant (route cells impassable) reaches the target in 5 of 53 Benes 8x8
searches, so the crossings the kernel pays for are forced by the committed
routes, not chosen over a detour. Rule (1) is too strict for the short
searches whose ports sit next to components: 40 of the 53 Benes 8x8 searches
(all in the up-to-10 k class) get no 2D bound at all (reported as 0). A
heuristic for the kernel would have to treat the footprint properly; the
numbers below are for the heavy searches, where the bound exists. On
ADEPT 16x16 the rule closes nearly every search (the MMI layout's ports and
corridors sit against the components), so that cell contributes the
f-distribution and the cost decomposition only, not the 2D bound. The bend
term of the decomposition counts 45-degree units between consecutive
states times the bend weight; where it overshoots the kernel's primitive
bend costs the crossing share comes out a little low (a few ADEPT lines
show a negative remainder).

## Result

1. No tie plateau. In every size class 97 to 100 % of the pops have
   f < C*; in the searches above 1 M pops, 99.9 %. Tie-breaking changes
   cannot remove work; only a heuristic that lifts f above C* can (or a
   bounded-suboptimal rule, `../weighted-astar/`).
2. The heavy searches run with a heuristic that sees less than half of the
   final cost: pop-weighted h(start)/C* is 0.45 on Benes 16x16 (minimum
   0.39) and 0.37 on ADEPT 16x16 above 1 M pops, against 0.76 to 0.83 for
   the searches of 10 k to 100 k pops. 72 to 98 % of the heavy searches'
   pops have f < 0.90 C*.
3. The slack is the crossing price. Above 1 M pops C* is 46 % length, 6 %
   bends and 49 % crossings on Benes 16x16, 39 / 5 / 56 % on ADEPT 16x16,
   50 / 6 / 44 % on Benes 8x8 (one to three crossings at 200 each). The obstacle-aware 2D length bound is no better
   than the Euclidean estimate (0.50 against 0.49 of C*): detours are not
   the gap.
4. A 2D Dijkstra that charges the cheapest crossing price on entering a
   committed route's cells reaches 0.87 to 0.91 of C* on the heavy searches
   (Benes 16x16: 0.875 pop-weighted for 100 k to 1 M pops and 0.912 above
   1 M, minimum 0.863; Benes 8x8: 0.866 and 0.899), where the current
   heuristic reaches 0.45 to 0.66. The bound never exceeded C* (maximum
   ratio 0.972 over the 263 searches of the two 16x16 cells, 0.906 on Benes
   8x8). With route cells as walls the target is unreachable in 231 of the
   263 searches: the crossings are forced.

Decision: research item 2 becomes a proposed item with this measurement as
its case. The heuristic would be a per-search 2D Dijkstra from the target
over the window's cells (a few thousand cells against millions of pops),
with the crossing price on route cells and the footprint rules the kernel
uses; it keeps the optimal cost, so lengths and crossings stay at the
optimum, but routes among equal-cost optima may change, so it is a strategy
switch the 27-cell table decides, not an exact change. Its upper bound is
the share of pops below the new h; the per-state version of the bound (not
only h(start)) is the first thing to measure when it is built.

## Reproduce

```bash
git checkout 95f1974
git am experiments/heuristic-tightness/patches/0001-kernel-f-value-profile.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_EXP_F_PROFILE=1 PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1 RESULTS_ROOT=$PWD/results_fprofile \
  scripts/results/run_and_archive.sh lidar-pure benes_16x16_flat --crossing-mode lidar-pure
.venv/bin/python experiments/heuristic-tightness/scripts/f_profile_summary.py results_fprofile/benes_16x16_flat/lidar-pure/*/run.log
```
