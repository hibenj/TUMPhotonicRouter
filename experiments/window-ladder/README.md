# Window ladder: window 0 only

Status: adopted (owner decision 2026-11-05, commit `60f4957`; three cells
re-pinned as `current` in `docs/paper_table_sources.json`). The code is in
the main tree: kernel defaults `routing_window_max_expansions = 0` and
`routing_window_fallback_full_grid = false`, the `SearchTuning` env switches
`PHOTONIC_ROUTER_ROUTING_WINDOW_MAX_EXPANSIONS` and
`PHOTONIC_ROUTER_ROUTING_WINDOW_FALLBACK_FULL_GRID` (unset = kernel default),
and the ablation variant `paper_ladder` (3 expansions, fallback on) that
reproduces the paper's ladder.

## Idea

Speed pass S4. The per-window log of ADEPT 64x64 lidar-pure (1,089 attempts,
899 logged windows, 830 M window expansions) showed that every success of
every attempt kind came from window 0, the first window of the ladder: plain
2 M attempts routed 370 in window 0 (plus 466 simple routes), the 10 M
retries 44 of 44 in window 0, the round-2 30 M attempts 92 in window 0 and
one in window 1, post-rip-up and probe attempts all in window 0; the
full-grid fallback never routed anything. Windows 1 to 3 and the fallback
only consumed the budget tail of attempts that had already lost in window 0:
114 M expansions in the 56 failed 2 M attempts and 145 M in the 15 failed
10 M retries, of 1,621 s attempt time. The strategy switch: window 0 only,
no fallback. Not exact (that one window-1 success turns into a failure and
a reroute), so the 27-cell table decided.

## Measurement

Three 27-cell tables on cda-server-4, 2026-11-05, run concurrently with each
other: `tables/reference.json` (speed-pass head `61a5f69`, the paper's
ladder), `tables/window0_only.json` and `tables/window0_full_budget.json`
(`082b59e` with the switches; the second adds the first attempt at the
retry budget, `../negotiation-first-attempt-full-budget/`). Rendering:
`tables/table.md`.

`window0_only`: 24 of 27 cells identical; ADEPT 64x64 lidar-pure 217 to 213
crossings and 1,305,628 to 1,305,375 um (-0.02 %), ADEPT 32x32 contribution 1
+0.003 % length, ADEPT 64x64 contribution 1 -0.0001 %; 0 verifier errors
everywhere. Wall against the reference: ADEPT 64x64 lidar-pure 1809 to
1584 s (0.88), ADEPT 64x64 contribution 1 0.84, ADEPT 32x32 contribution 1
148 to 69 s (0.47), ADEPT 128x128 contribution 2 0.91, Benes cells 0.96 to
1.05, total 0.91.

`window0_full_budget`: total 0.88, but Benes 16x16 contribution 1 doubles
(25 to 49 s) and seven cells move (the six length moves of the full-budget
switch plus the ladder's).

## Result and decision

Window 0 only removes a ladder tail that never produced a route on any
cell, costs nothing on the Benes cells and makes nothing worse (one cell
gains four crossings and gets shorter). Adopted as the default; the paper's
ladder stays reproducible as the ablation variant `paper_ladder`. The
re-pinned cells carry a `current` block with `since` naming this change.
The first-attempt-at-full-budget switch stayed off.

## Reproduce

```bash
git checkout 60f4957
.venv/bin/maturin develop --release
RESULTS_ROOT=$PWD/results_window0 scripts/results/reproduce_paper.sh all      # the default now
scripts/results/ablation.sh paper_ladder all                                 # the paper's ladder
```
