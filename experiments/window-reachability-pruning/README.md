# Window reachability pruning and window memo

Status: not adopted (closed negative 2026-11-05, before implementation). No
patch: the diagnostic that measured it is in the main tree behind
`PHOTONIC_ROUTER_WINDOW_REACHABILITY_DIAG=1` (`search/astar/window.rs`,
`search/astar/kernel.rs`; read once per process, no cost when off).

## Idea

Speed pass B3. The failing attempts are the lever of the heavy cells (40 %
of ADEPT 64x64's search time). An exact way to make giving up cheap is a
necessary-condition test per search window before the A*: cell-level
reachability (a BFS over passable cells inside the window, passable = not
blocked, or blocked by another net's crossable cell) from the source cell to
the goal cells. If the BFS says unreachable, the A* cannot succeed in that
window and the window is skipped with no change to the result.

B2, the window memo, was the exact companion: a retry re-searches the
windows the failed first attempt exhausted; skipping them and charging the
retry only for the new windows would save the repeated work.

## Measurement

Diagnostic flag that runs the BFS per window and logs its verdict next to
the A* outcome (2026-11-05, speed-pass head on `speed/kernel`):

- Benes 16x16 lidar-pure (workstation): 173 windows, BFS total 81 ms; all 16
  failing windows (8.0 M expansions, all by exhaustion, none capped) are
  cell-reachable, every routed window too. A first version that read the
  obstacle map's static flag alone reported 112 routed windows as
  unreachable: the per-net port-open cells are static in the map and free
  only in the dense grid, so the test has to read the dense grid.
- ADEPT 64x64 lidar-pure (cda-server-4, first 84 windows of the log): every
  failing window is cell-reachable, BFS 1 to 7 ms per window (231 ms total).
- B2 on the ADEPT 64x64 lidar-pure log (1,089 attempts, 899 logged windows):
  only one retry window repeats an exhausted first-attempt window identically
  (1.1 M expansions, 0.1 %). The 2 M first attempts' windows are cut by the
  shrinking per-window cap, not exhausted, so the retry runs them differently
  and there is nothing exact to skip.

## Result and decision

The windows that fail do so for geometric reasons inside a cell-connected
region (headings, bend arms, 45-degree moves through gaps a cell path
threads), on both cell families; a cell-level connectivity test prunes
nothing. The window memo has nothing to memoise. Both closed negative. What
the same log showed instead, that every success comes from window 0, became
the adopted change in `../window-ladder/`.

## Reproduce

```bash
PHOTONIC_ROUTER_WINDOW_REACHABILITY_DIAG=1 .venv/bin/python routing_flow.py benes_16x16_flat --crossing-mode lidar-pure 2>&1 | grep reachab
```
