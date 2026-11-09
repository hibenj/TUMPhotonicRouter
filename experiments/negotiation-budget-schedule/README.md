# Negotiation budget schedule: the first attempt runs at the retry budget

Status: adopted as an exact change (2026-11-09); see "Result" for the table.
No patch: the code is in the main tree (`engine/negotiation/loop_.rs`,
`engine/negotiation/budget.rs`, `Router::attempt_max_heap_pops`).

## Idea

Algorithm pass, item 1 (the give-up schedule). A fresh net's per-round
sequence in the negotiated loop is: a first plain attempt at `budget_first`
(2 M iterations), on failure one retry at `budget_first_retry` (10 M), then
probe, local rip-up and the post-rip-up search. The anatomy of the archived
27-cell table at the current pins (`scripts/attempt_anatomy.py` over the
`native_negotiated_search` trace lines, `tables/anatomy_current_pins_60f4957_unified.md`)
shows where the search time goes:

- 42,572 attempts, 3,210 s of search; attempts that failed take 807 s
  (25 %). The failed first attempts split into three classes by their
  error: 147 exhaust the 2 M cap ("No legal LiDAR crossing route found",
  485 s, 3.3 s each), 61 are instant rejections of the simple straight
  route at commit ("Illegal realized crossing", 0 s), 23 exhaust their
  open set ("No route found", 11 s).
- The retry after a failed first attempt routes 189 of 231 nets; the
  successful retries take a median of 2.3 M and at most 7.7 M expansions.
  The 42 retries that fail cost 260 s.
- With window 0 only (the default since `60f4957`) the retry is the same
  search call as the first attempt with a larger iteration cap: `SearchCall`
  is built identically by `PlainAttempt::first` and `::first_retry`, the
  budget reaches the kernel only as `max_iterations = min(20 M, budget)`
  (`window.rs::effective_max_iterations`), and the kernel's A* is
  deterministic. So a first attempt that exhausts 2 M iterations and a
  retry that then routes at, say, 4 M have spent the first 2 M iterations
  twice: 485 s over the table, 185 s of ADEPT 64x64 lidar-pure's 1,462 s of
  search, 106 s of Benes 32x32 lidar-pure's 432 s.

Speed pass B1 (`../negotiation-first-attempt-full-budget/`) had already run
the first attempt at 10 M, but skipped the retry unconditionally; the six
cells that moved and the Benes 16x16 contribution 1 slowdown came from the
instant commit rejections: their retry is a different search (the straight
probe route was rejected; the retry runs the A*) and must still run.

## What was built

The first attempt of a retry-eligible net (`failed_count == 0`, not the
last round) runs at `budget_first_retry` when that is the larger cap
(`budget::first_attempt_runs_at_retry_budget`). The router records the
largest `heap_pops` (the kernel's iteration counter) of any kernel search
during the attempt (`Router::attempt_max_heap_pops`, reset in
`run_plain_attempt`, noted at the four `search_engine.search` sites). When
the merged first attempt fails, the retry runs only if every search stayed
within `budget_first` (`budget::retry_after_failed_first`):

| merged first attempt at 10 M | two-step sequence would have done | retry now |
| --- | --- | --- |
| routes within 2 M iterations | routes identically at 2 M | none needed |
| routes beyond 2 M | 2 M attempt capped, retry routes identically | none |
| fails with every search within 2 M (commit rejection, exhausted open set) | 2 M attempt fails the same way, retry runs | runs (identical call) |
| fails with a search beyond 2 M | 2 M attempt capped, retry is this search and fails | none |

The kernel fails at `iterations > cap`, so a goal pop at exactly the cap
succeeds, and the rule uses `<=`. Routes, failures and the map after each
net are identical to the two-step sequence; what changes is the number of
attempts in the trace and the report counts (`search_attempts`,
`search_failures`), which are not pinned.

## Measurement

Nine-cell gate and PLM gate exact on the workstation; 27-cell table on
cda-server-4 against the pins, run concurrently with a reference table of
the previous head on the same machine for the wall times
(`tables/reference.json`, `tables/merged_first_attempt.json`, rendering in
`tables/table.md`).

## Result

27 of 27 cells identical to the reference and to the pins in crossings,
length and errors (`tables/table.md`, `compare_paper.py` "all reproduced
cells match" for both tables). Total wall 0.89 of the reference, all of it
on the cells with budget-exhausted first attempts; every other cell is
within the shared-server noise:

| cell | reference wall s | merged wall s | ratio |
| --- | --- | --- | --- |
| ADEPT 64x64 lidar-pure | 1,621 | 1,457 | 0.90 |
| ADEPT 64x64 contribution 1 | 540 | 452 | 0.84 |
| Benes 32x32 lidar-pure | 490 | 380 | 0.78 |
| Benes 32x32 contribution 1 | 368 | 299 | 0.81 |
| ADEPT 32x32 lidar-pure | 189 | 158 | 0.84 |
| Benes 16x16 lidar-pure | 75 | 63 | 0.84 |

The anatomy of the two tables (`tables/anatomy_reference_b478dac.md`,
`tables/anatomy_merged_1377ad6.md`) shows the mechanism: search time over
all cells 3,228 s to 2,746 s, time in failed attempts 807 s (25 %) to 325 s
(12 %). The 148 attempts that disappeared are the budget-exhausted first
attempts (every cell's "plain, failed" row at 2 M is gone; the remaining
first-attempt failures are the instant commit rejections at 0 s and the
ADEPT 64x64 nets that fail at 10 M, whose retry never routed). On Benes
32x32 lidar-pure the 52 retries of the reference became 17 (the commit
rejections) and the failed-attempt time 107 s became 0 s; on ADEPT 64x64
lidar-pure the 59 failed first attempts at 2 M became 18 at 10 M, which are
the nets the reference's retry also failed on.

The loop and search-time deltas are smaller than the attempt-time delta:
the search-time counter sums per-attempt elapsed seconds, and the merged
first attempt of a net that used to need the retry now carries the retry's
whole expansion count under one trace line.

## Reproduce

```bash
.venv/bin/python experiments/negotiation-budget-schedule/scripts/attempt_anatomy.py <results_root> --cells heavy
RESULTS_ROOT=$PWD/results_merged scripts/results/reproduce_paper.sh all
.venv/bin/python scripts/results/compare_paper.py results_merged docs/paper_table_sources.json
```
