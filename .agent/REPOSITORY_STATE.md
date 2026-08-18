# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
every agent stop, pause, or handoff. It does not replace the active ExecPlan.

Keep this file compact. Do not append a new dated section per stop; overwrite
the sections below in place so this file always reflects only the current
state. Detailed run-by-run history belongs in the active ExecPlan's `Progress`
and `Surprises & Discoveries` sections (per `.agent/PLANS.md`), which are
allowed to grow because they are the plan's own record, not this file's job.
Older history is still recoverable from `git log -p -- .agent/REPOSITORY_STATE.md`
if it is ever needed.

## Current Snapshot

- Date: 2026-08-18
- Branch: `crossings/verification-foundation`
- Current HEAD: `75edf33` (`Fix rust_blocked_cell_handle drop and stale
  blocked_cells guard in heater clearance expansion`)
- Working tree is clean (`git status --short` empty).
- All three readability plans are complete and committed:
  - `.agent/execplans/2026-08-11-refactor-python-routing-flow.md` (`routing_flow.py`
    split, commit `be95ee1`, including the fix for a 4-name monkeypatch
    regression found while validating it).
  - `.agent/execplans/2026-08-17-restructure-translation-route-rust.md`
    ("Phase 1": 7 commits `60047a3`..`bcce21e`, `translation/route_rust.py`
    11,327 -> 6,865 lines: moved ~100 free-standing helper functions into
    topic modules).
  - `.agent/execplans/2026-08-18-restructure-route-nets-rust.md` ("Phase 2":
    9 commits `82990b7`..`9925249`, `route_nets_rust` -- one ~6,218-line
    function with ~120 anonymous nested closures sharing hidden state --
    became a 103-method class, `_RouteNetsRustSession`, with every piece of
    shared state a named `self.attr`; `translation/route_rust.py` is now
    7,101 lines, an expected and correct increase from the `self.`/`def
    ...(self, ...)` boilerplate closures need as real methods, not a
    regression).
- On 2026-08-18, after the three readability plans, the user redirected
  priorities toward a **functionality-by-functionality correctness
  walkthrough** of the Python routing pipeline (not test-by-test, since
  tests themselves may be wrong): compare each stage's code against its
  intended behavior, before path-length matching / electrical routing
  (explicitly deferred). Proposed stage order: (1) benchmark loading,
  (2) `layout_from_schematic`, (3) obstacle map building, (4) grid
  snapping/port-to-grid-state, (5) routing/A*/crossings, (6) endpoint
  correction/port snapping, (7) geometry realization, (8) verification.
  Not yet written up as a formal ExecPlan; tracked ad hoc so far. Two real
  bugs found and fixed this way, each via the Claude+Codex flow with
  independent re-verification:
  - Commit `fae111d`: `translation/layout_from_schematic.py`'s `anchor is
    None` branch called `ref.rotate()`/`ref.mirror()` (which pivot around
    the global origin, not the reference's current position) *after*
    `movex`/`movey`, producing wrong placements for any non-zero
    `(x, y)` combined with non-zero rotation or mirror. Fixed by
    reordering to transform-then-translate, matching the already-correct
    `anchor is not None` branch. No live benchmark was affected (all
    avoid the dangerous combination); new regression tests added in
    `tests/test_layout_from_schematic.py`.
  - Commit `75edf33`: `python/photonic_router/static_obstacle_builder.py`'s
    `_apply_perpendicular_heater_clearance` dropped `rust_blocked_cell_handle`
    (silently `None`) and gated newly-expanded `blocked_cells` on the
    *pre-expansion* set's truthiness in its clearance-applied return
    branch. No live benchmark was affected today because the sole
    caller's merge step happens to reconstruct a correct handle from
    rects in the common case, but that was incidental, not guaranteed.
    Fixed to be correct on its own; new tests in
    `tests/test_static_obstacle_builder.py`.
  Currently mid-**Stage 3 (obstacle map building)**: the rest of
  `static_obstacle_builder.py` (1,048 lines; read through ~line 690 of the
  main flow so far) has not yet been fully walked, and Stages 4-8 have not
  started.
- Full test suite baseline is now `23 failed, 311 passed, 1 skipped` (was
  `23 failed, 307 passed, 1 skipped` at Phase 2 completion; the walkthrough
  has added 4 new passing tests across the two fixes above, same 23
  pre-existing failures, unchanged). Every slice/milestone across Phase 1
  and Phase 2, plus both walkthrough fixes, was independently re-verified
  by Claude (not just trusted from Codex's self-report).
- No active ExecPlan right now; the walkthrough above is the de facto
  current work but has not been written up as one yet (see Next
  Engineering Step).

## Current Goal

Make TUMPhotonicRouter a very fast, verified photonic router. The active
phase (per `.agent/PROJECT_GOAL.md`) is router-discovered optical crossings on
`benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`, with final-geometry
verification and PDK/gdsfactory crossing component realization. On 2026-08-17
the user set a prerequisite priority: the codebase needed to be readable and
structured with known good practice before more crossing-verification
feature work continues. That readability work (three plans, see Current
Snapshot) is now done. On 2026-08-18 the user additionally set a further,
explicitly *future* direction in `.agent/PROJECT_GOAL.md`'s "Future
Architecture Initiative" section: extracting real swappable interfaces
(Python `Protocol`/`ABC`, Rust `trait`) for routing stages (obstacle map
building, grid snapping, A* search, geometry realization, path-length
matching), each independently unit-tested, matching the header/interface/test
discipline the user is used to from C++. That initiative has not started;
Phase 2's `_RouteNetsRustSession` class is a necessary precursor to it (named
methods with explicit dependencies are judgeable for "should this be its own
module" in a way anonymous closures were not), not a substitute for it. See
Next Engineering Step for the open decision on what to do next.

## Worktree State

Clean. Nothing uncommitted. Two known pre-existing, unrelated test/benchmark
failures remain, both already covered by the 23-item baseline failure list
and neither caused by any of the three completed plans:

- `tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested`
  fails because `RouteAttemptRecord.as_dict()` now includes `crossing_hotpath_*`
  fields the test's expected literal does not list.
- The default `TOY` CLI benchmark fails at `gc1_to_mmi_in2` with `No route
  found` on this branch. Do not use `TOY` as a smoke test here.

## Recent Session Notes

On 2026-08-17, a Claude Code session (not Codex) added a Claude+Codex
orchestration flow on top of the existing `.agent/` workflow: Claude plays the
Orchestrator/Planner/Explorer/Reviewer/QA roles and delegates only the
Implementation Engineer role to Codex CLI via `.agent/scripts/codex_task.sh`.
See `.agent/CLAUDE_CODEX_FLOW.md` for the mechanics. The same session found
and fixed staleness in the pre-existing `.agent/` docs (this file had grown
to over 10,000 lines of un-pruned log entries; `ORCHESTRATOR.md` had a stale
active-plan pointer). A full audit of the two large, older crossing-
verification-era ExecPlans (`.agent/execplans/2026-07-10-crossing-verification-foundation.md`,
~175KB, and `.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md`,
~59KB) for the same kind of staleness was explicitly deferred; treat their
internal detail as unverified until that audit happens.

Across 2026-08-17 to 2026-08-18, executed all three readability plans (see
Current Snapshot) through the Claude+Codex flow: every slice/milestone
written as a task file with an exact function/name list, handed to Codex,
independently re-verified by Claude, then committed individually. Notable
process lessons, kept here since they apply beyond any one plan:

- One environment blocker on the very first Codex call: `bwrap` (Codex's
  sandbox) failed with a user-namespace error caused by an Ubuntu 24.04
  AppArmor default. Fixed once with a scoped `bwrap`-specific AppArmor
  profile (not a blanket relaxation); did not recur.
- Several times, a Claude-written task file's assumption turned out to be
  wrong (which tests were pre-existing failures; whether three cache helper
  closures were nested inside `_rect_ranges_by_y` or were actually its
  siblings; an "unexpected" nested closure that was actually a known,
  intentional exception never mentioned in that task file). Every time,
  Codex correctly stopped and reported the discrepancy rather than guessing,
  and Claude investigated, corrected the record, and re-dispatched rather
  than blaming the tool. This is recorded as a positive signal for the
  Claude+Codex role boundary working as designed, and as a reminder that
  task files need to be cross-checked against the actual current source, not
  written from memory or from an earlier report.
- A naive, non-scope-correct AST analysis script (checking "does this name
  appear again anywhere" instead of "is this name genuinely unshadowed at
  this point in the scope chain") produced large numbers of false positives
  twice during Phase 2 planning (144 vs. 68 names captured by closures; many
  false "crossing" hits analyzing the Phase 2 dispatch-block extraction).
  Both times the fix was the same: track each scope's own locals (parameters
  plus direct assignments, not descending into further-nested scopes) and
  only count a reference as real if unshadowed all the way to the scope
  being tested. Worth budgeting for writing this kind of script twice, not
  once, in similar future work.

## Reference Branches

The branch `baseline/lidar-pure-crossings` contains a WIP prototype snapshot
at commit `69ab9fd` ("wip: snapshot experimental lidar-pure crossing
prototype"). Treat it as reference material only, not a merge candidate. If
code is needed from it: audit the relevant diff, confirm the idea fits
`.agent/PROJECT_GOAL.md` and the active ExecPlan, port the smallest useful
piece manually or via a narrow reviewed cherry-pick, and add focused
verification before considering the port complete.

`.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md` is marked
reference-only in its own status note (added 2026-07-10) unless the user
explicitly resumes it.

## Next Engineering Step

Active, ongoing (not yet a formal ExecPlan): continue the Python
correctness walkthrough at **Stage 3 (obstacle map building)**, finishing
a full read of `python/photonic_router/static_obstacle_builder.py`
(1,048 lines; the main flow and the heater-clearance split path have been
read, the rest -- e.g. `build_static_obstacle_map_python_from_extracted`'s
body past line 391, port-open-cell logic, bbox-cell materialization
details -- has not), before moving to Stage 4 (grid snapping/port-to-grid
state). Consider writing this walkthrough up as a proper ExecPlan once a
stage or two more of findings accumulate, for continuity across sessions.

Deferred candidates, not in a mandated order:

1. The TOY benchmark's `gc1_to_mmi_in2` "No route found" failure (one of
   the 23 baseline failures) has an unresolved discrepancy between
   `FAILED.txt` (100% static overlap) and `diagnostics.txt` (0% overlap,
   but target-approach footprints show static blockers not visible in the
   base obstacle SVG); hypothesized "foreign port keepout" (936 cells) as
   the explanation, not confirmed. Revisit at Stage 5 (Routing) of the
   walkthrough rather than in isolation.
2. Start the "Future Architecture Initiative" from `.agent/PROJECT_GOAL.md`:
   extract real swappable interfaces (obstacle map building, grid snapping,
   A* search, geometry realization, path-length matching) with independent
   unit-test coverage. Explicitly deferred by the user until after the
   correctness walkthrough. Nothing scoped yet beyond `PROJECT_GOAL.md`.
3. A smaller, optional continuation of Phase 2: decompose
   `_write_route_diagnostics` (still 486 lines) and `_route_attempt_diagnostics`
   (still 247 lines), or split `run()` itself (1,104 lines) into a few named
   phase methods. Not started, not committed to.
4. Phase 3 for the large Rust files (`src/py_router.rs` at 17,399 lines,
   `src/astar.rs` at 10,388 lines, `src/geometry_realization.rs` at 8,125
   lines) -- the same kind of readability work as Phases 1-2, agreed to
   come after the current Python correctness walkthrough, not started.
5. Resume the crossing-verification-foundation objective (the pre-readability-
   work priority). Per the last recorded stable-benchmark checkpoint in git
   history (commit `a29dc00`), `benes_8x8`, `benes_16x16`, `multiportmmi_8x8`,
   and `multiportmmi_16x16` are marked full-run stable; `multiportmmi_32x32`
   is not yet, with route 156 / `n_155` the last known next slow/hanging
   route to investigate. Re-verify this is still current before acting on
   it, since it predates this cleanup.
