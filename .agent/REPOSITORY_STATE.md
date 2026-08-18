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
- Current HEAD: `087ba3f` (`Add automatic clearance-corridor diagnostic to
  failed-route logs`)
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
  Stages 3 (obstacle map building) and 4 (grid snapping / port-to-grid
  state) are now fully read with no further bugs found (Stage 4 has one
  flagged-but-dormant oddity, see below). Stage 5 (routing / A* / crossing
  verification) is large enough and spans Python and Rust closely enough
  that the user asked for a proper tracked ExecPlan before continuing
  rather than staying ad hoc; see
  `.agent/execplans/2026-08-18-stage5-routing-crossing-correctness-walkthrough.md`,
  now the active ExecPlan and now complete for its two originally-scoped
  milestones. Milestone 0 (orientation) is done. Milestone 1 (the TOY
  benchmark's `gc1_to_mmi_in2` "No route found" case study) ruled out
  crossing-legality, foreign-port-keepout, dense-obstacle-grid-cap, and
  routing-window-bounds as causes, found and fixed (commit `8dfb132`, a
  real Rust change, independently re-verified end to end) a real
  diagnostics bug along the way (failed plain-A* search attempts were
  silently discarding their real `RouteSearchStats`, so every failure just
  said "No route found" with no numbers), and then, with that fixed, used
  a direct geometric BFS analysis (independent of A* internals -- capture
  the real obstacle state via a monkeypatch, then check cell connectivity
  with and without simulated bend-radius clearance) to conclusively
  localize the failure: not a crossing-verification bug, not a router
  logic bug of any kind considered in this plan, but a genuine clearance
  shortage at the target port `mmi_0,o1`'s immediate approach -- a bare
  cell-connectivity corridor exists, but it disappears once ~2 cells of
  clearance (matching the ~2.5-cell default bend radius) is required, and
  the target's own reachable region at that clearance is a single isolated
  cell. See that ExecPlan's Outcomes & Retrospective for the full evidence
  trail. Two explanations remain open, not yet distinguished (a candidate
  Milestone 2, not started): `TOY`'s placement is simply too tight for the
  default bend radius at this grid resolution (a benchmark fact), or the
  port-opening/lane-width logic does not scale with the configured bend
  radius (a real, more general bug if true). The BFS-with-inflation
  technique used to reach that answer was then turned into a durable
  feature (commit `087ba3f`): `_write_failed_log` now automatically writes
  `corridor_clearance_*` fields into every `FAILED.txt`, so future "why
  won't this route" investigations don't need a one-off script. Stages 6-8
  (endpoint correction, geometry realization, verification) have not
  started. Also newly discovered and worth knowing before any further
  Rust work: a
  separate Rust unit-test baseline (`cargo test --lib`, needs
  `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu` since the checked-in
  `rust-toolchain.toml` is pinned to a Windows target) of `311 passed, 9
  failed`, all 9 failures crossing-related and confirmed pre-existing --
  a candidate for its own future milestone.
- One low-priority, unconfirmed finding from Stage 4, recorded in the Stage 5
  plan's Surprises & Discoveries rather than fixed: `_snap_same_heading_minimum_bend_offset`
  in `translation/route_rust.py` (around line 1081) snaps to `min_offset_cells + 1`
  cells of separation in its "missing == 1" case, where its own docstring
  implies `min_offset_cells` (no `+1`) is the true minimum -- a plausible
  off-by-one, but confirmed dormant (fires 0/81 times on `heater_s_mod`) and
  not the cause of any known test failure, so left alone pending a live case.
- Full Python test suite baseline is now `23 failed, 314 passed, 1 skipped`
  (was `23 failed, 307 passed, 1 skipped` at Phase 2 completion; the
  walkthrough and the Stage 5 fixes have added 7 new passing tests total
  across four changes -- `fae111d`, `75edf33`, `8dfb132`, `087ba3f` --
  same 23 pre-existing failures, unchanged throughout). Every change was
  independently re-verified by Claude (not just trusted from Codex's
  self-report). The separate Rust unit-test baseline (`cargo test --lib`)
  is `311 passed, 9 failed`, all 9 failures pre-existing and
  crossing-related (see above).
- Active ExecPlan:
  `.agent/execplans/2026-08-18-unify-port-access-region-computation.md`
  (see Next Engineering Step). Grew out of chasing the TOY/`mmi_0,o1`
  clearance question further: digging into *why* widening
  `port_lane_half_width_cells` (the candidate Milestone 2 fix from the
  Stage 5 plan, designed and implemented by Codex but never committed --
  it is correct in isolation but insufficient) had zero effect on the
  real opened-cell count revealed that "how big is a port's access/keepout
  region" is computed by seven independently-parameterized, uncoordinated
  pieces of logic across `src/py_router.rs` and `translation/route_rust.py`
  (full breakdown in the new plan's Context and Orientation), not one
  bug. The working tree currently has that uncommitted, insufficient
  precursor fix (`translation/route_rust.py:6195` plus its still-failing
  test in `tests/test_route_rust_opened_cells.py`) sitting as-is,
  deliberately not committed or discarded -- the new plan's Milestone 2
  explicitly decides what becomes of it (fold into the unified design, or
  delete as superseded). The Stage 5 plan itself is complete for its own
  two originally-scoped milestones and is now background/prerequisite
  reading rather than the actively-executed plan.

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
  found` on this branch. Do not use `TOY` as a smoke test here. Root cause
  (confirmed 2026-08-18, see the Stage 5 ExecPlan's Milestone 1): a real
  clearance shortage at the target port `mmi_0,o1`'s immediate approach,
  not a router bug -- the corridor exists at the bare-cell level but not
  once ~2 cells of clearance (matching the default bend radius) is
  required. Not yet fixed or confirmed as benchmark-vs-code; see that
  plan's candidate Milestone 2.

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

Follow the active ExecPlan,
`.agent/execplans/2026-08-18-unify-port-access-region-computation.md`:
start its Milestone 1 (add the new unified port-access/keepout sizing
computation in Rust and Python, additive only, no behavior change yet --
see that plan's Plan of Work for the exact function shapes agreed with
the user). Not started.

The prior active plan,
`.agent/execplans/2026-08-18-stage5-routing-crossing-correctness-walkthrough.md`,
has completed both of its originally-scoped milestones (orientation, and
the `gc1_to_mmi_in2` case study -- concluded as a real clearance shortage
at the target port's approach, not a router bug, later refined into the
seven-piece architectural finding that motivated the new plan above; see
that plan's Outcomes & Retrospective). Once the new plan's Milestone 2
resolves the `TOY` question for real, mark that plan's own candidate
Milestone 2 as resolved/superseded with a cross-reference, per the new
plan's own Milestone 2 instructions.

Other candidates, not in a mandated order:

1. The 9 pre-existing failing Rust crossing tests discovered while
   verifying the Stage 5 plan's diagnostics fix (`cargo test --lib`, see
   Current Snapshot) -- a separate, unexplored thread in the same
   crossing-legality code area. Not started.
2. Continue the broader Python-and-Rust correctness walkthrough into
   Stages 6-8 (endpoint correction, geometry realization, verification),
   or into a deeper systematic read of `src/astar.rs`/`src/py_router.rs`
   module by module (matching how Stages 1-4 were done), rather than
   staying driven by one specific benchmark case. Not started.

Deferred candidates, not in a mandated order:

1. Start the "Future Architecture Initiative" from `.agent/PROJECT_GOAL.md`:
   extract real swappable interfaces (obstacle map building, grid snapping,
   A* search, geometry realization, path-length matching) with independent
   unit-test coverage. Explicitly deferred by the user until after the
   correctness walkthrough. Nothing scoped yet beyond `PROJECT_GOAL.md`.
2. A smaller, optional continuation of Phase 2: decompose
   `_write_route_diagnostics` (still 486 lines) and `_route_attempt_diagnostics`
   (still 247 lines), or split `run()` itself (1,104 lines) into a few named
   phase methods. Not started, not committed to.
3. Phase 3 for the large Rust files (`src/py_router.rs` at 17,399 lines,
   `src/astar.rs` at 10,388 lines, `src/geometry_realization.rs` at 8,125
   lines) -- the same kind of readability work as Phases 1-2, agreed to
   come after the current Python correctness walkthrough, not started.
   Milestone 0 of the active Stage 5 plan will end up reading parts of
   `src/py_router.rs` and `src/astar.rs` anyway; note anything relevant to
   this future readability pass there too.
4. Resume the crossing-verification-foundation objective (the pre-readability-
   work priority). Per the last recorded stable-benchmark checkpoint in git
   history (commit `a29dc00`), `benes_8x8`, `benes_16x16`, `multiportmmi_8x8`,
   and `multiportmmi_16x16` are marked full-run stable; `multiportmmi_32x32`
   is not yet, with route 156 / `n_155` the last known next slow/hanging
   route to investigate. Re-verify this is still current before acting on
   it, since it predates this cleanup.
