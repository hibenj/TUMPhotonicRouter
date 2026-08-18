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
- Current HEAD: `20aab29` (`routing: fix crossing-aware endpoint correction
  rejecting a valid port-exit fix`)
- Working tree is clean (`git status --short` empty).
- Process docs revised after the user asked whether the dense-port-runway
  work followed the documented Claude+Codex flow (it didn't, in two
  concrete ways) and explicitly authorized adjusting the flow docs, not
  just noting the gap: `.agent/CLAUDE_CODEX_FLOW.md` gained "The
  diagnosis/implementation boundary" (diagnosis/probe-script iteration
  stays with Claude; the moment a fix is bounded and well-specified, an
  explicit dispatch-to-Codex-or-record-why decision is required, not a
  silent default to self-implementation); `.agent/WORKFLOW.md`'s
  Convergence policy now scopes its "fails twice" escalation trigger to
  validated implementation attempts (not disposable diagnostic scripts,
  which had made the trigger read as violated when it wasn't), and its
  Routing Verification Gate now requires an explicit `Verdict:
  PASS/FAIL/BLOCKED/INCONCLUSIVE` line read from the actual
  `build/verification/*.json` reports before routing work is called done
  -- a passing `pytest` run or exit code 0 does not satisfy it, even
  solo. `.agent/roles/harness.md`'s verdict-line format was tightened to
  match.
- `.agent/execplans/2026-08-18-dense-port-runway-clearance-reach.md` is
  **complete** (commits `ecc4b79`, `af00aeb`). It fixed the
  `multiportmmi_8x8` `n_24` / `multiportmmi_16x16` `n_48` finding left open
  by the unify-port-access-region plan: ports staggered by
  `_dense_target_port_runway_lengths`/`_dense_source_port_runway_lengths`
  (which deliberately give siblings on a crowded component face different
  forward reach, to avoid overlapping narrowed lanes) had their *raw*,
  un-narrowed footprints reserved globally regardless of that narrowing, so
  a longer-reaching sibling's wide raw footprint could seal a
  shorter-reaching neighbor in behind it -- a self-inflicted gap, not a real
  obstacle. Fix: `_equalize_dense_runway_reach` (new helper,
  `translation/route_rust.py`) raises every port in a dense group to the
  same forward reach as whichever sibling already reaches furthest, applied
  at both dense-runway-length call sites (target side, and both branches of
  the source side). The plan's own first implementation attempt (probing
  the static obstacle map for a fixed clearance margin) was a complete
  no-op, caught by validating a before/after value dump rather than trusting
  the change; a fresh investigation found the real mechanism (sibling
  reservations, not a margin) and the ExecPlan's Surprises & Discoveries
  section records the full trace as a worked example of catching a wrong
  diagnosis via measurement.
  `multiportmmi_8x8`'s entire 111-net routing stage now completes for the
  first time (previously always failed at `n_24`); `multiportmmi_16x16`'s
  `n_48` is independently confirmed fixed too. `pytest -q`:
  `23 failed, 314 passed, 1 skipped` -> `21 failed, 316 passed, 1 skipped`
  (only change: both `multiportmmi_8x8` tests in
  `tests/test_multiportmmi_benchmark.py` flip to passing).
  Two new, separate findings surfaced once routing got this far, both
  investigated enough to confirm neither was caused by this fix: (1) a
  crossing-legality rejection (`error=No legal LiDAR crossing route found`)
  on `multiportmmi_8x8` `n_32` and `multiportmmi_16x16` `n_102`, with the
  same *shape* as the earlier, separately-resolved `TOY` finding (a
  bare-cell corridor that exists but is too tight once any clearance is
  required) -- not a port-sizing problem, still open (see below); (2) a
  photonic-geometry verification `source_endpoint_mismatch` on
  `multiportmmi_8x8` nets `n_40`/`n_41`/`n_42` -- **now fixed**, see the
  next entry.
- `.agent/execplans/2026-08-18-crossing-aware-endpoint-correction-direction-sequence.md`
  is **complete** (commit `20aab29`). Root-caused and fixed the
  `source_endpoint_mismatch` finding above: the crossing-aware endpoint
  correction path (`translation/route_rust_endpoint_correction.py`) computes
  a fully correct port-exit fix by slicing the same rich Rust correction the
  non-crossed path uses, but a compatibility check,
  `_compatible_terminal_direction_sequence`, assumed the one newly-inserted
  correction segment always sits at a fixed position (index `0`) in the
  direction sequence; when the baseline's own port-adjacent segment already
  matched the port's facing direction (the common case), the real correction
  instead appended its segment at the *other* end, got rejected, and the
  code fell back twice more -- to a guard-window-limited solver with no bend
  available, then to the raw uncorrected route -- landing `2.0014um` off a
  `2.0um` tolerance. Fixed by checking both placements. `multiportmmi_8x8`
  now completes **fully clean** end to end for the first time: 111/111
  routed, zero crossing-verification issues, zero photonic-verification
  issues, GDS written (`build/verification/*.json` read directly to confirm,
  not inferred from console output). `pytest -q` unchanged (`21 failed, 316
  passed, 1 skipped`, identical names). Implemented directly (not via
  Codex) with the choice explicitly recorded in the ExecPlan's Decision Log,
  per `.agent/CLAUDE_CODEX_FLOW.md`'s "diagnosis/implementation boundary."
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
  (unchanged in count from Phase 2/Stage 5, but the failure *names* moved:
  the unify-port-access-region plan's Milestone 3 deleted one pre-existing
  failing test as genuinely dead code and added two newly-surfaced
  `multiportmmi_8x8` failures while flipping one unrelated heater test from
  failing to passing -- net zero on the count, see below for the full
  story). Every change was independently re-verified by Claude (not just
  trusted from Codex's self-report). The separate Rust unit-test baseline
  (`cargo test --lib`) is `312 passed, 9 failed`, all 9 failures
  pre-existing and crossing-related (see above; one more pass than the
  prior `311` because the same plan's Milestone 1 added a new Rust test).
- `.agent/execplans/2026-08-18-unify-port-access-region-computation.md` is
  **complete** (all three milestones done, commits `4055992`, `936958c`
  (merge decision only), `d20edcf`, `bf2ac9a`, `fc89187`). It grew out of
  chasing the TOY/`mmi_0,o1` clearance question further: digging into *why*
  widening `port_lane_half_width_cells` alone had zero effect on the real
  opened-cell count revealed that "how big is a port's access/keepout
  region" was computed by seven independently-parameterized, uncoordinated
  pieces of logic across `src/py_router.rs` and `translation/route_rust.py`,
  not one bug. There is now exactly one analytic sizing computation
  (`_resolve_port_footprint_cells` / Rust `build_port_footprint_cells`),
  shared by self-opening and foreign-keepout, differing only in one
  explicit must-stay-blocked subtraction; all the old scattered pieces are
  either migrated in or deleted as confirmed-dead code (verified by
  exhaustive grep, not assumed). Two Codex attempts at the merged
  self-opening+foreign-keepout migration regressed (the second one worse
  than the first) because both invented a new ad-hoc bolt-on mechanism
  instead of finding and reusing the pre-existing
  `_foreign_keepout_open_cells_for_spec`/`normal_port_runway_cells`
  exemption, which already composed correctly by construction; Claude then
  implemented it directly after reading that mechanism, and it worked
  cleanly. See `.agent/CLAUDE_CODEX_FLOW.md`'s new "What Codex is reliable
  at, and what it is not" section for the generalized lesson.
  The now-correct, wider clearance surfaced two genuine (not artifactual)
  routing constraints sharing one signature
  (`corridor_clearance_first_disconnected_radius=0`,
  `target_region_size=22`) at the same heater-pad-to-multiport port-spec
  pattern: `multiportmmi_8x8`'s `n_24` and `multiportmmi_16x16`'s `n_48`.
  Neither is a regression this plan introduced -- `multiportmmi_16x16` was
  independently confirmed (via an isolated `git worktree` at the
  pre-Milestone-2 commit) to already fail full-run at a different,
  unrelated net (`n_130`) before this plan touched anything, so it was
  never actually a clean full-run baseline. Both findings are open,
  tracked here, not yet investigated further -- see Next Engineering Step.
  The Stage 5 plan is complete for its own two originally-scoped milestones
  and is now background/prerequisite reading; its own "candidate
  Milestone 2" is superseded by (and answered by) the plan above.

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

Clean (`git status --short` empty). Known pre-existing, unrelated test/benchmark
failures, all already covered by the 23-item baseline failure list and none
caused by any of the completed plans:

- `tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested`
  fails because `RouteAttemptRecord.as_dict()` now includes `crossing_hotpath_*`
  fields the test's expected literal does not list.
- The default `TOY` CLI benchmark still fails at `gc1_to_mmi_in2` with `No
  route found` on this branch. Do not use `TOY` as a smoke test here. Root
  cause is now fully resolved as a question (see the unify-port-access-region
  plan's Outcomes & Retrospective): a genuine clearance shortage at the
  target port `mmi_0,o1`'s immediate approach, confirmed via the single,
  now-unified sizing computation rather than an intersection of several
  uncoordinated ones -- a benchmark-placement fact, not a router bug. Not
  planned to be fixed (would mean moving the benchmark's ports, not a code
  change).
- `multiportmmi_8x8` `n_24`/`n_32`'s port-sizing and `n_40`/`n_41`/`n_42`'s
  `source_endpoint_mismatch`, and `multiportmmi_16x16`'s `n_48`, are all now
  **fixed** (dense-port-runway-clearance-reach and
  crossing-aware-endpoint-correction-direction-sequence plans, see Current
  Snapshot). `multiportmmi_8x8` now completes **fully clean** end to end for
  the first time: 111/111 routed, zero crossing/photonic verification
  issues, GDS written, under this repository's default settings (repair/
  rip-up-reroute enabled). One finding remains open and deferred, and only
  manifests with `--ripup-reroute false` (repair papers over it in the
  default config, which is itself worth treating with some suspicion, not
  just relief): `multiportmmi_8x8` `n_32`'s (source `mmi0_multiport_0_0,o11`)
  very first search attempt hits `error=No legal LiDAR crossing route
  found`. Investigated further (2026-08-18): this is a *different* axis of
  the same dense-port-cluster problem the dense-port-runway-clearance-reach
  plan already fixed one axis of. `o11` is part of a second 6-port group on
  the same component (`o7`-`o12`); its forward *length* reach is already
  fixed by that plan's equalization (all six now reach the same far column).
  What's still narrow is *lateral width*: `_filter_dense_port_opening`
  splits the group's ~22 available rows unevenly across the 6 ports (`o7`/
  `o12` get 6 rows each, `o9`/`o11` get only 2), and 2 rows is likely too
  narrow to execute any bend at all (`bend_radius_cells=3`) regardless of
  forward reach -- confirmed via `corridor_clearance_source_region_size=1`
  at just 1 cell of inflation. Real device geometry is a red herring here:
  it borders the source only on the west, while `o11`'s own orientation
  (`0.0`, facing east) means the route does not need to go that way at all;
  every other direction is blocked by sibling reservations, not hardware.
  Unlike the length fix, there is no already-reserved, provably-safe ceiling
  to equalize width to -- giving `o11` more rows necessarily takes rows from
  a neighbor, risking recreating the exact overlap problem
  `_filter_dense_port_opening` exists to prevent, so this needs real design
  thought (how much lateral room a single-direction bend genuinely needs
  versus the current symmetric-half-width formula, and how to redistribute
  fairly) rather than a mechanical fix -- a good candidate to discuss with
  the user before starting, not "heavy work" to just do solo. Not yet
  independently checked whether `multiportmmi_16x16`'s `n_102` is the same
  width-axis shape or something else.

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

No active ExecPlan right now. `.agent/execplans/2026-08-18-unify-port-access-region-computation.md`,
`.agent/execplans/2026-08-18-stage5-routing-crossing-correctness-walkthrough.md`,
`.agent/execplans/2026-08-18-dense-port-runway-clearance-reach.md`, and
`.agent/execplans/2026-08-18-crossing-aware-endpoint-correction-direction-sequence.md`
are all complete (see Current Snapshot); pick the next one from the
candidates below with the user before starting.

Candidates, not in a mandated order:

1. Decide how to handle the dense-port *lateral width* allocation problem
   found investigating `multiportmmi_8x8` `n_32` (see Current
   Snapshot/Worktree State for the full diagnosis): `_filter_dense_port_opening`
   splits a dense group's available rows unevenly, and a port that lands on
   the narrow end (2 rows for `bend_radius_cells=3`) cannot execute any bend
   regardless of forward reach. This is a design question (how much lateral
   room a single-direction bend actually needs, how to redistribute fairly
   without recreating sibling overlap) worth discussing before implementing,
   not a mechanical fix. `multiportmmi_16x16`'s `n_102` not yet checked for
   the same shape. Not started.
2. The 9 pre-existing failing Rust crossing tests discovered while
   verifying the Stage 5 plan's diagnostics fix (`cargo test --lib`, see
   Current Snapshot) -- a separate, unexplored thread in the same
   crossing-legality code area. Not started.
3. Continue the broader Python-and-Rust correctness walkthrough into
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
   The (now complete) Stage 5 and unify-port-access-region plans already
   read parts of `src/py_router.rs` and `src/astar.rs`; their ExecPlans'
   Context and Orientation sections are useful orientation for this future
   readability pass.
4. Resume the crossing-verification-foundation objective (the pre-readability-
   work priority). The last recorded stable-benchmark checkpoint (commit
   `a29dc00`) marked `benes_8x8`, `benes_16x16`, `multiportmmi_8x8`, and
   `multiportmmi_16x16` full-run stable; this is now **confirmed stale**
   (2026-08-18, during the unify-port-access-region plan's Milestone 3):
   `benes_8x8` and `benes_16x16` are genuinely still stable (reconfirmed
   directly this session), but `multiportmmi_8x8` and `multiportmmi_16x16`
   both fail full-run at the heater-pad-to-multiport finding described in
   Worktree State -- `multiportmmi_16x16` was independently confirmed to
   already fail full-run (at an unrelated net, `n_130`) even at the
   pre-unify-plan baseline, so this staleness predates today's work.
   `multiportmmi_32x32` is not yet stable either, with route 156 / `n_155`
   the last known next slow/hanging route to investigate (not re-verified
   this session).
