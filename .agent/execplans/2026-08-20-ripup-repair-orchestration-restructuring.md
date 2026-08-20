# Ripup/repair orchestration restructuring

## Purpose / Big Picture

With the Future Architecture Initiative closed (all 4 recommended stages -- obstacle map/grid snapping, A* single-net search, geometry realization, path-length matching -- now have explicit `Protocol`/`trait` interfaces, see `.agent/REPOSITORY_STATE.md`), the clearest remaining candidate is the one piece that initiative deliberately excluded at every single stage: ripup/repair orchestration, `route_many_with_repair_and_commit` in `src/py_router.rs`. It was excluded because it was judged "the least separable code in the entire pipeline" -- a single ~3,000-line method on `PyPhotonicRouter`, the struct holding this repository's entire mutable session state as plain fields, with three real bugs found in or adjacent to it during this session alone. This plan characterizes its actual current shape (not just repeats the prior summary) before deciding what restructuring, if any, to do -- the same characterize-first-then-present-options discipline used for every stage of the just-closed initiative.

Two open benchmark findings (`.agent/REPOSITORY_STATE.md` Current Findings items 1 and 2) are parked pending this restructuring giving the surrounding code a clearer structure; this plan does not commit to fixing either, only to characterizing and then deciding how much restructuring work to do, if any, right now.

## Progress

- [x] Milestone 0 (full characterization) done, see Surprises & Discoveries. Dispatched to a fork for the read-only investigation (explicitly instructed to characterize only, not decide -- given this repository's own on-the-record lesson about a fork fabricating a decision on a similar prior characterization task, see `.agent/execplans/2026-08-20-extract-geometry-realization-plm-boundary.md`'s Decision Log). Summary: the function is larger than expected (~3,065 lines, bigger than `route_nets_rust` was pre-refactor) but a *different kind* of entangled -- not closure-heavy (7 closures total, 0 `move`), already delegating to ~40 named methods on `self`, but every one of those methods implicitly shares `PyPhotonicRouter`'s 20-field session state through `&mut self` rather than taking/returning plain data. One specific piece -- the victim-set-expansion logic behind the open `n_67`/`n_70`/`n_71` finding -- has literally no function boundary at all (inline in the loop), which is the sharpest, most concrete piece of evidence for why this needs a restructuring pass before any interface work. 3 options proposed below for Milestone 1.
- [x] Milestone 1 (present options) done (2026-08-20). Repository owner chose, via `AskUserQuestion`: fix the stale `tests/test_rust_batch_repair.py` test first and stop there, rather than either restructuring option.
- [x] Milestone 2 (fix the stale test) done (2026-08-20, Claude, small localized edit). See Outcomes & Retrospective for the diagnosis and fix. Full-suite re-run: `21 failed, 325 passed, 1 skipped` -> `20 failed, 326 passed, 1 skipped`, exactly one fewer failure, nothing else changed.
- [x] Milestone 3 (design) done (2026-08-20). Repository owner chose to proceed with extracting the victim-set-expansion logic. Re-reading the target code found it cleaner to extract than expected -- zero `self` coupling, a pure free function.
- [x] Milestone 4 (implement, Codex) done (2026-08-20). `compute_repair_victim_sets` extracted (`src/py_router.rs:1669`), byte-identical logic, 4 new tests, full validation ladder (cargo test, batch-repair integration test, `benes_4x4`, `multiportmmi_8x8`) all clean/unchanged. See Outcomes & Retrospective. This plan is complete per the repository owner's chosen scope -- the underlying `n_67`/`n_70`/`n_71` bug itself remains unfixed, future work.

## Surprises & Discoveries

- **`route_many_with_repair_and_commit` (`src/py_router.rs:8670-11734`, ~3,065 lines) is structured as one giant labeled loop over jobs (`'route_jobs: for (job_index, job) in native_jobs.iter().enumerate()` at `:8752`) using loop-label control flow (`continue`/`break 'route_jobs'`, 10 jump points scattered through the body) as a de facto per-net state machine, rather than named sub-functions per attempt strategy.** This is a genuinely different shape than `route_nets_rust`'s pre-refactor entanglement (~120 anonymous closures capturing shared scope): only 7 non-`move` closures exist in the whole body, 0 `move` closures. Instead, the function calls out to roughly 40 distinct named methods on `self` (`route_single_net_and_commit_native`, `route_single_net_and_commit_repair_native`, `crossing_local_ripup_candidates`, `rollback_committed_route`, `restore_saved_source_layer_routes`, and others). The entanglement is not "hidden anonymous state capture," it is "every named method implicitly shares the same god-object session state via `self`, with no enforced boundary between concerns, only convention."
- **The distinct repair strategies are interleaved across the full 3,065-line range, not grouped into contiguous phases.** By call site: first-attempt/probe routing (`route_single_net_and_commit_native` at `:8835,8874,8980,9120,9248,9300`; `route_single_net_ignore_dynamic_native` at `:9491,11511`), collision-crossing-guided routing (`try_route_through_collision_partner_set` at `:9738,10585,10809`; `try_route_with_collision_crossings_with_loss` at `:10685`), repair/rip-up (`route_single_net_and_commit_repair_native` at `:8899`; `crossing_local_ripup_candidates`; `crossing_reservation_blockers`), and restore-on-failure (`restore_saved_source_layer_routes`, a separate ~45-line helper at `src/py_router.rs:3067`, called from `:9090` with inline retry post-processing at `:9098-9130`). One piece, targeted illegal-crossing repair queueing, already delegates to a genuinely clean, plain-data free function outside any `impl` block, `enqueue_targeted_illegal_crossing_repair_set` (`src/py_router.rs:1669`), called 6 times (`:10315,10437,10946,11063,11203,11314`) -- an existing, working precedent for what "extract a repair sub-mechanism cleanly" looks like in this codebase.
- **The victim-set-expansion logic behind the open `n_67`/`n_70`/`n_71` finding (Current Findings item 1) has no function boundary at all -- it is inline in the per-net loop at `src/py_router.rs:10038-10074`, confirmed unmoved and still exactly matching `.agent/REPOSITORY_STATE.md`'s citation.** This is the single sharpest piece of evidence in this characterization: unlike the other two bugs found this session (see below), which lived in already-separate helper functions, this one was never given a boundary to test through at all -- it cannot be isolated or unit-tested without first extracting it.
- **`PyPhotonicRouter`'s full session-state field list (`src/py_router.rs:720-740`, 20 fields)**: `grid`, `primitive_cfg`, `astar_cfg`, `astar_cfg_cached`, `obstacle_map`, `primitives`, `crossing_context`, `committed_center_routes`, `committed_realized_center_routes`, `committed_target_terminal_bump_guards`, `committed_opened_cell_keys`, `crossing_events`, `use_collision_crossing_routing`, `static_cells`, `port_open_cells`, `registered_plm`, `last_meander_registration_profile`, `last_pending_straight_victim`, `long_straight_congestion_cells`, `long_straight_congestion_records`. Direct-touch counts inside `route_many_with_repair_and_commit`'s own body (not counting indirect touches through its ~40 called helper methods, which were not individually audited): `obstacle_map` 48, `committed_center_routes` 15, `committed_realized_center_routes` 12, `committed_target_terminal_bump_guards` 12, `committed_opened_cell_keys` 12, `crossing_events` 9, `crossing_context` 6, `use_collision_crossing_routing` 6, `long_straight_congestion_records` 2, `primitives` 2, `astar_cfg`/`long_straight_congestion_cells` 1 each. Notably, at `:10053-10059` the function clones 6 of these fields wholesale as a manual checkpoint/rollback mechanism before each repair round (`round_base_map`, `round_base_center_routes`, etc.) -- a hand-rolled snapshot/restore embedded directly in the orchestrator, not a separable abstraction.
- **The "zero stage-granular test coverage" claim from the prior (shallower) characterization is not fully accurate -- it holds for the orchestrator function itself, but not for everything inside it.** `route_many_with_repair_and_commit` itself has zero direct `#[test]`s. But `enqueue_targeted_illegal_crossing_repair_set` (the one already-extracted free function) has a direct, isolated unit test (`targeted_illegal_crossing_repair_promotes_learned_blocker`, `src/py_router.rs:14679`) using plain data, no `PyPhotonicRouter` instance at all -- real proof this kind of extraction is testable in this codebase. Several other tests construct a fresh, empty `PyPhotonicRouter` and call one specific repair-keepout/learning method directly without running actual routing (`dynamic_commit_error_creates_bounded_repair_keepout` `:14705`, `victim_repair_error_keepout_routes_current_owned_dynamic_overlap_to_victim_only` `:14810`, `local_repair_error_keepout_learns_dynamic_overlap_for_capped_retry` `:14746`, `targeted_illegal_crossing_repair_promotes_blocker_when_queue_capped` `:14883`, `learned_keepout_retry_is_capped_per_ripup_set` `:14917`, `realized_crossing_violations_create_targeted_repair_keepouts` `:15538`) -- narrower than a free-function test but real, focused coverage of individual repair sub-mechanisms.
- **The one existing direct integration-level test for the orchestrator itself, `tests/test_rust_batch_repair.py::test_rust_batch_repair_rips_and_reroutes_dynamic_blocker` (the only test in a 128-line file), is currently broken -- a stale-signature problem, not evidence the function is untestable.** It builds a small, focused 2-lane/2-job scenario (not a full benchmark) and calls `route_many_with_repair_and_commit` directly through PyO3. It fails today with `ValueError: expected tuple of length 8, but got tuple of length 7` -- the job-tuple signature has grown a field since this test was last updated. Already part of this session's known 21-failure pytest baseline (`tests/test_rust_batch_repair.py::test_rust_batch_repair_rips_and_reroutes_dynamic_blocker`). Its existence and design are themselves evidence that a focused, non-full-benchmark repair test *is* possible here -- it just needs recalibrating.
- **Of the three real bugs found in or adjacent to this function during the initiative, two lived in already-separate helper functions, and only one was genuinely inline with no boundary -- a more nuanced picture than "this whole function is equally dangerous everywhere."** Zero-event-acceptance bug: lived in `try_route_with_collision_crossings` (`src/py_router.rs:4159`), a separate helper called via `:10685`; its regression test is `collision_crossing_route_without_event_is_not_accepted` (`:15334`). Silent partial-restore data-loss bug: lived in `restore_saved_source_layer_routes` (`src/py_router.rs:3067`), also separate, called at `:9090`; the fix is inline at the orchestrator's own call site (`:9098-9130`, with a comment citing `.agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md` at `:9109`). Victim-set expansion (`n_67`/`n_70`/`n_71`): the one genuinely inline, boundary-less case, at `:10038-10074` (see above).
- **Current Findings item 2 (dense-port lateral-width allocation) does not actually live in this function at all.** It lives in `_filter_dense_port_opening`, `translation/route_rust.py:2418` (called at `:2460`) -- Python-side, pre-routing setup, unrelated to `route_many_with_repair_and_commit`/Rust-side repair code. It is parked in `REPOSITORY_STATE.md` alongside item 1 only because both await the same future restructuring initiative broadly, not because they share code. Any restructuring plan here should not expect to touch or fix item 2 as a side effect.
- **No existing session-class decomposition exists on the Rust side comparable to `route_nets_rust`'s Python-side `_RouteNetsRustSession` refactor.** `route_many_with_repair_and_commit` is still a single `impl PyPhotonicRouter` method reaching directly into `&mut self`; the structs declared near it (`NativeRouteJob`, `NativeRouteAttempt`, `NativeRepairTraceEvent`, `CrossingEvent`, etc., `:744-1007`) are plain input/output data structs, not stateful orchestration objects -- they do not reduce the god-object coupling. Repair is purely Rust-side: Python calls the whole batch method as one opaque unit from 3 separate call sites in `translation/route_rust.py` (`:4146,5791,6175`), none of which contain their own repair-loop logic -- any future interface work touching this function's call surface needs to account for all 3 callers.
- **Scope caveat, stated explicitly by the investigating fork**: not every one of the ~40 helper methods' own bodies was read in full (would be its own multi-hour pass) -- the "0 direct touches, likely via helpers, not verified" entries in the field-touch table above reflect that gap. Treat the touch counts as a lower bound on real coupling, not a complete audit.

## Decision Log

- Decision: pursue the "fix the stale test first" option -- recalibrate `tests/test_rust_batch_repair.py`'s one existing focused integration test so there's a working, non-full-benchmark regression harness before touching anything structural, then stop and reassess -- rather than immediately extracting the victim-set-expansion logic or attempting a full session-state restructuring.
  Rationale: repository owner's explicit choice, via `AskUserQuestion`, after being shown all three options with tradeoffs, given this is real routing-behavior code with genuine regression risk (unlike the past 4 stages' purely-additive interface work) (2026-08-20).
  Date/Author: 2026-08-20, repository owner.

## Outcomes & Retrospective

Milestone 2 complete, 2026-08-20. Diagnosed the exact cause of `tests/test_rust_batch_repair.py::test_rust_batch_repair_rips_and_reroutes_dynamic_blocker`'s failure directly (not delegated -- a small, fully-specified, already-diagnosed fix, per `.agent/CLAUDE_CODEX_FLOW.md`'s "small localized edits" exception): `route_many_with_repair_and_commit`'s job-tuple signature (`src/py_router.rs:8673-8682`) is an 8-tuple with a `static_cleanup_cells: Vec<(i32, i32)>` field (6th position, between `clearance_exempt_cells` and `source_port_um`) that the test's two job tuples were missing -- they had 7 elements, not 8, matching the exact `ValueError: expected tuple of length 8, but got tuple of length 7` failure signature. Fixed by adding an empty list (`[]`, meaning "no static cleanup cells for this scenario") as the 6th element of both job tuples. The single test now passes with every one of its detailed assertions intact (repair trace events, victim reroute, route cell geometry) -- this was a stale-signature problem, not evidence the underlying repair behavior had actually broken. Full-suite re-run confirms exactly one fewer failure and nothing else changed: `21 failed, 325 passed, 1 skipped` -> `20 failed, 326 passed, 1 skipped`.

This restores a working, focused (not full-benchmark), direct-PyO3-call regression harness for `route_many_with_repair_and_commit` -- the orchestrator itself had zero working test coverage of its own before this fix. Per the repository owner's decision, this plan stops here rather than proceeding to either of the larger restructuring options; extracting the victim-set-expansion logic or a full session-state decomposition remain available as future work, now with this test as a safety net for either.

## Decision Log (continued)

- Decision: proceed with the "extract victim-set-expansion logic" option (2026-08-20, repository owner, after a status/recommendation exchange -- not a formal `AskUserQuestion`, but an explicit "yes" to a stated recommendation naming this specific option over the full-restructuring alternative and over stopping here).
  Rationale: this is the most direct path to eventually fixing Current Findings item 1 (the `n_67`/`n_70`/`n_71` cluster on `multiportmmi_8x8`, one of the three active-phase benchmarks), with much smaller scope/risk than the full session-state restructuring.

## Milestone 3: extract victim-set-expansion logic (design)

Re-read the target code directly against current source rather than trusting Milestone 0's line-range citation verbatim (line numbers had shifted slightly, from small drift, not any change to this logic itself): the victim-set-expansion block is `src/py_router.rs:10063-10099` inside `route_many_with_repair_and_commit`, building `repair_victim_sets: Vec<(u32, Vec<u64>)>`.

**Finding that makes this extraction cleaner than expected**: this specific block touches zero `self` state -- it reads only local variables (`crossing_repair_enabled: bool`, `probe_realized_crossing_violations: Vec<InvalidCrossingIntersection>`, `candidate_blockers: Vec<u64>`, `max_victims: usize`, `max_rounds: u32`, all already computed earlier in the function) and builds a local `Vec`. It can be extracted as a **pure free function taking no `&self`/`&mut self` at all** -- an even cleaner shape than `enqueue_targeted_illegal_crossing_repair_set` (`src/py_router.rs:1669`, the existing precedent, which does need `&self` for some lookups).

Design:

```rust
fn compute_repair_victim_sets(
    crossing_repair_enabled: bool,
    probe_realized_crossing_violations: &[InvalidCrossingIntersection],
    candidate_blockers: &[u64],
    max_victims: usize,
    max_rounds: u32,
) -> Vec<(u32, Vec<u64>)> {
    // exact existing body of src/py_router.rs:10063-10099, unchanged logic,
    // adapted to slice parameters instead of reading self/outer-scope locals.
}
```

Call site (`src/py_router.rs:10063-10099`) replaced with:

```rust
let repair_victim_sets = compute_repair_victim_sets(
    crossing_repair_enabled,
    &probe_realized_crossing_violations,
    &candidate_blockers,
    max_victims,
    max_rounds,
);
```

This is a **pure extraction, zero behavior change** -- not a fix for Current Findings item 1 (the missing "fold a victim's own secondary blocker into the ripup set" reasoning stays exactly as absent as it is today). It only gives that reasoning gap a function boundary and, for the first time, direct unit test coverage -- setting up a future fix, not making one.

4 hand-computed test scenarios (exact expected outputs, not guessed) recorded in the Codex task file for Milestone 4.

## Milestone 4: implement (Codex, done)

First dispatch correctly stopped with zero edits: Codex found `repair_victim_sets` is mutated again later in `route_many_with_repair_and_commit`, via `&mut repair_victim_sets` passed to `enqueue_targeted_illegal_crossing_repair_set`/`enqueue_learned_keepout_repair_retry` during the repair-attempt loop -- the task's original instruction to declare the call site as plain `let` (not `mut`) was wrong, and Codex followed its own "stop and report rather than silently keep `mut` or change downstream code" instruction correctly rather than guessing. This is exactly the "Codex reliably stops and reports when a task file's assumption is wrong" behavior `.agent/REPOSITORY_STATE.md` already had on record -- a second confirming instance, not a new lesson.

Re-dispatched with the one-line correction (`let mut repair_victim_sets = compute_repair_victim_sets(...)`, everything else unchanged) plus an explicit note that the later mutation is a separate, out-of-scope concern. Result: `compute_repair_victim_sets` (`src/py_router.rs:1669`, right next to `enqueue_targeted_illegal_crossing_repair_set`, matching that precedent's placement), a byte-identical-logic extraction confirmed by diff read (the moved block is character-for-character the original, only the enclosing `fn`/parameter list and de-indentation are new), 4 new tests all matching their hand-computed expected outputs exactly, placed next to `enqueue_targeted_illegal_crossing_repair_set`'s own test. Codex also caught and reverted its own accidental `cargo fmt` spillover onto unrelated files before finishing, keeping the diff scoped to `src/py_router.rs` only.

**Reviewed by Claude**: full diff read (144 lines, `src/py_router.rs` only), confirmed the extracted block's logic is unchanged and the 4 tests' expected values match. Validation independently re-run, not just taken from Codex's report: `cargo test --lib` `387 passed, 0 failed` (383 baseline + 4 new), `tests/test_rust_batch_repair.py` still passes end-to-end (confirms behavior-preservation through a real routing scenario, not just independent compilation). Given this touches production ripup/repair code -- the area this whole plan exists because of its history of real bugs -- also ran the benchmark validation ladder: `benes_4x4` **Verdict: PASS** (`error_count=0`, both crossing and photonic verification); `multiportmmi_8x8` bare CLI defaults fails with the exact same pre-existing `RuntimeError: No route found for n_70` signature, byte-identical to every prior run this session -- confirmed not a regression, and confirming the underlying bug is (as designed) still present, since this was a pure extraction, not a fix. This plan's scope (per the repository owner's Milestone 3 decision) ends here: the victim-set-expansion logic now has a function boundary and direct test coverage for the first time; actually fixing the `n_67`/`n_70`/`n_71` bug (folding a victim's own secondary blocker into the ripup set) remains future work, now easier to attempt safely given this test coverage.

## Context and Orientation

**Ripup/repair orchestration** is the routing-pipeline layer that handles what happens when a net can't be routed cleanly on the first attempt: it decides which already-routed nets ("victims") to rip up and retry, attempts repair strategies (dynamic-blocker rip-up, collision-crossing-guided rerouting, targeted illegal-crossing repair), and rolls back cleanly on failure. It sits inside native (Rust) net routing, called once per routing pass from the Python orchestration layer.

Key files and locations, all confirmed by direct read in Milestone 0:

- `src/py_router.rs:8670-11734` -- `route_many_with_repair_and_commit`, the ~3,065-line orchestrator itself.
- `src/py_router.rs:720-740` -- `PyPhotonicRouter`'s 20-field session-state struct definition.
- `src/py_router.rs:10038-10074` -- the inline, boundary-less victim-set-expansion logic (root cause of Current Findings item 1).
- `src/py_router.rs:1669` -- `enqueue_targeted_illegal_crossing_repair_set`, the one existing clean, tested, plain-data free function extracted from this area -- the closest thing to a working precedent for what extraction should look like here.
- `src/py_router.rs:3067` -- `restore_saved_source_layer_routes`, a separate helper (site of the partial-restore bug fix).
- `src/py_router.rs:4159` -- `try_route_with_collision_crossings`, a separate helper (site of the zero-event-acceptance bug fix).
- `src/py_router.rs:14580-17696` -- `mod tests`, containing the existing focused repair-sub-mechanism unit tests listed in Surprises & Discoveries.
- `tests/test_rust_batch_repair.py` -- the one existing (currently broken, stale-signature) focused integration test for the orchestrator itself.
- `translation/route_rust.py:4146,5791,6175` -- the 3 Python call sites, each an opaque single call to the batch method.

## Plan of Work

### Milestone 0: full characterization (done, see Surprises & Discoveries)

### Milestone 1: present scope/approach options to the repository owner (done)

This is a larger, higher-risk area than any of the 4 stages just closed -- explicitly flagged throughout this session's own history as the least separable, most bug-prone code in the pipeline. Presented via `AskUserQuestion`; repository owner chose to fix the stale test first and stop. See Decision Log.

### Milestone 2: recalibrate the stale test (done)

`tests/test_rust_batch_repair.py::test_rust_batch_repair_rips_and_reroutes_dynamic_blocker` failed with `ValueError: expected tuple of length 8, but got tuple of length 7`. Diagnosed directly against current source (`route_many_with_repair_and_commit`'s job-tuple parameter type, `src/py_router.rs:8673-8682`): an 8th field, `static_cleanup_cells: Vec<(i32, i32)>`, was added at position 6 (between `clearance_exempt_cells` and `source_port_um`) since the test was last updated. Fixed by adding `[]` at that position in both job tuples in the test. See Outcomes & Retrospective for full validation.

## Concrete Steps

Milestone 0 was read-only source investigation -- no build or benchmark commands. See the geometry-realization plan's Toolchain notes for this machine's Rust toolchain override and the `build/verification/*.json` verdict-reading pattern once an implementation milestone exists. Given this area's history (three real bugs found nearby this session), any implementation milestone here should expect a QA/Harness verification pass per `.agent/WORKFLOW.md`'s Routing Verification Gate, not self-certification, regardless of which option is chosen.

## Validation and Acceptance

Milestone 0 is complete when Surprises & Discoveries has a direct-evidence-backed characterization of the orchestrator's real shape, session-state coupling, test coverage, and the three bugs' actual locations (all done above). Milestone 1 is complete when the repository owner has made an explicit, recorded decision. Later milestones' acceptance criteria depend on that decision.

## Idempotence and Recovery

Milestone 0 was entirely read-only and is safe to redo or resume freely; it made no code changes.

## Artifacts and Notes

Originating context: `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (original characterization that first flagged this code as "the least separable code in the entire pipeline, and the real reason 'the A* search' ... is not actually one coherent stage today") and `.agent/REPOSITORY_STATE.md`'s Next Engineering Step (pointed here after the Future Architecture Initiative's 4th stage closed).

## Interfaces and Dependencies

To be determined once Milestone 1's decision is known, per `.agent/PLANS.md`'s own guidance.
