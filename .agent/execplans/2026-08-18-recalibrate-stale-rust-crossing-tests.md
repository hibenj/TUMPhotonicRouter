# Recalibrate 9 pre-existing failing Rust crossing unit tests

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository's Rust unit test suite (`RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib`) had 9 pre-existing failing tests, all in the crossing-legality area (`src/astar.rs` and `src/py_router.rs`), present before any work in this session's active branch began and unrelated to it. This plan diagnoses why each one fails and, for the 8 that turn out to be stale (their hand-constructed scenarios or expected values were written before a later, legitimate, already-committed behavior change and never updated), recalibrates them so they exercise the *current*, intentional behavior instead of an outdated one. The ninth test is different: it caught what looks like a real, currently-unfixed bug in a Rust helper function, and this plan deliberately leaves it failing rather than forcing it to pass, since fixing production logic was out of this plan's scope.

After this plan is complete, a person can run `cargo test --lib` (see Concrete Steps) and see `320 passed; 1 failed`, with the one remaining failure being `py_router::tests::collision_crossing_route_without_event_is_not_accepted` -- a tracked, real finding (see Surprises & Discoveries), not an accident.

## Progress

- [x] (2026-08-18) Diagnosed and recalibrated 8 of 9 tests via a Codex Implementation Engineer task (`.agent/codex_runs/.../diagnose-rust-crossing-tests/`), each traced to a specific, real, already-committed commit that legitimately changed crossing-legality behavior. Independently verified: read every line of the resulting diff, re-derived the margin arithmetic by hand for two of the `py_router.rs` tests, confirmed all four cited commit hashes are real commits on this branch with topically consistent messages, and re-ran `cargo test --lib` myself, getting the identical `320 passed; 1 failed` result with the identical remaining test name. Plan complete.

## Surprises & Discoveries

- Observation: my own preliminary hypothesis, formed before dispatching the Codex task, was wrong. I suspected all 9 failures traced to a single commit, `5bf56d1` ("Fix terminal crossing guard for partner targets"), based on it adding a new, stricter boundary check (`terminal_bump_guard_satisfied`) to the exact subsystem these tests exercise. The actual investigation found the real causes are three *different* commits -- `35e30fe` ("routing: stabilize lidar crossing runout state"), `f9f9ce0` ("routing: checkpoint layer-one crossing invariants"), and `7dd8278` ("routing: count bend arms for crossing runout") -- none of which is `5bf56d1`. This is recorded as a reminder that a plausible-sounding lead from reading one diff is a hypothesis to hand off for verification, not a conclusion to build a fix around without checking; the task file explicitly asked for this lead to be verified rather than assumed, and it was correctly found to be an incomplete explanation.
- Observation: one test, `py_router::tests::collision_crossing_route_without_event_is_not_accepted`, was deliberately not touched and is left failing. It calls `try_route_with_collision_crossings` with a candidate route (`(0,0)` to `(12,0)`) and a committed partner route (`(20,20)` to `(30,20)`) that do not geometrically intersect at all -- two disjoint horizontal segments at different heights. The test's own name and assertion message state the expected contract plainly: "collision-crossing helper must not accept routes with zero crossing events." Tracing the actual call showed it returns `Some(...)` (accepting the route) with `events=[]`, `accepted=0`, `candidates=0`, and an internal `satisfies` flag of `true` -- i.e., a route that never engaged with the requested crossing partner at all is being reported as a successful *collision-crossing* result, which is a contradiction: this function's whole purpose is to route while crossing a specific partner, and a result with zero crossing events is not that. The commit `9e0a927` ("routing: hard fail realized crossing mismatches") is the identified cause: it changed the relevant helper to return success based on `satisfies && realized_violations.is_empty()`, without also requiring a non-empty set of accepted crossing events, so "nothing was even attempted" and "a crossing was legitimately achieved with zero violations" both satisfy the same condition. This was independently re-derived and re-confirmed (not just accepted from the Codex report): the test's own assertion, the geometric disjointness of the two routes in the test, and the commit's own stated purpose are all consistent with this being a real, unfixed logic gap, not a stale expectation.

## Decision Log

- Decision: dispatch this task to Codex via `.agent/scripts/codex_task.sh`, rather than diagnose and fix all 9 directly.
  Rationale: per `.agent/CLAUDE_CODEX_FLOW.md`'s "The diagnosis/implementation boundary," this is exactly the shape of task that belongs with Codex once a starting lead exists: bounded (two named files, tests-only), requiring careful but mechanical per-test archaeology (find the commit that changed behavior, verify it against this repository's documented crossing-legality invariants, derive corrected expected values), closely mirroring an earlier, successful test-recalibration task in this same session (`.agent/execplans/2026-08-18-unify-port-access-region-computation.md`'s Milestone 2 test-recalibration work). The task file explicitly required Codex to distinguish "stale test" from "possible real bug" and to leave any suspected real bug untouched rather than force it to pass -- and it did, on the one test where the evidence actually pointed that way.
  Date/Author: 2026-08-18, Claude.
- Decision: do not fix `collision_crossing_route_without_event_is_not_accepted`'s underlying production-code issue as part of this plan.
  Rationale: this plan's scope, stated in the Codex task file, was diagnosis and test-only recalibration; the file-scope restriction (only `src/astar.rs` and `src/py_router.rs` test blocks) was deliberate so a genuine production bug, if found, would be surfaced rather than silently patched by a task not designed to reason carefully about non-test code changes in a correctness-sensitive crossing-legality path. This finding is recorded here and should be evaluated as its own, separately-scoped fix.
  Date/Author: 2026-08-18, Claude.

## Outcomes & Retrospective

Complete for this plan's scope. 8 of 9 pre-existing failing Rust unit tests were confirmed stale (each traced to a specific, real, already-committed behavior change, not a regression) and recalibrated to exercise the current, correct behavior. `cargo test --lib` went from `312 passed; 9 failed` to `320 passed; 1 failed`. The ninth test surfaced a real, credible, currently-unfixed bug in `try_route_with_collision_crossings` (or the helper it calls) -- a route with zero crossing events against its requested partner is incorrectly treated as a successful collision-crossing result -- and was deliberately left failing and documented rather than forced to pass, per this repository's `.agent/roles/reviewer.md` principle of treating a found regression as a blocking finding to report, not paper over. See `.agent/REPOSITORY_STATE.md`'s "Next Engineering Step" for this as a tracked follow-up candidate.

## Context and Orientation

This repository (`TUMPhotonicRouter`) has an extensive Rust unit test suite for its A* crossing-legality logic, in `#[cfg(test)] mod tests` blocks inside `src/astar.rs` (which implements the grid-based A* search, including crossing-move legality checks like `crossing_move_outcome`) and `src/py_router.rs` (which implements the PyO3-exposed router object, including realized-route crossing validation and the LiDAR-style "collision crossing" search variant, `try_route_with_collision_crossings`). Both files' crossing logic has been actively developed over many commits (visible via `git log --oneline -- src/astar.rs`), and it is normal, expected repository maintenance for hand-constructed unit test fixtures -- specific coordinates, margins, and expected internal statistics counters chosen to exercise one exact code path -- to become stale when the logic they test is intentionally refined, if the person making that change does not happen to also touch every test that happens to exercise the changed code path. This plan is exactly that kind of catch-up maintenance for 9 such tests, discovered as a "pre-existing, unexplored" item in `.agent/REPOSITORY_STATE.md` during an earlier, unrelated investigation this session (the Stage 5 routing-crossing-correctness-walkthrough plan's diagnostics work).

## Plan of Work

Dispatch a single Codex Implementation Engineer task (task file preserved at `/tmp/.../scratchpad/diagnose_rust_crossing_tests_task.md` during this session; not committed to the repository, per this repository's convention that Codex task files are scratch inputs, not durable artifacts -- the durable record is this ExecPlan) with: the exact list of 9 failing test names; a documented, explicitly-labeled-as-unverified lead (the `5bf56d1` hypothesis, later found incomplete); a requirement to independently verify or refute that lead per test, not assume it; a requirement to check any candidate causal commit's diff directly before trusting a description of it; a requirement to judge whether new behavior is the *correct* direction (checked against this repository's documented crossing-legality contract in `.agent/WORKFLOW.md`) rather than merely "whatever the code currently does"; a strict file-scope limit to test-only edits in the two named files; and an explicit instruction to leave any test untouched and reported separately if its failure looks like a real bug rather than a stale expectation.

## Concrete Steps

All commands run from the repository root, `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`.

Full Rust unit test suite, before and after:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib 2>&1 | tail -30

Before this plan: `312 passed; 9 failed`. After: `320 passed; 1 failed` (the one remaining failure, `py_router::tests::collision_crossing_route_without_event_is_not_accepted`, is expected and intentional -- see Surprises & Discoveries).

No Python code was touched by this plan; `PYTHONPATH=. .venv/bin/pytest -q` is unaffected and was not expected to change (confirmed: still `21 failed, 316 passed, 1 skipped`, identical to the baseline before this plan).

## Validation and Acceptance

Verdict: PASS, for this plan's stated scope (recalibrate the 8 genuinely stale tests; do not touch the 9th). Evidence: `cargo test --lib` re-run directly by Claude (not just accepted from the Codex report), producing the identical `320 passed; 1 failed` result with the identical remaining failure name; the full diff (`git diff src/astar.rs src/py_router.rs`) read in full, confirming every change is scoped to test bodies/fixtures inside `mod tests`, no non-test code touched; the margin arithmetic for the two `py_router.rs` margin-formula tests re-derived by hand against the `7dd8278` commit's real diff and the test's own committed-route coordinates, both checking out exactly; all four cited commit hashes (`5bf56d1`, `35e30fe`, `f9f9ce0`, `7dd8278`, `9e0a927`) confirmed to be real commits on this branch with topically consistent commit messages. This plan does not claim `collision_crossing_route_without_event_is_not_accepted` is fixed -- it is intentionally not fixed, and that is part of what "PASS for this plan's scope" means here.

## Idempotence and Recovery

`cargo test --lib` is read-only with respect to source; safe to re-run at any time. This plan's change is entirely inside `#[cfg(test)]` blocks in two Rust files, so a mistake here can only break test compilation or test assertions, never runtime router behavior -- `cargo check --lib` or `cargo test --lib` will surface any problem immediately, and no Python code, benchmark, or `build/` artifact is affected.

## Artifacts and Notes

Full before/after test counts and the one remaining failure's panic message are reproduced in Surprises & Discoveries and Outcomes & Retrospective above; see the actual diff (`git diff src/astar.rs src/py_router.rs` as committed alongside this plan) for the exact per-test changes.

## Interfaces and Dependencies

No production interface changes. Every edit is inside `#[cfg(test)] mod tests` in `src/astar.rs` and `src/py_router.rs`: adjusted hand-constructed primitive/geometry fixtures, added missing `commit_route_with_clearance_and_allowed_core_overlaps` calls so partner geometry the test relies on is actually discoverable via the obstacle map (matching the "owner-first" dynamic contact discovery model these tests are meant to exercise), and updated expected `RouteSearchStats` field names/values and margin/coordinate constants to match already-correct, already-shipped behavior.
