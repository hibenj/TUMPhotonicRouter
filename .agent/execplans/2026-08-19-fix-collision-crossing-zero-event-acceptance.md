# Fix collision-crossing helper accepting zero-crossing-event routes

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

`cargo test --lib` had exactly one pre-existing, deliberately-tracked failing test: `py_router::tests::collision_crossing_route_without_event_is_not_accepted` (identified and left failing on purpose by `.agent/execplans/2026-08-18-recalibrate-stale-rust-crossing-tests.md`, which traced it to commit `9e0a927`). The test's own name and assertion state the contract plainly: `try_route_with_collision_crossings` -- the search helper whose entire purpose is "route this net while achieving a crossing with one of the given partner nets" -- must not accept a candidate route that never actually crossed any of its requested partners. It was doing exactly that: a route with zero crossing events was reported as a successful collision-crossing result.

This plan fixes that bug, confirms `cargo test --lib` goes to `322 passed; 0 failed`, and -- because fixing a real acceptance-criteria bug changes real routing behavior, not just test scaffolding -- also traces and resolves the knock-on effects on the Python-level `pytest` suite and this repository's benchmark validation ladder. One knock-on effect turned out to be a second, separate, pre-existing bug in native repair bookkeeping, newly exposed (not caused) by this fix; see Surprises & Discoveries and Decision Log for why that second bug is deliberately left unfixed and tracked as a named follow-up rather than fixed in the same pass.

## Progress

- [x] (2026-08-19) Root-caused and fixed the zero-crossing-event acceptance bug in `src/py_router.rs`. `cargo test --lib`: `321 passed; 1 failed` -> `322 passed; 0 failed`.
- [x] (2026-08-19) Traced and fixed two stale `pytest` assertions in `tests/test_multiportmmi_benchmark.py` that this fix legitimately changes (attempt/failure/repair counters, not correctness). Full `pytest -q` confirmed back at the exact pre-existing `21 failed, 322 passed, 1 skipped` baseline (empty diff of sorted `FAILED` lines against the pre-fix baseline).
- [x] (2026-08-19) Ran the benchmark validation ladder (`benes_4x4`, `multiportmmi_8x8` under both bare CLI defaults and its documented stable-baseline config) and discovered a real regression under bare defaults: `multiportmmi_8x8` now fails with `missing_route_record n_68`. Root-caused via extensive direct instrumentation (all temporary, removed before this commit) to a second, separate, pre-existing bug -- see Surprises & Discoveries. Confirmed the stable-baseline config, `benes_4x4`, and `cargo test --lib`/`pytest -q` are all unaffected by this second bug.
- [ ] Decide and execute on the second bug (fix now vs. track as a separate, named follow-up) -- see Decision Log; this is an explicit, not-yet-made-by-the-repository-owner choice as of this entry.

## Surprises & Discoveries

- Observation: the actual bug was narrower than "the whole partner-constraint check is broken." `crossing_events_satisfy_partner_constraints` (`src/py_router.rs`, around line 3965) has a `partner_ids.is_empty() => true` early return that is *correct* and needed elsewhere (a repair-probe call site around line 9316 legitimately means "no partners were required for this probe, so it's trivially compliant"). The bug was one specific caller, `try_route_with_collision_crossings_using_primitives` (around line 4165): when `crossing_cfg.allow_only_expected_pairs` is false, it derives its `required_partner_ids` argument to that shared helper from the crossing events *actually found* (`crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events)`), not from the originally-requested `partner_ids`. Zero events yields an empty derived set, which then vacuously satisfies the shared helper's empty-set case -- a call site derived exactly the wrong witness set from the very computation it was supposed to be constraining, not a bug in the shared helper itself. Fix: add an explicit `!crossing_events.is_empty()` condition to this call site's own `satisfies` computation, which does not touch the shared helper or the legitimate empty-partner-ids caller at all.
- Observation: validating the fix against `multiportmmi_8x8` surfaced a second, independent, real bug via the same "measure, don't trust the fix" discipline this repository's other recent plans have used (see `.agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md`'s own three-turn investigation for the precedent this generalizes). Full trace, reproduced via extensive temporary instrumentation (all removed before committing; see Decision Log for why the instrumentation approach was used instead of a smaller, permanent diagnostic): `multiportmmi_8x8` under bare CLI defaults (`enable_crossings=True` is the bare default -- `SCRIPT_ENABLE_CROSSINGS = True` in `routing_flow_config.py`) fails with `missing_route_record n_68` (the schematic net named `n_68`, whose *internal* `net_id` is `69` -- `translation/route_rust.py`'s job-building loop at line ~6509-6531 assigns `net_id` as a 1-indexed sequential counter over `nets.items()`, which is *not* the same number as the net name's numeric suffix; net name `n_68`'s internal `net_id` being `69`, one more than its name's suffix, is the normal, always-true relationship for this benchmark, not a symptom of anything wrong -- a fact that cost significant investigation time to establish, since several rounds of ad hoc probing conflated "internal net_id" with "net name's numeric suffix" before this was pinned down explicitly).
  Root cause of the missing record itself: `restore_saved_source_layer_routes` (`src/py_router.rs`, around line 3000) can fail *partway through* restoring a layer's saved routes -- it iterates the saved `(index, route)` pairs in order and calls `commit_native_route_with_clearance` for each, returning `Err` immediately on the first failure (`return Err(format!("failed to restore source-layer route for net {}", job.net_id))`, around line 3033), which silently abandons every remaining saved route in that iteration -- they are never restored, and no caller detects or retries this specific loss. In the reproduced case: a "source layer center-out" repair attempt (`try_route_source_layer_center_out_native`) for a layer of 8 nets sharing one source x-coordinate failed on its own first re-route attempt (a "No legal LiDAR crossing route found" error for one layer member), triggering a restore of all 8 members' original routes; that restore succeeded for 4 of the 8 (in net_id order) and then failed restoring the 5th, silently abandoning the remaining 4 (including the internal net_id whose name is `n_68`). A separate, unrelated repair mechanism (`pending_straight_repair_attempt`, triggered immediately afterward for a different net that happened to name one of the same 4 abandoned nets as its own "victim") coincidentally repaired 2 of those 4 abandoned nets as a side effect, leaving the other 2 permanently lost. One of those 2 (the one whose *own* job index in the outer processing loop had already been passed) never gets revisited and is missing from the final output; the other (whose job index hadn't been reached yet) is picked up naturally by the outer loop's normal "not yet routed" path and silently self-heals, which is why only one net (not several) shows up as missing in the final error.
  This is a genuine, pre-existing bug in native repair bookkeeping (a silent, unretried partial-failure path), not something this plan's own fix introduces -- it was already reachable before this fix, just apparently never reached in a way that mattered for any of this repository's tracked benchmarks. This fix increases how often a net's *first* collision-crossing attempt legitimately fails (since a formerly-vacuous "success" is now correctly rejected), which increases how often the "pending straight" / "source layer center-out" repair machinery gets exercised at all, which is how this pre-existing bug became newly visible. Confirmed via `git stash` A/B testing that `multiportmmi_8x8` bare-defaults was clean before this fix and fails with exactly this signature after it, and confirmed via three repeated clean (non-instrumented) runs that the failure is fully deterministic (always net `n_68`, not flaky) -- two probe-script runs during the investigation that appeared to show *different* nets missing turned out to be comparing two *separately executed, non-identical* program runs against each other, not evidence of nondeterminism in a single execution; this cost real investigation time before being ruled out and is recorded here as a reminder to control for "different process invocations can each be individually deterministic yet differ from each other" before concluding a system is flaky.
  Confirmed *not* affected: `multiportmmi_8x8`'s documented stable-baseline config (`STABLE_ROUTING_ENV`/`STABLE_ROUTING_FLAGS`), `benes_4x4`, `cargo test --lib`, and the full `pytest -q` suite -- all still clean/at-baseline after this fix, checked directly via `build/verification/*.json` per `.agent/WORKFLOW.md`'s Routing Verification Gate.

## Decision Log

- Decision: fix the zero-crossing-event acceptance bug directly (not via Codex), given it was already fully diagnosed by a prior plan (`.agent/execplans/2026-08-18-recalibrate-stale-rust-crossing-tests.md`) down to the exact function and exact commit that introduced it.
  Rationale: per `.agent/CLAUDE_CODEX_FLOW.md`'s "diagnosis/implementation boundary," a bounded, well-specified, already-diagnosed fix in a single function is exactly the shape that can be implemented directly with the choice explicitly recorded, rather than requiring a dispatch decision; the fix itself is a four-line, single-condition change plus a documenting comment.
  Date/Author: 2026-08-19, Claude, recorded here.
- Decision: use temporary, throwaway `eprintln!`/`print()` instrumentation across several files to trace the second bug (the `missing_route_record n_68` finding), rather than a smaller permanent diagnostic feature, and remove all of it before committing.
  Rationale: the investigation needed to distinguish between many candidate hypotheses (record-object identity, dict key corruption, non-determinism, an off-by-one in job/net-id construction, a partial-restore bug) that were not known in advance, so the instrumentation was necessarily broad and exploratory rather than a single well-scoped probe; per this repository's own established pattern (`.agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md`'s "temporary tracing, removed before this commit"), this kind of diagnostic code is disposable, not a durable feature, and is not preserved once the root cause is confirmed. All instrumentation was verified removed (`git diff --stat` showing only the real fix and the legitimate test-value updates) before this plan's commits.
  Date/Author: 2026-08-19, Claude, recorded here.
- Decision: do **not** fix the second bug (`restore_saved_source_layer_routes`'s silent partial-restore failure) as part of this plan; leave it as a tracked, named follow-up instead.
  Rationale: this is a materially different, larger piece of work than the plan's own scope (a native repair-bookkeeping correctness bug in a ~40-line restore loop plus its caller's error-handling, requiring a decision about *how* to make a partial restore failure safe -- retry the abandoned members individually, escalate to a hard failure instead of silently continuing, or something else -- each with different risk/complexity tradeoffs) and was only found as a downstream consequence of validating this plan's actual, narrower fix. Per this repository's own repeatedly-applied practice this session of surfacing a newly-found issue and asking rather than silently expanding scope (e.g. the endpoint-correction restructuring plan's Milestone 0.5 sequencing decision, itself escalated to the repository owner twice), this is recorded as an explicit open decision for the repository owner rather than assumed. Not yet confirmed with the repository owner as of this entry.
  Date/Author: 2026-08-19, Claude, recorded here; **awaiting repository owner input**.

## Outcomes & Retrospective

(To be filled in once the second-bug decision above is made and, if applicable, executed.)

## Context and Orientation

`try_route_with_collision_crossings` (`src/py_router.rs`) is the search entry point used when the router's "collision crossing" / LiDAR-pure crossing mode is active: given a net and a set of candidate partner nets it is allowed to cross, it searches for a route that both reaches the target and achieves at least one legal crossing with one of those partners. It delegates through `try_route_with_collision_crossings_with_loss` to `try_route_with_collision_crossings_using_primitives`, the function that actually runs the search and validates the result.

That validation, before this fix, computed `required_partner_ids` two different ways depending on `crossing_cfg.allow_only_expected_pairs`: if true, it used the original, caller-requested `partner_ids` (correct); if false (the common case, matching this repository's `--crossing-mode lidar-pure` default), it used `crossed_partner_ids` -- the set of partner ids *derived from whatever crossing events the search actually produced*. A route with zero crossing events produces an empty `crossed_partner_ids`, and `crossing_events_satisfy_partner_constraints` treats an empty partner-ids argument as "nothing to satisfy, trivially true" -- a case that is genuinely correct for at least one *other* caller (a repair-probe path that legitimately has no required partners) but wrong here, since this function's whole reason for existing is to guarantee at least one crossing was achieved.

## Plan of Work

Single milestone: apply the fix, verify `cargo test --lib`, then work outward through the validation ladder (`pytest -q`, `benes_4x4`, `multiportmmi_8x8` under both configs) fixing or explaining every difference from baseline until the ladder is clean or every remaining difference is a confirmed, understood, and explicitly decided-upon finding (not a silent gap). This plan's own Progress section tracks each step; no further milestone breakdown is needed for a fix this size.

## Concrete Steps

All commands run from the repository root, `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`, with the required toolchain override for any Rust build:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib 2>&1 | tail -10
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release
    PYTHONPATH=. .venv/bin/pytest -q

`benes_4x4` end to end (small, fast, safe with `--debug-svgs`):

    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4 --debug-svgs

`multiportmmi_8x8` under bare CLI defaults (no debug flags; **do not** run `--debug-svgs` with no value on this benchmark per this repository's standing safety rule):

    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8

As of this plan, this bare-defaults run is **expected to fail** with `missing_route_record n_68` -- this is the tracked, understood, not-yet-fixed second bug, not a regression to chase further without a repository-owner decision first.

`multiportmmi_8x8` under its documented stable-baseline configuration (unaffected by the second bug):

    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

Read verdicts directly from `build/verification/*.json` per `.agent/WORKFLOW.md`'s Routing Verification Gate, not from console output alone.

## Validation and Acceptance

**Verdict: PASS for this plan's own stated scope** (fix the zero-crossing-event acceptance bug; leave the second, newly-found bug as an explicit, tracked follow-up rather than silently unaddressed or silently expanded into).

Evidence: `cargo test --lib` `322 passed; 0 failed` (was `321 passed; 1 failed`); `pytest -q` `21 failed, 322 passed, 1 skipped` (identical failure-name set to the pre-fix baseline, confirmed via sorted-diff); `benes_4x4` end to end clean (`build/verification/benes_4x4_*.json` read directly, both crossing and photonic verification `status=complete, success=true, error_count=0`); `multiportmmi_8x8` stable-baseline config clean (`build/verification/multiportmmi_8x8_*.json` read directly, both `status=complete, success=true, error_count=0, warning_count=0`); `multiportmmi_8x8` bare-defaults confirmed to fail with exactly the one understood, tracked, root-caused finding (`missing_route_record n_68`), not some other unexplained difference.

This plan does **not** claim the second bug is fixed -- that is explicitly out of scope pending the repository owner's decision, per the Decision Log above.

## Idempotence and Recovery

The Rust fix is a pure logic change inside one function; `cargo check --lib`/`cargo test --lib` surface any mistake immediately. The two `pytest` assertion updates are read-only value changes reflecting already-confirmed-correct new behavior, safe to re-derive from a fresh benchmark run if ever in doubt. No Python source outside the test file was changed. Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or any comparably large benchmark at any point.

## Artifacts and Notes

The full instrumentation trail used to diagnose the second bug (temporary `eprintln!`/`print()` statements across `src/py_router.rs`, `translation/route_rust.py`, and `routing_flow_verification.py`) was entirely removed before committing; this ExecPlan's Surprises & Discoveries section is the durable record of what was found, not the instrumentation itself.

## Interfaces and Dependencies

No public interface changes. The fix is entirely inside `try_route_with_collision_crossings_using_primitives`'s own `satisfies` computation in `src/py_router.rs`. If the second bug is fixed in a future pass, expect it to touch `restore_saved_source_layer_routes` and/or its caller's error-handling in the same file.
