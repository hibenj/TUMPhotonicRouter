# Loss-driven endgame at 64 inputs: why our router aborts where LiDAR finishes

This ExecPlan is a living document (`.agent/PLANS.md`). Status: **analysis and proposal, no code changed, no runs started**. Owner decision needed on the approach (Decision Log).

## Purpose / Big Picture

At 64 inputs the original LiDAR routes `multiportmmi_64x64` clean in 89 min (alone, 4 h limit, 4 of 10 rip-up iterations, 224 crossings). Our loss-driven baseline (`lidar-pure`) aborts after 59 min with `No route found`, and contribution 1 (`lidar-guided`) aborts the same way on `benes_64x64` after 64 min (it finishes the mesh in 3 h 25 min with 275 crossings). The owner's requirement (2026-09-14): our router must handle these circuits too. This plan records the failure mechanism as read from the code and the logs, the candidate fixes, and a recommendation.

## Evidence (2026-09-14 overnight runs, `.agent/execplans/2026-09-10-64x64-scaling-benchmarks.md` Milestone 3)

| run | outcome | error |
|---|---|---|
| lidar-pure, multiportmmi_64x64 | failed after 3539 s, 0 search failures before | `source-layer repair could not restore or reroute net(s) [309, 310, 311] after a failed center-out attempt for net 313` (raised for `n_308`) |
| lidar-guided, benes_64x64 | failed after 3860 s, 0 search failures before | `source-layer repair could not restore or reroute net(s) [454] after a failed center-out attempt for net 462` (raised for `n_s6_2_o1_to_s7_1_i1`) |
| lidar-pure, benes_64x64 | timeout 4 h at route 671 of 768 | (outer layer, no repair reached) |
| lidar-guided, multiportmmi_64x64 | clean, 12 315 s, 1100 att / 63 fail / 42 repairs, 275 crossings | |

Both aborts come from the same place: `try_source_layer_center_out_repair` (`src/py_router.rs` ~11402-11605).

## Mechanism (read from the code)

1. A net whose search fails with a `pending_straight_victim_hint` on an already-routed net of the same source-x column triggers the coarsest repair: **rip up the whole column** (`rollback_source_layer_routes_for_retry`, at least `SOURCE_LAYER_CENTER_OUT_MIN_JOBS = 8` nets) and **reroute it center-out** (`try_route_source_layer_center_out_native`). Each column is tried **once per batch** (`batch.retried_source_layers`).
2. The center-out pass stops at the **first** net that fails (`return Err(...)` inside the loop); nets already rerouted in that pass stay committed until the restore.
3. Restore (`restore_saved_source_layer_routes`): roll back everything of the column again, then recommit every saved route with `commit_native_route_with_clearance`. The restore is best-effort: a saved route that no longer commits is reported as unrestored.
4. Each unrestored net gets **one plain reroute** (`route_single_net_and_commit_native`, no repair chain, no history cost). If that fails, `batch.failed_net_id` is set and the **whole batch aborts** (`return Err(())`), which the flow turns into `No route found` (`translation/route_rust.py` `_dispatch_native_routing`).
5. There is no outer iteration: no second pass over the batch, no global rip-up round. The negotiated engine of `2026-08-25-negotiated-repair-engine.md` exists as an opt-in entry point but lacks the crossing-aware strategies of the default chain (its Milestone 6 finding), so it is not a drop-in endgame.

Contrast with LiDAR: up to 10 global rip-up-and-reroute iterations with accumulated history cost; it needed 4 on the 64x64 mesh. The difference at 64 inputs is not search power (our A* dispatched every net with 0 search failures) but what happens after a coarse repair fails.

## Open questions (need a traced reproduction)

- Why does a saved route fail to recommit at all? It was legal when first committed. Hypotheses: (H1) legality is not symmetric against nets committed after it (crossing angle / straight-margin rules and clearance halos are checked from the committing net's side), so once other nets crossed or hugged it, its own recommit is refused; (H2) the aborted center-out pass leaves state the rollback does not undo (static cleanup, opened cells, long-straight congestion), so the restore sees a different map. Either way step 3 turns "repair failed" into "net lost".
- Repro cost: the failure is deterministic but 59-64 min into the run. `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` prints the native repair trace; `--debug-dir` writes the failed net's route diagnostics. One traced run per case gives the exact restore refusal reason. No shorter reproduction is known (multiportmmi_32x32 triggers 5 repairs and survives).

## Options

- **A. Never lose a net inside a repair strategy (bounded, mechanical).** (a) The center-out pass continues past a failing net instead of returning at the first failure, collecting the failures; (b) unrestored/unrouted nets are not aborted on but **deferred**: appended to the batch's own job order and routed at the end through the normal per-net path, i.e. with the full repair chain (pending-straight, preemptive, crossing subset, keepout retry) instead of one plain reroute; (c) the batch aborts only when a deferred net fails at the end of the batch. Behaviour on every run that never reached step 4 is unchanged by construction, so the ladder stays bit-identical (to be verified). Cost: a few hundred lines in `py_router.rs`, kernel unit test with a synthetic column whose center-out pass fails on one net and whose batch still completes, then the two 1 h reproductions. Does not guarantee mm64 routes; it removes the abort and gives the endgame the same tools as the main pass.
- **B. A real endgame: global negotiated rip-up rounds for the lidar modes.** After the first pass, iterate: rip up every failed net's blockers with history cost, reroute, repeat up to N rounds (LiDAR's model). The opt-in engine is the starting point but must gain crossing-aware displacement (Milestone 6 gap). Weeks, not days; the likely real answer if A is not enough.
- **C. Tuning only** (order, congestion weights, `SOURCE_LAYER_CENTER_OUT_MIN_JOBS`): moves the failure between nets (Milestone 7 experience on multiportmmi_16x16); not recommended as the fix.

## Recommendation

A first, then measure. It is the smallest change that addresses the observed abort, is testable in the kernel without a 1 h run, and cannot change any benchmark that did not abort. Run the two traced reproductions in parallel with implementing A (they answer H1/H2 and tell whether A's deferred pass has a chance). Decide on B with that evidence.

## Decision Log

- 2026-09-14 (owner): "we need to get our router to also handle this" -- the requirement. Approach: **A first** (owner, 2026-09-14 ~09:10). Slice 1 = deferral only: the center-out repair never aborts the batch; still-missing nets go to the end of the batch through the normal chain. The center-out pass still stops at its first failing net (continuing past failures is a follow-up, not in slice 1). Traced reproductions (mm64 lidar-pure, benes64 lidar-guided, `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, 4 h limit) start as soon as slice 1 passes its regression checks.

## Progress

- [x] 2026-09-14: mechanism traced in the code; evidence table; options written.
- [x] Slice 1 landed 2026-09-14 09:20 (implementation lane; verified by the lead): `RepairBatchState.deferred_job_indices`/`deferred_count`; work-queue loop in `route_many_with_repair_and_commit`; `try_source_layer_center_out_repair` defers instead of aborting; `deferred_count` in the batch result and the `route search:` line; Rust unit test for the queue helper, Python test for the summary field; regression: benes_8x8 and multiportmmi_8x8 lidar-pure bit-identical.

- Slice 1 validation: `cargo test --lib` 447/447; `pytest tests/test_route_rust_records.py tests/test_route_rust_obstacle_config.py` 14 passed; benes_8x8 lidar-pure 52/4/0, 16 crossings, 0 errors and multiportmmi_8x8 lidar-pure 111/0/0, 33 crossings, 0 errors -- bit-identical, `deferred=0` on both. Files: `src/py_router.rs`, `translation/route_rust.py`, `translation/route_rust_types.py`, `routing_flow_reporting.py`, `tests/test_route_rust_records.py`.
- [x] Reproductions started 2026-09-14 09:22 and **stopped by the owner at 10:44 (1 h 31 min), no verdict** (`overnight64/sliceA.sh`): lidar-pure multiportmmi_64x64 and lidar-guided benes_64x64 in parallel, `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, `--verbose-routes`, 4 h limit; LiDAR multiportmmi_32x32 running alongside. Expected: the former abort point (~59 / ~64 min) now defers the nets; verdict depends on the deferred pass.

## Trace findings (2026-09-14, slice 1 reproductions with `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`)

- **multiportmmi_64x64, lidar-pure** (~58 min): net 313 (`n_312`, mmi0_multiport_0_7,o14 -> ps_array_1 heater_35: a permutation net that must cross 13 same-column fan-in nets 299-312, all "not_perpendicular" in the probe) gets a pending-straight hint against **net 231, a net of the previous band** (mmi0_ps -> mmi0_multiport, count 112 852). Center-out repair of column x=2425 (10 jobs) fails on its first reroute (net 311); restore cannot recommit 309, 310, 311; plain reroutes fail; slice 1 defers [309, 310, 311] instead of aborting and net 313 continues its own chain. Verdict pending (deferred pass at the end of the batch).
- **benes_64x64, lidar-guided** (~64 min): net 462 hint against net 224 (count 1788); center-out of column x=4864 (64 jobs) reroutes 11 nets (481..476, 486) and fails at 486 (hint against 274); restore cannot recommit 454; deferred [454]; net 462 continues (probe: 461 "insufficient_straight_margin"/"not_perpendicular"). Verdict pending.
- **Layer-only testbed** (`overnight64/testbed/mm64_layer5.sh`, ids 256-319 hoisted in the stable order; the 64 net names/positions verified identical to the full run, and job 309's endpoints (2425, 2769) -> (3130, 2673) equal the full run's `n_308`): behaves like the full run inside the band (probes at 258/260 with the same "not_perpendicular" reasons; net 313 fails its plain search and enters the chain with 22 candidate blockers) but cannot reproduce the trigger itself, because the hint victim 231 belongs to the previous band, which the testbed does not route. First testbed run was cut by its 30 min cap inside net 313's repair. A faithful testbed needs bands 4+5 (ids 192-319) or at least net 231's band committed first.

- 2026-09-14 10:44 (owner): both slice 1 reproductions stopped. Both had passed the old abort points (deferral fired at ~58 min on the mesh, ~64 min on the Benes). Afterwards the mesh run spent 20+ min inside net 313's own repair chain (two pending-straight repairs against previous-band nets 231 and 281, probe still listing 10 non-perpendicular crossings with column nets 299-312) and had passed LiDAR's 89 min total without leaving that net; the Benes run resolved seven more nets of the x=4864 column (462..510, one local repair each, 7-8 s per victim diagnosis) and then went quiet for 13 min. Conclusion recorded: slice 1 removes the abort but the chain's per-failure enumeration is the bottleneck at this size; superseded by `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md` (option B).
