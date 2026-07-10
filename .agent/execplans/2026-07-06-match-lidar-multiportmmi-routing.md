# Paused context: make multiportmmi_8x8 route with router-discovered crossings

Status note added 2026-07-10: this ExecPlan is a task-specific historical
handoff, not the active repository-level project plan. The current active
project goal is in `.agent/PROJECT_GOAL.md`, and the active next-step plan is
`.agent/execplans/2026-07-10-crossing-verification-foundation.md`.

This plan uses the phrase "LiDAR-like" in older entries. Interpret that phrase
narrowly: LiDAR is useful guidance for the idea that crossings are explored by
the router during search. TUMPhotonicRouter is not trying to reimplement LiDAR's
architecture, queueing, repair loop, or performance profile. The goal is a much
faster Rust-backed router with verified final geometry.

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds.

This document follows `.agent/PLANS.md` in this repository. If the plan is revised, keep it self-contained: a future agent or human should be able to continue from only this file and the current working tree.

## Purpose / Big Picture

The current task is to make the `multiportmmi_8x8` benchmark route in a LiDAR-like way in this repository. "LiDAR-like" means that this router should use a workflow comparable to the reference router in `/home/benjamin/Documents/Repositories/working/LiDAR`: normal A* search, legal crossing handling during collision checks, route failure detection, ripup and reroute, congestion or history updates, and eventual routing through this benchmark without invalid route geometry. For now, exact physical port alignment in the final GDS is not the acceptance criterion; the important outcome is that A* finds plausible routed paths with crossings, the repair loop behaves similarly to LiDAR, and accepted route geometry is not self-overlapping or otherwise invalid.

The way to see this working is to run `routing_flow.py multiportmmi_8x8` in this repository with debug tracing, compare its route queue and repair events against the LiDAR reference run from `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/scripts/lidar/run_route.py`, and inspect the generated GDS/SVG/JSONL artifacts. The final target is that the benchmark runs through using this router's LiDAR-style baseline without invalid geometries, and that any remaining differences from LiDAR are visible in comparable trace records rather than inferred from screenshots alone.

## Progress

- [x] (2026-07-06 15:21Z) Read repository instructions in `AGENTS.md` and the ExecPlan policy in `.agent/PLANS.md`.
- [x] (2026-07-06 15:21Z) Added a `multiportmmi_8x8` benchmark scaffold and data under `benchmarks/multiportmmi_8x8.py` and `benchmarks/data/`.
- [x] (2026-07-06 15:21Z) Implemented and tested substantial LiDAR-style crossing and routing infrastructure in the current working tree. The changed files include `src/astar.rs`, `src/py_router.rs`, `src/obstacle_map.rs`, `translation/route_rust.py`, `translation/route_rust_realization.py`, `translation/route_rust_records.py`, `routing_flow.py`, and several tests.
- [x] (2026-07-06 15:21Z) Added collision-driven crossing behavior in A* and fast crossing legality checks, including crossing footprint checks, required straight-window checks, and final realized crossing validation updates.
- [x] (2026-07-06 15:21Z) Added LiDAR-inspired global repair approximation in Rust: strict routing, relaxed/static-only probe routing, candidate blocker detection, ripup queues, history updates, group-aware congestion exemption, and diagnostic trace output.
- [x] (2026-07-06 15:21Z) Changed relaxed conflict handling so candidate-blocker relaxed routes are treated as probe-only progress, not committed geometry. This is intended to avoid repeatedly accepting invalid relaxed-conflict routes as if they were final routes.
- [x] (2026-07-06 15:21Z) Added debug trace cleanup for `build/routes/<prefix>_attempt_paths.jsonl` and `build/routes/<prefix>_native_probe_debug.jsonl` so per-run JSONL artifacts are not appended across runs.
- [x] (2026-07-06 15:21Z) Verified the current implementation with `cargo fmt`, `cargo check`, `cargo test --lib` with 288 passing tests, and Python tests `tests/test_rust_backend_import.py`, `tests/test_rust_batch_repair.py`, `tests/test_multiportmmi_benchmark.py`, and `tests/test_realized_crossing_verification.py` with 20 passing tests after the major routing changes.
- [x] (2026-07-06 15:21Z) Generated a bounded diagnostic partial for `multiportmmi_8x8` through route 51 after LiDAR global relaxed round 1. The stable artifacts are `build/routed_multiportmmi_8x8_stop51_round1_probe_only.gds`, `build/routes/multiportmmi_8x8_native_probe_debug_round1_probe_only.jsonl`, and `build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl`.
- [x] (2026-07-06 15:21Z) Identified a real accepted-route defect: `route[34] n_33`, attempt 71, round 1, bucket `normal_route`, has a self-intersecting compressed waypoint path. The segment `(750,131) -> (728,153)` crosses the later segment `(722,147) -> (739,147)`.
- [x] (2026-07-06 15:29Z) Fresh handoff audit verified that the branch is still `baseline/lidar-pure-crossings` at `fdc20e4`, the cited stop-51 artifacts exist, the `n_33` self-intersection evidence is present, `cargo check` passes, `cargo test --lib` passes with 288 tests, and the listed Python tests pass with 20 tests.
- [x] (2026-07-06 15:29Z) Corrected this plan after audit: the LiDAR runner is `src/picroute/scripts/lidar/run_route.py`, not repository-root `run_route.py`, and the self-intersection guard must run before obstacle-map commits as well as before `final_routes.insert`.
- [x] (2026-07-06 15:38Z) Performed a skeptical handoff review of the current implementation direction. `cargo test` passes with 300 Rust tests, and `PYTHONPATH=. .venv/bin/pytest -q` passes with 256 Python tests and 1 skipped test. This proves the current tree is not trivially broken, but it does not prove the `multiportmmi_8x8` routing behavior is correct.
- [x] (2026-07-06 15:38Z) Identified two plan-level risks that must guide the next agent: the current artifacts are only a bounded debug partial, and the selected `attempt_paths` JSONL does not cover every accepted route because it was generated with `--debug-svgs 32-37,48-51`.
- [x] (2026-07-06 15:38Z) Identified one implementation-level risk that needs an explicit decision before more tuning: the new LiDAR global relaxed repair loop is enabled whenever `self.crossing_context.is_enabled()` in `src/py_router.rs`, so it can affect `crossing_mode="window"` and `crossing_mode="collision"`, not only `crossing_mode="lidar-pure"`.
- [x] (2026-07-06 15:47Z) Verified the current working tree and named stop-51 artifacts before editing. The branch is still `baseline/lidar-pure-crossings`, the three `_round1_probe_only` artifacts exist, no `routing_flow.py multiportmmi_8x8` process is running, and the `n_33` accepted self-intersection is still present in the saved attempt-path JSONL.
- [x] (2026-07-06 15:55Z) Added an accepted-route sanity gate in Rust. `src/astar.rs` now exposes `validate_route_has_no_self_intersection`, which rejects non-adjacent compressed-waypoint segment intersections, zero-length segments, and repeated non-adjacent expanded grid cells. `src/py_router.rs` calls the guard before normal, simple, crossing, repair, static-only probe commit, relaxed-conflict commit, and ignore-dynamic probe acceptance paths can mutate route state or feed probe analysis.
- [x] (2026-07-06 15:55Z) Added focused Rust tests for the exact `n_33` waypoint sequence, adjacent shared endpoints, a non-intersecting detour, and repeated non-adjacent cells. The focused command `cargo test route_sanity --lib` passes with 4 tests.
- [x] (2026-07-06 15:55Z) Scoped LiDAR global relaxed repair to the existing explicit flag. `src/py_router.rs` now enters the global relaxed repair loop only when `use_lidar_global_relaxed_repair_only` is true and the disabling environment variable is absent; the Python bridge already sets that flag only for `crossing_mode == "lidar-pure"`. Added `lidar_global_relaxed_repair_requires_explicit_lidar_flag` as regression coverage.
- [x] (2026-07-06 15:59Z) Rebuilt the Python extension with `.venv/bin/maturin develop --release` after the Rust changes. The targeted Python regression command `PYTHONPATH=. .venv/bin/pytest -q tests/test_rust_backend_import.py tests/test_rust_batch_repair.py tests/test_multiportmmi_benchmark.py tests/test_realized_crossing_verification.py` passes with 20 tests.
- [x] (2026-07-06 16:05Z) Reran the bounded stop-51 diagnostic after the extension rebuild and preserved new stable artifacts with a `self_guard` suffix. The run returned a debug partial with 44/51 routes after LiDAR global relaxed round 1 before ripup, `attempts=75`, `failures=14`, `repairs=1`, and elapsed time about 130 seconds.
- [x] (2026-07-06 16:05Z) Added `PHOTONIC_ROUTER_TRACE_ALL_ATTEMPT_PATHS=1` so `attempt_paths.jsonl` can record every successful attempt independently of selected SVG export. Reran stop-51 with the switch enabled and saved `build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard_all_attempts.jsonl`, which contains 61 successful route/probe records covering route indices 1 through 51. The non-adjacent segment-intersection scan reports `records_with_non_adjacent_intersections=0`.
- [x] (2026-07-06 16:10Z) Extracted a LiDAR reference trace from `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/scripts/lidar/run_route.py`. The copied artifact is `build/lidar_reference/lidar_multiportmmi8x8_trace.jsonl` with 1,764 JSONL records from a run that completed in about 74 seconds and whose log reports DRV 0 for the full benchmark.
- [x] (2026-07-06 16:10Z) Compared this repository's stop-51 trace to the LiDAR trace at queue/repair level for `n_31` through `n_36` and `n_47` through `n_50`. The actionable difference is crossing legality, not initial queue order: both engines start the first dense group with `n_31` then `n_32`, but LiDAR accepts legal strict crossings for `n_33` through `n_36` while this router defers comparable routes as realized/invalid crossing conflicts against earlier routes.
- [x] (2026-07-06 16:39Z) Implemented path-local crossing metadata preservation. `src/astar.rs` now records accepted crossing candidates on crossing A* nodes and returns the selected path's `RouteCrossing` events in `RouteResult`; `src/py_router.rs` converts those preserved candidates into `CrossingEvent`s only if the crossing point still lies on both reconstructed polylines with sufficient straight margin. Focused tests for route sanity, preserved crossing conversion, and LiDAR repair gating pass.
- [ ] Rebuild the Python extension and rerun the focused `--debug-stop-after-route 34` crossing trace to prove `n_33` no longer reports `crossing_accepted > 0` with `events=[]`.
- [ ] Rerun stop-51 with `PHOTONIC_ROUTER_TRACE_ALL_ATTEMPT_PATHS=1` and verify route indices 32-37 / nets `n_31` through `n_36` are accepted committed routes, not probe-only or deferred conflicts.
- [ ] Run the benchmark beyond route 51, then eventually run full `multiportmmi_8x8` and require no accepted invalid geometry. Do not run beyond route 51 until the crossing-legality mismatch above is addressed or explicitly deferred, because the current stop-51 partial already shows the wrong repair behavior.

## Surprises & Discoveries

- Observation: The current stop-51 GDS intentionally lacks several paths near `n_32` because it is a debug partial captured before the next ripup, not a final routed result.
  Evidence: The routing command reported `Native route batch returned debug partial: 45/51 routes` with reason `debug partial after LiDAR global relaxed round 1 before ripup`.

- Observation: The bad self-overlap seen in the current GDS is not only a rejected probe artifact. One accepted route is self-intersecting.
  Evidence: Parsing `build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl` found `attempt=71`, `bucket=normal_route`, `round=1`, `route_index=34`, `net=n_33`, with the self-intersection between waypoint segments `(750,131) -> (728,153)` and `(722,147) -> (739,147)`.

- Observation: A* can produce self-intersecting physical paths even when no exact grid cell repeats.
  Evidence: The same `n_33` path has `repeated_cells=0` and `repeated_xy_states=0`, but non-adjacent compressed waypoint segments intersect. This happens because the A* state is `(x, y, angle)`, while realized waveguide validity depends on the whole centerline, not only the current state.

- Observation: The attempt-path JSONL initially appeared inconsistent because it had stale lines from older runs.
  Evidence: Before cleanup, `build/routes/multiportmmi_8x8_attempt_paths.jsonl` had 144 lines while the run summary reported 74 attempts. After clearing JSONL files at run start, the clean current artifact has 20 successful path records and the native probe debug artifact has 13 deferred/failure records.

- Observation: LiDAR has a debug trace module already available in the reference checkout.
  Evidence: `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/routing/debug_trace.py` reads `LIDAR_ROUTE_DEBUG_FILE`, `LIDAR_ROUTE_DEBUG_NETS`, `LIDAR_ROUTE_DEBUG_BBOX`, and `LIDAR_ROUTE_DEBUG_CHECKSPACING_LIMIT`, and emits JSONL records such as `check_spacing`, `check_spacing_summary`, `bitmap_window`, `history_update`, and ripup events.

- Observation: The LiDAR checkout does not have `/home/benjamin/Documents/Repositories/working/LiDAR/run_route.py` at repository root.
  Evidence: `find /home/benjamin/Documents/Repositories/working/LiDAR -name 'run_route.py' -print` returned `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/scripts/lidar/run_route.py`.

- Observation: Validating only before `final_routes.insert(...)` is too late for normal routes, because normal routing commits to the dynamic obstacle map inside helper functions before `route_native_batch` records the route in `final_routes`.
  Evidence: In `src/py_router.rs`, `route_single_net_and_commit_native` commits with `commit_route_with_clearance_and_allowed_core_overlap_cells` or `commit_route_with_clearance_overlap` before returning `Ok(result)`, and callers later insert the returned route into `final_routes`.

- Observation: The current selected `attempt_paths` artifact is not a complete accepted-route proof.
  Evidence: `translation/route_rust.py` writes `attempt_paths.jsonl` only for selected debug route indices when `debug_route_indices` is not `None`. The recorded stop-51 command used `--debug-svgs 32-37,48-51`, and `build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl` contains 20 route records whose route indices are only 32 through 51, while the run summary reported 74 attempts.

- Observation: The current implementation can change behavior outside `lidar-pure` mode.
  Evidence: In `src/py_router.rs`, `route_many_with_repair_and_commit` sets `use_lidar_global_relaxed_repair = self.crossing_context.is_enabled() && env var not disabled`. The Python bridge enables crossing context for all crossing modes when `enable_crossings=True`; therefore `window` and `collision` modes can enter the global relaxed repair loop even though the task target is the LiDAR-style baseline.

- Observation: The LiDAR-style fanout and congestion reservations in `translation/route_rust.py` are heuristic scaffolding, not yet proven to match the LiDAR reference.
  Evidence: The code builds synthetic same-side port fanout lanes, keepout cells, and history cells around ports before routing. No LiDAR reference trace has been captured yet, so these choices are not yet tied to observed LiDAR route order, `checkSpacing`, ripup, or history behavior.

- Observation: Passing tests do not mean the feature works.
  Evidence: `cargo test` passed with 300 Rust tests, and `PYTHONPATH=. .venv/bin/pytest -q` passed with 256 Python tests and 1 skipped test, but the only `multiportmmi_8x8` routing artifact in this plan is a stop-51 debug partial with 45/51 committed routes and one known accepted self-intersection.

- Observation: The Rust/Python bridge already has `set_lidar_global_relaxed_repair_only`, and `translation/route_rust.py` already sets it only when `crossing_mode == "lidar-pure"`.
  Evidence: `rg` found `use_lidar_global_relaxed_repair_only` in `translation/route_rust.py` lines around 2692, and PyO3 setters/getters in `src/py_router.rs` around 5934. The remaining bug is the Rust entry condition around `route_many_with_repair_and_commit`, which still uses `self.crossing_context.is_enabled()` instead of the dedicated field.

- Observation: The known `n_33` bad route is rejected by both geometric segment intersection and expanded-cell overlap, because the diagonal segment crosses the later horizontal segment at an integer grid cell.
  Evidence: The first focused test run rejected the path before the expected assertion, but reported the repeated-cell guard first. Reordering the validator to check segment intersections before repeated cells made `cargo test route_sanity --lib` pass while preserving the repeated-cell test.

- Observation: `tests/test_rust_batch_repair.py::test_rust_batch_repair_rips_and_reroutes_dynamic_blocker` previously depended on a geometrically invalid terminal loop when `require_target_angle=True`.
  Evidence: After adding the guard and rebuilding the extension, the test failed with errors such as `route self-overlap: cell (47, 10) repeats between segment 2 and segment 4`. Setting the fixture's `AStarConfig.require_target_angle=False` makes the test exercise its intended dynamic ripup behavior and returns non-overlapping routes `[(2, 10), (2, 14), (47, 14), (47, 10)]` for net 1 and `[(5, 10), (44, 10)]` for net 2.

- Observation: The self-guard stop-51 run commits one fewer route than the previous probe-only artifact, but selected and all-attempt traces no longer include the known bad `n_33` normal route.
  Evidence: The previous artifact reported `45/51 routes` and had `route[34] n_33 attempt=71 bucket=normal_route` with a self-intersection. The new self-guard artifacts report `44/51 routes`; `n_33` appears only as successful `probe_route` records at attempts 37 and 72, and the all-attempt scan over 61 records reports zero non-adjacent intersections.

- Observation: Selected SVG export no longer constrains attempt-path JSONL coverage when `PHOTONIC_ROUTER_TRACE_ALL_ATTEMPT_PATHS=1` is set.
  Evidence: The command still used `--debug-svgs 32-37,48-51` and generated only 10 SVGs, but `build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard_all_attempts.jsonl` contains route indices 1 through 51.

- Observation: LiDAR completes the full `multiportmmi_8x8` reference run with DRV 0 using a different repair pattern around the first dense group.
  Evidence: `build/lidar_reference/lidar_multiportmmi8x8_trace.jsonl` has 1,764 records. The LiDAR log ends with `DRV: 0` and a critical path summary. The trace shows two `global_ripup_start` events, five `global_ripup_net` events, and 15 `history_update` events for the selected nets.

- Observation: The first major trace difference is crossing acceptance, not broad route ordering.
  Evidence: LiDAR routes `n_31` strictly, then `n_32` first fails strict and succeeds relaxed with `vioNets=['n_31']`, then accepts strict legal crossings for `n_33`, `n_34`, `n_35`, and `n_36`. This router also routes `n_31` first and detects `n_32` as conflicting with `n_31`, but then defers `n_33` through `n_36` as conflicts against earlier routes instead of accepting legal crossings.

- Observation: Crossing-aware A* can report accepted crossing candidates while the reconstructed route has no post-route crossing events.
  Evidence: A focused run with `PHOTONIC_ROUTER_TRACE_CROSSING_NET=34` and `--debug-stop-after-route 34` printed `crossing_accepted=16`, `candidates=348`, and returned waypoints `[(767,143), (758,143), (751,150), (751,156), (740,167), (730,167)]`, but `collision-crossing events ... events=[]`. The route was then deferred as a relaxed conflict with candidate blocker `[32]`.

- Observation: `RouteSearchStats.crossing_accepted` is aggregate search telemetry, not proof that the returned path itself contains a legal crossing.
  Evidence: The focused `n_33` trace had `crossing_accepted=16` while the returned route had no post-route crossing events. The implementation now stores path-local `RouteCrossing` events on `CrossingAStarNode` and reconstructs only the events along the selected parent chain.

## Decision Log

- Decision: Treat the stop-51 GDS as a diagnostic partial, not as a final LiDAR-equivalence result.
  Rationale: The command intentionally uses `PHOTONIC_ROUTER_NATIVE_PARTIAL_AFTER_GLOBAL_ROUND=1`, and the router returns before the next ripup. Missing routes in that file should not be diagnosed as final routing failures.
  Date/Author: 2026-07-06 / Codex

- Decision: Add a route-level self-intersection rejection before trying to make exact self-intersection a full neighbor-expansion rule in A*.
  Rationale: Exact prevention during neighbor expansion would require path-dependent geometry in the A* state or expensive ancestor reconstruction for many candidates. A reconstruction-time sanity gate is cheaper, keeps invalid routes out of committed state, and can be followed by cheaper local expansion guards if needed.
  Date/Author: 2026-07-06 / Codex

- Decision: Compare this engine to LiDAR by trace events and queue state, not primarily by screenshots.
  Rationale: Screenshots show symptoms but not why the repair loop chose a route. The important behavioral match is route order, strict/probe success, candidate blockers, ripup sets, history/congestion updates, and final crossing/legal geometry.
  Date/Author: 2026-07-06 / Codex

- Decision: For this milestone, port alignment in the GDS is out of scope.
  Rationale: The user explicitly wants the A* and crossing/ripup behavior to work first. Physical alignment to ports can be restored after route search and invalid geometry issues are solved.
  Date/Author: 2026-07-06 / Codex

- Decision: Implement the route self-intersection sanity gate as a pre-commit guard, not only as a final route table guard.
  Rationale: The dynamic obstacle map is mutated before several `final_routes.insert(...)` calls. A rejected route must not be committed to the obstacle map, registered as crossing geometry, remembered as a committed centerline, or used for probe analysis.
  Date/Author: 2026-07-06 / Codex

- Decision: Use the LiDAR script path `src/picroute/scripts/lidar/run_route.py` for reference traces.
  Rationale: The root-level `run_route.py` path named earlier in this plan is absent in the current LiDAR checkout; the script under `src/picroute/scripts/lidar/` is present and already configured to run `multiportmmi_8x8`.
  Date/Author: 2026-07-06 / Codex

- Decision: Treat the current implementation as an exploratory prototype until it has an all-route geometry scan and a LiDAR trace comparison.
  Rationale: The code compiles and tests pass, but the strongest behavior evidence is still a partial route with a known invalid accepted path. Continuing to tune heuristic costs without trace comparison risks fitting symptoms of this implementation rather than matching LiDAR's actual repair behavior.
  Date/Author: 2026-07-06 / Codex

- Decision: Do not continue broad heuristic tuning until the global relaxed repair mode is explicitly scoped.
  Rationale: The current Rust condition can run the new global relaxed repair loop for any enabled crossing context, which may regress existing `window` and `collision` modes. The next agent should either gate the path to `lidar-pure` or prove with tests that the broader behavior is intentional and correct.
  Date/Author: 2026-07-06 / Codex

- Decision: Implement self-overlap detection using both route segment intersection on `RouteResult.compressed_waypoints` and expanded integer cells along each compressed segment.
  Rationale: The known `n_33` defect is a geometric segment crossing that has no repeated route cell, while other invalid loops can return through a previously used cell without a clean segment crossing. Checking both forms keeps the guard focused on A* centerlines before GDS realization.
  Date/Author: 2026-07-06 / Codex

- Decision: Reuse the existing `use_lidar_global_relaxed_repair_only` PyO3 flag as the global repair entry gate instead of adding another field.
  Rationale: `translation/route_rust.py` already sets this flag only for `crossing_mode == "lidar-pure"`, and tests already cover the getter/setter. Changing the Rust condition to use this field is the smallest compatibility fix for `window` and `collision` modes.
  Date/Author: 2026-07-06 / Codex

- Decision: Do not advance to route 60/full benchmark tuning until the crossing-aware A* metadata/reconstruction mismatch is addressed.
  Rationale: The stop-51 trace already diverges from LiDAR at the first dense crossing bundle. Running farther would mostly amplify a known wrong behavior: this router ripups/defers routes that LiDAR accepts as legal crossings.
  Date/Author: 2026-07-06 / Codex

- Decision: Preserve crossing candidates as path-local route metadata instead of treating `crossing_accepted` as returned-route evidence.
  Rationale: A* may accept crossing moves on branches that do not end up in the final route. The post-route validator needs the crossing points selected by the actual parent chain, and it must still verify that those points survive reconstruction onto both polylines with required straight margins.
  Date/Author: 2026-07-06 / Codex

## Outcomes & Retrospective

As of this plan creation, the branch has made good progress toward a LiDAR-style crossing baseline, but it is not behaviorally equivalent to LiDAR and does not yet route `multiportmmi_8x8` cleanly. The current partial run shows the repair loop now defers relaxed-conflict candidate blockers rather than committing them, which is closer to the intended behavior. However, a self-intersecting `normal_route` was accepted, so accepted-route geometry is not yet safe. The next implementation step must prevent invalid same-net geometry from entering any committed route state, then use LiDAR trace comparison to continue aligning ripup and congestion behavior. A fresh audit on 2026-07-06 confirmed that the current code compiles and the listed Rust and Python tests pass, but also corrected the LiDAR runner path and strengthened the pre-commit placement requirement for the next fix.

After the deeper 2026-07-06 handoff review, the current implementation should be understood as a prototype with useful pieces, not as a proven route to the final result. The direction of adding a pre-commit self-intersection guard is sound because the current code demonstrably accepts invalid centerlines. The direction of continuing to add LiDAR-like costs, fanout reservations, and global repair heuristics is not yet proven. The next agent should first make accepted geometry safe, scope the experimental global relaxed repair mode, and produce comparable all-route traces before deciding whether to keep, tune, or remove the current LiDAR-inspired heuristics.

As of 2026-07-06 15:55Z, the first two immediate risks are addressed in code: invalid same-net centerlines now have a Rust pre-commit/probe guard, and the experimental LiDAR global relaxed repair loop is scoped to the explicit `lidar-pure` flag rather than all crossing modes. Rust validation is healthy: `cargo test` passes with 293 lib tests and 12 integration tests. The Python extension has not yet been rebuilt in this checkpoint, and no new stop-51 diagnostic has been generated yet.

As of 2026-07-06 15:59Z, the Python extension has been rebuilt successfully and the targeted Python regression set passes. The batch-repair test fixture was adjusted to avoid relying on a terminal-heading loop that the new geometry guard correctly rejects. The stop-51 benchmark diagnostic still needs to be rerun with the rebuilt extension.

As of 2026-07-06 16:05Z, the stop-51 self-guard milestone is complete. The known accepted `n_33` self-intersection is no longer accepted as a normal route, the all-attempt successful-route trace covers all 51 requested route indices, and the scan reports no non-adjacent self-intersections. This does not prove the full benchmark works: the stop-51 result is still a debug partial with 44/51 committed routes, and the next work is LiDAR reference trace extraction and queue-level comparison.

As of 2026-07-06 16:10Z, the LiDAR reference trace has been captured and compared enough to identify the next implementation target. The self-intersection guard improved accepted geometry, but it also exposed that this router is not accepting LiDAR-style legal crossings in the first dense bundle. The next milestone should focus on preserving or reconstructing crossing events from crossing-aware A* results, or otherwise making post-route crossing validation agree with the A* crossing decisions.

As of 2026-07-06 16:39Z, the code now preserves crossing metadata through A* reconstruction and validates preserved candidates before using them as post-route crossing events. This directly targets the `n_33` focused trace mismatch where A* had counted accepted crossing candidates but `crossing_events_for_route` found none. The Python extension has not yet been rebuilt after this metadata change, and the benchmark diagnostics still need to prove that the first dense group commits accepted routes.

## Audit Findings

The current worktree is in a healthy compile/test state. `cargo check`, `cargo test`, and the full Python test suite pass. This is good handoff evidence, but these tests mostly exercise unit behavior and small integration paths; they do not demonstrate a full successful `multiportmmi_8x8` route.

The known stop-51 artifact is intentionally partial. It should not be used as proof that the router almost works, because it was captured before a ripup step and reports only 45 committed routes out of 51 requested debug-stop routes. It is useful evidence for the `n_33` self-intersection and for inspecting the current repair trace.

The current `attempt_paths` artifact is filtered by debug route indices. The scan that found exactly one self-intersection only scanned selected route attempts, not every accepted route in the partial run. A future validation run must either generate an all-route attempt trace or expose all committed route centerlines directly from Rust for scanning.

The new global relaxed repair path in `src/py_router.rs` is too broadly enabled for a handoff without stronger tests. It currently keys off `self.crossing_context.is_enabled()`, not the requested Python `crossing_mode`. If that is intentional, the plan needs compatibility tests proving existing `window` and `collision` crossing modes still behave correctly. If it is not intentional, add a separate Rust flag such as `use_lidar_global_relaxed_repair` and set it only from Python when `crossing_mode == "lidar-pure"`.

The current Python fanout and history reservation logic is a local approximation of LiDAR behavior. It may be the right direction, but there is no trace evidence yet. Do not treat it as canonical until a LiDAR reference JSONL trace shows comparable route order, spacing failures, crossing events, ripups, and history updates.

## Context and Orientation

This repository is a hybrid Python and Rust photonic router. Python builds the schematic and GDS layout, while Rust performs grid-based A* routing through the PyO3 extension `photonic_router._rust`. The main user command is `routing_flow.py`, which loads a benchmark, translates it to an unrouted layout, and routes it using `translation/route_rust.py` and the Rust backend.

The benchmark under investigation is `multiportmmi_8x8`. It should model the same kind of dense MMI routing case used by the LiDAR reference repository. In this repository the benchmark entry point is `benchmarks/multiportmmi_8x8.py`. Related input data is under `benchmarks/data/`.

LiDAR is the reference router in `/home/benjamin/Documents/Repositories/working/LiDAR`. The key reference files are:

- `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/routing/astarsearch.py`, which implements LiDAR's A* search.
- `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/drc/drcmanager.py`, which checks the bitmap for obstacles, waveguide spacing, crossings, and blocked cells.
- `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/routing/drgridroute.py`, which manages route order, ripup, history maps, crossing insertion, and insertion-loss reporting.
- `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/routing/debug_trace.py`, which emits JSONL trace records when `LIDAR_ROUTE_DEBUG_FILE` is set.

An A* route here is a sequence of primitive moves on an integer grid. A primitive is a short straight or bend with a start angle and an end angle. A state is `(x, y, angle)`, where `x` and `y` are grid cells and `angle` is one of eight directions. A crossing is a legal place where two waveguides pass through each other with a crossing component. A ripup is removal of a previously routed net so that it can be rerouted after another net fails or conflicts. Congestion and history are costs added to discourage routing through areas that have repeatedly caused conflicts.

The current branch is `baseline/lidar-pure-crossings` at commit `fdc20e4` with uncommitted work. `git status --short` at plan creation showed:

    M routing_flow.py
    M src/astar.rs
    M src/obstacle_map.rs
    M src/py_router.rs
    M tests/test_realized_crossing_verification.py
    M tests/test_rust_backend_import.py
    M tests/test_rust_batch_repair.py
    M translation/route_rust.py
    M translation/route_rust_realization.py
    M translation/route_rust_records.py
    ?? .agent/
    ?? benchmarks/data/
    ?? benchmarks/multiportmmi_8x8.py
    ?? docs/photonic_router_graph_crossing_plm_full.tex
    ?? tests/test_multiportmmi_benchmark.py

The untracked file `docs/photonic_router_graph_crossing_plm_full.tex` is intentional and must not be committed or deleted as part of this task.

The most important current diagnostic artifacts are:

- `build/routed_multiportmmi_8x8_stop51_round1_probe_only.gds`
- `build/routes/multiportmmi_8x8_native_probe_debug_round1_probe_only.jsonl`
- `build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl`

The command that produced them was:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    PYTHONUNBUFFERED=1 BROWSER=/bin/true \
    PHOTONIC_ROUTER_NATIVE_PROGRESS=1 \
    PHOTONIC_ROUTER_TRACE_LIDAR_GLOBAL_REPAIR=1 \
    PHOTONIC_ROUTER_NATIVE_PROBE_DEBUG=1 \
    PHOTONIC_ROUTER_NATIVE_PARTIAL_AFTER_GLOBAL_ROUND=1 \
    /usr/bin/time -f 'elapsed=%e' timeout 240 \
    .venv/bin/python routing_flow.py multiportmmi_8x8 --debug-stop-after-route 51 --debug-svgs 32-37,48-51

The run completed in about 115 seconds and reported:

    Native route batch returned debug partial: 45/51 routes
    reason: debug partial after LiDAR global relaxed round 1 before ripup
    route search: astar_loop=111.2665s, attempts=74, failures=13, simple=0/51, repairs=1

## Plan of Work

First, add a route-level self-intersection and self-overlap sanity gate in Rust. This is the one current code path that is both clearly wrong and clearly evidenced. The check should run immediately after a route is reconstructed from A* and before the route is committed to the obstacle map, accepted by `route_native_batch` as a `normal_route`, analyzed as a `probe_route`, or used as a crossing-compliant relaxed route. It should reject non-adjacent centerline segment intersections and repeated non-adjacent physical cells. Adjacent segments sharing an endpoint must remain legal, because every normal path has adjacent primitives touching at endpoints. The check must return a useful error string that includes the net id or route context when available, and it must be visible in attempt diagnostics when a route is rejected.

The low-level helper should live near the `RouteResult` type and route reconstruction helpers in `src/astar.rs`, or in a small geometry helper module used by `src/astar.rs` tests. Prefer a helper that can be unit-tested independently, for example:

    fn validate_route_has_no_self_intersection(route: &RouteResult) -> Result<(), String>

If `RouteResult` is not the right type name in the local code, use the route result type that contains `cells`, `states`, and `compressed_waypoints`. The helper should use compressed waypoints or replayed grid-path segments, not GDS geometry, because this guard is meant to keep invalid A* centerlines out before GDS realization.

Call the validator in `src/py_router.rs` before any route can mutate committed state. The important call sites are `route_single_net_and_commit_native`, `route_single_net_and_commit_repair_native`, `commit_native_route_with_clearance`, and `commit_native_relaxed_conflict_route`. These functions cover normal routes, repair routes, static-only probe commits, and crossing-compliant relaxed commits. A useful rule is: if the next operation commits route cells to `self.obstacle_map`, registers crossing events, remembers committed centerlines, calls `analyze_native_probe_route`, or inserts into `final_routes`, validate first. To find remaining acceptance points, run `rg -n "final_routes\\.insert|commit_route_with_clearance|analyze_native_probe_route|remember_committed_route_centerlines" src/py_router.rs` after adding the helper and confirm each path is protected.

Second, add focused Rust unit tests. One test must construct the known bad waypoint pattern from `n_33`:

    (767,143), (763,143), (757,137), (757,131), (750,131),
    (728,153), (722,153), (722,147), (739,147),
    (749,157), (739,167), (730,167)

The test must prove the validator rejects it due to a non-adjacent segment intersection. Another test must prove ordinary adjacent segment sharing is legal, such as `(0,0) -> (4,0) -> (4,4)`. A third test should cover a non-intersecting detour to avoid over-rejecting useful LiDAR-style paths.

Third, scope the experimental global relaxed repair mode before further tuning. Inspect `src/py_router.rs` around `route_many_with_repair_and_commit`. The conservative implementation is to add a separate boolean field, setter, and getter for "use LiDAR global relaxed repair", set it from `translation/route_rust.py` only when `crossing_mode == "lidar-pure"`, and use that field instead of `self.crossing_context.is_enabled()` to enter the global relaxed repair block. If the broader behavior is intentionally desired for `window` or `collision`, add tests that demonstrate this explicitly and update this plan with the reason.

Fourth, rebuild and rerun the bounded diagnostic. Use `maturin develop --release` after Rust changes, then rerun the stop-51 command above. The expected outcome is that the previous self-intersecting `n_33` path is no longer accepted. It is acceptable if fewer than 45 routes are committed in the partial artifact after this guard; that means the guard is exposing a real search/repair problem for the next step rather than hiding it in GDS.

The rerun should include one pass with all-route attempt tracing. The simplest way is to run without narrowing `debug_route_indices` after confirming `MAX_DEBUG_ROUTE_SVG_EXPORTS` limits SVG output, or to add a separate debug flag so path JSONL records all successful attempts independently of SVG selection. The important result is an artifact that can scan every successful accepted or probe route, not only route indices 32-37 and 48-51.

Fifth, extract a LiDAR reference trace from the reference checkout. Run from `/home/benjamin/Documents/Repositories/working/LiDAR` using the local environment that already runs `src/picroute/scripts/lidar/run_route.py`. If the branch is not already `local_run`, check the current branch before changing anything. A useful tracing command is:

    cd /home/benjamin/Documents/Repositories/working/LiDAR
    rm -f /tmp/lidar_multiportmmi8x8_trace.jsonl
    LIDAR_ROUTE_DEBUG_FILE=/tmp/lidar_multiportmmi8x8_trace.jsonl \
    LIDAR_ROUTE_DEBUG_NETS=n_31,n_32,n_33,n_34,n_35,n_36,n_47,n_48,n_49,n_50 \
    LIDAR_ROUTE_DEBUG_CHECKSPACING_LIMIT=800 \
    .venv/bin/python src/picroute/scripts/lidar/run_route.py

If the LiDAR trace is too large, add `LIDAR_ROUTE_DEBUG_BBOX=min_x,max_x,min_y,max_y` after identifying the grid window around the problematic dense bundle. The LiDAR trace should be copied into this repository under `build/lidar_reference/` for comparison, but do not commit generated build artifacts unless explicitly requested.

Sixth, compare the two traces as queue state machines before adding new heuristics. For each net around the first dense issue, record whether LiDAR does strict route success, relaxed route success, violation, local ripup, global ripup, history update, and reroute. Compare those records against this repository's `native_probe_debug.jsonl`, `attempt_paths.jsonl`, and stderr trace lines emitted by `PHOTONIC_ROUTER_TRACE_LIDAR_GLOBAL_REPAIR=1`. If LiDAR pushes the route away earlier through `checkSpacing` or history, adjust `src/astar.rs` congestion cost and `src/py_router.rs` history/ripup wiring. If LiDAR uses a different queue order or ripup set, adjust the global repair ordering in `src/py_router.rs`. If the trace shows the Python fanout or group congestion scaffolding is not present in LiDAR's behavior, record that and consider removing or isolating the scaffolding instead of tuning it further.

Seventh, once the stop-51 bounded run no longer accepts invalid geometry and its queue behavior is explainable, increase the debug stop in stages: first route 60, then 80, then full `multiportmmi_8x8`. Keep SVG generation limited to at most 10 selected routes to avoid overloading the machine. Use `BROWSER=/bin/true` so debug SVG generation does not open many browser tabs.

## Concrete Steps

Work from this repository root:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter

Before editing, check that no benchmark process is still running:

    pgrep -af 'routing_flow\.py multiportmmi_8x8|timeout .*routing_flow\.py multiportmmi_8x8' || true

Inspect the current bad route evidence with:

    .venv/bin/python - <<'PY'
    import json
    from pathlib import Path
    p = Path('build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl')
    for line in p.read_text().splitlines():
        rec = json.loads(line)
        if rec.get('route_index') == 34 and rec.get('net_name') == 'n_33':
            print(rec['attempt_index'], rec['bucket_name'], rec.get('repair_round'))
            print(rec['compressed_waypoints'])
    PY

Expected evidence includes attempt 71 for `n_33` in `normal_route` with the self-crossing waypoint list shown in this plan.

After adding the validator and tests, run:

    cargo fmt
    cargo test
    .venv/bin/maturin develop --release
    PYTHONPATH=. .venv/bin/pytest -q

Then rerun the bounded diagnostic with a small selected SVG set, which is useful for visual inspection but does not by itself prove every accepted route is geometrically valid:

    PYTHONUNBUFFERED=1 BROWSER=/bin/true \
    PHOTONIC_ROUTER_NATIVE_PROGRESS=1 \
    PHOTONIC_ROUTER_TRACE_LIDAR_GLOBAL_REPAIR=1 \
    PHOTONIC_ROUTER_NATIVE_PROBE_DEBUG=1 \
    PHOTONIC_ROUTER_NATIVE_PARTIAL_AFTER_GLOBAL_ROUND=1 \
    /usr/bin/time -f 'elapsed=%e' timeout 240 \
    .venv/bin/python routing_flow.py multiportmmi_8x8 --debug-stop-after-route 51 --debug-svgs 32-37,48-51

After the run, preserve stable artifacts with a new suffix such as `self_guard`:

    cp build/routed_multiportmmi_8x8.gds build/routed_multiportmmi_8x8_stop51_round1_self_guard.gds
    cp build/routes/multiportmmi_8x8_native_probe_debug.jsonl build/routes/multiportmmi_8x8_native_probe_debug_round1_self_guard.jsonl
    cp build/routes/multiportmmi_8x8_attempt_paths.jsonl build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard.jsonl

Before relying on that scan as full evidence, ensure the artifact being scanned contains all successful accepted routes, not only selected debug route indices. If the current implementation still ties `attempt_paths.jsonl` to `--debug-svgs`, add a separate all-route path-tracing switch or temporarily run a no-SVG diagnostic that records every successful route centerline without exporting every SVG. Record the exact command and artifact path in this ExecPlan.

Use this Python script to check for remaining route self-intersections in the new all-route attempt-path artifact before inspecting GDS visually. It treats intersections between non-adjacent compressed waypoint segments as invalid and ignores adjacent segments that meet at their shared endpoint:

    .venv/bin/python - <<'PY'
    import json
    from pathlib import Path

    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(a, b, c):
        return (
            min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
            and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])
            and orient(a, c, b) == 0
        )

    def segments_intersect(a, b, c, d):
        o1 = orient(a, b, c)
        o2 = orient(a, b, d)
        o3 = orient(c, d, a)
        o4 = orient(c, d, b)
        if o1 == 0 and on_segment(a, c, b):
            return True
        if o2 == 0 and on_segment(a, d, b):
            return True
        if o3 == 0 and on_segment(c, a, d):
            return True
        if o4 == 0 and on_segment(c, b, d):
            return True
        return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)

    def bad_intersections(points):
        pts = [tuple(p) for p in points]
        bad = []
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            for j, (c, d) in enumerate(zip(pts, pts[1:])):
                if j <= i + 1:
                    continue
                if {a, b} & {c, d}:
                    continue
                if segments_intersect(a, b, c, d):
                    bad.append((i, a, b, j, c, d))
        return bad

    path = Path("build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard.jsonl")
    failures = []
    for line in path.read_text().splitlines():
        record = json.loads(line)
        bad = bad_intersections(record.get("compressed_waypoints") or [])
        if bad:
            failures.append((record.get("attempt_index"), record.get("bucket_name"), record.get("route_index"), record.get("net_name"), bad[:3]))
    print(f"records_with_non_adjacent_intersections={len(failures)}")
    for failure in failures:
        print(failure)
    PY

The expected output after the guard is `records_with_non_adjacent_intersections=0`. If self-intersections remain, treat that as a failed validation and fix the guard before proceeding to congestion tuning.

## Validation and Acceptance

The first milestone is accepted when `cargo test` passes and a focused test rejects the exact `n_33` self-intersection pattern. The bounded stop-51 diagnostic must no longer commit the known self-intersecting route as a successful normal route. If the route cannot be found after the guard, that is acceptable for this milestone because it means the router is no longer hiding an invalid candidate as a valid result.

The second milestone is accepted when `lidar-pure` global relaxed repair is explicitly scoped and the existing modes are protected. Either add a Rust/Python test proving `window` and `collision` modes do not enter the LiDAR global relaxed loop unless explicitly requested, or document and test that the broader behavior is intentional.

The third milestone is accepted when a new stop-51 run has an all-route attempt or committed-route trace, and the self-intersection scan reports `records_with_non_adjacent_intersections=0` for every successful accepted route. A scan over selected debug route indices is not enough.

The fourth milestone is accepted when a LiDAR reference JSONL trace exists and contains enough events to compare route order, checkSpacing, ripup, and history update behavior for the problematic nets. The trace does not need to be committed, but its path and command must be recorded in this ExecPlan.

The fifth milestone is accepted when this repository's trace can be lined up against LiDAR's trace for the first dense issue. For each of the nets around `n_31` through `n_36` and `n_47` through `n_50`, the plan should state whether the engines agree or differ on route success, blockers, ripup, and history/congestion updates. Any difference must lead to a specific next code change or an explicit decision that the difference is acceptable.

The full task is accepted when `multiportmmi_8x8` routes through the benchmark using the LiDAR-style baseline without accepted invalid geometries. For now, exact GDS port alignment is not required. Accepted invalid geometries include same-net self-intersection, same-net self-overlap, cross-net overlap not represented as a legal crossing, crossing footprints that overlap, or crossing footprints containing unrelated geometry.

## Idempotence and Recovery

All debug commands should be safe to rerun. `translation/route_rust.py` now clears the current-run `multiportmmi_8x8_attempt_paths.jsonl` and `multiportmmi_8x8_native_probe_debug.jsonl` files at the start of a debug run, but stable copied artifacts with suffixes such as `_round1_probe_only` are not cleared automatically.

Use `BROWSER=/bin/true` when running routing diagnostics so SVG generation does not open many browser windows. Keep `--debug-svgs` limited to 10 route indices or fewer. This SVG limit must not limit route-path JSONL coverage for validation; if it does, add or use a separate trace path that records all accepted route centerlines while keeping SVG export bounded.

Do not use destructive git commands such as `git reset --hard` or `git checkout --` to recover. The working tree already contains user-relevant uncommitted changes. If a generated artifact becomes confusing, create a new stable artifact suffix rather than overwriting old evidence.

Do not commit or delete `docs/photonic_router_graph_crossing_plm_full.tex`; it is intentional and unrelated to this task.

If `maturin develop --release` fails after Rust edits, run `cargo check` first and fix compiler errors in Rust before rerunning maturin. If Python imports fail, run `PYTHONPATH=. .venv/bin/pytest tests/test_rust_backend_import.py -q` to verify the extension import path.

## Artifacts and Notes

Current stable diagnostic artifacts:

    build/routed_multiportmmi_8x8_stop51_round1_probe_only.gds
    build/routes/multiportmmi_8x8_native_probe_debug_round1_probe_only.jsonl
    build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl
    build/routed_multiportmmi_8x8_stop51_round1_self_guard.gds
    build/routes/multiportmmi_8x8_native_probe_debug_round1_self_guard.jsonl
    build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard.jsonl
    build/routed_multiportmmi_8x8_stop51_round1_self_guard_all_attempts.gds
    build/routes/multiportmmi_8x8_native_probe_debug_round1_self_guard_all_attempts.jsonl
    build/routes/multiportmmi_8x8_attempt_paths_round1_self_guard_all_attempts.jsonl
    build/lidar_reference/lidar_multiportmmi8x8_trace.jsonl

Current artifact timestamps at plan creation:

    2026-07-06 09:25:31 +0200 126840 build/routed_multiportmmi_8x8_stop51_round1_probe_only.gds
    2026-07-06 09:25:31 +0200 16809 build/routes/multiportmmi_8x8_native_probe_debug_round1_probe_only.jsonl
    2026-07-06 09:25:31 +0200 139918 build/routes/multiportmmi_8x8_attempt_paths_round1_probe_only.jsonl

Current clean run summary:

    Native route batch returned debug partial: 45/51 routes
    reason: debug partial after LiDAR global relaxed round 1 before ripup
    attempts=74
    failures=13
    repairs=1
    probe_crossing_compliant=false for all 13 native probe debug records

Known bad accepted path:

    route_index=34
    net_name=n_33
    attempt_index=71
    bucket_name=normal_route
    repair_round=1
    crossing segment A: (750,131) -> (728,153)
    crossing segment B: (722,147) -> (739,147)

Current LiDAR debug environment variables:

    LIDAR_ROUTE_DEBUG_FILE=/tmp/lidar_multiportmmi8x8_trace.jsonl
    LIDAR_ROUTE_DEBUG_NETS=n_31,n_32,n_33,n_34,n_35,n_36,n_47,n_48,n_49,n_50
    LIDAR_ROUTE_DEBUG_BBOX=min_x,max_x,min_y,max_y
    LIDAR_ROUTE_DEBUG_CHECKSPACING_LIMIT=800

Current TUMPhotonicRouter debug environment variables:

    PHOTONIC_ROUTER_NATIVE_PROGRESS=1
    PHOTONIC_ROUTER_TRACE_LIDAR_GLOBAL_REPAIR=1
    PHOTONIC_ROUTER_NATIVE_PROBE_DEBUG=1
    PHOTONIC_ROUTER_NATIVE_PARTIAL_AFTER_GLOBAL_ROUND=1
    PHOTONIC_ROUTER_TRACE_ALL_ATTEMPT_PATHS=1

## Interfaces and Dependencies

The Rust A* implementation is in `src/astar.rs`. It produces route results with fields used by Python and debugging, including cells, states, primitive ids, compressed waypoints, length, cost, and search stats. Add the self-intersection validator near the route result reconstruction code or in a small internal helper module reachable from both tests and native batch routing.

Crossing-aware A* now also returns path-local crossing metadata. `RouteResult.crossing_events` is a `Vec<RouteCrossing>` containing only crossing candidates along the selected parent chain. `src/py_router.rs` must convert those candidates into `CrossingEvent`s only after checking that the candidate point still lies on the reconstructed current route and the committed partner route with enough straight margin. Do not use `RouteSearchStats.crossing_accepted` as a returned-route crossing proof; it is aggregate search telemetry.

The native batch repair loop is in `src/py_router.rs`, especially the section around `route_native_batch` that pushes `NativeRouteAttempt` records and inserts successful routes into `final_routes`. The validator must be called before any route reaches an obstacle-map commit, `final_routes.insert(job.net_id, route)`, probe analysis, or crossing-compliant relaxed commit. If a route is rejected, the code should record a failed attempt with an error message such as `route self-intersection`.

The LiDAR global relaxed repair loop is in `src/py_router.rs` inside `route_many_with_repair_and_commit`. Its current entry condition is too broad for a safe handoff. Add or use a dedicated Rust field for the requested LiDAR repair mode, expose it through PyO3, and set it from `translation/route_rust.py` when `crossing_mode == "lidar-pure"`. This keeps the experimental state machine from silently changing other crossing modes.

The Python orchestration and debug artifact handling is in `translation/route_rust.py`. It converts Rust batch results into GDS route records, writes per-route SVGs, and writes `attempt_paths.jsonl` and `native_probe_debug.jsonl`. Do not add expensive GDS-level checks here as the primary fix; use Python artifact parsing only for diagnostics. The accepted-route guard belongs in Rust so invalid routes cannot be committed to the dynamic obstacle map.

The benchmark command enters through `routing_flow.py`, which dynamically imports `benchmarks/multiportmmi_8x8.py`, translates the schematic, and calls the Rust router through `translation/route_rust.py`.

The LiDAR reference depends on its own Python environment under `/home/benjamin/Documents/Repositories/working/LiDAR`. Do not edit LiDAR files unless the user explicitly asks; use them as source material and trace generators. The current trace entry point is `/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/scripts/lidar/run_route.py`. If more LiDAR logging is needed, prefer enabling existing `debug_trace.py` environment variables before adding new instrumentation to the reference repository.

## Revision Notes

- 2026-07-06 15:21Z: Initial ExecPlan created at the user's request. It freezes the current in-progress state, records the known self-intersecting accepted route, and defines the remaining work needed to make `multiportmmi_8x8` route with LiDAR-style behavior.
- 2026-07-06 15:29Z: Fresh handoff audit corrected two misleading instructions: the LiDAR runner path is under `src/picroute/scripts/lidar/`, and the route sanity gate must run before obstacle-map commits, not only before `final_routes.insert`. The audit also recorded current validation evidence and added an exact post-run self-intersection scan so a new agent can start from a known compiling state.
- 2026-07-06 15:38Z: Deeper review clarified that the current implementation is not yet proven to point to the final result. It added audit findings about selected-route trace coverage, broad global-repair enablement, unproven fanout heuristics, and passing tests that do not cover full benchmark success. It also changed the next work from "keep tuning" to "make geometry safe, scope experimental repair, generate all-route evidence, then compare with LiDAR before further heuristic work."
- 2026-07-06 15:45Z: Clarified that all-route centerline tracing must be independent of selected SVG export. This prevents a future agent from scanning only the routes chosen for visual SVG debugging and mistaking that selected sample for complete geometry validation.
- 2026-07-06 15:47Z: Recorded the fresh working-tree/artifact audit and the implementation decision for a two-part Rust self-intersection guard before making code edits.
- 2026-07-06 15:55Z: Recorded implementation of the Rust route sanity gate, focused route-sanity tests, explicit LiDAR global repair gating, and passing full Rust test results. The next required step is rebuilding the Python extension and rerunning the bounded diagnostic.
- 2026-07-06 15:59Z: Recorded successful release extension rebuild, targeted Python test pass, and the batch-repair fixture adjustment required by the stricter route sanity guard.
- 2026-07-06 16:05Z: Recorded the stop-51 self-guard reruns, the new all-attempt trace switch, stable self-guard artifacts, and the all-route successful-attempt self-intersection scan result.
- 2026-07-06 16:10Z: Recorded LiDAR trace extraction, queue-level comparison, the focused crossing trace for net id 34, and the decision to address crossing-aware A* event reconstruction before running farther than stop-51.
- 2026-07-06 16:39Z: Recorded implementation of path-local crossing metadata preservation and the focused Rust tests that prove preserved candidates can survive reconstruction validation. The next required proof is a rebuilt extension plus focused and stop-51 benchmark diagnostics.
