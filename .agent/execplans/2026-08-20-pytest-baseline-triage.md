# Pytest baseline triage

## Purpose / Big Picture

After the batch-repair stale-test fix (see `.agent/execplans/2026-08-20-ripup-repair-orchestration-restructuring.md`) dropped the long-standing pytest baseline from 21 to 20 failures, the user asked for a triage pass: go through every remaining failure and either fix it (if cheap and safe) or document why it's deliberately staying failed, rather than continuing to treat "the baseline" as a fixed, unexamined number. This plan records that pass.

## Progress

- [x] Full triage of all 20 failures (as of the batch-repair fix) done, see Surprises & Discoveries. Investigation dispatched to a fork for the read-only diagnosis (report only, no fixes, no decisions -- matching this session's established fork-safety discipline). Grouped into 8 root-cause categories.
- [x] Fixed 6 stale/mechanical failures directly (small, fully-diagnosed, low-risk edits, per `.agent/CLAUDE_CODEX_FLOW.md`'s "small localized edits" exception -- no Codex round-trip needed given each fix was already fully diagnosed before editing):
  - `tests/test_benes_benchmark.py`'s 4 `_build_crossing_plan_info` tests: missing `crossing_search_loss` kwarg (added to production code since these tests were written), plus one of the four also needed `bend_runout_cells_per_crossing` set explicitly (a required-margin formula input that replaced `min_straight_cells_per_crossing` in that role via commit `8744da7d`, which this low-level test never picked up since it calls the builder directly).
  - `tests/test_rust_backend_import.py`'s crossing-rejection test: stale error-message regex (`"No route found"` -> `"No legal LiDAR crossing route found"`); the underlying rejection behavior was confirmed still intact.
  - `tests/test_routing_flow_stats.py`'s route-summary-stats test: missing 14 `crossing_hotpath_*` fields (added to production `route_attempt_records` serialization since the test's expected literal was written).
  - `tests/test_route_rust_opened_cells.py`'s dense-port fanout test: stale expected value (`source_dense_port_runway_cells=9` -> `6`), cross-referenced against the already-completed `.agent/execplans/2026-08-18-dense-port-runway-clearance-reach.md` recalibration.
  - `tests/test_path_length_graph.py`'s PLM-flag test: reached an unrelated, unconditional photonic-verification gate (`routing_flow_verification.py`) via fake `route_obj=object()` placeholders; opted out via the gate's own designed `realization_grid_spec=None` escape hatch, matching a sibling passing test's equivalent opt-out.
- [x] Investigated and rewrote one test whose expectation was invalidated by a deliberate, already-validated design change, not a live bug: `tests/test_port_alignment_diagnostics.py`'s `..._rejects_middle_static_contact` test (renamed to `..._allows_own_route_middle_static_contact`) -- see Surprises & Discoveries for the `git blame`-confirmed evidence this exemption has been in place since 2026-07-16, over a month before this session, and was itself validated end-to-end at the time.
- [x] Investigated and documented (not fixed) 6 failures found to be the same benchmark-placement-fact category as the already-documented `TOY` `gc1_to_mmi_in2` finding: `heater_s`/`heater_s_compact`/`mmi_heater`/`mmi_heater_8x4`/`mmi_heater_8x4_ripup_reroute`'s first gc-to-mmi-input connections. Confirmed (not assumed) these fail identically under bare `routing_flow.py <benchmark>` CLI defaults, not just under the stress-test settings the failing pytest cases use -- ruling out "search-mode artifact" as the explanation. Recorded in `.agent/REPOSITORY_STATE.md`'s "Not planned to be fixed" section at the same evidentiary standard as the existing TOY entry, explicitly flagged as inferred-by-shape rather than proven via TOY's own from-scratch BFS analysis.
- [x] Confirmed the remaining 5 (of 11) failures were already correctly explained by existing documented findings and needed no new work: 4 are `TOY`'s already-documented case (different test entry points into the same net), 1 is `heater_s_mod`'s already-documented PLM regression, both re-confirmed to match their exact recorded failure signatures.

Final baseline: **21 failed -> 11 failed**, 335 passed, 1 skipped. All 11 remaining failures are now documented (either pre-existing findings, re-confirmed, or the newly-added sibling-category finding above) -- none are unexplained.

## Surprises & Discoveries

- **The `test_checked_no_bump_endpoint_correction_rejects_middle_static_contact` failure looked like a possible live correctness regression at first (a route silently NOT being rejected for touching a static obstacle) but turned out to be a stale test from a deliberate, already-validated design change.** `src/py_router.rs:7101-7128`'s `old_core_keys` exemption -- any cell already part of a route's own pre-correction core cells is excluded from the static-overlap check, so a static obstacle added afterward at a cell the route's original geometry already passed through is not re-flagged -- was added in commit `8744da7d` (2026-07-16, "routing: stabilize multiport MMI crossing flow"), validated at the time against a real `multiportmmi_8x8` run (111/111 routes, 0 failures, crossing + photonic verification success). This test's scenario (add a static obstacle to a cell the route already occupies, expect rejection) has almost certainly been silently stale since that commit, over a month before this session -- not broken by anything done today. Renamed and rewrote the test to assert the current, intended behavior, with the git-blame evidence recorded in a comment so a future reader does not have to re-derive it.
- **A second, distinct bug was hiding behind the first `_build_crossing_plan_info` fix.** After adding the missing `crossing_search_loss` kwarg to all 4 `test_benes_benchmark.py` tests, 3 passed but `test_benes_crossing_plan_rejects_geometric_intersection_without_margin` failed differently -- not a `TypeError` anymore, but a real assertion mismatch (`actual_crossing_count == 1`, expected `0`). Traced to `translation/route_rust_crossing_plan.py:459-461`: the required-margin-cells formula now sums `crossing_half_size_cells + bend_runout_cells_per_crossing`, where `bend_runout_cells_per_crossing` is set by the real production caller (`translation/route_rust.py:6652`, from `self.bend_radius_cells`) but never appears in `_build_crossing_plan_info`'s own signature -- it's set on the returned `info` dict afterward, by the caller, not the builder. This test calls the builder directly and never set it, so it silently defaulted to 0, undershooting the margin the test's own hardcoded `required_margin_cells == 9` assertion expected. This value was almost certainly `min_straight_cells_per_crossing` before `bend_runout_cells_per_crossing` was introduced as a replacement input to the same formula (3 + 6 = 9 matches exactly). Fixed by setting `info["bend_runout_cells_per_crossing"] = 6` explicitly in the test, mirroring the real caller.
- **The "6 benchmark-placement-fact" finding was verified, not assumed, at each step**: confirmed the failures were not an artifact of `test_benchmarks_route_with_astar_only`'s stress settings (`enable_simple_routes=False`, `waveguide_clearance_um=0.0`) by reproducing both underlying nets (`gc_in_1_to_mmi_a_0_lower_in` via `heater_s`, `gc1_to_mmi0_in2` via `mmi_heater`) failing identically under bare `routing_flow.py <benchmark>` CLI defaults with no special flags. This rules out "an A*-only search-mode limitation" as the explanation and supports "genuine placement/clearance fact," matching TOY's already-documented category -- but this inference is by shape/pattern-match (same net-connection type, same unconditional-across-settings failure), not by redoing TOY's own from-scratch geometric BFS proof for each net. Recorded that distinction explicitly rather than overstating confidence.

## Decision Log

(No repository-owner decision was needed for this pass -- every fix applied was a small, fully-diagnosed, low-risk edit per the established exception, and the one ambiguous case (the static-contact test) was resolved by the user's explicit instruction to update the test to match current behavior after the investigation's findings were presented.)

## Outcomes & Retrospective

Complete, 2026-08-20. Net effect: pytest baseline dropped from 21 to 11 failures, and -- more importantly than the raw count -- every one of the 11 remaining failures now has a recorded, evidence-backed reason it's expected to stay failing, rather than being an unexamined part of "the baseline." This matches the actual goal the user stated when asking for this triage: not "zero failures" as a target, but "every failure explained."

What worked: dispatching the initial full-20 diagnosis to a fork with explicit report-only instructions kept the deep-dive investigation out of the main session's context while still producing evidence-dense, file:line-grounded findings for every failure. Two of the fork's findings turned out to need one more layer of direct investigation before a fix could be applied safely (the margin-formula bug hiding behind the kwarg fix; confirming the static-contact test's exemption was deliberate via `git blame` rather than assuming "probably stale") -- both cases where the extra step changed the actual disposition from what a shallower pass would have concluded.

## Context and Orientation

See the individual fix commits (`2aa42d7`, `59f1983`, `905b302`, `eb2d228`, `7d18e20`, `f940494`) for the exact diffs; each commit message states its own root cause and evidence. `.agent/REPOSITORY_STATE.md`'s "Not planned to be fixed" section has the durable record of the two benchmark-placement-fact categories (`TOY`, and the newly-added `heater_s`/`mmi_heater` sibling group).

## Plan of Work / Concrete Steps / Validation and Acceptance

Not applicable in the usual milestone sense -- this was a triage pass over a fixed, enumerated list (the 20 failures at the start), not a designed feature. Validation was simply: re-run the full suite after each fix and confirm the failure count dropped by exactly the expected amount with no new failures, which was done after every commit in this pass.

## Idempotence and Recovery

All changes are test-file-only edits (plus one `REPOSITORY_STATE.md` documentation update); safe to re-run `pytest` freely at any point.

## Artifacts and Notes

None beyond the commits and the `REPOSITORY_STATE.md` update.

## Interfaces and Dependencies

Not applicable.
