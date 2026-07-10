# Build a crossing verification foundation for router-discovered crossings

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds.

This document follows `.agent/PLANS.md` in this repository. It is informed by `.agent/PROJECT_GOAL.md`, which defines the repository-level goal: a very fast Rust/Python photonic router that can route benchmark designs and verify final physical geometry.

## Purpose / Big Picture

The current project goal is not to reimplement LiDAR. The goal is to make TUMPhotonicRouter a fast, verified photonic router. The immediate feature is router-discovered crossings: the route search should explore and choose legal crossings when they help complete a design.

Before adding more crossing heuristics, the repository needs a stable verification foundation. A future agent should be able to run `benes_4x4`, `benes_8x8`, and eventually `multiportmmi_8x8`, then know whether the final geometry is correct. If crossing insertion fails, the failure should say whether search failed, centerline geometry was invalid, realization changed the path, port snapping invalidated a protected segment, or final verification rejected the layout.

The first visible outcome is a repeatable harness that proves small crossing benchmarks either pass with legal crossings or fail with concrete classified errors.

## Progress

- [x] (2026-07-10 00:00Z) Captured the repository-level goal in `.agent/PROJECT_GOAL.md`.
- [x] (2026-07-10 00:00Z) Created this crossing verification foundation plan as the active next-step plan.
- [ ] Audit the existing crossing, realization, port-snapping, and verification code paths before editing behavior.
- [ ] Define the minimal final-geometry verification report format needed for crossing work.
- [ ] Add or update focused tests for legal and illegal crossing classification on small geometry fixtures.
- [ ] Run `benes_4x4` and `benes_8x8` with crossing debug artifacts and record pass/fail evidence.
- [ ] Use the stabilized harness to resume `multiportmmi_8x8` debugging.

## Surprises & Discoveries

No implementation discoveries have been recorded yet for this plan.

## Decision Log

- Decision: Treat LiDAR as a source of ideas for router-discovered crossings, not as the architecture to clone.
  Rationale: The repository goal is a faster Rust-backed router with verified output. Matching LiDAR's internal repair behavior is useful only when it helps explain crossing correctness failures.
  Date/Author: 2026-07-10 / Codex

- Decision: Keep topology-precomputed crossings as a later mode, but do not use them in the current `lidar-pure` / router-discovered path.
  Rationale: `multiportmmi_8x8` should be solved with crossings explored during routing itself. Topology-precomputed crossings can be included later as a separate comparison or optimization path.
  Date/Author: 2026-07-10 / Codex

- Decision: Use both structured JSON reports and pytest assertions for verification.
  Rationale: JSON reports under `build/` make benchmark failures inspectable by agents and humans; pytest assertions make fixture-level classification deterministic and prevent regressions.
  Date/Author: 2026-07-10 / Codex

- Decision: A* route selection should use a route-induced insertion-loss objective.
  Rationale: The router should choose among crossing and non-crossing alternatives using physical route quality: propagation length, bend penalty, and crossing penalty. Congestion/history costs can guide search and repair, but should remain distinguishable from physical insertion loss in reports.
  Date/Author: 2026-07-10 / Codex

- Decision: Physical crossings must come from the active PDK/gdsfactory crossing component.
  Rationale: Crossing legality depends on the actual component footprint and access geometry. Hard-coded abstract crossing markers are not enough to prove final GDS correctness.
  Date/Author: 2026-07-10 / Codex

- Decision: Put verification before more heuristic tuning.
  Rationale: Without a reliable pass/fail harness, agents can make the router appear to progress by changing traces or screenshots while still producing invalid physical geometry.
  Date/Author: 2026-07-10 / Codex

- Decision: Protect crossing-to-crossing route segments from port snapping.
  Rationale: Crossings should be decided during routing. Port snapping should only adjust source-to-first-crossing and last-crossing-to-target access geometry, otherwise it can silently invalidate crossing legality after search.
  Date/Author: 2026-07-10 / Codex

## Outcomes & Retrospective

This plan is newly created. No implementation milestone has completed yet.

## Context and Orientation

TUMPhotonicRouter is a hybrid Rust and Python photonic router. Python loads benchmarks, integrates with gdsfactory, orchestrates routing stages, performs geometry realization, and writes debug artifacts. Rust owns performance-sensitive routing logic such as obstacle maps, A* search, route state, crossing checks, and geometry-heavy kernels through the `photonic_router._rust` PyO3 extension.

The routing pipeline should be understood as setup, routing, path-length matching, port snapping, geometry realization, and verification. Crossings belong to the routing stage and must be preserved through later stages. Path-length matching already has regression coverage for some benchmarks, including heater-style cases such as `heater_s_mod`; crossing work must not regress those tests.

The realized crossing component should be read from the active PDK or
gdsfactory component library. Its footprint must inform crossing keepouts,
straight-access requirements, spacing checks, debug geometry, and final
verification. Do not treat a crossing as correct merely because two centerlines
intersect legally in the route record; the final geometry must contain the
crossing component and remain DRC-clean around it.

The immediate crossing benchmarks are `benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`. The smaller Benes benchmarks should establish basic correctness and harness confidence. `multiportmmi_8x8` is the current hard case and should be debugged after the harness can clearly classify failures. It must be solved on the router-discovered crossing path without topology-precomputed crossing hints.

Relevant files likely include:

- `routing_flow.py`, the end-to-end orchestration entry point.
- `translation/route_rust.py`, the Python bridge into Rust routing and crossing options.
- `translation/route_rust_realization.py`, where route records become physical geometry.
- `translation/photonic_verification.py`, where final route geometry should be classified.
- `translation/route_rust_records.py`, where route metadata and crossing records may need to preserve intent.
- `src/astar.rs`, where A* search and route result metadata live.
- `src/crossings.rs`, where crossing constraints and crossing-related Rust types live.
- `src/py_router.rs`, where the PyO3 router API and batch routing/repair behavior live.
- `src/geometry_realization.rs`, where Rust-side geometry helpers may affect realized paths.
- `tests/test_realized_crossing_verification.py`, existing crossing verification coverage.
- `tests/test_photonic_verification.py`, existing final verification coverage.
- `tests/test_route_rust_geometry.py`, geometry realization tests.

Do not assume these are the only files to change. Read the current code before editing.

## Plan of Work

First, audit the current pipeline. Identify where a route's intended crossing metadata is created, where the active PDK/gdsfactory crossing component is selected, where its footprint is converted into grid keepout/access rules, where route centerlines are transformed during realization, where the crossing component is inserted into final geometry, where port snapping occurs, and where final verification classifies intersections. Record the audit results in `Surprises & Discoveries` before changing behavior.

Second, audit the current A* cost model and insertion-loss report. Confirm where length, bend penalties, crossing penalties, history costs, congestion costs, and dynamic-conflict penalties enter route selection. Record whether the current `total_cost` already corresponds to route-induced insertion loss or whether it mixes physical loss with non-physical search guidance.

Third, define the smallest verification report that is useful for crossing work. It should distinguish successful legal crossings from search failures, illegal bend or near-bend crossings, unexpected route-route intersections, self-intersections, metadata/realization mismatches, and port-snapping protected-segment movement. Prefer extending existing verification structures over adding a parallel reporting system. The report should be available as structured data for tests and written as JSON under `build/` for benchmark runs. It should include insertion-loss terms: length loss, bend loss, crossing loss, total physical route-induced insertion loss, and any non-physical search penalties separately.

The report should also include crossing-component realization fields: component
name or factory, footprint/bbox used for legality checks, intended crossing
points, realized crossing placements, and any missing or mismatched component
issues.

Fourth, add focused regression tests around geometry classification. These tests should use small deterministic fixtures before full benchmarks. At minimum, include one legal straight-straight crossing, one illegal bend-adjacent crossing, one unexpected intersection, and one port-snapping or realization mismatch if the current code has a natural seam for that fixture.

Fifth, run `benes_4x4` and `benes_8x8` with crossing enabled and debug artifacts. The acceptance criterion is not merely a written GDS; final verification must report that crossings are legal and that no illegal intersections remain. If either benchmark fails, preserve the artifact paths and classified failure in this plan.

Sixth, only after the smaller benchmarks have classified evidence, resume `multiportmmi_8x8`. Use the same harness to decide whether the current blocker is crossing search, route repair, realization, port snapping, or final geometry verification.

## Concrete Steps

Work from the repository root:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter

Start with a status check:

    git status --short

Audit likely crossing and verification code:

    rg -n "crossing|snap|verify|intersection|realiz" translation src tests

Run focused existing tests before changing behavior, if the Python extension is built:

    PYTHONPATH=. .venv/bin/pytest -q tests/test_realized_crossing_verification.py tests/test_photonic_verification.py tests/test_route_rust_geometry.py

Run Rust checks around crossing and route geometry:

    cargo test crossing
    cargo test route_sanity --lib

After implementing harness or verification changes, run the narrow tests that cover the new behavior first. Then rebuild the Python extension if Rust/PyO3 behavior changed:

    .venv/bin/maturin develop --release

Then run the relevant Python tests again.

For benchmark evidence, use `BROWSER=/bin/true` so debug output does not open many browser tabs. The exact crossing flags must be confirmed from `routing_flow.py` before running, but the command shape should be:

    BROWSER=/bin/true .venv/bin/python routing_flow.py benes_4x4 --crossings --debug-timing
    BROWSER=/bin/true .venv/bin/python routing_flow.py benes_8x8 --crossings --debug-timing

Record the exact commands, outputs, and artifact paths here after they are run.

## Validation and Acceptance

The verification foundation milestone is accepted when:

- Focused tests classify legal and illegal crossing fixtures deterministically.
- Final-geometry verification reports legal crossings separately from illegal intersections.
- Benchmark runs write a structured JSON verification report under `build/`.
- The report separates physical route-induced insertion loss from non-physical search guidance costs.
- The report records the PDK/gdsfactory crossing component footprint and flags missing or misplaced realized crossing components.
- `benes_4x4` and `benes_8x8` have recorded pass/fail evidence with exact commands and artifact paths.
- If a benchmark fails, the failure is classified as search, route geometry, crossing legality, realization, port snapping, PLM interaction, or final verification.
- Existing path-length-matching regression tests relevant to heater benchmarks still pass or any failure is explicitly documented as unrelated pre-existing worktree state.
- The `lidar-pure` / router-discovered path does not consume topology-precomputed crossing hints.

This milestone does not require `multiportmmi_8x8` to route completely. It requires the harness to be strong enough that `multiportmmi_8x8` failures become actionable.

## Idempotence and Recovery

The audit and test commands are safe to rerun. Debug benchmark commands write under `build/`; generated artifacts may be overwritten unless copied to a timestamped or descriptive path. Do not delete existing diagnostic artifacts unless the active task explicitly requires cleanup.

The worktree currently contains unrelated modified and untracked files from earlier routing work. Do not revert them. If a file already has user or prior-agent changes, read it carefully and make only the edits needed for this plan.

## Artifacts and Notes

No validation artifacts have been generated by this plan yet.

## Interfaces and Dependencies

Prefer existing verification and route-record APIs. If new data is needed, keep it explicit and narrow:

- route records should preserve intended crossing points and protected route segments;
- realization should expose enough centerline information to compare intended and realized geometry;
- crossing realization should use the active PDK/gdsfactory crossing component and expose its footprint/bbox to verification;
- verification should return structured failures that tests can assert on;
- benchmark commands should emit artifact paths, summary status, and JSON verification reports.
- route-cost reporting should expose length, bend, crossing, total physical insertion loss, and separate search-guidance penalties.

Avoid a new broad orchestration framework until verification and active task planning are stable.
