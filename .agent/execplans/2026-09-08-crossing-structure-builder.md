# Contribution 2 as a general geometry builder: structure selector, router fallback

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Owner (2026-09-08): the crossings are always determined from the graph (the crossing plan); the pre-placed structures are alternative realizations of that one plan; a unit should decide per layer which realization to use, so that contribution 2 becomes a general geometry builder that resolves every planned crossing and, per layer, takes the cheapest feasible alignment -- or hands the layer to the guided router when none fits. After this plan the flow logs one decision line per crossing layer, the synthetic sweep reports fallback layers instead of rejections, and every layer of every benchmark gets a solution.

Steps (owner-approved order): (1) selector + tests, (2) per-layer router fallback, (3) verification of mixed runs, (4) tests, (5) validation on the seven benchmarks and the synthetic sweep, (6) docs. The dogleg (second column per lane) candidate is out of scope; it would slot into the selector as a third candidate.

## Progress

- [x] (2026-09-08 morning) Step 1, selector: `translation/crossing_structures.py` -- `evaluate_x_array` (all pairs opposite, no reversing lane, no dense source group, level columns fit; cost: 0 extra length, 0 corners), `evaluate_column_grid` (the column grid's probe: acyclic order with the 9 um corner floor, band fits; cost: Manhattan-minus-octile extra length, two corners per moving lane), `select_layer_structure` (least extra length, then fewest corners, then fewest tiles; `ROUTER` when nothing fits) and `LayerDecision.describe()`. `_derive_crossing_tiles` now asks the selector per layer, prints the decision, stores it on `DerivedCrossingLayout.layer_decisions`, and the verification metrics carry `layer_structures`; the old two-way rule `_stage_uses_column_grid` is gone. A fallback layer still stops the run with the reasons until step 2. Tests `tests/test_crossing_structures.py` (Benes layers -> X array with the column grid's cost reported; multiportmmi layers -> column grid with the X array's reason; a random single-port permutation -> router fallback with reasons on both candidates). Seven-benchmark grid ladder identical to before (all 0 failures / 0 repairs / 0 errors).
- [ ] Step 2, router fallback: a fallback layer keeps its original nets; those nets route with crossings enabled and the plan as guidance while the tiled layers stay crossing-free.
- [ ] Step 3, mixed-run verification: router-discovered crossings of fallback layers plus tiles must equal the plan.
- [ ] Step 5, synthetic sweep through the selector (expect the 28 rejections to become fallback layers that route).

## Surprises & Discoveries

- The selector's column-grid evaluation depends on the router's fan-out anchors, and those depend on the routing run's configuration (2 um grid, static stubs, bend radius): a test that probed with the session defaults (0.5 um grid) saw different anchor rows and declared every multiportmmi layer infeasible. The probe kwargs are part of the decision's inputs.

## Decision Log

- Cost model (lead): least extra waveguide length over the octile minimum, then fewest pre-wired corners, then fewest tiles. Reproduces today's choices exactly (X array on every Benes layer, column grid on every multiportmmi layer).

## Outcomes & Retrospective

Not yet applicable.

## Context and Orientation

`translation/preplaced_crossing_grids.py::_derive_crossing_tiles` (tiles mode) calls `translation/crossing_structures.py::select_layer_structure` per crossing layer; `_column_grid_stage(probe_only=True)` is the column grid's feasibility/cost probe; `evaluate_x_array` holds the X array's rules. The router fallback (step 2) will need a per-net crossing enable in `translation/route_rust.py` (today `enable_crossings`/`crossing_mode` are global).

## Concrete Steps

    PYTHONPATH=. .venv/bin/pytest -q tests/test_crossing_structures.py tests/test_preplaced_crossing_grids.py
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 --preplaced-crossing-grids true   # "crossing structure layer ..." lines

## Validation and Acceptance

Step 1 accepted (2026-09-08): tests pass, the grid ladder is unchanged. The plan is accepted when a mixed run (tiles + fallback layers) verifies with crossings == plan and the synthetic sweep has no rejections left.

## Idempotence and Recovery

Ordinary reversible edits; step 1 changes no geometry.

## Artifacts and Notes

Decision lines in every grid-mode log; `metrics.layer_structures` in the photonic verification JSON.

## Interfaces and Dependencies

`translation/crossing_structures.py`: `StructureCost`, `StructureCandidate`, `LayerDecision`, `evaluate_x_array`, `evaluate_column_grid`, `select_layer_structure`. `DerivedCrossingLayout.layer_decisions`.
