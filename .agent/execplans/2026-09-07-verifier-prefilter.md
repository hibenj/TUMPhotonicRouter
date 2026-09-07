# Photonic verifier: bounding-box prefilter and residue-based obstacle check

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

After contribution 2 (`.agent/execplans/2026-09-07-distributed-crossing-structure.md`) the router finishes benes_32x32 in grid mode in 4 s but the flow took 107 s, 93 s of it in `translation/photonic_verification.py`. The paper's timing claims for contribution 2 should not be dominated by a verifier that does 663 000 polygon booleans to find nothing. This plan makes the two hot checks cheap without changing any verdict.

## Progress

- [x] (2026-09-07 20:20) Profile (cProfile of the whole flow, benes_32x32 grid mode): `_verify_cross_net_route_overlaps` 90.4 s of 93.3 s -- 662 977 route pairs, each a `_combined_region` (45 s) plus a boolean; `_verify_route_obstacle_overlaps` 1.6 s. On multiportmmi_32x32 grid mode the split is the other way round: `_verify_route_obstacle_overlaps` 16.4 s of 18.5 s (916 routes each intersected with the whole obstacle layer because the residue is non-empty in grid mode: routes legitimately touch the tile instances at their ports).
- [x] Fix 1, bounding-box prefilter in `_verify_cross_net_route_overlaps`: a pair is examined only when the two route regions' bounding boxes overlap or touch (a polygon overlap implies a bbox overlap, so no verdict can change). benes_32x32 grid mode: verification 93.3 s -> 2.7 s, flow total 110 -> 15 s.
- [x] Fix 2, residue-based per-route obstacle check: `(route & obstacle) - (legal_global | per_key)` equals `(route & residue) - per_key` because `residue = (all_routes & obstacle) - legal_global` and the route is part of `all_routes`; the residue is tiny, the obstacle layer is the whole chip. multiportmmi_32x32 grid mode: verification 36 s (18.5 s in the profile run) -> 8.1 s, flow total 84 -> 24 s; multiportmmi_16x16 grid mode 2.5 s.
- [x] Guards: `tests/test_photonic_verification.py` 24 passed; full pytest 390 passed / same 10 baseline failures; multiportmmi_8x8 lidar-pure baseline verification 0 errors in both reports; grid-mode ladder verdicts unchanged (0 errors everywhere).

## Surprises & Discoveries

- The 2026-09-03 P5 pass already introduced the residue idea for the obstacle check but kept the per-route boolean against the full obstacle layer for routes touching the residue; in grid mode every route touches it (tile ports), so the full booleans came back through the side door.

## Decision Log

- No semantic change: both fixes are algebraically equivalent to the previous checks; issues, areas and bboxes are computed on the same regions as before.

## Outcomes & Retrospective

Grid-mode flow totals now track routing: benes_32x32 15 s, multiportmmi_32x32 24 s (were 107 s and 84 s). Lesson: profile the whole flow, not only the router -- the verifier was 85 % of the wall time nobody had looked at since the P5 pass.

## Context and Orientation

`translation/photonic_verification.py::verify_photonic_routing` runs the checks; `_verify_cross_net_route_overlaps` and `_verify_route_obstacle_overlaps` are the two changed functions. The flow wraps it in `routing_flow_verification.py::verify_and_attach_photonic_reports`.

## Concrete Steps

    PYTHONPATH=. .venv/bin/pytest -q tests/test_photonic_verification.py
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_32x32 --preplaced-crossing-grids true   # "Verification time" line
    PYTHONPATH=. .venv/bin/python -m cProfile -o prof.out routing_flow.py multiportmmi_32x32 --preplaced-crossing-grids true

## Validation and Acceptance

Accepted: verifier tests pass, every benchmark's verification verdict identical to before, grid-mode verification under 10 s on the largest cases. All met 2026-09-07.

## Idempotence and Recovery

Two local edits in one file; revert by `git revert` of the commit.

## Artifacts and Notes

Profiles in the session scratchpad only (`prof_benes32_grid.out`, `prof_mm32_grid.out`).

## Interfaces and Dependencies

None changed.
