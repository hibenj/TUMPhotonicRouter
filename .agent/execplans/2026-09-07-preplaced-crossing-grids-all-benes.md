# Contribution 2 on every Benes size: pre-placed crossing grids as a one-flag configuration

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Owner direction (2026-09-07): "for now it is important that just all contributions work". Contribution 1 (`--crossing-mode lidar-guided`, S1) is landed and measured; contribution 2 (pre-placed crossing grids, `--preplaced-crossing-grids true`) had last been exercised on 2026-08-27, where `benes_16x16` completed only under an env-gated span order that no longer exists, `benes_32x32` had never been tried, and the two ExecPlans of the contribution (`2026-08-26-preplaced-crossing-grids-for-benes.md`, `2026-08-27-router-fixes-for-crossing-grid-stubs.md`) were left without an outcome. This plan makes contribution 2 a complete, switchable configuration: one flag, every Benes benchmark (4x4, 8x8, 16x16, 32x32) routing verification-clean, with the baseline lidar-pure defaults untouched (owner rule: exactly one of baseline / contribution 1 / contribution 2 runs at a time).

After this plan, `PYTHONPATH=. .venv/bin/python routing_flow.py benes_32x32 --preplaced-crossing-grids true` routes all 516 stubs with 30 grids / 416 crossing components and `error_count=0`, in about 30 s (the lidar-pure baseline of the same benchmark takes about 15 min).

## Progress

- [x] (2026-09-07 09:40-09:50) State on today's build (`fcf982d`, `.so` of 2026-09-04 17:33): `benes_4x4` and `benes_8x8` grid mode clean (3 s each); `benes_16x16` grid mode FAILS under the default `topological` order at index 64 (`n_s0_2_o0_to_s1_1_i0__to_grid`, the 2026-08-27 planar fan-out finding, repair rips [62, 63] and cannot reroute them) and completes clean under `--net-order topological-span` (7 s); `benes_32x32` grid mode FAILS even under span order at net 148 (`n_s0_12_o0_to_s1_6_i0__to_grid`, `sw_s0_12,o3 -> crossing_grid_stage1_run0,in_23`, source cell (364, 369), target (402, 778)); `multiportmmi_8x8` grid mode is rejected by the lattice generator (`ValueError: lane n_32 reverses direction at level 4; the straight-diagonal crossing lattice cannot realize that`).
- [x] (10:00) Cause of the 32x32 failure, from geometry: the stage-1 (and stage-8) grid of `benes_32x32` at the default `lane_pitch_um=20` is 482.1 um wide and 582.7 um tall, placed at x = 696.7..1178.8 in the 544.5 um interstage band (switch bodies end at x = 665.5, the next stage starts at 1210), leaving 31 um on each side for the 32-lane fan-in from the switch rows (220 um pitch, 3.5 mm tall) to the lattice rows. At pitch 14 the grid is 392.1 um wide (76 um per side), at pitch 10 it is 332.1 um (107 um per side). Runs with `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM=14` and `=10` (span order) both complete `benes_32x32` clean (516/516, 0 failures, 0 repairs, 30 s); the whole Benes ladder is clean at 14.
- [x] (10:10) Defaults changed so the contribution is one flag: `CrossingGridGeometry.lane_pitch_um` 20 -> 14 (`translation/preplaced_crossing_grids.py`, env override unchanged); `translation/route_order.py::default_net_order(preplaced_crossing_grids=...)` -> `topological-span` for grid mode, `topological` otherwise, used by `run_routing_flow` when `net_order` is None (the CLI `--net-order` default is now None, so an explicit order always wins); the CLI `--crossings` default is None and resolves to False under `--preplaced-crossing-grids`, else the script default (True); `routing_flow.stable_flags_for_configuration` drops `--crossings`/`--crossing-mode` from a benchmark's stable block when the user asks for pre-placed grids (every other stable flag stays). Tests first: `tests/test_route_order.py::test_default_net_order_is_span_only_for_preplaced_crossing_grids`, `tests/test_routing_flow_stats.py::test_flow_net_order_default_follows_the_configuration` (three cases, monkeypatched routing stage captures `config.net_order`), `::test_stable_flags_drop_crossing_discovery_under_preplaced_grids`.
- [x] (10:20) Grid ladder with the single flag `--preplaced-crossing-grids true` (no `--crossings`, no `--net-order`, no env), photonic verification `error_count=0` everywhere, placed crossing components equal the topology count everywhere: `benes_4x4` 20/20, 20/0/0, 2 grids / 2 X, A* 0.02 s, flow total 0.38 s; `benes_8x8` 68/68, 68/0/0, 6 / 16, A* 0.27 s, total 1.03 s; `benes_16x16` 196/196, 196/0/0, 14 / 88, A* 2.79 s, total 5.82 s; `benes_32x32` 516/516, 516/0/0, 30 / 416, A* 11.7 s, endpoint correction 2.5 s, verification 10.5 s, total 29.9 s (wall 32 s). GDS `build/routed_benes_{4x4,8x8,16x16,32x32}_grid_full.gds`. Reference lidar-pure walls from the 2026-09-04 S1 A/B (same machine): benes8 35.8 s, benes16 165.8 s, benes32 991.9 s.
- [x] (10:25) Guards: pytest 380 passed / 10 failed / 1 skipped -- the 10 are the documented baseline set (unchanged); `benes_8x8` stable baseline (no flags) 52/4/0, 16 crossings, both verifications 0 errors, identical to the S1 A/B row; `multiportmmi_8x8` stable baseline 111/0/0, 33 crossings, 0 errors, identical. ruff: no new findings in the five touched files (counts equal to HEAD), `ruff format` clean on the touched files except the pre-existing state of `preplaced_crossing_grids.py`.
- [ ] Owner decisions and follow-ups (none blocking): see Decision Log.

## Surprises & Discoveries

- The 2026-08-27 acceptance of `benes_16x16` grid mode silently depended on an env var (`PHOTONIC_ROUTER_LAYER_ORDER=span`) that S3 of contribution 1 replaced by `--net-order topological-span` on 2026-09-04 -- so on 2026-09-07 the documented reproduction no longer worked, and the default order still fails 16x16 exactly as recorded then. The order is a property of the configuration (planar stubs), not a tuning knob; it is now selected by the flow.
- `benes_32x32` grid mode fails for a reason the 16x16 fix cannot touch: the grid itself consumes the interstage band. The lattice's lane pitch (20 um) was chosen on 4x4/8x8 geometry where the band is mostly empty; at 32 lanes and 120 crossings per outer stage, the grid width scales with pitch times level count, and the remaining band must hold a 32-lane, 3.5 mm tall fan-in. The router did not fail on the lattice or on the stubs' legality; it simply ran out of columns.
- Contribution 2 does not apply to the multiportmmi benchmarks as built: their crossing plan has lanes that reverse direction within a stage (`n_32` at level 4 of `multiportmmi_8x8`), which the straight-diagonal diamond lattice cannot realize by construction (`_lane_movement` raises). A different structure (a general bubble-sort lattice with bends, or per-level pairwise crossing cells) would be a new design, not a fix.
- The grid-mode runs are dominated by non-routing work at 32x32: A* 11.7 s, endpoint correction 2.5 s, verification 10.5 s of 29.9 s total.

## Decision Log

- Decision (2026-09-07, lead, evidence-based, reversible by env): `lane_pitch_um` default 20 -> 14. 14 is the largest tested pitch at which `benes_32x32` routes; 10 also works. Both leave the 4x4/8x8/16x16 verdicts clean. The owner may prefer another value after looking at `build/routed_benes_32x32_grid_full.gds`; `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM` overrides it without code changes.
- Decision (2026-09-07, lead, closes the 2026-08-27 open question "how to adopt shortest-first within-layer ordering"): Option 1 of that Decision Log -- the flow selects `topological-span` whenever pre-placed grids are on, declaration order everywhere else. Implemented as `default_net_order`; explicit `--net-order` wins.
- Decision (2026-09-07, lead): a benchmark's `STABLE_ROUTING_FLAGS` describe its lidar-pure baseline; under contribution 2 the crossing-discovery flags of the block are dropped, the rest stays. Without this, `--preplaced-crossing-grids true` alone contradicted the injected `--crossings true`.
- Open (owner): whether contribution 2 must also cover the multiportmmi benchmarks (needs a new crossing structure for reversing lanes, see Surprises) or stays Benes-only as the 2026-08-26 plan scoped it.
- Open (owner): the two 2026-08-27 Milestones that were never done (M3 straight run-in reservation for grid ports; the halo-in-plain-kernel flag question was resolved by M1 unconditionally) -- the ladder is clean without them; they are optional geometry-quality work.
- Open (owner): PLM pass-through of the grid-internal lengths (`grid_internal_length_um_by_original_net`) is still only prepared, as recorded on 2026-08-26 Milestone 4; no Benes benchmark requires path-length matching, so nothing fails.
- Not committed: the owner did not ask for a commit; the diff is in the working tree (`git diff --stat`: routing_flow.py, translation/route_order.py, translation/preplaced_crossing_grids.py, tests/test_route_order.py, tests/test_routing_flow_stats.py, this plan and the state files).

## Outcomes & Retrospective

Contribution 2 is now a switchable one-flag configuration that routes every Benes benchmark verification-clean, including `benes_32x32` (516/516, 30 grids, 416 crossing components, 30 s), with no change to the lidar-pure baseline verdicts. Three defaults were the whole gap: the lattice pitch (a geometry budget the 32x32 band could not afford), the within-layer net order (planar stubs need shortest-first), and the interaction with the benchmarks' stable blocks (which encode the baseline's crossing discovery). None of the router's legality rules needed touching.

Retrospective: an acceptance that depends on an env var is not an acceptance -- when the env var was later replaced by a CLI option in another plan, the contribution silently stopped working. Configuration-level consequences of a mode (order, crossings off) belong in the flow's resolution of that mode, pinned by tests, not in the reproduction command.

## Context and Orientation

Contribution 2 lives in `translation/preplaced_crossing_grids.py` (grid geometry `CrossingGridGeometry`, the diamond-lattice builder, `derive_preplaced_crossing_layout`) and is wired in `routing_flow.py::_preplaced_crossing_grids_stage`, guarded by the mutual-exclusion checks in `run_routing_flow`. Net order rules are in `translation/route_order.py`; the flow passes the resolved order through `OpticalRoutingStageConfig.net_order` to `translation/route_rust.py`. The benchmarks' stable blocks are `STABLE_ROUTING_FLAGS` / `STABLE_ROUTING_ENV` in `benchmarks/benes_*.py`, applied by `routing_flow.main` as CLI defaults.

## Concrete Steps

    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4 --preplaced-crossing-grids true
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8 --preplaced-crossing-grids true
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16 --preplaced-crossing-grids true
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_32x32 --preplaced-crossing-grids true
    PYTHONPATH=. .venv/bin/pytest -q tests/test_route_order.py tests/test_routing_flow_stats.py tests/test_preplaced_crossing_grids.py
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8        # baseline guard, stable block

## Validation and Acceptance

Accepted when every command above exits 0, `build/verification/<bench>_photonic_verification.json` has `error_count=0` and `metrics.preplaced_crossing_grids.crossing_component_count == expected_crossing_count` for all four Benes sizes, the baseline runs keep their recorded verdicts (benes8 52/4/0 with 16 crossings; mm8 111/0/0 with 33), and pytest shows only the documented 10 baseline failures. All met on 2026-09-07 (see Progress).

## Idempotence and Recovery

Ordinary reversible source edits; no Rust change, no rebuild. `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM=20` restores the previous lattice geometry for comparison.

## Artifacts and Notes

- `build/routed_benes_{4x4,8x8,16x16,32x32}_grid_full.gds` -- the four grid-mode layouts of 2026-09-07 (single flag, defaults).
- `build/routed_benes_32x32_grid_p10.gds`, `build/routed_benes_32x32_grid_p14.gds` -- the pitch experiments (span order, env override).
- Session logs (scratchpad, session-local): `c2_<bench>_grid_only.log`, `c2_benes_32x32_grid_span_p{10,14}.log`, `pytest_c2.log`, `guard_{benes_8x8,multiportmmi_8x8}_baseline.log`.

## Interfaces and Dependencies

`translation/route_order.py::default_net_order(*, preplaced_crossing_grids: bool) -> str`; `routing_flow.run_routing_flow(net_order: str | None = None)`; `routing_flow.stable_flags_for_configuration(stable_flags, *, preplaced_crossing_grids) -> list[str]`; `CrossingGridGeometry.lane_pitch_um = 14.0`.
