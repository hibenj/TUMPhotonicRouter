# Refactor Python routing flow orchestration

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This plan follows `.agent/PLANS.md`.

## Purpose / Big Picture

The user wants the Python side of the router to reflect the actual system flow more clearly. Today `routing_flow.py` contains the public `run_routing_flow()` entry point, CLI setup, optical routing orchestration, verification reporting, path-length-matching reporting, electrical routing, debug artifact handling, and statistics copying in one long function. After this refactor, someone reading `run_routing_flow()` should see the main story directly: load benchmark, build layout, route optics, verify, optionally route electrical metal, and write/show output. The behavior should remain unchanged; this is a readability and maintainability refactor.

## Progress

- [x] (2026-08-11 11:45 local) Read `.agent/PLANS.md`, `.agent/WORKFLOW.md`, and the current `routing_flow.py` structure.
- [x] (2026-08-11 11:45 local) Identified the first safe slice: extract helper functions inside `routing_flow.py` so the public API and imports remain stable.
- [x] (2026-08-11 11:55 local) Extracted debug artifact cleanup/failure reporting, route stats population, and path-length matching reporting into top-level helpers in `routing_flow.py`.
- [x] (2026-08-11 11:58 local) Ran syntax and whitespace checks: `.venv/bin/python -m py_compile routing_flow.py` and `git diff --check` both passed.
- [x] (2026-08-11 12:00 local) Ran a real PLM smoke flow: `heater_s_mod` with path-length matching and heater obstacles passed and wrote clean verification JSONs.
- [x] (2026-08-11 12:08 local) Extracted final crossing/photonic verification, optional electrical routing, debug SVG reporting/opening, and final GDS/show output into top-level helpers.
- [x] (2026-08-11 12:10 local) Ran `heater_s_mod` with PLM and `--electrical-routing true`; the run passed and produced clean crossing/photonic verification JSONs.
- [x] (2026-08-11 12:35 local) Added `routing_flow_reporting.py` and moved route-attempt formatting, optical timing reporting, debug artifact cleanup/opening, partial failure artifact reporting, and final GDS/show output out of `routing_flow.py`.
- [x] (2026-08-11 12:38 local) Replaced the large inline `if debug_timing:` optical timing block in `run_routing_flow()` with a single `report_optical_timing(...)` call.
- [x] (2026-08-11 12:42 local) Re-ran syntax, whitespace, and `heater_s_mod` PLM plus electrical routing validation; all passed and crossing/photonic JSON reports were clean.
- [x] (2026-08-11 12:48 local) Ran `heater_s_mod` with `--debug-timing true` to exercise the moved timing reporter; it passed and printed the detailed timing report from `routing_flow_reporting.py`.
- [x] (2026-08-11 13:10 local) Added `routing_flow_component_info.py`, `routing_flow_verification.py`, `routing_flow_electrical.py`, and `routing_flow_plm.py`.
- [x] (2026-08-11 13:12 local) Moved crossing/photonic verification, heater electrical routing, and path-length-matching metadata/reporting out of `routing_flow.py`.
- [x] (2026-08-11 13:15 local) Re-ran syntax, whitespace, `heater_s_mod` PLM plus electrical routing, and `heater_s_mod --debug-meanders`; all passed.
- [x] (2026-08-11 13:35 local) Added `routing_flow_optical.py` with `OpticalRoutingStageConfig` and `OpticalRoutingStageResult`.
- [x] (2026-08-11 13:38 local) Moved the large `route_match_and_realize(...)` call, metadata loading, route attempt extraction, routing failure artifact reporting, and optical timing reporting out of `run_routing_flow()`.
- [x] (2026-08-11 13:42 local) Added small documented helper functions for legacy display option normalization, static obstacle config construction, debug artifact routing options, benchmark loading, layout translation, and route-stat initialization.
- [x] (2026-08-11 13:45 local) Re-ran syntax, whitespace, `heater_s_mod` PLM plus electrical routing, and verification JSON checks; all passed.
- [x] (2026-08-11 14:05 local) Added `routing_flow_stats.py` and moved `RoutingFlowStats`, `populate_route_stats(...)`, and `record_initial_route_stats(...)` out of `routing_flow.py`.
- [x] (2026-08-11 14:08 local) Added `routing_flow_config.py` and moved `DebugSvgSelector`, all `SCRIPT_*` defaults, debug SVG selector parsing, legacy display alias normalization, static obstacle config construction, debug artifact routing options, and optical-stage config construction out of `routing_flow.py`.
- [x] (2026-08-11 14:12 local) Kept imported `RoutingFlowStats` and `SCRIPT_*` names visible from `routing_flow.py` for compatibility with existing imports.
- [x] (2026-08-11 14:15 local) Re-ran import, syntax, whitespace, `heater_s_mod` PLM plus electrical routing, and verification JSON checks; all passed.
- [ ] Optional follow-up: move CLI parser construction into a small CLI module if `routing_flow.py` needs to become mostly `main()`, `load_benchmark()`, and `run_routing_flow()`.
- [x] (2026-08-17, Claude) Ran the full test suite for the first time against this slice (previous validation only ran targeted subsets per step) and found 3 names missing from `routing_flow.py`'s re-export surface that blocked collection entirely: `_format_debug_route_indices`, `_verification_status_metadata`, and `parse_debug_svg_selector` (test still imports it under its old private name `_parse_debug_svg_selector`). Fixed all three by adding the missing imports/alias to `routing_flow.py`, matching this plan's own established compatibility pattern.
- [x] (2026-08-18, Claude) Fixed the regression noted below, and found it was bigger than first diagnosed: 3 more names have the identical problem, not just `load_benchmark_metadata`. The general pattern, for each: the function's real call site moved into a new sibling module during this refactor, but a test still does `monkeypatch.setattr(routing_flow, "<name>", fake)`, which now patches an unused, orphaned attribute rather than the name the real code actually calls. Because each affected test chains several `monkeypatch.setattr(routing_flow, ...)` calls in sequence and `monkeypatch.setattr` raises immediately on a missing attribute, the first broken one (`load_benchmark_metadata`) was masking later ones in the same test; fixing it just exposed the next mistargeted call, not a working test. Found systematically, not by guessing, using a small script that regex-scans every `monkeypatch.setattr(routing_flow, "...")` call across `tests/*.py` (multi-line calls included) and cross-checks each name against `hasattr(routing_flow, name)`. The 4 mistargeted names and their real (correct) module: `load_benchmark_metadata` -> `routing_flow_optical`, `route_match_and_realize` -> `routing_flow_optical`, `verify_photonic_routing` -> `routing_flow_verification`, `route_electrical_heaters` -> `routing_flow_electrical`. Fixed by retargeting all 8 call sites (6 in `tests/test_routing_flow_stats.py`, 2 in `tests/test_path_length_graph.py`) to the module that actually owns the call now, per the user's explicit direction that this aligns with the restructuring's own goal (each module should own and be tested at its own real dependency, not be a proxy for a name that used to live in `routing_flow.py`), not by routing the calls back through `routing_flow.py` for compatibility. Verified: `PYTHONPATH=. .venv/bin/pytest -q` now reports `23 failed, 307 passed, 1 skipped`, an exact match to the pre-refactor baseline captured on committed HEAD (`a29dc00`) via `git stash`; all 23 remaining failures are the same pre-existing, unrelated ones documented in `.agent/execplans/2026-08-17-restructure-translation-route-rust.md`'s baseline list. One of the originally-reported 6 "regressed" tests, `test_path_length_graph.py::test_main_flow_flag_enables_path_length_matching`, does not pass even after this fix; that is correct and expected, since re-isolating it against clean HEAD showed it was already failing there too, for a real, unrelated, pre-existing bug (`TypeError: argument 'route': 'object' object cannot be converted to 'RouteResult'` in `translation/route_rust_realization.py:324`) that the `load_benchmark_metadata` mistargeting had been masking. This slice can now honestly be called complete pending only the optional CLI-parser item above.

## Surprises & Discoveries

- Observation: `routing_flow.py` is 2622 lines and `translation/route_rust.py` is 11327 lines.
  Evidence: `wc -l routing_flow.py translation/route_rust.py` reported `2622 routing_flow.py` and `11327 translation/route_rust.py`.
- Observation: `run_routing_flow()` contains a nested `_report_partial_debug_artifacts()` helper plus large blocks for stats population, debug timing printing, verification, PLM reporting, electrical routing, SVG opening, GDS writing, and timing output.
  Evidence: reading `routing_flow.py` lines 1474-2618 showed all of these concerns inside one function body.
- Observation: The focused stats pytest currently fails on this branch because `RouteAttemptRecord.as_dict()` contains new `crossing_hotpath_*` fields that the test's expected literal does not include.
  Evidence: `.venv/bin/python -m pytest tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested -vv` failed with a diff containing extra `crossing_hotpath_no_contact`, `crossing_hotpath_contact_checks`, and related fields. This appears unrelated to the helper extraction because the refactor still assigns `route_attempt_records` from `_route_attempt_as_dict(record)` unchanged.
- Observation: The default `TOY` CLI flow is not a reliable smoke test on the current branch.
  Evidence: `.venv/bin/python -X utf8 routing_flow.py TOY --debug-timing false` failed at `gc1_to_mmi_in2` with `No route found`.

## Decision Log

- Decision: The first implementation slice will keep changes local to `routing_flow.py`.
  Rationale: Moving functions into new files immediately would require choosing module boundaries and updating imports across tests. Extracting top-level helpers first reduces risk while making the main flow readable.
  Date/Author: 2026-08-11 / Codex
- Decision: Do not change `route_match_and_realize()` or `route_nets_rust()` in the first slice.
  Rationale: The user's immediate complaint is that the orchestration flow is unreadable. The Rust bridge is larger and should get its own follow-up plan after the top-level flow is clean.
  Date/Author: 2026-08-11 / Codex
- Decision: Keep the new helpers in `routing_flow.py` for now instead of creating new modules.
  Rationale: This preserves import behavior and makes the first slice an extraction-only change. Once the main flow is decomposed, helpers can be moved to modules such as `routing_flow_reporting.py` or `routing_flow_steps.py` with less risk.
  Date/Author: 2026-08-11 / Codex
- Decision: Move reporting and debug-output helpers into `routing_flow_reporting.py` as the next module boundary.
  Rationale: The user specifically called out that the large debug timing branch was still inside `run_routing_flow()` and that the cleanup had not yet split code out of the file. Reporting is a low-risk boundary because it does not alter routing state.
  Date/Author: 2026-08-11 / Codex
- Decision: Split the remaining stage helpers by stage concern rather than making one generic `routing_flow_steps.py`.
  Rationale: Separate `routing_flow_verification.py`, `routing_flow_electrical.py`, and `routing_flow_plm.py` map directly to the flow narrative and make each file easier to explain independently.
  Date/Author: 2026-08-11 / Codex
- Decision: Add `routing_flow_component_info.py` for shared `Component.info` handling.
  Rationale: Both verification and electrical routing attach metadata to the routed layout. A tiny shared module avoids keeping that helper in the orchestrator or duplicating private copies.
  Date/Author: 2026-08-11 / Codex
- Decision: Extract the optical router bridge call into `routing_flow_optical.py` using explicit config/result dataclasses.
  Rationale: Passing dozens of public flow arguments directly to `route_match_and_realize()` made the orchestrator hard to scan. The dataclasses keep the mapping explicit while letting `run_routing_flow()` remain a stage outline.
  Date/Author: 2026-08-11 / Codex
- Decision: Use short comments and docstrings only at module/stage boundaries.
  Rationale: Comments cannot compensate for nested orchestration. The code is easier to read when functions are named by responsibility and comments explain why a boundary exists.
  Date/Author: 2026-08-11 / Codex
- Decision: Move stats and config into separate modules, while re-importing public names in `routing_flow.py`.
  Rationale: `_populate_route_stats` and the `SCRIPT_*`/builder config code are not orchestration. Keeping the names imported preserves existing `from routing_flow import RoutingFlowStats` and `SCRIPT_*` usage while separating responsibilities.
  Date/Author: 2026-08-11 / Codex

## Outcomes & Retrospective

Sixth slice completed. `routing_flow.py` now reads as a cleaner orchestrator and is about 1050 lines. Config/default handling lives in `routing_flow_config.py`; legacy route stats live in `routing_flow_stats.py`; the main flow keeps the imported public names for compatibility. The remaining bulk in `routing_flow.py` is mostly CLI parser construction, `load_benchmark()`, small load/layout stage helpers, and `run_routing_flow()`.

## Context and Orientation

The repository is a hybrid Python/Rust photonic router. `routing_flow.py` is the CLI and top-level Python orchestrator. It calls `load_benchmark()` to import a benchmark module from `benchmarks/`, `layout_from_schematic()` from `translation/layout_from_schematic.py` to build an unrouted gdsfactory `Component`, and `route_match_and_realize()` from `translation/route_rust.py` to perform optical routing, optional path-length matching, and route geometry realization. After optical routing, `routing_flow.py` writes crossing and photonic verification reports, optionally calls `route_electrical_heaters()` from `translation/electrical/route_electrical.py`, then writes or shows the final GDS.

The term "orchestrator" means code that coordinates steps but should not contain the detailed implementation of every step. The target shape of `run_routing_flow()` is:

    run_routing_flow(...)
      -> normalize aliases and configs
      -> clear debug artifacts
      -> load benchmark
      -> translate schematic to layout
      -> run photonic routing
      -> run crossing and photonic verification
      -> report path-length matching
      -> optional electrical routing
      -> report/open debug artifacts
      -> write/show final layout

## Plan of Work

First, add small top-level helper functions in `routing_flow.py` just above `run_routing_flow()`. These helpers should mostly move existing code unchanged. The first group should handle debug artifact cleanup and partial debug artifact reporting. The second group should handle stats copying from `RouteSearchSummary` and debug timing output. The third group should handle verification report attachment, path-length matching reporting, electrical routing, debug artifact opening, and final layout output.

Second, rewrite the body of `run_routing_flow()` to call those helpers. Keep the function signature and documented public behavior unchanged. Avoid changing defaults, CLI argument names, GDS paths, report paths, stdout wording where practical, and exception behavior.

Third, validate with focused tests and small commands. At minimum run Python syntax compilation, a lightweight unit test that exercises `run_routing_flow()` with mocked internals, and a small real routing command such as `TOY` or `heater_s_mod` if runtime is acceptable.

## Concrete Steps

Work from repository root `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`.

Edit `routing_flow.py` using top-level helper extraction. Then run:

    .venv/bin/python -m py_compile routing_flow.py
    .venv/bin/python -m pytest tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested -v

The focused stats test currently fails because its expected route attempt dictionary is missing new `crossing_hotpath_*` fields. Use the failure as evidence of the existing test mismatch, not as proof that the refactor changed behavior. For a real routing smoke test, run:

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing false --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000

## Validation and Acceptance

The first slice is accepted when syntax passes, whitespace checks pass, and the `heater_s_mod` PLM flow produces `build/routed_heater_s_mod.gds` plus crossing and photonic verification JSONs whose `success` fields are true and whose `issues` lists are empty. The full refactor is accepted only when `run_routing_flow()` reads as a concise orchestration function while existing commands still work.

## Idempotence and Recovery

The refactor is source-only and can be retried. If validation fails, inspect the failure and either adjust the helper extraction or revert only the newly introduced helper changes. Do not revert unrelated user changes.

## Artifacts and Notes

No artifacts yet.

First-slice validation transcript:

    .venv/bin/python -m py_compile routing_flow.py
    # passed

    git diff --check
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing false --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000
    # passed; reported Path-length groups over_tolerance=0 and Meander insertion unmatched=0.000um

    verification JSON check:
    crossing True []
    photonic True []

Second-slice validation transcript:

    .venv/bin/python -m py_compile routing_flow.py
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing false --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000 --electrical-routing true
    # passed; electrical routes reported heaters=21, pads=22

    verification JSON check:
    crossing True []
    photonic True []

Third-slice validation transcript:

    .venv/bin/python -m py_compile routing_flow.py routing_flow_reporting.py
    # passed

    git diff --check
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing false --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000 --electrical-routing true
    # passed; electrical routes reported heaters=21, pads=22

    verification JSON check:
    crossing True 0 []
    photonic True 0 []

Sixth-slice validation transcript:

    .venv/bin/python - <<'PY'
    from routing_flow import RoutingFlowStats, SCRIPT_BENCHMARK, SCRIPT_OBSTACLE_CLEARANCE_UM
    print(RoutingFlowStats.__name__, SCRIPT_BENCHMARK, SCRIPT_OBSTACLE_CLEARANCE_UM)
    PY
    # passed; printed: RoutingFlowStats benes_16x16 0.0

    .venv/bin/python -m py_compile routing_flow.py routing_flow_config.py routing_flow_stats.py routing_flow_optical.py routing_flow_component_info.py routing_flow_electrical.py routing_flow_plm.py routing_flow_reporting.py routing_flow_verification.py
    # passed

    git diff --check
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing true --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000 --electrical-routing true
    # passed

    verification JSON check:
    crossing True 0 []
    photonic True 0 []

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing true --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000
    # passed; exercised report_optical_timing(...) and wrote build/routed_heater_s_mod.gds

Fourth-slice validation transcript:

    .venv/bin/python -m py_compile routing_flow.py routing_flow_component_info.py routing_flow_electrical.py routing_flow_plm.py routing_flow_reporting.py routing_flow_verification.py
    # passed

    git diff --check
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing true --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000 --electrical-routing true
    # passed; exercised moved verification, PLM, and electrical stage modules

    verification JSON check:
    crossing True 0 []
    photonic True 0 []

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing false --debug-meanders --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000
    # passed; exercised moved detailed PLM diagnostic reporting

Fifth-slice validation transcript:

    .venv/bin/python -m py_compile routing_flow.py routing_flow_optical.py routing_flow_component_info.py routing_flow_electrical.py routing_flow_plm.py routing_flow_reporting.py routing_flow_verification.py
    # passed

    git diff --check
    # passed

    .venv/bin/python -X utf8 routing_flow.py heater_s_mod --debug-timing true --path-length-matching true --path-length-match-outputs true --allow-45-degree-turns false --bend-radius-um 10.0 --include-heater-obstacles true --grid-size-um 2.0 --waveguide-clearance-um 0.0 --heater-clearance-um 10.0 --routing-window-scale 0.05 --max-iterations 5000000 --electrical-routing true
    # passed; exercised the extracted optical routing stage plus verification, PLM, and electrical stages

    verification JSON check:
    crossing True 0 []
    photonic True 0 []

## Interfaces and Dependencies

The public interface `routing_flow.run_routing_flow(...) -> Component` must remain unchanged. `routing_flow.main(argv: list[str] | None = None) -> Component` must continue to pass CLI arguments into `run_routing_flow()`. Existing imports from tests should continue to work.
