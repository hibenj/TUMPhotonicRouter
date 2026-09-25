# Electrical heater-metal routing: correct guardrails, clean wire shapes, readable modules

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-25 -- plan written from the survey; Milestone 1 starting.**


## Purpose / Big Picture

After the optical routing, the flow can route each heater's two electrical terminals: one to a bondpad in a row above the die, the other into a common bus along the bottom that returns to its own pad. On `heater_s_mod` (21 heaters, 22 pads) this works and verifies clean, but the metal looks wrong in three ways the owner named from the rendered layout: pad wires descend in staircases of one-pitch jogs instead of a single L or Z, wires enter their terminal contacts through small extra elbows, and the common bus is a 81-bend Steiner tree whose stubs zigzag and whose junctions are drawn as overlapping rectangles. Separately, the electrical benchmark suite's guardrail check has been red on every case since June (Decision Log D11 of the restructure plan): four commits added pads, contacts and a bus bondpad to the metal the metrics count, and the guardrail limits were never re-derived.

After this plan: the suite check is green with guardrails that state real properties (no same-net metal drawn twice, wire length equal to the Manhattan distance unless an obstacle forces more); every pad wire on `heater_s_mod` is an L or a Z with zero detour, terminals are entered straight, the bus is a trunk with straight stubs and merged junctions; `translation/electrical` (8,100 lines in 17 modules today) is a pipeline of named stages with typed inputs and outputs, each with its own tests, documented in `docs/ARCHITECTURE.md`. The optical routing and the 27 paper cells are untouched: the electrical step runs after the waveguides are realized and verified, and no paper configuration enables it.


## Progress

- [x] (2026-09-25 06:30) Survey (evidence lane): module inventory, algorithms, metric definitions, the four June commits, per-wire detour table on `heater_s_mod`, the one redundant overlap pair identified, test coverage. Plan written.
- [ ] Milestone 1: the guardrails made true (D11 closed).
- [ ] Milestone 2: pad wires without staircases or terminal hooks.
- [ ] Milestone 3: the common bus as a trunk with straight stubs and merged junctions.
- [ ] Milestone 4: `translation/electrical` as named stages with tests.
- [ ] Milestone 5: acceptance.


## Surprises & Discoveries

- 2026-09-25: the one "same-net redundant overlap" on `heater_s_mod` is a bus branch whose last 5 um run into the bus-escape rectangle (a 20 x 5 um strip, 100 um2 raw, 66.7 um2 after the metric's attribution). Branches are clipped at their first entry into the bus stripe (commit 3ce1dfd) but not against the escape segment that commit d109169 added later. It is a real geometry slip, not a metric artefact, and the same slip appears on all three heater cases.
- 2026-09-25: the size guardrails (length, raw and union area, overcount) were pinned before the metrics counted pad markers, terminal contacts and the bus bondpad; the ten-fold area jump is the pads (a 22-pad row dwarfs the wires), not worse wires. A limit on "raw metal area" that mixes pads with wires cannot state anything about routing quality.
- 2026-09-25: 13 of the 21 pad wires on `heater_s_mod` carry a detour of 20 to 120 um (length minus Manhattan distance), every wire has 3 bends; the detours are the staircases. The bus tree: 52,770 um, 81 bends.


## Decision Log

- 2026-09-25 (owner): fix the guardrail issues first, then improve the wire shapes and detours, then clean the code. The plan follows that order.
- 2026-09-25 (lead, on D11, from the survey): neither "move the limits" nor "fix the router" alone. The two zero-tolerance guardrails stay at zero and the geometry is fixed (Milestone 1). The five size guardrails are replaced by limits on quantities that describe routing rather than pad inventory: total wire detour (length minus Manhattan distance, summed over pad wires), bends per pad wire, bus length and bends, wire-only metal area; the pad and contact areas are reported but not limited. The limits are derived from the improved output at the end of Milestone 3 and pinned with the baseline. Open for the owner only if the derived limits look wrong.


## Outcomes & Retrospective

Empty until work starts.


## Context and Orientation

Code: `translation/electrical/` (17 modules, 8,102 lines with `routing_flow_electrical.py`). Pipeline in `route_electrical.py::route_electrical_heaters`: `terminal_extraction` (each heater's two ports) -> `obstacle_extraction` (10 um grid, blocked cells, bus stripe) -> `common_bus_router` (a greedy group-Steiner tree by grid BFS from each heater's chosen terminal to the growing tree) -> trim the bus to its used span -> `individual_topology` (the other terminals grouped into escape bundles toward the pad row) -> `pad_slots` (pad positions and net ids) -> `escape_router` (the bus to its own pad, "endpoint dogleg" first) -> `bundle_detail_router` (grid A* per pad wire with turn penalties, then lane offsets inside a bundle: `DetailedBundleRoute.offset_path`) -> `metal_realization` (rectangles per path, pads, contacts, on the metal layer) -> `verification` (geometry contracts and the seven metrics in `_quality_metrics`, 1,349 lines) -> debug SVGs. Entry from the flow: `routing_flow_electrical.py::run_electrical_routing_step`; benchmark script `scripts/benchmark_electrical.py` with `BENCHMARK_GUARDRAILS` per case and `tests/baselines/electrical_suite_metrics.json`.

Run and look: `.venv/bin/python -m photonic_router route heater_s_mod --debug-svgs 1` writes `build/electrical/heater_s_mod_metal_snapshot.svg` and `build/routed_heater_s_mod.gds`; `rsvg-convert -w 12000 <svg> -o <png>` renders it. `scripts/benchmark_electrical.py heater_s_mod --artifacts-dir <dir> --output <json>` gives the metrics. The per-wire centerlines are public dataclass fields (`DetailedBundleRoute.offset_path`, `TerminalBusRoute.path` in `types.py`) but not exposed by the script.

Tests: `tests/test_electrical_routing.py` (40, some geometric), 4 flow-level tests in `tests/e2e/test_routing_flow_stats.py`, 15 script tests in `tests/e2e/test_benchmark_electrical_script.py`. Baseline pins: `scripts/test_baseline.sh` (Rust 567, Python 530); the paper gate `scripts/results/gate_short.sh` is run after every milestone as a guard although no paper cell routes electrically.

Terms. A *pad wire* is the route from a heater's individual terminal to its bondpad; a *bus branch* the route from the heater's other terminal into the common bus; the *bus escape* the segment from the bus stripe to the bus's own pad; the *detour* of a wire its centerline length minus the Manhattan distance between its endpoints; a *terminal contact* the rectangle where a wire lands on a heater port.


## Milestone 1: the guardrails made true (D11 closed)

Goal: `scripts/benchmark_electrical.py --suite --check --compare-baseline` exits 0 on the three heater cases and on `heater_s_mod` (added as a fourth case), with the zero-tolerance guardrails still at zero and the size guardrails replaced by routing quantities.

Work:
1. Geometry: clip every bus branch against the bus-escape rectangle the same way branches are clipped at the bus stripe (find `clip_manhattan_path_at_first_bbox_entry` and its use in `common_bus_router`/`metal_realization`; extend the clip target to the escape rectangle, or clip the escape at the branches, whichever keeps one rectangle per junction). Test: on the three heater fixtures and `heater_s_mod`, `same_net_redundant_overlap_pair_count == 0` and `metal_redundant_area_overcount_um2 == 0`; a unit test on a synthetic bus with an escape rectangle at a branch's column.
2. Metrics: add to `_quality_metrics` `pad_wire_detour_total_um`, `pad_wire_max_detour_um`, `pad_wire_max_bend_count`, `bus_length_um`, `bus_bend_count`, `wire_metal_area_um2` (wires and bus only, no pads, no contacts) and `pad_metal_area_um2`; keep the existing fields. Expose the per-wire table (terminal, heater, length, bends, Manhattan distance, detour) in the summary JSON under `pad_wires` so improvements are measurable without private helpers.
3. Guardrails: `BENCHMARK_GUARDRAILS` per case limits `same_net_redundant_overlap_pair_count` (0), `metal_redundant_area_overcount_um2` (0), `cross_net_min_spacing_um` (existing), `pad_wire_max_bend_count`, `pad_wire_detour_total_um`, `bus_bend_count`, `wire_metal_area_um2`; the raw/union/overcount area limits and the total centerline limit are dropped. For this milestone the new limits are set to today's values rounded up (they tighten in Milestones 2 and 3); the reasoning is written in the script's docstring. `heater_s_mod` becomes the fourth suite case. Baseline re-pinned.
4. Records: `.agent/REPOSITORY_STATE.md` item 3 closed with the outcome; the restructure plan's D11 marked closed.

Acceptance: the suite check green; `pytest -q tests`, `scripts/test_baseline.sh`, `scripts/results/gate_short.sh` green; the rendered `heater_s_mod` metal before and after compared by eye (only the junction strip changes).


## Milestone 2: pad wires without staircases or terminal hooks

Goal: on `heater_s_mod` every pad wire has zero detour and at most 3 bends (pad stub down, across, down to the terminal), enters its terminal contact straight, and keeps the required clearance; the same on the three heater cases.

Work: understand where the staircases come from (the bundle lane offsets in `bundle_detail_router`, the A* turn costs, or the pad-channel geometry), measured by the per-wire table of Milestone 1; then change the lane assignment so that wires in a bundle keep their lane from the pad row down to the terminal row (one horizontal jog at a lane-specific height, chosen so that lanes do not cross), and make the terminal entry a straight landing on the contact. Tests: per-wire detour 0 and bends <= 3 on the fixtures; a fixture with an obstacle between pad row and terminal that forces one extra jog, asserting the minimum. Guardrails tightened to the new values.

Acceptance: the per-wire table on `heater_s_mod` all zeros in the detour column; the suite check green with the tightened limits; the rendered metal reviewed by the owner.


## Milestone 3: the common bus as a trunk with straight stubs and merged junctions

Goal: the bus is a straight stripe plus one straight stub per heater (vertical from the terminal to the stripe, or one L when the terminal column is blocked), junctions realized as merged metal (one polygon per connected piece on the layer), the escape a single L; bend count and length on `heater_s_mod` reported before and after.

Work: replace the greedy Steiner growth with the direct stub construction where the column is free, keep the BFS only as the fallback for blocked columns; merge the realized bus rectangles per net before writing (klayout region merge), so no same-net overlap exists to count (the intentional-overlap allow-list in `verification.py` shrinks accordingly). Tests: stub straightness on the fixtures, one merged polygon per net on the metal layer, the fallback on a fixture with a blocked column.

Acceptance: bus bend count on `heater_s_mod` down from 81 to the number of forced L stubs plus the escape; suite check green with tightened limits; rendered metal reviewed by the owner.


## Milestone 4: `translation/electrical` as named stages with tests

Goal: the electrical router reads as the photonic one does after the restructure: a pipeline of stages with typed inputs and outputs, no module above about 600 lines, `verification.py` split into contracts and metrics, `debug.py` and the SVG writers separated from the routing, every stage with a test file, `docs/ARCHITECTURE.md` gaining a section on the electrical pipeline and `docs/CONFIGURATION.md` covering `ElectricalRoutingConfig`.

Work: stage by stage, behaviour-preserving (the suite's metrics and the per-wire table identical before and after each slice, checked by the baseline comparison), following the restructure plan's method: slice, verifier packet, independent check, commit.

Acceptance: metrics baseline identical to the Milestone 3 pin; module sizes; tests per stage; docs.


## Milestone 5: acceptance

The suite check green on four cases; `scripts/test_baseline.sh`, `scripts/results/gate_short.sh` green; one full 27-cell reproduction is not required (no paper cell routes electrically) unless a shared module outside `translation/electrical` changed, in which case it runs; the rendered `heater_s_mod` metal accepted by the owner; the plan's Outcomes written; the restructure plan's open-item list updated.
