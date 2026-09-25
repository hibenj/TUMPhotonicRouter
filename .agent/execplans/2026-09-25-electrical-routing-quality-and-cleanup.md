# Electrical heater-metal routing: correct guardrails, clean wire shapes, readable modules

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-25 09:10 -- Milestone 1 done (D11 closed: suite check green on four cases, redundant overlaps zero by geometry); Milestone 2 (L-shaped pad wires) starting.**


## Purpose / Big Picture

After the optical routing, the flow can route each heater's two electrical terminals: one to a bondpad in a row above the die, the other into a common bus along the bottom that returns to its own pad. On `heater_s_mod` (21 heaters, 22 pads) this works and verifies clean, but the metal looks wrong in three ways the owner named from the rendered layout: pad wires descend in staircases of one-pitch jogs instead of a single L or Z, wires enter their terminal contacts through small extra elbows, and the common bus is a 81-bend Steiner tree whose stubs zigzag and whose junctions are drawn as overlapping rectangles. Separately, the electrical benchmark suite's guardrail check has been red on every case since June (Decision Log D11 of the restructure plan): four commits added pads, contacts and a bus bondpad to the metal the metrics count, and the guardrail limits were never re-derived.

After this plan: the suite check is green with guardrails that state real properties (no same-net metal drawn twice, wire length equal to the Manhattan distance unless an obstacle forces more); every pad wire on `heater_s_mod` is an L or a Z with zero detour, terminals are entered straight, the bus is a trunk with straight stubs and merged junctions; `translation/electrical` (8,100 lines in 17 modules today) is a pipeline of named stages with typed inputs and outputs, each with its own tests, documented in `docs/ARCHITECTURE.md`. The optical routing and the 27 paper cells are untouched: the electrical step runs after the waveguides are realized and verified, and no paper configuration enables it.


## Progress

- [x] (2026-09-25 06:30) Survey (evidence lane): module inventory, algorithms, metric definitions, the four June commits, per-wire detour table on `heater_s_mod`, the one redundant overlap pair identified, test coverage. Plan written.
- [x] (2026-09-25 09:10) Milestone 1: root cause case (a): the bus escape's first rectangle kept its half-width end cap and so reproduced 200 um of the stripe's own footprint, where a branch's 10 um junction poke then counted as a second same-net rectangle; fixed by a `trim_start` option in `rect_geometry.wire_rects_for_points` used by the escape's realization and its verification twin (no allow-list change), with a unit test that fails on the old code. Metrics added: `pad_wires` table, detour and bend aggregates, `bus_length_um`/`bus_bend_count`, `wire_metal_area_um2`/`pad_metal_area_um2`. Guardrails: the nine pad-inventory limits dropped, the four routing quantities added at today's values, zero-tolerance kept at zero; `heater_s_mod` is the fourth suite case; baseline re-pinned; suite check exits 0 on all four cases. Rust 567, Python 533; gate 9/9 exact, verified independently. Today's values to beat in Milestones 2 and 3: heater_s_mod detour total 940 um, max bends 4, bus bends 79; heater_lanes_20 detour 740, bus bends 75; heater_lanes_ripup detour 220, bus bends 38.
- [ ] Milestone 2: pad wires without staircases or terminal hooks.
- [ ] Milestone 3: the common bus as a trunk with straight stubs and merged junctions.
- [ ] Milestone 4: `translation/electrical` as named stages with tests.
- [ ] Milestone 5: acceptance.


## Surprises & Discoveries

- 2026-09-25: the one "same-net redundant overlap" on `heater_s_mod` (a 20 x 5 um strip, 100 um2 raw, 66.7 um2 after the metric's attribution) was first read as a branch running into the escape; measured, it is the reverse: the escape's first rectangle extended 200 um back into the bus stripe (its start kept the half-width end cap every open wire end gets), so the strip where a branch legitimately pokes into the stripe was covered twice. A real geometry slip, on all four cases, fixed in Milestone 1.
- 2026-09-25: the size guardrails (length, raw and union area, overcount) were pinned before the metrics counted pad markers, terminal contacts and the bus bondpad; the ten-fold area jump is the pads (a 22-pad row dwarfs the wires), not worse wires. A limit on "raw metal area" that mixes pads with wires cannot state anything about routing quality.
- 2026-09-25: 13 of the 21 pad wires on `heater_s_mod` carry a detour of 20 to 120 um (length minus Manhattan distance), every wire has 3 bends; the detours are the staircases. The bus tree: 52,770 um, 81 bends.


## Decision Log

- 2026-09-25 (owner): fix the guardrail issues first, then improve the wire shapes and detours, then clean the code. The plan follows that order.
- 2026-09-25 (owner): Milestone 2 goes with the L-shape design (one horizontal run at the terminal row to the pad column, one vertical run into the pad, nested pad assignment, Z only as the fallback for a blocked column).
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

Where the staircases come from (evidence lane, 2026-09-25, four wires traced stage by stage on `heater_s_mod`): no obstacle is involved. Each pad wire is built as terminal -> sideways onto a *lane column* (the bundle skeleton's column shifted by the wire's rank times the 40 um track pitch: `_attach_point_and_tail`, `_route_prefix_to_pad_stub_start`) -> up the lane column -> at a shelf height below the pad row (`_individual_pad_lane_point`, staggered by rank so shelves do not collide) sideways to the pad's column (`_route_pad_stub_path`, grid A*) -> up into the pad. The lane column is neither the terminal's column nor the pad's, so every wire pays two horizontal jogs, and whenever the second jog runs back toward the first, the difference is the detour (20 to 120 um). A skeleton corner near the terminal adds a fourth bend on some wires. The elbow at the terminal contact is the adapter `terminal_contacts.terminal_access_path` drawing a margin stub out of the port before joining the trimmed tail.

Design: a pad wire is an L: from the terminal's port exit, one horizontal run at the terminal's row to the assigned pad's column, then one vertical run straight into the pad. Terminals stacked in one column (the four rows of a heater group) get pads whose columns nest: the top row the nearest pad, the bottom row the farthest, so a lower wire's vertical run never crosses an upper wire's horizontal run (river routing). When the pad column is blocked between the terminal row and the pad row (an obstacle cell, a reserved wire), the wire falls back to a Z: horizontal at the terminal row to the nearest free column, vertical to the shelf, horizontal to the pad column, vertical into the pad, i.e. today's mechanism, with the shelf heights kept. The pad assignment inside a bundle is chosen for nesting first (`pad_slots`), then the L is tried, then the Z. The terminal adapter becomes a straight landing: the wire's first segment starts at the contact and runs in the port's exit direction; the adapter dogleg disappears when the wire leaves in that direction.

Work: implement the L-first construction in `bundle_detail_router` (a new function per wire that returns the L or None, called before the lane/stub path; the Z path unchanged as the fallback), the nesting order in `pad_slots` (or in `_pad_lane_rank_maps` if the assignment order is decided there), the straight landing in `terminal_contacts`/`metal_realization`. Tests: on the three heater fixtures and `heater_s_mod`, every pad wire has detour 0 and at most 2 bends unless the test names the blocked column; a fixture with an obstacle above a terminal forces the Z and asserts its bend count and that the L was refused; the nesting test asserts no two wires of a bundle cross (segment intersection over the centerlines). Guardrails tightened: `pad_wire_max_bend_count` 2 (3 for the fixture with the blocked column), `pad_wire_detour_total_um` 0.

Acceptance: the per-wire table on `heater_s_mod` all zeros in the detour column and no wire above 2 bends; cross-net spacing unchanged (`cross_net_min_spacing_um` >= 20); the suite check green with the tightened limits; the rendered metal reviewed by the owner.


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
