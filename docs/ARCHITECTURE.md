# Architecture

TUMPhotonicRouter routes photonic integrated circuits: given a schematic
(instances of couplers, MMIs, heaters, and the nets between their ports) and a
placement, it produces a GDS layout in which every net is a waveguide path that
satisfies the geometric design rules (bend radius, clearance) and in which
waveguide crossings are realized by crossing cells.

Rust owns the grid database and the search; Python owns gdsfactory, the
benchmarks, the stage sequence and every artifact. Both sides are organised the
same way: one declared interface per replaceable concern (a Rust `trait`, a
Python `typing.Protocol`), one implementation file per concern, and a top-level
flow that can be read as a sequence of calls.

This document is the map. The knobs themselves are in
[`docs/CONFIGURATION.md`](CONFIGURATION.md) (generated from the dataclasses by
`scripts/generate_config_reference.py`); the paper's numbers and their
provenance are in [`docs/DATE2027_REPRODUCTION.md`](DATE2027_REPRODUCTION.md);
the restructuring that produced this layout is
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`; the
electrical heater-metal pipeline (section 2's `translation/electrical/`
subsection) was named as stages by
`.agent/execplans/2026-09-25-electrical-routing-quality-and-cleanup.md`'s
Milestone 4.

**How to read it.** Every claim below names the file it can be checked in.
Line counts are `wc -l` on the branch `restructure/modular-engine` at
2026-09-24 (Milestone 6 complete, commit `18530b8`) for sections 1 and 3-10,
and at 2026-09-28 (Electrical Milestone 4 slice 4 complete, commit `c71c6a5`)
for the `translation/electrical/` subsection of section 2; they are orders of
magnitude, not contracts. Historical file names appear only where a paragraph
says explicitly that they are historical.

Contents:

1. [The Rust crate, module by module](#1-the-rust-crate-module-by-module)
2. [The Python side, module by module](#2-the-python-side-module-by-module)
3. [The flow: the literal sequence of stage calls](#3-the-flow-the-literal-sequence-of-stage-calls)
4. [The search interface, and how to add an engine](#4-the-search-interface-and-how-to-add-an-engine)
5. [The negotiation loop, its policies, and how to add a policy](#5-the-negotiation-loop-its-policies-and-how-to-add-a-policy)
6. [Configuration objects and precedence](#6-configuration-objects-and-precedence)
7. [The three paper configurations](#7-the-three-paper-configurations)
8. [Build and test commands](#8-build-and-test-commands)
9. [Removal candidates (Milestone 8)](#9-removal-candidates-milestone-8)
10. [Shared state between stages](#10-shared-state-between-stages)


## 1. The Rust crate, module by module

`src/lib.rs` (80 lines) is the crate root: it declares the modules and
re-exports the public names other crates and the tests use. It contains no
logic. Below it are three module trees -- `search/` (one net), `engine/` (many
nets) and `bindings/` (the PyO3 surface) -- plus the grid and geometry modules
the first two are built on.

### `src/search/` -- one net, one path

`src/search/mod.rs` (128 lines) owns the search *interface* and nothing else:
`SearchEnvironment` (the obstacle map and primitive library a search runs
against), `SearchRequest` (source, target, port-open cells, and the two
optional shapes `DynamicExpansion` and `CrossingSearch`), `SearchOutcome`
(route plus counters), the `NetSearch` trait with its single `search` method,
and `anchor_open_cells`, the one helper both engines use to build their anchor
set. It must not know how any engine works, must not touch Python, and must not
know that a multi-net loop exists.

`src/search/state.rs` (269 lines) owns the state and result types shared by
every engine: `State` (grid position plus heading, with angle normalisation),
`RouteResult`, `RouteSearchStats`. `src/search/geometry.rs` (272 lines) owns the
grid-polyline geometry the engines and the reconstruction step share (waypoint
compression, orientation and segment-intersection primitives). Neither knows
about costs, heuristics or obstacle ownership.

`src/search/grid_dijkstra.rs` (474 lines) is the second `NetSearch`
implementation, `GridDijkstraSearch`: uniform cost (f = g), the full grid, no
routing window and no crossing hook, built by calling the A* kernel's own
building blocks. It exists to prove the seam is real and to serve as an
optimality oracle in tests; it refuses a crossing request with an
`unsupported_request` stat. It must not acquire crossing support -- that is what
makes it a useful oracle.

`src/search/test_support.rs` (417 lines, `#[cfg(test)]`) owns the search-side
fixtures; production code must not reach into it.

### `src/search/astar/` -- the production engine

The A* engine is one module tree so that each concern of the kernel can be read
on its own. `mod.rs` (1,033 lines) owns `AStarSearch`, the two stable
free-function conveniences (`route_single_net`, `route_single_net_with_config`)
and `run_search`, the single body every request shape flows through.
`kernel.rs` (1,479 lines) owns the loop itself (`run`), generic over a
`CrossingLegalityHook`: build grid, seed, pop, goal test, candidate moves,
legality, cost, push-or-improve. `config.rs` (169 lines) owns `AStarConfig` and
the small enums that parameterise a search. `cost.rs` (546 lines) owns every
g-cost term as a named function (`length_cost`, `bend_cost`,
`heuristic_estimate`, `step_cost`) -- no cost arithmetic belongs anywhere else.
`heuristic.rs` (439 lines) owns the plain, diagonal and heading-aware
estimates. `expansion.rs` (2,459 lines) owns move generation
(`candidate_moves`), the reject rules (`move_is_legal` returning a
`MoveVerdict`) and the push-or-improve steps. `window.rs` (664 lines) owns
routing-window bookkeeping and the progressively-larger-window driver.
`dense.rs` (2,147 lines) owns the dense per-search storage (`DenseRoutingGrid`,
bitsets, owner grids) and the JPS4 shortcut. `crossing_rules.rs` (2,254 lines,
plus `crossing_rules/tests.rs` 3,368) owns crossing legality and the
`CrossingLegalityHook` trait with its `NoCrossingHook` / `LiveCrossingHook`
implementations. `simple.rs` (915 lines) owns the straight/L/Z shortcut tried
before full A*. `diagnostics.rs` (833 lines) owns the tracing and report
printers (which take a `&mut dyn Write`, stderr in production, so tests can
read them). `svg.rs` (116 lines) owns debug SVG export. None of these modules
may read configuration from the environment or commit anything to the obstacle
map: a search reads the map, the engine writes it.

### `src/engine/` -- many nets, commitment, repair

`src/engine/mod.rs` (294 lines) owns the `PyPhotonicRouter` struct -- the
router's per-run state: the obstacle map, the primitive library, the crossing
context, the committed routes and opened cells, the history/congestion
bookkeeping, the typed `RouterConfig`, and `search_engine: Box<dyn NetSearch +
Send + Sync>`, chosen once at construction by `search_engine_for`. It declares
the submodules and owns no algorithm.

`jobs.rs` (331 lines) owns the per-net job and batch types (`NativeRouteJob`,
the repair batch state and its defaults). `cells.rs` (632 lines) owns the cell
arithmetic: footprint inflation, port footprint cells, clearance-exempt cells,
commit and core cells. `commit.rs` (805 lines) owns committing a route to the
obstacle map and everything that accumulates on a successful commit (history,
long-straight congestion, crossing spacing, post-commit guidance).
`search_calls.rs` (3,132 lines) owns the bridge from a job to a
`SearchRequest`: it builds `AStarConfig` and the crossing search configuration
and calls `self.search_engine.search(...)`. `crossing_reservation.rs` (3,218
lines) owns crossing reservations, partner sets and the realized-crossing
validation. `crossing_geometry.rs` (579 lines) owns the pure geometry used
there (segment intersection, point-segment distance, parallel overlap,
collinearity, centerline compression). `endpoint_correction.rs` (2,677 lines)
owns the checked grid-to-port correction pass. `diagnostics.rs` (248 lines)
owns the trace records and the batch timers. `test_support.rs` (750 lines,
`#[cfg(test)]`) owns the engine-side fixtures. No module here implements a search; each one calls the interface of
section 4.

### `src/engine/negotiation/` -- the loop and its policies

`mod.rs` (15 lines) is the module list. `loop_.rs` (1,302 lines) owns the loop
itself: `run_negotiated_batch(jobs, params) -> (RepairBatchState,
NegotiationCounters)` is the loop without PyO3, and
`route_many_with_negotiated_repair_and_commit_impl` is the thin converter above
it. Each *rule* the loop applies lives in its own module and can be replaced
without touching the loop: `budget.rs` (293 lines, `AttemptKind`,
`BudgetSchedule`, `LadderBudgets`), `crossing_free.rs` (102 lines,
`CrossingFreePolicy`, `PlannedPairsOnly`, `Never`), `ripup.rs` (409 lines,
`RipUpPolicy`, `LidarStyleRipUp`, `NoRipUp`), `queue.rs` (213 lines,
`NetQueue` and the three tables the rules read), `attempt.rs` (469 lines, one
attempt and its trace line), `probe.rs` (536 lines, "who is blocking this
net"), `braid.rs` (957 lines, the local swap of two crossing routes). Section 5
reads the loop in full.

### `src/bindings/` -- the PyO3 surface

`mod.rs` (3,290 lines) owns the single `#[pymethods]` block of
`PyPhotonicRouter`: every method Python can call, and nothing else.
`types.rs` (938 lines) owns the `#[pyclass]` configuration and result types
(including `PyRouterConfig`, which validates the search engine name at
construction). `convert.rs` (578 lines) owns the conversions in both directions
(route results, batch timing dictionaries, primitive descriptions).
`meander_py.rs` (600 lines) owns the meander-planning bindings. The rule that
makes this tree readable: bindings convert and delegate, they do not decide.
Anything that decides belongs in `engine/` or `search/`.

### The grid and geometry modules

`src/obstacle_map.rs` (1,725 lines) owns the routing database: the single
source of truth for blocked cells, static rectangles, dynamic per-net
ownership, packed `u64` cell keys, clearance metrics, congestion and history.
Everything above it queries it; it knows nothing about nets' names, search or
Python. `src/static_obstacle_builder.rs` (1,778 lines) owns building that map
from rasterized geometry (Python extracts the polygons and ports).
`src/primitives.rs` (427 lines) owns the movement primitives and their
footprint metadata. `src/crossings.rs` (374 lines) owns the crossing
constraints and the expected-pair context, deliberately independent of the A*
hot path. `src/geometry_realization.rs` (6,817 lines) owns centerline-to-polygon
realization and port access. `src/simple_routes.rs` (2,510 lines) owns the
straight/L/Z candidate representation and validation (no search).
`src/meander.rs` (988) and `src/auto_meander.rs` (1,895) own analytic and
obstacle-aware meander planning; `src/plm.rs` (2,435) owns the path-length
matching planner. `src/config.rs` (302 lines) owns the typed configuration tree
(`RouterConfig` and its four groups) -- since Milestone 1, `grep -rn
"env::var\|var_os" src/` is empty, so no algorithm in the crate reads the
environment.


## 2. The Python side, module by module

### `python/photonic_router/` -- the package a human drives

`flow.py` (809 lines) owns the flow: `route_benchmark(config, options)` and
`route_schematic(schematic, config, options)`, the two entry points, plus the
`run_routing_flow` keyword wrapper kept for existing callers. It reads neither
the environment nor a command line. `flow_options.py` (200 lines) owns
`FlowOptions`: the flow-level choices as eleven frozen groups, one group per
consuming stage. `cli.py` (811 lines) owns the command line: the parser, the
benchmark stable block, `--configuration`, and `_flow_options`, which turns the
parsed arguments into `FlowOptions`; `__main__.py` (51 lines) makes it `python
-m photonic_router route <benchmark>`. `config.py` (412 lines) owns
`RoutingConfig` and `RouterConfig` -- the typed configuration trees;
`env_overlay.py` (839 lines) owns the one `PHOTONIC_ROUTER_*` overlay table
(`ENV_OVERLAY`) and is the only place in the repository that parses those
strings; `config_loading.py` (37 lines) owns `build_config(stable_env)`, the
precedence of section 6. `static_obstacle_builder.py` (1,162 lines) owns
obstacle extraction and the `ObstacleMapBuilder` Protocol;
`primitive_library.py` (179) owns the 1:1 map from Rust primitive ids to
gdsfactory components; `topology_analysis.py` (322) and `crossing_plan.py`
(242) own depth/rank analysis and the crossing events derived from it;
`path_length_graph.py` (323), `graph_analysis.py` (85),
`benchmark_extractor.py` (177) and `routing_layers.py` (133) own their named
analyses. Nothing in this package may import `translation.routing`'s stage
internals; it goes through the routing API (`translation/routing/api.py`).

### `translation/routing/` -- the routing session as stages

`session.py` (372 lines) owns the session shell and nothing else: it validates
the keywords into `SessionSettings`, builds `SessionState`, and calls the nine
phases (section 3). `settings.py` (295 lines) owns `SessionSettings`, the
frozen inputs of one run (49 fields grouped by consumer) with the
constructor's validation rules. `state.py` (339 lines) owns `SessionState`,
everything one run computes: one documented field per value, grouped by first
writing phase, with writer and reader phases named (section 10).
`stages.py` (207 lines) owns the nine `Protocol`s and the small frozen
dataclasses a phase returns -- the declared contract each phase answers to.
`crossing_plan_info.py` (316 lines) owns `CrossingPlanInfo`, the typed crossing
plan (66 fields grouped by writer phase) with `to_dict`/`from_dict` at the
report boundaries. `timing.py` (38 lines) owns the pipeline timers.

The nine stage modules own one phase each and are named for it:
`obstacle_context.py` (94), `router_setup.py` (280), `route_jobs.py` (1,664),
`crossing_plan_stage.py` (669), `handoff.py` (766), `dispatch.py` (1,949),
`finalize.py` (1,161), `verify_repair.py` (1,112), `realize.py` (123). A stage
takes `(settings, state, <its input>)` and returns its product; it calls its
own helpers by name and another stage's helper through an explicit import. A
stage must not know the command line, must not read the environment, and must
not reach back into the session object -- since Milestone 5 there is no
session object to reach into. `api.py` (516 lines) owns the module-level
routing API (`route_match_and_realize` and the fan-out anchor query): the
boundary the flow's optical stage helper (`routing_flow_optical.py`) calls,
reaching it through the `translation.route_rust` re-export.

### The remaining `translation/*.py`

`route_rust.py` (445 lines) is a re-export shim over `translation.routing`,
kept because the reproduction scripts, the benchmark stable blocks and many
tests import names from that path; no behaviour lives in it. The modules with
real content are `route_rust_types.py` (1,380, the shared record and job
types), `route_rust_records.py` (477, the route bookkeeping),
`route_rust_geometry.py` (686), `route_rust_realization.py` (437),
`route_rust_endpoint_correction.py` (905), `route_rust_meanders.py` (3,117),
`route_rust_crossing_plan.py` (689), `route_rust_crossing_components.py` (461),
`route_rust_crossing_verification.py` (918), `route_rust_debug_artifacts.py`
(342), `route_rust_analysis.py` (59) and `route_rust_obstacle_config.py` (164).
Beside them: `preplaced_crossing_grids.py` (2,640, contribution 2's
crossing-structure placement), `crossing_structures.py` (248),
`crossing_modes.py` (57, the mode vocabulary), `crossing_verification_report.py`
(803), `photonic_verification.py` (1,155, the authoritative geometry
verifier), the three `path_length_*.py` modules (1,068 together),
`route_order.py` (203, the net orders), `layout_from_schematic.py` (93),
`route_gds.py` (179, the gdsfactory reference router) and
`gds_write_options.py` (18).

### `translation/electrical/` -- the heater-metal pipeline as stages

`translation/electrical/` (28 modules, 9,440 lines) owns the heater-metal
routing stack: terminal extraction, the common bus, pad planning, detailed
pad-wire routing, metal realization and its own verification and debug SVGs.
It shares the grid vocabulary with the optical side but has its own router;
the optical stages must not import it. `__init__.py` (61 lines) is the public
re-export surface every caller outside the package imports through. Below,
in the pipeline order of `route_electrical.py::route_electrical_heaters`:

`route_electrical.py` (214 lines) owns `route_electrical_heaters`, the
literal twelve-stage sequence, after the pattern of
`translation/routing/session.py::run` -- one call per stage, each returning
the frozen product its `stages.py` Protocol names. It decides nothing itself
beyond the guard that skips a stage when its inputs are not ready (the next
subsection).

`stages.py` (234 lines) owns the twelve `typing.Protocol` declarations
(`TerminalExtractor` through `DebugArtifactWriter`), one per stage, after the
pattern of `translation/routing/stages.py`; its module docstring records
every place an implementation's real parameter names or defaults differ from
the stage table below, so the Protocol and the function it types never
silently drift apart.

`types.py` (486 lines) owns every dataclass the pipeline passes between
stages: `ElectricalRoutingConfig` (the typed configuration, with its own
`validate()`), the terminal and port-access types
(`ElectricalTerminal`, `TerminalPairGroup`, `ElectricalPortAccess`), one
result type per stage (`ElectricalObstacleMap`, `CommonBusRoutingResult`,
`PadPlan`, `PadSideReconciliation`, `CommonBusEscapeResult`,
`IndividualEscapeTopologyResult`, `DetailedBundleRoutingResult`,
`ElectricalVerificationResult`), and the top-level `ElectricalRoutingResult`.
No algorithm lives here.

`terminal_extraction.py` (208 lines), stage 1: `extract_heater_terminal_pairs`
finds heater instances by component-name pattern and instance prefix
(`config.heater_component_patterns`, `heater_instance_prefixes`) and groups
each heater's electrical ports into its two interchangeable
`ElectricalTerminal`s (`TerminalPairGroup`). Must not touch the grid or the
obstacle map.

`obstacle_extraction.py` (187 lines), stage 2: `build_electrical_obstacle_map`
builds the layer-filtered electrical grid (reusing
`photonic_router.static_obstacle_builder`), the die and layout bounding
boxes, the fixed `BusStripe`, and, through `port_access`, every terminal's
grid-anchored port access. Must decide obstacles and anchors, never a route.

`port_access.py` (286 lines) owns the terminal-to-grid anchor selection every
later phase reuses: `build_electrical_port_accesses` /
`build_terminal_port_access` (one `ElectricalPortAccess` per terminal per
purpose), `choose_route_start_cell` / `ordered_route_start_cells`
(`RouteStartChoice`). Knows grid cells and physical points, not routing
algorithms.

`terminal_contacts.py` (365 lines) owns the physical contact geometry of one
terminal: `terminal_access_path` (`TerminalAccessPath`, the continuous
adapter before the gridded route tail), `select_terminal_port_for_side` /
`select_terminal_contact`, `terminal_contact_bboxes`,
`terminal_access_keepout_bbox`, `port_contact_bbox`. Pure geometry, no grid
search.

`common_bus_router.py` (539 lines), stages 3 and 4: `route_common_bus`
dispatches the column-trunk phase (`column_trunks.py`), then a local-trunk
phase, then a greedy per-terminal BFS through `bus_search.py`, selecting
exactly one terminal per heater; `trim_common_bus_to_connections` is the
public `CommonBusTrimmer`, limiting the realized stripe to its used span. It
owns the phase order and terminal-selection policy, not cell-level search.

`column_trunks.py` (442 lines) owns the straight column-trunk construction
(Milestone 3): `route_column_trunks` groups selected bus terminals by column
and drops one straight vertical trunk with a stub per member beside the
heater bodies; `straight_drop_to_bus` serves a single swapped terminal
directly. Falls back to `bus_search`'s BFS when a column is obstructed.

`bus_search.py` (191 lines) owns the grid-search primitives every common-bus
phase shares: `all_terminal_cells`, `forbidden_terminal_cells`,
`nearest_terminal_cell`, `axis_path`, `path_hits_blockers`,
`shortest_path_to_tree` (the BFS to the growing tree). No phase ordering or
terminal-selection policy lives here.

`individual_topology.py` (609 lines), stage 5:
`compute_individual_escape_topology` routes every individual (non-bus)
terminal coarsely toward the pad side, groups them into `EscapeBundle`s
ordered outward and downward for river routing, and records
`terminal_order` and `cell_usage`. Feeds `pad_slots`' nesting order and
`bundle_detail_router`'s river construction.

`pad_slots.py` (503 lines), stage 6: `plan_pad_slots` lays out the one-sided
pad-pitch grid (one `PadSlot` per index) and assigns every selected bus or
individual net to a slot (`PadAssignment`), nesting individual assignments
by the escape-topology bundle order so a lower wire's vertical run never
crosses an upper wire's horizontal run.

`pad_side_reconciliation.py` (243 lines), stage 7: `reconcile_pad_sides` /
`_apply_pad_side_consistency_swap` swap a lone heater's bus and individual
terminal roles when its individual exit points away from its own pad, then
re-run stages 4 through 6 (trim, topology, pad-slot planning) from the swap;
a heater with no such mismatch leaves every object unchanged.

`escape_router.py` (246 lines), stage 8: `route_common_bus_escape` is the
path from the common-bus tree to its assigned pad slot, trying the endpoint
dogleg first and a BFS to the target cells as the fallback.

`bundle_detail_router.py` (473 lines), stage 9, the dispatcher:
`route_detailed_bundles` tries the whole-bundle river construction
(`river_routing.try_river_route_bundle`) for every escape bundle and falls
back, wire by wire, to the full-grid search
(`pad_wire_search.route_full_grid_pad_wire`) when the river refuses. Owns
commit/fail bookkeeping and route ordering, not the geometry of either
construction.

`river_routing.py` (243 lines) owns `try_river_route_bundle`: routes a whole
escape bundle at once as nested corridor and channel lanes, each wire a
monotone L or 3-bend Z (`count_direction_changes`), committed only if every
wire in the bundle is collision-free.

`pad_wire_search.py` (345 lines) owns the per-wire fallback:
`route_full_grid_pad_wire` (a reachability check plus the A* stub search
`search_pad_wire`) routes one pad wire against obstacles, other terminals,
and already-committed wires.

`wire_geometry.py` (242 lines) owns the grid and wire geometry both pad-wire
modules share: `wire_reservation_radius_cells` / `wire_spacing_radius_cells`
(the clearance-versus-spacing distinction Milestone 2's verification fix
introduced), `dilate_cells`, `cells_from_point_path`, `centerline_points`,
and the plain grid-segment and cell-center helpers.

`metal_realization.py` (405 lines), stage 10: `realize_electrical_metal`
turns every routed path (bus, escape, pad wires) plus pads and terminal
contacts into klayout-merged metal rectangles on `config.metal_layer` (and
pad markers on `pad_marker_layer`), using `rect_geometry`'s clipping helpers
and `terminal_contacts.terminal_access_path`.

`rect_geometry.py` (396 lines) owns the rectangle primitives both
realization and verification use: `wire_rects_for_points` (Manhattan wire
rectangles, with the `trim_start` end-cap option Milestone 1 added),
`clip_manhattan_path_at_first_bbox_entry` / `clip_manhattan_path_start_at_bbox`,
`disjoint_union_rects`, `union_rect_area`, `rect_intersects` /
`rect_intersection`. No net or realization logic.

`pitch_grid.py` (69 lines) owns the small grid-cell helpers used across the
package: `bbox_to_grid_cells`, `grid_cell_center_um` (the one copy Milestone
4 slice 1 folded four private copies into), `disk_cells`, `cells_bbox`.

`net_geometry.py` (595 lines) owns `build_net_geometries`: one
`NetGeometry` (rectangles tagged with their source, allowed cells and boxes,
centerlines) per net, shared by `verification.py`'s contracts and
`metrics.py`'s quality numbers so neither module re-derives geometry from
the route objects.

`verification.py` (409 lines), stage 11 (the contracts): `verify_electrical_routing`
and every issue-producing check -- terminal contacts, common-bus
connectivity and pad contact, cross-net overlap and spacing, raw-obstacle
overlap, blocked-cell clearance -- as `ElectricalVerificationIssue`s;
delegates the geometry to `net_geometry` and the numeric summary to
`metrics.quality_metrics`.

`metrics.py` (545 lines), stage 11 (the metrics): `quality_metrics` is the
reported numbers that are not pass/fail contracts -- `pad_wire_detour_total_um`,
`bus_length_um` / `bus_bend_count`, `wire_metal_area_um2`, cross-net
spacing, the per-wire `pad_wires` table -- plus `min_rect_spacing` and the
same-net overlap classification the module's docstring says are not the
verifier's business.

`debug.py` (78 lines), stage 12, the dispatcher: `write_debug_artifacts`
decides which SVGs to write and returns their paths, delegating the actual
export to `debug_grid_svg` and `metal_snapshot_svg` -- the contract
`route_electrical_heaters` used to build inline before Milestone 4 slice 4.

`debug_grid_svg.py` (441 lines) owns `export_electrical_debug_svg` /
`electrical_debug_svg`: the grid-coordinate debug view (obstacles, terminal
openings, the bus, the routes).

`metal_snapshot_svg.py` (365 lines) owns
`export_electrical_metal_snapshot_svg` / `electrical_metal_snapshot_svg`:
the physical-coordinate snapshot of the realized metal read back off the
routed layout.

`svg_support.py` (20 lines) owns `_all_port_accesses`, the one helper both
SVG exporters share.

### The `routing_flow*.py` modules

`routing_flow.py` (130 lines) is a shim over `photonic_router.cli`, so
`python routing_flow.py <benchmark> [flags]` keeps working for the reproduction
scripts and the benchmark stable blocks. The siblings are the flow's stage
helpers, still at the repository root and imported by `photonic_router.flow`:
`routing_flow_config.py` (269, the `SCRIPT_*` defaults and option resolution),
`routing_flow_optical.py` (176, the call into `route_match_and_realize`),
`routing_flow_verification.py` (391), `routing_flow_plm.py` (332),
`routing_flow_electrical.py` (200), `routing_flow_reporting.py` (551, console
output and timing formatting), `routing_flow_stats.py` (227, the legacy stats
collector) and `routing_flow_component_info.py` (24).

### The electrical flow: the twelve stage calls

`route_electrical_heaters` (`translation/electrical/route_electrical.py`) is
the literal sequence; the types are the Protocols of
`translation/electrical/stages.py`:

| # | call in `route_electrical.py::route_electrical_heaters` | input | product |
| --- | --- | --- | --- |
| 1 | `TerminalExtractor` -- `extract_heater_terminal_pairs(component, schematic, config)` | `(Component, Schematic \| None, ElectricalRoutingConfig \| None)` | `tuple[TerminalPairGroup, ...]` |
| 2 | `ObstacleMapBuilder` -- `build_electrical_obstacle_map(component, terminal_groups, config)` | `+ terminal_groups` | `ElectricalObstacleMap` |
| 3 | `CommonBusRouter` -- `route_common_bus(terminal_groups, obstacle_map, config)` | `+ obstacle_map` | `CommonBusRoutingResult` |
| 4 | `CommonBusTrimmer` -- `trim_common_bus_to_connections(obstacle_map, common_bus, config)` | `+ common_bus` | `(ElectricalObstacleMap, CommonBusRoutingResult)` |
| 5 | `EscapeTopologyPlanner` -- `compute_individual_escape_topology(obstacle_map, common_bus, config)` | `+ trimmed obstacle_map, common_bus` | `IndividualEscapeTopologyResult` |
| 6 | `PadPlanner` -- `plan_pad_slots(common_bus, obstacle_map, config, escape_topology)` | `+ escape_topology` | `PadPlan` |
| 7 | `PadSideReconciler` -- `reconcile_pad_sides(obstacle_map, common_bus, individual_topology, pad_plan, config)` | `+ individual_topology, pad_plan` | `PadSideReconciliation` (re-runs stages 4-6 when it swaps a heater) |
| 8 | `BusEscapeRouter` -- `route_common_bus_escape(obstacle_map, common_bus, pad_plan, config)` | `+ reconciled pad_plan` | `CommonBusEscapeResult` |
| 9 | `PadWireRouter` -- `route_detailed_bundles(obstacle_map, common_bus, common_bus_escape, topology, pad_plan, config)` | `+ common_bus_escape` | `DetailedBundleRoutingResult` |
| 10 | `MetalRealizer` -- `realize_electrical_metal(component, obstacle_map, common_bus, common_bus_escape, detailed_bundle_routes, pad_plan, config)` | `+ detailed_bundle_routes` | `Component` |
| 11 | `ElectricalVerifier` -- `verify_electrical_routing(obstacle_map, common_bus, common_bus_escape, detailed_bundle_routes, pad_plan, config)` | `+ routed Component` | `ElectricalVerificationResult` |
| 12 | `DebugArtifactWriter` -- `write_debug_artifacts(debug_dir, debug_prefix, *, obstacle_map=..., ...)` | `+ every object above` | `dict[str, str]` |

Guards: if stage 1 finds
no heaters, the pipeline returns right after stage 2 with an empty
`CommonBusRoutingResult` and a plain component copy -- stages 3-12 never run.
Stages 5 and 6 (`EscapeTopologyPlanner`, `PadPlanner`) run only when
`common_bus.success` (no `failed_heaters`). Stage 7
(`PadSideReconciler`) always runs, even with `individual_topology` or
`pad_plan` still `None`; it returns its four inputs unchanged when no heater
needs its roles swapped. Stage 8 (`BusEscapeRouter`) runs only once a pad
plan exists. Stage 9 (`PadWireRouter`) needs a pad plan, a topology and a
*successful* bus escape. Stage 10 (`MetalRealizer`) runs whenever the common
bus itself succeeded, independent of the escape or the pad wires. Stage 11
(`ElectricalVerifier`) runs only once stage 10 produced a component. Stage
12 (`DebugArtifactWriter`) always runs and accepts a `None` for every object
a skipped stage never produced.

The entry point is `run_electrical_routing_step`
(`routing_flow_electrical.py` at the repository root), called from `photonic_router.flow.route_schematic` after path-length
matching, guarded by `options.electrical.enable_electrical_routing`. That
flag, and the `ElectricalRoutingConfig` it passes down, are set by
`--electrical-routing true` on the command line or
`FlowOptions.electrical.enable_electrical_routing` when calling
`route_benchmark` directly; `benchmarks/heater_s_mod.py`'s stable block sets
`--include-heater-obstacles true --electrical-routing true`, so that
benchmark routes electrically by default. No paper configuration (section 7)
enables it.

### Checking the electrical result

`scripts/benchmark_electrical.py --suite --check --compare-baseline` runs
the four deterministic electrical cases --
`heater_single`, `heater_lanes_20`, `heater_lanes_ripup`, `heater_s_mod` --
and checks each one's `quality_metrics` (`translation/electrical/metrics.py`)
against `BENCHMARK_GUARDRAILS`. The guardrail idea (`--check`'s module
docstring, Decision Log of
`.agent/execplans/2026-09-25-electrical-routing-quality-and-cleanup.md`):
two zero-tolerance geometry contracts stay pinned at zero
(`same_net_redundant_overlap_pair_count`, `metal_redundant_area_overcount_um2`)
plus the unconditional cross-net clearance check
(`cross_net_min_spacing_um`), and four routing-quality quantities are
limited instead of the old pad-inventory sizes -- total and per-wire pad-wire
detour and bend count (`pad_wire_detour_total_um`,
`pad_wire_max_bend_count`), the common bus's own bend count
(`bus_bend_count`), and the union area of wire-only metal
(`wire_metal_area_um2`, bus stripe/branches/escape, individual routes,
adapters and contacts, deliberately excluding pad markers). `--compare-baseline`
reports every metric that drifted from the pinned
`tests/baselines/electrical_suite_metrics.json`; the pin is rewritten with
`--suite --output tests/baselines/electrical_suite_metrics.json` when a
milestone changes the numbers on purpose.


## 3. The flow: the literal sequence of stage calls

The outer flow is `photonic_router/flow.py`: `route_benchmark` loads the
benchmark module named in `options.loading` and calls `route_schematic`, whose
body is the stage outline -- layout from schematic, optionally the pre-placed
crossing grid stage (contribution 2), `run_photonic_routing_stage` (the routing
session below), `verify_and_attach_photonic_reports`,
`attach_and_report_path_length_matching`, optionally
`run_electrical_routing_step`, then `write_or_show_routed_layout`.

The routing session itself is `translation/routing/session.py::run`. The nine
calls below are that function; the types are the Protocols of
`translation/routing/stages.py`:

| # | call in `session.py::run` | input | product |
| --- | --- | --- | --- |
| 1 | `obstacle_context.build_static_obstacle_context(settings, state)` | `(SessionSettings, SessionState)` | `ObstacleContext` (obstacle map, crossing device info, obstacle SVG) |
| 2 | `router_setup.configure_router_and_grid(settings, state, context.obstacle_map)` | `+ obstacle_map` | `None` -- writes `state.router`, `primitive_cfg`, `astar_cfg`, the grid origin and the clearance radii |
| 3 | `route_jobs_stage.build_route_jobs_and_fanout_clustering(settings, state, settings.schematic.netlist.routes)` | `+ nets` | `RouteJobsResult` (route jobs, endpoint port specs by instance, dense runway lengths by spec) |
| 3b | `route_jobs_stage.apply_long_straight_fanout_exemption(settings, state)` | `(settings, state)` | `None` -- contribution 2's long-straight exemption |
| 4 | `crossing_plan_stage.build_crossing_plan_and_port_footprints(settings, state, jobs.route_jobs, jobs.endpoint_port_specs_by_instance, jobs.dense_port_runway_length_by_spec, context.obstacle_map, context.crossing_device_info)` | `+ jobs, port specs, runway lengths, obstacle map, device info` | `PlannedJobs` (jobs after planning, foreign keepouts by instance) |
| 5 | `handoff.finalize_route_jobs_and_static_handoff(settings, state, planned.route_jobs, planned.foreign_port_keepout_cells_by_instance, context.obstacle_map)` | `+ planned jobs, keepouts, obstacle map` | `FinalJobs` (jobs as handed to the kernel, `astar_start_s`) |
| 6 | `dispatch.dispatch_native_routing(settings, state, final.route_jobs)` | `+ final jobs` | `None` -- runs the kernel's negotiated loop; results land in `state.route_bookkeeping` with the loop's counters |
| 7 | `finalize.finalize_routing_results(settings, state, final.route_jobs, final.astar_start_s)` | `+ final jobs, t0` | `RoutedRecords` (one `RoutedNetRecord` per net, `astar_elapsed_s`) |
| 8 | `verify_repair.repair_and_verify_final_geometry(settings, state, routed.routed_net_records)` | `+ records` | `VerifiedRecords` (records after final repair, the crossings still illegal) |
| 9 | `realize.realize_and_assemble_debug_artifacts(settings, state, final.route_jobs, verified.routed_net_records, verified.illegal_realized_crossings, context.obstacle_map, context.obstacle_svg, routed.astar_elapsed_s)` | `+ records, illegal crossings, obstacle map, SVG, elapsed` | `RustRouteDebugArtifacts` |

`run` returns `(state.routed_layout, debug_artifacts)`. Phases 2 and 6 are the
two that return `None`: their whole effect is on `state` and on the kernel, and
`stages.py` says so in the Protocol docstring. Which fields they write is
documented per field in `state.py` (section 10).


## 4. The search interface, and how to add an engine

Every single-net search in the crate goes through one trait, declared in
`src/search/mod.rs`:

```rust
pub trait NetSearch {
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome;
}
```

* `SearchEnvironment { obstacle_map, primitives }` -- what the search runs
  against. It switches on nothing: every request against the same environment
  sees the same map and the same primitive set.
* `SearchRequest { source, target, port_open_cells, dynamic_expansion,
  crossing, config }` -- one net's question. The two optional members are the
  request *shape*: `DynamicExpansion { radius_cells, clearance_exempt_cells }`
  widens the clearance halo (used by repair to let a route brush past a net it
  is about to rip up) and, being present, forgoes the JPS4 and simple-route
  shortcuts; `CrossingSearch { config, reservation_open_cells }` asks for a
  route that crosses named partner nets, with `Some(_)` selecting the
  collision-crossing reservation anchor set and `None` reusing
  `port_open_cells` for it.
* `SearchOutcome { route, stats }` -- `route` is `None` on failure whatever the
  cause (infeasible, timed out, budget exhausted), and `stats` carries the real
  counters on every path.

Two implementations exist: `AStarSearch` (`src/search/astar/mod.rs`), the
production engine, whose `search` is a straight call into one `run_search`
body; and `GridDijkstraSearch` (`src/search/grid_dijkstra.rs`), the
uniform-cost oracle. The router holds one of them as `search_engine: Box<dyn
NetSearch + Send + Sync>` (`src/engine/mod.rs`), chosen once at construction by
`search_engine_for(&RouterConfig)` from `RouterConfig::search.engine`
(`src/config.rs`: `SEARCH_ENGINE_ASTAR = "astar"`,
`SEARCH_ENGINE_GRID_DIJKSTRA = "grid-dijkstra"`).

**To add an engine.** (1) Write it as its own module under `src/search/`,
implementing `NetSearch`; reuse the kernel's building blocks
(`build_search_grid`, `candidate_moves`, `move_is_legal`,
`push_or_improve_dense`, `goal_reached`) rather than re-deriving them, as
`grid_dijkstra.rs` does. (2) Decide what it does with a request shape it does
not support and report it in the stats instead of failing silently
(`unsupported_request`). (3) Add a name constant in `src/config.rs` and a branch
in `search_engine_for` (`src/engine/mod.rs`). (4) Accept the name in
`PyRouterConfig::new` (`src/bindings/types.rs`), which is what rejects unknown
values, and in `SearchOverrides.engine` (`python/photonic_router/config.py`),
whose overlay name is `PHOTONIC_ROUTER_SEARCH_ENGINE`
(`python/photonic_router/env_overlay.py`). (5) Test it against the interface,
not against A*'s internals: the Dijkstra engine's tests (cost equal to A* on
the plain fixtures, crossing request refused, the negotiated loop runs on it,
`benes_4x4_flat` routes end to end under contribution 2) are the pattern.


## 5. The negotiation loop, its policies, and how to add a policy

`run_negotiated_batch` in `src/engine/negotiation/loop_.rs` is the loop every
paper row runs. Read as a sentence: route the nets in the order given; when one
fails, ask the probe who is blocking it; try the two crossing-accepting
repairs; rip up what the rip-up policy names and search once more; requeue
whoever did not resolve, and iterate until every net is routed or `max_rounds`
is exhausted. Around it: history and congestion are cleared at entry, a braid
pass follows every committed route, a round with no progress twice in a row
resets the epoch (`RESET_AFTER_ROUNDS_WITHOUT_PROGRESS = 2`), and a probe that
finds no path at all aborts the batch naming the failed net.

The rules are values, one module each, chosen before the first round:

| policy | trait and implementations | what it decides |
| --- | --- | --- |
| Budget | `BudgetSchedule`, `LadderBudgets` (`negotiation/budget.rs`) | the expansion budget of one attempt, given its `AttemptKind` (`First`, `FirstRetry`, `PostRipUp`, `ProbeGuided`, `DirectCrossing`, `Braid`), the net's failure count and the round; `None` means unbounded |
| Crossing-free | `CrossingFreePolicy`, `PlannedPairsOnly`, `Never` (`negotiation/crossing_free.rs`) | whether this net's searches this round run crossing-free (contribution 1's rule for nets the plan gives no crossing; `Never` without guidance) |
| Rip-up | `RipUpPolicy`, `LidarStyleRipUp`, `NoRipUp` (`negotiation/ripup.rs`) | which committed nets a blocked net displaces -- and it does the ripping, because rip-up mutates the obstacle map and `batch.final_routes`; it returns the ids it ripped, in the order it found them. `NoRipUp` rips nothing: the binding's `no_ripup=True` with `max_rounds = 1` is the no-repair mode (`RipupRerouteConfig(enabled=False)`) |
| Queue | `NetQueue` (`negotiation/queue.rs`) | the work order plus the three tables the other rules read: failure counts, the nets already ripped this epoch, and each net's most recent probe blockers |

`negotiation/attempt.rs` turns one attempt into its `native_negotiated_search`
trace line; `negotiation/probe.rs` answers "who blocks this net";
`negotiation/braid.rs` swaps two crossing routes locally.

**To add a policy.** (1) Declare the decision as a trait in its own module
under `src/engine/negotiation/`, with today's behaviour as the first
implementation, and keep the trait's method signature to what the decision
needs. (2) Construct it once in `run_negotiated_batch`, before the round loop,
next to `budgets`, `crossing_free_policy` and `ripup_policy`; the loop body
must ask the policy, never re-derive the decision. (3) If a configuration field
selects it, add the field to `NegotiationConfig` on both sides (`src/config.rs`
and `python/photonic_router/config.py`) and an `ENV_OVERLAY` entry
(`python/photonic_router/env_overlay.py`); `docs/CONFIGURATION.md` then picks it
up on the next regeneration. (4) Test the policy directly (the trace text per
`AttemptKind` in `negotiation/attempt.rs`'s tests) and the loop through
`run_negotiated_batch`, which needs no Python.


## 6. Configuration objects and precedence

Five objects, each with one job:

| object | file | what it carries |
| --- | --- | --- |
| `RouterConfig` (Rust) | `src/config.rs` | the kernel's own parameters: `negotiation`, `search`, `crossing`, `diagnostics`. Plain data with today's defaults; no algorithm in `src/` reads the environment |
| `RouterConfig` (Python) | `python/photonic_router/config.py` | the frozen mirror of the above, field for field and default for default; `to_rust` builds the flat-keyword `rust_backend.RouterConfig` |
| `RoutingConfig` | `python/photonic_router/config.py` | the whole Python-side tree: `router` plus `crossing_plan`, `crossing_grid`, `fanout`, `engine`, `search`, `diagnostics` and `write_gds_on_photonic_verification_failure` |
| `FlowOptions` | `python/photonic_router/flow_options.py` | the flow-level choices in eleven groups, one per consuming stage (`loading`, `layout`, `obstacles`, `preplaced_grids`, `optical`, `verification`, `path_length`, `electrical`, `debug`, `output`, `stats`) |
| `SessionSettings` / `SessionState` | `translation/routing/settings.py`, `state.py` | one routing session's frozen inputs, and everything it computes |

Precedence, lowest to highest, as `photonic_router/config_loading.py::build_config`
and `photonic_router/cli.py::main` apply it:

1. **Defaults** -- the dataclass defaults in `config.py` and `flow_options.py`.
2. **The benchmark's stable block** -- `STABLE_ROUTING_ENV` (and its stable
   flags) from the benchmark module, read by `cli.py::_benchmark_stable_defaults`
   and passed to `build_config` as `stable_env`.
3. **The process environment** -- `PHOTONIC_ROUTER_*`, applied through the one
   overlay table `ENV_OVERLAY` in `python/photonic_router/env_overlay.py`. It
   wins over the stable block, exactly as `os.environ.setdefault` did before
   Milestone 1. A variable absent from the environment is never applied, so
   "unset" keeps the dataclass default rather than parsing an empty value.
4. **The command line** -- the parsed arguments, turned into `FlowOptions` by
   `cli.py::_flow_options`. `--configuration`'s flags are inserted *before* the
   user's own flags, so an explicit flag always wins (section 7).

`docs/CONFIGURATION.md` lists every field of `RoutingConfig` and `FlowOptions`
with its type, default, overlay variable and command-line flag; it is generated
by `scripts/generate_config_reference.py` and asserted by
`tests/test_config_reference.py`, so it cannot drift from the code.


## 7. The three paper configurations

Exactly one of the three runs at a time (repository rule; the flags are pinned
in `python/photonic_router/cli.py`'s `_CONFIGURATION_FLAGS`, which says in a
comment that changing one changes what the paper's numbers mean):

| `--configuration` | expands to | what it does |
| --- | --- | --- |
| `baseline` | `--crossing-mode lidar-pure` | crossings are discovered during search |
| `contribution1` | `--crossing-mode lidar-guided` | a precomputed crossing plan guides costs and order |
| `contribution2` | `--preplaced-crossing-grids true --verbose-routes` | crossing structures are placed before routing, the rest is routed crossing-free |

They are exactly the flags `scripts/results/reproduce_date2027.sh` passes for
each row. So

```bash
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration contribution2
```

is the paper's contribution 2 row of `benes_8x8_flat`. Overrides are ordinary
flags after it: `--configuration contribution1 --crossing-mode lidar-pure` is
contribution 1 with the crossing mode overridden
(`cli.py::configuration_flags`).


## 8. Build and test commands

From the repository root, with `.venv` present. `rust-toolchain.toml` pins a
Windows channel, so the Linux host overrides the toolchain, and `cargo test`
additionally needs `PYO3_PYTHON`:

```bash
# Build the kernel into python/photonic_router/_rust.abi3.so
RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python3" \
  .venv/bin/maturin develop --release

# Rust unit tests
RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" \
  cargo test --release --lib
cargo fmt            # before testing; cargo fmt --check in review

# Python tests: the whole suite, and the fast part (markers in pyproject.toml)
.venv/bin/python -m pytest -q tests
.venv/bin/python -m pytest -q -m "not e2e" tests

# The electrical pipeline's own tests, one module per stage of stages.py
.venv/bin/python -m pytest -q tests/electrical

# Both suites against the pinned baseline (tests/baselines/test_baseline.txt)
scripts/test_baseline.sh

# The electrical benchmark suite: four deterministic cases against their
# guardrails and the pinned baseline (docs/ARCHITECTURE.md section 2,
# "Checking the electrical result")
.venv/bin/python scripts/benchmark_electrical.py --suite --check --compare-baseline

# The reproduction gate: the nine smallest paper cells, about two minutes
scripts/results/gate_short.sh

# The full 27-cell reproduction (about 2.5 h; RESULTS_ROOT is never results/)
scripts/results/reproduce_date2027.sh
.venv/bin/python scripts/results/compare_date2027.py results_date2027 \
  <path>/EXPERIMENTS_TABLE_SOURCES.json
# Byte-compare every verification report between two reproduction roots
scripts/results/compare_verification_reports.sh <root_a> <root_b>

# The configuration reference
.venv/bin/python scripts/generate_config_reference.py            # regenerate
.venv/bin/python scripts/generate_config_reference.py --check    # verify
```

`scripts/test_baseline.sh` passes only when the Rust suite has no failures, the
Python failure list equals the pinned one (empty), neither pass count has
dropped below the pin, and the not-e2e run is under 60 s.
`scripts/results/gate_short.sh` prints nine `ok` rows and `all reproduced cells
match`; it compares crossing count and GDS length exactly and requires zero
verifier errors (`docs/date2027_table_sources.json`,
`scripts/results/compare_date2027.py`). `docs/DATE2027_REPRODUCTION.md` records
which commit last reproduced all 27 cells.
`scripts/results/compare_verification_reports.sh` pairs the newest run of each
cell in two roots and byte-compares their photonic and crossing verification
reports; the reports carry no timing, so identical bytes mean identical
verdicts and issue lists (the acceptance of the 2026-09-24 verifier plan, which
took the verifier's share of the full run from 5,207 s to 140 s).
`tests/electrical/` holds the electrical pipeline's own tests, one module per
stage of `translation/electrical/stages.py` (Milestone 4 slice 5 of
`.agent/execplans/2026-09-25-electrical-routing-quality-and-cleanup.md`),
with fixtures in `tests/fixtures/` after the pattern the optical side already
follows; it replaces the single `tests/test_electrical_routing.py`.
`scripts/benchmark_electrical.py --suite --check --compare-baseline` is the
electrical equivalent of `scripts/results/gate_short.sh`: it is not part of
`scripts/test_baseline.sh` and has no paper-cell dependency, since no paper
configuration routes electrically (section 7).


## 9. Removal candidates (Milestone 8)

The repository owner decided every candidate on 2026-09-24 (all removals as
recommended, under the constraint that the working benchmarks stay exact), and
Milestone 8 carries them out one slice at a time. The label was the string
`candidate for removal, see Milestone 8`; after Slice C
`grep -rn "candidate for removal" src/ translation/ python/` is empty, so what
is left below is the one item still named in the ExecPlan's Milestone 8 section.

Already removed, Slice A, the legacy engine (2026-09-24): the legacy repair
loop with its ~25-helper chain (the whole `src/engine/legacy_repair.rs`, 3,739
lines) and its binding, the clean-probe commit, the orthogonal repair fallback
(its config field `enable_orthogonal_repair_fallback` removed with it), the
non-repair batch binding (the negotiated loop with `max_rounds = 1` and
`NoRipUp` replaces it),
the negotiated-displacement trio and `ripup_repair_set_victims`, the
engine-selection fields and their two environment variables, and every helper
those left unreachable. Two decisions came with the slice: the two
mid-pipeline repair passes of `translation/routing/verify_repair.py` now call
the negotiated loop (D8), and the probe-guided and direct-crossing budgets
follow `NegotiationConfig` instead of the constants, whose values are those
fields' defaults (D9).

Already removed, Slice B, the window and collision crossing modes (D2,
2026-09-24): `expected_pairs_partner_set`, `crossing_allowed_partner_set` and
`try_route_through_expected_crossing_partner` with every branch keyed on
`allow_only_expected_pairs == true` or `use_collision_crossing_routing ==
false`, the field `use_collision_crossing_routing` and its pymethod
`set_collision_crossing_routing`, the two mode names (`CROSSING_MODES` is now
`("lidar-pure", "lidar-guided")`, `is_collision_mode` folded into
`is_lidar_mode`, the flow default became `lidar-pure`), the hard-constraint
branch of `_build_crossing_plan_info` and the window-only Weighted-A*
iteration cap. `CrossingConfig::allow_only_expected_pairs` stays: it is what
`CrossingContext::allows_pair` reads, and Python passes `False` for both
remaining modes.

Already removed, Slice C, the small leftovers (2026-09-24): the five Python
helpers no stage reached (`record_elapsed` in `translation/routing/timing.py`
and `_append_grid_step`, `_fanout_stub_centerline_um`,
`_append_circular_stub_bend`, `_cross2` in `translation/routing/route_jobs.py`),
the 22-line demo wrapper module of `python/photonic_router/`, the two
`RipupRerouteConfig` budgets the deleted repair chain owned with their two
command-line flags and the two unused `routing_flow_config.py` constants
behind them, the dead negotiated-engine environment field `run.txt` recorded
(`scripts/results/run_and_archive.sh`), and the unused crossing-mode keyword
of `_effective_crossing_search_loss`. Two things came
with the slice: D7's doc comment now describes the code (the epoch reset fires
once per batch, when the global rip-up counter reaches 2 -- the behaviour the
paper's runs used -- pinned by a new
`third_global_ripup_does_not_clear_history_again` test), and the Weighted-A*
iteration cap Slice B removed with the window mode is back in
`translation/routing/router_setup.py`, its condition folded to
`not settings.enable_crossings`.

Still open:

Nothing. The list is empty since Slice D (2026-09-24) deleted the four toy
benchmark modules (`TOY`, `mmi_heater`, `mmi_heater_8x4`,
`mmi_heater_8x4_ripup_reroute`) together with the unused `generate_toy.py`:
`scripts/benchmark_electrical.py` now runs its three cases on the real
heater-only benchmarks `heater_single`, `heater_lanes_20` and
`heater_lanes_ripup`, built from the synthetic fixtures of Milestone 6. The
electrical guardrail finding those cases exposed (seven guardrails red on every
case since the 2026-06 changes to `translation/electrical`, owner decision
pending) is item 3 of `.agent/REPOSITORY_STATE.md`'s Current Findings.

Each removal is one slice and one commit with the short gate after it, so any
single one can be reverted; the full 27-cell reproduction runs at the end.


## 10. Shared state between stages

`SessionState` (`translation/routing/state.py`) is one mutable record the nine
phases share, grouped by the phase that *first* writes each field, with the
writing (`w:`), mutating (`m:`) and reading (`r:`) phases named per field.
Nineteen fields are written by more than one phase and carry a `# shared
writer` marker. That is real coupling between phases, not an accident of the
old attribute soup, and it is the concrete list Milestone 8 negotiates; a
later plan may split the state phase by phase. The phase numbers are the nine
`run()` steps of section 3, with `0` for the session constructor.

| field | writer phases | also read by |
| --- | --- | --- |
| `route_nets_timings_s` | 0, mutated by 6 | 1, 6, 9 |
| `route_svgs` | 1, mutated by 6 | 6, 9 |
| `raw_static_cells_by_y` | 2, 4 (built on demand) | 4 |
| `raw_static_rect_ranges_by_y` | 2, 4 (built on demand) | 4 |
| `heater_opening_rect_ranges_by_y` | 2, 4 (built on demand) | 4 |
| `static_blocked_cells_before_port_reservations` | 2, 5 (mutated by 5) | 5, 6 |
| `port_access_cells_by_spec` | 3, mutated by 4 | 4, 5 |
| `port_access_candidate_cells_by_spec` | 3, mutated by 4 | 4, 5 |
| `port_access_rule_by_spec` | 3, mutated by 4 | 4, 6 |
| `port_runway_cells_by_spec` | 3, mutated by 4 | 4, 5, 6 |
| `crossing_plan_info` | 4, mutated by 4, 7, 8, 9 | 4, 5, 7, 8, 9 |
| `route_bookkeeping` | 5 (its phase-7 writes go through its own attributes) | 6, 7, 8 |
| `route_timing_buckets` | 5, mutated by 6 | 6, 7, 9 |
| `route_attempt_records` | 5, mutated by 6 | 6, 9 |
| `deferred_count` | 5, 6 | 7, 9 |
| `repair_count` | 5, 6 | 7, 9 |
| `simple_route_count` | 5, 6 | 7, 9 |
| `total_expanded_states` | 5, 6 | 7 |
| `native_repair_trace_records` | 5, 6 | 6 |

Read as a shape: the crossing plan is written by four phases (4, 7, 8, 9); the
port-access tables by 3 and 4; the static row indexes by 2 and 4; the search
counters, the timing buckets, the attempt records and the trace records by 5 and
6; the route bookkeeping and the SVG registry span the dispatch boundary. Every
other field of `SessionState` has exactly one writing phase, and `state.py`
names it.
