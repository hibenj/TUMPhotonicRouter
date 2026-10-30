# Architecture

This document describes the router as the tree is now. It was rewritten from
scratch in phase 8 slice D of the 2026-10 do-over; the previous version, with
its milestone narrative and the removal lists, stays reachable in git history
(`git show cdecf9a:docs/ARCHITECTURE.md`). What changed when, and why, is in
[`CHANGELOG.md`](../CHANGELOG.md); the state that produced the paper's numbers
is the tag `date2027-results` (section 11).

Line counts are `wc -l` at the commit that introduced this document, rounded;
they are orders of magnitude, not contracts. Rust paths without a crate
prefix (`src/...`) are relative to `crates/router-core/`.

Contents:

1. [What the router does](#1-what-the-router-does)
2. [The pipeline, stage by stage](#2-the-pipeline-stage-by-stage)
3. [The data model](#3-the-data-model)
4. [The search interface, and how to add an engine](#4-the-search-interface-and-how-to-add-an-engine)
5. [The negotiation loop, its policies, and how to add a policy](#5-the-negotiation-loop-its-policies-and-how-to-add-a-policy)
6. [The configuration surface](#6-the-configuration-surface)
7. [Code layout: the Rust crates and the Python package](#7-code-layout-the-rust-crates-and-the-python-package)
8. [Build, tests and oracles; how to add a benchmark](#8-build-tests-and-oracles-how-to-add-a-benchmark)
9. [Switches for experiments](#9-switches-for-experiments)
10. [Invariants](#10-invariants)
11. [History and provenance](#11-history-and-provenance)


## 1. What the router does

Input: a gdsfactory `Schematic` (instances, their placements, and the nets
between their ports), built by a benchmark module's `build_schematic()`.
Output: a routed gdsfactory `Component`, written as
`build/routed_<benchmark>.gds`, in which

* every net is one waveguide polygon on the routing layer, built from
  straights and circular bends of one bend radius;
* two waveguides overlap only at a crossing, which is a placed crossing
  component, perpendicular, with straight run-outs on both sides;
* nothing overlaps a component or another waveguide's clearance;
* optionally, groups of paths have equal optical length (meanders inserted),
  and heater terminals are wired to pads on a metal layer.

The work is split by cost. Rust (`crates/router-core`) owns the grid
database, the single-net search, the multi-net negotiation, crossing
legality, endpoint correction geometry, polygon realization and the meander
planner: everything that runs per cell or per state. Python
(`python/photonic_router/`) owns gdsfactory, the benchmarks, the stage
sequence, the crossing plan, the path-length analysis, verification against
the realized polygons, the electrical router and every report and debug
artifact. The two meet in the PyO3 extension `photonic_router._rust`, built
from `crates/router-py`, whose only job is to convert at the boundary.

Both sides are organised the same way: one declared interface per
replaceable concern (a Rust `trait`, a Python `typing.Protocol`), one module
per concern, and a top-level function that reads as the sequence of stage
calls (`photonic_router.flow.route_schematic`,
`photonic_router.routing.session.run`, `Router::run_negotiated_batch`,
`photonic_router.electrical.route_electrical.route_electrical_heaters`).


## 2. The pipeline, stage by stage

```text
benchmark module ──build_schematic()──> Schematic
  └─ layout_from_schematic ─────────────> unrouted Component
       ├─ [contribution 2] crossing demand -> pre-placed crossing tiles
       └─ routing session (routing/session.py::run, nine phases)
            1 static obstacle map (Python extraction, Rust rasterization)
            2 router setup (PyPhotonicRouter, primitive library, A* config)
            3 route jobs (ports -> grid states, port access, fan-out runways)
            4 crossing plan (placement demand -> events, port footprints)
            5 static handoff (final jobs, net order)
            6 dispatch -> Rust: negotiated loop -> search calls -> A* kernel
            7 finalize: route records, endpoint correction
           7b [optional] detour pass
            8 final repair and verification of the realized crossings
            9 realization, crossing components, debug artifacts
       ├─ path-length matching (analysis, Rust meander planner, re-realization)
       ├─ photonic verification (realized polygons) -> build/verification/
       ├─ [optional] heater electrical routing (twelve stages)
       └─ GDS write, SUMMARY line
```

### 2.1 Entry points

`python -m photonic_router route <benchmark> [flags]` (`__main__.py`) and
`python routing_flow.py <benchmark> [flags]` (a five-line shim at the
repository root, which the reproduction scripts call) both run
`photonic_router.cli.main`. It reads the benchmark's defaults from
`benchmarks/<name>.toml`, builds a `RoutingConfig` (`config_loading.build_config`)
and a `FlowOptions` (`cli._flow_options`), and calls
`photonic_router.flow.route_benchmark(config, options)`, which loads
`benchmarks.<name>` and calls `route_schematic(schematic, config, options)`.
Nothing below `cli.py` sees a command line; nothing below `config_loading.py`
reads the environment.

### 2.2 Layout from schematic

`records/layout_from_schematic.py` turns the `Schematic` into an unrouted
`Component`: every instance placed, no route. Its bounding box sizes the
grid.

### 2.3 Pre-placed crossing grids (contribution 2 only)

With `--preplaced-crossing-grids true`, `flow._preplaced_crossing_grids_stage`
builds the placement's `CrossingDemand` (`crossings/crossing_demand.py`): the
chip is cut into vertical *bands* between *columns* of components (a column
is a maximal set of instances whose placed x-extents overlap or touch); a net
whose two terminals point into one band from its two walls is *confined* to
it; two confined nets must cross iff their top-to-bottom order differs
between the two walls. `CrossingDemand.require_band_layered()` refuses a
design with nets outside that model (same wall, non-adjacent walls, a
vertical port, a port pointing off the chip); the flow re-raises that as a
`RuntimeError` naming the nets and suggesting `--crossing-mode lidar-pure` or
`lidar-guided`. `stage_schedule()` lists the bands with crossing events, left
to right; `crossings/preplaced_crossing_grids.py`
(`derive_preplaced_crossing_layout`) places a tile of crossing components
per band from it, and the session then routes the stubs between ports and
tiles crossing-free. No paper benchmark has a net outside the band model.

### 2.4 Phase 1: the static obstacle map

`routing/obstacle_context.py` calls `static_obstacle_builder.build_static_obstacle_map`:
`benchmark_extractor.py` extracts the component polygons and ports from the
layout (layer selection in `routing_layers.py`), and the Rust builder
(`src/static_obstacle_builder.rs`, through `build_static_obstacle_map_rs` in
`crates/router-py/src/static_obstacle.rs`) does the grid work: grid spec,
bounding boxes (`bounding_boxes`, the default) or rasterized polygons
(`rasterized_polygons`), clearance inflation, port openings, and a
`StaticCellSet` handle that stays in Rust. A chip-boundary keep-out band is
added outside the routable box. The product is an `ObstacleContext`
(obstacle map data, crossing device info, the obstacle debug SVG path).

### 2.5 Phase 2: router setup

`routing/router_setup.py` builds the `PyPhotonicRouter` (`GridSpec`,
`PrimitiveLibraryConfig`, `AStarConfig`, the flat `RouterConfig` from
`RoutingConfig.router.to_rust`), installs the static cells and rectangles,
and records the grid origin and the clearance radii on the session state.
The bend radius is rounded up to whole grid cells.

### 2.6 Phase 3: route jobs

`routing/route_jobs.py` turns every net into a `RouteJob`: source and target
grid `State`s (cell plus heading, from the port position and orientation),
the cells opened for each port's access, clearance-exempt cells, and for
dense multi-port instances the fan-out access (`--fanout-access-mode`:
runways, static stubs, virtual anchor ports). Phase 3b
(`apply_long_straight_fanout_exemption`) marks the nets that contribution 2
searches without the long-straight congestion penalty.

### 2.7 Phase 4: the crossing plan

`routing/crossing_plan_stage.py` builds the crossing plan when crossings are
enabled: the same `CrossingDemand` as in 2.3, and per band the bubble-sort
sequencing of the two wall orders (`crossings/crossing_plan.py`,
`build_crossing_plan` over `LaneRank`s) into ordered `CrossingEvent`s. Under
`--crossing-mode lidar-guided` (contribution 1) the planned pairs guide the
search (costs, the crossing-free policy, net order); under `lidar-pure`
(baseline) crossings are discovered during search and the plan is reported
only. The stage also computes the port footprints and the foreign-port
keep-outs. Its typed record is `CrossingPlanInfo`
(`routing/crossing_plan_info.py`), which later phases extend with the
realized crossings. `crossings/topology_analysis.py` (graph depth and port
rank) is the older plan, kept as the test oracle
(`tests/e2e/test_crossing_demand_equality.py` holds both equal on the 17
paper benchmarks); the flow does not read it.

### 2.8 Phase 5: the static handoff

`routing/handoff.py` produces the jobs exactly as the kernel will see them
(snapped endpoints, foreign keep-outs) in their routing order
(`records/route_order.py`, `--net-order`; topological layers by default,
`topological-span` under contribution 2).

### 2.9 Phase 6: dispatch to the kernel

`routing/dispatch.py` hands the jobs to
`PyPhotonicRouter.route_many_with_negotiated_repair_and_commit`. The binding
converts the `RouteJob` objects to `NativeRouteJob`s once
(`convert.rs::native_jobs_from_py`) and calls
`Router::run_negotiated_batch` (`src/engine/negotiation/loop_.rs`, section
5). Every search the loop makes is a `SearchCall` value
(`src/engine/search_call.rs`: the job, the radii, `Plain` or `Repair`, the
budget, the history weight, the long-straight override, the crossing-free
flag, the probe-guided partners). One body,
`route_and_commit` (`src/engine/search_calls.rs`), turns a call into a
`SearchRequest`: first the simple-route candidates
(`src/search/astar/simple_route.rs`: straight, L, Z, turnaround, checked cell
by cell), then the engine's `NetSearch::search` (section 4), with a
`LiveCrossingHook` when the request carries a `CrossingSearch` (crossings
enabled) and `NoCrossingHook` otherwise. A found route is committed to
the obstacle map (`src/engine/commit.rs`: footprint, clearance halo,
history, congestion, crossing events and reservations in
`src/engine/crossing_reservation.rs`). The loop's result comes back as a
dictionary; the per-attempt records and trace lines land in the session
state (`routing/diagnostics.py` writes them up under the debug options).

### 2.10 Phase 7: route records and endpoint correction

`routing/finalize.py` builds one `RoutedNetRecord` per net from the kernel's
results and the native crossing events. A grid route starts and ends on cell
centres; endpoint correction splices its centerline onto the exact physical
port (`verify/route_rust_endpoint_correction.py` for the crossing-aware
splice, the Rust pass `src/engine/endpoint_correction.rs` for the checked
grid-to-port correction and commit). `EndpointCorrectionCategory`
(`records/route_rust_types.py`) names which pass treats which net (fan-out
stubs, crossing nets, plain nets).

Phase 7b, only with `--detour-pass-factor F > 0` (`routing/detour_pass.py`):
every net whose recorded length exceeds `F` times its free lower bound
(Manhattan distance of its port centres plus two bends) is ripped up and the
set is routed again in one negotiated batch, shortest first, up to three
passes while the set shrinks (SUMMARY `detours=`).

### 2.11 Phase 8: final repair and verification of the crossings

`routing/verify_repair.py` checks the realized crossings of the records
(`crossings/route_rust_crossing_verification.py`). Illegal ones go to two
repair passes (final crossing repair, issue-group repair), which call the
same negotiated loop; no paper cell reaches them. What stays illegal is
handed to phase 9 and reported.

### 2.12 Phase 9: realization

`routing/realize.py` turns every record into polygons
(`records/route_rust_realization.py` over the Rust realization in
`src/geometry_realization.rs`: centerline to one closed waveguide polygon),
places a PDK crossing component at every realized crossing
(`crossings/route_rust_crossing_components.py`) and assembles the
`RustRouteDebugArtifacts` (route SVGs, failure diagnostics, crossing JSON
and text reports under `build/`). `session.run` returns the routed layout
and the debug artifacts.

### 2.13 Path-length matching (PLM)

`routing/api.py::route_match_and_realize` (the boundary the flow's optical
stage, `flow_optical.py`, calls) continues after the session when
`--path-length-matching true`:

1. `plm/path_length_graph.py` builds the directed graph of the schematic,
   oriented along the light from the benchmark's `input` nodes
   (`port_roles.py` names each generic component's input-side ports; a
   benchmark extends it with `INPUT_PORTS_BY_COMPONENT`), and turns the
   routed lengths into arrivals and the missing length per edge.
2. `plm/path_length_requirements.py` computes the requirements (which edge
   needs how much, per matching group and, with `--path-length-match-outputs
   true`, at the outputs) and the delay-insertion candidates upstream of a
   requiring edge, found by structure.
3. `plm/route_rust_meanders.py` turns that into request vectors for one
   `MeanderPlanner` (`crates/router-py/src/meander_py.rs`): `register` /
   `register_geometries` once per pass, `plan_requests` per requirement,
   `plan_final` once, `reserve_rect` per committed plan. Rust
   (`src/meander/plan.rs`) decides every plan: which candidate bundle, the
   per-edge direct plan, or the same-edge split; `src/meander/geometry.rs` is
   the analytic comb, `src/meander/search.rs` the obstacle-aware placement.
4. The meanders are spliced into the records and realized; the report is a
   `MeanderInsertionReport` of `MeanderEdgeReport`s, stored in the GDS meta
   info as `meander_insertion_report`. `plm/path_length_diagnostics.py` and
   `flow_plm.py` check acceptance (a 1 nm tolerance on a group's realized
   spread); `--path-length-allow-residual true` finishes with residuals
   reported (`plm_over_tolerance` in the SUMMARY) instead of failing.

The timers and per-candidate attempts (`MeanderPlanningProfile`) exist only
under `--debug-meanders`; no SUMMARY token or acceptance decision depends on
that flag.

### 2.14 Photonic verification

`flow_verification.py` runs the verifiers on the realized geometry:
`verify/photonic_verification.py` (on the realized polygons, with klayout
and shapely: cross-net waveguide overlaps, waveguide-obstacle overlaps,
crossing-component overlaps, routes outside the chip, self-intersections,
missing, extra or duplicate route records, endpoint-correction errors) and
the crossing report (`crossings/crossing_verification_report.py`). The JSON
reports go to `build/verification/<benchmark>_{photonic,crossing}_verification.json`;
their error count is the SUMMARY's `verifier_errors`. A failing verification
does not write the GDS unless
`PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE` is set.

### 2.15 Heater electrical routing: the heater-metal pipeline as stages

With `--electrical-routing true` (on by default for `heater_s_mod`),
`flow_electrical.py::run_electrical_routing_step` calls
`electrical/route_electrical.py::route_electrical_heaters`, a literal
sequence of twelve stages, each typed by a Protocol in `electrical/stages.py`
and returning a frozen product from `electrical/types.py`:

| # | stage | module | product |
| --- | --- | --- | --- |
| 1 | terminal extraction | `terminal_extraction.py` | heater terminal pairs |
| 2 | electrical obstacle map | `obstacle_extraction.py`, `port_access.py` | `ElectricalObstacleMap` |
| 3 | common bus | `common_bus_router.py`, `column_trunks.py`, `bus_search.py` | `CommonBusRoutingResult` |
| 4 | bus trim | `common_bus_router.py` | trimmed bus |
| 5 | escape topology | `individual_topology.py` | `IndividualEscapeTopologyResult` |
| 6 | pad slots | `pad_slots.py` | `PadPlan` (6b trims the bus again to the pad plan) |
| 7 | pad-side reconciliation | `pad_side_reconciliation.py` | `PadSideReconciliation` |
| 8 | bus escape | `escape_router.py` | `CommonBusEscapeResult` |
| 9 | pad wires | `bundle_detail_router.py`, `river_routing.py`, `pad_wire_search.py` | `DetailedBundleRoutingResult` |
| 10 | metal realization | `metal_realization.py`, `rect_geometry.py` | routed `Component` |
| 11 | verification and metrics | `verification.py`, `net_geometry.py`, `metrics.py` | `ElectricalVerificationResult` |
| 12 | debug artifacts | `debug.py`, `debug_grid_svg.py`, `metal_snapshot_svg.py` | SVG paths |

A stage whose inputs are not ready is skipped (no heaters: return after
stage 2; no successful bus: no pad planning; no pad plan: no escape or pad
wires); stage 12 always runs. The electrical router shares the grid
vocabulary with the optical side but not its kernel; the optical stages do
not import `photonic_router.electrical`. Its parameters are
`ElectricalRoutingConfig` (seven `--electrical-*` flags reach it).

### 2.16 GDS and the SUMMARY line

`flow_reporting.write_or_show_routed_layout` writes
`build/routed_<benchmark>.gds` (or opens KLayout with `--show-klayout`), and
every run, a failed one too, ends with one line from
`flow_stats.format_flow_summary_line`:

```text
SUMMARY benchmark=... nets=... route_failures=... detours=... verifier_errors=...
verifier_warnings=... plm_groups=... plm_over_tolerance=... meander_inserted_um=...
electrical_heaters=... electrical_pads=... electrical_failed_wires=... electrical_ok=... total_s=...
```

(one line in the output). A stage that did not run prints `-`, never a
guessed zero.


## 3. The data model

**Grid and cells.** One single-layer grid per run: `GridSpec { width,
height, grid_size_um, origin_x_um, origin_y_um }` (`src/config.rs`; 2 um per
cell by default, `--grid-size-um`). A cell is `(x, y)` with `0 <= x < width`;
sets of cells use the packed key `CellKey = u64`, `pack_xy(x, y) = (x as u32)
<< 32 | (y as u32)` (`src/obstacle_map.rs`), reversible by `unpack_xy`.

**Headings and states.** Eight headings in 45-degree steps, `angle` 0 = east,
counter-clockwise (`DIRECTIONS` in `src/primitives.rs`: 1 = north-east, 2 =
north, ..., 7 = south-east). A search state is `State { x, y, angle }`
(`src/search/state.rs`), angle taken modulo 8.

**Primitives.** `create_photonic_primitive_library` builds, per start
heading, a short and a long straight, the two 90-degree bends and, when
`allow_45_degree_turns` (the CLI default; `--allow-45-degree-turns false`
turns it off), the two 45-degree bends. Every `Primitive` carries its id
(`u16`), start and end heading, grid displacement, the conservative
`footprint` cells the search checks, its length, its bend cost and a
`PrimitiveGeometry` (`Straight { length_um }` or `Bend { radius_um,
angle_delta }`) that realization draws. All bends share the one bend radius
in cells. `python/photonic_router/primitive_library.py` maps the ids to
gdsfactory components.

**The obstacle map** (`ObstacleMap`, `src/obstacle_map.rs`) is the routing
database: one dense record per in-bounds cell, stored column-wise as planes
-- `flags: u8` (static, static-rectangle, dynamic, dynamic-core bits, always
allocated), static and dynamic reference counts, the dynamic `owner`
(`NetId`), `history_cost` and `congestion_cost` (allocated on first write,
shared copy-on-write between clones) -- plus the static rectangles as given,
an overflow map of second and later owners, and per-net cell lists
(`net_routes`, `net_core_routes`). "Is this cell free" is the flags byte
alone. Static obstacles and committed nets are separate, so a net can be
ripped up and rerouted without rebuilding the layout geometry.

**Route jobs.** Python's `RouteJob` (`records/route_rust_types.py`) is the
per-net input; the binding's `#[pyclass] RouteJob`
(`crates/router-py/src/types.rs`, keyword-only, read-only) carries the net
id, source and target `State`, the opened, clearance-exempt and foreign
keep-out cells and the two port centres in um, and becomes the engine's
`NativeRouteJob` (`src/engine/jobs.rs`).

**Route results.** `RouteResult` (`src/search/state.rs`): the `states`, the
`primitives` (ids), the grid `cells`, the `compressed_waypoints`, total
length and cost, the requested and reached target, and `RouteSearchStats`
(window attempts, full-grid fallback, expanded states, heap traffic,
crossing candidate and reject counters, timings). On the Python side one
`RoutedNetRecord` per net holds the route, its centerline in um, the
correction and crossing metadata that realization and verification read.

**Crossing events and verdicts.** A committed crossing is a `CrossingEvent`
(`src/engine/crossing_reservation.rs`: the two net ids, the point, the two
grid segments and headings, the reserved cells). Under the unified legality
switch (section 9) every realized intersection also gets a
`CrossingVerdictRecord` with the `CrossingCandidate` (the two directions,
the four straight runs, the point, the nearest other crossings), the
`CrossingLegalityRule` (`half_size_cells`, `bend_runout_cells`) and the
`CrossingVerdict`: `Legal`, `NotPerpendicular`, `MarginShort { side }`,
`TooClose { other }`, `FootprintBlocked` or `MissingEvent`, spelled in
reports as `legal`, `not_perpendicular`, `margin_short_{route_before,
route_after,partner}`, `too_close_{same_route,registered}`,
`footprint_blocked`, `missing_event`. `PyPhotonicRouter.crossing_verdicts()`
reports them. The planning side's `CrossingEvent` (`crossings/crossing_plan.py`)
is a different record: a planned pair of edges, its band (`source_depth`,
`target_depth`: the two wall-column indices), level and order.

**Meander plans.** A requirement's request vector (candidate bundles in
order, one requested extra length per edge, the requiring edge as the split
fallback) goes to `MeanderPlanner.plan_requests`; the answer names which
candidate planned, or the split, or nothing, with the analytic comb
geometry per edge (`src/meander/geometry.rs`: boxes, U-turn count, exact
inserted length and footprint). The report side is `MeanderEdgeReport`
(edge, requested, credited and physical inserted length, planned edges,
status, reason) inside a `MeanderInsertionReport`.

**Session state.** `SessionSettings` (`routing/settings.py`) holds the
frozen inputs of one routing session; `SessionState` (`routing/state.py`)
everything it computes, one documented field per value with its writing,
mutating and reading phases (`w:`, `m:`, `r:`). Eighteen fields are written
by more than one phase and carry a `# shared writer` marker (the crossing
plan, the port-access tables, the static row indexes, the search counters
and timing buckets, the route bookkeeping); every other field has one
writer.


## 4. The search interface, and how to add an engine

Every single-net search goes through one trait (`src/search/mod.rs`):

```rust
pub trait NetSearch {
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome;
}
```

* `SearchEnvironment { obstacle_map, primitives }`: what the search runs
  against. Every request against the same environment sees the same map
  and primitive set.
* `SearchRequest { source, target, port_open_cells, dynamic_expansion,
  crossing, config }`: one net's question. The optional
  `DynamicExpansion { radius_cells, clearance_exempt_cells }` widens the
  clearance halo (repair uses it to let a route brush past a net it is
  about to rip up); the optional `CrossingSearch { config,
  reservation_open_cells }` asks for a route that may cross partner nets.
* `SearchOutcome { route, stats }`: `route` is `None` on any failure
  (infeasible, timed out, budget exhausted); `stats` carries the real
  counters on every path.

Two implementations exist. `AStarSearch` (`src/search/astar/`) is the
production engine: `search` calls `run_search`, which sets up the routing
window driver (`window.rs`: a source-target window, progressively larger,
then the full grid) around the kernel loop `kernel::run`, generic over a
`CrossingLegalityHook` (`NoCrossingHook` or `LiveCrossingHook`,
`crossing_rules.rs`): seed, pop, goal test, candidate moves
(`expansion.rs`), legality (`move_is_legal` returning a `MoveVerdict`),
cost (`cost.rs`: length, bend, history, long-straight and proactive
congestion terms), heuristic (`heuristic.rs`: plain, diagonal and
heading-aware estimates), push-or-improve on dense per-search storage
(`dense.rs`). `GridDijkstraSearch` (`src/search/grid_dijkstra.rs`) is the
uniform-cost oracle: f = g, full grid, no window, no crossing hook, built
from the kernel's own building blocks; it refuses a crossing request with
the `unsupported_request` stat.

The router holds one engine as `search_engine: Box<dyn NetSearch + Send +
Sync>` (`src/engine/mod.rs`), chosen once at construction by
`search_engine_for(&RouterConfig)` from `RouterConfig::search.engine`
(`SEARCH_ENGINE_ASTAR = "astar"`, `SEARCH_ENGINE_GRID_DIJKSTRA =
"grid-dijkstra"`; overlay variable `PHOTONIC_ROUTER_SEARCH_ENGINE`).

**To add an engine.** (1) Write it as a module under `src/search/`
implementing `NetSearch`; reuse the kernel's building blocks (grid setup,
`candidate_moves`, `move_is_legal`, push-or-improve, goal test) as
`grid_dijkstra.rs` does. (2) Decide what it does with a request shape it does
not support and report it in the stats (`unsupported_request`) instead of
failing silently. (3) Add a name constant in `src/config.rs` and a branch in
`search_engine_for`. (4) Accept the name in `PyRouterConfig::new`
(`crates/router-py/src/types.rs`), which rejects unknown values, and in
`SearchOverrides.engine` (`python/photonic_router/config.py`). (5) Test it
against the interface, not against A*'s internals: the Dijkstra tests (cost
equal to A* on the plain fixtures, crossing request refused, the negotiated
loop runs on it, `tests/e2e/test_search_engine.py` routes a benchmark end to
end) are the pattern.


## 5. The negotiation loop, its policies, and how to add a policy

`Router::run_negotiated_batch(jobs, params)` (`src/engine/negotiation/loop_.rs`)
is the loop every paper row runs, and the repository's only batch routing
entry point. Read as a sentence: route the nets in the order given; when one
fails, ask the probe who is blocking it; rip up what the local rip-up rule
names and search once more (guided by the probe's legal crossing partners
when it reported both legal and illegal ones); requeue whoever did not
resolve, rip up at round end what the probes blamed for the nets still
failing, and iterate until every net routes or `max_rounds` is exhausted.

The code has the same shape. `run_negotiated_batch` is the rounds;
`route_net_this_round` is one net's turn and returns `Routed`, `Failed` or
`ProbeAborted`; `global_ripup_round` is the round end (LiDAR's
`ripupfailedNets`). Around it:

* the batch starts by clearing history and congestion
  (`begin_negotiated_batch`);
* a net that has not failed yet gets one retry at the larger first-retry
  budget before the probe runs, except in the last round;
* the probe is diagnostic only; a probe that finds no path at all aborts the
  batch naming the failed net (the loop's only early exit);
* a braid pass (`braid.rs`, the local swap of two crossing routes) follows
  every committed route;
* two epoch resets, both as in the paper's runs: the global rip-up clears
  the history map and the once-per-epoch set exactly once per batch, on its
  second call; and after `RESET_AFTER_ROUNDS_WITHOUT_PROGRESS = 2` rounds
  without a committed route the loop clears the history and the failure
  counts;
* a batch that does not converge is returned with whatever routed and a
  `failed_net_id` / `failed_error` (the soft failure).

The rules are values, one module each, chosen once in `batch_context`
before the first round:

| policy | types (`src/engine/negotiation/`) | what it decides |
| --- | --- | --- |
| Budget | `BudgetSchedule`, `LadderBudgets` (`budget.rs`) | the expansion budget of one attempt from its `AttemptKind` (`First`, `FirstRetry`, `PostRipUp`), the net's failure count and the round (`NegotiationConfig.budget_first`, `budget_first_retry`, `budget_retry`); the braid passes use the constant `NEGOTIATED_BUDGET_BRAID` (`src/engine/jobs.rs`) |
| Crossing-free | `CrossingFreePolicy`, `PlannedPairsOnly`, `Never` (`crossing_free.rs`) | whether this net's searches run crossing-free this round (contribution 1: nets the plan gives no crossing; `Never` without guidance) |
| Rip-up | `ripup_illegal_crossing_partners` (`ripup.rs`), behind `if no_ripup` in the loop | which committed nets a blocked net displaces; it does the ripping and returns the ids. `no_ripup` with `max_rounds = 1` is the no-repair mode (`RipupRerouteConfig(enabled=False)`) |
| Queue | `NetQueue` (`queue.rs`) | the work order plus the tables the other rules read: failure counts, the nets already ripped this epoch, each net's most recent probe blockers |

`attempt.rs` builds each attempt's `SearchCall` from the batch's
`SearchArgs` (radii and flags, plus the two weights chosen per net: the
history weight once a net has failed, the long-straight override for the
exempt fan-out nets) and writes its `native_negotiated_search` trace line;
`probe.rs` answers "who blocks this net"; `trace.rs` holds the other
`PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` lines.

The loop accepts any legal path, however long, and never revisits a routed
net unless a later net rips it up. Two opt-in measures outside the loop
address the detours that leaves: `--net-order span` (nearest ports first
across the depth layers) and the detour pass (2.10). Both leave the defaults
unchanged.

**To add a policy.** (1) Declare the decision as a trait in its own module
under `src/engine/negotiation/`, with today's behaviour as the first
implementation and a method signature limited to what the decision needs.
(2) Construct it once in `batch_context`; the loop body asks the policy and
never re-derives the decision. A decision with two outcomes and no state is
an `if` on a `NegotiationParams` field, not a trait (the rip-up switch is the
example). (3) If a configuration field selects it, add the field to
`NegotiationConfig` on both sides (`src/config.rs`,
`python/photonic_router/config.py`), to `RouterConfig.to_rust` and
`PyRouterConfig`, and an `ENV_OVERLAY` entry
(`python/photonic_router/env_overlay.py`); regenerate
`docs/CONFIGURATION.md` and the stub (section 8). (4) Test the policy
directly and the loop through `run_negotiated_batch`, which needs no Python
(`cargo test -p router-core`).


## 6. The configuration surface

| object | file | what it carries |
| --- | --- | --- |
| `RouterConfig` (Rust) | `src/config.rs` | the kernel's parameters in four groups: `negotiation` (`NegotiationConfig`), `search` (`SearchOverrides`), `crossing` (`CrossingEngineConfig`), `diagnostics` (`KernelDiagnostics`); plain data, no algorithm reads the environment |
| `RouterConfig` (Python) | `python/photonic_router/config.py` | the frozen mirror, field for field and default for default; `to_rust` builds the binding's flat-keyword `RouterConfig` |
| `RoutingConfig` | `python/photonic_router/config.py` | the Python-side tree: `router`, `crossing_plan`, `crossing_grid`, `fanout`, `search` (`SearchTuning`), `diagnostics` (`FlowDiagnostics`) and `write_gds_on_photonic_verification_failure` |
| `FlowOptions` | `python/photonic_router/flow_options.py` | the flow-level choices in eleven frozen groups, one per consuming stage: `loading`, `layout`, `obstacles`, `preplaced_grids`, `optical`, `verification`, `path_length`, `electrical`, `debug`, `output`, `stats` |
| `ElectricalRoutingConfig` | `python/photonic_router/electrical/types.py` | the electrical pipeline's parameters, inside `FlowOptions.electrical` |
| `SessionSettings` | `python/photonic_router/routing/settings.py` | one routing session's frozen inputs, built from the two trees above |

Precedence, lowest to highest (`config_loading.build_config`, `cli.main`):

1. the dataclass defaults (`config.py`, `flow_options.py`, the `SCRIPT_*`
   constants of `flow_config.py` that the parser uses as defaults);
2. the benchmark's file `benchmarks/<name>.toml`: its `[flags]` table
   (flag names without `--`, in file order) is prepended to the command
   line, its `[config]` table (`PHOTONIC_ROUTER_*` names without the prefix,
   string values) is applied through the overlay; under
   `--preplaced-crossing-grids true` the `--crossings` / `--crossing-mode`
   flags of the file are dropped (`stable_flags_for_configuration`);
3. the process environment, `PHOTONIC_ROUTER_*`, through the same overlay
   table `ENV_OVERLAY` (`env_overlay.py`, the only module that parses those
   strings); it wins over the benchmark's `[config]`; an absent variable is
   never applied;
4. the command line: `--configuration`'s flags are inserted after the
   benchmark's flags and before the user's own, so an explicit flag wins.

The three paper configurations are pinned in `cli.py`'s
`_CONFIGURATION_FLAGS` (`baseline` = `--crossing-mode lidar-pure`,
`contribution1` = `--crossing-mode lidar-guided`, `contribution2` =
`--preplaced-crossing-grids true --verbose-routes`), exactly the flags
`scripts/results/reproduce_date2027.sh` passes; changing one changes what the
paper's numbers mean. Exactly one runs at a time. The crossing modes are
`CROSSING_MODES = ("lidar-pure", "lidar-guided")`
(`crossings/crossing_modes.py`); contribution 2 is a flow flag, not a mode.

[`docs/CONFIGURATION.md`](CONFIGURATION.md) lists every field of
`RoutingConfig`, `FlowOptions` and `ElectricalRoutingConfig` with its type,
default, overlay variable and flag. It is generated by
`scripts/generate_config_reference.py` from the dataclasses, the overlay
table and the parser, and `tests/test_config_reference.py` fails when the
checked-in file differs from a fresh generation.


## 7. Code layout: the Rust crates and the Python package

### 7.1 The Cargo workspace

The root `Cargo.toml` is the workspace: members `crates/router-core` and
`crates/router-py`, shared package fields and dependency versions
(`pyo3 0.22` with `abi3-py38`, `rustc-hash 2`), and two workspace-wide clippy
allows (`too_many_arguments`, `type_complexity`) that name the debt they
stand for. `rust-toolchain.toml` selects `stable`.

### 7.2 `crates/router-core` (about 50,000 lines, tests included)

Pure Rust, depends on `rustc-hash` only, so `cargo test -p router-core` runs
without a Python interpreter. `src/lib.rs` declares the modules and
re-exports the public names; it contains no logic.

| module | lines | responsibility |
| --- | ---: | --- |
| `config.rs` | 360 | `RouterConfig` and its four groups, `GridSpec`, the search-engine names |
| `error.rs` | 30 | `RouterError` (`Config`, a Python `ValueError` at the binding) |
| `obstacle_map.rs` | 2,100 | the routing database (section 3); `tests/obstacle_map_tests.rs` (180) is its integration test |
| `static_obstacle_builder.rs` | 1,600 | grid spec, rasterization, clearance inflation, port openings, `StaticCellSet` |
| `primitives.rs` | 390 | headings, primitives, the primitive library |
| `crossings.rs` | 200 | `CrossingConfig`, `CrossingContext`, `CrossingPair`: crossing settings and expected pairs, independent of the A* hot path |
| `geometry_realization.rs` | 5,460 | route to centerline to polygon, waypoint compression, endpoint snapping |
| `search/mod.rs` | 130 | the `NetSearch` interface (section 4) |
| `search/state.rs`, `geometry.rs` | 250, 270 | `State`, `RouteResult`, `RouteSearchStats`; grid-polyline geometry |
| `search/grid_dijkstra.rs` | 450 | the uniform-cost oracle engine |
| `search/astar/` | 15,000 + 3,100 tests | the A* engine: `mod.rs` (940, `AStarSearch`, `run_search`), `kernel.rs` (1,470), `expansion.rs` (2,440), `crossing_rules.rs` (2,240, plus `crossing_rules/tests.rs`), `simple_route.rs` (3,340), `dense.rs` (1,560), `window.rs` (660), `cost.rs` (550), `heuristic.rs` (440), `config.rs` (390, `AStarConfig`, `AStarParams`), `diagnostics.rs` (830, trace printers over `&mut dyn Write`), `svg.rs` (120) |
| `engine/mod.rs` | 300 | `Router`: the obstacle map, primitive library, crossing context, committed routes, history bookkeeping, `RouterConfig`, the search engine, the generation counters the meander planner syncs on |
| `engine/api.rs` | 210 | the plain-data operations the binding forwards to (static cells and rectangles, port openings, net cells, raw views, rip-up) |
| `engine/jobs.rs`, `cells.rs`, `commit.rs` | 320, 640, 470 | job and batch types; footprint and port cell arithmetic; committing a route |
| `engine/search_call.rs`, `search_calls.rs` | 260, 1,870 | the `SearchCall` value and the one `route_and_commit` body |
| `engine/crossing_reservation.rs` | 3,480 | crossing events, reservations, partner sets, realized-crossing validation, verdict records |
| `engine/crossing_geometry.rs` | 1,250 | segment geometry and the crossing-legality predicate |
| `engine/endpoint_correction.rs` | 1,060 | the checked grid-to-port correction pass |
| `engine/diagnostics.rs` | 250 | trace records and batch timers |
| `engine/negotiation/` | 4,010 | the loop (`loop_.rs`, 1,200 with its tests) and its rules: `budget.rs`, `crossing_free.rs`, `ripup.rs`, `queue.rs`, `attempt.rs`, `probe.rs`, `braid.rs` (930), `trace.rs` |
| `meander/` | 5,230 | `geometry.rs` (990, analytic comb), `search.rs` (1,640, obstacle-aware placement and occupancy prefix), `plan.rs` (2,570, the registered PLM planner and every plan decision); `mod.rs` re-exports |

Test fixtures live in `search/test_support.rs` and `engine/test_support.rs`
(`#[cfg(test)]`); production code does not use them.

### 7.3 `crates/router-py` (about 4,600 lines)

The PyO3 surface, built by maturin into `photonic_router._rust`
(`pyproject.toml`: `manifest-path = "crates/router-py/Cargo.toml"`,
`module-name = "photonic_router._rust"`, `python-source = "python"`).

| module | lines | responsibility |
| --- | ---: | --- |
| `lib.rs` | 1,150 | `PyPhotonicRouter { inner: Router }` and its `#[pymethods]` (about forty methods), the `#[pymodule]`, `py_err` (`RouterError` to Python) |
| `types.rs` | 940 | the `#[pyclass]` configuration, job and result types (`RouterConfig`, `AStarConfig`, `GridSpec`, `State`, `RouteJob`, `RouteResult`, ...) and their conversion into the core's structs |
| `convert.rs` | 590 | conversions in both directions (route results, timing dictionaries, `native_jobs_from_py`) |
| `meander_py.rs` | 1,490 | `MeanderPlanner` (`register`, `register_geometries`, `reserve_rect`, `clear_reserved`, `open_cell_count`, `profile`, `plan_requests`, `plan_final`, `plan_centerline`, static `realize`) and `auto_meander_search_config_rs` |
| `static_obstacle.rs` | 200 | `build_static_obstacle_map_rs`, `PyStaticCellSet` |
| `tests.rs` | 240 | binding tests that read a Python result (need `PYO3_PYTHON`) |

The rule: bindings convert and delegate, they do not decide. The planner
holds a router's path-length-matching state and syncs on the router's
`obstacle_generation` (bumped wherever the obstacle map changes) and
`static_reset_generation` counters, so a commit after registration drops its
occupancy prefix and a static reset drops the registration.
`python/photonic_router/_rust.pyi` (760 lines) is the generated type stub.

### 7.4 The Python package `python/photonic_router/` (about 45,700 lines)

Top-level modules (23 files, 7,300 lines):

| module | lines | responsibility |
| --- | ---: | --- |
| `__main__.py`, `cli.py` | 50, 850 | `python -m photonic_router route`; the parser, benchmark defaults, `--configuration`, `_flow_options` |
| `flow.py` | 870 | `route_benchmark`, `route_schematic` (the stage outline) and the `run_routing_flow` keyword wrapper |
| `flow_options.py`, `flow_config.py` | 210, 270 | `FlowOptions`; the `SCRIPT_*` defaults and option resolution |
| `flow_optical.py`, `flow_verification.py`, `flow_plm.py`, `flow_electrical.py` | 180, 400, 230, 200 | the flow's stage helpers |
| `flow_reporting.py`, `flow_stats.py`, `flow_component_info.py` | 550, 300, 30 | console output, GDS write; `RoutingFlowStats` and the SUMMARY line; component info |
| `config.py`, `config_loading.py`, `env_overlay.py` | 390, 40, 810 | the configuration trees, their precedence, the overlay table |
| `static_obstacle_builder.py`, `benchmark_extractor.py`, `routing_layers.py` | 930, 180, 130 | obstacle extraction and the `ObstacleMapBuilder` Protocol; geometry and port extraction; layer and port access tables |
| `primitive_library.py`, `port_roles.py`, `graph_analysis.py`, `benchmark_metadata.py` | 180, 150, 60, 190 | primitive ids to components; input-side ports; shared graph context; benchmark metadata (`NODE_TYPES`, `INTERNAL_DELAYS_UM`, `INPUT_PORTS_BY_COMPONENT`) |

Sub-packages:

| package | files, lines | responsibility |
| --- | --- | --- |
| `routing/` | 20, 10,100 | the routing session: `session.py` (the nine-phase `run`), `settings.py`, `state.py`, `stages.py` (the Protocols), one module per phase (`obstacle_context`, `router_setup`, `route_jobs` 1,490, `crossing_plan_stage`, `handoff`, `dispatch`, `finalize` 1,170, `detour_pass`, `verify_repair` 1,110, `realize`), `diagnostics.py` (1,210, the debug write-ups), `crossing_plan_info.py`, `timing.py`, `api.py` (`route_match_and_realize`), `route_rust.py` (a re-export shim for older import paths) |
| `crossings/` | 11, 5,800 | `crossing_demand.py` (bands, invariant pairs, the lower bound), `crossing_plan.py` (bubble-sort events), `preplaced_crossing_grids.py` (1,400, contribution 2's tiles), `crossing_structures.py`, `crossing_modes.py`, the realized-crossing plan, components and verification (`route_rust_crossing_*`, 1,300 for verification), `crossing_verification_report.py`, `topology_analysis.py` (test oracle) |
| `plm/` | 5, 3,600 | `path_length_graph.py`, `path_length_requirements.py`, `path_length_diagnostics.py`, `route_rust_meanders.py` (1,850) |
| `verify/` | 3, 2,150 | `photonic_verification.py` (1,240, the geometry verifier), `route_rust_endpoint_correction.py` (900) |
| `records/` | 8, 3,600 | `route_rust_types.py` (1,540, the shared records), `route_rust_records.py`, `route_rust_geometry.py`, `route_rust_realization.py`, `route_rust_obstacle_config.py`, `route_order.py` (`NET_ORDERS`), `layout_from_schematic.py` |
| `gds/` | 8, 2,850 | GDS write options, the routes overview SVG, debug probe artifacts, gdsfactory's reference router (`route_gds.py`), and circuit extraction and rebuild from a foundry GDS (`gds_circuit_extraction.py`, `gds_circuit_rebuild.py`, `gds_circuit_components.py`) |
| `electrical/` | 29, 10,300 | the heater-metal pipeline (2.15); `__init__.py` is its public surface |

A stage module takes `(settings, state, <its input>)` and returns its
product; it does not see the command line, the environment or a session
object. Code outside `routing/` reaches the session through `routing/api.py`
or the `photonic_router.routing` re-exports.

### 7.5 Around the package

| path | contents |
| --- | --- |
| `benchmarks/` | one module per benchmark (`build_schematic()`), `benchmarks/<name>.toml` defaults, shared builders (`benes.py`, `cascade.py`), `data/` (mesh YAML), `private/` (git-ignored) |
| `scripts/` | `lint.sh`, `test_baseline.sh`, the generators (`generate_config_reference.py`, `generate_rust_stubs.py`), `benchmark_photonic.py`, `benchmark_electrical.py`, `profile_astar.py`, `extract_private_benchmark.py`, `make_mesh_slice_benchmark.py`, `crossing_benchmark_report.py`, `lidar_bridge/`, `path_investigation/` (an exploratory harness kept as written) |
| `scripts/results/` | the oracles and the archive tooling (section 8) |
| `tests/` | unit tests by module, `e2e/` (full runs, marker `e2e`), `electrical/` (one module per electrical stage), `fixtures/` (synthetic layouts, sessions, PLM fixtures), `baselines/` (pinned numbers) |
| `docs/` | this document, the generated configuration reference, the reproduction documents and their pinned tables, `history/` (earlier states) |


## 8. Build, tests and oracles; how to add a benchmark

The commands, with the lint and commit conventions, are in
[`CONTRIBUTING.md`](../CONTRIBUTING.md). In short:

```bash
.venv/bin/pip install -e ".[dev]" pytest        # first build
.venv/bin/maturin develop --release             # after a change under crates/
cargo test --release -p router-core             # the engine, no Python
PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --release -p router-py
.venv/bin/python -m pytest -q -m "not e2e" tests
.venv/bin/python -m pytest -q tests             # with the end-to-end runs
scripts/test_baseline.sh                        # both suites against tests/baselines/test_baseline.txt
scripts/lint.sh                                 # ruff, generated files, mypy, rustfmt, clippy
```

### 8.1 The oracles

Two pinned reproductions guard behaviour; a change that may move a routed
result runs them.

* **The paper gate**, `scripts/results/gate_short.sh` (about two minutes):
  routes the nine smallest paper cells (Benes 4x4 and 8x8 flat, ADEPT 8x8;
  baseline, contribution 1, contribution 2) through
  `reproduce_date2027.sh short` into a temporary root and compares them with
  `compare_date2027.py` against `docs/date2027_table_sources.json`: crossing
  count and GDS length exact (to 0.01 um), zero verifier errors. It prints
  one `ok` row per cell and `all reproduced cells match`, and exits non-zero
  otherwise. The full 27 cells are `reproduce_date2027.sh` (about 2.5 h, into
  `results_date2027/`, never `results/`) followed by `compare_date2027.py
  results_date2027 docs/date2027_table_sources.json`; `--with-bounds` adds
  the placement lower bound (`docs/date2027_crossing_bounds.json`) and the
  excess per cell. [`docs/DATE2027_REPRODUCTION.md`](DATE2027_REPRODUCTION.md)
  records the provenance and every full re-verification.
* **The PLM gate**, `scripts/results/plm_gate.sh` (two to three minutes):
  routes `heater_s_mod` and `cascade_8_compact` and compares GDS length,
  meander length, group count, groups over tolerance, realized spreads, nets,
  rc and verifier errors against `docs/plm_baseline.json` (`plm_table.py
  --compare`), ending with `all PLM cells match`. The full thirteen cells are
  `plm_baseline.sh`; see [`docs/PLM_REPRODUCTION.md`](PLM_REPRODUCTION.md).

Beside them: `scripts/test_baseline.sh` (pass counts and the failure list of
both suites, and the fast suite under 60 s),
`scripts/benchmark_electrical.py --suite --check --compare-baseline` (the four
heater cases against their guardrails and
`tests/baselines/electrical_suite_metrics.json`; no paper configuration
routes electrically), and `scripts/results/compare_verification_reports.sh
<root_a> <root_b>` (byte-compares the verification reports of two roots;
they carry no timing). Routing-loop times are printed as ratios and never
compared: they depend on the machine.

A change that moves a pinned number on purpose re-pins it in the same
commit and says why in the message (CONTRIBUTING.md).

### 8.2 How to add a benchmark

1. Write `benchmarks/<name>.py` with `build_schematic() -> Schematic`
   (gdsfactory's `Schematic`: instances, placements, nets). Reuse a builder
   where one exists (`benes.py`, `cascade.py`); `heater_single.py` is a small
   example. Optional module attributes the analyses read:
   `INPUT_PORTS_BY_COMPONENT` (input-side ports of custom cells, for PLM),
   `NODE_TYPES` and `INTERNAL_DELAYS_UM` (benchmark metadata).
2. If it needs non-default flags or overlay settings, add
   `benchmarks/<name>.toml` with a `[flags]` table (flag names without `--`;
   booleans as `true`/`false`) and a `[config]` table (`PHOTONIC_ROUTER_*`
   names without the prefix, string values). A comment block at the top
   says why. `tests/test_benchmark_defaults.py` loads every TOML through the
   CLI helper.
3. Run it: `.venv/bin/python -m photonic_router route <name>` and read the
   SUMMARY line (`verifier_errors=0`).
4. If it is to be pinned, add it to `scripts/results/plm_baseline.sh` (PLM)
   or a results script, and re-pin the table in the same commit.

A benchmark built from a foundry GDS goes through
`scripts/extract_private_benchmark.py <name>` into the git-ignored
`benchmarks/private/` and runs as `route private.<name>`; nothing in the
repository names a foundry cell.

### 8.3 Generated files

| file | generator | checked by |
| --- | --- | --- |
| `docs/CONFIGURATION.md` | `scripts/generate_config_reference.py` | `--check`, `tests/test_config_reference.py` |
| `python/photonic_router/_rust.pyi` | `scripts/generate_rust_stubs.py` (reads the built module) | `--check`, `tests/test_rust_stub.py` |
| `docs/date2027_crossing_bounds.json` | `scripts/results/crossing_lower_bound.py --all-paper-cells --json docs/date2027_crossing_bounds.json` | `compare_date2027.py --with-bounds` reads it |
| `docs/plm_baseline.json` | `scripts/results/plm_table.py results_plm --json docs/plm_baseline.json` | `plm_gate.sh` |
| `tests/baselines/test_baseline.txt` | `TEST_BASELINE_UPDATE=1 scripts/test_baseline.sh` | `scripts/test_baseline.sh` |
| `tests/baselines/electrical_suite_metrics.json` | `scripts/benchmark_electrical.py --suite --output tests/baselines/electrical_suite_metrics.json` | `--compare-baseline` |
| `benchmarks/<slice>.py` + `.toml` | `scripts/make_mesh_slice_benchmark.py` | the slice's own tests |

`docs/date2027_table_sources.json` is the paper's `EXPERIMENTS_TABLE_SOURCES.json`
(written by `scripts/results/paper_table.py` next to the paper's sources); it
is the reference, not regenerated here.


## 9. Switches for experiments

These exist to measure alternatives; all are off (or neutral) by default, so
the defaults route the paper's numbers.

* **Unified crossing legality**, `RouterConfig.crossing.unified_legality`
  (`PHOTONIC_ROUTER_UNIFIED_CROSSING_LEGALITY`, presence). On, the search
  hook (`crossing_rules.rs`), the commit check and the post-commit validation
  all decide a crossing with one predicate,
  `crossing_geometry::crossing_legality(&CrossingCandidate,
  &CrossingLegalityRule) -> CrossingVerdict`: one perpendicularity test
  (normalised dot product within 1e-6); one margin, `required_margin_cells =
  half_size + bend_runout`, on the grid route (where a bend contributes its
  arms as straight cells) for the route's run before the crossing and both
  sides of the partner; `half_size` cells of pure straight after it; spacing,
  i.e. the candidate's reservation window (every cell within `half_size` of
  its centre on both axes) disjoint from the nearest other crossing of the
  same route and of the registered crossings of other nets; then the
  footprint (no third net's dynamic cell in the window) and, post-commit,
  the registered event. The verdict records are committed state: a rip-up
  drops them and the braid rollback restores them with the events. Python
  (`route_rust_crossing_verification.py`) then classifies realized crossings
  from the verdicts instead of re-deciding them. Off, the three legacy paths
  and the Python re-decision run as before. It is a switch until the 27-cell
  ablation decides which side stays.
* **Negotiation switches** (`NegotiationConfig`, overlay
  `PHOTONIC_ROUTER_NEGOTIATED_*` and `PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR`):
  `disable_braid_repair`, `braid_escalation`, `crossing_free_unplanned`
  (default on), and the ablation switches `disable_source_column_widening`,
  `disable_post_ripup_guidance`, `disable_simple_route_probe`,
  `history_on_first_attempt`; plus the three budgets.
* **Search** (`SearchOverrides`): `engine` (`astar` / `grid-dijkstra`),
  `astar_timeout_ms`, `max_dense_states`, `long_straight_congestion_weight`;
  and on the command line `--heuristic-mode`, `--primitive-ordering`,
  `--heap-tie-breaker`, `--use-indexed-heap`, `--enable-simple-routes`,
  `--routing-window-scale`, `--max-iterations`.
* **Order and detours**: `--net-order` (`NET_ORDERS` in
  `records/route_order.py`), `--detour-pass-factor`.
* **Diagnostics**: `--debug-timing true` (stage and per-route timing),
  `--debug-meanders` (the PLM planning profile and per-candidate log),
  `--debug-svgs <selector>`, `--attempt-diagnostics`, `--verbose-routes`;
  the kernel traces of `KernelDiagnostics` (`PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`,
  `PHOTONIC_ROUTER_TRACE_CROSSING*`, ...), which `reproduce_date2027.sh` turns
  on for the archived runs. `--debug-stop-after-route N` is the exception
  that does change the result: it routes only the first `N` nets.

`scripts/results/ablation.sh` and `ablation_table.py` run and tabulate the
27-cell ablation over these switches.


## 10. Invariants

Each of these is true of the current code and checked as named.

1. **No algorithm reads the environment.** `grep -rn "env::var\|var_os"
   crates/` is empty; on the Python side `PHOTONIC_ROUTER_*` strings are
   parsed only in `env_overlay.py`, and only `config_loading.py` and the
   `from_environment` constructors in `config.py` read `os.environ` for them.
2. **Python and Rust configuration mirror each other.** Every
   `RouterConfig` field exists on both sides with the same default; a field
   is added on both sides together and `docs/CONFIGURATION.md` is
   regenerated (`tests/test_config_reference.py`).
3. **A search reads the map, the engine writes it.** No module under
   `src/search/` commits to or rips up the obstacle map outside its tests;
   commitment is `src/engine/commit.rs`, rip-up the negotiation rules.
4. **One batch entry point.** Every multi-net routing, the first routing,
   the detour pass and the two verify-repair passes alike, goes through
   `run_negotiated_batch`; `routing/dispatch.py::native_route_jobs` is the
   one place that packs the kernel's jobs.
5. **One search interface.** Every single-net search goes through
   `NetSearch::search`; the oracle engine refuses what it cannot do
   (`unsupported_request`) rather than approximating it.
6. **The obstacle map's flags agree with its counts.** `STATIC_BIT` iff a
   static reference exists, `DYNAMIC_BIT` iff a dynamic one does,
   `STATIC_RECT_BIT` iff a static rectangle covers the cell; a cell's owner
   list mirrors `net_routes` exactly (module documentation of
   `src/obstacle_map.rs` and its tests).
7. **The search never produces a crossing the commit rejects** when unified
   legality is on: the hook, the commit check and the post-commit validation
   call the same predicate with the router's one `crossing_legality_rule()`.
8. **Bindings convert and delegate.** `crates/router-py` holds no routing
   decision; `cargo tree -p router-core` contains no `pyo3`.
9. **Stages are pure of the outside.** No routing stage reads the command
   line or the environment; the optical stages do not import
   `photonic_router.electrical`.
10. **Pre-placed grids refuse what they cannot model.** A design with a net
    outside the band model is rejected before any tile is placed.
11. **Tracing does not change results.** `--debug-meanders` changes what
    is printed, not a plan, a SUMMARY token or the acceptance decision; the
    kernel traces change output and time, not routes (the reproduction runs
    use `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` and match the archives;
    `--verbose-routes` is part of the contribution 2 configuration).
12. **Generated files cannot drift.** `docs/CONFIGURATION.md` and
    `_rust.pyi` are compared with a fresh generation in the tests and in CI.


## 11. History and provenance

The release history is [`CHANGELOG.md`](../CHANGELOG.md): the do-over of
2026-10 (phases 0 to 8, which produced this layout) is its Unreleased
section, one bullet per slice with the evidence that it kept the numbers.
The paper's numbers were produced by the engine at the tag
`date2027-results` (commit `960d0ee`, 2026-09-22; branch `DATE2027-results`);
the archived runs are attached to that tag's GitHub release, and
[`docs/DATE2027_REPRODUCTION.md`](DATE2027_REPRODUCTION.md) traces every row
to its commit. Other tags mark earlier states: `baseline-2026-09-03`,
`paper-base-2026-09-07`, `pre-do-over-2026-10-27`. Documents that describe
earlier states of the repository are kept in [`docs/history/`](history/README.md);
they are not descriptions of the current code.
