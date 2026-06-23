# Pass 13A - Metal Routing Architecture Audit

## Current State

Metal routing already exists as a Python-first electrical heater-routing stage
under `translation/electrical/`.

The implemented pipeline is:

1. Extract logical two-terminal heater groups from placed heater instances.
2. Build a layer-filtered electrical obstacle grid.
3. Route one terminal per heater to a common bus.
4. Compute individual escape topology for the remaining terminals.
5. Assign abstract pad slots.
6. Route detailed individual bundle centerlines.
7. Realize bus, wires, and assigned pads as metal polygons.
8. Optionally export an electrical debug SVG.

The primary orchestrator is `translation/electrical/route_electrical.py`.
The routing data model and defaults are in `translation/electrical/types.py`.
The existing integration tests are in `tests/test_electrical_routing.py`.

## Layer Model

The shared layer definitions live in `python/photonic_router/routing_layers.py`.

Current optical obstacle layer:

- Optical route / waveguide obstacles: `(1, 0)`

Current heater and metal obstacle layers:

- Heater: `(47, 0)`
- M2: `(45, 0)`
- M3 / MTOP: `(49, 0)`
- VIA1: `(44, 0)`
- VIA2: `(43, 0)`

Current electrical routing defaults:

- Realized metal layer: `(49, 0)`
- Heater layers: `((47, 0),)`
- Metal obstacle layers: `HEATER_METAL_OBSTACLE_LAYERS`
- Routing grid pitch: `10.0 um`
- Wire width: `20.0 um`
- Obstacle clearance: `10.0 um`
- Terminal open radius: `15.0 um`

The optical router can include heater/metal geometry as static obstacles through
`include_heater_obstacles=True`. The electrical router separately builds its own
obstacle map from configured metal/heater layers.

## Obstacle Representation

Electrical obstacles reuse `photonic_router.static_obstacle_builder`.

`build_electrical_obstacle_map()` constructs a `StaticObstacleMapConfig` with:

- `grid_size_um=config.routing_grid_pitch_um`
- `security_margin_um=0.0`
- `clearance_um=config.obstacle_clearance_um`
- `obstacle_layers=config.metal_obstacle_layers`
- `heater_obstacle_layers=None`
- a routing die bbox expanded for bus and pad rows

It then clears:

- bus stripe cells
- source terminal opening cells

This is a good reuse boundary: geometry extraction/rasterization stays shared,
while electrical search behavior stays local to the Python electrical package.

## Rust Grid/A* Reuse Decision

Do not move metal routing into the Rust primitive A* engine yet.

Reasons:

- The Rust router is optimized around optical state `(x, y, angle)` and
  primitive IDs, including bend-angle constraints and route realization through
  photonic primitives.
- The current metal router needs rectilinear paths, same-net buses, grouped
  terminal selection, pad slot assignment, and bundle ordering. Those are
  topology/planning problems around electrical semantics, not primitive
  photonic path problems.
- The existing Python stage already reuses the Rust-capable static obstacle
  builder, so the expensive geometry rasterization path is shared without
  coupling metal routing to optical primitive semantics.

The right near-term architecture is:

- Keep electrical routing Python-first.
- Reuse the shared static obstacle builder for grid construction.
- Integrate the electrical stage into `routing_flow.py` after optical routing.
- Reconsider Rust acceleration only if profiling shows BFS/grid search time is
  a real bottleneck on larger metal benchmarks.

## Benchmarks

Existing heater benchmarks already expose electrical ports through
`straight_heater_metal` instances:

- `benchmarks/mmi_heater.py`
- `benchmarks/mmi_heater_8x4.py`
- `benchmarks/mmi_heater_8x4_ripup_reroute.py`

`tests/test_electrical_routing.py` currently uses the single-heater and
multi-heater benchmarks for terminal extraction, common bus routing, pad
planning, detailed bundle routing, and metal realization checks.

The first useful routing-flow benchmark should be `mmi_heater.py`, then
`mmi_heater_8x4.py` once the stage is callable from `routing_flow.py`.

## Gaps Before Routing-Flow Integration

1. `routing_flow.py` has only the optical 3-stage flow today. It does not import
   or call `route_electrical_heaters()`.
2. The final returned/written component is always the optical `routed_layout`.
   When electrical routing is enabled, the final component should become
   `electrical_result.routed_component`.
3. `RoutingFlowStats` has no electrical counters or timings.
4. CLI/script defaults have no electrical enable flag or pad/routing controls.
5. Debug cleanup/opening only covers optical static obstacles and route SVGs.
   Electrical SVGs currently write under `build/electrical/`.
6. The electrical router assumes heater detection by instance prefix
   `("heater",)` and component pattern `("straight_heater_metal*",)`. This is
   fine for current benchmarks, but future benchmarks should make this explicit
   through config rather than hidden naming conventions.

## First Implementation Plan

Pass 13B should be a narrow routing-flow integration, not a routing algorithm
rewrite.

Recommended scope:

1. Add `enable_electrical_routing: bool = False` to `run_routing_flow()`.
2. Import `ElectricalRoutingConfig` and `route_electrical_heaters`.
3. After optical routing and optional path-length matching, call
   `route_electrical_heaters(routed_layout, schematic, electrical_config, ...)`
   when enabled.
4. If `electrical_result.routed_component is None`, raise a clear
   `RuntimeError` summarizing failed heaters/routes.
5. Replace `routed_layout` with `electrical_result.routed_component` before
   KLayout display and GDS write.
6. Store a compact electrical summary in `routed_layout.info`, including:
   terminal group count, common-bus success, pad assignment count, detailed
   route count, failed detailed route count, and debug artifact paths.
7. Add CLI flags:
   - `--electrical-routing BOOL`
   - `--electrical-pad-side {top,bottom}`
   - `--electrical-grid-pitch-um`
   - `--electrical-obstacle-clearance-um`
8. Add an integration test that runs `run_routing_flow("mmi_heater",
   enable_electrical_routing=True, ...)` and verifies metal polygons exist on
   the configured metal layer.

This keeps the first routing-flow metal pass deterministic and reviewable.
Once this lands, the next benchmark-driven work should focus on cases that fail
the existing Python topology planner, not speculative Rust migration.
