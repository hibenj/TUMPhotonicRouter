# TUMPhotonicRouter

TUMPhotonicRouter is a Rust + Python photonic integrated circuit routing prototype. It starts from a Python/gdsfactory schematic, creates an unrouted layout, builds a grid obstacle model, routes optical waveguides with a Rust backend, can insert path-length matching meanders, and can optionally route heater electrical metal.

The project is currently aimed at research and implementation work on photonic routing algorithms rather than being a polished package. The core idea is to keep layout orchestration and GDS/gdsfactory integration in Python while moving the expensive routing kernels into Rust through PyO3.

## What This Repository Can Do

- Load parametric Python benchmarks from `benchmarks/`.
- Translate a `gdsfactory.schematic.Schematic` into an unrouted `Component`.
- Rasterize gdsfactory geometry into a grid obstacle map with configurable clearance, security margin, port openings, and layer selection.
- Route optical nets with an 8-heading, primitive-based router using straights, 45-degree bends, and 90-degree bends.
- Use deterministic straight, L, and Z route checks before invoking full A*.
- Route around existing routed nets with dynamic obstacle tracking.
- Repair conflicts through probe routing, rip-up, reroute, and congestion history penalties.
- Preserve physical realization by converting Rust primitive results back into gdsfactory geometry.
- Analyze routed path lengths and insert analytic meanders for matching requirements.
- Route heater electrical metal after optical routing.
- Export debug SVGs for static obstacles, routed nets, failed routes, meanders, and electrical metal.
- Produce benchmark and profiling reports with detailed A* counters.

## Positioning

[LiDAR](https://github.com/ScopeX-ASU/LiDAR) is an established open-source PIC detailed router focused on large-scale curvy waveguide routing, crossing insertion, congestion-aware ordering, and DRV-clean GDS generation.

This repository explores a complementary implementation style:

- A hybrid Python/Rust architecture where Python keeps direct gdsfactory access and Rust owns the search-heavy kernels.
- Primitive-level route planning that maps Rust primitive IDs back to physical gdsfactory components.
- Built-in path-length analysis and meander insertion after routing.
- Optional heater electrical routing in the same top-level flow.
- Fine-grained profiling and debug artifacts for algorithm development.
- Dense grid data structures and prefix-sum based collision checks designed for fast experimentation.

The emphasis is not only on producing a final layout, but also on making each routing decision measurable and debuggable.

## Architecture

The main flow is in `routing_flow.py`:

```text
benchmark module
    |
    v
load_benchmark()
    |
    v
layout_from_schematic()
    |
    v
route_match_and_realize()
    |
    +--> static obstacle extraction
    +--> Rust optical routing
    +--> endpoint correction and realization
    +--> optional path-length matching
    +--> optional electrical heater routing
    |
    v
routed gdsfactory Component / GDS / debug artifacts
```

### Python Side

Python handles the parts that need layout and design-system context:

- benchmark loading from `benchmarks/*.py`
- schematic to layout translation in `translation/layout_from_schematic.py`
- gdsfactory port extraction and orientation conversion
- primitive realization into gdsfactory geometry
- path-length requirement analysis
- meander insertion orchestration
- electrical heater routing
- CLI, debug output, and benchmark reports

Important files:

- `routing_flow.py` - end-to-end orchestrator
- `translation/route_rust.py` - Python/Rust optical routing bridge
- `translation/route_rust_meanders.py` - path-length matching meander flow
- `translation/electrical/route_electrical.py` - heater metal routing orchestrator
- `python/photonic_router/static_obstacle_builder.py` - layout rasterization
- `python/photonic_router/primitive_library.py` - Rust primitive ID to gdsfactory component mapping

### Rust Side

Rust handles the performance-sensitive geometry and search work:

- grid state representation
- obstacle map storage
- static and dynamic obstacle queries
- primitive library generation
- simple straight/L/Z route validation
- dense A* routing
- optional JPS4 experiments for plain 4-connected grid baselines
- route realization helpers and meander geometry helpers
- PyO3 bindings exposed as `photonic_router._rust`

Important files:

- `src/astar.rs` - single-net A* router and search accelerators
- `src/simple_routes.rs` - deterministic straight, L, and Z candidate routing
- `src/obstacle_map.rs` - static/dynamic obstacle map and rip-up database
- `src/primitives.rs` - photonic primitive definitions
- `src/geometry_realization.rs` - route polygon, port access, and meander realization helpers
- `src/static_obstacle_builder.rs` - Rust-backed obstacle rasterization
- `src/py_router.rs` - PyO3 API

## Routing Stages

### 1. Benchmark Loading

Benchmarks are Python modules. Each benchmark exposes:

```python
from gdsfactory.schematic import Schematic


def build_schematic() -> Schematic:
    ...
```

This keeps benchmark generation parametric and avoids a separate YAML or JSON intermediate format.

### 2. Schematic to Layout

`layout_from_schematic()` places the schematic instances into a gdsfactory `Component`. At this point the layout contains devices and fixed structures, but not routed waveguides.

### 3. Static Obstacle Map

`build_static_obstacle_map()` extracts layout geometry, converts physical coordinates to integer grid cells, expands obstacles by clearance, and opens small access regions around ports.

The obstacle builder supports:

- configurable grid size, normally `0.5 um`
- security margin around the die
- waveguide clearance
- separate heater obstacle clearance
- bounding-box or cell-set obstacle materialization
- selected obstacle layers
- debug SVG export
- Rust backend with Python fallback

### 4. Straight, L, and Z Routes

Before running A*, the router tries deterministic simple routes:

- straight route
- one-bend L route
- two-bend Z route

These routes are fast because they do not search a graph. The router constructs a small number of candidate polylines and validates their axis-aligned segments directly against the obstacle map. For open space or simple channel cases, this avoids heap operations and large state expansion entirely.

### 5. Primitive-Based A*

If no simple route is legal, Rust runs A* over states:

```text
(x_grid, y_grid, angle_idx)
```

`angle_idx` uses 8 discrete headings:

```text
0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE
```

Neighbors are not arbitrary grid steps. They are photonic movement primitives:

- short straight
- long straight
- optional 45-degree left/right bend
- 90-degree left/right bend

Each primitive carries:

- start angle
- end angle
- grid displacement
- conservative grid footprint
- physical length
- bend cost
- realization geometry

The cost model combines length, bend penalty, and optional history cost from rip-up/reroute.

### 6. Dynamic Obstacles and Rip-Up/Reroute

After each successful net, the route is committed to the Rust obstacle map as dynamic occupancy. Later nets treat these cells as blocked.

When a net fails, the repair flow can:

1. route a probe path while ignoring dynamic obstacles
2. find existing routed nets that block the probe
3. add history cost to congested cells
4. rip up a bounded number of victim nets
5. route the failed net
6. reroute the victims with history-aware cost
7. rollback if a repair round fails

This keeps the normal flow simple while still allowing local conflict repair.

### 7. Physical Realization

The Rust result contains primitive IDs, states, route cells, and length data. Python turns that back into layout geometry using the primitive library and realization helpers.

The primitive library is deliberately 1:1 with Rust IDs, so the search representation and gdsfactory realization stay aligned.

### 8. Path-Length Matching and Meanders

The path-length matching flow analyzes routed edges, computes missing extra length, and inserts analytic meanders when legal.

The meander planner uses Rust geometry helpers for:

- centerline extraction
- available interval probing
- depth sweep planning
- obstacle checks
- reserved meander cell registration
- route polygon realization with the inserted meander

This makes path-length matching part of the routing pipeline instead of a disconnected post-processing script.

### 9. Electrical Heater Routing

The optional electrical flow extracts heater terminal groups, builds an electrical obstacle grid, plans pad slots, routes common bus or detailed metal, realizes metal polygons, verifies the result, and writes debug SVGs.

This is useful for layouts where optical waveguides and heater metal need to be considered together.

## Why the Router Is Fast

The current implementation contains several performance-oriented choices.

### Rust for Search Kernels

The high-frequency loops are in Rust. Python calls into a PyO3 extension, but individual A* expansions, obstacle checks, heap operations, and route reconstruction do not run through Python.

### Dense State Storage

Within a routing window, A* uses dense arrays instead of hash maps:

```text
state_index = ((local_y * width) + local_x) * 8 + angle
```

The router stores:

- `g_costs: Vec<f64>`
- `best_generation: Vec<u32>`
- `parent_idx: Vec<u32>`
- `parent_primitive: Vec<u16>`
- closed-state bitset

This improves locality and removes hashing from the inner search loop.

### Bitsets and Packed Cell Keys

Obstacle cells use compact `u64` keys for sparse maps and bitsets for dense maps. The packed key stores signed grid coordinates in one value:

```text
((x as u32 as u64) << 32) | (y as u32 as u64)
```

The code also uses `rustc_hash::FxHashMap` and `FxHashSet` for fast non-cryptographic hashing in internal routing data structures.

### Prefix-Sum Tables

Dense routing grids build summed-area tables for blocked cells and history costs. That makes rectangular queries O(1):

```text
sum(rect) = A + D - B - C
```

This matters because many primitive footprints are straight segments or full rectangles. Instead of checking every cell one by one, the router can reject or accept a segment with a small number of table lookups.

### Footprint Profiles

Each primitive footprint is classified once. If the footprint is a full rectangle or line segment, legality checks use the dense prefix tables. Only irregular footprints fall back to per-cell checks.

### Routing Windows

A* normally searches inside a window around the source and target, then grows the window if needed. This avoids allocating and scanning the full die for most nets. A full-grid fallback can still be enabled for difficult cases.

### Simple Route Fast Path

Straight, L, and Z routes are checked before A*. A successful simple route has zero expanded A* states and avoids heap traffic.

### Heading-Aware Heuristic

The default heuristic adds a conservative minimum bend lower bound to Euclidean distance. This keeps the heuristic admissible while reducing search in cases where the target position or target orientation cannot be reached without at least one bend.

### Heap Experiments

The default open set uses Rust's `BinaryHeap` with generation counters to skip stale duplicate entries. There is also an indexed heap mode that keeps one heap position per state. Current benchmark notes in `docs/profiling.md` keep the duplicate-entry heap as the default because it was faster in the tested photonic stress case despite more stale entries.

### Instrumented Optimization

The router reports counters for:

- expanded states
- generated neighbors
- heap pushes and pops
- stale heap entries
- dense storage bytes
- best-cost updates
- parent updates
- footprint checks
- rectangle footprint checks
- primitive class generated/accepted/rejected counts
- routing-window area
- full-grid fallbacks
- timing buckets

This makes speed work evidence-based.

## Build and Setup

Create or activate a Python environment with gdsfactory, maturin, pytest, and the normal scientific Python dependencies used by the benchmarks. The examples below use `python3`; if your virtual environment exposes `python`, that works as well.

Build the Rust extension into the Python package:

```bash
maturin develop --release
```

For a Rust-only build:

```bash
cargo build
```

The PyO3 module is configured in `pyproject.toml` as:

```text
photonic_router._rust
```

## Running the Flow

Run the default script configuration:

```bash
python3 routing_flow.py
```

Run a specific benchmark from Python:

```bash
python3 -c "from routing_flow import run_routing_flow; run_routing_flow('TOY', debug_svgs=True)"
```

Run with common options from the CLI:

```bash
python3 routing_flow.py mmi_heater_8x4_ripup_reroute \
  --include-heater-obstacles \
  --ripup-reroute \
  --debug-timing
```

Enable path-length matching:

```bash
python3 routing_flow.py mmi_heater_8x4 \
  --path-length-matching \
  --path-length-match-outputs \
  --include-heater-obstacles
```

Enable heater electrical routing:

```bash
python3 routing_flow.py heater_s \
  --electrical-routing \
  --include-heater-obstacles
```

Debug artifacts are written under `build/`, for example:

- `build/static_obstacles/*_obstacles.svg`
- `build/routes/*.svg`
- `build/routes/*_diagnostics.txt`
- `build/electrical/*.svg`
- `build/routed_<benchmark>.gds`

## Benchmarks and Profiling

End-to-end photonic benchmark:

```bash
python3 scripts/benchmark_photonic.py --include-heater-obstacles --ripup-reroute
```

Isolated Rust A* profiling:

```bash
python3 scripts/profile_astar.py
```

Electrical benchmark:

```bash
python3 scripts/benchmark_electrical.py
```

The latest checked-in photonic baseline is in `docs/photonic_baseline.md`. At the time of that snapshot, the end-to-end flow routed:

| Benchmark | Instances | Nets | Grid | Total s |
| --- | ---: | ---: | --- | ---: |
| `TOY` | 5 | 4 | 645x332 | 0.0591 |
| `mmi_heater` | 7 | 7 | 1805x292 | 0.1797 |
| `mmi_heater_8x4` | 61 | 78 | 13005x1252 | 1.0004 |

These numbers are a local baseline, not a general performance guarantee.

## Tests

Run the full Python test suite:

```bash
python3 -m pytest
```

Run Rust tests:

```bash
cargo test
```

Useful targeted tests:

```bash
python3 -m pytest tests/test_rust_backend_import.py -v
python3 -m pytest tests/test_routing_flow_stats.py -v
python3 -m pytest tests/test_route_rust_records.py -v
python3 -m pytest tests/test_electrical_routing.py -v
```

## Adding a Benchmark

Create `benchmarks/MY_DESIGN.py`:

```python
from gdsfactory.schematic import Schematic


def build_schematic() -> Schematic:
    schematic = Schematic()
    # Add instances, placements, and routes here.
    return schematic
```

Then run:

```bash
python3 -c "from routing_flow import run_routing_flow; run_routing_flow('MY_DESIGN', debug_svgs=True)"
```

## Current Limitations

- This is an active prototype, so APIs and defaults still move.
- The main router is grid-based and angle-quantized, not a continuous global optimizer.
- Crossing insertion is not the main feature of this repository at the moment.
- Some features depend on recent gdsfactory APIs such as all-angle Euler bends.
- The best results currently come from inspecting debug SVGs and profiling counters, then tuning router configuration per benchmark.

## Repository Map

```text
.
+-- routing_flow.py                  # End-to-end flow entry point
+-- benchmark_metadata.py            # Benchmark metadata for matching/timing
+-- benchmarks/                      # Python schematic benchmarks
+-- translation/                     # Python translation and routing orchestration
|   +-- route_rust.py                # Rust optical router bridge
|   +-- route_rust_meanders.py       # PLM/meander insertion
|   +-- electrical/                  # Heater metal routing
+-- python/photonic_router/          # Python package and helper APIs
+-- src/                             # Rust router, geometry, and PyO3 bindings
+-- scripts/                         # Benchmark and profiling entry points
+-- docs/                            # Profiling notes and baselines
+-- tests/                           # Python and Rust integration tests
+-- Agent_implementation_files/      # Architecture notes and implementation summaries
```
