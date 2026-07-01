# TUMPhotonicRouter

TUMPhotonicRouter is a Rust + Python framework for automated photonic
integrated circuit routing. Python owns gdsfactory integration, benchmark
loading, layout realization, path-length matching, and electrical orchestration;
Rust owns the routing database, obstacle-map acceleration, A* search, crossing
constraints, rip-up/repair, and geometry-heavy planning kernels.

The project is an active research/prototype router, but it is already more than
a standalone optical detailed router: it contains schematic translation, optical
routing, topology-derived crossing support, path-length matching with meanders,
heater electrical routing, diagnostics, and benchmarking.

## Current Capabilities

| Area | Implemented behavior |
| --- | --- |
| Schematic flow | Load Python/gdsfactory benchmarks, build unrouted layouts, route and write GDS/debug artifacts |
| Optical routing | 8-heading primitive router with straight, 45-degree, and 90-degree transitions |
| Fast optical paths | Straight, L, Z, and turnaround candidates are tried before dense A* where legal |
| Obstacle handling | Rust static/dynamic obstacle database with compact rectangles, packed cell keys, dense bitsets, and prefix tables |
| Crossing support | Optional topology-derived crossing constraints, expected-pair validation, crossing reservations, and local crossing repair |
| Repair | Dynamic route commitment, rip-up/reroute, rollback, and history costs |
| Path-length matching | Graph analysis, missing-length requirements, analytic meander insertion, and Rust meander planners |
| Electrical routing | Heater terminal extraction, pad planning, bus/detail routing, and verification |
| Realization | Rust route records are converted into gdsfactory geometry/components |
| Diagnostics | Obstacle SVGs, route SVGs, crossing reports, failed-route logs, timing buckets, and per-attempt counters |

## Routing Flow

```text
Python benchmark schematic
        |
        v
gdsfactory unrouted layout
        |
        v
static obstacle grid, compact rectangles, and port openings
        |
        v
optional topology crossing plan
        |
        v
Rust optical routing
  - simple-route candidates
  - dense primitive A*
  - expected crossing search
  - rip-up/reroute repair
        |
        v
physical route realization
        |
        +--> optional path-length matching and meanders
        |
        +--> optional heater electrical routing
        |
        v
routed Component / GDS / debug artifacts
```

## Why It Is Fast

The main speed difference is not a single heuristic. It comes from keeping the
high-frequency routing operations in compiled, cache-friendly data structures.

| Technique | Effect |
| --- | --- |
| Rust hot path | A* expansion, primitive legality, route commitment, rip-up, history updates, and crossing checks avoid Python interpreter/object overhead |
| Direct state indexing | `(x, y, angle)` maps to dense arrays for cost, parent, generation, and closed-state data |
| Compact obstacle map | Static/dynamic cells use packed `u64` keys and dense occupancy bits instead of per-cell Python objects |
| Compact static rectangles | Bounding-box obstacles can stay as rectangles instead of being fully materialized into millions of cells |
| Prefix-sum occupancy | Segment and rectangular footprint checks become constant-time table queries inside routing windows |
| Primitive footprint profiles | Straight and rectangular footprints take the fast prefix path; only irregular footprints fall back to per-cell checks |
| Simple-route bypass | Easy nets finish without heap traffic or broad graph search |
| Routing windows | A* first searches a source-target window and only expands/falls back when needed |
| Topology-derived crossings | Expected crossing partners are precomputed from net topology, so crossing search and repair can focus on valid pairs instead of discovering arbitrary overlaps |
| Native repair database | Committed routes, owners, rollback snapshots, history costs, and crossing reservations are maintained in Rust |

This design makes obstacle queries and bookkeeping cheap enough that the router
can spend its time on actual search decisions. The checked-in baseline numbers
are in `docs/photonic_baseline.md`; they are local measurements, not portable
hardware-independent guarantees.

## Important Files

| File | Role |
| --- | --- |
| `routing_flow.py` | End-to-end benchmark, layout, optical, PLM, crossing, and electrical flow |
| `translation/route_rust.py` | Python bridge into the Rust optical router and crossing context builder |
| `python/photonic_router/static_obstacle_builder.py` | Static obstacle extraction, Rust fallback handling, compact bbox payloads |
| `python/photonic_router/topology_analysis.py` | Depth/rank analysis used for crossing-aware routing |
| `python/photonic_router/crossing_plan.py` | Converts topology rank inversions into ordered crossing events |
| `src/astar.rs` | Dense primitive A*, routing windows, prefix occupancy, and search counters |
| `src/obstacle_map.rs` | Static/dynamic route database, packed cells, rip-up, history costs |
| `src/crossings.rs` | Crossing constraints and expected-pair context |
| `src/py_router.rs` | PyO3 router API, route batch/repair logic, crossing repair, meander helpers |
| `src/simple_routes.rs` | Deterministic straight/L/Z/turnaround route candidates |
| `src/primitives.rs` | Photonic movement primitives and footprint metadata |
| `src/geometry_realization.rs` | Route polygons, port access, and meander geometry |
| `translation/electrical/` | Heater electrical routing stack |
| `scripts/benchmark_photonic.py` | End-to-end photonic benchmark runner |
| `scripts/profile_astar.py` | Isolated Rust A* profiler |

## Build

```bash
maturin develop --release
```

This builds the Rust extension as:

```text
photonic_router._rust
```

Rust-only check:

```bash
cargo build
```

## Run

Default flow:

```bash
python3 routing_flow.py
```

Run a benchmark with crossings and timing:

```bash
python3 routing_flow.py benes_16x16 \
  --crossings \
  --debug-timing
```

Run a heater-obstacle rip-up benchmark:

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

## Debug Output

When debug SVGs or crossing diagnostics are enabled, artifacts are written under
`build/`:

| Output | Meaning |
| --- | --- |
| `build/static_obstacles/*_obstacles.svg` | Rasterized/compact obstacle view and port openings |
| `build/routes/*.svg` | Per-net routed paths |
| `build/routes/*_FAILED.txt` | Failure diagnostics |
| `build/routes/*_diagnostics.txt` | Port/opening/occupancy details |
| `build/crossings/*_crossings.json` | Expected and realized crossing metadata |
| `build/crossings/*_crossings.txt` | Human-readable crossing plan and realization summary |
| `build/electrical/*.svg` | Electrical routing snapshots |
| `build/routed_<benchmark>.gds` | Final routed layout |

## Benchmarking

End-to-end photonic benchmark:

```bash
python3 scripts/benchmark_photonic.py --include-heater-obstacles --ripup-reroute
```

Isolated Rust A* profiler:

```bash
python3 scripts/profile_astar.py
```

Electrical benchmark:

```bash
python3 scripts/benchmark_electrical.py
```

Current checked-in baseline: `docs/photonic_baseline.md`.

## Tests

```bash
python3 -m pytest
cargo test
```

Useful targeted checks:

```bash
python3 -m pytest tests/test_rust_backend_import.py -v
python3 -m pytest tests/test_routing_flow_stats.py -v
python3 -m pytest tests/test_route_rust_records.py -v
python3 -m pytest tests/test_electrical_routing.py -v
cargo test crossing
```

## Related Notes

- `docs/tumphotonicrouter_vs_lidar.md` - code-based comparison with LiDAR.
- `docs/tumphotonicrouter_vs_lidar2_functionality.md` - functionality-only
  comparison with the local LiDAR 2.0 paper.
- `docs/profiling.md` - profiling workflow and optimization notes.
- `docs/photonic_baseline.md` - local baseline timing snapshot.
- `Agent_implementation_files/ROUTING_FLOW_ARCHITECTURE.md` - older architecture notes.
