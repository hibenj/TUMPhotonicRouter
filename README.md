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

The full module map, with one paragraph per module and the interfaces between
them, is [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md); every configuration
field is in [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md).

| File | Role |
| --- | --- |
| `docs/ARCHITECTURE.md` | The module map, the stage sequence, the search and negotiation interfaces |
| `python/photonic_router/flow.py` | The flow: `route_benchmark(config, options)` and `route_schematic` |
| `python/photonic_router/cli.py` | The command line, including `--configuration` (`routing_flow.py` is a shim over it) |
| `python/photonic_router/config.py` | `RoutingConfig` / `RouterConfig`, the typed configuration trees |
| `translation/routing/` | The routing session as nine phases (`session.py::run`) behind the Protocols of `stages.py` |
| `python/photonic_router/static_obstacle_builder.py` | Static obstacle extraction, Rust fallback handling, compact bbox payloads |
| `python/photonic_router/topology_analysis.py` | Depth/rank analysis used for crossing-aware routing |
| `python/photonic_router/crossing_plan.py` | Converts topology rank inversions into ordered crossing events |
| `src/search/` | One net: the `NetSearch` interface, the A* engine (`astar/`), the Dijkstra oracle |
| `src/engine/` | Many nets: jobs, commitment, crossing reservations, endpoint correction |
| `src/engine/negotiation/` | The rip-up-and-repair loop and its budget / crossing-free / rip-up / queue policies |
| `src/bindings/` | The PyO3 surface: the `#[pymethods]` block, the config/result types, the converters |
| `src/obstacle_map.rs` | Static/dynamic route database, packed cells, rip-up, history costs |
| `src/crossings.rs` | Crossing constraints and expected-pair context |
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

The entry point is `python -m photonic_router route <benchmark> [flags]`.
`--configuration` names one of the three configurations the paper compares;
exactly one of them runs at a time.

```bash
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration baseline
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration contribution1
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration contribution2
```

A flag after `--configuration` overrides it, so the configurations are
starting points, not modes:

```bash
.venv/bin/python -m photonic_router route benes_16x16_flat \
  --configuration contribution1 \
  --debug-timing
```

Enable path-length matching:

```bash
.venv/bin/python -m photonic_router route heater_s_mod \
  --path-length-matching \
  --path-length-match-outputs \
  --include-heater-obstacles
```

Enable heater electrical routing:

```bash
.venv/bin/python -m photonic_router route heater_s_mod \
  --electrical-routing \
  --include-heater-obstacles
```

The older script form still works and is what the reproduction scripts and the
benchmark stable blocks use, because `routing_flow.py` is a shim over the same
command line:

```bash
.venv/bin/python routing_flow.py benes_8x8_flat --crossing-mode lidar-pure
```

Every flag, with its default and the `RoutingConfig` / `FlowOptions` field it
sets, is in [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md).

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
.venv/bin/python -m pytest -q tests                 # everything
.venv/bin/python -m pytest -q -m "not e2e" tests    # without the benchmark-loading tests
cargo test --release --lib
```

The exact toolchain overrides this host needs (`RUSTUP_TOOLCHAIN`,
`PYO3_PYTHON`), the pinned baseline script and the reproduction gate are in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) section 8.

Useful targeted checks:

```bash
.venv/bin/python -m pytest tests/test_rust_backend_import.py -v
.venv/bin/python -m pytest tests/test_routing_stages.py -v
.venv/bin/python -m pytest tests/test_route_rust_records.py -v
.venv/bin/python -m pytest tests/test_electrical_routing.py -v
.venv/bin/python -m pytest tests/e2e/test_routing_flow_stats.py -v
cargo test crossing
```

## Related Notes

- `docs/ARCHITECTURE.md` - the module map, the flow, and the interfaces.
- `docs/CONFIGURATION.md` - every configuration field, generated from the
  dataclasses by `scripts/generate_config_reference.py`.
- `docs/DATE2027_REPRODUCTION.md` - the paper's rows, their provenance, and how
  to reproduce them.
- `docs/repository_finished_state.md` - focused finished-state target for
  crossings and PLM.
- `docs/tumphotonicrouter_vs_lidar.md` - code-based comparison with LiDAR.
- `docs/tumphotonicrouter_vs_lidar2_functionality.md` - functionality-only
  comparison with the local LiDAR 2.0 paper.
- `docs/profiling.md` - profiling workflow and optimization notes.
- `docs/photonic_baseline.md` - local baseline timing snapshot.
- `Agent_implementation_files/ROUTING_FLOW_ARCHITECTURE.md` - older architecture notes.
