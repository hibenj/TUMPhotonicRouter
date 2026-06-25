# TUMPhotonicRouter

A Rust + Python framework for automated photonic integrated circuit routing.

TUMPhotonicRouter takes a gdsfactory schematic, builds an unrouted layout, rasterizes obstacles, routes optical waveguides with a Rust A* backend, optionally performs path-length matching with meanders, and can route heater electrical metal in the same flow.

The project is currently an active research/prototype router, but the architecture is already broader than a standalone optical detailed router: it combines schematic translation, fast routing kernels, physical realization, PLM, electrical routing, diagnostics, and benchmarking.

## Highlights

| Area | What is implemented |
| --- | --- |
| Optical routing | 8-heading primitive A* with straight, 45-degree, and 90-degree moves |
| Curvy-aware legality | Bend/route footprints are checked against grid obstacles before acceptance |
| Fast paths | Deterministic straight, L, and Z routes before full A* |
| Rust acceleration | Dense state arrays, bitsets, prefix-sum obstacle tables, PyO3 bindings |
| Dynamic routing database | Static obstacles, committed routed nets, rip-up, reroute, history costs |
| Path-length matching | Graph analysis, missing-length requirements, analytic meander insertion |
| Electrical routing | Heater terminal extraction, pad planning, bus/detail routing, verification |
| Realization | Rust route records become gdsfactory polygons/components |
| Debugging | Obstacle SVGs, route SVGs, failed-route logs, timing and attempt counters |
| Benchmarks | Python/gdsfactory benchmarks plus profiling scripts |

## Flow

```text
Python benchmark schematic
        |
        v
gdsfactory unrouted layout
        |
        v
static obstacle grid + port openings
        |
        v
Rust optical routing
  - straight/L/Z fast path
  - primitive A*
  - rip-up/reroute repair
        |
        v
physical route realization
        |
        +--> optional PLM + meanders
        |
        +--> optional heater electrical routing
        |
        v
routed Component / GDS / debug artifacts
```

## Why This Router Is Fast

| Technique | Why it matters |
| --- | --- |
| Rust inner loop | A* expansion, legality checks, queue work, and reconstruction avoid Python overhead |
| Dense state index | `(x, y, angle)` maps to one array index, avoiding hash maps in the hot path |
| Bitsets | Closed states and dense blocked cells are compact and cache-friendly |
| Packed cell keys | Sparse grid cells are stored as reversible `u64` keys |
| Prefix-sum tables | Rectangular/segment obstacle checks become constant-time table queries |
| Footprint profiles | Straight and rectangular primitive footprints use fast prefix checks |
| Routing windows | A* searches near the source/target first, with controlled growth |
| Simple-route fast path | Direct, L, and Z routes avoid graph search entirely when legal |
| Heading-aware heuristic | Adds a conservative bend lower bound to the distance heuristic |
| Instrumentation | Every optimization can be measured with route-attempt counters |

The cost model targets the physical goals that matter in PIC routing: short waveguides, fewer bends, conflict avoidance, and congestion/history avoidance. Crossing-aware placement/routing is planned as part of the future placer work.

## Core Files

| File | Role |
| --- | --- |
| `routing_flow.py` | End-to-end benchmark, layout, optical, PLM, and electrical flow |
| `translation/route_rust.py` | Python bridge into the Rust optical router |
| `src/astar.rs` | Primitive A*, dense search storage, routing windows, JPS experiments |
| `src/simple_routes.rs` | Straight, L, and Z candidate routing |
| `src/obstacle_map.rs` | Static/dynamic obstacle database, rip-up, history costs |
| `src/primitives.rs` | Photonic movement primitives and footprints |
| `src/geometry_realization.rs` | Route polygons, port access, meander geometry |
| `translation/route_rust_meanders.py` | PLM/meander orchestration |
| `translation/electrical/` | Heater electrical routing stack |
| `scripts/benchmark_photonic.py` | End-to-end photonic benchmark runner |
| `scripts/profile_astar.py` | Isolated Rust A* profiling |

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

Run a benchmark with timing:

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

When debug SVGs are enabled, artifacts are written under `build/`:

| Output | Meaning |
| --- | --- |
| `build/static_obstacles/*_obstacles.svg` | Rasterized blocked/open grid cells |
| `build/routes/*.svg` | Per-net routed paths |
| `build/routes/*_FAILED.txt` | Failure diagnostics |
| `build/routes/*_diagnostics.txt` | Port/opening/occupancy details |
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

| Benchmark | Instances | Nets | Grid | Total s |
| --- | ---: | ---: | --- | ---: |
| `TOY` | 5 | 4 | 645x332 | 0.0591 |
| `mmi_heater` | 7 | 7 | 1805x292 | 0.1797 |
| `mmi_heater_8x4` | 61 | 78 | 13005x1252 | 1.0004 |

These are local baseline numbers, not general hardware-independent guarantees.

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
```

## Roadmap

- Import and run LiDAR-style benchmark suites for direct measurement.
- Add a placement stage that reasons about crossings before detailed routing.
- Use placement/routing co-optimization to reduce crossings before the router has to repair them.
- Extend crossing-aware routing once the placer can reserve and expose intentional crossing sites.
- Continue optimizing Rust kernels around dense occupancy, route windows, and meander planning.

## Related Notes

- `docs/tumphotonicrouter_vs_lidar.md` - code-based comparison with LiDAR.
- `docs/profiling.md` - profiling workflow and optimization notes.
- `Agent_implementation_files/ROUTING_FLOW_ARCHITECTURE.md` - older architecture notes.
