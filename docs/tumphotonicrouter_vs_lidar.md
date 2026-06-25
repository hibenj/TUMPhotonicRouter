# TUMPhotonicRouter vs LiDAR

This is a code-based comparison between this repository and [ScopeX-ASU/LiDAR](https://github.com/ScopeX-ASU/LiDAR). I inspected the LiDAR repository source, not only the paper/README claims.

The short version: LiDAR is a useful published PIC detailed-router baseline, but TUMPhotonicRouter is architecturally broader and has a stronger implementation foundation for the direction of this project: Rust kernels, detailed profiling, PLM, meanders, electrical routing, and future placement/routing co-optimization.

## Executive Takeaway

TUMPhotonicRouter is not just another Python A* router. It is becoming a full routing framework:

- schematic-to-layout orchestration
- Rust accelerated optical routing
- curvy/primitive-aware obstacle legality
- fast straight/L/Z route shortcuts
- rip-up/reroute with history costs
- physical gdsfactory realization
- path-length matching and meander insertion
- heater electrical routing
- detailed profiling and debug artifacts
- planned placer that reasons about crossings before routing

LiDAR's main current code-level advantage is that it already contains crossing insertion logic and published benchmark examples. For this project, that is not a fundamental architectural advantage: crossings are better handled earlier by placement and exposed to the router intentionally, instead of only being inserted reactively when a detailed-route neighbor hits an existing waveguide.

## Code Structure Comparison

| Area | TUMPhotonicRouter | LiDAR |
| --- | --- | --- |
| Core language | Rust routing kernels + Python orchestration | Python router |
| Backend module | PyO3 extension `photonic_router._rust` | No native routing backend found |
| Hot loop storage | Dense arrays, bitsets, prefix tables | Python objects, dicts, numpy object bitmap |
| Routing state | `(x, y, angle)` with 8 headings | `(x, y, orientation)` with 45-degree support |
| Neighbor model | Photonic primitives with explicit footprints and geometry | Step tables and DRC checks in Python |
| Simple fast paths | Straight, L, Z before A* | No equivalent source-level fast path found |
| Cost objective | Length, bend cost, history/congestion, physical route length | Propagation, bend, crossing, congestion weights |
| PLM/meanders | Integrated | Not found |
| Heater electrical routing | Integrated | Not found as a routing flow |
| Profiling | Per-attempt counters and timing buckets | Logging/evaluation output, less kernel instrumentation |
| Benchmark style | Python/gdsfactory schematic functions | YAML/netlist-style benchmark/config flow |
| Future direction | Placer + crossing-aware routing | Detailed router with reactive crossing insertion |

## What LiDAR Actually Does in Code

LiDAR's main detailed router is under:

- `/tmp/lidar-code/src/picroute/routing/astarsearch.py`
- `/tmp/lidar-code/src/picroute/routing/drgridroute.py`
- `/tmp/lidar-code/src/picroute/drc/drcmanager.py`
- `/tmp/lidar-code/src/picroute/drc/bitmap.py`
- `/tmp/lidar-code/src/picroute/queue/heapdict.py`

Important observations:

- A* nodes are Python `GridAstarNode` objects with `__slots__`.
- The open set is a custom Python `heapdict`.
- The global grid is a `numpy` array of Python `bitmapNode` objects.
- DRC and crossing checks are performed in Python with many branch-heavy per-cell checks.
- Neighbor transitions are stored in Python dictionaries such as `nextSteps`.
- Cost uses weighted propagation, bend, crossing, congestion, and history terms.
- Crossing insertion exists and is routed through DRC checks and post-processing that inserts `gf.components.crossing()`.
- The comp config sets `loss_crossing: 0`, while evaluation uses `il_cross`.
- I did not find an integrated PLM/meander routing stack.
- I did not find a Rust/native backend.
- I did not find a heater electrical routing flow comparable to this repository.

LiDAR is therefore a good algorithmic reference for crossing insertion and PIC-specific DRC checks, but its implementation is much more Python-object-heavy.

## What TUMPhotonicRouter Does Better

### 1. Faster Core Architecture

TUMPhotonicRouter puts the expensive router operations in Rust:

- A* expansion
- primitive legality checks
- dense obstacle map construction
- route commitment
- rip-up
- history costs
- route reconstruction
- meander geometry probing

This matters because routing spends most time in repeated local operations. Keeping those operations in Python makes every node expansion and DRC check pay interpreter/object overhead.

Relevant files:

- `src/astar.rs`
- `src/obstacle_map.rs`
- `src/primitives.rs`
- `src/geometry_realization.rs`
- `src/py_router.rs`

### 2. Better Low-Level Data Structures

TUMPhotonicRouter's A* state is indexed directly:

```text
idx = ((local_y * width) + local_x) * 8 + angle
```

That enables:

- `Vec<f64>` for costs
- `Vec<u32>` for parent indices and generations
- `Vec<u16>` for parent primitive IDs
- dense closed-state bitsets
- packed `u64` sparse cell keys
- fast `FxHashMap`/`FxHashSet` where sparse maps are still needed

LiDAR's bitmap is a `numpy` object array containing Python `bitmapNode` instances. That is convenient, but it is not a strong foundation for a high-performance detailed router.

### 3. Prefix-Sum Obstacle Checks

TUMPhotonicRouter builds summed-area tables for dense occupancy and history costs. Rectangular collision checks become:

```text
sum(rect) = A + D - B - C
```

This is a major difference. Many routing primitives are straight segments or rectangular footprints, so legality can be checked by table lookup instead of iterating through every touched cell.

LiDAR checks route legality through Python DRC methods such as `bViolateDRC()`, which repeatedly calls bitmap-node checks while walking a candidate step.

Relevant files:

- `src/astar.rs`
- `src/geometry_realization.rs`

### 4. Simple Routes Avoid A*

TUMPhotonicRouter checks deterministic routes before A*:

- straight
- L route
- Z route

If one is legal, the route finishes with zero A* expanded states. This is exactly the kind of pragmatic optimization that matters in large circuits because many nets are not hard enough to justify full search.

I did not find an equivalent straight/L/Z pre-route layer in LiDAR's source.

Relevant file:

- `src/simple_routes.rs`

### 5. Stronger Physical Framework

LiDAR is focused on optical detailed routing. TUMPhotonicRouter already reaches beyond that:

- optical waveguide routing
- path-length graph analysis
- local and output path-length matching
- analytic meander insertion
- meander obstacle registration
- heater electrical terminal extraction
- pad planning
- electrical bus/detail routing
- electrical verification

That makes this project closer to a full PIC layout automation framework, not only a waveguide router.

Relevant files:

- `translation/path_length_requirements.py`
- `translation/path_length_candidates.py`
- `translation/route_rust_meanders.py`
- `src/meander.rs`
- `src/geometry_realization.rs`
- `translation/electrical/`

### 6. Better Debuggability

TUMPhotonicRouter records detailed route-attempt data:

- expanded states
- generated neighbors
- heap pushes/pops
- stale generation skips
- closed-entry skips
- dense state count
- memory use
- footprint checks
- rectangle checks
- primitive-class reject counters
- routing-window areas
- full-grid fallbacks
- timing split by neighbor generation, heap operation, legality check, reconstruction

This is much better for router development than only checking whether a route succeeded.

Relevant files:

- `translation/route_rust_types.py`
- `translation/route_rust_records.py`
- `scripts/profile_astar.py`
- `scripts/benchmark_photonic.py`

### 7. Cleaner Python/Rust Boundary

TUMPhotonicRouter keeps a useful separation:

- Python owns gdsfactory, schematic loading, high-level orchestration, debug output, and final layout construction.
- Rust owns the route search, route database, collision acceleration, and geometry-heavy helpers.

This is the right split for a research router: high-level workflows remain easy to change, while performance-sensitive code is compiled and measurable.

LiDAR keeps nearly all routing behavior in Python. That is easier to start with, but it becomes limiting when the bottleneck is search and legality checking.

### 8. Cost Function Is Not a LiDAR Advantage

LiDAR exposes propagation, bending, crossing, congestion, and history costs. That sounds like an insertion-loss objective, but in practice the useful routing goals are:

- shorter waveguides
- fewer bends
- fewer crossings
- less congestion
- legal spacing

TUMPhotonicRouter already optimizes the same core physical drivers through:

- primitive length cost
- bend cost
- dynamic obstacle avoidance
- history cost during repair
- path-length analysis and matching after routing
- route realization diagnostics

Crossing count is not yet a first-class objective because the planned design is to handle crossing opportunities earlier through placement and intentional crossing reservation.

## Current LiDAR Advantages That Still Matter

After source inspection, the remaining LiDAR advantages are narrower than the public positioning suggests:

| LiDAR point | Why it matters | Why it is not a fundamental blocker here |
| --- | --- | --- |
| Existing crossing insertion | It can insert `gf.components.crossing()` during/post routing | This repo plans a placer that handles crossings before routing, which is a stronger long-term architecture |
| Published benchmark context | It has ISPD paper context and example benchmark sets | This repo can import/run LiDAR-style benchmarks later for direct comparison |
| Existing YAML benchmark/config flow | Useful for reproducible batch experiments | This repo's Python benchmarks are more flexible; YAML import can be added if needed |

I would not frame LiDAR as stronger overall for this project. It has a published optical-router baseline and crossing insertion implementation. TUMPhotonicRouter has the stronger framework architecture.

## Future Work: Placer Before Router

The most important next architectural step is a placer.

The placer should:

- position components with routing demand in mind
- estimate unavoidable crossings before detailed routing
- reserve intentional crossing regions
- expose crossing opportunities to the router as legal resources
- reduce crossings by placement before the router starts
- keep the router focused on detailed legal realization

This is better than treating crossing insertion only as a local repair when a route hits another route. A crossing-aware placer can reduce the problem before A* sees it.

## Core Functionality Comparison

These are framework-level capabilities, not just A* implementation details.

| Capability | TUMPhotonicRouter | LiDAR | Why this is better here |
| --- | --- | --- | --- |
| Native router backend | Rust/PyO3 backend behind Python orchestration | Python router | Keeps gdsfactory flexibility while compiling the expensive search, legality, commitment, rip-up, and geometry helper loops. |
| Full routing pipeline | Optical routing + PLM + meanders + heater electrical routing | Optical detailed routing | This repository is becoming a PIC routing framework, not only a single optical router. |
| Path-length matching + meanders | Integrated graph analysis and obstacle-aware analytic meander insertion | Not found | Routed designs can be corrected for arrival/edge length requirements instead of stopping at shortest legal routes. |
| Electrical heater routing | Heater terminal extraction, pad planning, metal realization, verification | Not found as a routing flow | Optical and heater-metal concerns can be handled in one layout pipeline. |
| Debug/profiling artifacts | SVGs, failed-route logs, route-attempt records, timing buckets, per-class counters | More limited logging/evaluation output | Failures and performance regressions can be diagnosed at the route-attempt and primitive-class level. |
| Crossing strategy | Planned placer + intentional crossing reservation before detailed routing | Reactive detailed-route crossing insertion | Handling crossings during placement can reduce conflicts before A* sees them, instead of discovering crossings only after route collision. |
| Benchmark publication | Local baselines today | Published ISPD 2025 baseline | LiDAR is stronger here today; this repository is set up to import those benchmarks and compare with deeper profiling. |

## Shared Router Features, Better Kernel

Both projects have grid-based A* concepts, oriented routing states, curvy/bend-aware movement, costs, DRC-style legality, history/congestion ideas, and rip-up/reroute behavior. The difference is how those ideas are implemented.

| Shared feature | TUMPhotonicRouter implementation | LiDAR implementation | Which is better here and why |
| --- | --- | --- | --- |
| A* runtime | Rust dense A* kernel | Python A* over `GridAstarNode` objects | TUMPhotonicRouter is better for speed because expansion, queue work, cost updates, legality checks, and reconstruction avoid interpreter overhead. |
| State storage | Direct dense index: `((local_y * width) + local_x) * 8 + angle` | Python node objects stored in dictionaries/heapdict | Dense arrays improve locality and avoid object allocation/hash lookup in the hot path. |
| Closed/open bookkeeping | Closed-state bitset, generation counters, optional indexed heap | Python custom `heapdict` and node flags | Bitsets and generation counters are cheaper and make stale-entry handling measurable. |
| Obstacle map | Sparse packed `u64` keys plus dense blocked bitsets | `numpy` object array of `bitmapNode` instances | Packed keys and bitsets are much more compact than per-cell Python objects. |
| Footprint legality | Primitive footprint profiles with fast rectangle/segment paths | Python DRC walks candidate cells through bitmap checks | TUMPhotonicRouter avoids many per-cell checks by recognizing simple footprints once. |
| Prefix-sum checks | Blocked-cell and history summed-area tables | No equivalent found | Rectangular/segment checks become O(1), which directly speeds repeated legality queries. |
| Routing windows | Configurable source-target search windows with growth and fallback | Net bounding regions | TUMPhotonicRouter exposes detailed window counters and can tune search area without losing fallback behavior. |
| Simple-route bypass | Straight, L, and Z candidates before A* | No equivalent found | Easy nets finish with zero A* expanded states, no heap traffic, and no broad search. |
| Heuristic | Distance or heading-aware lower bound with target-angle handling | Manhattan/custom heuristic with bend terms | Heading-aware mode keeps admissible distance behavior while adding unavoidable bend lower bounds. |
| Primitive ordering | Library, long-straight-first, target-biased experiments | Python `nextSteps` order | TUMPhotonicRouter can benchmark ordering changes without changing route semantics. |
| Cost drivers | Length, bend cost, dynamic conflicts, history cost, PLM validation | Propagation, bend, crossing, congestion weights | The same physical goals are covered, plus timing/path-length is analyzed and repaired after routing. |
| Rip-up/reroute | Probe route, blocker discovery, victim rip-up, history update, rollback | Failed-net rip-up and local crossing-net rip-up | TUMPhotonicRouter's repair is more instrumented and transactional. |
| Curvy/bend awareness | Primitive footprints checked against obstacles, then physically realized | Parametric bend/neighbor DRC in Python | Both are curvy-aware; TUMPhotonicRouter has the stronger compiled legality and realization path. |
| Diagnostics | Per-attempt counters for expansions, heap, rejects, footprint checks, dense memory, timing | Logs and final evaluations | TUMPhotonicRouter makes algorithmic improvements measurable instead of relying mainly on wall time or final success. |

## Bottom Line

LiDAR is a good reference for crossing-aware photonic detailed routing. But after reading the code, the implementation is mostly a Python A* router with object-grid DRC checks and crossing insertion logic.

TUMPhotonicRouter is a stronger foundation for a full PIC routing framework because it already combines a compiled routing kernel, richer diagnostics, PLM/meanders, electrical routing, and a path toward crossing-aware placement before detailed routing.

## Source Notes

LiDAR source inspected from:

- `src/picroute/routing/astarsearch.py`
- `src/picroute/routing/drgridroute.py`
- `src/picroute/drc/drcmanager.py`
- `src/picroute/drc/bitmap.py`
- `src/picroute/queue/heapdict.py`
- `src/picroute/config/default_config.yml`
- `src/picroute/config/comp_LiDAR.yml`

TUMPhotonicRouter source inspected from:

- `src/astar.rs`
- `src/simple_routes.rs`
- `src/obstacle_map.rs`
- `src/geometry_realization.rs`
- `src/py_router.rs`
- `translation/route_rust.py`
- `translation/route_rust_meanders.py`
- `translation/electrical/`
