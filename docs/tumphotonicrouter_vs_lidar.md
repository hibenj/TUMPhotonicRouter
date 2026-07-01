# TUMPhotonicRouter vs LiDAR 2.0 / LiDAR 3.0

This is a code-based comparison between this repository and
[ScopeX-ASU/LiDAR](https://github.com/ScopeX-ASU/LiDAR), plus a paper-level
comparison against LiDAR 3.0 for electrical routing:

- LiDAR / LiDAR 2.0: photonic waveguide detailed routing.
- LiDAR 3.0: photonics-aware automated electrical routing for active PICs,
  described in the ISPD 2026 paper
  [LiDAR 3.0: Photonics-Aware Planning-Guided Automated Electrical Routing for Large-Scale Active Photonic Integrated Circuits](https://dl.acm.org/doi/10.1145/3764386.3779589).

The conclusion is split accordingly. For optical/photonic routing,
TUMPhotonicRouter has the stronger implementation foundation for fast,
extensible routing. For electrical routing, LiDAR 3.0 appears ahead right now:
this repository has a useful heater-routing prototype, but not yet the same
paper-level planning-guided electrical router.

## Executive Takeaway

For photonic waveguide routing, TUMPhotonicRouter is faster by design because
its bottlenecks are not Python object operations. It keeps routing state,
obstacle data, legality checks, route commitment, rip-up, crossing repair, and
many geometry queries inside Rust data structures.

The two most important speed reasons are:

- Obstacle handling is compiled and compact: static/dynamic obstacles use
  packed cell keys, dense occupancy bits, compact rectangles, and prefix tables.
- Crossing work is constrained by topology: expected crossing pairs are derived
  before detailed routing, so the router can focus search/repair on valid
  partner nets instead of treating every collision as a generic local surprise.

Against LiDAR 2.0, this repository's photonic-routing advantages are the Rust
routing kernel, stronger obstacle-map representation, topology-derived crossing
constraints, richer repair/profiling, PLM/meanders, and a cleaner path toward
larger integrated flows.

Against LiDAR 3.0, the electrical-routing comparison is different. LiDAR 3.0 is
specifically about large-scale active-PIC electrical routing with
photonics-aware planning. TUMPhotonicRouter currently has heater electrical
routing, pad planning, realization, and verification, but it is narrower and
less mature.

## Photonic Routing: vs LiDAR 2.0

This section is the photonic/waveguide comparison. It should be read against
LiDAR and LiDAR 2.0, not LiDAR 3.0's electrical-routing paper.

| Area | TUMPhotonicRouter | LiDAR 2.0 |
| --- | --- | --- |
| Core implementation | Python orchestration with Rust/PyO3 routing kernels | Python routing implementation |
| A* state | Dense indexed `(x, y, angle)` arrays | Python `GridAstarNode` objects |
| Queue/bookkeeping | Rust heap, generation counters, dense closed-state bitsets | Python custom heap/dict structures |
| Static obstacles | Rust builder, compact bbox rectangles, optional materialized cells | Python/NumPy bitmap object grid |
| Dynamic obstacles | Rust route database with owners, core/clearance cells, rip-up, rollback | Python bitmap/DRC state |
| Legality checks | Primitive footprints, prefix-sum rectangle/segment checks, fallback per-cell checks | Python DRC walks and bitmap-node checks |
| Simple routes | Straight, L, Z, and turnaround candidates before A* | No equivalent pre-A* layer found in inspected source |
| Crossing strategy | Topology-derived expected pairs, Rust crossing search, reservations, validation, local repair | Reactive detailed-route crossing insertion |
| Crossing realization | Crossing reservations and realized-overlap diagnostics; crossing component size used for keepout | Inserts `gf.components.crossing()` in the routing/post-processing flow |
| Repair | Transactional route attempts, blocker discovery, victim rip-up, rollback, history costs | Rip-up/reroute behavior tied to Python route/DRC logic |
| PLM/meanders | Integrated graph analysis and Rust-backed analytic meander planning | Not found as an integrated routing stack |
| Diagnostics | Per-attempt counters, timing buckets, primitive-class counters, SVGs, crossing JSON/text | Logging/evaluation output, less kernel-level instrumentation |
| Benchmarks | Native Python/gdsfactory benchmarks plus imported LiDAR-style Clements benchmark | Published benchmark/config flow |

## Why TUMPhotonicRouter Scales Better

### Obstacle Map And Legality

LiDAR's inspected routing path keeps much of the detailed-router state in
Python objects: A* nodes, heap entries, DRC calls, and a NumPy array of
`bitmapNode` cells. That structure is convenient, but expensive in the hot loop.
Every candidate expansion can involve Python dispatch, object access, and
branch-heavy DRC logic.

TUMPhotonicRouter uses a different layout:

| Mechanism | Why it is faster |
| --- | --- |
| Packed `u64` cell keys | Sparse cells can be stored and compared cheaply without tuple/object overhead |
| Dense occupancy bits | Common blocked/free checks become cache-friendly bit lookups |
| Compact static rectangles | Bounding-box obstacles do not have to be expanded into large cell sets before handoff |
| Summed-area tables | Horizontal, vertical, and rectangular footprint checks become O(1) prefix queries |
| Primitive profiles | Regular footprints take the prefix path; irregular footprints are the exception |
| Rust route database | Commit, rip-up, owner lookup, core/clearance overlap rules, and history costs stay native |

This matters because detailed routing is dominated by repeated local queries:
"is this primitive legal?", "which route owns this blocked region?", "can this
reservation be opened?", and "what history cost applies here?". Moving those
queries out of Python is a structural speedup, not just an implementation tweak.

Relevant TUMPhotonicRouter files:

- `src/obstacle_map.rs`
- `src/static_obstacle_builder.rs`
- `src/astar.rs`
- `src/primitives.rs`
- `python/photonic_router/static_obstacle_builder.py`

### Search State

TUMPhotonicRouter maps each routed state directly to a dense index:

```text
idx = ((local_y * width) + local_x) * 8 + angle
```

That enables compact `Vec` storage for costs, parents, primitive IDs,
generations, and closed-state bookkeeping. LiDAR's source uses Python node
objects and heap/dict structures for comparable state. On large grids, dense
native arrays have much better memory locality and much lower per-expansion
overhead.

### Fast Path Before A*

Many photonic nets do not need full graph search. TUMPhotonicRouter checks
deterministic route candidates first, including straight, L, Z, and turnaround
variants. If one is legal, the route completes without heap traffic and without
expanding A* states.

This is especially important in larger circuits: the hard nets should pay for
A*, but easy nets should not.

Relevant file:

- `src/simple_routes.rs`

## Crossing Handling

TUMPhotonicRouter now has topology-derived crossing support:

1. Benchmarks can expose node depths, node ranks, and edge ranks.
2. Python analyzes rank inversions between topology stages.
3. A crossing plan turns those inversions into ordered expected crossing events.
4. Python passes compact `CrossingConstraint` records into Rust.
5. Rust only allows/repairs core overlaps that match the expected partner set
   when strict expected-pair mode is active.
6. The router reserves crossing windows, rejects invalid geometry such as
   non-perpendicular or insufficient-straight crossings, and writes crossing
   debug artifacts.
7. The current implementation also adds spacing history around viable crossing
   windows and preemptive local rip-up for expected crossing partners when
   history shows an existing route is blocking the likely crossing region.

This is different from LiDAR's reactive insertion model. LiDAR can insert a
crossing component when detailed routing finds a crossing opportunity. This
repository instead precomputes which nets are supposed to cross from topology,
then uses that information to guide search, validation, and repair.

That topology-first approach is faster and cleaner for structured PIC networks
because the router does not have to rediscover the crossing graph from local
collisions. It already knows the valid partner set and can reject unrelated
overlaps early.

Relevant TUMPhotonicRouter files:

- `python/photonic_router/topology_analysis.py`
- `python/photonic_router/crossing_plan.py`
- `src/crossings.rs`
- `src/astar.rs`
- `src/py_router.rs`
- `translation/route_rust.py`
- `benchmarks/benes.py`

## Electrical Routing: vs LiDAR 3.0

LiDAR 3.0 changes the electrical-routing comparison. The paper title and public
metadata describe a photonics-aware, planning-guided automated electrical router
for large-scale active PICs. That is a more ambitious and probably stronger
electrical-routing contribution than the current TUMPhotonicRouter electrical
stack.

TUMPhotonicRouter's electrical routing is still valuable: it extracts heater
terminals, builds an electrical obstacle grid, routes a common bus, plans pad
slots, routes individual terminals, realizes metal, and verifies the result.
But it is currently a milestone implementation around heater access and pad
escape, not a full LiDAR 3.0-class electrical router.

| Electrical-routing area | TUMPhotonicRouter today | LiDAR 3.0 paper direction | Current position |
| --- | --- | --- | --- |
| Scope | Heater terminal extraction, common bus, pad slots, individual escape, metal realization | Large-scale active-PIC electrical routing | LiDAR 3.0 is broader |
| Planning level | Local/common-bus topology plus pad-side escape planning | Photonics-aware planning-guided electrical routing | LiDAR 3.0 is likely stronger |
| Optical awareness | Uses extracted layout obstacles and heater/metal obstacle layers in one local flow | Paper-level framing targets waveguide-aware electrical routing around photonic constraints | LiDAR 3.0 is likely stronger |
| Congestion/DRC | Grid obstacles, clearances, pad spacing, verification, debug SVGs | Public descriptions emphasize congestion/DRC-aware planning and guidance-driven detailed routing | LiDAR 3.0 is likely stronger |
| Multi-net electrical strategy | Common-bus plus one-sided individual terminal routing | Large-scale metal-wire routing for active PICs | LiDAR 3.0 is likely stronger |
| Implementation maturity | Prototype-quality but inspectable code in `translation/electrical/` | Published ISPD 2026 paper result | LiDAR 3.0 is more mature |
| Debuggability | Strong local SVG snapshots and verification objects | Paper-level comparison only here; source not inspected in this repo pass | Unclear without source |
| Integration with this repo | Already callable from `routing_flow.py` and shares local benchmark/debug infrastructure | External LiDAR 3.0 flow | TUMPhotonicRouter is easier to evolve inside this codebase |

The fair summary: TUMPhotonicRouter is stronger today for the Rust-backed
photonic routing kernel, but weaker today for electrical routing if compared to
LiDAR 3.0's stated scope.

## Framework Scope

| Capability | Status here |
| --- | --- |
| Schematic-to-layout orchestration | Implemented in `routing_flow.py` and translation modules |
| Optical primitive routing | Rust A* and simple-route candidates |
| Path-length matching | Graph analysis, output matching, analytic meander insertion |
| Meander planning | Rust-backed candidate/sequence/split/final planners |
| Heater electrical routing | Terminal extraction, pad planning, detailed routing, verification |
| Benchmark diagnostics | Markdown/CSV/JSON reports and route-attempt records |
| LiDAR-style benchmark import | `benchmarks/clements_8x8.py` exists as a converted Clements benchmark |

This broader scope matters because PIC layout automation is not only shortest
legal waveguides. Timing/path length, heater metal, pad escape, and debugging
all affect whether a routed layout is usable.

## LiDAR Advantages That Still Matter

| LiDAR advantage | Practical meaning |
| --- | --- |
| Published baseline | Easier to cite and compare against in papers |
| Existing crossing-device insertion | It has a mature flow for placing `gf.components.crossing()` as part of routing/post-processing |
| LiDAR 3.0 electrical-routing paper | Stronger current position for large-scale active-PIC electrical routing |
| Benchmark ecosystem | Its YAML/config benchmarks are useful for reproducibility |
| Algorithmic reference value | Its crossing and DRC logic remain useful to study |

These are real advantages, but they are not the same as having a stronger
photonic-routing performance foundation. For waveguide routing speed,
TUMPhotonicRouter's Rust obstacle/search kernel is the more scalable
architecture.

## Claim Boundaries

- The performance argument here is code-structure based. Use
  `docs/photonic_baseline.md`, `scripts/benchmark_photonic.py`, and
  `scripts/profile_astar.py` for measured local numbers.
- The comparison should not be read as a final quality benchmark until the same
  benchmark suite is run through both projects on the same machine and process.
- TUMPhotonicRouter currently has crossing constraints/reservations and
  realization diagnostics; full physical crossing-device placement should be
  described separately when that implementation is complete.
- The LiDAR 3.0 electrical-routing section is paper-level. I did not inspect
  LiDAR 3.0 source code in this repository pass.

## Source Notes

LiDAR source areas inspected:

- `src/picroute/routing/astarsearch.py`
- `src/picroute/routing/drgridroute.py`
- `src/picroute/drc/drcmanager.py`
- `src/picroute/drc/bitmap.py`
- `src/picroute/queue/heapdict.py`
- `src/picroute/config/default_config.yml`
- `src/picroute/config/comp_LiDAR.yml`

LiDAR 3.0 paper/source note:

- DOI: `10.1145/3764386.3779589`
- ACM page: `https://dl.acm.org/doi/10.1145/3764386.3779589`
- PDF link supplied by the user: `https://dl.acm.org/doi/pdf/10.1145/3764386.3779589`
- Used here as a paper-level electrical-routing reference, not as inspected
  source code.

TUMPhotonicRouter source areas inspected:

- `routing_flow.py`
- `translation/route_rust.py`
- `python/photonic_router/static_obstacle_builder.py`
- `python/photonic_router/topology_analysis.py`
- `python/photonic_router/crossing_plan.py`
- `src/astar.rs`
- `src/simple_routes.rs`
- `src/obstacle_map.rs`
- `src/static_obstacle_builder.rs`
- `src/crossings.rs`
- `src/py_router.rs`
- `translation/route_rust_meanders.py`
- `translation/electrical/`
