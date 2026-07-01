# Functionality Comparison: TUMPhotonicRouter vs LiDAR 2.0

This note compares functionality only. It is based on the local LiDAR 2.0 paper
PDF at:

```text
/home/benjamin/Documents/Paper/Photonics/Lidar/2505.17239v2.pdf
```

## Short Answer

The current TUMPhotonicRouter implementation covers most of LiDAR 2.0's
detailed-routing kernel capabilities natively, especially the obstacle database,
grid/angle state routing, geometry checks, crossing constraints, rip-up/repair,
and GDS realization flow.

It does not yet implement LiDAR 2.0's main new hierarchy contribution:
hierarchical YAML module trees, bottom-up module routing, and reuse of routed
identical submodules through translation, rotation, or flipping.

There are also several features where TUMPhotonicRouter has the same broad goal
but a different or narrower implementation: port access spreading, port-group
routing order, crossing component insertion, and full critical-path insertion
loss optimization.

## Functionality Matrix

| LiDAR 2.0 functionality | Status here | TUMPhotonicRouter implementation / gap |
| --- | --- | --- |
| Schematic or netlist to routed GDS | Implemented | `routing_flow.py` loads benchmarks, builds gdsfactory layouts, routes, realizes geometry, and writes GDS/debug artifacts. |
| Curvy/non-Manhattan detailed routing | Implemented | Rust A* routes `(x, y, angle)` states with straight, 45-degree, and 90-degree primitive transitions. |
| Adaptive bend-radius-aware neighbor generation | Partial | Effective bend radius and bend-radius cells are configurable in Rust. The movement model is primitive-library based rather than LiDAR's exact adaptive-neighbor derivation. |
| Geometry-aware legality checking | Implemented | Primitive footprints, static/dynamic obstacle maps, compact rectangles, prefix occupancy, clearance cells, and route ownership are checked in Rust. |
| Static obstacle extraction from placed devices | Implemented | Python extracts layout geometry and Rust stores compact obstacle data; port openings are passed into the routing database. |
| Dynamic route database | Implemented | Committed routes, owners, core/clearance cells, rollback snapshots, rip-up, and history costs live in Rust. |
| Port access creation | Partial | Port openings, runways, lane offsets, and foreign-port keepouts are supported, but this is not LiDAR's exact port-propagation and staggered reservation algorithm. |
| Congested port spreading | Partial | Coincident or difficult port starts are handled with lane/runway mechanisms and route repair. There is no full LiDAR-style group spreading pass that redistributes ports at the device boundary. |
| Staggered port access reservation | Partial | TUMPhotonicRouter reserves/opens port approach regions, but does not yet reproduce LiDAR's formula-driven staggered mountain-shaped reservation based on port order, bend radius, and crossing size. |
| Port-group-based net ordering | Partial | TUMPhotonicRouter can use topology-derived crossing plans and local repair, but it does not yet have LiDAR's explicit port-group scheduler with inter-group and intra-group priority rules. |
| Routing order refinement after failures | Partial | Rip-up/reroute, rollback, blocker discovery, and history costs are implemented. LiDAR 2.0's specific failure-driven port-region/order refinement is not implemented as a named pass. |
| Insertion-loss-aware routing cost | Partial | Length, bend, and crossing-related costs exist in the routing stack. A full LiDAR-style `ILmax` critical-path objective with device, propagation, crossing, and bending loss tables is not yet the top-level optimizer. |
| Congestion/history penalties | Implemented | History costs and route repair discourage repeated use of congested or blocking regions. This is functionally similar to LiDAR's negotiation idea, but implemented in the Rust route database. |
| Automatic crossing handling | Partial | Expected crossing pairs, crossing reservations, geometry validation, and crossing diagnostics are implemented. Full physical crossing-device insertion into the realized GDS should still be treated as incomplete. |
| Crossing space preservation | Partial | The router reserves crossing windows, validates perpendicular/straight-margin requirements, tracks crossing blockers, and can add history around viable crossing regions. It does not implement LiDAR's exact `gcr` penalty formulation. |
| Crossing-waveguide optimization | Partial | TUMPhotonicRouter has crossing-aware search, expected-pair validation, repair, and diagnostics. It does not yet implement LiDAR's exact "reroute with crossings disabled, compare insertion loss, then keep lower-loss route" local replacement pass. |
| Rip-up and reroute | Implemented | Route commitment, victim rip-up, rollback, and reattempt logic are implemented in Rust/Python orchestration. |
| Redundant-bend elimination | Partial | Simple-route candidates, waypoint compression, port correction, and route realization reduce unnecessary geometry. There is no dedicated LiDAR-style redundant-bend elimination pass tied to hierarchical routing. |
| Port/grid alignment refinement | Partial | Route realization and port-access geometry handle endpoint alignment cases. LiDAR's final sine-bend correction is not reproduced exactly. |
| Variable bend radius support | Implemented | Bend radius can be configured and exposed through Rust helper methods. |
| Variable crossing size support | Partial | Crossing half-size and straight-margin constraints are used for reservations/validation. Full device-level crossing-size insertion is not complete. |
| Hierarchical netlist tree | Missing | No equivalent parser builds a hierarchy tree from nested YAML modules. |
| Bottom-up hierarchical routing | Missing | Routing is currently done at the benchmark/layout level, not by routing leaf modules, then progressively higher module levels. |
| Reuse of identical routed modules | Missing | There is no routed-module cache with translation/rotation/flipping reuse and DRC fallback. |
| Hierarchical YAML PIC intermediate representation | Missing / different | The project uses Python/gdsfactory schematic benchmarks and translation modules, not LiDAR 2.0's YAML IR. |
| Benchmark generators for TeMPO/GWOR/Benes/etc. | Partial | Benes and Clements-style benchmarks exist locally. The full LiDAR 2.0 YAML benchmark-generator ecosystem is not implemented. |
| Real-layout verification/debugging | Implemented | SVGs, crossing JSON/text, route diagnostics, verification helpers, and GDS output are part of the local flow. |
| Path-length matching and meanders | Implemented here; not the main LiDAR 2.0 claim | TUMPhotonicRouter has graph-based path-length analysis and Rust-backed analytic meander planning, which is broader than the LiDAR 2.0 comparison scope. |
| Heater/electrical routing | Implemented here; outside LiDAR 2.0 | The local repo includes heater electrical routing. This belongs to the LiDAR 3.0 comparison, not LiDAR 2.0 photonic routing. |

## Native Implementation Map

| Capability area | Main local files |
| --- | --- |
| End-to-end flow | `routing_flow.py`, `translation/layout_from_schematic.py`, `translation/route_rust.py` |
| Rust A* and primitives | `src/astar.rs`, `src/primitives.rs`, `src/simple_routes.rs` |
| Obstacle and dynamic routing database | `src/obstacle_map.rs`, `src/static_obstacle_builder.rs`, `python/photonic_router/static_obstacle_builder.py` |
| Crossing constraints and topology planning | `src/crossings.rs`, `src/py_router.rs`, `python/photonic_router/topology_analysis.py`, `python/photonic_router/crossing_plan.py` |
| Geometry realization and endpoint handling | `src/geometry_realization.rs`, `translation/route_rust.py` |
| Rip-up, repair, and history | `src/py_router.rs`, `src/obstacle_map.rs` |
| Path-length matching and meanders | `python/photonic_router/path_length_graph.py`, `translation/path_length_requirements.py`, `translation/route_rust_meanders.py`, `src/meander.rs` |
| Verification and diagnostics | `translation/photonic_verification.py`, `scripts/benchmark_photonic.py`, debug outputs under `build/` |

## Practical Interpretation

For a functionality claim, the defensible position is:

```text
TUMPhotonicRouter already implements most of the detailed-router machinery that
LiDAR 2.0 relies on, and several pieces are implemented more natively because
the hot routing database and search loop are in Rust. However, LiDAR 2.0's
hierarchical routing stack is not implemented here. Port-group ordering,
staggered access reservation, physical crossing-device insertion, and full
critical-path insertion-loss optimization are partial or different rather than
one-to-one matches.
```

So the answer to "apart from building hierarchies, do we mostly have everything
implemented natively?" is:

```text
Mostly yes for the detailed-routing kernel and repair database.
No for exact LiDAR 2.0 feature parity.
The biggest missing block is hierarchy; the next important gaps are exact
port-group scheduling/access reservation, physical crossing insertion, and
LiDAR-style ILmax optimization.
```

## Recommended Next Work For Closer LiDAR 2.0 Parity

| Priority | Work item | Why it matters |
| --- | --- | --- |
| 1 | Add a hierarchical module representation and bottom-up routing API | This is the central LiDAR 2.0 differentiator. |
| 2 | Add routed-module reuse with transform-aware DRC fallback | Enables LiDAR 2.0-style scalability on repeated subcircuits. |
| 3 | Finish physical crossing-device insertion in realized GDS | Converts current crossing reservations/diagnostics into complete layout devices. |
| 4 | Add a port-group scheduler and failure-driven order refinement | Closes a real functional gap in dense port regions. |
| 5 | Promote insertion-loss scoring to a top-level `ILmax` objective | Makes loss comparison against LiDAR 2.0 cleaner and paper-defensible. |
| 6 | Import or mirror the LiDAR 2.0 benchmark-generator set | Gives a fair same-benchmark functionality and quality comparison. |
