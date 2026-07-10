# Repository Finished-State Target

This document defines the intended finished state for the current research
direction of TUMPhotonicRouter. It is not a full project roadmap. The broader
repository goal is a very fast Rust/Python photonic router that can run
benchmark designs end to end and verify that the final physical geometry is
correct. This document narrows the near-term goal to photonic crossings and
path-length matching (PLM).

Metal/electrical routing is intentionally out of scope for this phase. The
existing electrical code should remain in the repository, but comparisons,
architecture decisions, and success criteria here should not depend on it.

## Focus

The repository should be evaluated as a photonic routing framework with two
core themes:

1. Router-discovered crossing-aware optical routing that can run the current
   crossing benchmark targets.
2. PLM that can insert delay using multiple physical structures, not only the
   current bump-meander structure.

LiDAR can provide guidance for the concept of crossings being explored by the
router during search. This repository is not trying to reimplement LiDAR. The
target is a faster Rust-backed router with its own architecture, strong
verification, and reproducible benchmark evidence. LiDAR 3.0 and metal routing
are separate topics and should not drive this phase.

## Finished-State Summary

When this phase is finished, the repository should support:

| Area | Finished-state expectation |
| --- | --- |
| Benchmark coverage | Run current crossing targets first: `benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`; expand only after these have reliable verification evidence. |
| Router-discovered crossing mode | Route with crossings explored during search, using legal crossing geometry and cost tradeoffs instead of requiring precomputed expected crossing pairs. |
| Verification | Report whether final geometry is correct, including legal crossings, illegal bend/near-bend intersections, unexpected intersections, self-intersections, and realization or port-snapping mismatches. |
| Pipeline integrity | Preserve crossing decisions through path-length matching, port snapping, and geometry realization. Port snapping may adjust only endpoint-to-first-crossing and last-crossing-to-endpoint access geometry. |
| PDK crossing realization | Use the active PDK/gdsfactory crossing component for physical crossing insertion; its footprint drives keepout, straight-access, spacing, and verification checks. |
| PLM | Preserve existing path-length-matching regression behavior and grow toward multiple delay structures when crossing correctness is stable. |
| Documentation | Keep the comparison clear: LiDAR is a reference for router-explored crossings, not the implementation target. |

## Immediate Implementation Priority

For the next implementation agent, crossings come first. PLM remains part of
the finished-state target, but it should not be the first work item unless
crossing routing is already stable on the benchmark set.

The immediate crossing milestone is:

| Step | Goal | Done when |
| --- | --- | --- |
| 1 | Stabilize final-geometry verification for crossings | The harness classifies legal crossings, illegal intersections, self-intersections, realization mismatches, and port-snapping protected-segment changes. |
| 2 | Stabilize router-discovered crossings on `benes_4x4` and `benes_8x8` | Both benchmarks have exact commands, debug artifacts, and verification results showing legal final geometry or classified failures. |
| 3 | Resume `multiportmmi_8x8` crossing debugging | Failures are classified by the same harness instead of inferred from screenshots or partial traces. |
| 4 | Preserve existing PLM regressions | Heater/path-length-matching tests such as `heater_s_mod` remain passing or any failure is documented and fixed. |
| 5 | Add route-induced insertion-loss accounting and broader benchmark reports | Reports contain length, bend penalty, crossing count, route-induced IL, runtime, and verification status. |

Do not start by extending metal routing. Do not start by building hierarchy
unless crossing benchmark coverage is blocked without it.

### Immediate Crossing Correctness Blocker

Before adding physical crossing-cell insertion or expanding to more benchmarks,
fix the illegal crossing geometry currently visible in `benes_16x16`: some nets
still cross other waveguides at or near bends instead of at a valid
straight-straight crossing window. This is a first-order design-rule issue, not
only an SVG/debug artifact.

The next implementation pass should:

| Work item | Done when |
| --- | --- |
| Detect all geometric intersections after routing and endpoint correction | Every route/route intersection is classified as expected legal crossing, illegal unexpected crossing, or same-net/contact-adjacent geometry. |
| Reject bend or near-bend crossings | A crossing is legal only when both involved realized centerlines have sufficient straight access before and after the crossing point. |
| Keep logical crossing reservations aligned with realized geometry | The grey crossing footprint in debug SVG/GDS corresponds to the actual straight-straight route intersection, not a nearby bend or shifted path segment. |
| Repair illegal realized crossings | If post-route verification finds an illegal bend crossing, local ripup/reroute should choose a blocking route and re-route until the crossing is legal or the benchmark fails explicitly. |

## Crossing Goal

Crossing support should become the main near-term feature. The primary crossing
mode for this phase is router-discovered: the search itself explores legal
crossing candidates and chooses them when they improve the route. Topology
precomputation may remain useful later, but it is not the current functional
goal.

The current `lidar-pure` / router-discovered path must not use
topology-precomputed crossing hints. Topology-precomputed crossings can remain
as a later separately selectable comparison or optimization mode.

### Primary Variant: Router-Discovered Loss-Guided Crossings

This mode should discover crossings during detailed routing. LiDAR is relevant
because it demonstrates this kind of crossing decision, not because this router
must copy LiDAR's queueing, ripup, or implementation structure.

Expected behavior:

| Requirement | Meaning |
| --- | --- |
| Insertion-loss cost | A* route selection should explicitly combine path length, bend loss, and crossing loss. Device loss can be included in reports, but route selection between fixed endpoints is mainly driven by physical route-induced loss from length, bends, and crossings. |
| Crossing as a routing decision | The router should not assume an expected crossing pair from topology. It should discover legal crossing opportunities during detailed routing. |
| Competing alternatives | The router should try many legal non-crossing and crossing alternatives before accepting a crossing, because crossings are costly and should not be inserted only because they are locally convenient. |
| Legal crossing geometry | Crossings must satisfy perpendicularity, straight access margin, footprint/keepout, layer/type compatibility where applicable, and spacing to other crossings or obstacles. |
| Physical crossing component | Accepted crossings must be realized with the active PDK/gdsfactory crossing component, not only as route-record metadata or debug marks. |
| Loss reporting | Reports should break out length, bend count or bend-angle penalty, crossing count, and resulting route-induced insertion loss. |
| Benchmark role | This mode is the main current crossing mode for `benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`. |

This mode should answer: "Can this Rust-backed router quickly discover legal
crossings and prove the final geometry is correct?"

### Possible Later Variant: Topology-Precomputed Crossings

This remains a possible optimization or comparison mode. It uses network
topology to precompute which nets are expected to cross, then guides detailed
routing with that information. Do not prioritize this mode until the
router-discovered crossing mode has stable verification on the current
benchmarks.

Expected behavior:

| Requirement | Meaning |
| --- | --- |
| Topology crossing plan | Python analyzes graph depth/rank/order and emits expected crossing constraints before detailed routing. |
| Expected-pair validation | Rust should allow crossing overlaps only when they match the expected partner set in strict mode. |
| Crossing reservation | The router should reserve crossing windows and reject invalid crossing locations early. |
| Local repair | If an expected crossing is blocked by its intended partner, the router can rip up or locally repair the partner instead of treating the conflict as an arbitrary obstacle. |
| Speed goal | This mode should be faster than router-discovered crossing search on structured crossing-heavy benchmarks because it avoids discovering the crossing graph from local collisions. |
| Quality goal | The mode should keep insertion loss competitive with router-discovered crossing mode; any extra crossings or detours must be visible in the report. |

This mode should answer: "How much speed do we gain when the crossing structure
is known from topology before detailed routing?"

## Crossing Benchmarks

The finished repository should be able to run the benchmark families used by
LiDAR 2.0, or locally equivalent converted versions.

Required benchmark work:

| Work item | Finished-state expectation |
| --- | --- |
| Benchmark inventory | Maintain a list of LiDAR 2.0 benchmarks and their local names/status. |
| Converted inputs | Provide local Python/gdsfactory or importer-backed equivalents for each benchmark family used in the LiDAR 2.0 paper. |
| Crossing-heavy cases | Include structured high-crossing benchmarks where topology-precomputed crossing mode should have the clearest advantage. |
| Same-input comparison | Both crossing modes must run on the same local benchmark definitions. |
| Output table | Emit comparable metrics for success, runtime, length, bends, crossings, route-induced IL, and verification issues. |

Current local benchmark references include Benes and Clements-style cases, but
the full LiDAR 2.0 benchmark set still needs to be inventoried and mirrored.

### LiDAR 2.0 Benchmark Inventory

The paper lists the following photonic benchmark families. The local repository
does not need to reproduce LiDAR's YAML format exactly, but it should provide
equivalent benchmark definitions that both crossing modes can run.

Local LiDAR source checkouts are available at:

```text
/home/benjamin/Documents/Repositories/working/LiDAR
/home/benjamin/Documents/Repositories/original/LiDAR
```

Important naming note: the LiDAR 2.0 paper calls the ADEPT tensor-core
benchmarks `ADEPT_*`, but the local LiDAR code stores these as
`multiportmmi_*` benchmarks:

```text
/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/benchmarks/multiportmmi_8x8/multiportmmi_8x8.yml
/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/benchmarks/multiportmmi_16x16/multiportmmi_16x16.yml
/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute/benchmarks/multiportmmi_32x32/multiportmmi_32x32.yml
```

The `original/LiDAR` checkout also contains corresponding `.layout.yml` files
for the 16x16 and 32x32 `multiportmmi` cases, which may be useful when
reconstructing placements.

| Paper benchmark | Paper crossing character | Local status | Crossing priority |
| --- | --- | --- | --- |
| `Clements_8x8` | No inherent topological crossings | Partial: `benchmarks/clements_8x8.py` exists | Low; useful sanity baseline |
| `Clements_16x16` | No inherent topological crossings | Missing | Low; scale sanity baseline |
| `Clements_8x8_C` | Compact, no inherent topological crossings | Missing | Low; congestion baseline |
| `Clements_16x16_C` | Compact, no inherent topological crossings | Missing | Low; congestion baseline |
| `ADEPT_8x8` | Moderate inherent crossings | Missing locally; source data is LiDAR `multiportmmi_8x8` | High |
| `ADEPT_16x16` | Higher inherent crossings | Missing locally; source data is LiDAR `multiportmmi_16x16` | High |
| `ADEPT_32x32` | Large inherent crossing count | Missing locally; source data is LiDAR `multiportmmi_32x32` | High |
| `ADEPT_8x8_C` | Compact crossing-heavy tensor-core case | Missing locally; start from LiDAR `multiportmmi_8x8` and compact placement if needed | Very high |
| `ADEPT_16x16_C` | Compact crossing-heavy tensor-core case | Missing locally; start from LiDAR `multiportmmi_16x16` and compact placement if needed | Very high |
| `ADEPT_32x32_C` | Large compact crossing-heavy tensor-core case | Missing locally; start from LiDAR `multiportmmi_32x32` and compact placement if needed | Very high, after smaller ADEPT works |
| `TeMPO_8x8_C` | Hierarchical/reusable, crossing-heavy compact case | Missing | Very high |
| `TeMPO_16x16_C` | Larger TeMPO compact case | Missing | Very high, after 8x8 works |
| `TeMPO_32x32_C` | Very large TeMPO compact case | Missing | Later scale target |
| `GWOR_16x16_C` | Optical-switch benchmark with routing-order pressure | Missing | High |
| `GWOR_32x32_C` | Larger GWOR case | Missing | High, after 16x16 works |
| `Benes_16x16_C` | Structured hierarchical topology with very high crossing density | Partial local equivalent: `benchmarks/benes_16x16.py` exists, compactness parity unknown | Very high |
| `Benes_32x32_C` | Large high-crossing Benes case | Partial local equivalent: `benchmarks/benes_32x32.py` exists, compactness parity unknown | Very high scale target |
| `Light_a`, `Light_b`, `Light_c`, `Light_d` | Optical-switch cases; paper reports no routed topological crossings after optimization | Missing | Medium; useful loss/crossing tradeoff cases |

Existing local crossing-oriented benchmarks:

| Local benchmark | Current role |
| --- | --- |
| `benes.py` | Base Benes generator/reference. |
| `benes_4x4.py` | Small crossing smoke test. |
| `benes_8x8.py` | Medium local crossing test. |
| `benes_16x16.py` | Larger local crossing test. |
| `benes_32x32.py` | Scale/stress crossing test. |
| `clements_8x8.py` | No-crossing or low-crossing baseline imported from LiDAR-style data. |

Recommended crossing-first benchmark order:

1. Local `benes_4x4` and `benes_8x8` for fast strict-crossing correctness.
2. Local `benes_16x16` and `benes_32x32` for structured high-crossing scale.
3. Add/import `ADEPT_8x8_C` and `ADEPT_16x16_C` for dense multi-port crossing
   pressure.
4. Add/import `TeMPO_8x8_C` and `TeMPO_16x16_C` for hierarchy/reuse-like
   crossing pressure, even before full hierarchy reuse is implemented.
5. Add/import `GWOR_16x16_C` and `GWOR_32x32_C` for routing-order pressure.
6. Fill in Clements, Light, and larger variants for the final paper-style
   comparison table.

## PLM Goal

PLM already works well for the current bump-meander structure. The next step is
to make delay insertion structure-aware: the planner should be able to choose a
delay geometry that fits the available box and minimizes unnecessary bends.

### Existing Structure: Bump Meander

The current implementation inserts delay through analytic meanders with repeated
bumps inside an available box.

Strengths:

| Property | Meaning |
| --- | --- |
| Compact fill | Can add delay inside a bounded local box. |
| Existing support | Already integrated with route records, obstacle checks, and Rust meander planning. |
| Good general fallback | Useful when the available box is not long enough for a large parallel run. |

Weakness:

| Issue | Meaning |
| --- | --- |
| Bend-heavy | Repeated bumps add many turns, which can increase bend loss and make the shape less attractive when a simpler foldback would fit. |

### New Structure: Parallel Foldback Delay

The attached sketch shows a delay structure that runs a long straight segment
parallel to the original route segment and connects it with two turn regions.
Compared with repeated bump meanders, this can add a large delay with fewer
bends when the route has enough available length.

Intended behavior:

| Requirement | Meaning |
| --- | --- |
| Same insertion box model | The planner should use the same obstacle-aware box concept as the existing PLM flow where possible. |
| Long parallel run | The inserted delay should prefer a long straight segment parallel to the base segment. |
| Few turns | The structure should normally use two foldback/U-turn regions instead of many repeated bumps. |
| Bend-radius aware | The foldback spacing and turn geometry must respect the configured optical bend radius and clearance. |
| Exact or bounded extra length | The planner should report the requested extra length, physically inserted extra length, and residual error. |
| Obstacle-aware selection | The structure must be checked against static obstacles, routed waveguides, meander reservations, and port/crossing keepouts. |

This structure is useful when the delay box is long and relatively narrow. The
bump meander remains useful when the available region is shorter, wider, or more
fragmented.

### PLM Planner Finished State

The PLM planner should treat delay insertion as a choice among candidate
structures.

| Planner requirement | Meaning |
| --- | --- |
| Candidate generation | Generate bump-meander and parallel-foldback candidates for each eligible routed edge. |
| Cost model | Prefer lower bend count/loss for similar inserted length and legality. |
| Shared obstacle checks | Reuse the same obstacle registration and box-free checks across delay structures. |
| Deterministic selection | Given the same inputs, choose the same structure and report why. |
| Debug output | Include selected structure type, rejected candidate reasons, inserted length, residual mismatch, bend estimate, and occupied/reserved cells. |

## Metrics

Every benchmark report for this phase should make the crossing and PLM tradeoffs
visible.

| Metric | Why it matters |
| --- | --- |
| Runtime | Shows whether the Rust-backed router is meeting the speed goal and whether later crossing variants improve throughput. |
| Routed nets / failed nets | Basic completion signal. |
| Total route length | Propagation-loss component. |
| Bend count or bend-angle penalty | Bend-loss component. |
| Crossing count | Crossing-loss component. |
| Route-induced insertion loss | Unified route-quality score from length, bends, and crossings. |
| Search-guidance penalties | Congestion, history, repair, or dynamic-conflict costs that influenced search but are not physical optical loss. |
| Verification issues | Ensures speed and loss are not hiding invalid geometry. |
| Verification JSON | Gives agents, tests, and humans the same structured pass/fail evidence for benchmark runs. |
| Crossing component footprint | Records which PDK/gdsfactory crossing component and bbox were used for legality and realization. |
| PLM requested/inserted/residual length | Shows whether timing/path-length constraints were actually met. |
| PLM structure mix | Shows when bump meanders versus parallel foldbacks are used. |

## Out Of Scope For This Phase

| Topic | Reason |
| --- | --- |
| Metal/electrical routing | Existing code can stay, but it should not be part of the crossing/PLM comparison target. |
| LiDAR 3.0 comparison | LiDAR 3.0 is electrical-routing focused and belongs in a separate evaluation. |
| Exact LiDAR implementation cloning | The goal is comparable functionality and fair baseline behavior, not copying LiDAR's internal Python structure. |
| Full hierarchy as a blocker | Hierarchy remains important for LiDAR 2.0 parity, but crossing and PLM work can progress before full hierarchical module reuse exists. |

## Main Implementation Tasks

| Priority | Task | Main files likely involved |
| --- | --- | --- |
| 1 | Stabilize final-geometry verification for crossing correctness | `translation/photonic_verification.py`, `translation/route_rust_realization.py`, `translation/route_rust_records.py`, crossing verification tests |
| 2 | Stabilize router-discovered crossing search without topology expected pairs | `src/astar.rs`, `src/crossings.rs`, `src/py_router.rs`, `translation/route_rust.py` |
| 3 | Preserve crossing decisions through port snapping and geometry realization | `translation/route_rust_realization.py`, `src/geometry_realization.rs`, route record types, primitive/crossing component helpers |
| 4 | Prove `benes_4x4`, `benes_8x8`, and `multiportmmi_8x8` with exact commands, artifacts, and verification status | `benchmarks/`, `benchmark_metadata.py`, `scripts/benchmark_photonic.py`, `routing_flow.py` |
| 5 | Preserve existing PLM regression behavior while crossing work changes route records | `src/meander.rs`, `translation/route_rust_meanders.py`, `translation/path_length_candidates.py`, PLM tests |
| 6 | Define a route-induced insertion-loss model shared by A* route selection and reports | `src/astar.rs`, `src/primitives.rs`, `translation/route_rust.py`, benchmark scripts |
| 7 | Add parallel-foldback PLM candidate planning | `src/meander.rs`, `translation/route_rust_meanders.py`, `translation/path_length_candidates.py` |
| 8 | Optionally keep topology-precomputed crossing mode as a separately selectable later comparison mode | `python/photonic_router/topology_analysis.py`, `python/photonic_router/crossing_plan.py`, `translation/route_rust.py`, `src/crossings.rs` |

## Success Criterion

The phase is successful when the repository can run the current crossing
benchmark targets, produce valid routed GDS/debug artifacts, classify final
geometry correctness, report route-induced insertion loss from
length/bends/crossings, and preserve existing PLM regression behavior.

After that foundation is stable, success can expand to additional benchmark
families, parallel-foldback PLM, and optional comparisons between
router-discovered and topology-precomputed crossing modes.
