# Repository Finished-State Target

This document defines the intended finished state for the current research
direction of TUMPhotonicRouter. It is not a full project roadmap. It narrows the
near-term goal to photonic crossings and path-length matching (PLM).

Metal/electrical routing is intentionally out of scope for this phase. The
existing electrical code should remain in the repository, but comparisons,
architecture decisions, and success criteria here should not depend on it.

## Focus

The repository should be evaluated as a photonic routing framework with two
core themes:

1. Crossing-aware optical routing that can run the LiDAR 2.0 benchmark family.
2. PLM that can insert delay using multiple physical structures, not only the
   current bump-meander structure.

The comparison target is LiDAR 2.0 for photonic routing. LiDAR 3.0 and metal
routing are separate topics and should not drive this phase.

## Finished-State Summary

When this phase is finished, the repository should support:

| Area | Finished-state expectation |
| --- | --- |
| Benchmark coverage | Run all benchmark families used in the LiDAR 2.0 evaluation, including the high-crossing cases relevant to crossing insertion. |
| LiDAR-like crossing mode | Route with an insertion-loss cost function that weighs path length, bends, and crossings; crossing insertion is discovered during routing and chosen only after competing paths have been explored. |
| Topology-precomputed crossing mode | Route with expected crossings derived before detailed routing from topology/rank information; the detailed router uses those expectations to constrain, reserve, and repair crossings quickly. |
| Comparison protocol | Run both modes on the same benchmark inputs and report success, runtime, insertion-loss terms, crossings, bends, route length, DRV/verification issues, and debug artifacts. |
| PLM | Match path lengths with a planner that can choose between at least bump meanders and a parallel foldback delay structure. |
| Documentation | Keep the comparison clear: LiDAR-like mode is for fair baseline parity, topology-precomputed mode is the proposed faster approach. |

## Crossing Goal

Crossing support should become the main comparison axis against LiDAR 2.0.
There should be two intentional router variants.

### Variant A: LiDAR-Like Loss-Guided Mode

This mode should behave similarly to LiDAR 2.0 at the decision level. It is not
required to copy LiDAR's Python implementation, but it should expose a fair
functional baseline.

Expected behavior:

| Requirement | Meaning |
| --- | --- |
| Insertion-loss cost | The route cost should explicitly combine path length, bend loss, and crossing loss. Device loss can be included in reports, but route selection between fixed endpoints is mainly driven by length, bends, and crossings. |
| Crossing as a routing decision | The router should not assume an expected crossing pair from topology. It should discover legal crossing opportunities during detailed routing. |
| Competing alternatives | The router should try many legal non-crossing and crossing alternatives before accepting a crossing, because crossings are costly and should not be inserted only because they are locally convenient. |
| Legal crossing geometry | Crossings must satisfy perpendicularity, straight access margin, footprint/keepout, layer/type compatibility where applicable, and spacing to other crossings or obstacles. |
| Loss reporting | Reports should break out length, bend count or bend-angle penalty, crossing count, and resulting route-induced insertion loss. |
| Benchmark role | This mode is the fair comparison mode against LiDAR 2.0's crossing behavior. |

This mode should answer: "If we route in the same conceptual style as LiDAR,
how does this Rust-backed framework perform and what route quality does it get?"

### Variant B: Topology-Precomputed Crossing Mode

This is the intended differentiated version of TUMPhotonicRouter. It should use
the network topology to precompute which nets are expected to cross, then guide
the detailed router with that information.

Expected behavior:

| Requirement | Meaning |
| --- | --- |
| Topology crossing plan | Python analyzes graph depth/rank/order and emits expected crossing constraints before detailed routing. |
| Expected-pair validation | Rust should allow crossing overlaps only when they match the expected partner set in strict mode. |
| Crossing reservation | The router should reserve crossing windows and reject invalid crossing locations early. |
| Local repair | If an expected crossing is blocked by its intended partner, the router can rip up or locally repair the partner instead of treating the conflict as an arbitrary obstacle. |
| Speed goal | This mode should be much faster than the LiDAR-like mode on structured crossing-heavy benchmarks because it avoids discovering the crossing graph from local collisions. |
| Quality goal | The mode should keep insertion loss competitive with the LiDAR-like mode; any extra crossings or detours must be visible in the report. |

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
| Runtime | Shows whether topology-precomputed crossings provide the expected speedup. |
| Routed nets / failed nets | Basic completion signal. |
| Total route length | Propagation-loss component. |
| Bend count or bend-angle penalty | Bend-loss component. |
| Crossing count | Crossing-loss component. |
| Route-induced insertion loss | Unified route-quality score from length, bends, and crossings. |
| Verification issues | Ensures speed and loss are not hiding invalid geometry. |
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
| 1 | Define a route-induced insertion-loss model shared by reports and LiDAR-like routing mode | `src/astar.rs`, `src/primitives.rs`, `translation/route_rust.py`, benchmark scripts |
| 2 | Add LiDAR-like crossing discovery mode without topology expected pairs | `src/astar.rs`, `src/crossings.rs`, `src/py_router.rs` |
| 3 | Keep topology-precomputed crossing mode as a separately selectable mode | `python/photonic_router/topology_analysis.py`, `python/photonic_router/crossing_plan.py`, `translation/route_rust.py`, `src/crossings.rs` |
| 4 | Build a LiDAR 2.0 benchmark inventory and local conversion status table | `benchmarks/`, `benchmark_metadata.py`, `scripts/benchmark_photonic.py` |
| 5 | Finish physical crossing realization if route reports contain accepted crossings | `translation/route_rust.py`, `src/geometry_realization.rs`, primitive/crossing component helpers |
| 6 | Add parallel-foldback PLM candidate planning | `src/meander.rs`, `translation/route_rust_meanders.py`, `translation/path_length_candidates.py` |
| 7 | Extend PLM reports to include structure type and rejected-candidate reasons | `translation/path_length_diagnostics.py`, `translation/route_rust_types.py` |
| 8 | Add same-input benchmark reports comparing LiDAR-like and topology-precomputed modes | `scripts/benchmark_photonic.py`, `scripts/crossing_benchmark_report.py`, `docs/` |

## Success Criterion

The phase is successful when the repository can run a LiDAR 2.0-style photonic
benchmark suite in both crossing modes, produce valid routed GDS/debug artifacts,
report route-induced insertion loss from length/bends/crossings, and perform PLM
with both bump-meander and parallel-foldback delay structures.

The expected result is that the LiDAR-like mode is a fair baseline, while the
topology-precomputed crossing mode routes structured crossing-heavy benchmarks
substantially faster with comparable route-induced insertion loss.
