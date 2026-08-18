# TUMPhotonicRouter Project Goal

This document defines the repository-level goal for agentic work. It is stable
project guidance, not a task plan. Task-specific implementation plans belong in
`.agent/execplans/`.

## Big Goal

TUMPhotonicRouter should become a very fast, reproducible photonic router that
routes benchmark designs end to end and verifies that the final physical
geometry is correct.

The Python/Rust split exists to support that speed goal. Python owns benchmark
loading, gdsfactory integration, orchestration, geometry realization, and debug
artifacts. Rust owns the hot routing path: obstacle maps, A* search, route
state, crossing legality, ripup/repair primitives, and geometry-heavy checks.

## Current Functional Goal

The current major feature is router-explored optical crossings.

The router should discover crossings during route search. It should not require
a precomputed list of expected crossing pairs for the crossing mode currently
being stabilized. LiDAR can provide ideas and reference behavior for this
specific capability, but this repository is not trying to reimplement LiDAR.
The target is a faster Rust-backed router that reasons correctly about
crossings, verifies the generated geometry, and produces usable benchmark
evidence.

The current `lidar-pure` or router-discovered path must not consume
topology-precomputed crossing hints. It should prove that crossings can be
explored and selected during routing itself. Topology-precomputed crossings can
remain in the repository as an existing path and should be included later as a
separate comparison or optimization mode.

A* route selection should be driven by a route-induced insertion-loss objective.
At minimum, that objective should account for propagation length, bend penalty,
and crossing penalty, so a shorter route with many bends or crossings is not
automatically preferred over a slightly longer but lower-loss route. Congestion,
history, and repair penalties may still be used as search guidance, but they
should be reported separately from physical insertion loss unless they represent
actual optical loss.

Physical crossing realization should use the active PDK/gdsfactory crossing
component instead of a hard-coded abstract marker. The component footprint should
drive crossing keepout, straight-access requirements, spacing checks, debug
geometry, and final verification. A route is not fully correct until the
crossing selected during routing is represented by an inserted crossing
component in the realized geometry and the surrounding waveguides remain legal.

## Routing Pipeline Model

Future agents should reason about crossing work in the context of the whole
pipeline:

1. Setup: load benchmark schematics, placements, ports, routing configuration,
   layers, and obstacle inputs.
2. Routing: find route centerlines, including legal router-discovered crossings
   where needed.
3. Path-length matching: add delay structures for benchmarks that require
   matched path lengths. Existing regression coverage includes cases such as
   `heater_s_mod`.
4. Port snapping: adjust only endpoint access geometry so routes connect
   cleanly to component ports.
5. Geometry realization: convert route records into physical gdsfactory
   geometry and PDK/gdsfactory crossing components.
6. Verification: classify final geometry as correct or report concrete
   failures.

Crossings should be established before port snapping. Port snapping may adjust
only the route portions from a source port to the first crossing, and from the
last crossing to the destination port. It must not move or invalidate route
segments between crossings.

## Immediate Benchmark Targets

Crossing support should first work on:

- `benes_4x4`
- `benes_8x8`
- `multiportmmi_8x8`

`multiportmmi_8x8` is the current hard case and should be treated as the main
debugging target after the smaller Benes cases have stable crossing verification
and artifacts.

## Correctness Bar

A benchmark is not considered working just because the command exits or a GDS is
written. The router must be able to say whether the output is correct.

For crossing work, verification should detect and report:

- legal crossings inserted at valid straight-straight crossing windows;
- illegal intersections at bends or near bends;
- unexpected route-route intersections;
- self-overlapping or self-intersecting route centerlines;
- crossing reservations that do not match realized geometry;
- missing, misplaced, or footprint-mismatched PDK crossing components;
- port-snapping changes that move protected crossing-to-crossing route
  segments;
- path-length-matching changes that invalidate crossings or route clearance.

When crossing insertion fails, the failure should identify whether the search
failed to find a route, the route contains invalid geometry, realization changed
the intended path, or final verification rejected the physical layout.

Benchmark reports should expose the insertion-loss terms used by routing:
length loss, bend loss, crossing loss, total route-induced insertion loss, and
any non-physical search penalties that affected the chosen path.

## Near-Term Work Order

1. Stabilize verification around the routing pipeline so final geometry failures
   are classified clearly.
2. Stabilize router-discovered crossings on `benes_4x4` and `benes_8x8`.
3. Use the same verification harness to debug `multiportmmi_8x8`.
4. Preserve existing PLM regression behavior, including heater path-length
   matching benchmarks.
5. Broaden benchmarks only after the first three crossing targets have reliable
   pass/fail evidence.

## Future Architecture Initiative: Modular, Swappable, Independently Tested Stages

The user set this direction on 2026-08-18, after the codebase-readability
restructuring (see the `2026-08-17-restructure-translation-route-rust.md` and
`2026-08-18-restructure-route-nets-rust.md` ExecPlans) reached the point where
`route_nets_rust`'s internal closures were being converted into named methods
on a session class. The user has a C++ background where functionality lived
behind a header-declared interface, with implementation and tests each
answering to that interface. The equivalent in this codebase does not exist
yet: `translation/route_rust.py` and the Rust files under `src/` mix
orchestration and algorithm together, and most correctness evidence today
comes from full end-to-end benchmark runs compared against verification JSON,
not from unit tests of one stage in isolation.

The target shape, once this initiative starts: pipeline stages that are
currently buried inside larger functions become their own modules with an
explicit interface, so a different implementation could be substituted later
without touching the rest of the pipeline. Concrete candidate stages, in
roughly the order they occur in the routing pipeline: obstacle map building,
grid snapping, the A* search itself, geometry realization, and path-length
matching. In Python, the interface is a `typing.Protocol` or `abc.ABC`
declaring the stage's contract (for example, an obstacle-map builder that
takes a layout and configuration and returns an obstacle map, with no
assumptions about how it is implemented); in Rust, the equivalent is a
`trait`. Each stage should be testable with a focused fixture that does not
require running a full benchmark, the same way a C++ unit test exercises one
translation unit against its header contract.

This is explicitly a future initiative, not part of the current readability
restructuring. The current ExecPlans (Phase 1 and Phase 2 of the
`route_rust.py`/`route_nets_rust` restructuring) are deliberately scoped to
zero-behavior-change code motion: moving functions to files, turning closures
into methods, nothing more. They are a necessary step toward this initiative,
not a substitute for it: a stage cannot be judged for "does this need
everything else in this function, or could it stand alone behind an
interface" while it is still an anonymous closure with no name; once it is a
plain, named method with explicit `self.` dependencies (the state of things
once the current restructuring finishes), that question becomes answerable.
Do not fold interface extraction or new module boundaries into the current
restructuring's milestones; treat it as the next major initiative once the
current restructuring reaches a stopping point the user is satisfied with,
and expect it to require writing substantial new unit-test coverage, not just
moving code, since that coverage does not exist at this granularity today.

## Out Of Scope For The Current Phase

- Reimplementing LiDAR wholesale.
- Optimizing for exact LiDAR route order or repair behavior unless it explains a
  crossing failure in this router.
- Using topology-precomputed crossing hints in the current `lidar-pure` /
  router-discovered crossing path.
- Expanding metal/electrical routing as a primary goal.
- Building a large multi-agent orchestration system before the project goal,
  active plan, and validation harness are stable.
- Extracting swappable interfaces (Python `Protocol`/`ABC`, Rust `trait`) or
  new module boundaries for routing stages (obstacle map building, grid
  snapping, A* search, geometry realization, path-length matching) while the
  current readability restructuring is in progress. See "Future Architecture
  Initiative" above: it is the next major initiative, not part of the current
  zero-behavior-change restructuring's milestones.

## Agent Guidance

Agents should optimize for fast, verified routing. Prefer changes that make
router behavior observable and testable. If a routing change cannot be verified
with tests, debug artifacts, or geometry classification, the harness is
incomplete and should be improved before deeper heuristic tuning.
