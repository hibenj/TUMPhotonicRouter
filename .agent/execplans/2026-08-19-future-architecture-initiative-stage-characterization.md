# Future Architecture Initiative: characterize pipeline stage boundaries

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

`.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative" section (added 2026-08-18) sets the next major direction for this repository, once the readability-restructuring and open-benchmark-finding work reached a natural stopping point (both happened 2026-08-19, see `.agent/REPOSITORY_STATE.md`): give the routing pipeline's stages -- obstacle map building, grid snapping, the A* search itself, geometry realization, and path-length matching -- real, explicit interfaces (`typing.Protocol`/`abc.ABC` in Python, `trait` in Rust), so each stage can be tested in isolation with a focused fixture instead of only through full end-to-end benchmark runs, and so a different implementation could later be substituted without touching the rest of the pipeline. The repository owner's own frame of reference (a C++ background where functionality lives behind a header-declared interface that both the implementation and its tests answer to) is the target shape.

This plan is deliberately scoped to characterization only: read the current state of all five candidate stages (across both `translation/*.py` and `src/*.rs`), determine how entangled each one currently is with orchestration state and with the other stages, note what test coverage already exists at what granularity, and produce a concrete, evidence-based recommendation for which stage to extract first and in what order to tackle the rest. Per `.agent/PLANS.md`'s guidance against over-specifying before source inspection justifies the design, this plan does not yet commit to an interface design, a stage order, or an implementation approach for any specific stage -- that is deliberately left for a follow-up plan (or this plan's own later milestones, added once Milestone 0's findings justify them) once the terrain is actually known.

This plan also follows the repository owner's standing direction (2026-08-19) to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a fix or extraction is bounded and well-specified -- diagnosis and design stay with Claude. Given this plan is characterization-only, no implementation work is expected within it; that begins in whatever follow-up plan this one recommends.

This plan directly follows the repository owner's 2026-08-19 decision (see `.agent/execplans/2026-08-19-fix-open-repair-and-dense-port-findings.md`'s Decision Log and Outcomes & Retrospective) to stop chasing benchmark-specific routing fixes for now and prioritize restructuring instead: three real, root-caused, open findings (`multiportmmi_8x8`'s `n_67`/`n_70`/`n_71` cluster, `multiportmmi_16x16`'s `n_50`, and the dense-port lateral-width allocation) remain deliberately unfixed and parked in `.agent/REPOSITORY_STATE.md`, on the reasoning that the code they live in (particularly `route_many_with_repair_and_commit` in `src/py_router.rs`) is a direct target of this restructuring and fixing it now risks being rewritten shortly after landing.

## Progress

- [ ] Not started. This plan was just written; Milestone 0 (characterize all five stages) has not yet begun.

## Surprises & Discoveries

(To be filled in as work proceeds.)

## Decision Log

- Decision: scope this plan to characterization only, not extraction or implementation of any interface. Rationale: `.agent/PLANS.md` discourages over-specifying milestones before source inspection justifies the design, and `.agent/PROJECT_GOAL.md` itself only names the five candidate stages without prescribing which to do first or how entangled each currently is -- that has to be established with real evidence before a sensible extraction order or interface shape can be chosen.
  Date/Author: 2026-08-19, Claude, following the repository owner's direction to move on to the Future Architecture Initiative.

## Outcomes & Retrospective

(To be filled in as this plan's milestones complete.)

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan. Every fact needed to understand and execute this plan is repeated here.

### The five candidate stages, per `.agent/PROJECT_GOAL.md`

In roughly the order they occur in the routing pipeline:

1. **Obstacle map building**: turning a benchmark's placed layout (polygons, ports, layers) into the grid-based obstacle representation the router searches against.
2. **Grid snapping**: converting between physical (um) coordinates and the discrete routing grid.
3. **The A* search itself**: the core route-finding algorithm, including router-discovered crossing legality and ripup/repair.
4. **Geometry realization**: converting route records (grid-cell paths) into physical gdsfactory geometry, including PDK crossing components.
5. **Path-length matching**: adding delay structures for benchmarks that require matched path lengths (e.g. `heater_s_mod`).

### Known starting points (not yet verified to be complete or authoritative -- Milestone 0 must confirm)

A quick line-count and structure check (2026-08-19, at the start of this plan) found:

- `src/obstacle_map.rs` (1,452 lines) and `src/static_obstacle_builder.rs` (1,453 lines) already exist as dedicated Rust modules, with mostly free functions and structs (`build_static_obstacle_map_from_geometry`, `make_grid_spec`, `physical_to_grid`, `rasterize_polygon`, etc.) rather than being buried inside `src/py_router.rs`. This is a promising sign that obstacle-map-building and grid-snapping (stages 1-2) may already be closer to a clean extraction than the other three stages, but this needs confirmation: are these modules actually called through a narrow, stable entry point, or do callers reach into their internals from many places; is there already an implicit "contract" that just needs to be made explicit as a `trait`; how much of the *Python*-side orchestration for these same stages (in `translation/route_rust_obstacle_config.py` or similar) is entangled with other stages.
- `src/astar.rs` (10,608 lines) is presumably where stage 3 (A* search) lives, but this repository's own history (this session's Milestone 0/2 investigation into `route_many_with_repair_and_commit`, which lives in `src/py_router.rs`, not `src/astar.rs`) shows that ripup/repair -- arguably part of "the A* search" from a black-box perspective -- actually lives in `src/py_router.rs` (17,671 lines), the largest and most orchestration-heavy Rust file in the repository. This is a strong hint that stage 3's real boundary does not line up cleanly with any one file today, and this plan's Milestone 0 needs to establish where the real, defensible line is (if any) between "search" and "orchestration."
- `src/geometry_realization.rs` (8,272 lines) is presumably stage 4, and `src/plm.rs` (not yet sized/read this session) is presumably stage 5.
- On the Python side, `translation/route_rust.py` (7,451 lines, already the result of two prior restructuring phases -- see `.agent/execplans/2026-08-17-restructure-translation-route-rust.md` and `2026-08-18-restructure-route-nets-rust.md`) plus a dozen-plus sibling files (`route_rust_obstacle_config.py`, `route_rust_geometry.py`, `route_rust_meanders.py`, `route_rust_endpoint_correction.py`, `route_rust_realization.py`, `route_rust_records.py`, `route_rust_types.py`, `route_rust_crossing_*.py`, `route_rust_debug_artifacts.py`, `route_rust_analysis.py`) already exist as a result of that prior work. Milestone 0 needs to map which of these Python files correspond to which of the five stages, and how much orchestration state (e.g. a shared session/config object) crosses stage boundaries today.

None of the above should be treated as confirmed fact -- it is a starting hypothesis for Milestone 0 to verify or correct with direct source reading.

### Toolchain notes

This machine's Rust toolchain needs an explicit override, since the checked-in `rust-toolchain.toml` targets Windows:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release

Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16` at any point in this plan's execution, per this repository's standing safety rule about resource use on this machine. This plan is unlikely to need benchmark runs at all (it is source-reading/characterization work), but this note is repeated here per this repository's convention of restating standing constraints in every ExecPlan.

Per the repository owner's 2026-08-19 direction (see the closed `2026-08-19-fix-open-repair-and-dense-port-findings.md` plan), the three open benchmark findings (`multiportmmi_8x8` `n_67`/`n_70`/`n_71`, `multiportmmi_16x16` `n_50`, dense-port lateral-width) are deliberately parked and out of scope for this plan. Benchmarks should not be used as the primary evidence of progress here; focused unit tests at the stage boundary are.

## Plan of Work

### Milestone 0: characterize all five stages' current boundaries

For each of the five candidate stages, determine with direct source evidence (not inference from file names alone):

- Which Rust module(s) and Python file(s) currently implement it (confirming or correcting the hypotheses in Context and Orientation above).
- Its current entry point(s): is there already something close to a narrow, stable function signature that could become a trait/Protocol method, or is the stage's logic interleaved with orchestration/other stages throughout.
- What state it depends on beyond its own natural inputs (e.g. does it reach into a shared session object, mutate global obstacle-map state directly, or depend on results from a different stage mid-computation).
- What test coverage already exists at this granularity (a focused unit test exercising just this stage) versus only at full-benchmark granularity.
- A rough entanglement assessment: cleanly separable already / separable with moderate refactoring / deeply interleaved with other stages.

Record all five stages' findings in Surprises & Discoveries, then propose a concrete extraction order (which stage first, and why) as this milestone's conclusion -- informed by, not assumed from, the findings above. Do not begin designing a trait/Protocol for any stage yet; that is this plan's next milestone (added once Milestone 0 concludes, per `.agent/PLANS.md`'s guidance against over-specifying).

## Concrete Steps

This is characterization work: read the files named in Context and Orientation (and any others Milestone 0's investigation finds are actually relevant), using `grep`/direct reads rather than running benchmarks. No build or benchmark commands are expected to be part of this milestone's validation loop.

## Validation and Acceptance

Milestone 0 is complete when Surprises & Discoveries has a direct-evidence-backed characterization for all five stages (not just the two that already look promising), and a concrete, justified recommendation for extraction order exists that a future session (or this plan's own next milestone) can act on without re-deriving the analysis.

## Idempotence and Recovery

This milestone is entirely read-only and safe to redo or resume freely; it makes no code changes.

## Artifacts and Notes

Originating decision: `.agent/execplans/2026-08-19-fix-open-repair-and-dense-port-findings.md`'s Decision Log and Outcomes & Retrospective (2026-08-19), and `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative" section (added 2026-08-18).

## Interfaces and Dependencies

To be determined once Milestone 0's characterization justifies a design, per `.agent/PLANS.md`'s own guidance.
