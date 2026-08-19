# Future Architecture Initiative: extract A* single-net search interface

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This is the second concrete extraction step of `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative," continuing the recommended order from `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (obstacle map building + grid snapping, done -- see `.agent/execplans/2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md` -- then A* single-net search, then geometry realization, then path-length matching; ripup/repair orchestration deliberately excluded until it gets its own restructuring pass).

The characterization plan found `src/astar.rs`'s single-net search entry points (`route_single_net`, `route_single_net_with_config`, `route_single_net_with_crossing_config`, and siblings) already close to the target shape: fully explicit arguments (`&ObstacleMap, &PrimitiveLibrary, State, State, Option<&FxHashSet<CellKey>>, &AStarConfig[, &CrossingSearchConfig]`), zero `self`/session coupling, and 97 existing `#[test]`s -- the deepest unit-test coverage of any pipeline stage.

**This plan's own preliminary check (2026-08-19, before writing Milestone 0 in full) already found one thing the prior characterization did not mention**: `src/simple_routes.rs` (2,510 lines) defines its own trait, `SimpleRouteObstacleQuery`, used by a family of `try_*_candidate*` functions that `src/astar.rs` calls into (via `try_simple_route_with_config`, gated by `config.enable_simple_routes`) as a fast-path alternative to full graph search for simple cases. This means "A* single-net search" may not be one monolithic algorithm behind one interface, but two related search strategies (full A* graph search, and a faster simple-candidate fast path that already has its own trait) -- and the existing `SimpleRouteObstacleQuery` trait may be directly relevant prior art for this plan's own interface design, either as a pattern to match or as something that should itself become part of the extracted interface. Given the obstacle-map/grid-snapping plan's own experience -- an initial "already clean, no duplication" characterization missed a duplication footprint that turned out to span 7 files once actually grepped for -- this plan's Milestone 0 must not simply confirm the prior characterization's claims are correct; it must actively look for what that pass might have missed, the same way this preliminary check already found one thing.

This plan follows the repository owner's standing direction (2026-08-19) to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a change is bounded and well-specified: diagnosis and interface design stay with Claude; implementation is dispatched to Codex via `.agent/scripts/codex_task.sh`, with Claude reviewing and running the validation ladder as QA/Harness afterward, unless a specific reason is recorded for implementing directly.

This plan follows the zero-behavior-change discipline already established in this repository's prior restructuring phases: every milestone here should either move/rename code, formalize an already-real contract as an explicit interface, or unify literally-identical logic behind one call site -- never change routing/search behavior. Any benchmark or test result difference after a milestone is a bug to fix, not an expected outcome.

## Progress

- [ ] Not started. This plan was just written; Milestone 0 (full characterization, including the `simple_routes.rs`/`SimpleRouteObstacleQuery` relationship and an active search for anything the prior characterization missed) has not yet begun.

## Surprises & Discoveries

- **Preliminary finding (2026-08-19, before Milestone 0 formally began)**: `src/simple_routes.rs` already defines a `SimpleRouteObstacleQuery` trait (used by `try_straight_candidate_with_query`, `try_l_candidate_with_min_leg_len_query`, `try_z_candidate_with_config_query`, and siblings), and `src/astar.rs`'s `try_simple_route_with_config` (a fast-path alternative gated by `config.enable_simple_routes`) calls into this family. Not yet investigated in depth -- Milestone 0 must determine the actual relationship between this existing trait and the full A* search, and whether it changes this plan's scope or design.

## Decision Log

- Decision: continue the Future Architecture Initiative's recommended order with A* single-net search next (not geometry realization or path-length matching), per the characterization plan's own ordering and the repository owner's 2026-08-19 direction to use unsupervised time productively on already-agreed-upon priorities rather than starting new, undiscussed initiatives.
  Date/Author: 2026-08-19, Claude, working autonomously per the repository owner's explicit invitation to use available time on already-scoped work without needing further guidance.

## Outcomes & Retrospective

(To be filled in as this plan's milestones complete.)

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan. Every fact needed to understand and execute this plan is repeated here.

### Confirmed entry points (2026-08-19, read directly, not assumed from the characterization plan)

- `pub fn route_single_net(obstacle_map: &ObstacleMap, primitives: &PrimitiveLibrary, source: State, target: State, port_open_cells: Option<&FxHashSet<CellKey>>) -> Option<RouteResult>` (`src/astar.rs:1688`) -- thin wrapper calling `route_single_net_with_config` with `AStarConfig::default()`.
- `pub fn route_single_net_with_config(...) -> Option<RouteResult>` (`src/astar.rs:1859`) -- thin wrapper calling `route_single_net_with_config_reporting_stats` with a discarded stats output.
- `pub fn route_single_net_with_config_reporting_stats(...) -> Option<RouteResult>` (`src/astar.rs:1879`) -- the real workhorse; takes an additional `out_stats: &mut RouteSearchStats`.
- `pub fn route_single_net_with_crossing_config(..., dynamic_expansion_radius_cells: i32, dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>, crossing: &CrossingSearchConfig) -> Option<RouteResult>` (`src/astar.rs:2403`) -- the crossing-aware variant.
- All four take fully explicit arguments, no `self`/session state. Confirmed via direct read, not assumed.
- `PyPhotonicRouter::route_single_net` (`src/py_router.rs:8461`, a *different*, PyO3-exposed method of the same name) is a thin adapter -- resolves `self.astar_cfg`/`self.obstacle_map`/`self.primitives` into explicit arguments, then calls the free function `route_single_net_with_config` from `src/astar.rs`. Confirmed this is not a duplicate reimplementation (the grid-snapping plan's lesson: check, don't assume).
- 9 real call sites of the four free functions exist outside `src/astar.rs` itself, all in `src/py_router.rs` (confirmed via `grep -rn` across all `src/*.rs`, not just an assumption): lines 4577, 5416, 5455 (`route_single_net_with_crossing_config`), 5880 (`route_single_net_with_config_reporting_stats`), 6246, 6610, 6712 (`route_single_net_with_config`), plus the `PyPhotonicRouter::route_single_net` wrapper itself at 8461/13852 (self-referential, calls back into the free function).
- 97 `#[test]`s confirmed via `grep -c '#\[test\]' src/astar.rs`.

### Toolchain notes

This machine's Rust toolchain needs an explicit override, since the checked-in `rust-toolchain.toml` targets Windows:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release

Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16` at any point in this plan's execution, per this repository's standing safety rule about resource use on this machine. Prefer `benes_4x4` and `multiportmmi_8x8` (both configs) for benchmark validation; avoid `multiportmmi_16x16` given this machine's demonstrated resource constraints (a prior diagnostic run on that benchmark had to be killed after 10 minutes without completing) unless a specific reason requires it.

The three open, deliberately-parked benchmark findings and the ripup/repair restructuring are out of scope for this plan, same as the closed grid-snapping plan.

## Plan of Work

### Milestone 0: full characterization

Confirm (not assume) with direct source evidence:

- The exact relationship between `src/astar.rs`'s single-net search entry points and `src/simple_routes.rs`'s `SimpleRouteObstacleQuery` trait -- is the simple-route fast path a genuinely separate concern that should stay out of this extraction's scope, or does it need to be part of the same interface (e.g. as an internal implementation detail behind one trait method, or as a second trait method)?
- Whether `RouteSearchStats`, `AStarConfig`, and `CrossingSearchConfig` (the non-trivial types threaded through these signatures) are themselves clean, already-stable value types, or whether they have their own hidden coupling/duplication worth knowing about before designing a trait around them.
- All 9 real call sites in `src/py_router.rs` -- read each one, confirm what state each resolves from `self` before calling the free function, and whether any two call sites' resolution logic is itself duplicated (the grid-snapping lesson: formula-level duplication does not show up in a call-site-count grep alone).
- Whether there is any Python-side equivalent or wrapper around single-net search that the characterization plan's "A* search lives entirely in Rust" framing might have missed.
- Active test coverage shape: are the 97 tests genuinely testing the four public entry points end-to-end, or mostly testing internal helpers -- this matters for judging how safe a trait-boundary refactor is.

Record findings in Surprises & Discoveries, explicitly noting anything that confirms, refines, or contradicts the preliminary finding above and the prior characterization plan's claims.

### Milestone 1: design the trait interface

Not yet specified in detail, per `.agent/PLANS.md`'s guidance against over-specifying before source inspection justifies the design -- Milestone 0 must inform the exact shape, in particular whether `SimpleRouteObstacleQuery` and the full A* search share one interface or two.

### Milestone 2: implement

Dispatch to Codex via `.agent/scripts/codex_task.sh` once Milestone 1's design is precise, per `.agent/CLAUDE_CODEX_FLOW.md`, unless a specific reason is recorded for implementing directly.

### Milestone 3: broaden test coverage

Add focused tests exercising the new interface boundary directly, matching the pattern from the grid-snapping plan.

### Milestone 4: broad validation

Full validation ladder: `cargo test --lib`, `PYTHONPATH=. .venv/bin/pytest -q`, `benes_4x4`, `multiportmmi_8x8` under both bare CLI defaults and its documented stable-baseline config. Every verdict read from `build/verification/*.json` directly, per `.agent/WORKFLOW.md`'s Routing Verification Gate.

## Concrete Steps

See "Toolchain notes" above for build/test commands. Read verdicts directly:

    python3 -c "
    import json
    for name in ['crossing_verification', 'photonic_verification']:
        with open(f'build/verification/<benchmark>_{name}.json') as f:
            d = json.load(f)
        print(name, {k: d.get(k) for k in ['status','success','error_count','warning_count']})
        for issue in d.get('issues', []):
            print(' ', issue)
    "

## Validation and Acceptance

A* single-net search has an explicit `trait` interface that the existing implementation satisfies, with zero behavior change confirmed by the full validation ladder. New focused tests exercise the interface boundary itself. The relationship to `simple_routes.rs`'s existing `SimpleRouteObstacleQuery` trait is resolved deliberately, not left ambiguous.

## Idempotence and Recovery

Milestone 0 is read-only and safe to redo freely. Milestones 1-3 should land as one or a small number of coherent, reviewable commits once validated, matching this repository's established zero-behavior-change restructuring pattern.

## Artifacts and Notes

Originating plan: `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (recommended extraction order) and `.agent/execplans/2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md` (immediate predecessor, whose Outcomes & Retrospective's lesson about characterization completeness directly shaped this plan's Milestone 0 scope).

## Interfaces and Dependencies

To be determined during Milestone 1, once Milestone 0's characterization justifies the exact design, per `.agent/PLANS.md`'s own guidance.
