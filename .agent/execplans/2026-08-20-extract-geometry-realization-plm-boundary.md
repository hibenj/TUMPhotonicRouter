# Future Architecture Initiative: geometry realization / path-length matching boundary

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This is the third step of `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative," continuing the recommended order from `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (obstacle map building + grid snapping, done; A* single-net search, done -- see `.agent/execplans/2026-08-19-extract-astar-single-net-search-interface.md`; now geometry realization and path-length matching).

Unlike the two prior extractions, this one starts with an open design question instead of a clean scope: the stage-characterization plan found `src/geometry_realization.rs` (8,273 lines, 70 `#[test]`s) and `src/plm.rs` (1,053 lines, **zero** `#[test]`s) coupled at the Rust type level -- `plm.rs` directly imports meander-geometry types (`AutoMeanderConfig`, `AutoMeanderPlanningProfile`, `AutoMeanderSidePolicy`, `AutoRouteAnalyticMeanderPlan`, `DenseOccupancyPrefix`, `GeometryGridSpec`, `SparseCellIndex`, plus the function `plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix` and `cell_count_in_grid_rect`) straight from `geometry_realization.rs`. **A preliminary check while scoping this plan (2026-08-20) found the picture is more tangled than a two-file coupling**: a third module, `src/meander.rs` (898 lines), defines its own parallel-looking meander types (`AnalyticMeanderConfig`, `AnalyticMeanderPlan`, `MeanderBox`, `MeanderPlanningMode`, `MeanderTurnModel`) with similar-but-differently-named counterparts to `geometry_realization.rs`'s `AutoMeanderConfig`/`AutoRouteAnalyticMeanderPlan` -- whether these are two genuinely separate systems (older/newer, or different purposes), or one wraps/supersedes the other, is not yet established. On the Python side, at least 5 files plausibly split across these same concerns without a confirmed mapping yet: `translation/route_rust_meanders.py` (3,139 lines, the largest single Python file in `translation/`), `translation/path_length_candidates.py`, `translation/path_length_diagnostics.py`, `translation/path_length_requirements.py`, and `python/photonic_router/path_length_graph.py` (already confirmed, by the characterization plan, to use `typing.Protocol` -- existing prior art for this exact pattern).

Given this, this plan does not commit to a boundary or an interface design up front. Milestone 0 is a thorough characterization of the real coupling (which types/functions genuinely belong to "realize this route's geometry" versus "plan delay-matching meanders" versus something in between, and how `meander.rs` relates to both). Milestone 1 is presenting concrete boundary options, with tradeoffs, to the repository owner -- **this milestone requires the repository owner's decision, not a solo Claude choice**, per the repository owner's own direction (2026-08-20): characterize first, then present options, matching how the dense-port lateral-width finding (a different, unrelated design question, `.agent/REPOSITORY_STATE.md` Current Findings item 2) was handled. Milestones after that (design the actual interface(s), implement, test, validate) are intentionally left unspecified until Milestone 1's decision lands, per `.agent/PLANS.md`'s guidance against over-specifying before the design is justified.

This plan follows the repository owner's standing direction to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a change is bounded and well-specified, and the zero-behavior-change discipline established by the two prior extractions -- once a boundary is chosen and an interface designed, implementation should move/rename/formalize existing behavior, not change it.

## Progress

- [ ] Not started. This plan was just written; Milestone 0 (full characterization of the geometry-realization/meander/PLM coupling, dispatched to a fork) has not yet begun.

## Surprises & Discoveries

- **Preliminary finding (2026-08-20, before Milestone 0 formally began)**: `src/meander.rs` (898 lines) exists as a third module with its own meander-planning types (`AnalyticMeanderConfig`, `AnalyticMeanderPlan`, `MeanderBox`, `MeanderPlanningMode`, `MeanderTurnModel`, `MeanderPlanningError`, `MeanderSide`, `PhysicalPoint`, `StraightSegment`, `AnalyticMeanderFootprint`), separate from `geometry_realization.rs`'s `AutoMeanderConfig`/`AutoMeanderPlanningProfile`/`AutoMeanderSidePolicy`/`AutoRouteAnalyticMeanderPlan`/`DenseOccupancyPrefix`/`SparseCellIndex`. `plm.rs` imports `MeanderPlanningMode` from `meander.rs` directly (`src/plm.rs:10`) in addition to its imports from `geometry_realization.rs` (`src/plm.rs:5-9`). Not yet investigated whether `meander.rs`'s `Analytic*` family and `geometry_realization.rs`'s `AutoMeander*`/`AutoRouteAnalytic*` family are two genuinely distinct systems, one supersedes/wraps the other, or they represent different granularities of the same concern (e.g. `meander.rs` = low-level geometric primitives, `geometry_realization.rs` = higher-level planning built on them). `src/plm.rs` itself has zero `#[test]`s (confirmed via `grep -c '#\[test\]' src/plm.rs`), consistent with the stage-characterization plan's earlier finding.

## Decision Log

- Decision: characterize the coupling fully before presenting any boundary options, and present options rather than choosing a boundary solo, per the repository owner's explicit direction when asked how to proceed with this stage (2026-08-20): "Characterize first, then show options."
  Date/Author: 2026-08-20, repository owner (via direct choice on presented options), recorded here.

## Outcomes & Retrospective

(To be filled in as this plan's milestones complete.)

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan. Every fact needed to understand and execute this plan is repeated here.

### What's known so far (2026-08-20, not yet a full characterization)

- `src/geometry_realization.rs`: 8,273 lines, 70 `#[test]`s. Converts route records (grid-cell paths) into physical gdsfactory geometry, including PDK crossing components (per `.agent/PROJECT_GOAL.md`'s stage description) -- but also contains meander-planning machinery (`AutoMeanderConfig` and friends) that `plm.rs` depends on.
- `src/plm.rs`: 1,053 lines, 0 `#[test]`s. Path-length-matching candidate planning and meander insertion (per the stage-characterization plan's prior finding: `plan_registered_geometry_requirement_candidates`, `plan_registered_geometry_request_sequence`, `plan_registered_geometry_split_request`, `plan_registered_geometry_final_requests`, `src/plm.rs:391-791` -- these line numbers are from that earlier characterization and not yet reconfirmed in this plan). Its only exercise today is production call sites in `src/py_router.rs:13215-13467` (also from that earlier characterization, not yet reconfirmed), reading from `self.registered_plm` (a `RefCell<RegisteredPlmContext>` field on the shared session struct) and other `self.*` fields.
- `src/meander.rs`: 898 lines, not yet characterized in this plan. Has its own meander-geometry type family, imported into `plm.rs` alongside the `geometry_realization.rs` imports.
- Python side, not yet mapped to the above: `translation/route_rust_meanders.py` (3,139 lines), `translation/path_length_candidates.py` (319 lines), `translation/path_length_diagnostics.py` (460 lines), `translation/path_length_requirements.py` (308 lines), `python/photonic_router/path_length_graph.py` (324 lines, already confirmed by the stage-characterization plan to define `SchematicLike(Protocol)` -- existing prior art for exactly the interface pattern this initiative wants).

None of the above should be treated as a confirmed, complete picture -- it is the starting point for Milestone 0's investigation.

### Toolchain notes

This machine's Rust toolchain needs an explicit override, since the checked-in `rust-toolchain.toml` targets Windows:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release

Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16` at any point in this plan's execution, per this repository's standing safety rule about resource use on this machine. This is characterization work; no build or benchmark commands are expected until an implementation milestone exists.

The three open, deliberately-parked benchmark findings and the ripup/repair restructuring are out of scope for this plan, same as the two prior extraction plans. `heater_s_mod` and related PLM regression benchmarks are the relevant existing test coverage for path-length matching at the integration level (`tests/test_routing_flow_stats.py::test_heater_s_mod_90_degree_plm_regression`, already a known pre-existing failure per `.agent/REPOSITORY_STATE.md`'s current test baseline -- confirm during Milestone 0 whether this plan's own work could plausibly interact with that failure, without necessarily fixing it, since fixing pre-existing unrelated failures is out of this plan's scope unless directly implicated).

## Plan of Work

### Milestone 0: full characterization of the geometry-realization/meander/PLM coupling

With direct source evidence (file:line citations, not inference from names):

- Map every type and function `src/plm.rs` imports from `src/geometry_realization.rs` and `src/meander.rs`, and for each, determine: does it conceptually belong to "realizing a route's physical geometry" or "planning a path-length-matching meander," or is it genuinely shared machinery neither stage should solely own?
- Determine the actual relationship between `src/meander.rs`'s `Analytic*` type family and `src/geometry_realization.rs`'s `AutoMeander*`/`AutoRouteAnalytic*` family -- distinct systems, wrapper/wrapped, or different granularity of the same concern. Read enough of both to answer with real evidence, including how `geometry_realization.rs` itself uses `meander.rs` (if it does) alongside how `plm.rs` uses both.
- Map the Python side: which of `translation/route_rust_meanders.py`, `translation/path_length_candidates.py`, `translation/path_length_diagnostics.py`, `translation/path_length_requirements.py`, `python/photonic_router/path_length_graph.py` (and any others found along the way) correspond to which Rust-side concern, and whether the Python split already reflects a sensible boundary that the Rust side could mirror, or whether the Python side has its own version of the same tangle.
- Check test coverage shape: `src/geometry_realization.rs`'s 70 tests -- do any of them exercise the meander-planning functions specifically, or are they all realization-proper? `src/plm.rs`'s zero tests and the production call sites in `src/py_router.rs` -- get a real sense of how `plm.rs`'s functions are actually invoked and what state they need from `self`.
- Actively look for anything a symbol-name-level check would miss (formula-level duplication, hidden coupling through shared mutable state, etc.), per this initiative's own accumulated lesson (see `.agent/REPOSITORY_STATE.md`'s Recent Session Notes) that characterization passes can under- or over-state real structural problems in either direction -- verify, don't assume either way.

Record findings in Surprises & Discoveries. Conclude with 2-3 concrete boundary options (e.g. "meander planning stays entirely with PLM, geometry realization exposes only route-polygon primitives," or "meander geometry is its own third stage/module," or some other real option the evidence suggests), each with a one-paragraph tradeoff summary -- but do not choose one.

### Milestone 1: present options to the repository owner

Present Milestone 0's boundary options directly to the repository owner (via `AskUserQuestion` or equivalent) and get an explicit decision before any design or implementation work begins. Record the decision and its rationale in the Decision Log.

### Milestones 2+: not yet specified

To be added once Milestone 1's decision is known, per `.agent/PLANS.md`'s guidance against over-specifying before source inspection and a settled design justify the next steps. Expected shape, subject to revision: design the interface(s) for whichever stage(s) the chosen boundary implies, implement via Codex once well-specified, broaden test coverage (particularly for `plm.rs`, which currently has none), full validation ladder.

## Concrete Steps

Milestone 0 is read-only source investigation -- no build or benchmark commands. See "Toolchain notes" above for later milestones' build/test commands, and this plan's sibling extraction plans for the `build/verification/*.json` verdict-reading pattern once an implementation milestone exists.

## Validation and Acceptance

Milestone 0 is complete when Surprises & Discoveries has a direct-evidence-backed characterization of the full geometry-realization/meander/PLM coupling (not just the two-file version originally assumed) and 2-3 concrete, evidence-grounded boundary options exist for Milestone 1. Milestone 1 is complete when the repository owner has made an explicit, recorded decision. Later milestones' acceptance criteria depend on that decision.

## Idempotence and Recovery

Milestone 0 is entirely read-only and safe to redo or resume freely; it makes no code changes.

## Artifacts and Notes

Originating plan: `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (original two-file coupling finding) and `.agent/execplans/2026-08-19-extract-astar-single-net-search-interface.md`'s Outcomes & Retrospective (flagged this plan as needing more design work than the two prior extractions).

## Interfaces and Dependencies

To be determined once Milestone 1's decision is known, per `.agent/PLANS.md`'s own guidance.
