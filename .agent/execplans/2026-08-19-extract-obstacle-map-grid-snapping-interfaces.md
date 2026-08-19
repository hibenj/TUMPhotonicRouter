# Future Architecture Initiative: extract obstacle-map-building + grid-snapping interfaces

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This is the first concrete extraction step of `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative," following the characterization done in the now-closed `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md`. That plan surveyed all five candidate pipeline stages and found obstacle-map-building and grid-snapping to be, by a wide margin, the closest to the target shape already: a single stable Python entry point (`build_static_obstacle_map(component, config) -> StaticObstacleMapData`, `python/photonic_router/static_obstacle_builder.py:101`) that already dispatches between a Rust-accelerated implementation and a pure-Python fallback behind one signature, with the deepest existing test coverage of any stage (20 Rust unit tests in `src/obstacle_map.rs`, 20 more in `src/static_obstacle_builder.rs`, plus `tests/obstacle_map_tests.rs` and `tests/test_static_obstacle_builder.py`). It was recommended as the lowest-risk starting point for the whole initiative.

While scoping this plan (2026-08-19), a genuine, previously-uncaught piece of duplication was found: grid-snapping math (`physical (um) <-> grid cell` conversion) is not implemented once per language as the characterization plan assumed, but **three times** in the Python codebase alone -- `python/photonic_router/static_obstacle_builder.py`'s `physical_to_grid`/`grid_cell_center` (the canonical pair, mirrored in Rust at `src/static_obstacle_builder.rs:457,464`), and a second, independently-written pair in the orchestration layer, `translation/route_rust.py`'s `_grid_cell_center_um` (line 1416) and a sibling `_physical_point_to_grid_cell` helper -- confirmed to use the exact same formula (`origin + (cell + 0.5) * grid_size_um`) but re-derived from `self.origin_x_um`/`self.grid.grid_size_um` rather than calling the canonical function. This makes this extraction more than a documentation exercise: unifying grid-snapping behind one interface is a real deduplication with a real (if narrow) risk of behavior drift if the two implementations are not, in fact, identical in every code path -- Milestone 0 must confirm this before anything is unified.

The goal, per `.agent/PROJECT_GOAL.md`: give obstacle-map-building and grid-snapping an explicit interface (`typing.Protocol` in Python, `trait` in Rust) so a different implementation could be substituted later without touching the rest of the pipeline, and so each is testable in isolation -- much of which already exists here; this plan's job is to make the existing contract explicit and to remove the one real duplication found, not to invent new structure from nothing.

This plan follows the repository owner's standing direction (2026-08-19) to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a change is bounded and well-specified: diagnosis and interface design stay with Claude; once each milestone below is precisely specified (exact signatures, exact before/after), implementation is dispatched to Codex via `.agent/scripts/codex_task.sh`, with Claude reviewing and running the validation ladder as QA/Harness afterward, unless a specific reason is recorded for implementing directly.

This plan follows the zero-behavior-change discipline already established in this repository's prior restructuring phases (`2026-08-17-restructure-translation-route-rust.md`, `2026-08-18-restructure-route-nets-rust.md`): every milestone here should either move/rename code or unify literally-identical logic behind one call site, never change routing behavior. Any benchmark or test result difference after a milestone is a bug to fix, not an expected outcome.

## Progress

- [ ] Not started. This plan was just written; Milestone 0 (confirm the grid-snapping duplication is truly identical, and map every other current call site/consumer of both stages) has not yet begun.

## Surprises & Discoveries

- **Grid-snapping's duplication footprint is much larger than "a third implementation" -- Milestone 0's fuller grep (2026-08-19) found the two core formulas inlined ad-hoc throughout the codebase, not just re-implemented once more.** Two formulas are involved: (a) physical-to-grid, `floor((coord - origin) / grid_size_um)`; (b) grid-cell-to-physical-center, `origin + (cell + 0.5) * grid_size_um`. Confirmed occurrences beyond the canonical pair (`python/photonic_router/static_obstacle_builder.py:817-832`'s `physical_to_grid`/`grid_cell_center`, mirrored in `src/static_obstacle_builder.rs:457,464`):
  - Formula (a), inlined directly (not via a named helper) at `translation/route_rust.py:1321-1333` (margin-adjusted bbox variant -- adds a margin before the floor division, a related but distinct operation, not pure snapping), `:5391-5392`, `:5853-5855`; and factored into a named helper, `_physical_point_to_grid_cell` (`translation/route_rust_geometry.py:372-387`), used from `translation/route_rust.py:1436` and `translation/route_rust_crossing_verification.py:199,272,843`. This helper is **not just a re-derivation** -- it adds real guard behavior the canonical `physical_to_grid` lacks (`grid_size_um <= 0.0` and non-finite `x`/`y` both return `None` instead of raising or propagating `nan`/`inf`), which any unification must decide whether to keep, drop, or push into the canonical function too.
  - Formula (b), inlined directly at `translation/route_rust.py:1418-1419` (`_grid_cell_center_um`, a bound method, not a free function), `:1814`, `:2415-2416`, `:2436-2437`; factored into a named helper, `_grid_cell_center_um` in `translation/route_rust_records.py:61-72` (a **separate, differently-scoped function from the same-named method in `route_rust.py`** -- same formula, but one is a bound method reading `self.origin_x_um`, the other a free function taking explicit `origin_x_um`/`origin_y_um`/`grid_size_um` arguments); and independently re-implemented in a **second Rust file**, `src/geometry_realization.rs:501-502`, separate from `src/static_obstacle_builder.rs:466-467`'s canonical Rust `grid_cell_center`.
  - A third, related-but-distinct formula also exists: `_grid_point_to_physical_um` (`translation/route_rust_geometry.py:390-400`, used from `translation/route_rust_crossing_verification.py:763-805`) computes `origin + point * grid_size_um` (**no** `+0.5` cell-center offset) -- this converts a fractional/sub-cell-resolution grid coordinate to physical um directly, a genuinely different operation from cell-center snapping, not a duplicate of formula (b). Milestone 1's interface design must not conflate this with grid-snapping proper.
  **The Rust side has the same pattern, independently confirmed**: beyond `src/static_obstacle_builder.rs:457-459`'s canonical `physical_to_grid`, the same floor-division formula is inlined again in the same file at `:628-629` (a bbox-min variant) and at `:601` with a **different** rounding rule (`((coord - origin) / grid_size_um - 0.5 + EPS).floor()`, i.e. round-to-nearest rather than floor-to-cell -- a genuinely different snapping rule, not a duplicate, and must not be conflated with the others), plus independently again in `src/geometry_realization.rs:3565-3566,4112` and `src/py_router.rs:3757-3758`. So this duplication spans at least 4 Rust files and 4+ Python files/locations, not 2.
  This means the earlier characterization plan's claim of "a single canonical definition each side, no duplication found" was too narrow -- it checked whether the *named* `physical_to_grid`/`grid_cell_center` functions were duplicated (they aren't), not whether their *formulas* were reimplemented inline elsewhere (they are, extensively, primarily in `translation/route_rust.py`). This raises both the value and the risk of this extraction: unifying all of this behind one interface is a much bigger real deduplication than originally scoped, but touches more call sites, so Milestone 1's design must explicitly account for the margin-adjusted variant (a), the guard-clause variant (a), the bound-method-vs-free-function split (b), the second independent Rust implementation (b), and the genuinely-different grid-to-physical-without-offset operation, rather than assuming one drop-in replacement covers every call site.

## Decision Log

- Decision: scope this plan to obstacle-map-building + grid-snapping only (stage 1+2 from the characterization plan's recommended order), not the other three stages. Rationale: this is the lowest-risk, best-evidenced starting point per that plan's own conclusion, and per `.agent/PLANS.md`'s guidance, tackling one well-bounded stage at a time (rather than all remaining stages in one plan) keeps each plan's Validation and Acceptance bar clean and reviewable.
  Date/Author: 2026-08-19, Claude, following the repository owner's direction to move forward with the Future Architecture Initiative after the ripup/repair ordering question was resolved (see `.agent/REPOSITORY_STATE.md`'s recorded net-ordering idea, which stays parked for the separate future repair-restructuring effort, not part of this plan).

## Outcomes & Retrospective

(To be filled in as this plan's milestones complete.)

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan. Every fact needed to understand and execute this plan is repeated here.

### Current state of obstacle-map-building (confirmed 2026-08-19)

- Python entry point: `build_static_obstacle_map(component: object, config: StaticObstacleMapConfig | None) -> StaticObstacleMapData` (`python/photonic_router/static_obstacle_builder.py:101`). Dispatches to `_build_static_obstacle_map_rust` (calls into `src/static_obstacle_builder.rs`'s `build_static_obstacle_map_from_geometry`, `src/static_obstacle_builder.rs:208`) when the PyO3 extension is loadable (`_load_rust_backend()`), else falls back to `build_static_obstacle_map_python_from_extracted`. Both return `StaticObstacleMapData` (`python/photonic_router/static_obstacle_builder.py:56`), a frozen dataclass.
- `translation/route_rust.py` calls `build_static_obstacle_map` at lines 223, 488, 6300 with explicit arguments; this file has zero imports back into `python/photonic_router/static_obstacle_builder.py`'s internals beyond this one function and its config/data types (confirmed by the characterization plan's Milestone 0).
- Test coverage: `src/obstacle_map.rs`'s own `mod tests` (20 `#[test]`s), `src/static_obstacle_builder.rs`'s own `mod tests` (20 `#[test]`s), `tests/obstacle_map_tests.rs` (183 lines), `tests/test_static_obstacle_builder.py` (616 lines), `tests/test_route_rust_obstacle_config.py` (97 lines).

### Current state of grid-snapping (confirmed 2026-08-19, including the newly-found duplication)

- Canonical pair: `physical_to_grid(x, y, grid) -> GridCell` and `grid_cell_center(gx, gy, grid) -> Point` (`python/photonic_router/static_obstacle_builder.py:817,826`), mirrored in Rust at `src/static_obstacle_builder.rs:457` (`physical_to_grid`) and `:464` (`grid_cell_center`).
- Duplicate pair: `translation/route_rust.py`'s `_grid_cell_center_um` (line 1416, a bound method on the routing session class, using `self.origin_x_um`/`self.grid.grid_size_um`) and `_physical_point_to_grid_cell` (referenced at line 1436, a free function taking explicit `grid_size_um`/`origin_x_um`/`origin_y_um` arguments -- not yet read in full, Milestone 0 must do so). Also used from `translation/route_rust_records.py` (lines 61, 115, 121, 150, 156) and several more call sites within `translation/route_rust.py` itself (lines 1373, 1840, 1933, 2104, 5033).

### Toolchain notes

This machine's Rust toolchain needs an explicit override, since the checked-in `rust-toolchain.toml` targets Windows:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release

Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16` at any point in this plan's execution, per this repository's standing safety rule about resource use on this machine.

The three open, deliberately-parked benchmark findings (`multiportmmi_8x8` `n_67`/`n_70`/`n_71`, `multiportmmi_16x16` `n_50`, dense-port lateral-width) and the ripup/repair restructuring (needed before that stage is extraction-ready, per the characterization plan) are all out of scope for this plan.

## Plan of Work

### Milestone 0: confirm the grid-snapping duplication and map every consumer

Read `translation/route_rust.py`'s `_physical_point_to_grid_cell` in full (not yet done -- referenced but not read while scoping this plan) and every call site of `_grid_cell_center_um`/`_physical_point_to_grid_cell` (both files), confirming with direct evidence whether they are truly always mathematically equivalent to the canonical `physical_to_grid`/`grid_cell_center` given the same inputs (same origin, same grid size), or whether some call site relies on a subtle difference (e.g. rounding mode, a different origin convention) that would make naive unification a real behavior change. Also confirm there is no third/fourth implementation elsewhere (grep `translation/*.py` and `src/*.rs` broadly, not just the files already found). Record findings in Surprises & Discoveries before proceeding.

### Milestone 1: design the Protocol/trait interfaces

Not yet specified in detail, per `.agent/PLANS.md`'s guidance against over-specifying before source inspection justifies the design -- Milestone 0 must inform the exact shape (in particular, whether grid-snapping becomes its own small `Protocol`/`trait` separate from the obstacle-map-builder one, or a method on the same interface). Should formalize the existing `build_static_obstacle_map` contract as a Python `Protocol` (e.g. an `ObstacleMapBuilder` protocol with a `build(component, config) -> StaticObstacleMapData` method, with `_build_static_obstacle_map_rust`/`build_static_obstacle_map_python_from_extracted` becoming the two implementations satisfying it) and the Rust side as a `trait` mirroring the same shape, plus a unified grid-snapping call path that removes the `translation/route_rust.py` duplication found in Milestone 0.

### Milestone 2: implement

Dispatch to Codex via `.agent/scripts/codex_task.sh` once Milestone 1's design is precise (exact trait/Protocol signatures, exact call sites to update), per `.agent/CLAUDE_CODEX_FLOW.md` and the repository owner's standing direction, unless a specific reason is recorded for implementing directly. Zero behavior change is the bar -- see Purpose/Big Picture.

### Milestone 3: broaden test coverage

Add focused tests that exercise the new interface boundary directly (e.g. a test that both the Rust-backed and pure-Python implementations satisfy the same `Protocol`/contract with the same output for the same input), not just re-running the existing full-benchmark/integration tests.

### Milestone 4: broad validation

Full validation ladder: `cargo test --lib`, `PYTHONPATH=. .venv/bin/pytest -q`, `benes_4x4`, `multiportmmi_8x8` under both bare CLI defaults and its documented stable-baseline config. Every verdict read from `build/verification/*.json` directly, per `.agent/WORKFLOW.md`'s Routing Verification Gate. Confirm identical behavior to pre-plan baselines (this is a zero-behavior-change plan) -- any difference is a bug, not an expected outcome.

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

The obstacle-map-building and grid-snapping stages have explicit `Protocol`/`trait` interfaces that the existing implementations satisfy, with zero behavior change confirmed by the full validation ladder (Milestone 4) and the grid-snapping duplication from Surprises & Discoveries eliminated (one call path, not three). New focused tests exercise the interface boundary itself.

## Idempotence and Recovery

Milestone 0 is read-only and safe to redo freely. Milestones 1-3 should land as one or a small number of coherent, reviewable commits once validated, matching this repository's established zero-behavior-change restructuring pattern -- not a partially-consistent intermediate state.

## Artifacts and Notes

Originating plan: `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (Milestone 0's Surprises & Discoveries, "Recommended extraction order"). `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative" section is the ultimate source of this whole effort.

## Interfaces and Dependencies

To be determined during Milestone 1, once Milestone 0's confirmation of the grid-snapping duplication justifies the exact design, per `.agent/PLANS.md`'s own guidance.
