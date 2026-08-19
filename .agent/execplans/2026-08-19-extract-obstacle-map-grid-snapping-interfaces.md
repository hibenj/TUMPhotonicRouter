# Future Architecture Initiative: extract obstacle-map-building + grid-snapping interfaces

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This is the first concrete extraction step of `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative," following the characterization done in the now-closed `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md`. That plan surveyed all five candidate pipeline stages and found obstacle-map-building and grid-snapping to be, by a wide margin, the closest to the target shape already: a single stable Python entry point (`build_static_obstacle_map(component, config) -> StaticObstacleMapData`, `python/photonic_router/static_obstacle_builder.py:101`) that already dispatches between a Rust-accelerated implementation and a pure-Python fallback behind one signature, with the deepest existing test coverage of any stage (20 Rust unit tests in `src/obstacle_map.rs`, 20 more in `src/static_obstacle_builder.rs`, plus `tests/obstacle_map_tests.rs` and `tests/test_static_obstacle_builder.py`). It was recommended as the lowest-risk starting point for the whole initiative.

While scoping this plan (2026-08-19), a genuine, previously-uncaught piece of duplication was found: grid-snapping math (`physical (um) <-> grid cell` conversion) is not implemented once per language as the characterization plan assumed, but **three times** in the Python codebase alone -- `python/photonic_router/static_obstacle_builder.py`'s `physical_to_grid`/`grid_cell_center` (the canonical pair, mirrored in Rust at `src/static_obstacle_builder.rs:457,464`), and a second, independently-written pair in the orchestration layer, `translation/route_rust.py`'s `_grid_cell_center_um` (line 1416) and a sibling `_physical_point_to_grid_cell` helper -- confirmed to use the exact same formula (`origin + (cell + 0.5) * grid_size_um`) but re-derived from `self.origin_x_um`/`self.grid.grid_size_um` rather than calling the canonical function. This makes this extraction more than a documentation exercise: unifying grid-snapping behind one interface is a real deduplication with a real (if narrow) risk of behavior drift if the two implementations are not, in fact, identical in every code path -- Milestone 0 must confirm this before anything is unified.

The goal, per `.agent/PROJECT_GOAL.md`: give obstacle-map-building and grid-snapping an explicit interface (`typing.Protocol` in Python, `trait` in Rust) so a different implementation could be substituted later without touching the rest of the pipeline, and so each is testable in isolation -- much of which already exists here; this plan's job is to make the existing contract explicit and to remove the one real duplication found, not to invent new structure from nothing.

This plan follows the repository owner's standing direction (2026-08-19) to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a change is bounded and well-specified: diagnosis and interface design stay with Claude; once each milestone below is precisely specified (exact signatures, exact before/after), implementation is dispatched to Codex via `.agent/scripts/codex_task.sh`, with Claude reviewing and running the validation ladder as QA/Harness afterward, unless a specific reason is recorded for implementing directly.

This plan follows the zero-behavior-change discipline already established in this repository's prior restructuring phases (`2026-08-17-restructure-translation-route-rust.md`, `2026-08-18-restructure-route-nets-rust.md`): every milestone here should either move/rename code or unify literally-identical logic behind one call site, never change routing behavior. Any benchmark or test result difference after a milestone is a bug to fix, not an expected outcome.

## Progress

- [x] Milestone 0 (confirm the grid-snapping duplication, map every consumer): done, see Surprises & Discoveries. Confirmed inventory: 2 formulas (floor-snap, cell-center) plus one genuinely-distinct round-to-nearest variant and one genuinely-distinct no-offset variant, inlined or re-derived across `python/photonic_router/static_obstacle_builder.py`, `translation/route_rust.py`, `translation/route_rust_geometry.py`, `translation/route_rust_records.py` (Python) and `src/static_obstacle_builder.rs`, `src/geometry_realization.rs`, `src/py_router.rs` (Rust) -- 7 files total, no other files in the repository contain either formula (confirmed by exhaustive grep of all remaining `src/*.rs` and `python/photonic_router/*.py`/`translation/*.py` files).
- [x] Milestone 1 (design): done, see Plan of Work below (obstacle-map-building gets a real `Protocol`; grid-snapping gets a canonicalize-and-migrate design, no `Protocol`/`trait`, including the `floor_snap_to_grid`/`cell_center_coordinate` scalar-helper refinement).
- [x] (2026-08-19, Codex) Rust half of Milestone 2 completed: `src/static_obstacle_builder.rs` now owns `floor_snap_to_grid` and `cell_center_coordinate`, the existing Rust `physical_to_grid`/`grid_cell_center` wrappers delegate to them, and the five assigned Rust call sites in `src/static_obstacle_builder.rs`, `src/geometry_realization.rs`, and `src/py_router.rs` call the shared helpers instead of re-deriving the formulas inline. Four focused Rust unit tests were added for the scalar helpers, wrapper outputs, and snapped-cell-center bounds. Validation passed with `cargo check --lib`, `cargo test --lib`, and `maturin develop --release`; the pre-change `cargo test --lib` baseline was `326 passed; 0 failed`, the post-change result was `330 passed; 0 failed`, and the before/after failure list was byte-identical with zero entries. **Reviewed by Claude (2026-08-19)**: diff read in full, all 5 target sites confirmed correctly migrated (including good judgment on `polygon_to_grid_bbox`, using the `physical_to_grid` wrapper since a `StaticGridSpec` was already in scope, rather than the raw scalar function), the `.ceil() as i32 - 1` sibling lines correctly left untouched at both `geometry_realization.rs` sites, `cargo test --lib` independently re-run and confirmed `330 passed; 0 failed`. Committed.
- [x] (2026-08-19, Codex) Python half of Milestone 2 completed: `python/photonic_router/static_obstacle_builder.py` now exposes a minimal runtime-checkable `ObstacleMapBuilder` `Protocol`; the assigned Python floor-snap and cell-center duplicates in `translation/route_rust.py`, `translation/route_rust_geometry.py`, and `translation/route_rust_records.py` now delegate to the canonical `physical_to_grid`/`grid_cell_center` functions where semantically identical. Focused tests were added for the canonical snap/center functions, the Protocol's structural runtime behavior, `_physical_point_to_grid_cell`'s guard-preserving wrapper, and `_grid_cell_center_um`'s center/`None` behavior. Validation passed for `py_compile`; focused pytest had the two pre-existing `tests/test_route_rust_geometry.py` failures while all 24 other focused tests passed; full-suite post-change failure IDs were byte-identical to the saved pre-change baseline, with pass count increasing from 322 to 325 because of the three new tests. **Reviewed by Claude (2026-08-19)**: diff read in full, all migration sites in `translation/route_rust.py` confirmed correct (including the more involved `_next_grid_axis_value` refactor, which changes its parameter from a raw `origin` float to an `axis` string tag internally but preserves identical output); `git stash`-based independent re-run confirmed the two `tests/test_route_rust_geometry.py` failures are pre-existing on the unstashed baseline too, not introduced by this change; full suite independently re-run, confirmed `21 failed, 325 passed, 1 skipped` matching the reported byte-identical-failure-list claim (21 failed both before and after, pass count exactly +3 for the new tests).
  **One code-quality finding, fixed directly by Claude rather than a further Codex round-trip** (small, mechanical, low-risk -- adding two ~3-line functions and simplifying two call sites, not new design): `translation/route_rust_geometry.py`'s `_physical_point_to_grid_cell` and `translation/route_rust_records.py`'s `_grid_cell_center_um` originally satisfied their delegation by constructing a throwaway `GridSpec(width=0, height=0, die_bbox=<degenerate>, ...)` just to call `physical_to_grid`/`grid_cell_center` -- functionally correct (those two fields are unused by either function) but genuinely confusing to a future reader (a `GridSpec` with zero width/height looks like a bug at a glance). Fixed by mirroring the Rust side's own pattern exactly: added `floor_snap_to_grid(coord, origin, grid_size_um)` and `cell_center_coordinate(cell, origin, grid_size_um)` as the real canonical scalar functions in `python/photonic_router/static_obstacle_builder.py` (with `physical_to_grid`/`grid_cell_center` becoming thin per-axis wrappers around them, exactly matching `src/static_obstacle_builder.rs`'s structure), and had both call sites use the scalar functions directly instead of constructing a fake `GridSpec`. Re-validated after this fix: `py_compile` clean, full suite still `21 failed, 325 passed, 1 skipped`, `tests/test_static_obstacle_builder.py` (22 tests, includes the coordinate-transformation test) all passing.
- [x] Milestone 2 (implement) complete, both halves reviewed and committed. Milestone 3 (broaden test coverage) is substantially already satisfied by the tests added alongside Milestone 2 (see above -- canonical-function tests, guard-behavior tests, negative-origin cases, `Protocol` structural test, on both sides of the language boundary). Milestone 4 (broad validation) next.

## Surprises & Discoveries

- **Grid-snapping's duplication footprint is much larger than "a third implementation" -- Milestone 0's fuller grep (2026-08-19) found the two core formulas inlined ad-hoc throughout the codebase, not just re-implemented once more.** Two formulas are involved: (a) physical-to-grid, `floor((coord - origin) / grid_size_um)`; (b) grid-cell-to-physical-center, `origin + (cell + 0.5) * grid_size_um`. Confirmed occurrences beyond the canonical pair (`python/photonic_router/static_obstacle_builder.py:817-832`'s `physical_to_grid`/`grid_cell_center`, mirrored in `src/static_obstacle_builder.rs:457,464`):
  - Formula (a), inlined directly (not via a named helper) at `translation/route_rust.py:1321-1333` (margin-adjusted bbox variant -- adds a margin before the floor division, a related but distinct operation, not pure snapping), `:5391-5392`, `:5853-5855`; and factored into a named helper, `_physical_point_to_grid_cell` (`translation/route_rust_geometry.py:372-387`), used from `translation/route_rust.py:1436` and `translation/route_rust_crossing_verification.py:199,272,843`. This helper is **not just a re-derivation** -- it adds real guard behavior the canonical `physical_to_grid` lacks (`grid_size_um <= 0.0` and non-finite `x`/`y` both return `None` instead of raising or propagating `nan`/`inf`), which any unification must decide whether to keep, drop, or push into the canonical function too.
  - Formula (b), inlined directly at `translation/route_rust.py:1418-1419` (`_grid_cell_center_um`, a bound method, not a free function), `:1814`, `:2415-2416`, `:2436-2437`; factored into a named helper, `_grid_cell_center_um` in `translation/route_rust_records.py:61-72` (a **separate, differently-scoped function from the same-named method in `route_rust.py`** -- same formula, but one is a bound method reading `self.origin_x_um`, the other a free function taking explicit `origin_x_um`/`origin_y_um`/`grid_size_um` arguments); and independently re-implemented in a **second Rust file**, `src/geometry_realization.rs:501-502`, separate from `src/static_obstacle_builder.rs:466-467`'s canonical Rust `grid_cell_center`.
  - A third, related-but-distinct formula also exists: `_grid_point_to_physical_um` (`translation/route_rust_geometry.py:390-400`, used from `translation/route_rust_crossing_verification.py:763-805`) computes `origin + point * grid_size_um` (**no** `+0.5` cell-center offset) -- this converts a fractional/sub-cell-resolution grid coordinate to physical um directly, a genuinely different operation from cell-center snapping, not a duplicate of formula (b). Milestone 1's interface design must not conflate this with grid-snapping proper.
  **The Rust side has the same pattern, independently confirmed**: beyond `src/static_obstacle_builder.rs:457-459`'s canonical `physical_to_grid`, the same floor-division formula is inlined again in the same file at `:628-629` (a bbox-min variant) and at `:601` with a **different** rounding rule (`((coord - origin) / grid_size_um - 0.5 + EPS).floor()`, i.e. round-to-nearest rather than floor-to-cell -- a genuinely different snapping rule, not a duplicate, and must not be conflated with the others), plus independently again in `src/geometry_realization.rs:3565-3566,4112` and `src/py_router.rs:3757-3758`. So this duplication spans at least 4 Rust files and 4+ Python files/locations, not 2.
  This means the earlier characterization plan's claim of "a single canonical definition each side, no duplication found" was too narrow -- it checked whether the *named* `physical_to_grid`/`grid_cell_center` functions were duplicated (they aren't), not whether their *formulas* were reimplemented inline elsewhere (they are, extensively, primarily in `translation/route_rust.py`). This raises both the value and the risk of this extraction: unifying all of this behind one interface is a much bigger real deduplication than originally scoped, but touches more call sites, so Milestone 1's design must explicitly account for the margin-adjusted variant (a), the guard-clause variant (a), the bound-method-vs-free-function split (b), the second independent Rust implementation (b), and the genuinely-different grid-to-physical-without-offset operation, rather than assuming one drop-in replacement covers every call site.

- **Rust Milestone 2 implementation found no semantic mismatch at the five assigned Rust migration sites.** `src/static_obstacle_builder.rs`'s `polygon_to_grid_bbox` already had a `&StaticGridSpec`, so its bbox-min floor-snap now delegates through `physical_to_grid`; the adjacent `ceil() as i32 - 1` bbox-max calculation remains untouched because it is a different "last covered cell" rule. `src/geometry_realization.rs`'s `GeometryGridSpec::cell_center`, `meander_box_to_grid_rect` bbox-min calculation, and `projected_free_interval_segments` first-index calculation all use the same origin/grid-size fields as the canonical scalar formula, so they now call `cell_center_coordinate` or `floor_snap_to_grid` directly. `src/py_router.rs`'s `grid_cell_for_physical_point` kept its existing non-finite/grid-size guards and bounds wrapping; only the two floor-snap lines changed. No site had to be skipped.

- **Python Milestone 2 found one task-description mismatch in `translation/route_rust.py`'s `_grid_rect_from_um_bbox`: the site is not margin-adjusted and the max edges use `ceil()`, not `floor()`.** The min x/y edges were safe to migrate through `physical_to_grid(min_x_um, min_y_um, self.grid)`. The max x/y edges were deliberately left as `ceil((max - origin) / grid_size)` because that is a different exclusive/right-edge bbox rule and replacing it with floor-snap would be a behavior change. The other assigned Python call sites matched the canonical formulas with the same `self.grid` origin/grid size.

- **Python helper wrappers used throwaway `GridSpec` construction rather than leaving the formula inline.** `translation/route_rust_geometry.py`'s `_physical_point_to_grid_cell` and `translation/route_rust_records.py`'s `_grid_cell_center_um` have only scalar origin/grid-size arguments, while the canonical Python functions accept `GridSpec`. The implementation constructs a minimal `GridSpec(width=0, height=0, die_bbox=(origin_x, origin_y, origin_x, origin_y))`; the canonical functions read only `grid_size_um` and `origin`, so this avoids duplicating the arithmetic while preserving each helper's public signature and existing guard/`None` behavior.

## Decision Log

- Decision: scope this plan to obstacle-map-building + grid-snapping only (stage 1+2 from the characterization plan's recommended order), not the other three stages. Rationale: this is the lowest-risk, best-evidenced starting point per that plan's own conclusion, and per `.agent/PLANS.md`'s guidance, tackling one well-bounded stage at a time (rather than all remaining stages in one plan) keeps each plan's Validation and Acceptance bar clean and reviewable.
  Date/Author: 2026-08-19, Claude, following the repository owner's direction to move forward with the Future Architecture Initiative after the ripup/repair ordering question was resolved (see `.agent/REPOSITORY_STATE.md`'s recorded net-ordering idea, which stays parked for the separate future repair-restructuring effort, not part of this plan).

- Decision: pursue **full unification** of the grid-snapping duplication found in Milestone 0 (all ~10+ inlined call sites across 4 Python and 4 Rust locations), not a narrow pass covering only the two already-named canonical functions. The genuinely different round-to-nearest rounding rule (`src/static_obstacle_builder.rs:601`) stays as its own clearly-named, separate function -- it must not be folded into the main floor-based snapping function.
  Rationale: the repository owner explicitly chose full unification over a narrower first pass when presented with both options and the full scope found. This matches the initiative's actual goal (eliminate real duplication, not just formalize an interface around already-clean code) and the repository's established zero-behavior-change restructuring discipline can absorb a larger mechanical migration safely as long as each call site is verified individually, which Milestone 1/2 must do.
  Date/Author: 2026-08-19, repository owner (via direct choice on the presented options), recorded here.

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

### Milestone 1: design the interfaces

**Obstacle-map-building** genuinely has two swappable implementations today (Rust-accelerated, pure-Python fallback), which is exactly the case `.agent/PROJECT_GOAL.md`'s `Protocol`/`trait` pattern is for: formalize `build_static_obstacle_map`'s existing contract as an explicit Python `Protocol` (e.g. `ObstacleMapBuilder` with a `build(component, config) -> StaticObstacleMapData` method), with `_build_static_obstacle_map_rust`/`build_static_obstacle_map_python_from_extracted` becoming the two implementations satisfying it, and `build_static_obstacle_map` staying as today's default-dispatch factory function (zero behavior change -- this is close to a pure naming/documentation exercise, per the characterization plan's original finding).

**Grid-snapping does not fit the same "swappable implementation" shape** -- there is exactly one correct implementation (floor-division snap, cell-center offset), just hand-duplicated many times rather than genuinely varying. Forcing a `Protocol`/`trait` with only one real implementer would be interface-shaped noise, not the initiative's actual goal. The right shape here is narrower and more mechanical: designate one canonical implementation per language and route every other call site through it.

Concrete design (per the repository owner's 2026-08-19 "full unification" decision):

- **Python canonical functions** (unchanged location/signature): `physical_to_grid(x, y, grid) -> GridCell` and `grid_cell_center(gx, gy, grid) -> Point` in `python/photonic_router/static_obstacle_builder.py:817,826`.
- **Migrate to call the canonical functions** (removing the inline re-derivations, keeping call-site behavior identical): `translation/route_rust.py`'s inline occurrences at lines 1321-1333 (margin-adjusted -- keep the margin arithmetic, but the floor-division core calls the canonical function), 1418-1419, 1814, 2415-2416, 2436-2437, 5391-5392, 5853-5855 (margin-adjusted, same treatment as 1321-1333).
- **`_physical_point_to_grid_cell`** (`translation/route_rust_geometry.py:372-387`) keeps its distinct name and its `None`-on-invalid-input guard (real, load-bearing behavior, not incidental) -- but its body becomes a thin wrapper: validate, then delegate to the canonical `physical_to_grid` (which needs a `GridSpec`-shaped argument or an equivalent adapter; exact signature reconciliation is an implementation-time decision, not a design-time one, since `GridSpec` and this function's explicit `origin_x_um`/`origin_y_um`/`grid_size_um` triple carry the same information in different shapes).
- **`_grid_cell_center_um`** (`translation/route_rust_records.py:61-72`, free function) and the same-named bound method (`translation/route_rust.py:1416-1420`) both become thin delegates to the canonical `grid_cell_center`, for the same reason.
- **`_grid_point_to_physical_um`** (`translation/route_rust_geometry.py:390-400`) is a genuinely different operation (no cell-center offset) -- **out of scope for this unification**, left as-is.
- **Rust design refinement (found while preparing Milestone 2's Codex task, 2026-08-19)**: `physical_to_grid`/`grid_cell_center` take `&StaticGridSpec`, but two of the migration targets do not have a `StaticGridSpec` value to pass -- `src/geometry_realization.rs`'s `cell_center` method (line 498) reads `self.origin_x_um`/`self.origin_y_um`/`self.grid_size_um` off a *different* struct with the same field shape, and `src/py_router.rs`'s `grid_cell_for_physical_point` (line 3753) reads the equivalent fields off `self.grid` (also not a `StaticGridSpec`). Constructing a throwaway `StaticGridSpec` at each such call site just to call the existing functions would be awkward and adds indirection the original code doesn't have. Instead: extract the core arithmetic into two new scalar-level free functions in `src/static_obstacle_builder.rs`, next to the existing `physical_to_grid`/`grid_cell_center` (which become thin wrappers around them):

      pub fn floor_snap_to_grid(coord: f64, origin: f64, grid_size_um: f64) -> i32 {
          ((coord - origin) / grid_size_um).floor() as i32
      }

      pub fn cell_center_coordinate(cell: i32, origin: f64, grid_size_um: f64) -> f64 {
          origin + (cell as f64 + 0.5) * grid_size_um
      }

  `physical_to_grid(x, y, grid)` becomes `(floor_snap_to_grid(x, grid.origin.0, grid.grid_size_um), floor_snap_to_grid(y, grid.origin.1, grid.grid_size_um))`, `grid_cell_center` mirrors this with `cell_center_coordinate`. Every migration target -- `StaticGridSpec`-based or not -- calls whichever of the two scalar functions fits, per axis. This is the actual canonical implementation; `physical_to_grid`/`grid_cell_center` stay as the `StaticGridSpec`-convenience wrappers most existing callers already expect.
- **Migrate to call `floor_snap_to_grid`/`cell_center_coordinate` (via the wrappers where a `StaticGridSpec` is already at hand, directly where it is not)**: the bbox-min variant at `src/static_obstacle_builder.rs:628-629` (same file, via the wrappers), and the independent re-derivations in `src/geometry_realization.rs:501-502` (`cell_center` method, via `cell_center_coordinate` directly -- no `StaticGridSpec` available), `:3565-3566,4112` (check at implementation time whether a `StaticGridSpec`/`GeometryGridSpec` value is already at hand at each site; use the wrapper if so, the scalar function directly if not), and `src/py_router.rs:3757-3758` (`grid_cell_for_physical_point`, via `floor_snap_to_grid` directly).
- **`src/static_obstacle_builder.rs:596-602`'s round-to-nearest functions are already distinctly named** (`first_cell_center_at_or_after`, `last_cell_center_at_or_before`) -- correcting Milestone 0's earlier note, which described them as "inline/anonymous"; they are not, no rename is needed. They must still never be merged into the floor-based functions -- confirmed distinct and intentional (used for a "range of cell centers within a physical span" computation, not point snapping).

This migration is mechanical (replace inline arithmetic with a call to an existing, already-tested function) but touches ~15 call sites across 7 files, so Milestone 2's implementation must verify each site's inputs genuinely match the canonical function's expected `GridSpec`/origin/grid-size semantics before replacing it -- a call site using a *different* origin or grid size than the canonical `GridSpec` would not be safe to unify blindly. Milestone 2 should re-confirm this per call site as part of implementation, not assume Milestone 1's design survey was exhaustive on this specific point.

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

2026-08-19 Codex Rust-half validation evidence:

    Pre-change baseline:
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    test result: ok. 326 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

    Required validation after the Rust helper extraction:
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib 2>&1 | tail -40
    Finished `dev` profile [unoptimized + debuginfo]

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib 2>&1 | tail -80
    test result: ok. 330 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release 2>&1 | tail -15
    Finished `release` profile [optimized]
    Installed photonic-router-rs-0.1.0

    Full-output comparison used `/tmp/tumphotonicrouter_cargo_test_lib_before.txt` and `/tmp/tumphotonicrouter_cargo_test_lib_after.txt`; both failure lists had zero entries and compared byte-identical.

Revision note, 2026-08-19 Codex: updated this living plan after completing the Rust half of Milestone 2 so future agents can see the exact helper extraction, migration-site semantic confirmation, and validation evidence without relying on chat history.

2026-08-19 Codex Python-half validation evidence:

    Pre-change baseline:
    PYTHONPATH=. .venv/bin/pytest -q
    21 failed, 322 passed, 1 skipped, 5 warnings in 57.33s

    Required validation after the Python helper extraction:
    .venv/bin/python -m py_compile python/photonic_router/static_obstacle_builder.py translation/route_rust.py translation/route_rust_geometry.py translation/route_rust_records.py
    no output; exit code 0

    PYTHONPATH=. .venv/bin/pytest -q tests/test_static_obstacle_builder.py tests/test_route_rust_geometry.py -v
    tests/test_static_obstacle_builder.py ......................             [ 84%]
    tests/test_route_rust_geometry.py ..FF                                   [100%]
    2 failed, 24 passed in 6.03s

    PYTHONPATH=. .venv/bin/pytest -q
    21 failed, 325 passed, 1 skipped, 5 warnings in 63.99s

    Full-output comparison used `/tmp/tumphotonicrouter_pytest_baseline_before.txt` and `/tmp/tumphotonicrouter_pytest_after.txt`; the `FAILED ...` short-summary lines were byte-identical before and after. The focused pytest failures and the full-suite 21 failures were all pre-existing baseline failures.

## Interfaces and Dependencies

Per Milestone 1's design (Plan of Work above):

- Python `Protocol` for obstacle-map-building, in `python/photonic_router/static_obstacle_builder.py`:

      class ObstacleMapBuilder(Protocol):
          def build(
              self, component: object, config: StaticObstacleMapConfig
          ) -> StaticObstacleMapData: ...

  `_build_static_obstacle_map_rust` and `build_static_obstacle_map_python_from_extracted` become (or are wrapped by) classes/callables satisfying this; `build_static_obstacle_map` remains the default-dispatch entry point, unchanged in signature and behavior.
- No new Rust `trait` is introduced for obstacle-map-building in this plan unless Milestone 2's implementation finds a concrete need -- the Rust side (`build_static_obstacle_map_from_geometry`) has exactly one implementation, called from Python via PyO3; the swappable-implementation boundary already lives at the Python dispatch layer described above.
- No `Protocol`/`trait` for grid-snapping -- per Milestone 1's design, this is a canonicalize-and-migrate mechanical change (one implementation per language, ~15 call sites redirected to it), not an interface-extraction target, since there is nothing to swap.
