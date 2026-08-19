# Future Architecture Initiative: extract A* single-net search interface

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This is the second concrete extraction step of `.agent/PROJECT_GOAL.md`'s "Future Architecture Initiative," continuing the recommended order from `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md` (obstacle map building + grid snapping, done -- see `.agent/execplans/2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md` -- then A* single-net search, then geometry realization, then path-length matching; ripup/repair orchestration deliberately excluded until it gets its own restructuring pass).

The characterization plan found `src/astar.rs`'s single-net search entry points (`route_single_net`, `route_single_net_with_config`, `route_single_net_with_crossing_config`, and siblings) already close to the target shape: fully explicit arguments (`&ObstacleMap, &PrimitiveLibrary, State, State, Option<&FxHashSet<CellKey>>, &AStarConfig[, &CrossingSearchConfig]`), zero `self`/session coupling, and 97 existing `#[test]`s -- the deepest unit-test coverage of any pipeline stage.

**This plan's own preliminary check (2026-08-19, before writing Milestone 0 in full) already found one thing the prior characterization did not mention**: `src/simple_routes.rs` (2,510 lines) defines its own trait, `SimpleRouteObstacleQuery`, used by a family of `try_*_candidate*` functions that `src/astar.rs` calls into (via `try_simple_route_with_config`, gated by `config.enable_simple_routes`) as a fast-path alternative to full graph search for simple cases. This means "A* single-net search" may not be one monolithic algorithm behind one interface, but two related search strategies (full A* graph search, and a faster simple-candidate fast path that already has its own trait) -- and the existing `SimpleRouteObstacleQuery` trait may be directly relevant prior art for this plan's own interface design, either as a pattern to match or as something that should itself become part of the extracted interface. Given the obstacle-map/grid-snapping plan's own experience -- an initial "already clean, no duplication" characterization missed a duplication footprint that turned out to span 7 files once actually grepped for -- this plan's Milestone 0 must not simply confirm the prior characterization's claims are correct; it must actively look for what that pass might have missed, the same way this preliminary check already found one thing.

This plan follows the repository owner's standing direction (2026-08-19) to use the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices once a change is bounded and well-specified: diagnosis and interface design stay with Claude; implementation is dispatched to Codex via `.agent/scripts/codex_task.sh`, with Claude reviewing and running the validation ladder as QA/Harness afterward, unless a specific reason is recorded for implementing directly.

This plan follows the zero-behavior-change discipline already established in this repository's prior restructuring phases: every milestone here should either move/rename code, formalize an already-real contract as an explicit interface, or unify literally-identical logic behind one call site -- never change routing/search behavior. Any benchmark or test result difference after a milestone is a bug to fix, not an expected outcome.

## Progress

- [x] Milestone 0 (full characterization) done, see Surprises & Discoveries. Summary: `SimpleRouteObstacleQuery` is already correctly-scoped prior art (an obstacle-query trait, not a search trait) and should stay untouched, used internally. The real single-net-search public surface is 10 functions (not 4), forming a mode x stats-reporting matrix -- Milestone 1's trait needs config-driven dispatch, not one method per function. Found one real, actionable, in-scope duplication (`CrossingSearchConfig` construction repeated 3x in `py_router.rs`, no shared helper unlike `AStarConfig`'s existing one) -- recommend folding into this plan's scope. Found one architectural finding out of scope for this plan (`route_single_net_and_commit_native` reimplements the simple-then-full fallback pattern rather than reusing the entry points that already have it) -- flag for a future follow-up, do not attempt here, same reasoning as the parked ripup/repair restructuring. `AStarConfig`/`CrossingSearchConfig`/`RouteSearchStats` are all clean owned value types. Test coverage is genuinely entry-point-level (54/97 tests call the public functions directly). No Python-side reimplementation exists -- the one characterization claim that held up unmodified.
  **Recommendation for Milestone 1**: design a trait covering the 10 `astar.rs` single-net-search entry points (config-driven mode dispatch, not 10 separate methods), and fold in the `CrossingSearchConfig`-construction-helper fix as part of the same milestone's implementation scope (small, adjacent, same spirit as the grid-snapping plan's own in-review fixes). Do not touch `SimpleRouteObstacleQuery` (already correct) or `route_single_net_and_commit_native` (out of scope, needs its own future restructuring).
- [x] Milestone 1 (design) done, see Plan of Work below. A 4-method, purely-additive `SingleNetSearch` trait (one method per genuine mode, not per stats-reporting variant) implemented by a new zero-sized marker type delegating to the existing free functions -- no existing call site migrates. Plus the `CrossingSearchConfig` dedup helper, folded in as adjacent scope.
- [ ] Milestone 2 (implementation) in progress. Codex completed the `src/astar.rs` trait-addition half on 2026-08-19: `SingleNetSearch` and `AStarSingleNetSearch` were added, with four focused equivalence tests. The `py_router.rs` `CrossingSearchConfig` helper from the original Milestone 1 design was intentionally not touched in this slice because the assigned task scope limited edits to the A* trait half.
  **Reviewed by Claude (2026-08-19)**: diff read in full. Codex correctly caught and fixed a real drift between the task file's draft signature and the actual code -- `route_single_net_with_collision_crossing_config_with_stats`'s real signature (`src/astar.rs:2407-2418`) takes an extra `reservation_open_cells: Option<&FxHashSet<CellKey>>` parameter and returns `(Option<RouteResult>, RouteSearchStats)` directly rather than the task draft's assumed `out_stats: &mut RouteSearchStats` pattern; `search_with_collision_crossing`'s trait method signature was adapted to match reality, confirmed correct by direct read of the real function. All 4 tests build a fixture, call both the trait method and the corresponding free function with identical inputs, and assert equivalence (including a `zero_route_search_timing_stats` helper that correctly neutralizes legitimately-nondeterministic timing fields before comparing `RouteSearchStats`, a thoughtful touch not explicitly requested in the task file). `cargo test --lib` independently re-run: `334 passed, 0 failed` (was `330 passed, 0 failed`, exactly +4 for the new tests). Committed.

## Surprises & Discoveries

- **Preliminary finding (2026-08-19, before Milestone 0 formally began)**: `src/simple_routes.rs` already defines a `SimpleRouteObstacleQuery` trait (used by `try_straight_candidate_with_query`, `try_l_candidate_with_min_leg_len_query`, `try_z_candidate_with_config_query`, and siblings), and `src/astar.rs`'s `try_simple_route_with_config` (a fast-path alternative gated by `config.enable_simple_routes`) calls into this family.

- **Q1 answer -- `SimpleRouteObstacleQuery` is a narrow, self-contained obstacle-query abstraction, not a search-strategy trait, and it is already the right shape; the "simple route" fast path is a clean, self-contained concern.** `SimpleRouteObstacleQuery` (`src/simple_routes.rs:131-150`) has exactly three methods (`rect_free`, `check_cells_free`, `check_segment_free`) answering only "is this region obstacle-free" -- it says nothing about search/pathfinding. It has exactly two implementors: `ObstacleMap` itself (`src/simple_routes.rs:152`) and `ExpandedDynamicObstacleQuery` (`src/simple_routes.rs:315`, a wrapper that presumably widens/relaxes the query for dynamic-clearance scenarios -- not fully read, but its existence as a *second* implementor is itself evidence this trait already does real polymorphic work, not just documentation). The simple-route candidate functions (`try_straight_l_or_z_candidate_with_config` and friends) are generic over `impl SimpleRouteObstacleQuery + ?Sized` (e.g. `src/simple_routes.rs:1416,1471,1513,1555,1640`), so they already work against either implementor without caring which. Control-flow-wise, `try_simple_route_with_config` (`src/astar.rs:1706-1770`) is called from *inside* `route_single_net_with_config_reporting_stats` (`src/astar.rs:1948`) as step one, with a graceful `?`-based fallthrough to full graph search if it returns `None` -- confirmed by test names like `simple_straight_route_used_before_astar`/`simple_l_route_used_before_astar`/`simple_z_route_used_before_astar` in `src/astar.rs`'s `mod tests`, which exist specifically to pin this fallback behavior down. **Recommendation: `SimpleRouteObstacleQuery` should not be redesigned or absorbed into a new trait -- it is already correctly-scoped prior art and can stay exactly as it is.** A new A*-search trait for Milestone 1 should treat the simple-route fast path as an internal implementation detail of "search for a single net," not a second public interface to formalize.

- **Q1 correction to the plan's own "4 confirmed entry points" framing -- the real public single-net-search surface in `src/astar.rs` is 10 functions, not 4, forming a 2x2x2-ish layered family the prior characterization undercounted.** Full list, all `pub fn`, all in `src/astar.rs`: `route_single_net` (`:1688`), `try_simple_route_with_config` (`:1706`), `try_simple_route_with_dynamic_expansion_config` (`:1779`), `route_single_net_with_config` (`:1859`), `route_single_net_with_config_reporting_stats` (`:1879`), `route_single_net_with_dynamic_expansion_config` (`:2056`), `route_single_net_with_dynamic_expansion_config_reporting_stats` (`:2080`), `route_single_net_with_collision_crossing_config` (`:2223`), `route_single_net_with_collision_crossing_config_with_stats` (`:2250`), `route_single_net_with_crossing_config` (`:2403`). The axes of variation are: base vs. dynamic-expansion vs. crossing-aware vs. collision-crossing-aware (four "modes"), each with or without an explicit `out_stats: &mut RouteSearchStats` reporting variant, plus the two `try_simple_route_with_*` functions which are the *standalone* simple-route-only entry points (callable directly, not just as an internal step of the full-search functions -- see next finding). All 10 are confirmed to share the same clean shape as the original 4: fully explicit arguments, zero `self`/session coupling. This is still good news for extraction (the shape is uniform), but Milestone 1's trait design needs to account for a 4-mode x optional-dynamic-expansion x optional-stats-reporting matrix, not a flat set of 4 methods -- likely via config-driven dispatch inside one or two trait methods (mirroring how `AStarConfig`/`CrossingSearchConfig` already parameterize behavior) rather than one trait method per current free function.

- **Q3 real finding, matching the exact shape of duplication the grid-snapping plan warned to look for: `CrossingSearchConfig` struct-literal construction is duplicated near-identically 3 times in `src/py_router.rs`, with no shared helper, unlike `AStarConfig` which already has one (`fn astar_config` at `src/py_router.rs:2788`).** The three sites (`src/py_router.rs:4333-4341`, `:4532-4540`, `:5371-5379`) all construct `CrossingSearchConfig`'s same 8 fields using the identical formula for 6 of them: `net_id`, `partners: crossing_partners`, `min_straight_cells: crossing_cfg.min_straight_cells_per_crossing`, `crossing_half_size_cells: crossing_cfg.crossing_half_size_cells`, `bend_runout_cells: self.primitive_cfg.bend_radius_cells`, `terminal_bump_guard: self.terminal_bump_guard_for_target(target, target_port_um)` -- byte-identical expressions at all three sites. They differ only in `crossing_loss` (site 1 allows an override via `crossing_loss_override.unwrap_or(...)`, sites 2-3 use the config value directly) and `require_all_partners` (site 1 uses `crossing_cfg.allow_only_expected_pairs`, site 2 hardcodes `true`, site 3 uses a local `require_all_expected_partners` variable). This is real, actionable, in-scope duplication: a `crossing_search_config(net_id, target, target_port_um, crossing_loss_override, require_all_partners)`-shaped helper (mirroring `astar_config`'s existing pattern) would eliminate it. **Recommendation: fold this into Milestone 1/2's scope** -- it is directly adjacent to (arguably part of) "how does the orchestration layer build the inputs this stage's functions need," the same category of finding the grid-snapping plan fixed.

- **Q3 second finding, more architectural: `route_single_net_and_commit_native` (`src/py_router.rs:5491`+) reimplements the "try simple route first, fall back to full search" *pattern* directly (`src/py_router.rs:5619-5644`, calling `try_simple_route_with_dynamic_expansion_config`/`try_simple_route_with_config` directly) instead of reusing the entry points that already have this fallback built in.** This is not formula duplication (the calls are correct, not reimplemented arithmetic) but *pattern* duplication -- the same two-step "simple then full" control flow exists both inside `route_single_net_with_config_reporting_stats` (`src/astar.rs:1948`, internal to astar.rs) and again here in `route_single_net_and_commit_native` (external, interleaved with crossing-attempt and commit logic). This function is large (spans at least `src/py_router.rs:5491` to past `5830`, not fully read line-by-line given time budget) and tightly interleaves crossing-partner resolution, the simple/full search attempt, obstacle-map commit, and crossing-event bookkeeping -- it reads as the single-net analogue of the same "god-method interleaving orchestration and algorithm" pattern the characterization plan already flagged for `route_many_with_repair_and_commit`, just smaller in scope. **This function itself should very likely stay out of this plan's extraction scope** (matching the characterization plan's decision to exclude ripup/repair orchestration) -- but Milestone 1 should note explicitly that a clean `AStarSearch` trait covering the 10 `astar.rs` functions will *not*, by itself, make `route_single_net_and_commit_native` simpler or safer; that would need its own follow-up, analogous to the parked ripup/repair restructuring.

- **Q2 answer -- `AStarConfig`, `CrossingSearchConfig`, and `RouteSearchStats` are all clean, owned value types with no hidden coupling.** Direct read of all three struct definitions in `src/astar.rs` confirms zero `Rc`/`RefCell`/`Arc`/`Mutex`/borrowed-reference (`&'...`) fields in any of them (checked via targeted grep over each struct's field list, not assumed). `AStarConfig` has 31 fields, `RouteSearchStats` has 83 (a wide but flat stats/counter struct, not a coupling concern), `CrossingSearchConfig` has exactly 8, all owned (`NetId`, `Vec<CrossingSearchPartner>`, primitives, `Option<TerminalBumpGuard>`). `AStarConfig` construction is already centralized behind one existing helper (`fn astar_config`, `src/py_router.rs:2788`) -- good precedent to extend to `CrossingSearchConfig` per the finding above.

- **Q4 answer -- test coverage is genuinely entry-point-level, not just internal-helper-level.** Of the 97 `#[test]`s in `src/astar.rs`'s `mod tests`, 54 (>50%) call one of `route_single_net`/`route_single_net_with_config`/`route_single_net_with_config_reporting_stats`/`route_single_net_with_crossing_config` directly (confirmed via grep count against the `mod tests` region only, not the whole file), including tests specifically named for the simple-route fallback behavior (`simple_straight_route_used_before_astar`, `simple_l_route_used_before_astar`, `simple_z_route_used_before_astar`). This is strong existing protection for a refactor that preserves these functions' external behavior. The remaining ~43 tests were not individually categorized (time-boxed), but based on sampling likely exercise lower-level helpers (heap tie-breaking, primitive expansion, heuristic modes) directly -- consistent with, not contradicting, the characterization plan's "deepest test coverage of any stage" claim.

- **Q5 answer -- confirmed, no Python-side reimplementation of pathfinding logic exists.** `grep -rln` for `a_star`/`astar`/`heapq`/priority-queue/open-set/closed-set patterns across `translation/*.py` and `python/photonic_router/*.py` returned zero matches. The prior characterization's "A* search lives entirely in Rust" claim holds up under direct check, unlike the grid-snapping plan's analogous claim (which held for named functions but not inlined formulas) -- this is a case where the original characterization was actually right, not a case to be suspicious of further without new evidence.

- **Milestone 2 implementation discovery (2026-08-19, Codex)**: the real `route_single_net_with_collision_crossing_config_with_stats` signature in `src/astar.rs` has drifted from the Milestone 1 trait sketch. It now takes `reservation_open_cells`, `dynamic_expansion_radius_cells`, and `dynamic_clearance_exempt_cells`, and returns `(Option<RouteResult>, RouteSearchStats)` rather than writing through an `out_stats: &mut RouteSearchStats`. The added `SingleNetSearch::search_with_collision_crossing` method follows the real signature so `AStarSingleNetSearch` remains a direct one-line delegate to the existing free function, preserving zero behavior change.

## Decision Log

- Decision: continue the Future Architecture Initiative's recommended order with A* single-net search next (not geometry realization or path-length matching), per the characterization plan's own ordering and the repository owner's 2026-08-19 direction to use unsupervised time productively on already-agreed-upon priorities rather than starting new, undiscussed initiatives.
  Date/Author: 2026-08-19, Claude, working autonomously per the repository owner's explicit invitation to use available time on already-scoped work without needing further guidance.
- Decision: for the trait-addition half, match `SingleNetSearch::search_with_collision_crossing` to the current collision-crossing free-function signature instead of the stale Milestone 1 sketch, because otherwise the trait method could not delegate with exactly matching arguments and return value.
  Date/Author: 2026-08-19, Codex.

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

**Design decision (2026-08-19, Claude, informed by Milestone 0's findings)**: the trait is purely additive -- it formalizes a substitutable-algorithm contract without migrating any of the 9 existing real call sites in `src/py_router.rs`, matching the posture that worked for the grid-snapping plan's `ObstacleMapBuilder` `Protocol` (which also didn't force `build_static_obstacle_map`'s two existing implementations into formal classes). Rationale: Milestone 0 found the 10-function surface is a genuine mode x stats-reporting matrix, not one algorithm with incidental variation; collapsing it into fewer methods by making crossing/dynamic-expansion parameters optional inside one method would be a real signature redesign (higher risk, arguably out of scope for "give this an explicit interface" versus "redesign this interface"), while forcing the 9 call sites to route through a new trait object right now would touch already-tested production code for no immediate behavioral benefit, given `route_single_net_and_commit_native` (the main caller) is itself flagged as not ready for this kind of restructuring (Milestone 0's out-of-scope finding). PROJECT_GOAL.md's actual ask -- "a different implementation could be substituted later without touching the rest of the pipeline" -- is satisfied by a trait a future alternative search algorithm can implement, without requiring current callers to migrate today.

Concrete design: one trait, `SingleNetSearch`, in `src/astar.rs`, with 4 methods -- one per genuine mode found in Milestone 0 (base, dynamic-expansion, collision-crossing, crossing-config -- the mode axis, not the stats-reporting axis, since the non-stats-reporting free functions are themselves trivial wrappers that discard `out_stats`, the same pattern the trait methods should follow). Each method's signature matches the fullest-capability existing free function for that mode (the `_reporting_stats`/`_with_stats` variant, or `route_single_net_with_crossing_config` itself for the 4th mode, which has no separate stats variant):

    pub trait SingleNetSearch {
        fn search(
            &self,
            obstacle_map: &ObstacleMap,
            primitives: &PrimitiveLibrary,
            source: State,
            target: State,
            port_open_cells: Option<&FxHashSet<CellKey>>,
            config: &AStarConfig,
            out_stats: &mut RouteSearchStats,
        ) -> Option<RouteResult>;

        fn search_with_dynamic_expansion(
            &self,
            obstacle_map: &ObstacleMap,
            primitives: &PrimitiveLibrary,
            source: State,
            target: State,
            port_open_cells: Option<&FxHashSet<CellKey>>,
            config: &AStarConfig,
            dynamic_expansion_radius_cells: i32,
            dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
            out_stats: &mut RouteSearchStats,
        ) -> Option<RouteResult>;

        fn search_with_collision_crossing(
            &self,
            obstacle_map: &ObstacleMap,
            primitives: &PrimitiveLibrary,
            source: State,
            target: State,
            port_open_cells: Option<&FxHashSet<CellKey>>,
            config: &AStarConfig,
            crossing: &CrossingSearchConfig,
            out_stats: &mut RouteSearchStats,
        ) -> Option<RouteResult>;

        fn search_with_crossing_config(
            &self,
            obstacle_map: &ObstacleMap,
            primitives: &PrimitiveLibrary,
            source: State,
            target: State,
            port_open_cells: Option<&FxHashSet<CellKey>>,
            config: &AStarConfig,
            dynamic_expansion_radius_cells: i32,
            dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
            crossing: &CrossingSearchConfig,
        ) -> Option<RouteResult>;
    }

Implemented by one new zero-sized marker type, `pub struct AStarSingleNetSearch;`, whose 4 method bodies each call straight through to the corresponding existing free function (`route_single_net_with_config_reporting_stats`, `route_single_net_with_dynamic_expansion_config_reporting_stats`, `route_single_net_with_collision_crossing_config_with_stats`, `route_single_net_with_crossing_config` respectively) -- one line each, no logic duplication. `SimpleRouteObstacleQuery` is deliberately not touched or referenced by this trait, per Milestone 0's Q1 finding that it is already correctly-scoped, separate prior art.

**Folded into this milestone's scope** (per Milestone 0's Q3 recommendation, same shape of finding as the grid-snapping plan's own in-review fixes): add `fn crossing_search_config(&self, net_id: NetId, target: State, target_port_um: Option<(f64, f64)>, crossing_loss_override: Option<f64>, require_all_partners_override: Option<bool>) -> CrossingSearchConfig` to `impl PyPhotonicRouter` in `src/py_router.rs`, mirroring the existing `astar_config` helper's pattern (`src/py_router.rs:2788`), and migrate the 3 duplicate `CrossingSearchConfig` struct-literal construction sites (`src/py_router.rs:4333-4341`, `:4532-4540`, `:5371-5379`) to call it -- passing `crossing_loss_override`/`require_all_partners_override` as `None` at the two sites that don't need an override, and the appropriate `Some(...)` at site 1 (`crossing_loss_override`) and the appropriate value at each site for `require_all_partners` (site 2's hardcoded `true`, site 3's `require_all_expected_partners` variable, site 1's `crossing_cfg.allow_only_expected_pairs`). Zero behavior change -- each site's resulting `CrossingSearchConfig` value must be identical before and after.

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

Per Milestone 1's design (Plan of Work above): `pub trait SingleNetSearch` (4 methods, one per mode) in `src/astar.rs`, implemented by a new zero-sized marker type `pub struct AStarSingleNetSearch;`. No Python-side interface -- this stage has no Python-side presence per Milestone 0's Q5 finding.
