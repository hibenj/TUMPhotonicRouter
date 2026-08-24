# Wire the SingleNetSearch trait into production call sites

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository, TUMPhotonicRouter, routes optical waveguides for photonic chips on a discrete grid, using an A* graph search (see `.agent/execplans/2026-08-24-modular-routing-strategies.md` for term definitions this plan reuses without redefining: obstacle map, ripup/repair, PyO3, Rust trait).

That prior plan found and closed most of a specific gap: `src/astar.rs` has a `SingleNetSearch` trait with four methods (`search`, `search_with_dynamic_expansion`, `search_with_collision_crossing`, `search_with_crossing_config`), each delegating to one of four public "wrapper" functions that implement the actual single-net search dispatch (JPS4 attempt, simple-route attempt, windowed A* retry loop, full-grid fallback). That trait was added months ago by an earlier initiative but was never used by production code — only by the trait's own tests. The prior plan unified the internal duplication inside those four wrapper functions, but deliberately did not change who calls them: `src/py_router.rs`, the file that actually orchestrates routing for real benchmarks, still calls the four free functions (and three more "discard the stats" convenience wrappers around them: `route_single_net_with_config`, `route_single_net_with_dynamic_expansion_config`, `route_single_net_with_collision_crossing_config`) directly by name, not through the trait.

This plan closes that remaining gap. After this plan, every production call site in `src/py_router.rs` that starts a single-net A* search goes through `SingleNetSearch`'s trait methods (via `AStarSingleNetSearch`, the trait's sole current implementation) instead of calling the free functions directly. This does not, by itself, let you swap in a different search algorithm today -- no second implementation of the trait exists yet, and this plan does not invent one just to prove the point (that would be scope-creep past what's actually motivated). What it does deliver: the one place in the codebase that decides "how do I search for a route for this one net" is now genuinely one seam (the trait), not thirteen scattered call sites to four free functions plus three more indirection layers -- so a future second implementation (a different heuristic, a different algorithm, a benchmarking harness that runs both and compares) needs to change one thing (what `AStarSingleNetSearch`-typed value production code holds or is generic over), not thirteen call sites.

A person can see this working by running `grep -n "route_single_net_with_config\|route_single_net_with_dynamic_expansion_config\|route_single_net_with_collision_crossing_config\|route_single_net_with_crossing_config" src/py_router.rs` after this plan and finding zero matches outside of the `use crate::astar::{...}` import list itself being trimmed to remove now-unused imports; production code will instead show `.search(`, `.search_with_dynamic_expansion(`, `.search_with_collision_crossing(`, or `.search_with_crossing_config(` at each of the same call sites. The full benchmark/pytest validation ladder produces byte-identical results to before this plan, since this is a pure call-site substitution with no behavior change (the trait methods' bodies are, today, exactly the free-function calls being replaced).

## Progress

- [x] (2026-08-24) Milestone 0: characterized every production (non-test) call site of the seven free functions in scope, across both `src/py_router.rs` and `src/astar.rs`. Found 12 call sites in `src/py_router.rs` (not the "8" estimated when this work was first deferred during the modular-routing-strategies plan) across 9 enclosing methods; confirmed `src/astar.rs`'s own apparent "production" matches at lines 1755-1852 are the `SingleNetSearch` trait's own delegating method bodies (not call sites to migrate) plus one unrelated convenience function, `route_single_net`, itself unused in production. Full list recorded in Surprises & Discoveries.
- [x] (2026-08-24) Milestone 1: read all 12 call sites directly (not assumed) and confirmed every one passes arguments to its free function in exactly the order and shape the corresponding trait method expects -- zero discrepancies found. Exact substitution table recorded in Decision Log.
- [x] (2026-08-24T11:33Z) Milestone 2: implemented Milestone 1's design across all 12 call sites in `src/py_router.rs`, removed the now-unused free-function imports, added `AStarSingleNetSearch` and `SingleNetSearch` to the import block, and validated with unchanged Rust results: pre-change and post-change `cargo check --lib` both passed cleanly, and pre-change and post-change `cargo test --lib` both reported `393 passed; 0 failed`.
  Reviewed by Claude (2026-08-24): full diff read against the Decision Log's substitution table, site by site -- every one matches exactly, including the two subtlest details (`route_single_net_with_auto_meander_and_commit`'s `opened_ref` passed without an extra `Some(...)` wrapper, preserved correctly; the shared `discarded_stats` local in `route_single_net_and_commit_repair_native` declared once before the `if`/`else`, not duplicated per branch). Confirmed via `git status`/`git diff` that only `src/py_router.rs` and this plan file were touched (Codex correctly left the separately-in-progress `.agent/ORCHESTRATOR.md` edit alone). Independently re-ran (not just trusted Codex's own report): `cargo check --lib` clean, `cargo test --lib` `393 passed, 0 failed`; `maturin develop --release` rebuild; full `pytest -q` byte-identical to baseline (`11 failed, 335 passed, 1 skipped`, empty `diff`); `benes_4x4` clean; `multiportmmi_8x8` bare and stable-baseline both clean, identical error/warning counts to every prior milestone's validation this session; `multiportmmi_16x16` stable-baseline reproduces the exact pre-existing `n_50` error text byte-for-byte -- not a regression.
- [ ] Milestone 3: broad validation (full ladder) and close out.

## Surprises & Discoveries

- Observation (Milestone 0): the free-function family in scope is larger than "the four wrapper functions" the prior plan unified -- three of the four also have a "discard the stats" convenience wrapper with no `_reporting_stats`/`_with_stats` suffix (`route_single_net_with_config`, `route_single_net_with_dynamic_expansion_config`, `route_single_net_with_collision_crossing_config`), each just calling its `_reporting_stats`/`_with_stats` sibling with a throwaway local `RouteSearchStats`. `route_single_net_with_crossing_config` has no such sibling (it never exposed stats to begin with). The `SingleNetSearch` trait has no discard-stats method variants -- all four of its methods take `out_stats: &mut RouteSearchStats`. This means the discard-stats call sites need a one-line adaptation (a local throwaway `RouteSearchStats::default()`) when migrated to call through the trait, not a direct 1:1 substitution.
- Observation (Milestone 0): full production call-site census, confirmed via `grep -n` for each of the seven function names in `src/py_router.rs`, cross-checked against `src/astar.rs`'s own matches to exclude the trait's own delegating bodies and test-module matches (everything at `src/astar.rs` line 7000+ is inside `mod tests`, confirmed by checking `mod tests {`'s own line number). 12 real call sites, across 9 enclosing `PyPhotonicRouter` methods:
  1. `route_single_net_and_commit_native` (`src/py_router.rs:5707`) -- 2 call sites: `route_single_net_with_config_reporting_stats` (line 6096), `route_single_net_with_dynamic_expansion_config_reporting_stats` (line 6076).
  2. `route_single_net_and_commit_repair_native` (`src/py_router.rs:6244`) -- 2 call sites: `route_single_net_with_dynamic_expansion_config` (line 6443), `route_single_net_with_config` (line 6462).
  3. `try_route_with_collision_crossings_using_primitives` (`src/py_router.rs:4471`) -- 1 call site: `route_single_net_with_collision_crossing_config_with_stats` (line 4580).
  4. `try_route_through_collision_partner_set` (`src/py_router.rs:4706`) -- 1 call site: `route_single_net_with_crossing_config` (line 4786).
  5. `try_route_through_expected_crossing_partner` (`src/py_router.rs:5533`) -- 2 call sites: `route_single_net_with_crossing_config` (lines 5632, 5671).
  6. `route_single_net_and_commit_orthogonal_native_with_repair_keepout` (`src/py_router.rs:6792`) -- 1 call site: `route_single_net_with_config` (line 6826).
  7. `route_single_net_ignore_dynamic_native` (`src/py_router.rs:6905`) -- 1 call site: `route_single_net_with_config` (line 6928).
  8. `route_single_net` (a `PyPhotonicRouter` method, `src/py_router.rs:11649` -- distinct from `src/astar.rs`'s free function of the same name, which is unused in production) -- 1 call site: `route_single_net_with_config` (line 11669).
  9. `route_single_net_with_auto_meander_and_commit` (`src/py_router.rs:14564`) -- 1 call site: `route_single_net_with_config` (line 14636).
  All 9 are `&self`/`&mut self` methods on `PyPhotonicRouter`, so each substitution is a same-scope, same-argument change -- no new state needs to be threaded in.
- Observation (Milestone 0): `SingleNetSearch`'s four methods are all defined on `AStarSingleNetSearch`, a zero-sized unit struct (`pub struct AStarSingleNetSearch;`, `src/astar.rs:1742`). Calling `AStarSingleNetSearch.search(...)` (or any of its other three methods) directly at each call site, with no stored field on `PyPhotonicRouter`, is sufficient to route every call through the trait -- no change to `PyPhotonicRouter`'s struct definition or its PyO3 constructor is needed. A `Box<dyn SingleNetSearch>` field (or a generic type parameter, not viable here since PyO3's `#[pyclass]` macro requires a concrete, non-generic type) would only be worth the added complexity once a second real implementation exists to select between -- none does yet, and inventing one with no real second algorithm behind it would be speculative abstraction this plan's own goal (converging toward cleaner code, not more code) argues against.
- Observation (Milestone 2): implementation matched the Milestone 1 substitution table exactly. `src/py_router.rs` had no pre-existing `discarded_stats` or `discarded_search_stats` locals, so every discard-stats adaptation used the requested `discarded_stats` name; the two mutually exclusive branches in `route_single_net_and_commit_repair_native` share one `discarded_stats` local before the `if`/`else` expression. After the edit, per-name `rg` checks for `route_single_net_with_config`, `route_single_net_with_config_reporting_stats`, `route_single_net_with_dynamic_expansion_config`, `route_single_net_with_dynamic_expansion_config_reporting_stats`, `route_single_net_with_collision_crossing_config`, `route_single_net_with_collision_crossing_config_with_stats`, and `route_single_net_with_crossing_config` found zero matches in `src/py_router.rs`.
  Evidence: before the edit, `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib` passed and `cargo test --lib` reported `test result: ok. 393 passed; 0 failed`; after the edit, the same commands passed with the same `393 passed; 0 failed` test result.

## Decision Log

- Decision: substitute each of the 12 call sites 1:1 with the matching `AStarSingleNetSearch` trait method call, with no other logic changes, confirmed safe by reading every site directly.
  Rationale/evidence: read all 12 sites in `src/py_router.rs` in full (not sampled). Every one passes its arguments to the free function in exactly the order and shape the corresponding trait method's parameter list expects -- no reordering, no type coercion, no site calling a *different* free function than what its trait method delegates to. The exact substitution:
  1. `route_single_net_and_commit_native` (line 6076, `route_single_net_with_dynamic_expansion_config_reporting_stats`) -> `AStarSingleNetSearch.search_with_dynamic_expansion(&self.obstacle_map, &self.primitives, source_state, target_state, Some(opened_search_ref), &search_cfg, block_radius_cells.max(0), dynamic_clearance_exempt_keys, &mut fallback_search_stats)`.
  2. `route_single_net_and_commit_native` (line 6096, `route_single_net_with_config_reporting_stats`) -> `AStarSingleNetSearch.search(search_obstacle_map, &self.primitives, source_state, target_state, Some(opened_search_ref), &search_cfg, &mut fallback_search_stats)`.
  3. `route_single_net_and_commit_repair_native` (line 6443, `route_single_net_with_dynamic_expansion_config`, discard-stats) -> `AStarSingleNetSearch.search_with_dynamic_expansion(&self.obstacle_map, &self.primitives, source_state, target_state, Some(opened_search_ref), &cfg, block_radius_cells.max(0), dynamic_clearance_exempt_keys, &mut RouteSearchStats::default())`.
  4. `route_single_net_and_commit_repair_native` (line 6462, `route_single_net_with_config`, discard-stats) -> `AStarSingleNetSearch.search(search_obstacle_map, &self.primitives, source_state, target_state, Some(opened_search_ref), &cfg, &mut RouteSearchStats::default())`.
  5. `try_route_with_collision_crossings_using_primitives` (line 4580, `route_single_net_with_collision_crossing_config_with_stats`) -> `AStarSingleNetSearch.search_with_collision_crossing(&self.obstacle_map, primitives, source, target, Some(opened_ref), opened_cell_keys, &crossing_search_cfg, block_radius_cells.max(0), dynamic_clearance_exempt_keys, &crossing_search)`.
  6. `try_route_through_collision_partner_set` (line 4786, `route_single_net_with_crossing_config`) -> `AStarSingleNetSearch.search_with_crossing_config(&search_map, &self.primitives, source, target, Some(opened_ref), &crossing_search_cfg, block_radius_cells.max(0), dynamic_clearance_exempt_keys, &crossing_search)`.
  7-8. `try_route_through_expected_crossing_partner` (lines 5632 and 5671, both `route_single_net_with_crossing_config`) -> same `search_with_crossing_config` shape as #6, with each site's own `&search_map`/`&crossing_search` locals.
  9. `route_single_net_and_commit_orthogonal_native_with_repair_keepout` (line 6826, `route_single_net_with_config`, discard-stats) -> `AStarSingleNetSearch.search(&self.obstacle_map, &orthogonal_primitives, State::new(source.x, source.y, source.angle), State::new(target.x, target.y, target.angle), Some(opened_keys_for_route), &cfg, &mut RouteSearchStats::default())`.
  10. `route_single_net_ignore_dynamic_native` (line 6928, `route_single_net_with_config`, discard-stats) -> `AStarSingleNetSearch.search(&static_only_obstacle_map, &self.primitives, State::new(...), State::new(...), Some(opened_ref), &cfg, &mut RouteSearchStats::default())`.
  11. `route_single_net` (line 11669, `route_single_net_with_config`, discard-stats) -> `AStarSingleNetSearch.search(&self.obstacle_map, &self.primitives, State::new(...), State::new(...), Some(opened_ref), &cfg, &mut RouteSearchStats::default())`.
  12. `route_single_net_with_auto_meander_and_commit` (line 14636, `route_single_net_with_config`, discard-stats) -> `AStarSingleNetSearch.search(&self.obstacle_map, &self.primitives, State::new(...), State::new(...), opened_ref, &cfg, &mut RouteSearchStats::default())` (note: `opened_ref` here is already `Option<&FxHashSet<CellKey>>`, not wrapped in an extra `Some(...)` like the other sites -- confirmed by reading this specific site, not assumed from the pattern of the others).
  `route_single_net_with_collision_crossing_config`'s zero-call-site finding from Milestone 0 held on re-check -- no site 13 exists for it; the import can be dropped entirely once Milestone 2 lands, not merely left unused.
  Given this is 12 mechanical, well-specified substitutions with no correctness subtlety comparable to the modular-routing-strategies plan's Milestone 6 (confirmed above: no argument-shape mismatches, no discovered behavioral divergence to reconcile), dispatch Milestone 2 to Codex per `.agent/CLAUDE_CODEX_FLOW.md`'s default.
  Date/Author: 2026-08-24, Claude, recorded here.

## Outcomes & Retrospective

Milestone 2 is complete. Production single-net search call sites in `src/py_router.rs` now enter the existing `SingleNetSearch` trait through `AStarSingleNetSearch`, while preserving the same argument lists, local control flow, and Rust validation results. Full Python and benchmark validation remain for Milestone 3 or the orchestrator/harness lane, as scoped by the task handoff.

Plan update note (2026-08-24T11:33Z): recorded the Milestone 2 implementation and validation evidence so a future agent can resume from the active plan without chat history. The update is limited to progress, implementation discoveries, and the milestone outcome.

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan, except where it explicitly points to `.agent/execplans/2026-08-24-modular-routing-strategies.md` for term definitions already established there.

### The trait and its sole implementation

`src/astar.rs:1689` defines:

    pub trait SingleNetSearch {
        fn search(&self, obstacle_map: &ObstacleMap, primitives: &PrimitiveLibrary, source: State, target: State, port_open_cells: Option<&FxHashSet<CellKey>>, config: &AStarConfig, out_stats: &mut RouteSearchStats) -> Option<RouteResult>;
        fn search_with_dynamic_expansion(&self, ..., dynamic_expansion_radius_cells: i32, dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>, out_stats: &mut RouteSearchStats) -> Option<RouteResult>;
        fn search_with_collision_crossing(&self, ..., reservation_open_cells: Option<&FxHashSet<CellKey>>, ..., crossing: &CrossingSearchConfig) -> (Option<RouteResult>, RouteSearchStats);
        fn search_with_crossing_config(&self, ..., crossing: &CrossingSearchConfig) -> Option<RouteResult>;
    }
    pub struct AStarSingleNetSearch;
    impl SingleNetSearch for AStarSingleNetSearch {
        fn search(&self, ...) -> Option<RouteResult> { route_single_net_with_config_reporting_stats(...) }
        fn search_with_dynamic_expansion(&self, ...) -> Option<RouteResult> { route_single_net_with_dynamic_expansion_config_reporting_stats(...) }
        fn search_with_collision_crossing(&self, ...) -> (Option<RouteResult>, RouteSearchStats) { route_single_net_with_collision_crossing_config_with_stats(...) }
        fn search_with_crossing_config(&self, ...) -> Option<RouteResult> { route_single_net_with_crossing_config(...) }
    }

Each trait method's body is, today, exactly a call to one free function with the same arguments in the same order (confirmed by reading `src/astar.rs:1744-1841`). This means substituting `AStarSingleNetSearch.search(...)` for a direct `route_single_net_with_config_reporting_stats(...)` call, with identical arguments, is a behavior-neutral change by construction -- the only way it could differ is if a call site currently calls a *different* free function than the one its matching trait method delegates to, which Milestone 1 must check explicitly for each of the 12 sites, not assume.

### The seven free functions and which trait method (if any) each maps to

`src/astar.rs`, all `pub fn`:

- `route_single_net_with_config_reporting_stats` (line 2036) -- maps directly to `search`.
- `route_single_net_with_config` (line 2016) -- a discard-stats wrapper around the above (`let mut discarded_stats = RouteSearchStats::default(); route_single_net_with_config_reporting_stats(..., &mut discarded_stats)`). No direct trait method; migrating a call site that uses this one means calling `search` with a local throwaway `RouteSearchStats::default()` in place of this free function's own internal one.
- `route_single_net_with_dynamic_expansion_config_reporting_stats` (line 2173) -- maps directly to `search_with_dynamic_expansion`.
- `route_single_net_with_dynamic_expansion_config` (line 2149) -- discard-stats wrapper around the above, same adaptation as `route_single_net_with_config`.
- `route_single_net_with_collision_crossing_config_with_stats` (line 2275) -- maps directly to `search_with_collision_crossing`.
- `route_single_net_with_collision_crossing_config` (line 2248) -- discard-stats wrapper (returns only `Option<RouteResult>`, discarding the tuple's stats half) around the above. Confirmed via Milestone 0's census that this specific free function has **zero production call sites** in `src/py_router.rs` -- if Milestone 1 confirms this holds, no call site needs migrating for it, but the now-unused import should still be checked.
- `route_single_net_with_crossing_config` (line 2359) -- maps directly to `search_with_crossing_config`. No discard-stats variant exists for this one since it never returned stats to begin with.

### Toolchain notes

Identical to the modular-routing-strategies plan's own Toolchain notes section -- this machine's Rust toolchain needs the same `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python"` override for `cargo check`/`cargo test`, and `maturin develop --release` must be rerun after any `src/*.rs` change before Python-level validation will see it. Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16`.

## Plan of Work

**Milestone 1** confirms, by reading each of the 12 call sites listed in Surprises & Discoveries directly (not assuming from the free-function name alone), that the arguments passed at that call site match the corresponding trait method's parameter list exactly (same order, same types, same `Option`/reference shapes) -- this is expected to be true by construction (the trait methods were written to delegate to these exact free functions), but must be confirmed per site since PLANS.md's own discipline this repository has followed all session says to verify rather than assume. Records the exact substitution for each of the 12 sites (which trait method, and for the 5 discard-stats sites, the exact local-variable name and placement for the throwaway `RouteSearchStats`) in the Decision Log before Milestone 2 begins. Also confirms whether `route_single_net_with_collision_crossing_config`'s zero-call-site finding still holds, and whether `use crate::astar::{...}`'s import list can drop any of the seven free-function names once all call sites are migrated (some may still be needed if a test inside `src/py_router.rs` itself, if any, calls them directly -- check before removing).

**Milestone 2** implements Milestone 1's design across all 12 call sites in `src/py_router.rs`, replacing each free-function call with the matching `AStarSingleNetSearch`-method call, and trims the `use crate::astar::{...}` import list to drop any of the seven free-function names no longer referenced anywhere in the file (adding `SingleNetSearch` and `AStarSingleNetSearch` to that same import list, since the trait must be in scope for the `.search(...)`-style method-call syntax to resolve). Given the scope is 12 mechanical, well-specified substitutions with no correctness subtlety comparable to the modular-routing-strategies plan's Milestone 6 (no behavioral divergence to reconcile, no snapshot/restore timing to reason about -- confirmed in Milestone 0/1 that the trait bodies are exact pass-throughs), dispatch this to Codex via `.agent/scripts/codex_task.sh` per `.agent/CLAUDE_CODEX_FLOW.md`'s default, unless Milestone 1 surfaces a specific reason not to. Validate: `cargo check --lib` and `cargo test --lib` (expect `393 passed, 0 failed`, unchanged) before any Python-level check; then `maturin develop --release`, full `pytest -q` (expect byte-identical to the `11 failed, 335 passed, 1 skipped` baseline), `benes_4x4`, `multiportmmi_8x8` (both bare defaults and stable-baseline config).

**Milestone 3** runs the remaining full-ladder items not already covered by Milestone 2's own validation (`multiportmmi_16x16` stable-baseline, expecting the same pre-existing `n_50` failure signature already documented, not a regression), writes this plan's Outcomes & Retrospective, and updates `.agent/REPOSITORY_STATE.md` and `.agent/ORCHESTRATOR.md`'s Active ExecPlan pointer.

## Concrete Steps

From the repository root (`/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`, but do not assume this exact absolute path):

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" .venv/bin/python -m maturin develop --release
    PYTHONPATH=. .venv/bin/pytest -q
    rm -rf build/routes build/verification && PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4
    rm -rf build/routes build/verification && PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8
    rm -rf build/routes build/verification && PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 0
    rm -rf build/routes build/verification && PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 0

`multiportmmi_8x8` bare CLI defaults is expected clean (`error_count=0, warning_count=2`, the two pre-existing benign `endpoint_correction_fallback_used` notices), confirmed current as of 2026-08-24 (do not trust older plan text claiming otherwise without re-checking, per the modular-routing-strategies plan's own corrected mistake on this exact point). `multiportmmi_16x16` stable-baseline is expected to fail with the exact pre-existing `RuntimeError: No route found for n_50` signature.

## Validation and Acceptance

After Milestone 2: `grep -n "route_single_net_with_config\b\|route_single_net_with_config_reporting_stats\|route_single_net_with_dynamic_expansion_config\|route_single_net_with_collision_crossing_config\|route_single_net_with_crossing_config" src/py_router.rs` shows matches only inside the `use crate::astar::{...}` block for whichever names are still needed (ideally none, if all are fully replaced and the import list trimmed); `cargo test --lib` `393 passed, 0 failed`; full `pytest -q` byte-identical to baseline; `benes_4x4` and `multiportmmi_8x8` (both configs) clean with unchanged error/warning counts.

After Milestone 3: `multiportmmi_16x16` reproduces the exact pre-existing `n_50` error text; `.agent/REPOSITORY_STATE.md` and `.agent/ORCHESTRATOR.md` reflect this plan's completion.

## Idempotence and Recovery

Every milestone that changes code must be validated against the byte-identical ladder before being considered complete. If a milestone's change does not validate, revert it via `git` (check `git status` first) and retry from the relevant milestone's design step. Build artifacts under `build/routes`, `build/verification`, and `target/` are gitignored and safe to delete/regenerate at any point.

## Artifacts and Notes

The 12-call-site census (enclosing method, free function, line number) is recorded in full in Surprises & Discoveries above -- treat that as the authoritative starting list for Milestone 1, re-verified via `grep -n` immediately before editing rather than trusted from this document alone, per this repository's own repeated lesson that characterization claims (including this plan's own) need active re-verification before being acted on.

## Interfaces and Dependencies

In `src/py_router.rs`, all 12 call sites listed in Surprises & Discoveries: replace each free-function call with the matching `AStarSingleNetSearch`-method call (`AStarSingleNetSearch.search(...)`, `.search_with_dynamic_expansion(...)`, `.search_with_collision_crossing(...)`, or `.search_with_crossing_config(...)`), adding a local `let mut discarded_stats = RouteSearchStats::default();` immediately before the call at each of the discard-stats sites. Add `SingleNetSearch` (the trait, for the method-call syntax to resolve) and `AStarSingleNetSearch` to the `use crate::astar::{...}` import block at the top of the file; remove any of the seven free-function names from that same block that become unused as a result (confirmed per-name via `grep`, not assumed).

Do not change `src/astar.rs`'s trait definition, `AStarSingleNetSearch`'s implementation, or any of the seven free functions themselves -- this plan only changes which syntax production code uses to reach the same underlying behavior.
