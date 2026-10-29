# Changelog

All notable changes to this repository. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the project has no
versioned release yet.

## Unreleased

The do-over of 2026-10 (analysis and phase plan: the owner's
`working/.agent/TUMPhotonicRouter/DO_OVER_ANALYSIS.md`). Phase 0, hygiene
and freeze, changes no routing behaviour; the nine-cell paper reproduction
(`scripts/results/gate_short.sh`) is the check for every slice.

### Changed

- The crossing plan comes from the placement, not from the layered-DAG
  depth/rank analysis (phase 4 slice C). New
  `python/photonic_router/crossing_demand.py`: `build_crossing_demand(
  schematic, unrouted_layout)` clusters the placed instances into columns
  (overlapping or touching x-extents), treats the space between two
  consecutive columns as a band, confines a net to the band its two
  horizontal ports point into, orders the terminals on each wall top to
  bottom (equal y: by x, then net name) and takes the pairs whose order
  differs between the walls as the band's invariant crossings;
  `CrossingDemand.lower_bound()` is their sum (the crossing count of every
  band-confined routing), `counts_by_net_name()` the per-net share, and
  `outside` lists the nets the model does not cover with a reason
  (`same_wall`, `non_adjacent_walls`, `vertical_port`, `chip_boundary`; none
  on the 17 paper benchmarks). `crossing_plan.build_crossing_plan` takes an
  iterable of the new `LaneRank` (edge key, the stage's two indices, the two
  ranks) instead of a `TopologyAnalysisResult`, so the band orders go through
  the same bubble-sort sequencing as the depth/rank tables did.
  `translation/route_rust_crossing_plan._build_crossing_plan_info` takes the
  `unrouted_layout` and builds the demand (`plan_text` describes the bands;
  the Rust interface `set_crossing_guidance(pairs, loss, single_per_pair)` is
  unchanged); `translation/preplaced_crossing_grids.
  build_crossing_plan_for_benchmark(schematic, unrouted_layout)` returns the
  demand's plan for contribution 2. The `node_depths` / `node_ranks` /
  `edge_ranks` keywords left `route_match_and_realize`, `RoutingSession`,
  `SessionSettings` and the optical stage; the Benes `TOPOLOGY_METADATA`
  tables stay in the benchmark modules as descriptive metadata and the test
  oracle but no routing code reads them. `topology_analysis.py` has no
  production reader and is kept as that oracle: `tests/test_crossing_demand.py`
  and `tests/e2e/test_crossing_demand_equality.py` assert that the demand
  reproduces its stages, wall orders, pairs, per-net counts and ordered
  events (level and order index) on all 17 paper benchmarks (Benes 4x4 to
  128x128, plain and `_flat`, ADEPT 8x8 to 128x128); `GraphAnalysisContext`
  lost its topology product. `scripts/results/crossing_lower_bound.py`
  (`lower_bound_for_benchmark`) is the helper the reproduction table will
  subtract in a later slice. Same guidance pairs, same tiles in the same
  order; the nine-cell gate is identical.
- The negotiated batch takes `RouteJob` objects (phase 3 slice E):
  `PyPhotonicRouter.route_many_with_negotiated_repair_and_commit` accepts a
  list of the new `photonic_router._rust.RouteJob` (`#[pyclass]` in
  `src/bindings/types.rs`; keyword-only constructor over the eight fields
  `net_id`, `source`, `target`, `opened_cells`, `clearance_exempt_cells`,
  `foreign_keepout_cleanup_cells`, `source_port_um`, `target_port_um`, each
  readable) instead of eight-field tuples, and `native_jobs_from_py` converts
  them to the engine's `NativeRouteJob` once at the boundary. On the Python
  side `translation/routing/dispatch.py::native_route_jobs(settings, state,
  jobs)` is the one packer; the first routing, the detour pass
  (`detour_pass.py`) and the two verify-repair passes (`verify_repair.py`)
  built the same eight values in four copies before and call it now. The
  batch result dictionary is unchanged; the nine-cell gate is identical.
- Seven `PyPhotonicRouter` methods Python never called but the engine does
  (`invalidate_meander_base_prefix`, `ensure_meander_base_prefix`,
  `ensure_meander_registered_reserved_index`, `clear_static_cells`,
  `add_history_cells`, `long_straight_congestion_records`,
  `effective_bend_radius_um`) moved out of the `#[pymethods]` block into a
  plain `impl` block, so they leave the Python surface (98 -> 81 pymethods
  with the removals below); `python/photonic_router/_rust.pyi` regenerated.
- The negotiated loop in its 150-line shape (phase 3 slice D;
  `src/engine/negotiation/loop_.rs` 1,377 -> 1,278 lines, the module 4,084
  -> 4,046): `run_negotiated_batch` (103 lines) is the rounds, the per-net
  ladder is `route_net_this_round(..) -> NetOutcome` (100 lines), and the
  setup, the per-net `SearchArgs`, the braid requeue and the post-rip-up
  guidance are small helpers. `global_ripup_round` takes `&mut NetQueue`
  instead of nine borrows of its fields, which are private again. The
  `RipUpPolicy` trait (`LidarStyleRipUp`, `NoRipUp`) is a plain `if
  no_ripup` in the loop; its two wrapper tests became a test of the function
  and an end-to-end test of the `no_ripup` branch. Every
  `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` line of the loop but the per-attempt
  one prints from `negotiation/trace.rs`. Same searches, commits, rip-ups,
  requeues, counters, trace text and errors; the nine-cell gate is identical.
- `translation/routing/dispatch.py` is the kernel dispatch only (1,941 ->
  569 lines, phase 3 slice C): its two near-identical branches are one
  (`max_rounds=1, no_ripup=True` for the test-only no-repair baseline), and
  the fourteen route/attempt/failure write-up functions moved verbatim to
  `translation/routing/diagnostics.py`. The timing printout and the bucket
  table name only the buckets the kernel produces (`normal_route`,
  `probe_route`, `braid_ripup`, `endpoint_correction`).
- `rust-toolchain.toml` pins the host's `stable` toolchain. Linux and macOS
  build with plain `cargo` and `maturin`; the Windows-without-Visual-Studio
  setup is a local, git-ignored configuration described in
  `docs/WINDOWS_RUST_TOOLCHAIN.md`.
- The Python sources are formatted by `ruff format` and pass `ruff check`
  with the rule set in `pyproject.toml`; the remaining scoped ignores are
  listed there with their reasons.
- `docs/CONFIGURATION.md` regenerated (a union type's member order).
- The simple-route (straight/L/Z) shortcut runs in one place, the engine's
  probe before each ordinary search; the second copy inside the single-net
  search body is gone (phase 2 slice B). The static-only probe keeps it
  explicitly. Routes are unchanged on the nine-cell gate; the single-net
  `route_single_net` API now always runs the kernel.
- One bend weight and an admissible heuristic (phase 2 slice E).
  `PrimitiveLibraryConfig.bend_weight` is gone (it was always 1.0 and was
  multiplied into `AStarConfig.bend_weight`), and so are
  `AStarConfig.heuristic_weight`, `SearchTuning.min_heuristic_weight`,
  `PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT` and the weighted-A* iteration cap in
  `router_setup.py`. No paper cell ever ran a weight other than 1.0, so the
  27-cell reproduction is unchanged; the Dijkstra oracle keeps its zero
  heuristic.
- `ObstacleMap` is a dense per-cell grid database (phase 2 slice D). Every
  cell owns one record: a flags byte (static cell, static rectangle, dynamic
  halo, dynamic core), three `u16` reference counts, the dynamic owner, and
  lazily allocated history and congestion costs (15 bytes per cell, 23 with
  costs). It replaces the five hash maps keyed by cell, the per-row static
  rectangle interval index and the two cost hash maps; static rectangles are
  rasterised into their flag bit when set, so every "is this cell free"
  question reads one byte, and the dense search grid is built by one pass
  over the window instead of walking rectangles, static keys and dynamic
  keys. The cold planes are shared copy-on-write, so the engine's per-net
  static-only probe clone costs what it did before. Public API unchanged;
  the per-cell accessors now iterate in grid order (all callers sort or
  build sets). Nine-cell gate identical.
- One simple-route module (phase 2 slice F): `src/simple_routes.rs` (the
  straight/L/Z/turnaround candidates, their obstacle-query trait and the grid
  geometry) and `src/search/astar/simple.rs` (the adapter to a `RouteResult`)
  are merged into `src/search/astar/simple_route.rs`. The two engine entry
  points keep their names and signatures but share one body, differing only
  in the obstacle query the candidates are checked against; the fifteen
  thin `ObstacleMap` wrappers that only tests used, the unused `rect_free`
  query method and the crate-root re-exports nothing consumed are gone, and
  everything in the module is `pub(crate)`. The module doc states the one
  deliberate difference to the kernel's legality (congested cells are
  rejected; the expanded query checks core cells plus a diagonal halo),
  kept as is until phase 5. No legality rule, candidate order or returned
  route changed; nine-cell gate identical.
- One route-and-commit body driven by a `SearchCall` value (phase 3 slice
  B). The two copy-pasted single-net bodies (`route_single_net_and_commit_native`,
  12 parameters, and `..._repair_native`, 13) are one `route_and_commit(&SearchCall)`;
  what used to tell them apart is now data on the call: the A* overrides
  (`ignore_dynamic`, the probe's `enable_simple_routes`, the history weight),
  the collision try order (`PlainFirst`/`CrossingFirst`), whether the simple
  probe and the deferred lidar-pure crossing search run, and the keepout retry
  (a plain call retries once as a repair call at history weight 0.0). The
  four per-attempt fields the negotiated loop used to set and reset on the
  router (`negotiated_search_budget`, `negotiated_crossing_free_search`,
  `commit_history_weight`, `long_straight_weight_override`) are gone; the
  budget, the crossing-free flag and the two weights arrive on each call, the
  probe search takes the net's long-straight override as a parameter, and
  `astar_config` takes the budget and the override as parameters. The
  three-level collision-crossing wrapper chain is one function. The Python
  `route_single_net_and_commit` builds a `NativeRouteJob` and makes a plain
  call. No search, commit, error string or trace line changed; nine-cell gate
  identical.

### Added

- `python/photonic_router/_rust.pyi`, the type stub of the Rust extension,
  generated from the built module by `scripts/generate_rust_stubs.py`, and
  `py.typed`.
- GitHub Actions CI (`.github/workflows/ci.yml`): rustfmt, clippy, the Rust
  tests, ruff, the generated-file checks and the fast Python suite on every
  push; the nine-cell reproduction gate nightly.
- `CONTRIBUTING.md`, `CHANGELOG.md`, `.pre-commit-config.yaml`.
- `NegotiationConfig.disable_post_ripup_guidance`
  (`PHOTONIC_ROUTER_NEGOTIATED_DISABLE_POST_RIPUP_GUIDANCE`, ablation variant
  `no_post_ripup_guidance`), default off (phase 3 slice A): runs the
  post-rip-up search unguided even when the probe reported legal crossing
  partners, so the one guidance the 2026-10 ablation did not cover can be
  measured next.

### Removed

- The hard crossing-constraint API, whose only production writer was
  `router.set_crossing_constraints([])` (phase 4 slice A): Rust
  `CrossingConstraint`, `CrossingContext::{constraints, allowed_pairs,
  expected_crossings_by_net, has_expected_pair, allows_pair,
  expected_crossing_count, allowed_partners_for, ordered_constraints_for,
  replace_constraints, clear_constraints}` and
  `CrossingConfig::allow_only_expected_pairs` (always `False` from Python,
  so `allows_pair` had reduced to `config.enabled`); the pyclass
  `CrossingConstraint` and the six pymethods `crossing_constraints`,
  `set_crossing_constraints`, `clear_crossing_constraints`,
  `crossing_expected_count`, `crossing_has_expected_pair`,
  `crossing_allows_pair` (81 -> 75 pymethods); the Python keyword
  `allow_only_expected_crossings` of `route_nets_rust` / the session /
  `SessionSettings` with `effective_allow_only_expected_crossings`, and the
  `CrossingPlanInfo` fields `constraint_count`,
  `requested_allow_only_expected_crossings`, `allow_only_expected_crossings`.
  The `unexpected_pair` verdict is gone from the Rust and the Python
  verifier; the legality predicates themselves are untouched.
- Everything that was constant because of it: `net_has_crossing_requirements`
  and the crossing spacing history (`add_crossing_spacing_history_for_route`,
  `crossing_spacing_history_cells_for_route`,
  `insert_crossing_reservation_window`, `CROSSING_SPACING_HISTORY_AMOUNT`);
  `CrossingSearchConfig::require_all_partners` with `all_partner_mask` /
  `all_partner_count`, the `goal_extra_ok` and `heuristic_bonus` hook methods
  (`crossing_progress_heuristic`, `distance_to_trimmed_grid_segment`,
  `distance_to_segment`), the window-mode partner tracking in
  `evaluate_crossing_move` and the `next_partner_index` component of the
  crossing search key (always 0); the aliases `lidar_pure_crossing_enabled`,
  `lidar_pure_owner_lookup_partner_set` and the unreachable non-lidar branch
  of `upfront_collision_crossing_partner_ids` with
  `crossing_partner_lookup_set_for_route` /
  `lidar_route_window_partner_lookup_set`.
- `min_straight_cells_per_crossing` end to end: the
  `--min-straight-cells-per-crossing` flag of the CLI and
  `scripts/benchmark_photonic.py`, the flow and session keywords,
  `SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING` /
  `DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING`, the `CrossingConfig` /
  `CrossingSearchConfig` fields and the `CrossingPlanInfo` field.
  `crossing_required_margin_cells` never read it (half size plus bend
  run-out); the trace line `collision-crossing start` loses its
  `min_straight=` token.
- The dead second and third copies of the rank-inversion analysis:
  `TopologyCrossing`, `TopologyAnalysisResult.crossings` /
  `crossings_by_depth`, `_find_rank_inversions`
  (`python/photonic_router/topology_analysis.py`) and
  `benchmarks/benes.py::find_rank_crossings` / `BenesCrossing` with the
  `crossings` / `crossings_by_stage` metadata keys, the benchmark modules'
  `EXPECTED_CROSSINGS` and `load_benchmark_metadata()["expected_crossings"]`.
  `crossing_plan.build_crossing_plan` is the one copy; the tests assert its
  event counts (2, 16, 88, 416 for Benes 4..32, 33 for multiportmmi_8x8) and
  that every event is a genuine rank inversion.
- The third `realized_intersections` producer
  `_augment_crossing_plan_with_realized_overlaps` (and
  `_first_perpendicular_route_intersection`, `_segment_length_cells`) with the
  `CrossingPlanInfo` fields `actual_crossing_count`,
  `actual_crossing_cell_count`, `actual_geometric_crossing_count`,
  `actual_crossings`, `unrealized_expected_crossings`,
  `unrealized_expected_crossing_count`, `actual_crossing_reason` and the
  `realized_crossing_overlap_augment` timing bucket;
  `scripts/crossing_benchmark_report.py` reports `planned_crossings`
  (`event_count`) and `actual_crossings` (`native_crossing_event_count`)
  instead.
- The unreachable pre-placed grid builders of
  `translation/preplaced_crossing_grids.py` (2,663 -> 1,383 lines): fan modes
  `router`, `in_grid` and `stretched` (`build_crossing_grid_component`,
  `_build_stretched_grid_component`, `_run_placement`,
  `split_stage_into_participating_runs` and their helpers) and the `lines`
  tile placement (`_derive_crossing_tiles_on_lines`), with the
  `CrossingGridGeometry` / `CrossingGridConfig` fields `fan_mode`,
  `tile_placement`, `lane_pitch_um`, `entry_straight_um`,
  `fan_column_pitch_um`, `port_pair_spread_um`,
  `unrouted_sibling_clearance_um`, `slot_spread_um`, `stub_stagger_um`,
  `tile_min_spacing_um` and their ten `PHOTONIC_ROUTER_CROSSING_GRID_*`
  variables, none of which any paper cell set. The `tiles` builder every
  paper cell ran is unchanged.
- `is_lidar_mode`, `LIDAR_MODES` and `CROSSING_MODE_ALIASES`
  (`translation/crossing_modes.py`): the predicate had been constant true
  since the collision mode went, and the command line only ever accepted the
  two canonical names; `normalize_crossing_mode` now rejects the aliases
  `pure`, `lidar`, `guided`, `lidar_guided`.
  Nine-cell gate identical.

- Ten dead `PyPhotonicRouter` pymethods of the negotiation, history, dynamic-
  and static-map and crossing-guidance surface with no caller in Python
  (production, tests or scripts) or Rust (phase 3 slice E):
  `add_history_for_route`, `clear_history`, `clear_dynamic`,
  `clear_port_open_cells`, `clear_crossing_guidance`,
  `crossing_guidance_planned_pair_count`, `commit_route_with_clearance`,
  `inflated_route_cells`, `dynamic_owners_for_route`,
  `dynamic_owners_for_cells`, and with them the engine helper
  `remove_crossing_events_for_all_routes` only `clear_dynamic` reached.
  `ObstacleMap::clear_history` and `::clear_dynamic` stay (the loop uses
  them). The meander, PLM, realisation, endpoint-correction and port-access
  pymethods without callers are untouched; they belong to phases 5 and 6.
- The two negotiation tactics the 27-cell ablation found inert (phase 3
  slice A): the legal-only probe-guided attempt and the direct-crossing
  attempt (`try_lidar_direct_crossing_subset`, the `lidar_direct_crossing`
  bucket, `AttemptKind::ProbeGuided`/`DirectCrossing`/`Braid`, the
  three-function `commit_native_route_with_clearance_*` chain only the
  direct-crossing attempt reached), with their switches
  `disable_probe_guided` and `disable_direct_crossing`
  (`PHOTONIC_ROUTER_NEGOTIATED_DISABLE_PROBE_GUIDED`,
  `PHOTONIC_ROUTER_NEGOTIATED_DISABLE_DIRECT_CROSSING`). The mixed-case
  guidance of the post-rip-up search stays as it was, widening included;
  `disable_source_column_widening` now gates that widening. Also gone, all
  write-only: the pending-straight victim hint (`PendingStraightVictimHint`,
  `pending_straight_ripup_threshold`,
  `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD`), the batch result's
  `deferred_count` (always 0; the `deferred=` token of the route-search
  summary line goes with it), the `commit_rejected` counter of the
  `native_negotiated_done` trace line (always 0), and the probe's trace-only
  `probe_repair_keepout_keys` with its two keep-out helpers. Nine-cell gate
  identical.

- The JPS4 accelerator (`src/search/astar/dense.rs`), the two unit-grid
  primitive libraries (`jps4_unit_grid`, `grid4_unit_grid`) and the
  `--enable-jps4` flag (phase 2 slice A1). JPS4 was eligible only for a
  plain 4-connected unit primitive library, which the photonic library never
  is, so no production run could reach it; the `jps4_*` route-stat fields go
  with it.
- The pure-Python static obstacle builder
  (`build_static_obstacle_map_python_from_extracted`, its rasterise and
  point-in-polygon helpers) and the legacy-signature retry shim around the
  Rust builder (phase 2 slice A2). Routing needs the extension anyway;
  without it `build_static_obstacle_map` now raises an `ImportError` with a
  rebuild hint. A heater layer group without geometry keeps a 20-line
  port-opening constructor so the split merge is unchanged.
- Tracked IDE settings (`.idea/`, `.vscode/`) and the per-machine
  `.cargo/config.toml`; `Agent_implementation_files/` (superseded by
  `docs/ARCHITECTURE.md`); `benes_test/` (an early experiment with three GDS
  files); `rust_router_reference/` (an unrelated pose router).
