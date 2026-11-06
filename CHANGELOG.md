# Changelog

All notable changes to this repository. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the project has no
versioned release yet.

## Unreleased

The do-over of 2026-10, in nine phases: 0 hygiene and freeze, 1 ablation switches, 2 routing
database and single-net search, 3 negotiation loop, 4 crossings, 5 ports (open), 6 path-length
matching, 7 electrical (open), 8 release layout (one package, a Cargo workspace, benchmark TOML,
documentation). Each slice preserves the routed results unless its entry says otherwise; the
checks are `scripts/results/gate_short.sh` and, since phase 6, `scripts/results/plm_gate.sh`.

### Added

- Phase 5 slice 5A, experiment switch `PHOTONIC_ROUTER_PORT_RUNWAYS=1`
  (`FanoutAccessConfig.port_runways`, default off = exact): every cardinal
  port without a fan-out stub gets a runway, a static stub from the port
  centre onto the cell-centre lattice (`route_jobs._port_runway_anchor`: a
  0.5 um straight lead-in, two shallow arcs of the bend radius that shift the
  centerline by the port's sub-cell lateral residue, a straight run to the
  first cell centre at or beyond the arcs, at least one cell outward). The
  anchor is that cell centre with the port's heading; the search starts and
  ends at the anchors exactly as it does for the fan-out stubs
  (`fanout_anchor_by_port_spec`), the stub is committed static and the
  realized route is stub + search path, so endpoint correction has nothing to
  move. A net whose two ports are closer than their two runways (the
  pre-placed crossing grids' tile links) keeps the old path on both ports,
  and `static_fanout_anchors_um` (the crossing structures' entry-tile probe)
  reports the fan-out anchors only: runways are lattice entries, not fan-out
  lanes (with them in the probe the ADEPT 8x8 contribution-2 planner lost a
  column grid and fell back to the guided router).
  `_append_target_fanout_stubs_after_correction` now appends the reversed
  stub (any shape) before the true port; for the straight fan-out stubs
  that is the same two points as before. Nine-cell table with the switch on:
  crossings identical on every cell, 0 verifier errors, Benes lengths
  +0.5-0.6 % (4x4: 9484.511 -> 9537.623, 8x8 lidar-pure 31418.224 ->
  31571.384), ADEPT 8x8 lidar-pure 25171.624 -> 25110.519 (-0.24 %),
  contribution 1 25090.058 -> 25078.927, contribution 2 26319.97 ->
  26457.472 (+0.52 %), and the number of arcs per net
  rises from 2.2 to 6.6 (Benes 8x8) and 2.9 to 5.1 (ADEPT 8x8): two shallow
  arcs (3 to 17 degrees at R = 10 um) per runway port. Decision (default or
  not, with the deletion of the correction machinery in 5B) by the 27-cell
  table and the owner's view of the arcs; `docs/CONFIGURATION.md`
  regenerated. Correction after the first 27-cell table with the switch on
  (b05bffd: six cells failed on crossing legality, ADEPT 32x32 lidar-pure
  121 -> 123 crossings): the runway anchors had entered every consumer of
  `fanout_anchor_by_port_spec`, including the five that carry dense fan-out
  *semantics*, so each runway port's access footprint shrank to the stub
  lane size 0, its siblings formed a dense cluster, its nets were exempt
  from the long-straight penalty, the dense target reservations skipped it
  and the footprint was laid from the anchor. `SessionState.fanout_stub_anchor`
  now hides runway anchors from those five (`_resolve_port_footprint_cells`,
  the port-opening centre, the dense lateral windows,
  `_dense_target_port_runway_lengths`, the static stub clusters,
  `apply_long_straight_fanout_exemption` via `fanout_stub_net_ids`, the
  `topological-stub` order); the geometry consumers (search endpoints, stub
  prepend, static cells, endpoint correction) still take every anchor.
  Switch off exact (nine-cell gate); with the switch on the nine cells keep
  their crossings and 0 errors, and the lengths now read Benes 4x4
  lidar-pure +1.00 % (9579.669), contribution 1/2 +0.57 / +0.54 %, Benes 8x8
  +1.12 / +0.59 / +0.54 %, ADEPT 8x8 -0.04 / +0.23 / +0.69 %: the
  lidar-pure Benes cells lost the shortcut the collapsed footprints had
  given them. Benes 16x16 lidar-pure routes again (88 crossings, 0 errors).
- `SearchTuning.routing_window_max_expansions` /
  `PHOTONIC_ROUTER_ROUTING_WINDOW_MAX_EXPANSIONS` and
  `SearchTuning.routing_window_fallback_full_grid` /
  `PHOTONIC_ROUTER_ROUTING_WINDOW_FALLBACK_FULL_GRID` (speed pass S4, both
  unset by default = the kernel's ladder of 3 expansions with the full-grid
  fallback): env-only strategy switches for the window ladder, with the
  ablation variants `window0_only` and `window0_full_budget`. The ADEPT 64
  lidar-pure per-window log showed every success of every attempt kind in
  window 0 and the later windows plus the fallback only burning the budget
  tail of attempts already lost (about 20 % of attempt time); not exact, so
  the 27-cell table decides.
- `NegotiationConfig.first_attempt_full_budget` /
  `PHOTONIC_ROUTER_NEGOTIATED_FIRST_ATTEMPT_FULL_BUDGET` (speed pass B1,
  default off): a never-failed net's first attempt runs at the retry budget
  and the retry is skipped. Experiment 1 showed the heavy cells spending 20
  to 60 % of all expansions on 2 M first attempts that then failed; the
  change is not exact (the per-window cap follows the budget), so the 27-cell
  table decides.
- The path-length-matching baseline and gate (phase 6 slice 0): `scripts/results/plm_baseline.sh`
  routes `heater_s_mod` and the twelve `cascade_{8,12,16}_{compact,diagonal,folded,
  folded_compact}` under their stable flags into `results_plm/` (config label `plm`,
  one benchmark per call, `--list` for GNU parallel, `STOP` file, 1 h timeout);
  `scripts/results/plm_table.py` reads the latest run per benchmark (run.txt,
  metrics.json, the SUMMARY line and the flow's `path_length_analysis` /
  `meander_insertion_report` / `electrical_routing` results from the routed GDS's
  kfactory meta info) into a Markdown table, `--json` pins it, `--compare` checks a
  root against `docs/plm_baseline.json` on rc, verifier errors, nets, GDS length,
  meander length, group count, groups over tolerance and the realized group and
  output spreads (lengths to 0.01 um; times are information only);
  `scripts/results/plm_gate.sh` is the short gate (two cells, about 10 s).
  `docs/plm_baseline.json` was captured at 6f1a46d on cda-server-4 on 2026-10-29:
  all thirteen route clean (rc 0, 0 errors, 0 groups over tolerance, spread 0);
  `docs/PLM_REPRODUCTION.md` has the table and the procedure.
- `scripts/results/compare_paper.py --with-bounds` adds the placement-derived
  lower bound on crossings (band inversions, `scripts/results/crossing_lower_bound.py`,
  table `docs/paper_crossing_bounds.json` written by its `--json` option) and the
  excess "realised minus unavoidable" per cell (phase 4 slice E). On the 27 paper
  cells the excess is 0 everywhere except ADEPT 64x64 lidar-pure (+8).
- One crossing-legality predicate behind a switch (phase 4 slice B).
  `RouterConfig.crossing.unified_legality` (`PHOTONIC_ROUTER_UNIFIED_CROSSING_LEGALITY`,
  presence; Rust `CrossingEngineConfig`, `PyRouterConfig.crossing_unified_legality`,
  the Python dataclass, the env overlay; default off). On, the search hook, the
  commit check and the post-commit validation all decide a crossing with
  `crossing_geometry::crossing_legality(&CrossingCandidate, &CrossingLegalityRule)
  -> CrossingVerdict` (`Legal`, `NotPerpendicular`, `MarginShort { side }`,
  `FootprintBlocked`, `MissingEvent`): one perpendicularity test (normalised dot
  product within 1e-6), one margin `half_size + bend_runout` cells on the grid
  route for the run before the crossing and both sides of the partner, `half_size`
  pure straight cells after it (the search's debt rule), the footprint check
  (`footprint_blocked`: a third net's dynamic cell in the element's window, the
  obstacle-map form of the Python footprint-blocker check) and the registered
  event. The post-commit validation (`unified_crossing_assessment`) registers the
  legal assessments as the net's `CrossingEvent`s and records one
  `CrossingVerdictRecord` per realized intersection; the new
  `PyPhotonicRouter.crossing_verdicts()` reports them (legal, reason, the four
  straight runs and the two required margins in cells, perpendicularity,
  footprint, event, the matched grid segments). `translation/
  route_rust_crossing_verification.py` then classifies realized crossings from
  those verdicts (`legality_source`, `rust_verdict` on each record;
  `required_margin_um` = `search_required_margin_um` = `required_margin_cells *
  grid`; an intersection Rust never assessed is `missing_event`) instead of
  re-deciding them; the repair loop maps the new reasons onto its tactics
  (`_repair_reason`); `CrossingPlanInfo` carries `native_crossing_verdicts`,
  `native_crossing_verdict_count`, `unified_crossing_legality`. Off, the three
  legacy paths and the Python re-decision run byte-identically (nine-cell gate:
  all reproduced cells match); the 27-cell ablation decides, and a later slice
  deletes the losing side.

### Changed

- Release decision 24 (owner, 2026-11-05): the paper's venue leaves file names
  and texts while the review is double-blind. `reproduce_date2027.sh` ->
  `reproduce_paper.sh`, `compare_date2027.py` -> `compare_paper.py`,
  `docs/date2027_table_sources.json` -> `docs/paper_table_sources.json`,
  `docs/date2027_crossing_bounds.json` -> `docs/paper_crossing_bounds.json`,
  `docs/DATE2027_REPRODUCTION.md` -> `docs/PAPER_REPRODUCTION.md`, results root
  `results_paper/`, reproduce.txt header "paper reproduction"; CITATION.cff
  names the venue as a TODO until acceptance. Unchanged on purpose: the tag
  `date2027-results`, the branch `DATE2027-results` and the release asset
  `results_date2027.tar.gz`, which `gh release download` resolves by name.
- Release decisions 20 and 21 (owner, 2026-11-05): the license is MIT
  (`LICENSE`, `pyproject.toml` `license` and classifier, `CITATION.cff`
  `license`; the copyright line names the authors collectively until the owner
  sets the legal holder), and the agent-process layer (`.agent/`, `.claude/`,
  `AGENTS.md`, 89 files of plans, roles and run logs) leaves the public tree;
  it lives on privately with the owner and the three paths are git-ignored.
  Comments that cited `.agent/...` plans now name the private layer.
- Path-length matching, phase 6 slice D (owner decisions 10 to 13,
  2026-11-05). Decision 11: `--path-length-allow-residual BOOL` is replaced by
  `--path-length-declared-unmatched-groups NAMES`
  (`PathLengthMatchingOptions.path_length_declared_unmatched_groups`): a
  benchmark names the groups it leaves unmatched on purpose, a residual there
  is reported (`plm_over_tolerance`) and the run finishes, a residual in any
  other group fails the run (before, the flag accepted every residual).
  Decision 12: the `RegisteredMeanderPlanner` trait and its only
  implementation `RustRegisteredMeanderPlanner` (310 lines, no production
  caller) are deleted; the free functions in `meander/plan.rs` are the
  planner. Decision 10: path-length matching with the crossing-aware search on
  is documented as unsupported and the flow warns when both are requested.
  Decision 13 (feedback reroute on `no_candidate`) is not built: none of the
  13 pinned PLM cells reaches `no_candidate`, so there is no case to measure
  it on; it stays a proposed item with its first measurement (a cell that
  provokes `no_candidate`, e.g. a reduced meander height) in the analysis.
  PLM gate exact.
- Crossing legality is one predicate, without a switch (owner decision
  2026-11-05 on the 27-cell ablation of phase 4 slice B: 20 cells identical,
  crossings never increase, ADEPT 64x64 baseline 217 -> 211 crossings with the
  paper's ladder). `RouterConfig.crossing.unified_legality`,
  `PHOTONIC_ROUTER_UNIFIED_CROSSING_LEGALITY`, `PyRouterConfig.crossing_unified_legality`
  and `CrossingPlanInfo.unified_crossing_legality` are gone; the search hook,
  the commit check (`invalid_crossing_intersections_for_route`), the
  pre-commit violation check and the post-commit validation call
  `crossing_geometry::crossing_legality` only, registration keeps the legal
  assessments' events, and `route_rust_crossing_verification.py` classifies
  every realized intersection from `crossing_verdicts()` (an intersection
  without a verdict is `missing_event`; the reasons are the predicate's:
  `not_perpendicular`, `margin_short_<side>`, `too_close_<other>`,
  `footprint_blocked`, `missing_event`). Deleted: the legacy search checks,
  `crossing_violations_for_realized_centerline` and the event-reconstruction
  fallback in `register_geometric_crossing_events_for_route`, the Python
  geometric re-decision with its footprint-blocker scan (about 950 lines with
  their tests; the kernel and engine tests that compared both paths now test
  the predicate). The paper's engine is the tag `date2027-results`. Cells
  that move carry a `current` block in `docs/paper_table_sources.json`.
  27-cell table (cda-server-4, 2026-11-05/06, switch on at 60f4957; this
  switch-free head is exact against it on the nine gate cells): 20 cells
  unchanged, seven moved, crossings never increase, 0 verifier errors. Benes
  32 baseline and contribution 1 +7.706 um each (416 crossings), ADEPT 8x8
  baseline 25160.335 -> 25171.624 um and contribution 1 25073.626 ->
  25090.058 um (33 crossings), ADEPT 32 contribution 1 334827.361 ->
  334811.783 um, ADEPT 64 baseline 213 -> 211 crossings and 1305375.37 ->
  1305454.237 um, ADEPT 64 contribution 1 1302768.792 -> 1302746.103 um
  (the last three against their window-0 `current`). Loop-time ratio
  0.35-0.81 against the paper's archive. Details in
  `docs/PAPER_REPRODUCTION.md`.
- The window ladder's default is window 0 only (speed pass S4, owner decision
  2026-11-05): `routing_window_max_expansions` 3 -> 0 and
  `routing_window_fallback_full_grid` on -> off in the kernel defaults and the
  `AStarConfig` binding signature. On the 27 paper cells every route ever
  found came from window 0 (per-window log of ADEPT 64 lidar-pure: 370 + 44 +
  92 successes in window 0, one in window 1, none from the fallback), the
  three further expansions and the fallback only spent the budget tail of
  attempts already lost. Results: 24 cells identical, ADEPT 64 lidar-pure 217
  -> 213 crossings and -0.02 % length, ADEPT 32 c1 +0.003 %, ADEPT 64 c1
  -0.0001 %; total wall 0.91 of the round-2 head on the server, ADEPT 64
  lidar-pure 0.88, ADEPT 32 c1 0.47. `docs/paper_table_sources.json`
  carries a `current` block for the three cells and `compare_paper.py`
  checks against it (status `(current)`); the paper's ladder is the ablation
  variant `paper_ladder` (`PHOTONIC_ROUTER_ROUTING_WINDOW_MAX_EXPANSIONS=3`,
  `PHOTONIC_ROUTER_ROUTING_WINDOW_FALLBACK_FULL_GRID=1`). The variants
  `window0_only` / `window0_full_budget` of the previous entry are gone
  (`first_attempt_full_budget` covers the second; B1 stays off: six length
  moves and Benes 16 c1 twice as slow).
- Speed pass A7 (phase 9), exact: the two straight-run geometry values of a
  primitive (initial straight run for its start angle, terminal straight
  run for its end angle) are computed once per primitive into
  `PrimitiveCrossingMetadata` instead of by a footprint walk per candidate
  in the legality check, the hook's no-contact path, the contact path, the
  dense push and the completion chain (`initial_straight_run_distance` falls
  back to the walk when a caller's angle is not the primitive's own). The
  lateral-congestion band tests each cell with one bounds check
  (`is_blocked_in_grid`) instead of two. 509 core tests, nine-cell gate
  exact, Benes 16x16 lidar-pure expansion counts identical; routing loop
  136.8 s (bc2c0d8) -> 104.7 s side by side (111.5 s before A7); round 2 so far
  about -23 %.
- Speed pass A5/A6 (phase 9), two exact-preserving kernel fixes from a
  callgrind profile with cache simulation and line tables (Benes 8x8
  lidar-pure, A4 build). A5: each extended node stores the bounding box of
  its active reservation keys (`KeyBounds`); the own-window re-entry check
  tests a candidate's witness cells against the box before scanning the keys
  (the keys x witnesses scan was 18 % of the instructions). A6: footprint and
  halo rows of up to three cells are tested bit by bit in the blocked bitset
  instead of through the 2D prefix sum (`SHORT_RUN_CELLS`); diagonal
  primitives are rows of one or two cells, and the prefix path was 23 % of
  the instructions, called 520 M times from the footprint, halo and hook
  checks. A cell outside the grid is blocked for both paths, so the answers
  are identical. A short row is one bounds test, one index and one or two
  word loads (`DenseBitset::run_clear`, equivalence test against per-bit
  `get` across word boundaries). 509 core tests, nine-cell gate exact; Benes
  16x16 lidar-pure expansion counts identical, routing loop 138.7 s
  (bc2c0d8) -> 113.1 s side by side with the first A6 (131.5 s with A4 alone
  in the earlier pair), 111.5 s with the word-level rows.
- Speed pass A4 (phase 9): the two reservation parent-chain walks (own-window
  re-entry, own-window overlap) follow a per-node `reservation_ancestor`
  link to the nearest key-bearing node instead of visiting every node of the
  chain. Callgrind on Benes 8x8 lidar-pure attributed 41 % of all
  instructions to the two walks (the second one shows up under the symbol
  `blocked_count_in_rect` because the stripped extension gives the local
  function no symbol of its own): under lidar-pure every chain below a
  crossing kept the old cumulative `chain_has_reservations` flag, so the
  early stop never fired and each candidate walked O(depth) nodes. The link
  replaces the flag; the same key-bearing nodes are tested, in the same
  nearest-first order, so the answers are unchanged (508 core tests, nine-cell
  gate exact).
- Speed pass A3 (phase 9): three exact-preserving kernel fixes found by
  `perf` on Benes 16x16 lidar-pure. The proactive lateral-congestion band is
  precomputed per primitive (`PrimitiveTables::lateral`) instead of rebuilt
  with a hash set per straight move; the two reservation parent-chain walks
  stop at the first node without `chain_has_reservations` and test the
  witness keys by linear scan instead of building a hash set per candidate;
  the extended-state map is sized from the window instead of growing from
  4096. Same cells, same membership tests, same map semantics; the nine-cell
  gate is exact and Benes 16x16 lidar-pure drops from 318 s to 182 s on the
  workstation under equal load (identical attempts and simple-route counts).
  A4 measured side by side against the A3 build on the same cell: routing
  loop 145 s -> 131 s, per-net expansion counts identical.
- Release documentation and metadata (phase 8 slice D; documentation only, no
  code or behaviour change: nine-cell gate exact). `README.md` rewritten for a
  user (what the router does, install, quick start, the 27-cell paper table
  with the placement lower bound, reproduction, status with the license
  pending); `docs/ARCHITECTURE.md` rewritten against the current tree (the
  pipeline stage by stage, the data model, both crates and the package with
  line counts, the configuration surface, the oracles, how to add a benchmark,
  the experiment switches, the invariants; the previous text stays in git
  history); `CONTRIBUTING.md` refreshed (build, test, lint, the two gates, the
  re-pin rule for both oracles, the generated files, the commit convention).
  `pyproject.toml` gains `description`, `readme`, `authors` (placeholder),
  `keywords`, `classifiers`, `[project.urls]`, and `requires-python` becomes
  `>=3.12` (CI runs 3.12; `cli.py` needs `tomllib`); no `license` field.
  `CITATION.cff` added with the paper as `preferred-citation`, authors and the
  license left as owner placeholders. The documents that describe earlier
  states (`repository_finished_state.md`, `photonic_baseline.md`,
  `profiling.md`, `crossing_baselines.md`, `crossing_flow_repair_ideas.md`, the
  two LiDAR comparisons) moved to `docs/history/` with a README;
  `docs/astar_quality_baseline.json` stays because `scripts/profile_astar.py`
  reads it.
- Benchmark defaults as one TOML file per benchmark (phase 8 slice C; no
  behaviour change: nine-cell and PLM gates exact, the parsed flags and
  overlay identical for every benchmark). The `STABLE_ROUTING_FLAGS` /
  `STABLE_ROUTING_ENV` module attributes are gone; `benchmarks/<name>.toml`
  holds a `[flags]` table (flag names without `--`, in order) and a `[config]`
  table (`PHOTONIC_ROUTER_*` names without the prefix, string values), with
  the old blocks' comments. `cli.py::load_benchmark_defaults(name)` reads it
  and `_benchmark_stable_defaults` applies it as before; the printed
  "Benchmark stable defaults applied" lines are unchanged. The slice
  generators (`write_benchmark_module`, `scripts/make_mesh_slice_benchmark.py`)
  write the TOML next to the module. Guard: no benchmark module defines the
  attributes (`tests/test_no_legacy_imports.py`).
- A Cargo workspace with the engine and the bindings as two crates (phase 8
  slice B; no behaviour change: nine-cell and PLM gates exact, the extension
  stub unchanged). `crates/router-core` is the former `src/` without
  `bindings/` (moved by `git mv`) and has no `pyo3` dependency:
  `cargo test -p router-core` runs without a Python interpreter.
  `crates/router-py` holds the PyO3 surface and builds the same
  `photonic_router._rust` module (`pyproject.toml`: `manifest-path`). The
  engine struct is `router_core::Router` (constructor
  `Router::new(GridSpec, PrimitiveLibraryConfig, AStarParams, RouterConfig)`,
  all plain structs); the `PyPhotonicRouter` class wraps one (`inner`) and
  converts at the edge. The A* settings resolve through
  `AStarConfig::from_parts(&AStarParams, &AStarOverrides, &RouterConfig)`
  (formerly `astar_config_from_py`); the core's former `PyValueError`s are
  `RouterError::Config`, raised as `ValueError` by `router-py`'s `py_err`. The
  static obstacle builder's pyfunction and `PyStaticCellSet` moved to
  `router-py` over the core's `build_static_obstacle_map_from_geometry` and
  `StaticCellSet`; the plain-data `PyPhotonicRouter` methods forward to
  `Router` methods of the same name (`engine/api.rs`). CI runs
  `cargo clippy --workspace --all-targets`, `cargo test -p router-core` without
  `PYO3_PYTHON` and `cargo test -p router-py` with it.
- One Python package (phase 8 slice A): `translation/` and the nine root
  `routing_flow*.py` modules moved by `git mv` into `python/photonic_router/`
  (`routing/`, `electrical/`, `crossings/`, `plm/`, `verify/`, `gds/`,
  `records/`, `flow_*.py`, `benchmark_metadata.py`); imports rewritten, no
  logic change. A five-line `routing_flow.py` stays at the root as the entry
  point the reproduction archives use. `tests/test_no_legacy_imports.py`
  fails on any import of the old paths.
- The unified crossing-legality predicate enforces crossing-to-crossing spacing
  (phase 4 slice B-2; switch off byte-identical, nine-cell gate reproduces). Two
  crossing elements never share a grid cell: `crossing_legality` now tests the
  candidate's reservation window (`[floor(c-h), ceil(c+h)]` on both axes, the
  window the reservations are built from) against the nearest other crossing of
  the same route and the nearest crossing registered by other nets, verdict
  `CrossingVerdict::TooClose { other: SameRoute | Registered, have_cells,
  need_cells }` (reasons `too_close_same_route` / `too_close_registered`,
  `CrossingLegalityRule::required_spacing_cells = 2 * half_size + 1`, 5 cells =
  10 um on the paper grid). `CrossingCandidate` carries the centre and the two
  neighbours (`nearest_crossing_neighbour` picks them, overlapping windows first);
  the search hook feeds this move's earlier intersections and the partner's
  registered elements (`CrossingSearchPartner::registered_crossing_centres`,
  filled under the switch), the commit check and the post-commit assessment the
  route's other intersections and the registered events. `crossing_verdicts()`
  records gain `required_spacing_cells`, `nearest_same_route_cells`,
  `nearest_registered_cells`, `too_close_other`; `verify_repair` maps the two
  reasons onto `crossing_footprint_overlap`. Fixed under the switch: the braid
  repair's rollback restores the verdict records with the crossing events
  (`try_braid_repair`, `try_ripup_single_victim_and_reroute_with`); before, the
  records of a discarded reroute survived and the Python verifier, which reports
  verdicts no event covers as their own records, placed phantom crossing
  components for them (benes_32x32 contribution 1: 429 crossings instead of 416,
  three `crossing_component_overlap` errors between net 851's phantoms and net
  850's real crossings).
- One meander module tree and one `MeanderPlanner` class (phase 6 slice C; no
  behaviour change: the nine-cell reproduction gate, the two-cell PLM gate and the
  thirteen PLM cells of `docs/plm_baseline.json` reproduce exactly). Rust:
  `src/meander.rs`, `src/auto_meander.rs` and `src/plm.rs` moved verbatim to
  `src/meander/{geometry,search,plan}.rs` under a `mod.rs` that re-exports every
  public item (`crate::meander::X` still names them); the 36 `*_rejects_*` unit
  tests of the registered planner are one table-driven test
  (`registered_planning_rejects_invalid_inputs`, one row per former test, named
  after it; 504 lib tests from 539). PyO3: the registration state that lived on
  `PyPhotonicRouter` (the registered route cells and open sets, the route
  geometries, the reserved cells, the occupancy-prefix and reserved-index caches,
  the registration profile) is a `#[pyclass] MeanderPlanner(router)` in
  `src/bindings/meander_py.rs` with `register(routes, base_static_cells= |
  base_static_cell_handle=, route_clearance_radius_cells=, reset=)`,
  `register_geometries`, `reserve_rect`, `clear_reserved`, `open_cell_count`,
  `profile`, `plan_requests`, `plan_final`, `plan_centerline` (the depth sweep) and
  the static `realize` (the terminal-tangent polygon). The thirteen router
  methods they replace are removed (`register_meander_route_geometries`,
  `register_meander_route_cells_as_static`,
  `set_static_and_register_meander_route_cells_as_static[_handle]`,
  `plan_auto_analytic_meander_{for_centerline_depth_sweep,
  requirement_requests_registered_opened_auto_config,
  final_requests_registered_opened_auto_config}`,
  `realize_centerline_polygon_from_planned_auto_meander_with_terminal_tangents`,
  `last_meander_registration_profile`, `registered_meander_open_cell_count`,
  `clear_registered_meander_reserved_cells`,
  `add_registered_meander_reserved_grid_rect` and the never-called
  `add_registered_meander_reserved_cells`); `PyPhotonicRouter` keeps 42 methods
  (from 55), `route_single_net_with_auto_meander_and_commit` and
  `realize_route_polygon_from_planned_auto_meander` among them. Instead of the
  state the router keeps two counters: `obstacle_generation` (bumped by
  `invalidate_meander_base_prefix` at every commit, rip-up and static edit) and
  `static_reset_generation` (bumped by `clear_static_cells`); the planner rebuilds
  its prefix, or drops its registration, when they moved -- what the router's own
  context did before. `translation/route_rust_meanders.py` builds
  `rust_backend.MeanderPlanner(router)` next to the router and drives it
  (`_MeanderPlannerContext.planner`, the `MeanderPlannerLike` protocol);
  `route_rust_realization.py` calls `MeanderPlanner.realize`; the stub is
  regenerated. The test fakes of the planner live in
  `tests/fixtures/path_length.py` (`fake_backend`, `FakeRouter`, `FakePlanner`,
  `planned_meander`, `planned_result`, the real-backend, report and search-config
  helpers; seven fake routers in `tests/test_path_length_graph.py` before).
  Tests: `tests/test_path_length_graph.py` (3,268 lines, 43 tests across five
  concerns) and `tests/test_path_length_candidates.py` (141 lines, 3 tests) are
  five files on that one fixture module, the same 46 tests with the same
  assertions: `test_path_length_graph.py` (the graph and arrival analysis, 5),
  `test_path_length_requirements.py` (requirements and the upstream delay
  candidates, 12, the former candidates file merged in),
  `test_meander_planner.py` (the planner with the fakes and the real backend,
  19), `test_path_length_diagnostics.py` (residuals and acceptance, 6) and
  `test_path_length_flow.py` (the monkeypatched flow, 4); the four copies of
  `_Bundle`/`_Netlist`/`_Schematic` are `schematic_from_links` (with `edge_key`
  and `records_from_links`), the two `_Layout`s are `FakeLayout`.

- Rust decides the meander cascade, the PLM reports go behind `--debug-meanders`
  (phase 6 slice B; the thirteen PLM cells of `docs/plm_baseline.json`, the two-cell
  PLM gate and the nine-cell reproduction gate reproduce exactly). One policy owner:
  `translation/route_rust_meanders.py` (1,855 lines, from 3,010) sends one Rust call
  per requirement with the request vector (the candidate bundles in order, one
  requested extra length per edge -- the edge's new total where it already carries
  a planned meander -- and the requiring edge as the same-edge split fallback) to
  the new `PyPhotonicRouter.plan_auto_analytic_meander_requirement_requests_
  registered_opened_auto_config` (`plm.rs::plan_registered_geometry_requirement_
  requests`, which `plan_registered_geometry_requirement_candidates` now wraps),
  and one final call with every physical edge's total as before. The Python-side
  "mixed" planner (one Rust call per edge for bundles whose edges already carried
  a meander, tried ahead of the batch) and the Python split fallback after the
  batch are deleted; the three bindings they alone used
  (`plan_auto_analytic_meander_{requirement_candidate_indices,geometry_sequence,
  split_request}_registered_opened_auto_config`) are removed. Reports: the planner
  returns a `MeanderInsertionReport` of `MeanderEdgeReport`s (`route_rust_types.py`;
  edge, requested, credited and physical inserted length, planned edges, status,
  reason, the drawn plan's scalars) that `path_length_diagnostics.py` and
  `routing_flow_plm.py` (227 lines, from 336) consume; `to_info_dict()` is the GDS
  meta info `meander_insertion_report` with the keys `plm_table.py`, the overview
  SVG and the e2e tests read unchanged. Timers, counters (`planner_calls`,
  `bundle_*`, `requirement_batch_*`, the setup, commit and Rust profile splits) and
  the per-candidate attempts are built only when `--debug-meanders true` (the
  existing flow flag, now threaded to `route_match_and_realize(debug_meanders=)`)
  and then land in `MeanderPlanningProfile`; the SUMMARY tokens
  (`plm_groups`, `plm_over_tolerance`, `meander_inserted_um`) and the acceptance
  decision are computed unconditionally. The `hasattr` probes on the Rust router
  and backend (`route_port_corrected_centerline`,
  `add_registered_meander_reserved_grid_rect`, `auto_meander_search_config_rs`,
  the blocked-cell handle's `cells`, the planner methods) are gone; the test
  fakes carry those methods. `translation/path_length_candidates.py` is folded
  into `path_length_requirements.py` and the `route_rust_analysis.py` facade is
  dropped (every importer takes the real module). Tests: 41 + 3 before, 43 + 3
  after (the split test now pins the request vector, one test pins that no
  profile is built without the flag, one that the planner context builds none).
- Contribution 2 takes a `StageSchedule` and refuses non-layered designs
  (phase 4 slice D). `CrossingDemand.stage_schedule()` (new type
  `StageSchedule` in `python/photonic_router/crossing_demand.py`) is the
  ordered list of bands with crossing events, each with its x-extent (wall
  face to wall face), wall orders, confined nets and `CrossingStagePlan`;
  `translation/preplaced_crossing_grids.py` (`_derive_crossing_tiles`,
  `derive_preplaced_crossing_layout(schematic, unrouted_layout, schedule)`,
  `build_stage_schedule_for_benchmark` replacing
  `build_crossing_plan_for_benchmark`) iterates over it instead of the
  plan's stage dictionary. `CrossingDemand.require_band_layered()` raises
  `ValueError` naming the nets outside the band model by reason
  (`same_wall`, `non_adjacent_walls`, `vertical_port`, `chip_boundary`);
  the flow's `_preplaced_crossing_grids_stage` calls it first and fails
  with "pre-placed crossing grids need a band-layered design: ...; route
  with --crossing-mode lidar-pure or lidar-guided" instead of crashing or
  misplacing. The tiles builder still measures each stage's free band
  between the lanes' port faces (equal to the schedule band on Benes, 15.9
  um past the right wall face on three multiport-MMI stages; a test pins
  the relation), so no tile moves; `CrossingStagePlan` documents that
  `source_depth`/`target_depth` are column indices. The four
  `scripts/path_investigation/*_sim.py` / `column_grid_budget.py` scripts
  read `build_crossing_demand(...).plan` directly. Nine-cell gate exact.
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

- The dead meander generations and their bindings (phase 6 slice A; the
  production PLM path -- `route_rust_meanders.py` -> the `plan_*` wrappers of
  `src/plm.rs` -> `src/auto_meander.rs` -> `src/meander.rs`, realised through
  the stored `selected_meander_centerline` -- is untouched). Rust: the
  box-planner section of `src/geometry_realization.rs` (`select_meander_segment`,
  `RouteAnalyticMeanderPlan`, `plan_analytic_meander_for_route`, the route-level
  `plan_auto_analytic_meander_for_route{,_depth_sweep,_depth_sweep_with_prefix}`
  and `probe_..._route_depth_sweep_with_prefix` wrappers,
  `splice_meander_into_centerline`, the three
  `realize_route_polygon_with_{analytic_meander,checked_analytic_meander_box,
  auto_checked_analytic_meander}`), the pre-routing port-access connector
  (`PortAccess`, `PortAccessConfig`, `PortAccessError`, `build_port_access{,es}`,
  `realize_route_polygon_with_port_access`, their anchor-search helpers and the
  `GeometryError::{PortAccess, RouteStartDoesNotMatchSourceAnchor,
  RouteEndDoesNotMatchTargetAnchor}` variants), the feasibility probe
  (`AutoRouteAnalyticMeanderProbe`,
  `probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix`,
  `auto_meander_probe_to_py_object`), `check_meander_box_free` (the obstacle-map
  form; the prefix form stays) and `parse_meander_side`; the pyclass
  `PortAccess` and 19 pymethods with no caller in `python/`, `translation/`,
  `tests/` or `scripts/` (76 -> 57): `plan_analytic_meander_for_route`,
  `plan_auto_analytic_meander_for_route`,
  `plan_auto_analytic_meander_for_route_depth_sweep`,
  `probe_auto_analytic_meander_for_centerline_depth_sweep`,
  `probe_auto_analytic_meander_for_route_depth_sweep`,
  `realize_route_polygon_with_analytic_meander`,
  `realize_route_polygon_with_checked_analytic_meander_box`,
  `realize_route_polygon_with_auto_checked_analytic_meander`,
  `check_meander_box_free`, `cells_for_auto_analytic_meander_box`,
  `realize_centerline_polygon`, `realize_centerline_polygon_from_planned_auto_meander`,
  `realize_route_polygon_with_port_access`, `describe_bend_radius`,
  `clear_registered_meander_route_cells`,
  `invalidate_meander_registered_reserved_index`, `build_port_access`,
  `build_port_accesses`, `all_net_core_cells`. Python: the two realisation
  fallbacks of `route_rust_realization.py` for meander records without a stored
  centerline (box replanning through `realize_route_polygon_with_analytic_meander`
  and `..._with_auto_checked_analytic_meander`; such a record now raises), the
  test-only compat API `insert_meanders_for_requirements` with
  `MeanderInsertionReport` / `MeanderInsertionResult` and its re-exports, the
  never-passed `requirement_edge_alternatives` parameter (and the
  `legacy_single_edge_alternative` candidate reason), and the `hasattr`
  capability probing of `_build_planner_context` for older backend generations
  (with the unreachable plain `register_meander_route_cells_as_static` branch; the
  Python fakes in the tests now carry the production method names). Tests: 32
  Rust tests of the deleted items went, 10 Rust tests and one Python test were
  rewritten against the surviving production entry (`plan_auto_for_route` helper
  on `plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix`,
  `check_meander_box_free_with_prefix`, `analyze_meander_insertion_for_requirements`).
  Kept on purpose: `plan_auto_analytic_meander_for_centerline_depth_sweep` (one
  test), `route_single_net_with_auto_meander_and_commit` (no caller, owner
  question open; it now calls the centerline planner directly),
  `centerline_port_corrected_checked` and
  `realize_centerline_polygon_from_planned_auto_meander_with_terminal_tangents`
  (both reached from production through attribute access, not a call). Nine-cell
  gate and PLM gate identical, 13-cell PLM compare identical.
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
