# Restructure the router into modular, human-readable engines with a swappable search kernel

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-22 18:30 -- Milestone 0 complete on branch `restructure/modular-engine` (gate 9/9 exact, test baseline pinned with zero failures). Next: Milestone 1 on the owner's go.**


## Purpose / Big Picture

The repository owner wants the router to read like a well-designed C++ code base: every piece of functionality behind a declared interface, one implementation file per concern, tests that answer to the interface, and a top-level flow that a human can follow by reading the sequence of function calls. Concretely, at the end of this plan there is one clean A* search kernel that is one of several possible search engines behind a single `NetSearch` interface, one clean rip-up-and-repair loop that reads as a loop and not as a 948-line method, a Rust crate split into modules by concern instead of one 25,000-line binding file, a Python side that a human can drive from a small typed configuration object and a short command line (so that a GUI can later sit on top of the same entry point), and unit tests per module instead of correctness evidence coming almost only from full benchmark runs.

The plan has one hard constraint that did not exist before 2026-09-22: `main` (commit `960d0ee`, the former branch `DATE2027-results`) reproduces every one of the 27 cells of ours in the DATE 2027 paper table exactly, and it must keep doing so. Every milestone below ends with the reproduction gate described in Milestone 0. A restructuring step that changes a crossing count or a routed length by one micrometre on any gated cell is either a bug or a behaviour change that needs an owner decision; it is never silently accepted.

The plan deliberately does not start from the rejected "cleanup" of 2026-09-19 (parked as branch `cleanup/crossing-clean`, commit `15e24aa`). That diff removed both paper contributions, the paper benchmarks, the results tooling and about twenty tests; it is a different, smaller program, not a restructuring of this one. Nothing from it is applied as a diff. It may be consulted for ideas about what is dead.


## Progress

- [x] (2026-09-22 18:30) Structural survey of `main` taken (numbers in Context and Orientation); plan written.
- [x] (2026-09-22) Owner decisions D1-D6 recorded in the Decision Log.
- [x] (2026-09-22 17:40) Milestone 0, gate: `reproduce_date2027.sh short` + `scripts/results/gate_short.sh` against `docs/date2027_table_sources.json`; first run 9/9 exact in 80 s (commit 58c9cc1).
- [x] (2026-09-22 17:40) Milestone 0, baseline: `scripts/test_baseline.sh` with `tests/baselines/test_baseline.txt` (rust 486/0, python 411 passed, 11 pinned failures).
- [x] (2026-09-22 18:30) Milestone 0, D3: the 11 stale tests resolved (details in the Decision Log); baseline re-pinned with an empty failure list.
- [ ] Milestone 1: configuration as data (one typed configuration tree replaces the 88 environment variables as the algorithms' input).
  - [x] (2026-09-22 20:40) Slice 1: the 41 Rust-side variables are fields of `crate::config::RouterConfig` (`src/config.rs`), passed from Python as `rust_backend.RouterConfig` built by `photonic_router/config.py`; the single overlay table lives in `photonic_router/env_overlay.py`; `grep -rn "env::var\|var_os" src/` is empty; rust 492/0, python 447/0 (32 new overlay tests), gate 9/9 exact; verified independently by the lead.
  - [ ] Slice 2: the 46 Python-side variables.
  - [ ] Slice 3: `build_config` loader, CLI integration, full 27-cell reproduction.
- [ ] Milestone 2: the Rust engine file split by concern (pure code motion, bindings separated from the engine).
- [ ] Milestone 3: one search interface, one A* kernel module, a second search engine proving the seam.
- [ ] Milestone 4: one readable rip-up-and-repair loop with named policies.
- [ ] Milestone 5: the Python side as stages behind Protocols, a flow function without argparse, a short human command line.
- [ ] Milestone 6: unit tests per module; Rust tests moved next to their modules.
- [ ] Milestone 7: architecture document, configuration reference, full 27-cell reproduction on the restructured code.


## Surprises & Discoveries

- 2026-09-22, Slice 1: after the Rust environment reads were removed but before the Python side passed a configuration, the gate showed the two ADEPT 8x8 cells of baseline and contribution 1 with the right crossings and a longer GDS. Cause: the mesh benchmarks set `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT=0.05` in their `STABLE_ROUTING_ENV` block, exported by the CLI and formerly read by Rust; without the overlay the kernel ran with weight 0. The implementation agent attributed the difference to compiler codegen, which two independently built kernels reproducing all 27 cells on 2026-09-22 already ruled out. Lesson for every later slice: a gate run is only meaningful once the value's whole path (environment, overlay, config object, kernel) is wired; and a mismatch with identical crossings and a few percent longer routes is the signature of a lost search-cost parameter.

- 2026-09-22: The kernel already has a search interface, `crate::astar::SingleNetSearch` (four methods) with `AStarSingleNetSearch` as its only implementation, wired into every production call site on 2026-08-24. It is a seam, but its four methods and their nine positional parameters each are the old free functions in disguise; Milestone 3 replaces it with one method over a request struct rather than adding a fifth.
- 2026-09-22: `python/photonic_router/static_obstacle_builder.py` already declares an `ObstacleMapBuilder` Protocol. Milestone 5 keeps that pattern and extends it to the other stages.
- 2026-09-22: Two environment variables are read on both sides of the language boundary (`PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT`, `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS`), and `routing_flow.py` sets one of them with `os.environ.setdefault` so the kernel sees it. That is the clearest example of why configuration must become an explicit object (Milestone 1).


## Decision Log

Decisions the repository owner has to make before or during the work. None of them is made in this document; a line here is a question until the owner answers it, and the answer is recorded with date and author.

- D1 (before Milestone 1): which of the three legacy engine paths stay? The negotiated engine (`PHOTONIC_ROUTER_NEGOTIATED_REPAIR`, default on since 2026-09-15) is the engine of every paper row. The legacy repair chain (`PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN`), the orthogonal repair fallback (`PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK`) and the non-negotiated `route_many` paths in `src/py_router.rs` are not. The plan proposes deleting them in Milestone 4 after the gate proves them unused by any gated cell; deletion is the owner's call.
- D2 (before Milestone 1): which crossing modes stay? The paper uses `lidar-pure` and `lidar-guided`. `window` and `collision` (`translation/crossing_modes.py`) predate them. The plan proposes keeping them only if a benchmark or test depends on them, else deleting.
- D3 (before Milestone 0): the eleven Python tests that have failed since August (`tests/test_routing_flow_stats.py`, `tests/test_route_rust_geometry.py`, `tests/test_route_rust_opened_cells.py`; toy benchmarks that no longer route with the current rules, plus one stale fake config). Fix them to the current rules, or delete them with a note. The plan proposes fixing the ones that test a still-existing behaviour and deleting the rest.
- D4 (Milestone 1): the `PHOTONIC_ROUTER_*` environment variables remain readable as an overlay on the configuration object forever (the benchmark modules' `STABLE_ROUTING_ENV` blocks and the reproduction scripts depend on some of them), or only until the benchmark modules are rewritten to set configuration fields directly. The plan proposes: keep the overlay as one compatibility loader in one file, and rewrite the benchmark blocks in Milestone 5 so that the overlay is only needed for ad-hoc experiments.
- D5 (Milestone 3): the second search engine. The seam is only proven if something other than A* runs through it. Options: a small breadth-first / Dijkstra grid search used only by tests (cheap, proves the interface), or a real alternative engine the owner has in mind. The plan builds the test engine; a real one is a separate plan.
- D6 (Milestone 5): the name and shape of the human entry point. The plan proposes `python -m photonic_router route <benchmark> --configuration {baseline,contribution1,contribution2} [overrides]` with the three named configurations as first-class values (owner rule 2026-09-04: exactly one of them runs at a time), and a `route_benchmark(config) -> RoutedLayout` function underneath for notebooks and a later GUI.

Recorded decisions:

- 2026-09-22 (owner): the DATE 2027 paper is final; its numbers are frozen; `main` is the reproduction base; the 2026-09-19 cleanup is rejected and a cleanup is done from this base instead.
- 2026-09-22 (lead, executing D3): diagnosis showed ten of the eleven failures share one cause: the toy benchmarks `TOY`, `heater_s`, `heater_s_compact`, `mmi_heater`, `mmi_heater_8x4` and `mmi_heater_8x4_ripup_reroute` cannot be routed at all (a placement clearance shortage at one MMI input approach, root-caused on 2026-08-18 by BFS analysis and recorded as "not planned to be fixed"; `--crossings false`, `--fanout-access-mode off` and the die keepout make no difference). The eleventh was a test double missing `max_dense_obstacle_cells`. Applied: `test_routing_flow_populates_stats`, `test_rust_routed_layout_uses_waveguide_geometry` and the electrical end-to-end test now run on `heater_s_mod` (63 instances, 81 nets, 21 heater terminal groups, routes in about a second), the parametrized `test_benchmarks_route_with_astar_only` keeps `clements_8x8` and `heater_s_mod` only, the fake config gained the field, and `test_toy_ten_um_bend_radius_does_not_backtrack_on_one_cell_short_s_bend` was deleted because its subject (a one-cell-short S-bend at 10 um radius) exists only in TOY's geometry; Milestone 6 should re-cover that property with a synthetic fixture. Owner decision 2026-09-22: delete the unroutable toy benchmarks. Applied to `heater_s` and `heater_s_compact` (unreferenced). `TOY` (path-length graph tests, schematic-level), `mmi_heater`, `mmi_heater_8x4` and `mmi_heater_8x4_ripup_reroute` (electrical benchmark script, its tests and `tests/baselines/electrical_suite_metrics.json`) stay until Milestone 6 retargets those tests to fixtures; they are marked in their module docstrings as not routable optically.
- 2026-09-22 (owner, on the lead's recommendations): D1 delete the legacy repair chain, the orthogonal repair fallback and the non-negotiated multi-net paths in Milestone 4 once the gate proves no paper cell uses them. D2 delete the `window` and `collision` crossing modes unless a benchmark or test still needs them; keep `lidar-pure` and `lidar-guided`. D3 fix the stale tests that cover a behaviour that still exists, delete the rest with a note. D4 keep one environment overlay loader in one file permanently for ad-hoc experiments; rewrite the benchmark stable blocks and the reproduction scripts so nothing else depends on it. D5 the second search engine is the Dijkstra test oracle; a real alternative engine is a separate plan. D6 the entry point is `python -m photonic_router route <benchmark> --configuration {baseline,contribution1,contribution2}` with overrides, over `route_benchmark(config)`.


## Outcomes & Retrospective

Empty until work starts.


## Context and Orientation

This section is self-contained: a reader who knows nothing about the repository can orient from it.

The repository routes photonic integrated circuits: given a schematic (instances of components such as couplers, MMIs, heaters, and nets connecting their ports) and a placement, it produces a GDS layout in which every net is a waveguide path that satisfies geometric design rules (bend radius, clearance) and in which waveguide crossings are realized by crossing cells. The paper being reproduced compares three configurations of our engine: the baseline (`--crossing-mode lidar-pure`, crossings discovered during search), contribution 1 (`--crossing-mode lidar-guided`, a precomputed crossing plan guides costs and order), and contribution 2 (`--preplaced-crossing-grids true`, crossing structures placed before routing, the rest routed crossing-free). The reference router LiDAR is external and not part of this plan.

Sizes on `main` at commit `960d0ee` (survey of 2026-09-22):

    Rust, src/, 12 files, 59,102 lines
      py_router.rs            25,086  PyO3 bindings + the whole multi-net engine + 6,280 lines of tests
                                      impl PyPhotonicRouter (engine)     lines  3,709-13,406   138 methods
                                      impl PyPhotonicRouter (pymethods)  lines 13,409-18,286   101 methods
                                      route_many_with_negotiated_repair_and_commit  lines 14,956-15,903 (948 lines)
                                      probe_net_for_repair 9,655-9,925; try_braid_repair 12,761-12,929
                                      commit_native_route_* 8,295-8,532; build_native_batch_result_dict 10,375-10,472
      astar.rs                14,997  single-net A* search; AStarConfig has 31 fields; 10 public entry points;
                                      trait SingleNetSearch (4 methods) at 1,761; heuristic at 8,805-8,829
      geometry_realization.rs  6,817  centerline to polygons
      simple_routes.rs         2,510  Z-route shortcuts
      plm.rs                   2,435  path-length matching planner
      auto_meander.rs          1,895  obstacle-aware meander search
      static_obstacle_builder  1,778  static obstacle map construction
      obstacle_map.rs          1,725  the routing grid database
      meander.rs, primitives.rs, crossings.rs, lib.rs   small
    Python
      routing_flow.py          1,326  CLI (54 arguments) + the flow (run_routing_flow, ten phases)
      translation/            32,327  route_rust.py 8,295 (one class _RouteNetsRustSession, run() = 9 phase methods,
                                      _dispatch_native_routing alone 449 lines); preplaced_crossing_grids.py 2,646;
                                      route_rust_meanders.py 3,117; 13 electrical-routing files
      python/photonic_router/  2,768  topology analysis, crossing plan, static obstacle builder, primitive library
    Environment variables: 88 distinct PHOTONIC_ROUTER_* names (41 read in Rust, 49 in Python, 2 in both)
    Tests: 390 Python tests in 30 files (11 failing since August, see D3); 500 Rust #[test] functions,
           101 of them inside py_router.rs's own mod tests

Terms used below. A *kernel* is the single-net search: given a source state, a target state and the obstacle map, find one legal path. The *engine* or *loop* is the multi-net part: order the nets, call the kernel per net, commit routes to the obstacle map, and when a net fails, find which committed routes block it (the *probe*), remove them (*rip-up*), re-queue and retry (*repair*); *braid repair* is a local swap of two crossing routes; a *negotiated* engine is the LiDAR-style version of this loop that every paper row uses. A *stage* is one phase of the Python flow (build the obstacle context, build route jobs, plan crossings, dispatch to the kernel, finalize, verify, realize geometry). An *interface* means a Rust `trait` or a Python `typing.Protocol`: a declared contract that an implementation satisfies and a test can be written against, the header-file idea from C++. *Zero-behaviour-change* means the routed GDS of every gated cell is identical before and after.

The reproduction tooling that makes the gate possible lives on `main` since 2026-09-22: `scripts/results/reproduce_date2027.sh` runs the paper's rows into `results_date2027/` (never into `results/`, which holds the paper's archives), and `scripts/results/compare_date2027.py <root> <sources.json>` compares crossing count and GDS length exactly against the paper's `EXPERIMENTS_TABLE_SOURCES.json` (kept next to the paper's `main.tex` in the paper repository; the path used on this machine is `/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/fcn/papers/2027_photonic_crossings/DATE2027/EXPERIMENTS_TABLE_SOURCES.json`). `docs/DATE2027_REPRODUCTION.md` explains both and holds the provenance of every cell. The kernel is built with `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu .venv/bin/maturin develop --release` from the repository root (the checked-in `rust-toolchain.toml` pins a Windows channel), and `cargo test --release --lib` needs `PYO3_PYTHON=$PWD/.venv/bin/python` in addition.

Working branch: all milestones happen on a branch off `main` (proposed name `restructure/modular-engine`), merged to `main` milestone by milestone after the gate. `DATE2027-results` is never touched again; it is the frozen reference of the paper's engine.


## Milestone 0: the gate and the baselines

Goal: before any code moves, make it cheap and mechanical to prove that nothing moved. At the end of this milestone there is a one-command gate that takes about two minutes and answers "identical to the paper or not" for the smallest cells of all three configurations, the test baselines are pinned, and the eleven stale tests are resolved per D3.

Work. Add a `short` mode to `scripts/results/reproduce_date2027.sh` that runs `benes_4x4_flat` and `benes_8x8_flat` in all three configurations plus `multiportmmi_8x8` in all three (nine cells, about 90 seconds on the paper's machine) into a caller-chosen `RESULTS_ROOT`, and a wrapper `scripts/results/gate_short.sh` that runs it into a fresh temporary root and then calls `compare_date2027.py`, exiting non-zero on any mismatch or missing cell. Pin the test baselines: record the exact `cargo test --release --lib` pass count and the exact list of `pytest` failures in this plan's Progress, and add `scripts/test_baseline.sh` that runs both and diffs the failure list against the pinned one (empty diff = pass). Resolve D3: fix or delete the eleven failing tests so that the pinned pytest failure list is empty from here on; every deletion is noted in the Decision Log with the reason.

Commands, from the repository root, `.venv` present, kernel built:

    scripts/results/gate_short.sh
    scripts/test_baseline.sh

Acceptance: `gate_short.sh` prints nine `ok` rows and `all reproduced cells match`; `test_baseline.sh` prints the pinned counts and an empty diff. The full 27-cell reproduction is not rerun here (it was done on 2026-09-22 and recorded in `docs/DATE2027_REPRODUCTION.md`).


## Milestone 1: configuration as data

Goal: the algorithms take their parameters from typed configuration objects passed explicitly, not from 88 environment variables read wherever they happen to be needed. This is the first milestone because a loop or a kernel cannot be read while some of its inputs arrive through `std::env::var` inside the algorithm, and because a future GUI needs a configuration object to edit.

Work, Rust. Introduce `crate::config` with one struct per concern, all plain data with `Default`: `SearchConfig` (the current `AStarConfig`'s 31 fields regrouped into `SearchLimits` for budgets, iterations, time and dense-state caps, `SearchCosts` for bend, congestion, long-straight and history weights, `SearchStrategy` for heuristic mode, tie breaker, primitive ordering, JPS flags), `NegotiationConfig` (budget ladder first / first-retry / retry, rounds, braid escalation, clean-probe escalation, crossing-free-unplanned, long-straight exempt net ids), `CrossingConfig` (already exists in `crossings.rs`; keep), and `DiagnosticsConfig` (every `TRACE_*`, `*_DIAG`, `SEARCH_FAILURE_MAP`, `NATIVE_PROGRESS` flag). Every `std::env::var` read inside `src/astar.rs` and `src/py_router.rs` (33 call sites) is replaced by a field read; the only place that reads the environment is one function `crate::config::overlay_from_env(&mut RouterConfig)` that applies the `PHOTONIC_ROUTER_*` names to fields and is called once, at the binding boundary, when the Python side asks for it. Rust unit tests cover the overlay (every name maps to exactly one field, unknown names are reported).

Work, Python. Introduce `photonic_router/config.py` with frozen dataclasses mirroring the Rust structs plus the flow-level settings (benchmark name, configuration name, output paths, debug artifacts), and one function `build_config(cli_args, benchmark_module) -> RoutingConfig` in `photonic_router/config_loading.py` that merges, in this order, defaults, the benchmark's stable block, the command line, and finally the environment overlay if `--env-overlay` is on (it is on by default for the reproduction scripts and off for the human command line, subject to D4). Every `os.environ.get("PHOTONIC_ROUTER_...")` in `translation/` and `routing_flow.py` (about 50 sites) becomes a field read. The `os.environ.setdefault` in `routing_flow.py` disappears: the flow sets `config.negotiation.long_straight_exempt_dense_fanout = True` when pre-placed grids are on, and the value crosses to Rust inside the config object, not through the process environment.

A trap to preserve exactly: several variables have "unset" semantics that differ from any value (the repository memory of 2026-09-07 records that even setting `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` to its default changes behaviour). The configuration fields for those are `Option`s, and the overlay sets them only when the variable is present. The gate catches a mistake here on the first run.

Acceptance: gate and test baseline pass; `grep -rn "std::env::var" src/` reports only `src/config.rs`; `grep -rn "os.environ" translation/ routing_flow.py python/` reports only the loader; the full 27-cell reproduction is run once at the end of this milestone because it touches every configuration path.


### Milestone 1, inventory and slicing (2026-09-22)

The inventory (evidence agent, 2026-09-22) found 88 distinct names: 23 crossing (15 of them the pre-placed grid geometry read in one function of `translation/preplaced_crossing_grids.py`), 33 diagnostics and trace, 11 fan-out access, 9 negotiation, 4 search cost, 3 search limit, 2 search strategy, 1 debug artifact (`WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE`), 1 flow (`SYNTH`, a parameter of the synthetic benchmark generator, out of scope: it configures a benchmark, not the router), 1 dead (`LAYER_ORDER`, comment only). No name is read in both languages. Three facts matter for the mapping: (1) many Rust flags are presence flags (`var_os(..).is_some()`, any value counts), some are exact `"1"` flags, and two are "on unless exactly `0`" flags (`NEGOTIATED_CROSSING_FREE_UNPLANNED`, `NEGOTIATED_REPAIR`); (2) `FANOUT_LANE_SPACING_CELLS` has default 11 at one site and 3 at three others, so its field is an `Optional[int]` and each site keeps its own literal default; (3) `DENSE_FANOUT_INSTANCES` is written by the pre-placed grid stage into the process environment for the routing stage to read, which becomes an explicit value handed from one stage to the next.

Rule for the whole milestone: one field per variable, named after it, with exactly today's parse rule and default; no two variables are merged and no default is changed. Merging (for example the enable/disable pair for guided collision crossing) is Milestone 4's business, when the loop is rewritten and the gate can attribute a change.

Slice 1 (Rust side, 41 names): `src/config.rs` with `NegotiationConfig` (budget_first 2 000 000, budget_first_retry 10 000 000, budget_retry 30 000 000, braid_escalation false, crossing_free_unplanned true, disable_braid_repair false, pending_straight_ripup_threshold 100, enable_orthogonal_repair_fallback false), `SearchOverrides` (astar_timeout_ms `Option<u64>`, max_dense_states `Option<usize>`, long_straight_congestion_weight `Option<f64>`), `CrossingEngineConfig` (enable_guided_collision_crossing false, disable_guided_collision_crossing false, disable_rust_crossing_validation false), `KernelDiagnostics` (one field per trace name: native_progress, native_repair_diag, search_failure_diag, search_failure_map `Option<Vec<i32>>`, chain_diag, hot_loop_timing, move_diag bool plus move_diag_cell `Option<(i32,i32)>`, pop_diag_below_y `Option<i32>`, probe_cells `Vec<(i32,i32)>`, trace_crossing, trace_crossing_net `Option<u64>`, trace_crossing_candidates, trace_crossing_candidate_max 120, trace_crossing_level1, trace_crossing_pending, trace_crossing_pending_threshold `Option<usize>`, trace_crossing_perp_reject_threshold `Option<usize>`, trace_partner_net `Option<u64>`, trace_plain_route_net `Option<u64>`, trace_endpoint_bump_nets as an enum None / All / Names(Vec<String>), trace_endpoint_correction_net `Option<u64>`, crossing_mismatch_dump bool plus crossing_mismatch_dump_net `Option<u64>`, crossing_mismatch_fatal, analysis_crossing_partner_counters), and `RouterConfig` holding the four. `AStarConfig` and `CrossingSearchConfig` carry a `KernelDiagnostics` (cloned in) so the trace helpers in `src/astar.rs`, which already receive `&CrossingSearchConfig` or `&AStarConfig`, read fields instead of the environment; the `OnceLock` for hot-loop timing becomes a field read. The binding gains `#[pyclass(name = "RouterConfig")]` with keyword constructor arguments for every field (flat, prefixed by group: `negotiation_budget_first`, `diag_native_progress`, ...) and `PyPhotonicRouter::new` takes it as an optional last argument; `astar_config_from_py` takes the overrides from it. After the slice `grep -rn "env::var\|var_os" src/` is empty. On the Python side `photonic_router/config.py` gets the mirroring frozen dataclasses (`NegotiationConfig`, `SearchOverrides`, `CrossingEngineConfig`, `KernelDiagnostics`, `RouterConfig` with `to_rust(rust_backend)`), `photonic_router/env_overlay.py` gets the single table of `(name, field path, parser)` entries with `apply_env_overlay(config, environ)` and `RouterConfig.from_environment()`, and `_RouteNetsRustSession` builds the router with `router_config or RouterConfig.from_environment()` (identical behaviour for every caller that passes nothing). Rust tests of the old `*_from_env` parsers become tests of the defaults; the parse rules are tested in `tests/test_env_overlay.py`, one per rule, plus one test that every Rust-side name of the inventory appears in the table.

Slice 2 (Python side, 46 names): the same dataclass and overlay pattern for crossing plan (3), pre-placed grid geometry and stage options (17, replacing `crossing_grid_geometry_from_env`), fan-out access (11), engine selection (`NEGOTIATED_REPAIR`, `LEGACY_REPAIR_CHAIN`), search cost and strategy on the Python side (`MIN_BEND_WEIGHT`, `MIN_HEURISTIC_WEIGHT`, `HEAP_TIE_BREAKER`, `LONG_STRAIGHT_EXEMPT_DENSE_FANOUT`), Python trace flags (9) and the GDS-on-failure switch; `DENSE_FANOUT_INSTANCES` becomes a value returned by the pre-placed stage and passed into the routing stage; `os.environ.setdefault` in `routing_flow.py` disappears (the flow sets the exemption field when pre-placed grids are on). The top-level `RoutingConfig` holds the Rust-side `RouterConfig` and the Python-side groups; it is threaded as one `config` parameter through `run_routing_flow`, `build_optical_routing_stage_config`, `route_match_and_realize`, `route_nets_rust` and the session, defaulting to `RoutingConfig.from_environment()` when a caller passes nothing.

Slice 3 (loading): `build_config(cli_args, benchmark_module)` in `photonic_router/config_loading.py` merges defaults, the benchmark's `STABLE_ROUTING_ENV` block (through the overlay table, without touching `os.environ`), the command line, and the process environment overlay; `routing_flow.main` uses it. `run_and_archive.sh` and the reproduction scripts keep exporting their variables, which the overlay picks up. End of milestone: the full 27-cell reproduction.

## Milestone 2: the Rust engine split by concern

Goal: `src/py_router.rs` stops existing as a 25,000-line file. This milestone is pure code motion with zero behaviour change; nothing is renamed except modules and nothing is rewritten. Its value is that Milestones 3 and 4 can then be read and reviewed as diffs of small files.

Work. Create the module tree and move code into it, keeping every function's body intact:

    src/lib.rs                       crate root, module wiring only
    src/config/                      (from Milestone 1)
    src/grid/                        obstacle_map.rs, static_obstacle_builder.rs, primitives.rs (moved as-is)
    src/search/                      astar.rs moved as-is (split in Milestone 3)
    src/engine/mod.rs                the router struct (renamed from PyPhotonicRouter's engine half to Router;
                                     no #[pyclass] on it)
    src/engine/jobs.rs               NativeRouteJob, NativeRouteAttempt, batch types
    src/engine/commit.rs             commit_native_route_* and realized_dynamic_blockers (lines 8,295-8,654)
    src/engine/crossing_reservation.rs   CrossingReservationBlockers, reservation windows, crossing events
    src/engine/endpoint_correction.rs    the endpoint bump / correction methods
    src/engine/negotiation/mod.rs    route_many_with_negotiated_repair_and_commit and its helpers (as-is here;
                                     restructured in Milestone 4)
    src/engine/negotiation/probe.rs  probe_net_for_repair, probe_names_no_blocker, blocker analysis
    src/engine/negotiation/braid.rs  try_braid_repair, run_braid_repair_passes, requeue_braid_victims
    src/engine/diagnostics.rs        trace printing helpers
    src/geometry/                    geometry_realization.rs, meander.rs, auto_meander.rs, plm.rs, simple_routes.rs
    src/bindings/mod.rs              PyPhotonicRouter: a thin #[pyclass] wrapping engine::Router
    src/bindings/convert.rs          every *_to_py_dict / *_to_py_object, build_native_batch_result_dict
    src/bindings/types.rs            PyGridSpec, PyAStarConfig, PyCrossingConfig, PyState, PyPortAccess ...

The 101 tests in `py_router.rs`'s `mod tests` move next to the code they test (`src/engine/negotiation/tests.rs` and so on) unchanged. Legacy engine paths named in D1 are moved, not deleted, in this milestone; their deletion is Milestone 4's business once the owner has decided.

Commands: `cargo build --release`, `cargo test --release --lib` (count unchanged from the pinned baseline), `maturin develop --release`, `scripts/test_baseline.sh`, `scripts/results/gate_short.sh`.

Acceptance: `wc -l` of the largest file under `src/` below 4,000 lines except `src/search/astar.rs` (split next); test count identical; gate identical. The full 27-cell reproduction is not needed for pure code motion; the short gate plus the unchanged Rust test count is the evidence, and the Milestone 3 full run covers this milestone as well.


## Milestone 3: one search interface, one A* kernel, a second engine

Goal: the single-net search is one interface with one method, the A* implementation of it is a readable module tree, and a second implementation exists so that "the search engine is swappable" is a demonstrated fact, not a claim.

Work, interface. Define in `src/search/mod.rs`:

    pub struct SearchRequest<'a> {
        pub source: State,
        pub target: State,
        pub port_open_cells: Option<&'a FxHashSet<CellKey>>,
        pub reservation_open_cells: Option<&'a FxHashSet<CellKey>>,
        pub dynamic_expansion: Option<DynamicExpansion<'a>>,   // radius + clearance-exempt cells
        pub crossing: Option<&'a CrossingSearchConfig>,
        pub config: &'a SearchConfig,
    }
    pub struct SearchEnvironment<'a> { pub obstacle_map: &'a ObstacleMap, pub primitives: &'a PrimitiveLibrary }
    pub struct SearchOutcome { pub route: Option<RouteResult>, pub stats: RouteSearchStats }
    pub trait NetSearch { fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome; }

The four methods of the old `SingleNetSearch` and the ten public `route_single_net_*` functions become this one method; the differences between them (dynamic expansion or not, collision crossing or not, stats returned or not) are fields of the request, all `Option`s, so that the plain case is the request with everything `None`. The old names are deleted, not kept as wrappers, once every call site in the engine uses the new one.

Work, A* module. Split `src/search/astar.rs` (14,997 lines) into `src/search/astar/`: `state.rs` (State, CellKey, angle bookkeeping), `config.rs` (the `SearchConfig` from Milestone 1 lives in `crate::config`; this file holds derived search-time constants only), `heuristic.rs` (the `SearchHeuristic` and its modes, from lines 8,805-8,829 today), `cost.rs` (the g-cost terms: length, bend, crossing price, congestion and long-straight penalties, history), `expansion.rs` (primitive moves, legality checks, terminal straight rules), `crossing_rules.rs` (the collision-triggered crossing legality that Milestone 5 of the 2026-08-25 kernel plan unified), `kernel.rs` (the open list, the closed set, the loop; target: under 600 lines, one function `run`), `dense.rs` (dense-state storage and its cap), `svg.rs` (the debug export), and `mod.rs` with `pub struct AStarSearch` implementing `NetSearch`. The 140 tests move with their code.

Work, second engine. `src/search/grid_dijkstra.rs`: a plain Dijkstra over the same `State` space with the same primitive moves and legality checks from `expansion.rs`, no heuristic, no crossing support (it returns `route: None` with a stats flag when the request asks for crossing search). It exists to prove the seam and to serve as a slow oracle in tests: on small fixtures the A* route cost must equal the Dijkstra route cost. The engine (`engine::Router`) becomes generic over `S: NetSearch` with `AStarSearch` as the default type; the binding constructs `Router<AStarSearch>`. A test constructs `Router<GridDijkstraSearch>` and routes a two-net fixture through the whole negotiated loop.

Acceptance: `grep -rn "route_single_net" src/` reports nothing outside `src/search/`; the Rust test count is the pinned baseline plus the new tests; `gate_short.sh` identical; the full 27-cell reproduction identical (this milestone touches the kernel, so it is run, about 4.5 hours, sequential, on an otherwise idle machine).


## Milestone 4: one readable rip-up-and-repair loop

Goal: `route_many_with_negotiated_repair_and_commit` reads as the loop it is. A reader opens `src/engine/negotiation/loop.rs` and sees, in under 200 lines, the rounds, the queue, the per-net attempt, and the calls to named policies; the policies live in their own files behind small traits and can be tested with fixtures.

Work. Extract from the 948-line method, without changing what happens or in which order:

    trait BudgetSchedule   { fn budget(&self, net: &NetAttemptContext) -> Option<u64>; }
                             impl LadderBudgets (first / first-retry / retry / last round unbounded,
                             the values every paper row used; Milestone 9's span scaling is not on main)
    trait RipUpPolicy      { fn choose_victims(&self, probe: &ProbeReport, ctx: &NetAttemptContext) -> RipUpDecision; }
                             impl LidarStyleRipUp (rip the illegal partners at once, re-queue behind the net,
                             each partner at most once per epoch)
    trait CrossingFreePolicy { fn crossing_free(&self, net: NetId, round: u32) -> bool; }
                             impl PlannedPairsOnly (contribution 1's unplanned-nets-first rule) and impl Never
    struct NetQueue        (the order, the failed counts, the ripped-once set, the requeue operation)
    struct AttemptRecord   (what one attempt did: kind, budget, expanded, outcome; feeds diagnostics and Python)

The loop body then reads: for each round, for each net in the queue, compute the budget, search, if routed commit and run braid passes, else probe, if the probe names no blocker escalate the budget once (the clean-probe rule), else apply the rip-up policy, search again, commit or re-queue; after the last round build the result. Every branch that exists today exists afterwards; the difference is that each branch is a call to a named policy with a docstring stating the rule and the date it was introduced, and that the policies are unit-tested with the `crossing_conflict_fixture` that already exists in the tests.

Legacy paths per D1: if the owner decides to delete them, `route_many` without negotiation, the legacy repair chain and the orthogonal fallback are removed here together with their configuration fields and tests, and the Decision Log records the commit. If the owner keeps any, it becomes another `RipUpPolicy` implementation, not a parallel loop.

Acceptance: `loop.rs` under 250 lines including comments; every policy has at least three unit tests (routed first try, blocked then repaired, blocked and unrepairable); test baseline; `gate_short.sh` identical; full 27-cell reproduction identical (the loop is what every paper row ran through).


## Milestone 5: the Python side as stages, a flow without argparse, a human command line

Goal: a human can route a benchmark from Python in three lines and from the shell with one short command, the flow is a sequence of stage calls each behind a Protocol, and the argument parser is one file that produces the configuration object of Milestone 1 and nothing else.

Work, stages. `translation/route_rust.py` (8,295 lines, one class) becomes the package `translation/routing/` with one module per phase that `run()` already calls in order: `obstacle_context.py` (Protocol `ObstacleContextBuilder`), `route_jobs.py` (`RouteJobBuilder`, including fan-out clustering and static stubs), `crossing_plan.py` (`CrossingPlanner`, guided or pre-placed), `dispatch.py` (`KernelDispatcher`, the only module that talks to the Rust binding; today's 449-line `_dispatch_native_routing`), `finalize.py`, `verify_repair.py`, `realize.py`, and `session.py` holding the nine-line `run()` that calls them. Each Protocol declares inputs and outputs as dataclasses from `route_rust_types.py`, and each stage gets a fixture-based unit test that does not run a benchmark. The electrical routing package already has this shape and is left alone.

Work, entry points. `routing_flow.py` is split into `photonic_router/cli.py` (argparse, 54 arguments regrouped under the configuration sections of Milestone 1, plus `--configuration {baseline,contribution1,contribution2}` which expands to the exact flags the paper rows used) and `photonic_router/flow.py` with `route_benchmark(config: RoutingConfig) -> RoutedLayout` and `route_schematic(schematic, config) -> RoutedLayout`. `python routing_flow.py <benchmark> ...` keeps working as a shim that calls the CLI, because the reproduction scripts and the benchmark stable blocks use it; the shim is deleted only after those are rewritten to the new command (D4, D6). Benchmark modules keep `build_schematic()` and replace `STABLE_ROUTING_ENV` / `STABLE_ROUTING_FLAGS` with a `stable_config() -> RoutingConfig` function.

Acceptance: `python -m photonic_router route benes_8x8_flat --configuration contribution2` routes and writes the same GDS as the reproduction script's command for that cell; `gate_short.sh` (which goes through the shim) identical; `flow.py` has no `argparse` import and no `os.environ` access; `session.py`'s `run()` is a straight sequence of stage calls.


## Milestone 6: tests per module

Goal: every module created in Milestones 2-5 has unit tests written against its interface, and no test relies on a full benchmark run unless it is explicitly an end-to-end test in `tests/e2e/`.

Work. Inventory the 390 Python and 500 Rust tests, classify each as unit (one module, fixture), integration (several modules, small synthetic layout) or end-to-end (a benchmark), and move end-to-end tests under `tests/e2e/` with a pytest marker so `pytest -m "not e2e"` finishes in under a minute. Write the missing unit tests for: `config` (defaults, overlay, Option semantics), every `NetSearch` implementation (route found, blocked, budget exhausted, crossing request handling), every negotiation policy, `NetQueue`, commit and uncommit round trips on the obstacle map, and each Python stage Protocol. Where a fixture does not exist, build one small synthetic layout generator in `tests/fixtures/` (two to eight nets, a few instances) reusable by both Python and Rust tests through a JSON description.

Acceptance: `pytest -m "not e2e"` under 60 seconds with zero failures; `cargo test --release --lib` zero failures; the test count and the module list are recorded in the plan; `gate_short.sh` identical.


## Milestone 7: the architecture document and the final full reproduction

Goal: the structure is written down once, in the repository, generated where possible, and the restructured code reproduces all 27 paper cells.

Work. `docs/ARCHITECTURE.md`: the module map of Rust and Python with one paragraph per module, the flow as the literal sequence of stage calls, the search interface and how to add an engine, the negotiation loop and how to add a policy, and the build and test commands. `docs/CONFIGURATION.md` generated from the configuration dataclasses' field docstrings by a small script, replacing the environment-variable folklore spread over the ExecPlans; the overlay names are listed in a table next to their fields. `.agent/REPOSITORY_STATE.md` gets a short entry pointing to both. Then the full reproduction: `scripts/results/reproduce_date2027.sh` into a fresh root and `compare_date2027.py`, all 27 cells exact, recorded in `docs/DATE2027_REPRODUCTION.md` with the commit.

Acceptance: both documents exist and match the code (spot-checked by a reviewer who has not worked on the plan); the 27-cell comparison prints `all reproduced cells match`; the Outcomes & Retrospective section of this plan is written.


## Order, size and what could go wrong

The order is deliberate: configuration first because hidden inputs make every later diff unreviewable; the file split second because it is mechanical and makes the real work reviewable; the kernel interface before the loop because the loop is generic over it; Python after Rust because the Python stages' outputs are defined by what the kernel needs; tests and documents last but written against interfaces that exist by then. Milestones 2, 3 and 4 are the large ones; Milestone 1 is medium and the most error-prone because 88 variables have to be mapped without changing any default; Milestones 0, 5, 6 and 7 are medium.

The likely failure modes, all caught by the gate if it is run after every step and not only at milestone ends: an environment variable whose absence meant something (Milestone 1); a moved function whose `self.` field access silently changed which struct it reads (Milestone 2); a request-struct default that differs from the old positional default (Milestone 3); a reordered branch in the loop (Milestone 4). When the gate fails, the diff since the last green gate is small by construction, and `git bisect` over the branch's commits with `gate_short.sh` as the test finds the commit.

Revision notes: 2026-09-22, first version, written after the 27-cell reproduction and the rejection of the 2026-09-19 cleanup.
