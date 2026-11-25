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

- `tests/logic/test_hash_seed.py`: a run does not depend on the hash seed
  of the process. Four cells, one per way of routing (the standard
  router with path-length matching, the guided search, the X array and
  the column grid of the pre-placed crossings), are each routed in two
  processes with different `PYTHONHASHSEED`; the routed records (order,
  centerlines, lengths) have to be equal. The column-grid cell fails on
  the code before the fix of 2026-11-24 (python 982 to 986).
- Logic harness, stage `electrical` (`logic_checks/electrical.py`; owner,
  2026-11-23): the wiring of the heaters is checked on every benchmark
  that asks for it (13 public ones). Every heater has one terminal on the
  common bus and the other wired to a pad by exactly one wire; no pad
  slot holds two nets; a wire is a path of neighbouring cells that ends
  on its pad; two wires share no cell; an error of the electrical
  router's own verification is a finding. The flow reports the stage
  after its electrical step and a last stage `done`, where
  `scripts/logic_check.py` now ends a run. No benchmark has a finding;
  `tests/logic/test_electrical_checks.py` shows each statement failing
  on a wiring built wrong (python 979 to 990).
- `scripts/private_circuits_check.py`, run by `scripts/check.sh --full`
  (owner, 2026-11-23): every circuit of `benchmarks/private/` is routed
  with its own defaults and has to end with exit code 0 and no verifier
  error. The directory is not in the repository, so no gate routed its
  circuits; they had been unroutable since 2026-11-05 (the full-grid
  search after the search window went off by default, and
  `--path-length-allow-residual` was replaced) without a check
  noticing. Without the directory the script passes and says so; nothing
  about the circuits is pinned. `tests/test_private_circuits_check.py`.
- A register of the router's fallbacks (owner, 2026-11-23; rule 3.8 of
  `docs/CODING_STANDARDS.md`). `records/fallbacks.py` names thirteen
  fallbacks and what each takes instead of what; the place that takes one
  calls `fallbacks.take(name)`, the flow resets the counts when a run
  starts. `scripts/logic_check.py` reports them per benchmark as notes
  `fallback.<name>`, once per time taken, and the pin holds the counts:
  over the 123 pinned runs two are ever taken (`flow.standard_router` in
  15 runs, `crossings.layer_to_router` for 8 nets of one run). The new
  ratchet measure `fallbacks_never_taken` counts the ones no pinned
  benchmark and no test takes (11). Whether a fallback is needed is
  judged on every benchmark the router has to route: the full-grid
  search was switched off on 2026-11-05 on the evidence of the 27 paper
  cells, and the private circuits have not routed since (found today;
  open with the owner). `tests/test_fallbacks.py` holds the register and
  the code to each other.
- Two rules of `docs/CODING_STANDARDS.md` (owner, 2026-11-23). 4.11: a
  function reads every parameter it takes; ruff `ARG001` is in the lint
  set. The last four cases are settled: `port_access_rule_for` loses
  `state`, and two callbacks name the parameters they do not need with an
  underscore. 4.12: a function or class of the package is used by the
  package, a script or a benchmark; `scripts/quality_ratchet.py` counts
  the names only tests mention (`names_only_tests_use`, 16) and the count
  may only fall.
- `tests/test_gds_circuit_extraction.py` checks that every extracted
  port's local position and normal, moved by its component's rotation
  and origin, give the port's position and normal: on the circuit with
  grating couplers turned by 180 degrees and on the one with grouped
  directional couplers.
- `scripts/logic_check.py --plm` routes the 13 path-length-matching
  benchmarks with the standard router, the configuration they are made for
  (owner, 2026-11-22: the two crossing contributions are meant for the
  Benes and ADEPT designs). `REQUIRED_CELLS` (the paper's 27 cells and
  these 13) is what the router has to route; a test holds that none of them
  has a known error or is refused.

- Logic harness, stage `matched` (`logic_checks/matching.py`): are the
  path lengths equal where the design needs them equal? After the
  meanders are planned, `route_match_and_realize` reports a
  `MatchedLengths`; the checks recompute every arrival along the graph
  from the records' final lengths (to 1 nm) and, at `realized`, from the
  polygons of the layout (area over width, to 0.5 um), for every group
  and for the matched outputs. A group the benchmark declares unmatched
  is a note. Every run with path-length matching is checked, in the pin
  and under `--verify-stages`.
- The logic harness runs every public benchmark in the three router
  configurations (`scripts/logic_check.py`: `--configuration`, repeatable,
  default all three; `--jobs N`; `--route-wide` for the designs from
  16 x 16 on). The pin `tests/baselines/logic_findings.json` is per
  benchmark and configuration (123 runs). A run the flow ends is a finding
  (`routing.net_not_routed`, `flow.run_failed`).
  `--paper` routes the 27 cells of the paper's results table
  (`PAPER_CELLS`): what the router has to route for now (owner,
  2026-11-22), each design with the configurations it has in the paper.
- `docs/CODING_STANDARDS.md` is adopted (owner, 2026-11-22), on the
  condition that it stays close to the public standards of its section 1;
  every departure is listed there as a deviation. Rules marked *proposed*
  are binding for new code, and the quality ratchet holds the existing
  violations.
- `docs/CODING_STANDARDS.md` has the rules of the logic harness (6.5 to
  6.9: statements per stage, checked on the real product by pure
  functions, each with a test that breaks one thing, findings pinned per
  benchmark, one path for a new stage) and, in section 1, the practices
  they follow with their sources as read on 2026-11-21 (LLVM's module
  verifier and its assertion and expensive-check switches, GCC's
  `--enable-checking`, SQLite's `assert`/`ALWAYS`/`testcase` and mutation
  testing, Power of 10 rule 5, TensorFlow Data Validation), and what was
  not taken over or not read. The document stays a draft.
- `--verify-stages`: a normal run of the flow checks what every stage
  hands on. `logic_checks/stages.py` holds which checks read which stage
  (`stage_findings`) and the verifier (`verify_stages()`, around
  `route_schematic` when `DebugArtifactOptions.verify_stages` is set): one
  line per stage, and `StageVerificationError` at the first stage with an
  error; notes do not end a run. Off by default. The flow reports the
  placed design (`"loaded"`, `LoadedDesign`), so the reading checks and the
  crossing demand run on the same path, and `scripts/logic_check.py` now
  collects its findings through `stage_findings` too: one path for both.
  All eight stages report no error on Benes 8 x 8 in the three
  configurations (baseline, contribution 1, contribution 2 with its probe
  session). Test baseline python 870 -> 875.
- The logic harness, stage "realized". `logic_checks/realized.py` has nine
  statements about phase 8's product and the polygons of the routed
  layout: no realized crossing is left illegal; polygons and nets belong
  to each other; a net is one polygon more than it has crossing
  components; its ports lie in its polygons; its centerline lies in its
  polygons and crossing footprints; the polygons are as long as the
  record says (with a meander: the recorded length plus the extra length
  of the plan); polygons of two nets do not overlap. The session reports
  phase 8's product (`"verified"`), `route_match_and_realize` the realized
  nets (`"realized"`, `RealizedNets` in `routing/products.py`); an
  observer is now called with `None` for the settings outside a session.
  Found on the 21 public benchmarks that route: no error. Noted: the nets
  with a meander; such a record keeps the centerline and length from
  before the meander and carries the meander as a plan. With this stage
  the harness follows a design from the schematic to the polygons. Test
  baseline python 863 -> 870.
- The logic harness, stage "records". `logic_checks/records.py` has eight
  statements about the routed net records after the endpoint correction
  (phase 7): the records are the links; no correction failed; a corrected
  centerline runs from port centre to port centre (to 1 nm), has no
  repeated point and leaves and arrives along the ports' directions (to 5
  degrees; 2.8 is the most seen); the recorded length is the centerline's
  and the base length the kernel route's. The session reports the records
  after phase 7 (`"records"`). Found on the 21 public benchmarks that
  route: nothing; every net is corrected onto both of its ports. Test
  baseline python 857 -> 863.
- The logic harness, stage "routing". `logic_checks/routing.py` has twelve
  statements about what the kernel batch hands back: the routed nets are
  the jobs; a route starts and ends in its job's two states; its segments
  are chained and as long as they say; the lengths of segments, route and
  record agree; its cells are a path through free or opened cells; two
  nets' cell paths meet exactly where a crossing of the two is recorded
  (in a shared cell or across the two diagonals of a unit square), and
  every recorded crossing is legal. The session reports the state after
  phase 6 (`"routed"`). The harness routes the 23 public benchmarks below
  16 x 16 (about 100 s together) and stops the wider ones at the hand-off.
  Found: no statement fails on a routed design (Benes 8 x 8: exactly the
  16 pairs with a recorded crossing meet). Two default runs do not route:
  `heater_single` (a known limit written in the benchmark) and
  `permutation_synthetic` (the baseline does not converge on the random
  permutation within 10 rounds); both are the error
  `routing.net_not_routed`, pinned as known per benchmark. Test baseline
  python 848 -> 857.
- The logic harness, stage "crossing plan". `logic_checks/crossing_plan.py`
  has nine statements about the crossing demand of the placement analysis
  (every link in one band or outside; ends on the walls; wall orders top to
  bottom; invariant pairs recounted as the inversions between the walls;
  events are the pairs and sort one wall order into the other) and four
  about phase 4's plan info (the baseline holds nothing of a plan; in
  guided mode the router gets the invariant pairs and the expected counts
  are theirs). The demand is checked on all 41 public benchmarks including
  the large ones, the plan info at the hand-off. Found: no error; the
  default runs are `lidar-pure`, so the pin shows on every benchmark that
  the baseline holds no plan. Noted: 545 links outside the band model on
  the cascade, Clements and heater designs, each with a way through the
  passages. Test baseline python 836 -> 848.
- `scipy` is a declared dependency of the package (`scipy>=1.13,<2`).
  `logic_checks/route_jobs.py` imports it; until now it was installed only
  because gdsfactory depends on it.
- The logic harness, stage "route jobs". `logic_checks/route_jobs.py` has
  nine statements about what phases 3 to 5 hand to the kernel: the jobs are
  the links, both grid states of a job are on a free or opened cell and
  head as the port points, and the two ends of every net are connected
  through cells that are free or opened for it. The session reports the
  state after phase 5 (`"handoff"`). Found on the 41 public benchmarks:
  nothing; every net is routable as far as the static cells go. The pin
  test reads the twelve 64 x 64 and 128 x 128 designs through the
  reading stage only unless `LOGIC_CHECK_LARGE=1`. Test baseline python
  828 -> 835.
- The logic harness, stage "obstacle map". `logic_checks/obstacle_map.py`
  has ten statements about phase 1's static obstacle map.
  `routing/stage_observer.py` is the seam the harness reads a session
  through: `observe_stages(observer)`, `notify` after phases 1 and 2,
  `StopAfterStage` to end the run before any net is routed; a run without
  an observer is unchanged. Found on the 41 public benchmarks: every
  obstacle is blocked and every linked port can leave (the recessed heater
  ports too). One error, on the 40 benchmarks routed with heater obstacles:
  `build_stats` counts ports and port-open cells twice and its
  `blocked_cell_count` is not the blocked area, because the map is built
  from two layer groups whose stats are added up; the flow statistics take
  their `port_open_cells` from it. Pinned as a known error, not fixed here.
  One note: the keep-out band is rounded outward, so the cells straddling
  the edge of the routable box stay routable. Test baseline python 817 ->
  827.
- The logic harness, stage "reading". `python/photonic_router/logic_checks/`
  tests what a stage hands on against statements that have to hold for the
  next stage to mean anything; a finding is an error (the product
  contradicts itself) or a note (legal, a later stage has to handle it).
  `reading.py` has 17 statements about the schematic and the placed layout.
  `scripts/logic_check.py` runs them, `tests/baselines/logic_findings.json`
  pins the counts on the 41 public benchmarks. Found: no error anywhere;
  on the 29 benchmarks with heaters every heater's optical port lies 15.9 um
  inside the heater's bounding box, short of the face it points at (9,468
  ports), so a waveguide starts over its own instance's box; on the two
  heater-lane benchmarks the die outline overlaps every instance, by
  design. Test baseline python 765 -> 817 (10 unit tests, 42 pinned).
- `mypy` passes on the whole package (348 errors at the first measurement,
  277 before this change) and is a check of `scripts/lint.sh` and CI instead
  of a count of the ratchet; `types-shapely` joins the `dev` extra. The
  errors went by typing what was `object`: the report functions take the
  records they read, the realization, probe and crossing-plan callers name
  the router methods they use as Protocols, one variable no longer holds
  two types. `records.route_rust_types.int_or_none` replaces four private
  copies. `resolve_obstacle_config`, `with_obstacle_mode` and
  `with_bbox_cell_materialization` return a `StaticObstacleMapConfig` and
  refuse (`TypeError`) what is neither that record, a mapping of its fields
  nor None; before, an unknown object was handed on unchecked. Removed as
  dead: `_apply_crossing_aware_endpoint_corrections_to_debug_artifacts`
  (no caller), the `degraded_reason` variable that was None on every path,
  and five shape checks on values whose type already guarantees the shape.
- A local harness for `docs/CODING_STANDARDS.md`: `scripts/check.sh` is the
  one entry to every check (fast stage: `scripts/lint.sh` and the harness
  tests; `--full`: also the kernel build, `scripts/test_baseline.sh` and the
  two gates), `scripts/hooks/pre-commit` runs the fast stage before a commit
  (`git config core.hooksPath scripts/hooks`; it replaces
  `.pre-commit-config.yaml`, which needed the `pre-commit` package and ran
  neither the ratchet nor clippy), and `tests/test_coding_standards.py`
  fails when a rule marked "Tool, in force" quotes no check the harness
  runs, when a proposed Tool rule has no row in the ratchet table, or when
  that table and the pin list different measures. The test found two counts
  that had reached zero and were still pinned: ruff `D100` and `D104` are
  now in the lint set, and the ratchet holds 18 counts.
- The quality ratchet measures rule 3.4 (dependencies point one way):
  `upward_imports` counts the import statements from a lower layer of the
  package into a higher one (`records`, `config` < the stage packages <
  `flow*` < `cli`). Three exist, all out of `records/`: two of
  `plm.path_length_graph` (`PortRef`, `RoutedEdgeKey`) and one of
  `routing.crossing_plan_info`. The ratchet holds 23 counts.
- The quality ratchet: `scripts/quality_ratchet.py` measures 23 counts of
  what does not meet the proposed rules of `docs/CODING_STANDARDS.md` yet
  (ruff rules that are not selected, mypy's errors on the whole package,
  imports of private names, untyped dictionaries, the fields of the session
  state, three clippy lints on `router-core`, the exceptions) and compares
  them with `tests/baselines/quality_ratchet.json`. A count may only fall,
  and a fall is pinned in the same commit. `scripts/lint.sh` and CI run it
  in place of the informational mypy step. The limits are configuration:
  `max-args = 7`, `max-statements = 50`, `max-complexity = 10` in
  `pyproject.toml`, `too-many-lines-threshold = 120` in the new
  `clippy.toml`; none of them changes a lint result today. Section 8 of the
  standards is corrected by the measurement: 58 functions exceed 7
  parameters (136 was counted against ruff's default of 5), and one
  `unwrap()` is outside test code (184 included the tests).
- `docs/CODING_STANDARDS.md`, a draft (not adopted): the reference standards,
  the proposed rules for architecture, Python, Rust, tests and documentation
  with the way each is enforced (tool, test, review), how exceptions are
  written, and a table of where the code stands against each rule on
  2026-11-19. No code or check changes with it.
- Coding standards, section 1 after reading the references: the Google
  Python Style Guide and Microsoft's Pragmatic Rust Guidelines join the PEPs
  and the Rust project's guides, with the deviations named (line length 100,
  ruff, name imports) and the origin of every limit. Two rules follow from
  them: imports are absolute (rule 4.10; the ratchet holds the 96 relative
  import statements as `relative_imports`), and a Rust exception is written
  `#[expect(lint, reason = "...")]` (the ratchet counts both spellings).
- The placement analysis as a stage of its own (region graph, intermediate
  step between phases 1 and 2; a restructuring). New
  `python/photonic_router/placement_analysis.py`:
  `analyze_placement(schematic, unrouted_layout)` returns a
  `PlacementAnalysis` (the `CrossingDemand` and its `RegionGraph`). Until now
  three places built the demand for themselves (`flow.py` for contribution 2,
  `route_rust_crossing_plan._build_crossing_plan_info` for the guided
  session, the stage-schedule helper of `preplaced_crossing_grids.py`). The
  flow now runs `_placement_analysis_stage` (step 2a of its output) once per
  placed design and only for a stage that reads the result: contribution 2's
  tile stage takes it as an argument; the guided session receives it through
  `build_optical_routing_stage_config`, `OpticalRoutingStageConfig`,
  `route_match_and_realize` and `SessionSettings.placement_analysis`, and
  `_build_crossing_plan_info` reads `placement_analysis.crossing_demand`.
  With a router fallback under contribution 2 the derived design is analysed
  as well, for the fallback nets. The baseline (`lidar-pure`) gets no
  analysis. A session entered without the flow analyses its own design with
  the same function. One error path changed: a net naming a port the layout
  does not have now fails in the flow's analysis stage with its `KeyError`
  instead of being recorded as `invalid_crossing_demand` in the session's
  plan info (which let the guided search run without guidance); route jobs
  are built from the same ports before that, so no routable design reaches
  it. `preplaced_build_s` still includes the analysis time. Contract tests:
  `tests/test_placement_analysis.py` (the session reads the flow's analysis
  and does not analyse again; without one it analyses itself; the baseline
  never reads one) and `tests/e2e/test_placement_analysis_flow.py` (analyses
  per configuration on Benes 4x4: 0, 1, 1, and the demand is never built
  outside the stage). Nine-cell gate and PLM gate exact.
- The passage report (region graph phase 3, a report: no plan changes).
  `region_paths.passage_load_report` lists every passage with a path by the
  height each path would get if they shared it evenly;
  `PlacementAnalysis.passage_summary()` is one line with the number of nets
  led through passages, the vertical travel the columns add to them, and
  the tightest passage. The flow prints it under the analysis line when the
  design has such nets (none of the paper benchmarks has). Whether a
  passage is too full depends on the waveguide pitch, which the analysis
  does not know; the line states the height per path and no verdict. Test
  baseline python 763 -> 765.
- Paths through the passages for nets that pass columns (region graph phase
  2, a report: no plan changes). New
  `python/photonic_router/crossings/region_paths.py`: `plan_region_paths`
  leads every net whose two terminals point into different bands through one
  passage of each column in between, band, passage, band, ..., and takes the
  sequence with the least vertical travel (ties to the lower passage);
  `RegionPath` holds the regions, the passing heights and the detour against
  the terminals' height difference, `passage_loads` counts the paths per
  passage. `CrossingDemand.passage_paths` holds them for the
  `non_adjacent_walls` nets; these nets stay in `outside`, get no events,
  and nothing reads the paths yet. Measured on the 20 benchmarks with such
  nets (twelve cascades, `heater_s_mod`, `heater_single`, `clements_8x8`,
  five private circuits): all 707 nets have a path, none is blocked by a
  solid column, none has a detour; the fullest passage carries 14 paths, and
  the tightest one has 13.4 um of height per path. Test baseline python 754
  -> 763 (`tests/test_region_paths.py`, one test in
  `tests/test_crossing_demand.py`).
- The placement's free space as a region graph (region graph phase 1, a
  restructuring: no plan changes). New
  `python/photonic_router/crossings/routing_regions.py`: `cluster_columns`
  and `Column` moved here from `crossing_demand.py`, and a column now carries
  the merged y-intervals of its instances (`occupied`).
  `build_region_graph(columns)` returns a `RegionGraph` of *bands* (the space
  between two consecutive columns over the chip height), *passages* (the
  y-intervals of a column that no instance occupies: above, below and
  between its instances) and *portals* (the segment a passage shares with
  each band beside its column). `crossing_chords` is the crossing test of a
  region: two nets cross in it iff their ends alternate around its boundary;
  it replaces `crossing_demand._alternating_pairs`, which was that test for
  a band's two walls. `demand_from_geometry` takes the `RegionGraph` instead
  of the column tuple, and `CrossingDemand` gains `regions` and `paths`
  (every confined net: the one band it runs through). Why: on the 20
  benchmarks outside the paper table (the twelve cascades, `heater_s_mod`,
  `heater_single`, `clements_8x8`, the five private circuits) 719 nets, 10
  to 35 % per benchmark, are outside the band model, 659 of them because
  they pass one or more columns; a path through passages is what the next
  phase gives them. The plan (columns, bands, wall orders, invariant pairs,
  events, outside nets and their text) is identical before and after on all
  46 benchmarks with a schematic; nine-cell gate and PLM gate exact.
  `tests/test_routing_regions.py` (11 tests).

- `experiments/`: the ideas tried against the paper table that did not
  become part of the main code, and the records of the strategy switches
  that did (`experiments/README.md` has the convention). One folder per
  experiment with the idea, what was built, the measurement, the decision
  and how to reproduce it; the code of a closed experiment as
  `git format-patch` output against its base commit, its measurement
  scripts, and compact records of its 27-cell tables (`tables/*.json`,
  rendered by `experiments/table.py` against another record or against the
  pinned table). Entries: `port-runways` (phase 5, not adopted),
  `crossing-hook-kernel-grid` (fd70232 / 89fe1b1, not adopted),
  `negotiation-first-attempt-full-budget` (speed pass B1, not adopted),
  `window-ladder` (speed pass S4, adopted), `window-reachability-pruning`
  (speed pass B2/B3, not adopted), `lto-build` (speed pass A1, not adopted).
  Rule from here on: an experiment that is not adopted leaves no switch in
  the main code; its commits become patches in its folder and the gate shows
  the tree without them exact (`CONTRIBUTING.md`, "Experiments").
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
  Second table (b45479b): 26 of 27 cells route with 0 errors and the pinned
  crossings except Benes 32x32 lidar-pure 416 -> 418; ADEPT 128x128
  contribution 2 routed (311 crossing components) but failed verification
  on two dead-straight nets (heater -> multiport MMI, 0.875 um residue on
  both ports): the dense *target* stub is straight at the port's true
  cross-axis coordinate and leaves its sub-cell shift to the checked target
  correction, which had no free axis-aligned segment once the source side
  carried a runway. With the switch on, the dense target stubs therefore
  take the runway geometry too (`_port_runway_anchor(min_forward_cells=
  <staggered length>)`: same anchor cell, same lane protection, lattice-
  exact anchor, zero-length checked correction); switch off they stay as
  they are (gate exact). Nine cells with the switch on: Benes unchanged,
  ADEPT 8x8 lidar-pure 25166.341 (-0.02 %), contribution 1 25153.798
  (+0.25 %), contribution 2 26479.705 (+0.61 %), crossings and errors as
  before; ADEPT 128x128 contribution 2 verifies again (311 crossing
  components, 0 errors). Third table (d3a62fe, switch on): all 27 cells
  route with 0 verifier errors; crossings equal the pins on 26 cells (Benes
  32x32 lidar-pure 418 against 416); lengths against the pins Benes
  lidar-pure +0.98 to +1.12 %, Benes contribution 1/2 +0.44 to +0.59 %,
  Benes 64x64 / 128x128 contribution 2 +0.28 / +0.16 %, ADEPT -0.02 to
  +0.61 % (8x8 pays most, 64x64 and 128x128 +0.07 to +0.12 %); ADEPT
  64x64 needs 6 / 4 / 5 negotiation rounds against the pinned 3 / 3 / 1,
  ADEPT 128x128 contribution 2 9 against 8. Decision (owner, 2026-11-09):
  not adopted; the paper numbers would carry a measurable percent of length
  and four more arcs per net for code simplicity. The three slices live on
  as `experiments/port-runways/` (patches, measurement scripts, the three
  tables); the main tree is without them (see "Removed"), the nine-cell
  gate exact, 552 fast tests.
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
  table decides. Not adopted (owner, 2026-11-05: a wash, total wall 0.99,
  six lengths move, Benes 16x16 contribution 1 twice as slow); the switch
  left the tree 2026-11-09 and the record is
  `experiments/negotiation-first-attempt-full-budget/` (patch, both tables).
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

### Fixed

- The column grid of contribution 2 was placed differently from process
  to process (found 2026-11-24 while comparing two runs). `column_grid_stage`
  went through the two lanes of a crossing as a set of names, and Python
  orders a set of strings by a hash it seeds per process; the order of
  the nets between a lane's tiles and the numbers and places of the
  static links followed it, and with them the order the router takes
  the nets in. The lanes are sorted now. The X array (Benes) was not
  affected. `tests/test_preplaced_crossing_grids.py` places ADEPT 8x8
  under two hash seeds and wants the same schematic (it failed before);
  under three seeds the eight designs tried (ADEPT 8x8 to 128x128,
  Benes, `permutation_synthetic`) come out the same.
- `flow.load_benchmark` reported every `ModuleNotFoundError` as
  "Benchmark not found", also one raised by an import inside the
  benchmark module, and every `AttributeError` inside
  `build_schematic()` as "must have a build_schematic() function". It
  rewrites only the two cases it names; any other error is the
  benchmark's own (`tests/test_load_benchmark.py`).
- `scripts/extract_private_benchmark.py` wrote
  `--path-length-allow-residual true` into every private circuit's
  defaults, a flag the command line has not had since 2026-11-05, so a
  regenerated circuit could not be started. It writes the matching flag
  alone now; a circuit names the groups it leaves unequal in its own
  flags (`--path-length-declared-unmatched-groups`). The script also
  writes a `[config]` table that turns the full-grid search back on for
  these circuits (owner, 2026-11-23: the paper benchmarks stay as they
  are); `write_benchmark_module` and `defaults_toml` take the overlay.
- `gds/gds_circuit_extraction.py`: the outer ports of a grouped coupler get
  their local position in the frame every other port has (rotation and
  displacement, `_local_frame`); only the displacement was undone, which
  is wrong for a rotated instance. The extraction of the five private
  circuits is unchanged (hash of all components and ports before and
  after, read in place).

- No file names a path of one machine any more: the LiDAR comparison
  scripts (`scripts/results/lidar_12h.sh`, `lidar_12h_adept64.sh`,
  `queue_night.sh`, `rerun_all.sh`) find the LiDAR checkout through
  `LIDAR_ROOT` (default: next to this repository), the `source:` line of
  `benchmarks/data/multiportmmi_8x8.yml` and two notes in `docs/history/`
  name paths inside the checkout. Removed:
  `scripts/path_investigation/benes32_layer273_testbed.sh`, a one-off
  testbed bound to one machine that nothing referred to.
- Findings of the docstring pass of 2026-11-21, checked against the code:
  - `electrical/rect_geometry.py`: `rect_is_covered` (was
    `rect_is_covered_by_any`) tests the union of the covers. The
    electrical verifier no longer reports an obstacle overlap as illegal
    when two allowed contact boxes cover it together. The electrical
    suite's numbers do not move.
  - `OpticalRouteClearancePolicy.from_dimensions` refuses a clearance
    that is not a number; `max(0.0, nan)` had turned it into 0.
  - `crossings/crossing_demand.py`: a net between two ports of one column
    that point into two bands is reported as such, no longer as "skips -1
    column(s)".
  - `experiments/table.py`: a failed reference cell no longer ends the
    table with a `TypeError` and is left out of the geometric mean; a pin
    whose crossing count is only in `current` is read.
  - `benchmarks/clements_8x8.py`: `SOURCE_YAML` and `REFERENCE_GDS` are
    paths inside a LiDAR checkout, not absolute paths of one machine; the
    alignment test finds the checkout through `LIDAR_ROOT` (default: next
    to this repository).
  - Docstrings of `routing/finalize.py`, `routing/verify_repair.py` and
    `MeanderPlannerLike` name the settings and the file that exist.

- Contribution 1 routes a design whose crossing plan has no crossing.
  The hand-off took "the plan has no event" for "there is no plan" and
  ended the run with "run with --crossing-mode lidar-guided" although that
  was set (found by the logic harness on `clements_8x8`,
  `heater_lanes_20`, `heater_lanes_ripup`, `heater_single` and
  `multiportmmi_64x64_bands1to2`). The plan-based net order is refused
  only when no plan was made, and the message names why.
- `build_stats["blocked_cell_count"]` and `raw_blocked_cell_count` of the
  static obstacle map are the blocked area: a cell under several boxes
  counts once. In bounding-box mode the Rust builder added up the areas of
  the boxes, which overlap (Benes 8 x 8 without heater obstacles: 306,884
  reported, 299,524 blocked), and the merge of the two layer groups added
  up the groups (with heater obstacles: 1,176,884 reported, 884,244
  blocked). New in the core: `union_area_cells` (a sweep with a segment
  tree), and `static_blocked_area_rs` in the extension, which the merge
  uses too. With materialized cells the builder counted its cell list
  before duplicates were removed; it counts the rectangles' area in both
  modes now. `raw_box_area_cells` stays the sum of the raw boxes. Nothing
  routed reads the count; the flow statistics fall back to it for their
  `blocked_cells`. The logic harness' known error
  `obstacle_map.stats_disagree` is gone from all 40 benchmarks it was
  pinned on, and the list of known errors is empty. Test baseline rust
  513 -> 515, python 835 -> 836.

### Changed

- The two checked endpoint passes (nets without stubs, nets ending in
  a fan-out stub) each held the same call of the kernel with its
  timing and its check for an old kernel. They share
  `_kernel_checked_corrections` and `_require_checked_corrections`; the
  stub pass (12 branches) merges the answers in
  `_take_fanout_stub_corrections`. The routed records of the 23 cells
  are the same, and the timing steps keep their names (`ruff_C901` 26
  to 25).
- `_apply_checked_endpoint_corrections_for_net_ids` (14 branches) packs
  its jobs in `_unrestricted_correction_jobs` and writes the kernel's
  answers into the records in `_take_checked_corrections`, as the pass
  for the fan-out stubs already did. The routed records of 23 cells
  (the three configurations, PLM designs, the private circuits) have
  the same order, centerlines, lengths and errors as before
  (`ruff_C901` 27 to 26).
- Electrical router: the bus search and the escape search each held
  the same breadth-first search and the same path reconstruction. Both
  call `wire_geometry.breadth_first_path` now and say only what is
  theirs: the starts, the order of the neighbours and which cell may be
  entered. The escape search gives the same path on 40,000 random
  grids, and the wiring of all 13 benchmarks with heaters is the same
  as text (`ruff_C901` 29 to 27).
- Electrical router, column trunks: `route_column_trunks` (15 branches)
  hands one group to `_route_one_trunk` and takes its members from
  `_trunk_members`; `_find_legal_column_trunk` (11) asks
  `_trunk_at_column` whether a trunk is legal in a column. The wiring
  of all 13 benchmarks with heaters is the same: the whole routing
  result (bus tree, pad plan, escape, pad wires, verification) compared
  as text, old against new (`ruff_C901` 31 to 29).
- Two functions without state, each compared with its old version on
  random input (`ruff_C901` 33 to 31).
  `format_native_repair_trace_lines` (14 branches) turns one record in
  `_repair_trace_entry` (20,000 lists of records with values of every
  kind). `merge_terminal_corrected_route_centerline` (13) finds the
  stitch point of a frozen end in `_stitch_index`, which was a nested
  function and a second lookup (60,000 centerlines, 11,688 of them
  merged, 35,330 refused).
- The two snapping functions of the hand-off
  (`_snap_nearly_collinear_states`, `_snap_same_heading_minimum_bend_offset`;
  13 and 12 branches) had one branch for ports facing along x and a
  mirrored one for y; each works on the coordinate along the heading
  and the one across it now. Compared with the old functions in seven
  runs, on the real calls and on states shifted around them (366,000
  and 117,000 calls, 174,000 of them snapped): the same states and
  cells (`ruff_C901` 35 to 33).
- The two builders of the static fan-out anchors (16 and 15 branches)
  share what they both did: `_cardinal_port_specs_by_angle` (the ports
  that face along an axis, by angle) and `_fanout_anchor`. The source
  side places its two-bend stubs through `_SourceStubs` (was a nested
  function over six variables) and takes a row's two halves from
  `_source_fanout_halves`; the target side takes its ranks from
  `_ranked_target_specs`. On 14 runs (ADEPT 8x8 to 32x32, Benes,
  `permutation_synthetic`, three private circuits) the 1,268 anchors of
  the old and the new builders are equal field by field (`ruff_C901`
  37 to 35).
- `_propagate_roles` of the path-length graph (22 branches) says its
  steps: `_beyond_the_inputs` starts the light at the input nodes,
  `_sides_as_entered` gives the input and the output side of a
  component for the port the light enters at, and `_LightFront` holds
  the roles given so far. On the graphs of all 46 benchmarks, the
  private circuits included (83 graphs, 49,247 port roles, 28
  conflicts), the roles, their order and the conflicts are those of the
  old function (`ruff_C901` 38 to 37).
- Two checks with one `if` per case read as lists (`ruff_C901` 40 to
  38). `ElectricalRoutingConfig.validate` (24 branches) goes through
  `_settings_out_of_range`, which yields each check with its message in
  the order they were written. `parse_debug_svg_selector` (18) reads a
  list like "1,3-5" in `_indices_of_selector_text`. Compared with the
  old functions: 45 selectors of every kind and 3,589 configurations
  with one or two settings out of range or of the wrong type give the
  same result or the same error.
- The endpoint correction calls the kernel's methods and no longer asks
  whether the kernel has them. Four `getattr(router, ..., None)` probes
  stood for a kernel without `route_primitive_centerline`,
  `route_port_corrected_centerline`,
  `realize_centerline_polygon_with_terminal_tangents` or
  `centerline_port_corrected_checked`; the kernel has all four, and
  without the third a corrected centerline counted as realizable
  unseen. `EndpointCorrectionRouter` names the methods; the fake router
  of two tests rejects a checked correction the way the kernel does,
  with an error.
- `_verify_route_obstacle_overlaps` (11 parameters) takes
  `_ObstacleSources` (the layers and the layout each is read from) and
  `_LegalOverlaps` (where a route may lie on an obstacle layer; its
  `of_layer` is the choice between the per-route regions the function
  made inline). The two tests that compare it with the older copy call
  both with the same arguments as before, through a helper
  (`ruff_PLR0913` 2 to 1; the one left is the `directional_coupler`
  cell).
- A request the meander planner cannot read is an error (rule 3.8;
  `fallbacks_never_taken` 7 to 5). The kernel's planner answers "nothing
  fits" with a status and raises only for an argument it refuses (a
  height of zero, an index out of range, a split given half); the two
  callers turned that into "no candidate" under the fallback name
  `plm.planner_error_as_no_candidate`, which no run of the 27 paper
  cells, the 13 PLM cells and the private circuits ever took. They
  raise a `RuntimeError` with the net and the planner's words now, and
  the name leaves the register (eight names). The split of an extra
  length over several runs of one edge (`plm.meander_split`) is a way
  the kernel is tested for; the test of the request vector now shows
  that it is counted.
- Four signatures over seven parameters take records (rule 4.4,
  `ruff_PLR0913` 6 to 2): `route_attempt_record_from_route` (13) takes
  the job and an `AttemptOutcome`; `build_route_debug_artifacts` (10)
  takes `RealizationRules` and `RouteRunStatistics`;
  `_find_legal_column_trunk` (9) takes `_TrunkCells` and reads the
  terminals' cells from the obstacle map itself; `_coupler_profile` (8)
  takes its two rows as one pair. Left: the `directional_coupler` cell
  (eight dimensions) and `_verify_route_obstacle_overlaps`, which a
  test compares with a verbatim older copy of itself.
- Rule 4.5, the statement limit, is in force: ruff `PLR0915` is in the
  lint set for the package (50 statements; not for the tests, the
  scripts and the benchmark modules). The package had 43 such functions
  when the rule was written down and none since `column_grid_stage` was
  split; the measure `ruff_PLR0915` leaves the ratchet. The complexity
  limit (`C901`, 40 functions) stays a ratchet measure.
- `column_grid_stage` (244 statements, one flag that made it return
  either a placement or a summary) is two functions.
  `plan_column_grid` returns a `ColumnGridPlan`: the lanes' rows, the
  order constraints, the columns, the band; nothing is placed, and
  `probe_summary()` is what the structure selection reads (it passed a
  throw-away schematic and tile before). `place_column_grid` places a
  plan: the tiles, the corners, the short links and the nets of each
  lane, step by step in a `_ColumnGridPlacer`; a tile or corner is a
  `_GridElement` (was a dict with eleven keys). What the tile
  derivation returns and adds to the schematic is byte for byte the
  same for the 17 designs with crossings, up to 128x128. No function of
  the package has more than 50 statements now.
- `_derive_crossing_tiles` (122 statements) chooses the structure per
  band and hands the placing on: `_TileDerivation.add_x_array` (the X
  array of one band, with `_split_lane` for the nets between a lane's
  tiles), `add_column_grid`, and `_dense_fanout_instances` for the
  instances that get static stubs. What the function returns and adds
  to the schematic is the same for the 17 designs with crossings (Benes
  4x4 to 128x128, plain and flat, ADEPT 8x8 to 128x128,
  `permutation_synthetic`), old against new with one hash seed
  (`PYTHONHASHSEED=0`): without a fixed seed the old code already
  placed the ADEPT designs differently from process to process (next
  entry).
- `analyze_meander_insertion_for_requirements` (92 statements): the
  selection pass is `_SelectionPass`, which decides one requirement per
  call of `select` (its request vector, the planner's choice as a
  `_Choice`, the credit of the edges the choice satisfies) and keeps
  what the final pass reads. Compared with the old function in one
  process on the 13 PLM cells and the private circuits (17 calls, 655
  report entries): the reports, without the planner's own timers, and
  the records are equal.
- `apply_crossing_aware_endpoint_correction_to_record` (86 statements,
  three nested functions) says what it does in three calls: a net
  without crossings is corrected as a whole
  (`_correct_without_crossings`, `_correct_one_end_again`), a net with
  crossings gets its two terminal pieces corrected and checked
  (`_checked_splice` over a `_CheckedCorrection`, which holds the kernel
  calls and their trace lines), and a net without an accepted result
  gets its error. Compared with the old function in one process: the
  426 calls of sixteen runs (232 of nets with crossings) return equal
  records.
- `_two_bend_static_stub_centerline_um` (79 statements, 9 parameters)
  builds the stub's points (`_two_bend_stub_points`), its end
  (`_stub_anchor_point`) and the cell of the end (`_stub_anchor_cell`);
  a stub that cannot be built is a `_StubFailure` with the reason the
  trace prints. The three forward distances are one `_StubForward`. Four
  refusals could not happen (no next cell centre in a direction that is
  never zero there) and are gone. Compared with the old function in one
  process: 376 calls of fifteen runs and 107,520 calls with shifted
  ports, angles, target rows and distances (97,000 of them refused)
  give the same result.
- `populate_realized_intersections_from_native_crossing_events` (90
  statements) reads an event (`_native_event`), finds its point in um
  (`_event_point_um`) and writes its record (`_event_record`). An event
  that is not as the kernel writes it (a missing net id, a point or a
  segment without two coordinates) raises now; it was skipped, which
  dropped a crossing without a word and never happened in a measured
  run. On twelve runs (Benes 4x4, 8x8 and 16x16 flat, ADEPT 8x8 and
  16x16, `permutation_synthetic`, two PLM designs; 534 records) the
  records and the illegal ones are equal to those of the old function,
  called on the same events in the same process.
- `extract_circuit` of the GDS extraction (133 statements) is its steps:
  the leaves, the components and the waveguide region, the groups, the
  nets of the polygons, the abutting nets, the named components. The
  ports found so far are a `_PortBook` (was a dictionary, a second one
  for the ports already seen and a nested function over both). The
  result is the same on the five private circuits (components, nets,
  skipped cells and the report compared in memory, old against new) and
  in the nine tests.
- `write_route_diagnostics` (184 statements, eight nested functions) is
  `routing/job_diagnostics.py`: one function per group of lines of the
  file, in the order the module docstring lists, over one record of the
  job's cells (`_JobCells`). The branches for a run without tracked
  cells are gone: the dispatch hands a file path only when the run has a
  debug directory. The 729 files of eight runs with
  `--attempt-diagnostics` (Benes 8x8 flat in the three configurations,
  ADEPT 8x8 in two, `heater_single` with its failing net,
  `cascade_8_compact`, `permutation_synthetic`) are the same byte for
  byte.
- Electrical router: a terminal has at least one port (owner, 2026-11-24).
  `ElectricalTerminal` refuses an empty `ports` with a `ValueError` that
  names the terminal and its heater. The extraction already refused a
  heater without electrical ports, so no terminal of a run was without
  one; three places further on still took the centre of the terminal's
  box as the landing point of the wire (`select_terminal_port_for_side`
  returning None, `_select_terminal_contact_with_port`,
  `_exit_facing_port_name`), where no metal has to lie. Those paths and
  their two fallback boxes are gone, the selected port is not optional
  any more and `ElectricalPortAccess.port_name` is a `str`.
- `_build_planner_context` of the meander planner (78 statements) hands
  two parts on: `_base_static_cells` (the static cells of the planning
  space as a set) and `_register_routes`, which registers the routes'
  cells and geometries and returns the record `_Registration`. The
  statements are moved, none changed. Ratchet: PLR0915 9 to 8, C901 50
  to 49.
- `routing/api.route_match_and_realize` (106 statements) is its three
  steps: the session, `_match_path_lengths` and `_realize`. Path-length
  matching returns the record `_Matching` (the records with their
  meanders, the three info forms, the report, the `MatchedLengths` of
  the harness) and is itself split into `_meander_planning_space` (the
  obstacle map and the clearances the meander boxes are checked against)
  and `_judge_matching` (the acceptance, which fails the run for a group
  that stays unequal and was not declared so). The statements are moved,
  none changed. Ratchet: PLR0915 10 to 9, C901 51 to 50.
- `write_routes_overview_svg` takes what `layout_overview_shapes` returns
  as one value, the named tuple `LayoutShapes` (instances and waveguide
  polygons), instead of two parameters (PLR0913 9 to 8).
- `docs/ARCHITECTURE.md` section 7.4: the module table measured again
  after the rework of 2026-11-23/24 (the package is about 48,300 lines in
  125 files, 2,200 fewer than two days before; `reporting.py` and
  `records/fallbacks.py` are listed).
- The package has no blind `except Exception` any more (rule 4.9, now in
  force as ruff `BLE001`; 35 at the first measurement, 31 on the morning
  of 2026-11-23). The last three: `load_benchmark_metadata` caught every
  error of a benchmark's `build_schematic()` and went on without the
  instances' delays, and took a `schematic_loader` nothing passed; it
  lets the benchmark's error through and the parameter is gone. The
  debug picture of a route catches `KeyError` and `TypeError`, a job the
  run stored no openings for.
- Three stage functions of the flow take records instead of long
  parameter lists (rule 4.4; PLR0913 12 to 9). `FlowRun`
  (`flow_stats.py`) is what the stages of one run report with: the
  benchmark's name, the number of steps, the counters, whether timings
  are printed. `run_electrical_routing_step` takes it (9 parameters to
  6); `verify_and_attach_photonic_reports` takes it, the `LoadedDesign`
  and `FinalCheckSettings` (10 to 6); `_preplaced_crossing_grids_stage`
  takes the `LoadedDesign` (8 to 7).
- The package reports through `logging` (rule 4.6, now in force as ruff
  `T201`; owner, 2026-11-23). The 135 `print` calls outside `cli.py` are
  `_LOGGER.info` of the module's logger; the one that wrote to standard
  error is a warning. `reporting.show_reports()`, which the command line
  calls, shows the reports as before: one line each on standard output,
  warnings on standard error, no prefix. A program that uses the package
  as a library sees nothing unless it configures `logging` or calls
  `show_reports()` itself; the tests call it once per session.
  Compared on six runs with all numbers masked: four are identical line
  for line. Two things differ and are meant to: with `--verbose-routes`
  each `Routing [i/n] ...` line is a line of its own (they were written
  without a line end and ran together), and a report is written when it
  is made, so it no longer overtakes a message another library wrote
  before it.
- The logic harness runs a benchmark through the flow's own final check
  (owner, 2026-11-23). The flow reports a new stage `checked` after
  `verify_and_attach_photonic_reports`, and `scripts/logic_check.py` ends
  a run there instead of after `realized` (the wide designs still at the
  hand-off). Until now the final check of the geometry lay outside the
  harness; a layout the flow refuses is the error `flow.run_failed` now.
- `flow._route_schematic` is the stage outline its docstring asks for
  (79 statements to under 50). What the stages before the search settle
  about crossings is `_crossing_setup`, which returns the record
  `_CrossingSetup` (the design to route, the configuration, crossings
  on or off and in which mode, the net order, the guidance nets, the
  plan for the guided search); the check that pre-placed grids stand
  alone is `_check_grids_stand_alone`; the routes overview is
  `_write_routes_overview`. No statement changed. Ratchet: PLR0915 11
  to 10, C901 52 to 51, untyped dict annotations 194 to 193.
- Ten more blind `except Exception` (rule 4.9; fallback review,
  2026-11-23). Removed where nothing can raise on the real object:
  `flow._component_bbox_um` and `layout_overview_shapes` take a
  `Component` and read it directly (a test now holds the shapes of a
  small layout). Narrowed to what the library raises: `ValueError` for
  a gdsfactory component that is not in the PDK or a call without an
  active PDK (`benchmark_metadata.py`, the crossing component),
  `OSError` and `webbrowser.Error` for opening the SVGs, `OSError`,
  `RuntimeError` and `ValueError` for showing the layout in KLayout.
  Four are left: two around building a benchmark's schematic for its
  metadata, one in the debug SVG of a route, one in the
  test-only topology analysis. Ratchet: blind excepts 14 to 4.
- Fourteen `except Exception` around kernel calls catch what the binding
  raises (rule 4.9; fallback review, 2026-11-23): `(TypeError,
  ValueError)` for the centerline calls, with `RuntimeError` for the
  checked endpoint correction and the meander planner, which report a
  rejection that way. Any other error is a defect and ends the run
  instead of turning into "no centerline", "no candidate" or a length
  measured along straight pieces. One `try` around `polyline_length_um`
  on freshly built points is removed; nothing in it can raise. In the
  measured runs none of the fifteen ever caught anything. Ratchet: blind
  excepts 29 to 14.
- One reading of the legal crossings for both verifications (owner,
  2026-11-23: one of the two near copies, the one that makes more sense).
  `legal_crossing_overlap_polygons` and
  `legal_crossing_component_footprints` in
  `crossings/route_rust_crossing_components.py` take the fields of the
  crossing plan; the check inside the router and the final check of the
  flow call them. The copy in `flow_verification.py` is gone. The final
  check now skips a footprint polygon with a point that is not finite,
  as the check inside the router did. `tests/test_legal_crossing_polygons.py`
  holds the reading.
- `path_length_requirements.build_requirement_delay_candidates` within
  the limit of 50 statements, no behaviour change: its five nested
  functions are module functions (`_edge_tuple_identity`,
  `_dedupe_edge_tuple`, `_candidate_sort_key`,
  `_complete_common_mode_source`) and the dataclass
  `_CandidateCollector`, which keeps a requirement's candidates without
  repeats.
- `bundle_detail_router.route_detailed_bundles` (87 statements, two
  closures over seven accumulators) is an outline of five steps now:
  `_pad_wire_blockers`, `_collect_bundle_entries`,
  `_commit_placed_wires`, `_route_fallback_wires` and
  `_fail_terminals_outside_every_bundle`, with what is placed and
  refused held by the dataclass `_DetailProgress` (`commit`, `fail`).
  No behaviour change: on the electrical suite the metal of every GDS
  layer is identical before and after, as are the 8 debug SVGs, and the
  suite's baseline comparison passes.
- `docs/ARCHITECTURE.md`, section 7.2: four line counts of the Rust
  table measured again (`static_obstacle_builder.rs` 1,740,
  `engine/crossing_reservation.rs` 3,130, `engine/negotiation/` 4,120,
  `meander/` 4,880 with `plan.rs` 2,220).
- `docs/ARCHITECTURE.md`, section 7.4: the module table states the
  measured file and line counts again (the package has about 50,500
  lines, not 45,700) and names `schematic_protocols.py` and
  `records/route_keys.py`, which it did not list.
- Two more functions of the routing core within the limit of 50
  statements, no behaviour change: `dispatch.dispatch_native_routing`
  hands the per-job debug tables, the recording of the attempts and the
  failure of an unrouted net to `_batch_job_debug_tables`,
  `_record_batch_attempts` and `_raise_for_unrouted_net`; the fan-out
  stub endpoint correction of `finalize.py` hands its job packing, the
  merge failure text and the kernel's length with its fallback to
  `_fanout_stub_correction_jobs`, `_stub_merge_failure_message` and
  `_router_centerline_length_um`.
- `route_jobs._build_static_fanout_anchors` within the limit of 50
  statements, no behaviour change: the lower and the upper half of a
  dense port row were two mirrored copies of the same staggering; both
  call `_stagger_fanout_half` now, with the side as a sign.
- Two diagnostic functions within the limit of 50 statements, no
  behaviour change: `electrical_debug_svg` draws the pad plan, the bus,
  the pad wires and the topology in four functions (the 8 debug SVGs
  of the electrical suite are byte-identical before and after, and
  identical to those of the SVG code before the style dataclasses of
  this release), and `route_attempt_diagnostics` hands the raw dynamic
  cell counts to `_raw_dynamic_consistency`.
- Two more functions within the limit of 50 statements, no behaviour
  change: `RouteTimingBucket.record_route` adds the 61 summed
  statistics, the 2 maxima and the 7 per-class counter dicts by name
  from three tables instead of one statement each (compared field by
  field with the previous implementation on the same routes; new
  `tests/test_route_timing_bucket.py`), and
  `place_realized_crossing_components` hands the placing of one
  reference, the owners of a shared crossing and the reading of the
  realized records to three functions.
- Two more functions of the crossing code within the limit of 50
  statements, no behaviour change:
  `route_rust_crossing_plan.build_crossing_plan_info` reads the request
  where it copied six of its fields into locals, and
  `_plan_event_records` builds the event records (typed as the
  TypedDict `_PlanEventRecord`, which removes two casts);
  `verify_repair._repair_final_illegal_crossings` hands the re-route
  order of a not-perpendicular crossing to
  `_not_perpendicular_repair_priority` and the rip-up and re-route to
  `_reroute_for_crossing_repair`.
- Three functions of the routing core within the limit of 50
  statements, no behaviour change:
  `photonic_verification.verify_photonic_routing` reads its options
  from the three option objects where it used to copy 16 of their
  fields into locals, and `_crossing_clip_plan_info` builds the clip
  record; `router_setup.configure_router_and_grid` hands the A*
  settings to `_tune_search`;
  `finalize._apply_checked_endpoint_corrections_for_net_ids` and its
  fan-out stub variant store a failed correction through one function,
  `_fail_endpoint_correction` (four copies before; the stub variant is
  shorter but still over the limit).
- Another three functions within the limit of 50 statements, no
  behaviour change: `column_trunks.route_column_trunks`
  (`_commit_trunk_members`),
  `plan_requirement_requests_registered` of the meander planner
  (`_registered_candidate_inputs`) and
  `finalize._record_terminal_bump_distance_check_candidates`
  (`_bump_room`, `_print_terminal_bump_checks`, with
  `tests/test_terminal_bump_room.py`).
- Three more functions within the limit of 50 statements, no behaviour
  change: `route_overview_svg.write_routes_overview_svg` hands the
  instance boxes and the waveguides to `_instance_svg` and
  `_waveguide_svg`; `flow_stats.populate_route_stats` copies the 31 search
  counters and the 9 search times by name from two tables
  (`_SUMMARY_COUNTERS`, `_SUMMARY_TIMES`) instead of one assignment
  each, and `dump_photonic_probe_failure_artifacts` hands the issue
  scan and the per-net centerlines to two functions (the per-net entry
  is the TypedDict `_NetCenterlines`). The last two got a test:
  every listed counter and time is a field on both sides and arrives
  (`tests/test_flow_summary_line.py`), and the probe failure files hold
  the centerlines of every net an issue names
  (`tests/test_probe_failure_artifacts.py`, the first test of that
  writer).
- Seven functions within the limit of 50 statements (rule 4.5), each by
  moving a self-contained part into a function of its own, no behaviour
  change: `cli._build_arg_parser` (six argument groups; the `--help`
  text is byte-identical), `path_length_graph._propagate_roles`
  (`_component_sides`, `_port_neighbours`),
  `gds_circuit_rebuild.generic_schematic` (`_nets_in_order`),
  `diagnostics._corridor_clearance_diagnostic` (`_cells_inflated_by`,
  `_cells_reachable_from`), `handoff.states_and_openings`
  (`_endpoint_grid_state`, one implementation for source and target),
  `crossing_plan_stage._instance_static_geometry_open_cells`
  (`_static_columns_in_row`) and
  `route_jobs._dense_source_port_runway_lengths` (one function per
  fan-out access mode).
- No module imports a private name of another (rule 3.5);
  `private_imports` falls from 7 to 0. The five functions that other
  modules used are public now, each with a docstring:
  `static_obstacle_builder.load_rust_backend` (was `_load_rust_backend`,
  also under that name in the modules that re-export it),
  `route_rust_debug_artifacts.cells_bbox`,
  `route_rust_crossing_plan.index_records_by_net_id` (was
  `_routed_records_by_net_id`),
  `route_rust_endpoint_correction.polyline_length_um` (was
  `_centerline_length_um`) and `electrical.port_access.all_port_accesses`.
  The last one replaces two copies (`electrical/metrics.py` and
  `electrical/svg_support.py`, which is gone).
- Four more signatures within the limit of 7 parameters, no behaviour
  change; `ruff_PLR0913` falls from 16 to 12:
  `photonic_verification._verify_one_port_connection` reads the port
  centre and the endpoint from the record by its `role`;
  `route_rust_meanders._planned_record` takes the meander search
  config instead of three of its fields;
  `dump_photonic_probe_failure_artifacts` lost `allow_unchecked_bumps`
  (its one caller passed True); `select_layer_structure` lost `allowed`
  (never passed, both structures are always evaluated).
- No parameter that the function does not read: 73 functions, most of
  them in `photonic_router/routing/`, lost a `settings` or `state`
  parameter they only passed on or never touched (left over from the
  time these functions were methods of the session), and every call
  lost the argument. No behaviour change; `ruff_PLR0913` falls from 18
  to 16. Three functions keep an unread parameter because their
  signature is fixed from outside: `port_roles.no_port_roles` and
  `gds_circuit_rebuild._stub_settings` are callbacks, and the tests
  replace `route_jobs.port_access_rule_for` by a stub that reads `state`.
- Shorter signatures in the electrical router (rule 4.4, at most 7
  parameters), no behaviour change; `ruff_PLR0913` falls from 30 to 18:
  - the terminal route helpers of `electrical/metal_realization.py` and
    `electrical/net_geometry.py` take the `ElectricalRoutingConfig`
    instead of the wire and contact widths, and one `trim_route_tail`
    flag replaces the two trim flags that were always set together;
  - `river_routing.PadWireBlockers` holds the four cell sets a pad wire
    is checked against, and `PadWireBlockers.for_member` is the one place
    that takes a wire's own terminal and pad out of them (the channel
    router and the full-grid fallback each had a copy);
  - the `commit` step of `bundle_detail_router` takes the bundle member
    instead of its five parts;
  - `_ShapeStyle` (`metal_snapshot_svg.py`) and `_LineStyle`
    (`debug_grid_svg.py`) carry the drawing style of the SVG helpers;
  - `pad_slots._make_slot` lost the `center_y_um` and `side` parameters
    no caller passed and takes the pad size as one `size_um` pair;
    `individual_topology._best_source_path_to_boundary` reads the grid
    size and the terminal's open cells from the obstacle map;
    `column_trunks._find_legal_column_trunk` derives the forbidden cells
    itself (8 parameters, still counted).
- One implementation where there were copies (rule "one implementation
  per concern"), no behaviour change:
  - `photonic_router/schematic_protocols.py` holds `SchematicLike` (read
    for the netlist) and `PlacedSchematicLike` (netlist and placements);
    the four Protocols named `SchematicLike` in `benchmark_metadata.py`,
    `graph_analysis.py`, `plm/path_length_graph.py` and
    `crossings/topology_analysis.py` are gone.
  - `electrical/wire_geometry.manhattan` is the one Manhattan distance of
    the electrical router; the private copies in `bus_search.py` and
    `port_access.py` are gone.
  - The two functions named `terminal_open_cells` read different maps and
    say so in their names: `bus_search.common_bus_terminal_open_cells`
    and `wire_geometry.pad_wire_terminal_open_cells`.

- Contribution 2 (`--configuration contribution2`,
  `--preplaced-crossing-grids true`) on a design that is not band-layered
  no longer ends with "pre-placed crossing grids need a band-layered
  design": the flow routes the design with the standard router, exactly as
  `--configuration baseline` does, and logs one warning that names how
  many nets are outside the band model and why (owner decision,
  2026-11-22). This concerns 15 public designs (the 13
  path-length-matching benchmarks, `clements_8x8`, `heater_single`); a
  band-layered design runs as before. `PreplacedCrossingGridOptions`
  gets `standard_router_crossings`, which `cli.main` sets from the
  benchmark's stable block. The logic harness reports the note
  `flow.standard_router_fallback` in place of `flow.design_refused`; the
  pin of the 15 contribution-2 runs moves from the refusal to the
  baseline's findings plus the note.

- The obstacle map's `build_stats` count every port and every port-open
  cell once. With heater obstacles the map is merged from two layer groups
  that both see all ports, and the merge added their counts up, so
  `port_count` and `port_open_cell_count` were twice the map's; the flow
  statistic `port_open_cells` halves accordingly (reports only; nothing
  routes from it). Found by the logic harness; its pin drops from three
  `obstacle_map.stats_disagree` per benchmark to one (`blocked_cell_count`,
  open). Test baseline python 827 -> 828.
- Rule 4.7 is in force: every public class, method and function of the
  package, the benchmarks and the two experiment tables has a docstring
  (268 added; ruff `D101`, `D102`, `D103` join the lint set and leave the
  ratchet, which holds 14 counts). Docstrings only: the syntax tree of
  every touched file is the same before and after once docstrings are
  removed.
- Two more long signatures take records (rule 4.4, 32 -> 30).
  `apply_crossing_aware_endpoint_correction_to_record` (16 parameters) takes
  `setup: EndpointCorrectionSetup` (what a run's corrections share),
  `scope: EndpointCorrectionScope` (which ends, which baseline) and
  `clearance: EndpointClearance` (opened and exempt cells, radii).
  `realize_routed_net_records` (10) takes `geometry: RealizationGeometry`.
  No value changes.
- Every Rust exception is an `#[expect(..., reason = "...")]`; no `#[allow]`
  is left in the two crates, and the workspace denies
  `clippy::allow_attributes` and `clippy::allow_attributes_without_reason`.
  The compiler reported five `too_many_arguments` exceptions as no longer
  needed; they are gone (`rust_exceptions` 39 -> 34). Three `dead_code`
  exceptions hold outside the tests only and say so.
- `build_crossing_plan_info` (13 parameters) takes the crossing settings of
  the run as one `CrossingPlanRequest` (rule 4.4, 33 -> 32). No value changes.
- Six routing signatures shrink (rule 4.4, 39 -> 33). `StateOpenings`, the
  per-net product of `handoff.states_and_openings`, is a named tuple instead
  of a bare five-tuple; `write_route_diagnostics` and `write_failed_log` take
  it whole. `write_route_diagnostics` derives the status, the route cells
  and the clearance-exempt cells from its inputs (both callers passed
  exactly these) and loses `repair_note`, which no caller passed, as
  `_finalize_committed_route` loses `debug_obstacle_cells`.
  `port_to_grid_state` reads the grid origin and size from the state, which
  all six callers passed from there. The two heading helpers of the
  diagnostics take grid states as tuples. No value changes.
- Seven electrical signatures take records (rule 4.4, 46 -> 39). The debug
  stage (`write_debug_artifacts`, its protocol `DebugArtifactWriter`, and the
  two grid SVG functions) reads the `ElectricalRoutingResult` of stages 1 to
  11 instead of nine loose products; `route_electrical_heaters` builds the
  result first and adds the artifact paths to it. The three common bus phases
  (`route_column_trunks`, `_route_local_trunks`, `_route_lone_straight_drops`)
  share one `CommonBusProgress` (remaining heaters, selected and unselected
  terminals, routes, tree cells, blocked cells), updated in place as before.
  No value changes.
- Two long signatures take records instead of loose keywords (rule 4.4).
  `verify_photonic_routing` (22 parameters) takes `geometry:
  RealizedRouteGeometry` (grid, width, layers, turn rules, the die),
  `crossings: LegalCrossings` and `checks: VerificationChecks`.
  `analyze_meander_insertion_for_requirements` (13) and the two planner
  functions behind it take `space: MeanderPlanningSpace` (grid, turn rules,
  blocked cells, clearances). No value changes; the final meander pass
  computes the minimum insertable length from the space instead of
  receiving it.

- Every function of the package is annotated (S4, slice 2; the annotation
  half of rule 4.2 is in force). The last 13 functions without a parameter
  or return annotation have one, and ruff's `ANN001`, `ANN201` and `ANN202`
  are in the lint set for the package (tests and scripts are exempt). The
  three measures leave the quality ratchet, which holds 20 counts. mypy's
  error count, the other half of the rule, stays in the ratchet at 277.
- The helpers of the routing phases declare what they take (S4, slice 1).
  214 parameters named `settings` and `state` in seven modules of
  `routing/` are annotated `SessionSettings` and `SessionState`, so mypy
  checks those function bodies for the first time. That surfaced 16 errors;
  they are fixed where they came from: `RunPlan.realization_grid_spec` and
  `RunPlan.repair_config` are no longer optional (the empty plan holds a
  grid of no cells and the default configuration), two loops in
  `finalize.py` no longer rebind a non-optional name to an optional lookup,
  and the debug export narrows its directory before using it. Parameters
  without an annotation fall from 218 to 4, mypy's errors from 285 to 277.
  Behaviour is unchanged.
- Helpers that existed more than once have one implementation or a name of
  their own (S2, slice 5; rule 3.3). The three private copies of
  `in_bounds` in `electrical/` use the one of `wire_geometry.py`; the two
  `_port_center_um` are one `records.route_rust_records.center_of_port_um`
  (it accepts any iterable of two numbers, as the copy in `crossings/` did);
  `gds.route_rust_debug_artifacts.ensure_dir` is public and the wrapper in
  the crossing plan says why it imports late; the crossing report's own
  `_as_float`/`_as_int`, which behave differently, are `_finite_float` and
  `_int_or_none`, and the shared ones are `as_float`, `as_int`; the two
  session wrappers in `route_jobs.py` are `cell_in_grid` and
  `state_cell_center_um`. Imports of private names from another module
  fall from 27 to 7. Left: `_load_rust_backend` (five bindings of one
  function, which tests replace per module), `_all_port_accesses` (two
  versions that sort differently) and three names whose public spelling is
  taken.
- No module of `records/` imports from a stage package any more (rule 3.4
  of the coding standards is in force for Python). `PortRef` and
  `RoutedEdgeKey` move from `plm/path_length_graph.py` to the new
  `records/route_keys.py`, and `routing/crossing_plan_info.py` moves to
  `records/crossing_plan_info.py`; both were records that lower layers
  needed. **API change**: import them from the new modules (23 files of
  the package and the tests are updated). `upward_imports` is pinned at 0.
- No `unwrap()` in library code of the Rust workspace (rule 5.3 of the coding
  standards is in force). The two calls outside tests are gone: the one in
  the search's diagnostics dump is an index, the one in the bindings'
  result conversion states its invariant with `expect`; `clippy::unwrap_used` is in the
  workspace lints and `clippy.toml` exempts the tests, so the measure leaves
  the quality ratchet (22 counts). The table of section 8 of the standards
  shows the counts after S1 and S2 next to the first measurement.
- Docstrings for the 14 modules whose helpers became public in S2 slice 3:
  94 public functions, methods and classes say what they are for and what
  they assume (units, the degenerate case, what is raised). Docstrings
  only; an AST comparison that ignores docstrings shows no other change.
- 88 helpers that other modules import lose their leading underscore (S2,
  slice 3; rule 3.5: a name starting with an underscore stays in its
  module). `_grid_cell_center_um`-style names that several modules used
  were public in effect; they are now public in name, e.g.
  `records.route_rust_geometry.physical_point_to_grid_cell`. **API change**
  for whoever imported one of them. Imports of private names from another
  module fall from 143 to 27. The 27 are eleven names the rename left
  alone: eight are defined in more than one module (`_in_bounds` and
  `_load_rust_backend` five times each, `_as_float`, `_as_int`,
  `_ensure_dir`, `_port_center_um`, `_grid_cell_center_um`,
  `_all_port_accesses` twice), which is a finding of its own under "one
  implementation per concern", and three would collide with an existing
  public name. The rename makes 73 functions and one class visible to the
  docstring rule that it did not count while they were private: the pinned
  counts of `ruff_D103` and `ruff_D101` rise from 75 to 147 and from 56 to
  57 with this change, as debt that was there and is now measured.
- The module `photonic_router.routing.route_rust` is gone (S2, slice 2). It
  was a shim of 430 lines that re-exported 180 names of the routing package
  and of five other packages under the path the routing code had before it
  was split, 104 of them private. **API change**: import a name from the
  module that defines it (`route_match_and_realize` from
  `photonic_router.routing.api`, `RipupRerouteConfig` from
  `photonic_router.records.route_rust_types`, and so on). The 22 files that
  used the shim (7 of the package, 14 tests, one script) are rewritten.
  Imports of private names from another module fall from 247 to 143.
- Imports are absolute everywhere (S2, slice 1; rule 4.10 of the coding
  standards is in force). The 96 relative import statements of the package,
  nearly all in `electrical/` and `gds/`, and those of the tests are
  rewritten (`from .types import X` is `from photonic_router.electrical.types
  import X`). ruff's `TID252` with `ban-relative-imports = "all"` is in the
  lint set, so the measure `relative_imports` leaves the quality ratchet,
  which holds 23 counts again.
- `run_routing_flow` is gone (S1, slice 7). It was the compatibility wrapper
  that turned 50 keywords into the `FlowOptions` of `route_benchmark`, kept
  for 25 calls in tests and two scripts. **API change**: call
  `route_benchmark(config, options)`. For a caller that sets a few options,
  `FlowOptions.from_names(benchmark_name=..., debug_timing=True)` puts each
  leaf name into the group that declares it and refuses an unknown name.
  Two differences to the wrapper for whoever ports a call: the
  configuration is an argument (`RoutingConfig.from_environment()` is what
  the wrapper used when none was given), and a `stats` record no longer
  implies `collect_route_stats`. `scripts/benchmark_photonic.py` and
  `scripts/crossing_benchmark_report.py` are ported; the tests use
  `tests/fixtures/flows.run_flow`.
- The session state names what is still mutable (S1, slice 6; a
  restructuring). The seven fields the search writes while it runs (route
  bookkeeping, attempt records, repair trace, timing buckets, three
  counters) and the list of route SVGs are one record, `SearchLog`
  (`state.search`), which phase 5 starts fresh; phase 8's three probe
  fields are its own `ProbeCache` (`state.probe`). The detour pass returns
  its re-route count in `RoutedRecords` instead of writing it to the state.
  Phase 9 takes the products of phases 7 and 8 (`routed`, `verified`)
  instead of five values picked out of them and of phase 1's, and phase 5
  reads the obstacle map from phase 1's product instead of a parameter. The
  state has 14 fields (24 before; 82 at the start of S1). Behaviour is
  unchanged.
- Routing phase 5 runs in steps with two frozen results (S1, slice 5; a
  restructuring). `finalize_route_jobs_and_static_handoff` (230 lines) is
  now `_in_routing_order`, `_hand_static_cells_to_kernel`,
  `_jobs_of_this_run` and `_precompute_openings`. Thirteen fields of the
  shared state become `StaticHandoffCells` (`state.static_cells`) and
  `RunPlan` (`state.run`) of `routing/products.py`; a fourteenth,
  `chip_boundary_static_rects`, was written and never read and is gone. The
  state has 24 fields (36 before). Behaviour is unchanged.
- Routing phase 4 runs in five steps with one frozen result (S1, slice 4; a
  restructuring). `build_crossing_plan_and_port_footprints` was one function
  of 330 lines. It is now `_build_crossing_plan`, `_build_port_access`,
  `_build_foreign_keepouts`, `_build_dense_lateral_windows` and
  `_assign_lane_offsets`; the nine port tables they produce are one
  `PortFootprints` (`routing/products.py`, `state.ports`) instead of nine
  fields of the shared state. The three row indexes of the raw static
  geometry are cached properties of phase 2's `KernelSetup` instead of three
  lazily filled state fields. The phase loses two parameters it did not use
  (`obstacle_map`) or can read from phase 1's product
  (`crossing_device_info`), and a per-instance map of non-static keep-out
  cells that was computed and never read is gone. The state has 36 fields
  (47 before). Behaviour is unchanged.
- Routing phase 3 runs in three steps with frozen results (S1, slice 3; a
  restructuring). `build_route_jobs_and_fanout_clustering` was one function
  of 190 lines that filled 19 fields of the shared state as it went. It is
  now `_build_jobs_and_endpoint_index`, `_build_fanout_stubs` and
  `_build_dense_runways`, each returning one record of
  `routing/products.py` (`EndpointIndex`, `FanoutStubs`, `DenseRunways`)
  that the phase stores in its slot of the state (`state.endpoints`,
  `state.fanout`, `state.runways`). The fields inside lose their repeated
  prefixes (`state.fanout.anchor_by_port_spec` was
  `state.fanout_anchor_by_port_spec`). The four port-access tables are no
  longer re-initialised by phase 3; phase 4 is their only writer. The state
  has 47 fields (59 before). Behaviour is unchanged.
- Routing phases 1 and 2 return their results as values (S1, slice 2; a
  restructuring). `build_static_obstacle_context` and
  `configure_router_and_grid` no longer take the session state: they return
  the frozen `ObstacleContext` and `KernelSetup` of the new
  `python/photonic_router/routing/products.py`, and the session builds the
  state from them (`_RouteNetsRustSession.prepare`,
  `SessionState.after_setup`). The 25 fields the two phases used to write
  into the shared state are gone from it; the later phases read
  `state.obstacles.grid`, `state.kernel.router`,
  `state.kernel.radii.bend_radius_cells` and so on. The state has 59 fields
  (82 before) and four of them are required, so a state without a grid or a
  router cannot be built; mypy's errors fall from 346 to 320 because the
  grid and the layout are no longer optional. Behaviour is unchanged.
- The options of a routing session are one typed argument (S1, slice 1 of
  the data flow between the stages; a restructuring). New
  `python/photonic_router/routing/options.py`: `SessionOptions` with the
  groups `SearchOptions`, `CrossingOptions`, `PortAccessOptions`,
  `DebugOptions` and `RealizationOptions`, plus `MatchingOptions` and
  `CircuitMetadata`. They replace the keyword lists of
  `route_nets_rust`, `_RouteNetsRustSession`, `static_fanout_anchors_um`
  (41 keywords each), `route_match_and_realize` (49) and
  `SessionSettings.from_arguments` (41, now `from_options`), and the 40
  flat fields of `OpticalRoutingStageConfig`. The flow builds them in
  `flow_optical.build_optical_routing_stage_config(options, config,
  inputs)` from its `FlowOptions` and an `OpticalStageInputs` (what the
  earlier stages decided); the builder moved there from `flow_config.py`.
  The pre-placed crossing grids take the probe's configuration as
  `router_probe_options: SessionOptions` instead of the untyped
  `router_probe_kwargs` dictionary. **API change:** callers of these
  functions pass the records; `enable_grid_endpoint_correction` of
  `route_match_and_realize` is `realization.enable_checked_endpoint_correction`.
  The tests use `tests/fixtures/sessions.py::session_options(**flat)`.
  `run_routing_flow` keeps its keyword list for now.
- Algorithm pass measurements (2026-11-09, records only, no code in the
  main tree): `experiments/pattern-pass/` closes research item 8 on the
  attempt anatomy of the archived table (the searches a pattern route could
  replace hold 3.6 % of the search time); `experiments/heuristic-tightness/`
  profiles the kernel's f-values (branch `exp/f-profile`): the heavy
  searches have no tie plateau, their heuristic at the start sees about
  half of the final cost because the crossing price is invisible to it, and
  an obstacle- and crossing-aware 2D Dijkstra bound reaches 0.86 to 0.91 of
  the final cost; research item 2 becomes a proposed item with that
  measurement. `experiments/weighted-astar/` closes research item 6: two
  27-cell tables at heuristic weights 1.05 and 1.10 (branch
  `exp/weighted-astar`) fail four paper cells at either weight (ADEPT
  16x16 lidar-pure verifier errors, ADEPT 64x64 lidar-pure and
  contribution 2 and ADEPT 128x128 contribution 2 without convergence),
  move lengths by up to 0.7 % and run at total wall 0.94 and 1.13; not
  adopted, the main tree carries no weight.
- Negotiation budget schedule (algorithm pass, item 1, 2026-11-09): a fresh
  net's first plain attempt runs at `budget_first_retry` (10 M) right away,
  and the retry at that budget runs only when the first attempt failed with
  every kernel search within `budget_first` (2 M iterations:
  `Router::attempt_max_heap_pops`, `budget::retry_after_failed_first`). With
  window 0 only the retry was the identical search call with a larger
  iteration cap, so a first attempt that exhausted 2 M iterations only had
  them recomputed by the retry; the anatomy of the pinned table
  (`experiments/negotiation-budget-schedule/`) counted 147 such attempts and
  485 s of 3,210 s of search time across the 27 cells (ADEPT 64x64
  lidar-pure 185 of 1,462 s, Benes 32x32 lidar-pure 106 of 432 s). The
  retry still runs after an instant commit rejection of the straight probe
  route or an exhausted open set, the cases in which the two-step sequence
  ran a search the first attempt had not (speed pass B1 skipped it and
  moved six cells). Routes and failures are identical to the two-step
  sequence; `search_attempts` / `search_failures` in the reports drop by the
  skipped retries. Nine-cell gate and PLM gate exact; 27-cell table exact
  against the pins, total wall 0.89 of a reference table run concurrently on
  the same server (Benes 32x32 lidar-pure 0.78, ADEPT 64x64 contribution 1
  0.84, ADEPT 64x64 lidar-pure 0.90), time in failed attempts 807 s to
  325 s over the table.
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

- Interfaces nothing in the package used (owner, 2026-11-24). The stage
  Protocols of `routing/stages.py` and `electrical/stages.py` typed no
  parameter and no variable of the package; only a test assigned each
  phase function to its Protocol. The five phase products that lived
  next to them are in `routing/products.py` now. `graph_analysis.py`
  (`GraphAnalysisContext`, exported by the package) and
  `build_stage_schedule_for_benchmark` were called by tests only; the
  tests of the crossing structures read the schedule from
  `build_crossing_demand(...).stage_schedule()`, as the flow does.
- Seven functions only tests called (rule 4.12, `names_only_tests_use`
  11 to 4): `CrossingPlan.events_for_edge` and `.event_for_pair` (the
  test reads `events_by_edge` and `events_by_pair`, as the code does),
  `RouteBookkeeping.restore_records` and `.routed_edge_lengths`,
  `StaticObstacleMapData.rust_static_cells`,
  `terminal_contact_seed_points` and `select_terminal_contact`. One test
  that tested only `select_terminal_contact` goes with it (python 992 to
  991). The four names left are the two port-role resolvers a caller of
  the path-length graph may pass, the `DetourPass` protocol and
  `GraphAnalysisContext.from_schematic` (open with the owner). Also gone:
  `write_crossing_verification_report`, a second writer of the crossing
  report next to the one the flow uses, called by one test only.
- Four fallbacks no run ever took, replaced by an error that names the
  net (owner, 2026-11-23). Counted with the fallback register over every
  public benchmark below 16 x 16 in three configurations, the 13 PLM
  cells, the five private circuits and all 27 cells of the paper (the
  64 x 64 and 128 x 128 designs included): zero times each.
  - A record without a corrected centerline was given an unchecked
    endpoint correction at realization, or drawn as its grid route
    (`realization.unchecked_port_centerline`,
    `realization.grid_route_polygon`): geometry that does not end on the
    ports, without a word; `finalize.py` names the first as a past source
    of wrong layouts. `realize_routed_net_records` and
    `realized_record_polygons` now raise "net ... reached realization
    without a corrected centerline" with the endpoint correction's own
    error when there is one, and lose `enable_endpoint_correction`.
  - Such a record was left out of the meander planner's registration and
    stood as an obstacle, its requirement left to fail later
    (`plm.unregistered_route`). `_build_planner_context` raises the same
    way; the bookkeeping for unregistered routes is gone.
  - A centerline the kernel refused to measure (fewer than two points, a
    point that is not finite, a piece of length zero) got the plain sum
    of its pieces instead (`endpoint.length_along_straight_pieces`). The
    sum is the same number; what the fallback hid is the broken
    centerline. `kernel_centerline_length_um` asks the kernel and lets
    its refusal through.
  The register has nine names, seven of them not taken by any pinned
  benchmark or test (`fallbacks_never_taken` 11 to 7). Twelve tests of
  the meander planner built records without a centerline and let the
  fake router supply one; their records carry it now, and one test
  holds the refusal.
- Two modules of the package that only tests used (owner, 2026-11-23).
  `primitive_library.py` mapped the kernel's primitive ids to gdsfactory
  components and nothing asked it; it is removed with its test (python
  986 to 979). `crossings/topology_analysis.py`, the older depth/rank
  plan that the tests keep as the independent oracle of the crossing
  demand, is `tests/fixtures/topology_analysis.py` now: it is test code
  and lives with the tests. Ratchet: names only tests use 16 to 11,
  blind excepts 4 to 3.
- Two switches nothing read (fallback review, 2026-11-23):
  `PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING` and
  `PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING`, with
  `CrossingEngineConfig.enable_guided_collision_crossing` and
  `.disable_guided_collision_crossing` on both sides of the binding. They
  travelled from the environment into the kernel's configuration, and no
  line of the kernel asked for them. `docs/CONFIGURATION.md` and the stub
  are regenerated.
- The length checks on the result of `_derive_crossing_tiles`
  (`preplaced_crossing_grids.py`): the function has one `return` with
  eleven values, so "fewer than nine, ten, eleven" never held. The result
  is the named tuple `_CrossingTiles` now, with a type per field instead
  of a bare `tuple`.
- The internal photonic probe verification (owner, 2026-11-23, after
  the fallback review showed that nothing could turn it on).
  `SessionOptions.enable_internal_photonic_probe_verification` had no
  command-line flag, no environment variable and no benchmark that set
  it, and 105 archived result runs report zero photonic repair
  attempts. Gone with it: the photonic repair loop and the probe layout
  of `routing/verify_repair.py`, `verify_realized_route_intersections`
  (the Python search for intersections that looked up the kernel's
  verdict for each), `dump_photonic_probe_failure_artifacts`,
  `ProbeCache` (session state 14 to 13 fields), the plan fields
  `final_photonic_repair_attempts` and
  `photonic_probe_failure_artifacts`, the report metrics
  `final_photonic_repair_attempt_count` and `..._success_count`, 29
  helpers nothing else used, and the 30 tests of all of it (python 1003
  to 973). About 2,600 lines. What still checks the geometry: the
  kernel's crossing verdicts with the final crossing repair loop, which
  stays; `verify_photonic_routing` in the flow before the GDS is
  written; and the logic harness (`routing.nets_meet_without_crossing`,
  `realized.illegal_crossing_left`, `realized.nets_overlap`).
- Silent probes for what the kernel and the obstacle map always have
  (fallback review, 2026-11-23). The package runs with the kernel it is
  built with, so `hasattr(router, ...)` before `set_route_width_um`,
  `set_long_straight_exempt_net_ids`, `crossing_events`,
  `crossing_verdicts`, `add_static_cells`, `ripup_route`, the three
  `raw_dynamic_*` readers and `export_debug_svg_with_obstacle_cells`
  only hid a stale build: the setting was skipped without a word. The
  calls are direct now; a kernel without the method fails with its
  name. The same for the fields of `StaticObstacleMapData`
  (`raw_blocked_cells`, `raw_static_rects`, `blocked_static_rects`,
  `chip_boundary_rects`), of `AStarConfig` and for `GridSpec.origin`.
  The checks that raise "rebuild with maturin" stay. Two
  `except Exception` around `router.crossing_events()` are gone
  (`finalize.py`, `verify_repair.py`): an error there read as "no
  crossings", which `finalize.py` already called wrong at a third
  place. In no measured run (123 harness runs, 82 full flows, the
  electrical suite) had either caught anything. Ratchet: blind excepts
  31 to 29.
- Branches of the kernel that no call can reach (fallback review,
  2026-11-23; each shown by reading every caller, and the routed results
  are unchanged). In `engine/search_calls.rs`: `ordinary_collision_fallback`
  and the fall-through after a failed crossing validation (a crossing
  attempt exists only where crossings are open to the call), the check
  for unreserved crossings in a "fallback route" (an upfront crossing
  attempt belongs to a repair call, the check to a plain one), and the
  cloned obstacle map for clearance exemptions at block radius 0 (the
  exemptions are set only at a positive radius). The static-only probe
  takes the opened cell keys its one caller always passed, without the
  two substitutes. In `geometry_realization.rs` the two 45-degree
  endpoint corrections lose `allow_unchecked_bumps`, which both callers
  passed as `false`, and with it two parameters and two
  `too_many_arguments` exceptions (ratchet `rust_exceptions` 34 to 32).
- `gds/route_gds.route_nets_gds` and `print_instance_ports`: no caller
  in the package, the scripts or the tests. `route_nets_gds` routed
  with gdsfactory's `route_single`, and its `on_error="error"` mode did
  not raise (the raise was commented out). The module keeps
  `get_port_from_instance`. `ruff_T201` falls from 157 to 141 and
  `ruff_BLE001` from 32 to 31.
- Fourteen more functions no line of the repository names besides
  their definition: `crossing_structures.octile_extra_length_um`,
  `individual_topology._terminal_access_anchor_cell`,
  `gds_circuit_extraction._pair_pitch` and `_pair_rows`,
  `gds_circuit_rebuild._mmi_ports` and `_heater_ports`,
  `route_rust_debug_artifacts._format_cells_preview`,
  `route_rust_geometry._crossing_footprint_blockers` and
  `_same_undirected_segment`, `RouteBookkeeping.set_committed_cells`,
  `StaticObstacleMapData.rust_open_cells`, and in
  `route_rust_endpoint_correction.py`
  `_apply_endpoint_corrections_to_debug_artifacts`,
  `_build_realization_router` and `_load_rust_backend`. A commented-out
  call in `layout_from_schematic.py` went with them.
- `electrical/pitch_grid.cells_bbox` and
  `PhotonicVerificationResult.raise_for_errors`: no caller.
- `MeanderInsertionConfig.enabled`, `max_extra_length_per_region_um` and
  `conservative_legal_check` (owner, 2026-11-22): nothing read them, the
  run only copied them into `analysis_info["meander_config"]`, which loses
  the three keys.
- The parameter `config` of
  `electrical/wire_geometry.wire_reservation_cells_from_point_path`
  (not read) and a comparison in `plm/path_length_requirements.py` that
  could not be true after the deduplication before it.

- The speed-pass B1 switch `NegotiationConfig.first_attempt_full_budget` /
  `PHOTONIC_ROUTER_NEGOTIATED_FIRST_ATTEMPT_FULL_BUDGET`, its binding field,
  overlay entry and ablation variant: not adopted on the 27-cell table
  (owner, 2026-11-05), moved to
  `experiments/negotiation-first-attempt-full-budget/` as a patch against
  `45c9aa2`. The negotiation loop is the pre-`bc2c0d8` loop again; `_rust.pyi`
  and `docs/CONFIGURATION.md` regenerated; nine-cell gate and PLM gate exact.
- The port-runway switch `PHOTONIC_ROUTER_PORT_RUNWAYS` /
  `FanoutAccessConfig.port_runways` and its code (`route_jobs.
  _port_runway_anchor`, `_build_port_runway_anchors`, the runway branch of
  `_build_static_fanout_target_anchors`, `SessionState.fanout_stub_anchor`,
  `port_runway_anchor_specs`, `fanout_stub_net_ids`, the generalized target
  stub append in `finalize.py`, `tests/test_port_runways.py`; 492 lines):
  the experiment closed not adopted and moved to
  `experiments/port-runways/` as three patches against `89fe1b1`. The tree
  without it is the tree before `b05bffd` in these files; nine-cell gate
  exact.
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
