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
