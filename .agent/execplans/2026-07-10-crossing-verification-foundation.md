# Build a crossing verification foundation for router-discovered crossings

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds.

This document follows `.agent/PLANS.md` in this repository. It is informed by `.agent/PROJECT_GOAL.md`, which defines the repository-level goal: a very fast Rust/Python photonic router that can route benchmark designs and verify final physical geometry.

## Purpose / Big Picture

The current project goal is not to reimplement LiDAR. The goal is to make TUMPhotonicRouter a fast, verified photonic router. The immediate feature is router-discovered crossings: the route search should explore and choose legal crossings when they help complete a design.

Before adding more crossing heuristics, the repository needs a stable verification foundation. A future agent should be able to run `benes_4x4`, `benes_8x8`, and eventually `multiportmmi_8x8`, then know whether the final geometry is correct. If crossing insertion fails, the failure should say whether search failed, centerline geometry was invalid, realization changed the path, port snapping invalidated a protected segment, or final verification rejected the layout.

The first visible outcome is a repeatable harness that proves small crossing benchmarks either pass with legal crossings or fail with concrete classified errors.

## Progress

- [x] (2026-07-10 00:00Z) Captured the repository-level goal in `.agent/PROJECT_GOAL.md`.
- [x] (2026-07-10 00:00Z) Created this crossing verification foundation plan as the active next-step plan.
- [x] (2026-07-10 11:41Z) Audited the existing crossing, realization, port-snapping, verification, and A* cost paths before editing behavior.
- [x] (2026-07-10 11:55Z) Defined the minimal structured crossing verification report contract in `translation/crossing_verification_report.py`.
- [x] (2026-07-10 11:55Z) Added focused report fixture tests in `tests/test_crossing_verification_report.py`; full pytest execution is pending project toolchain setup.
- [x] (2026-07-10 12:23Z) Wired the structured crossing report into `routing_flow.py` so crossing-enabled benchmark runs write `build/verification/*_crossing_verification.json` outside the A* hot path.
- [x] (2026-07-10 12:23Z) Added a focused routing-flow harness test that checks the emitted crossing verification JSON and component-info summary using a fake router result.
- [x] (2026-07-10 13:14Z) Converged the Windows setup enough to run focused validation: created `.venv`, installed Python dependencies, installed Rust via rustup, built the PyO3 extension with maturin, and recorded the required gnullvm/rust-lld environment.
- [x] (2026-07-10 13:14Z) Ran focused Python and Rust validation for the crossing verification foundation.
- [x] (2026-07-10 13:14Z) Ran `benes_4x4` and `benes_8x8` with crossings enabled and recorded structured JSON pass/fail evidence.
- [x] (2026-07-10 13:39Z) Placed active gdsfactory/PDK crossing refs for legal realized route intersections and exposed the placement list to both crossing debug JSON and the structured verification report.
- [x] (2026-07-10 13:39Z) Added net-id preservation to routed records so `route_match_and_realize` can refresh crossing verification after endpoint correction and path-length meander planning before final realization.
- [x] (2026-07-10 13:39Z) Re-ran focused tests and `benes_4x4`/`benes_8x8`; both benchmark reports now pass with all legal crossings matched to realized crossing components.
- [x] (2026-07-10 13:47Z) Closed review findings: insertion-loss reports now count final `realized_intersections`, the structured report validates crossing component rotation, and stale handoff wording was made historical.
- [x] (2026-07-11 01:15Z) Backfilled the orchestration workflow after user-provided GDS screenshots showed that the prior loop over-centralized reasoning in the orchestrator and lacked an independent harness/reviewer gate.
- [x] (2026-07-11 10:10Z) Tightened the final crossing verifier so crossing-enabled runs reject missing corrected physical centerlines instead of falling back to compressed waypoint chords.
- [x] (2026-07-11 10:10Z) Added a real photonic verifier fixture that realizes a bent route through Rust/gdsfactory geometry and catches a same-layer static optical blocker as `waveguide_obstacle_overlap`.
- [x] (2026-07-11 10:10Z) Rebuilt the Rust extension and re-ran focused crossing/static validation after the verifier fixes.
- [x] (2026-07-11 12:15Z) Fixed the `multiportmmi_8x8` cluster loop around `n_31`/`n_32` by clipping generated port-opening cells to the forward side of the port plane, then regenerated the six-route overlay SVG for `n_31` through `n_36`.
- [x] (2026-07-11 12:45Z) Tightened realized crossing endpoint-access ignores so opened cells near fanout/internal bends no longer hide the illegal `n_32` x `n_35` crossing in the `multiportmmi_8x8` cluster.
- [x] (2026-07-11 13:23Z) Mirrored the endpoint-access tightening in Rust native commit validation and added a Rust regression so opened cells several cells away from a real route endpoint no longer suppress illegal realized crossings.
- [x] (2026-07-11 13:36Z) Implemented the next native repair-feedback slice: strict validation errors now promote learned committed crossing participants into the adaptive blocker/victim set, victim-reroute failures are attributed to the victim net rather than always to the original failed job, and two Rust regression tests document the learned-blocker behavior.
- [x] (2026-07-11 17:01Z) Completed the bounded route-37 repair audit. The verifier now catches the `n_32` cluster during native routing, repair-feedback retries are capped/scoped, and focused tests pass, but the route-37 benchmark remains blocked at `n_32`; this is now a concrete blocker packet rather than an unbounded repair loop.
- [x] (2026-07-12 12:30Z) Ran the QA / Harness route-37 packet after cleanup and rebuilt the Rust extension first so Python used current `src/py_router.rs` code. The bounded `multiportmmi_8x8 --debug-stop-after-route 37` command failed in about 163 seconds at `n_32` / native `net33` with blockers `[36, 31]`. Added a deterministic failure-packet helper and `tests/test_route_failure_diagnostics.py` so `*_FAILED.txt` logs expose `root_cause_illegal_crossings` as JSON when repair exhaustion follows illegal realized crossings.
- [ ] Repair the `multiportmmi_8x8` `n_32` cluster routing failure exposed by stricter native validation; current repair finds partial legal sub-attempts but cannot converge on a globally valid `n_32`/`n_35` pair reroute.
- [x] (2026-07-12 19:33Z) Verified that LiDAR-pure crossing legality is partially enforced during A* neighbor exploration (`crossing_move_outcome` rejects non-perpendicular, wrong-order, low-margin, unexpected-owner, and uncleared-footprint moves), but the native repair loop was incorrectly widening victim sets by route order. Removed route-order victim expansion from `src/py_router.rs` and updated the workflow/role briefs so repair victims must be geometry-evidence-backed.
- [ ] Solve the remaining dense `multiportmmi_8x8` routing strategy blocker: strict native crossing validation now stops at `n_51` rather than accepting a non-perpendicular/bend-footprint crossing.
- [x] (2026-07-14 12:20Z) Established the current low-level crossing invariant for the `multiportmmi_8x8` `n_31`/`n_32` cluster: A* Layer 1 now detects offset one-cell diagonal collisions with a compact diagonal halo, evaluates them as crossing candidates, and rejects illegal moves before repair.
- [x] (2026-07-14 12:20Z) Removed temporary validation-bypass experiments from the checkpoint. The standing policy is that Python/Rust realized verification must not be disabled to write invalid GDS during normal routing work.
- [ ] Continue from the clean invariant boundary: when Layer 1 rejects illegal crossing moves and cannot find a legal route, repair/rip-up is the intended next phase. Future work should improve legal route discovery or repair convergence, not reintroduce post-route illegality acceptance.

## Surprises & Discoveries

- Observation: The current Windows checkout is at
  `C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter`, not the older
  Ubuntu path used in historical notes.
  Evidence: `git status --short --branch` was run from that path on 2026-07-10.

- Observation: The current Windows shell is not ready to run the full router
  validation ladder yet.
  Evidence: `.venv` is absent, `python`, `py`, `cargo`, `rustc`, and `maturin`
  are not available on `PATH`, and Codex's bundled Python does not have
  `pytest`, `maturin`, or `gdsfactory` installed.

- Observation: The Windows toolchain gap was reconfirmed before this audit
  attempted validation.
  Evidence: On 2026-07-10 11:41Z, `Test-Path .venv` and
  `Test-Path .\.venv\Scripts\python.exe` returned false. `python --version`,
  `py --version`, `pytest --version`, `cargo --version`, `rustc --version`,
  and `maturin --version` all failed as commands unavailable in PowerShell.
  Benchmark validation and pytest/cargo validation were therefore not run.

- Observation: A local LiDAR checkout exists on this Windows machine at
  `C:\Users\benja\Documents\Repositorys\LiDAR`.
  Evidence: `Test-Path "C:\Users\benja\Documents\Repositorys\LiDAR"` returned
  true. The old Ubuntu `working/LiDAR` path and the local paper PDF path were
  not present in this Windows environment.

- Observation: This Codex runtime can spawn subagents that inherit the parent
  model by default.
  Evidence: The discovered `multi_agent_v1.spawn_agent` schema says spawned
  agents inherit the current model when `model` is omitted. The Planner audit
  spawned read-only explorer subagents without a model override and used only a
  role-appropriate reasoning override.

- Observation: The current crossing metadata path has two sources. Topology
  crossing events are built by `python/photonic_router/crossing_plan.py` and
  loaded into Rust as `CrossingConstraint`s by
  `translation/route_rust.py::_build_crossing_plan_info`. Router-discovered
  crossing events are stored globally in `src/py_router.rs` and exposed through
  `PyPhotonicRouter.crossing_events()`, then copied into
  `crossing_plan_info["native_crossing_events"]`.
  Evidence: Read-only audits of `crossing_plan.py`, `translation/route_rust.py`,
  `src/crossings.rs`, and `src/py_router.rs` on 2026-07-10.

- Observation: The active gdsfactory crossing component is currently used to
  derive an abstract footprint, not inserted into the realized optical layout.
  Evidence: `translation/route_rust.py::_crossing_component_bbox_size_um`
  calls `gf.components.crossing()` and `_resolve_crossing_half_size_cells`
  derives `crossing_half_size_cells` from its bbox plus clearance. In contrast,
  `translation/route_rust_realization.py::realize_routed_net_records` only adds
  route polygons with `routed_layout.add_polygon(...)`; no audited realization
  path places crossing refs with `add_ref(...)`.

- Observation: The current `lidar-pure` routing attempt is mostly
  router-discovered, but the full pipeline is not cleanly isolated from
  topology crossing hints.
  Evidence: `translation/route_rust.py` maps `lidar-pure` to
  `allow_only_expected_crossings=False` and enables collision crossing routing.
  The main collision route can use already committed route partners rather than
  only expected pairs. However, Python still builds topology constraints when
  metadata is available, and Rust auxiliary/repair paths still consult
  expected-count or ordered-constraint state in some places.

- Observation: Port snapping and endpoint correction can change realized
  centerlines after crossing search, but route records do not carry protected
  crossing-to-crossing segment metadata.
  Evidence: `translation/route_rust.py` applies pre-route state snapping in
  `_snap_nearly_collinear_states`, `_snap_same_heading_minimum_bend_offset`,
  and `_states_and_openings`. Later, `apply_checked_endpoint_corrections_and_commit`
  writes `corrected_centerline_um`. `RoutedNetRecord` has route, length, opened
  cells, port centers, corrected centerline, and meander metadata, but no
  intended crossing list or protected segment ranges.

- Observation: Final photonic verification is still generic polygon
  verification, while crossing legality is checked in a separate route-level
  helper.
  Evidence: `translation/route_rust.py::_verify_realized_route_intersections`
  classifies legal expected/unexpected crossings and illegal crossing reasons
  such as non-perpendicular geometry, bend-containing footprints, route geometry
  inside the crossing footprint, and overlapping crossing footprints. By
  contrast, `translation/photonic_verification.py::verify_photonic_routing`
  reports route record coverage, endpoint/contact issues, generic cross-net
  waveguide overlaps, and obstacle overlaps, but does not classify legal
  crossings, self-intersections, or missing/misplaced crossing components.

- Observation: `RouteResult.total_cost` currently mixes physical route-quality
  proxies with non-physical search guidance whenever history or proactive
  congestion is enabled.
  Evidence: In `src/astar.rs`, primitive metadata sets base step cost to
  length plus `bend_weight * bend_cost`. Dense A* adds history and proactive
  congestion costs to `total_cost`; crossing-aware A* also adds
  `crossing_count * crossing_loss`. Simple and JPS4 routes use length-only
  costs. Python has a partial crossing insertion-loss summary in
  `_augment_insertion_loss_report`, but route attempts and final verification
  do not expose a decomposed physical-versus-guidance cost report.

- Observation: A pure-Python crossing verification report layer can cover the
  immediate harness contract without touching Rust routing or realization.
  Evidence: `translation/crossing_verification_report.py` now defines
  `CrossingVerificationReport`, `CrossingVerificationIssue`, `CrossingRecord`,
  `RouteCostTerms`, JSON writing, component-placement checks,
  protected-segment movement checks, illegal-crossing issue conversion, and
  physical-versus-guidance cost decomposition. The module was syntax-compiled
  and smoke-tested with Codex's bundled Python on 2026-07-10 11:55Z.

- Observation: The structured report can now be produced by the end-to-end
  flow without adding work to Rust A* search.
  Evidence: `routing_flow.py` maps `crossing_plan_info["insertion_loss_by_net"]`
  into physical route-cost report fields, writes
  `build/verification/<benchmark>_crossing_verification.json`, and stores the
  report summary in `routed_layout.info["crossing_verification"]` after
  routing. `tests/test_routing_flow_stats.py` has a monkeypatched pipeline
  fixture that checks this JSON path and its missing-component classification.

- Observation: The Windows setup converged without installing Visual Studio
  Build Tools by using Rust's `stable-x86_64-pc-windows-gnullvm` host toolchain
  plus `rust-lld`.
  Evidence: `cargo check` failed under the default MSVC host because
  `link.exe` was missing. `rustup toolchain install
  stable-x86_64-pc-windows-gnullvm --profile minimal` succeeded. With
  `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm`,
  `RUSTFLAGS="-C linker=C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin\rust-lld.exe -C target-feature=+crt-static"`,
  and `PYO3_PYTHON` pointing at `.venv\Scripts\python.exe`, `cargo check`
  and `maturin develop --release` both succeeded on 2026-07-10.

- Observation: Installing current `gdsfactory` into the deep repository path
  hit the Windows 260-character path limit in Jupyter widget assets.
  Evidence: The first `pip install pytest maturin gdsfactory pyyaml` failed on
  a `.venv\share\jupyter\labextensions\...js.map` path whose length was 260.
  Retrying the install through a temporary `subst T:
  C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter` mapping succeeded.

- Observation: Python benchmark commands on this Windows console need UTF-8
  stdout encoding because `routing_flow.py` prints checkmark symbols.
  Evidence: The first `benes_4x4` CLI run failed before routing with
  `UnicodeEncodeError: 'charmap' codec can't encode character '\u2713'`.
  Rerunning with `PYTHONIOENCODING=utf-8` fixed the console output.

- Observation: Before crossing component placement was implemented, the
  smaller Benes benchmarks already showed that crossing search itself was
  finding legal route intersections.
  Evidence: The pre-placement `benes_4x4` run wrote
  `build/verification/benes_4x4_crossing_verification.json` with
  `legal_crossing_count=2`, `illegal_crossing_count=0`, and two
  `missing_crossing_component` errors. The pre-placement `benes_8x8` run wrote
  `build/verification/benes_8x8_crossing_verification.json` with
  `legal_crossing_count=16`, `illegal_crossing_count=0`, and sixteen
  `missing_crossing_component` errors.

- Observation: The missing-component blocker for the smaller Benes benchmarks
  is now resolved by inserting active crossing refs after final route
  centerlines are known.
  Evidence: `translation/route_rust.py::_place_realized_crossing_components`
  filters `crossing_plan_info["realized_intersections"]` to legal crossings,
  centers/rotates active `gf.components.crossing()` refs at `point_um`, writes
  placement metadata to `crossing_plan_info["realized_crossing_components"]`,
  and mirrors it into `routed_layout.info`. On 2026-07-10 13:39Z,
  `benes_4x4` reported `success=True`, `legal_crossing_count=2`,
  `matched_crossing_component_count=2`, `issues=0`; `benes_8x8` reported
  `success=True`, `legal_crossing_count=16`,
  `matched_crossing_component_count=16`, `issues=0`.

- Observation: `route_match_and_realize` now re-verifies crossings after
  endpoint correction and path-length meander planning, instead of placing
  components from stale pre-PLM route records.
  Evidence: `RoutedNetRecord` carries an optional `net_id`, `RouteBookkeeping`
  sets it from `RouteJob.net_id`, meander-planned records preserve it, and
  `route_match_and_realize` rebuilds a final `{net_id: RoutedNetRecord}` map
  before calling `_verify_realized_route_intersections`.

- Observation: Crossing component placement is a report and GDS-reference
  foundation, not complete physical waveguide clipping.
  Evidence: The realization path still draws route polygons through the
  crossing footprint before adding the crossing ref. If same-layer route and
  crossing-device overlaps become DRC errors, a later slice must trim or split
  waveguide polygons around crossing footprints.

- Observation: The final insertion-loss report now follows final realized
  geometry instead of earlier native crossing events.
  Evidence: `_augment_insertion_loss_report_from_realized_intersections`
  derives crossing counts from legal `crossing_plan_info["realized_intersections"]`
  after final crossing verification. `insertion_loss_model` records
  `crossing_count_source="realized_intersections"`, and
  `tests/test_realized_crossing_verification.py::test_insertion_loss_report_uses_final_realized_intersections`
  covers this behavior.

- Observation: The visible `multiportmmi_8x8` loop in the user-provided
  `n_32`-cluster screenshot was actually `n_31` curling left/backward through
  the port-opening area at `mmi0_multiport_0_0,o12`.
  Evidence: Before the fix,
  `build/routes/multiportmmi_8x8_n_31_diagnostics.txt` started with
  `straight:(706, 162)->(707, 162); turn90:(707, 162)->(710, 159);
  turn90:(710, 159)->(707, 156); turn90:(707, 156)->(704, 159)`,
  and reported opened static overlap around the source cluster. After clipping
  port openings to cells with non-negative forward projection, the fresh route
  starts `turn45:(706, 162)->(712, 165)` and no longer visits `x=704`.

- Observation: The static-obstacle bug was in port-opening generation, not in
  the static map itself. The static map preserved blocked port cells, but
  `route_collect_inflated_step_cells` previously used a square inflation around
  each forward step. For east-facing ports this opened cells behind the port
  plane, making raw static geometry legal to A* as normal route space.
  Evidence: `src/py_router.rs::route_collect_inflated_step_cells` now keeps the
  square width behavior in front of the port but drops cells whose projection
  along the port direction is negative. The regression fixture
  `tests/test_route_rust_opened_cells.py::test_rust_port_opening_batch_is_directional_not_behind_port`
  asserts an east-facing port does not open `(2, 10)` behind the base cell.

- Observation: The `n_32` x `n_35` cluster crossing was real in the route
  geometry, but final crossing verification hid it as endpoint access because
  opened cells near fanouts/internal bends used an over-broad tolerance.
  Evidence: The stopped `multiportmmi_8x8` run through route 37 now reports
  `Illegal realized route crossing(s) ... n_32 x n_35 at [1403.715334,
  686.584666]`, with `perpendicular=false` and an approximate realized segment
  angle difference of 59.1 degrees. The primary reason is
  `crossing_footprint_contains_bend` because the intersection is only about
  0.177 um from an internal `n_35` bend, but the non-perpendicular flag is also
  preserved.

- Observation: The same over-broad endpoint-access suppression existed in Rust
  native commit validation, so Python final verification could reject geometry
  that Rust had allowed into committed route state. Tightening the Rust
  suppression moves the `multiportmmi_8x8` route-37 failure earlier: routing
  now fails at `n_32` with recent native errors such as `Illegal realized
  crossing: net 33 intersects net 36 ... (not_perpendicular)` instead of
  writing a final partial route with `n_32` x `n_35` hidden until final
  verification.
  Evidence: `cargo test
  committed_crossing_validation_rejects_opened_cell_crossing_away_from_endpoint
  --lib` passed, `tests/test_realized_crossing_verification.py` passed with 21
  tests, and `maturin develop --release` rebuilt the extension. The bounded
  stop-after-37 run after the Rust tightening failed at `n_32` with
  `candidate_blockers=[36, 31]` and native `not_perpendicular` errors; later
  route-order and wider-feedback-keepout experiments were reverted or stopped
  because they only moved the failure or exceeded the quick-feedback timing
  budget.

- Observation: `cargo fmt --check` cannot currently run on this Windows
  gnullvm toolchain because `rustfmt` is not installed for
  `stable-x86_64-pc-windows-gnullvm`.
  Evidence: The command reported `cargo-fmt.exe is not installed for the
  toolchain 'stable-x86_64-pc-windows-gnullvm'`.

- Observation: The current PowerShell runtime can no longer rebuild or test
  Rust code until a Windows linker is installed or exposed on `PATH`.
  Evidence: `cargo test targeted_illegal_crossing_repair_promotes_learned_blocker
  --lib` with the default target failed before repository code with missing
  `link.exe`. Retrying with
  `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm` and
  `CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm` failed with missing
  `x86_64-w64-mingw32-clang`. `maturin develop --release` fails the same way
  after adding `C:\Users\benja\.cargo\bin` to `PATH`. `where clang`,
  `where lld-link`, `where x86_64-w64-mingw32-clang`, `where zig`, and the
  standard Visual Studio `vswhere.exe` location were all absent.

- Observation: A lightweight Zig fallback was attempted but did not produce a
  working native Rust validation path.
  Evidence: `winget install --id MartinStorsjo.LLVM-MinGW.UCRT` found the
  package but stayed in archive extraction for an extended period without
  producing a usable linker, so the elevated `winget` process was stopped.
  `pip install ziglang cargo-zigbuild` succeeded in the project `.venv`, and
  `rustup +stable-x86_64-pc-windows-gnullvm target add x86_64-pc-windows-gnu`
  succeeded. A temporary ignored wrapper at
  `build/tools/x86_64-w64-mingw32-clang.cmd` let Cargo invoke Zig, but native
  test builds then failed on MinGW runtime lookup (`msvcrt`) and Windows `.def`
  linker argument handling before reaching repository code.

- Observation: The structured report now catches centered but mis-rotated
  crossing components.
  Evidence: `translation/crossing_verification_report.py` compares each
  matched component's `rotation_deg` to the realized crossing `segment_a_um`
  axis modulo 180 degrees. `tests/test_crossing_verification_report.py`
  includes a 90-degree mismatch fixture that fails with
  `crossing_component_rotation_mismatch`.

- Observation: Crossing verification must use the same corrected physical
  centerline source as realization; compressed route waypoints can hide bends.
  Evidence: On 2026-07-11, `_verify_realized_route_intersections` was changed
  to emit `illegal_route_geometry` with reason `missing_corrected_centerline`
  or `endpoint_correction_error` when a crossing-enabled route record lacks a
  corrected centerline. The regression
  `tests/test_realized_crossing_verification.py::test_realized_crossing_verifier_rejects_missing_corrected_centerline_fallback`
  covers a record whose `route_obj.compressed_waypoints` would otherwise form
  a plausible crossing chord.

- Observation: The static-blocker issue from the screenshots belongs to final
  physical geometry verification, not only grid search.
  Evidence: `tests/test_photonic_verification.py::test_photonic_verifier_reports_realized_bend_static_overlap`
  now builds a real Rust `RouteResult`, realizes a bent corrected centerline,
  and verifies that overlap with an unrouted `(1, 0)` obstacle reports
  `waveguide_obstacle_overlap` without monkeypatching route regions.

- Observation: With stricter legality gates, `multiportmmi_8x8` no longer
  silently accepts the bad crossings seen in the GDS screenshots; it stops at a
  dense fanout routing failure.
  Evidence: On 2026-07-11, running
  `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
  --debug-svgs false --debug-timing false` failed while routing `n_51`
  (`mmi0_multiport_1_1,o8 -> mmi0_ps_array_2_heater_4,o1`) with recent native
  errors including `Illegal realized crossing: net 52 intersects net 51 ...
  (not_perpendicular)`. This is now a routing-strategy blocker around the
  adjacent fanout cluster, not a verifier miss.

- Observation: A repair experiment that let the orthogonal repair path run
  collision-crossing search was correct in concept but too slow for this
  router's speed requirement when tested through route 52 of `multiportmmi_8x8`.
  Evidence: The bounded route-52 diagnostic did not return within the local cap
  even after narrowing lidar-pure partners. The speed-risk behavior was removed;
  only the behavior-neutral helper refactor and the verifier guardrails remain.

- Observation: The fresh route-37 QA / Harness packet preserves the `n_32`
  root cause in a machine-readable failed-log field. The route still fails at
  `n_32` / native `net33`, but
  `build\routes\multiportmmi_8x8_n_32_FAILED.txt` now has
  `root_cause_illegal_crossings` with the two `net33 -> net36`
  non-perpendicular points and the victim `net36 -> net33`
  non-perpendicular point.
  Evidence: After rebuilding the Rust extension, the bounded route-37 command
  returned in about 163 seconds with
  `No repair route found; candidate_blockers=[36, 31]`. The refreshed failed
  log was written at `2026-07-12 14:29:10 +02:00`, no final
  `build\verification\multiportmmi_8x8_*_verification.json` files were
  produced, and `tests/test_route_failure_diagnostics.py` reported `2 passed`.

- Observation: The `n_32` probe path in `multiportmmi_8x8` showed an offset
  diagonal collision against committed `n_31` that was not represented as a
  shared core grid cell. A* therefore needed a search-time halo for one-cell
  diagonal segments rather than relying only on exact cell overlap.
  Evidence: After rebuilding with the diagonal halo instrumented,
  `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
  lidar-pure --debug-stop-after-route 33 --debug-svgs false
  --debug-timing false --attempt-diagnostics` with
  `PHOTONIC_ROUTER_TRACE_CROSSING_NET=33` and
  `PHOTONIC_ROUTER_TRACE_PARTNER_NET=32` printed lines such as
  `diagonal-halo reject-margin net=33 partner=32 x=724.500 y=170.500
  route_margin=0.500 partner_margin=12.021 required_margin=5`.

- Observation: The compact halo fixed the low-level detection question, but it
  did not make the first collision-crossing A* attempt find a legal route.
  Evidence: In the same run, A* saw and rejected multiple offset diagonal
  candidates, then the flow reached `native_repair_probe net=33 ...
  candidate_blockers=[32]`. That is acceptable behavior under the current
  invariant: rip-up follows legal-search failure. It is not evidence that
  illegal crossings should be accepted.

- Observation: Temporary debug switches that allow invalid GDS or disable
  realized crossing validation are too dangerous for the stable checkpoint.
  Evidence: The worktree previously had env-gated bypasses named
  `PHOTONIC_ROUTER_DEBUG_ALLOW_INVALID_GDS` and
  `PHOTONIC_ROUTER_DISABLE_REALIZED_CROSSING_VALIDATION*`; these were removed
  before the checkpoint because they conflict with the rule that verifier
  rejection after A* acceptance is a blocking model mismatch.

## Decision Log

- Decision: Treat LiDAR as a source of ideas for router-discovered crossings, not as the architecture to clone.
  Rationale: The repository goal is a faster Rust-backed router with verified output. Matching LiDAR's internal repair behavior is useful only when it helps explain crossing correctness failures.
  Date/Author: 2026-07-10 / Codex

- Decision: Keep topology-precomputed crossings as a later mode, but do not use them in the current `lidar-pure` / router-discovered path.
  Rationale: `multiportmmi_8x8` should be solved with crossings explored during routing itself. Topology-precomputed crossings can be included later as a separate comparison or optimization path.
  Date/Author: 2026-07-10 / Codex

- Decision: Use both structured JSON reports and pytest assertions for verification.
  Rationale: JSON reports under `build/` make benchmark failures inspectable by agents and humans; pytest assertions make fixture-level classification deterministic and prevent regressions.
  Date/Author: 2026-07-10 / Codex

- Decision: A* route selection should use a route-induced insertion-loss objective.
  Rationale: The router should choose among crossing and non-crossing alternatives using physical route quality: propagation length, bend penalty, and crossing penalty. Congestion/history costs can guide search and repair, but should remain distinguishable from physical insertion loss in reports.
  Date/Author: 2026-07-10 / Codex

- Decision: Physical crossings must come from the active PDK/gdsfactory crossing component.
  Rationale: Crossing legality depends on the actual component footprint and access geometry. Hard-coded abstract crossing markers are not enough to prove final GDS correctness.
  Date/Author: 2026-07-10 / Codex

- Decision: Put verification before more heuristic tuning.
  Rationale: Without a reliable pass/fail harness, agents can make the router appear to progress by changing traces or screenshots while still producing invalid physical geometry.
  Date/Author: 2026-07-10 / Codex

- Decision: Preserve native illegal-crossing root causes in failed route logs
  as structured diagnostic JSON before another `n_32` implementation loop.
  Rationale: The verifier was already catching non-perpendicular geometry, but
  repair exhaustion can obscure the original `net33`/`net36` cause behind a
  terminal `No repair route found` error. A small diagnostic field gives QA and
  reviewers a deterministic hook without changing routing behavior or slowing
  the A* hot path.
  Date/Author: 2026-07-12 / Codex

- Decision: Make QA / Harness verification a mandatory gate after nontrivial
  routing implementation.
  Rationale: The `multiportmmi_8x8` debug loop showed that an orchestrator can
  spend too long serially tuning route order and can overstate a partial/stale
  artifact. Harness must inspect structured reports and artifacts and return a
  verdict before the user is told a routing slice is fixed or complete.
  Date/Author: 2026-07-11 / Codex

- Decision: Do not keep routing-strategy experiments that materially slow the
  dense benchmark, even if they are geometrically promising.
  Rationale: The router is explicitly optimized for speed. Repair ideas for the
  `multiportmmi_8x8` fanout cluster need bounded fixtures or narrow native
  tests before being installed into the benchmark path.
  Date/Author: 2026-07-11 / Codex

- Decision: Protect crossing-to-crossing route segments from port snapping.
  Rationale: Crossings should be decided during routing. Port snapping should only adjust source-to-first-crossing and last-crossing-to-target access geometry, otherwise it can silently invalidate crossing legality after search.
  Date/Author: 2026-07-10 / Codex

- Decision: Move next to QA / Harness Engineer before behavior edits.
  Rationale: The Planner audit found missing evidence surfaces: no final
  crossing-aware verification report, no placed PDK crossing components, no
  protected-segment mismatch classification, and no decomposed route-cost
  report. The next useful step is to define focused failing fixtures and the
  structured report contract before changing routing or realization behavior.
  Date/Author: 2026-07-10 / Codex

- Decision: Use the Rust gnullvm host toolchain plus `rust-lld` for this
  Windows setup unless MSVC Build Tools are later installed.
  Rationale: The machine has no `link.exe`, Visual Studio, Windows SDK, LLVM,
  MSYS, or MinGW on PATH. The gnullvm toolchain with `rust-lld` is enough for
  `cargo check`, focused Rust tests, and `maturin develop --release` when
  `PYO3_PYTHON` points at the project `.venv`.
  Date/Author: 2026-07-10 / Codex

- Decision: For router-discovered crossings, Layer 1 A* is responsible for
  the primary legality decision.
  Rationale: The desired flow is that A* explores primitive moves, detects a
  possible crossing, checks perpendicularity, margin, ownership/order, and
  footprint clearance, then rejects only that move if it is illegal and keeps
  searching. Rip-up is correct only after the legal search cannot find a path.
  A later Python or Rust verifier rejecting a crossing that A* accepted is a
  blocking model mismatch, not a normal repair signal.
  Date/Author: 2026-07-14 / Codex

- Decision: One-cell diagonal primitive pieces need a compact search-time halo.
  Rationale: Offset diagonal crossings can touch physically even when the two
  one-cell diagonal centerline rasters do not share the same grid cell. The
  halo must be a compact adjacent lane like the user's red/green sketch, and
  it applies to straight diagonals and diagonal arms inside bend primitives.
  The halo detects a possible collision only; the actual crossing is still
  validated against the true route and partner centerline segments.
  Date/Author: 2026-07-14 / Codex

- Decision: Make Layer-1 crossing detection owner-first and local.
  Rationale: The clean hot-path contract is:
  `effective_footprint = primitive_footprint + compact diagonal halo cells`.
  Static contacts in that effective footprint reject the A* move. Dynamic
  contacts produce only the nearby committed route owners that need crossing
  checks. Crossing legality is then resolved from the true candidate primitive
  centerline and true committed-owner centerline, not from the halo cell itself.
  This differs from the current mixed model, which still performs broad
  geometric/partner scans in addition to cell-owner collision checks. The
  owner-first contract should preserve behavior while reducing unnecessary
  per-move segment checks; broad scans should remain diagnostic/fallback only
  until removed.
  Date/Author: 2026-07-14 / Codex

- Decision: Endpoint correction is post-routing and must not feed back into
  A* crossing legality.
  Rationale: Crossing search and native validation must reason about the
  primitive/grid route model and protected primitive-realized centerline. For
  crossed nets, endpoint correction may only modify terminal regions outside
  the first/last crossing span. For crossing-free nets, normal endpoint
  correction can run after routing. This prevents a route from being legal in
  A* and then becoming illegal because port correction moved a crossing.
  Date/Author: 2026-07-14 / Codex

- Decision: Do not keep validation-disable or invalid-GDS bypasses in the
  stable routing path.
  Rationale: They were useful for visual experiments, but a clean checkpoint
  must fail closed. If A* accepts an illegal crossing or final verification
  rejects geometry, the repository should expose that failure through reports
  and tests rather than writing a normal-looking GDS.
  Date/Author: 2026-07-14 / Codex

## Outcomes & Retrospective

The Planner / Technical Lead audit for the first milestone completed on
2026-07-10 without behavior edits. The audit established where crossing
metadata, crossing bbox selection, endpoint correction, realization, final
verification, and A* cost accounting currently live. Local validation did not
run because the Windows checkout lacks `.venv`, Python, pytest, Rust/Cargo, and
maturin on `PATH`. The next role should be QA / Harness Engineer to define the
minimal crossing verification report and focused fixture tests.

The QA / Harness Engineer slice on 2026-07-10 added the first structured report
contract and fixture tests. This is harness work only: it does not yet insert
PDK crossing components, protect endpoint-correction spans, or wire benchmark
runs to write `build/verification/*.json`. Full pytest and benchmark validation
remain blocked until the project `.venv`, Python, pytest, Rust/Cargo, and
maturin are available.

The follow-up wiring slice on 2026-07-10 connected the structured report to
`routing_flow.py`. Crossing-enabled runs now write a verification JSON file
under `build/verification/` after routing completes, using the existing
`crossing_plan_info` diagnostics and insertion-loss summary. At that point,
before the later component-realization slice, legal route crossings reported
`missing_crossing_component`.

The setup convergence and validation slice on 2026-07-10 turned the Windows
checkout into a working local validation environment. The project `.venv` now
has pytest, maturin, gdsfactory, and PyYAML. Rust is installed through rustup,
and the PyO3 extension builds with the `stable-x86_64-pc-windows-gnullvm`
toolchain and `rust-lld`. Focused Python and Rust tests pass. `benes_4x4` and
`benes_8x8` route successfully with crossings enabled and produce structured
verification reports. Those reports classify the current remaining blocker as
missing physical crossing components, not crossing search failure or illegal
route intersections. That blocker was closed by the later
crossing-component realization slice.

The crossing-component realization slice on 2026-07-10 closed that blocker for
the smaller Benes benchmarks. Legal final route intersections now receive active
gdsfactory crossing refs, the placement list is included in debug and
verification JSON, and the match-and-realize pipeline refreshes crossing
verification after endpoint correction and PLM using final routed records.
After review, focused Python checks reported `25 passed` for the
report/realized-crossing fixture set and `6 passed` for record/geometry
regression checks. `benes_4x4` and `benes_8x8` now both produce successful
crossing verification reports with zero issues and all legal crossings matched
to realized components.

Workflow retrospective, 2026-07-11: The follow-up `multiportmmi_8x8` debugging
loop fixed some earlier failure signatures, but it violated the intended team
workflow by keeping too much reasoning, implementation, and verification in the
orchestrator window. User screenshots then showed additional invalid geometry
and exposed that the validation packet and artifact wording were not strong
enough. Stable workflow files were updated so nontrivial routing work now
requires a verifier packet, QA / Harness verdict, and reviewer gate before
claiming completion. Future `multiportmmi_8x8` work should resume with Harness
Engineer first: inspect the new screenshots/GDS, classify which deterministic
check is missing, and only then assign implementation.

## Context and Orientation

TUMPhotonicRouter is a hybrid Rust and Python photonic router. Python loads benchmarks, integrates with gdsfactory, orchestrates routing stages, performs geometry realization, and writes debug artifacts. Rust owns performance-sensitive routing logic such as obstacle maps, A* search, route state, crossing checks, and geometry-heavy kernels through the `photonic_router._rust` PyO3 extension.

The routing pipeline should be understood as setup, routing, path-length matching, port snapping, geometry realization, and verification. Crossings belong to the routing stage and must be preserved through later stages. Path-length matching already has regression coverage for some benchmarks, including heater-style cases such as `heater_s_mod`; crossing work must not regress those tests.

The realized crossing component should be read from the active PDK or
gdsfactory component library. Its footprint must inform crossing keepouts,
straight-access requirements, spacing checks, debug geometry, and final
verification. Do not treat a crossing as correct merely because two centerlines
intersect legally in the route record; the final geometry must contain the
crossing component and remain DRC-clean around it.

The immediate crossing benchmarks are `benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`. The smaller Benes benchmarks should establish basic correctness and harness confidence. `multiportmmi_8x8` is the current hard case and should be debugged after the harness can clearly classify failures. It must be solved on the router-discovered crossing path without topology-precomputed crossing hints.

Relevant files likely include:

- `routing_flow.py`, the end-to-end orchestration entry point.
- `translation/route_rust.py`, the Python bridge into Rust routing and crossing options.
- `translation/route_rust_realization.py`, where route records become physical geometry.
- `translation/photonic_verification.py`, where final route geometry should be classified.
- `translation/route_rust_records.py`, where route metadata and crossing records may need to preserve intent.
- `src/astar.rs`, where A* search and route result metadata live.
- `src/crossings.rs`, where crossing constraints and crossing-related Rust types live.
- `src/py_router.rs`, where the PyO3 router API and batch routing/repair behavior live.
- `src/geometry_realization.rs`, where Rust-side geometry helpers may affect realized paths.
- `tests/test_realized_crossing_verification.py`, existing crossing verification coverage.
- `tests/test_photonic_verification.py`, existing final verification coverage.
- `tests/test_route_rust_geometry.py`, geometry realization tests.

Do not assume these are the only files to change. Read the current code before editing.

## Plan of Work

First, audit the current pipeline. Identify where a route's intended crossing metadata is created, where the active PDK/gdsfactory crossing component is selected, where its footprint is converted into grid keepout/access rules, where route centerlines are transformed during realization, where the crossing component is inserted into final geometry, where port snapping occurs, and where final verification classifies intersections. Record the audit results in `Surprises & Discoveries` before changing behavior.

Second, audit the current A* cost model and insertion-loss report. Confirm where length, bend penalties, crossing penalties, history costs, congestion costs, and dynamic-conflict penalties enter route selection. Record whether the current `total_cost` already corresponds to route-induced insertion loss or whether it mixes physical loss with non-physical search guidance.

Third, define the smallest verification report that is useful for crossing work. It should distinguish successful legal crossings from search failures, illegal bend or near-bend crossings, unexpected route-route intersections, self-intersections, metadata/realization mismatches, and port-snapping protected-segment movement. Prefer extending existing verification structures over adding a parallel reporting system. The report should be available as structured data for tests and written as JSON under `build/` for benchmark runs. It should include insertion-loss terms: length loss, bend loss, crossing loss, total physical route-induced insertion loss, and any non-physical search penalties separately.

The report should also include crossing-component realization fields: component
name or factory, footprint/bbox used for legality checks, intended crossing
points, realized crossing placements, and any missing or mismatched component
issues.

Fourth, add focused regression tests around geometry classification. These tests should use small deterministic fixtures before full benchmarks. At minimum, include one legal straight-straight crossing, one illegal bend-adjacent crossing, one unexpected intersection, and one port-snapping or realization mismatch if the current code has a natural seam for that fixture.

Fifth, run `benes_4x4` and `benes_8x8` with crossing enabled and debug artifacts. The acceptance criterion is not merely a written GDS; final verification must report that crossings are legal and that no illegal intersections remain. If either benchmark fails, preserve the artifact paths and classified failure in this plan.

Sixth, only after the smaller benchmarks have classified evidence, resume `multiportmmi_8x8`. Use the same harness to decide whether the current blocker is crossing search, route repair, realization, port snapping, or final geometry verification.

## Concrete Steps

Work from the repository root:

    cd C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter

On Linux, use the local checkout path instead. Do not hard-code the historical
Ubuntu path in new subagent prompts.

Start with a status check:

    git status --short

Audit likely crossing and verification code:

    rg -n "crossing|snap|verify|intersection|realiz" translation src tests

Run focused existing tests before changing behavior, if the Python extension is built:

    PYTHONPATH=. .venv/bin/pytest -q tests/test_realized_crossing_verification.py tests/test_photonic_verification.py tests/test_route_rust_geometry.py

Windows PowerShell equivalent:

    $env:PYTHONPATH='.'; .\.venv\Scripts\pytest.exe -q tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_route_rust_geometry.py

Run Rust checks around crossing and route geometry:

    cargo test crossing
    cargo test route_sanity --lib

After implementing harness or verification changes, run the narrow tests that cover the new behavior first. Then rebuild the Python extension if Rust/PyO3 behavior changed:

    .venv/bin/maturin develop --release

Windows PowerShell equivalent:

    .\.venv\Scripts\maturin.exe develop --release

Then run the relevant Python tests again.

For benchmark evidence, use `BROWSER=/bin/true` so debug output does not open many browser tabs. The exact crossing flags must be confirmed from `routing_flow.py` before running, but the command shape should be:

    BROWSER=/bin/true .venv/bin/python routing_flow.py benes_4x4 --crossings true --debug-timing true
    BROWSER=/bin/true .venv/bin/python routing_flow.py benes_8x8 --crossings true --debug-timing true

Windows PowerShell equivalent:

    .\.venv\Scripts\python.exe routing_flow.py benes_4x4 --crossings true --debug-timing true
    .\.venv\Scripts\python.exe routing_flow.py benes_8x8 --crossings true --debug-timing true

On Windows, note whether debug SVG generation opens browser tabs; the current
script opens generated SVGs automatically when debug SVGs are enabled.

Record the exact commands, outputs, and artifact paths here after they are run.

During the QA harness slice on 2026-07-10, only lightweight syntax/smoke checks
ran because the project toolchain was unavailable:

    C:\Users\benja\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -m py_compile translation\crossing_verification_report.py tests\test_crossing_verification_report.py

This command exited successfully.

    C:\Users\benja\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -c "<direct import smoke for translation/crossing_verification_report.py>"

This command printed `True 1`, proving the pure report builder can produce a
successful report with one legal crossing and a matching component. A first
direct-import smoke attempt failed because the temporary module was not inserted
into `sys.modules`, which dataclasses requires; this was a smoke harness issue,
not a repository code issue, and the corrected smoke passed.

During the routing-flow wiring slice on 2026-07-10, lightweight checks ran with
Codex's bundled Python:

    C:\Users\benja\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -m py_compile routing_flow.py translation\crossing_verification_report.py tests\test_crossing_verification_report.py tests\test_routing_flow_stats.py

This command exited successfully.

    git -c safe.directory=C:/Users/benja/Documents/Repositorys/TUMPhotonicRouter diff --check

This command exited successfully, with only existing Git line-ending warnings
about LF being replaced by CRLF when Git next touches the files.

During the setup convergence slice on 2026-07-10, the following Windows setup
commands established a working local environment:

    C:\Users\benja\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -m venv .venv

The first dependency install attempt failed because of the Windows path-length
limit. The successful dependency install used a temporary short drive mapping:

    subst T: C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter
    T:\.venv\Scripts\python.exe -m pip install pytest maturin gdsfactory pyyaml
    subst T: /D

Rust was installed through rustup, then a non-MSVC host toolchain was installed
because `link.exe` and Visual Studio Build Tools were absent:

    curl.exe -L https://win.rustup.rs/x86_64 -o %TEMP%\rustup-init.exe
    %TEMP%\rustup-init.exe -y --profile minimal --default-toolchain stable
    C:\Users\benja\.cargo\bin\rustup.exe toolchain install stable-x86_64-pc-windows-gnullvm --profile minimal

Use this environment in Windows PowerShell for Rust and maturin commands in this
checkout:

    $env:PATH = 'C:\Users\benja\.cargo\bin;' + $env:PATH
    $env:RUSTUP_TOOLCHAIN = 'stable-x86_64-pc-windows-gnullvm'
    $env:RUSTFLAGS = '-C linker=C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin\rust-lld.exe -C target-feature=+crt-static'
    $env:PYO3_PYTHON = 'C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\.venv\Scripts\python.exe'

With those variables set, these commands succeeded:

    C:\Users\benja\.cargo\bin\cargo.exe check
    .\.venv\Scripts\maturin.exe develop --release

For Rust test executables, add Python and gnullvm runtime DLL directories to
`PATH` before running `cargo test`:

    $runtime = 'C:\Users\benja\.cache\codex-runtimes\codex-primary-runtime\dependencies\python;C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\.venv\Scripts;C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\bin;C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin;C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\lib;'
    $env:PATH = $runtime + $env:PATH

The focused validation packet passed:

    $env:PYTHONPATH='.'
    $env:MPLCONFIGDIR=(Resolve-Path .\.mplconfig).Path
    .\.venv\Scripts\python.exe -m pytest -q tests\test_crossing_verification_report.py tests\test_routing_flow_stats.py::test_run_routing_flow_writes_crossing_verification_report tests\test_rust_backend_import.py

This command reported `16 passed in 3.40s`.

    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_route_rust_geometry.py

This command reported `13 passed in 3.67s`.

    C:\Users\benja\.cargo\bin\cargo.exe test crossing

This command reported `17 passed`; integration test binaries filtered out all
matching tests. The historical command `cargo test route_sanity --lib` also
exited successfully, but this branch currently has no tests matching
`route_sanity`, so it should not be treated as behavioral evidence.

The benchmark evidence commands were:

    $env:PYTHONPATH='.'
    $env:MPLCONFIGDIR=(Resolve-Path .\.mplconfig).Path
    $env:PYTHONIOENCODING='utf-8'
    .\.venv\Scripts\python.exe routing_flow.py benes_4x4 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false
    .\.venv\Scripts\python.exe routing_flow.py benes_8x8 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false

The pre-placement `benes_4x4` run completed in `0.3215 s` total and wrote
`build\verification\benes_4x4_crossing_verification.json`. The report has
`legal_crossing_count=2`, `illegal_crossing_count=0`, and two
`missing_crossing_component` errors.

The pre-placement `benes_8x8` run completed in `25.6607 s` total and wrote
`build\verification\benes_8x8_crossing_verification.json`. The report has
`legal_crossing_count=16`, `illegal_crossing_count=0`, and sixteen
`missing_crossing_component` errors.

After component placement and review fixes were implemented, the same commands
produced green reports: `benes_4x4` completed in `0.2990 s` total with
`success=True`, `legal_crossing_count=2`,
`matched_crossing_component_count=2`, and `issues=0`; `benes_8x8` completed in
`24.7861 s` total with `success=True`, `legal_crossing_count=16`,
`matched_crossing_component_count=16`, and `issues=0`.

## Validation and Acceptance

The verification foundation milestone is accepted when:

- Focused tests classify legal and illegal crossing fixtures deterministically.
- Final-geometry verification reports legal crossings separately from illegal intersections.
- Benchmark runs write a structured JSON verification report under `build/`.
- The report separates physical route-induced insertion loss from non-physical search guidance costs.
- The report records the PDK/gdsfactory crossing component footprint and flags missing or misplaced realized crossing components.
- `benes_4x4` and `benes_8x8` have recorded pass/fail evidence with exact commands and artifact paths.
- If a benchmark fails, the failure is classified as search, route geometry, crossing legality, realization, port snapping, PLM interaction, or final verification.
- Existing path-length-matching regression tests relevant to heater benchmarks still pass or any failure is explicitly documented as unrelated pre-existing worktree state.
- The `lidar-pure` / router-discovered path does not consume topology-precomputed crossing hints.

This milestone does not require `multiportmmi_8x8` to route completely. It requires the harness to be strong enough that `multiportmmi_8x8` failures become actionable.

## Idempotence and Recovery

The audit and test commands are safe to rerun. Debug benchmark commands write under `build/`; generated artifacts may be overwritten unless copied to a timestamped or descriptive path. Do not delete existing diagnostic artifacts unless the active task explicitly requires cleanup.

The worktree currently contains unrelated modified and untracked files from earlier routing work. Do not revert them. If a file already has user or prior-agent changes, read it carefully and make only the edits needed for this plan.

## Artifacts and Notes

No validation artifacts have been generated by this plan yet.

## Interfaces and Dependencies

Prefer existing verification and route-record APIs. If new data is needed, keep it explicit and narrow:

- route records should preserve intended crossing points and protected route segments;
- realization should expose enough centerline information to compare intended and realized geometry;
- crossing realization should use the active PDK/gdsfactory crossing component and expose its footprint/bbox to verification;
- verification should return structured failures that tests can assert on;
- benchmark commands should emit artifact paths, summary status, and JSON verification reports.
- route-cost reporting should expose length, bend, crossing, total physical insertion loss, and separate search-guidance penalties.

Avoid a new broad orchestration framework until verification and active task planning are stable.

Revision note, 2026-07-10 11:41Z: Updated the plan after the Planner /
Technical Lead audit. The audit stayed read-only because the local Windows
toolchain is missing. The update records the crossing, realization, endpoint
correction, final verification, and A* cost findings, then points the next step
to QA / Harness Engineer report and fixture design.

Revision note, 2026-07-10 11:55Z: Added the QA / Harness Engineer report
contract and focused report fixtures. This revision records the new files,
the lightweight syntax/smoke evidence that could run without the project
toolchain, and the remaining need to wire the report into benchmark output and
run full pytest/cargo validation once the environment exists.

Revision note, 2026-07-10 12:23Z: Added the routing-flow wiring pass. Benchmark
runs with crossings enabled now write `build/verification/*_crossing_verification.json`
and store the structured report in component info. The new harness test uses a
fake route result to keep this coverage independent of the unavailable local
Rust/Python project toolchain.

Revision note, 2026-07-10 13:14Z: Recorded setup convergence and benchmark
evidence. The Windows checkout now has a project `.venv`, Python dependencies,
Rust via rustup, a working gnullvm/rust-lld maturin build, focused passing
Python/Rust validation, and classified `benes_4x4` / `benes_8x8` verification
JSON reports. The next implementation blocker is physical crossing component
realization.

Revision note, 2026-07-10 18:10Z: Recorded the current `multiportmmi_8x8`
LiDAR-pure debugging slice. Focused route stops now pass through route 68 after
adding LiDAR-pure verifier policies for unexpected router-discovered crossings,
clustered crossing footprints, endpoint/opened-cell diagnostics, and native
repair bookkeeping for committed opened cells. The route-69 stop remains the
active blocker: `n_68` can be committed only by the LiDAR-pure fallback, but
final validation then reports internal non-perpendicular crossings against
`n_65`/`n_66`, while validation-feedback keepouts slide the conflict along the
same vertical channel instead of finding a legal route. This is classified as a
route-geometry/search-repair blocker, not a missing toolchain or report-harness
issue. Latest focused checks passed:

    C:\Users\benja\.cargo\bin\cargo.exe check
    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py

The latest focused route evidence is:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false --debug-stop-after-route 68
    # passed, total about 40.7 s, repairs=7

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false --debug-stop-after-route 69
    # blocked at n_68 with non-perpendicular internal route crossings

Revision note, 2026-07-11 01:15Z: Backfilled workflow changes after user
feedback that the prior debugging loop used the orchestrator as a single large
reasoning window instead of activating the intended Harness/Reviewer pipeline.
Updated `.agent/ORCHESTRATOR.md`, `.agent/WORKFLOW.md`, and role briefs so a
nontrivial routing implementation must produce a verifier packet and receive a
QA / Harness verdict before completion is reported. The next technical role for
new invalid multiport screenshots is QA / Harness Engineer, not implementation.

Revision note, 2026-07-11 01:45Z: Ran the QA / Harness Engineer pass for the
new `multiportmmi_8x8` invalid-geometry screenshots. Verdict: FAIL for the
current verification gate, with artifact freshness caveat. The current
`build\routed_multiportmmi_8x8.gds` is the route-68 debug-stop artifact from
2026-07-11 00:53:34 +02:00, not a full 111-net result. Its photonic verifier
reports `success=true`, `routed_record_count=68`, `expected_route_count=111`,
`cross_net_waveguide_overlap_count=0`, and
`waveguide_obstacle_overlap_count=0`, but the per-net diagnostics still show
static and dynamic overlaps near the photographed fanouts: `n_32` has
`route_static_blocked_overlap_count=8`; `n_34` has
`route_dynamic_overlap_count=2`; `n_51`, `n_52`, `n_53`, and `n_54` have
dynamic overlap counters in the same MMI-adjacent slice. Direct KLayout GDS
inspection found raw-over-union overlap on layer `1/0`
(`65.955 um^2`) and one crossing-footprint overlap on layer `2/0`
(`3.057 um^2`, bbox approximately `(2552.990,648.490)-(2555.010,650.510) um`).
The crossing report also records overlapping footprints under
`allowed_lidar_pure_cluster`, matching the user screenshot of two adjacent
blue crossing components. This means the next implementation loop must first
add deterministic harness coverage for: incomplete partial artifacts being
reported as clean, MMI-adjacent endpoint/opened-cell route-through-static
masks, non-adjacent self/cross-net route overlap in dense fanouts, and
crossing-component footprint spacing/overlap policy.

Subagent verifier addendum: the likely route-under-crossing locations are the
orthogonal crossings around `n_50` with `n_51`, `n_52`, `n_53`, `n_54`, plus
`n_51` with `n_54`, with centers `(2039.6,850)`, `(2039.6,950)`,
`(2039.6,1050)`, `(2039.6,750)`, and `(2023.6,850) um`. The clustered
crossing overlap is around `n_65`, `n_66`, and `n_67`, with centers
`(2555.25,648.25)` and `(2552.75,650.75) um`. A further same-layer route
polygon overlap exists around `(1236.998,708.998)-(1251.656,723.656) um`, but
route identity is inferential until realized polygons carry route labels.

Revision note, 2026-07-11 02:20Z: Implemented the first QA/Harness gate after
the failed screenshot audit. `routing_flow.py` now marks verification JSON with
`status`, `partial`, `debug_stop_after_route_index`, expected/routed/missing
route counts, and `route_coverage_check_enabled`. `translation/photonic_verification.py`
now reports `crossing_component_route_overlap` and `crossing_component_overlap`
from legal crossing footprint polygons. Focused tests were added in
`tests/test_photonic_verification.py` and `tests/test_routing_flow_stats.py`.
Validation evidence:

    .\.venv\Scripts\python.exe -m py_compile routing_flow.py translation\photonic_verification.py tests\test_photonic_verification.py tests\test_routing_flow_stats.py
    .\.venv\Scripts\python.exe -m pytest -q tests\test_photonic_verification.py tests\test_routing_flow_stats.py -q
    .\.venv\Scripts\python.exe -m pytest -q tests\test_crossing_verification_report.py tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_routing_flow_stats.py -q

The bounded `multiportmmi_8x8` route-68 debug-stop command now fails before GDS
write, as intended, with `17` photonic verification errors:
`crossing_component_route_overlap_count=16` and
`crossing_component_overlap_count=1`. The photonic report is now
`success=false`, `status=partial_debug_stop`, `expected_route_count=111`,
`routed_record_count=68`, and `missing_route_count=43`. The old
`build\routed_multiportmmi_8x8.gds` timestamp stayed at
`2026-07-11 00:53:34 +02:00`, confirming the failing verifier did not overwrite
the stale visual artifact. The next role can be Implementer for crossing
realization/clipping and clustered-crossing spacing, but must return to QA /
Harness after each nontrivial routing behavior change.

Revision note, 2026-07-11 03:49Z: Completed an implementer -> QA/Harness ->
reviewer convergence loop for the full `multiportmmi_8x8` LiDAR-pure
benchmark. The loop fixed the concrete visual failure classes from the user
screenshots: endpoint correction failures on output nets, cross-net route
overlap, and third-party routes through realized crossing component footprints.
Key behavior changes now include checked endpoint-correction contact handling,
final crossing and photonic repair loops with structured attempt metadata,
bounded endpoint/opened-cell crossing suppression, owner-only crossing component
footprint exemptions, shared-owner metadata for clustered crossing components,
and native endpoint-bump metadata refresh plus crossing validation. The
authoritative final benchmark evidence is:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_crossing_verification_report.py tests\test_port_alignment_diagnostics.py::test_checked_no_bump_endpoint_correction_allows_active_endpoint_static_contact tests\test_port_alignment_diagnostics.py::test_checked_no_bump_endpoint_correction_rejects_middle_static_contact tests\test_route_rust_realization.py
    # 42 passed in 3.53 s

    C:\Users\benja\.cargo\bin\cargo.exe check
    # passed

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    Copy-Item -LiteralPath target\x86_64-pc-windows-gnullvm\release\photonic_router.dll -Destination python\photonic_router\photonic_router.dll -Force
    # rebuilt and loaded C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\python\photonic_router\_rust.pyd

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-svgs false --debug-timing false
    # passed and wrote build\routed_multiportmmi_8x8.gds at 2026-07-11 05:48:28 +02:00

The final JSON gate passed with
`build\verification\multiportmmi_8x8_photonic_verification.json` reporting
`success=true`, `expected_route_count=111`, `routed_record_count=111`,
`cross_net_waveguide_overlap_count=0`, `crossing_component_route_overlap_count=0`,
`crossing_component_overlap_count=0`, and `waveguide_obstacle_overlap_count=0`.
`build\verification\multiportmmi_8x8_crossing_verification.json` reports
`success=true`, `error_count=0`, `illegal_crossing_count=0`,
`legal_crossing_count=20`, `realized_crossing_component_count=18`,
`matched_crossing_component_count=18`, `final_crossing_repair_attempt_count=2`,
and `final_photonic_repair_attempt_count=4`. The final photonic repair attempts
were endpoint connection repair for net ids `[106, 111]`, cross-net overlap
repair for `[51, 55]`, and crossing-component footprint repairs for `[68]` and
`[89]`; all had empty `endpoint_correction_failed_net_ids`.

QA / Harness and Reviewer pipeline notes: a QA sidecar defined the focused gate
for the prior `n_105`/`n_110`, `n_50`/`n_54`, and `n_88` failures. The first
reviewer flagged stale partial repair state, over-broad opened-cell suppression,
and repair metric ambiguity; the second reviewer flagged stale native metadata
after endpoint bump commits and shared-cluster owner under-exemption. The
implementation loop addressed each actionable finding before the final pass.

Revision note, 2026-07-11 17:01Z: Completed the bounded `n_32` route-37
repair audit after stricter native endpoint/crossing validation. The code now
keeps dynamic clearance exemptions endpoint-local, avoids reopening raw sibling
static keepouts, uses geometry-backed commit/core cells for realized native
routes, persists learned crossing repair keepouts per rip-up set, caps
learned-feedback retries per rip-up set, and at that time temporarily added
bounded route-order neighbor expansion for `No route found` repair failures.
That route-order expansion is now superseded and removed by the 2026-07-12
19:33Z correction below. Focused Rust and Python tests pass, and
`maturin develop --release --target x86_64-pc-windows-gnullvm` rebuilt the
extension. The route-37 gate still fails at `n_32`:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # fails at n_32 / net33

The current blocker is classified as route search/repair convergence, not a
verification blind spot: recent errors include `Illegal realized crossing: net
33 intersects net 36 ... (not_perpendicular)`, `Failed to commit routed cells
to obstacle map`, and `Illegal realized crossing: net 36 intersects net 33 at
(1478.382, 582.007) (not_perpendicular)`. The failure log is
`build\routes\multiportmmi_8x8_n_32_FAILED.txt`. The next implementer should
focus on a native repair strategy that uses a failed victim reroute as a probe
to force an alternate current-route corridor, rather than adding broader port
openings or unbounded keepout growth.

Revision note, 2026-07-12 12:30Z: Ran the next QA / Harness packet for the
route-37 `n_32` cluster after cleanup. A sidecar harness audit returned FAIL
for the current partial artifact and identified a failure-packet blind spot:
repair exhaustion can hide the original illegal-crossing cause. The main lane
rebuilt the Rust extension, ran the bounded route-37 command twice under the
5-minute cap, and confirmed the fresh failure remains `n_32` / native `net33`
with blockers `[36, 31]` and `net33`/`net36` non-perpendicular root causes.
`translation/route_rust.py` now adds `root_cause_illegal_crossings` JSON to
failed route logs by scanning the terminal error and recent attempt errors, and
`tests/test_route_failure_diagnostics.py` covers the `net33`/`net36` packet
shape. This is harness/reporting work only; the router still needs an
Implementation Engineer pass for the native repair strategy.

Revision note, 2026-07-12 13:49Z: Ran an Implementation Engineer repair pass
for the same bounded route-37 `n_32` / native `net33` failure. Two read-only
sidecars were used in parallel: one inspected the native repair loop and
identified symmetric learned keepouts as a likely self-blocking issue, and one
mapped the artifact geometry, confirming `n_32 -> net33`, `n_35 -> net36`,
with `candidate_blockers=[36,31]` meaning `n_35` and `n_30`. The sidecar
geometry packet found `n_32` attempts near
`(706,165)->(731,106)->(759,93)->(767,93)` and `n_35` reroute attempts as a
clean L route `(706,172)->(719,172)->(719,43)->(767,43)`.

Implemented and retained in `src/py_router.rs`:

    - parsed `dynamic_overlap_owners=[...]` from dynamic commit errors;
    - split learned repair keepouts into shared and victim-only maps;
    - when a victim reroute error names the current net, route the learned
      dynamic-overlap or illegal-crossing keepout into the victim-only map;
    - apply victim-only learned keepouts while rerouting victims, and remove
      them before retrying the current net;
    - added a focused Rust fixture proving current-owned dynamic-overlap and
      current-involved illegal-crossing errors do not enter the shared/current
      retry keepout.

Speculative changes tested and removed during the loop:

    - strict orthogonal-only victim reroute fallback, because it shifted the
      packet toward `No route found` without convergence;
    - rerouted-victim core reservation during current retry, because it can
      block a legal perpendicular current crossing of the cleaned victim route.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe test keepout --lib
    # 6 passed with gnullvm/LLVM-MinGW/PYO3_PYTHON environment

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with the same environment

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # still fails at n_32 / net33 inside the 5 minute cap

The latest failure packet is still
`build\routes\multiportmmi_8x8_n_32_FAILED.txt`. It preserves the original
root cause as `net33` x `net36` non-perpendicular at `(1491.500,571.125)` and
`(1491.500,614.500)`, but the recent tail now mostly exhausts on
`repair_failed_net:net33:roundSome(2):rip[36,31,30]:No route found` and
`reroute_victims:net36:roundSome(2):rip[36,31,30]:No route found`. The next
role should be QA / Harness Engineer for a small native repair fixture that
replays the current-first sequence: route current `net33`, reroute victims
`[36,31,30]`, then assert why the otherwise successful victim reroutes do not
produce a committed repair. Do not add another heuristic until that fixture or
trace identifies the exact rejected phase.

Revision note, 2026-07-12 16:52Z: Completed the requested QA / Harness trace
slice for the route-37 `n_32` / native `net33` repair blocker. This is
diagnostic/harness work only; no routing heuristic or cost behavior was changed.
`src/py_router.rs` now emits a native `repair_trace` array from
`route_many_with_repair_and_commit`, with per-phase records for repair mode
start, current-route attempts, victim-reroute attempts, and repair mode result.
Each record includes `route_order` (`current_first` or `victim_first`),
`repair_set_index`, `repair_round`, `candidate_blockers`, `ripup_ids`,
`victim_order`, order flags, success, and error where relevant.

`translation/route_rust.py` now copies the native trace into failed-route logs
as `native_repair_trace_count` plus JSON `native_repair_trace_tail`.
`tests/test_rust_batch_repair.py` asserts the tiny native two-lane repair
fixture exposes current-first repair trace events, and
`tests/test_route_failure_diagnostics.py` covers the trace-tail formatter.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm,
    # CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm, LLVM-MinGW linker,
    # and PYO3_PYTHON=.venv\Scripts\python.exe

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe -m pytest -q tests\test_route_failure_diagnostics.py tests\test_rust_batch_repair.py
    # 4 passed in 3.50 s after the final rerun

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # failed inside the 5 minute cap at n_32 / net33, as expected for this harness slice

The refreshed failure packet is
`build\routes\multiportmmi_8x8_n_32_FAILED.txt`. It now reports
`native_repair_trace_count=141`. The trace tail identifies two concrete
rejected phases for the current blocker: in `current_first` reverse order,
victim `net31` reroutes successfully but victim `net36` then fails both normal
and repair-fallback reroute with `No route found`; in `victim_first` order,
all victims `[36,31,30]` reroute successfully but current `net33` then fails
both normal and repair-fallback routing with `No route found`. The original
root cause remains `net33` x `net36` non-perpendicular crossings at
`(1491.500,571.125)` and `(1491.500,614.500)`.

Local tooling note: `cargo fmt` could not run because `rustfmt` is not
installed for either `stable-x86_64-pc-windows-msvc` or
`stable-x86_64-pc-windows-gnullvm`. `git diff --check` for the touched source
and test files passed.

Revision note, 2026-07-12 19:33Z: User inspection corrected the route-37
cluster diagnosis. The upper 2x2 structure routes (`n_29`/native `net30` and
`n_30`/native `net31`) are not physical participants in the lower 6x6 cluster
failure and should not be repair victims. Source audit verified that
LiDAR-pure crossing search is not purely post-route validation:
`src/astar.rs::crossing_move_outcome` checks candidate crossing moves during
neighbor expansion and rejects non-perpendicular, wrong-order, insufficient
margin, unexpected-owner, unmatched-centerline, and uncleared-footprint cases.
However, the native repair loop then polluted the victim set with route-order
inference: realized crossing blockers dragged in adjacent native jobs, and
`No route found` repair failures could enqueue route-order neighbor victims.

Implemented and retained in `src/py_router.rs`:

    - removed route-order adjacent blocker promotion from realized crossing
      victim selection;
    - removed the `No route found` route-order neighbor repair helper and all
      call sites;
    - removed the old Rust test that documented route-order neighbor expansion
      as expected behavior.

Workflow updates: `.agent/WORKFLOW.md`, `.agent/ORCHESTRATOR.md`, and the
implementer/harness/reviewer role briefs now require geometry-backed repair
victims. Route order, netlist adjacency, and SVG/debug sequence are explicitly
not valid blocker evidence. This is a behavior-correction slice, not yet proof
that the full `n_32` route converges; the route-37 gate must be rebuilt/rerun.

Validation evidence after rebuild:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm,
    # CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm, LLVM-MinGW linker,
    # and PYO3_PYTHON=.venv\Scripts\python.exe

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    C:\Users\benja\.cargo\bin\cargo.exe test targeted_illegal_crossing_repair_promotes_learned_blocker --lib
    # 1 passed

    git -c safe.directory=C:/Users/benja/Documents/Repositorys/TUMPhotonicRouter diff --check -- src/py_router.rs .agent/WORKFLOW.md .agent/ORCHESTRATOR.md .agent/roles/implementer.md .agent/roles/harness.md .agent/roles/reviewer.md .agent/execplans/2026-07-10-crossing-verification-foundation.md
    # passed with line-ending warnings only

Fresh route-stop evidence:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 29-37 --attempt-diagnostics --debug-timing true
    # failed quickly at n_24 / native net25 after route-order widening was removed

The fresh failure packet is `build\routes\multiportmmi_8x8_n_24_FAILED.txt`.
It reports geometry-backed `candidate_blockers=[24]`: the initial failure is a
dynamic overlap with owner `net24` at cell `(629,162)`, and native repair
diagnostics report one grid violation plus three realized violations between
`net25` and `net24` (`not_perpendicular`, `collinear_route_overlap`,
`not_perpendicular`). The repair keepout has `396` cells and both current-first
and victim-first repair modes fail with `No route found`. This proves the
route-order victim pollution is gone, but it also exposes the next real repair
strategy blocker earlier than `n_32`.

Revision note, 2026-07-12 20:31Z: User stepped through the `multiportmmi_8x8`
fan-in routes one GDS at a time and established a clean boundary:

    - `--debug-stop-after-route 25` through `n_24`: fast, clean, partial GDS.
    - `--debug-stop-after-route 26` through `n_25`: fast, clean, partial GDS.
    - `--debug-stop-after-route 27` through `n_26`: fast, clean, partial GDS.
    - `--debug-stop-after-route 28` through `n_27`: fast, clean, partial GDS.
    - `--debug-stop-after-route 29` through `n_28` with
      `--crossings true --crossing-mode lidar-pure`: does not complete in the
      fast window; native trace shows collision/repair against `n_27`.

Diagnostic evidence for the first programming task:

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --debug-timing true --verbose-routes --max-iterations 10000

This fails at `n_28` / native `net29` with `candidate_blockers=[28, 27]`.
The native trace reports `crossing_events=0`, `grid_violations=1`,
`realized_violations=3`, and realized reasons against native `net28`
including `not_perpendicular` and `collinear_route_overlap`. It attempts
`ripup=[28]` and `ripup=[28, 27]`.

Control evidence:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 29 --debug-svgs false --debug-timing true --verbose-routes

This completed in `4.3385 s`, wrote a partial after-`n_28` GDS, and photonic
verification reported `success=true`, `routed_record_count=29`, and
`error_count=0`. Therefore the basic clean route exists; the blocker is
specific to crossing-enabled `lidar-pure` collision/repair selection.

Code audit finding:

    - `route_match_and_realize` and `route_nets_rust` default
      `crossing_loss=0.0`.
    - Rust crossing-aware A* adds only
      `crossing_outcome.crossing_count * crossing.crossing_loss` to step cost.
    - The benchmark CLI currently does not set a nonzero search penalty for
      `lidar-pure`, so crossing candidates can be chosen before a clearly clean
      local route.

Acceptance boundary for the next implementation slice:

    1. `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
       --debug-stop-after-route 29 --debug-svgs false --debug-timing true
       --verbose-routes` completes in the fast window, expected under 15 s on
       this Windows checkout.
    2. The resulting artifact is a partial after-`n_28` GDS with
       `routed_record_count=29`, `debug_stop_after_route_index=29`,
       `error_count=0`, `cross_net_waveguide_overlap_count=0`, and
       `waveguide_obstacle_overlap_count=0`.
    3. `n_28` must not enter native repair/rip-up against `n_27`; verifier
       should check timing/trace output for no `candidate_blockers=[28, 27]`
       and no `ripup=[28, 27]` in this route stop.
    4. Keep physical insertion-loss accounting distinct from any search-only
       crossing penalty. Do not turn a search penalty into reported physical
       crossing loss.

Planned role loop:

    - Implementation Engineer: make the smallest routing-cost/selection change
      that makes `lidar-pure` prefer the clean route before collision-crossing
      repair. Likely file scope is `translation/route_rust.py`,
      `routing_flow.py`, `src/py_router.rs`, and/or `src/astar.rs`, plus a
      focused test if practical.
    - QA / Harness Engineer: independently inspect the patch and rerun the
      bounded stop-after-29 acceptance command, plus any focused unit tests.
      Return `PASS`, `FAIL`, `BLOCKED`, or `INCONCLUSIVE`.
    - Orchestrator: integrate worker output, record verifier verdict, and loop
      implementation only if the verifier returns actionable failure evidence.

Revision note, 2026-07-13 10:25 +02:00: User clarified that the port-side
straight should not mean physically forcing every net to route farther before
bending; instead every port should reserve a short straight access runway that
other nets cannot occupy. Audit showed the MMI fan-in bug was overlapping dense
same-instance port openings: `n_27` could reopen and occupy cells that belonged
to the adjacent `n_28` MMI access region.

Implemented in `translation/route_rust.py`:

    - dense same-instance endpoint openings now use exclusive nearest-lateral
      port ownership for access cells;
    - west-facing/east-facing MMI ports no longer share boundary/opening cells
      merely because their access rectangles geometrically overlap.

Focused regression:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_route_rust_opened_cells.py -k "same_instance_port_access or foreign_port_keepout"
    # 4 passed, 13 deselected

Acceptance reruns:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 29 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=21/29, failures=0, repairs=0

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=21/29, failures=0, repairs=0

Verdict for the stop-after-29 programming task: `PASS`. The clean boundary now
extends through `n_28` without native repair/rip-up in both no-crossing and
`lidar-pure` modes. Continue with the next cluster only after preserving this
boundary as a regression gate.

Revision note, 2026-07-13 10:40 +02:00: The clean boundary was extended through
`n_30` (`debug_stop_after_route_index=31`) and preserved as a regression.

Manual boundary checks:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 31 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; simple=23/31, failures=0, repairs=0, total=4.6623 s

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 31 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; simple=23/31, failures=0, repairs=0, total=5.5062 s

Regression:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_multiportmmi_benchmark.py::test_multiportmmi_8x8_routes_cleanly_through_first_mmi_fanin_boundary
    # 2 passed in 12.68 s

The regression asserts both modes route cleanly through route 31 with
`routed_record_count=31`, zero photonic errors, `route_attempts=31`,
`route_failures=0`, and `repair_count=0`.

Revision note, 2026-07-13 11:35 +02:00: `n_31` / native `net32` now routes
through the first cluster crossing with `n_32` / native `net33` without the
long endpoint-correction-induced detour.

Implementation changes:

    - Guided collision-crossing acceptance now uses realized/endpoint-aware
      crossing events in `src/py_router.rs`.
    - A guided crossing candidate rejected by realized validation now adds a
      targeted keepout around the rejected crossing point and retries locally
      before any victim rip-up.
    - The localized keepout fallback no longer masks invalid realized crossing
      candidates in collision/lidar-pure mode.
    - Crossing-enabled runs disable checked grid endpoint correction for now,
      because endpoint correction must not move/check a crossing after A* has
      selected it.
    - Realized crossing verification now separates the actual component
      legality margin from the larger A* search guard margin:
      `required_margin_um` is the crossing footprint half-size, and
      `search_required_margin_um` records the conservative search margin.
    - Final photonic verification and realization now respect
      endpoint-correction-disabled mode so this debug path does not fail merely
      because corrected centerlines are absent.

Focused acceptance:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.8726 s; GDS written

Key observed behavior:

    - `native_repair_probe net=32 ... candidate_blockers=[33]`
    - `native_repair_guided_crossing net=32 partners={33} events=1`
    - accepted waypoints:
      `(706,162)->(709,162)->(709,154)->(722,141)->(755,174)->(755,235)->(763,243)->(767,243)`

Reports:

    - `build\verification\multiportmmi_8x8_crossing_verification.json`:
      `error_count=0`, `crossing_count=1`, `legal_crossing_count=1`,
      `illegal_crossing_count=0`; crossing `n_31` x `n_32` at
      `[1442.5, 662.125]`, `required_margin_um=4.0`,
      `search_required_margin_um=14.0`.
    - `build\verification\multiportmmi_8x8_photonic_verification.json`:
      `success=true`, `error_count=0`, no route/obstacle overlap errors.

Regression:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py -q
    # 31 passed

Revision note, 2026-07-13 12:47 +02:00: Fix 1 implemented crossing-aware
endpoint correction after crossing discovery.

Implementation changes:

    - `route_match_and_realize` no longer applies checked endpoint correction
      inside native route bookkeeping when crossings are enabled.
    - A new crossing-aware endpoint pass runs after `realized_intersections`
      exists.
    - Crossing-free nets keep the normal endpoint-corrected centerline.
    - Nets with inserted legal crossings preserve the primitive-realized
      crossing-bearing interior. Terminal anchors are attempted only when the
      resulting centerline can be realized by the terminal-tangent polygon
      builder; otherwise the pass falls back endpoint-by-endpoint and finally
      to the frozen primitive centerline.
    - `routing_flow.py` re-enables endpoint correction for crossing runs; the
      crossing-aware pass now decides per net.

Focused validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.3173 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`.

Follow-up, 2026-07-14: Promoted post-A* realized crossing rejection to a
hard blocker.

User clarification:

    - If A* finds and accepts a crossing route, but the later realized/Python-
      side crossing validation says one of those crossings is illegal, this
      must not silently trigger the normal fallback route process or become an
      ordinary repair signal.
    - That case means the router's search model and realized verification
      model disagree. It should stop with an explicit error.

Fix:

    - Changed collision-crossing helper return types in `src/py_router.rs`
      from `Option<(RouteResult, Vec<CrossingEvent>)>` to
      `Result<Option<(RouteResult, Vec<CrossingEvent>)>, String>`.
    - Normal "no collision-crossing route found" remains `Ok(None)`.
    - If crossing events satisfy the requested partner constraints but
      `crossing_violations_for_route_with_ports` reports realized violations,
      the helper now returns a hard error:
      `Realized crossing validation failed for A*-accepted crossing route...`.
    - Updated single-route and native batch repair call sites to propagate that
      error instead of falling through to detour/fallback routing.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe check --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs false --debug-timing false --attempt-diagnostics
    # passed; clean through n_34

    $env:PHOTONIC_ROUTER_TRACE_CROSSING_NET='36'
    $env:PHOTONIC_ROUTER_TRACE_PARTNER_NET='35'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 36 --debug-svgs false --debug-timing false --attempt-diagnostics
    # failed intentionally with
    # RuntimeError: Realized crossing validation failed for A*-accepted crossing route ...

    cargo fmt was attempted but could not run because `rustfmt` is not
    installed for the local Windows Rust toolchains.

Follow-up, 2026-07-14: Added symmetric diagonal halo collision detection.

User clarification:

    - A one-cell diagonal cannot only reserve/detect the previously implemented
      single adjacent lane. It needs a compact adjacent lane on both sides.
    - The purpose is Layer-1 collision detection only. If A* detects such a
      collision, the normal crossing legality logic still decides whether the
      move is legal, illegal, or should be rejected as non-perpendicular.

Fix:

    - Updated `compact_diagonal_halo_cells` in `src/astar.rs` to return both
      compact adjacent lanes for each diagonal unit step.
    - Added regression
      `crossing_move_rejects_parallel_diagonal_on_mirrored_halo_side`.
      The test commits one diagonal, attempts to place a parallel one-cell-
      offset diagonal on the previously unprotected side, and expects A* to
      reject it as `crossing_reject_not_perpendicular` instead of treating it
      as free space.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe check --target x86_64-pc-windows-gnullvm
    # passed

    C:\Users\benja\.cargo\bin\cargo.exe test --target x86_64-pc-windows-gnullvm --no-run crossing_move_rejects_parallel_diagonal_on_mirrored_halo_side
    # passed compile/no-run

    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs false --debug-timing false --attempt-diagnostics
    # passed; clean through n_34

    $env:PHOTONIC_ROUTER_TRACE_CROSSING_NET='36'
    $env:PHOTONIC_ROUTER_TRACE_PARTNER_NET='33'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 36 --debug-svgs false --debug-timing false --attempt-diagnostics
    # still hard-fails as intended with realized crossing validation mismatch

Remaining observation:

    - The symmetric halo is correct and covered, but it does not by itself solve
      the current `n_35` x `n_32` hard-fail.
    - Reconstructing the traced candidate shows a direct grid centerline
      interaction near `(738,143)` between the candidate `n_35` route and the
      committed `n_32` centerline. The next implementation step should inspect
      direct crossing event extraction and grid-vs-realized margin consistency,
      not endpoint correction.

Follow-up, 2026-07-14: Established the low-level Layer-1 crossing invariant for
the `n_31`/`n_32` cluster.

Outcome:

    - A* must make the first legality decision for router-discovered crossings.
    - Offset one-cell diagonal contacts are now detected by a compact
      search-time halo lane, then validated against the true centerline
      crossing geometry.
    - In the observed `n_32` probe, A* detects the offset diagonal contact
      against `n_31` and rejects it because the route-side margin is only
      `0.500` cells while `required_margin=5`.
    - Falling through to probe/rip-up after legal-search failure is now
      considered the intended control flow, not a bug by itself.
    - The remaining work is route discovery/repair convergence after Layer 1
      correctly rejects illegal moves.

Clean checkpoint policy:

    - Do not reintroduce debug flags that allow invalid GDS or disable realized
      crossing validation in the stable routing path.
    - If A* accepts a crossing and the Rust/Python realized verifier rejects it
      as illegal, treat that as a blocking model mismatch.
    - Endpoint correction remains post-routing and must not alter crossing
      decisions made by A*.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed before cleanup; rebuilt the extension used by the traced run

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='.mplconfig'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_TRACE_CROSSING_NET='33'
    $env:PHOTONIC_ROUTER_TRACE_PARTNER_NET='32'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --debug-timing false --attempt-diagnostics
    # passed; trace showed diagonal-halo reject-margin events before repair

Note: direct Rust test execution for the new
`crossing_move_detects_offset_diagonal_halo_contact` fixture still fails in
this Windows environment with `STATUS_DLL_NOT_FOUND`, matching earlier Rust
test-run limitations. The test compiles as part of the Rust test binary.

Follow-up, 2026-07-13: corrected split crossed-net terminal handling.

User clarification:

    - A route with one or more crossings is split by those crossings.
    - Endpoint correction must be allowed on both terminal pieces of that same
      net: source port to first crossing, and last crossing to target port.
    - The protected crossing-bearing middle must remain stable.

Fix:

    - Use guarded cut points before the first realized crossing and after the
      last realized crossing, instead of freezing the whole primitive segment
      from the port when the first crossing is on that segment.
    - Preserve the guarded crossing interior while exposing a real editable
      prefix and suffix to the terminal absorber.
    - Prefer route-state endpoint tangent angles for crossed-net terminal
      direction checks so endpoint correction and realization agree.
    - Added a regression proving one split crossed net can absorb source-side
      and target-side endpoint corrections in the same splice.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 5 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 36 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.1749 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`,
      `cross_net_waveguide_overlap_count=0`,
      `waveguide_obstacle_overlap_count=0`.

Follow-up, 2026-07-13: fixed the real `n_31` source-side split prefix.

User observation:

    - The route-33 partial has one legal crossing involving two nets.
    - One crossed net corrected both terminal sides.
    - The other still failed specifically on the source port to first crossing
      side, even though the terminal region had a usable y-straight/diagonal
      structure.

Concrete finding:

    - Inspection of post-correction records showed `n_32` had zero endpoint
      error at both ports.
    - `n_31` still started at primitive point `(1393.5, 687.125)` instead of
      source port `(1391.8, 687.5)`.
    - The corrected prefix compatibility check accepted direction-compatible
      geometry without requiring the prefix to be anchored at the source port.
    - After that was tightened, the absorber could geometrically reach the
      source port but returned a candidate whose first segment followed the
      original bend arc instead of the port tangent, so realization rejected it
      as an unsupported terminal stub.

Fix:

    - Terminal prefixes/suffixes from the normal endpoint-correction path must
      match the relevant source/target port anchor before they are accepted.
    - Absorbed terminal candidates must match the known route/port terminal
      tangent when one is available.
    - When an inserted port-local straight is needed, the solver requires that
      inserted segment to have positive length instead of collapsing it to zero.
    - Added regressions for rejecting unanchored source prefixes and preserving
      a required positive port straight.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 7 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 38 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.6155 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`,
      `cross_net_waveguide_overlap_count=0`,
      `waveguide_obstacle_overlap_count=0`.
    - crossing debug JSON has
      `routes_missing_corrected_centerline_count=0` and
      `illegal_realized_crossing_count=0`.

Follow-up, 2026-07-13: constrained crossed-net endpoint absorption to existing
straight/diagonal runs.

User observation:

    - The previous `n_31` source-side fix still produced visually invalid
      warped bend-like geometry in the GDS.
    - The intended rule is stricter: endpoint correction may modify existing
      straights or 45-degree diagonals, and may insert a final port-facing
      straight when port and net face each other. It may not reshape sampled
      bend-arc geometry.

Fix:

    - `_absorbed_terminal_centerline` now treats only exact axis/45-degree
      segment directions as length-adjustable.
    - Sampled bend-arc segments with other directions are copied unchanged in
      length and direction.
    - The explicit port-local straight remains adjustable, with positive length
      required when it is selected.
    - Added a regression proving a non-axis/non-diagonal bend sample is not
      length-modified during crossed-net endpoint correction.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_does_not_modify_bend_samples tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 8 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_does_not_modify_bend_samples tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 39 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.4628 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`,
      `cross_net_waveguide_overlap_count=0`,
      `waveguide_obstacle_overlap_count=0`.
    - crossing debug JSON has
      `routes_missing_corrected_centerline_count=0`,
      `illegal_realized_crossing_count=0`, and one legal crossing counted for
      both `n_31` and `n_32`.

Route-34 boundary attempt, same full-run ordering:

    - route 32: `n_31`
    - route 33: `n_32`
    - route 34: `n_33`
    - route 35: `n_34`
    - route 36: `n_35`

Command:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true

Result:

    - failed after roughly two minutes before writing a route-34 GDS;
    - failure occurs while routing `n_31` in the 34-route batch context;
    - error: `No repair route found; candidate_blockers=[33, 34]`;
    - recent errors are local collision-crossing failures while ripping
      `[33, 34]`.

Implication:

    - route-33 remains the current clean boundary;
    - including route 34 / `n_33` changes the earlier `n_31` repair context, so
      the next cluster work should inspect the three-net interaction
      `n_31`/`n_32`/`n_33` in the real full-run order.

Follow-up: implemented the crossed-net terminal absorber.

Implementation:

    - Crossed-net correction now solves length deltas on existing terminal
      segment directions instead of importing arbitrary full-route corrected
      geometry.
    - It supports single-segment and two-segment adjustment, covering straight
      and 45-degree diagonal combinations.
    - It retries with one port-local straight direction when the port
      orientation permits a stub.
    - Segment lengths must remain positive and the solved terminal side must
      land exactly on the desired port/frozen crossing-span endpoint.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 35 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.5970 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`.

Follow-up: allow orientation-correct port-local straight stubs.

User clarification:

    - The no-new-geometry rule was too strict.
    - A crossed-net endpoint correction may insert a small straight at the port
      if its orientation is correct.
    - The rest of the corrected terminal side must still preserve the original
      route direction sequence; arbitrary inserted mid-access bends remain
      rejected.

Fix:

    - Allow one extra segment at the source-side prefix start when it matches
      the source port orientation.
    - Allow one extra segment at the target-side suffix end when it points into
      the target port.
    - Require the remaining segment directions to match the primitive terminal
      side.
    - Added a regression for an accepted source-side port stub.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 35 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.8196 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`.

Follow-up: constrained crossed-net endpoint correction to existing terminal
straight topology.

User clarification:

    - Endpoint correction should lengthen/shorten existing straights inside the
      allowed terminal region.
    - It should not insert new geometry away from the port in that region.
    - If no existing straight can absorb the correction, the future fallback
      should be a valid bump directly at the port rather than a mid-access
      detour.

Fix:

    - Added a segment-direction sequence check for crossed-net corrected
      prefix/suffix geometry.
    - Accept a corrected side only when the compressed direction sequence
      matches the corresponding original primitive side.
    - Reject newly inserted bends in the terminal region and fall back to the
      primitive side for now.
    - Added regressions for rejecting inserted mid-access geometry and allowing
      same-topology straight lengthening.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 34 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.4426 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`.
    - crossing JSON: one legal perpendicular `n_31` x `n_32` crossing at
      `[1443.5, 663.125]`.

Follow-up, 2026-07-13: Fixed a missed no-crossing endpoint correction case
found by GDS inspection.

Root cause:

    - Crossing-enabled route recording stores primitive-realized centerlines in
      `corrected_centerline_um` so crossing verification/insertion use the
      same geometry as GDS realization.
    - The crossing-aware endpoint pass sent crossing-free nets through
      `apply_port_endpoint_corrections`, which skips records that already have
      `corrected_centerline_um`.
    - Therefore crossing-free nets kept the primitive baseline and did not get
      normal endpoint correction.

Fix:

    - Clear the primitive baseline before invoking normal endpoint correction
      for nets with no inserted legal crossing.
    - Keep crossed-net behavior unchanged: the primitive baseline remains the
      protected crossing-bearing interior.
    - Strengthen the crossing-free regression so it starts with a primitive
      centerline already present and verifies that normal correction replaces
      it.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 2 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.7583 s; GDS written

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

Follow-up: refined crossed-net endpoint splice to preserve the whole crossing
segment span.

User clarification:

    - For nets with crossings, endpoint correction may only affect the region
      before the first crossing and after the last crossing.
    - The previous crossed-net splice preserved crossing points, but cutting at
      the crossing center could make the crossing a segment endpoint and cause
      the verifier to report adjacent contact instead of a legal crossing.

Fix:

    - Sort legal crossing points by arclength on the primitive-realized
      baseline.
    - Freeze the full primitive segment span containing the first through last
      crossing.
    - Splice the normally endpoint-corrected prefix before that frozen span and
      suffix after it.
    - Keep crossing-free nets on normal endpoint correction.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.1051 s; GDS written

Report checks:

    - crossing report: `issue_count=0`, `legal_crossing_count=1`,
      `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
    - photonic report: `issue_count=0`.
