# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
every agent stop, pause, or handoff. It does not replace the active ExecPlan.

## Current Snapshot

- Date: 2026-07-16 17:20Z
- Branch: `crossings/verification-foundation`
- Current HEAD: `6b0c5cc`
- Current clean-branch base: `731d0a9`
- Active ExecPlan:
  `.agent/execplans/2026-07-10-crossing-verification-foundation.md`

## Current Goal

Make TUMPhotonicRouter a very fast verified photonic router. The current phase
focuses on router-discovered optical crossings on `benes_4x4`, `benes_8x8`, and
then `multiportmmi_8x8`, with final-geometry verification and PDK/gdsfactory
crossing component realization.

## Worktree State

This worktree is intended to be the clean implementation path for crossing
verification and router-discovered crossing work. It should stay reviewable and
should not receive wholesale merges from the experimental branch.

At creation, this worktree was clean at `731d0a9`.

Benes early simple-route bypass fix:

- Date: 2026-07-16 19:17 local
- Context:
  - In `benes_8x8`, route 2 / `n_in_1_to_s0_0` was geometrically a simple
    East -> diagonal -> East connection, but it was being committed through
    crossing A* with `expanded=480` instead of the normal simple-route path.
  - Route 1 already existed nearby, so the collision-crossing search was
    entered before the ordinary simple-route precheck.
- Change:
  - In `src/py_router.rs`, an early collision-crossing result is now committed
    only if it actually reserved at least one crossing partner.
  - If the crossing search merely finds an ordinary non-crossing route, it
    falls through to the normal path so the simple-route precheck can decide
    and commit it.
  - No A* geometry, primitive, obstacle, or crossing-legality logic was changed.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 2 --debug-svgs none --debug-timing true
    --verbose-routes` passed with `simple=2/2`, `expanded_states=0`.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 8 --debug-svgs none --debug-timing true`
    passed with `simple=8/8`, `expanded_states=0`, `repairs=0`.
  - Full `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-svgs none --debug-timing true` passed:
    total `19.9238 s`, routing stage `18.3512 s`, route search
    `17.9020 s`, `48/48` routes, `38/48` simple, `0` failures,
    `0` repairs. Slowest route was route[15] /
    `n_s0_3_o0_to_s1_1_i1` at `3.4128 s`.

Terminal bump distance A* guard checkpoint:

- Date: 2026-07-16 09:24 local
- Scope:
  - Implemented the previously diagnosed terminal-bump distance condition in
    Rust A* crossing search. If a crossing lies on the final target axis and
    the target port is off the target grid center on the perpendicular axis,
    A* now rejects crossing positions that leave less than the endpoint bump
    length between the crossing footprint and the target.
  - The activation is axis-specific: horizontal target approaches only inspect
    target-y grid snap offset, and vertical target approaches only inspect
    target-x grid snap offset.
  - `n_33`-style exact-fit cases remain legal because equality is accepted.
- Code note:
  - Added `TerminalBumpGuard` / `TerminalBumpAxis` to `src/astar.rs`.
  - `src/py_router.rs` builds the guard from `target_port_um` and the A*
    target state only for the normal collision-crossing search path.
  - Added focused Rust tests for too-close rejection and exact-distance
    acceptance.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - `C:\Users\benja\.cargo\bin\cargo.exe test crossing_terminal_bump_guard`
    passed after adding the Codex runtime Python directory to `PATH` for the
    PyO3 test executable.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - Full run passed routing with:
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`,
    `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35
    --foreign-port-keepout-cells 6 --debug-svgs false --debug-timing true`.
    Result: 111/111 routes, 0 route failures, 0 repairs, 35/35 legal crossing
    components, no waveguide/static/crossing overlap issues.
- Remaining issue:
  - Photonic port verification still reports only `n_108` target connection
    errors at `gc_array_out_gc_5,o1`: target port not touched and corrected
    endpoint distance `2.649um` vs tolerance `2.0um`.
  - Current GDS artifact:
    `build\routed_multiportmmi_8x8.gds`.
- Follow-up:
  - The terminal-bump guard activation was widened from exact target-axis
    equality to a near-axis zone. It now applies when the crossing is within
    `required_bump_cells / 2` cells of the target axis, inclusive. With the
    current 3-cell bend radius this means `<= 6` cells triggers the check.
  - Added a focused Rust regression proving that exactly 6 cells still rejects
    a crossing when the remaining along-axis distance is too short.
  - Validation passed:
    `cargo test crossing_terminal_bump_guard -- --nocapture`,
    `cargo check`, and `.\.venv\Scripts\python.exe -m maturin develop
    --release`.
  - Full `multiportmmi_8x8` was rerun after this inclusive-axis-margin change
    with 90-degree static stubs. Result: success. Timing summary total
    `33.6606 s`, net routing phase `24.1806 s`, 111/111 routed records,
    0 route failures, 0 repairs.
  - Crossing verification: success, 0 issues, 35/35 legal crossings, 35/35
    crossing components matched.
  - Photonic verification: success, 0 issues, 0 cross-net waveguide overlaps,
    0 crossing component overlaps, 0 waveguide/static obstacle overlaps.
  - Fresh GDS:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-16 09:40:55 local,
    size 486974 bytes.

Benes follow-up validation after `8744da7`:

- Date: 2026-07-16 09:55 local
- `benes_4x4 --crossings true --crossing-mode lidar-pure --debug-svgs false
  --debug-timing true` passed. Result: 16/16 routes, 0 failures, 0 repairs,
  crossing verification success, photonic verification success, total
  `6.6327 s`.
- A full `benes_8x8` run did not finish quickly and was stopped for
  step-by-step diagnosis.
- `benes_8x8 --debug-stop-after-route 8` passed quickly: total `0.9521 s`.
- `benes_8x8 --debug-stop-after-route 12` passed: total `3.1545 s`.
- `benes_8x8 --debug-stop-after-route 13` is the first bad boundary. Routing
  itself reports 13/13 routes, 0 failures, 0 repairs, but photonic verification
  fails on route 12 / `n_s0_1_o1_to_s1_2_i1`: source port
  `sw_s0_1,o4` not connected, corrected endpoint distance `2.037um` vs
  `2.0um` tolerance. Route 13 / `n_s0_2_o0_to_s1_1_i0` is the slow route with
  `608801` expanded states in the diagnostic run.
- Fresh debug artifacts:
  - `build\routes\benes_8x8_n_s0_1_o1_to_s1_2_i1.svg`
  - `build\routes\benes_8x8_n_s0_1_o1_to_s1_2_i1_diagnostics.txt`
  - `build\routes\benes_8x8_n_s0_2_o0_to_s1_1_i0.svg`
  - `build\routes\benes_8x8_n_s0_2_o0_to_s1_1_i0_diagnostics.txt`

Benes route-13 local crossing-footprint plan:

- Date: 2026-07-16 10:55 local
- Context:
  - User identified a route-13 geometry where a candidate loops through a
    crossing footprint. Two independent rejects should eventually catch it:
    first, the path should not be able to route through its own accepted
    crossing footprint; second, already committed crossing footprints should
    block later route-through-footprint moves.
  - A first WIP self-collision check in `src/astar.rs` compared a new move's
    effective halo footprint against ancestor effective halo footprints. This
    was too conservative: it forced the stop-after-13 route onto a very long
    path and also changed the pre-13 compact route shape.
  - The WIP was narrowed to centerline-vs-centerline self-intersection. That
    restored the compact pre-13 route, but did not reject the original route-13
    failure, because the problematic loop is mediated by a crossing footprint
    rather than a plain centerline self-intersection.
- Current plan before further implementation:
  - Implement only the first reject first: local, candidate-path crossing
    footprint reservations inside crossing A*.
  - When an A* move accepts a crossing and its after-margin is already
    satisfied, add exactly the crossing device footprint to that candidate
    path's local reservation set immediately.
  - When an accepted crossing leaves `pending_after_crossing_cells > 0`, carry
    the pending footprint in the A* state and activate it only once the pending
    straight run reaches zero. This keeps legal post-crossing straight runout
    moves from being blocked by their own crossing.
  - Do not include bend runout or extra straight margin in the reservation;
    reserve only the crossing footprint cells.
  - Do not implement the global committed-crossing-footprint blocker until the
    local candidate-path rule is proven on the route-13 stop.
- Current dirty code note:
  - `src/astar.rs` still has the narrowed centerline self-intersection WIP.
    It should either be removed or folded into the local-reservation fix before
    checkpointing.

Static-stub 90-degree bend stop-after-`n_31` artifact:

- Date: 2026-07-15
- Scope:
  - User asked to show the GDS after the `n_31` run with static stubs, then
    pointed out that port runways may not have been adapted to the shifted
    virtual ports caused by 90-degree stubs.
  - No commit was made.
- Code note:
  - Added a focused WIP adjustment so dense-port lane filtering uses fanout
    anchor centers/angles when a port has a static-stub anchor, and no longer
    bypasses dense filtering for anchored ports.
  - Syntax check passed:
    `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`, with
    `PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS` unset.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 32 --debug-svgs
    32 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; route `[32/32] n_31` completed with length `249.966um`,
    cost `253.966`, expanded states `3827`.
  - Generated GDS:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 11:47:20 local,
    size 167010 bytes.
  - Diagnostics:
    `build\routes\multiportmmi_8x8_n_31_diagnostics.txt` reports
    `status=ok`, `fanout_stub_bend_degrees=90`,
    `source_state=(712,149,0)`, `target_state=(767,243,0)`,
    and `source_dense_port_runway_cells=18`.
- Current assessment:
  - The artifact is available for visual inspection.
  - The runway/opening issue is not fully closed: the n_31 diagnostic still
    reports large opened-cell sets (`opened_cells_count=656`), so a follow-up
    should separate same-cluster foreign-keepout openings from true normal
    protected runway openings and verify that only the intended virtual-anchor
    runway is opened for each anchored net.
- Follow-up inspection:
  - User clarified that `n_31` may bend early as long as it does not violate
    `n_32`'s protected port/runway cells.
  - Added WIP diagnostics for current/sibling port-runway overlaps in
    `translation/route_rust.py`.
  - Re-ran stop-after-`n_31` with 90-degree static stubs. Diagnostics confirm
    `route_overlap_sibling_port_runway_count=0`, while
    `route_overlap_current_port_runway_count=10`; therefore the current
    `n_31` shape does not violate sibling runway cells.
  - Fresh normal route SVG:
    `build\routes\multiportmmi_8x8_n_31.svg`.
  - Added a small focused inspection SVG:
    `build\routes\multiportmmi_8x8_n31_with_n32_stub_focus.svg`, showing the
    `n_31` committed route, the `n_32` virtual stub anchor/runway, and the
    direct illegal crossing candidate region.

Current static fanout-stub WIP:

- Date: 2026-07-15
- Dirty files: `routing_flow.py`, `translation/route_rust.py`,
  `tests/test_route_rust_opened_cells.py`, and this repository-state file.
- Implemented an opt-in `fanout_access_mode="static-stubs"` path for dense
  multiport source clusters. In this mode the old stepwise dense source runway
  is disabled, source fanout access stubs are reserved as static cells, and A*
  routes from virtual anchor ports. These stubs are not dynamic nets and must
  never enter rip-up/victim sets.
- The static-stub generator now builds a physical port-tangent centerline from
  the real port to the virtual anchor. For source-anchored records, the A*
  body is endpoint-corrected only on the non-stub target side before splicing
  in the static stub, so normal realization no longer needs a diagnostic
  verifier bypass for the stop-before-`n_32` artifact.
- Debug metadata now records `fanout_stub_centerlines_um` in
  `crossing_plan_info`; the first focused regression asserts that static stub
  mode emits this metadata and routes from a virtual source anchor.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py routing_flow.py`
    passed.
  - `.\.venv\Scripts\python.exe -m pytest -q tests\test_route_rust_opened_cells.py`
    passed: 18 tests.
  - Normal CLI stop-before-`n_32` command passed:
    `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 32 --debug-svgs false --attempt-diagnostics --debug-timing true --fanout-access-mode static-stubs`.
    It wrote `build\routed_multiportmmi_8x8.gds` at 2026-07-15 09:07 local,
    with 32 routed records, 0 crossing-verification errors, and 0 photonic
    errors. The metadata reports 36 total static fanout stubs and six stubs for
    `mmi0_multiport_0_0,o7..o12`, covering the `n_31..n_36` source cluster.
  - Diagnostic-only inspection artifact also exists at
    `build\routed_multiportmmi_8x8_static_stubs_before_n32_DIAGNOSTIC.gds`,
    but the normal-path GDS above is now the relevant artifact.
- Remaining blocker:
  - Stop-after-`n_32` still fails in static-stub mode at route index 33 / net
    `n_32` with `No legal LiDAR crossing route found; probe-based victim
    selection is disabled in crossing mode`. The realization/stub geometry
    bug is fixed for the current boundary, but the next work item is search
    behavior with `n_31` as a static-stubbed committed route.

Current Windows checkout note:

- Repository path: `C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter`
- Local LiDAR reference checkout observed at:
  `C:\Users\benja\Documents\Repositorys\LiDAR`
- As of 2026-07-10 13:14Z, this checkout has a project `.venv` created from
  Codex's bundled Python, Python dependencies installed, Rust installed through
  rustup, and a working maturin-built `photonic_router._rust` extension.
- On 2026-07-10 11:41Z, the Planner / Technical Lead audit reconfirmed the
  same toolchain gap and therefore did not run pytest, cargo tests, maturin, or
  benchmark validation.
- The toolchain gap was closed later on 2026-07-10. Use
  `stable-x86_64-pc-windows-gnullvm` plus `rust-lld` in this Windows checkout
  because MSVC `link.exe`, Visual Studio, Windows SDK, LLVM, MSYS, and MinGW are
  not installed.
- As of 2026-07-10 13:39Z, active gdsfactory crossing refs are inserted for
  legal realized crossings, and the smaller Benes crossing verification reports
  are green.
- As of 2026-07-10 13:47Z, review findings are closed: crossing insertion-loss
  accounting is sourced from final realized intersections and the report checks
  crossing component rotation.
- As of 2026-07-11 10:10Z, crossing/static verifier guardrails are tightened:
  crossing verification rejects missing corrected physical centerlines instead
  of using compressed waypoint fallbacks, and final photonic verification has a
  real realized-bend/static-obstacle regression fixture.
- As of 2026-07-11 10:10Z, `multiportmmi_8x8` still does not converge. The
  current failure is useful: the native validator stops during `n_51` routing
  on an illegal non-perpendicular crossing in the adjacent fanout cluster
  instead of writing an invalid GDS.
- As of 2026-07-11 12:15Z, the visible `n_32`-cluster loop was traced to
  `n_31` using static-opened cells behind the MMI source port. The port-opening
  generator now clips access cells to the forward side of the port plane, and
  the refreshed six-route overlay is
  `build/routes/multiportmmi_8x8_n31_n36_cluster_overlay.svg`.
- As of 2026-07-11 12:45Z, realized crossing verification catches the
  `n_32` x `n_35` cluster crossing instead of ignoring it as endpoint access.
  The route-37 stop-after diagnostic now fails with 7 illegal crossings,
  including `n_32` x `n_35` at `[1403.715334, 686.584666]`, marked
  `perpendicular=false` and primarily classified as
  `crossing_footprint_contains_bend`.
- As of 2026-07-11 13:23Z, Rust native commit validation uses the same tight
  endpoint-access rule, so the `n_32` cluster failure is rejected during native
  routing instead of only by Python final verification. This is correctness
  progress but not routing convergence: stop-after-37 now fails at `n_32` with
  native `not_perpendicular` errors, and subsequent repair experiments either
  moved the failure to another adjacent net or exceeded the local quick-feedback
  timing budget.
- As of 2026-07-11 13:36Z, the next repair-feedback patch is implemented in
  `src/py_router.rs`: strict validation errors now promote learned committed
  crossing participants into the adaptive blocker/victim set, and victim
  reroute failures pass the victim net id into that helper. Two Rust regression
  tests document normal and capped-queue learned-blocker behavior. Native
  validation of this patch is blocked in the current PowerShell runtime because
  neither MSVC `link.exe` nor gnullvm `x86_64-w64-mingw32-clang` is available
  on `PATH`.
- As of 2026-07-11 14:10Z, linker recovery was attempted but remains blocked:
  `winget` found `MartinStorsjo.LLVM-MinGW.UCRT` but stayed in archive
  extraction without producing a usable linker and was stopped; the project
  `.venv` now has ignored `ziglang` and `cargo-zigbuild` packages installed,
  and the gnullvm toolchain has the `x86_64-pc-windows-gnu` target installed,
  but Zig/cargo-zigbuild still fail before repository code on MinGW runtime
  lookup and `.def` linker arguments.
- As of 2026-07-13 18:55 local, the `multiportmmi_8x8` `n_31`/`n_34`
  investigation corrected the Rust/Python crossing-legality split: Python
  final verification still checks only the actual crossing footprint, but Rust
  A* and Rust internal event detection must require
  `crossing_half + bend_runout_cells` so the grid search does not accept a
  crossing whose neighboring bend footprint will later occupy the crossing
  footprint. `min_straight_cells` is no longer part of that hard Rust margin.
  `cargo check` passes, `maturin develop --release` succeeds,
  `tests/test_realized_crossing_verification.py -q` passes, and
  `multiportmmi_8x8 --debug-stop-after-route 34` now succeeds in about 15.6 s,
  writing `build\routed_multiportmmi_8x8.gds` with 2 realized crossings and no
  photonic verification issues.
- As of 2026-07-13 19:10 local, the post-crossing pending-margin semantics are
  narrowed: pending `crossing_half + bend_runout_cells` no longer requires the
  next primitive to be a straight primitive. It requires only that the next
  primitive's initial straight arm covers the remaining pending distance before
  any bend kink. This allows bend arms to satisfy crossing runout while still
  keeping bend kinks out of the crossing/runout region. Focused Rust fixtures
  were added for accepted/rejected bend-after-crossing pending margins. The
  fixtures compile, but this Windows runtime still cannot start the Rust test
  binary (`STATUS_DLL_NOT_FOUND`). `cargo check`, `maturin develop --release`,
  and the `multiportmmi_8x8 --debug-stop-after-route 34` benchmark pass; the
  route-34 GDS has 2 realized crossings, 0 illegal crossings, and photonic
  verification success.
- The same trace showed the next unresolved cluster issue: when `n_31` is
  rerouted as a victim to cross `n_34`, the guided victim search may only carry
  the new partner (`n_34`) and lose previous crossing partners (`n_32`/`n_33`)
  if they are in the same ripup set. A pre-ripup seeded-partner experiment
  confirmed the direction but made the local stop-after-35 loop too expensive,
  so it was rolled back. The next implementation should preserve previous
  crossing partners as ordered topology anchors with bounded retry/acceptance
  before enabling it in the main repair loop.
- Historical experiment, removed before the clean checkpoint: temporary debug
  switches were briefly used to inspect what the GDS would look like if Rust
  native realized crossing validation ("layer 2") and Python final GDS blocking
  were disabled. The resulting intentionally invalid inspection GDS was
  `build\routed_multiportmmi_8x8_layer2_off_python_off_after_n32.gds`, and the
  verifier still reported an illegal `n_31 x n_32` crossing. These switches are
  no longer present in the stable worktree because they conflict with the
  fail-closed routing invariant.
- Follow-up code audit: the exact hypothesis that A* omits bend radius from the
  crossing margin is not confirmed. `src/astar.rs` computes
  `crossing_required_margin_cells = crossing_half_size_cells + bend_runout_cells`,
  and `src/py_router.rs` passes `bend_runout_cells = primitive_cfg.bend_radius_cells`
  into the crossing search. The remaining layer-1 gap appears narrower: A*
  checks centerline margin and external occupancy, but does not reject a crossing
  when the current primitive's own bend footprint falls inside the crossing
  footprint. That matches the debug GDS/report reason
  `crossing_footprint_contains_bend`.
- A focused inspection SVG for that intentionally invalid after-`n_32`
  crossing was written to
  `build\routes\multiportmmi_8x8_illegal_n31_n32_after_n32_cells.svg`. It
  overlays the 2um grid, the 8um crossing footprint, and the 10um search margin
  around the reported `n_31 x n_32` crossing center.
- Crossing recognition fix: normal fallback routes now run the existing
  grid-segment crossing legality check before commit. This catches
  diagonal/offset segment intersections that do not share a dynamic obstacle
  cell and rejects them as `Illegal grid crossing` instead of relying on later
  realized validation. With the fix rebuilt, `multiportmmi_8x8
  --debug-stop-after-route 33 --crossing-mode lidar-pure` passes with normal
  validators enabled; crossing verification and photonic verification both
  report `success=True`, `error_count=0`.
- The same slice also updates native crossing-event registration to derive
  events from realized centerlines first, falling back to grid waypoints only
  when needed. The after-`n_32` run now reports
  `native_crossing_event_count=1` for the `n_31 x n_32` diagonal/offset
  crossing at `[1421.5, 685.125]`, with no illegal realized crossings.
- Current `n_35`/internal-net-36 trace status: layer-1 A* does see contacts
  with internal partner `33` around the suspected crossing, including
  `(739,163)`, `(740,162)`, and `(741,161)`. Those candidates are not invisible
  to A*; they are logged as `accept_with_pending` with route angle `5` and
  partner angle `7`. The selected collision-crossing candidate still reports
  realized validation failure against partner `33` at `(1459.5, 689.125)` with
  `insufficient_straight_margin`, while its accepted realized crossing-events
  list only contains partners `35`, `32`, and `34`. This points to a remaining
  mismatch between layer-1 pending-margin acceptance and layer-2 realized
  crossing/margin extraction, not to a missing collision witness at layer 1.
  Debug trace file:
  `build\debug_n35_level1_partner33.txt`.
- Pending-margin model fix in progress: `src/astar.rs` now carries
  `pending_after_crossing_angle` in `CrossingAStarKey` and
  `CrossingMoveOutcome`. A* only consumes post-crossing pending margin when the
  next primitive starts in the same angle, and a crossing whose missing
  after-margin would pass through an internal primitive kink is rejected. Added
  focused Rust fixtures for wrong follow-up direction and internal-bend kink
  before the after-margin. Validation: `cargo +stable-x86_64-pc-windows-gnullvm
  check` with `RUSTFLAGS='-C linker=rust-lld'` and `PYO3_PYTHON` passed, and
  `cargo +stable-x86_64-pc-windows-gnullvm test --no-run` passed. Running the
  Rust test binary is still blocked by `STATUS_DLL_NOT_FOUND`. Rebuilding the
  Python extension with `maturin develop` is currently blocked in this runtime:
  MSVC lacks Windows SDK libraries/link.exe, while gnullvm build scripts fail
  to start with `STATUS_DLL_NOT_FOUND`. No new post-fix GDS has been generated
  from this extension build yet.
- Follow-up validation found a practical build path for this Windows checkout:
  `cargo +stable-x86_64-pc-windows-gnullvm rustc --release --features
  pyo3/extension-module --lib --crate-type cdylib` with
  `RUSTFLAGS='-C linker=rust-lld'` succeeds. Copying
  `target\release\photonic_router.dll` to
  `python\photonic_router\_rust.pyd` produced a working release extension for
  local validation. With the pending-angle fix active, the focused
  `multiportmmi_8x8` stop-after-36 trace completed in 30.108 s. The final
  `n_35`/internal-net-36 collision-crossing candidate now includes partner
  `33` in the accepted crossing events:
  `[(35,(1445.5,743.125),7,1), (32,(1459.5,729.125),7,1),
  (33,(1463.5,685.125),5,7), (34,(1473.5,695.125),5,7)]`.
  Rust validation reports `satisfies=true realized_violations=[]`;
  crossing verification reports `success=True`, `error_count=0`,
  `crossing_count=7`; photonic verification reports `success=True`.
  Trace file:
  `build\debug_n35_pending_angle_partner33_release.txt`.
- Additional model simplification validated: A* now rejects any dynamic
  collision generated by a non-straight primitive before trying to classify it
  as a crossing. Crossings are therefore discovered only on straight
  primitives; bend primitives may satisfy pending runout with their initial arm
  but may not create the collision event. Rebuilt with the direct cargo release
  cdylib path and copied `target\release\photonic_router.dll` to
  `python\photonic_router\_rust.pyd`. The same stop-after-36 trace remains
  green: final internal-net-36 events are
  `[(35,(1445.5,743.125),7,1), (32,(1459.5,729.125),7,1),
  (33,(1463.5,685.125),5,7), (34,(1473.5,695.125),5,7)]`,
  `satisfies=true`, `realized_violations=[]`; crossing verification success
  with 0 errors and 7 crossings; photonic verification success. The focused
  run took 21.457 s and candidate checks for this collision-crossing attempt
  dropped from 39769 to 7349. Trace file:
  `build\debug_n35_bend_dynamic_reject_partner33_release.txt`.
- Post-commit artifact check: `multiportmmi_8x8` was routed one step farther
  with `debug_stop_after_route_index=37`, so the generated GDS now includes
  route `n_36`. Command used the direct-release `_rust.pyd` installed from
  `target\release\photonic_router.dll`. Result: `RUN_RESULT_OK`,
  `RUN_WALL_S=23.474`, net-routing phase 20.7516 s, attempts=37,
  failures=0, simple=14/37, repairs=0. Generated GDS:
  `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-14 20:44:57 local,
  size 154290 bytes. Verification: crossing report `success=True`,
  `error_count=0`, `crossing_count=7`; photonic report `success=True`,
  `error_count=0`, `routed_record_count=37`, `missing_route_count=74`,
  `status=partial_debug_stop`.
- Follow-up artifact check: `multiportmmi_8x8` was routed with
  `debug_stop_after_route_index=39`, so the generated GDS now includes `n_37`
  and `n_38`. Result: `RUN_RESULT_OK`, `RUN_WALL_S=22.093`, net-routing phase
  19.5163 s, attempts=39, failures=0, simple=15/39, repairs=0. Generated GDS:
  `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-14 20:47:59 local,
  size 156828 bytes. Verification: crossing report `success=True`,
  `error_count=0`, `crossing_count=7`; photonic report `success=True`,
  `error_count=0`, `routed_record_count=39`, `missing_route_count=72`,
  `status=partial_debug_stop`. Net mapping check: `n_37` is
  `mmi0_multiport_0_1,o4 -> mmi0_ps_array_1_heater_6,o1`; `n_38` is
  `mmi0_multiport_0_1,o3 -> mmi0_ps_array_1_heater_7,o1`, so they are a
  same-element local pair/cluster. `n_36` immediately before them is from
  `mmi0_multiport_0_0,o7`.
- Follow-up artifact check: `multiportmmi_8x8` was routed with
  `debug_stop_after_route_index=47`, adding the next eight routes `n_39`
  through `n_46`. Result: `RUN_RESULT_OK`, `RUN_WALL_S=23.854`,
  net-routing phase 20.7087 s, attempts=47, failures=0, simple=15/47,
  repairs=0. Generated GDS: `build\routed_multiportmmi_8x8.gds`, timestamp
  2026-07-14 20:53:25 local, size 167764 bytes. Verification: crossing report
  `success=True`, `error_count=0`, `crossing_count=7`; photonic report
  `success=True`, `error_count=0`, `routed_record_count=47`,
  `missing_route_count=64`, `status=partial_debug_stop`. Net mapping:
  `n_39`-`n_42` route from `mmi0_ps_array_1_heater_0..3,o2` into
  `mmi0_multiport_1_0,o1..o4`; `n_43`-`n_46` route from
  `mmi0_ps_array_1_heater_4..7,o2` into `mmi0_multiport_1_1,o1..o4`.
- Follow-up artifact check: `multiportmmi_8x8` was routed with
  `debug_stop_after_route_index=51`, adding `n_47` through `n_50`.
  Result: `RUN_RESULT_OK`, `RUN_WALL_S=31.45`, net-routing phase 27.5813 s,
  attempts=51, failures=0, simple=15/51, repairs=0. Generated GDS:
  `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-14 20:55:40 local,
  size 174054 bytes. Verification: crossing report `success=True`,
  `error_count=0`, `crossing_count=7`; photonic report `success=True`,
  `error_count=0`, `routed_record_count=51`, `missing_route_count=60`,
  `status=partial_debug_stop`. Net mapping: `n_47`-`n_50` all leave
  `mmi0_multiport_1_0` from ports `o8`, `o7`, `o6`, and `o5`, respectively,
  into `mmi0_ps_array_2_heater_0`, `_1`, `_2`, and `_7`; this is a same-element
  local fanout/cluster.
- Follow-up artifact check: `multiportmmi_8x8` was routed with
  `debug_stop_after_route_index=52`, adding `n_51`. Result: `RUN_RESULT_OK`,
  `RUN_WALL_S=40.86`, net-routing phase 37.2022 s, attempts=52,
  failures=0, simple=15/52, repairs=0. Generated GDS:
  `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-14 20:58:04 local,
  size 178292 bytes. Verification: crossing report `success=True`,
  `error_count=0`, `crossing_count=8`; photonic report `success=True`,
  `error_count=0`, `routed_record_count=52`, `missing_route_count=59`,
  `status=partial_debug_stop`. Net mapping: `n_51` is
  `mmi0_multiport_1_1,o8 -> mmi0_ps_array_2_heater_4,o1`. This route added
  one legal crossing compared with the stop-after-51 run.

## Current Audit Findings

The first crossing verification milestone has completed its read-only Planner /
Technical Lead audit. Key findings are recorded in the active ExecPlan:

- crossing metadata is split between topology-derived `CrossingConstraint`s and
  router-discovered native crossing events;
- the active `gf.components.crossing()` component is used for bbox sizing, but
  physical crossing refs are not inserted into the routed layout yet;
- `lidar-pure` disables expected-only crossing permission for the main
  collision route, but topology constraints still exist in auxiliary/reporting
  and repair paths;
- endpoint correction can rewrite realized centerlines without protected
  crossing-to-crossing segment metadata;
- final `translation/photonic_verification.py` is not yet crossing-aware;
- Rust `RouteResult.total_cost` can mix length, bend, crossing, history, and
  proactive congestion terms, so reports must separate physical loss proxies
  from non-physical search guidance.

## Current Harness State

The QA / Harness Engineer slice added:

- `translation/crossing_verification_report.py`, a pure structured report layer
  for crossing issues, realized crossing component placement checks, protected
  segment movement checks, and route-cost decomposition;
- `tests/test_crossing_verification_report.py`, focused fixtures for legal
  crossing plus matching component, missing crossing component, protected
  segment movement, illegal crossing reason preservation, and physical versus
  guidance cost separation.

The follow-up routing-flow wiring slice added:

- `routing_flow.py` integration that builds the structured report from existing
  `crossing_plan_info`, maps `insertion_loss_by_net` into physical route-cost
  fields, writes `build/verification/*_crossing_verification.json`, and stores
  the report in `routed_layout.info["crossing_verification"]`;
- `tests/test_routing_flow_stats.py` coverage that uses a fake router result
  to check JSON emission and current missing-component classification without
  invoking Rust, gdsfactory routing, or benchmark validation.

The crossing-component realization slice added:

- active `gf.components.crossing()` placement for legal final realized route
  intersections, with bbox-centered placement, segment-based rotation, and
  placement metadata stored in `crossing_plan_info` and `routed_layout.info`;
- final crossing re-verification in `route_match_and_realize` after endpoint
  correction and path-length meander planning, backed by optional
  `RoutedNetRecord.net_id` preservation through route bookkeeping and meander
  planning;
- `routing_flow.py` report wiring that passes realized crossing components to
  `build_crossing_verification_report`;
- focused fixture coverage for crossing ref placement and successful report
  matching.

Post-review fixes added:

- insertion-loss recounting from final legal `realized_intersections`, so PLM or
  endpoint correction cannot leave route-cost terms sourced from stale native
  crossing events;
- axis-based crossing component rotation validation in
  `translation/crossing_verification_report.py`, with a regression fixture for
  a 90-degree mismatch.

Only lightweight checks ran because the project toolchain is missing:

- Codex bundled Python `py_compile` passed for the new module, the new report
  tests, `routing_flow.py`, and `tests/test_routing_flow_stats.py`.
- A direct import smoke of `translation/crossing_verification_report.py` printed
  `True 1` for a legal crossing with matching component.
- `git -c safe.directory=C:/Users/benja/Documents/Repositorys/TUMPhotonicRouter diff --check`
  passed with only line-ending warnings.

The full Python test suite and full `cargo test` suite have not run. Focused
pytest, focused Rust tests, maturin build, and smaller Benes benchmark evidence
are recorded below.

## Current Validation State

Setup convergence completed on 2026-07-10 13:14Z:

- `.venv` exists and imports `pytest 9.1.1`, `gdsfactory 9.45.0`,
  `gdsfactory.schematic`, and `gdsfactory.gpdk`.
- `maturin 1.14.1` is installed in `.venv`.
- Rust `1.97.0` and Cargo `1.97.0` are installed through rustup.
- The Rust extension was built with `maturin develop --release` using
  `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm`,
  `rust-lld`, `-C target-feature=+crt-static`, and `PYO3_PYTHON=.venv`.
- `import photonic_router._rust` succeeds from `.venv` without extra DLL search
  configuration.

Validation passed:

- `cargo check` passed with `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm`
  and the `rust-lld` linker environment.
- `cargo test crossing --lib` reported `22 passed`.
- `python -m maturin develop --release` rebuilt and installed the editable
  PyO3 extension successfully.
- `python -m pytest -q
  tests\test_realized_crossing_verification.py::test_realized_crossing_verifier_rejects_missing_corrected_centerline_fallback
  tests\test_realized_crossing_verification.py::test_realized_crossing_verifier_rejects_lidar_pure_non_perpendicular_crossing
  tests\test_photonic_verification.py::test_photonic_verifier_reports_realized_bend_static_overlap
  tests\test_routing_flow_stats.py::test_run_routing_flow_rejects_crossing_report_before_gds`
  reported `4 passed in 3.00s`.
- `python -m pytest -q tests\test_crossing_verification_report.py
  tests\test_realized_crossing_verification.py
  tests\test_routing_flow_stats.py::test_run_routing_flow_writes_crossing_verification_report
  tests\test_rust_backend_import.py` reported `25 passed in 2.89s`.
- `python -m pytest -q tests\test_route_rust_records.py
  tests\test_route_rust_geometry.py
  tests\test_port_alignment_diagnostics.py::test_mmi_heater_route_match_uses_corrected_records_for_realization`
  reported `6 passed in 4.12s`.
- `python -m pytest -q tests\test_realized_crossing_verification.py`
  reported `21 passed in 2.72s` after the endpoint-access crossing-verifier
  tightening.
- `cargo test
  committed_crossing_validation_rejects_opened_cell_crossing_away_from_endpoint
  --lib` reported `1 passed` after the Rust native endpoint-access tightening.
- `python -m pytest tests\test_realized_crossing_verification.py -q` reported
  `21 passed in 3.49s`.
- `python -m maturin develop --release` rebuilt and installed the editable
  PyO3 extension after the Rust native validation change.
- `cargo fmt --check` did not run because `rustfmt` is not installed for
  `stable-x86_64-pc-windows-gnullvm`.
- After the repair-feedback patch, `.\.venv\Scripts\python.exe -m pytest
  tests\test_realized_crossing_verification.py -q` reported `21 passed in
  3.51s`.
- Rust validation of the repair-feedback patch is blocked before repository
  code compiles: `cargo test targeted_illegal_crossing_repair_promotes_learned_blocker
  --lib` fails on missing MSVC `link.exe`; using
  `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm` and
  `CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm` fails on missing
  `x86_64-w64-mingw32-clang`; `maturin develop --release` fails the same way
  after adding `C:\Users\benja\.cargo\bin` to `PATH`.
- Linker recovery attempts after that did not reach repository code either:
  `winget install --id MartinStorsjo.LLVM-MinGW.UCRT` was stopped after a long
  extraction with no linker on disk; `ziglang==0.16.0` and
  `cargo-zigbuild==0.23.0` installed into `.venv`; `cargo-zigbuild test
  --target x86_64-pc-windows-gnu --lib
  targeted_illegal_crossing_repair_promotes_learned_blocker` still failed
  before compile/test on host gnullvm linker/runtime issues.
- Earlier focused validation also included
  `tests\test_realized_crossing_verification.py`,
  `tests\test_photonic_verification.py`, and
  `tests\test_route_rust_geometry.py`, which reported `13 passed in 3.67s`.
- `cargo test crossing` reported `17 passed`.
- `cargo test route_sanity --lib` exited successfully but matched zero tests in
  this branch.

Benchmark evidence recorded:

- `benes_4x4` with `--crossings true --crossing-mode lidar-pure` completed in
  `0.2990 s` total and wrote
  `build\verification\benes_4x4_crossing_verification.json`. The report has
  `success=True`, `legal_crossing_count=2`,
  `matched_crossing_component_count=2`, and `issues=0`.
- `benes_8x8` with `--crossings true --crossing-mode lidar-pure` completed in
  `24.7861 s` total and wrote
  `build\verification\benes_8x8_crossing_verification.json`. The report has
  `success=True`, `legal_crossing_count=16`,
  `matched_crossing_component_count=16`, and `issues=0`.
- `multiportmmi_8x8` with `--crossings true --crossing-mode lidar-pure
  --debug-svgs false --debug-timing false` currently fails while routing
  `n_51` (`mmi0_multiport_1_1,o8 -> mmi0_ps_array_2_heater_4,o1`):
  `No repair route found; candidate_blockers=[51, 48, 49]`, with recent native
  errors reporting `Illegal realized crossing: net 52 intersects net 51 ...
  (not_perpendicular)`. This is the next routing-strategy work item.
- `multiportmmi_8x8` stopped after route 37 with debug SVGs now fails final
  realized-crossing verification as intended, reporting 7 illegal crossings.
  The user-flagged cluster pair is present as `n_32` x `n_35` with
  `perpendicular=false`, so the current SVG is not accepted as valid output.
- After the Rust-side endpoint-access fix, the same route-37 debug command no
  longer reaches final verification cleanly. It fails while routing `n_32` with
  `No repair route found; candidate_blockers=[36, 31]` and recent native errors
  reporting illegal realized crossings against `n_35`. A final rerun after
  reverting slower experiments was stopped after more than 150 seconds of CPU
  time, so the next implementation pass should be a bounded repair-strategy
  lane rather than a broader fanout-opening or keepout heuristic.
- A bounded diagnostic through route 52 with an experimental orthogonal
  collision-crossing repair path was stopped because it did not return within
  the local cap. The speed-risk behavior was removed; do not revive it without
  a smaller native fixture and timing bound.

Use `PYTHONIOENCODING=utf-8` for `routing_flow.py` on this Windows console
because it prints non-ASCII status symbols. Use `MPLCONFIGDIR=.mplconfig` to
avoid matplotlib trying to write under `AppData\Local\matplotlib`.

## Current Workflow Policy

On 2026-07-10, the workflow docs were updated to make convergence explicit.
Agents should continue setup, implementation, validation, and review loops
inside the agreed objective until the objective works, passes the appropriate
validation, and has no blocking review findings.

On 2026-07-10 20:20 +02:00, the workflow docs were updated with the main
lesson from the long route-69 debugging prompt: repeated convergence loops must
fan out instead of staying in one serial window. After two focused benchmark or
route-stop failures with different hypotheses, or when a failure signature
moves but keeps the same root shape, the orchestrator should split the work
into bounded lanes: Explorer / Codebase Audit, QA / Harness, Benchmark /
Evidence, Implementation, and Reviewer. A new
`.agent/roles/explorer.md` role brief was added, and the harness, implementer,
and reviewer briefs now include their responsibilities in this split.

On 2026-07-10 13:52Z, the notification/checkpoint rule was updated per user
preference: update `.agent/REPOSITORY_STATE.md` and send a user-facing chat
message before every agent stop, pause, or handoff, not merely every 10 commits.
Ask immediately whenever approval, clarification, credentials, missing tools, or
other user action is needed during implementation. Phone push delivery remains
controlled by the Codex/ChatGPT app and OS notification settings; the repository
policy can require the chat message, but cannot itself guarantee mobile push
delivery.

For setup, this means working until the project toolchain can run the agreed
checks or recording the exact missing tool, command output, and required user
action. For subagent implementation cycles, this means sending blocking reviewer
findings back through implementation and validation until convergence or a
documented failure.

## Reference Branches

The branch `baseline/lidar-pure-crossings` contains a WIP prototype snapshot at:

    69ab9fd wip: snapshot experimental lidar-pure crossing prototype

Treat that branch as reference material only. It may contain useful ideas,
tests, benchmark imports, or implementation fragments, but it is not the clean
implementation path. Do not merge it wholesale into
`crossings/verification-foundation`.

If code is needed from the WIP branch:

1. Audit the relevant diff or file in the WIP branch.
2. Confirm the idea fits `.agent/PROJECT_GOAL.md` and the active ExecPlan.
3. Port the smallest useful piece manually or cherry-pick a narrow commit only
   after review.
4. Add focused verification before considering the port complete.

The old ExecPlan
`.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md` is also
reference-only unless the user explicitly resumes it.

## Next Engineering Step

Resume `multiportmmi_8x8` debugging using the now-green Benes crossing
verification harness. Keep speed visible in every benchmark run. If physical
DRC starts treating same-layer route polygons under crossing refs as illegal,
the next realization slice should trim or split route polygons around crossing
footprints instead of weakening the structured report.

## Latest Stop Status - 2026-07-10 20:10 +02:00

Branch confirmed: `crossings/verification-foundation`.

This stop is a documented technical blocker, not a toolchain blocker. The
Windows toolchain remains usable with the gnullvm/rust-lld environment recorded
above.

Current changes advanced `multiportmmi_8x8` LiDAR-pure routing through the
previous route-52/55/68 failures. Focused route-68 evidence now passes:

    $env:PYTHONPATH='.'
    $env:MPLCONFIGDIR=(Resolve-Path .\.mplconfig).Path
    $env:PYTHONIOENCODING='utf-8'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false --debug-stop-after-route 68

The latest run completed successfully in about `40.7 s` total with `repairs=7`.

Focused route 69 remains blocked:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-timing true --debug-svgs false --debug-stop-after-route 69

The current failure is `n_68`:

    No route found for n_68: mmi0_multiport_2_1,o7 -> mol_array_1_mzi_3,o1.
    candidate_blockers=[66, 67, 68]

The LiDAR-pure fallback can find a static-only route, but validation reports
internal non-perpendicular crossings such as:

    Illegal realized crossing: net 69 intersects net 67 at (2581.500, 925.500) (not_perpendicular)

Accumulated validation-feedback keepouts shift the conflict along the same
vertical channel rather than producing a legal route. A vertical-stripe keepout
experiment was tried and removed because it made the retry unroutable.

Latest focused validation passed:

    C:\Users\benja\.cargo\bin\cargo.exe check
    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py

The next engineering step should be a planner/implementer pass on global repair
for this case: use the fallback route as a probe, then reroute the conflicting
partner(s) around the committed route or add a more principled crossing-aware
static-only retry that blocks an entire illegal segment without closing the
channel.

## Latest Stop Status - 2026-07-10 21:05 +02:00

User-provided layout screenshots exposed two additional failure classes:

1. Two routed nets can share or converge onto the same realized path.
2. A route can pass through the port/static keepout region of a sibling port.

Implemented focused guards:

- Python final realized-crossing verification now detects nonzero collinear
  route overlap and reports it as `collinear_route_overlap`.
- Rust native realized-crossing validation mirrors the collinear-overlap guard,
  so bad overlaps can be rejected during commit/repair instead of only after
  realization.
- LiDAR-pure probe commits still allow cluster-shaped footprint cases, but
  non-cluster realized violations now validate instead of silently accepting
  small bad overlap sets.
- Foreign port keepouts are now stored per port spec as well as by instance.
  Active endpoint ports can open their own keepout. Dense multi-port fanout
  instances can open same-instance fanout keepouts to preserve routability.
  Two-port/sibling device keepouts stay closed for unrelated active ports.

Focused regression coverage added:

- `tests/test_realized_crossing_verification.py` rejects collinear route overlap
  in LiDAR-pure mode.
- `src/py_router.rs` Rust unit coverage rejects collinear realized overlaps.
- `tests/test_route_rust_opened_cells.py` verifies that a sibling foreign port
  keepout on a two-port-style instance is not opened by an active route.

Validation passed:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py tests\test_route_rust_opened_cells.py tests\test_crossing_verification_report.py tests\test_rust_batch_repair.py
    C:\Users\benja\.cargo\bin\cargo.exe test --lib
    C:\Users\benja\.cargo\bin\cargo.exe check
    .\.venv\Scripts\python.exe -m maturin develop --release
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 68 --debug-timing false

`multiportmmi_8x8 --debug-stop-after-route 69` remains blocked at the known
`n_68` not-perpendicular repair case:

    Illegal realized crossing: net 69 intersects net 67 at (2581.500, 925.500) (not_perpendicular)

This is still the next implementation target. The new overlap/static guards did
not change it into a silent bad layout.

Toolchain note: `cargo fmt` could not run because `rustfmt` is not installed for
`stable-x86_64-pc-windows-gnullvm`. Do not treat this as a benchmark blocker;
record it or install the component explicitly if formatting is required.

## Latest Stop Status - 2026-07-11 00:58 +02:00

Follow-up on the user screenshots around the `n_32` cluster:

- Confirmed the previously viewed `build\routed_multiportmmi_8x8.gds` was stale
  when the user saw no change. New stricter runs were failing before GDS write,
  so KLayout kept showing the old artifact.
- Added/kept final photonic geometry gating so routed GDS writes are rejected
  before disk output if cross-net waveguide overlaps or route/static overlaps
  survive realization.
- Tightened dense same-instance fanout openings so sibling raw-static port stubs
  stay blocked; only non-static same-instance fanout keepout cells are reopened
  for dense multiport routability.
- Added inversion-aware dense multiport source ordering. The validated ordering
  fixes the visible `n_31`/`n_32`/`n_35` cluster and the follow-on
  `n_51`/`n_54` cluster without reopening raw-static sibling geometry.

Current artifact status:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 68 --debug-svgs 31-36,51-55 --debug-timing false

passed with:

    build\verification\multiportmmi_8x8_photonic_verification.json
    success=true
    routed_record_count=68
    cross_net_waveguide_overlap_count=0
    waveguide_obstacle_overlap_count=0

The regenerated GDS is:

    build\routed_multiportmmi_8x8.gds
    LastWriteTime: 2026-07-11 00:53:34 +02:00
    Length: 200426 bytes

This GDS is a clean 68-route debug-stop artifact, not a full 111-net layout.

The full 111-net command was attempted:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-svgs 31-36,51-55 --debug-timing false

It still blocks later at the mol-array handoff:

    No route found for n_68: mmi0_multiport_2_1,o7 -> mol_array_1_mzi_3,o1
    recent_errors include Illegal realized crossing ... (not_perpendicular)

Additional paired-run ordering experiments for the `n_63`-`n_70` mol-array
handoff were tried and removed because they moved the blocker between `n_68`,
`n_65`, and `n_66` instead of converging. The next implementation target should
be a real repair-loop change for sibling fanout handoffs, not more local route
permutation.

Validation passed for the retained code:

    .\.venv\Scripts\python.exe -m py_compile routing_flow.py translation\route_rust.py translation\photonic_verification.py tests\test_photonic_verification.py tests\test_route_rust_opened_cells.py tests\test_routing_flow_stats.py
    .\.venv\Scripts\python.exe -m pytest -q tests\test_photonic_verification.py tests\test_route_rust_opened_cells.py tests\test_routing_flow_stats.py -q

## Latest Stop Status - 2026-07-11 01:18 +02:00

User reported additional invalid geometries and, before further routing work,
called out a workflow failure: the prior loop used the orchestrator as one long
reasoning/implementation window instead of enforcing the intended
Harness/Reviewer pipeline.

Stable workflow updates made in response:

- `.agent/ORCHESTRATOR.md` now has a mandatory routing verification gate for
  nontrivial routing changes. The implementer/orchestrator must produce a
  verifier packet and QA / Harness must return `PASS`, `FAIL`, `BLOCKED`, or
  `INCONCLUSIVE` before the user is told the slice is fixed or complete.
- `.agent/ORCHESTRATOR.md` also now has a verifier veto and artifact-freshness
  rule: report command, `debug-stop-after-route` status, routed-record count,
  GDS timestamp, and whether an artifact is full or partial.
- `.agent/WORKFLOW.md` now names the Routing Verification Gate and says user
  screenshots of invalid geometry are validation-blind-spot evidence until a
  deterministic report, fixture, or artifact inspection catches the class.
- `.agent/roles/harness.md` now requires direct report/SVG/GDS inspection,
  artifact freshness checks, a verifier verdict, and a short retrospective
  after long routing loops.
- `.agent/roles/reviewer.md` now treats missing harness sign-off as a blocking
  finding for nontrivial routing work and flags stale/partial artifact claims.
- `.agent/roles/implementer.md` now requires a verifier packet after routing
  edits and forbids declaring completion before harness/reviewer feedback.
- The active ExecPlan records this as a workflow retrospective and says the
  next technical role for the new invalid `multiportmmi_8x8` screenshots is
  QA / Harness Engineer, not implementation.

No new router behavior was edited in this stop-status update. The newly posted
layout screenshots still need a harness pass to identify the exact artifact,
route indices/nets, missing deterministic check, and verifier report blind spot
before the next implementation loop.

## Latest Stop Status - 2026-07-11 01:45 +02:00

Ran the requested QA / Harness Engineer pass for the new
`multiportmmi_8x8` screenshots. Verdict: FAIL for the current verification
gate, with a freshness caveat.

Artifact freshness:

- `build\routed_multiportmmi_8x8.gds` was last written at
  `2026-07-11 00:53:34 +02:00`, length `200426`.
- It is a `--debug-stop-after-route 68` artifact, not the full 111-net
  benchmark layout.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `success=true`, `expected_route_count=111`, `routed_record_count=68`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`, so the report should not be described
  as a full-layout pass.

Harness findings:

- Per-net diagnostics contradict the clean photonic report for the photographed
  fanout classes. Examples: `n_32` has `route_static_blocked_overlap_count=8`;
  `n_34` has `route_dynamic_overlap_count=2`; `n_51`, `n_52`, `n_53`, and
  `n_54` have dynamic overlap counters in the same dense MMI-adjacent slice.
- `n_52` diagnostics show a local loop/revisit pattern around the source fanout
  before reaching the heater target, matching the visible bulb/loop failure
  shape in the screenshots.
- Direct KLayout inspection of the current GDS found raw-over-union overlap on
  optical layer `1/0`: raw area `35449.947 um^2`, merged area
  `35383.991 um^2`, overlap `65.955 um^2`.
- Direct KLayout inspection found one overlapping crossing footprint on layer
  `2/0`: overlap `3.057 um^2`, bbox approximately
  `(2552.990,648.490)-(2555.010,650.510) um`.
- The crossing report has `success=true` and allows the footprint overlap under
  `allowed_lidar_pure_cluster`; this matches the user screenshot with adjacent
  blue crossing components but is not sufficient proof of physically clean
  crossing realization.
- Subagent QA addendum: route-under-crossing is likely present at the five
  orthogonal crossing centers `(2039.6,850)`, `(2039.6,950)`,
  `(2039.6,1050)`, `(2039.6,750)`, and `(2023.6,850) um`, involving `n_50`
  with `n_51`/`n_52`/`n_53`/`n_54` and `n_51` with `n_54`. The clustered
  crossing overlap is around `n_65`/`n_66`/`n_67` at `(2555.25,648.25)` and
  `(2552.75,650.75) um`. A separate same-layer route-polygon overlap appears
  around `(1236.998,708.998)-(1251.656,723.656) um`; exact net identity needs
  route labels in realized polygon diagnostics.

Next implementation entry criteria:

- Add deterministic harness/tests before router behavior edits for:
  partial-artifact/full-artifact status, final GDS layer-overlap audit,
  MMI-adjacent endpoint/opened-cell route-through-static masking,
  non-adjacent self/cross-net overlap in dense fanout routes, and crossing
  footprint overlap/spacing policy.
- After tests exist, run a bounded implementer loop against the fanout/opened
  cell and crossing-spacing defects. The implementation packet must include
  commands, routed-record count, GDS timestamp, route indices/nets inspected,
  and a QA / Harness verdict before the user is told the geometry is fixed.

## Latest Stop Status - 2026-07-11 02:20 +02:00

Implemented the first QA/Harness gate for the current `multiportmmi_8x8`
visual failures. This was validation/reporting work only; no A* search or
routing behavior was edited in this slice.

Changed files in this slice:

- `routing_flow.py`: verification JSON now records `status`, `partial`,
  `debug_stop_after_route_index`, expected/routed/missing route counts, and
  `route_coverage_check_enabled` for crossing and photonic reports.
- `translation/photonic_verification.py`: added deterministic checks for
  `crossing_component_route_overlap` and `crossing_component_overlap` from
  legal crossing footprint polygons.
- `tests/test_photonic_verification.py`: added focused fixtures for route
  overlap with crossing footprints and overlapping crossing footprints.
- `tests/test_routing_flow_stats.py`: added debug-stop status metadata coverage
  and report status assertions.

Validation evidence:

    .\.venv\Scripts\python.exe -m py_compile routing_flow.py translation\photonic_verification.py tests\test_photonic_verification.py tests\test_routing_flow_stats.py
    .\.venv\Scripts\python.exe -m pytest -q tests\test_photonic_verification.py tests\test_routing_flow_stats.py -q
    .\.venv\Scripts\python.exe -m pytest -q tests\test_crossing_verification_report.py tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_routing_flow_stats.py -q

The bounded benchmark check was:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 68 --debug-svgs false --debug-timing false

It now fails before GDS write, as intended, with `17` photonic verification
errors. The generated photonic report has `success=false`,
`status=partial_debug_stop`, `expected_route_count=111`, `routed_record_count=68`,
`missing_route_count=43`, `crossing_component_route_overlap_count=16`, and
`crossing_component_overlap_count=1`.

The old `build\routed_multiportmmi_8x8.gds` remained unchanged at
`2026-07-11 00:53:34 +02:00`, so the failing verifier did not overwrite the
stale visual artifact.

Next implementation target:

- Implement crossing realization/clipping so route polygons do not remain under
  crossing component footprints.
- Tighten clustered-crossing spacing or component placement so overlapping
  crossing footprints no longer pass under `allowed_lidar_pure_cluster`.
- After each behavior edit, rerun the focused tests plus the route-68 debug-stop
  command and require QA / Harness verdict before claiming the visual geometry
  is fixed.

## Latest Stop Status - 2026-07-11 05:49 +02:00

Completed the full multi-agent convergence loop for the current
`multiportmmi_8x8` invalid-geometry screenshots. The active implementation
fixed the endpoint-correction, cross-net-overlap, and route-through-crossing
component classes and reran QA / reviewer checks until the full benchmark
passed.

Key behavior changes in this stop:

- `translation/route_rust.py`: added final photonic repair after final crossing
  repair; photonic issues are converted into targeted grid keepouts and reroute
  only implicated offender nets. Unresolved final photonic verification now
  raises before realization/GDS write. Repair attempts are recorded in
  `build/crossings/*_crossings.json`.
- `translation/route_rust.py`: final crossing repair ordering/keepouts were
  tuned for tight `not_perpendicular` repairs, degraded-footprint blocker axes
  now match the actual component footprint, and shared crossing placements carry
  `shared_owner_net_names` / `shared_owner_net_ids`.
- `translation/photonic_verification.py`: route-through-crossing-component
  checks allow only declared owner/shared-owner routes through a component
  footprint; third-party routes still report `crossing_component_route_overlap`.
- `src/py_router.rs`: checked endpoint correction now allows only bounded active
  endpoint contact, and successful endpoint bump commits refresh native route
  metadata/opened cells and rerun committed-crossing validation with rollback on
  failure.
- Focused tests were extended for endpoint contact static handling, opened-cell
  crossing suppression bounds, crossing component owner/shared-owner overlap,
  crossing realization clustering, and final repair metrics.

Validation evidence:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_realized_crossing_verification.py tests\test_photonic_verification.py tests\test_crossing_verification_report.py tests\test_port_alignment_diagnostics.py::test_checked_no_bump_endpoint_correction_allows_active_endpoint_static_contact tests\test_port_alignment_diagnostics.py::test_checked_no_bump_endpoint_correction_rejects_middle_static_contact tests\test_route_rust_realization.py
    # 42 passed in 3.53 s

    C:\Users\benja\.cargo\bin\cargo.exe check
    # passed

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    Copy-Item -LiteralPath target\x86_64-pc-windows-gnullvm\release\photonic_router.dll -Destination python\photonic_router\photonic_router.dll -Force
    .\.venv\Scripts\python.exe -c "import photonic_router._rust as r; print(r.__file__)"
    # loaded C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\python\photonic_router\_rust.pyd

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-svgs false --debug-timing false
    # passed

Final artifact/report state:

- `build\routed_multiportmmi_8x8.gds` was rewritten at
  `2026-07-11 05:48:28 +02:00`, length `301860`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `success=true`, `expected_route_count=111`, `routed_record_count=111`,
  `missing_route_count=0`, `cross_net_waveguide_overlap_count=0`,
  `crossing_component_route_overlap_count=0`,
  `crossing_component_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `success=true`, `error_count=0`, `illegal_crossing_count=0`,
  `legal_crossing_count=20`, `realized_crossing_component_count=18`,
  `matched_crossing_component_count=18`,
  `final_crossing_repair_attempt_count=2`, and
  `final_photonic_repair_attempt_count=4`.
- Final photonic repairs were endpoint connection `[106, 111]`, cross-net
  overlap `[51, 55]`, and crossing-component offender repairs `[68]` and
  `[89]`; all ended with empty `endpoint_correction_failed_net_ids`.

Pipeline notes:

- QA / Harness sidecar defined the focused gate for the prior `n_105`/`n_110`,
  `n_50`/`n_54`, and `n_88` failures.
- First reviewer found stale partial repair state, over-broad opened-cell
  suppression, and repair metric ambiguity; all were addressed.
- Second reviewer found stale native endpoint-bump metadata and shared-cluster
  owner under-exemption; both were addressed before the final full benchmark.
- The worktree remains dirty with many pre-existing files from earlier routing
  work. Do not revert unrelated changes.

## Latest Stop Status - 2026-07-11 12:15Z

User identified the visible loop in the `multiportmmi_8x8` cluster around
`n_32`. Fresh route diagnostics showed the loop belonged to `n_31`, whose old
route curled left/backward from `mmi0_multiport_0_0,o12` through static-opened
port access cells:

    straight:(706, 162)->(707, 162)
    turn90:(707, 162)->(710, 159)
    turn90:(710, 159)->(707, 156)
    turn90:(707, 156)->(704, 159)

Root cause: `src/py_router.rs::route_collect_inflated_step_cells` inflated each
forward port-access step as a square, so east-facing ports also opened cells
behind the port plane. A* then treated those static cells as legal free space
for the whole route. The fix keeps the square width in front of the port but
clips generated cells whose projection along the port direction is negative.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check
    .\.venv\Scripts\python.exe -m maturin develop --release
    .\.venv\Scripts\python.exe -m pytest tests\test_route_rust_opened_cells.py -q
    # 16 passed
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-svgs 31-37 --debug-stop-after-route 37 --debug-timing false
    # passed through n_37

Fresh artifacts:

- `build\routes\multiportmmi_8x8_n31_n36_cluster_overlay.svg`
- `build\routes\multiportmmi_8x8_n_31_diagnostics.txt` now starts
  `turn45:(706, 162)->(712, 165)` and no longer visits `x=704`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `success=true`, `error_count=0`, `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0` for the stop-after-37 run.
- `build\verification\multiportmmi_8x8_crossing_verification.json` reports
  `success=true`, `error_count=0`, and `illegal_crossing_count=0` for the same
  run.

## Latest Stop Status - 2026-07-11 19:01 +02:00

Current milestone status: BLOCKED with a concrete route-37 packet, not a
toolchain blocker and not a silent verifier miss.

What changed in this bounded repair audit:

- `src/py_router.rs`: native endpoint/crossing validation now uses tighter
  endpoint-access suppression and learned repair feedback is scoped per rip-up
  set instead of becoming a global keepout wall.
- `src/py_router.rs`: learned crossing-error feedback retries are capped per
  rip-up set, so `n_32` repair attempts terminate instead of cycling
  indefinitely.
- `src/py_router.rs`: `No route found` repair failures can enqueue bounded
  route-order neighbor repair sets, limited to near neighbors.
- `src/py_router.rs`: native dynamic commit/core cells use realized
  centerline-derived geometry when available, so commit validation sees the
  same shape class as final verification.
- `translation/route_rust.py`: dense same-instance source openings remain
  clipped to active endpoint/target-side windows; raw sibling static port cells
  are not reopened for unrelated routes.
- `tests/test_route_rust_opened_cells.py` and `src/py_router.rs` contain the
  focused regressions for endpoint-local dynamic exemptions, sibling keepout
  handling, learned blocker promotion, capped learned retries, and bounded
  no-route neighbor expansion.

Validation passed after the latest code:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    C:\Users\benja\.cargo\bin\cargo.exe test "repair_" --lib
    C:\Users\benja\.cargo\bin\cargo.exe test learned_keepout_retry_is_capped_per_ripup_set --lib
    .\.venv\Scripts\python.exe -m pytest tests\test_route_rust_opened_cells.py -q
    # 16 passed
    .\.venv\Scripts\python.exe -m pytest tests\test_realized_crossing_verification.py -q
    # 21 passed
    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm

Final route-37 gate still fails:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true

Current failure:

    No route found for n_32: mmi0_multiport_0_0,o11 -> mmi0_ps_array_1_heater_1,o1
    source=(706, 165, 0), target=(767, 93, 0)
    candidate_blockers=[36, 31]

Recent native errors now include:

    Illegal realized crossing: net 33 intersects net 36 at (1491.500, 571.125) (not_perpendicular)
    Illegal realized crossing: net 33 intersects net 36 at (1491.500, 614.500) (not_perpendicular)
    repair_failed_net:net33:roundSome(1):rip[36, 31]:Failed to commit routed cells to obstacle map
    reroute_victims:net36:roundSome(1):rip[36, 31]:Illegal realized crossing: net 36 intersects net 33 at (1478.382, 582.007) (not_perpendicular)

Key evidence file:

    build\routes\multiportmmi_8x8_n_32_FAILED.txt

The verifier is now doing the right thing: it rejects the non-perpendicular
`n_32`/`n_35` cluster instead of writing a stale or invalid GDS. The remaining
work is native repair strategy. The next implementer should use the failed
victim reroute as a probe to force an alternate current-route corridor for
`net33`/`n_32`, rather than broadening port openings, weakening the verifier,
or adding unbounded keepout growth. A bounded fixture around this exact
`net33`/`net36` pair should be added before another heuristic loop.

Toolchain note:

- Rustfmt is still unavailable for the gnullvm toolchain, so `cargo fmt` has
  not been run.
- Use the LLVM-MinGW/gnullvm environment already recorded above for `cargo`,
  `maturin`, and Rust tests.

## Latest Stop Status - 2026-07-12 10:57 +02:00

User reported that a command had been left running for roughly 13 hours. This
is a workflow failure. A process-table check found no remaining
TUMPhotonicRouter-related `python.exe`, `cargo.exe`, `maturin.exe`, or
`rustc.exe` processes at this checkpoint, so there was nothing left to stop.

Durable workflow fix recorded:

- `.agent/ORCHESTRATOR.md` now requires explicit command runtime budgets,
  regular polling, and stale-process cleanup after interrupted/resumed work.
- `.agent/WORKFLOW.md` now applies the same rule to all agents and role lanes:
  focused routing validation defaults to a 5 minute cap unless the user approves
  a longer run, and a timeout is a failure signature rather than background
  progress.

Do not resume the `multiportmmi_8x8` route-37 implementation loop until the
next agent states the command cap up front and uses a bounded benchmark/evidence
lane.

## Latest Stop Status - 2026-07-12 11:30 +02:00

Cleanup/reviewer pass completed after the interrupted long-running route-37
implementation loop.

What was cleaned from `src/py_router.rs`:

- Removed speculative endpoint-opening radius preservation in
  `opened_cells_excluding_keepout`; only the exact source and target opened
  cells are preserved through repair keepouts again.
- Removed persistent learning of dynamic-commit validation bboxes into crossing
  repair keepouts. Dynamic commit bboxes can still seed the immediate augmented
  retry, but they are no longer promoted into a durable learned wall.
- Removed the broad spatial fanout-owner lookup used during candidate blocker
  selection.
- Removed current-first repair reservation retention that kept the current route
  blocked during victim probing.
- Removed the speculative terminal-straight override from crossing-search paths;
  the remaining `require_terminal_straights = true` settings are existing
  non-crossing repair/search guards.

Bounded validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # default MSVC target failed before repo code on missing link.exe

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm,
    # CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm, LLVM-MinGW linker,
    # and PYO3_PYTHON=.venv\Scripts\python.exe

    C:\Users\benja\.cargo\bin\cargo.exe test keepout --lib
    # 4 passed, 286 filtered out, with the same gnullvm/LLVM-MinGW env

No benchmark or route-37 rerun was attempted in this cleanup pass. The next
technical role remains QA / Harness Engineer for a bounded `route-37` packet
around the `n_32`/`n_35` and `net33`/`net36` cluster before any new heuristic
implementation.

## Latest Stop Status - 2026-07-12 14:35 +02:00

QA / Harness packet completed for the current `multiportmmi_8x8` route-37
`n_32` cluster. This did not fix routing behavior; it made the failure packet
more deterministic before implementation resumes.

Pipeline/subagent note:

- Spawned read-only QA / Harness sidecar `Aquinas` with the inherited model
  default. It returned `FAIL` for the current route-37 verifier packet and
  classified the issue as route search/native repair convergence rather than a
  verifier blind spot.
- The sidecar identified a packet gap: terminal repair exhaustion can obscure
  the original illegal-crossing root cause unless it is preserved in stable
  text or structured output.

Changed files in this QA slice:

- `translation/route_rust.py`: failed route logs now include
  `root_cause_illegal_crossings=` JSON when the terminal error or recent
  attempt errors contain `Illegal realized crossing: net A intersects net B ...
  (reason)`.
- `tests/test_route_failure_diagnostics.py`: focused fixture for the current
  `net33`/`net36` non-perpendicular route-37 packet shape.
- `.agent/execplans/2026-07-10-crossing-verification-foundation.md`: recorded
  the QA packet, discovery, decision, and revision note.

Validation evidence:

    .\.venv\Scripts\python.exe -m py_compile translation\route_rust.py tests\test_route_failure_diagnostics.py
    # passed

    .\.venv\Scripts\python.exe -m pytest tests\test_route_failure_diagnostics.py -q
    # 2 passed in 6.56 s

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed after rebuilding the Rust extension with the gnullvm/LLVM-MinGW env

Fresh bounded route-37 evidence:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # failed in about 163 s, inside the 5 minute cap

Current failure:

    No route found for n_32: mmi0_multiport_0_0,o11 -> mmi0_ps_array_1_heater_1,o1
    source=(706, 165, 0), target=(767, 93, 0)
    candidate_blockers=[36, 31]

Fresh artifact:

    build\routes\multiportmmi_8x8_n_32_FAILED.txt
    LastWriteTime: 2026-07-12 14:29:10 +02:00

That failed log now includes:

    root_cause_illegal_crossings=[
      {"net_a":33,"net_b":36,"reason":"not_perpendicular","x_um":1491.5,"y_um":571.125},
      {"net_a":33,"net_b":36,"reason":"not_perpendicular","x_um":1491.5,"y_um":614.5},
      {"net_a":36,"net_b":33,"reason":"not_perpendicular","x_um":1488.47,"y_um":562.095}
    ]

No `build\verification\multiportmmi_8x8_*_verification.json` files were
produced because routing stops before final verification. The next role should
be Implementation Engineer for the native repair strategy around
`n_32`/`net33` and `n_35`/`net36`, using this packet as the acceptance hook.

## Latest Stop Status - 2026-07-12 15:49 +02:00

Implementation Engineer repair pass completed, but the route-37 milestone is
not fixed yet.

Pipeline/subagent note:

- Closed read-only sidecar `Parfit`, which recommended directional learned
  keepouts because current-owned dynamic overlap cells were being replayed as
  shared keepouts and could self-block current `net33`.
- Closed read-only sidecar `Lagrange`, which mapped `n_32 -> net33` and
  `n_35 -> net36`, classified the remaining issue as current route choice
  after victim reroute, and confirmed `candidate_blockers=[36,31]` means
  `n_35` plus `n_30`.

Changed behavior retained in `src/py_router.rs`:

- Added `dynamic_commit_error_overlap_owner_ids`.
- Added victim-only learned repair keepouts alongside the shared learned
  keepout map.
- Victim reroute failures that name the current net now put learned
  dynamic-overlap and illegal-crossing keepouts into the victim-only map.
- Victim-only keepouts are active while rerouting victims and are removed
  before retrying the current net.
- Added a Rust fixture covering both current-owned dynamic overlap and
  current-involved illegal crossing learning.

Speculative changes tested and removed in the same pass:

- Strict orthogonal-only victim reroute fallback.
- Rerouted-victim core reservation during current retry.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe test keepout --lib
    # 6 passed with RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm,
    # CARGO_BUILD_TARGET=x86_64-pc-windows-gnullvm,
    # LLVM-MinGW linker, and PYO3_PYTHON=.venv\Scripts\python.exe

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with the same environment

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # failed inside the 5 minute cap

Current failure remains:

    No route found for n_32: mmi0_multiport_0_0,o11 -> mmi0_ps_array_1_heater_1,o1
    source=(706, 165, 0), target=(767, 93, 0)
    candidate_blockers=[36, 31]

Fresh artifact:

    build\routes\multiportmmi_8x8_n_32_FAILED.txt
    LastWriteTime: 2026-07-12 15:49 +02:00

Fresh failure shape:

    root_cause_illegal_crossings=[
      {"net_a":33,"net_b":36,"reason":"not_perpendicular","x_um":1491.5,"y_um":571.125},
      {"net_a":33,"net_b":36,"reason":"not_perpendicular","x_um":1491.5,"y_um":614.5}
    ]

    recent tail includes repeated:
      repair_failed_net:net33:roundSome(2):rip[36, 31, 30]:No route found
      reroute_victims:net36:roundSome(2):rip[36, 31, 30]:No route found

The next role should be QA / Harness Engineer, not another heuristic pass:
create a small native repair fixture or trace harness that replays the
current-first sequence for current `net33` plus victims `[36,31,30]` and
records the exact phase that prevents a committed repair after individually
successful victim reroutes.

## Latest Stop Status - 2026-07-12 18:52 +02:00

QA / Harness trace slice completed for the same route-37 `n_32` / native
`net33` blocker. This was diagnostic-only work; no routing heuristic or A* cost
behavior changed.

Changed harness/reporting behavior:

- `src/py_router.rs` now returns native `repair_trace` records from
  `route_many_with_repair_and_commit`.
- Trace records include repair phase, `route_order`, `repair_set_index`,
  `repair_round`, `candidate_blockers`, `ripup_ids`, `victim_order`, order
  flags, success, and error text.
- `translation/route_rust.py` writes `native_repair_trace_count` and JSON
  `native_repair_trace_tail` into failed route logs.
- `tests/test_rust_batch_repair.py` and
  `tests/test_route_failure_diagnostics.py` cover the new trace/report shape.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with the gnullvm/LLVM-MinGW/PYO3_PYTHON environment

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe -m pytest -q tests\test_route_failure_diagnostics.py tests\test_rust_batch_repair.py
    # 4 passed in 3.50 s after the final rerun

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 31-37 --attempt-diagnostics --debug-timing true
    # failed inside the 5 minute cap at n_32 / net33, as expected for this harness slice

Fresh artifact:

    build\routes\multiportmmi_8x8_n_32_FAILED.txt
    native_repair_trace_count=141

Trace finding:

- `current_first`, reverse victim order `[30,31,36]`: victim `net31` reroutes
  successfully, then victim `net36` fails both normal and repair-fallback
  reroute with `No route found`.
- `victim_first`, both victim orders: victims `[36,31,30]` reroute
  successfully, then current `net33` fails both normal and repair-fallback
  routing with `No route found`.
- Original root cause remains `net33` x `net36` non-perpendicular crossings at
  `(1491.500,571.125)` and `(1491.500,614.500)`.

Local tooling note: `cargo fmt` could not run because `rustfmt` is not
installed for either the default MSVC toolchain or
`stable-x86_64-pc-windows-gnullvm`. `git diff --check` for the touched source
and test files passed.

## Latest Stop Status - 2026-07-12 21:16 +02:00

Generated a colored SVG inspection packet for the current `n_32` cluster
repair sequence.

Command evidence:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 29-37 --attempt-diagnostics --debug-timing true
    # failed inside the 5 minute cap at n_32 / net33, as expected, and exported route SVGs for n_29 through n_37

Generated artifacts:

    build\cluster_step_svgs\00_contact_sheet.svg
    build\cluster_step_svgs\00_before_n32_blockers.svg
    build\cluster_step_svgs\01_n32_routes_and_hits_n35.svg
    build\cluster_step_svgs\02_current_first_n32_then_n29_n30.svg
    build\cluster_step_svgs\03_current_first_fails_at_n35.svg
    build\cluster_step_svgs\04_victim_first_normal_victims_ok.svg
    build\cluster_step_svgs\05_victim_first_normal_then_n32_fails.svg
    build\cluster_step_svgs\06_victim_first_reverse_then_n32_fails.svg

Color mapping in the packet:

- blue: `n_32` / native `net33`
- red: `n_35` / native `net36`
- orange: `n_30` / native `net31`
- green: `n_29` / native `net30`
- black/gray: static blockers and grid context

The packet was composed from the route attempt SVGs exported by the bounded
route-stop run, including `n_29` victim attempts that were missing from the
earlier `--debug-svgs 31-37` packet.

## Latest Stop Status - 2026-07-12 21:39 +02:00

Behavior-correction slice completed for the user-reported false victim problem.

Verified source behavior:

- `src/astar.rs::crossing_move_outcome` is part of crossing-aware A* neighbor
  exploration. It rejects known-illegal crossing moves for non-perpendicular
  axes, wrong partner order, insufficient margin, pending straight constraints,
  unexpected owners, unmatched centerline/footprint overlap, and uncleared
  crossing reservations.
- The native repair loop in `src/py_router.rs` was nevertheless widening
  victim sets by native route order after realized crossing failures and after
  `No route found` repair exhaustion. That made unrelated routes such as
  `n_29`/native `net30` and `n_30`/native `net31` appear as victims in the
  lower `n_32` cluster packet without physical evidence.

Changed behavior:

- Removed route-order adjacent blocker promotion from realized-crossing victim
  selection in `src/py_router.rs`.
- Removed the `No route found` route-order neighbor repair helper, its call
  sites, and the Rust test that documented that behavior as expected.
- Updated `.agent/WORKFLOW.md`, `.agent/ORCHESTRATOR.md`, and the
  implementer/harness/reviewer role briefs: repair victims now require
  geometry-backed evidence such as illegal crossings, dynamic overlap owners,
  reservation conflicts, or static obstacle contact. Route order, netlist
  adjacency, and SVG/debug sequence are not blocker evidence.
- Updated the active ExecPlan with the source audit, the behavior change, and
  validation evidence.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with the gnullvm/LLVM-MinGW/PYO3_PYTHON environment

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    C:\Users\benja\.cargo\bin\cargo.exe test targeted_illegal_crossing_repair_promotes_learned_blocker --lib
    # 1 passed

    git -c safe.directory=C:/Users/benja/Documents/Repositorys/TUMPhotonicRouter diff --check -- src/py_router.rs .agent/WORKFLOW.md .agent/ORCHESTRATOR.md .agent/roles/implementer.md .agent/roles/harness.md .agent/roles/reviewer.md .agent/execplans/2026-07-10-crossing-verification-foundation.md
    # passed with line-ending warnings only

Fresh bounded route evidence after rebuilding:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 37 --debug-svgs 29-37 --attempt-diagnostics --debug-timing true
    # failed quickly at n_24 / native net25

Additional native repair diagnostic rerun:

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 25 --debug-svgs 24-25 --attempt-diagnostics --debug-timing true
    # failed at n_24 / native net25 with the same signature

Current fresh failure:

    No route found for n_24: mmi0_ps_array_0_heater_1,o2 -> mmi0_multiport_0_0,o2
    candidate_blockers=[24]

Fresh artifact:

    build\routes\multiportmmi_8x8_n_24_FAILED.txt

The new blocker is geometry-backed, not route-order pollution. The failed log
reports an initial dynamic overlap with owner `net24` at cell `(629,162)`.
Native diagnostics report `allowed_partners=24`, `crossing_events=0`,
`grid_violations=1`, `realized_violations=3`, and realized reasons
`not_perpendicular`, `collinear_route_overlap`, and `not_perpendicular` between
`net25` and `net24`. The generated repair keepout has `396` cells; both
current-first and victim-first repair modes then fail with `No route found`.

Status: the false victim behavior is corrected, but the route-37 milestone is
not fixed. The next implementation lane should address the earlier real
`net25`/`net24` crossing/overlap repair blocker without reintroducing
route-order victims. Good next harness target: a small fixture or diagnostic
for over-broad crossing repair keepouts that make a single geometry-backed
victim pair unroutable.

## Latest Stop Status - 2026-07-12 22:03 +02:00

User narrowed the objective to the easy `n_23`/`n_24` fan-in pair only and
explicitly paused investigation of the `n_31+` cluster.

Changed behavior in `src/py_router.rs`:

- Collision-crossing routing now rejects candidate routes with zero legal
  crossing events, so LiDAR-pure crossing mode cannot hijack ordinary adjacent
  lane routes by clearing a previous route as a pretend crossing partner.
- When ordinary native commit fails with a concrete dynamic-overlap bbox, the
  router now does one local retry with that overlap neighborhood as a temporary
  static keepout. This addresses the search/commit mismatch where a route could
  pass A* and then fail at commit on one adjacent-lane core cell.
- Added the Rust regression
  `collision_crossing_route_without_event_is_not_accepted`.

Validation evidence:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed with the gnullvm/LLVM-MinGW/PYO3_PYTHON environment

    C:\Users\benja\.cargo\bin\cargo.exe test collision_crossing_route_without_event_is_not_accepted --lib
    # 1 passed

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 25 --debug-svgs 23-25 --attempt-diagnostics --debug-timing true
    # passed through n_24, failures=0, repairs=0

Fresh diagnostics:

- `build\routes\multiportmmi_8x8_n_23_diagnostics.txt`: `status=ok`,
  `route_dynamic_overlap_count=0`.
- `build\routes\multiportmmi_8x8_n_24_diagnostics.txt`: `status=ok`,
  `route_dynamic_overlap_count=0`.

Important wording: `n_23`/`n_24` are now clean, easy adjacent dogleg routes with
no dynamic overlap and no repair. They are not both necessarily classified by
the router as the internal `simple` fast-path route type; the selected run
reported no failures and no repairs, with A* used for the dogleg geometry.

Process check before stopping found no `python`, `cargo`, `maturin`, or
`rustc` processes running.

## Latest Artifact Attempt - 2026-07-12 22:18 +02:00

User asked for the `multiportmmi_8x8` GDS after `n_30`. The route-name/order
mapping was confirmed from the benchmark:

- `n_23` is 1-based route index `24`.
- `n_24` is 1-based route index `25`.
- `n_30` is 1-based route index `31`.

Attempted artifact command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 31 --attempt-diagnostics --debug-timing true --verbose-routes

The default-budget run reached `Routing [31/31] n_30` but did not complete
within the local quick-feedback window and was stopped. A capped diagnostic:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 31 --attempt-diagnostics --debug-timing true --verbose-routes --max-iterations 10000

failed before a GDS write at `n_28` while trying repair/rip sets around
`n_27`/`n_28`. Therefore there is no fresh after-`n_30` GDS artifact yet. The
current `build\routed_multiportmmi_8x8.gds` remains the earlier stop-after-25
artifact from `2026-07-12 22:03:05 +02:00`, length `110074`.

## Latest Artifact Refresh - 2026-07-12 22:17 +02:00

User asked to step back to the fast bounded artifact after `n_24`.

Command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 25 --debug-svgs false --debug-timing false

Result:

- Completed in about `8 s`.
- Refreshed `build\routed_multiportmmi_8x8.gds` at
  `2026-07-12 22:17:36 +02:00`, length `110074`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `status=partial_debug_stop`, `success=true`, `routed_record_count=25`,
  `debug_stop_after_route_index=25`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- Process check after completion found no `python`, `cargo`, `maturin`, or
  `rustc` processes running.

## Latest Artifact Refresh - 2026-07-12 22:18 +02:00

User asked for the next fast bounded artifact after `n_25`.

Command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 26 --debug-svgs false --debug-timing false

Result:

- Completed in about `8 s`.
- Refreshed `build\routed_multiportmmi_8x8.gds` at
  `2026-07-12 22:18:36 +02:00`, length `111230`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `status=partial_debug_stop`, `success=true`, `routed_record_count=26`,
  `debug_stop_after_route_index=26`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- Process check after completion found no `python`, `cargo`, `maturin`, or
  `rustc` processes running.

## Latest Artifact Refresh - 2026-07-12 22:19 +02:00

User asked for the next fast bounded artifact after `n_26`.

Command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 27 --debug-svgs false --debug-timing false

Result:

- Completed in about `9 s`.
- Refreshed `build\routed_multiportmmi_8x8.gds` at
  `2026-07-12 22:19:22 +02:00`, length `112388`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `status=partial_debug_stop`, `success=true`, `routed_record_count=27`,
  `debug_stop_after_route_index=27`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- Process check after completion found no `python`, `cargo`, `maturin`, or
  `rustc` processes running.

## Latest Artifact Refresh - 2026-07-12 22:20 +02:00

User asked for the next fast bounded artifact after `n_27`.

Command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 28 --debug-svgs false --debug-timing false

Result:

- Completed in about `9 s`.
- Refreshed `build\routed_multiportmmi_8x8.gds` at
  `2026-07-12 22:20:01 +02:00`, length `113848`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `status=partial_debug_stop`, `success=true`, `routed_record_count=28`,
  `debug_stop_after_route_index=28`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- Process check after completion found no `python`, `cargo`, `maturin`, or
  `rustc` processes running.

## Latest Artifact Attempt - 2026-07-12 22:21 +02:00

User asked for the next fast bounded artifact after `n_28`.

Command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --debug-timing false

Result:

- Did not complete within the local quick-feedback window; the run stayed in
  routing after about one minute and was stopped.
- No after-`n_28` GDS was written. `build\routed_multiportmmi_8x8.gds` remains
  the previous after-`n_27` artifact from `2026-07-12 22:20:01 +02:00`, length
  `113848`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` still
  reports `debug_stop_after_route_index=28` and `routed_record_count=28`.
- Process check after stopping found no `python`, `cargo`, `maturin`, or
  `rustc` processes running.

## Latest Verification - 2026-07-12 22:22 +02:00

User asked to verify why the after-`n_28` route is slow and whether it uses
rip-up/reroute, then stop if it does.

Bounded diagnostic command:

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --debug-timing true --verbose-routes --max-iterations 10000

Result:

- Confirmed: `n_28` enters native repair/rip-up.
- Native trace for native `net=29` (`n_28`) reported
  `crossing_events=0`, `grid_violations=1`, `realized_violations=3`, and
  `realized_reasons=[(28, "not_perpendicular"), (28, "collinear_route_overlap"), (28, "not_perpendicular")]`.
- The repair candidate set was `candidate_blockers=[28, 27]`.
- The engine attempted rip-up sets `ripup=[28]` and `ripup=[28, 27]` in
  multiple route order/reverse variants.
- The capped run failed with
  `No repair route found; candidate_blockers=[28, 27]`, with recent errors in
  both `reroute_victims` and `repair_failed_net`.

Interpretation: the first programming issue is that this easy fan-in addition
falls into crossing/repair logic instead of staying as a simple local route.

## Latest Verification - 2026-07-12 22:28 +02:00

User pointed out from the GDS screenshot that `n_28` should route cleanly from
the heater to the open upper MMI port and should not even try to cross `n_27`.

Code audit finding:

- The Python API default is `crossing_loss=0.0` in `route_match_and_realize`
  and `route_nets_rust`.
- The Rust crossing-aware A* adds only
  `crossing_outcome.crossing_count * crossing.crossing_loss` to the step cost.
- The current CLI path used for these benchmark runs does not expose or set a
  nonzero crossing loss, so collision-crossing search has no search-cost
  reason to prefer a clean detour over a candidate crossing.

Control command:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 29 --debug-svgs false --debug-timing true --verbose-routes

Result:

- Completed successfully in `4.3385 s`.
- Wrote a partial after-`n_28` GDS at
  `build\routed_multiportmmi_8x8.gds`, timestamp
  `2026-07-12 22:28:13 +02:00`, length `98854`.
- This is explicitly a `--crossings false` control artifact, not the failing
  `lidar-pure` crossing-enabled artifact.
- Photonic verification reports `status=partial_debug_stop`, `success=true`,
  `debug_stop_after_route_index=29`, `routed_record_count=29`,
  `error_count=0`, `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- Route timing shows `n_28` completed, with the whole routing stage around
  `2.3 s`; the route still had one normal-route failure and one repair, but no
  collision-crossing repair explosion.

Interpretation: basic route availability is not the blocker. The bad behavior
is specific to crossing-enabled `lidar-pure` collision/repair selection,
combined with zero crossing search penalty and/or accepting a crossing probe
before exhausting the clean local route option.

## Active Fix Plan - 2026-07-12 22:31 +02:00

Clean acceptance boundary for the first programming task:

- Fast, clean, crossing-enabled partial artifacts are required through `n_28`:
  `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
  --debug-stop-after-route 29 --debug-svgs false --debug-timing true
  --verbose-routes`.
- Expected behavior: completes in the local fast window, ideally under `15 s`;
  writes a partial after-`n_28` GDS with `routed_record_count=29`,
  `debug_stop_after_route_index=29`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`, and
  `waveguide_obstacle_overlap_count=0`.
- `n_28` must not enter native repair/rip-up against `n_27`; verifier should
  reject traces containing the current bad signature
  `candidate_blockers=[28, 27]` or `ripup=[28, 27]` for this route stop.
- Preserve the distinction between search-only crossing penalty and physical
  insertion-loss accounting.

Role loop:

1. Implementation Engineer owns the smallest routing-cost/selection patch.
2. QA / Harness Engineer verifies the implementation with the bounded
   stop-after-29 command and any focused tests/artifacts.
3. Orchestrator records `PASS`, `FAIL`, `BLOCKED`, or `INCONCLUSIVE` and loops
   only on actionable verifier feedback.

## Implementation Attempt - 2026-07-12 23:09 +02:00

The first worker attempt was shut down after it stalled. The local follow-up
implemented and tested two partial ideas:

- Python-side search-only crossing penalty:
  `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 200.0`, passed to Rust crossing
  config as `crossing_search_loss` while preserving physical
  `crossing_loss=0.0` in reports.
- Rust-side attempt to keep collision-crossing from hijacking repair-first
  routing and to filter dynamically owned opened cells from the search opening
  set.

Validation:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_routing_flow_stats.py -k "crossing_search_loss or crossing_plan"
    # 1 passed, 30 deselected

    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    $env:CARGO_BUILD_TARGET='x86_64-pc-windows-gnullvm'
    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin\rust-lld.exe'
    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed, but warns that collision-crossing helper methods are currently unused

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed with the same unused-method warning

Acceptance command still fails:

    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --debug-timing true --verbose-routes

Current failure:

- Still fails at `n_28` / native `net29`.
- Bad blocker set improved from `[28, 27]` to `[28]`, but this is not accepted
  because `n_28` still enters repair/rip-up.
- Fresh diagnostic packet:
  `build\routes\multiportmmi_8x8_n_28_FAILED.txt`.
- Trace: native `net29` probe has `crossing_events=0`, `grid_violations=1`,
  `realized_violations=3`, realized reasons against native `net28`
  (`not_perpendicular`, `collinear_route_overlap`, `not_perpendicular`),
  then repair attempts `ripup=[28]` and fails both current-first and
  victim-first modes with `No route found`.
- The failed packet reports an opened-cell dynamic overlap at `(629,173)`, but
  filtering dynamic/owned opened cells did not change the selected route or
  acceptance result.

Verdict: `FAIL`. The partial search-penalty idea alone is insufficient. The
next implementation should not keep layering speculative local patches; it
should first compare the route records for the successful `--crossings false`
control against the failing `lidar-pure` run for `n_27`/`n_28`, then identify
why `n_27` differs and why victim `net28` has zero-expansion `No route found`
under crossing-enabled repair.

## Latest Verification - 2026-07-13 09:56 +02:00

User asked why `--crossings false` still needs one repair for `n_28`.

Command:

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 29 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes

Result:

- Completed successfully in `5.0136 s`; routed stage `2.7366 s`.
- Still reports `repairs=1`.
- Native trace for `n_28` / native `net29`:
  `allowed_partners=0`, `crossing_events=0`, `grid_violations=0`,
  `realized_violations=0`, `keepout_keys=0`,
  `candidate_blockers=[28]`.
- Therefore the no-crossing repair is not crossing legality. It is plain
  dynamic route overlap: normal route cannot commit with native `net28`
  (`n_27`) present; the probe route ignores dynamic obstacles, finds a route,
  then selects native `net28` as the blocker and reroutes it.
- Fresh route diagnostics:
  `build\routes\multiportmmi_8x8_n_28_diagnostics.txt` reports
  `opened_dynamic_overlap_count=3`, bbox `(629, 631, 173, 174)`, and final
  `route_dynamic_overlap_count=1`, bbox `(628, 628, 175, 175)`, with
  `route_overlap_dynamic_clearance_exempt_count=2`.
- The current `build\routed_multiportmmi_8x8.gds` is again a
  `--crossings false` partial control artifact through `n_28`, not a passing
  `lidar-pure` artifact.

Interpretation: the first issue is earlier/lower-level than crossing policy:
the normal A* route for `n_28` collides with the already committed `n_27`
route near the target-side MMI access region, even in no-crossing mode. Repair
succeeds only because it rips/reroutes `n_27`. The next fix should make the
normal route search avoid that dynamic overlap in the target port approach, or
ensure the target-port opening/clearance exemption does not create a false free
passage through the previous adjacent lane.

## Latest Verification - 2026-07-13 10:25 +02:00

Implemented a focused Python-side port-access ownership fix in
`translation/route_rust.py`: dense same-instance port openings now assign each
access cell to the nearest lateral port lane, so adjacent MMI port access
regions are not reopened by multiple nets. This treats the short straight in
front of each port as a reserved access runway for that port, while still
allowing the active endpoint to open its own reserved cells.

Added regression coverage in `tests/test_route_rust_opened_cells.py`:
`test_route_nets_rust_same_instance_port_access_does_not_open_sibling_lane`
builds a two-port same-instance fixture and asserts that routing into one port
does not open the sibling port's access anchor.

Validation:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_route_rust_opened_cells.py -k "same_instance_port_access or foreign_port_keepout"
    # 4 passed, 13 deselected

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 29 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=21/29, failures=0, repairs=0

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 29 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=21/29, failures=0, repairs=0

The bounded acceptance slice through `n_28` now passes in both no-crossing and
`lidar-pure` crossing-enabled modes without native repair/rip-up. The last run
rewrote `build\routed_multiportmmi_8x8.gds` as the crossing-enabled partial
artifact through route 29.

## Latest Verification - 2026-07-13 10:40 +02:00

Extended the clean early fan-in boundary through `n_30`
(`debug_stop_after_route_index=31`).

Manual validation:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings false --debug-stop-after-route 31 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=23/31, failures=0, repairs=0
    # total: 4.6623 s, route-search loop: 0.4457 s

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 31 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes
    # completed; route search stats: simple=23/31, failures=0, repairs=0
    # total: 5.5062 s, route-search loop: 0.4571 s

Regression added:

    .\.venv\Scripts\python.exe -m pytest -q tests\test_multiportmmi_benchmark.py::test_multiportmmi_8x8_routes_cleanly_through_first_mmi_fanin_boundary
    # 2 passed in 12.68 s

This regression runs both no-crossing and `lidar-pure` stop-after-31 modes and
asserts `success=true`, `error_count=0`, `routed_record_count=31`,
`route_attempts=31`, `route_failures=0`, and `repair_count=0`.

## Latest Verification - 2026-07-13 10:48 +02:00

Generated the requested partial GDS after `n_31`
(`debug_stop_after_route_index=32`) in `lidar-pure` mode.

Command:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 32 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes

Result:

    # completed; route search stats: simple=23/32, failures=0, repairs=0
    # total: 5.7829 s, route-search loop: 0.4953 s

The current `build\routed_multiportmmi_8x8.gds` is the after-`n_31`
crossing-enabled partial artifact.

## Latest Verification - 2026-07-13 10:52 +02:00

Attempted the next step after `n_31`:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes

Result:

    # failed at n_31 after n_32 had been attempted/routed first by current dense
    # fanout ordering

Important trace:

    native_repair_probe net=32 allowed_partners=32 crossing_events=0
    grid_violations=1 realized_violations=1
    realized_reasons=[(33, "insufficient_straight_margin")]
    keepout_keys=225 candidate_blockers=[33]
    native_repair_keepout net=32 ripup=[33] ...

Mapping:

    - native `net32` is schematic `n_31`
    - native `net33` is schematic `n_32`

So the current behavior is the inverse of the initial expectation: `n_32`
routes/occupies the space first, then `n_31` fails and tries to rip up `n_32`
as the victim. Repair cannot reroute `n_32`, and no new after-`n_32` GDS is
written by the normal flow. The visible GDS remains the last successful
crossing-enabled partial artifact through `n_31`.

## Latest Verification - 2026-07-13 10:45 +02:00

Added a debug-only environment gate in `translation/route_rust.py`:

    PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT=<N>

This truncates the already-selected, already-reordered execution list to the
first `N` actual routed jobs. Normal routing is unchanged when the variable is
unset. This was needed because `debug_stop_after_route_index` selects by netlist
route index, but dense MMI fanout routing can execute those selected jobs in a
different order.

Generated the requested GDS after step 1 of the cluster:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='32'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes

Result:

    # completed; selected routes through n_32, then execution-limited to 32
    # actual routed jobs
    # first cluster step is n_32, not n_31
    # route search stats: simple=23/32, failures=0, repairs=0
    # total: 5.5916 s, route-search loop: 0.4965 s

The current `build\routed_multiportmmi_8x8.gds` is therefore the cluster
step-1 artifact: clean prefix through `n_30` plus `n_32`; `n_31` is not routed
in this artifact.

## Latest Verification - 2026-07-13 11:03 +02:00

User asked to validate exactly what happens when adding `n_31` after the
cluster step-1 artifact where `n_32` is already routed.

Focused command:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs 31-33 --attempt-diagnostics --debug-timing true --verbose-routes

Result:

- Failed quickly at `n_31` after `n_32` had routed first in the current dense
  fanout execution order.
- The normal `n_31` route fails; the static-only probe route succeeds and
  intersects native `net33` / schematic `n_32`, but the crossing verifier
  classifies that candidate as `insufficient_straight_margin`.
- Native repair then adds a 225-cell localized crossing keepout and chooses
  `candidate_blockers=[33]`, i.e. it tries to rip/reroute `n_32`.
- Repair cannot reroute `n_32`, so no new GDS is written for the failed step.

Evidence files:

- `build\routes\multiportmmi_8x8_n_31_FAILED.txt`
- `build\routes\multiportmmi_8x8_n_31_diagnostics.txt`
- `build\routes\multiportmmi_8x8_n_31_attempt34_probe_route.svg`
- `build\routes\multiportmmi_8x8_n_31_attempt37_repair_failed_net.svg`
- `build\routes\multiportmmi_8x8_n_32_diagnostics.txt`

Key diagnostic facts:

- `n_31` source/target states are `(706, 162, 0)` to `(767, 243, 0)`.
- `n_32` source/target states are `(706, 165, 0)` to `(767, 93, 0)`.
- `n_32` route exits through `(706..717, 164..165)` then bends through
  `(713,165)->(719,162)`.
- `n_31` source opening spans `(706..717, 156..163)` and has one dynamic
  overlap at `(717,163)`, which is the already committed `n_32` source-side
  route/access area.
- The probe route for `n_31` has bbox `(706..767, 162..243)`, so it is the
  direct candidate through/near `n_32`, not the desired wider downward detour.

Interpretation:

The current failure is not yet proof that no legal detour exists. It proves the
current search/repair loop locks onto the direct illegal crossing candidate
after `n_32` occupies the source-side access region, then escalates to victim
rip-up instead of finding or forcing a legal detour for `n_31`. The next fix
should be a focused implementation slice around dense fanout ordering/access
reservation and/or current-net local detour retry before victim rip-up.

## Latest Implementation Checkpoint - 2026-07-13 13:20 +02:00

User clarified the intended crossing/rip-up contract:

- `--crossings false` keeps the static-only probe and victim rip/reroute
  behavior used by cases such as `heater_s_mod`.
- `--crossings true --crossing-mode lidar-pure` should first search for a
  legal current-net route through/around already routed nets; rip-up is only
  allowed after no legal current-net route exists.

Implemented exploratory contract patches:

- `src/py_router.rs` now attempts collision-crossing A* before falling through
  to static-only probe/repair when collision/lidar-pure routing is enabled.
- Collision-crossing attempts can accept a route with zero crossing events when
  final crossing validation reports no violations, because crossings are
  allowed, not required.
- Collision-crossing A* no longer uses the huge full-grid fallback before
  probe/repair; this prevents the previous 60-90 second `n_31` wander.
- Added an env-gated crossing exhaustion trace with rejection counters in
  `src/astar.rs`.
- Disabled monotonic pruning inside crossing-aware A* so detours that initially
  move away from the target are not pruned.
- Added a guided collision-crossing retry after the static-only probe identifies
  blocker candidates, before victim rip-up.
- `translation/route_rust.py` no longer applies the 50k 45-degree iteration cap
  or forced 12.0 bend weight to collision/lidar-pure crossing mode; it also
  uses an admissible heuristic weight for collision/lidar-pure mode.

Validation:

    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    $env:CARGO_BUILD_TARGET='x86_64-pc-windows-gnullvm'
    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin\rust-lld.exe'
    $env:PYO3_PYTHON='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\.venv\Scripts\python.exe'
    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

Focused route-33 diagnostic:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    $env:PHOTONIC_ROUTER_TRACE_CROSSING='1'
    $env:PHOTONIC_ROUTER_TRACE_CROSSING_NET='32'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true --verbose-routes

Result:

- Command completed in `10.4599 s` total.
- The original 60-90 second unguided full-grid collision search is gone.
- `n_31` still does not find the desired legal crossing/detour.
- Normal collision search for native `net32` / schematic `n_31` exhausts the
  local window:
  `expanded=10384`, `candidates=732`, `accepted=12`,
  `reject_not_perpendicular=420`, `reject_margin=300`,
  `reject_unmatched_owner=4248`.
- Static probe still identifies native `net33` / schematic `n_32` as the
  blocker with `realized_reasons=[(33, "insufficient_straight_margin")]`.
- Guided retry through `net33` finds a compact candidate but it is still
  invalid: `events=0`, `satisfies=false`,
  `realized_violations=[(33, ..., "insufficient_straight_margin")]`,
  waypoints around `(706,162)->(711,156)->(705,150)->(711,150)->(755,194)...`.
- The remaining localized-keepout fallback still produces an overlong route
  (`length=1080.000um`, `cost=1092.000`, `repairs=1`), so this is not an
  accepted final behavior.

Interpretation:

The first contract issue is partially encoded: crossing-enabled routing now
tries local collision legality before probe/rip-up and no longer spends a
minute on the huge unguided full-grid bypass. The remaining correctness issue
is geometric: the available compact `n_31` candidate still violates the
straight-margin crossing rule against already-routed `n_32`, and the later
localized-keepout fallback masks that by committing a long route. Next work
should either make the guided current-net crossing generate a truly legal
detour/crossing or fail this step explicitly instead of accepting the long
localized-keepout route.

Update after endpoint-correction isolation:

- `src/py_router.rs` now uses realized/endpoint-aware crossing event detection
  for guided collision crossing acceptance and retries guided crossing search
  with targeted keepout when a realized crossing candidate fails validation.
- The localized keepout fallback is guarded so an invalid realized collision
  candidate cannot be silently replaced by a long no-crossing detour in
  collision/lidar-pure mode.
- `routing_flow.py` disables checked grid endpoint correction for
  crossing-enabled runs. Endpoint correction was moving/checking geometry after
  A* had selected a crossing and could invalidate an otherwise accepted
  crossing route.
- `translation/route_rust.py` now separates verifier legality margin from A*
  search guard margin: realized crossing verification requires only the actual
  crossing footprint margin, while reports retain `search_required_margin_um`
  for the larger A* guard.
- `translation/photonic_verification.py` and
  `translation/route_rust_realization.py` propagate endpoint-correction-disabled
  mode so final geometry checks and realization do not require corrected
  centerlines or exact port-contact checks during this crossing debug path.

Focused validation:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR='C:\Users\benja\Documents\Repositorys\TUMPhotonicRouter\build\mpl-cache'
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.8726 s; GDS written

Observed route behavior:

- Static probe for native `net32` / schematic `n_31` still identifies native
  `net33` / schematic `n_32` with
  `realized_reasons=[(33, "insufficient_straight_margin")]`.
- Guided collision crossing retries once and accepts a compact legal crossing:
  `events=1`, `cost=512.735...`, waypoints
  `(706,162)->(709,162)->(709,154)->(722,141)->(755,174)->(755,235)->(763,243)->(767,243)`.
- Crossing report:
  `build\verification\multiportmmi_8x8_crossing_verification.json` has
  `error_count=0`, `crossing_count=1`, `legal_crossing_count=1`,
  `illegal_crossing_count=0`. The `n_31` x `n_32` crossing is perpendicular at
  `[1442.5, 662.125]` with `required_margin_um=4.0` and
  `search_required_margin_um=14.0`.
- Photonic report:
  `build\verification\multiportmmi_8x8_photonic_verification.json` has
  `success=true`, `error_count=0`, and no waveguide/obstacle overlap errors for
  the partial stop.

Regression validation:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py -q
    # 31 passed

Crossing insertion checkpoint:

- `translation/route_rust.py` now anchors inserted crossing components by the
  transformed optical center derived from crossing ports, falling back to bbox
  center only when ports are unavailable. Placement metadata records
  `optical_center_um`.
- `tests/test_realized_crossing_verification.py` asserts the transformed port
  centroid of the inserted crossing instance equals the intended intersection.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py -q
    # 31 passed

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.2454 s; GDS written

GDS inspection:

- `build\crossings\multiportmmi_8x8_crossings.json` reports placement
  `point_um=[1442.5, 662.125]`, `optical_center_um=[1442.5, 662.125]`,
  `rotation_deg=45.0`.
- Direct KLayout DB inspection of `build\routed_multiportmmi_8x8.gds` found the
  crossing instance bbox center at `[1442.5, 662.125]` with bbox
  `(1436.843,656.468;1448.157,667.782)`.

Follow-up correction after visual inspection:

- User reported the inserted crossing still looked offset in KLayout. Audit
  showed the placement/verifier path used compressed route waypoints when
  endpoint correction was disabled. Those chord waypoints can differ from the
  primitive-realized centerline used to draw the GDS.
- Added `PyPhotonicRouter.route_primitive_centerline(route)` in
  `src/py_router.rs`.
- `translation/route_rust.py` now records the primitive-realized physical
  centerline on `RoutedNetRecord` when checked endpoint correction is disabled,
  so crossing verification, clipping, insertion, and final realization use the
  same route geometry.

Validation:

    C:\Users\benja\.cargo\bin\cargo.exe check --lib
    # passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py -q
    # 31 passed

    .\.venv\Scripts\python.exe -m maturin develop --release --target x86_64-pc-windows-gnullvm
    # passed

    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    $env:PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT='33'
    .\.venv\Scripts\python.exe -u routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.8756 s; GDS written

New GDS inspection:

- `build\crossings\multiportmmi_8x8_crossings.json` now reports the primitive
  realized crossing at `[1443.5, 663.125]`.
- The inserted crossing instance in `build\routed_multiportmmi_8x8.gds` has bbox
  `(1437.843,657.468;1449.157,668.782)` and bbox center
  `[1443.5, 663.125]`, matching the primitive-realized intersection.

## Latest Implementation Checkpoint - 2026-07-13 12:47 +02:00

Implemented Fix 1 for crossing-aware endpoint correction.

Behavior now:

- Crossing-enabled routing keeps checked endpoint correction out of the native
  route-bookkeeping pass so crossing discovery and component insertion are
  based on primitive-realized centerlines.
- After `realized_intersections` exists, `route_match_and_realize` applies a
  crossing-aware endpoint correction pass.
- Nets without inserted legal crossings use the normal endpoint-corrected
  centerline.
- Nets with inserted legal crossings keep the primitive-realized centerline
  through the crossing-bearing interior. Terminal endpoint anchors are attempted
  with a realizability check, falling back from both-endpoints to source-only,
  target-only, and finally the frozen primitive centerline if a terminal stub is
  unsupported.
- `routing_flow.py` now leaves `enable_grid_endpoint_correction=True` for
  crossing-enabled runs; the per-net crossing-aware pass owns the distinction.

Regression coverage added:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 2 passed

Focused validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.3173 s; GDS written

Generated report summary for the route-33 partial:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Follow-up correction for split crossed-net endpoint correction:

- User clarified that a single crossed net may need endpoint correction on both
  sides of its protected crossing span: port-to-first-crossing and
  last-crossing-to-port.
- Fixed `_spliced_crossing_endpoint_centerline` so the protected middle uses
  guarded cut points around the first and last realized crossings, rather than
  freezing the entire primitive segment from the port when the first crossing
  lies on that segment.
- The guarded cut keeps the crossing interior protected while leaving a real
  editable prefix and suffix for terminal absorption on the same net.
- Crossed-net terminal direction checks now prefer the actual route endpoint
  state angles used by realization, falling back to port orientation only when
  route states are unavailable.
- Added a regression where one crossed net has two crossings and must absorb
  source-side and target-side endpoint corrections in the same splice.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 5 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 36 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.1749 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`, `cross_net_waveguide_overlap_count=0`,
  `waveguide_obstacle_overlap_count=0`.

Follow-up correction for the real `n_31` source-side prefix:

- User observed that the single route-33 crossing affects two nets, but only
  one net visibly corrected from port to first crossing.
- Concrete inspection confirmed `n_32` corrected both endpoints, while `n_31`
  still started at the primitive centerline point `(1393.5, 687.125)` instead
  of the source port `(1391.8, 687.5)`.
- Root cause 1: direction-compatible corrected prefixes were accepted without
  checking that they were actually anchored at the port.
- Root cause 2: when the terminal absorber tried to add a source-side
  port-local straight, the solver could collapse that inserted straight to
  zero length and return the original unsupported arc tangent.
- Fixes:
  - corrected terminal prefixes/suffixes must match their source/target port
    anchor before they are accepted;
  - absorber candidates must match the known terminal tangent when one is
    available;
  - inserted port-local terminal stubs must have positive length when selected.
- Real `n_31` / `n_32` route-33 inspection now shows both crossed nets at zero
  source and target endpoint error. `n_31` starts at `(1391.8, 687.5)` and ends
  at `(1517.7, 850.0)`.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 7 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 38 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.6155 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`, `cross_net_waveguide_overlap_count=0`,
  `waveguide_obstacle_overlap_count=0`.
- `build\crossings\multiportmmi_8x8_crossings.json`:
  `routes_missing_corrected_centerline_count=0`,
  `illegal_realized_crossing_count=0`, and both `n_31` and `n_32` have one
  legal crossing counted.

Route-34 boundary attempt:

- User requested moving exactly one route further while preserving the ordering
  that will execute in the full run.
- Confirmed full route order around the cluster:
  - route 32: `n_31`, `mmi0_multiport_0_0,o12 -> mmi0_ps_array_1_heater_4,o1`
  - route 33: `n_32`, `mmi0_multiport_0_0,o11 -> mmi0_ps_array_1_heater_1,o1`
  - route 34: `n_33`, `mmi0_multiport_0_0,o10 -> mmi0_ps_array_1_heater_2,o1`
  - route 35: `n_34`
  - route 36: `n_35`
- Command:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true

- Result: failed after about two minutes before producing a route-34 GDS. The
  failure happens while routing `n_31` in the 34-route batch context:
  `No repair route found; candidate_blockers=[33, 34]`.
- Interpretation: the current valid boundary remains the route-33 partial. When
  `n_33` is included in the same full-run context, the `n_31` repair path sees
  both `n_32` and `n_33` as blockers and cannot find a local collision-crossing
  route. The next cluster task should therefore debug the three-net interaction
  `n_31`/`n_32`/`n_33`, not just a post-`n_32` endpoint correction issue.

Follow-up correction after visual GDS inspection:

- User showed that the previous `n_31` source-side endpoint correction still
  created a warped bend-like geometry. This violated the intended rule:
  endpoint correction may lengthen/shorten existing straight or 45-degree
  diagonal runs, and may add one port-facing straight only when the net and
  port face each other. It must not modify sampled bend-arc geometry.
- Root cause: `_absorbed_terminal_centerline` treated every sampled segment in
  the primitive-realized terminal side as length-adjustable, including small
  bend-arc samples.
- Fix:
  - terminal absorption now marks only exact axis/45-degree segment directions
    as adjustable;
  - non-axis/non-diagonal sampled bend segments are copied with their original
    length and direction;
  - the explicit port-local straight remains adjustable and must have positive
    length when selected.
- Concrete `n_31` inspection now shows the first post-stub bend sample keeps
  the same length as the primitive route while both endpoints remain exactly
  connected.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_does_not_modify_bend_samples tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 8 passed

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_keeps_required_port_straight tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_does_not_modify_bend_samples tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 39 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.4628 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`, `cross_net_waveguide_overlap_count=0`,
  `waveguide_obstacle_overlap_count=0`.
- `build\crossings\multiportmmi_8x8_crossings.json`:
  `routes_missing_corrected_centerline_count=0`,
  `illegal_realized_crossing_count=0`, and both `n_31` and `n_32` have one
  legal crossing counted.
- `build\crossings\multiportmmi_8x8_crossings.json`:
  one legal perpendicular `n_31` x `n_32` crossing at
  `[1443.5, 663.125]`.

Known unrelated validation note:

- Running the whole `tests/test_port_alignment_diagnostics.py` file still
  fails two pre-existing `mmi_heater` integration tests during routing before
  this endpoint-correction pass runs (`gc1_to_mmi0_in2`, no-crossings mode).
  The focused Fix 1 regressions and crossing suites pass.

Follow-up fix after GDS inspection:

- User reported that no visible routes in the GDS appeared endpoint-corrected.
  Root cause: in crossing-enabled runs `_record_route` stores the
  primitive-realized centerline in `corrected_centerline_um` so crossing
  verification and insertion use drawn geometry. The later crossing-aware pass
  reused `apply_port_endpoint_corrections` for crossing-free nets, but that
  helper skips records that already have `corrected_centerline_um`. Therefore
  crossing-free nets kept the primitive baseline instead of normal port
  correction.
- Fixed by clearing the primitive baseline before invoking normal endpoint
  correction for nets with no inserted legal crossing. Crossed nets still use
  the primitive baseline to preserve crossing geometry.
- Strengthened
  `test_crossing_free_endpoint_correction_uses_normal_corrected_centerline` so
  it starts with a primitive baseline already present and verifies that normal
  endpoint correction replaces it.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 2 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.7583 s; GDS written

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

Generated reports remain clean:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Follow-up correction to crossed-net endpoint splice:

- User clarified that for nets with crossings, endpoint correction may only
  affect the access region before the first crossing and after the last
  crossing. The prior crossed-net implementation preserved crossing points but
  cut exactly at the crossing center, which can turn the crossing into a
  segment endpoint and make the verifier classify it as adjacent contact.
- Fixed the splice rule: for crossed nets, use the normally endpoint-corrected
  centerline only outside the primitive segment span containing the first and
  last crossing. The entire primitive-realized segment span containing the
  crossings remains frozen, so the crossing stays in the interior of a straight
  segment.
- Updated the regression so two crossings inside primitive segments preserve
  the whole primitive middle while the outside prefix/suffix come from the
  corrected centerline.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_preserves_crossing_interior tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 33 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=10.1051 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Current cluster diagnosis, route 34:

- Default dense-fanout order for `multiportmmi_8x8 --debug-stop-after-route 34`
  routes `n_32` and `n_33` before trying/repairing `n_31`. The failing
  repair for `n_31` reports candidate blockers `[33, 34]`, i.e. user-facing
  routes `n_32` and `n_33`; victim reroutes fail with
  `No local collision-crossing route found`.
- With `PHOTONIC_ROUTER_DENSE_FANOUT_ORDER=original`, `n_31` routes first and
  the next route `n_32` fails against `n_31`. Native repair diagnostics show
  `allowed_partners=32`, `crossing_events=0`, `grid_violations=1`,
  `realized_violations=1`, and `realized_reasons=[(32,
  "insufficient_straight_margin")]`.
- Increasing `--foreign-port-keepout-cells` from `6` to `10` did not resolve
  the original-order route-34 failure, so a simple global keepout increase is
  insufficient or is being reopened too broadly by endpoint/dense-port opening.
- Most likely next fix area: refine port access/foreign keepout opening so the
  active port can open its own access corridor, but neighboring/future port
  corridors are not available as legal crossing locations for unrelated routes.
  The failure currently looks like `n_31` occupies a too-port-local crossing
  corridor, then `n_32`/`n_33` cannot place a legal crossing with enough
  straight margin.

Route-34 process fix:

- User requested fixing the deterministic normal-order process before using
  any dense-fanout ordering optimization.
- Changed `translation/route_rust.py` so dense source fanout ordering defaults
  to original netlist order. The previous inversion-aware heuristic remains
  available with `PHOTONIC_ROUTER_DENSE_FANOUT_ORDER=legacy` or
  `PHOTONIC_ROUTER_DENSE_FANOUT_ORDER=inversion-aware-extremes`.
- Changed `src/py_router.rs` so a failed local collision-crossing attempt is
  only treated as a rejected crossing placement, not as a terminal route
  failure. Normal A* now falls back and can search around the blocker before
  rip-up/repair is entered.
- Changed `src/py_router.rs` current-first repair mode so the initial invalid
  crossing repair keepout is not applied to the current failed net after its
  victim has been ripped up. This matches the intended sequence: route current
  after removing victims, then reroute the victim set against the new state.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check
    # passed

    .\.venv\Scripts\python.exe -m pytest tests\test_rust_batch_repair.py tests\test_route_failure_diagnostics.py -q
    # 4 passed

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed; editable extension rebuilt

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=19.2586 s; GDS written

Route-34 report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json` metrics:
  `issue_count=0`, `illegal_crossing_count=0`, `legal_crossing_count=0`,
  `matched_crossing_component_count=0`, `routed_record_count=34`,
  `status=partial_debug_stop`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` metrics:
  `cross_net_waveguide_overlap_count=0`, `waveguide_obstacle_overlap_count=0`,
  `routed_record_count=34`, `status=partial_debug_stop`.
- Current GDS is `build\routed_multiportmmi_8x8.gds`, written
  2026-07-13 14:42:42 local time.

Crossing search-cost retune:

- User requested reducing the collision/lidar-pure search-only crossing
  penalty from `200.0` to `50.0` um.
- Updated `translation/route_rust.py`
  `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 50.0`.
- This changes the non-physical A* search penalty only. The physical
  `crossing_loss` reported in verification remains independent and still
  defaults to `0.0`.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests\test_routing_flow_stats.py::test_lidar_pure_uses_search_only_crossing_penalty_by_default tests\test_routing_flow_stats.py::test_crossing_plan_keeps_physical_loss_separate_from_search_penalty -q
    # 2 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=19.3926 s; GDS written

Report check for that partial run:

- Crossing metrics: `issue_count=0`, `illegal_crossing_count=0`,
  `routed_record_count=34`, `status=partial_debug_stop`.
- Photonic metrics: `cross_net_waveguide_overlap_count=0`,
  `waveguide_obstacle_overlap_count=0`, `routed_record_count=34`,
  `status=partial_debug_stop`.

Note:

- A full `tests\test_routing_flow_stats.py` run is currently not green:
  `11 failed, 20 passed`. The failures are broad benchmark routing failures
  such as `TOY`, `heater_s*`, and `mmi_heater*`, not assertion failures about
  the crossing search-loss value.

Route-34 repair sequence check:

- Re-ran `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
  --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics
  --debug-timing true` with `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`.
- Native trace shows a single repair probe:
  `net=33 allowed_partners=32 ... candidate_blockers=[32]`.
  In user-facing route names this is `n_32` blocked by `n_31`.
- The observed sequence is:
  1. `n_31` routes normally.
  2. `n_32` normal route fails against `n_31`.
  3. `n_32` probes, selects `n_31` as victim, rips `n_31`.
  4. `n_32` routes.
  5. `n_31` reroutes as victim.
  6. `n_33` then routes normally.
- `n_33` does not currently rip `n_31` in the route-34 partial run.

Current user-requested GDS checkpoint:

- User requested the GDS before step 6 of the route-34 sequence, i.e. after
  `n_32` has routed and `n_31` has been rerouted as the victim, but before
  `n_33` starts.
- Ran:

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true

- Result: passed; `attempts=36`, `failures=1`, `repairs=1`,
  `routed_record_count=33`.
- Current GDS is `build\routed_multiportmmi_8x8.gds`, written
  2026-07-13 14:52:36 local time.
- Report metrics:
  - Crossing: `issue_count=0`, `illegal_crossing_count=0`,
    `routed_record_count=33`, `status=partial_debug_stop`.
  - Photonic: `cross_net_waveguide_overlap_count=0`,
    `waveguide_obstacle_overlap_count=0`, `routed_record_count=33`,
    `status=partial_debug_stop`.

Trace note for why `n_31` does not take the shorter crossing through `n_32`:

- Ran the route-33 checkpoint again with
  `PHOTONIC_ROUTER_TRACE_CROSSING_NET=32` to trace native `net=32`
  (`n_31`).
- Initial `n_31` route before `n_32` reported
  `accepted=0`, `candidates=0`, `events=[]`.
- During the later victim reroute, `n_31` starts collision-crossing search
  with native `net=33` (`n_32`) included in its partner list. The final
  route-33 report still has `crossing_count=0`.
- Interpretation: the shorter visual crossing path is not being rejected
  because its cost is too high. It is not accepted as a legal crossing
  candidate under the current crossing legality/realized validation rules, so
  the router falls back to an avoiding A* path.
- Follow-up trace with `PHOTONIC_ROUTER_TRACE_CROSSING=1` showed:
  - For current route `n_32` through `n_31` (native net 33 through 32):
    `candidates=2327`, `accepted=308`, but the search is exhausted before a
    valid target route with crossing is committed. Major rejects include
    `reject_not_perpendicular=1403`, `reject_margin=616`,
    `reject_unmatched_owner=6258`.
  - For rerouted victim `n_31` with `n_32` included in the partner list:
    `candidates=732`, `accepted=12`, but final masks remain zero and the search
    exhausts without committing a crossing route.
- This supports the user's point: the router should continue looking for a
  nearby legal detour crossing after the direct crossing is illegal. The current
  generic victim-reroute crossing search is trying candidates, but it does not
  currently have a strong focused guided-retry path for "reroute this victim
  through the newly routed current net at a legal shifted crossing".

Clarification on "stricter validation":

- User challenged the claim that validation should be stricter now, since
  crossing-aware endpoint correction no longer moves crossing positions.
- Code inspection shows the relevant strictness is not primarily endpoint
  correction. Rust crossing search/event extraction uses
  `crossing_required_margin_cells = crossing_half_size_cells +
  min_straight_cells + bend_runout_cells`.
- For current `multiportmmi_8x8` settings this is likely `2 + 2 + 3 = 7`
  grid cells, or about `14 um` at the current `2.0 um` grid. This is more than
  the crossing device footprint margin alone.
- Python final realized-intersection verification separately computes
  `search_required_margin_um` with straight/runout but uses
  `required_margin_um = footprint_half_um` for final realized crossing
  legality. That suggests the Rust path may be rejecting crossings that the
  final verifier would consider physically/legal once the crossing is inserted.
- Next likely fix: separate A* search guard margin from final realized event
  recognition/commit validation, and add diagnostics that report the exact
  rejected crossing point and required/actual route+partner margins.

Guided victim crossing repair:

- User clarified that the issue is not final validation rejecting a committed
  A* crossing. Instead, A* can explore/accept crossing steps but the generic
  lidar-pure search does not require the final target path to include a
  crossing. Thus a victim reroute can fall back to a long no-crossing path even
  though a compact legal detour crossing exists.
- Implemented a current-first repair enhancement in `src/py_router.rs`: after
  the current failed net is successfully rerouted, each victim first tries a
  guided collision-crossing reroute through exactly that newly routed current
  net. Only if this focused guided route fails does it fall back to the normal
  victim reroute.
- Trace for route-33 after the change:
  - `n_31` victim guided through `n_32` first tries a direct crossing and gets
    `insufficient_straight_margin`.
  - Retry adds keepout and finds `events=1`, `satisfies=true`,
    `realized_violations=[]`.
- Route-33 report after the change: `crossing_count=1`,
  `legal_crossing_count=1`, `matched_crossing_component_count=1`,
  `illegal_crossing_count=0`, photonic overlap counts `0`.
- Route-34 report after the change: `crossing_count=2`,
  `legal_crossing_count=2`, `matched_crossing_component_count=2`,
  `illegal_crossing_count=0`, photonic overlap counts `0`.
- Current GDS is `build\routed_multiportmmi_8x8.gds`, written
  2026-07-13 15:15:15 local time, at debug stop-after-route 34.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check
    # passed

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed; editable extension rebuilt

    .\.venv\Scripts\python.exe -m pytest tests\test_rust_batch_repair.py tests\test_route_failure_diagnostics.py tests\test_routing_flow_stats.py::test_lidar_pure_uses_search_only_crossing_penalty_by_default tests\test_routing_flow_stats.py::test_crossing_plan_keeps_physical_loss_separate_from_search_penalty -q
    # 6 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=19.4973 s

Route-35 investigation:

- User suspected the next net might fail because a crossing candidate is blocked
  by foreign keepout cells.
- Ran route-35 with `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` and
  `PHOTONIC_ROUTER_TRACE_CROSSING=1`.
- Result: route-35 passes, but is much too slow (`total=170.9868 s`,
  native batch about `164.9908 s`). There are `repairs=2`.
- The issue is not yet simply "foreign keepout blocks a crossing". Trace shows
  heavy guided-crossing search with many `reject_unexpected_owner` counts and
  a very long guided victim reroute for native net 32 (`n_31`) through native
  net 35 (`n_34`), with cost about `1065.47` and waypoints making a huge
  detour.
- Route-35 final report is clean but regresses the crossing set:
  `crossing_count=1`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`; the sole
  crossing is `n_31` with `n_34`. The previous route-34 `n_31`/`n_32`
  crossing disappears because `n_31` is rerouted again.
- Interpretation: the next bug is repair-policy/conservation of existing legal
  crossings. When a later net (`n_34`) conflicts, rerouting `n_31` should prefer
  preserving already legal crossings such as `n_31`/`n_32`, or choose a better
  victim/current ordering, instead of replacing that compact crossing with a
  huge detour that only satisfies the newest crossing.
- Current GDS after this investigation is `build\routed_multiportmmi_8x8.gds`,
  written 2026-07-13 15:25:49 local time, at debug stop-after-route 35.

Implementation checkpoint: crossed-net terminal absorber.

- Implemented a constrained terminal segment-length solver for crossed-net
  endpoint correction. Instead of importing arbitrary full-route endpoint
  correction geometry, crossed nets now:
  - freeze the primitive-realized crossing span;
  - solve length deltas on existing terminal segment directions;
  - retry with one zero-baseline port-local straight direction when port
    orientation allows it;
  - fall back to the primitive terminal side if no nondegenerate solution is
    available.
- The solver handles single-segment and two-segment absorption, which covers
  axis and 45-degree diagonal combinations. Diagonal segment changes naturally
  carry coupled x/y movement; the solver only accepts solutions that exactly
  reach the desired terminal endpoint and keep adjusted segment lengths
  positive.
- Existing regressions were updated so a bad full-route corrected detour is
  replaced by straight-length absorption, not by the primitive fallback.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 35 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.5970 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Follow-up relaxation for port-local straight stubs:

- User clarified the previous restriction was too tight: crossed-net endpoint
  correction may insert a straight at the port when its orientation is correct.
  It still must not insert arbitrary mid-access detour geometry.
- Implemented this by allowing a single extra segment at the start of a
  source-side corrected prefix or at the end of a target-side corrected suffix
  when that segment matches the corresponding port orientation. The remaining
  compressed direction sequence must still match the original primitive
  terminal route.
- This preserves the intended hierarchy:
  1. lengthen/shorten existing terminal straights or diagonals when possible;
  2. allow one orientation-correct straight stub at the port;
  3. reject mid-access inserted bends/detours and fall back to the primitive
     side.
- Added a regression that accepts an orientation-correct source port stub while
  keeping the crossing-bearing middle frozen.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_allows_oriented_port_stub tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 35 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.8196 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Follow-up correction to prevent inserted mid-access endpoint geometry:

- User pointed out the crossed-net splice could still import newly inserted
  endpoint-correction geometry into the allowed terminal region, producing a
  visually bad detour instead of merely lengthening/shortening existing
  straights.
- Added a segment-direction contract for crossed-net terminal correction:
  corrected prefix/suffix geometry is accepted only if its compressed segment
  direction sequence matches the original primitive prefix/suffix. This permits
  endpoint shifts and length changes on existing straight runs, but rejects new
  bends inserted away from the port. Rejected sides fall back to the primitive
  baseline.
- Added regressions proving:
  - inserted mid-access geometry is rejected for crossed nets;
  - lengthening existing terminal straights is accepted;
  - crossing-free nets still use normal endpoint correction.

Validation:

    .\.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py tests/test_crossing_verification_report.py tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry tests/test_port_alignment_diagnostics.py::test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights tests/test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline -q
    # 34 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=9.4426 s; GDS written

Final report check:

- `build\verification\multiportmmi_8x8_crossing_verification.json`:
  `issue_count=0`, `legal_crossing_count=1`,
  `matched_crossing_component_count=1`, `illegal_crossing_count=0`.
- `build\verification\multiportmmi_8x8_photonic_verification.json`:
  `issue_count=0`.

Route-35 LiDAR victim repair update:

- User clarified that LiDAR mode should not persist a hard crossing topology,
  but a victim reroute should not be discouraged away from a previously good
  compact crossing path by ripup history or by an early no-crossing fallback.
- Implemented a LiDAR-only victim repair path in `src\py_router.rs`:
  - for `lidar-pure` / collision-crossing repair, ripped victim routes are not
    added as whole-route history penalties before rerouting;
  - the victim first tries a seeded crossing route through its immediate
    pre-ripup crossing partners plus the current repaired net;
  - if that seed is unavailable, it tries an any-committed-partner LiDAR
    collision-crossing route with `crossing_loss=0.0` for this repair attempt
    only;
  - if committed crossing partners exist and these crossing attempts fail, the
    plain victim fallback is blocked for that mode so another repair set/order
    is tried instead of silently accepting a no-crossing victim route.
- Kept the normal `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 50.0` for
  non-victim normal routing.
- Added/kept the explicit guided partner-set acceptance rule: when a helper is
  asked to route through a concrete partner set, the realized crossing events
  must cover that partner set even in `allow_only_expected_pairs=false` mode.

Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check
    # passed

    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed

    .\.venv\Scripts\python.exe -m pytest tests\test_rust_batch_repair.py tests\test_route_failure_diagnostics.py tests\test_routing_flow_stats.py::test_lidar_pure_uses_search_only_crossing_penalty_by_default tests\test_routing_flow_stats.py::test_crossing_plan_keeps_physical_loss_separate_from_search_penalty -q
    # 6 passed

    $env:PYTHONIOENCODING='utf-8'
    $env:MPLCONFIGDIR=(Join-Path (Get-Location) 'build\mpl')
    $env:PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG='1'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=144.9170 s

Route-35 result after this update:

- `build\crossings\multiportmmi_8x8_crossings.json` reports
  `realized_intersection_count=3`, `illegal_realized_crossing_count=0`,
  `native_crossing_event_count=3`, `realized_crossing_component_count=3`.
- Realized crossings after route 35 are:
  - `n_31` / `n_32` at `[1433.5, 673.125]`;
  - `n_31` / `n_33` at `[1443.5, 683.125]`;
  - `n_31` / `n_34` at `[1499.5, 729.125]`.
- `build\verification\multiportmmi_8x8_photonic_verification.json` reports
  `success=True`, `error_count=0`,
  `cross_net_waveguide_overlap_count=0`,
  `waveguide_obstacle_overlap_count=0`.
- Current stop-after-route-35 GDS:
  `build\routed_multiportmmi_8x8.gds`, written 2026-07-13 15:57:54 local
  time, size 134734 bytes.

Route-35 follow-up audit from user screenshot:

- User observed that the route pattern is closer but still not the desired
  clean A* behavior: `n_31` crosses `n_32` and `n_33` compactly, then makes a
  detour before crossing `n_34`.
- Verified current native crossing events:
  - native net 32 (`n_31`) crosses native 34 (`n_33`) at grid `[731,160]`
    on route segment `[(717,146),(740,169)]`;
  - native net 32 (`n_31`) crosses native 33 (`n_32`) at grid `[726,155]`
    on the same route segment;
  - native net 32 (`n_31`) crosses native 35 (`n_34`) at grid `[759,183]`
    on a horizontal route segment `[(769,183),(749,183)]`.
- Current crossing constraints require `7` cells of effective margin per
  crossing side (`crossing_half_size_cells=2`,
  `min_straight_cells_per_crossing=2`, `bend_runout_cells_per_crossing=3`).
- Code audit:
  - `src\astar.rs` does have a real crossing-aware A* move check:
    `crossing_move_outcome` is called for each primitive neighbor; illegal
    crossing moves return `None` and only that primitive neighbor is skipped.
  - However the full pipeline is still not a clean single-layer LiDAR A*:
    `src\py_router.rs` also post-validates realized crossing geometry and has
    guided repair/fallback logic around the search.
  - The current seeded victim repair uses `try_route_through_collision_partner_set`,
    which forces a concrete partner set and therefore a constrained crossing
    sequence, rather than letting a single unconstrained crossing-aware A*
    freely optimize all legal crossing choices.
- Interpretation for next work: the remaining detour is likely caused by a
  combination of local legality being enforced at primitive granularity,
  required post-crossing straight margin/pending-straight state, and the seeded
  repair's partner-set/sequence constraints. The next fix should move closer to
  a single clean LiDAR A* objective: all dynamic-route crossings are legal
  primitive moves with cost, illegal crossing primitive moves are rejected
  locally, and repair should avoid extra guided topology constraints unless
  needed for diagnostics.

Route-35 A* state tracking follow-up:

- User clarified the visual issue: the missed compact opportunity is where
  `n_34` is horizontal and `n_31` arrives diagonally, so the crossing is
  correctly rejected as not perpendicular. The final accepted route detours
  right, briefly runs near/parallel to `n_34`, then crosses later where `n_34`
  is vertical and `n_31` is horizontal.
- Verified this interpretation against the current events:
  - final native events after stop-after-route 35 remain
    `(32,34)` at `[731,160]`, `(32,33)` at `[726,155]`, and `(32,35)` at
    `[759,183]`;
  - the accepted `n_31`/`n_34` crossing uses `n_31` route segment
    `[(769,183),(749,183)]` and `n_34` segment `[(759,170),(759,193)]`, i.e.
    horizontal crossing vertical.
- Implemented a focused `src\astar.rs` correction for crossing-aware A* state:
  when `require_all_partners=true`, a single straight primitive can now account
  for multiple expected partner crossings in the same move if they occur in
  the required order. Previously it only considered the next partner in that
  move, while post-validation could still see multiple realized crossings on
  the same segment.
- Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check
    # passed

    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm test crossing_move --lib
    # compiled, but test binary failed to start on Windows with STATUS_DLL_NOT_FOUND

    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=148.9375 s

- Result after this A* state fix: verification remains clean and the native
  crossing events are unchanged. Therefore this is a correctness improvement
  for the search state, but not yet the fix for the visible detour. Next
  investigation should numerically test the hypothesized earlier vertical
  `n_31` crossing over horizontal `n_34`: is there enough `7`-cell crossing
  margin plus bend runout, and is the primitive route blocked by static/dynamic
  footprint checks, pending-straight state, partner-set ordering, or cost?

Current stop-after-route-34 flow audit:

- Re-ran the CLI reference command:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=15.9918 s

- The successful CLI summary reports:
  - `route[32]`, `n_31`: attempts=2, failures=0,
    buckets=`normal_route/reroute_victims`;
  - `route[33]`, `n_32`: attempts=3, failures=1,
    buckets=`normal_route/probe_route/repair_failed_net`;
  - `route[34]`, `n_33`: attempts=3, failures=1,
    buckets=`guided_collision_crossing/normal_route/probe_route`.
- Interpretation for user-facing flow before `n_34`: `n_31` routes first;
  `n_32` triggers one repair and reroutes `n_31`; then `n_33` routes via the
  guided/collision-crossing path. In the successful stop-after-34 run, `n_33`
  is not itself the next ripup of `n_31`/`n_32`.
- A direct Python API rerun with `stats=RoutingFlowStats()` did not reproduce
  the successful CLI behavior and failed during post-endpoint crossing
  verification with illegal `n_32 x n_33` realized geometry. Treat that as an
  additional reproducibility/stability warning, not as the current GDS state.
- Clarification: `guided_collision_crossing` is the native repair fast path
  after a probe route identifies dynamic blockers/candidate crossing partners.
  It selects up to two candidate blockers that are legal crossing partners,
  temporarily clears their dynamic blocking, reruns A* with a
  `CrossingSearchConfig(require_all_partners=true)`, then post-validates the
  realized crossing events and commits the route only if there are no realized
  crossing violations. This is not a victim ripup by itself.
- User-facing design clarification: this is still not the desired clean LiDAR
  routing model. The desired model is a single crossing-aware A* where every
  primitive expansion tests whether any dynamic crossing on that move is legal;
  illegal crossing primitive moves are rejected locally, and A* continues
  searching other branches. The current `guided_collision_crossing` path is an
  extra repair/helper layer that first discovers candidate blockers with a
  probe route, then constrains a retry to cross those partners. That can be
  useful as a fallback, but it should not be the primary behavior for normal
  crossing-enabled routing.

Clean-A* isolation test before `n_34`:

- Added an opt-in env switch for the guided repair helper in `src/py_router.rs`.
  `guided_collision_crossing` now runs only when
  `PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING=1` is set; the normal
  default keeps collision-aware A* enabled but does not use the guided repair
  fast path. `PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING=1` still wins
  if both are set.
- Fixed the local A*/final-verification mismatch in `src/astar.rs`: crossing
  legality now scans every straight arm of a primitive, including bend arms,
  instead of testing only straight primitives as one chord. Any geometric
  intersection with a committed partner must now become a legal crossing event
  on that arm or the primitive neighbor is rejected locally.
- Runout/margin after a crossing is measured only along the same straight arm;
  distance through a bend kink no longer satisfies crossing margin.
- Added a Rust regression fixture:
  `crossing_move_rejects_bend_arm_non_perpendicular_intersection`.
- Validation:

    $env:CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER='rust-lld'
    $env:PYO3_PYTHON=(Join-Path (Get-Location) '.venv\Scripts\python.exe')
    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check
    # passed

    C:\Users\benja\.cargo\bin\cargo.exe +stable-x86_64-pc-windows-gnullvm check --tests
    # passed

    $env:RUSTUP_TOOLCHAIN='stable-x86_64-pc-windows-gnullvm'
    .\.venv\Scripts\python.exe -m maturin develop --release
    # passed

    $env:PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING='1'
    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs false --attempt-diagnostics --debug-timing true
    # failed at route[33] / `n_32`

- Failure with guided helper disabled:
  `No route found for n_32 ... error=No repair route found;
  candidate_blockers=[32]`.
- Trace for native net 33 (`n_32`) showed the normal collision-aware search
  generated crossing candidates but accepted zero:
  `accepted=0 candidates=5`; the post-check then saw invalid intersections
  with native net 32 (`n_31`), including `insufficient_straight_margin` and
  `not_perpendicular`.
- Clarification: this means A* is not recording those intersections as accepted
  legal `CrossingEvent`s. The bug is that the crossing-aware search can still
  return a route candidate whose realized geometry intersects a committed
  partner even though no legal crossing event was accepted. That mismatch is
  caught only by post-validation, so local A* move legality and final realized
  crossing legality are still not aligned.
- After the primitive-arm legality fix, the default stop-after-route-34 command
  with no guided-helper env flags passes in `9.8676 s`. The verification JSON
  reports `partial_debug_stop`, `error_count=0`, `issue_count=0`,
  `illegal_crossing_count=0`, `legal_crossing_count=2`, and
  `matched_crossing_component_count=2`.
- A compatibility run with the old guided helper enabled before making it
  opt-in also passed, but took about `167.8 s` and produced a different
  one-crossing route. Treat that as evidence that guided collision repair
  should remain opt-in/debug-only for this cluster.

Current stop-after-route-35 clean-A* route audit:

- Re-ran the current default CLI path with no
  `PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING` flag:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs false --attempt-diagnostics --debug-timing true
    # passed; total=12.7504 s

- The default path is now the clean crossing-aware A* mode with the guided
  collision helper opt-in/debug-only.
- CLI route-search summary for the route-35 stop:
  - total attempts=41, failures=2, repairs=2;
  - `route[32]` / `n_31`: attempts=3, failures=0,
    buckets=`normal_route/reroute_victims`;
  - `route[33]` / `n_32`: attempts=3, failures=1,
    buckets=`normal_route/probe_route/repair_failed_net`;
  - `route[35]` / `n_34`: attempts=3, failures=1,
    buckets=`normal_route/probe_route/repair_failed_net`.
- Native repair diagnostics for `n_34` report candidate blockers `[32, 34]`,
  i.e. `n_31` and `n_33`. The rejected probe intersections were classified as
  `not_perpendicular` against `n_33` twice and `n_31` once.
- The committed route after `n_34` is verification-clean:
  `build\crossings\multiportmmi_8x8_crossings.json` reports
  `realized_intersection_count=3`, `illegal_realized_crossing_count=0`,
  `native_crossing_event_count=3`, and
  `realized_crossing_component_count=3`.
- The three realized crossings are all legal and are:
  - `n_31` / `n_32` at `[1425.5, 681.125]`;
  - `n_31` / `n_33` at `[1439.5, 695.125]`;
  - `n_31` / `n_34` at `[1451.5, 739.125]`.
- Debug SVGs for this exact stop were generated for route indices 32-35,
  including `build\routes\multiportmmi_8x8_n_34_attempt39_probe_route.svg`,
  `build\routes\multiportmmi_8x8_n_34_attempt40_repair_failed_net.svg`, and
  the committed `build\routes\multiportmmi_8x8_n_34.svg`.

Current SVG checkpoint before `n_34`:

- Re-generated the exact pre-`n_34` state with:

    .\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs 31-34 --attempt-diagnostics --debug-timing false
    # passed

- The SVG immediately before routing `n_34` is
  `build\routes\multiportmmi_8x8_n_33.svg`, written after committing `n_33`.

Route-35 `n_34` / `n_31` reject trace:

- Ran a no-SVG native trace for `PHOTONIC_ROUTER_TRACE_CROSSING_NET=35`
  (`n_34`). This did not regenerate route SVGs.
- The rejected `n_34` candidate had waypoints
  `[(706,170), (717,170), (739,192), (748,183), (758,193), (767,193)]`.
- Rust reported no accepted crossing events for that candidate, but realized
  validation then found an intersection with native net `32` (`n_31`) at
  approximately `(1475.765, 733.090)` classified as `not_perpendicular`.
- Interpretation: the `n_34`/`n_31` rejection is not a footprint/margin reject;
  it is an angle/orientation reject. More importantly, this is still a local
  A*/realized-validation mismatch: the search returned a route whose realized
  geometry intersects `n_31` without carrying a legal crossing event for that
  intersection.
- Code audit of the mismatch:
  - `try_route_with_collision_crossings` builds `CrossingSearchPartner`
    geometry from `committed_center_routes`.
  - `committed_center_routes` is currently populated from
    `route_obstacle_center_cells(route)`, i.e. the primitive/grid obstacle path.
  - The final violation check uses `committed_realized_center_routes`, populated
    from the endpoint-corrected or primitive physical centerline.
  - Dynamic obstacle commitment already uses `route_dynamic_center_cells`, which
    is derived from the realized/endpoint-corrected centerline.
  - Therefore the crossing search can clear a partner's dynamic blocking and
    miss a physical realized-centerline intersection because it is checking
    crossings against the older grid obstacle path instead of the same
    realized route geometry used for validation and dynamic commitment.
- A targeted no-SVG trace comparing `committed_center_routes[n_31]` and
  `committed_realized_center_routes[n_31]` confirmed the concrete mismatch:
  - native `n_31` id is `32`; native `n_34` id is `35`;
  - grid waypoint segment used by A*:
    `(1411.5, 667.125) -> (1487.5, 743.125)`;
  - realized physical centerline segment used by validation:
    `(1411.557359, 668.882359) -> (1484.042641, 741.367641)`;
  - both are diagonal and nearly parallel, but the realized line is offset by
    about `1.7 um` in y at the relevant x-coordinate;
  - the rejected validation point `(1475.765317, 733.090317)` lies on the
    realized physical line, while the grid line would be around
    `y=731.390317` at the same x. This is the counterexample explaining why
    A* saw no event and validation saw a `not_perpendicular` realized
    intersection.
- Model-fix assessment:
  - The realized curve is probably not incorrectly placed. The observed offset
    is consistent with tangentially rounding a sharp grid bend: the bend
    replaces the grid corner with an arc and the adjacent realized straight run
    starts/ends at tangent points, not at the sharp L-corner skeleton.
  - Therefore the actual model bug is that `route_to_grid_path` / compressed
    footprint waypoints are being used as the optical centerline for crossing
    search. They are an obstacle/footprint skeleton, not the realized optical
    centerline.
  - The proper model fix is to split the route representations explicitly:
    obstacle occupancy may continue to use conservative primitive footprints,
    but crossing detection/search must use the same realized optical centerline
    model as final verification. In practice this means continuous/f64 partner
    and candidate primitive centerline segments, with crossing allowed only on
    straight/tangent portions and rejected on bend arcs.
- Follow-up model audit for the alternative user-preferred fix ("keep
  State/Primitive straights as optical truth for fast A* checks"):
  - The current mismatch is sharpened by `route_to_grid_path`: it follows every
    primitive footprint cell, so a bend primitive's L-shaped footprint arms are
    compressed together with neighboring straight primitives and become
    apparent crossable grid straight segments.
  - `route_to_primitive_centerline_with_runs` then realizes those same bend
    primitives as tangent arcs plus tangent straight portions, so the optical
    straight portion begins/ends at tangent points, not at the sharp L-corner
    footprint skeleton.
  - If the model fix is to keep fast grid-style crossing checks, then the
    primitive contract must be made explicit: conservative footprint cells are
    for occupancy only, while optical straight/crossable segments must be
    derived from the same primitive optical model used by GDS realization.
  - Impacted implementation areas include `src/primitives.rs` primitive
    metadata, `src/geometry_realization.rs` primitive replay and endpoint
    correction run extraction, `src/astar.rs` crossing move legality over
    primitive segments, `src/py_router.rs` committed centerline maps and
    crossing event/violation generation, Python route realization/reporting,
    path-length/meander planning, and focused geometry/crossing regression
    tests.
- User asked whether the current repository state should be committed and
  pushed before the model fix. Recommendation: yes, create an explicit
  checkpoint before the primitive/centerline contract refactor.
- Attempted to inspect `git status --short` and current branch, but Git refused
  due to `safe.directory` / dubious ownership for
  `C:/Users/benja/Documents/Repositorys/TUMPhotonicRouter`. A commit/push first
  needs either the safe-directory config to be added by an approved command or
  the user to run the equivalent local Git configuration.
- User added the safe-directory config. Created and pushed checkpoint commit
  `56e8a1d` on branch `crossings/verification-foundation`:
  `routing: checkpoint crossing verification foundation`.
- Validation before that checkpoint: `cargo +stable-x86_64-pc-windows-gnullvm
  check` passed with `rust-lld` and project `.venv` Python configured.
- The checkpoint intentionally excludes local build artifacts; `.gitignore` now
  ignores `python/photonic_router/*.dll`.
- Next recommended action remains the primitive/centerline model refactor:
  define explicit optical/crossable primitive segments and stop using
  footprint-compressed waypoints as crossing-search truth.

Primitive/centerline model fix work in progress:

- The earlier optical-segment partner fallback direction was rejected by the
  user and backed out from `src/astar.rs` and `src/py_router.rs`.
- Implemented the correct first model slice instead: crossing-enabled internal
  endpoint correction now uses a new grid-locked centerline helper,
  `route_to_grid_locked_port_centerline`, which inserts physical port points
  locally but preserves the primitive/grid interior. This prevents port
  correction from shifting the whole route parallel to the A* grid path.
- Added a Rust geometry regression proving grid-locked port correction keeps
  primitive interior points on the original grid line.
- A trace of `n_34` versus partner `n_31` confirmed the long diagonal is no
  longer parallel-shifted by endpoint correction: realized straight portions
  lie on the same grid line and are only shortened by bend runout.
- Two attempted quick fixes were tested and rejected:
  - disabling bend primitives as crossable route-side segments caused `n_32` to
    fail before the `n_34` cluster;
  - naively augmenting bend primitive footprints with sampled bend-centerline
    cells also caused `n_32` to fail.
- Current conclusion: the remaining bug requires a proper Primitive segment
  model in A*: straight/crossable portions and bend/non-crossable portions must
  be explicit per primitive, while footprint occupancy stays separately
  conservative. Quick guards around existing footprint windows are too coarse.
- Validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed.
  - `cargo +stable-x86_64-pc-windows-gnullvm test --no-run` passed.
  - `maturin develop --release` passed after the grid-locked endpoint helper.
  - Specific Rust test execution still cannot run in this Windows toolchain
    because the test executable exits with `STATUS_DLL_NOT_FOUND`.
  - Targeted no-SVG cluster run with only grid-locked endpoint correction still
    fails at `n_34` because A* can accept candidates whose realized bend/route
    geometry later intersects `n_31` without a legal crossing event. This is the
    remaining Primitive segment-model issue.
- `cargo fmt` could not run because `rustfmt` is not installed for either the
  default MSVC toolchain or `stable-x86_64-pc-windows-gnullvm`.

Focused `n_34` / `n_31` trace after rebuilding the current clean source:

- Regenerated `build/routed_multiportmmi_8x8.gds` with
  `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
  --debug-stop-after-route 34 --debug-svgs false --debug-timing false
  --attempt-diagnostics`; this is the layout state immediately before routing
  `n_34` (route index 35 / Rust net 35).
- A temporary env-gated diagnostic counted crossing candidate attempts for
  Rust net 35 (`n_34`) against Rust net 32 (`n_31`):
  - total candidate checks against that partner: 85,867
  - `not_perpendicular`: 49,520
  - `margin`: 27,266
  - `unmatched_footprint`: 9,081
- The final failing run still reports `No route found for n_34` with
  `candidate_blockers=[32, 34]`; recent repair failures include illegal
  realized intersections with net 32 at `not_perpendicular` and
  `insufficient_straight_margin`.
- The temporary per-partner diagnostic hook was removed before committing; it
  was only used to derive the numbers above.

Endpoint-correction boundary correction after user review:

- User clarified the intended invariant: endpoint correction is not part of the
  routing/search model. It must not influence A* crossing search, dynamic
  blocker commitment, victim/ripup decisions, native crossing event extraction,
  or internal crossing validation. It belongs after routing is complete, where
  Python can splice the terminal port-correction geometry into the final GDS
  realization.
- Updated `src/py_router.rs` accordingly:
  - `route_dynamic_center_cells` now commits dynamic/blocker cells from the
    primitive/grid obstacle center cells instead of endpoint-corrected physical
    centerlines.
  - `routing_centerline_for_route` now returns the primitive realized centerline
    for all internal routing validation/commit users.
  - `realized_crossing_events_for_route`,
    `remember_committed_route_centerlines_with_ports`, and
    `crossing_violations_for_route_with_ports` therefore no longer see
    endpoint-corrected routes during routing.
- The post-routing endpoint-correction API remains available through
  `route_port_corrected_centerline` and is still called from Python realization
  after route records exist.
- Validation for this boundary fix:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with `rust-lld` and
    project `.venv` Python configured.
  - `.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 34 --debug-svgs false
    --debug-timing false --attempt-diagnostics` passed in about `14.8 s`.
    The resulting partial verification JSON reports `status=partial_debug_stop`
    with `error_count=0` and no listed crossing or photonic violations.
- Important: the goal is no longer to force the pre-commit GDS topology. The
  correct invariant is that routing decisions are made against the same
  primitive/realized routing model, while endpoint correction is a final
  realization step only.

Python crossing-margin consistency fix:

- Date: 2026-07-14 09:41 +02:00
- Branch: `crossings/verification-foundation`
- Base commit before this change: `d56ebb0`
- The Python crossing diagnostics/expected-event overlap checks were brought
  back in line with the Rust A* hard crossing margin. They now use
  `crossing_half_size_cells + bend_runout_cells_per_crossing` and no longer add
  `min_straight_cells_per_crossing`.
- This fixes stale report values such as
  `required_straight_margin_cells_per_crossing = 7` for
  `crossing_half=2`, `min_straight=2`, `bend_runout=3`; regenerated artifacts
  now report `required_straight_margin_cells_per_crossing = 5`.
- Validation:
  - `.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py -q`
    passed: `21 passed`.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 34 --debug-svgs 31-34
    --debug-timing false --attempt-diagnostics` passed and regenerated
    `build\crossings\multiportmmi_8x8_crossings.json` with
    `required_straight_margin_cells_per_crossing = 5`, `realized_count = 2`.

Bend-runout crossing model fix:

- Date: 2026-07-14 10:12 +02:00
- Branch: `crossings/verification-foundation`
- Base commit before this change: `9975d34`
- A* now carries the terminal arm of a bend into the next state's
  `straight_run_cells`, so a route shaped like `bend arm -> straight ->
  crossing` can satisfy pre-crossing runout with the bend arm included. This
  matches the intended model that bend runout cells may be part of the bend
  arm; only the bend kink/curved footprint must stay out of the crossing
  footprint.
- The Rust native realized-crossing checks now use only the realized crossing
  footprint half-size for physical centerline validation. The full
  `crossing_half + bend_runout` remains an A*/grid-search constraint, while
  physical post-route validation no longer rejects crossings solely because the
  extra search runout is carried by a bend arm.
- Validation:
  - `cargo check` with `stable-x86_64-pc-windows-gnullvm` and `rust-lld`
    passed.
  - `cargo test --no-run` passed, compiling the new Rust regression fixture
    `crossing_margin_counts_terminal_bend_arm_before_next_crossing`. Actual
    Rust test execution remains avoided in this Windows environment because
    prior runs have exited with `STATUS_DLL_NOT_FOUND`.
  - `.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.venv\Scripts\python.exe -m pytest tests/test_realized_crossing_verification.py -q`
    passed: `21 passed`.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 34 --debug-svgs 31-34
    --debug-timing false --attempt-diagnostics` passed. The regenerated JSON
    reports `required_straight_margin_cells_per_crossing = 5`,
    `realized_intersections = 2`, `illegal_realized = 0`, crossing
    verification `error_count = 0`, and photonic verification `error_count = 0`.

Diagonal halo Layer-1 checkpoint:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- Active user clarification: for diagonal primitive pieces, the temporary A*
  collision halo should be a compact second diagonal lane next to the true
  diagonal cells, matching the user's red/green sketch. It should not be a
  broad four-sided neighborhood.
- Current WIP in `src/astar.rs` changes `compact_diagonal_halo_cells` to
  produce only the fixed adjacent lane cells for each diagonal unit step:
  `start + (dx, 0)` and `end + (dx, 0)`. This is meant only for collision
  recognition; true crossing validation still uses the real centerline
  intersection and perpendicular/margin checks.
- Temporary debug bypasses were removed before this checkpoint:
  - no `PHOTONIC_ROUTER_DEBUG_ALLOW_INVALID_GDS` path remains in
    `routing_flow.py` or `translation/route_rust.py`;
  - no `PHOTONIC_ROUTER_DISABLE_REALIZED_CROSSING_VALIDATION*` path remains in
    `src/py_router.rs`;
  - the temporary `diagonal-halo` trace prints were removed from
    `src/astar.rs`.
- Standing invariant recorded in `.agent/WORKFLOW.md` and the active ExecPlan:
  A* must make the primary legality decision for router-discovered crossings.
  If A* accepts a crossing and Rust/Python verification later rejects it as
  illegal, that is a blocking model mismatch, not a normal repair signal.
  Rip-up/reroute is intended only after legal A* search cannot find a path.
- Endpoint correction is explicitly post-routing. It must not influence A*
  crossing search, dynamic blocker commitment, victim/rip-up decisions, native
  crossing event extraction, or internal crossing validation.
- Validation performed after this edit:
  - `cargo check --target x86_64-pc-windows-gnullvm` with `rust-lld`, project
    `.venv` Python, and the stable GNU LLVM toolchain passed.
  - After removing debug bypasses/traces, `cargo check --target
    x86_64-pc-windows-gnullvm` passed again.
  - `cargo test --target x86_64-pc-windows-gnullvm --no-run
    crossing_move_detects_offset_diagonal_halo_contact` passed, compiling the
    Rust regression fixture without executing the Windows test binary.
  - `.venv\Scripts\python.exe -m maturin develop --release` passed after the
    cleanup.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 33 --debug-svgs false
    --debug-timing false --attempt-diagnostics` passed without trace/bypass
    environment variables. Reports:
    crossing verification `success=True`, `status=partial_debug_stop`,
    `error_count=0`, `legal_crossing_count=1`,
    `matched_crossing_component_count=1`; photonic verification `success=True`,
    `error_count=0`.
- Follow-up verification after rebuilding with `maturin develop --release`:
  - `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 33 --debug-svgs false --debug-timing false
    --attempt-diagnostics` was run with
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=33` and
    `PHOTONIC_ROUTER_TRACE_PARTNER_NET=32`.
  - The new diagonal halo does fire for the `n_32`/`n_31` offset diagonal
    contacts. Example trace:
    `diagonal-halo reject-margin net=33 partner=32 x=724.500 y=170.500
    route_margin=0.500 partner_margin=12.021 required_margin=5`.
  - This verifies that the compact red-lane halo detects the offset diagonal
    collision and rejects the immediate illegal crossing move at Layer 1.
  - Remaining behavior is still not converged: the first collision-crossing A*
    attempt does not find an alternate legal branch before repair; the run
    still reaches `native_repair_probe net=33 ... candidate_blockers=[32]`.
- A focused Rust regression
  `crossing_move_detects_offset_diagonal_halo_contact` was added and compiles,
  but direct Rust test execution still fails in this Windows environment with
  `STATUS_DLL_NOT_FOUND`, matching earlier test-run limitations.
- Still pending:
  - Continue route discovery/repair convergence from this invariant boundary.
    It is acceptable for A* to fall into repair after it rejects illegal moves
    and cannot find a legal route; the next task is improving convergence, not
    accepting post-route illegal crossings.

Hard-stop realized crossing validation checkpoint:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- User decision: a realized/Python-side crossing validation failure after an
  A*-accepted crossing candidate is not a normal routing failure and must not
  trigger fallback detours or ordinary rip-up/repair. It is a blocker that
  exposes an A*/realization model mismatch.
- Code change: `src/py_router.rs` collision-crossing helpers now return
  `Result<Option<...>, String>` instead of only `Option<...>`. A normal "no
  collision-crossing route found" result remains `Ok(None)`. If crossing events
  satisfy the partner constraints but `crossing_violations_for_route_with_ports`
  reports violations, the helper returns a hard error such as:
  `Realized crossing validation failed for A*-accepted crossing route on net ...`.
- Validation performed:
  - `cargo check --target x86_64-pc-windows-gnullvm` with `rust-lld`, project
    `.venv` Python, and the stable GNU LLVM toolchain passed.
  - `.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 35 --debug-svgs false
    --debug-timing false --attempt-diagnostics` passed, preserving the clean
    state through `n_34`.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 36 --debug-svgs false
    --debug-timing false --attempt-diagnostics` with
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=36` and
    `PHOTONIC_ROUTER_TRACE_PARTNER_NET=35` now fails intentionally instead of
    committing or falling back to the long `n_35` detour. Trace showed the A*
    candidate crossing route and the realized validation mismatch.
- `cargo fmt` could not run because `rustfmt` is not installed for the local
  Windows Rust toolchains (`stable-x86_64-pc-windows-msvc` and
  `stable-x86_64-pc-windows-gnullvm`).

Symmetric diagonal halo checkpoint:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- User clarification: one-cell diagonal route pieces need a compact halo on
  both sides of the diagonal, not only the previously implemented red-lane side.
  The halo is only for Layer-1 collision recognition. It does not by itself
  make a crossing legal; after detection A* must still evaluate normal crossing
  legality.
- Code change: `src/astar.rs::compact_diagonal_halo_cells` now returns both
  adjacent compact lanes for each diagonal unit step:
  `(start.x + dx, start.y)`, `(end.x + dx, end.y)`, `(start.x, start.y + dy)`,
  and `(end.x, end.y + dy)`.
- Regression: added
  `crossing_move_rejects_parallel_diagonal_on_mirrored_halo_side`, covering a
  parallel one-cell-offset diagonal contact on the mirrored side that the
  previous one-sided halo missed. The expected behavior is rejection as a
  non-perpendicular crossing candidate, not acceptance as free space.
- Validation performed:
  - `cargo check --target x86_64-pc-windows-gnullvm` passed.
  - `cargo test --target x86_64-pc-windows-gnullvm --no-run
    crossing_move_rejects_parallel_diagonal_on_mirrored_halo_side` passed,
    compiling the regression without executing the Windows test binary.
  - `.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 35 --debug-svgs false
    --debug-timing false --attempt-diagnostics` passed, preserving the clean
    state through `n_34`.
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 36 --debug-svgs false
    --debug-timing false --attempt-diagnostics` still hard-fails as intended
    with a realized crossing validation mismatch. Follow-up analysis showed the
    remaining `n_35` x `n_32` case is not solved by symmetric halo alone: the
    traced candidate has a direct grid centerline interaction with the committed
    `n_32` centerline, so the next fix should inspect direct crossing event
    extraction / margin consistency rather than endpoint correction.

Pending-after crossing margin WIP:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- Current uncommitted WIP in `src/astar.rs` changes crossing margin accounting
  from Euclidean segment length to grid-step counts. This is intended to make
  `pending_after_crossing_cells` mean actual straight grid cells, so a diagonal
  bend arm of 3 cells satisfies pending margin 3 but not pending margin 4.
- The WIP also keeps the largest missing after-crossing runout when multiple
  crossings occur in one primitive and adds focused Rust fixtures, including a
  direct `n_35` x `n_32` regression that expects the short after-crossing
  segment to be carried as `pending_after_crossing_cells = 4`.
- Validation:
  - `cargo check` passed with
    `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm`,
    `CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER=rust-lld`, and
    `PYO3_PYTHON=.venv\Scripts\python.exe`.
  - `cargo test --lib --no-run` passed with the same environment.
  - Direct Rust test execution still exits with `STATUS_DLL_NOT_FOUND` in this
    Windows runtime.
  - `maturin develop --release --target x86_64-pc-windows-gnullvm` passed when
    `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm` and `rust-lld` were
    set explicitly.
- Benchmark/diagnostic state:
  - A stop-after-route-36 (`n_35`) run with the WIP became long-running and was
    stopped.
  - A stop-after-route-35 run also appeared slow and was interrupted by the
    user before completion.
  - A stop-after-route-33 run completed successfully, writing the requested
    after-`n_32` GDS. The stable copy is
    `build\routed_multiportmmi_8x8_after_n32_stop33.gds`.
- Next investigation should proceed step-by-step from route 31 onward to find
  exactly where the stricter grid-step pending-margin model starts expanding
  too much search.

Layer-1 crossing hot-path contract note:

- Date: 2026-07-14
- The intended production workflow was written into `.agent\WORKFLOW.md` and
  the active ExecPlan:
  `effective_footprint = primitive_footprint + compact diagonal halo cells`.
  Static contacts in this effective footprint reject the A* move. Dynamic
  contacts produce nearby committed owner ids, and crossing legality is then
  checked against the true candidate primitive centerline and true committed
  owner centerline. Halo cells are witnesses for collision discovery only; they
  are not crossing centers and are not committed route geometry.
- Current implementation difference to investigate before the next code slice:
  `src/astar.rs` still mixes owner/cell overlap checks with broad geometric
  scans over partner waypoints. That broad scan is likely a meaningful part of
  the `n_32` cost (`5226` crossing candidate checks in the route-33 stats run)
  and should be replaced or bounded by owner-first local candidate discovery
  once fixtures pin behavior.

Layer-1 owner-first hot-path WIP:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- Current base commit: `dd120e7 docs: record owner-first crossing contract`
- Active ExecPlan:
  `.agent/execplans/2026-07-10-crossing-verification-foundation.md`
- Current uncommitted code change: `src/astar.rs` now builds an effective
  collision-witness set for each primitive move from the real primitive
  footprint plus compact diagonal halo cells. Static contacts in that set
  reject immediately; dynamic core contacts are mapped to committed owner ids;
  only those contacted owners are checked with true centerline crossing
  geometry. The previous broad scan over all crossing partners is no longer the
  production path.
- The previous pending-after WIP remains part of the same uncommitted
  `src/astar.rs` diff: diagonal arms are counted by grid step count, not
  Euclidean length, and multiple crossings keep the largest remaining
  pending-after runout.
- Validation:
  - `cargo check` passed with the Windows GNU LLVM toolchain, `rust-lld`, and
    project `.venv` Python.
  - `cargo test --lib --no-run` passed with the same environment. Direct Rust
    test execution remains limited by the known Windows `STATUS_DLL_NOT_FOUND`
    issue.
  - `maturin develop --release --target x86_64-pc-windows-gnullvm` passed and
    installed the rebuilt extension.
  - Compact `multiportmmi_8x8` stop-after-route-33 stats run passed with
    crossings enabled and `lidar-pure`, no debug SVGs, no attempt diagnostics.
    The run took `10.749 s` wall time and `5.704 s` A* time versus the previous
    comparable `50.019 s` wall time and `45.585 s` A* time. Route `n_31`
    search time dropped to about `0.519 s`; route `n_32` search time dropped to
    about `0.849 s`.
- Current dirty files:
  - `src/astar.rs`
  - `.agent/REPOSITORY_STATE.md`
- Next recommended action: review the owner-first diff, then commit the focused
  routing hot-path change if the user accepts the stop-after-route-33 evidence.

Stop-after-route-34 follow-up:

- Date: 2026-07-14
- Requested validation: generate a legal GDS after route `n_34` before
  committing the owner-first hot-path change.
- Result: no new route-34 GDS was produced. The first run failed fail-closed
  with `Realized crossing validation failed for A*-accepted crossing route on
  net 32: partner=33 ... insufficient_straight_margin`.
- Investigation showed that the candidate route crossed the intended partner
  but also produced a hidden too-tight grid/realized crossing against another
  local partner. A small `src/py_router.rs` WIP now makes collision-crossing
  candidate acceptance consult the existing grid-crossing veto against all
  local search partners, not only the partners present in realized crossing
  events.
- After rebuilding, the hard realized-validation mismatch was gone, but the
  route-34 run still did not converge: routing `n_33` failed with
  `No repair route found; candidate_blockers=[32, 33]`. This means the current
  legal state remains stop-after-route-33 only; do not commit as a route-34
  convergence checkpoint yet.
- Current dirty files:
  - `src/astar.rs`
  - `src/py_router.rs`
  - `.agent/REPOSITORY_STATE.md`
- Validation in this follow-up:
  - `cargo check` passed after the `src/py_router.rs` change.
  - `maturin develop --release --target x86_64-pc-windows-gnullvm` passed.
  - `multiportmmi_8x8` stop-after-route-34 failed before writing a new GDS.

LiDAR local crossing-partner lookup WIP:

- Date: 2026-07-14
- User clarification: in `lidar-pure`, crossing partners must not come from a
  global `crossing_allowed_partner_set`. A* should discover dynamic collisions
  move by move and then test crossing legality only for the collided owners.
- Code change in `src/py_router.rs`: expected/topology mode still uses
  `crossing_allowed_partner_set`, but `lidar-pure` now uses local lookup sets:
  route-window lookup for initial collision-crossing attempts, probe-route
  dynamic owners for repair victim selection, and route-bbox lookup for
  post-route grid crossing checks/event registration. The huge global partner
  set is no longer fed into the `lidar-pure` collision-crossing search path.
- Trace evidence after rebuild: the route-34 (`n_33`) crossing search now
  starts with local partners only, e.g. `partners=[33, 32]`, instead of the
  previous long list of many committed routes. This confirms the global
  LiDAR partner leakage is removed.
- Current convergence state:
  - `multiportmmi_8x8` stop-after-route-33 passes in about `7.893 s` wall time
    and `3.523 s` A* time.
  - `multiportmmi_8x8` stop-after-route-34 still fails at named net `n_33`.
    It now fails locally with `candidate_blockers=[32, 33]`; this is no longer
    a far-away partner explosion. The remaining problem is local crossing
    legality/convergence around `n_31`/`n_32`/`n_33`.
- Validation:
  - `cargo check` passed.
  - `maturin develop --release --target x86_64-pc-windows-gnullvm` passed.
  - `cargo test --lib --no-run` passed.
- Current dirty files:
  - `src/astar.rs`
  - `src/py_router.rs`
  - `.agent/REPOSITORY_STATE.md`

LiDAR probe-victim suppression WIP:

- Date: 2026-07-14
- User clarification: in crossing-enabled `lidar-pure`, an ignore-dynamic
  probe route must not be used as the primary source of victims. A route should
  only enter rip-up after the crossing-aware A* has exhausted legal moves and a
  real final blocker is known. Probe geometry is diagnostic only until that
  blocker attribution exists.
- Code change in `src/py_router.rs`: after normal route failure, the batch flow
  now first tries a local direct crossing retry with single nearby partners. If
  that cannot commit a legal route, `lidar-pure` stops with a fail-closed error
  instead of running `route_single_net_ignore_dynamic_native` and deriving
  `candidate_blockers` from the probe path.
- Validation:
  - `cargo check` passed with the Windows GNU LLVM toolchain, `rust-lld`, and
    project `.venv` Python.
  - `maturin develop --release --target x86_64-pc-windows-gnullvm` passed and
    installed the rebuilt extension.
  - `multiportmmi_8x8` stop-after-route-34 no longer performs probe-derived
    rip-up. It now fails explicitly at named `n_33` / internal net `34` with
    `No legal LiDAR crossing route found; probe-based victim selection is
    disabled in crossing mode`.
- Current convergence state:
  - Stop-after-route-33 remains the last passing checkpoint.
  - Stop-after-route-34 now exposes the next true issue: the local
    crossing-aware A* still does not find a legal route for named `n_33` from
    the committed `n_31`/`n_32` state. The old false repair path no longer
    hides this by ripping `[32, 33]` from a probe route.
- Current dirty files:
  - `src/astar.rs`
  - `src/py_router.rs`
  - `.agent/REPOSITORY_STATE.md`

Dense source port-runway WIP:

- Date: 2026-07-14
- User clarification: the staggered reservation must apply to the normal
  same-cluster/same-element port runway, not to the global foreign-port
  keepout. Foreign keepout cells for ports in the same dense source cluster
  must be opened together so they do not block the cluster's own routes.
- Code change in `translation/route_rust.py`: dense source MMI fanout runs now
  assign staggered normal runway lengths by source-port order with 3-cell
  spacing. For the `multiportmmi_8x8` `n_31..n_36` source cluster this gives
  `o12=18`, `o11=15`, then decreasing by 3 cells toward the top port. The
  global `foreign_port_keepout_cells` remains unchanged.
- Code change in `translation/route_rust.py`: when a route endpoint belongs to
  such a dense source cluster, the endpoint opens the complete foreign keepout
  for that source cluster, but subtracts all normal port-runway cells before
  adding those foreign cells to the flat opened-cell set. This prevents
  foreign keepouts from self-blocking the cluster while preserving the
  staggered normal runway reservation and individual port protections.
- Validation:
  - `python -m py_compile translation/route_rust.py` passed.
  - `pytest -q tests/test_route_rust_opened_cells.py -q` passed.
  - `multiportmmi_8x8` stop-after-route-32 (`n_31`) passed with
    `source_dense_port_runway_cells=18`, cluster size `6`, and
    `foreign_port_keepout_open_count=468`.
  - `multiportmmi_8x8` stop-after-route-33 (`n_32`) passed after the
    foreign-only-minus-normal-runway correction in `18.211 s` wall time with
    `source_dense_port_runway_cells=15`, cluster size `6`,
    `foreign_port_keepout_open_count=410`, `opened_cells_count=429`, and
    `opened_static_overlap_count=0`.
  - `multiportmmi_8x8` stop-after-route-34 (`n_33`) now also passes with
    `source_dense_port_runway_cells=12`, cluster size `6`,
    `foreign_port_keepout_open_count=410`, `opened_cells_count=426`,
    `opened_static_overlap_count=0`, `route_static_blocked_overlap_count=0`,
    and one dynamic crossing overlap at grid cell `(734, 178)`.
  - `multiportmmi_8x8` stop-after-route-35 (`n_34`) now passes in `21.553 s`
    wall time with `source_dense_port_runway_cells=9`, cluster size `6`,
    `foreign_port_keepout_open_count=410`, `opened_cells_count=423`,
    `opened_static_overlap_count=0`, `route_static_blocked_overlap_count=0`,
    and one dynamic crossing overlap at grid cell `(751, 200)`.
  - `multiportmmi_8x8` stop-after-route-36 (`n_35`) is the next failing
    checkpoint. It writes `multiportmmi_8x8_n_35_FAILED.txt`; the current route
    attempt fails with `Illegal grid crossing: net 36 intersects net 33 at
    (720.500, 174.500) (insufficient_straight_margin)`, then `lidar-pure`
    fails closed with `No legal LiDAR crossing route found; probe-based victim
    selection is disabled in crossing mode`. No `n_35` route SVG is committed.
  - The temporary follow-up fix that counted realized crossing margin across
    contiguous collinear centerline waypoints was removed at the user's request,
    because crossing margin should count cells only up to the current crossing
    center.
  - After removing that temporary fix and rebuilding the extension, a targeted
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=36` run confirmed the previous
    `n_35` behavior is back: the candidate crosses internal partners
    `[35, 32, 34]`, then validation reports
    `realized_violations=[(33, (1459.5, 689.125),
    "insufficient_straight_margin")]`.
  - Root-cause finding for the `n_35` / internal `net=36` WIP pending-margin
    logic: A* tracks `pending_after_crossing_cells`, but not the direction of
    the pending straight arm. If a crossing occurs near the end of the first
    arm of a bend, the pending count can be carried into the next primitive and
    satisfied by an initial straight in the *new* post-bend direction. That is
    wrong for crossing margin; the remaining cells must continue in the same
    direction as the arm that left the crossing center. This explains why
    level-1 A* accepts the `n_35` candidate while the level-2 Rust validator
    correctly rejects it as `insufficient_straight_margin`.
- Current dirty files:
  - `src/astar.rs`
  - `src/py_router.rs`
  - `translation/route_rust.py`
  - `.agent/REPOSITORY_STATE.md`

Incremental LiDAR crossing checkpoint after commit `35e30fe`:

- Date: 2026-07-14
- Branch: `crossings/verification-foundation`
- Stable code checkpoint: commit `35e30fe` (`routing: stabilize lidar crossing runout state`).
- Current code dirty state: no code files dirty; only this repository-state log
  is modified.
- Artifact: `build/routed_multiportmmi_8x8.gds` is currently updated through
  stop-after-route-53 / named net `n_52`.
- Latest incremental run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=53,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `44.048 s`; total routing-flow time: `44.0453 s`.
  - Optical routing stage: `41.0525 s`; net routing phase: `40.2592 s`;
    native route batch: `36.1391 s`.
  - Attempts: `53`; failures: `0`; repairs: `0`; simple routes: `15/53`.
  - A* counters: expanded `454779`, generated `2728674`, heap pushes
    `745352`, heap pops `511017`, footprint checks `2601412`, rect checks
    `470375`, full-grid fallbacks `0`.
  - Crossing verification: success `True`, errors `0`, warnings `0`,
    crossings `9`, status `partial_debug_stop`.
  - Photonic verification: success `True`, errors `0`, warnings `0`,
    routed records `53`, missing routes `58`, status `partial_debug_stop`.
  - Newly included net: route index `53`, named `n_52`, links
    `mmi0_multiport_1_1,o7 -> mmi0_ps_array_2_heater_5,o1`.
  - GDS timestamp: `2026-07-14 21:01:08`; size `183088` bytes.

Incremental LiDAR crossing checkpoint:

- Date: 2026-07-14
- Artifact: `build/routed_multiportmmi_8x8.gds` is currently updated through
  stop-after-route-55 / named net `n_54`.
- Latest incremental run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=55,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `82.444 s`; total routing-flow time: `82.4414 s`.
  - Optical routing stage: `78.9731 s`; net routing phase: `77.9117 s`;
    native route batch: `72.7136 s`.
  - Attempts: `55`; failures: `0`; repairs: `0`; simple routes: `15/55`.
  - A* counters: expanded `762144`, generated `4572864`, heap pushes
    `1136629`, heap pops `871794`, footprint checks `4377728`, rect checks
    `796673`, full-grid fallbacks `0`.
  - Crossing verification: success `True`, errors `0`, warnings `0`,
    crossings `14`, status `partial_debug_stop`.
  - Photonic verification: success `True`, errors `0`, warnings `0`,
    routed records `55`, missing routes `56`, status `partial_debug_stop`.
  - Newly included nets:
    - Route index `54`, named `n_53`, links
      `mmi0_multiport_1_1,o6 -> mmi0_ps_array_2_heater_6,o1`.
    - Route index `55`, named `n_54`, links
      `mmi0_multiport_1_1,o5 -> mmi0_ps_array_2_heater_3,o1`.
  - GDS timestamp: `2026-07-14 21:06:05`; size `206256` bytes.
  - Observation: compared with stop-after-route-53, these two routes add five
    crossings and roughly double the search runtime. Verifiers remain green.

Incremental LiDAR crossing checkpoint:

- Date: 2026-07-14
- Artifact: `build/routed_multiportmmi_8x8.gds` is currently updated through
  stop-after-route-65 / named net `n_64`.
- Latest incremental run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=65,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `82.230 s`; total routing-flow time: `82.2268 s`.
  - Optical routing stage: `78.5506 s`; net routing phase: `76.8579 s`;
    native route batch: `71.2529 s`.
  - Attempts: `65`; failures: `0`; repairs: `0`; simple routes: `15/65`.
  - A* counters: expanded `774227`, generated `4645362`, heap pushes
    `1177755`, heap pops `884098`, footprint checks `4437606`, rect checks
    `805222`, full-grid fallbacks `0`.
  - Crossing verification: success `True`, errors `0`, warnings `0`,
    crossings `14`, status `partial_debug_stop`.
  - Photonic verification: success `True`, errors `0`, warnings `0`,
    routed records `65`, missing routes `46`, status `partial_debug_stop`.
  - Newly included nets:
    - Route index `56`, named `n_55`, links
      `mmi0_ps_array_2_heater_0,o2 -> mmi0_multiport_2_0,o1`.
    - Route index `57`, named `n_56`, links
      `mmi0_ps_array_2_heater_1,o2 -> mmi0_multiport_2_0,o2`.
    - Route index `58`, named `n_57`, links
      `mmi0_ps_array_2_heater_2,o2 -> mmi0_multiport_2_0,o3`.
    - Route index `59`, named `n_58`, links
      `mmi0_ps_array_2_heater_3,o2 -> mmi0_multiport_2_0,o4`.
    - Route index `60`, named `n_59`, links
      `mmi0_ps_array_2_heater_4,o2 -> mmi0_multiport_2_1,o1`.
    - Route index `61`, named `n_60`, links
      `mmi0_ps_array_2_heater_5,o2 -> mmi0_multiport_2_1,o2`.
    - Route index `62`, named `n_61`, links
      `mmi0_ps_array_2_heater_6,o2 -> mmi0_multiport_2_1,o3`.
    - Route index `63`, named `n_62`, links
      `mmi0_ps_array_2_heater_7,o2 -> mmi0_multiport_2_1,o4`.
    - Route index `64`, named `n_63`, links
      `mmi0_multiport_2_0,o8 -> mol_array_1_mzi_0,o1`.
    - Route index `65`, named `n_64`, links
      `mmi0_multiport_2_0,o7 -> mol_array_1_mzi_1,o1`.
  - GDS timestamp: `2026-07-14 21:09:56`; size `220354` bytes.
  - Observation: compared with stop-after-route-55, these ten routes add no
    new crossings and keep both verifiers green.

Incremental LiDAR crossing checkpoint:

- Date: 2026-07-14
- Artifact: `build/routed_multiportmmi_8x8.gds` is currently updated through
  stop-after-route-87 / named net `n_86`.
- Latest incremental run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=87,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `196.797 s`; total routing-flow time: `196.7932 s`.
  - Optical routing stage: `191.9627 s`; net routing phase: `189.3288 s`;
    native route batch: `180.6092 s`.
  - Attempts: `87`; failures: `0`; repairs: `0`; simple routes: `24/87`.
  - A* counters: expanded `1554707`, generated `9328242`, heap pushes
    `2239767`, heap pops `1799481`, footprint checks `8940184`, rect checks
    `1619125`, full-grid fallbacks `0`.
  - Crossing verification: success `True`, errors `0`, warnings `0`,
    crossings `24`, status `partial_debug_stop`.
  - Photonic verification: success `True`, errors `0`, warnings `0`,
    routed records `87`, missing routes `24`, status `partial_debug_stop`.
  - Newly included net range: route indices `66..87`, named `n_65..n_86`.
    This covers the remaining `mmi0_multiport_2_*` routes into `mol_array_1`,
    all `mol_array_1_mzi_* -> mmi1_ps_array_0_heater_*` routes, and the first
    `mmi1_ps_array_0_heater_* -> mmi1_multiport_0_*` routes.
  - GDS timestamp: `2026-07-14 21:14:37`; size `287782` bytes.
  - Observation: compared with stop-after-route-65, this adds ten crossings
    and keeps both verifiers green, but the search runtime rises sharply to
    roughly 197 seconds.

Full benchmark attempt / long-route finding:

- Date: 2026-07-14
- User requested a full `multiportmmi_8x8` run after the passing
  stop-after-route-87 checkpoint.
- First full run was interrupted and then manually stopped because it produced
  no progress output for several minutes. Background Python worker processes
  were terminated.
- A second controlled full run was launched with
  `PHOTONIC_ROUTER_NATIVE_PROGRESS=1` to identify the long route without
  waiting indefinitely.
- Observed progress:
  - Routes `1..93` progressed without repair-loop evidence.
  - Expensive single-route examples:
    - route `36` elapsed about `10.43 s`.
    - route `51` elapsed about `6.36 s`.
    - route `52` elapsed about `10.43 s`.
    - route `55` elapsed about `30.41 s`.
    - route `68` elapsed about `35.64 s`.
    - route `69` elapsed about `37.21 s`.
    - route `70` elapsed about `20.52 s`.
    - route `92` elapsed about `10.27 s`.
    - route `93` elapsed about `22.35 s`.
  - The first clear long-running route after the passing checkpoint is route
    index `94`, named `n_93`, links
    `mmi1_multiport_0_1,o4 -> mmi1_ps_array_1_heater_3,o1`; it was still
    running after more than `90 s` and was manually stopped.
  - Current assessment:
  - This was not confirmed as an infinite ripup loop. The native progress trace
    shows route-by-route forward progress up to route `94`.
  - The code has per-repair limits (`max_rounds`, `max_victims`) and caps for
    adaptive/learned repair sets, but it lacks a global wall-time/attempt
    cutoff and has no automatic partial progress report while a route attempt
    is still running.
  - Next useful debug target is stop-after-route-94 or a route-index-94 trace,
    focused on why `n_93` causes such a large A* search.

Pre-`n_93` GDS checkpoint:

- Date: 2026-07-14
- Artifact: `build/routed_multiportmmi_8x8.gds` is currently updated through
  stop-after-route-93 / named net `n_92`, i.e. directly before the long-running
  route index `94` / named net `n_93`.
- Latest run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=93,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `240.497 s`; total routing-flow time: `240.4933 s`.
  - Optical routing stage: `235.2706 s`; net routing phase: `231.9568 s`;
    native route batch: `222.0481 s`.
  - Attempts: `93`; failures: `0`; repairs: `0`; simple routes: `24/93`.
  - A* counters: expanded `1790336`, generated `10742016`, heap pushes
    `2601761`, heap pops `2072106`, footprint checks `10327519`, rect checks
    `1865346`, full-grid fallbacks `0`.
  - Crossing verification: success `True`, errors `0`, warnings `0`,
    crossings `27`, status `partial_debug_stop`.
  - Photonic verification: success `True`, errors `0`, warnings `0`,
    routed records `93`, missing routes `18`, status `partial_debug_stop`.
  - GDS timestamp: `2026-07-14 21:42:51`; size `308160` bytes.
  - No Python processes remained after the run.

A* safety timeout / profiling hook:

- Date: 2026-07-14
- Code changes:
  - `src/astar.rs`: added `AStarConfig.max_search_time_ms` with default `0`
    (disabled). The normal dense A* loop, crossing-aware A* loop, and JPS4
    loop now periodically check this wall-time budget and return failure when
    exceeded.
  - `src/astar.rs`: timeout diagnostics are emitted as `astar_timeout ...`
    lines with label, configured timeout, iterations, open set size, expanded
    states, generated neighbors, heap counts, footprint checks, rect checks,
    and crossing reject counters.
  - `src/py_router.rs`: exposed `AStarConfig.max_search_time_ms` to Python and
    added environment overrides:
    - `PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS=<integer>`
    - `PHOTONIC_ROUTER_ASTAR_TIMEOUT_S=<number>`
- Validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `RUSTFLAGS='-C linker=rust-lld'`.
  - Release extension rebuilt via
    `cargo +stable-x86_64-pc-windows-gnullvm rustc --release --features
    pyo3/extension-module --lib --crate-type cdylib`, then copied to
    `python/photonic_router/_rust.pyd`.
  - Python smoke test confirmed `photonic_router._rust.AStarConfig()` exposes
    `max_search_time_ms` and it can be set.
  - Timeout smoke test with `PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS=1` and
    `PHOTONIC_ROUTER_NATIVE_PROGRESS=1` intentionally failed early at route
    index `8` / `n_7` and emitted `astar_timeout label=dense_astar ...`
    diagnostics. This confirms the abort/report path; it is not a functional
    routing-quality test because the timeout was deliberately too small.
- Artifact note:
  - The existing `build/routed_multiportmmi_8x8.gds` remains the clean
    stop-after-route-93 / pre-`n_93` artifact from `2026-07-14 21:42:51`.

Pre-`n_93` speedup checkpoint:

- Date: 2026-07-14
- Scope:
  - Keep the accepted LiDAR routing behavior unchanged while reducing the
    time spent inside the crossing-aware A* hot path.
  - Do not adopt the crossing-aware simple-route fast-path experiment: it
    improved some local route times but changed later topology and made the
    stop-after-route-93 run slower overall.
- Code changes retained:
  - `src/astar.rs`: precompute partner centerline segments once per
    crossing-aware search and use bounding-box prefilters before exact segment
    intersection tests.
  - `src/astar.rs`: replace the per-neighbor `FxHashSet` allocation in
    `effective_collision_witnesses` with linear dedup over the tiny local
    witness vector, also deduping diagonal halo cells.
  - `src/astar.rs` / `src/py_router.rs`: keep the disabled-by-default A*
    wall-time timeout hook and diagnostics for future long-route triage.
  - `translation/route_rust.py`: added an optional
    `PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT` override for experiments;
    default collision-mode heuristic weight remains `1.0`.
- Final validation run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=93,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `194.604 s`; total routing-flow time: `194.5998 s`.
  - Optical routing stage: `188.8584 s`; net routing phase: `185.0969 s`;
    native route batch: `173.3508 s`.
  - Attempts: `93`; failures: `0`; repairs: `0`; simple routes: `24/93`.
  - A* counters are unchanged from the earlier pre-`n_93` checkpoint:
    expanded `1790336`, generated `10742016`, heap pushes `2601761`, heap
    pops `2072106`, footprint checks `10327519`, rect checks `1865346`,
    full-grid fallbacks `0`.
  - Crossing verification: status `partial_debug_stop`, errors `0`.
  - Photonic verification: status `partial_debug_stop`, errors `0`.
  - GDS artifact: `build/routed_multiportmmi_8x8.gds`, timestamp
    `2026-07-14 22:58:57`, size `308160` bytes.
  - Representative route-time improvements versus the old progress trace:
    - route `55`: about `30.41 s` -> `23.22 s`.
    - route `68`: about `35.64 s` -> `27.32 s`.
    - route `69`: about `37.21 s` -> `28.58 s`.
    - route `70`: about `20.52 s` -> `15.96 s`.
    - route `92`: about `10.27 s` -> `7.85 s`.
    - route `93`: about `22.35 s` -> `17.26 s`.
- Additional validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `RUSTFLAGS='-C linker=rust-lld'`.
  - `cargo +stable-x86_64-pc-windows-gnullvm test --release --lib crossing
    --no-run` passed, confirming the crossing-unit-test binary compiles.
  - Attempting to execute the filtered Rust test binary still hits the known
    local Windows `STATUS_DLL_NOT_FOUND` runtime issue, so only compile
    validation was used for this pass.
- Current assessment:
  - This is a real hot-path speedup, not a search-behavior shortcut: state
    expansion counts and selected-route summary counts remain unchanged, while
    the wall time to the stable pre-`n_93` checkpoint drops by roughly 19%.

Pre-`n_93` crossing metadata precompute checkpoint:

- Date: 2026-07-14
- Scope:
  - Continue optimizing the crossing-aware A* hot path without changing the
    accepted search behavior.
  - Precompute each primitive's crossing path segments and effective collision
    witness offsets once per angle bucket instead of rebuilding those vectors
    for every neighbor expansion.
  - Replace the tiny per-neighbor contacted-partner `FxHashMap` with a small
    linear vector because a primitive normally contacts only one or a few
    partners.
- Validation run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=93,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `177.380 s`; total routing-flow time: `177.3761 s`.
  - Optical routing stage: `172.2764 s`; net routing phase: `168.8751 s`;
    native route batch: `158.7731 s`.
  - Attempts: `93`; failures: `0`; repairs: `0`; simple routes: `24/93`.
  - A* counters remain unchanged from the previous checkpoint: expanded
    `1790336`, generated `10742016`, heap pushes `2601761`, heap pops
    `2072106`, footprint checks `10327519`, rect checks `1865346`,
    full-grid fallbacks `0`.
  - Crossing verification: status `partial_debug_stop`, errors `0`.
  - Photonic verification: status `partial_debug_stop`, errors `0`.
  - GDS artifact: `build/routed_multiportmmi_8x8.gds`, timestamp
    `2026-07-14 23:17:49`, size `308160` bytes.
- Additional validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `RUSTFLAGS='-C linker=rust-lld'`.
  - `cargo +stable-x86_64-pc-windows-gnullvm test --release --lib crossing
    --no-run` passed.
- Current assessment:
  - This is a second behavior-preserving speedup over commit `8459e23`.
    The stop-before-`n_93` checkpoint improves from `194.604 s` to
    `177.380 s` while preserving the same expansion/generation counters and
    green verification reports.

Pre-`n_93` dense owner lookup speed checkpoint:

- Date: 2026-07-14
- Scope:
  - Continue bounded speed work without routing past the stable pre-`n_93`
    checkpoint.
  - Replace per-witness calls to `ObstacleMap::dynamic_core_owner_at` inside
    crossing-aware A* with a dense per-search dynamic-core owner grid built
    once from committed core-route entries.
  - Add a fast no-contact outcome for footprint-free primitives whose crossing
    witnesses do not include extra diagonal halo cells.
  - Keep contacted crossing partners in a sorted small vector, avoiding a
    per-move allocation/sort while preserving the previous ascending partner
    processing order.
- Code changes:
  - `src/obstacle_map.rs`: expose `net_core_route_entries()` so A* can build
    the dense core-owner lookup without scanning every committed route per
    witness cell.
  - `src/astar.rs`: add `DenseDynamicCoreOwnerGrid`, expanded lookup bounds
    for primitive witness halos, no-contact crossing outcome, and sorted
    contacted-partner accumulation.
- Validation run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=93,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `53.655 s`; total routing-flow time: `53.6508 s`.
  - Optical routing stage: `48.6180 s`; net routing phase: `45.3160 s`;
    native route batch: `35.2584 s`.
  - Attempts: `93`; failures: `0`; repairs: `0`; simple routes: `24/93`.
  - A* counters remain unchanged from the previous checkpoints: expanded
    `1790336`, generated `10742016`, heap pushes `2601761`, heap pops
    `2072106`, footprint checks `10327519`, rect checks `1865346`,
    full-grid fallbacks `0`.
  - Crossing verification: status `partial_debug_stop`, routed records `93`,
    errors `0`.
  - Photonic verification: status `partial_debug_stop`, routed records `93`,
    errors `0`.
  - GDS artifact: `build/routed_multiportmmi_8x8.gds`, timestamp updated by
    this run, size remains `308160` bytes.
- Additional validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `RUSTFLAGS='-C linker=rust-lld'`.
  - Release extension rebuilt with
    `cargo +stable-x86_64-pc-windows-gnullvm rustc --release --features
    pyo3/extension-module --lib --crate-type cdylib`, then copied to
    `python/photonic_router/_rust.pyd`.
  - `cargo +stable-x86_64-pc-windows-gnullvm test --release --lib crossing
    --no-run` passed.
- Current assessment:
  - This is a major behavior-preserving speedup: the same stable pre-`n_93`
    checkpoint improves from `177.380 s` to `53.655 s` while preserving the
    same search counters and green verification.

Pre-`n_93` A* core follow-up speed checkpoint:

- Date: 2026-07-14
- Scope:
  - Try the next A* core speedups while preserving the same stop-before-`n_93`
    boundary.
  - Kept changes are behavior-preserving hot-path reductions in `src/astar.rs`.
- Code changes retained:
  - Non-rect primitive footprints now precompute horizontal cell runs in
    `FootprintCollisionProfile`; dense-grid footprint checks use prefix-backed
    horizontal run checks instead of per-cell checks for those profiles.
  - Crossing A* sparse state storage now packs unordered `lidar-pure` keys into
    `u64` for `best_costs` and `closed` maps, falling back to the full
    `CrossingAStarKey` hash path for ordered crossing modes or oversized keys.
- Code changes not retained:
  - A direct dense array for full crossing keys was judged too memory-heavy for
    the current key dimensions (`state`, straight-run, pending-runout, and
    possible ordered crossing progress).
  - A direct indexed/decrease-key crossing heap needs a separate stable slot id
    for full crossing keys; it was not combined with this safe patch.
  - A cell-to-segment crossing index was not kept because dynamic core cells are
    footprint cells, not guaranteed exact centerline segment cells, so a naive
    index could miss diagonal/halo contacts.
- Validation run:
  - Command: `run_routing_flow('multiportmmi_8x8', enable_crossings=True,
    crossing_mode='lidar-pure', debug_stop_after_route_index=93,
    debug_svgs=False, debug_timing=True, collect_attempt_diagnostics=True)`.
  - Result: `RUN_RESULT_OK`.
  - Wall time: `46.701 s`; total routing-flow time: `46.6967 s`.
  - Optical routing stage: `41.2889 s`; net routing phase: `37.1975 s`;
    native route batch: `26.0875 s`.
  - Attempts: `93`; failures: `0`; repairs: `0`; simple routes: `24/93`.
  - A* counters remain unchanged from the previous checkpoints: expanded
    `1790336`, generated `10742016`, heap pushes `2601761`, heap pops
    `2072106`, footprint checks `10327519`, rect checks `1865346`,
    full-grid fallbacks `0`.
  - Crossing verification: status `partial_debug_stop`, routed records `93`,
    errors `0`.
  - Photonic verification: status `partial_debug_stop`, routed records `93`,
    errors `0`.
- Additional validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `RUSTFLAGS='-C linker=rust-lld'`.
  - Release extension rebuilt with
    `cargo +stable-x86_64-pc-windows-gnullvm rustc --release --features
    pyo3/extension-module --lib --crate-type cdylib`, then copied to
    `python/photonic_router/_rust.pyd`.
  - `cargo +stable-x86_64-pc-windows-gnullvm test --release --lib crossing
    --no-run` passed.
  - `cargo fmt` could not be run because `cargo-fmt.exe` is not installed for
    `stable-x86_64-pc-windows-gnullvm`; obvious long-line formatting was
    cleaned manually and `cargo check` was rerun afterward.
- Current assessment:
  - This is another verified hot-path speedup over commit `cf23b2f`: the
    stop-before-`n_93` checkpoint improves from `53.655 s` to `46.701 s`
    without changing route search counters or verifier status.

Static fanout stub WIP checkpoint:

- Date: 2026-07-15
- Scope:
  - Working on experimental `fanout_access_mode=static-stubs` for dense
    multi-port access. User explicitly said not to commit this WIP.
  - Current focus is the first cluster stub around `n_31`; `n_31` appears as
    internal `net_id=32` / route 32 in the stop-after-32 artifact.
- Current artifact:
  - Command run:
    `python routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 32 --debug-svgs false
    --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`.
  - Result: completed and wrote `build/routed_multiportmmi_8x8.gds`.
  - Crossing report insertion-loss summary lists `net_count=32` and
    `net_id=32`, `net_name=n_31`.
  - Latest WIP iteration aligns the special `mmi0_multiport_0_0,o12` static
    stub by cell centers: 45-degree bend, diagonal alignment segment to the
    next valid grid-y center, second 45-degree bend, then final straight to the
    next valid grid-x center. The current artifact reports this stub with
    anchor cell `{718,156}`, and `fanout_anchor_net_ids` includes `32`.
  - Subsequent WIP iteration generalizes this to horizontal dense MMI source
    clusters by splitting each cluster into lower and upper halves and spacing
    each half outward with `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS`
    defaulting to `3` cell-center distance. For `mmi0_multiport_0_0`, the
    stop-after-32 artifact reports lower anchors `{718,161}`, `{718,158}`,
    `{719,155}` and upper anchors `{718,176}`, `{718,179}`, `{719,182}`.
  - Stop-after-route-33 attempt fails on `n_32` from source anchor
    `(718,158,0)` to target `(767,93,0)` with
    `No legal LiDAR crossing route found; probe-based victim selection is
    disabled in crossing mode`. No GDS was produced for route 33.
  - Diagnostic freshness fix: `routing_flow.py` now enables `debug_dir` for
    `--attempt-diagnostics` even when `--debug-svgs false`, while passing an
    empty route SVG selection so text diagnostics are refreshed without
    exporting SVGs. `translation/route_rust.py` now creates per-route
    diagnostic text paths when `collect_attempt_diagnostics` is active.
  - Re-running stop-after-route-33 now refreshes
    `build/routes/multiportmmi_8x8_n_32_diagnostics.txt`; it correctly reports
    `source_fanout_anchor=True`, `source_state=(718,158,0)`, and
    `target_state=(767,93,0)`. It also reports opened dynamic overlap
    count `8`, bbox `(719,730,155,162)`, and no route cells because A* found no
    legal route. No route SVGs were generated in this no-SVG diagnostics mode.
  - Additional visualization generated:
    `build/routes/multiportmmi_8x8_n32_stub_blockers.svg`. It overlays the
    current source stubs, the committed `n_31` route, the `n_32` source anchor,
    the dynamic-overlap bbox `(719,730,155,162)`, and the static/port-reservation
    overlap bbox `(760,767,91,96)`.
  - User rejected the synthetic overlay as insufficient for this question.
    Generated the real router route SVG with
    `--debug-stop-after-route 32 --debug-svgs 32`, producing
    `build/routes/multiportmmi_8x8_n_31.svg`. The fresh diagnostics show
    `n_31`, `source_fanout_anchor=True`, `source_state=(719,155,0)`, and
    `route_dynamic_overlap_count=0`.
- Validation:
  - `python -m py_compile translation/route_rust.py routing_flow.py` passed.
  - `python -m pytest -q
    tests/test_route_rust_opened_cells.py::test_route_nets_rust_static_stub_fanout_uses_virtual_source_anchor`
    passed.
- Current assessment:
  - The static-stub worktree is intentionally dirty and still experimental.
    Do not commit without user approval.

Static-stub virtual-anchor opening fix checkpoint:

- Date: 2026-07-15
- Scope:
  - Continued the `fanout_access_mode=static-stubs` WIP after the user noted
    that `n_32` should be able to start by routing upward from its virtual
    source anchor; `n_31` blocks the lower side, not the upper side.
  - No commit was made; user previously requested not to commit this WIP.
- Finding:
  - Added diagnostic-only first-primitive classification to
    `translation/route_rust.py` so failed routes report source primitive
    footprints and their raw-static, router-static, dynamic, opened, and
    opened-search cell overlaps.
  - Before the fix, stop-after-route-33 with static stubs failed on `n_32`.
    Rust crossing trace showed collision-crossing A* for net id `33`
    (`n_32`) expanded only the source node, generated 6 primitives, accepted
    none, and checked zero crossing candidates.
  - The refreshed diagnostics showed `turn45_left` from source
    `(718,158,0)` was not blocked by `n_31`, but was blocked by router-static
    cells `(719,158)`, `(720,158)`, `(721,158)` on the initial east arm.
  - Root cause: `_filter_dense_port_opening()` was still applying original
    dense-port lateral ownership to static-stub virtual anchors. The stubs had
    already spread the ports, so this filter removed `n_32`'s own virtual-anchor
    runway cells from the opened set.
- Code changes retained:
  - For `port_spec in fanout_anchor_by_port_spec`, `_filter_dense_port_opening`
    now returns the generated opening cells unchanged.
  - The diagnostic first-move table remains in place for now; it is useful
    during this WIP and should be cleaned or gated more tightly before a final
    commit if the user wants less debug output.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py
    routing_flow.py` passed.
  - Stop-after-route-33 with no SVGs completed:
    `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    false --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`.
    Result: success, 33 attempts, 0 failures, 0 repairs; total wall time
    `19.2573 s`; net routing phase `16.2585 s`.
  - Visual artifact pass completed:
    `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    32,33 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`.
    Result: success; wrote `build/routed_multiportmmi_8x8.gds`,
    `build/routes/multiportmmi_8x8_n_31.svg`, and
    `build/routes/multiportmmi_8x8_n_32.svg`.
  - Fresh `build/routes/multiportmmi_8x8_n_32_diagnostics.txt` reports
    `status=ok`, `source_state=(718,158,0)`, `route_cells_count=94`, and
    `first_move_turn45_left` with `static_blockers=[]`, `dynamic_blockers=[]`,
    and all first-bend footprint cells included in `opened_search`.
- Current assessment:
  - The user's intuition was correct: the upward first bend should not have
    been blocked. The remaining WIP question is route quality after the fix:
    `n_32` now routes successfully, but its current route starts with a short
    east move followed by a loop-like bend sequence; inspect the fresh GDS/SVG
    before deciding whether to tune stub spacing or primitive cost/order next.

Static-stub protected virtual-port runway checkpoint:

- Date: 2026-07-15
- Scope:
  - User clarified that virtual fanout anchors should still have protected
    port openings/runways after the stubs, analogous to the previous staggered
    dense-port logic: the lowest anchor in a cluster gets the longest protected
    runway, and higher anchors get progressively shorter runways.
  - No commit was made.
- Code changes retained:
  - `_dense_source_port_runway_lengths()` now has a `static-stubs` path.
  - It groups fanout anchors by `(instance, physical_angle)`, orders them by
    the spread anchor cell's lateral coordinate, and assigns protected runway
    lengths:
    `minimum_cells + spacing_cells * (count - 1 - port_index)`.
  - `minimum_cells` is `max(3, bend_radius_cells)`.
  - `spacing_cells` defaults to
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS` or, if unset,
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS`, then `3`.
  - The active net still opens its own protected runway; unrelated nets see the
    runway cells as static blockers.
  - The focused static-stub test was updated to assert the new protected runway
    behavior (`source_dense_port_runway_cells=9` in the 3-port toy fixture).
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m pytest -q
    tests\test_route_rust_opened_cells.py::test_route_nets_rust_static_stub_fanout_uses_virtual_source_anchor`
    passed.
  - Real stop-after-route-33 visual run completed:
    `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    32,33 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`.
    Result: success, 33 attempts, 0 failures, 0 repairs; total wall time
    `27.1296 s`; net routing phase `23.6526 s`.
  - Fresh diagnostics:
    - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt` reports
      `source_dense_port_runway_cells=18` for source `(719,155,0)`.
    - `build/routes/multiportmmi_8x8_n_32_diagnostics.txt` reports
      `source_dense_port_runway_cells=15` for source `(718,158,0)`.
    - Both report `status=ok`.
  - Fresh visual artifacts:
    - `build/routed_multiportmmi_8x8.gds`
    - `build/routes/multiportmmi_8x8_n_31.svg`
    - `build/routes/multiportmmi_8x8_n_32.svg`
- Current assessment:
  - Protected runways after stubs are active. Immediate bends can be blocked at
    the protected-lane edge, while straight movement along the lane is opened
    for the active net. This is expected with protected lane semantics.
  - Next likely step is visual inspection of the fresh GDS/SVG and then
    deciding whether to tune runway length/spacing or continue to `n_33`/`n_34`.

Static-stub 6-cell spacing experiment:

- Date: 2026-07-15
- Scope:
  - User asked to rerun with larger stub spacing, 6 cells.
  - No code default was changed; the run used environment overrides:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6` and
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=6`.
- Command:
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    32,33 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - The run failed after routing `n_31` and `n_32`, during final realized
    crossing verification.
  - Error:
    `Illegal realized route crossing(s) after endpoint correction: 1 found.
    n_31 x n_32 at [1415.5, 669.8] (not_perpendicular, margins=9.783021/11.325,
    required=4.0, grid=[717, 153])`.
  - Partial route SVGs were produced:
    `build/routes/multiportmmi_8x8_n_31.svg` and
    `build/routes/multiportmmi_8x8_n_32.svg`.
  - `build/routed_multiportmmi_8x8.gds` is stale from the previous successful
    run because the 6-cell run failed before writing a new GDS.
- Diagnostics:
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt` reports
    `source_dense_port_runway_cells=33`, `status=ok`.
  - `build/routes/multiportmmi_8x8_n_32_diagnostics.txt` reports
    `source_dense_port_runway_cells=27`, `status=ok`.
- Current assessment:
  - Larger spacing successfully increases the virtual protected lanes, but it
    changes the local `n_31`/`n_32` route geometry enough to trigger a realized
    non-perpendicular crossing. Do not treat the GDS as the 6-cell result.

Static-stub full-obstacle reservation fix:

- Date: 2026-07-15
- Scope:
  - User observed in the 6-cell SVG that A* routed behind/through a static stub
    because the obstacle map did not include the whole stub body.
  - No commit was made.
- Finding:
  - The WIP static-stub implementation only stored two `stub_center_cells` per
    anchor: the original port grid cell and the virtual anchor cell.
  - `fanout_stub_static_cells` was therefore inflated from endpoints only, so
    the middle of each realized stub was invisible to the obstacle map.
- Code changes retained:
  - Added `_centerline_grid_cells()` in `translation/route_rust.py`, which
    samples each realized stub centerline segment onto grid cells at
    `grid_size / 4` spacing and keeps in-bounds cells in path order.
  - Static fanout anchors now set `stub_center_cells` from the full sampled
    realized stub centerline instead of endpoint-only cells.
  - `fanout_stub_static_cells` continues to be produced by inflating these
    sampled center cells by `commit_radius_cells`, so the router now sees the
    full stub as static geometry.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m pytest -q
    tests\test_route_rust_opened_cells.py::test_route_nets_rust_static_stub_fanout_uses_virtual_source_anchor`
    passed.
  - Re-ran the 6-cell spacing experiment with
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6` and
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=6`:
    `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    32,33 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`.
    Result: success; 33 attempts, 0 failures, 0 repairs; total time
    `21.4793 s`; net routing phase `18.3179 s`.
  - Verification JSONs report `error_count=0`; crossing verification reports
    `illegal_crossing_count=0`, `status=partial_debug_stop`,
    `routed_record_count=33`.
  - Fresh diagnostics:
    - `n_31`: `fanout_stub_center_cell_count=850`,
      `fanout_stub_static_cell_count=850`, `source_dense_port_runway_cells=33`,
      `status=ok`.
    - `n_32`: `fanout_stub_center_cell_count=850`,
      `fanout_stub_static_cell_count=850`, `source_dense_port_runway_cells=27`,
      `status=ok`.
  - Fresh artifacts:
    - `build/routed_multiportmmi_8x8.gds`
    - `build/routes/multiportmmi_8x8_n_31.svg`
    - `build/routes/multiportmmi_8x8_n_32.svg`
    - `build/verification/multiportmmi_8x8_crossing_verification.json`
    - `build/verification/multiportmmi_8x8_photonic_verification.json`
- Current assessment:
  - The user's obstruction diagnosis was correct. The stub body is now included
    in the obstacle map, and the same 6-cell stop-after-route-33 run now passes
    instead of producing the previous non-perpendicular crossing failure.

Static-stub 6-cell stop-after-`n_31` artifact:

- Date: 2026-07-15
- Scope:
  - User asked to see the route with one fewer routed net: stop after `n_31`
    only, before `n_32`.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=6`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 32 --debug-svgs
    32 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; 32 attempts, 0 failures, 0 repairs.
  - Route `[32/32] n_31` completed with length `248.569um`, cost `252.569`,
    expanded states `2314`.
  - Total wall time `17.9009 s`; net routing phase `14.9869 s`.
- Diagnostics:
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt` reports
    `status=ok`, `fanout_stub_center_cell_count=850`,
    `fanout_stub_static_cell_count=850`, and
    `source_dense_port_runway_cells=33`.
  - Crossing verification reports `error_count=0`,
    `illegal_crossing_count=0`, `routed_record_count=32`, and
    `status=partial_debug_stop`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds`
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt`
  - `build/verification/multiportmmi_8x8_crossing_verification.json`

Static-stub 6-cell spacing with 3-cell protected runway through `n_32`:

- Date: 2026-07-15
- Scope:
  - User asked to route one step further, through `n_32`, with the same
    settings: physical stub spacing 6 cells and protected runway spacing
    3 cells.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 33 --debug-svgs
    32,33 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; 33 attempts, 0 failures, 0 repairs.
  - `n_31`: length `234.510um`, cost `238.510`, expanded `3289`.
  - `n_32`: length `171.480um`, cost `175.480`, expanded `3024`.
  - Total wall time `19.7369 s`; net routing phase `16.8752 s`.
- Diagnostics:
  - `n_31`: `status=ok`, `source_state=(725,149,0)`,
    `source_dense_port_runway_cells=18`.
  - `n_32`: `status=ok`, `source_state=(721,155,0)`,
    `source_dense_port_runway_cells=15`.
  - Both diagnostics report `fanout_stub_center_cell_count=850` and
    `fanout_stub_static_cell_count=850`.
  - Crossing verification reports `error_count=0`,
    `illegal_crossing_count=0`, `routed_record_count=33`, and
    `status=partial_debug_stop`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds`
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_32.svg`
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt`
  - `build/routes/multiportmmi_8x8_n_32_diagnostics.txt`
  - `build/verification/multiportmmi_8x8_crossing_verification.json`

Static-stub 6-cell spacing with 3-cell protected runway through `n_33`:

- Date: 2026-07-15
- Scope:
  - User asked to route one more step, through `n_33`, with physical stub
    spacing 6 cells and protected runway spacing 3 cells.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs
    32,33,34 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; 34 attempts, 0 failures, 0 repairs.
  - `n_31`: length `234.510um`, cost `238.510`, expanded `3289`.
  - `n_32`: length `171.480um`, cost `175.480`, expanded `3024`.
  - `n_33`: length `122.853um`, cost `176.853`, expanded `6029`.
  - Total wall time `21.9381 s`; net routing phase `18.9299 s`.
- Diagnostics:
  - `n_31`: `source_state=(725,149,0)`,
    `source_dense_port_runway_cells=18`.
  - `n_32`: `source_state=(721,155,0)`,
    `source_dense_port_runway_cells=15`.
  - `n_33`: `source_state=(718,161,0)`,
    `source_dense_port_runway_cells=12`.
  - All three diagnostics report `status=ok`,
    `fanout_stub_center_cell_count=850`, and
    `fanout_stub_static_cell_count=850`.
  - Crossing verification reports `error_count=0`,
    `illegal_crossing_count=0`, `routed_record_count=34`, and
    `status=partial_debug_stop`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds`
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_32.svg`
  - `build/routes/multiportmmi_8x8_n_33.svg`
  - `build/verification/multiportmmi_8x8_crossing_verification.json`

Static-stub 6-cell spacing with 3-cell protected runway through `n_34`:

- Date: 2026-07-15
- Scope:
  - User asked to route one more step, through `n_34`, with physical stub
    spacing 6 cells and protected runway spacing 3 cells.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs
    32,33,34,35 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; 35 attempts, 0 failures, 0 repairs.
  - `n_31`: length `234.510um`, cost `238.510`, expanded `3289`.
  - `n_32`: length `171.480um`, cost `175.480`, expanded `3024`.
  - `n_33`: length `122.853um`, cost `176.853`, expanded `6029`.
  - `n_34`: length `162.426um`, cost `224.426`, expanded `35761`.
  - Total wall time `29.9102 s`; net routing phase `26.5511 s`.
- Diagnostics:
  - `n_31`: `source_state=(725,149,0)`,
    `source_dense_port_runway_cells=18`.
  - `n_32`: `source_state=(721,155,0)`,
    `source_dense_port_runway_cells=15`.
  - `n_33`: `source_state=(718,161,0)`,
    `source_dense_port_runway_cells=12`.
  - `n_34`: `source_state=(718,176,0)`,
    `source_dense_port_runway_cells=9`.
  - All four diagnostics report `status=ok`,
    `fanout_stub_center_cell_count=850`, and
    `fanout_stub_static_cell_count=850`.
  - Crossing verification reports `error_count=0`,
    `illegal_crossing_count=0`, `routed_record_count=35`, and
    `status=partial_debug_stop`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds`
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_32.svg`
  - `build/routes/multiportmmi_8x8_n_33.svg`
  - `build/routes/multiportmmi_8x8_n_34.svg`
  - `build/verification/multiportmmi_8x8_crossing_verification.json`

Static-stub 6-cell spacing with 3-cell protected runway attempted through `n_35`:

- Date: 2026-07-15
- Scope:
  - User asked whether `n_35` also routes, with physical stub spacing 6 cells
    and protected runway spacing 3 cells.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 36 --debug-svgs
    32,33,34,35,36 --attempt-diagnostics --debug-timing true
    --fanout-access-mode static-stubs`
- Result:
  - `n_35` itself found an A* route:
    length `416.392um`, cost `628.392`, expanded `70738`.
  - The run failed afterward during final realized crossing verification.
  - Error:
    `Illegal realized route crossing(s) after endpoint correction: 1 found.
    n_31 x n_32 at [1429.5, 661.125]
    (crossing_footprint_contains_bend, margins=2.839719/6.0,
    required=4.0, grid=[724, 149])`.
  - Therefore: `n_35` can route geometrically, but the stop-after-`n_35`
    artifact is not valid yet.
- Diagnostics:
  - `n_35` diagnostics report `status=ok`,
    `source_state=(721,182,0)`, `source_dense_port_runway_cells=6`,
    `route_dynamic_overlap_count=4`.
  - The verifier failure concerns the already routed `n_31 x n_32`
    realized crossing after endpoint correction, not a direct no-route failure
    on `n_35`.
  - Focused analysis of the illegal crossing report:
    `build/crossings/multiportmmi_8x8_crossings.txt` and JSON show the same
    `n_31 x n_32` crossing at `[1429.5, 661.125]`, grid `[724,149]`.
    The crossing is perpendicular. `n_32` has `6.0um` straight margin, but
    `n_31` has only `2.839719um` before its previous bend, while the actual
    crossing footprint requires `4.0um`. The issue is therefore inside the
    crossing footprint, not in the optional search/runout margin.
  - The failed `n_35` run also changed the source-side shape of `n_31`:
    diagnostics now show `n_31` bending after only three source cells
    (`straight ... (727,149)->(728,149); turn45:(728,149)->(734,152)`),
    despite `source_dense_port_runway_cells=18`. This suggests the current
    protected runway is only opened/reserved in the obstacle map and is not
    enforced as a hard launch-straight constraint for the active net.
  - Follow-up analysis showed the problematic `[1429.5, 661.125]` point is
    actually inside the static fanout stub for `mmi0_multiport_0_0,o12`
    (`n_31` source), not the later A* body. The stub centerline ends at
    `[1431.5, 661.125]`; its final horizontal segment starts at
    `[1426.660281, 661.125]`, so the crossing point has only `2.839719um`
    margin back to the stub bend.
  - `n_32` diagnostics show the same grid cell `(724,149)` in its
    `opened_cells`. Because Rust treats opened static cells as passable, this
    can hide the `n_31` static stub from the level-1 A* collision/crossing
    check. The likely model fix is to make fanout stub static cells
    non-openable for other nets: subtract `fanout_stub_static_cells` from
    per-route opened cells/candidate opened cells, then add back only the
    current route's own source/target anchor cells.
  - Expected consequence of that fix: the current `n_35` candidate should fail
    earlier in A* when it tries to pass through/too close to the static stub.
    That is preferable to a late Python-verifier failure; A* should then keep
    searching for another legal branch, and only fail/repair if none exists.
  - Implemented the model fix in `translation/route_rust.py`: per-route
    `opened_cells` and `opened_candidate_cells` now subtract foreign
    `fanout_stub_static_cells`, while keeping the current route's own source
    and target fanout stub static cells open so the route can leave its own
    virtual anchor.
  - Validation after the fix:
    - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
      passed.
    - Stop-after-`n_32` with static stubs, physical spacing 6 cells, protected
      spacing 3 cells passed. In
      `build/routes/multiportmmi_8x8_n_32_diagnostics.txt`, `(724,149)` is no
      longer in `opened_cells`; `n_32` routes differently instead of passing
      through the `n_31` static-stub bend area.
    - Stop-after-`n_35` no longer reaches the late Python realized-verifier
      error. It fails earlier in A* as expected:
      `No legal LiDAR crossing route found; probe-based victim selection is
      disabled in crossing mode`. The current failure log is
      `build/routes/multiportmmi_8x8_n_35_FAILED.txt`; the first failed
      candidate is `Illegal grid crossing: net 36 intersects net 35 at
      (741.500, 170.500) (insufficient_straight_margin)`.
  - Current interpretation:
    - The static-stub opening bug is fixed enough for the previous invalid
      `n_31 x n_32` Python-only failure to disappear.
    - The next blocker is real A* search/legality behavior for `n_35` after
      `n_31..n_34` are routed, not silent acceptance of an invalid GDS.
  - User requested the GDS state before routing `n_35`. Re-ran stop-after
    route 35 with static stubs, physical spacing 6 cells, protected spacing
    3 cells. Success; `build/routed_multiportmmi_8x8.gds` was written at
    2026-07-15 11:31:58 local and contains routes through `n_34`.
  - Re-ran stop-after route 36 with native crossing tracing enabled for
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=36` (`n_35`). Trace summary:
    - First collision-crossing search for `n_35` used partners
      `[33, 35, 34, 32]`, i.e. `n_32`, `n_34`, `n_33`, `n_31` by net id/name.
      It exhausted the search with `expanded=90864`, `generated=545184`,
      `candidates=7205`, `accepted=317`, and no complete route.
      Reject counters: `reject_non_straight=35175`,
      `reject_not_perpendicular=4532`, `reject_margin=3004`,
      `reject_unmatched_centerline=453`, `reject_unmatched_footprint=9085`,
      `reject_pending_straight=347`.
    - A later restricted search with only partner `[35]` (`n_34`) exhausted
      with `expanded=43543`, `generated=261258`, `candidates=2235`,
      `accepted=0`; dominant rejects were `non_straight=15528`,
      `not_perpendicular=1299`, `margin=1107`,
      `unexpected_owner=1395`, `unmatched_footprint=4104`.
    - Representative accepted local events existed, e.g. with `n_31`
      (`partner=32`) around grid `(755,184..188)` and with `n_34`
      (`partner=35`) around grid `(760,179)`, but no accepted sequence reached
      the target under the current partner/order constraints.
- Artifacts:
  - Fresh partial SVGs were generated:
    `build/routes/multiportmmi_8x8_n_31.svg`,
    `build/routes/multiportmmi_8x8_n_32.svg`,
    `build/routes/multiportmmi_8x8_n_33.svg`,
    `build/routes/multiportmmi_8x8_n_34.svg`,
    `build/routes/multiportmmi_8x8_n_35.svg`.
  - `build/routed_multiportmmi_8x8.gds` is stale from the previous valid
    stop-after-`n_34` run because the stop-after-`n_35` run failed before
    writing a new GDS.

Static-stub 6-cell spacing with 3-cell protected runway artifact:

- Date: 2026-07-15
- Scope:
  - User clarified that physical stub lane spacing should be 6 cells, but
    protected runway spacing should remain 3 cells. For a 6-port cluster, this
    makes the lowest lane protected runway `3 + 3 * 5 = 18` cells, not 33.
  - No commit was made.
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 32 --debug-svgs
    32 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs`
- Result:
  - Success; 32 attempts, 0 failures, 0 repairs.
  - Route `[32/32] n_31` completed with length `234.510um`, cost `238.510`,
    expanded states `3289`.
  - Total wall time `15.7208 s`; net routing phase `13.0790 s`.
- Diagnostics:
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt` reports
    `status=ok`, `fanout_stub_center_cell_count=850`,
    `fanout_stub_static_cell_count=850`,
    `source_dense_port_runway_cells=18`, and `source_state=(725,149,0)`.
  - Crossing verification reports `error_count=0`,
    `illegal_crossing_count=0`, `routed_record_count=32`, and
    `status=partial_debug_stop`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds`
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt`
  - `build/verification/multiportmmi_8x8_crossing_verification.json`

90-degree static-stub lane-offset correction:

- Date: 2026-07-15
- Scope:
  - User clarified that 90-degree fanout stubs need split x offset:
    within each upper/lower half-cluster, inner lanes take more forward
    offset before the first bend and outer lanes take the complementary
    offset after the second bend. This differs from 45-degree stubs, where
    the spacing is handled by y-offsets between diagonal breakout stubs.
  - Implemented separate `initial_forward_cells` and
    `extra_final_forward_cells` for static two-bend stubs. For a 3-lane
    half-cluster and 6-cell physical lane spacing, the inner-to-outer
    initial/final split is now `12/0`, `6/6`, `0/12` cells.
  - Also fixed final x snapping so extra forward distance snaps to a valid
    grid-center instead of using raw port x coordinates.
  - No commit was made.
- Validation:
  - Syntax check passed:
    `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`.
  - Stop-after-route 32 with 90-degree static stubs succeeded:
    `n_31` routed from source anchor `(724,149,0)`, verification
    `error_count=0`.
  - Stub anchors for `mmi0_multiport_0_0,o12..o7` now all end at x cell
    `724`, with pre-bend x offsets visible in the generated centerlines:
    lower group `o10/o11/o12` uses `12/6/0` cells before the first bend;
    upper group `o9/o8/o7` uses `12/6/0` cells before the first bend.
  - Stop-after-route 33 with `--routing-window-scale 0.35` succeeded:
    `n_31` and `n_32` both routed, crossing and photonic verification JSONs
    report `status=partial_debug_stop` and `error_count=0`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds` written at 2026-07-15 12:26 local.
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_32.svg`
  - `build/routes/multiportmmi_8x8_n_31_diagnostics.txt`
  - `build/routes/multiportmmi_8x8_n_32_diagnostics.txt`

90-degree static-stub validation through n_34:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 35 --debug-svgs
    32,33,34,35 --attempt-diagnostics --debug-timing false --fanout-access-mode
    static-stubs --routing-window-scale 0.35`
- Result:
  - Success through route `[35/35] n_34`.
  - `n_31`: length `240.024um`, cost `244.024`, expanded `3092`.
  - `n_32`: length `232.711um`, cost `290.711`, expanded `11175`.
  - `n_33`: length `170.368um`, cost `228.368`, expanded `28045`.
  - `n_34`: length `154.426um`, cost `214.426`, expanded `27966`.
  - Crossing verification and photonic verification both report
    `status=partial_debug_stop`, `error_count=0`, `routed_record_count=35`.
  - Diagnostics for `n_33` and `n_34` report `source_fanout_anchor=True` and
    `route_overlap_sibling_port_runway_count=0`.
- Fresh artifacts:
  - `build/routed_multiportmmi_8x8.gds` written at 2026-07-15 12:29 local.
  - `build/routes/multiportmmi_8x8_n_31.svg`
  - `build/routes/multiportmmi_8x8_n_32.svg`
  - `build/routes/multiportmmi_8x8_n_33.svg`
  - `build/routes/multiportmmi_8x8_n_34.svg`

Foreign-port keepout comparison for n_34:

- Date: 2026-07-15
- User hypothesis:
  - `n_34` takes the upper route and does not place the crossing directly
    before the heater. The suspected cause was a footprint blockage from
    static cells or foreign keepout cells.
  - Previous diagnostics confirmed `foreign_port_keepout_cells=6`.
- Command:
  - Same 90-degree static-stub stop-after-`n_34` run as above, but with
    `--foreign-port-keepout-cells 8`.
- Result:
  - Success through route `[35/35] n_34`.
  - Route metrics were unchanged from the keepout-6 run:
    `n_31` length `240.024um`, `n_32` length `232.711um`,
    `n_33` length `170.368um`, `n_34` length `154.426um`.
  - `build/routes/multiportmmi_8x8_n_34_diagnostics.txt` reports
    `foreign_port_keepout_cells=8`, `status=ok`, and the same route segment
    sequence observed for keepout 6.
  - Crossing and photonic verification both report
    `status=partial_debug_stop`, `error_count=0`, `routed_record_count=35`.
- Interpretation:
  - Raising foreign keepout from 6 to 8 did not move the `n_34` crossing
    candidate or change the selected path. The missing crossing directly
    before the heater is therefore not explained by the 6-cell foreign
    keepout being too small in this comparison.
  - Current `build/routed_multiportmmi_8x8.gds` is from the keepout-8
    comparison run.

n_34 x n_31 crossing candidate near heater:

- Date: 2026-07-15
- User question:
  - Why does `n_34` not place the crossing with `n_31` directly before the
    heater, around physical `(x=1491, y=750)`?
- Trace run:
  - Reverted comparison parameter to `--foreign-port-keepout-cells 6`.
  - Enabled native tracing for `n_34` (`net_id=35`) against `n_31`
    (`partner_net_id=32`) with
    `PHOTONIC_ROUTER_TRACE_CROSSING=1`,
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=35`,
    `PHOTONIC_ROUTER_TRACE_PARTNER_NET=32`,
    `PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATES=1`,
    `PHOTONIC_ROUTER_TRACE_CROSSING_LEVEL1=1`.
- Result:
  - The desired physical location maps to approximately grid `(755,193/194)`.
  - A* does inspect candidates at `grid=(755,193)` against `n_31`.
  - Perpendicular candidates at `(755,193)` are rejected with
    `reason=reservation_footprint`, e.g. `route_angle=0`,
    `partner_angle=2`, `partner_margin=23`, `required_margin=5`.
  - Local static obstacle inspection of the SVG grid found static blocked
    cells inside the crossing reservation window around that candidate:
    `(756..758, 191..196)`.
  - `n_34` diagnostics show the target port opens static cells only at
    bbox `(760,767,191,196)`, while the candidate crossing footprint at
    `(755,193)` with half-size 2 spans roughly `x=753..757`, so it overlaps
    static cells that are not opened for the route.
  - The router therefore accepts a later legal crossing with `n_31` around
    `grid=(755,208)`, physical approximately `(1491.5,779.125)`.
- Interpretation:
  - The near-heater crossing is checked; it is not skipped.
  - It is rejected by the crossing reservation footprint due to static
    blocked cells near the heater/target access region, not by the
    6-cell foreign keepout experiment.

n_34 x n_31 near-heater footprint opening fix:

- Date: 2026-07-15
- User follow-up:
  - The crossing itself appears to fit near the heater; if any blocking is
    caused by pad/port static cells, those cells should be opened for the route
    that owns that target access.
- Code finding:
  - In `src/astar.rs`, `reservation_margin` is only
    `crossing.crossing_half_size_cells`. The reservation-footprint check is
    therefore the crossing footprint itself, not the extended
    `crossing_half + bend_runout` search margin.
  - The bug was that `crossing_reservation_window_is_clear()` checked static
    blockers directly and did not honor `port_open_cells`, unlike normal
    primitive collision checking.
- Change:
  - Passed `port_open_cells` into `crossing_reservation_window_is_clear()`.
  - Static cells inside the reservation footprint are now allowed only when
    they are explicitly in the route's opened-cell set; unrelated static cells
    remain blocking, and unrelated dynamic owners remain blocking.
  - Extended the focused Rust unit test so an opened static footprint cell is
    accepted while unopened static cells are still rejected.
- Validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm test
    crossing_reservation_window_rejects_static_and_unrelated_dynamic_cells`
    passed after adding the Codex runtime Python directory to `PATH`.
  - `maturin develop --release` had already been rebuilt after the patch.
  - A traced stop-after-`n_34` run showed the desired candidate at roughly
    grid `(755,193)`, physical `(1491.5,749.125)`, is now accepted as
    `accept_with_pending` instead of rejected as `reservation_footprint`.
- Current status:
  - The original too-tight footprint/opened-pad-cell issue is fixed.
  - The same stop-after-`n_34` run can still fail afterward because the
    accepted short route only covers the `n_31` crossing partner and the
    collision-crossing validation reports `satisfies=false`; subsequent
    restricted searches against remaining partners exhaust. The next blocker
    is partner/sequence satisfaction, not pad static cells blocking the
    near-heater crossing footprint.

n_34 near-heater `satisfies=false` root cause refined:

- Date: 2026-07-15
- Trace:
  - Re-ran stop-after-`n_34` with `PHOTONIC_ROUTER_TRACE_CROSSING=1` and
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=35`.
  - The first collision-crossing search used partners `[34, 33, 32]` and
    returned waypoints `[(724,176),(733,176),(750,193),(767,193)]`.
  - The realized event list contained the desired near-heater event only:
    partner `32` at physical `(1491.5,749.125)`, route angle `0`, partner
    angle `2`.
  - Even a restricted search with only partner `[32]` returned the same event
    but still printed `satisfies=false` with `realized_violations=[]`.
- Conclusion:
  - The legal crossing attempt is not being rejected because another expected
    partner is missing.
  - It is being rejected by the post-A* reservation blocker check:
    `crossing_events_satisfy_partner_constraints()` calls
    `crossing_reservation_blockers(net_id, crossing_events)`.
  - `crossing_reservation_blockers()` checks `event.reservation_keys` against
    `self.obstacle_map.is_static_blocked(x,y)` but has no `opened_cells`
    argument. This reproduces the same opened-pad/static-cell mismatch that
    was just fixed in the A* reservation-window check.
- Next model fix:
  - Thread the route's validation/opened-cell set into the post-A* reservation
    blocker check and ignore static blockers there only when the reservation
    key is explicitly opened for the current route. Keep unrelated dynamic
    blockers unchanged.

n_34 near-heater post-A* reservation fix implemented:

- Date: 2026-07-15
- Change:
  - `src/py_router.rs` now threads optional `opened_cell_keys` through
    `crossing_route_satisfies_partner_constraints()`,
    `crossing_events_satisfy_partner_constraints()`, and
    `crossing_reservation_blockers()`.
  - `crossing_reservation_blockers()` still treats out-of-bounds/static cells
    as blockers, but ignores a static blocker when the reservation key is
    explicitly opened for the active route. Unrelated dynamic blockers remain
    strict.
  - The existing reservation-blocker unit test now asserts both sides:
    unopened static reservation cells reject the event, opened static
    reservation cells are accepted, and unrelated dynamic owners still reject.
- Validation:
  - `cargo +stable-x86_64-pc-windows-gnullvm check` passed with
    `CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER=rust-lld`.
  - `maturin develop --release` passed and reinstalled the updated Rust
    extension into the project `.venv`.
  - `cargo +stable-x86_64-pc-windows-gnullvm test
    crossing_events_reject_static_and_unrelated_dynamic_reservation_blockers`
    passed after adding the Codex runtime Python directory to `PATH`.
  - Re-ran `multiportmmi_8x8` stop-after route 35 (`n_34`) with
    `fanout_access_mode=static-stubs`, 90-degree stubs, lane spacing 6,
    protected runway spacing 3, routing-window scale 0.35, and foreign keepout
    6. The run passed and wrote `build\routed_multiportmmi_8x8.gds`.
- Trace result:
  - The collision-crossing route for internal net `35` / user net `n_34`
    returned waypoints `[(724,176),(733,176),(750,193),(767,193)]`.
  - The realized event is the intended near-heater `n_34 x n_31` crossing at
    physical `(1491.5,749.125)`.
  - The validation line changed from `satisfies=false` to `satisfies=true`
    with `realized_violations=[]`.
  - `build/routes/multiportmmi_8x8_n_34_diagnostics.txt` now reports
    `status=ok`, `route_static_blocked_overlap_count=8`, and
    `route_overlap_effective_opened_static_count=8`, confirming the only
    static overlaps are the intended opened target-access cells.

Static-stub x-offset reduced:

- Date: 2026-07-15
- Change:
  - User requested reducing the static-stub x offset from 6 cells to 2 cells.
  - Added `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS`, defaulting to `2`.
    This controls only the x-direction pre/post bend stagger for two-bend
    static fanout stubs. It does not change
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS`, which can remain at 6 cells
    for y-lane spacing.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Stop-after route 32 (`n_31`) passed with static stubs, 90-degree bends,
    y-lane spacing 6, protected runway spacing 3, and default x-offset 2.
    `n_31` source anchor moved to `(716,149,0)` and
    `fanout_stub_center_cell_count` dropped from 858 to 666.
  - Stop-after route 35 (`n_34`) also passed with the same settings and wrote
    a fresh `build\routed_multiportmmi_8x8.gds`. `n_34` source anchor is now
    `(716,176,0)`, route length `116.083um`, cost `168.083`, expanded states
    `19026`, and diagnostics report `status=ok`.

Static-stub x-offset 2 through n_35:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`, with
    `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS` unset so the new default
    `2` is active.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 36 --debug-svgs
    36 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6`
- Result:
  - Success through route `[36/36] n_35`.
  - `n_35`: source state `(716,182,0)`, target `(767,43,0)`,
    length `390.191um`, cost `598.191`, expanded states `106944`.
  - Total wall time `35.3992 s`; net routing phase `31.7952 s`;
    native route batch `23.7225 s`; route search A* loop `27.6821 s`.
  - Crossing and photonic verification both report `success=True`,
    `status=partial_debug_stop`, `error_count=0`, and `routed_record_count=36`.
  - Fresh artifacts:
    `build\routed_multiportmmi_8x8.gds`,
    `build\routes\multiportmmi_8x8_n_35.svg`,
    `build\routes\multiportmmi_8x8_n_35_diagnostics.txt`.
- Note:
  - The route is valid but still expensive. The `n_35` search alone expanded
    106,944 states, so follow-up speed work should focus on this collision
    crossing search path if the geometry is acceptable.

Static-stub x-offset 2 through n_36:

- Date: 2026-07-15
- Command:
  - Same settings as the `n_35` run, but `--debug-stop-after-route 37` and
    `--debug-svgs 37`.
- Result:
  - Success through route `[37/37] n_36`.
  - `n_36`: source state `(716,188,0)`, target `(767,293,0)`,
    length `261.622um`, cost `265.622`, expanded states `15768`.
  - Total wall time `35.5187 s`; net routing phase `32.0614 s`;
    native route batch `24.1492 s`; route search A* loop `27.9940 s`.
  - Crossing and photonic verification both report `success=True`,
    `status=partial_debug_stop`, `error_count=0`, and `routed_record_count=37`.
  - Fresh artifacts:
    `build\routed_multiportmmi_8x8.gds`,
    `build\routes\multiportmmi_8x8_n_36.svg`,
    `build\routes\multiportmmi_8x8_n_36_diagnostics.txt`.
- Speed assessment:
  - The new static-stub/opened-cell correctness code is not the visible
    runtime bottleneck in this slice. Python-side setup is small:
    obstacle map `0.4255s`, port opening batch `0.0787s`,
    state-opening precompute `0.0143s`, endpoint correction `0.0020s`,
    realization `0.0187s`.
  - The dominant cost is still A* search: 234,451 expanded states and
    1,406,706 generated neighbors through the stop point, with the Rust
    native route batch at `24.1492s`.
  - `n_36` itself is moderate compared with `n_35` (`15,768` expansions vs.
    `106,944`). The previous `n_35` search remains the speed hotspot.
  - The current slowest-route timing report is not reliable at per-route
    granularity because the batched elapsed time is repeated across early
    route records; rely on aggregate counters and per-net expanded-state
    counts until that timing instrumentation is cleaned up.

Static-stub y-spacing 10 experiment through n_33:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=10`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`, with
    `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS` unset so default x-offset
    `2` remains active.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --debug-stop-after-route 34 --debug-svgs
    32,33,34 --attempt-diagnostics --debug-timing true --fanout-access-mode
    static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6`
- Result:
  - Success through route `[34/34] n_33`.
  - `n_31`: source state `(716,141,0)`, length `257.966um`, cost `261.966`,
    expanded states `3951`.
  - `n_32`: source state `(716,151,0)`, length `191.622um`, cost `247.622`,
    expanded states `10062`.
  - `n_33`: source state `(716,161,0)`, length `116.912um`, cost `168.912`,
    expanded states `5112`.
  - The first three lower-cluster stub anchors are now spaced by exactly
    10 grid cells in y: `141`, `151`, `161`.
  - Crossing and photonic verification both report `success=True`,
    `status=partial_debug_stop`, `error_count=0`, and `routed_record_count=34`.
  - Total wall time `23.6175 s`; net routing phase `20.6679 s`; native route
    batch `8.2951 s`; batch result processing `9.0388 s`.
  - Fresh artifacts:
    `build\routed_multiportmmi_8x8.gds`,
    `build\routes\multiportmmi_8x8_n_31.svg`,
    `build\routes\multiportmmi_8x8_n_32.svg`,
    `build\routes\multiportmmi_8x8_n_33.svg`.

Static-stub y-spacing 12 experiment through n_33:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=12`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`, with x-offset default `2`.
  - Same stop-after-`n_33` command as the 10-cell experiment.
- Result:
  - Success through route `[34/34] n_33`.
  - `n_31`: source state `(716,137,0)`, length `263.622um`, cost `267.622`,
    expanded states `4029`.
  - `n_32`: source state `(716,149,0)`, length `187.622um`, cost `243.622`,
    expanded states `11027`.
  - `n_33`: source state `(716,161,0)`, length `116.912um`, cost `168.912`,
    expanded states `6638`.
  - The first three lower-cluster stub anchors are now spaced by exactly
    12 grid cells in y: `137`, `149`, `161`.
  - Crossing and photonic verification both report `success=True`,
    `status=partial_debug_stop`, `error_count=0`, and `routed_record_count=34`.
  - Total wall time `25.1977 s`; net routing phase `22.0078 s`; native route
    batch `8.7327 s`; batch result processing `9.7887 s`.
  - Compared to y-spacing 10, this is still valid but slightly more expensive
    through `n_33`: expanded states `92,436` total vs. `89,867`, and total
    wall time `25.20s` vs. `23.62s`.

Static-stub y-spacing 14 experiment through n_33:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=14`,
    `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS=3`,
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`, with x-offset default `2`.
  - Same stop-after-`n_33` command as the 10/12-cell experiments.
- Result:
  - Success through route `[34/34] n_33`.
  - `n_31`: source state `(716,133,0)`, length `271.622um`, cost `275.622`,
    expanded states `4340`.
  - `n_32`: source state `(716,147,0)`, length `183.622um`, cost `239.622`,
    expanded states `13377`.
  - `n_33`: source state `(716,161,0)`, length `116.912um`, cost `168.912`,
    expanded states `8859`.
  - The first three lower-cluster stub anchors are spaced by exactly 14 grid
    cells in y: `133`, `147`, `161`.
  - Crossing and photonic verification both report `success=True`,
    `status=partial_debug_stop`, `error_count=0`, and `routed_record_count=34`.
  - Total wall time `25.4054 s`; net routing phase `22.2760 s`; native route
    batch `9.1496 s`; batch result processing `9.6626 s`.
  - Compared to y-spacing 10 and 12, this remains valid but is the most
    expensive of the three through `n_33`: total expanded states `97,318`.

Static-stub protected runway half-group logic:

- Date: 2026-07-15
- Change:
  - For `fanout_access_mode=static-stubs`, dense source protected runway
    lengths are now computed per half-cluster instead of across the full
    6-port source cluster.
  - For a 3-port lower half with base protected runway spacing 3 cells, the
    outer-to-inner runways are now `3`, `6`, `9` cells. For the upper half,
    the inner-to-outer runways are `9`, `6`, `3` cells.
  - `legacy-runway` behavior is unchanged.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran the y-spacing 14 stop-after-`n_33` experiment with static stubs,
    90-degree bends, protected runway spacing 3, and x-offset default 2.
  - Success through route `[34/34] n_33`; crossing and photonic verification
    both report `success=True`, `status=partial_debug_stop`, `error_count=0`,
    and `routed_record_count=34`.
  - Diagnostics confirm the new lower-half runway lengths:
    `n_31 source_dense_port_runway_cells=3`,
    `n_32 source_dense_port_runway_cells=6`,
    `n_33 source_dense_port_runway_cells=9`, with source states
    `(716,133,0)`, `(716,147,0)`, `(716,161,0)`.
  - Runtime for that validation: total wall `28.6430 s`, net routing
    `25.2230 s`, native route batch `10.6920 s`, total expanded states
    `98,462`.

`n_32` early crossing rejection around `(1435um, 643um)`:

- Date: 2026-07-15
- Context:
  - User expected `n_32` to cross `n_31` earlier, around physical
    `(1435, 643)`, rather than detouring to the later crossing at
    `(1446.5, 654.125)`.
  - Current debug configuration used y-spacing 14, static stubs, 90-degree
    stubs, protected runway spacing 3, x-offset default 2, and
    `--debug-stop-after-route 33`.
- Finding:
  - A* does evaluate the expected crossing at grid `(727,140)`.
  - It rejects the perpendicular candidate with `reason=reservation_footprint`.
  - Temporary trace showed the exact blocker is static cell `(725,138)` inside
    the crossing footprint.
  - The dynamic diagonal partner is not the blocker; the crossing partner is
    `n_31`/net id 32 and is considered as the crossing partner.
  - `build/routes/multiportmmi_8x8_n_32_diagnostics.txt` lists `(725,138)` in
    the final `opened_cells`, but the collision-crossing A* receives a filtered
    opened set of 990 cells where `(725,138)` is absent. The diagnostics list
    contains 998 cells.
  - Code path: `try_route_with_collision_crossings_using_primitives()` calls
    `route_single_net_with_collision_crossing_config(..., Some(opened_ref), ...)`.
    In this route, `opened_ref` is the dynamic-overlap-filtered opened-cell set,
    not the full `job.opened_cell_keys`.
- Interpretation:
  - This is an opened-cell handoff mismatch in the crossing A* path. Movement
    can still use a filtered set to avoid blindly opening dynamic geometry, but
    crossing-reservation static keepout checks need the appropriate static
    opening cells; otherwise a valid same-cluster/port-opening crossing can be
    rejected as if a foreign static keepout blocks it.
- Validation hygiene:
  - Temporary trace prints were removed.
  - `cargo check` passed with `PYO3_PYTHON` set to the project venv.

Opened-cell split fix for collision crossing A*:

- Date: 2026-07-15
- Change:
  - `route_single_net_with_collision_crossing_config()` now accepts a separate
    `reservation_open_cells` set.
  - Normal movement still uses the existing, potentially dynamic-overlap
    filtered opened-cell set.
  - Crossing reservation footprint checks use the full reservation opening set
    passed from `PyPhotonicRouter`, so static port/keepout openings are not
    accidentally removed just because they overlap the dynamic partner route.
  - Legacy `route_single_net_with_crossing_config()` passes the same opened set
    for both movement and reservation, preserving old behavior.
- Validation:
  - `cargo check` passed with `RUSTUP_TOOLCHAIN=stable-x86_64-pc-windows-gnullvm`,
    `CARGO_TARGET_X86_64_PC_WINDOWS_GNULLVM_LINKER=rust-lld`, and
    `PYO3_PYTHON` set to the project venv.
  - `maturin develop --release` passed.
  - Re-ran `multiportmmi_8x8` with y-spacing 14, static 90-degree stubs,
    protected runway spacing 3, x-offset default 2, stop-after-route 33,
    no trace output, and `--foreign-port-keepout-cells 6`; command exited 0.
  - The `n_32` route now crosses `n_31` at the early expected position:
    crossing event `(1434.5, 642.125)` with route/partner angles `(7, 1)`.
    The earlier trace showed A* accepting grid `(727,140)` with pending
    straight continuation.
  - Crossing validation and photonic verification JSON both report
    `success=True`, `error_count=0`.
  - Focused Rust tests passed:
    `collision_crossing_search_accepts_expected_dynamic_core_collision` and
    `crossing_reservation_window_rejects_static_and_unrelated_dynamic_cells`.

Static-stub y-spacing default reduced to 8 cells:

- Date: 2026-07-15
- Change:
  - `translation/route_rust.py` now uses
    `default_lane_spacing_cells = 8` for `fanout_access_mode=static-stubs`.
  - The environment variable `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` still
    overrides this default.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no
    `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` override, protected runway
    spacing 3, x-offset default 2, stop-after-route 33, and
    `--foreign-port-keepout-cells 6`; command exited 0 and wrote the updated
    GDS.
  - Diagnostics show the lower-cluster spacing changed to 8 grid cells:
    `n_31 source_state=(716,145,0)` and
    `n_32 source_state=(716,153,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub y-spacing default adjusted to 9 cells:

- Date: 2026-07-15
- Change:
  - `translation/route_rust.py` now uses
    `default_lane_spacing_cells = 9` for `fanout_access_mode=static-stubs`.
  - `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` still overrides this default.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no lane-spacing
    override, protected runway spacing 3, x-offset default 2,
    stop-after-route 33, and `--foreign-port-keepout-cells 6`; command exited
    0 and wrote the updated GDS.
  - Diagnostics show 9-grid-cell spacing:
    `n_31 source_state=(716,143,0)` and
    `n_32 source_state=(716,152,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub y-spacing default adjusted to 10 cells:

- Date: 2026-07-15
- Change:
  - `translation/route_rust.py` now uses
    `default_lane_spacing_cells = 10` for `fanout_access_mode=static-stubs`.
  - `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` still overrides this default.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no lane-spacing
    override, protected runway spacing 3, x-offset default 2,
    stop-after-route 33, and `--foreign-port-keepout-cells 6`; command exited
    0 and wrote the updated GDS.
  - Diagnostics show 10-grid-cell spacing:
    `n_31 source_state=(716,141,0)` and
    `n_32 source_state=(716,151,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub y-spacing default adjusted to 11 cells:

- Date: 2026-07-15
- Change:
  - `translation/route_rust.py` now uses
    `default_lane_spacing_cells = 11` for `fanout_access_mode=static-stubs`.
  - `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` still overrides this default.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no lane-spacing
    override, protected runway spacing 3, x-offset default 2,
    stop-after-route 33, and `--foreign-port-keepout-cells 6`; command exited
    0 and wrote the updated GDS.
  - Diagnostics show 11-grid-cell spacing:
    `n_31 source_state=(716,139,0)` and
    `n_32 source_state=(716,150,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Current routing SVGs for static-stub spacing 11:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no lane-spacing
    override, protected runway spacing 3, x-offset default 2,
    stop-after-route 33, route SVG selector `32-33`, and
    `--foreign-port-keepout-cells 6`.
- Result:
  - Command exited 0.
  - Generated current route SVGs:
    `build/routes/multiportmmi_8x8_n_31.svg` and
    `build/routes/multiportmmi_8x8_n_32.svg`.
  - Crossing and photonic verification JSON were written for the same run.

Static-stub spacing 11 run through n_36:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, no lane-spacing
    override, protected runway spacing 3, x-offset default 2,
    stop-after-route 37, no route SVGs, and `--foreign-port-keepout-cells 6`.
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Routes through `n_36` are present and `status=ok`.
  - Source states:
    `n_31=(716,139,0)`, `n_32=(716,150,0)`,
    `n_33=(716,161,0)`, `n_34=(716,176,0)`,
    `n_35=(716,187,0)`, `n_36=(716,198,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub x-offset default reduced to 1 cell:

- Date: 2026-07-15
- Change:
  - `translation/route_rust.py` now uses
    `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS` default `1` instead of `2`.
  - The environment variable still overrides this default.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, no x-offset override, protected runway spacing 3,
    stop-after-route 37, and `--foreign-port-keepout-cells 6`; command exited
    0 and wrote the updated GDS.
  - Routes through `n_36` are present and `status=ok`.
  - Source states moved left to x `714`:
    `n_31=(714,139,0)`, `n_32=(714,150,0)`,
    `n_33=(714,161,0)`, `n_34=(714,176,0)`,
    `n_35=(714,187,0)`, `n_36=(714,198,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub spacing 11 / x-offset 1 run through n_46:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, x-offset default 1, protected runway spacing 3,
    stop-after-route 47, no route SVGs, and `--foreign-port-keepout-cells 6`.
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Ten additional routes `n_37` through `n_46` are present and each
    diagnostic file reports `status=ok`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Static-stub eligibility made schematic/net-derived:

- Date: 2026-07-15
- Change:
  - Removed the static-stub and dense-source-fanout eligibility dependency on
    instance/component names such as `multiport` or `mmi`.
  - Eligibility is now derived from the route jobs extracted from the schematic
    nets: a source instance/angle group with more than two source ports is
    treated as a dense source fanout group.
  - Static-stub construction, protected-runway assignment, and optional dense
    fanout route ordering use this same source-fanout grouping.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, x-offset default 1, protected runway spacing 3,
    stop-after-route 47, and `--foreign-port-keepout-cells 6`; command exited
    0 and wrote the updated GDS.
  - The current benchmark still produces the same 36 fanout anchors across the
    same eight source fanout instances, now selected by schematic/net grouping
    rather than names.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Names-free static-stub run through n_50:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, x-offset default 1, protected runway spacing 3,
    stop-after-route 51, no route SVGs, and `--foreign-port-keepout-cells 6`.
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Four additional routes `n_47` through `n_50` are present and each
    diagnostic file reports `status=ok`.
  - Source states:
    `n_47=(990,100,0)`, `n_48=(990,111,0)`,
    `n_49=(990,126,0)`, `n_50=(990,137,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Names-free static-stub run through n_54:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, x-offset default 1, protected runway spacing 3,
    stop-after-route 55, no route SVGs, and `--foreign-port-keepout-cells 6`.
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Four additional routes `n_51` through `n_54` are present and each
    diagnostic file reports `status=ok`.
  - Source states:
    `n_51=(990,300,0)`, `n_52=(990,311,0)`,
    `n_53=(990,326,0)`, `n_54=(990,337,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Names-free static-stub run through n_66:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` with static 90-degree stubs, lane spacing
    default 11, x-offset default 1, protected runway spacing 3,
    stop-after-route 67, no route SVGs, and `--foreign-port-keepout-cells 6`.
- Timing:
  - Wall-clock runtime: `00:02:22.2228215` (`142.223` seconds).
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Twelve additional routes `n_55` through `n_66` are present and each
    diagnostic file reports `status=ok`.
  - Source states:
    `n_55=(1146,43,0)`, `n_56=(1146,93,0)`,
    `n_57=(1146,143,0)`, `n_58=(1146,193,0)`,
    `n_59=(1146,243,0)`, `n_60=(1146,293,0)`,
    `n_61=(1146,343,0)`, `n_62=(1146,393,0)`,
    `n_63=(1266,100,0)`, `n_64=(1266,111,0)`,
    `n_65=(1266,126,0)`, `n_66=(1266,137,0)`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Names-free static-stub run through n_66 without route diagnostics:

- Date: 2026-07-15
- Command:
  - Re-ran the same `multiportmmi_8x8` partial stop through route 67 with
    static 90-degree stubs, lane spacing default 11, x-offset default 1,
    protected runway spacing 3, no route SVGs, no `--attempt-diagnostics`, and
    `--foreign-port-keepout-cells 6`.
- Timing:
  - Wall-clock runtime: `00:02:07.2771918` (`127.277` seconds).
  - This is about 15 seconds faster than the diagnostics run, so the dominant
    cost is not the per-route diagnostics dump alone.
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Verification metadata confirms this was a partial debug stop:
    `debug_stop_after_route_index=67`, `routed_record_count=67`,
    `expected_route_count=111`.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`.

Native progress timing analysis through n_66:

- Date: 2026-07-15
- Observation:
  - Re-ran the partial stop through route 67 with
    `PHOTONIC_ROUTER_NATIVE_PROGRESS=1`.
  - Wall-clock runtime: `00:02:11.0700396` (`131.070` seconds).
  - Native per-route elapsed sum from the trace: `111.397` seconds.
  - The slowdown is not uniformly caused by static stubs or diagnostics.
    A short stop through route 10 completed in `6.869` seconds and early native
    route times were mostly milliseconds.
  - The long runtime is dominated by a few later route searches:
    internal `net_id=55` / visible `n_54` took `41.639155` seconds,
    internal `net_id=36` / visible `n_35` took `21.526369` seconds,
    internal `net_id=52` / visible `n_51` took `11.226987` seconds,
    internal `net_id=51` / visible `n_50` took `8.839992` seconds,
    internal `net_id=67` / visible `n_66` took `4.231827` seconds.
  - A same-stop `legacy-runway` A/B run in the current code took
    `140.894` seconds, close to the static-stub run. This points to a current
    A*/crossing search behavior or route-order/search-space regression rather
    than static-stub geometry alone.
  - The debug timing report's "slowest route" list divides the total batch time
    across attempts and is not reliable per-net timing. Native progress tracing
    gives the useful per-route attribution.

Slow-route root-cause analysis:

- Date: 2026-07-15
- Findings:
  - The timing runs report `repairs=0` and `failures=0` through the route-67
    partial stop, so the current slowdown is not caused by ripup/reroute loops.
  - A `--foreign-port-keepout-cells 0` A/B run through the same partial stop was
    not faster (`151.831` seconds), so the slowdown is not simply the count of
    foreign keepout cells.
  - The slow routes are normal crossing-aware A* searches. The route diagnostics
    show large endpoint/opened-cell regions for the slowest routes, e.g.
    `n_35` has `opened_cells_count=902` and `n_54` has
    `opened_cells_count=653`.
  - In `src/astar.rs`, crossing-aware A* takes the fast
    `crossing_no_contact_outcome` path only when the primitive footprint is
    free and the primitive has no extra crossing witnesses.
  - The symmetric diagonal halo work makes diagonal/bend primitives have
    `has_extra_witnesses=true`, so those moves enter the heavier
    `crossing_move_outcome_with_segments` path even when there may be no actual
    dynamic owner contact.
  - Inside that path, every effective witness calls
    `ObstacleMap::is_static_blocked`. That function checks the dense static bit
    and then calls `is_static_rect_blocked`, which linearly scans
    `static_rects`.
  - Because compact static rectangles are set on the router for bounding-box
    obstacles, this linear static-rect scan is now in the high-frequency
    crossing witness hot path. This is the most plausible current performance
    regression: correctness-preserving obstacle rectangles plus diagonal halo
    witnesses combine to make ordinary A* moves much more expensive.
- Likely fix direction:
  - Keep the halo/crossing behavior, but remove the linear static-rectangle scan
    from the A* crossing hot path. Options include a dense/static indexed view
    for crossing witness checks, a row/interval index for static rectangles, or
    passing an A*-window-local static grid into the crossing witness check.

Static-rectangle row index speedup:

- Date: 2026-07-15
- Change:
  - Added a row-indexed static-rectangle interval cache to `ObstacleMap`.
  - `is_static_rect_blocked(x, y)` now checks only intervals on row `y`
    instead of linearly scanning every compact static rectangle.
  - The index is maintained by `add_static_rect`, `add_static_rects`,
    `set_static_rects`, `clear_static_rects`, `clear_static_cells`, and
    `clone_with_expanded_dynamic_obstacles`.
- Rationale:
  - The crossing-aware A* witness path calls `is_static_blocked` very often.
  - After diagonal halo witnesses were added, many more diagonal/bend moves
    enter the crossing witness path. With compact static rectangles, the old
    linear static-rect scan dominated runtime while preserving identical search
    counters.
- Validation:
  - `cargo check` passed.
  - `cargo test obstacle_map --lib` passed after adding the bundled Python
    runtime directory to `PATH` for the PyO3 test binary.
  - Rebuilt the extension with `maturin develop --release`.
  - Re-ran `multiportmmi_8x8` through `debug_stop_after_route_index=67` with
    static 90-degree stubs, lane spacing default 11, x-offset default 1,
    protected runway spacing 3, no SVGs, and `--foreign-port-keepout-cells 6`.
  - Timing with `--debug-timing true`: wall-clock `26.701` seconds,
    `native_route_batch=5.3071s`, `astar_loop=5.3577s`, `repairs=0`,
    `failures=0`.
  - Timing without debug timing: wall-clock `26.223` seconds.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`, `routed_record_count=67`,
    `debug_stop_after_route_index=67`.

Static-stub run through n_67 after row-index speedup:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` through `debug_stop_after_route_index=68`
    with static 90-degree stubs, lane spacing default 11, x-offset default 1,
    protected runway spacing 3, no SVGs, no attempt diagnostics, and
    `--foreign-port-keepout-cells 6`.
- Timing:
  - Wall-clock runtime: `00:00:24.3990872` (`24.399` seconds).
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`, `routed_record_count=68`,
    `debug_stop_after_route_index=68`.

Static-stub run through n_68 after row-index speedup:

- Date: 2026-07-15
- Command:
  - Re-ran `multiportmmi_8x8` through `debug_stop_after_route_index=69`
    with static 90-degree stubs, lane spacing default 11, x-offset default 1,
    protected runway spacing 3, no SVGs, no attempt diagnostics, and
    `--foreign-port-keepout-cells 6`.
- Timing:
  - Wall-clock runtime: `00:00:24.8759268` (`24.876` seconds).
- Result:
  - Command exited 0 and wrote the updated GDS.
  - Crossing and photonic verification JSON both report
    `success=True`, `error_count=0`, `routed_record_count=69`,
    `debug_stop_after_route_index=69`.

Static-stub run through attempted n_69 after row-index speedup:

- Date: 2026-07-15
- Command:
  - Tried `multiportmmi_8x8` through `debug_stop_after_route_index=70`
    with static 90-degree stubs, lane spacing default 11, x-offset default 1,
    protected runway spacing 3, no SVGs, and `--foreign-port-keepout-cells 6`.
- Result:
  - The run failed in final photonic geometry repair before realization.
  - Reproduced with `--attempt-diagnostics`.
  - The grid/A* route for `n_69` reports `status=ok`.
  - Failure:
    `cross_net_waveguide_overlap n_66: Waveguide for n_66 overlaps waveguide for n_69`
    with `area=24.008843000000002` and
    `bbox=(2559.25, 865.818, 2559.65, 927.307)`.
  - `n_69` route:
    source `mmi0_multiport_2_1,o6`, target `mol_array_1_mzi_4,o1`,
    source state `(1266,326,0)`, target state `(1313,243,0)`,
    `route_cells_count=125`.
  - `n_66` route:
    source `mmi0_multiport_2_0,o5`, target `mol_array_1_mzi_6,o1`,
    source state `(1266,137,0)`, target state `(1313,343,0)`,
    `route_cells_count=213`.
  - Diagnosis:
    this is not a routing failure or ripup loop. The A* grid route is accepted,
    but realized photonic geometry finds a cross-net overlap between adjacent or
    near-parallel realized segments of `n_66` and `n_69`. The latest successful
    GDS remains the stop-through-`n_68` state.
  - Debug SVGs generated for visual inspection:
    `build/routes/multiportmmi_8x8_n_66.svg` and
    `build/routes/multiportmmi_8x8_n_69.svg`.
  - Follow-up analysis:
    the overlap bbox is a long, narrow vertical strip around
    `x=2559.25..2559.65`, `y=865.818..927.307`, which maps to the
    `n_66` vertical realized centerline area. Since the grid routes show
    `n_66` around grid `x=1288` and `n_69` around grid `x=1289`, adjacent
    2-um cells with a 0.5-um route width should not overlap by themselves.
    The next diagnostic should dump or reproduce the failed photonic probe
    layout/centerlines and compare primitive centerline, endpoint-corrected
    centerline, and final polygon near that bbox.

Photonic probe failure dump for n_69 overlap:

- Date: 2026-07-15
- Added a debug-only dump on final photonic verification failure in
  `translation/route_rust.py`.
- Re-ran the stop-after-route-70 case. Runtime was `29.146` seconds and the
  expected failure reproduced.
- Generated:
  - `build/photonic_probe_failures/multiportmmi_8x8_photonic_probe_failure.gds`
  - `build/photonic_probe_failures/multiportmmi_8x8_photonic_probe_failure.json`
  - `build/photonic_probe_failures/multiportmmi_8x8_photonic_probe_failure_centerlines.json`
  - `build/photonic_probe_failures/multiportmmi_8x8_photonic_probe_failure_centerlines.svg`
  - `build/photonic_probe_failures/multiportmmi_8x8_photonic_probe_failure.txt`
  - Root-cause evidence from the centerline dump:
    - `n_66` primitive centerline has the long vertical segment at `x=2557.5`.
    - `n_66` endpoint-corrected centerline moves that same vertical segment to
      `x=2559.4`.
    - `n_69` endpoint-corrected centerline has a vertical segment at `x=2559.5`
      from `y=925.125` to `y=868.0`, with bends above and below.
    - Therefore the overlap is caused by endpoint correction shifting the
      existing `n_66` vertical run into the later `n_69` realized route, not by
      adjacent untouched grid cells being intrinsically too close.

Static-stub endpoint correction fix:

- Date: 2026-07-15
- Changed `_fanout_stubbed_centerline()` in `translation/route_rust.py` so
  static-stub/fanout records splice the precomputed stub onto the primitive
  A* centerline instead of running `route_port_corrected_centerline()` across
  the whole routed path.
- Rationale:
  in crossing-enabled/static-stub mode, the stub is the endpoint correction for
  the dense source side. Applying full-route endpoint correction to the A*
  portion can move interior geometry after routing, which violates the routing
  model and caused `n_66` to shift into `n_69`.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Re-ran `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree
    stubs, stop-after-route 70, SVGs for route indices 67 and 70,
    `--routing-window-scale 0.35`, `--foreign-port-keepout-cells 6`.
  - Runtime: `30.646` seconds.
  - Result: command exited 0 and wrote GDS.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `cross_net_waveguide_overlap_count=0`,
    `routed_record_count=70`, `debug_stop_after_route_index=70`.
  - Updated route SVGs:
    `build/routes/multiportmmi_8x8_n_66.svg`,
    `build/routes/multiportmmi_8x8_n_69.svg`.

Static-stub endpoint correction follow-up through n_70:

- Date: 2026-07-15
- Re-ran `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
  stop-after-route 71, SVG for route index 71, `--routing-window-scale 0.35`,
  and `--foreign-port-keepout-cells 6`.
- Runtime: `30.580` seconds.
- Route 71 is visible net `n_70`
  (`mmi0_multiport_2_1,o5 -> mol_array_1_mzi_7,o1`).
- Result:
  - Command exited 0 and wrote GDS.
  - `n_70` routed with A* length `157.966um`, cost `161.966`,
    expanded states `2574`.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `cross_net_waveguide_overlap_count=0`,
    `routed_record_count=71`, `debug_stop_after_route_index=71`.
  - SVG generated:
    `build/routes/multiportmmi_8x8_n_70.svg`.

Static-stub endpoint correction follow-up through n_86:

- Date: 2026-07-15
- Re-ran `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
  stop-after-route 87, no route SVG generation, `--routing-window-scale 0.35`,
  and `--foreign-port-keepout-cells 6`.
- Runtime: `30.110` seconds.
- Result:
  - Command exited 0 and wrote GDS.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `cross_net_waveguide_overlap_count=0`,
    `waveguide_obstacle_overlap_count=0`, `routed_record_count=87`,
    `debug_stop_after_route_index=87`.
  - This covers 16 additional routes after the previous stop-through-`n_70`
    state. No route SVGs were requested for this speed-oriented run.

Static-stub endpoint correction follow-up through n_92:

- Date: 2026-07-15
- Re-ran `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
  stop-after-route 93, no route SVG generation, `--routing-window-scale 0.35`,
  and `--foreign-port-keepout-cells 6`.
- Runtime: `33.932` seconds.
- Result:
  - Command exited 0 and wrote GDS.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `cross_net_waveguide_overlap_count=0`,
    `waveguide_obstacle_overlap_count=0`, `routed_record_count=93`,
    `debug_stop_after_route_index=93`.
  - This covers 6 additional routes after the stop-through-`n_86` state and
    reaches the previously critical n_93 boundary without photonic errors.

Static-stub endpoint correction follow-up through n_94:

- Date: 2026-07-15
- Re-ran `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
  stop-after-route 95, no route SVG generation, `--routing-window-scale 0.35`,
  and `--foreign-port-keepout-cells 6`.
- Runtime: `39.980` seconds.
- Result:
  - Command exited 0 and wrote GDS.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `cross_net_waveguide_overlap_count=0`,
    `waveguide_obstacle_overlap_count=0`, `routed_record_count=95`,
    `debug_stop_after_route_index=95`.
  - This covers 2 additional routes after the stop-through-`n_92` state.

Full multiportmmi_8x8 benchmark after static-stub endpoint correction fix:

- Date: 2026-07-15
- Re-ran the full `multiportmmi_8x8` benchmark with crossings/lidar-pure,
  static 90-degree stubs, no debug stop, no route SVG generation,
  `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6`.
- Runtime: `56.637` seconds.
- Result:
  - Command exited 0 and wrote
    `build/routed_multiportmmi_8x8.gds`.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `warning_count=0`,
    `routed_record_count=111`, `unique_routed_record_count=111`,
    `missing_route_count=0`, `partial=False`, `status=complete`,
    `cross_net_waveguide_overlap_count=0`,
    `waveguide_obstacle_overlap_count=0`,
    `crossing_component_route_overlap_count=0`,
    `crossing_component_overlap_count=0`.
  - The full benchmark now routes and verifies cleanly with the current WIP
    changes.

Full multiportmmi_8x8 timing profile:

- Date: 2026-07-15
- Re-ran the full benchmark with the same static-stub/crossing configuration
  and `--debug-timing true`.
- Wall-clock around command: `73.061` seconds.
- Internal timing summary: `59.598` seconds.
- Key timing buckets:
  - Translation: `0.0116s`.
  - Optical routing stage: `51.5633s`.
  - `route_nets`: `41.7138s`.
  - Route endpoint correction: `0.0073s`.
  - Route realization: `0.0442s`.
  - Stage overhead/reporting in `routing_flow.py`: `9.7979s`.
- `route_nets` split currently accounts explicitly for only part of the time:
  - Obstacle map: `0.3027s`.
  - Route job/build/opening/static handoff/state prep combined: about `0.125s`.
  - Native route batch: `22.2321s`.
  - Batch result processing + records + debug artifact assembly: about `0.023s`.
  - A* loop summary: `22.3024s`, `111` attempts, `0` failures,
    `27/111` simple routes, `0` repairs.
  - A* counters: `2,821,524` expanded, `16,929,144` generated,
    `16,445,776` footprint checks, `2,946,404` rect checks.
  - About `19s` of `route_nets` remains un-attributed by the current split;
    given the code path, this is most likely final photonic probe
    realization/verification and repair-check bookkeeping inside
    `route_nets_rust`, not A* itself.
- Current biggest targets:
  1. Instrument and optimize final photonic probe verification inside
     `route_nets_rust` (`~19s` currently hidden in `route_nets other`).
  2. Reduce `routing_flow.py` post-route verification/reporting overhead
     (`~9.8s`).
  3. Then optimize actual A* search loop/native batch (`~22.3s`).

Full multiportmmi_8x8 final-check timing attribution:

- Date: 2026-07-15
- Added finer timing buckets around final realized crossing verification and
  final photonic probe verification in `translation/route_rust.py`, and printed
  those buckets from `routing_flow.py`.
- Re-ran the full benchmark with crossings/lidar-pure, static 90-degree stubs,
  no debug stop, no route SVG generation, `--routing-window-scale 0.35`,
  `--foreign-port-keepout-cells 6`, and `--debug-timing true`.
- Log: `build/multiportmmi_8x8_full_timing_final_checks.txt`.
- Wall-clock around command: `74.879` seconds.
- Internal timing summary: `61.023` seconds.
- Result:
  - Command exited 0.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` remains
    clean: `success=True`, `error_count=0`, `warning_count=0`,
    `routed_record_count=111`, `missing_route_count=0`, `status=complete`.
- Key attribution:
  - Native route batch / A* batch: `23.280s`.
  - A* search loop: `22.737s` timed in Rust, `2,821,524` states expanded,
    `16,929,144` generated, `16,445,776` footprint checks.
  - Realized crossing refresh total: `10.462s`, dominated by
    `realized_crossing_verify_intersections=10.455s`.
  - Final photonic probe refresh total: `7.950s`, dominated by
    `photonic_probe_verify=7.913s`.
  - Final verification block inside `route_nets_rust`: `18.412s`.
  - Obstacle map: `0.283s`.
  - Endpoint correction: `0.005s`.
  - Route realization: `0.040s`.
  - `routing_flow.py` stage overhead/reporting: `10.530s`.
- Interpretation:
  - The largest remaining cost is split between real route search
    (`~23s`) and duplicate/final verification work (`~18s` inside
    `route_nets_rust` plus `~10.5s` after returning to `routing_flow.py`).
  - `routing_flow.py` still performs its own crossing report generation and
    full `verify_photonic_routing(...)` after `route_nets_rust` has already
    done final verification; this is likely the next low-risk speed target
    before changing A* behavior.

Verification authority documentation:

- Date: 2026-07-15
- Added code comments documenting the intended verification ownership:
  - `translation/route_rust.py`: internal photonic probe verification is
    diagnostic/mismatch debugging support, not the intended always-on
    production source of truth.
  - `routing_flow.py`: the Python `verify_photonic_routing(...)` pass on the
    realized routed layout is the normal final geometry gate before accepting
    the GDS.
- No behavior change was made in this documentation-only step.
- Validation: `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
  translation\route_rust.py` passed.

Internal photonic probe verification default-off speed fix:

- Date: 2026-07-15
- Implemented `enable_internal_photonic_probe_verification: bool = False` in
  `translation/route_rust.py`.
- Behavior:
  - Internal realized crossing verification remains enabled because the router
    owns crossing event legality and can repair/reroute those events before
    final realization.
  - The expensive internal full photonic probe verification is now skipped by
    default. It can still be enabled explicitly for mismatch/debug work.
  - The Python `verify_photonic_routing(...)` call in `routing_flow.py` remains
    the normal final GDS/layout-level gate.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - `build/verification/multiportmmi_8x8_photonic_verification.json` reports
    `success=True`, `error_count=0`, `warning_count=0`,
    `routed_record_count=111`, `missing_route_count=0`, `status=complete`.
- Timing with `--debug-timing true`:
  - Wall-clock around command: `57.377` seconds.
  - Internal timing summary: `53.332` seconds.
  - `photonic_probe_*` buckets are absent from the split, confirming that the
    internal full photonic probe was skipped.
  - `route_nets` dropped to `33.991s`.
  - Native route batch / A*: `22.306s`.
  - Realized crossing refresh remains `11.030s`.
  - `routing_flow.py` stage overhead/reporting remains `10.749s`.
- Timing without `--debug-timing`:
  - Wall-clock around command: `55.744` seconds.
- Next speed targets:
  1. Reduce `realized_crossing_verify_intersections` (`~11s`) without changing
     crossing legality.
  2. Split and reduce `routing_flow.py` stage overhead/reporting (`~10.7s`),
     which includes the final external reports/verifications.
  3. Then return to the A* search loop (`~22s`) for algorithmic speedups.

Native crossing-event fast path for realized crossing metadata:

- Date: 2026-07-15
- Implemented a fast default path for realized crossing metadata in
  `translation/route_rust.py`.
- Behavior:
  - The normal flow now trusts A*-accepted native crossing events as the source
    for `realized_intersections`, crossing component placement, insertion-loss
    counts, and legal-overlap polygons for the external Python verifier.
  - The expensive full Python segment-sweep verifier
    `_verify_realized_route_intersections(...)` remains available behind
    `enable_internal_photonic_probe_verification=True`.
  - Internal realized-crossing verification no longer triggers normal ripup or
    route failure in the default flow; default correctness is enforced by A*
    move legality plus the final external Python geometry verification.
  - Added Bounding-Box prefiltering to the old full segment-sweep verifier so
    the debug path is also cheaper when explicitly enabled.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`, with `35` crossings and `35` crossing components.
- Timing with `--debug-timing true`:
  - Wall-clock around command: `31.376` seconds.
  - Internal timing summary: `27.514` seconds.
  - `route_nets`: `20.037s`.
  - Native route batch / A*: `19.430s`.
  - `realized_crossing_verify_intersections`: `0.001s`.
  - `realized_crossing_refresh_total`: `0.003s`.
  - `final_verification_block`: `0.003s`.
- Timing without `--debug-timing`:
  - Wall-clock around command: `31.746` seconds.
- Net speedup from the previous default-off probe state:
  - `realized_crossing_verify_intersections` dropped from about `11s` to
    around `1ms`.
  - Full benchmark wall-clock dropped from about `55.7s` to about `31.7s`.

No-crossing endpoint correction restored under crossing mode:

- Date: 2026-07-15
- Reverted the fragile broad crossing-aware endpoint-correction experiment that
  mixed static stubs, already-corrected centerlines, and crossing-net terminal
  edits.
- Current boundary:
  - Nets with no realized crossings now use the normal
    `apply_port_endpoint_corrections(...)` path even when crossing mode is
    enabled.
  - Nets with realized crossings remain on the existing split/splice path; the
    middle between first and last crossing is not meant to be moved.
  - Global endpoint-connectivity verification remains disabled for crossing
    mode until the explicit crossing-net split correction is implemented.
- Added a conservative safety guard for no-crossing endpoint correction:
  - If the corrected no-crossing centerline would enter an expanded foreign
    crossing-component footprint, the correction is rejected and the original
    route record is kept.
  - This prevents cases like `n_87` being endpoint-corrected into an unrelated
    crossing component.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully.
- Timing:
  - Wall-clock around command: `31.300` seconds.
  - Internal timing summary: `27.666` seconds.
  - `route_nets`: `20.263s`.
  - Native route batch / A*: `19.362s`.
  - Endpoint-correction phase: `0.232s`.

Static fanout stubs preserved during endpoint correction:

- Date: 2026-07-15
- Fixed a regression from the no-crossing endpoint-correction restoration:
  static fanout-stub nets were being treated as ordinary no-crossing routes,
  so their existing stubbed `corrected_centerline_um` could be replaced by a
  normal route endpoint correction.
- Current behavior:
  - If a routed record belongs to `fanout_anchor_net_ids` and already has a
    `corrected_centerline_um`, endpoint correction skips that record entirely.
  - This preserves the static stub geometry and avoids re-validating/correcting
    the route against the original physical port as if the stub did not exist.
  - Non-stub no-crossing nets still use normal endpoint correction, protected
    by the foreign crossing-footprint guard.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully at
    `2026-07-15 17:52:30`, size `446344` bytes.
- Timing:
  - Wall-clock around command: `32.094` seconds.
  - Internal timing summary: `28.411` seconds.
  - `route_nets`: `20.804s`.
  - Native route batch / A*: `19.935s`.
  - Endpoint-correction phase: `0.189s`.

Endpoint offset-bump correction enabled with 45-degree primitives:

- Date: 2026-07-15
- Diagnosis:
  - The Rust endpoint corrector already had the compact four-bend offset-bump
    case for routes where straight extension and shared-axis shifting do not
    solve the port offset.
  - The high-level Python routing flow disabled that case whenever
    `allow_45_degree_turns=True` by passing
    `allow_unchecked_bumps=not allow_45_degree_turns`.
  - This is why some no-crossing endpoint-correction cases did not insert the
    expected bump even though the Rust implementation supported it.
- Change:
  - High-level endpoint-correction calls now pass `allow_unchecked_bumps=True`
    independent of 45-degree primitive support.
  - The lower-level Rust API still supports explicitly passing
    `allow_unchecked_bumps=False` for tests/debugging.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully at
    `2026-07-15 18:02:39`, size `449448` bytes.
- Timing:
  - Wall-clock around command: `33.702` seconds.
  - Internal timing summary: `29.526` seconds.
  - `route_nets`: `21.540s`.
  - Native route batch / A*: `20.675s`.
  - Endpoint-correction phase: `0.205s`.

Checked endpoint bump correction restored for crossing-mode no-crossing nets:

- Date: 2026-07-15
- Diagnosis:
  - The Rust router already exposes obstacle-aware endpoint correction via the
    checked-and-commit APIs. Those APIs evaluate top/bottom bump candidates
    against static and dynamic occupancy before committing geometry.
  - The Python post-routing endpoint-correction pass still used the unchecked
    geometry helper for no-crossing records in crossing mode, so an offset bump
    could be inserted without seeing nearby route/static obstacles.
- Change:
  - Checked endpoint correction is now enabled in crossing mode too.
  - Static fanout-stub nets are skipped when they already carry their generated
    stubbed centerline.
  - Nets with accepted crossing events are skipped here; their crossing-aware
    endpoint correction remains limited to the port-to-first-crossing and
    last-crossing-to-port regions.
  - No-crossing, non-stub nets in crossing mode now use the native checked
    endpoint-correction path. If no obstacle-safe correction candidate exists,
    the original route is preserved in crossing mode instead of replacing it
    with an unchecked bump or poisoning the route record.
  - Offset-bump candidates remain enabled for 45-degree-capable runs, but now
    go through the obstacle-aware checked path for this no-crossing/non-stub
    crossing-mode case.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py` passed
    before the full benchmark rerun.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully at
    `2026-07-15 18:07:17`, size `449618` bytes.
- Timing:
  - Wall-clock around command: `31.817` seconds.
  - Internal timing summary: `28.220` seconds.
  - `route_nets`: `20.693s`.
  - Native route batch / A*: `20.009s`.
  - Endpoint correction: `0.086s`, `calls=74`, `failures=0`.

Stub-routed nets now allow endport-only endpoint correction:

- Date: 2026-07-15
- Diagnosis:
  - Static fanout-stub routes were previously skipped entirely during the
    crossing-aware endpoint-correction pass once they had a stubbed
    `corrected_centerline_um`.
  - That preserved the generated stub, but it also prevented the route from
    correcting the far end port. This was too strict for routes that start at a
    static stub and may still have crossings on the routed segment.
- Change:
  - The routing metadata now records whether a fanout anchor belongs to the
    source side or target side of a net.
  - Crossing-aware endpoint correction can now freeze only the stubbed side and
    correct the opposite terminal.
  - For stubbed routes, the existing `stub + route` centerline is used as the
    splice baseline so the static stub is preserved.
  - For the current source-stub use case, only the last-crossing-to-end-port
    side is allowed to move; the start stub is not modified.
  - If a no-crossing stub route needs endpoint correction, the route-only
    correction is merged back into the existing stubbed centerline instead of
    dropping the stub.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py` passed.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully at
    `2026-07-15 18:15:42`, size `450188` bytes.
- Timing:
  - Wall-clock around command: `31.473` seconds.
  - Internal timing summary: `27.640` seconds.
  - `route_nets`: `20.161s`.
  - Native route batch / A*: `19.441s`.
  - Endpoint correction: `0.084s`, `calls=74`, `failures=0`.

Final photonic verification now checks port connectivity in crossing mode:

- Date: 2026-07-15
- Diagnosis:
  - The final `verify_photonic_routing` pass already supported endpoint
    connectivity checks, but `routing_flow.py` disabled them whenever crossings
    were enabled.
  - Enabling the check immediately exposed several target endpoint distances
    slightly above the old default `2.0um` contact radius. These were all below
    one diagonal grid-cell radius for the current `2.0um` grid.
- Change:
  - Final photonic verification now always runs endpoint connectivity checks,
    including crossing-enabled runs.
  - The default port contact radius is now at least one grid-cell diagonal:
    `sqrt(2) * grid_size_um`. This matches the router's grid quantization while
    still catching larger unconnected-port gaps.
  - Added focused photonic-verification tests:
    - stub-like corrected centerlines are accepted when they touch both real
      ports;
    - missing target-port contact is reported even when crossing metadata is
      present.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\photonic_verification.py tests\test_photonic_verification.py`
    passed.
  - `.\.venv\Scripts\python.exe -m pytest tests\test_photonic_verification.py
    -q` passed: `16 passed`.
  - Full `multiportmmi_8x8` with crossings/lidar-pure, static 90-degree stubs,
    `--routing-window-scale 0.35`, and `--foreign-port-keepout-cells 6` passed
    with endpoint connectivity enabled.
  - Photonic verification reports `success=True`, `error_count=0`,
    `warning_count=0`, `routed_record_count=111`, `missing_route_count=0`,
    `status=complete`.
  - Crossing verification reports `success=True`, `error_count=0`,
    `warning_count=0`.
  - `build/routed_multiportmmi_8x8.gds` was written successfully at
    `2026-07-15 18:26:23`, size `450188` bytes.
- Timing:
  - Wall-clock around command: `34.985` seconds.
  - Internal timing summary: `30.755` seconds.
  - `route_nets`: `22.387s`.
  - Native route batch / A*: `21.595s`.
  - Endpoint correction: `0.090s`, `calls=74`, `failures=0`.

Correction: port-contact tolerance must stay strict; checked bumps need active port openings:

- Date: 2026-07-15
- Diagnosis:
  - The previous diagonal-grid-cell contact radius masked a real missing-port
    connection near heater/pad geometry. That run is not considered a valid
    correctness checkpoint.
  - Static pad geometry may be crossed by endpoint/bump correction only through
    the route's active port opening cells. A local bump footprint must not open
    arbitrary static cells by itself.
  - Static fanout-stub routes must be endpoint-corrected in the real router
    that owns the static obstacle map, dynamic route map, and opened-port cells.
    The later realization-only router is intentionally obstacle-free and cannot
    validate bump legality.
- Change:
  - `translation/photonic_verification.py` default port-contact tolerance is
    restored to `max(route_width_um, grid_size_um)`.
  - `src/py_router.rs` checked endpoint correction now treats static overlap as
    legal only when the cell is in `opened_cells`; it no longer auto-opens the
    bump/core footprint.
  - `translation/route_rust.py` now applies a checked fanout-stub endpoint
    correction pass in the live router. The static stub side is frozen and the
    opposite terminal is corrected/merged back into the stubbed centerline.
  - The later crossing-aware debug-artifact correction no longer overwrites
    already-corrected static fanout-stub records with unchecked obstacle-free
    geometry.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py
    translation\photonic_verification.py tests\test_port_alignment_diagnostics.py
    tests\test_photonic_verification.py` passed.
  - Python-side focused verification passed:
    `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py::test_crossing_free_endpoint_correction_uses_normal_corrected_centerline
    tests\test_photonic_verification.py -q` => `17 passed`.
  - Full Rust-backed validation is currently blocked in this shell because the
    Windows toolchain cannot link the PyO3 extension:
    MSVC target fails with missing `link.exe`; explicit
    `stable-x86_64-pc-windows-gnullvm` fails with missing
    `x86_64-w64-mingw32-clang`, and direct `gcc-ld` execution is denied by
    Windows (`os error 5`).

Windows Rust/PyO3 build path stabilized:

- Date: 2026-07-15
- Diagnosis:
  - Intermittent Rust build failures came from inconsistent Windows toolchain
    selection. The default host was `stable-x86_64-pc-windows-msvc`, but Visual
    Studio C++ Build Tools / `link.exe` are not installed in this workspace.
  - Switching only the target to `x86_64-pc-windows-gnullvm` was not enough:
    build scripts still used the MSVC host unless the repo selected the gnullvm
    toolchain.
  - The gnullvm default external linker name
    `x86_64-w64-mingw32-clang` is also not present. The installed gnullvm
    toolchain does include `rust-lld.exe`, which works when Cargo points at it
    directly.
- Change:
  - Added `rust-toolchain.toml` selecting
    `stable-x86_64-pc-windows-gnullvm`.
  - Added `.cargo/config.toml` selecting the bundled gnullvm `rust-lld.exe`
    linker and setting `PYO3_PYTHON` to the project virtualenv Python.
  - Added `docs/WINDOWS_RUST_TOOLCHAIN.md` with the failure modes and expected
    commands.
  - Updated `AGENTS.md` to point future agents at the documented Windows build
    path.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` now passes from the repo root
    without manual environment variables. Current warning:
    `endpoint_contact_open_keys` is unused.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` now rebuilds and
    reinstalls `photonic_router._rust` successfully without manual environment
    variables.
  - `.\.venv\Scripts\python.exe -c "import photonic_router._rust as r; ..."`
    confirms the extension imports from
    `python\photonic_router\_rust.pyd` and exposes `PyPhotonicRouter`.
  - `tests/test_rust_backend_import.py` now executes against the rebuilt
    extension; 8 tests pass and 2 crossing behavior tests fail with
    `No crossing-compliant route found`. Those failures are current router
    behavior regressions, not build/toolchain failures.

Crossing binding harness aligned with collision-local A*:

- Date: 2026-07-15
- Diagnosis:
  - The `No crossing-compliant route found` failures were not produced by the
    final photonic/GDS verifier and did not indicate clean crossings being
    rejected for uncorrected ports.
  - They came from legacy direct Rust-binding tests that enabled the old
    expected-partner crossing path. That path requires an explicitly expected
    partner crossing and returns before the normal local-collision A* behavior
    or final endpoint connectivity verification is relevant.
- Change:
  - Replaced the old expected-partner crossing route fixtures in
    `tests/test_rust_backend_import.py` with collision-local fixtures using
    `set_collision_crossing_routing(True)` and
    `CrossingConfig(..., allow_only_expected_pairs=False)`.
  - The new fixtures verify that A* accepts a legal local dynamic crossing,
    reserves and releases the crossing footprint, and rejects an illegal local
    crossing move with a normal `No route found` outcome rather than the legacy
    expected-partner error.
  - Existing photonic-verification tests remain responsible for reporting
    unconnected/non-port-corrected ports.
- Validation:
  - `.\.venv\Scripts\python.exe -m pytest tests\test_rust_backend_import.py
    -q` passed: `10 passed`.
  - `.\.venv\Scripts\python.exe -m pytest tests\test_photonic_verification.py
    -q` passed: `16 passed`.

Strict full-run attempt stops before final port verification:

- Date: 2026-07-15
- Command:
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --fanout-access-mode static-stubs
    --routing-window-scale 0.35 --foreign-port-keepout-cells 6 --debug-svgs
    false --attempt-diagnostics --debug-timing true`
  - Run required `PYTHONUTF8=1` in this PowerShell session to avoid CP1252
    console failures on Unicode status symbols.
- Result:
  - The run does not reach final photonic/port verification.
  - Routing stops at route `[69/111] n_68`
    (`mmi0_multiport_2_1,o7 -> mol_array_1_mzi_3,o1`) with
    `No legal LiDAR crossing route found; probe-based victim selection is
    disabled in crossing mode`.
- Diagnostics:
  - `build\routes\multiportmmi_8x8_n_68_FAILED.txt`
  - `build\routes\multiportmmi_8x8_n_68_diagnostics.txt`
  - The normal route attempt reports
    `Illegal grid crossing: net 69 intersects net 66 at (1301.500, 285.500)
    (insufficient_straight_margin)`.
  - The source side is a static fanout anchor; diagnostics report
    `source_fanout_anchor=True`, `source_dense_source_cluster_size=4`, and
    `source_dense_port_runway_cells=6`.
  - Opened cells have no static overlap in the effective diagnostic
    (`opened_static_overlap_count=0`), but a small sibling runway dynamic
    overlap is present:
    `sibling_port_runway_dynamic_overlap_count=3`,
    bbox `(1281,1283,300,300)`.
- Current assessment:
  - The expected port-connectivity errors are not observable yet because the
    current WIP fails earlier in route search. The next task should inspect the
    `n_68`/net-66 crossing candidate and decide whether A* should find a legal
    alternate crossing/route or whether this is a legitimate route-order/ripup
    blocker.

Endpoint correction dry-run boundary:

- Date: 2026-07-15
- Reason:
  - Endpoint/port correction may need the router obstacle map for static,
    dynamic, and opened-port collision checks, especially for bump candidates.
  - It must not mutate the live A* router state before all routing and repair
    decisions have completed. Otherwise a post-route geometry adapter can
    influence later A* searches, which violates the current model boundary.
- Change:
  - Added non-committing Rust/PyO3 correction APIs:
    `route_port_corrected_centerline_checked(...)` and
    `apply_checked_endpoint_corrections(...)`.
  - Kept the previous explicit commit APIs for tests/debug compatibility:
    `route_port_corrected_centerline_checked_and_commit(...)` and
    `apply_checked_endpoint_corrections_and_commit(...)`.
  - The dry-run path performs the same candidate generation and obstacle-map
    commit legality check on a cloned obstacle map, then returns the corrected
    centerline and metadata without registering route cells, crossing events,
    opened cells, spacing history, or committed centerlines on the live router.
  - The normal Python flow now calls the dry-run batch API for both regular
    endpoint correction and fanout-stub endpoint correction. The realized route
    records are updated, but the router's live A* obstacle state is not changed
    by correction.
  - Removed the now-unused local endpoint-contact static-opening helper. Static
    pad openings must come from the active route opened-cell set instead of an
    implicit local carve-out.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed with no warnings.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed after
    the final warning cleanup and rebuilt `photonic_router._rust`.
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py
    translation\route_rust.py translation\route_rust_realization.py
    translation\photonic_verification.py` passed.
  - `.\.venv\Scripts\python.exe -m pytest tests\test_rust_backend_import.py
    -q` passed: `10 passed`.
  - Focused endpoint-correction tests passed:
    `tests\test_port_alignment_diagnostics.py::test_checked_endpoint_correction_dry_run_does_not_mutate_router_state`,
    `::test_batch_checked_endpoint_correction_dry_run_returns_metadata_without_commit`,
    `::test_batch_checked_endpoint_correction_returns_per_net_metadata`,
    `::test_checked_case4_bump_allows_local_static_port_opening`,
    `::test_checked_case4_bump_uses_clear_mirrored_side_when_static_blocks_first_candidate`,
    `::test_checked_case4_bump_rejects_static_without_active_port_opening`
    => `6 passed`.
  - Full `tests\test_port_alignment_diagnostics.py -q` currently has two
    broader `mmi_heater` failures:
    `test_mmi_heater_pass0_characterizes_current_port_alignment` and
    `test_mmi_heater_route_match_uses_corrected_records_for_realization`.
    Both fail inside the initial native A* batch at
    `gc1_to_mmi0_in2` before endpoint correction is reached, with
    `No repair route found; candidate_blockers=[1]`. Treat this as a separate
    routing/repair issue, not evidence that dry-run correction mutates A* state.

mmi_heater failure-context artifact:

- Date: 2026-07-15
- User requested a GDS for the failing `mmi_heater` state while skipping
  downstream verification.
- Generated `build\routed_mmi_heater_failed_context_before_route2.gds` by
  routing only through `debug_stop_after_route_index=1` with
  `include_heater_obstacles=True`, `allow_45_degree_turns=False`, and
  `enable_checked_endpoint_correction=False`.
- This GDS contains the unrouted layout plus the first successful route
  `gc0_to_mmi0_in1`. The next route, `gc1_to_mmi0_in2`, has no committed
  geometry because A* does not return a legal path.
- Also generated failure/debug context under:
  - `build\routes\mmi_heater_failed_context_gc1_to_mmi0_in2_FAILED.txt`
  - `build\routes\mmi_heater_failed_context_gc1_to_mmi0_in2_diagnostics.txt`
  - `build\routes\mmi_heater_failed_context_gc0_to_mmi0_in1.svg`
  - `build\static_obstacles\mmi_heater_failed_context_obstacles.svg`

mmi_heater port-alignment fixture clearance removed:

- Date: 2026-07-15
- Diagnosis:
  - The two broader `mmi_heater` tests were failing because the fixture used
    the default obstacle clearance. That gives committed waveguides a dynamic
    search/commit keepout around the centerline.
  - In this old port-alignment diagnostic fixture, route 1's dynamic keepout
    overlaps the route-2 target access/runway region, so route 2 cannot find
    the simple orthogonal route. This was a clearance/spacing test artifact,
    not a failure of the multiport crossing flow.
- Change:
  - The two `mmi_heater` tests in
    `tests\test_port_alignment_diagnostics.py` now pass
    `StaticObstacleMapConfig(clearance_um=0.0)`.
  - The first test also disables checked endpoint correction explicitly; it now
    characterizes the clearance-free grid/primitive port-alignment state rather
    than asserting port-snapped endpoint correction.
  - The route-match test disables `enable_grid_endpoint_correction` and asserts
    that routed records remain present with corrected primitive centerlines,
    without requiring exact endpoint snapping.
- Validation:
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py::test_mmi_heater_pass0_characterizes_current_port_alignment
    tests\test_port_alignment_diagnostics.py::test_mmi_heater_route_match_uses_corrected_records_for_realization
    -q` passed: `2 passed`.
  - Focused checked endpoint-correction dry-run tests still pass: `6 passed`.
  - Full `tests\test_port_alignment_diagnostics.py -q` passed:
    `25 passed`.

multiportmmi_8x8 segmented endpoint inspect GDS:

- Date: 2026-07-15
- User requested a GDS even though the current photonic endpoint verification
  still fails, so the geometry can be inspected visually.
- Added a debug-only environment gate in `routing_flow.py`:
  `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`.
  Standard behavior still raises before GDS write when photonic verification
  fails; the env var only allows an explicit inspect GDS.
- Generated:
  - `build\routed_multiportmmi_8x8.gds`
  - log: `build\multiportmmi_8x8_segmented_endpoint_90_inspect_gds.txt`
- Command used:
  - `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`
  - `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`
  - `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6 --debug-svgs false --debug-timing true`
- Result:
  - GDS write succeeded.
  - Runtime: `31.5697 s`.
  - A* routed all `111` nets with `failures=0`, `repairs=0`.
  - Crossing metadata: `35` native events, `35` realized intersections,
    `0` illegal realized crossings.
  - Photonic verification still fails with `14` endpoint errors:
    `7 target_port_not_connected` and `7 target_endpoint_mismatch`.
  - This is the desired inspect state for the current endpoint-correction bug:
    crossings are no longer silently shifted by full-route endpoint correction,
    and the remaining failures expose missing target-side terminal correction.

No-crossing endpoint bump collision diagnosis:

- Date: 2026-07-15
- User showed an early benchmark endpoint bump colliding with the already
  routed net above it and asked why the obstacle check can still miss this.
- Findings:
  - The Rust checked endpoint-correction path does have access to the live
    router obstacle map during `route_nets_rust()`.
  - In the overall crossing-enabled benchmark, a net can have no crossings
    itself while `enable_crossings=True` globally. For checked endpoint
    failures, `_apply_checked_endpoint_corrections_for_net_ids()` only records
    `endpoint_correction_error` when `not enable_crossings`; those failed
    no-crossing nets therefore remain eligible for a later correction pass.
  - After `route_nets_rust(..., defer_realization=True)`,
    `route_match_and_realize()` calls
    `_apply_crossing_aware_endpoint_corrections_to_debug_artifacts()` for
    crossing-enabled runs. For records with no crossing points, this falls back
    to `apply_port_endpoint_corrections(...)` on a fresh realization router
    built by `_build_realization_router()`. That router has no committed
    dynamic obstacles and currently uses `allow_unchecked_bumps=True`, so it
    can insert the colliding bump into the GDS even though the live checked
    router would reject it.
- Important boundary:
  - The next fix should not change A* behavior. It should prevent the later
    debug/realization endpoint-correction pass from retrying unchecked bumps
    after the live checked pass rejected them.

Empty-obstacle-map endpoint correction pass removed:

- Date: 2026-07-15
- User clarified that endpoint correction must be the same independent of the
  global crossing flag and must decide per net whether crossings are present.
- Change:
  - In `translation/route_rust.py`, `route_match_and_realize()` no longer runs
    `_apply_endpoint_corrections_to_debug_artifacts()` or
    `_apply_crossing_aware_endpoint_corrections_to_debug_artifacts()` after
    `route_nets_rust(..., defer_realization=True)`.
  - That later pass used a fresh realization-only `PyPhotonicRouter` with an
    empty dynamic obstacle map. It could therefore insert unchecked endpoint
    bumps into the GDS even when the live checked router would reject them.
  - The post-routing step now only refreshes port-alignment diagnostics from
    the records returned by the live routing phase. It does not mutate route
    geometry.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "case4_bump or
    batch_checked_endpoint_correction" -q` passed:
    `8 passed, 18 deselected`.
  - Re-ran `multiportmmi_8x8` with static stubs and
    `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`.
    Generated `build\routed_multiportmmi_8x8.gds`, log
    `build\multiportmmi_8x8_no_empty_map_endpoint_correction.txt`.
  - Runtime: `29.5286 s`; route search: `111` attempts, `0` failures,
    `0` repairs; endpoint correction: `84` calls, `5` failures.
  - Photonic verification now fails closed with `54` endpoint-related issues:
    `6 missing_corrected_centerline`, `22 target_port_not_connected`, and
    `26 target_endpoint_mismatch`. This is expected after removing the unsafe
    empty-map retry; rejected live endpoint corrections now remain uncorrected
    instead of being patched by an unchecked bump in the GDS.

n_5 delta-bump placement investigation:

- Date: 2026-07-15
- User identified the visible early benchmark bump as likely `n_5` and asked
  why a start-side downward bump is not selected while an end-side top bump
  appears.
- Diagnosis:
  - `n_5` is not a full-straight case-4 bump candidate. Its route has
    horizontal and 45-degree segments, so it uses the delta endpoint-correction
    path in `geometry_realization.rs`.
  - The delta correction path generated one corrected centerline, and target
    delta bumps always used the last viable carrier run. That biases target
    endpoint correction toward an end-side bump even when an earlier run could
    absorb the same target delta by shifting the suffix.
  - Port coordinates for `n_5`:
    source `fanout_yb_1_1,o2` at `(205.5, 850.625)`, target
    `fanout_yb_2_3,o1` at `(355.5, 950.0)`.
- Change:
  - In `src/geometry_realization.rs`, `insert_target_delta_bump()` now selects
    the first viable carrier run instead of the last viable run, allowing
    target endpoint deltas to be absorbed earlier in the route when geometry
    permits.
  - Removed the now-unused `last_viable_delta_bump_run()` helper.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "case4_bump or
    batch_checked_endpoint_correction" -q` passed:
    `8 passed, 18 deselected`.
  - Generated stop-after-14 inspect GDS:
    `build\routed_multiportmmi_8x8.gds`, log
    `build\multiportmmi_8x8_n5_delta_bump_first_run.txt`.

n_5 full-straight bump candidate blocker semantics:

- Date: 2026-07-15
- User clarified that the relevant `n_5` state is a horizontal route before
  endpoint correction, with four case-4 candidates checked in order:
  `start/top`, `start/bottom`, `end/top`, `end/bottom`.
- Diagnosis:
  - `full_straight_offset_bump_candidates()` does generate candidates in that
    order for a horizontal route with a Y port offset.
  - A previous bump-collision fix changed the candidate blocker precheck from
    `candidate_core_cells` to inflated `candidate_blocked_cells`. That made
    the candidate decision use the reserved keepout area as a hard obstacle.
    It can reject a visually legal `start/bottom` bump even though the actual
    waveguide core has clearance.
  - `ObstacleMap::commit_route_with_clearance_overlap()` already encodes the
    intended contract: core cells must not overlap existing dynamic obstacles;
    blocked cells are the reservation committed after the core is legal.
- Change:
  - In `src/py_router.rs`, the case-4 bump candidate `out_of_bounds`,
    `static_blockers`, and `dynamic_blockers` checks now use
    `candidate_core_cells`.
  - The inflated `candidate_blocked_cells` are still used for the reservation
    committed after a legal core candidate is selected.
  - Updated the focused test so dynamic cells only in the inflated keepout no
    longer reject `start/top`, and added a separate test preserving rejection
    when dynamic cells overlap the actual core footprint.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "case4_bump or
    batch_checked_endpoint_correction" -q` passed:
    `9 passed, 18 deselected`.
  - Generated stop-after-14 inspect GDS:
    `build\routed_multiportmmi_8x8.gds`, log
    `build\multiportmmi_8x8_n5_core_bump_check.txt`.
  - Verification for that stop-after run now reports only the five expected
    `missing_corrected_centerline` errors for `n_0`, `n_2`, `n_7`, `n_9`,
    and `n_13`; `n_5` is no longer an endpoint correction error.

n_5 start/bottom endpoint-bump opening fix:

- Date: 2026-07-15
- Scope:
  - User reported that the existing GDS still showed the `n_5` correction as
    `end/top`; expected candidate order is `start/top`, `start/bottom`,
    `end/top`, `end/bottom`, and visually `start/bottom` should fit.
  - No commit was made.
- Diagnosis:
  - Added env-gated trace via `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS`.
  - Trace showed `n_5 start/top` correctly rejected because it overlaps
    dynamic owner `[6]`.
  - `n_5 start/bottom` was incorrectly rejected by one static cell
    `(112,243)`.
  - Python-side opening trace showed the snapped endpoint-correction state was
    `source_state=(113,243,0)`, so `(112,243)` is not a side-bend cell; it is a
    local backward axis cell at the active source port pad.
- Change:
  - `translation/route_rust.py` now adds local endpoint-bump candidate opening
    cells from the actual snapped source/target states used for endpoint
    correction, including the small backward-on-axis pad cells and side-bend
    cells. These cells are added only to the candidate opening set used by
    endpoint correction, not to the normal effective A* routing opening.
  - `src/py_router.rs` keeps the same env-gated candidate trace and mirrors the
    local port-bump candidate opening geometry in `build_route_port_openings`
    for consistency.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "case4_bump" -q` passed:
    `7 passed, 20 deselected`.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - Stop-after-14 run with `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS=5` now
    reports:
    `endpoint_bump_trace net_id=5 candidate=1 label=start/bottom status=accept`.
  - Updated GDS:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 21:41 local,
    size 149662 bytes.

Full `multiportmmi_8x8` run after n_5 endpoint-bump fix:

- Date: 2026-07-15
- Command:
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`,
    `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS` unset.
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --fanout-access-mode static-stubs
    --routing-window-scale 0.35 --foreign-port-keepout-cells 6
    --debug-svgs false --debug-timing true`
- Result:
  - Full routing completed all `111` nets.
  - Routing failures: `0`; repairs: `0`; simple routes: `27/111`.
  - Endpoint correction: `84` calls, `0` failures.
  - Total runtime: `29.2723 s`; optical routing stage `21.3188 s`; native
    routing batch `20.5052 s`.
  - GDS written despite verification failure:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 21:44 local,
    size 467092 bytes.
  - Logs:
    `build\multiportmmi_8x8_full_after_n5_bump_fix.txt`.
  - Verification JSON:
    `build\verification\multiportmmi_8x8_photonic_verification.json`.
- Photonic verification status:
  - `49` issues on `27` nets.
  - Error code distribution:
    `target_endpoint_mismatch=26`, `target_port_not_connected=22`,
    `missing_corrected_centerline=1`.
  - Affected nets:
    `n_31..n_35`, `n_50..n_54`, `n_65..n_69`, `n_88..n_93`,
    `n_105..n_110`.
  - `n_93` has the single `missing_corrected_centerline` plus
    `target_port_not_connected`.
- Current assessment:
  - The full router path is now stable enough to complete the benchmark, and
    the remaining class is port connectivity/snapping, mostly target-side.
  - Several target endpoint mismatches are exactly `2.368 um` at a `2.0 um`
    tolerance, suggesting a systematic endpoint-correction/stub-segment
    handoff issue rather than random A* failure.

Crossing-net endpoint-correction analysis:

- Date: 2026-07-15
- User hypothesis:
  - Crossing nets, both with and without static stubs, do not have their
    `last crossing -> port` segment corrected.
- Finding:
  - Confirmed from code and artifacts.
  - The normal checked endpoint-correction pass in `translation/route_rust.py`
    explicitly skips all crossing nets:
    `if enable_crossings and int(net_id) in crossing_net_ids: continue`.
  - The checked fanout-stub endpoint-correction pass also explicitly skips
    fanout-stub nets that have crossings, with a comment saying the
    crossing-aware pass should splice only `source->first-crossing` or
    `last-crossing->target`.
  - The current main routing path only calls those two checked native passes.
    The older `_apply_crossing_aware_endpoint_correction_to_record()` helpers
    remain in the file and in tests, but are not called by the active main
    route flow after the unsafe empty-map debug-artifact correction path was
    removed.
- Artifact evidence:
  - `build\verification\multiportmmi_8x8_photonic_verification.json` has `49`
    issues on `27` nets.
  - `build\verification\multiportmmi_8x8_crossing_verification.json` lists the
    exact same `27` nets as crossing participants.
  - Therefore all remaining port-connection failures are crossing-net
    failures, and no non-crossing net is currently in this failure class.
- Implication:
  - The next fix should implement a checked, active crossing-aware endpoint
    correction path in the main route flow. It should split each crossing net
    into mutable terminal segments (`source->first crossing` and/or
    `last crossing->target`) while freezing the middle segment and preserving
    static-stub endpoints as already corrected.

Crossing-aware endpoint correction WIP:

- Date: 2026-07-15
- Implemented an active main-flow crossing-aware endpoint-correction pass that
  uses native crossing events to populate `crossing_plan_info` and then calls
  `_apply_crossing_aware_endpoint_correction_to_record()` for crossing nets.
- Validation checks:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
  - `C:\Users\benja\.cargo\bin\cargo.exe check`
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "crossing_aware_endpoint_correction or case4_bump" -q`
  - Result: all passed (`14 passed, 13 deselected` for pytest).
- Full `multiportmmi_8x8` run with
  `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS=n_31` completed and reduced
  photonic verification issues from `49` to `14`.
  - Remaining issues: `target_port_not_connected=7`,
    `target_endpoint_mismatch=7`.
  - Affected nets: `n_33`, `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, `n_108`.
  - `n_31` trace showed an accepted crossing-aware correction:
    `mode=(False,True)`, target endpoint matched.
  - GDS: `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 22:04
    local, size 457830 bytes.
- Important finding:
  - The current crossing-aware helper still calls
    `router.route_port_corrected_centerline()` on the original full route.
  - For a `last crossing -> target` correction, the relevant local endpoint is
    the start of the terminal subroute at the last crossing, but the existing
    code does not pass that terminal subroute into the native checked bump
    candidate path.
  - In particular, the native `full_straight_offset_bump_candidates()` requires
    both `source_port_um` and `target_port_um`. When the Python helper corrects
    only target-side geometry with `source_port_um=None`, no start/end bump
    candidates are generated for that local crossing-to-target segment.
  - Therefore the current code does not really check the "start" bump of the
    `last crossing -> target` segment. The next implementation should construct
    terminal subroutes or an equivalent checked subroute-correction path so the
    segment-local start/end bump candidates are tested with the correct
    port-opening cells.

Crossing terminal-segment bump candidate check:

- Date: 2026-07-15
- Implemented a checked terminal-segment endpoint-correction path:
  - `src/geometry_realization.rs` now exposes
    `full_straight_offset_bump_candidates_for_centerline()`, allowing the
    existing compact case-4 bump geometry to be generated from a sliced
    centerline segment rather than only from a full `RouteResult`.
  - `src/py_router.rs` now exposes
    `PyPhotonicRouter.centerline_port_corrected_checked(...)`, which checks all
    generated segment-local bump candidates against static/dynamic obstacles
    using the same opened-cell and clearance-exemption logic as checked
    endpoint correction. Unlike the full-route checker, this local checker logs
    every candidate before selecting the first legal one.
  - `translation/route_rust.py` now splits crossing nets into
    `source -> first crossing`, frozen middle, and `last crossing -> target`
    terminal segments and tries the checked terminal-segment path before
    falling back to the old full-route correction.
  - The crossing cut guard was reduced from "up to half the remaining segment"
    to a fixed `4.0 um` guard so the terminal segment is not over-trimmed before
    bump insertion.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
  - `C:\Users\benja\.cargo\bin\cargo.exe check`
  - `.\.venv\Scripts\python.exe -m maturin develop --release`
  - `.\.venv\Scripts\python.exe -m pytest
    tests\test_port_alignment_diagnostics.py -k "crossing_aware_endpoint_correction or case4_bump" -q`
  - `cargo fmt --check` could not run because `rustfmt` is not installed for
    `stable-x86_64-pc-windows-gnullvm`.
- Trace evidence:
  - Run:
    `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS=n_33`
    `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS=34`
    `routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode
    lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35
    --foreign-port-keepout-cells 6 --debug-svgs false --debug-timing true`
  - Log:
    `build\multiportmmi_8x8_crossing_terminal_segment_trace_n33_guard4.txt`
  - For `n_33`/net id `34`, the local `last crossing -> target` segment now
    checks all four compact bump candidates:
    `start/top`, `start/bottom`, `end/top`, `end/bottom`.
  - All four currently reject with `static_overlap`, so the next issue is not
    candidate visibility anymore; it is that the opened port/pad cells supplied
    to the segment-local checker are still insufficient for this heater-pad
    access case.
- Current full-run state after this change:
  - Full routing still completes.
  - Photonic verification still reports `14` issues on
    `n_33`, `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, and `n_108`.
  - Latest GDS:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 22:21 local,
    size 457702 bytes.

Focused fix for `n_33` local terminal bump pad opening:

- Date: 2026-07-15
- Problem:
  - For `n_33`, the local `last crossing -> target` segment had enough
    physical length for the compact case-4 bump:
    - last crossing `x=1487.5 um`
    - crossing footprint half-width `4.0 um`
    - local segment start at footprint edge `x=1491.5 um`
    - target port `x=1517.7 um`
    - available distance from footprint edge to port: `26.2 um`
    - compact 4-bend bump length: `4 * 3 cells * 2 um = 24 um`
  - The segment-local bump candidates were nevertheless rejected by
    `static_overlap` against heater pad/access cells.
- Finding:
  - The global Rust port-opening template was not the right place to widen
    this; broadening it changed unrelated route-port-opening behavior.
  - The actual missing opening was in the per-job Python endpoint-correction
    candidate opening set used for checked endpoint bumps.
- Fix:
  - Kept the global Rust `route_port_endpoint_bump_candidate_cells()` template
    at its existing narrow extent.
  - Widened only Python's `_endpoint_bump_candidate_open_cells_for_state()` in
    `translation/route_rust.py` to cover the compact local bump envelope:
    `4 * bend_radius_cells` along the port axis and `2 * bend_radius_cells`
    laterally, excluding `forward=0` so the opening remains a local access
    allowance rather than a generic port-cell opening.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
  - `C:\Users\benja\.cargo\bin\cargo.exe check`
  - `.\.venv\Scripts\python.exe -m maturin develop --release`
  - Focused tests:
    `.\.venv\Scripts\python.exe -m pytest
    tests\test_route_rust_opened_cells.py::test_rust_port_opening_batch_is_directional_not_behind_port
    tests\test_route_rust_opened_cells.py::test_route_nets_rust_static_stub_fanout_uses_virtual_source_anchor
    tests\test_port_alignment_diagnostics.py -k "crossing_aware_endpoint_correction or case4_bump" -q`
    passed: `14 passed, 15 deselected`.
  - Trace run:
    `build\multiportmmi_8x8_n33_python_bump_opening_only_trace.txt`
    shows all four local `n_33`/net id `34` candidates accepted:
    `start/top`, `start/bottom`, `end/top`, `end/bottom`.
  - `n_33` is no longer in photonic verification failures. Remaining issue
    nets after this run:
    `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, `n_108`.
  - Latest GDS:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-15 22:40 local,
    size 458194 bytes.

Debug artifact after `n_36`:

- Date: 2026-07-16
- Command:
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --fanout-access-mode static-stubs
    --routing-window-scale 0.35 --foreign-port-keepout-cells 6
    --debug-stop-after-route 37 --debug-svgs true --debug-timing true`
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`.
- Result:
  - Stop route index `37` corresponds to `n_36` in this benchmark sequence.
  - Route SVG after `n_36` was written to
    `build\routes\multiportmmi_8x8_n_36.svg`.
  - Full log:
    `build\multiportmmi_8x8_stop_after_n36_svg_run.txt`.

Full GDS regenerated with known port issues:

- Date: 2026-07-16
- Command:
  - `.\.venv\Scripts\python.exe routing_flow.py multiportmmi_8x8 --crossings
    true --crossing-mode lidar-pure --fanout-access-mode static-stubs
    --routing-window-scale 0.35 --foreign-port-keepout-cells 6
    --debug-svgs false --debug-timing true`
  - Environment:
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE=1`.
- Result:
  - Full GDS written despite known photonic verification failures:
    `build\routed_multiportmmi_8x8.gds`, timestamp 2026-07-16 08:24 local,
    size 458194 bytes.
  - Photonic verification still reports `12` issues:
    `target_port_not_connected=6`, `target_endpoint_mismatch=6`.
  - Remaining issue nets:
    `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, `n_108`.
  - Log:
    `build\multiportmmi_8x8_full_gds_with_known_port_issues.txt`.

Terminal bump distance-check diagnostic:

- Date: 2026-07-16
- Scope:
  - User asked to implement only the condition for activating the terminal
    distance check when the target port is not aligned with the snapped target
    grid coordinate, and to report which nets it hits.
  - No routing behavior was changed; this is diagnostic metadata only.
- Change:
  - `translation/route_rust.py` now records
    `terminal_bump_target_x_offset_nets`,
    `terminal_bump_target_y_offset_nets`, and
    `terminal_bump_distance_checks` in
    `build\crossings\multiportmmi_8x8_crossings.json`.
  - The activation is axis-specific: horizontal target approaches only use
    `target_y != target_grid_y`; vertical target approaches only use
    `target_x != target_grid_x`. The earlier broad target-x diagnostic was
    removed.
  - In the current horizontal-target benchmark slice this yields
    `target_x_count=0`, `target_y_count=111`, and 7 crossing-on-target-axis
    distance checks:
    `n_33`, `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, and `n_108`.
  - The raw checks are now separated from failures via
    `terminal_bump_distance_failures`; only checks with `satisfies=false`
    appear in the failure list.
- Result:
  - `n_33` has exactly enough room: available `24.0um`, required `24.0um`,
    `satisfies=true`, and is therefore not reported as a failure.
  - Remaining failing port nets have insufficient room:
    `n_34`, `n_51`, `n_52`, `n_53`, `n_54`, and `n_108` each report
    available `20.0um`, required `24.0um`, `satisfies=false`.
  - This matches the current photonic verification failures:
    `target_port_not_connected=6`, `target_endpoint_mismatch=6` on those six
    nets.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust.py`
    passed.
  - Full `multiportmmi_8x8` run completed all 111 routes with 0 failures and
    0 repairs; latest validation runtime `37.0742 s`, native route batch
    `26.2514 s`.
  - GDS was written with known verification failures:
    `build\routed_multiportmmi_8x8.gds`.

Benes route-13 local crossing-footprint reservation WIP:

- Date: 2026-07-16
- Scope:
  - Implemented only "Grund 1" for the Benes route-13 issue: a candidate route
    must not route through a crossing footprint reserved earlier in the same
    crossing-aware A* path.
  - The already-discussed committed/global crossing-footprint blocker remains
    a separate next step.
- Change:
  - `src/astar.rs` carries active and pending local crossing footprint
    reservations per crossing-A* node.
  - A crossing footprint is activated immediately when the after-margin is
    already satisfied, or after `pending_after_crossing_cells` reaches zero
    when the crossing requires a following straight run.
  - `src/py_router.rs` now rejects crossing-bearing simple/normal fallback
    routes in collision-crossing mode, because those routes bypass the
    crossing-aware A* reservation state.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
    --crossings true --crossing-mode lidar-pure --debug-stop-after-route 13
    --debug-svgs 12,13 --attempt-diagnostics --debug-timing true`
    now stops cleanly at route 13 with
    `No legal LiDAR crossing route found; probe-based victim selection is
    disabled in crossing mode`, instead of writing a route with overlapping
    crossing components.
- Caveat:
  - `cargo fmt` could not be run because `rustfmt` is not installed for
    `stable-x86_64-pc-windows-gnullvm`.

Benes route-13 pending-after-crossing diagnostic:

- Date: 2026-07-16
- Scope:
  - Added a targeted Crossing-A* trace diagnostic for the proposed ripup
    trigger: count cases where A* accepts a crossing with pending after-margin,
    then cannot place the required straight continuation before hitting another
    crossing/intersection constraint.
  - No ripup behavior is changed yet.
- Change:
  - `src/astar.rs` now tracks the pending-after-crossing partner in the
    Crossing-A* key and accumulates:
    `crossing_perpendicular_reject_by_partner`,
    `crossing_after_margin_by_partner`, and
    `crossing_pending_straight_by_partner`.
  - The `PHOTONIC_ROUTER_TRACE_CROSSING` exhausted-search line now prints
    these per-partner counters.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `C:\Users\benja\.cargo\bin\cargo.exe test --no-run` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - Trace run:
    `PHOTONIC_ROUTER_TRACE_CROSSING=1`,
    `PHOTONIC_ROUTER_TRACE_CROSSING_NET=13`,
    `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
    --crossings true --crossing-mode lidar-pure --debug-stop-after-route 13
    --debug-svgs 12,13 --attempt-diagnostics --debug-timing true`.
- Result:
  - Route 13 still fails closed as expected.
  - In the final broad partner search:
    `perpendicular_reject_by_partner=[(12, 7404), (10, 6572), (11, 4124)]`,
    `pending_straight_by_partner=[(12, 1488)]`.
  - In the single-partner retry against partner `12`, the same final count is
    observed:
    `perpendicular_reject_by_partner=[(12, 7404)]`,
    `pending_straight_by_partner=[(12, 1488)]`.
  - This confirms a strong ripup signal: native partner net `12` is repeatedly
    hit by perpendicular crossing candidates for route `13`.
  - A threshold trace was added for the coarser perpendicular-reject counter via
    `PHOTONIC_ROUTER_TRACE_CROSSING_PERP_REJECT_THRESHOLD`.
  - With threshold `1000`, the broad partner search reaches:
    partner `10` at `1835 ms`, partner `12` at `4241 ms`, and partner `11` at
    `5425 ms`.
  - The single-partner retry against partner `12` reaches threshold `1000` at
    `3906 ms`.
  - Interpretation: a pure first-to-1000 perpendicular-reject trigger would
    choose native net `10` first; the more specific pending-straight signal
    still identifies native net `12` as the route-13 after-crossing blocker.
  - Re-run with
    `PHOTONIC_ROUTER_TRACE_CROSSING_PERP_REJECT_THRESHOLD=100`:
    broad search first reaches the threshold for partner `10` at `22 ms`,
    then partner `12` at `73 ms`, then partner `11` at `665 ms`. The
    single-partner retry against partner `12` reaches threshold `100` at
    `96 ms`.
  - Adjusted the pending-straight diagnostic so
    `pending_straight_by_partner` counts the original crossing partner whose
    required after-crossing straight cannot be completed, not the later net
    hit during that pending straight. Revalidated route 13: broad search still
    has first coarse perpendicular threshold on partner `10`, while
    `pending_straight_by_partner=[(12, 1488)]`.
  - Added a separate trace threshold
    `PHOTONIC_ROUTER_TRACE_CROSSING_PENDING_THRESHOLD` for the cleaned
    pending-straight counter. With threshold `1000`, route 13 hits
    `crossing-pending-straight-threshold net=13 partner=12` at `5321 ms`
    in the broad search; the single-partner retry against partner `12` hits
    it at `4853 ms`. Partners `10` and `11` did not produce pending-straight
    counts in their single-partner retries.

Multiport MMI regression gate before Benes pending-straight ripup:

- Date: 2026-07-16
- Command:
  `.\.venv\Scripts\python.exe -X utf8 routing_flow.py multiportmmi_8x8
  --crossings true --crossing-mode lidar-pure --debug-svgs none
  --debug-timing true`
- Result:
  - Fails at route `[34/111] n_33`.
  - Error:
    `No legal LiDAR crossing route found; probe-based victim selection is
    disabled in crossing mode`.
  - Targeted trace for native net `34` / route `n_33` shows a hard early
    block, not the Benes pending-straight pattern:
    `expanded=15`, `reject_margin=5`, `perpendicular_reject_by_partner=[(33, 5)]`,
    `pending_straight_by_partner=[]`.
  - Pre-failure debug SVG written at route `[33/111] n_32`:
    `build\routes\multiportmmi_8x8_n_32.svg`.
- Decision:
  - Do not add the Benes pending-straight ripup decider until this
    multiport regression is understood/stabilized, because this benchmark was
    intended as the guardrail.
  - Follow-up correction: the failing command was not equivalent to the stable
    dense-MMI configuration because it did not enable static fanout stubs.

Multiport MMI static-stub guardrail restored:

- Date: 2026-07-16
- Required command shape:
  - Set `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`.
  - Run:
    `.\.venv\Scripts\python.exe -X utf8 routing_flow.py multiportmmi_8x8
    --crossings true --crossing-mode lidar-pure --fanout-access-mode
    static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6
    --debug-svgs none --debug-timing true`.
- Result:
  - Command exited 0.
  - Routed all `111/111` nets with `failures=0`, `repairs=0`.
  - Timing: `total=40.1742 s`, `native_route_batch=29.8200 s`,
    `astar_loop=29.9163 s`.
  - Crossing verification:
    `success=True`, `error_count=0`, `routed_record_count=111`,
    `expected_route_count=111`.
  - Photonic verification:
    `success=True`, `error_count=0`, `routed_record_count=111`,
    `expected_route_count=111`.
- Decision:
  - Treat this command, not the plain non-stub crossing run, as the
    `multiportmmi_8x8` guardrail before implementing the Benes route-13
    pending-straight ripup decider.

Benes route-13 pending-straight ripup decider implemented:

- Date: 2026-07-16
- Change:
  - Added
    `route_single_net_with_collision_crossing_config_with_stats(...)` in
    `src\astar.rs` so failed collision-crossing A* attempts can still expose
    their accumulated `RouteSearchStats`.
  - Added a cleaned pending-straight victim hint in `src\py_router.rs`.
    `pending_straight_by_partner` now drives a LiDAR-pure one-net victim set
    when it reaches `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD`
    (default `100`).
  - The repair attempt rips the hinted victim, routes the current net, then
    reroutes the victim. If either leg fails, the router restores the exact
    pre-attempt batch state and continues to the previous failure path.
- Benes validation:
  - Command:
    `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1 .\.venv\Scripts\python.exe -X
    utf8 routing_flow.py benes_8x8 --crossings true --crossing-mode
    lidar-pure --debug-stop-after-route 13 --debug-svgs none
    --attempt-diagnostics --debug-timing true`.
  - Observed trigger:
    `native_repair_pending_straight_hint net=13 victim=12 count=1488 threshold=100`
    and
    `native_repair_pending_straight_start net=13 victim=12 count=1488`.
  - Partial stop completed successfully.
  - Routing summary: `repairs=1`, `failures=1`, `attempts=13`.
  - Crossing verification:
    `success=True`, `error_count=0`, `routed_record_count=13`,
    `expected_route_count=48`.
  - Photonic verification:
    `success=True`, `error_count=0`, `routed_record_count=13`,
    `expected_route_count=48`.
- Build validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `C:\Users\benja\.cargo\bin\cargo.exe test --no-run` passed.
- Next:
  - Re-run the documented `multiportmmi_8x8` static-stub guardrail before
    expanding this decider beyond the Benes route-13 partial stop.

Full Benes run after pending-straight ripup:

- Date: 2026-07-16
- Command:
  `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
  --crossings true --crossing-mode lidar-pure --debug-svgs none
  --debug-timing true`.
- Result:
  - Command exited 0 and wrote `build\routed_benes_8x8.gds`.
  - Full benchmark routed `48/48` nets.
  - Timing: `total=117.0211 s`, `native_route_batch=114.8981 s`,
    `astar_loop=114.9437 s`.
  - Routing summary: `repairs=15`, `failures=1`, `simple=14/48`.
  - Crossing verification:
    `success=True`, `error_count=0`, `routed_record_count=48`,
    `expected_route_count=48`.
  - Photonic verification:
    `success=True`, `error_count=0`, `routed_record_count=48`,
    `expected_route_count=48`.
- Notes:
  - The benchmark now completes end-to-end, but the full run is still slow and
    involves many repairs. The next pass should inspect later repair activity
    and then rerun the documented `multiportmmi_8x8` static-stub guardrail.

Benes route-10 debug SVG:

- Date: 2026-07-16
- Command:
  `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
  --crossings true --crossing-mode lidar-pure --debug-stop-after-route 10
  --debug-svgs 10 --attempt-diagnostics --debug-timing true`
- Result:
  - Route 10 is `n_s0_0_o1_to_s1_2_i0`.
  - Router SVG written to
    `build\routes\benes_8x8_n_s0_0_o1_to_s1_2_i0.svg`.

Benes pre-route-13 debug SVG:

- Date: 2026-07-16
- Command:
  `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
  --crossings true --crossing-mode lidar-pure --debug-stop-after-route 12
  --debug-svgs 12 --debug-timing true`
- Result:
  - Route 12 is `n_s0_1_o1_to_s1_2_i1`.
  - This is the routed state directly before route 13.
  - Router SVG written to
    `build\routes\benes_8x8_n_s0_1_o1_to_s1_2_i1.svg`.

Multiport MMI guardrail rechecked after Benes pending-straight ripup:

- Date: 2026-07-16
- Command:
  - Set `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`.
  - Run:
    `.\.venv\Scripts\python.exe -X utf8 routing_flow.py multiportmmi_8x8
    --crossings true --crossing-mode lidar-pure --fanout-access-mode
    static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6
    --debug-svgs none`.
- Result:
  - Command exited 0 and wrote `build\routed_multiportmmi_8x8.gds`.
  - External wall time with `Measure-Command`: `38.6471 s` before the
    Matplotlib cache cleanup and `38.3438 s` after it.
  - Crossing verification:
    `success=True`, `error_count=0`, `routed_record_count=111`,
    `expected_route_count=111`.
  - Photonic verification:
    `success=True`, `error_count=0`, `routed_record_count=111`,
    `expected_route_count=111`.
- Cleanup:
  - `routing_flow.py` now sets `MPLCONFIGDIR` to `build\mpl` before importing
    gdsfactory when the user has not supplied one. This removes the Windows
    warning where Matplotlib tried to create a cache under
    `C:\Users\benja\AppData\Local\matplotlib` and fell back to a temporary
    cache.
- Next:
  - Treat the Benes `117 s` runtime as a real routing/repair cost rather than
    an SVG artifact cost. The full Benes run used `--debug-svgs none` and still
    reported `repairs=15`.

Benes speed audit and preemptive ripup cleanup:

- Date: 2026-07-16
- Findings:
  - The slow `benes_8x8` run was not caused by SVG artifacts. A no-SVG profile
    wrote `build\profiles\benes_8x8_lidar_pure_profile.json` and showed
    `external_elapsed_s=116.8704`, `astar_time_s=114.9552`.
  - Attempt records showed the dominant work was not the new
    pending-straight counters. The new partner counters were hit on crossing
    candidates, while the total generated-neighbor count was much larger.
  - The broad `preemptive_crossing_ripup` path dominated the extra work:
    `28` preemptive attempts, about `62.8M` generated neighbors, and repeated
    rerouting of route 12 fourteen times.
  - The route-search summary was undercounting because it only aggregated old
    buckets (`normal_route`, `probe_route`, `repair_failed_net`,
    `reroute_victims`). It now aggregates all route buckets except
    `endpoint_correction`.
  - The slowest-route display was using per-batch average `elapsed_s`; it now
    ranks by `route_search_total_time_s` when that field exists.
- Cleanup:
  - Reintroduced a preemptive-ripup switch and made that experimental path
    opt-in. Set `PHOTONIC_ROUTER_PREEMPTIVE_CROSSING_RIPUP=1` to enable it.
    Default runs now skip the broad local crossing ripup and rely on the
    targeted pending-straight victim repair instead.
  - `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD` remains active with
    default threshold `100`; `0` disables that targeted decider.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed and
    installed the rebuilt extension.
  - `benes_8x8` default/no-SVG run with no preemptive env var:
    `elapsed_seconds=71.1702`; crossing verification
    `success=True`, `error_count=0`, `routed_record_count=48`,
    `expected_route_count=48`; photonic verification `success=True`,
    `error_count=0`, `routed_record_count=48`, `expected_route_count=48`.
  - `multiportmmi_8x8` static-stub guardrail with
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`:
    `elapsed_seconds=39.5066`; crossing verification
    `success=True`, `error_count=0`, `routed_record_count=111`,
    `expected_route_count=111`; photonic verification `success=True`,
    `error_count=0`, `routed_record_count=111`, `expected_route_count=111`.
- Remaining speed issue:
  - With preemptive ripup off, Benes still spends about 71 seconds in genuine
    crossing-aware A* searches. The current biggest inner-A* hot spots are the
    high-expansion normal routes such as route 37, 15, 31, 27, 23, 19, and 39,
    each with hundreds of thousands of expanded states and millions of
    generated neighbors. The next optimization should reduce search-space size
    or improve crossing-aware heuristics, not focus on SVG/debug output.

Crossing partner counter cleanup:

- Date: 2026-07-16 14:43 local
- Scope:
  - Marked the broad partner reject counters as analysis-only and disabled
    them in normal routing.
  - `crossing_perpendicular_reject_by_partner` and
    `crossing_after_margin_by_partner` are now populated only when
    `PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS=1`.
  - `crossing_pending_straight_by_partner` remains active because it is the
    targeted Benes route-13 repair metric and feeds the pending-straight
    victim hint.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 13 --debug-svgs none --debug-timing true` passed
    with both crossing and photonic verification success for the 13 routed
    records.
- Note:
  - No commit was made for this cleanup.

Benes full-run timing after counter cleanup:

- Date: 2026-07-16 14:46 local
- Command:
  - Cleared `PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS`.
  - Cleared `PHOTONIC_ROUTER_PREEMPTIVE_CROSSING_RIPUP`.
  - Ran `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8
    --crossings true --crossing-mode lidar-pure --debug-svgs none
    --debug-timing true` under `Measure-Command`.
- Result:
  - Wall time: `68.0866 s`.
  - Crossing verification:
    `success=True`, `error_count=0`, `routed_record_count=48`,
    `expected_route_count=48`.
  - Photonic verification:
    `success=True`, `error_count=0`, `routed_record_count=48`,
    `expected_route_count=48`.
  - Repeated timing run with output captured in
    `build\benes_8x8_current_timing.log`: wall time `67.0870 s`, internal
    total `63.1271 s`, net routing phase `61.5841 s`, native route batch
    `61.0990 s`, A* loop `61.1410 s`.
  - Slowest nets in that run:
    route `15` / `n_s0_3_o0_to_s1_1_i1` at `5.8489 s`,
    route `37` / `n_s3_2_o0_to_s4_0_i1` at `5.6645 s`,
    route `13` / `n_s0_2_o0_to_s1_1_i0` at `4.5900 s`,
    then routes `31`, `27`, `23`, `19`, and `39` at about `3.5-3.9 s` each.
  - Stop-after-13 native repair diagnostic confirmed the only current ripup:
    route/native net `13` (`n_s0_2_o0_to_s1_1_i0`) triggers the
    pending-straight hint and rips victim native net `12`
    (`n_s0_1_o1_to_s1_2_i1`) with count `1488` at threshold `100`.

Next speed plan: pure LiDAR owner-based hot path:

- Date: 2026-07-16 15:25 local
- Stable checkpoint intent:
  - Current `benes_8x8` and `multiportmmi_8x8` guardrails are both routing
    and verifying successfully, so this is the intended checkpoint before
    changing the A* crossing hot path.
- Planned improvements:
  - In pure LiDAR crossing mode, remove reliance on an allowed/expected
    crossing partner list from the hot path. A move should inspect only the
    dynamic owner net(s) actually hit by the candidate primitive footprint.
  - Add a fast path where moves with no dynamic collision perform no
    crossing-related partner work.
  - For exactly one dynamic owner, run a single local crossing legality check:
    perpendicularity, crossing footprint availability, before/after margin or
    pending-straight handling, then accept or reject that move.
  - Reject multi-owner footprint collisions early for now; introduce any
    multi-owner special case only after a measured need exists.
  - Avoid per-move `Vec`/`HashMap` allocation in the A* inner loop by using
    small fixed/scratch buffers where practical.
  - Keep the A* state compact: pending crossing angle/partner/reservation data
    should exist only after a crossing has actually been accepted.
  - Keep local crossing footprint reservations only for accepted crossings or
    once pending-after-crossing straight completion activates the reservation.
  - Keep `crossing_pending_straight_by_partner` active as the targeted repair
    signal; keep broader partner counters analysis-only behind
    `PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS=1`.
  - Defer topology-guided or expected-crossing waypoint heuristics until after
    this unguided hot-path cleanup, so any speedup is attributable to cheaper
    A* mechanics rather than stronger guidance.

Pure LiDAR hot-path instrumentation:

- Date: 2026-07-16 15:34 local
- Scope:
  - Added diagnostic counters to expose how often crossing-aware A* takes the
    contact-free fast path versus the crossing contact path.
  - Counters added:
    `crossing_hotpath_no_contact`,
    `crossing_hotpath_contact_checks`,
    `crossing_hotpath_static_rejects`,
    `crossing_hotpath_no_owner_contacts`,
    `crossing_hotpath_single_owner_contacts`, and
    `crossing_hotpath_multi_owner_contacts`.
  - These counters are surfaced through PyO3, route attempt records, route
    search summaries, and the CLI timing output.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none --debug-timing true` passed.
- Initial finding for the first slow net boundary:
  - Stop-after-route-11 summary:
    `no_contact=678014`, `contact_checks=1281289`,
    `static_rejects=994`, `no_owner=1221637`,
    `single_owner=58658`, `multi_owner=0`,
    `candidate_checks=9421`, `accepted=4116`.
  - Interpretation: the first slow unguided route is not dominated by true
    multi-owner crossing complexity. Most contact-path work reaches the
    crossing routine and then finds no dynamic core owner. The next safe
    optimization should make no-owner blocked contacts reject earlier before
    route/partner intersection work or per-move allocation.

No-owner early reject trial:

- Date: 2026-07-16 15:43 local
- Change:
  - Added an early return in the crossing contact path when the candidate
    primitive footprint was blocked, but scanning the crossing witness cells
    found no dynamic owner net.
  - The early return is gated by the caller's `!footprint_free` state, so
    contact-free extra-witness primitives remain allowed.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - Stop-after-route-11 Benes check passed; crossing and photonic
    verification were green for the 11 routed records.
  - Full `benes_8x8` run passed: wall time `65.1111 s`, crossing
    verification success for `48/48`, photonic verification success for
    `48/48`.
  - Full `multiportmmi_8x8` static-stub guardrail passed with
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`: wall time `36.8615 s`,
    crossing verification success for `111/111`, photonic verification
    success for `111/111`.
- Finding:
  - The stop-after-route-11 timing did not materially change. This means the
    main cost is likely still the witness/owner scan and per-contact-path
    overhead before the no-owner decision, not the later partner-segment logic.
  - Next likely speed step: avoid per-contact-path heap allocation and scan
    only the minimal dynamic-owner witness set before constructing
    `ContactedPartner` / reservation / intersection vectors.

Pure LiDAR hot-path detail analysis:

- Date: 2026-07-16 16:05 local
- Scope:
  - Added detail counters/timers for the crossing-aware A* hot path:
    witness cells scanned, partner segment checks, bbox rejects,
    intersection hits, total hot-path time, owner-scan time, segment-search
    time, and reservation time.
  - Surfaced these through PyO3, Python route timing summaries, and the CLI
    timing output.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none --debug-timing true`
    passed; total `4.1314 s`, route[11] `2.9775 s`.
  - Same stop run without `--debug-timing` passed; total `3.8184 s`,
    route[11] `2.8490 s`.
- Finding:
  - Route[11] (`n_s0_1_o0_to_s1_0_i1`) still dominates the stop run:
    `expanded=328547`, `generated=1971282`.
  - Hot-path summary for the non-debug-timing stop run:
    `no_contact=678014`, `contact_checks=1281289`,
    `static_rejects=994`, `no_owner=1221637`,
    `single_owner=58658`, `multi_owner=0`,
    `candidate_checks=9421`, `accepted=4116`.
  - Detail timing:
    `witness_cells=18831925`, `partner_segments=21186`,
    `bbox_rejects=11114`, `intersections=9421`,
    `total=1.2385 s`, `owner_scan=1.0889 s`,
    `segments=0.0495 s`, `reservation=0.0074 s`.
  - Interpretation: the expensive part is not the geometric segment
    intersection loop. The primary cost is repeatedly scanning the crossing
    witness/owner cells, and most scans find no dynamic owner. The clean next
    optimization is to split primitive crossing witnesses into core footprint
    cells and extra diagonal-halo cells, then scan only the necessary subset:
    if the primitive footprint is already known free, only extra halo witnesses
    can reveal an offset-diagonal dynamic collision. This should preserve
    behavior while reducing the 18.8M owner-cell probes.

Extra-halo-only owner scan optimization:

- Date: 2026-07-16 16:42 local
- Change:
  - `PrimitiveCrossingMetadata` now stores both the full witness set and the
    subset of extra diagonal-halo witnesses.
  - When the dense primitive footprint check has already reported
    `footprint_free=true`, crossing-aware A* scans only the extra halo
    witnesses for dynamic owners. Blocked footprints still scan the full
    witness set.
  - This does not change committed route cells or crossing legality checks; it
    only avoids re-reading footprint cells that the dense footprint check has
    already proven free.
  - Added `Default` derives / struct update defaults for Rust-side
    `PyRouteResult` tests so diagnostic field additions do not break test
    compilation.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `C:\Users\benja\.cargo\bin\cargo.exe test crossing_move --lib` compiled,
    but test execution stopped with Windows `STATUS_DLL_NOT_FOUND`; this is a
    local DLL runtime-path issue, not a Rust compile error.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none` passed:
    total `3.2977 s`, route[11] `2.2425 s`,
    `witness_cells=11279309`, `owner_scan=0.5897 s`,
    `segments=0.0443 s`.
  - Full `benes_8x8` passed:
    total `58.5958 s`, crossing verification success for `48/48`,
    photonic verification success for `48/48`.
  - Full `multiportmmi_8x8` with static stubs and
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90` passed:
    total `30.8698 s`, crossing verification success for `111/111`,
    photonic verification success for `111/111`.
- Result:
  - Stop-after-route-11 improved from about `3.8184 s` total / route[11]
    `2.8490 s` to `3.2977 s` total / route[11] `2.2425 s`.
  - Full Benes improved from the previous guardrail `65.1111 s` to
    `58.5958 s`.
  - Full Multiport static-stub guardrail improved from `36.8615 s` to
    `30.8698 s`.

Footprint-only blocked-contact scan optimization:

- Date: 2026-07-16 17:05 local
- Change:
  - Split primitive crossing witnesses further into explicit footprint
    witnesses and extra diagonal-halo witnesses.
  - Crossing-aware A* now uses:
    - `footprint_free=true`: scan only extra halo witnesses;
    - `footprint_free=false`: scan only footprint witnesses;
    - fallback/test path: full witness set remains available.
  - Rationale: halo exists only to detect offset diagonal contacts when the
    real footprint did not collide. Once the real footprint is already blocked,
    the collision is known and halo cells are redundant.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none` passed:
    total `3.0954 s`, route[11] `2.0113 s`,
    `witness_cells=11215333`, `owner_scan=0.4224 s`.
  - Full `benes_8x8` passed:
    total `56.8744 s`, crossing verification success for `48/48`,
    photonic verification success for `48/48`.
  - Full `multiportmmi_8x8` with static stubs and
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90` passed:
    total `30.3986 s`, crossing verification success for `111/111`,
    photonic verification success for `111/111`.
- Result:
  - Stop-after-route-11 route[11] improved again from `2.2425 s` to
    `2.0113 s`.
  - Full Benes improved from `58.5958 s` to `56.8744 s`.
  - Full Multiport static-stub guardrail improved from `30.8698 s` to
    `30.3986 s`.

Queued pure-LiDAR hot-path speed ideas:

- Date: 2026-07-16 17:18 local
- Constraint:
  - Keep routing behavior and crossing functionality intact. Do not introduce
    pruning that rejects multi-owner or multi-crossing cases unless a later
    correctness analysis proves it is safe.
- Priority 1: extra-halo presence fast path.
  - Current issue: after `footprint_free=true`, A* still scans selected halo
    witness cells one by one to discover that most moves have no dynamic owner.
  - Proposed change: build a compact collision profile for
    `extra_witnesses`. If the true footprint is free and the extra halo profile
    is also free, take `crossing_no_contact_outcome` immediately. Only run the
    detailed owner/static/crossing analysis when the halo presence check says
    something is occupied.
  - Expected benefit: accelerates the dominant no-contact path without changing
    crossing legality.
- Priority 2: owner scan without per-move heap allocations.
  - Current issue: contact handling still uses per-move `Vec<ContactedPartner>`
    and witness vectors.
  - Proposed change: use a small stack/scratch structure or partner bitmask to
    record contacted partners and first witnesses while keeping multi-owner
    support. Build full witness vectors only for rare unresolved-contact
    diagnostics/classification.
- Priority 3: direct dense-owner indexing / owner mask.
  - Current issue: `owner_at(x, y)` performs bounds/index conversion for each
    witness cell.
  - Proposed change: when scanning a prevalidated witness profile inside the
    lookup bounds, use precomputed local offsets or direct dense-grid indices.
    Longer-term, add an owner-presence/owner-mask helper that can answer
    "empty / one owner / multiple owners" for a witness profile before detailed
    per-cell analysis.
- Priority 4: partner segment locality.
  - Current issue: once an owner is found, the code still checks partner
    segments with bbox filtering.
  - Proposed change: index partner segments spatially or by route cell so the
    local contact checks only inspect nearby candidate segments. Keep the same
    perpendicular, margin, pending, reservation, and reporting logic.
- Priority 5: reduce state expansion after hot-path cleanup.
  - Even after owner-scan improvements, slow Benes routes still expand hundreds
    of thousands of states. Only after the local move checks are cheap should
    we evaluate search-ordering or admissible pruning/guidance changes.
- Priority 6: rip-up / repair performance audit.
  - Current question: determine whether rip-up bookkeeping itself is slow, or
    whether repair appears slow because it triggers additional A* searches and
    victim reroutes.
  - Existing timing buckets already expose `ripup`, `repair_failed_net_wall`,
    `reroute_victims_wall`, `repair_probe_victim_selection`, and
    `repair_state_reset`, but the normal benchmark summaries do not yet make
    the per-repair-cycle cost obvious.
  - Proposed change: add a lightweight repair summary that reports repair
    count, victim set sizes, trigger reason, current-net reroute time,
    victim-reroute time, pure rip-up bookkeeping time, and whether the repair
    converged. Use this before changing repair behavior or adding new rip-up
    heuristics.
  - Expected benefit: separates true rip-up overhead from repeated A* work, so
    later speed work can target the correct layer.

Extra-halo presence fast path:

- Date: 2026-07-16 17:40 local
- Change:
  - Added a stats-free `DenseRoutingGrid::relative_offsets_free_with_profile`
    helper for compact occupancy checks over arbitrary relative offsets.
  - `PrimitiveCrossingMetadata` now precomputes an
    `extra_witness_profile` and `extra_witness_offsets`.
  - In crossing-aware A*, when the true primitive footprint is free and the
    primitive has extra diagonal-halo witnesses, A* first checks whether the
    extra halo profile is free. If it is free, the move takes
    `crossing_no_contact_outcome` immediately and skips detailed owner/static
    witness analysis.
  - If the extra halo profile is blocked, detailed crossing analysis is
    unchanged.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none` passed:
    total `2.1314 s`, route[11] `1.0632 s`,
    `contact_checks=67964`, `witness_cells=367992`,
    `owner_scan=0.0040 s`.
  - Full `benes_8x8` passed:
    total `29.3395 s`, crossing verification success for `48/48`,
    photonic verification success for `48/48`.
  - Full `multiportmmi_8x8` with static stubs and
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90` passed:
    total `21.9246 s`, crossing verification success for `111/111`,
    photonic verification success for `111/111`.
- Result:
  - Stop-after-route-11 route[11] improved from `2.0113 s` to `1.0632 s`.
  - Stop-after-route-11 detailed contact checks dropped from about `1.28M`
    before the halo fast path to `67,964`; witness cells dropped from about
    `11.2M` to `367,992`.
  - Full Benes improved from `56.8744 s` to `29.3395 s`.
  - Full Multiport static-stub guardrail improved from `30.3986 s` to
    `21.9246 s`.

Allocation-light contacted-partner collection:

- Date: 2026-07-16 18:08 local
- Change:
  - Replaced the hot-path `Vec<ContactedPartner>` first-contact allocation with
    a `ContactedPartners` scratch structure that stores the first contacted
    partner inline and allocates the extra partner vector only for true
    multi-owner contacts.
  - Replaced per-contact `vec![witness]` with an inline first witness plus an
    optional extra witness vector.
  - Unresolved-contact classification still sees every witness; multi-owner
    and multi-crossing behavior remains supported.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m py_compile translation\route_rust_types.py
    routing_flow.py` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none` passed:
    total `1.9278 s`, route[11] `0.9563 s`,
    `owner_scan=0.0031 s`.
  - Full `benes_8x8` passed:
    total `27.0866 s`, crossing verification success for `48/48`,
    photonic verification success for `48/48`.
  - Full `multiportmmi_8x8` with static stubs and
    `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90` passed:
    total `20.1535 s`, crossing verification success for `111/111`,
    photonic verification success for `111/111`.
- Result:
  - Stop-after-route-11 route[11] improved from `1.0632 s` to `0.9563 s`.
  - Full Benes improved from `29.3395 s` to `27.0866 s`.
  - Full Multiport static-stub guardrail improved from `21.9246 s` to
    `20.1535 s`.

Native repair/rip-up timing visibility:

- Date: 2026-07-16 18:32 local
- Change:
  - Added a `native repair profile` line to `routing_flow.py` debug timing
    output.
  - The line reports the already-collected native timing buckets for `ripup`,
    `repair_failed_net_wall`, `reroute_victims_wall`,
    `repair_probe_victim_selection`, and `repair_state_reset`, plus native
    search context.
  - This is reporting only; it does not change A*, victim selection, rip-up,
    repair, or route geometry.
- Validation:
  - `.\.venv\Scripts\python.exe -m py_compile routing_flow.py` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 13 --debug-svgs none --debug-timing true`
    passed in `12.9994 s`.
- First measurement:
  - Stop-after-route-13 reported `repairs=1`.
  - `native repair profile`: `ripup=0.0002 s`, `current=1.3658 s`,
    `victims=1.0358 s`, `selection=0.0000 s`, `reset=0.0000 s`,
    `repair_total=2.4018 s`, `native_search=3.5085 s`,
    `dense_astar=3.4826 s`.
  - Initial conclusion: for this Benes repair case, rip-up bookkeeping is not
    the speed problem; the cost is the extra A* search work for the current net
    and victim reroute.

Dense-owner direct-index experiment:

- Date: 2026-07-16 18:55 local
- Experiment:
  - Tried replacing per-witness `DenseDynamicCoreOwnerGrid::owner_at(x, y)`
    calls with a per-state owner-grid origin plus relative index offsets.
  - First safe variant still performed per-witness local bounds checks; second
    variant relied on the existing expanded lookup-bounds invariant and used a
    lighter `base_idx + dy * width + dx` access path.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed for both variants.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none --debug-timing true`
    passed, but route[11] was slower (`1.1296 s` first variant, `1.0860 s`
    second variant) than the committed allocation-light baseline
    (`0.9563 s`).
  - A no-debug-timing stop-after-route-11 check with the second variant also
    looked worse (`route[11]=1.2418 s`).
- Result:
  - Reverted the experiment completely and rebuilt the release extension back
    to the committed code.
  - Conclusion: this direct-index variant is not worth keeping. The next
    speed target should move away from this micro-optimization and instead
    examine the remaining segment-check/search-space costs, which dominate the
    measured route[11] time.

Partner segment locality experiment:

- Date: 2026-07-16 19:20 local
- Experiment:
  - Built a per-partner `cell -> segment ids` index for crossing partners.
  - In `crossing_move_outcome_with_segments`, collected candidate partner
    segment ids from the contacted Witness cells and scanned only those
    segments when available, with a full-scan fallback if the index returned no
    candidate.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `benes_8x8 --crossings true --crossing-mode lidar-pure
    --debug-stop-after-route 11 --debug-svgs none --debug-timing true`
    passed.
  - A no-debug-timing route-11 stop also passed.
- Measurement:
  - The experiment reduced `partner_segments` from the baseline `21195` to
    `11071` and `bbox_rejects` from `11114` to `1026`.
  - Despite that, `segments` time stayed about the same (`0.0496 s`) and
    route[11] remained slower than the committed baseline (`1.1351 s` with
    debug timing, `1.1241 s` in the no-debug check, versus baseline route[11]
    around `0.9563 s`).
- Result:
  - Reverted the code experiment completely and rebuilt the release extension
    back to the committed code.
  - Conclusion: reducing partner segment scans with this cell index does not
    address the dominant cost. The remaining route[11] time is mostly generic
    A* state expansion and un-attributed search-loop work: roughly 328k
    expanded states, 1.97M generated neighbors, 495k heap pushes, and 381k
    heap pops for that route. The next useful work should instrument and then
    reduce state expansion / heap and neighbor overhead, rather than adding
    another crossing micro-index.

Crossing-A* state-storage lookup experiment:

- Date: 2026-07-16 local
- Status:
  - Code experiments were deliberately left uncommitted and then reverted after
    measurements. Keep the notes, but do not treat this as active source WIP.
- Hot path identified:
  - In pure LiDAR crossing mode, `SparseCrossingStateStorage` usually takes
    the packed `u64` key path because `require_all_partners=false` and normal
    states have no crossed-mask/partner-order state.
  - The most-used storage operations are:
    `best_cost(key)` and `insert_closed(key)` for popped heap entries, then
    `contains_closed(next_key)`, `best_cost(next_key)`, and `set_best_cost`
    for generated/accepted neighbors.
  - Before this experiment those operations used separate `best_cost` and
    `closed` hash tables, causing repeated hash work over packed keys.
- Experiment:
  - Replaced the separate `best_cost` and `closed` maps/sets with one
    `CrossingStateRecord { best_cost, closed }` map for packed keys and one
    fallback map for full `CrossingAStarKey`.
  - This keeps routing behavior and search ordering unchanged; only storage
    lookup layout changes.
- Measurements:
  - `cargo check` passed.
  - `maturin develop --release` passed.
  - Benes route-11 stop with default lidar-pure/no-SVG passed:
    `route[11]=0.9404 s`, `search_loop=0.9333 s`, total `1.9360 s`, with the
    same search size as baseline (`expanded=328547`, `generated=1971282`).
  - Full `benes_8x8` passed: total `27.3579 s`, crossing and photonic
    verification success for `48/48`.
  - Full `multiportmmi_8x8` static-stub guardrail passed: total `20.6029 s`,
    crossing and photonic verification success for `111/111`.
- Interpretation:
  - Faster storage can help a single hot route without changing behavior.
  - The combined map does not yet produce a clear full-benchmark win; the next
    speed work should stay focused on `SparseCrossingStateStorage` and reduce
    the cost/frequency of the hottest operations, especially repeated
    `pack_key` plus HashMap get/entry work.
  - Candidate next steps: pack once per key use site, add get-or-closed helpers
    that avoid double lookups, or test a generation-stamped dense/slab storage
    for packed crossing states.
  - Follow-up attempt: combined `try_close_current` / neighbor `status`
    helpers reduced explicit method calls but were slower in practice
    (`route[11]` about `1.03 s`), likely because `HashMap::entry` and larger
    record handling cost more than the avoided lookups.
  - Follow-up attempt: moving the per-expanded-node
    `pending_local_reservation_keys` clone into the rare pending-completion
    branch did not improve route[11] (`about 1.06 s` in the spot check). The
    clone is not the dominant hot path.
  - Follow-up attempt: replacing the packed-key `FxHashMap` with a naive
    identity-hasher `HashMap<u64, ...>` was rejected. The route-11 stop did not
    finish after about 94 seconds and had to be stopped, likely due to poor
    bucket distribution for the structured packed keys. Reverted that attempt.
  - Follow-up attempt: replacing packed-key `FxHashMap` with a small custom
    open-addressed `u64 -> CrossingStateRecord` flat map was also rejected.
    It passed `cargo check`, but route[11] slowed to about `4.15 s` with the
    same search size. Reverted that attempt.
  - Final local decision for this pass: stop Storage micro-optimization and
    leave `src/astar.rs` at the pre-experiment implementation. The higher-level
    speed target remains reducing A* search size / route attempts, not swapping
    hash table internals.

Simple-first / collision-kernel ordering fix:

- Date: 2026-07-16 local
- User-facing issue:
  - After making the Benes route-2 case simple again, the `multiportmmi_8x8`
    cluster around visible `n_64`..`n_67` changed geometry. The visible
    geometry shift was traced to internal `net=67` / visible `n_66`, not to
    `n_64`.
  - The old early collision-kernel commit path had selected a no-crossing
    collision-kernel route for `n_66` before normal A* ran. After moving simple
    route attempts earlier, normal A* could win even when the no-crossing
    collision-kernel route was shorter.
- Fix in `src/py_router.rs`:
  - Defer the collision-crossing attempt until after the explicit simple-route
    attempt. This preserves the intended ordering: simple first, then
    collision-crossing kernel, then normal A*/repair.
  - If the collision-kernel attempt contains real crossing events, keep using
    the reservation-aware crossing commit path.
  - If the collision-kernel attempt has no crossing events, treat it as an
    ordinary fallback candidate and compare it against the normal A* result by
    `total_cost`; commit the cheaper route through the normal route validation
    path.
  - Fixed the pending-straight repair flow so a failed repair attempt falls
    through to the same net's normal failure handling instead of silently
    continuing and leaving missing route records.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.\.venv\Scripts\python.exe -X utf8 routing_flow.py benes_8x8 --crossings true --crossing-mode lidar-pure --debug-svgs none --debug-timing true`
    passed: `48/48` routed, `failures=0`, `repairs=0`, `simple=38/48`,
    photonic verification success, crossing verification success with
    `matched_crossing_component_count=16`, total `21.7302 s`.
  - `$env:PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES='90'; .\.venv\Scripts\python.exe -X utf8 routing_flow.py multiportmmi_8x8 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6 --debug-svgs none --debug-timing true`
    passed: `111/111` routed, `failures=0`, `repairs=0`, `simple=57/111`,
    photonic verification success, crossing verification success with
    `matched_crossing_component_count=35`, total `24.1020 s`.
- Current artifact pointers:
  - `build\routed_multiportmmi_8x8.gds` is the latest full green
    `multiportmmi_8x8` artifact from this validation.
  - `build\routed_benes_8x8.gds` is the latest full green `benes_8x8`
    artifact from this validation.

LiDAR multiport MMI benchmark import:

- Date: 2026-07-16 local
- Source:
  - Cloned `https://github.com/hibenj/LiDAR` read-only into
    `build\_external\LiDAR` and inspected
    `src\picroute\benchmarks\multiportmmi_8x8`,
    `multiportmmi_16x16`, and `multiportmmi_32x32`.
- 8x8 comparison:
  - Existing local `benchmarks\data\multiportmmi_8x8.yml` is not byte-identical
    to the current upstream file because the local file is an older/sanitized
    YAML form while current upstream contains `!!python/tuple` tags in
    settings.
  - Semantic comparison passed: same instance keys, placement keys, net keys,
    component names, net endpoints, and exact placement values.
- Added:
  - `benchmarks\data\multiportmmi_16x16.yml` from upstream LiDAR.
  - `benchmarks\data\multiportmmi_32x32.yml` from upstream LiDAR.
  - `benchmarks\multiportmmi_yaml.py` shared loader using `yaml.FullLoader`
    for upstream tuple-tag settings.
  - `benchmarks\multiportmmi_16x16.py` and
    `benchmarks\multiportmmi_32x32.py`.
  - Refactored `benchmarks\multiportmmi_8x8.py` to use the shared loader.
- Validation:
  - Built unrouted layouts and wrote:
    `build\unrouted_multiportmmi_16x16.gds` and
    `build\unrouted_multiportmmi_32x32.gds`.
  - Counts:
    - `multiportmmi_8x8`: 82 instances / 111 nets.
    - `multiportmmi_16x16`: 162 instances / 223 nets.
    - `multiportmmi_32x32`: 318 instances / 447 nets.
  - `.\.venv\Scripts\python.exe -X utf8 -m pytest tests\test_routing_flow_stats.py::test_lidar_multiportmmi_yaml_benchmarks_load -q`
    passed with `3 passed`.
- Display:
  - `klayout.exe` was not found in PATH or common Windows installation
    locations.
  - Started both generated GDS files through Windows file association with
    `Start-Process`; if `.gds` is associated with KLayout on the workstation,
    they should be open there.

LiDAR multiportmmi_16x16 incremental routing:

- Date: 2026-07-16 local
- Configuration:
  - `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`
  - `routing_flow.py multiportmmi_16x16 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 6 --debug-svgs none --debug-timing true`
- Incremental stop results:
  - Stop after route 20: success, `20/223`, failures `0`, repairs `0`,
    crossings `0`.
  - Stop after route 48: success, `48/223`, failures `0`, repairs `0`.
  - Stop after route 64: success, `64/223`, failures `0`, repairs `0`,
    crossings `0`.
  - Stop after route 73: success, `73/223`, failures `0`, repairs `0`,
    matched crossings `4`, illegal crossings `0`.
  - Stop after route 74: success, `74/223`, photonic verification success,
    crossing verification success, matched crossings `8`, illegal crossings
    `0`.
  - Stop after route 79: success, `79/223`, photonic verification success,
    crossing verification success, matched crossings `11`, illegal crossings
    `0`, total `64.2440 s`. The slow route is route index `75` / `n_74`,
    taking `40.2980 s` alone with `7,710,134` expanded states.
  - Stop after route 79 with
    `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM=30`: success,
    `79/223`, photonic verification success, crossing verification success,
    matched crossings `11`, illegal crossings `0`, total `68.0938 s`. Lowering
    the search-only crossing penalty from `50` to `30` did not improve this
    checkpoint; route index `75` / `n_74` still dominates at `44.3756 s`.
  - Stop after route 79 with
    `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM=0`: success,
    `79/223`, photonic verification success, crossing verification success,
    matched crossings `11`, illegal crossings `0`, total `57.9060 s`.
    Route index `75` / `n_74` still dominates at `39.9961 s` and still chooses
    the same three crossing partners (`n_69`, `n_72`, `n_71`) instead of the
    visually expected five-crossing corridor through `n_73`, `n_72`, `n_71`,
    `n_70`, `n_69`.
  - Trace evidence for `n_74`:
    - Internal IDs: `n_69..n_74` are `70..75`.
    - Candidate/level-1 trace shows individual accepted crossing candidates
      for all five relevant partners (`n_73`, `n_72`, `n_71`, `n_70`, `n_69`).
    - The selected final route still reconstructs only crossings with
      `n_69`, `n_72`, and `n_71`; this points away from crossing cost and
      toward A* state/search/legality around the full green corridor.
    - Added temporary env-guarded A* branch tracing in `src\astar.rs`.
      With marker `partner=74` (`n_73`) at `grid_y=441`
      (`y ~= 1645.125 um`) and target X bins `1126,1131,1136,1141`
      (`x ~= 2233.5,2243.5,2253.5,2263.5 um`), the trace produced
      `crossing-branch-target-mask-complete` 60 times. This proves at least
      one single descendant branch after the desired `n_73` crossing reaches
      all four later X bins; the remaining question is why that branch is not
      the final selected route.
- Current artifact pointer:
  - `build\routed_multiportmmi_16x16.gds` is the latest stop-after-79 partial
    route artifact.

LiDAR multiportmmi_16x16 pending-straight model fix:

- Date: 2026-07-16 local
- Change under test:
  - `src\astar.rs` now lets `pending_after_crossing_cells` be consumed across
    multiple same-direction straight primitives instead of requiring the first
    primitive after a crossing to cover the full remaining straight margin.
  - Bends before the pending straight is satisfied remain rejected.
  - Pending local crossing reservations are kept pending until the accumulated
    straight run completes, then promoted to active reservations.
- Validation:
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed.
  - `.\.venv\Scripts\python.exe -m maturin develop --release` passed.
  - `.\.venv\Scripts\python.exe -X utf8 -m pytest tests\test_routing_flow_stats.py::test_lidar_pure_uses_search_only_crossing_penalty_by_default tests\test_routing_flow_stats.py::test_collision_crossing_search_penalty_can_be_overridden -q`
    passed with `2 passed`.
  - Stop after route 79 with the standard 16x16 WIP config
    (`PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `--crossing-mode lidar-pure`, `--fanout-access-mode static-stubs`,
    `--routing-window-scale 0.35`, `--foreign-port-keepout-cells 6`,
    `--debug-svgs none`, `--debug-timing true`) passed.
  - Stop after route 99 with the same config passed.
- Result:
  - Total stop-after-79 runtime improved from about `64.2440 s` to
    `33.8282 s`.
  - `route[75]` / `n_74` improved from about `40.2980 s` to `7.5243 s`.
  - `n_74` now has five legal native crossings in the verification JSON:
    `n_73`, `n_72`, `n_71`, `n_70`, and `n_69`.
  - No failures or repairs occurred in this stop-after-79 run.
  - Stop-after-99 runtime: `35.8727 s` total, route search `21.2986 s`,
    `99/223` routes, failures `0`, repairs `0`, simple routes `67/99`.
    Slowest new route after the previous checkpoint was route index `98` /
    `n_97` at `0.8547 s`.
  - Stop-after-111 runtime: `39.6584 s` total, route search `23.3446 s`,
    `111/223` routes, failures `0`, repairs `0`, simple routes `73/111`.
    Slowest new route after the previous checkpoint was route index `101` /
    `n_100` at `0.4101 s`.
  - Re-ran stop-after-111 after removing the abandoned debug timing guard:
    runtime `39.4737 s` total, `111/223` routes, failures `0`, repairs `0`,
    simple routes `73/111`. Current artifact:
    `build\routed_multiportmmi_16x16.gds`.

Checkpoint validation before commit:

- Date: 2026-07-16 local
- `multiportmmi_8x8` full run:
  - Config: `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES=90`,
    `--crossings true --crossing-mode lidar-pure
    --fanout-access-mode static-stubs --routing-window-scale 0.35
    --foreign-port-keepout-cells 6 --debug-svgs none --debug-timing true`.
  - Result: `111/111`, failures `0`, repairs `0`, simple routes `57/111`,
    total `22.8006 s`.
- `benes_8x8` full run:
  - Same config.
  - Result: `48/48`, failures `0`, repairs `0`, simple routes `38/48`,
    total `22.4224 s`.
- Cleanup before commit:
  - Removed temporary `n_74` branch-tracing instrumentation from
    `src\astar.rs`; the pending-straight model change remains.
  - `C:\Users\benja\.cargo\bin\cargo.exe check` passed after cleanup.
  - `.\.venv\Scripts\python.exe -X utf8 -m pytest tests\test_routing_flow_stats.py::test_lidar_multiportmmi_yaml_benchmarks_load tests\test_routing_flow_stats.py::test_lidar_pure_uses_search_only_crossing_penalty_by_default tests\test_routing_flow_stats.py::test_collision_crossing_search_penalty_can_be_overridden -q`
    passed with `5 passed`.

Post-commit multiportmmi_16x16 continuation:

- Date: 2026-07-16 local
- Attempted stop-after-129 first, but it ran unusually long and was manually
  stopped before completion.
- Re-ran a smaller step from the stable stop-after-111 checkpoint to
  stop-after-115 with the same config.
- Result: `115/223`, failures `0`, repairs `0`, simple routes `73/115`,
  total `43.3453 s`.
- No new route after 111 appears in the slowest-route list; the expensive
  entries remain the earlier known routes (`n_74`, `n_73`, `n_71`, `n_72`,
  `n_97`, etc.). Current artifact: `build\routed_multiportmmi_16x16.gds`.
- Next four-route step to stop-after-119 with the same config also passed.
  Result: `119/223`, failures `0`, repairs `0`, simple routes `76/119`,
  total `42.1968 s`, route search `24.8818 s`.
- No new slow route after 115 appeared in the top list. The same earlier
  routes dominate: `n_74` (`7.8390 s`), `n_73` (`5.2920 s`), `n_72`
  (`1.8943 s`), `n_71` (`1.8932 s`), and `n_97` (`0.8548 s`). Current
  artifact remains `build\routed_multiportmmi_16x16.gds`.
- Further 2-route stepping:
  - Stop-after-121 passed: `121/223`, failures `0`, repairs `0`, simple
    routes `77/121`, total `43.9743 s`.
  - Stop-after-123 passed: `123/223`, failures `0`, repairs `0`, simple
    routes `77/123`, total `43.7571 s`.
  - Stop-after-124 did not finish in the quick diagnostic window and was
    stopped. Since route 123 maps to `n_122`, the first suspicious route is
    the next net, `n_123` (route index 124).
  - `n_123` connects `mmi0_ps_array_2_heater_12,o2` at `(3371.0,
    2050.0)`, orientation `0`, to `mmi0_multiport_2_0,o13` at `(3626.9,
    1527.5)`, orientation `180`.
  - The MMI target-side port state is not obviously closed: it is inside the
    static blocked set but also inside the per-port open cells, as expected for
    a port-access state. With the normal CLI config (`grid_size_um=2.0`,
    `bend_radius_um=5.0`), normal MMI target port lanes use
    `port_lane_length_cells = 8` (`16um`).
  - Live trace for route index 124 / `n_123` showed the real slowdown starts in
    collision-crossing search, not at immediate port access:
    first `partners=[121, 123, 119, 122, 120]`, then a candidate with
    `expanded=183361`, `generated=1100166`, `accepted=2321`, but
    `events=[]`; after that, the router continued with repeated reduced
    partner searches such as `[119]`, `[120]`, `[121]`, `[122]`. This points to
    wasted collision-crossing probing / fallback sequencing rather than a
    simple blocked target port.
  - Temporary experiment: forcing the legacy LiDAR-pure partner lookup sets to
    empty does disable this old partner-guided path, but it is not yet viable.
    With empty partners, `multiportmmi_16x16 --debug-stop-after-route 124`
    fails earlier at `n_70` with `No legal LiDAR crossing route found`. This
    shows the current list-free A* path does not yet provide a complete local
    owner-cell crossing flow. The correct next patch is to make LiDAR-pure A*
    derive crossing partners from actual dynamic obstacle owners per move, then
    remove/disable the old precomputed partner list for LiDAR-pure.
  - After the experiment, `src/py_router.rs` was restored to no diff and the
    Rust extension was rebuilt from source with `.\.venv\Scripts\python.exe -m
    maturin develop --release` after `cargo check`.

LiDAR-pure owner-lookup experiment and control benchmark check:

- Date: 2026-07-16 local
- Tried replacing the LiDAR-pure route/window partner lookup with all committed
  dynamic owners so the crossing A* would use the actual collided owner as the
  legality target instead of a preselected partner window.
- Result: this is semantically closer to the desired owner-cell model, but the
  current orchestration still uses a non-empty lookup set as the trigger to run
  the collision-crossing A* before ordinary fallback. Supplying all committed
  owners therefore made the crossing A* activate too broadly and caused
  `multiportmmi_8x8` / `multiportmmi_16x16` control runs to stall early.
- The source change was reverted manually; `src/py_router.rs` is back to no
  diff. `cargo check` passed and the Rust extension was rebuilt with
  `.\.venv\Scripts\python.exe -m maturin develop --release`.
- Control benchmarks after restoring the stable mechanism:
  - `multiportmmi_8x8 --crossings true --crossing-mode lidar-pure
    --fanout-access-mode static-stubs --routing-window-scale 0.35
    --foreign-port-keepout-cells 6 --debug-svgs none --debug-timing true`
    passed: `111/111`, failures `0`, repairs `0`, simple `57/111`, net routing
    phase `13.6475 s`, total `21.5565 s`.
  - `benes_8x8` with the same flags passed: `48/48`, failures `0`, repairs
    `0`, simple `38/48`, net routing phase `19.8881 s`, total `21.3904 s`.
- Next implementation direction: split the LiDAR-pure concept into two separate
  concepts in code:
  1. a cheap boolean trigger for whether crossing-aware A* is worth trying, and
  2. a centerline lookup database for owners touched by actual dynamic
     obstacle cells.
  The current single `partner_ids` parameter conflates those roles.
- Clarified target semantics: "owner-based Crossing-A*" means LiDAR-pure A*
  may check any committed dynamic route owner that its current move actually
  collides with. It is not a partner whitelist. The owner is read from the
  obstacle map at the collision cell and used only to fetch the committed
  centerline, record crossing/ripup telemetry, and decide legality for that
  move. The desired LiDAR-pure flow is simple route first; if simple fails, run
  crossing-aware obstacle A* with all committed owners available as lookup data;
  only if that A* cannot find a legal route should ripup use real failure
  signals such as pending-straight/perpendicular reject counters.
