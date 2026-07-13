# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
every agent stop, pause, or handoff. It does not replace the active ExecPlan.

## Current Snapshot

- Date: 2026-07-12 20:03Z
- Branch: `crossings/verification-foundation`
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
