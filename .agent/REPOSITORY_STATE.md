# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
every agent stop, pause, or handoff. It does not replace the active ExecPlan.

Keep this file compact. Do not append a new dated section per stop; overwrite
the sections below in place so this file always reflects only the current
state. Detailed run-by-run history belongs in the active ExecPlan's `Progress`
and `Surprises & Discoveries` sections (per `.agent/PLANS.md`), which are
allowed to grow because they are the plan's own record, not this file's job.
Older history is still recoverable from `git log -p -- .agent/REPOSITORY_STATE.md`
if it is ever needed.

(Compacted 2026-08-19: this file had grown to 838 lines of accumulated
per-plan narrative that belongs in each plan's own Outcomes & Retrospective.
Every fact below is still current; detail on *how* each was found or fixed
now lives only in the referenced ExecPlan and `git log`.)

## Current Snapshot

- Date: 2026-08-24
- Branch: `crossings/verification-foundation`
- Current HEAD: `77677d7`. Working tree clean (one uncommitted docs-only
  close-out edit to this file and `.agent/ORCHESTRATOR.md` pending as
  this note is written). All code work committed, one commit per
  milestone group, per repository owner's direction: `cefcfc3` the n_50
  investigation, `8202f16` plan draft + Milestones 0-1, `70a3c62`
  Milestone 2's design, `f189d11` Milestone 3 via Codex, `fb7442f` state
  sync, `00946a1` Milestone 4, `788dd6e` Milestone 6, `aa13828`
  Milestone 7, `0063736` plan close-out, `77677d7` the follow-up
  SingleNetSearch wiring plan's Milestones 0-2.
- **No active ExecPlan right now.** Both the modular-routing-strategies
  plan and its direct follow-up,
  `.agent/execplans/2026-08-24-wire-single-net-search-trait-into-production.md`
  (wiring `SingleNetSearch` into all 12 of `src/py_router.rs`'s
  production single-net search call sites -- the "deferred, not yet
  characterized" item the first plan left open), are complete -- see
  Completed ExecPlans below for both summaries. The second plan's own
  Outcomes & Retrospective is worth reading for a concrete confirmation
  of the "characterization needs active re-verification" lesson: its own
  Milestone 0 found 12 call sites where the first plan's own deferred-work
  note had estimated 8, because that estimate never actually re-grepped
  three sibling "discard the stats" functions it hadn't characterized.
- Current test baselines: `cargo test --lib` `393 passed, 0 failed` (389 +
  4 new direct tests for `run_windowed_single_net_search`, added during
  the modular-routing-strategies plan's Milestone 7);
  `PYTHONPATH=. .venv/bin/pytest -q` `11 failed, 335 passed, 1 skipped`
  (dropped from the long-standing `21 failed, 325 passed` baseline via
  two 2026-08-20 passes: the batch-repair stale-signature fix, then a
  full triage of the remaining 20 -- see Completed ExecPlans'
  `2026-08-20-pytest-baseline-triage.md` entry. Every one of the
  remaining 11 failures is now individually documented, not just
  "the stable baseline" -- see Current Findings below).
- **Future Architecture Initiative: complete.** `.agent/PROJECT_GOAL.md`'s
  "Future Architecture Initiative" (giving routing-pipeline stages
  explicit `Protocol`/`trait` interfaces), per the recommended order in
  `.agent/execplans/2026-08-19-future-architecture-initiative-stage-characterization.md`:
  (1) obstacle map building + grid snapping -- **done**, see
  `.agent/execplans/2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md`;
  (2) A* single-net search -- **done**, see
  `.agent/execplans/2026-08-19-extract-astar-single-net-search-interface.md`;
  (3) geometry realization -- **done**, see
  `.agent/execplans/2026-08-20-extract-geometry-realization-plm-boundary.md`;
  (4) path-length matching -- **done**, see
  `.agent/execplans/2026-08-20-path-length-matching-interface.md`. All four
  recommended stages now have explicit interfaces. Ripup/repair
  orchestration (`route_many_with_repair_and_commit`, `src/py_router.rs`)
  was explicitly excluded from this initiative throughout -- it needs its
  own restructuring pass (god-object session state, zero stage-granular
  tests, three real bugs found in/near it this session) before interface
  extraction is well-posed; see "Next Engineering Step" below. The three
  benchmark findings under Current Findings below are parked until that
  restructuring gives their surrounding code a clearer structure.
- **Completed ExecPlans** (all still valid, no known regressions; each
  plan's own Outcomes & Retrospective has the full story):
  - `2026-08-24-wire-single-net-search-trait-into-production.md` -- closed
    the one item the modular-routing-strategies plan below deliberately
    deferred: `src/py_router.rs`'s production code called the
    `SingleNetSearch` trait's four free functions (and 3 sibling
    "discard the stats" convenience wrappers) directly, never through the
    trait itself. Census found 12 real call sites across 9 methods (the
    deferring plan's own "8" estimate had missed the 3 convenience
    wrappers entirely). Every site's arguments were confirmed by direct
    read to match its trait method exactly before any code moved;
    dispatched to Codex as a single, fully-specified task (12 exact
    substitutions spelled out site by site) and reviewed line-by-line
    before acceptance -- zero review findings, the cleanest Codex
    dispatch either plan this session needed. Does not add a second
    search algorithm (none exists yet, none was invented to prove the
    point) -- makes the existing one the thing production code actually
    calls through, so a future second implementation changes one seam
    instead of 12 call sites. Full validation ladder clean and
    byte-identical to baseline throughout.
  - `2026-08-24-modular-routing-strategies.md` -- made the restructured
    routing pipeline genuinely modular, not just readable-in-isolation, in
    three phases by risk. Phase A: `_RouteNetsRustSession.run()`
    (`translation/route_rust.py`) went from a ~1070-line single method to a
    37-line ordered sequence of 8 named, documented phase calls (7 new
    private methods); 3 real self-inflicted editing mistakes and 1 stale
    "expected to fail" claim were caught by validating after every single
    sub-extraction, not batching. Phase B: the windowed-bounds-expansion
    retry loop duplicated across all 4 `src/astar.rs` single-net search
    wrapper functions (confirmed 4, not 3 as first assumed, and one is a
    mathematically exact special case of another) is now one shared
    generic helper, `run_windowed_single_net_search`, implemented by Codex
    against a fully-specified task and reviewed line-by-line before
    acceptance; an initial hot-path timing regression turned out to be
    system noise from comparing runs at different points in the session,
    not a real effect (confirmed via a controlled back-to-back comparison).
    Phase C: characterized and documented all 18 `try_*` repair methods in
    `route_many_with_repair_and_commit` (trigger, variation, historical
    motivation, call graph); presented the repository owner with concrete
    design options (a full cross-18-method `RepairStrategy` abstraction, a
    narrow single-pair unification, or stopping after characterization) --
    the owner chose the narrow option, confirming Claude's own evidence-
    based recommendation, since the 18 methods vary along genuinely
    different axes not cheaply unifiable. Implemented that one pair's
    unification directly (not via Codex, a recorded exception: the design
    surfaced a real behavioral difference -- one method updates routing
    history before ripping up its victim, the other doesn't -- that the
    prior characterization's summary-level read had missed, needing
    subtler correctness reasoning than a task file could cheaply capture).
    Full validation ladder clean and byte-identical to baseline throughout
    every one of 8 milestones. Deliberately out of scope, left for a future
    plan: wiring `SingleNetSearch`'s trait methods into `src/py_router.rs`'s
    8 direct call sites; the other 16 of the 18 repair methods;
    `run_routing_flow`'s ~40-parameter signature.
  - `2026-08-20-route-many-with-repair-restructuring.md` -- full structural
    restructuring of `route_many_with_repair_and_commit`, the last major
    routing-pipeline component without explicit structure or real test
    coverage. ~3,035 lines -> 663 lines. Whole-batch, per-net,
    per-repair-attempt, and per-mode-order-iteration state promoted into 4
    named structs (`RepairBatchState`, `RepairAttemptState`, `ProbeState`,
    `RepairModeAttemptState`); every phase from the original 13-phase
    survey with genuine branching complexity, borrow-checker subtlety, or
    historical-bug lineage extracted into one of ~30 new named methods with
    an explicit outcome type (12 new enums). Phases 0, 1, and 13 (job
    unpacking/indexing, the skip-if-already-routed loop entry, and PyDict
    result assembly) deliberately left inline throughout -- straightforward
    sequential bookkeeping, no genuine complexity to name. Every one of 20
    implementation milestones independently reviewed (full diff read, not a
    summary) and validated against the full ladder (`cargo test --lib`, the
    batch-repair integration test, `multiportmmi_8x8`, `benes_4x4`, full
    `pytest -q`) before commit -- byte-identical to the pre-restructuring
    baseline throughout, zero behavior change. Two genuinely new
    Rust-specific traps found and correctly handled along the way: a
    for-loop-over-a-struct-field's-borrow conflicting with a later
    `&mut`-of-the-whole-struct need (fixed by snapshotting the field before
    the loop), and the final phase needing its `RepairAttemptState`/
    `ProbeState` parameters by value instead of by reference because both
    are genuinely consumed as their last use in the function. See that
    plan's own Outcomes & Retrospective for the full story.
  - `2026-08-20-pytest-baseline-triage.md` -- triaged all 20 pytest
    failures remaining after the batch-repair fix. Fixed 6 stale/mechanical
    ones directly (missing kwargs, stale error-message text, a stale
    expected-value literal, a formula input the test never set, an
    unrelated verification gate reached via fake test data). Investigated
    and rewrote one test whose expectation was invalidated by a deliberate,
    already-validated design change over a month ago, not a live bug
    (confirmed via `git blame`, not assumed). Investigated and documented
    (not fixed) 6 more as the same benchmark-placement-fact category as the
    already-documented `TOY` finding -- confirmed via direct reproduction
    that they fail identically under bare CLI defaults, not just under a
    stress-test's settings. Net: `21 failed` -> `11 failed`; every
    remaining failure is now individually explained, not just stable.
  - `2026-08-20-ripup-repair-orchestration-restructuring.md` -- 3 passes at
    the ripup/repair orchestrator (`route_many_with_repair_and_commit`,
    `src/py_router.rs`, ~3,065 lines), the code explicitly excluded from
    the Future Architecture Initiative at every stage. Full characterization
    (dispatched to a fork, read-only, no-decision instructions) found a
    different entanglement shape than expected: not closure-heavy, but
    ~40 named methods all implicitly sharing `PyPhotonicRouter`'s 20-field
    session state. Fixed the one existing, currently-broken focused
    integration test (`tests/test_rust_batch_repair.py`, stale 7- vs
    8-element job-tuple signature). Extracted the victim-set-expansion
    logic into `compute_repair_victim_sets` (`src/py_router.rs:1669`), a
    pure free function needing zero `self` coupling, with 4 new direct
    unit tests. **Then fixed the underlying `n_67`/`n_70`/`n_71` bug
    itself** (see Resolved Findings) -- found the "capture victim's
    collision partner, expand ripup set" mechanism a 2026-08-19
    investigation said was missing already existed and predated that
    investigation; the real gap was a narrow string-prefix mismatch in
    two parsing functions, fixed by recognizing a second message prefix.
    Verified end-to-end (not just unit tests): `multiportmmi_8x8` now
    routes cleanly under bare CLI defaults for the first time since this
    bug was documented, `error_count: 0`. Full validation ladder clean,
    zero regressions anywhere. A full session-state decomposition of the
    rest of the orchestrator remains available future work, not started.
  - `2026-08-20-path-length-matching-interface.md` -- closed out the
    Future Architecture Initiative (stage 4 of 4). Characterization found
    PLM splits into an already-clean graph/requirement half and a
    meander-insertion half that already had more interface infrastructure
    (two private Protocols, a real non-closure session class) than the
    prior stage-characterization pass credited it with. Repository owner
    chose (via `AskUserQuestion`) to promote the existing Protocols;
    mid-design, found the Rust and Python halves were not symmetric (Rust's
    4 `plm.rs` functions had clean public signatures ready for a direct
    trait mirror of `astar.rs`'s `SingleNetSearch` pattern; Python's
    `_MeanderPlannerContext` methods used private internal bookkeeping
    types with no public-shaped equivalent) -- presented back to the
    repository owner as its own scoping decision rather than resolved
    solo. Final result: `src/plm.rs` gained a purely-additive
    `RegisteredMeanderPlanner` trait + `RustRegisteredMeanderPlanner`
    marker struct (each method a direct delegation to the existing free
    function, one new smoke test); `translation/route_rust_meanders.py`'s
    `_RustBackendProtocol`/`_MeanderRouterProtocol` renamed to public
    `MeanderRustBackendLike`/`MeanderRouterLike` with real docstrings. Zero
    behavior change, zero regressions across the full validation ladder.
  - `2026-08-20-extract-geometry-realization-plm-boundary.md` -- resolved
    the geometry-realization/PLM coupling by moving the Auto-meander-search
    layer (the actual shared code, ~1,700+ lines across 3 non-contiguous
    regions) out of `src/geometry_realization.rs` into a new module,
    `src/auto_meander.rs`; `plm.rs` now depends only on that clean module
    boundary, not on realization-proper internals. Fixed an accidental
    `GridRect` name collision found along the way (renamed the moved one to
    `MeanderGridRect`). Added 48 new direct unit tests for code that
    previously had none: 6 for the moved centerline-search core plus 1
    probe-consistency check in `auto_meander.rs`, and 42 for `plm.rs`'s 4
    public planning functions (validation-error coverage plus
    candidate/sequence/split/final-request success paths). Full validation
    ladder run clean: `cargo test --lib` `382 passed, 0 failed`, full
    `pytest -q` byte-identical to the pre-plan baseline, `benes_4x4` and
    `multiportmmi_8x8` stable-baseline both `Verdict: PASS`,
    `multiportmmi_8x8` bare-defaults and `heater_s_mod` PLM regression both
    still fail with their exact pre-existing, already-documented
    signatures -- zero regressions anywhere in the ladder.
  - `2026-08-19-extract-astar-single-net-search-interface.md` -- added a
    purely-additive `SingleNetSearch` trait (`src/astar.rs`, 4 methods, one
    per genuine mode) implemented by a new `AStarSingleNetSearch` marker
    type delegating to the existing 10 free functions; no production call
    site migrated. Also deduped `CrossingSearchConfig` construction (was
    repeated 3x in `src/py_router.rs`) behind a new helper mirroring the
    existing `astar_config` pattern. Flagged two findings for later, not
    acted on: `route_single_net_and_commit_native` reimplements the
    simple-then-full fallback pattern inline instead of reusing the entry
    points that already have it (smaller instance of the same shape as the
    parked ripup/repair god-method); `SimpleRouteObstacleQuery`
    (`src/simple_routes.rs`) confirmed already correctly-scoped, left
    untouched.
  - `2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md` -- added
    `ObstacleMapBuilder` `Protocol`; unified grid-snapping math (was
    duplicated ~10+ times across 7 files in both languages) behind one
    canonical implementation per language.
  - `2026-08-19-future-architecture-initiative-stage-characterization.md` --
    characterized all 5 candidate stages, set the extraction order above.
  - `2026-08-19-fix-open-repair-and-dense-port-findings.md` -- root-caused
    (not fixed) the `n_67`/`n_70`/`n_71` cluster; deliberately left unfixed
    pending the repair-orchestration restructuring.
  - `2026-08-19-restructure-crossing-partner-discovery.md` -- consolidated 7
    overlapping crossing-partner-lookup functions into 2, named the one
    real intentional divergence (`CollisionCrossingTryOrder`).
  - `2026-08-19-fix-collision-crossing-zero-event-acceptance.md` -- fixed a
    zero-crossing-event route being silently accepted as legal, and a
    second bug it surfaced (partial repair-restore silently losing routes).
    This fix is what caused `multiportmmi_16x16` stable-baseline to start
    failing at `n_50` (see Current Findings) -- a net that previously got a
    vacuous "success" from the bug now genuinely can't be routed.
  - `2026-08-19-restructure-port-endpoint-correction.md` (+ Milestone 0.5,
    which fixed a real physical `n_196`/`n_197` waveguide overlap on
    `multiportmmi_16x16`) and its follow-up
    `2026-08-19-collision-avoiding-endpoint-correction.md` (fixed
    `n_196`/`n_203` colliding instead of cleanly failing to connect) --
    both real fixes, both still in effect; superseded as the reason for
    16x16's current failure by the zero-event-acceptance fix above.
  - `2026-08-18-unify-port-access-region-computation.md` -- unified 7
    independently-parameterized port-access/keepout sizing computations
    into one (`_resolve_port_footprint_cells`/`build_port_footprint_cells`).
  - `2026-08-18-dense-port-runway-clearance-reach.md` -- fixed
    `multiportmmi_8x8` `n_24`/`multiportmmi_16x16` `n_48` (sibling
    reservations sealing off a shorter-reach port).
  - `2026-08-18-crossing-aware-endpoint-correction-direction-sequence.md` --
    fixed a `source_endpoint_mismatch` on `multiportmmi_8x8` `n_40`-`n_42`;
    `multiportmmi_8x8` reached fully-clean end-to-end for the first time.
  - `2026-08-18-recalibrate-stale-rust-crossing-tests.md` -- recalibrated 8
    of 9 stale Rust crossing tests; left 1 real bug
    (`collision_crossing_route_without_event_is_not_accepted`) failing on
    purpose until the zero-event-acceptance plan above fixed it.
  - `2026-08-18-stage5-routing-crossing-correctness-walkthrough.md` --
    root-caused the `TOY` benchmark's `gc1_to_mmi_in2` failure to a genuine
    placement/clearance fact (see Current Findings), not a router bug; added
    the `corridor_clearance_*` diagnostic fields to `FAILED.txt` as a
    reusable technique.
  - Three earlier readability plans (`2026-08-11-refactor-python-routing-flow.md`,
    `2026-08-17-restructure-translation-route-rust.md`,
    `2026-08-18-restructure-route-nets-rust.md`) -- split `routing_flow.py`;
    moved ~100 helpers out of `translation/route_rust.py` into topic
    modules; turned `route_nets_rust`'s ~120 anonymous closures into a
    103-method class, `_RouteNetsRustSession`.

## Current Goal

Make TUMPhotonicRouter a very fast, verified photonic router. The active
phase (per `.agent/PROJECT_GOAL.md`) is router-discovered optical crossings on
`benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`, with final-geometry
verification and PDK/gdsfactory crossing component realization. Two
prerequisite priorities were set before more crossing-verification feature
work: (2026-08-17) a readability/structure pass, now done (see Completed
ExecPlans); (2026-08-18) the "Future Architecture Initiative" -- real
swappable interfaces (Python `Protocol`/`ABC`, Rust `trait`) for routing
stages, each independently unit-tested, matching the header/interface/test
discipline the repository owner is used to from C++. That initiative is now
**active** (see Current Snapshot). On 2026-08-19 the repository owner
additionally directed: fix the open benchmark findings *or* explicitly decide
not to (done, see Current Findings) before continuing the initiative, and use
the Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`) for implementation
slices by default once a change is well-specified.

## Current Findings (open, tracked, not active work)

Two of the original three remain parked, pending a real design decision or
restructuring; the third (below, in Resolved Findings) is fixed:

1. **`multiportmmi_8x8` dense-port lateral-width allocation** (only
   manifests with `--ripup-reroute false`; default repair papers over it):
   `_filter_dense_port_opening` splits a dense port group's available rows
   unevenly; a port landing on the narrow end (2 rows, `o9`/`o11` on
   `mmi0_multiport_0_0`) can't execute any bend regardless of forward
   reach (`bend_radius_cells=3`). Needs a design decision (how much lateral
   room a single-direction bend needs, how to redistribute fairly without
   recreating sibling overlap) before implementing -- present options to
   the repository owner first, do not implement solo.
   `multiportmmi_16x16`'s `n_102` looks superficially similar but is a
   *different*, likely-unfixable benchmark-placement problem (tight real
   corridor, not a reservation-logic bug) -- do not conflate the two.
2. **`multiportmmi_16x16` stable-baseline**: fails with `RuntimeError: No
   route found for n_50` (`candidate_blockers=[49, 50]`), caused by the
   zero-event-acceptance fix (bisected: clean at `9302efd`, broken at
   `3bea008`/`HEAD`). Not root-caused to the same depth as the now-resolved
   `n_70` finding was.
   **Checked 2026-08-24, ruled out**: this does *not* share the `n_70`
   prefix-recognition gap. Direct reproduction on current `HEAD` (`034b489`,
   `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, the documented stable-baseline
   command below) shows the failure trace uses only the
   `"Illegal realized crossing"` prefix throughout -- `"Illegal grid
   crossing"` (the message `n_70` needed) never appears -- and the
   victim-set-expansion mechanism the `n_70` fix relies on
   (`enqueue_targeted_illegal_crossing_repair_set`) is already working
   correctly here: `native_repair_keepout net=51 ripup=[49, 50]` is tried
   in both `victim_first`/`reverse` orderings, i.e. net 50's collision
   partner (net 49) is already correctly folded into net 51's ripup set.
   Every combination still fails (`not_perpendicular` crossing between
   nets 50/51, `No legal LiDAR crossing route found` otherwise) --
   confirming this is a genuine, separate repair-exhaustion/geometry
   problem, not a string-matching bug. Error signature is otherwise
   essentially byte-identical to the 2026-08-19 trace recorded in
   `.agent/execplans/2026-08-19-fix-open-repair-and-dense-port-findings.md`'s
   Finding 2. Root-causing to `n_70`'s depth (why no legal arrangement
   exists among nets 49/50/51, corridor-clearance evidence, etc.) is still
   not done -- this session only closed the "does it share the known fixed
   bug" question, not the underlying finding.
   Reproduction command (from repo root, `.venv/bin/python`):
   `rm -rf build/routes build/verification && PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1 PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 0`
   (completes in under 2 minutes without `--attempt-diagnostics`; the
   ~10-minute-and-killed prior attempt was the heavier
   `--attempt-diagnostics` variant, not this one).

## Resolved Findings

1. **`multiportmmi_8x8` bare CLI defaults, `n_67`/`n_70`/`n_71` cluster --
   fixed 2026-08-20**, see
   `.agent/execplans/2026-08-20-ripup-repair-orchestration-restructuring.md`.
   Was: `RuntimeError: No route found for n_70` (`candidate_blockers=[70]`),
   repair-time only -- net 71's repair rips net 70 up as a victim, net 70's
   reroute then hits an illegal grid crossing against net 67, but net 67
   was never folded into the ripup set. The "capture victim's collision
   partner and expand ripup set" mechanism this fix needed
   (`enqueue_targeted_illegal_crossing_repair_set`) already existed, predating
   the original 2026-08-19 investigation that concluded a new one was
   needed -- the real bug was a narrow string-prefix mismatch (two parsing
   functions only recognized `"Illegal realized crossing"` messages, not
   the identically-shaped `"Illegal grid crossing"` message net 70's
   failure actually produces). Fixed by generalizing both to recognize
   either prefix. Verified end-to-end: `multiportmmi_8x8` bare CLI defaults
   now routes cleanly, `error_count: 0` on both crossing and photonic
   verification. Full pytest suite and the `multiportmmi_8x8`
   stable-baseline config both unaffected.

**Idea saved for the eventual repair rewrite** (not acted on):
repair's necessity is not just a net-ordering artifact -- confirmed
`--ripup-reroute false` still allows per-net crossing search, only disables
victim rip-up, and even `benes_4x4`/`benes_8x8` fail immediately without it.
Net order today is raw netlist declaration order, not a heuristic. A
smarter ordering could plausibly *reduce how often* repair needs to fire
(worth carrying into the repair-restructuring plan as a design lever) but
can't eliminate the need for it -- no single static order can be proven to
avoid every conflict for an arbitrary netlist.

**Not planned to be fixed**: `TOY` benchmark's `gc1_to_mmi_in2` fails `No
route found` -- confirmed via direct geometric BFS analysis to be a genuine
clearance shortage at the target port's immediate approach (a bare corridor
exists but disappears once ~2 cells of bend-radius clearance is required),
i.e. a benchmark-placement fact, not a router bug. Do not use `TOY` as a
smoke test.

**Likely the same category, found 2026-08-20 during a pytest-baseline triage,
not yet confirmed to TOY's own BFS-analysis depth**: `heater_s`,
`heater_s_compact`, `mmi_heater_8x4`, and `mmi_heater_8x4_ripup_reroute` all
fail `RuntimeError: No route found for gc_in_1_to_mmi_a_0_lower_in:
gc_in_1,o1 -> mmi_a_0,o1` (source=(31,276,0), target=(104,296,0)); `mmi_heater`
fails the analogous `gc1_to_mmi0_in2: gc_1,o1 -> mmi_0,o1`
(source=(31,36,0), target=(84,56,0)) -- both are first gc-to-mmi-input
connections, same shape and naming pattern as TOY's documented case.
Confirmed these are not an artifact of `test_benchmarks_route_with_astar_only`'s
stress settings (`enable_simple_routes=False`, `waveguide_clearance_um=0.0`):
both fail identically under bare `routing_flow.py <benchmark>` CLI defaults
(no flags). Treated as the same benchmark-placement-fact category as TOY by
inference (same net-connection shape, same unconditional failure across
settings), not by redoing TOY's own BFS-level proof for each -- if picked up
later, get the same depth of geometric confirmation TOY's finding has before
concluding either way. Affects `tests/test_routing_flow_stats.py::test_benchmarks_route_with_astar_only[heater_s|heater_s_compact|mmi_heater|mmi_heater_8x4|mmi_heater_8x4_ripup_reroute]`
and `test_routing_flow_routes_single_heater_electrical_metal_end_to_end`
(6 of the current 11 pytest baseline failures). Do not use these benchmarks
as smoke tests either, for the same reason as `TOY`.

**Stale, low-priority, not re-verified recently**: `multiportmmi_32x32` is
not yet stable; route 156/`n_155` was the last known slow/hanging route to
investigate. `benes_8x8`/`benes_16x16` were last confirmed stable
2026-08-18; not re-checked since.

## Recent Session Notes (durable process lessons)

- The Claude+Codex flow (`.agent/CLAUDE_CODEX_FLOW.md`, added 2026-08-17):
  Claude holds Orchestrator/Planner/Explorer/Reviewer/QA; only the
  Implementation Engineer role goes to Codex CLI via
  `.agent/scripts/codex_task.sh`, for bounded, well-specified slices.
  Default to it once a change is well-specified; verify Codex's own
  claims (diff read in full, validation commands independently re-run)
  before committing, and note direct-implementation exceptions with a
  reason. Two large, older crossing-verification-era ExecPlans
  (`2026-07-10-crossing-verification-foundation.md`,
  `2026-07-06-match-lidar-multiportmmi-routing.md`) have never had a
  staleness audit -- treat their internal detail as unverified.
- **Characterization claims need active verification, not just trust --
  but they aren't always wrong.** Twice, an initial "this looks clean" pass
  understated real structural problems (`2026-08-18-unify-port-access-region-computation.md`'s
  "seven independent sizing computations" was an undercount;
  `2026-08-19-extract-obstacle-map-grid-snapping-interfaces.md`'s "no
  duplication found" missed a formula duplicated across 7 files because it
  only checked named-function duplication, not inline re-derivation).
  `2026-08-19-extract-astar-single-net-search-interface.md`'s own
  Milestone 0 found a third instance (the "4 clean entry points" claim
  undercounted a real 10-function surface) *and* a case where a prior
  claim held up exactly as stated ("no Python-side reimplementation").
  Net lesson: when starting a new extraction/refactor milestone, budget
  time to actively look for what a prior characterization might have
  missed -- but the answer is genuinely "it depends," not "always
  distrust." Verify either way; don't assume the direction of the error.
- Codex reliably stops and reports when a task file's assumption is wrong
  (which tests were pre-existing failures, whether closures were nested
  where a task file assumed) rather than guessing -- a positive signal for
  the role boundary, and a reminder to cross-check task files against
  current source, not memory.
- Two Codex attempts at a merged migration (unify-port-access-region
  plan) both invented a new ad-hoc bolt-on mechanism instead of reusing an
  existing, correctly-composing exemption already in the code; Claude
  implementing directly after reading that mechanism worked cleanly. Codex
  is reliable at fully-specified, bounded slices; less reliable at
  discovering "there's already a mechanism for this" on its own.
- A naive AST "does this name appear again" analysis produced large false-
  positive counts twice (name-shadowing not accounted for) -- budget
  writing this kind of script twice, not once.
- One environment blocker (resolved, unlikely to recur): Codex's `bwrap`
  sandbox failed under Ubuntu 24.04's default AppArmor profile; fixed with
  a scoped `bwrap`-specific profile.
- This machine has real resource constraints: a `multiportmmi_16x16`
  diagnostic run had to be killed after ~10 minutes without completing.
  Prefer `benes_4x4`/`multiportmmi_8x8` for iteration; run multiple Codex
  tasks sequentially, not in parallel, unless they're both small/fast.

## Reference Branches

The branch `baseline/lidar-pure-crossings` contains a WIP prototype snapshot
at commit `69ab9fd` ("wip: snapshot experimental lidar-pure crossing
prototype"). Treat it as reference material only, not a merge candidate. If
code is needed from it: audit the relevant diff, confirm the idea fits
`.agent/PROJECT_GOAL.md` and the active ExecPlan, port the smallest useful
piece manually or via a narrow reviewed cherry-pick, and add focused
verification before considering the port complete.

`.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md` is marked
reference-only in its own status note (added 2026-07-10) unless the user
explicitly resumes it.

## Next Engineering Step

**No active ExecPlan right now.** The `route_many_with_repair_and_commit`
restructuring, the modular-routing-strategies plan, and its
SingleNetSearch-wiring follow-up are all complete (see Completed
ExecPlans). **Next step** (no mandated single choice):
1. `multiportmmi_8x8` dense-port lateral-width allocation -- needs a
   design decision (how much lateral room a single-direction bend
   needs, how to redistribute fairly), present options to the
   repository owner first, do not implement solo.
2. `multiportmmi_16x16` stable-baseline `n_50` -- the "does it share the
   `n_70` prefix-recognition gap" question is now answered (2026-08-24,
   see Current Findings item 2): no, it's a separate, still-unsolved
   repair-exhaustion problem. Next step, if picked up, is root-causing it
   to the same depth as `n_70` (corridor-clearance BFS probe, exact
   geometric reason nets 49/50/51 have no legal arrangement).
3. The other 16 of the 18 `try_*` repair methods in
   `route_many_with_repair_and_commit` -- the repository owner explicitly
   chose not to generalize these further right now (Milestone 5 of the
   same plan); each is individually documented with a `///` doc comment
   if picked up later.
4. `run_routing_flow`'s ~40-keyword-parameter signature
   (`routing_flow.py:753`) -- a real design smell (a config object would
   likely be cleaner), noted during the same plan's drafting but never
   discussed with the repository owner beyond that note; touches this
   repository's public API surface, a different kind of risk than the
   internal-only refactors done so far.

**Correction (2026-08-21): the `round_base_*` re-cloning "optimization
candidate" this section previously named is not real -- retracted.**
It was based on misreading `multiportmmi_8x8`'s `native repair
profile:` debug-timing line: `victims=` (`reroute_victims_wall`, A*
search-and-commit wall time) was mistaken for the actual clone cost, a
separate bucket, `reset=` (`repair_state_reset`). A fresh
`--debug-timing true` run on the fully-restructured function (HEAD
`a8dac59`) shows `reset=0.0045s` against `victims=96.0845s` out of a
110s routing phase -- the clone/reset cost is about 21,000x smaller
than the number cited as evidence for it. Do not restart this specific
idea without first re-measuring `reset=` on whatever benchmark
motivates it. See `.agent/execplans/2026-08-20-route-many-with-repair-restructuring.md`'s
Outcomes & Retrospective and Surprises & Discoveries for the full
correction. The real cost driving `multiportmmi_8x8`'s routing time is
`reroute_victims_wall` itself (A* search time, not state management).

**Investigation findings so far (2026-08-21, via a read-only fork, no code changes made)**:
1. Confirmed `dense_astar`/`route_search_total` stats are only ever
   populated inside success arms (~23 `add_route_result_stats_if` call
   sites in `src/py_router.rs`, every one inside `Ok(route) =>`/
   `Ok(true) =>`) -- failed searches are structurally invisible to
   those stats, wall-clock timing is the only thing that sees them.
2. **Refuted**: genuinely failed final-attempt reroutes are not the
   cause -- isolated via temporary instrumentation (reverted) to just
   0.7% of the `reroute_victims_wall` bucket (0.64s of 90.7s on a
   CLI-matched run with `attempts=174, failures=44`, identical counts
   to the original run).
3. **Found, with direct source evidence**: `try_crossing_aware_victim_reroute`
   (`src/py_router.rs`, Milestone 3.13's own extraction) runs up to
   **three separate A* search attempts per victim** before ever
   reaching the plain fallback -- lidar-seeded
   (`try_route_through_collision_partner_set` with a seeded partner
   set), lidar-direct (`try_route_with_collision_crossings_with_loss`),
   and guided (`try_route_through_collision_partner_set` with the
   current job as sole partner). Each attempt's full wall-clock
   duration is added to `reroute_victims_wall_us` unconditionally, but
   `add_route_result_stats_if` (populating `dense_astar`) is only
   called inside that specific attempt's own nested commit-success arm.
   So for any victim that ultimately resolves via the plain fallback
   (the common case), up to 3 prior expensive-but-uncommitted searches
   may already have run -- real computational work, not just a
   reporting gap -- fully explaining the ~90s-wall-vs-~9s-stats
   disparity (the 9s figure is `dense_astar` summed across the *whole*
   run, all bucket types).
4. **Not yet isolated**: which of the three preliminary strategies
   dominates, and whether the cost concentrates on a few hard nets
   (the "slowest route nets" list in `--debug-timing true` output
   suggests nets like n_69/n_70/n_66/n_67/n_68 as candidates, not yet
   confirmed) or spreads broadly. Needs finer per-attempt
   instrumentation than the current wall-clock buckets provide.

This reframes the optimization question: not "make A* faster" but
"the router tries up to three crossing-aware strategies serially
before falling back to plain reroute, and most of that work may be
thrown away." Whether that's worth changing depends on why those three
strategies exist (likely: try smarter/lower-loss options before the
plain fallback) -- reducing them could trade routing quality for
speed. This tradeoff has not been discussed with the repository owner
yet; do not implement a reduction without that discussion first.

`.agent/execplans/2026-08-20-ripup-repair-orchestration-restructuring.md`'s
and `.agent/execplans/2026-08-20-route-many-with-repair-restructuring.md`'s
own Surprises & Discoveries sections have the full characterization any
of these would build on -- read both before starting rather than
re-characterizing from scratch.

A smaller, optional leftover from the just-closed PLM plan: Option C
(not chosen) would have decomposed `analyze_meander_insertion_for_requirements`'s
795-line body (`translation/route_rust_meanders.py:2263-3057`) into named
phases alongside the interface work. That readability gap is still open
if anyone wants to pick it up -- lower-risk and much smaller in scope than
the ripup/repair restructuring, since the function already takes plain
arguments and returns a plain tuple (no hidden shared-state
capture-analysis risk the way `route_nets_rust`'s closures had).

Other deferred candidates, not in a mandated order:

1. Phase 3 readability pass for the large Rust files (`src/py_router.rs`,
   `src/astar.rs`, `src/geometry_realization.rs`) -- likely superseded in
   practice by the Future Architecture Initiative's own extraction work,
   which touched the same files; revisit whether this is still a separate
   need now that the initiative is done.
2. Continue the Stage 6-8 Python/Rust correctness walkthrough (endpoint
   correction, geometry realization, verification) -- not started, lower
   priority than the items above.
3. Resume the crossing-verification-foundation objective (full-run
   stability across all benchmark sizes) -- last known state above in
   Current Findings; stale, not re-verified this session.
