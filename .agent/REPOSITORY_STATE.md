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

- Date: 2026-09-03 (evening)
- **BOTH 32x32 BASELINES REACHED under the stable defaults (lidar-pure), branch `crossings/verification-foundation`, no PR yet (owner):** multiportmmi_32x32 447/447, 0 failures, 0 repairs, verification 0/0, 490 s; **benes_32x32 320/320, 336 attempts / 16 first-window failures / 0 repairs, 420 crossings, verification 0/0, 945 s** (2026-08-31: net 59/320 after 90 min). Full seven-benchmark timing table, per-net anomaly analysis and the performance findings P1-P5 are in `.agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md` (complete, with retrospective).
- What the evening fixed: benes_32x32.py got its stable block (it had none -- bare runs were window mode) incl. `--max-iterations 20000000` and `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT=1.0` (owner experiment E1: same +-5 halo, higher weight -> no more long verticals 4 cells next to an existing one); two more post-search/kernel inconsistencies of the n_286 family: the grid-level post-search check demanded 5 cells after a crossing (`d9f140f`, now half_size like kernel and realized validator) and blocked-footprint moves skipped halo witnesses, sliding over a second partner's between-cells X (`dd5d043`). Both pinned by unit tests from the real geometry; cargo lib 434/434; pytest baseline; ladder green; mm32 full clean.
- New process assets: `.agent/PATH_INVESTIGATION_HARNESS.md` (mandatory procedure for "net X does not route", written from the day's mistakes), `scripts/path_investigation/` (`grid_um.py`, `gds_runs.py`, `benes32_layer273_testbed.sh` -- a 47-second layer-only testbed reproducing the full run's geometry), `PHOTONIC_ROUTER_TRACE_GRID=1` (grid origin print), the `collision-crossing partner-constraints` trace, per-slot blocker caps, `dense_blocked` on probe cells, and the owner's 3-minute per-net watchdog rule (a hanging net is the finding; 3-hour timeouts are not).
- Performance pass done (2026-09-03 21:30-23:25, `.agent/execplans/2026-09-03-performance-knobs-p1-p5.md`): four rule-free changes (check-only commit in endpoint correction, union-first verification booleans, scanline rasterizer with the same oracle, chain-has-reservations short-circuit) -- wall times before -> after: heater 10.7 -> 5.6 s, mm8 18.4 -> 13.3, benes8 38.4 -> 33.7, mm16 66.6 -> 39.3, benes16 190.7 -> 152.8, **multiportmmi_32x32 490.5 -> 298.2 s, benes_32x32 944.7 -> 815.4 s**; every attempts/failures/repairs count and verification verdict identical; pytest baseline. Refuted: crossing search loss as time driver, small-rect fast path. Remaining cost is structural (expansion count = cost slack vs heuristic + Tier-2 key multiplicity) -- owner decision (predicate-1 simplification shrinks the key).
- (superseded) Next (owner, 2026-09-03 21:30): performance knob P1 -- endpoint correction clones the whole ObstacleMap per candidate (14 sites; 66.6 s of 488 s on mm32, 4.8 of 8 s on heater); replace by a check-only `can_commit_...` with a unit test, then re-measure the seven benchmarks. Then P5 (non-routing phases, 16 % on mm32). Structural search cost (P4: cost slack vs heuristic, Tier-2 key multiplicity) and the predicate-1 simplification remain owner decisions.
- Previous snapshot (2026-09-03 midday):
- Date: 2026-09-03
- **ACTIVE PLAN (afternoon): `.agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md`** -- owner goal on this branch: `benes_32x32` must also route under the stable defaults; that run is the lidar-pure baseline and is to be recorded as such. No PR for now. Milestone 0 (fresh diagnostic run) launched 2026-09-03 ~12:30.
- **Completed the same morning (through Milestone 4, optional follow-up open):
  `.agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md`.**
  `multiportmmi_32x32` under the stable defaults (lidar-pure) routes
  **completely and verification-clean for the first time**: 447/447,
  attempts 447, failures 0, repairs 0, 149 crossings, both verification
  JSONs 0 errors, 469 s (LiDAR reference 2026-09-01: 1257 s). Ladder
  green with fewer attempts/failures/repairs than before (mm16 225/2/0
  vs 404/119/11). cargo lib 431/431; pytest 10 failed / 357 passed
  (documented baseline). GDS snapshots
  `build/routed_multiportmmi_32x32_P2_full.gds` and `..._P2_net287.gds`.
- What fixed the crossing kernel (commits in evidence order): `a6dc419`
  accept clean zero-event routes; `947e76d` the move after an
  end-of-move crossing may re-contact the partner it just crossed;
  `4c623ae` straight-after debt = `crossing_half_size_cells` paid by
  pure straights (owner "Point 2"); and **predicate 2** -- a crossing's
  +-half_size reservation window must be disjoint from the route's own
  earlier windows *inside the search* (`crossing_reject_reservation_overlap`,
  intra-move + node-chain check). The last one is the decisive fix: the
  post-search `crossing_events_have_disjoint_reservations` had been
  silently discarding the otherwise perfect `n_286` route (two crossings
  3 cells apart on the x=3072 descent); the search now descends at
  x=3104 and crosses the fan of parallel diagonals with one straight
  135-degree line, 15 crossings, first attempt.
- Crossing rule audit (in the plan): the ruleset reduces to ONE number
  (`half_size` from the GDS crossing) and TWO predicates -- (1) only
  straight cells inside the +-half_size window on both nets, missing
  straights inserted eagerly by the completion chain; (2) reservation
  windows disjoint (third nets, committed windows as static, own earlier
  windows). Turn primitives never touch a partner (`non_straight` reject,
  pinned by three tests). `required_margin = half_size + bend_runout` (5)
  is a compensating number (exact for 90-degree corners, ~1-2 cells
  over-strict at 45-degree corners) -- its replacement by explicit fillet
  trims is an OPTIONAL simplification, not a correctness fix.
  `min_straight_cells_per_crossing` is a dead config knob (accepted,
  validated, ignored).
- Diagnostics kept (env-gated): `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG`,
  `_CHAIN_DIAG`, `_MOVE_DIAG`, `_PROBE_CELLS`, `_POP_DIAG_BELOW_Y`,
  `_TRACE_CROSSING_{NET,LEVEL1,CANDIDATES,PENDING}`, `_DEBUG_ROUTE_FIRST_NETS`;
  every line carries a per-search `seq`. The collision-crossing
  validation trace prints both `satisfies` sub-conditions. Lesson recorded
  in the plan's retrospective: confirm which attempt (seq, partner-set
  size) produced a failure line before analysing it -- the day's detour
  came from reading a single-partner repair search as the full search.
- Previous snapshot below (2026-09-01):
- Date: 2026-09-01
- **ACTIVE PLAN: `.agent/execplans/2026-09-01-forced-90-degree-route-degradation.md`**
  (owner priority). Landed there: the orthogonal repair fallback is
  disabled by default (commit 3592a6c) -- repair routes carry diagonals
  again (octile excess 1.31/1.20 -> 1.07/1.03), full ladder clean, owner
  visual sign-off on multiportmmi_16x16, and multiportmmi_32x32
  progressed from failing at net 109 to net 156 of 447. The new front is
  a SEARCH-COST problem: net 156's direct path needs ~8-12 crossings and
  the 200 um-per-crossing search loss forces admissible A* to exhaust
  the whole window first (40+ min, zero repair activity). Owner decision:
  try lowering the search loss first (A/B at 50 in flight at handoff),
  fall back to LiDAR-style crossing-count budget escalation. benes_32x32
  is running its first post-fix probe and is already past its old
  net-57 cliff. See that plan's Progress for the full evidence chain,
  in-flight experiment pointers, artifacts, and the open tooling gap.
- Also new since the last snapshot: engine -50% (perf plan complete
  through its measured milestones), heuristics audit round closed,
  LiDAR reference runs (mm16 206 s / mm32 1257 s clean, DRV 0), and the
  resolved CLI-vs-run_routing_flow env mystery (STABLE_ROUTING_ENV is
  only applied by main()).
- Previous snapshot below (2026-08-31):
- Date: 2026-08-31
- **Latest (2026-08-31), `.agent/execplans/2026-08-31-admissible-astar-heuristic-45-degree.md`**:
  the 45-degree router's `heuristic_weight` clamp of 1.25
  (`translation/route_rust.py`) is Weighted A* -- inadmissible, up to 25%
  suboptimal paths, the root cause of the owner-observed wasted geometry
  (chicanes, overshoot-and-return detours). With the clamp at 1.0
  (admissible A*), **`multiportmmi_16x16` routes fully clean for the
  first time ever** (223/223, both verifications 0/0, 182 s), and the
  three pre-existing fanout defects `n_15`/`n_16`/`n_74` vanish with it.
  But the ladder is red at 1.0: `multiportmmi_8x8` gets a physical
  parallel-diagonal waveguide overlap (`n_13`/`n_14`, sibling fanout
  nets on adjacent diagonal cells -- a legality hole the grid model
  cannot see, only the photonic gate catches it), `benes_16x16` gets one
  net of the known `crossing-aware endpoint correction produced no
  realizable centerline` class, `benes_8x8` is 2.2x slower (55 s).
  **Update, same day: 1.0 is now the default** (owner decision). The
  regressions it exposed were themselves latent holes and are fixed
  (commit b52a19a): the A* crossing kernel accepted parallel-diagonal
  halo contacts that produce no crossing event; commit validation missed
  near-parallel realized centerlines closer than the waveguide width
  (`parallel_route_overlap`, width via new `set_route_width_um`); and the
  checked endpoint-correction batch validated each net against stale,
  uncorrected neighbor geometry (now: geometric sweep in
  `realized_dynamic_blockers` -- the cell-ownership prefilter is blind to
  the unowned corner-cell gap between adjacent diagonals -- plus
  remembering each corrected centerline before correcting the next net).
  The 50k `max_iterations` cap for plain 45-degree searches now applies
  only at weights > 1.0. **The `benes_16x16` no-candidate correction hole
  is fixed and its 1.25 pin removed (2026-08-31, same plan):** the
  segment corrector's offset-bump generator returned empty for any
  non-axis-aligned centerline; it now handles mixed segments (bump on the
  terminal axis-aligned run, remainder verbatim) and, as a last resort,
  an `anchored_tilt_scale` candidate (rigid rotate+scale about the fixed
  cut anchor, capped at |scale-1|<=0.2 / 5 degrees, finishing in an exact
  1 um axis-aligned port stub so `validate_target_tangent` holds); the
  segment candidate loop also gained the full-route corrector's old-core
  static exemption. **Every benchmark in the ladder now routes clean at
  the admissible default with no exceptions** (mm_16x16 200 s,
  benes_16x16 309 s at 1.0, benes_8x8 57 s, mm_8x8 52 s, heater 25 s;
  pytest byte-identical 10-failure baseline). Double ladder measured (plan Progress has the
  table): at 1.0 everything green except that benes net; at 1.25
  regression-free (`pytest` byte-identical 10-failure baseline,
  `benes_16x16` 40% faster), while `multiportmmi_16x16`@1.25 now grinds
  1139 s to fail at `n_113` -- the forbidden parallel-diagonal adjacency
  was the illegal "solution" weighted search used to commit. The `n_123`
  fix directions (a)/(b) below are superseded at the default; they matter
  only for weight > 1.0 configurations.
- Branch: `crossings/verification-foundation`, HEAD `e78e8d1`. The three
  newest commits are the PLM plan's work described in the next bullet
  (`0f39ade` arc sampling + GDS record cap, `b9587d9` meander length
  model, `e78e8d1` sampled-arc booking **and** a flow change worth
  knowing: `routing_flow.py` now applies the selected benchmark's
  `STABLE_ROUTING_FLAGS`/`STABLE_ROUTING_ENV` as CLI defaults -- explicit
  flags still win, the applied defaults are printed -- so a bare
  `routing_flow.py <benchmark>` is the stable configuration, not the
  script defaults). Working tree clean. The experiment in
  `translation/route_rust.py` (`PHOTONIC_ROUTER_LAYER_ORDER=span`,
  shortest-span-first within a layer) is committed as `c01b17a`
  (2026-08-30), env-gated and off by default -- this is option (a) of the
  router-fix plan's open decision below; it stays an experiment until
  the owner decides.
- **2026-08-27, `.agent/execplans/2026-08-27-router-fixes-for-crossing-grid-stubs.md`**
  (Milestones 1/1b/1c done -- `8facc36` diagonal halo honoured in the A*
  fast path in both kernels, `4dac768`, `ce4e256`, `242eb16` route indices
  in execution order; Milestone 2 partial -- `bba99e4` threshold-driven
  dense fanout groups + target-anchor correction in the plain pass).
  **Open, owner decision**: the real `benes_16x16` grid-mode blocker is
  planar fan-out ordering (widest-span stub must take the outermost
  lane); options (a) span-ascending order within a layer (the env-gated
  experiment above) or (b) victim escalation in repair -- see that plan's
  Progress. Also open there: the endpoint corrector's missing "shift the
  final run, absorb in the preceding bend" strategy for short run-ins to
  anchors (it only absorbs a y delta into a vertical run).
- **2026-08-28, committed**: `.agent/execplans/2026-08-28-plm-geometry-arc-sampling-and-meander-length-model.md`
  -- Milestone 1 done and ladder-validated: one sagitta-driven arc sampler
  (`arc_samples_per_90_deg`, 1 nm sagitta, gdsfactory-equivalent density; the verifier's self-intersection check got a vectorized bbox prefilter so the denser centerlines cost nothing -- heater stable config 15.7 -> 7.8 s) for primitive bends, endpoint-correction
  jogs (were 4 chords per arc) and meanders (were 8), plus a GDS write cap
  of 4000 vertices per polygon (`translation/gds_write_options.py`) so
  long meanders stay inside the GDSII record limit. Milestone 2 (ladder clean): the
  fill-box meander length model booked (amplitude - r) too little per
  meander -- every matched group with a meander was 58-240 um off in the
  GDS; fixed at the formula (`legs * (A - 4r + pi*r)`), realized group
  mismatch now < 0.05 um
  (`test_heater_s_mod_realized_meanders_leave_no_residual_mismatch`).
  Side effect: `test_heater_s_mod_90_degree_plm_regression[3.0]` passes
  (smaller meanders fit), known pytest failure set 11 -> 10.
- **Latest (2026-08-28)**: `.agent/execplans/2026-08-28-port-adjacent-clearance-waiver.md`
  -- by owner direction the waveguide clearance is now waived by default in
  a run-in corridor at every port (other nets' halos only, never cores), so
  nets serving ports closer than the clearance route side by side.
  `heater_s_mod` 90-degree / 3 um routes 81/81 (`error_count=0`) in both
  crossings-off and lidar-pure modes; full ladder clean (see that plan's
  Milestone 3). Committed as `946dfbd`. Also 2026-08-28: the
  repository owner retired the Codex flow -- Claude implements directly
  (`.agent/ORCHESTRATOR.md`, `.agent/CLAUDE_CODEX_FLOW.md`).
  **Open, owner decision needed**: `test_heater_s_mod_90_degree_plm_regression[3.0]`
  now fails in path-length matching, not routing (see Current Findings
  item 0). The env-gated `PHOTONIC_ROUTER_LAYER_ORDER=span` experiment in
  `translation/route_rust.py` (committed `c01b17a`) predates this work and is unrelated to it.
- Previous snapshot (2026-08-26, evening)
- Branch: `crossings/verification-foundation`
- Current HEAD: `946dfbd` (port-adjacent clearance waiver). Before that: the `feat: pre-place topology-derived crossing grids`
  commit on top of `e6432b2`. Three things happened on 2026-08-26, each
  with its own ExecPlan:
  1. `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
     (committed as `e6432b2`): dense *target* ports now get real,
     length-staggered pre-committed stubs like dense source ports already
     did; `multiportmmi_16x16`'s `n_49` routes; that benchmark now fails
     74 nets later at `n_123` (adjacent-diagonal contention at staggered
     anchors, fix direction (a)/(b) noted in that plan, **not decided**).
  2. `.agent/execplans/2026-08-26-diagonal-crossing-conflict-adjacent-fanout-nets.md`:
     investigation only; root cause of the `n_15`/`n_16` forced-orthogonal
     detour proven, `--proactive-congestion-weight` correlation observed
     but its mechanism deliberately *not* trusted yet. Converged on the
     same root problem as `n_123` above.
  3. `.agent/execplans/2026-08-26-preplaced-crossing-grids-for-benes.md`
     (Milestones 0-3 complete, 4 partly): new opt-in
     `--preplaced-crossing-grids true` mode that builds every Benes
     interstage layer's crossings from the topology as pre-wired grid
     components before routing and routes only crossing-free stubs with
     crossings off. `benes_4x4`/`benes_8x8`/`benes_16x16` all
     `error_count=0` with 2/16/88 crossing components; `benes_16x16`
     routing stage 2.3 s (wall 27.5 s) vs 215 s in `lidar-pure`. Open in
     that plan: PLM pass-through only prepared; repository owner visual
     GDS review of the final lattice geometry pending; and a verifier
     blind spot found on the way -- `verify_photonic_routing`'s
     `min_route_overlap_area_um2=2.0` hides route-route crossings of a
     full waveguide width (a 1.09 um^2 one was observed and fixed at the
     source) -- threshold deliberately left unchanged pending the owner's
     decision, since lowering it changes pass/fail elsewhere.
- Prior to that (2026-08-25/26): the negotiated-repair-engine and
  kernel-unification plans completed (8/8 and 5/6 milestones), plus four
  findings fixed outside any plan (self-intersecting routes, stub
  port-lane reservation zeroed, `max_iterations` cap in the
  require-all-partners search, orthogonal-repair try order) -- see
  Resolved Findings and the Completed ExecPlans list.
- **No active ExecPlan right now.** The repository owner is directing next
  steps turn by turn; see Next Engineering Step for the real current
  candidates.
- Current test baselines: `cargo test --lib` `401 passed, 0 failed`;
  `PYTHONPATH=. .venv/bin/pytest -q` `11 failed, 351 passed, 1 skipped`
  (335 + 16 new `tests/test_preplaced_crossing_grids.py`; the 11 are the
  same pre-existing set (10 since 2026-08-28: `[3.0]` passes after the
  meander length-model fix) -- the four not listed by name anywhere,
  `test_rust_routed_layout_uses_waveguide_geometry`,
  `test_toy_ten_um_bend_radius_does_not_backtrack_on_one_cell_short_s_bend`,
  `test_routing_flow_populates_stats`,
  `test_heater_s_mod_90_degree_plm_regression[3.0]` (2026-08-28: routing
  fixed, now fails in PLM instead, see Current Findings item 0), were re-run at
  `e6432b2` in a throwaway worktree and fail there too).
- **Full benchmark ladder re-run 2026-08-25, after the self-intersection
  fix, confirmed clean end-to-end**: `benes_4x4` (plain defaults) passes;
  `multiportmmi_8x8` bare CLI defaults fails byte-identically to its
  already-documented pre-existing failure (`n_33`/`n_52` endpoint
  correction, see Current Findings); `multiportmmi_8x8` stable baseline,
  `benes_8x8` stable baseline, and `benes_16x16` stable baseline (3.8 min)
  all pass cleanly under the default engine, `self_intersecting_route_count=0`
  where applicable; `multiportmmi_16x16` stable baseline fails byte-
  identically to its already-documented, known gap (`n_49`, see Current
  Findings). Zero regressions and zero surprises anywhere in the ladder.
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
  - `2026-08-25-negotiated-repair-engine.md` -- all 8 milestones complete,
    with a materially honest, narrower-than-originally-hoped outcome
    (stated plainly in its own Outcomes & Retrospective, not glossed
    over). Grew from the repository owner's own challenge to the sibling
    kernel-unification plan's Milestone 5 finding ("reachability is not
    evidence of good design"). Milestones 1-4: replaced raw netlist
    declaration order with real topological net ordering
    (`_topological_net_route_order`, `translation/route_rust.py`) and made
    history-cost application systematic on every commit
    (`add_repair_history_for_route`, `src/py_router.rs`) -- both land
    directly in the still-default dispatch chain. Milestones 5-6: built a
    complete, separate, opt-in negotiated-congestion repair engine
    (`route_many_with_negotiated_repair_and_commit`, behind
    `PHOTONIC_ROUTER_NEGOTIATED_REPAIR=1`) with cascading displacement and
    a real distance/slack conflict-resolution heuristic -- genuinely
    faster than the old chain on `benes_4x4`/`benes_8x8`, but the old
    17-method dispatch chain was **not** deleted and remains the permanent
    default: `benes_16x16` was found (by direct trace, not assumption) to
    need crossing-legality-aware repair strategies the new loop does not
    implement. Milestone 7's own investigation, redirected by the
    repository owner's own physical reasoning (port spreading cannot
    apply to Benes networks; some other spacing rule must already exist),
    found the actual fix for `benes_16x16` instead: `--proactive-congestion-weight`/
    `--proactive-congestion-radius-cells`, a general, already-built,
    live lateral-congestion mechanism that had simply never been turned
    on anywhere. Enabling it (`4.0`/`3`) resolves `benes_8x8`/`benes_16x16`
    (both engines) and `multiportmmi_8x8` outright with zero source
    changes; could not be made a global default (regressed 3 unrelated
    cases, confirmed by direct revert-and-rerun), so it landed as an
    addition to those 3 benchmarks' own `STABLE_ROUTING_FLAGS`.
    `multiportmmi_16x16` remains a known, documented gap under every
    configuration tried -- see Current Findings.
  - `2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md` --
    Milestones 1-5 of 6 complete (Milestone 6, final validation ladder +
    retrospective, deprioritized in favor of the negotiated-repair-engine
    plan above, not abandoned). Unified the two previously-duplicated A*
    search kernels (`src/astar.rs`) into one two-tier kernel
    (`unified_kernel` module: dense-array Tier 1 for never-crossed states,
    sparse Tier 2 for post-crossing-corridor states) with crossing
    legality as a monomorphized, pluggable `CrossingLegalityHook` instead
    of a second ~600-900 line duplicate implementation -- the repository
    owner's own target architecture, recorded verbatim in that plan's
    Purpose. A real correctness bug (`straight_run_cells` not tracked for
    Tier-1 states, causing `benes_4x4` to spuriously reject legal
    crossings) was found and fixed by the benchmark ladder itself, not a
    unit test. Milestone 5 traced and deleted 2 undocumented, off-by-
    default repair experiments (`try_guided_collision_crossing`,
    `try_preemptive_crossing_ripup`) after confirming via call-graph
    tracing that neither had any load-bearing caller or historical
    rationale; that same tracing found 4 of a candidate "6 duplicated
    kernel artifacts" were actually load-bearing core-routing
    infrastructure, correcting Milestone 1's own initial classification
    before any deletion was attempted on those 4. This investigation is
    the direct origin of the negotiated-repair-engine plan above.
  - `2026-08-24-crossing-cost-function-soundness.md` -- three milestones,
    all a direct response to the repository owner's own architectural
    question ("why should crossing-aware A* even behave differently --
    shouldn't it only include crossings and stuff like that?"): (1)
    `require_terminal_straights` is now honored by the crossing-aware
    search kernel, not just the plain one (`src/astar.rs`); (2)
    `bend_weight`/`heuristic_weight` no longer vary with `crossing_mode`
    -- crossing-awareness now only changes crossing-legality reasoning
    and `crossing_loss` (re-tuned `50.0` -> `200.0`, empirically
    validated against real crossing counts), not unrelated cost tuning;
    (3) full validation ladder run, one real illegal-geometry-acceptance
    bug found and fixed along the way (a missing port-anchor check in
    the endpoint-correction cascade, commit `2755a22`) that was not part
    of the plan's original scope. Two things remain open by explicit
    repository owner instruction ("keep the fix, document n_67 as
    blocked, move on" then "finish the current plan, then park the
    rest"): `multiportmmi_8x8` bare-defaults' `n_67` now fails endpoint
    correction honestly instead of silently producing bad geometry, and
    the endpoint-correction subsystem's deeper structural issues are
    recorded as a Next Engineering Step candidate, not yet a written
    plan. See that plan's own Surprises & Discoveries, Decision Log, and
    Outcomes & Retrospective for the full evidence trail.
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

0. (Resolved 2026-08-28) `heater_s_mod` 90-degree / 3 um path-length
   matching "no meander candidate": a consequence of the meander length
   model over-length (see Resolved Findings); passes since the model fix.
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
2. **`multiportmmi_16x16` stable-baseline**: **routes fully clean with
   `PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT=1.0`** (2026-08-31, see the
   admissible-heuristic plan in Current Snapshot -- the greedy 1.25
   Weighted A* was the common root of the `n_50`/`n_49`/`n_123` failure
   chain). At the default 1.25 the historical record below still applies:
   fails with `RuntimeError: No
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
   **Re-checked 2026-08-25** (`.agent/execplans/2026-08-25-negotiated-repair-engine.md`
   Milestone 7): still fails identically at `n_49` with the exact command
   above (unchanged `STABLE_ROUTING_FLAGS`, no `--proactive-congestion-*`).
   Tried enabling `--proactive-congestion-weight`/`--proactive-congestion-radius-cells`
   (the mechanism that fixes `benes_8x8`/`benes_16x16`/`multiportmmi_8x8`
   outright, see the plan entry above) at `4.0`/`3`: still fails, different
   net (`n_50`); at `6.0`/`5`: still fails, yet another net (`n_85`). Each
   parameter change relocates the failure rather than converging toward
   zero -- a materially different signal from under-tuning, consistent
   with this benchmark being genuinely denser than the other three in a
   way this one mechanism does not fully resolve alone. Not added to this
   benchmark's own `STABLE_ROUTING_FLAGS` as a result (it does not
   actually make the benchmark route cleanly). Likely needs the
   crossing-aware-rerouting-inside-negotiation work identified and
   deferred in that same plan's Milestone 6, on top of more spacing
   tuning, not either alone -- not attempted further per the repository
   owner's explicit instruction to stop guessing rather than keep
   spending ~8-minute runs on parameter search.

## Resolved Findings

- **PLM meander length model booked (amplitude - r) too little per meander
  (found and fixed 2026-08-28, `.agent/execplans/2026-08-28-plm-geometry-arc-sampling-and-meander-length-model.md`).**
  `MeanderTurnModel::inserted_extra_length_um` counted one vertical leg per
  U-turn; the geometry has one per U-turn plus the entry leg. Every planned
  meander was physically A - r longer than booked (58-240 um on
  `heater_s_mod`), nothing measured the realized meander, and the report
  said residual 0. Fixed at the formula and its inversion; realized group
  mismatch < 0.05 um; also resolved the 3 um configuration's PLM failure.
  Same plan: every realized arc sampled at 1 nm sagitta (was 4-16 chords
  per quarter turn), GDS polygons split at 4000 vertices on write, the
  verifier's O(n^2) self-intersection loop given a bbox prefilter.
- **Port-adjacent clearance blocked the sibling port (found and fixed
  2026-08-28, `.agent/execplans/2026-08-28-port-adjacent-clearance-waiver.md`).**
  `mmi_a_0`'s inputs are 1.25 um apart; with 3 um clearance the first net's
  keepout covered the sibling port's cell and whole approach. Fix (owner
  direction: waiving clearance near ports is the default): the per-net
  exempt set (`route_dynamic_clearance_exempt_cells`, `src/py_router.rs`)
  is now a keepout-radius box plus a run-in corridor along the port axis
  following the net's own opened cells; `opened_cells_without_dynamic_overlap`
  keeps exempt halo-only cells; lidar-pure treats a halo-only probe conflict
  as a spacing problem (skips the collision-crossing search, takes the
  ordinary A*) instead of a collision needing repair -- a no-op at clearance
  0 by construction. New trace `PHOTONIC_ROUTER_TRACE_PLAIN_ROUTE_NET`.

1. **A routed net's own path could cross itself, undetected -- fixed
   2026-08-25**, not part of any ExecPlan (found via the repository
   owner's direct visual inspection of a GDS, `build/routed_multiportmmi_8x8.gds`,
   while investigating fanout lane spacing under
   `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS=6` -- see the
   negotiated-repair-engine plan's own Milestone 7 for that spacing
   investigation, separate from this finding). Net `n_31` needed to
   legally cross four other nets in a tight vertical corridor
   (`x~750`, `y` 148 to 182, ~12-14 cells apart) and the raw A* result
   looped back and crossed its own earlier diagonal segment before
   continuing to its target -- confirmed via direct `shapely`
   `Polygon.is_valid`/`LineString.is_simple` inspection of the rendered
   GDS geometry and the raw `route_obj.cells`, not assumed. Root cause:
   this codebase's A* search state is `(x, y, angle)`, so revisiting a
   cell at a different heading is a legitimate, distinct state (real
   crossings need exactly this), but nothing anywhere -- search,
   crossing legality, endpoint correction, or verification -- ever
   checked whether the *resulting physical path* crosses itself. Fixed
   in two parts, both committed: (1) `src/astar.rs`'s
   `polyline_self_intersects`, rejecting a self-crossing candidate at
   every place a `RouteResult` gets constructed (3 sites), treating it
   as an ordinary "no route found" so the existing dispatch/repair
   fallbacks take over (commit `a4baba7`); (2)
   `translation/photonic_verification.py`'s `_verify_self_intersecting_routes`,
   checking the final corrected centerline as defense in depth, since
   endpoint correction could in principle introduce the same class of
   defect downstream of the search (commit `8fdd435`). Both checks are
   deliberately more permissive than the textbook "is this polyline
   simple" definition: a route revisiting an *exact* earlier vertex
   (e.g. a one-cell overshoot-and-return to satisfy a required terminal
   heading) is legitimate and not flagged -- the first version of the
   Rust check was too strict, broke
   `test_rust_batch_repair_rips_and_reroutes_dynamic_blocker`, and was
   refined before landing; see `a4baba7`'s own commit message for the
   exact distinction (transversal crossing / T-junction / collinear
   overlap of more than a point, vs. a shared-vertex touch). Verified
   end-to-end: the real repro case (`multiportmmi_8x8`, reduced fanout
   lane spacing) now routes 111/111 with a genuinely different path for
   `n_31` and zero invalid polygons in the GDS, instead of silently
   committing the loop; full benchmark ladder re-run afterward (see
   Current Snapshot) shows zero regressions anywhere.

2. **`multiportmmi_8x8` bare CLI defaults, `n_67`/`n_70`/`n_71` cluster --
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

3. **Dense-source-fanout stub port-lane reservation tuned to zero -- landed
   2026-08-26**, not part of any ExecPlan (a direct follow-on to the
   `n_31` self-intersection fix above, found while the repository owner was
   visually inspecting the same GDS region for an oversized red port-
   keepout area). `_resolve_port_footprint_cells`
   (`translation/route_rust.py`) returns `(length_cells, half_width_cells)`
   controlling how many forward grid steps get reserved past each port,
   and how wide (full square side `2*half_width+1`) each step's stamp is
   (`route_collect_inflated_step_cells`, `src/py_router.rs:1193-1271`);
   real forward reach is `length_cells + half_width_cells`, not
   `length_cells` alone (the width-square is stamped at every step,
   including the last). At a dense-source-fanout instance (a multi-port
   MMI splitter with several stubbed ports stacked closely, e.g.
   `multiportmmi_8x8`), this general per-port reservation, sized for a
   port whose own waveguide isn't committed yet, was overlapping heavily
   across stacked ports and merging into one large blocked region visible
   in the rendered GDS. A stub's waveguide, unlike a general port's, *is*
   already committed static geometry by the time this reservation would
   matter, and legality of anything placed at its exit (including a
   crossing) is governed separately by the crossing-legality rules
   (`crossing_half_size_cells`, `min_straight_cells_per_crossing`), not by
   this reservation -- so there was nothing left for it to protect at
   stubs specifically. Landed in three steps, each validated on
   `multiportmmi_8x8`'s stable baseline (still 111/111 routed, 0 errors,
   0 warnings on both crossing and photonic verification, `PYTHONPATH=.
   .venv/bin/pytest -q` unaffected -- same 11 pre-existing failures, 335
   passed, throughout): (1) added a scoped override,
   `PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS`/
   `PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS`, applying only to
   `_is_dense_source_fanout_instance` instances, confirmed via a real
   regression (`n_76`, an unrelated heater port) that a *global* reduction
   is unsafe -- other ports still need the full general formula; (2)
   promoted `half_width_cells=2` (down from the general formula's
   `bend_radius_cells + commit_radius_cells + 1`) to the real default,
   shrinking the merged keepout region for 6 stacked ports from 21 to 17
   cells tall; (3) this entry -- promoted both knobs to `0` (no
   reservation at all at stubs), after confirming `half_width=0` alone,
   then both `half_width=0` and `length=0` together, each pass clean.
   Commits: `5f6fd0f` (scoped override), `b776541` (`half_width=2`
   default), and the commit alongside this entry (`0`/`0` default).

4. **`try_route_through_collision_partner_set`'s uncapped
   `max_iterations` was the dominant cost in repair -- fixed 2026-08-26**,
   not part of any ExecPlan (direct follow-on from the repository owner
   asking why routing was much slower than LiDAR's reference timing).
   Confirmed via direct instrumentation (temporary `eprintln!`s under the
   existing `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`/
   `PHOTONIC_ROUTER_TRACE_CROSSING_NET=<net>` flags, kept as permanent
   per-attempt timing diagnostics in `try_crossing_aware_victim_reroute`
   since they're cheap and match the function's existing gated-`eprintln!`
   style): on `multiportmmi_8x8`'s stable baseline, of `astar_loop=113.5s`
   total A*-loop time, `repair_total=96.9s` (85%) was repair, and
   `victims=95.4s` of that was two single searches on one net (`net 32`,
   `47.9s` and `36.6s`) inside `try_route_through_collision_partner_set`
   (the "seeded"/"guided" strategies in the 3-strategy victim-reroute
   cascade), both of which **failed** after exhausting the search space.
   Root cause: this function requires the found route to cross *every*
   partner in its set simultaneously (`require_all_partners=true`,
   `src/py_router.rs`); when that joint constraint is infeasible, A* has
   no early-exit signal and searches up to the uncapped
   `max_iterations=5,000,000` before giving up. The same partner set
   handed to the sibling "collision" strategy
   (`try_route_with_collision_crossings_using_primitives`, which does
   *not* require all partners) failed or succeeded in `0.1s-1.0s` on the
   identical net. Across every victim captured, "seeded"/"guided" never
   won once -- it only ever burned time before falling through to a
   cheaper strategy that actually resolved the net. Fixed by capping
   `crossing_search_cfg.max_iterations` to `500_000` inside
   `try_route_through_collision_partner_set` (10x lower than the default,
   still ~15x more headroom than any observed real success, which
   completed in under 30,000 expanded states). Verified end-to-end, all
   clean (`error_count=0`, `warning_count=0`, full route count unchanged)
   on the three benchmarks that exercise this path: `multiportmmi_8x8`
   stable baseline `121-140s -> 53.7s` total; `benes_8x8` stable baseline
   unaffected (`28.9s`, already fast); `benes_16x16` stable baseline
   `392.4s -> 215.2s` total. `cargo test --lib` `401 passed, 0 failed`
   and `PYTHONPATH=. .venv/bin/pytest -q` `11 failed (same pre-existing
   set), 335 passed, 1 skipped` both unaffected. `multiportmmi_16x16`
   (the known, separate `n_49`/`n_50` gap; see Current Findings entry 2)
   not re-checked as part of this fix -- it already fails before reaching
   a stable baseline, orthogonal to this timing finding.

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
investigate.

**Correction (2026-08-24, later the same day)**: the "Re-verified clean,
2026-08-24" entry immediately above used the wrong configuration. Both
`benes_8x8` and `benes_16x16` document their own "Stable crossing-router
baseline" in their benchmark files (`benchmarks/benes_8x8.py:23-25`,
`benchmarks/benes_16x16.py:23-25`): `--crossings true --crossing-mode
lidar-pure`. The bare-defaults command recorded above does not supply
this (`enable_crossings=False` by default) and is not representative --
caught directly by the repository owner ("i mean benes 16x16 clearly has
a lot of crossings"). Re-verified with the correct configuration as part
of `.agent/execplans/2026-08-24-crossing-cost-function-soundness.md`'s
Milestone 3: `benes_8x8` (`PYTHONPATH=. .venv/bin/python routing_flow.py
benes_8x8 --crossings true --crossing-mode lidar-pure`, `25.2s`, both
verifications `error_count=0, warning_count=0`) and `benes_16x16` (same
command with `benes_16x16`, `392.4s`, both verifications `error_count=0,
warning_count=0`). No regression relative to 2026-08-18 either way, but
the `252.9s` figure below should not be treated as a valid baseline for
this configuration -- `392.4s` is the first real timing recorded for the
correct, crossing-enabled configuration.

**Re-verified clean, 2026-08-24, superseded by the correction above**
(kept for the record, not for reuse): `benes_8x8` (`PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8`,
22.2s, both verifications `error_count=0, warning_count=0`) and
`benes_16x16` (same command with `benes_16x16`, both verifications
`error_count=0, warning_count=0`) -- `benes_16x16` takes 252.9s (~4.2
minutes). This was run bare-defaults, not the documented stable-baseline
configuration; see the correction above.

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

**Roadmap set by the repository owner (2026-08-31), in order:**

1. **32x32 baseline measurement -- CONCLUDED (2026-08-31, two runs):**
   with the full day's engine gains (-50%) and legality fixes,
   `benes_32x32` reaches **net 59 of 320 in 90 minutes** (timeout):
   the repair loop CAN break each adjacent-diagonal contention after
   ~40-50 min of grinding (net 57 eventually fell), but the dense layer
   repeats the same pattern net after net -- structurally unusable at
   this scale regardless of engine speed. This is the recorded baseline
   number and the quantified motivation for both contributions
   (preplaced crossing grids: the 16x16 grid mode routed its whole
   lattice in 2.3 s routing time; crossing-guided A*: the search front
   balloons against the 200 um search loss, up to 2.8M expansions per
   net). The remaining baseline-level alternative is the owner-gated
   lateral-diagonal repair strategy. **Owner correction (2026-08-31,
   same day, supersedes the contribution framing): 32x32 under
   lidar-pure is a BASELINE MUST** -- "eigentlich ist das ein muss, dass
   auch die 32x32 so laufen". Agreed path: reproduce a 32x32 run in the
   reference router (~/Documents/Repositories/working/LiDAR, which
   claims the capability and ran the 16x16 cases), take the learnings
   ("unsere engine sollte nicht so unterschiedlich sein"), and close the
   baseline gap; if that fails, try other approaches until it works. A
   partial failure-state GDS (through net 56, via
   --debug-stop-after-route) was produced for owner inspection
   (`build/routed_benes_32x32_partial_net56.gds`).
   **LiDAR reference results (2026-08-31/09-01, first-ever runs of these
   cases in that repo):** `multiportmmi_16x16` routes clean in 206 s
   (DRV 0, 30 crossings; our engine: 98 s clean -- we are 2x faster
   there); **`multiportmmi_32x32` routes COMPLETELY clean in 1257 s
   (DRV 0, DRV_path 0, 50 crossings, WL 15200 um)** -- the feasibility
   proof the owner asked for. Logs: scratchpad lidar_mm16.log /
   lidar_mm32.log of session afcc7efa; GDS artifacts under
   LiDAR/src/picroute/result/LiDAR/main_results/.
   **Our multiportmmi_32x32 counterpart run (overnight 2026-09-01):
   fails at net 109/447 with a clean, fast RuntimeError** -- `n_108`
   (`mmi0_ps_array_0_heater_13,o2 -> mmi0_multiport_0_0,o14`,
   `candidate_blockers=[108]`, "No legal LiDAR crossing route found"):
   the adjacent-diagonal fan-in class at the FIRST dense multiport
   group, the exact o13/o14 analogue of 16x16's old n_123/n_124 --
   fixed at 16x16 density by the admissible default, returning at
   32x32 density. Unlike benes there is no grind: fully-diagnosed
   failure (logs mm32_ours*.log, session afcc7efa). LiDAR routes THIS
   SAME topology clean, o14 groups included -- the engine-difference
   analysis has a precise, reproducible A/B. Next: analyze the engine
   difference; first candidate: LiDAR's A* has no perpendicularity
   constraint (per-node crossing_budget instead) and uses 10 ripup
   rounds with history costs and topological net order. Note benes has
   NO LiDAR counterpart (TUM-only topology; a TUM-benes -> LiDAR-yml
   export bridge is feasible since our importer knows the format).
   Evidence: scratchpad
   b32_retry2*.log of session afcc7efa. Original first-probe record:
   `benes_32x32` with the inherited benes_16x16 stable config reaches
   roughly net 57 of 320 in ~35 minutes and then thrashes in repair on
   the adjacent-diagonal class (`native_repair_probe net=57
   allowed_partners=12 crossing_events=3 realized_violations=9`, all
   `not_perpendicular`, victims ripped across the whole layer,
   `source_layer_center_out` reroutes) with no forward progress for 20+
   minutes -- aborted per the owner's rule (no progress -> engine work
   first). Evidence: `scratchpad/p32b_out.log` / `p32b_diag.log` of
   session afcc7efa (2026-08-31). An earlier blind attempt burned 77
   min CPU without finishing. So the baseline at 32x32 is not
   practically routable at current engine speed; `multiportmmi_32x32`
   not yet attempted this round (stale note: slow/hanging at n_155).
   Two harness gaps found on the way: the native route batch emits no
   progress signal at all (needs an env-gated per-net progress print),
   and `kernel.perf_event_paranoid=4` on this machine blocks all
   unprivileged perf profiling (needs a one-time
   `sudo sysctl kernel.perf_event_paranoid=1` from the owner before
   profiling sessions).
2. **Engine performance** on the baseline — the owner: "wir müssen auf
   jeden Fall die Engine noch schneller bekommen."
3. **Contributions on top of the baseline, deliberately NOT part of it**
   (owner, verbatim intent: the baseline must stay clean so these
   measure as deltas against it): (a) precomputed/preplaced crossing
   structures (partially implemented, opt-in
   `--preplaced-crossing-grids`); (b) crossing-guided A* — use the
   precomputed expected crossings to guide the search, e.g. expected
   crossings get zero crossing cost so the intended path is reinforced.
   Neither may leak into baseline defaults or baseline
   `STABLE_ROUTING_FLAGS`.

**No ExecPlan is actively being driven by an agent right now (2026-08-30)
-- the repository owner is directing next steps turn by turn.** Live
threads, all waiting on an owner decision (see Current Snapshot for the
pointers): (1) `2026-08-27-router-fixes-for-crossing-grid-stubs.md` --
planar fan-out ordering for `benes_16x16` grid mode, option (a) span-first
(env-gated experiment, committed off by default) vs (b) repair victim escalation, plus the
corrector strategy for short run-ins to anchors; (2)
`2026-08-26-preplaced-crossing-grids-for-benes.md` -- verifier
`min_route_overlap_area_um2=2.0` blind spot, PLM pass-through,
retrospective; (3) `2026-08-28-port-adjacent-clearance-waiver.md` --
its original acceptance check now passes since the meander length fix,
its own checklist still says blocked; (4) the `multiportmmi_16x16`
`n_123` adjacent-diagonal fix (two directions, not yet decided).

`.agent/execplans/2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md`
and `.agent/execplans/2026-08-25-negotiated-repair-engine.md` are both
complete (5/6 and 8/8 milestones respectively) -- see Completed ExecPlans
above for the full story. The real open candidates right now:

-1. **PLM meander length model: fixed (2026-08-28).** See
   `.agent/execplans/2026-08-28-plm-geometry-arc-sampling-and-meander-length-model.md`.
   `heater_s_mod`'s stable configuration is
   `benchmarks/heater_s_mod.py::STABLE_ROUTING_FLAGS` (90-degree, r=10,
   PLM + outputs, heater obstacles, electrical, 3 um clearance, crossings
   off) with the end-to-end test
   `test_heater_s_mod_stable_configuration_routes_matches_and_wires`;
   `routing_flow.py <benchmark>` applies a benchmark's stable flags as
   its CLI defaults. The meander model books the sampled chord length, so
   matched paths are equal in the GDS to float precision (verified by
   polygon area / width on `mmi_a_0 -> mmi_b_0`: 596.084 vs 596.088 um,
   heater waveguide 320.000 um). The lower path's 20 um detour in that
   pair is the foreign-port keepout at 0 um clearance and the waveguide
   clearance at 3 um -- both intended; owner chose to keep the keepout
   default (2026-08-28). **Owner decisions pending**: (1) should
   routed electrical metal treat waveguides as obstacles (today it crosses
   them freely, 161 crossings in the stable heater GDS) -- see that plan's
   Surprises; (2) sibling nets leave an MMI at its 1.25 um port pitch and
   run parallel ~35 um before bending away -- acceptable, or bend away
   right after the port lane?
0. **Port-adjacent clearance waiver: done (2026-08-28, commit `946dfbd`), one
   open question.** Routing fixed and ladder-validated (see Resolved
   Findings). Open: the 3 um configuration's path-length matching failure
   (Current Findings item 0) -- owner to decide whether it is worth
   pursuing. Committed as `946dfbd`; the unrelated pre-existing
   `PHOTONIC_ROUTER_LAYER_ORDER=span` experiment stays uncommitted in
   `translation/route_rust.py`.

1. **`multiportmmi_16x16` stable-baseline still fails.** See Current
   Findings above (updated today) for the exact reproduction command and
   what has already been ruled out (proactive-congestion tuning alone
   relocates the failure rather than resolving it). Most promising
   untried direction: integrate crossing-aware rerouting into
   `route_many_with_negotiated_repair_and_commit`'s displacement logic
   (`try_negotiated_displacement`, `src/py_router.rs`) -- identified and
   explicitly deferred in `2026-08-25-negotiated-repair-engine.md`
   Milestone 6's Surprises & Discoveries, on the reasoning that a
   displaced blocker's plain reroute has no way to satisfy a specific
   crossing angle/straight-margin constraint. Not yet attempted.
2. **Kernel-unification plan's own Milestone 6** (final validation ladder,
   `.agent/REPOSITORY_STATE.md` update, retrospective) remains open --
   deprioritized in favor of the negotiated-repair-engine plan, not
   abandoned. Low risk to resume whenever there is time for it; the
   kernel itself has been stable and validated throughout every plan
   since.
3. `.agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md`
   (written 2026-08-24) is still paused mid-Milestone-4 (accepted the
   tier-2 removal's larger-than-measured consequence -- 5 nets newly fail
   in `multiportmmi_8x8` stable-baseline, not the 1 originally measured,
   see that plan's own Surprises & Discoveries), at the repository
   owner's direct request to prioritize getting benchmarks to actually
   route first (2026-08-25 direction: "the big problem for now is that
   the routing does not work... i do not really care about something
   like endpoint correction if we can not route these benchmarks").
   Revisit once the routing-correctness work above is in a good place,
   not before.
4. `.agent/execplans/2026-08-25-python-rust-linting-and-coding-standards.md`
   (written 2026-08-25): adopt `ruff`/`mypy`/`cargo clippy`/`cargo fmt`,
   add a coding-standards doc, wire CI. Not started; a baseline scoping
   pass already found 509 ruff findings, 235 clippy warnings, 137
   `cargo fmt` diff blocks, and 246 mypy errors -- see that plan's own
   Surprises & Discoveries. Lower priority than the routing-correctness
   items above per the same 2026-08-25 direction.

The rest of this list is preserved for context and for whatever gets
picked up next.

1. **Endpoint-correction cascade** -- now `.agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md`,
   the active plan. `_apply_crossing_aware_endpoint_correction_to_record`
   (`translation/route_rust_endpoint_correction.py`) is a multi-tier
   fallback cascade (a checked per-segment tier, then an unchecked
   rich-correction-plus-splice tier tried across 4 candidate modes, that
   tier itself falling back through an "absorbed-terminal" solver to a
   raw, uncorrected baseline), called from three separately-evolved
   pipeline call sites (`_finalize_routing_results`,
   `_repair_final_illegal_crossings`, `_repair_final_photonic_issues`,
   all in `translation/route_rust.py`). One real bug in this cascade was
   found and fixed this session (a missing port-anchor check in the
   unchecked tier, `translation/route_rust_endpoint_correction.py`,
   commit `2755a22`) -- but the cascade's deeper structure is unresolved
   and produces real, visible symptoms: `multiportmmi_8x8` bare-defaults'
   `n_67` is now a documented, understood, currently-blocked endpoint-
   correction failure (see the ExecPlan's Surprises & Discoveries for the
   full trace); a scratch diagnostic found 11 of 111 routed nets in the
   same benchmark overshoot their own source/target port y-range by more
   than 5 micrometers; and the repository owner independently
   screenshotted two visible symptoms from `build/routed_multiportmmi_8x8.gds`
   -- a bowtie/hourglass geometry pinch where a route crosses a component
   footprint, and a spurious route loop near heater components -- both
   plausibly connected to this same cascade. The repository owner's own
   proposed direction: a crossing net's port-to-first-crossing and
   last-crossing-to-port segments should use exactly the same checked
   correction logic a crossing-free net already uses, with no separate
   weaker cascade; the checked, per-segment tier already partially
   implements this but currently fails to cover every case (a too-short
   segment when a crossing sits very close to a port, and an unexplained
   native-corrector rejection on a longer segment that was not root-
   caused). Full context: `.agent/execplans/2026-08-24-crossing-cost-function-soundness.md`'s
   Surprises & Discoveries and Decision Log.
2. `multiportmmi_16x16` stable-baseline repair-exhaustion failure --
   still open, but the specific net changed (2026-08-24, found while
   validating the crossing-cost-function-soundness plan above): the
   failure signature moved from `n_50` (`candidate_blockers=[49, 50]`,
   the finding `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md`
   was investigating) to `n_49` (`candidate_blockers=[49]`, a different
   `recent_errors` trace -- see that ExecPlan for the exact text). Not
   root-caused further per the repository owner's "park the rest"
   instruction; whoever resumes that plan's paused Milestone 1 needs to
   re-establish which net (or both) is the actual root blocker before
   continuing the `n_70`-style investigation it already started.
3. `multiportmmi_8x8` dense-port lateral-width allocation -- needs a
   design decision (how much lateral room a single-direction bend
   needs, how to redistribute fairly), present options to the
   repository owner first, do not implement solo.
4. The other 16 of the 18 `try_*` repair methods in
   `route_many_with_repair_and_commit` -- the repository owner explicitly
   chose not to generalize these further right now (Milestone 5 of the
   same plan); each is individually documented with a `///` doc comment
   if picked up later.
5. `run_routing_flow`'s ~40-keyword-parameter signature
   (`routing_flow.py:753`) -- a real design smell (a config object would
   likely be cleaner), noted during the same plan's drafting but never
   discussed with the repository owner beyond that note; touches this
   repository's public API surface, a different kind of risk than the
   internal-only refactors done so far.
6. A functional-verification test suite -- the repository owner proposed
   (2026-08-24) small, edge-case-targeted regression tests that can be
   rerun on demand and eventually gated in CI (e.g. GitHub Actions), as a
   durable verification pass rather than the current ad-hoc pattern of
   manually running a benchmark and inspecting a GDS/screenshot. Agreed
   in principle as the right next investment (matches this repository's
   own stated priority: a readable, tested, well-structured repo, not
   just passing benchmarks), explicitly deferred to its own properly-
   scoped ExecPlan once the queue above clears, not folded into any
   single item above.

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
