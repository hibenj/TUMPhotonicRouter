# Engine performance on the baseline router: measure first, then remove the dominant costs

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

The owner's roadmap (recorded in `.agent/REPOSITORY_STATE.md`, Next Engineering Step) is baseline -> engine performance -> contributions (preplaced crossing structures; crossing-guided A*). The 32x32 baseline probe showed why performance comes now: `benes_32x32` reaches roughly net 57 of 320 in ~35 minutes and then thrashes in repair with no forward progress -- the baseline is not practically routable at this scale at current engine speed. This plan makes the engine faster WITHOUT using topology-derived crossing knowledge (that is contribution territory and must not leak into the baseline, per the owner's explicit direction).

A person sees this working as: the same benchmark commands route the same benchmarks to the same clean verifications, in materially less wall time, with the routing stage's own timing summary as the measure; and long runs report live per-net progress instead of hours of silence.

## Progress

- [x] (2026-08-31) Plan created. Profiling unblocked: the owner set `kernel.perf_event_paranoid=1` (was 4, which forbids all unprivileged perf sampling). An autonomous harness is reproducing the `benes_32x32` net-57 repair thrash and capturing a 90 s `perf record -g` profile of exactly that workload (`scratchpad/profile_stall.sh` of session afcc7efa; outputs `benes32_stall_report.txt` / `_flat.txt`).
- [x] (2026-08-31) Milestone 0 (visibility): resolved by discovery, not code -- `PHOTONIC_ROUTER_NATIVE_PROGRESS=1` ALREADY exists in `route_many_with_repair_and_commit` (`src/py_router.rs`, `trace_native_progress`): per-net `native_route_start index=i net_id=...` plus `native_route_elapsed` lines on stderr. It was simply never used or documented in any run recipe. Use it in every long run from now on.
- [x] (2026-08-31) Milestone 0b: bounding-box prefilter added to `crossing_violations_for_realized_centerline` at two levels -- whole partners whose centerline bbox stays farther than the waveguide width from the route bbox are skipped before the O(n*m) pair loop (this also accelerates the pre-existing intersection scan, which dominated commit validation), and inside the parallel-overlap branch a per-segment-pair bbox reject runs before any direction/distance math. `cargo test --lib` 418/418; wall-time effect to be measured with Milestone 1's ladder.
- [x] (2026-08-31) Milestone 1, measurement: full-run `perf record -g` around bare-CLI `benes_16x16` (305.9 s under perf) and `multiportmmi_16x16` (180.5 s). Consistent ranking on both: **`ObstacleMap::dynamic_owners_for_key` 17-19% of TOTAL wall time** (plus `dynamic_owners_for_cells` 7% on multiportmmi) -- the function scans ALL committed nets' cell sets to answer one cell's ownership, O(nets) hash probes plus a fresh FxHashSet per call, in the kernel hot path; the fix (inverted cell->owners index) is dispatched as an impl-lane slice. Next tiers: the unified kernel itself 12-14%, hashbrown insert/rehash/contains ~8-10% combined, `blocked_count_in_rect` ~5%, congestion lookups 3-5%, `Instant::elapsed`/clock_gettime 3-4% (timing collection -- check whether stats collection is on by default in production and whether it should be). Existing stats logs additionally show dense A* is 84% (benes) / 55% (multiportmmi) of astar_loop and that repair time is >90% renewed searches; failures (24/73) are full wasted searches.
- [x] (2026-08-31) Milestone 1, first fix landed: inverted dynamic-owner index in `ObstacleMap` (`dynamic_owner_index: FxHashMap<CellKey, Vec<NetId>>`, maintained at both commit variants -- which ripup first, preserving the no-duplicates precondition -- at `ripup_route`, `clear_dynamic`, and consistent-empty on the expanded clone; `dynamic_owners_for_key`/`_for_cells` now read it, signatures unchanged). Implemented by the impl lane, lead-reviewed (clone-path and ripup-precondition claims verified against source), one new consistency test, `cargo test` 419/419. Measured (bare CLI, together with Milestone 0b's prefilter, full ladder 0/0 and pytest baseline throughout): **`multiportmmi_16x16` 200 -> 133.5 s (-33%)**; `benes_16x16` 239.9 s vs 239.2 s reference -- neutral, despite the profile attributing 18.8% there. Honest open note: the benes profile-share vs wall-time contradiction is unexplained (perf-overhead distribution or phase overlap); do not re-count that 19% as future headroom.
- [ ] Milestone 1 (profile-driven, owner-directed priority 2026-08-31: profile the GREEN benchmarks first -- the 32x32 stall is a repair pathology and must not steer the optimization order; its captured profile is secondary evidence for the later repair-strategy question): full-run `perf record -g` around `benes_16x16` and `multiportmmi_16x16`, rank the true hotspots, fix the top ones, re-measure the ladder after each. Candidates already evidenced, to be confirmed or refuted by the profile, not assumed: (a) 14 sites in `src/py_router.rs` clone the full `ObstacleMap` per net/per repair attempt/per correction check; (b) single A* searches expanding millions of states (n_74-class); (c) the repair loop re-running near-identical layerwide searches against unchanged obstacles; (d) `realized_dynamic_blockers` / commit-validation O(points^2) sweeps.
- [ ] Milestone 1b (repair effectiveness, owner 2026-08-31: "schauen welche repairs auch sinnvoll sind ... auf diesen benchmarks"): mine the existing per-attempt stats (`--debug-timing` / `collect_route_stats` timing buckets: attempts, successes, wall time and expansions per bucket -- `normal_route`, `probe_route`, `reroute_victims`, `repair_failed_net`, `lidar_pure_probe_commit`, ...) on the green ladder into a strategy balance sheet: fires how often, converts how often, costs how much per conversion. Strategies with high cost and low conversion become candidates for reordering, gating, or removal -- presented to the owner as options, not changed unilaterally (repair-strategy changes are decision-gated by standing practice).
- [ ] Milestone 1c (heuristics justification audit, owner 2026-08-31: "ob es evtl noch andere heuristiken gibt, die derzeit verwendet werden und keine berechtigung haben, so wie eben die 1.25 weighting"): INVENTORY DELIVERED (evidence lane, 2026-08-31) and persisted as `.agent/execplans/2026-08-31-heuristics-inventory.md`: **99 behavioral knobs, 86 with no in-repo justification**, plus two internal default discrepancies (`routing_window_fallback_full_grid` Rust-vs-pyo3-ctor; `long_straight_congestion_weight` reachable only via env, not the Python ctor). Remaining in this milestone: the lead's classification (principled / evidenced tuning / unjustified-and-result-shaping) and the owner decision list with neutral-value ladder tests for the result-shaping ones. Original scope text follows. Inventory every tunable constant and heuristic in the search/repair/commit path (AStarConfig fields and their production overrides, env-gated weights, hard-coded thresholds and magic numbers in `src/astar.rs` / `src/py_router.rs` / `translation/route_rust.py`, benchmark STABLE flag values) into a table: name, value, where set, what it distorts (path shape / legality / order / performance only), and what evidence justifies it (commit, plan, or none). Classify each as principled / evidenced tuning / unjustified. Known suspects from this session: the 45-degree `bend_weight` boost (cost shaping like the 1.25 was), `routing_window_scale` (windows bound optimality exactly like weighted search did), `LONG_STRAIGHT_CONGESTION_WEIGHT=0.05`, `proactive_congestion_weight 4.0 / radius 3`, the repair `pending_straight` `threshold=100`, crossing/search loss values, victim orderings. Unjustified entries become owner decisions (test at neutral value on the ladder), NOT unilateral changes.
- [ ] Milestone 2 (acceptance): the full ladder (benes_4x4/8x8/16x16, multiportmmi_8x8/16x16, heater_s_mod) stays clean at recorded-or-better times, pytest at its baseline; then one fresh `benes_32x32` attempt to re-measure how far the baseline now gets.

## Milestone 1c classification, first tranche (lead, 2026-08-31) -- OWNER DECISION LIST

Result-shaping knobs with no in-repo justification, each with the concrete
neutral-value experiment that would settle it (ladder = the five green
benchmarks plus pytest; "moves results" means any verification or
geometry change, not just timing):

1. `bend_weight = max(cur, 12.0)` at 45 degrees (route_rust.py:6713) --
   shapes path cost directly (a 90-degree bend costs 24 length-units
   ~= 24 um of detour tolerance). Test: ladder at bend_weight floor 1.0
   and at 4.0. Expect: more zig-zag-ish but shorter paths; possibly
   slower search. Classification: result-shaping, magnitude unjustified.
2. `routing_window_scale`: CLI default 0.05 vs Rust default 0.35 vs
   benchmark pins 0.35 (three-way mismatch). Windows bound completeness:
   a too-small window plus fallback settings can change which route is
   found, not just how fast. Test: ladder at unified 0.35 and with
   use_routing_window=false (full grid) to quantify both result drift
   and cost. Classification: result-shaping via search-space truncation.
3. `routing_window_fallback_full_grid`: Rust default false vs pyo3-ctor
   true (production uses true). DISCREPANCY -- decide the intended value
   and align both defaults. With fallback=true the window is performance-
   only (full-grid retry preserves completeness); with false it is
   result-shaping. Recommendation: keep true, fix the Rust default,
   document.
4. `heap_tie_breaker` smaller_g -> larger_g at 45 degrees
   (route_rust.py:6721). Tie-breaking among equal-f paths is
   result-shaping only among equally-cheap paths (benign class), but the
   45-degree-only flip is unexplained. Test: ladder with the flip
   removed. Classification: likely benign, cheap to verify.
5. `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM = 200.0` --
   search-time crossing penalty steering how many/which crossings routes
   accept; 200 um equivalent detour per crossing is a strong prior.
   Test: ladder at 100/400 to see crossing-count sensitivity.
   Classification: result-shaping, value unjustified.
6. `PENDING_STRAIGHT_RIPUP_THRESHOLD = 100` -- repair-victim trigger
   (the 32x32 thrash shows counts of 2692-7132, so 100 fires early
   there). Belongs to the Track-2 repair-strategy review rather than a
   lone neutral test. Classification: repair-strategy knob.
7. `LONG_STRAIGHT_CONGESTION_*` (200 um / radius 5 / amount 1, env
   weight 0.05 pinned by 4 benchmarks) plus `proactive_congestion 4.0/3`
   (benes pins): congestion shaping -- these exist to make dense fanouts
   converge and demonstrably do (documented in the 2026-08-25 plan for
   proactive congestion). Classification: evidenced tuning, but the
   specific magnitudes untested; lowest priority.
8. Timing collection on by default (`SCRIPT_DEBUG_TIMING = True` ->
   `collect_detailed_timing` -> per-expansion Instant calls, 3-4% wall
   clock): performance-only. Proposal: keep per-attempt timers, gate the
   per-expansion micro-timers behind an opt-in; output content unchanged.

## Surprises & Discoveries

- Observation (2026-08-31, from the probe): the `benes_32x32` failure mode is not slow-but-steady routing; it is a repair livelock signature -- `native_repair_probe net=57 allowed_partners=12 crossing_events=3 realized_violations=9`, all `not_perpendicular`, victims ripped across the whole layer. Engine speed will shorten the time to hit it; whether the baseline can pass net 57 at all may be a separate repair-strategy question. Record, do not scope-creep into it here.

## Decision Log

- Decision (owner, 2026-08-31): performance work on the baseline only; no crossing-precomputation-based speedups (those are the contributions). Profiling before optimizing; the two Milestone-0 items are exempt because their cost/benefit is already evidenced from this session's own measurements.

## Outcomes & Retrospective

Not finished.

## Context and Orientation

The routing engine is Rust (`src/astar.rs` search kernels, `src/py_router.rs` the `PyPhotonicRouter` session driving batches, repair, commits, endpoint correction; `src/obstacle_map.rs` the grid). Python (`translation/route_rust.py`) packs jobs and calls one native batch (`route_many_with_repair_and_commit`) -- during that call nothing prints from Python, which is why long runs are silent. Timing lands in the flow's own "Timing summary" and, with `--debug-timing`/stats, per-attempt records. `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` is an existing env-gated eprintln channel from the repair loop and the pattern to follow for the progress print.

## Concrete Steps

    # profile a stuck/hot workload (paranoid must be <=2; owner set 1)
    perf record -F 397 -g -p <pid> -o out.perf -- sleep 90
    perf report -i out.perf --stdio --no-children -s dso,sym | head -60

    # ladder acceptance commands are the benchmarks' bare CLI invocations

## Validation and Acceptance

Milestone 0/0b: behavior-identical routing (ladder verifications unchanged), progress lines visible under the env var, no measurable slowdown. Milestone 1: each landed optimization cites the profile lines it removes and re-measures at least `benes_16x16` and `multiportmmi_16x16` wall time plus the full clean ladder. Milestone 2 as described in Progress.

## Idempotence and Recovery

Source edits only; `git checkout` restores. Profiles and logs live in the session scratchpad and are re-creatable.

## Interfaces and Dependencies

No public interface changes planned; env vars are additive. Anything touching search cost/legality semantics is out of scope here (that is correctness work, not performance work).
