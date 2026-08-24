# Make crossing-aware A* search cost-sound: crossings pay their own cost, terminal-straight launch/landing is honored everywhere

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository, TUMPhotonicRouter, routes optical waveguides for photonic chips on a discrete grid. A route is built out of "primitives" (short pre-defined path segments -- straight runs, 45-degree bends, 90-degree bends, etc., defined in `src/primitives.rs`) chained together by an A* pathfinding search (`src/astar.rs`). Two of the eight compass-style directions on the grid are "cardinal" (horizontal/vertical, 90-degree turns between them) and the other six are "diagonal" (45-degree turns); the grid uses an angle index 0-7 for these eight directions, called "eighths of a circle" in this repo's own vocabulary.

Two configuration values control how the search chooses between path shapes: `bend_weight` (how much extra cost is charged per unit of turning, in `src/astar.rs`'s `AStarConfig`) and `heuristic_weight` (how aggressively the search trusts its distance estimate to the target, trading optimality for speed). A third, unrelated value, `crossing_loss`, is the cost charged when a route's search decides to place a legal "crossing" -- a point where two different nets' waveguides physically cross paths on the chip, which is allowed under certain geometric conditions when `enable_crossings` is on.

Today, whenever a benchmark is run with `crossing_mode` set to `"collision"` or `"lidar-pure"` (an internal flag this repo calls `collision_crossing_mode`, computed in `translation/route_rust.py`), `bend_weight` and `heuristic_weight` silently change for every net in that benchmark, including nets that end up never crossing anything at all. This was discovered and confirmed empirically during this session's investigation of `build/routed_multiportmmi_8x8.gds`: two nets (`n_7` and `n_8` in the `multiportmmi_8x8` stable-baseline benchmark) route with unexpected, cost-suboptimal 90-degree bends instead of diagonals, and a direct check of the router's own `crossing_events()` output showed these two nets have **zero** crossing events anywhere in the final routed layout of 33 total events -- meaning they never had any crossing-legality reason to be treated any differently from a plain, non-crossing-aware route, yet they were, because `crossing_mode` is a single global setting applied to the entire benchmark rather than a per-net decision.

Separately, a second, independently confirmed gap: `require_terminal_straights` (an `AStarConfig` field that, when true, forces a route to launch and land using a straight-line primitive right at the port face, instead of bending immediately at the port) is fully implemented and honored in the router's plain search function (`route_single_net_with_bounds_dynamic_expansion` in `src/astar.rs`), but is never read at all by the router's separate crossing-aware search function (`route_single_net_with_bounds_crossing`, same file). Several call sites in `src/py_router.rs` explicitly set `require_terminal_straights = true` on a config and then pass that same config into the crossing-aware function expecting the constraint to hold -- it silently does not.

After this plan: (1) `bend_weight` and `heuristic_weight` are the same regardless of `crossing_mode` -- crossing-awareness only changes whether the search is allowed to reason about placing a legal crossing and what a crossing costs, not how it prices ordinary bends and distance; (2) `crossing_loss` (the per-crossing cost penalty) is re-validated and, if needed, re-tuned against the corrected weight scale so that crossings remain meaningfully "expensive" relative to a bend-heavy detour, exactly as intended; (3) a route that asks for a straight port launch/landing gets one whether or not its search happens to escalate into the crossing-aware code path. A person can see this working by re-running the `multiportmmi_8x8` stable-baseline benchmark (exact command in Concrete Steps) and observing that `n_7` and `n_8` (net ids 8 and 9) now route with a genuinely cost-minimal path shape (diagonal-then-straight, not all-90-degree), while `build/verification/*.json` for every benchmark in the validation ladder still reports `error_count=0`, and the router's `crossing_events()` output for a crossing-heavy benchmark shows crossing counts that are unchanged or explicable, not silently inflated by cheapened bends.

## Progress

- [x] (2026-08-24) Milestone 1: `require_terminal_straights` is now honored by the crossing-aware search kernel. Implemented via `.agent/scripts/codex_task.sh` (Codex CLI, Implementation Engineer role) against a fully-specified task derived directly from this plan's own Milestone 1 text; reviewed line-by-line before acceptance. Both guards inserted exactly as specified: source-launch guard after `stats.primitive_generated_by_class[primitive_class] += 1;` (now `src/astar.rs:4563-4568`), target-landing guard after `next_angle` is computed (now `src/astar.rs:4625-4632`). New test `terminal_straight_requirement_rejects_immediate_port_bends_in_crossing_kernel` added, mirroring the plain-kernel test exactly (same source/target `State::new(1,1,0)` -> `State::new(3,3,2)`, same first/last-primitive-is-straight assertions), using a minimal `CrossingSearchConfig` with one deliberately-unreachable dummy partner so `route_single_net_with_crossing_config`'s early `partners.is_empty()` return does not short-circuit the test. `cargo test --lib`: `394 passed; 0 failed` (up from 392 pre-change plus the 2 filtered `terminal_straight_requirement` tests both passing). No pre-existing test regressed.
- [x] (2026-08-24) Milestone 2: removed the `collision_crossing_mode`-gated override of `bend_weight`/`heuristic_weight` in `translation/route_rust.py` (both now uniform regardless of crossing mode), deleted the resulting dead `PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT` environment variable (confirmed zero other references before removal), and re-tuned `crossing_loss`'s default from `50.0` to `200.0` in `translation/route_rust_crossing_plan.py` based on an empirical sweep on `multiportmmi_8x8` stable-baseline: total `crossing_events()` count was `38` at the old default `50.0` post-weight-unification (up from a `33`-event pre-Milestone-2 baseline), dropped to `35` at `150.0`, and reached and held at `33` (matching the pre-change baseline exactly) from `175.0` through `600.0` -- `200.0` was chosen as a round value with margin above the empirical `175.0` threshold, not needlessly large. `pytest -q` returns to the exact pre-existing baseline (11 known-failing, 335 passing, confirmed identical by direct A/B stash comparison) after updating one test's hardcoded attempt/repair counters (`tests/test_multiportmmi_benchmark.py`, `route_attempts` 42->39 and `repair_count` 1->0 for the `lidar-pure` parametrization, with a full explanatory comment -- the fix genuinely makes that benchmark's collision-crossing-fallback path cleaner, needing fewer attempts and zero repair rounds). One new, investigated, accepted side effect: see Surprises & Discoveries below.
- [x] (2026-08-24) Milestone 3: `cargo test --lib` clean (394 passed). `benes_4x4` clean (`error_count=0, warning_count=0`). `multiportmmi_8x8` stable-baseline (crossing-enabled) clean (`success=True, error_count=0, warning_count=1`, the already-investigated, accepted `n_68` fallback warning, unaffected by the anchor-match fix below). One real regression found and fixed along the way: `multiportmmi_8x8` **bare-defaults** (no `--crossings` flag; this benchmark's own defaults still enable crossings) went from clean (pre-Milestone-1 baseline, confirmed via a temporary `git worktree` at commit `f59c3bb`: `success=True, error_count=0, warning_count=2`) to a hard failure (`success=False, error_count=2`, `n_67` genuinely not touching its own source port) after Milestone 2. Isolated via the same worktree technique to Milestone 2 specifically (Milestone 1 alone, commit `8a0309a`, was still clean). Root-caused with the `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS` trace hook: the unchecked rich-correction/splice candidate loop in `_apply_crossing_aware_endpoint_correction_to_record` (`translation/route_rust_endpoint_correction.py`) only checked that a candidate centerline realized as a valid shape (`_realization_accepts`), never that it actually reached the real port (`_terminal_anchor_matches`, which the checked, per-segment tier tried first already does) -- a pre-existing, latent bug that Milestone 2's weight change merely exposed by shifting `n_67`'s route geometry. Fixed by adding the missing `_terminal_anchor_matches` check to that loop (commit `2755a22`), matching the checked tier's own acceptance logic exactly. Repository owner's direct decision after reviewing the fix and its consequence: "keep the fix, document n_67 as blocked, move on" -- see Decision Log.

  A methodology mistake was found and corrected mid-milestone: `benes_8x8` and `benes_16x16` were first re-run bare (no `--crossings` flag), which is **not** representative -- both benchmark files (`benchmarks/benes_8x8.py:23-25`, `benchmarks/benes_16x16.py:23-25`) document a "Stable crossing-router baseline" requiring `--crossings true --crossing-mode lidar-pure`, which bare defaults do not supply (`enable_crossings=False` by default at `routing_flow.py:769`). The repository owner caught this directly ("i mean benes 16x16 clearly has a lot of crossings"). This invalidated not only an in-flight timing comparison in this plan (both a "pre-plan" and a "post-plan" bare-defaults run showed `enable_crossings=False`, so neither could have exercised the crossing-aware kernel this plan changed at all -- the ~165s timing gap between them was pure noise, not a regression) but also an earlier, already-recorded claim in `.agent/REPOSITORY_STATE.md` and `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md`'s Milestone 0 ("benes_8x8/benes_16x16 re-verified clean, 2026-08-24"), which was also run bare. Both benchmarks were re-run with the correct flags and are genuinely clean: `benes_8x8` (`error_count=0, warning_count=0`, `25.2s`), `benes_16x16` (`error_count=0, warning_count=0`, `392.4s` -- there is no valid prior correct-config timing to compare against, since the earlier "252.9s" figure was itself from the wrong, bare-defaults config; `392.4s` is recorded here as the first real baseline for this configuration). Both corrections are also recorded in `.agent/REPOSITORY_STATE.md` and `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md`.

  `multiportmmi_16x16` stable-baseline still fails, but **not** with the same error this plan expected to confirm unchanged: the failure signature moved from `n_50` (`candidate_blockers=[49, 50]`, as documented in `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md`) to `n_49` (`candidate_blockers=[49]`, a related but distinct net and a different `recent_errors` trace: `RuntimeError: No route found for n_49: mmi0_ps_array_0_heater_2,o2 -> mmi0_multiport_0_0,o3. ... error=No repair route found; candidate_blockers=[49]; recent_errors=["reroute_victims:net49:roundSome(1):rip[49]:No route found", "reroute_victims:net49:roundSome(1):rip[49]:No legal LiDAR crossing route found", "repair_failed_net:net50:roundSome(1):rip[49]:No legal LiDAR crossing route found"]`). This plan's own changes (weight unification, `crossing_loss` retuning) plausibly shifted the routing order or the specific congestion point enough to move which net in this tightly-packed cluster fails first, without resolving the underlying congestion itself -- consistent with `n_49`/`n_50` already being flagged together as a "candidate_blockers" pair in the original `n_50` finding. This is a genuine, new fact for the *separate*, already-paused `stabilize-16x16-benchmarks` plan's Milestone 1 (which was investigating `n_50` specifically) -- recorded there and in `.agent/REPOSITORY_STATE.md`, not root-caused further here, per the repository owner's explicit instruction to finish this plan and park the rest rather than start new investigation.

## Surprises & Discoveries

- Observation: the `bend_weight`/`heuristic_weight` divergence is not an accidental bug or an inverted condition -- it was added deliberately in commit `56e8a1d` ("routing: checkpoint crossing verification foundation", a large squashed checkpoint commit), which bolted a `collision_crossing_mode` gate onto weight-tuning code that previously (commit `5c8a00b`, "Enable tuned 45-degree A star routing") applied unconditionally to every 45-degree-turn-enabled route. The squash commit's message gives no per-line rationale for the split.
  Evidence: `git log --all --oneline -S "collision_crossing_mode = bool" -- translation/route_rust.py` returns only `56e8a1d`; `git show 56e8a1d -- translation/route_rust.py` shows the diff introducing the `collision_crossing_mode` conditional directly on top of the previously-unconditional weight-boost lines.

- Observation: a dedicated "make a crossing cost extra" knob already exists and is exactly the right mechanism the repository owner asked for -- it does not need to be invented, only re-validated. `crossing_loss` is added once per crossing actually encountered during search, as a flat additive term in the A* cost function.
  Evidence: `src/astar.rs:4815`: `... + f64::from(crossing_outcome.crossing_count) * crossing.crossing_loss;`. Its default value, when no explicit override is supplied and `enable_crossings` is true with `crossing_mode` in `{"collision", "lidar-pure"}`, is `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 50.0` (`translation/route_rust_crossing_plan.py:29`), and it is already independently overridable at runtime via the environment variable named by `COLLISION_CROSSING_SEARCH_LOSS_ENV = "PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM"` (`translation/route_rust_crossing_plan.py:30`), read inside `_effective_crossing_search_loss` (`translation/route_rust_crossing_plan.py:109-129`).

- Observation: unifying `bend_weight` changes the relative cost of "bend a lot to avoid a crossing" versus "just cross" as a side effect, because `crossing_loss` is denominated in the same additive cost units as bend cost and path length. `primitive.bend_cost` is stored in angle-eighths (`src/primitives.rs:292`: `bend_cost: angle_delta.unsigned_abs() as f64`), so a 90-degree bend has `bend_cost = 2`. Today, under `collision_crossing_mode`, `bend_weight` stays at its un-boosted default of `1.0` (`src/astar.rs:90`), so a 90-degree bend costs `2.0` against a `crossing_loss` of `50.0` -- a crossing is about 25 times as expensive as one 90-degree bend. If `bend_weight` becomes uniformly `12.0` (the plain 45-degree-tuned value, `translation/route_rust.py:6441`), the same 90-degree bend costs `24.0` against the same `crossing_loss` of `50.0` -- a crossing becomes only about 2 times as expensive as one bend. This is a real, evidence-backed reason `crossing_loss` needs re-validation as part of this same plan, not a separate follow-up; Milestone 2 must not be considered complete on the weight change alone.
  Evidence: computed directly from the constants above; `50.0 / 2.0 = 25.0` versus `50.0 / 24.0 ≈ 2.08`.

- Observation: unifying `bend_weight`/`heuristic_weight` (Milestone 2) causes one new, investigated, accepted `photonic_verification` warning on `multiportmmi_8x8` stable-baseline: `n_68` now triggers `endpoint_correction_fallback_used` (`warning_count` 0 -> 1; `error_count` stays `0`, `success` stays `true`). Isolated by A/B testing (weight change alone, with `crossing_loss` forced back to `50.0` via its env var override) to be caused specifically by the weight-unification change, not the `crossing_loss` retune. Root cause, traced with the existing `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS` diagnostic hook (`translation/route_rust_endpoint_correction.py:1108-1122`): `n_68`'s route shape near its port changed (since `bend_weight` is now correctly boosted for it, where previously it was not), and the changed shape no longer satisfies the crossing-aware endpoint-correction cascade's first, checked, per-segment tier (`_checked_terminal_segment`, same file, lines 1266-1422) for either terminal segment -- the suffix segment (18 points) was actively rejected by the native `router.centerline_port_corrected_checked` call with `"No port endpoint correction candidates found for centerline segment"`, and the prefix segment produced no trace at all (consistent with hitting the function's own `len(segment) < 2` silent early return, meaning `n_68`'s first crossing sits very close to its source port). The cascade correctly falls through its later, weaker tiers and still produces a fully valid, in-tolerance route (confirmed by `error_count=0`); this is `severity: warning` in `photonic_verification.py`, explicitly by design (comment at `translation/photonic_verification.py:399-404`), not a correctness regression. Accepted as an expected side effect of Milestone 2 rather than blocking it -- but this same investigation surfaced a materially deeper problem with the endpoint-correction subsystem itself (a multi-tier fallback cascade, called from three separately-evolved pipeline call sites, producing measurable route-geometry overshoot on 11 of 111 nets in this same benchmark run, and visually confirmed by the repository owner in two screenshots of `build/routed_multiportmmi_8x8.gds` showing a bowtie/hourglass geometry pinch at a crossing and a spurious loop near a component). That problem is out of scope for this plan and is being written up as its own new ExecPlan next; see `.agent/REPOSITORY_STATE.md`'s "Next Engineering Step" for the pointer once that plan exists.
  Evidence: `/tmp/n68_trace.log` (this session's own scratch output, not preserved in the repository) captured two trace lines: `endpoint_trace net=n_68 id=69 mode=checked_terminal_segment segment_len=18 ... status=reject error=No port endpoint correction candidates found for centerline segment` and `endpoint_trace net=n_68 id=69 crossings=5 mode=(False,True) len=129 ... accepts=True` (the eventual accepted candidate came from the unchecked global-correction-plus-splice cascade's `(correct_source=False, correct_target=True)` mode). A separate scratch diagnostic (`check_output_overshoot.py`, same session) found 11 of 111 routed nets in this benchmark overshoot their own source/target port y-range by more than 5 micrometers, worst three being `n_69` (128.88um), `n_31` (46.38um), and `n_68` (38.88um) -- `n_68` appearing in both the endpoint-correction-fallback list and the geometric-overshoot list is strong circumstantial evidence the two are the same underlying phenomenon.

- Observation: `max_iterations` is a third value gated by the same `collision_crossing_mode` conditional in `translation/route_rust.py:6419-6424` (capped to `50_000` outside collision/lidar-pure mode, left at its caller-supplied value, `500_000` in the stable-baseline configuration, inside it) -- this is a search-budget knob, not a cost-function term, and the repository owner's explicit direction so far (see Decision Log) was scoped to `bend_weight`/`heuristic_weight` and `crossing_loss` only. `max_iterations` is deliberately left untouched by this plan; note it here so a future soundness pass does not have to rediscover it.
  Evidence: `translation/route_rust.py:6419-6424`.

- Observation: a `CrossingSearchConfig` test partner that is deliberately placed far from the route under test (so it is never actually crossed) must be paired with `require_all_partners: false`, not `true`. `require_all_partners: true` requires every listed partner to actually be crossed for the search to succeed at all, so a non-interacting dummy partner combined with `require_all_partners: true` would make `route_single_net_with_crossing_config` fail to find any route.
  Evidence: Codex's Milestone 1 implementation report; confirmed by the passing test using `require_all_partners: false`.

- Observation: `multiportmmi_8x8`'s own bare-defaults benchmark configuration (`benchmarks/multiportmmi_8x8.py`, run via plain `routing_flow.py multiportmmi_8x8` with no `--crossings`/`--crossing-mode` CLI flags) already enables crossings internally -- "bare defaults" is not "crossings disabled" for this benchmark. This matters for reading this plan's own regression finding correctly: the `n_67` failure below was found under bare defaults, not the explicit `--crossings true --crossing-mode lidar-pure ...` stable-baseline configuration used elsewhere in this plan; both configurations exercise the crossing-aware endpoint-correction code path, just with different derived settings.
  Evidence: bare `routing_flow.py multiportmmi_8x8` produces `endpoint_correction_fallback_used` issues referencing "crossing-aware endpoint correction fallback" in its `photonic_verification` output, which only the crossing-aware code path emits.

- Observation (Milestone 3 regression, found, root-caused, and fixed): `multiportmmi_8x8` bare-defaults regressed from clean to a hard failure after Milestone 2, isolated to Milestone 2 specifically using a temporary `git worktree` at each of the three relevant commits (`f59c3bb` pre-plan, `8a0309a` Milestone 1 only, current HEAD). Full trace for `n_67` (net id 68, `mmi0_multiport_2_1.o8 -> mol_array_1_mzi_2.o1`), captured via `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS=n_67`, before the fix in commit `2755a22`:
      endpoint_trace net=n_67 id=68 mode=checked_terminal_segment segment_len=18 segment_start=(2501.5, 993.125) segment_end=(2517.5, 987.125) source_port=(2499.4, 992.5) target_port=(2517.5, 987.125) status=reject error=No port endpoint correction candidates found for centerline segment
      endpoint_trace net=n_67 id=68 mode=checked_terminal_segment segment_len=22 segment_start=(2537.5, 705.125) segment_end=(2607.5, 649.125) source_port=(2537.5, 705.125) target_port=(2609.4, 650.0) status=accept candidate_label=normal_segment corrected_len=22
      endpoint_trace net=n_67 id=68 crossings=2 mode=checked_terminal_segments prefix_checked=False suffix_checked=True len=57 start=(2501.5, 993.125) end=(2609.4, 650.0) source=(2499.4, 992.5) target=(2609.4, 650.0) accepts=False
      endpoint_trace net=n_67 id=68 crossings=2 mode=(True,True) len=57 start=(2501.5, 993.125) end=(2609.4, 650.0) source=(2499.4, 992.5) target=(2609.4, 650.0) accepts=True
  The checked, per-segment tier (tier 1) correctly rejected the combined candidate at `mode=checked_terminal_segments` because its start point, `(2501.5, 993.125)`, is about 2.1 micrometers from the real source port, `(2499.4, 992.5)` -- close, but outside the roughly 1e-6-um tolerance `_terminal_anchor_matches` requires. The unchecked tier (tier 2, the `_candidate`/`candidate_modes` loop) then produced the *exact same* disconnected start point but `accepts=True`, because its acceptance check (`_realization_accepts`) verifies only that the shape realizes as a valid waveguide polygon, never that it reaches the intended port. After the fix (commit `2755a22`, adding the same `_terminal_anchor_matches` check to tier 2), the identical trace now shows every `candidate_modes` entry correctly rejected (`source_ok=False`), and the net falls through to a genuine, correctly-diagnosed `endpoint_correction_error` instead of silently committing disconnected geometry -- `multiportmmi_8x8` bare-defaults is still `success=False` after the fix, but for an honest, understood reason instead of a silent one.
  Evidence: `/tmp/n67_trace.log` and `/tmp/n67_trace_fixed.log` (this session's own scratch output, not preserved in the repository); full post-fix `photonic_verification` issue: `endpoint_correction_error` for `n_67` reading "crossing-aware endpoint correction produced no realizable centerline. source_port_um=(2499.4, 992.5), target_port_um=(2609.4, 650.0), source_route_cell=(1260, 315), target_route_cell=(1313, 143), source_route_center_um=(2501.5, 993.125), target_route_center_um=(2607.5, 649.125)."

## Decision Log

- Decision: unify `bend_weight` and `heuristic_weight` so they no longer vary with `crossing_mode`; keep `crossing_loss` as the sole crossing-specific cost term, and re-tune its default as part of the same change rather than as a separate follow-up.
  Rationale: repository owner's direct statement, verbatim: "1. idk why the weight for bends should vary for the crossing mode. in the crossings mode only the cost function should add cost for crossings and crossings should be 'quite expensive', whatever that means. this should be tuned as well then." Confirmed by the repository owner replying "Yes, write the ExecPlan" immediately after this plan's scope (including the `crossing_loss` re-tuning) was proposed back to them.
  Date/Author: 2026-08-24, repository owner (via direct chat) and Claude (proposal, confirmed by owner).

- Decision: implement `require_terminal_straights` in the crossing-aware search kernel rather than removing the field or the call sites that set it to `true`.
  Rationale: the constraint is conceptually sound (forcing a straight launch/landing segment at a port is a real, intentional geometric requirement used elsewhere in the router) and several call sites already explicitly opt into it expecting it to apply; the gap is that the crossing kernel silently ignores it, not that the constraint itself is wrong. The repository owner asked "I dont even know if require_terminal_straights makes any sense tbh. Is it always active?" -- investigation (recorded in the chat, not repeated in full here) showed it defaults to `false` and is explicitly set `true` only for primary-attempt searches and `false` for repair/fallback searches, a deliberate and defensible strict-first-relaxed-later pattern; the owner did not object to this pattern once it was explained, only to the inconsistency between kernels.
  Date/Author: 2026-08-24, Claude (investigation and proposal), repository owner (implicit confirmation via "Yes, write the ExecPlan").

- Decision: `max_iterations`'s divergence between crossing and non-crossing mode is out of scope for this plan.
  Rationale: not part of what the repository owner asked to be fixed; recorded in Surprises & Discoveries instead so it is not silently lost.
  Date/Author: 2026-08-24, Claude.

- Decision: accept the new `n_68` `endpoint_correction_fallback_used` warning as an expected, non-blocking side effect of Milestone 2, and do not attempt to fix the endpoint-correction cascade inside this plan.
  Rationale: repository owner's direct instruction after reviewing the traced root cause and two additional visual symptoms (a bowtie/hourglass geometry pinch and a spurious route loop, both screenshotted from `build/routed_multiportmmi_8x8.gds`): "i think it makes more sense to go through the code and fix the logic and then in the end look at the symptoms to fix it further" -- i.e. finish this plan's already-scoped, already-understood fix first, then open a new, dedicated ExecPlan for the endpoint-correction subsystem with all of today's findings (this plan's `n_68` trace and overshoot measurement, the three-separate-call-site pipeline structure, and both screenshots) folded in as context, rather than expanding this plan's scope or chasing individual visual symptoms ad hoc.
  Date/Author: 2026-08-24, repository owner (direct instruction) and Claude (proposal).

- Decision: fix the tier-2 (`_candidate`/`candidate_modes` loop) missing port-anchor check in `_apply_crossing_aware_endpoint_correction_to_record` now, inside this plan, rather than deferring it to the endpoint-correction follow-up plan alongside the rest of that subsystem's issues; and, having fixed it, keep the fix and document `multiportmmi_8x8` bare-defaults' resulting `n_67` failure as a known, currently-blocked case rather than reverting Milestone 2's weight change to hide it.
  Rationale: repository owner's direct instruction, verbatim: "keep the fix, document n_67 as blocked, move on" -- given after I reported that (a) the fix is a small, precisely-scoped, unambiguous correctness fix (a missing check that an already-correct sibling code path already performs, not a redesign), (b) reverting Milestone 2 to avoid exposing it would hide a real, pre-existing bug rather than fix anything, and (c) the fix does not fully resolve `n_67` -- it converts a silent illegal-geometry acceptance into an honest, correctly-diagnosed failure, which is strictly better but leaves `multiportmmi_8x8` bare-defaults still failing until the deeper endpoint-correction cascade problem (already deferred) is addressed.
  Date/Author: 2026-08-24, repository owner (direct instruction) and Claude (proposal).

- Decision: after this plan's three milestones close, do not write the endpoint-correction-cascade follow-up plan yet, and do not resume `stabilize-16x16-benchmarks`'s paused `n_50`/`n_49` investigation. Record both as documented next-step candidates in `.agent/REPOSITORY_STATE.md` only, and stop.
  Rationale: repository owner's direct instruction, verbatim: "finish the current plan, then park the rest." Given separately, in the same turn: agreement that a functional-verification test suite (small, edge-case-targeted regression tests, eventually CI-gated) is the right next investment in principle, but explicitly to be scoped as its own future initiative once the current queue of open plans is cleared, not squeezed into this one.
  Date/Author: 2026-08-24, repository owner (direct instruction).

## Outcomes & Retrospective

All three milestones are complete. The plan closes with `bend_weight`/`heuristic_weight` uniform regardless of crossing mode, `crossing_loss` re-tuned to `200.0` and empirically validated to preserve real crossing counts, `require_terminal_straights` honored by both search kernels, and a genuine illegal-geometry-acceptance bug in the endpoint-correction cascade fixed as a direct consequence of validating this plan's own changes -- none of which were part of the plan's original scope, all found by taking benchmark validation seriously rather than treating a passing `error_count=0` as sufficient on its own.

What remains open, by design, per explicit repository owner instruction: `multiportmmi_8x8` bare-defaults' `n_67` fails endpoint correction honestly now instead of silently producing bad geometry (Surprises & Discoveries); the endpoint-correction subsystem's deeper structural issues (the multi-tier cascade, three separately-evolved call sites, the `n_68`/`n_67`/overshoot-geometry evidence, and the repository owner's two screenshots) are recorded as a next-step candidate in `.agent/REPOSITORY_STATE.md`, not yet a written plan; and `stabilize-16x16-benchmarks`'s paused Milestone 1 now has a new fact (the failure signature moved from `n_50` to `n_49`) it will need to account for when resumed.

The key process lesson, worth repeating for any future work in this repository: before treating a benchmark run as representative of "the stable baseline," check that specific benchmark's own file for a documented stable-configuration comment (as `benchmarks/benes_8x8.py` and `benchmarks/benes_16x16.py` both have) rather than assuming bare CLI defaults are representative -- this plan repeated the same mistake twice (once for `multiportmmi_8x8` earlier in the session, discovered independently, and once for `benes_8x8`/`benes_16x16` inside this plan's own Milestone 3, caught by the repository owner) before it was made an explicit checklist item.

## Context and Orientation

### Files this plan touches

`src/astar.rs` -- contains both search kernels (`route_single_net_with_bounds_dynamic_expansion`, the "plain kernel", and `route_single_net_with_bounds_crossing`, the "crossing kernel"), their shared `AStarConfig` struct (fields include `bend_weight`, `heuristic_weight`, `require_terminal_straights`, all documented inline where they are declared, near line 55-120), and their Rust unit tests (in the `#[cfg(test)] mod tests` block near the end of the file).

`translation/route_rust.py` -- the Python orchestration layer that builds the `AStarConfig` object (called `self.astar_cfg` on the `_RouteNetsRustSession` class) before handing routing jobs to the compiled Rust extension. The `collision_crossing_mode` conditional block that this plan removes lives at lines 6415-6441 of this file as of this writing; treat line numbers throughout this plan as approximate anchors to re-locate with `grep -n`, not as guaranteed-exact after other edits.

`translation/route_rust_crossing_plan.py` -- defines `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` (line 29) and `COLLISION_CROSSING_SEARCH_LOSS_ENV` (line 30), and the `_effective_crossing_search_loss` function (lines 109-129) that decides what `crossing_loss` value actually reaches the Rust search when the caller has not supplied an explicit non-zero value.

`.agent/REPOSITORY_STATE.md` -- the living snapshot document this repository keeps of overall project state; Milestone 3 updates it with this plan's outcome, following the same pattern used by every other ExecPlan completed this session (see, for example, `.agent/execplans/2026-08-24-modular-routing-strategies.md`'s own closing updates to that file).

### Key terms, defined for a reader new to this repository

"Net": one logical point-to-point waveguide connection to be routed, identified by a `net_id` integer and a human-readable name like `n_7`.

"Primitive": a short, pre-defined path segment (straight run, 45-degree bend, 90-degree bend, etc.) that the A* search chains together to build a full route; primitives are enumerated per starting angle in `src/primitives.rs`.

"Angle" / "eighths of a circle": this repository represents direction as an integer 0-7, where even values (0, 2, 4, 6) are the four cardinal (horizontal/vertical) directions and odd values (1, 3, 5, 7) are the four 45-degree diagonal directions.

"Crossing": a point where two different nets' physical waveguide paths cross each other on the chip. Some crossings are geometrically legal (for example, two waveguides crossing at a near-90-degree angle with enough separation); the router's crossing-aware search exists to find and validate these.

"`collision_crossing_mode`" (Python-only name, not a Rust concept): a boolean computed in `translation/route_rust.py` as `bool(self.enable_crossings) and self.crossing_mode in {"collision", "lidar-pure"}`. It is a single value computed once per routing session (i.e., once per benchmark run), not per net.

"Plain kernel" (this plan's shorthand, not an existing repository term): `route_single_net_with_bounds_dynamic_expansion` in `src/astar.rs`, the search function used when a net is not currently being searched for a legal crossing placement.

"Crossing kernel" (this plan's shorthand): `route_single_net_with_bounds_crossing` in `src/astar.rs`, the search function used when a net's search is actively reasoning about a candidate crossing against one or more specific "partner" nets (represented by `CrossingSearchConfig.partners`, a list of `CrossingSearchPartner` values each carrying the partner net's `net_id` and the waypoints of its already-committed path).

"`AStarConfig`": the Rust struct (`src/astar.rs`, declared near line 40-120) carrying every tunable knob for a single search call, including `bend_weight` (default `1.0`, `src/astar.rs:90`), `heuristic_weight`, `require_terminal_straights` (default `false`, `src/astar.rs:117`), and `max_iterations`.

"`CrossingSearchConfig`": the Rust struct carrying crossing-specific search parameters, including `net_id`, `partners`, `min_straight_cells`, `crossing_half_size_cells`, `bend_runout_cells`, `crossing_loss`, `require_all_partners`, and `terminal_bump_guard`. An example fully-populated literal (from an existing test) appears in this plan's Milestone 1 section.

### How the per-net decision to use the crossing kernel already works (and why it is not the problem)

This repository already has a correct, per-net (not global) decision about which search kernel to actually run for a given net: a cheap local lookup first checks whether any other already-committed net's path is geometrically near enough to be a plausible crossing partner, and only escalates to the more expensive crossing kernel if it finds one (`src/py_router.rs:2938` and the `lidar_pure_owner_lookup_partner_set` logic near `src/py_router.rs:3467`, both covered by existing unit tests near `src/py_router.rs:16583`). This part of the system already matches the repository owner's stated mental model ("A* explores, and when crossings are enabled it can find collisions and decide whether or not to insert a crossing") and this plan does not change it.

The problem this plan fixes is narrower: even when that per-net decision correctly resolves a net through the cheap, no-crossing-nearby path, the *cost function weights* that net's search runs with were still decided once, globally, from the benchmark-level `crossing_mode` flag -- not from that same per-net decision. This plan makes the weights uniform (removing the discrepancy entirely) rather than trying to thread the existing per-net decision through to weight selection, because the repository owner's own framing ("in the crossings mode only the cost function should add cost for crossings") calls for the simpler fix: stop varying bend/heuristic cost by mode at all, and rely on `crossing_loss` (which *is* naturally zero-effect for a net that never encounters a crossing candidate, since `crossing_outcome.crossing_count` is `0` for it) to carry 100% of the crossing-specific cost signal.

### Toolchain and safety notes (identical to every other ExecPlan completed in this repository this session)

Any `src/*.rs` change requires rebuilding the compiled Python extension before Python-level testing will see it. From the repository root:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" maturin develop --release

Do not run the `--debug-svgs` CLI flag with no value (or `all`) on `multiportmmi_8x8` or `multiportmmi_16x16` -- it triggers an expensive full-SVG export; always pass a specific numeric net selector if SVG debugging is needed.

This plan is independent of the currently-parked `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` plan (which is investigating a specific `n_50` repair-exhaustion failure in `multiportmmi_16x16`, unrelated to this plan's cost-function scope per that plan's own findings so far). Either plan can proceed without waiting on the other; if both are active at once, validate each independently before assuming a shared benchmark run covers both.

## Plan of Work

**Milestone 1** adds the two missing guard checks to the crossing kernel, `route_single_net_with_bounds_crossing` in `src/astar.rs` (function begins at line 4320 as of this writing; re-locate with `grep -n "fn route_single_net_with_bounds_crossing" src/astar.rs` if it has moved). The plain kernel already implements this correctly, in its own neighbor-expansion loop, as two `continue`-based guards (`src/astar.rs:3651-3667` as of this writing):

    if config.require_terminal_straights
        && state == source
        && !primitive_class_is_straight(primitive_class)
    {
        continue;
    }
    let next_x = state.x.checked_add(primitive.dx)?;
    let next_y = state.y.checked_add(primitive.dy)?;
    let next_angle = primitive.end_angle % 8;
    if config.require_terminal_straights
        && (next_x - target.x).abs() <= target_tolerance
        && (next_y - target.y).abs() <= target_tolerance
        && accepted_target_angles[next_angle as usize]
        && !primitive_class_is_straight(primitive_class)
    {
        continue;
    }

The first guard rejects any candidate primitive at the very first expansion step (when the search is standing at `source`) that is not a "straight" class primitive, forcing the route to leave the port going straight. The second guard rejects any candidate primitive whose destination would land within `target_tolerance` cells of `target` at an accepted target angle, unless that primitive is also straight-class, forcing the route to arrive at the port straight. Both reuse variables (`source`, `target`, `target_tolerance`, `accepted_target_angles`, `primitive_class_is_straight`) that already exist in both kernels under the same names -- confirm this with `grep -n "target_tolerance\|accepted_target_angles" src/astar.rs` before editing, since the crossing kernel computes `target_tolerance` and `accepted_target_angles` near its own line 4427-4428.

Insert the equivalent two guards into the crossing kernel's own neighbor-expansion loop, which begins near line 4555 (`for primitive_idx in primitive_order.into_iter().take(primitive_order_len) {`) as of this writing. Place the first guard immediately after the existing `stats.primitive_generated_by_class[primitive_class] += 1;` line (near 4562) and before the existing crossing-specific "pending straight run after a crossing" logic that follows it -- the order relative to that unrelated logic does not matter functionally, since both are independent `continue`-based rejections, but placing the terminal-straight check first keeps it visually adjacent to the plain kernel's equivalent placement for future readers. Place the second guard immediately after the crossing kernel computes `next_angle` (near line 4618, `let next_angle = primitive.end_angle % 8;`) and before its own bounds check (`if !bounds.contains(next_x, next_y) {`), mirroring the plain kernel's placement exactly.

Add a new Rust unit test named `terminal_straight_requirement_rejects_immediate_port_bends_in_crossing_kernel` in the `#[cfg(test)] mod tests` block of `src/astar.rs`, directly modeled on the existing plain-kernel test `terminal_straight_requirement_rejects_immediate_port_bends` (locate it with `grep -n "fn terminal_straight_requirement_rejects_immediate_port_bends" src/astar.rs`). The existing test builds an `AStarConfig` with `require_terminal_straights: true` and calls `route_single_net_with_config`; the new test must instead call `route_single_net_with_crossing_config` (the public wrapper around the crossing kernel, declared at line 2359 as of this writing), which additionally requires a `CrossingSearchConfig` argument with at least one entry in `partners` (an empty `partners` list makes the function return `None` immediately, at line 2370-2374, without exercising the search at all -- this is a real early-return in the production code, not a test artifact, so the test's partner does not need to actually interact with the route being tested; it only needs to exist). Use this pattern, adapted from an existing crossing-kernel test in the same file (`grep -n "fn crossing_move_accepts_benes8_route15_diagonal_sequence" src/astar.rs` to find the full original for reference):

    let crossing = CrossingSearchConfig {
        net_id: <the source net's id used in the test>,
        partners: vec![CrossingSearchPartner {
            net_id: <any other id>,
            waypoints: vec![/* any short path far away from the test route, so it is never actually reached */],
            target_terminal_bump_guard: None,
        }],
        min_straight_cells: 2,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: true,
        terminal_bump_guard: None,
    };

The test should route between a source and target chosen so that, absent `require_terminal_straights`, the shortest path would bend immediately at the source or target (the existing plain-kernel test's own source/target pair, `State::new(1, 1, 0)` to `State::new(3, 3, 2)`, is a reasonable starting point to adapt, adjusted only if the crossing kernel's additional bounds/partner requirements force a different obstacle map size). Assert, as the existing plain-kernel test does, that the first primitive used is straight-class and the last primitive's start angle equals its own end angle (i.e., it did not bend on its final step).

Validate Milestone 1 with:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib terminal_straight_requirement

Expect two passing tests: the pre-existing plain-kernel one and the new crossing-kernel one. Then run the full Rust suite (`cargo test --lib`, no filter) to confirm nothing else broke, since narrowing the crossing kernel's accepted primitives at source/target could in principle change which routes some other existing crossing-kernel test expects to find -- if any pre-existing crossing-kernel test starts failing, read its failure message and its `AStarConfig` construction: if that test does not set `require_terminal_straights: true` explicitly (checked with `grep -n "require_terminal_straights" src/astar.rs` around the failing test), then the new guards should not affect it at all, since both new guards are gated on `config.require_terminal_straights` being true, and any failure would indicate a mistake in the guard placement, not an intended consequence, and must be fixed before proceeding.

**Milestone 2** removes the `collision_crossing_mode`-gated weight override in `translation/route_rust.py` and re-validates `crossing_loss`. Locate the block with `grep -n "collision_crossing_mode = bool" translation/route_rust.py` (lines 6415-6441 as of this writing):

    collision_crossing_mode = bool(self.enable_crossings) and self.crossing_mode in {
        "collision",
        "lidar-pure",
    }
    if (
        self.allow_45_degree_turns
        and not collision_crossing_mode
        and hasattr(self.astar_cfg, "max_iterations")
    ):
        self.astar_cfg.max_iterations = min(int(self.astar_cfg.max_iterations), 50_000)
    if collision_crossing_mode and hasattr(self.astar_cfg, "heuristic_weight"):
        collision_heuristic_weight = os.environ.get(
            "PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT"
        )
        self.astar_cfg.heuristic_weight = (
            float(collision_heuristic_weight)
            if collision_heuristic_weight
            else 1.0
        )
    elif self.allow_45_degree_turns and hasattr(self.astar_cfg, "heuristic_weight"):
        self.astar_cfg.heuristic_weight = max(float(self.astar_cfg.heuristic_weight), 1.25)
    if collision_crossing_mode and hasattr(self.astar_cfg, "bend_weight"):
        self.astar_cfg.bend_weight = float(self.astar_cfg.bend_weight)
    elif self.allow_45_degree_turns and hasattr(self.astar_cfg, "bend_weight"):
        # LiDAR heavily penalizes bends relative to propagation. Matching that
        # scale keeps 45-degree A* from spending work on short zig-zag variants.
        self.astar_cfg.bend_weight = max(float(self.astar_cfg.bend_weight), 12.0)

Replace it with a version that keeps the `max_iterations` cap logic completely unchanged (per this plan's Decision Log, that divergence is out of scope) but removes `collision_crossing_mode` from the `heuristic_weight` and `bend_weight` decisions entirely, so both always use the plain, 45-degree-tuned path when `self.allow_45_degree_turns` is true, regardless of crossing mode:

    collision_crossing_mode = bool(self.enable_crossings) and self.crossing_mode in {
        "collision",
        "lidar-pure",
    }
    if (
        self.allow_45_degree_turns
        and not collision_crossing_mode
        and hasattr(self.astar_cfg, "max_iterations")
    ):
        self.astar_cfg.max_iterations = min(int(self.astar_cfg.max_iterations), 50_000)
    if self.allow_45_degree_turns and hasattr(self.astar_cfg, "heuristic_weight"):
        self.astar_cfg.heuristic_weight = max(float(self.astar_cfg.heuristic_weight), 1.25)
    if self.allow_45_degree_turns and hasattr(self.astar_cfg, "bend_weight"):
        # A bend costs bend_weight per angle-eighth of turn (see
        # src/primitives.rs's bend_cost field); boosting it keeps 45-degree
        # A* from spending work on short zig-zag variants. This applies
        # uniformly regardless of crossing_mode -- crossing-awareness only
        # changes whether/what a *crossing* costs (see crossing_loss,
        # translation/route_rust_crossing_plan.py), not how bends are priced.
        self.astar_cfg.bend_weight = max(float(self.astar_cfg.bend_weight), 12.0)

`collision_crossing_mode` remains defined and used (by the still-unchanged `max_iterations` logic immediately above it), so do not delete the variable itself -- only its use in the two weight blocks.

Delete the now-fully-unused environment variable read. Confirm it truly has no other reader before deleting, with:

    grep -rn "PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT" --include="*.py" --include="*.rs" --include="*.md" .

As of this writing this returns exactly one match, the deleted line itself, confirming it is safe to remove outright (no test, benchmark script, or documentation file references it).

Next, re-validate `crossing_loss`. This does not require any code change to explore, because the existing `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM` environment variable (`translation/route_rust_crossing_plan.py:30`) already overrides the default at runtime. Rebuild the extension first (the weight-unification change above must be compiled in), then run a crossing-heavy benchmark -- `multiportmmi_8x8` under its stable-baseline crossing configuration is the fastest one available and was the benchmark used to originally discover this whole issue -- at a small sweep of `crossing_loss` values including the current default, and compare crossing counts and route quality using this diagnostic pattern (adapted from a disposable scratch script used earlier in this investigation; write it to a temporary file, e.g. `/tmp/check_crossing_loss_sweep.py`, and delete it when done since it is not part of the repository):

    import sys, os
    sys.path.insert(0, ".")
    os.environ["PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT"] = "0.05"
    os.environ["PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES"] = "90"
    # Set this to each value being swept before running, e.g.:
    # os.environ["PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM"] = "50.0"

    import translation.route_rust as route_rust_mod
    captured = {}
    _orig_dispatch = route_rust_mod._RouteNetsRustSession._dispatch_native_routing
    def _dispatch_tap(self, route_jobs):
        result = _orig_dispatch(self, route_jobs)
        captured["router"] = self.router
        return result
    route_rust_mod._RouteNetsRustSession._dispatch_native_routing = _dispatch_tap

    from routing_flow import run_routing_flow
    run_routing_flow(
        "multiportmmi_8x8",
        enable_crossings=True,
        crossing_mode="lidar-pure",
        fanout_access_mode="static-stubs",
        routing_window_scale=0.35,
        foreign_port_keepout_cells=0,
    )
    events = captured["router"].crossing_events()
    print(f"crossing_loss={os.environ.get('PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM', 'default')} total_crossing_events={len(events)}")

Run this at, at minimum, the current default (`50.0`), a value close to what the un-boosted-`bend_weight` era implied it should feel like relative to the new bend cost scale (roughly `12x` the old effective ratio, i.e. in the `500-600` range, since a 90-degree bend now costs `24.0` instead of `2.0` and the goal is to preserve roughly the same "crossing is about 25x one bend" relationship the un-tuned default happened to produce), and at least one point in between (for example `150.0` or `200.0`). For each value, record the total crossing-event count and, separately, confirm via `build/verification/*.json` that `multiportmmi_8x8`'s stable-baseline run still completes with `error_count=0` (a `crossing_loss` set too high could in principle make the search prefer an illegal-crossing-avoiding detour that fails elsewhere, though this has not been observed and is not expected). Choose the smallest value that keeps crossing-event counts and route legality consistent with the pre-Milestone-2 baseline (the actual pre-change crossing-event count for this exact benchmark and config, captured once before making any Milestone 2 edit, is the number to compare against) -- prefer the smallest value that preserves that baseline rather than the largest one tried, since a needlessly large `crossing_loss` would bias the search away from using crossings even when a crossing is legitimately the best available option, which is a real design goal of the crossing feature itself (crossings should be avoided when a clean same-net route exists, not avoided unconditionally). Record the chosen value, the sweep data that justified it, and update `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` in `translation/route_rust_crossing_plan.py:29` to that value (or leave it at `50.0` with a recorded rationale, if the sweep shows `50.0` already behaves acceptably at the new weight scale -- do not assume either outcome in advance).

Validate Milestone 2 with:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" maturin develop --release
    PYTHONPATH=. .venv/bin/python -m pytest -q
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

Expect `pytest -q` to report all tests passing (no test in the current suite asserts on the specific pre-change `bend_weight`/`heuristic_weight` values under collision/lidar-pure mode, confirmed by `grep -rn "collision_crossing_mode\|bend_weight\|heuristic_weight" tests/` returning no matches as of this writing, so this change is not expected to require any test updates beyond the new Rust unit test added in Milestone 1). Expect the `multiportmmi_8x8` run to complete with `build/verification/multiportmmi_8x8_verification.json` (or the equivalently-named file this benchmark produces; confirm the exact filename with `ls build/verification/` after the run) reporting `error_count=0`.

**Milestone 3** confirms the fix's user-visible effect and runs the complete validation ladder this repository has used for every change this session, then records the outcome. First, confirm `n_7`/`n_8` (net ids 8 and 9) now route without the anomalous 90-degree-only bend shape, reusing the diagnostic pattern above but capturing route geometry instead of crossing events (adapt the `realize_routed_net_records` monkeypatch tap used earlier in this same investigation, or inspect `build/routed_multiportmmi_8x8.gds` visually as the repository owner originally did). Then run the full ladder:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    PYTHONPATH=. .venv/bin/python -m pytest -q
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

The last command is expected to still fail with the pre-existing, separately-tracked `n_50` error documented in `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` -- that failure is out of scope for this plan and its presence or absence must not be treated as this plan's success criterion; only confirm that its error text is unchanged (still names `n_50` with the same `candidate_blockers=[49, 50]` signature), which would indicate this plan's changes did not alter that unrelated failure.

Update `.agent/REPOSITORY_STATE.md` with this plan's outcome (both the code changes and the final chosen `crossing_loss` value with its rationale), following the structure already used there for every other completed ExecPlan this session.

## Concrete Steps

See the exact commands embedded in each milestone's description above in Plan of Work; they are the authoritative, exact commands for this plan and are not repeated redundantly here. Run every command from the repository root, `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter` (do not hard-code this absolute path in any script or test; it is stated here only to orient a reader unfamiliar with the checkout).

## Validation and Acceptance

Milestone 1: `cargo test --lib` reports all tests passing, including a new test demonstrating that `require_terminal_straights: true` is honored by `route_single_net_with_crossing_config` the same way it already is by `route_single_net_with_config`.

Milestone 2: `pytest -q` reports all tests passing; the `multiportmmi_8x8` stable-baseline crossing-configuration run completes with `error_count=0`; a recorded `crossing_loss` sweep (values tried, crossing-event counts observed, legality outcome observed) justifies the final chosen default, whether that default changes from `50.0` or is confirmed to stay at `50.0`.

Milestone 3: the full validation ladder (`cargo test --lib`, `pytest -q`, `benes_4x4`, `multiportmmi_8x8` both configurations, `benes_8x8`, `benes_16x16`, `multiportmmi_16x16` stable-baseline) completes with `error_count=0` on every benchmark except two documented, non-blocking exceptions: `multiportmmi_16x16` stable-baseline still failing with the unchanged, separately-tracked `n_50` error (`.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md`), and `multiportmmi_8x8` **bare-defaults** (not the stable-baseline crossing configuration, which is clean) failing on `n_67` for the documented, understood, currently-blocked endpoint-correction reason recorded in Surprises & Discoveries and the Decision Log above -- plus a direct observation (geometry inspection or a route-record diagnostic) confirming `n_7`/`n_8` no longer exhibit the anomalous all-90-degree bend shape that originally motivated this investigation.

## Idempotence and Recovery

All diagnostic scripts described in this plan are read-only with respect to the repository; they only read `translation/route_rust.py` and write to the gitignored `build/` directory or to `/tmp`. The `crossing_loss` sweep in Milestone 2 is safe to re-run any number of times with different environment-variable values; it does not require reverting anything between runs. If a Rust change in Milestone 1 causes an unexpected pre-existing test failure, do not proceed to Milestone 2 until it is understood and fixed (or the guard placement is revised) -- Milestone 2's weight-unification change is independent of Milestone 1's terminal-straight fix and could be implemented and validated first if that proves easier, but this plan sequences terminal-straights first because it is the smaller, more mechanically-verified change of the two, and de-risks Milestone 2's larger weight/cost re-tuning by not having two unvalidated changes in flight at once.

## Artifacts and Notes

(to be filled in as Milestone 2's `crossing_loss` sweep and Milestone 3's final validation runs produce concrete numbers worth preserving)

## Interfaces and Dependencies

None prescribed beyond what is already described in Context and Orientation. This plan does not depend on `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` completing first, and that plan does not depend on this one; both may proceed independently, though re-running the full validation ladder after both are complete (rather than only after each individually) is good practice before considering the repository's benchmark baseline fully re-stabilized.
