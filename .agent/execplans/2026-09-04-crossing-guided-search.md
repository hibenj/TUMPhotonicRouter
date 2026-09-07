# Contribution 1: precomputed crossings as guidance for A* ("lidar-guided")

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Owner (2026-09-04): first contribution on top of the lidar-pure baseline. The topology plan (`photonic_router.crossing_plan.build_crossing_plan`: rank inversions per stage = which pairs of nets must cross) is precomputed and used to GUIDE the A* search. Hard requirements: the engine must still be able to insert additional crossings when needed, and it decides by the cost function; predicted crossings are accepted faster / cost nothing in the search. The lidar-pure path itself stays untouched (standing rule in `.agent/ORCHESTRATOR.md`; memory: baseline vs contributions).

**Switchability (owner, 2026-09-04 14:30, verbatim intent): the contribution must be clearly switchable on and off. Exactly one of three configurations runs at a time: (a) lidar-pure = the baseline (`--crossing-mode lidar-pure`, today's stable defaults), (b) contribution 1 = this plan, crossing-guided A* (`--crossing-mode lidar-guided`, opt-in), (c) contribution 2 = the precomputed crossing STRUCTURES (`--preplaced-crossing-grids true`, opt-in, forces router crossings off). Nothing of (b) or (c) leaks into (a): the guidance code path is entered only in the `lidar-guided` mode, `STABLE_ROUTING_FLAGS` of the benchmarks stay baseline, and the flow rejects `lidar-guided` combined with preplaced grids (they are alternative contributions, measured separately against the same baseline). Pinned by: a kernel test (empty guidance == baseline pricing), a flow test (mode combination rejected), and the benchmark verdicts under lidar-pure staying identical after the contribution lands.**

Status: COMPLETE (S1 landed `a94750b` and is the contribution; S2 landed off by default; S3 landed opt-in) -- see Outcome at the end. The line below is the original draft header.
Original status: DESIGN DRAFT -- nothing implemented; the owner chooses the slices.

## What the plan knows, and how good it is (measured 2026-09-04 14:20)

Plan vs the realized lidar-pure crossings of today's layouts (`scratchpad/plan_vs_realized.py`, pairs as unordered net-name sets):

| benchmark | plan events (stages) | realized | realized in plan | realized NOT in plan | plan never realized |
|---|---|---|---|---|---|
| multiportmmi_8x8 | 33 (16) | 33 | 33 | 0 | 0 |
| benes_8x8 | 16 (6) | 16 | 16 | 0 | 0 |
| multiportmmi_16x16 | 63 (17) | 65 | 63 | 2 (n_129 x n_130 crossed twice = the braid the repair did not undo) | 0 |
| benes_16x16 | 88 (8) | 88 | 88 | 0 | 0 |
| multiportmmi_32x32 | 121 (18) | 121 | 121 | 0 | 0 |
| benes_32x32 | 416 (10) | 416 | 416 | 0 | 0 |

So: (1) the plan is EXACT -- every realized crossing is a planned pair and every planned pair is realized (once); lidar-pure already reaches the topological minimum everywhere except the mm16 braid. (2) The contribution's gain is therefore NOT the crossing count; it is search effort (A* today first explores the detour space worth 200 um per planned crossing before accepting it), route length/shape, and robustness (fewer exhausted searches, fewer repairs) -- to be measured. (3) Caveat from 2026-09-03 P3 (benes32 net-273 testbed): search loss 200/50/20 um changed expansions only 5.24M -> 4.57M, same route -- the price is not the dominant time driver for that hard net; the benefit may be moderate and must be measured per benchmark, not assumed. (4) Plan coverage bug: the multiportmmi metadata ships EMPTY `node_depths`/`node_ranks`/`edge_ranks` dicts; `analyze_graph_topology` treats `{}` as explicit and fails with `KeyError: 'fanout_yb_0_0'`. With `{}` -> derive (`or None`) the plan is exact (table). Small fix, tests first (slice S0).

Per net the plan gives: the set of partners it must cross, exactly one crossing per pair, the stage (for our benchmarks every net lives in one stage, so the stage corridor adds nothing), and the swap order (= the order in which the net passes its partners along the stage).

## Design options (ranked)

**O1 -- price discount per planned pair (the owner's proposal, recommended first slice).** In the kernel the crossing price is `extra_cost = crossing_count * crossing_loss` (`src/astar.rs:3898`, `LiveCrossingHook::evaluate`). Change: per crossing event, `expected(partner) ? planned_loss : crossing_loss`, with `planned_loss = 0` by default (env/CLI-tunable). `CrossingSearchConfig` gets `planned_partner_ids` (per net, from the plan); the partner id is known at pricing time (the collided owner). Unplanned crossings keep the 200 um search loss -> still possible, still decided by cost. No new state, no key change, no new rule: pure cost. Braids stay handled by the existing braid repair.

**O2 -- per-pair budget (one free crossing per planned pair).** The plan predicts exactly ONE crossing per pair; a second crossing with the same partner is a braid and should pay full price. Needs "already crossed p" in the search state: reuse the window-mode `crossed_mask` in `CrossingExtension` restricted to the planned partners of this net (<= 11 in mm32, few in benes; cap 64, beyond -> no discount) and put it in the Tier-2 key (exactness; multiplicity small because planned partners are few). Turns braid avoidance into the cost function instead of a post-hoc repair; measurable on mm16 (the surviving braid). Second slice, only if O1 shows braids or the owner wants the cleaner form.

**O3 -- plan-driven net ORDER (orthogonal, no kernel change).** In sequential routing the later net pays the crossing; which of a planned pair goes first decides whether the first net's greedy path seals the second's pocket (the 2026-09-04 braid finding). The plan's stage structure + swap order gives a principled order (e.g. per stage by target rank, LiDAR-style level-then-length). Cheap experiment via the existing `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS` hoist; would become a `--net-order plan` option. Candidate for slice 3.

**O4 -- heuristic pull (NOT recommended).** Window mode has `crossing_progress_heuristic` (-10000 per crossed partner + distance to the next partner). It is inadmissible and overrides the cost function -- exactly what the owner does not want. With `planned_loss = 0` the length heuristic stays admissible and nothing else is needed.

**O5 -- plan as partner/lookup filter (NOT for guidance).** Restricting the lidar-pure partner lookup DB to planned partners would forbid unplanned crossings; rejected. (At most a performance trick later, and only with a fallback to the full map.)

**O6 -- reporting / quality metric (part of every slice).** Each realized crossing gets `planned: bool`; the report gets `planned_crossing_count`, `realized_planned`, `realized_unplanned`, `plan_unrealized`. This is the contribution's own quality measure and the guard that unplanned crossings stay visible.

Later ideas, not now: use the plan when routing the EARLIER net of a pair (it knows partner X will cross it later -> prefer a straight run where X's corridor is; helps sealed pockets); a per-net "expected crossing count" as a plausibility warning when a route has far more events than planned.

## Plumbing (for O1)

- New crossing mode `lidar-guided` (name open): lidar-pure mechanics + plan. `_build_crossing_plan_info` builds the plan in this mode too, but does NOT call `set_crossing_constraints` (those are the hard window-mode constraints that leaked into lidar-pure on 2026-08-25); instead a new `router.set_crossing_guidance(pairs, planned_loss)`. `allow_only_expected_pairs=False` as in lidar-pure.
- Rust: `CrossingGuidance { planned_partners: FxHashMap<NetId, FxHashSet<NetId>>, planned_loss: f64 }` on the router; `CrossingSearchConfig.planned_partner_ids: FxHashSet<NetId>` + `planned_crossing_loss` filled per net; pricing at the one site. Stats: `crossing_planned_accepted` / `crossing_unplanned_accepted` counters in `RouteSearchStats`.
- lidar-pure unchanged by construction (guidance empty -> identical pricing); pinned by a kernel test (empty guidance == old cost) and by the benchmark verdicts under `--crossing-mode lidar-pure`.
- Tests first: kernel test with two partners, one planned: the planned crossing costs 0, the unplanned one `crossing_loss`; a route choosing between a planned crossing and a longer detour takes the crossing; with guidance empty the verdicts equal today's. Python: plan coverage for multiportmmi (S0), report fields (O6).
- Measurement per benchmark (lidar-pure vs lidar-guided, same build): attempts/failures/repairs, crossings (planned/unplanned), total route length, expansions (`route search:` astar_loop + native counters), wall time; both verifications 0 errors; GDS pair for the owner.

## Progress

- [x] Design + plan-quality measurement (this document).
- [x] S0 (15:10, `5cd6fc8`): `analyze_graph_topology` treats an empty depth/rank mapping as "derive" (test `test_topology_plan_derives_depths_and_ranks_when_metadata_dicts_are_empty`, red -> green; mm8 plan = 33 events).
- [x] S1 implementation (15:20-16:20, `a94750b`), tests first:
  - Kernel: `CrossingSearchPartner.crossing_loss_override: Option<f64>` (per-partner search price; `None` = baseline `crossing_loss`), `CrossingMoveOutcome.crossing_cost` summed per accepted event at the single accept site, priced at the single pricing site (`LiveCrossingHook::evaluate`); stat `crossing_accepted_planned`. Tests `planned_partner_crossing_uses_its_own_price_and_unplanned_pays_crossing_loss` (two vertical partners, one planned: cost = length + 1 x 3.0) and `partners_without_price_override_price_like_the_baseline` (cost = length + 2 x 3.0). Lesson: `stats.crossing_accepted` counts accepted MOVES over the whole search, not path events -- the price is the exact assertion.
  - Context: `CrossingGuidance { planned_pairs, planned_crossing_loss }` on `CrossingContext`, separate from constraints; `PyPhotonicRouter.set_crossing_guidance / clear_crossing_guidance / crossing_guidance_planned_pair_count`; applied in `crossing_search_config` (the one config-build site) as partner overrides. BUG found by the mm8 smoke (identical A* counters, `accepted_planned=0`): `replace_constraints` rebuilt the context and dropped the guidance, and the flow calls `set_crossing_constraints([])` right after `set_crossing_guidance` -> guidance now survives constraint/config replacement (pinned in `guidance_is_soft_and_separate_from_constraints`).
  - Python: `translation/crossing_modes.py` (`normalize_crossing_mode`, `is_lidar_mode`, `is_guided_mode`, `is_collision_mode`; every former `== "lidar-pure"` site now uses the helper so lidar-guided inherits the lidar-pure mechanics exactly); `_build_crossing_plan_info` builds the plan in lidar-guided and hands it over ONLY via `set_crossing_guidance` (constraints stay `[]`), `PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM` (default 0); flow `--crossing-mode lidar-guided`, rejects the combination with `--preplaced-crossing-grids`; stdout line `crossing search: mode=… search_loss=… planned_pairs=… planned_loss=…`; report: each crossing `planned: true/false/None`, metrics `planned_crossing_count / realized_planned_crossing_count / realized_unplanned_crossing_count / plan_unrealized_pair_count`, `Crossing hot path: … accepted_planned=…`. Tests: `test_lidar_guided_builds_the_plan_as_guidance_not_constraints`, `test_lidar_pure_still_ignores_the_topology_plan`, `test_planned_crossing_search_loss_env_override`, `test_lidar_guided_shares_the_lidar_pure_search_penalty`, `test_flow_rejects_guided_search_combined_with_preplaced_crossing_grids`. cargo 443/443, pytest 366 passed / same 10 baseline failures.
  - Smoke mm8 (16:17, same build, back to back): lidar-pure expanded 2 838 915, astar_loop 8.9 s; **lidar-guided expanded 1 566 887 (-45 %), astar_loop 5.0 s, accepted_planned 21 726 of 25 178 accepted moves**; both 111/0/0, 33 crossings (guided: 33 planned / 0 unplanned / 0 unrealized), identical total route length 24 823.0 um, verifications 0 errors.
- [x] S1 A/B (16:20-17:10, `scratchpad/c1ab.summary` + `c1ab_table.py`, build `a94750b`, same machine, back to back, 3-minute watchdog never fired; every run rc=0, crossing AND photonic verification 0 errors):

| benchmark | mode | att/fail/rep | expanded | A* s | wall s | crossings (planned/unplanned/unrealized) | total length um |
|---|---|---|---|---|---|---|---|
| mm8 | lidar-pure | 111/0/0 | 2 838 915 | 8.8 | 14.2 | 33 | 24 823.0 |
| mm8 | lidar-guided | 111/0/0 | 1 566 887 (-45 %) | 5.1 | 10.4 | 33 (33/0/0) | 24 823.0 |
| benes8 | lidar-pure | 52/4/0 | 5 786 027 | 33.1 | 35.8 | 16 | 26 159.7 |
| benes8 | lidar-guided | 52/4/0 | 3 650 808 (-37 %) | 25.2 | 27.9 | 16 (16/0/0) | 26 171.0 |
| mm16 | lidar-pure | 227/2/1 | 11 951 295 | 32.0 | 42.7 | 65 | 89 534.9 |
| mm16 | lidar-guided | 227/2/1 | 6 903 182 (-42 %) | 18.5 | 29.4 | 65 (63/2/0) | 89 506.8 |
| benes16 | lidar-pure | 136/8/0 | 33 011 532 | 161.2 | 165.8 | 88 | 77 177.0 |
| benes16 | lidar-guided | 136/8/0 | 18 862 560 (-43 %) | 99.1 | 103.7 | 88 (88/0/0) | 77 202.4 |
| mm32 | lidar-pure | 457/0/5 | 82 225 492 | 268.9 | 303.0 | 121 | 334 671.9 |
| mm32 | lidar-guided | 457/0/5 | 45 996 755 (-44 %) | 138.9 | 172.6 | 121 (121/0/0) | 334 726.8 |
| benes32 | lidar-pure | 340/16/2 | 187 628 317 | 977.4 | 991.9 | 416 | 229 985.6 |
| benes32 | lidar-guided | 340/16/2 | 124 581 451 (-34 %) | 654.7 | 669.5 | 416 (416/0/0) | 230 083.9 |

  Reading: identical attempts/failures/repairs and identical crossing counts on every benchmark (the mm16 braid pair is reported as the 2 unplanned crossings -- exactly what O2 would price); total route length within +-0.05 % (guided is 11-98 um longer on four benchmarks, 28 um shorter on mm16: with free planned crossings A* no longer trades a few cells of detour against the 200 um price); search effort -34 % to -45 % expansions, A* time -24 % (benes8) to -48 % (mm32), benes32 977 -> 655 s. Unplanned crossings remained possible throughout (price 200 um) and none were needed.
- [x] S2 implementation (17:20-17:35, owner "mach S2, tests zuerst"): `CrossingSearchPartner.single_discounted_crossing` -- only the first crossing of a budgeted planned partner on a path is discounted, every further one pays `crossing_loss`; tracked in the search key via `crossed_mask` (one bit per budgeted partner, assigned at search setup by `crossing_partner_budget_bits`, at most 64; the field is otherwise unused in lidar modes because window-mode partner tracking is off). Stat `crossing_accepted_over_budget`. `CrossingGuidance.single_discounted_crossing_per_pair` (default true), PyO3 `set_crossing_guidance(pairs, loss, single_discounted_crossing_per_pair=True)`, env `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET` (1 = default, 0 = unlimited = S1 behaviour), stdout `budget=1/pair|unlimited`, summary `accepted_over_budget=`. Tests first: kernel `second_crossing_of_a_budgeted_planned_partner_pays_the_full_price` (U-shaped planned partner crossed twice: cost = length + 3.0), `without_the_budget_every_crossing_of_a_planned_partner_is_discounted` (cost = length), `budget_is_per_partner_two_planned_partners_are_both_discounted`; context flag test; Python `test_planned_crossing_budget_env_switches_s2_off`. cargo 446/446, pytest baseline set only.
- [x] S2 smoke on mm16 (17:35): budget 0 vs 1 -- identical verdicts (227/2/1, 65 crossings, length 89 506.8), expanded 6 903 182 -> 6 949 392 (+0.7 %), over-budget accepted moves 12 755 (explored detours that re-cross a planned partner; none on final paths). CORRECTION of the S2 motivation: the mm16 braid (n_129 x n_130) is an UNPLANNED pair crossed twice (63 planned events, 65 realized), so a per-planned-pair budget cannot touch it -- it already pays 200 um per crossing and the braid repair could not reduce it. S2's value is the exact plan semantics ("one crossing per pair") in the cost function; its cost is the mask in the key.
- [x] S2 A/B (17:36-18:00, budget 1, `scratchpad/s2.summary`, vs the 16:20 S1 runs, same build family, watchdog never fired, all rc=0, both verifications 0 errors):

| benchmark | S1 expanded | S2 expanded | delta | S1 -> S2 A* s | verdicts / crossings / length | over-budget moves explored |
|---|---|---|---|---|---|---|
| mm8 | 1 566 887 | 1 566 282 | -0.0 % | 5.1 -> 4.9 | identical (111/0/0, 33, 24 823.0) | 4 344 |
| benes8 | 3 650 808 | 3 650 786 | -0.0 % | 25.2 -> 26.2 | identical (52/4/0, 16, 26 171.0) | 5 618 |
| mm16 | 6 903 182 | 6 949 392 | +0.7 % | 18.5 -> 18.6 | identical (227/2/1, 65, 89 506.8) | 12 755 |
| benes16 | 18 862 560 | 18 860 352 | -0.0 % | 99.1 -> 99.7 | identical (136/8/0, 88, 77 202.4) | 39 828 |
| mm32 | 45 996 755 | 45 958 023 | -0.1 % | 138.9 -> 138.5 | identical (457/0/5, 121, 334 726.8) | 57 327 |
| benes32 | 124 581 451 | 133 906 064 | **+7.5 %** | 654.7 -> 740.3 | identical (340/16/2, 416, 230 083.9) | 864 722 |

  Reading: S2 changes NO route on any benchmark (lengths equal to 0.1 um) -- no final path ever crossed a planned partner twice under S1 either; the budget only re-prices explored detours. Its cost is the mask in the Tier-2 key: negligible up to mm32, +7.5 % expansions / +86 s on benes32 (the most partners per net, the most over-budget detours). So S2 buys exact plan semantics in the cost function and nothing measurable in the layouts; the owner decides the default of `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET` (1 = exact, 0 = S1 pricing).
- [x] S3 implementation (18:10-18:40, owner "mach S3, tests zuerst"): `translation/route_order.py` -- `depth_by_node_from_jobs` (the batch-graph depth, extracted from the session) and `order_route_jobs(net_order=…)` with `topological` (default = baseline: depth, declaration), `topological-span` (depth, shortest grid span first = the 2026-08-27 `PHOTONIC_ROUTER_LAYER_ORDER=span` experiment, now an option and the env var is gone), `plan-crossings-desc` / `plan-crossings-asc` (depth, most/fewest planned crossings first; needs the lidar-guided plan, otherwise a loud ValueError). Flow flag `--net-order`, plumbed like `heuristic_mode`; stdout `net order: …` when not default. Tests first: `tests/test_route_order.py` (6: depth, each rule, tiebreaks, input guards), flow test `test_plan_crossing_net_order_needs_the_guided_mode`. Prior evidence to keep in mind: on 2026-08-27 (old kernel) span-ascending as a global default broke lidar-pure Benes crossing discovery (8/16 verification errors) -- the legal kernel of 2026-09-03 may or may not have changed that.
- [x] S3 ladder (21:00-21:15, guided mode, budget default 0, `scratchpad/s3.summary`, all rc=0, all verifications 0 errors, watchdog never fired; "topological" = the S1 runs of 16:20):

| benchmark | order | att/fail/rep | expanded (vs topological) | A* s | crossings (unplanned) | length um |
|---|---|---|---|---|---|---|
| mm8 | topological | 111/0/0 | 1 566 887 | 5.1 | 33 (0) | 24 823.0 |
| mm8 | topological-span | 111/0/0 | 1 706 511 (+9 %) | 5.6 | 33 (0) | 24 693.9 |
| mm8 | **plan-crossings-desc** | 111/0/0 | 1 280 318 (-18 %) | 3.9 | 33 (0) | 24 811.7 |
| mm8 | plan-crossings-asc | 119/5/1 | 2 326 103 (+48 %) | 27.0 | 35 (2) | 24 832.4 |
| benes8 | topological | 52/4/0 | 3 650 808 | 25.2 | 16 (0) | 26 171.0 |
| benes8 | topological-span | 48/0/0 | 7 586 465 (+108 %) | 27.5 | 16 (0) | 26 208.7 |
| benes8 | **plan-crossings-desc** | 48/0/0 | 190 890 (-95 %) | 0.5 | 16 (0) | 26 064.1 |
| benes8 | plan-crossings-asc | 48/0/0 | 7 586 465 (+108 %) | 27.4 | 16 (0) | 26 208.7 |
| mm16 | topological | 227/2/1 | 6 903 182 | 18.5 | 65 (2) | 89 506.8 |
| mm16 | topological-span | 224/1/0 | 4 597 433 (-33 %) | 13.3 | 63 (0) | 89 397.1 |
| mm16 | **plan-crossings-desc** | 223/0/0 | 5 335 411 (-23 %) | 14.6 | 63 (0) | 89 497.8 |
| mm16 | plan-crossings-asc | 228/1/2 | 6 016 243 (-13 %) | 67 (4) | 89 515.3 |
| benes16 | topological | 136/8/0 | 18 862 560 | 99.1 | 88 (0) | 77 202.4 |
| benes16 | topological-span | 128/0/0 | 23 633 102 (+25 %) | 94.5 | 88 (0) | 77 221.0 |
| benes16 | **plan-crossings-desc** | 129/1/0 | 10 342 960 (-45 %) | 38.7 | 88 (0) | 76 993.4 |
| benes16 | plan-crossings-asc | 128/0/0 | 23 633 102 (+25 %) | 94.3 | 88 (0) | 77 221.0 |

  Reading: `plan-crossings-desc` (most planned crossings first) wins on every benchmark -- fewer expansions (-18 % to -95 %), fewer failed attempts (benes8 4 -> 0, benes16 8 -> 1, mm16 2 -> 0), no repairs, and on mm16 the braid is gone: 63 crossings = the plan's minimum, 0 unplanned, without any braid repair. Route length equal or shorter (benes16 -209 um). Mechanism: a net with many planned crossings routed first is a greedy straight line with nothing to legalize; each partner later crosses one committed straight once. `asc` is the mirror image (worse); `span` is mixed (+108 % on benes8 -- the 2026-08-27 finding in a milder form, no verification errors on the legal kernel). benes8 asc == span exactly (the same order there).
- [x] S3 32x32 with `plan-crossings-desc` (21:08-21:31): **mm32** 457/0/5, expanded 46 436 802 (+1 % vs topological), 121 crossings (0 unplanned), length +79 um, verifications 0/0 -- neutral (mm32's crossing-heavy nets are the output permutation; the 5 braid repairs stay). **benes32: FAILS the 3-minute rule** -- the run reached index 276 (net_id 282, `n_s7_9_o1_to_s8_3_i1`, the middle stage x=3864 -> 4134) after ~15.5 min and that net's first search ran > 181 s without a result (0 search failures, 0 rip-ups logged: still inside its first full-map attempt) -> killed by the watchdog at 21:31:05, rc=143. In the topological order the same stage was the slowest part too (route[275] 69 s, [273] 39 s, [276] 27 s) but finished; under desc the 32 widest-span nets of that stage are committed first as a dense pack of steep parallel diagonals and the shorter nets (282: rows 368 -> 698) must then cross that pack inside a narrow band -- the opposite of the ladder mechanism, where the first-routed straight lines are far apart. Slip on the way: the first benes32 attempt was killed by my own watchdog because its file pattern did not match the log name (`s3b_*` vs `s3_*`) -- 200 s of "no progress" on a non-existent file; fixed and rerun.
- [x] S3 verdict: `plan-crossings-desc` is a large win on the ladder (mm8 -18 %, benes8 -95 %, mm16 -23 % + braid gone, benes16 -45 % expansions, fewer failures, no repairs), neutral on mm32, and breaks benes32's middle stage. Not a default; `--net-order` stays opt-in with `topological` as default. Open for the owner: whether to pursue a hybrid (e.g. desc only in stages whose planned-crossing pack stays sparse, or an interleaving rule) or to investigate net 282 with the harness -- both are new experiments, not part of the S3 claim.

## Surprises & Discoveries

- (14:20) lidar-pure already realizes exactly the plan's crossing set on all six crossing benchmarks (mm16: +1 braid). The contribution must be justified by effort/quality/robustness numbers, not by crossing counts.

## Decision Log

- (2026-09-04, owner) Contribution 1 = precomputed crossings as A* guidance; the engine keeps the freedom to add crossings and decides by cost; planned crossings accepted faster / free.
- (2026-09-04 14:30, owner) Three mutually exclusive, switchable configurations: baseline lidar-pure / contribution 1 (guided A*, this plan) / contribution 2 (precomputed crossing structures, `--preplaced-crossing-grids`). Numbering per the owner: contribution 1 = guidance, contribution 2 = structures (the 2026-08-31 memory note had them the other way round; corrected).
- (2026-09-04 18:05, owner) "ja, default 0": `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET` defaults to 0 (S1 pricing); 1 switches the exact per-pair budget on. Python default `DEFAULT_SINGLE_DISCOUNTED_CROSSING_PER_PAIR = False` (the Rust `CrossingGuidance::new` keeps `true` as its own default; the flow always passes the flag explicitly). Was: S2 default: 1 (exact "one crossing per planned pair" semantics, +7.5 % search on benes32, identical layouts) or 0 (S1 pricing). Recommendation: 0 as default -- the measured routes are identical on all six benchmarks and the exact semantics stay one env switch away; revisit if a benchmark ever shows a planned pair crossed twice on a final path.
- (open for the owner) Which slices: S0+S1 first (recommended); mode name; `planned_loss` default 0 or a small positive value (a tiny price, e.g. 1 um, keeps A* from wandering through a planned partner twice without O2 -- to be tested).

## Outcomes & Retrospective

**Outcome (2026-09-04 17:15, S1 complete):** contribution 1 is in as `--crossing-mode lidar-guided`, opt-in, with the baseline untouched (empty guidance == baseline pricing, pinned by kernel test; lidar-pure verdicts in the A/B identical to this morning's). Effect on all six crossing benchmarks: same routes in every verdict that matters (attempts/failures/repairs, crossing count, both verifications clean), route length within +-0.05 %, and 34-45 % fewer A* expansions (benes32 977 -> 655 s, mm32 269 -> 139 s of search). The plan's prediction quality is the whole story: it is exact on these benchmarks, so paying nothing for planned crossings removes the detour space A* used to explore before accepting each of them.

**Retrospective:** measuring the plan against the realized crossings BEFORE designing settled what the contribution can and cannot claim (not crossing counts; search effort). The one bug (guidance dropped by `replace_constraints`) was caught by the cheapest possible check -- identical A* counters between the two modes on mm8 -- which argues for always printing the effective configuration and a mode-specific counter (`accepted_planned`) in the summary line. `is_lidar_mode` instead of string equality keeps the two lidar modes from drifting apart.

**S2/S3 outcome (2026-09-04 evening):** S2 (per-pair budget) landed, off by default (identical layouts, +7.5 % search on benes32). S3 (`--net-order`) landed, `topological` default; `plan-crossings-desc` is a large win on the four ladder benchmarks (up to -95 % expansions, mm16 braid gone) but neutral on mm32 and breaks the benes32 middle stage (net 282 > 3 min) -- an opt-in knob with a documented failure case, and the next question for the owner (hybrid order or net-282 investigation).

**Re-measured 2026-09-07 on the paper base (tag `paper-base-2026-09-07`, after the contribution 2 work; same machine, lidar-pure ladder and lidar-guided ladder back to back, benes_32x32 in the background):** every expansion count and verdict is bit-identical to the 2026-09-04 A/B table; the search is deterministic.

| benchmark | mode | att/fail/rep | expanded | A* s | crossings | length um |
|---|---|---|---|---|---|---|
| multiportmmi_8x8 | lidar-pure | 111/0/0 | 2,838,915 | 8.7 | 33 | 24,823.0 |
| multiportmmi_8x8 | lidar-guided | 111/0/0 | 1,566,887 | 5.0 | 33 | 24,823.0 |
| benes_8x8 | lidar-pure | 52/4/0 | 5,786,027 | 33.7 | 16 | 26,159.7 |
| benes_8x8 | lidar-guided | 52/4/0 | 3,650,808 | 25.6 | 16 | 26,171.0 |
| multiportmmi_16x16 | lidar-pure | 227/2/1 | 11,951,295 | 33.1 | 65 | 89,534.9 |
| multiportmmi_16x16 | lidar-guided | 227/2/1 | 6,903,182 | 18.8 | 65 | 89,506.8 |
| benes_16x16 | lidar-pure | 136/8/0 | 33,011,532 | 165.1 | 88 | 77,177.0 |
| benes_16x16 | lidar-guided | 136/8/0 | 18,862,560 | 103.4 | 88 | 77,202.4 |
| multiportmmi_32x32 | lidar-pure | 457/0/5 | 82,225,492 | 272.3 | 121 | 334,671.9 |
| multiportmmi_32x32 | lidar-guided | 457/0/5 | 45,996,755 | 143.4 | 121 | 334,726.8 |
| benes_32x32 | lidar-pure | 340/16/2 | 187,628,317 | 984.7 | 416 | 229,985.6 |
| benes_32x32 | lidar-guided | 340/16/2 | 124,581,451 | 653.4 | 416 | 230,083.9 |

**S3 follow-up, 2026-09-07 evening (owner: tackle the open parts on the side): `--net-order plan-crossings-hybrid`** = `plan-crossings-desc` inside depth layers of at most `HYBRID_DESC_MAX_LAYER_NETS` = 16 nets, declaration order in larger layers (`translation/route_order.py`; unit test `test_hybrid_order_is_desc_in_small_layers_and_declaration_in_large_ones`). Rationale: the S3 evidence separates by layer size, not by benchmark -- desc won on every layer of up to 16 nets and stalled the 32-net middle stage of benes_32x32. Measured on the paper base, lidar-guided, all verifications 0 errors:

| benchmark | S1 guided (topological) expanded / att/fail/rep / crossings | hybrid expanded / att/fail/rep / crossings | change |
|---|---|---|---|
| multiportmmi_8x8 | 1 566 887 / 111/0/0 / 33 | 1 280 318 / 111/0/0 / 33 | -18 % |
| benes_8x8 | 3 650 808 / 52/4/0 / 16 | 190 890 / 48/0/0 / 16 | -95 %, no failures |
| multiportmmi_16x16 | 6 903 182 / 227/2/1 / 65 | 5 335 411 / 223/0/0 / 63 | -23 %, braid gone, no repair |
| benes_16x16 | 18 862 560 / 136/8/0 / 88 | 10 342 960 / 129/1/0 / 88 | -45 %, 1 failure instead of 8 |
| multiportmmi_32x32 | 45 996 755 / 457/0/5 / 121 | identical (32-net layers -> declaration order) | 0 |
| benes_32x32 | 124 581 451 / 340/16/2 / 416 | identical, 657 s | 0 |

Verdict: strictly better than S1 on every ladder benchmark and identical on the 32x32 cases -- a safe candidate for contribution 1's default net order. Left opt-in: making it the default changes contribution 1's ladder layouts (mm16 65 -> 63 crossings), which is the owner's call for the paper (the S1 claim "identical layouts to the baseline with fewer expansions" would become "fewer crossings, fewer failures and fewer expansions"). The net-282 harness investigation is not needed for this rule and stays open.
