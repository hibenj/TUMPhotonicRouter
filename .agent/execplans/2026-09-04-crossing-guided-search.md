# Contribution 1: precomputed crossings as guidance for A* ("lidar-guided")

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Owner (2026-09-04): first contribution on top of the lidar-pure baseline. The topology plan (`photonic_router.crossing_plan.build_crossing_plan`: rank inversions per stage = which pairs of nets must cross) is precomputed and used to GUIDE the A* search. Hard requirements: the engine must still be able to insert additional crossings when needed, and it decides by the cost function; predicted crossings are accepted faster / cost nothing in the search. The lidar-pure path itself stays untouched (standing rule in `.agent/ORCHESTRATOR.md`; memory: baseline vs contributions).

**Switchability (owner, 2026-09-04 14:30, verbatim intent): the contribution must be clearly switchable on and off. Exactly one of three configurations runs at a time: (a) lidar-pure = the baseline (`--crossing-mode lidar-pure`, today's stable defaults), (b) contribution 1 = this plan, crossing-guided A* (`--crossing-mode lidar-guided`, opt-in), (c) contribution 2 = the precomputed crossing STRUCTURES (`--preplaced-crossing-grids true`, opt-in, forces router crossings off). Nothing of (b) or (c) leaks into (a): the guidance code path is entered only in the `lidar-guided` mode, `STABLE_ROUTING_FLAGS` of the benchmarks stay baseline, and the flow rejects `lidar-guided` combined with preplaced grids (they are alternative contributions, measured separately against the same baseline). Pinned by: a kernel test (empty guidance == baseline pricing), a flow test (mode combination rejected), and the benchmark verdicts under lidar-pure staying identical after the contribution lands.**

Status: DESIGN DRAFT -- nothing implemented; the owner chooses the slices.

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
- [ ] S0: plan coverage for multiportmmi (`{}` metadata -> derive), tests first.
- [ ] S1: O1 + O6 (mode, kernel pricing, report), tests first; ladder + both 32x32 A/B vs lidar-pure.
- [ ] S2 (optional): O2 per-pair budget; S3 (optional): O3 plan-driven order.

## Surprises & Discoveries

- (14:20) lidar-pure already realizes exactly the plan's crossing set on all six crossing benchmarks (mm16: +1 braid). The contribution must be justified by effort/quality/robustness numbers, not by crossing counts.

## Decision Log

- (2026-09-04, owner) Contribution 1 = precomputed crossings as A* guidance; the engine keeps the freedom to add crossings and decides by cost; planned crossings accepted faster / free.
- (2026-09-04 14:30, owner) Three mutually exclusive, switchable configurations: baseline lidar-pure / contribution 1 (guided A*, this plan) / contribution 2 (precomputed crossing structures, `--preplaced-crossing-grids`). Numbering per the owner: contribution 1 = guidance, contribution 2 = structures (the 2026-08-31 memory note had them the other way round; corrected).
- (open for the owner) Which slices: S0+S1 first (recommended); mode name; `planned_loss` default 0 or a small positive value (a tiny price, e.g. 1 um, keeps A* from wandering through a planned partner twice without O2 -- to be tested).

## Outcomes & Retrospective

Not started.
