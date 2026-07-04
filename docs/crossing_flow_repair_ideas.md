# Crossing Flow Repair Ideas

This file tracks candidate fixes for the current crossing-routing bottlenecks before physical crossing-cell insertion.

## Current observation

The `benes_16x16` layout is geometrically clean after strict realized-crossing verification, but routes `110` and `111` are slow. The pre-route layout before these routes is available as:

- `build/routed_benes_16x16_before_routes_110_111.gds`

The hot routes are:

- route 110: `n_s5_6_o1_to_s6_5_i1` (`sw_s5_6,o4 -> sw_s6_5,o1`)
- route 111: `n_s5_7_o0_to_s6_6_i1` (`sw_s5_7,o3 -> sw_s6_6,o1`)

By eye, the next route wants a simple diagonal-ish path through two already-routed nets. The hard part is not finding a route; it is finding two legal crossing footprints where the crossed routes are too close to each other or too close to a bend/port.

## Current route-110/111 diagnostics

The Rust diagnostic can be enabled with:

```bash
PHOTONIC_ROUTER_CROSSING_SPACING_DIAG=build/crossing_spacing_net110.txt \
PHOTONIC_ROUTER_CROSSING_SPACING_DIAG_NET=110 \
PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16 \
  --debug-stop-after-route 112 \
  --foreign-port-keepout-cells 6
```

Generated examples:

- `build/crossing_spacing_net110.txt`
- `build/crossing_spacing_net111.txt`

Route 110 crosses partners `104` and `103`.

For the accepted two-crossing candidate:

- partner 104 crossing point: `(3118, 248)`
- partner 103 crossing point: `(3126, 256)`
- crossing reservation boxes do not overlap, but have only `3` grid cells of bbox gap in both x and y
- route progress gap between the two crossing centers is `8` cells
- route-side margins are only `5` cells at both crossings
- partner margins are `4` and `5` cells
- history is higher at partner 103 (`history_max=5`) than partner 104 (`history_max=3`)

Route 111 only crosses partner `104` in the current run:

- crossing point: `(3122, 155)`
- route margin: `5` cells
- partner margin: `88` cells
- no dynamic blockers in the crossing reservation

So the immediate spacing problem is concentrated in route 110, not route 111.

## What LiDAR does

Relevant files in the sibling `LiDAR` repo:

- `src/picroute/routing/drgridroute.py`
- `src/picroute/drc/drcmanager.py`
- `src/picroute/database/schematic.py`

LiDAR has a local crossing-net rip-up concept, but it is net-local rather than segment-window-local:

- `routeSingleNet()` first routes with crossing enabled.
- If the route uses crossings and the crossing solution looks suspicious, it backs up the current net.
- `ripuplocalnets(cur_net_crossing)` removes every net listed in the current route's `crossing_nets`.
- It reroutes the current net with crossing budget restored.
- Later, crossed nets are re-registered/rerouted through the normal loop.

LiDAR's DRC crossing check also validates crossing feasibility at candidate expansion time:

- The crossing is only accepted when the two waveguides are perpendicular.
- The crossed net must have enough straight length, through `crossing_check(host, slave, straight_count)`.
- It checks the crossing footprint neighborhood before returning a crossing neighbor.
- Port access regions are explicitly reserved so unrelated nets cannot consume the region in front of a port.

Important difference for this repo: LiDAR does not appear to do a true partial segment-only rip-up. It clears whole local crossing nets from the bitmap. We can adapt the idea but make it more precise.

## Congestion/history map role

The congestion/history map is relevant, but it is not sufficient by itself.

What it currently does well:

- marks crossing-window cells that were repeatedly used or made difficult,
- makes later A* searches prefer less congested crossing windows,
- supports preemptive crossing rip-up by finding partners whose valid crossing windows are heavily consumed.
- records owner-tagged future crossing corridors for routes that are expected to be crossed later.
  This gives the router enough information to identify when two existing routes should be spaced
  because a still-unrouted later net is expected to cross both of them.

What it does not encode yet:

- two adjacent crossing reservations may be legal individually but too close for robust physical insertion,
- the lower/upper partner route may be the one that should move, not necessarily the one with the highest accumulated history,
- moving a partner route may require a local route-shape change around one window, not a global detour,
- port-side straight access must be judged against crossing-cell footprint requirements.

So the congestion map is the right substrate, but crossing repair needs an additional footprint-aware decision layer on top of it.

## Candidate implementation ideas

### 0. Future shared-crossing separation invariant

The main geometric problem is two already-routed nets running too close in parallel when both
are expected to be crossed by the same later net. Each future crossing may be legal in isolation,
but the later crossing route cannot insert two physical crossing footprints if those host routes
consume the same local corridor.

The router has already tried two first-order versions of this idea:

- owner-tagged future-crossing spacing corridors with hard keepout
  (`PHOTONIC_ROUTER_ENABLE_FUTURE_CROSSING_KEEPOUT`),
- owner-tagged future-crossing spacing corridors with weighted history pressure
  (`PHOTONIC_ROUTER_FUTURE_CROSSING_SPACING_WEIGHT`).

Those mechanisms proved the information is available, but they are too blunt as defaults: hard
keepout can block otherwise valid dense routes, and history pressure alone does not reliably move
the right route far enough. The next implementation should make this a specific pairwise rule:

- when route A is committed, compare it with existing route B only if A and B share an unrouted
  future crossing consumer C,
- skip or soften the rule near shared source/target ports, splitters, and places where A and B
  must intentionally travel together before they diverge,
- soften or defer the rule when A and B also must cross each other or both are still missing their
  own required crossing sequence,
- require enough normal-direction separation for two crossing-cell footprints plus the configured
  reservation gap along the part of A/B that C is likely to cross,
- if the pair violates this invariant, first choose the route with the easier local escape window
  as a spacing victim; only then fall back to whole-net repair.

This is the targeted version of "drive apart parallel future-crossed routes", but parallel
routing itself must remain allowed. The rule is only "not parallel too close": if a later net C
must route through host routes A and B, the normal-direction distance between A and B along the
candidate crossing corridor must be large enough for two crossing footprints plus the configured
reservation/free-space gap. That required distance should be derived from the crossing cell
footprint before routing, then used as the threshold for diagnostics, pressure, and repair.

### 1. Crossing spacing repair

Current implemented first step:

- diagnostics now flag consecutive crossing reservation boxes whose bbox gaps are small and positive on both axes,
- the default minimum free bbox gap is `crossing_half_size_cells + 1`,
- with the current `crossing_half_size_cells=2`, the route-110 pair with gap `(3, 3)` is reported as a spacing conflict,
- setting `PHOTONIC_ROUTER_ENFORCE_CROSSING_SPACING=1` makes those close-spacing partners participate in crossing-compliance rejection and repair blocker selection.

The enforcement switch is intentionally off by default for now. A first global enforcement attempt exposed that the existing whole-net repair loop is not strong enough yet: it can move route 110 away from the two-close-crossing case, but then may accept a fallback crossing that is too close to a bend after endpoint correction. The next implementation step is to make repair choose a route that remains legal after endpoint correction, not merely a route with non-overlapping logical crossing reservations.

Current endpoint-correction guard:

- endpoint-corrected centerline candidates are checked against committed realized centerlines before they are accepted,
- candidates that would create an illegal realized crossing are skipped,
- after all endpoint corrections, final realized-crossing verification can rip up/reroute an offending net,
- final reroute repair adds a temporary local keepout around the illegal realized intersection and accumulates keepouts if the crossing shifts to a nearby point,
- `benes_16x16 --debug-stop-after-route 112 --foreign-port-keepout-cells 6` now passes this final verification with local endpoint-crossing reroute enabled.

This is still a whole-net local repair, not window-local splicing. It avoids emitting an illegal GDS and no longer drops the endpoint correction as the primary repair. Many endpoint corrections are still skipped earlier because the route topology is close to the crossing margin, so the next improvement is to reduce those skips by making routing reserve enough endpoint/crossing margin up front.

Detect when a new route needs to cross two existing partner routes and the two crossing reservations would be too close or overlapping.

Then:

1. Identify the two partner nets and proposed crossing centers.
2. Score which partner should move. Prefer the partner with more available slack and less port proximity.
3. Rip up that spacing-victim route.
4. Reserve the new route's crossing footprints.
5. Add local history/keepout around the old crossing corridor to push the victim away.
6. Route the new net.
7. Reroute the victim with the new crossing reservations present.

This directly addresses the observed case where the lower existing route should move away so two crossings fit cleanly.

### 2. Window-local rip-up

Instead of ripping an entire victim route, remove only the victim segment cells inside a repair window around the invalid crossing area.

Sketch:

- Build a repair rectangle around the crossing centers plus crossing footprint margin.
- Remove only victim route cells inside that rectangle from the dynamic obstacle map.
- Keep the victim's outside-window prefix/suffix fixed.
- Route a splice between the two cut points with local obstacles and crossing reservations active.
- Reassemble the victim centerline.
- Validate realized crossings on both modified nets.

This is more precise than LiDAR's whole-net local crossing rip-up, but it needs route-splicing infrastructure. It should be treated as a second step after whole-net spacing repair works.

### 3. Footprint-aware partner choice

Current preemptive rip-up mostly scores candidate partners by consumed crossing windows and history overlap. It does not explicitly answer: "which existing route is preventing enough spacing for the crossing cell?"

Add a diagnostic/score for each partner:

- nearest legal crossing-window center to the current route,
- distance to adjacent partner crossing reservation,
- margin to bends on both routes,
- margin to port keepout,
- reroute freedom around the local window.

Use that score to choose the spacing victim, not simply the net with the most history overlap.

### 4. Port-protection consistency

If a crossing must be placed near a port, the port-side straight access must be at least the crossing-cell access requirement.

The current default is:

- `SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS = 6`
- crossing geometry checks use `min_straight_cells_per_crossing` and `crossing_half_size_cells`

Before physical crossing insertion, derive these from the actual PDK crossing footprint:

- crossing body half-size,
- required straight access before/after crossing,
- port access/runway keepout.

If the second crossed net does not have enough straight before the port, increase the foreign port keepout or create crossing-footprint-based port reservations.

### 5. Diagnostics to add first

For hot routes like 110/111, emit a compact report:

- current route net id/name,
- ordered crossing partners,
- candidate crossing centers per partner,
- crossing reservation boxes,
- spacing between consecutive crossing reservations,
- route margin and partner margin at each crossing,
- distance from crossing to nearest port keepout,
- selected spacing victim and reason.

This should make visual inspection and automated repair agree.

## Proposed order

1. Add the hot-route crossing-spacing diagnostic.
2. Implement whole-net spacing-victim rip-up and reroute.
3. Tune victim selection against `benes_16x16` routes 110/111.
4. Only after that, consider window-local route splicing.
5. Derive port keepout and crossing straight margins from the physical crossing cell before insertion.

## Latest validation notes

- `benes_8x8 --foreign-port-keepout-cells 6` passes and now reports final verifier stats:
  `realized_intersections=16`, `initial_illegal=0`, `remaining_illegal=0`,
  `endpoint_reroute_repairs=0`.
- Full `benes_16x16 --foreign-port-keepout-cells 6` passes with the new summary line:
  `realized_intersections=88`, `initial_illegal=1`, `remaining_illegal=0`,
  `endpoint_reroute_repairs=5`, `failed_repairs=0`,
  `repaired_nets=111,111,111,111,111`.
- Interpretation: final correctness is currently held by endpoint-correction rejection plus
  repeated whole-net reroute of net 111 around temporary local keepouts. The next real
  algorithmic improvement should make the initial route/crossing placement reserve enough
  realized crossing margin so these endpoint correction skips and final reroutes become rare.
- Added a general crossing-search memory for the previous crossing center. Crossing A* now
  rejects a subsequent crossing when both physical reservation boxes are disjoint but too
  close in both axes to fit the crossing footprint spacing. This is not Benes-specific, but
  it does not remove the net-111 final repair because that net has only one expected crossing
  in the diagnostic.
- Added future-crossing history pressure. A route with expected future crossings contributes
  crossing-window history after commit. A later route that also has expected future crossings
  can route with a history weight and skip the simple-route shortcut so it is driven away
  from those future crossing windows. This also applies while doing crossing-aware search.
  The broad default was tested with weights `0.05` and `1.0` on route 110 and did not change
  the accepted two-crossing geometry: the reservations remained at bbox gap `(3, 3)`. The
  `1.0` run was also much slower, so `PHOTONIC_ROUTER_FUTURE_CROSSING_HISTORY_WEIGHT`
  now defaults to `0.0`.
- Added owner-tagged future-crossing spacing corridors. A later future-crossable route can
  temporarily see only corridors owned by previous routes that share a still-unrouted future
  crossing consumer. This matches the desired general rule: if routes A and B will both be
  crossed by future route C, A and B should not be allowed to consume all crossing footprint
  spacing while they run in parallel. Two variants were tested:
  - hard temporary keepout via `PHOTONIC_ROUTER_ENABLE_FUTURE_CROSSING_KEEPOUT`, which kept
    `benes_8x8` correct but made even small/deep stops too slow when applied broadly,
  - tagged history pressure via `PHOTONIC_ROUTER_FUTURE_CROSSING_SPACING_WEIGHT`, gated to
    source depth >= 4. This kept `benes_8x8` under one second after depth gating, but the
    `benes_16x16 --debug-stop-after-route 112` hotspot still ran past two minutes. Therefore
    the default spacing weight is currently `0.0`; the mechanism is kept as an opt-in
    experiment and diagnostic substrate, not as a default repair.
- Enlarged the final local repair keepout from only `crossing_half_size + 1` to
  `crossing_half_size + max(min_straight, crossing_half_size) + 1`. This reduced the
  `benes_16x16 --debug-stop-after-route 112 --foreign-port-keepout-cells 6` final repair
  count from 5 to 4 while keeping the case passing. More aggressive adaptive expansion was
  tested and rejected because it created a new unresolved illegal realized crossing.
- Implemented the first default local spacing-victim ripup. After a route commits, the native
  batch router recomputes its expected crossing events and checks whether consecutive crossing
  reservation boxes are too close. If so, it tries each crossed partner from the spacing conflict
  as a whole-net victim:
  1. restore the post-current-route state,
  2. add history for the victim's old route,
  3. rip up only that victim,
  4. reroute the victim around the already committed current route,
  5. keep the repair only if the current route's close-spacing conflict disappears and realized
     crossing validation still passes.
  This is intentionally general and constraint-driven: it does not name Benes routes or use
  benchmark coordinates. It is not yet window-local splicing; it is the first whole-net version
  of the LiDAR-like local crossing ripup idea.
- Extended the existing probe/repair victim selector so close crossing-reservation spacing now
  contributes blockers by default. Previously this was hidden behind
  `PHOTONIC_ROUTER_ENFORCE_CROSSING_SPACING`, which meant the normal repair loop often ignored
  a route that was legal logically but physically too tight for crossing-cell insertion.
- Validation note: on `benes_16x16 --debug-stop-after-route 112`, this adds extra local repair
  attempts around route 110 (`repair_failed_net` appears in the route-110 bucket), but it does
  not yet reduce the final verifier repair count. The route-110 diagnostic after this change
  reports single-partner crossings against partner 104 rather than the original two-close-crossing
  pair, while final correction still reroutes net 111 four times. So the whole-net spacing ripup
  infrastructure is in place, but the remaining blocker is now a realized single-crossing
  margin/endpoint issue, not the original two-reservation spacing conflict.
- Added an earlier realized-crossing repair hook in endpoint-correction processing. When checked
  endpoint correction rejects a candidate with an `Illegal realized crossing: net A intersects
  net B ...` error, the Python flow now records that structured candidate failure and tries a
  bounded partner reroute before the final verifier. The preferred victim is the partner route
  named by the failed endpoint-correction candidate, with a temporary keepout around the reported
  realized intersection. This targets the remaining single-crossing margin issue before waiting
  for the final all-route verifier. The hook is gated to dense crossing plans
  (`constraint_count >= 64`) and crossing source depth >= 4 so it does not spend time on
  smaller Benes cases where final realized verification is already clean.
- Validation note: with the gated early hook, `benes_8x8 --foreign-port-keepout-cells 6`
  stays clean with zero endpoint reroute repairs. On
  `benes_16x16 --debug-stop-after-route 112 --foreign-port-keepout-cells 6`, the final
  verifier now starts with `initial_illegal=0` instead of `initial_illegal=1`; the early
  partner reroutes remove the realized illegal crossing before the final all-route verifier.
- Better endpoint rule: endpoint snapping must not move the crossing-bearing body of a net.
  For routes with realized legal crossing anchors, the checked endpoint-correction commit path
  now compares the original anchor set against each candidate before committing it. A candidate
  is allowed only if it preserves the same crossing partners and crossing points within a small
  tolerance. This implements the intended invariant that snapping may alter only the source
  tail before the first crossing and the target tail after the last crossing. If the first or
  last crossing is already too close to a port, the grid route is invalid and must be rerouted
  with port/crossing keepout pressure instead of being fixed by endpoint snapping.
- Validation note: with the crossing-body guard, `benes_8x8 --foreign-port-keepout-cells 6`
  remains clean (`remaining_illegal=0`, zero endpoint reroute repairs, total about `0.69s`).
  Full `benes_16x16 --foreign-port-keepout-cells 6` also remains clean
  (`remaining_illegal=0`, zero failed repairs), but the existing endpoint-correction candidate
  generator still proposes many body-moving candidates that are now rejected. Therefore the
  safety invariant is in place, but the next efficiency step is to generate explicit terminal
  tail-only endpoint corrections instead of relying on rejection.
- Rejected main-router experiment: raising the native crossing straight margin globally to a
  footprint-derived value (`2 * crossing_half_size + 2`, currently 6 cells) kept `benes_8x8`
  legal but failed early in `benes_16x16` at route 31. Gating that stricter margin to deeper
  crossing routes avoided the early failure but made the route-112 hotspot run too slow to be
  acceptable. This means the correct main-router fix is not a broad larger margin. It needs
  to be local and footprint-aware: identify only the crossing windows near the hotspot and
  move/rip up the route that consumes the needed crossing footprint.
- Added normal summary output for native batch timing breakdowns. Future `--debug-timing` runs
  now show how much native time is spent in normal routes, probes, failed-net repair, victim
  reroutes, state reset, ripup/history, and actual A* search. This should guide the next
  cleanup of the repair loop.
- Native timing result: full `benes_16x16 --foreign-port-keepout-cells 6` shows native batch
  wall time around `181-186s`, while measured A* route search is only about `6.8-7.4s`.
  The expensive buckets are failed normal route attempts, failed repair of the current net,
  and failed victim reroutes. Therefore the main bottleneck is not A* expansion; it is the
  wrapper trying too many whole-net repair combinations that later fail crossing validation.
- Default flow change: checked endpoint correction and final realized-crossing reroute repair
  are now disabled by default. The final realized-crossing verifier remains enabled as a
  report-only diagnostic, so `initial_illegal` and `remaining_illegal` intentionally expose
  main-router geometry defects without mutating or dropping routes in the emitted layout.
  Endpoint correction can still be enabled explicitly for the PLM/post-processing tests, but
  the main engine work should now focus on making A*/native crossing placement legal before
  any grid-to-port snapping or physical crossing-cell insertion.
- Added native repair-controller diagnostics and guardrails:
  `PHOTONIC_ROUTER_NATIVE_PROGRESS=1` reports real per-route native wall time,
  `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` reports native repair modes, and
  `PHOTONIC_ROUTER_MAX_REPAIR_MODES_PER_NET` can cap per-net repair exploration. A failed-mode
  cache is now in the native batch controller, but the full `benes_16x16` run only used six
  repair modes, so this was not the primary runtime cause.
- Corrected runtime interpretation: the `route_search_total` counter only aggregates successful
  route objects. Failed crossing searches were hidden inside `normal_route_failed_wall`,
  `repair_failed_net_failed_wall`, and `reroute_victims_failed_wall`. Native progress showed
  route 110 spending about `104s`; tracing showed it repeatedly entered broad crossing-search
  fallback after candidate-window crossing search failed for partner `103` and for pair
  `[104, 103]`.
- Default crossing-search policy change: broad crossing fallback is now opt-in via
  `PHOTONIC_ROUTER_CROSSING_BROAD_FALLBACK=1`. The default crossing search is candidate-window
  first and stops there if no legal crossing-compliant candidate is found. This keeps the search
  footprint tied to known valid crossing windows instead of clearing entire partner routes and
  exploring broad failed cases.
- Validation after disabling broad fallback by default:
  `benes_8x8 --foreign-port-keepout-cells 6` remains clean
  (`initial_illegal=0`, `remaining_illegal=0`, total about `0.75s`).
  Full `benes_16x16 --foreign-port-keepout-cells 6` remains clean
  (`realized_intersections=88`, `initial_illegal=0`, `remaining_illegal=0`) and total runtime
  drops from about `180s` to about `98.7s`.
- Small main-path cleanup: crossing validation now borrows committed realized centerlines
  instead of cloning every partner centerline for each candidate check. This reduced one full
  `benes_16x16` timing run from about `190.6s` to `185.8s`, which confirms validation copying
  was not the dominant cost. The next target must reduce failed repair attempts, not just make
  each validation slightly cheaper.
- Root cause for the latest `initial_illegal=1`: the native crossing event was legal before
  endpoint correction, but endpoint correction changed the crossing-bearing body. The diagnostic
  for `net 111 x net 104` showed native A* accepted a crossing at `y=192.027um` with about
  `10um` route-side straight margin, while the final realized centerline crossed at
  `y=193.027um` with only `3um` route-side margin against a `4um` requirement. Therefore this
  specific violation was not caused by A* failing to check the current net after the crossing;
  it was post-routing endpoint correction moving/shortening the crossing segment.
- Tightened the endpoint anchor guard accordingly: it now preserves all expected crossing
  intersections, not only those already classified as legal anchors. After this change, full
  `benes_16x16 --foreign-port-keepout-cells 6` reports `initial_illegal=0` and
  `remaining_illegal=0`.
- Added native attempt-level instrumentation for the repair controller. Each native batch
  attempt now reports its own elapsed time instead of Python assigning an averaged batch time
  to every attempt, and crossing-search failures include compact diagnostics for the phase,
  partner set, candidate-window keys, and search counters when available. This should make
  the next runtime cleanup target explicit instead of inferred from broad timing buckets.
- Added a diagnostic-only topology inversion for the shared-crossing separation invariant.
  The Python crossing debug path now derives groups of the form "future crossing consumer C
  crosses host nets A/B/..." from the expected crossing events, then scans realized host
  centerlines for long close parallel overlaps. The normal summary reports
  `future_parallel_overlaps`; full `benes_16x16 --foreign-port-keepout-cells 6` currently
  reports 41 such overlaps while remaining clean (`initial_illegal=0`, `remaining_illegal=0`).
  This confirms that the topology grouping exposes the pattern we want to engineer next
  without yet changing routing decisions.
- Made the shared-crossing parallel spacing threshold physical when the PDK crossing component
  is available. Correction: this threshold must be the crossing body footprint only, not the
  footprint plus the A* straight-access margin. With the current gdsfactory crossing bbox
  (`8um x 8um`), this prints as `future_parallel_required_spacing=8um`. The straight-access
  fields remain useful as routing/search margin diagnostics, but they are not a real
  host-host spacing requirement. If the component is unavailable, diagnostics fall back to the
  grid proxy threshold of `2 * crossing_half_size_cells`.
- Added an env-gated native experiment,
  `PHOTONIC_ROUTER_PREVENT_FUTURE_PARALLEL_HOSTS=1`, that uses the same physical threshold in
  Rust. After a route is committed, the router checks whether it now runs too close and parallel
  to a previously routed future crossing host. If so, it rolls the current route back, adds local
  congestion pressure around the overlap, and retries that current route once. The original route
  is restored if the retry fails or still violates the spacing diagnostic.
- Result of that experiment under the old over-constrained `16um` threshold: it is geometry-safe
  but not the right main fix. `benes_8x8` remains
  clean but slows from the default sub-second run to about `1.49s` with the gate enabled.
  `benes_16x16` also remains clean but slows to about `100s`, reduces
  `future_parallel_overlaps` only from 41 to 39, and leaves the important 105/106 host pair too
  close for consumers 99 through 104. This says retrying the consumer/current net is too weak:
  the next repair should target the host pair itself and reroute one selected host with a strong
  local keepout around the actual long close overlap.
- Better version of the same idea: push future-crossing host spacing into the congestion map
  before the host routes are first chosen. From the topology crossing groups, each net can know
  which already-routed or soon-to-be-routed host nets it may later need to be crossed alongside.
  When routing a host net, add a high, net-specific soft cost in the exact corridor where running
  too close to the partner would leave less than the physical crossing footprint. This should push
  the first-instance host placement apart instead of relying on
  a post-commit repair. The cost must be net/pair-specific rather than a global obstacle, because
  parallel routes are allowed when no future crossing route needs to pass through them.
- Implemented the first gated version of that pre-route cost as
  `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1`. It builds a temporary, net-specific
  history-cost band around already committed host routes with width equal to the physical
  spacing threshold. Early variants showed why this must be constrained carefully:
  disabling simple routes or applying the cost in early source-depth stages makes full
  `benes_16x16` fail at route 31. The safer guarded variant applies only after the same source
  depth threshold used by future-crossing spacing logic and leaves simple routes enabled.
- Result of the guarded congestion experiment: it is clean but not useful yet. `benes_8x8`
  remains clean and fast but returns to the baseline diagnostic (`future_parallel_overlaps=8`).
  Full `benes_16x16 --foreign-port-keepout-cells 6` completes cleanly, but slows to about
  `205s`, leaves `future_parallel_overlaps=41`, and leaves the 105/106 host pair at about
  `14.142um` for consumers 99 through 104. This means a cell-only history band is either bypassed
  by simple routing or, when made strong enough to matter, disrupts earlier crossing placement.
  The next version needs rollback-aware host placement or direction/state-specific parallel cost,
  not a plain cell-history band.
- Added `benchmarks/mmi_parallel_spacing.py` as a tiny experiment for this exact mechanism. It
  routes two GCs into `mmi_a`, two host nets from `mmi_a` to `mmi_b`, two outputs to GCs, and a
  vertical probe net whose topology metadata says it crosses both host nets. This isolates the
  question "does the second MMI-to-MMI host move away from the first when congestion is applied?"
  Initial measurements:
  - default/simple routing: benchmark completes, but the host pair remains too close
    (`distance=14um`, required `16um` under the old over-constrained threshold);
  - `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1` with
    `--enable-simple-routes false`: benchmark completes and the host pair moves to about the
    required threshold (`distance=16um`, required `16um` under the old threshold).
  After adding `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION_SAFETY_CELLS` with default `2`,
  the same A*-only run reports `future_parallel_overlaps=0`, meaning the host pair is pushed
  beyond the old diagnostic `16um` threshold instead of landing exactly on it. This confirms the
  congestion signal can work in A*, and also confirms why the production path needs either
  simple-route cost awareness or a post-simple validation/retry for host spacing.
- Added the first simple-route guard for the same experiment. When
  `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1`, routes in the host-placement phase
  disable simple-route shortcuts so A* sees the future-host congestion. This covers both the
  explicit dynamic-clearance simple route path and the lower-level zero-radius simple route path.
  `mmi_parallel_spacing --foreign-port-keepout-cells 6` now succeeds with normal simple routes
  enabled and reports `future_parallel_overlaps=0`; only 1 of 7 routes uses the simple shortcut.
- Transfer check on Benes shows the current simple-route guard is too broad. With
  `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1`, `benes_8x8` remains clean and reduces
  `future_parallel_overlaps` from 8 to 6, but runtime rises to about `3.68s` because early route
  13 gets an expensive failed crossing search. Full `benes_16x16` fails at route 31 with
  "No crossing-compliant route found". This means the MMI experiment proves the mechanism, but
  production Benes needs a targeted host-spacing guard for the late/deep shared-host groups
  instead of disabling simple shortcuts for every future-host placement route.
- New user screenshot/inspection on `benes_8x8`: the pressure is visibly not enough in the
  current benchmark, because later crossing-section routing can still place the relevant routes
  too close. For now, use only `benes_8x8` for this experiment loop because it runs fast enough
  for repeated controlled tests.
- Isolation test: `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1` plus
  `PHOTONIC_ROUTER_PREEMPTIVE_CROSSING_RIPUP=0` still completes cleanly but keeps the same
  `future_parallel_overlaps=6`, including the severe late pair at `2um` spacing. This points away
  from preemptive crossing ripup as the only spacing undoer.
- Important implementation gap found: future-host pressure was applied only to routes with no
  expected crossing partners. The bad late `benes_8x8` pair includes routes that are themselves
  crossing routes, so they were exempt from this pressure. A first strict trial that both applied
  pressure to crossing routes and rejected crossing candidates with future-host spacing conflicts
  failed early at route 13. That behavior is now split behind explicit experimental switches:
  `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION_CROSSING_ROUTES=1` and
  `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_REJECT_CROSSING_CANDIDATES=1`, so the normal congestion
  gate remains runnable while crossing-route pressure can be tested separately.
- Follow-up 8x8 results:
  - default `benes_8x8 --foreign-port-keepout-cells 6` still completes fast, about `0.71s`, with
    `future_parallel_overlaps=8`;
  - normal future-host congestion remains clean but leaves `future_parallel_overlaps=6`, including
    the severe late 37/38 host pair at `2um`;
  - crossing-route pressure alone also leaves `future_parallel_overlaps=6`, so plain history cost
    is still too weak for crossing A* in this pattern;
  - adding severity-gated crossing-candidate rejection with the broader "shares any crossing
    consumer" predicate removes the late 37/38 `2um` pair and gives `future_parallel_overlaps=4`
    at about `5.2s`, while preserving `initial_illegal=0` and `remaining_illegal=0`;
  - raising the severity ratio from `0.5` to `0.6` did not improve the remaining early group,
    which suggests those remaining overlaps are not visible at the same candidate-acceptance point.
    They likely need a commit-time host-pair validation/reroute or a true pair-spacing objective.
- Added `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_COMMIT_REPAIR=1` as the next 8x8-only experiment.
  This reuses the native post-commit rollback/retry path, but switches it to the broader
  "shares any crossing consumer" predicate. It also validates crossing-spacing victim reroutes so
  that a repair cannot immediately reintroduce the same host-pair spacing violation. With the
  full 8x8 experiment gate set (`FUTURE_PARALLEL_HOST_CONGESTION=1`,
  `FUTURE_PARALLEL_HOST_CONGESTION_CROSSING_ROUTES=1`,
  `FUTURE_PARALLEL_HOST_REJECT_CROSSING_CANDIDATES=1`,
  `FUTURE_PARALLEL_HOST_COMMIT_REPAIR=1`), `benes_8x8 --foreign-port-keepout-cells 6` stays clean
  and drops to `future_parallel_overlaps=2` at about `1.43s` under the old over-constrained
  `16um` threshold. The eliminated cases were the severe `2um` pairs; the remaining
  `14.142um` pair would not be a physical spacing violation under the corrected `8um` crossing
  footprint rule. This result is still useful as evidence that route-level cell history is the
  wrong control surface for severe local pairs.
- Follow-up pair-repair experiment:
  - Added a separate `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_PARTNER_CORRIDOR_REPAIR=1` gate that
    pressures a scoped corridor around the partner host, not only the old current route. On
    `benes_8x8` this was too blunt: it regressed to `future_parallel_overlaps=4` and about `9.6s`
    because early crossing searches became hard/failing. Keep this as diagnostic only, not part
    of the useful gate.
  - The commit-repair validator now can compare realized centerlines as well as grid centerlines,
    matching the Python final diagnostic more closely.
  - Added `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_STRICT_REPAIR=1` to reject generic victim/current
    repair acceptances that reintroduce a future-host spacing miss. In strict mode the benchmark
    fails explicitly at route 15 with a spacing conflict such as `distance_cells=7.000,
    required_cells=8`; this confirms the last `14um`/`16um` miss is a real remaining blocker, not
    only a final-report artifact.
  - Added adaptive repair-set expansion: when a victim reroute is rejected for host spacing against
    a third host, enqueue that third host and the pair as alternate victims. This moves the final
    unresolved overlap to a shorter route 13/15 pair (`14um` vs `16um`), but still does not reach
    zero overlaps. The next algorithmic step should therefore be a true local pair-spacing
    displacement/splice, not more whole-net ripup or scalar history pressure.
- Correction to the interpretation: the previous future-host spacing experiments were useful as
  diagnostics, but they attacked the symptom too broadly. The real constraint is not "parallel
  hosts should generally be far apart"; it is "a future crossing net must be able to place one
  physical crossing cell at each host it crosses, with the crossing footprint fitting at the
  actual insertion sites." Two host routes may be perfectly legal waveguides and still be too
  close for two crossing cells to fit. The next implementation should therefore
  derive concrete crossing insertion windows from the expected crossing topology plus routed host
  geometry, reserve those windows as temporary hard obstacles, and only then rip up/reroute the
  local blocking host. Acceptance should be based on the physical crossing footprint fitting at
  those windows, not on generic host-host parallel spacing pressure.
- First concrete insertion-window experiment:
  - Added `PHOTONIC_ROUTER_ENFORCE_CROSSING_INSERTION_WINDOWS=1`, which compares a new crossing
    event's reservation window against already committed crossing reservation windows that share
    the same physical host route. This is closer to the real invariant than generic parallel-host
    spacing because it reasons about actual crossing insertion windows.
  - Added `PHOTONIC_ROUTER_CROSSING_INSERTION_WINDOW_HARD_KEEPOUT=1` as a stricter sub-experiment
    that expands already committed crossing windows into hard temporary keepouts during crossing
    search. On `benes_8x8`, this is too broad when applied globally: route 15 fails because the
    current whole-net repair choices cannot find a crossing-compliant alternative.
  - With only `PHOTONIC_ROUTER_ENFORCE_CROSSING_INSERTION_WINDOWS=1`, `benes_8x8` completes but
    slows to about `9.1s` and remains at `future_parallel_overlaps=8`. It successfully identifies
    insertion-window conflicts but feeds them into the existing whole-net repair machinery, which
    is the wrong repair shape.
  - Conclusion: the corrected invariant is now represented, but it must drive a local splice/local
  obstacle repair around the specific crossing insertion window. Applying the reservation as a
  broad global crossing-search keepout or whole-net reroute is too expensive and can make legal
  alternatives disappear.
- Latest correction to validation: the PDK/gdsfactory crossing cell footprint is `8um x 8um`,
  with ports at `+/-4um`. Therefore the real spacing diagnostic for two consecutive crossing
  insertions is `8um`, not `8um + 2 * straight_margin`. The previous `16um` threshold mixed the
  physical crossing body with an A* search-margin workaround, so those runs likely rejected good
  routes and should be treated as over-constrained diagnostics. After changing the default
  spacing source to `crossing_component_bbox`, default `benes_8x8 --foreign-port-keepout-cells 6`
  completes in about `0.83s`, reports `initial_illegal=0` and `remaining_illegal=0`, and keeps
  only real future-parallel host spacing misses below `8um`.
- The final realized geometry checker is now footprint-based instead of straight-margin-based.
  For every realized route-route crossing it places the physical crossing footprint at the
  intersection, aligned to the two crossing arms. A crossing is legal only when the pair is
  expected/allowed, the arms are perpendicular, both involved route segments pass straight through
  the footprint, no unrelated route segment enters the footprint interior, and no two crossing
  footprints overlap with positive area. A bend immediately outside the footprint is legal; the
  old `min_straight_cells_per_crossing` value is no longer a final-geometry validity requirement.
  Default `benes_8x8 --foreign-port-keepout-cells 6` remains clean under this checker
  (`initial_illegal=0`, `remaining_illegal=0`) and runs in about `0.74s`.
- Rechecked the old `benes_8x8 --foreign-port-keepout-cells 6` idea matrix after correcting the
  final validator and the future-host threshold to the physical `8um` footprint:
  - baseline: `0.587s`, `initial_illegal=0`, `remaining_illegal=0`,
    `future_parallel_overlaps=8`;
  - `PHOTONIC_ROUTER_PREVENT_FUTURE_PARALLEL_HOSTS=1`: `0.994s`, clean final geometry,
    `future_parallel_overlaps=8`; this still does not solve the close-host pattern;
  - `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1`: `3.565s`, clean final geometry,
    `future_parallel_overlaps=4`; useful signal, but too slow/broad alone;
  - plus `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION_CROSSING_ROUTES=1`: `5.137s`,
    clean final geometry, `future_parallel_overlaps=2`; helps, but still broad;
  - plus `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_REJECT_CROSSING_CANDIDATES=1`: `5.312s`,
    clean final geometry, `future_parallel_overlaps=0`; this idea now works under the corrected
    physical threshold;
  - plus `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_COMMIT_REPAIR=1`: `1.977s`, clean final geometry,
    `future_parallel_overlaps=0`; this is the best corrected 8x8 result among the old gates;
  - plus `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_STRICT_REPAIR=1`: `1.705s`, clean final geometry,
    `future_parallel_overlaps=0`; the strict repair gate no longer fails on the old
    over-constrained `16um` miss;
  - plus `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_PARTNER_CORRIDOR_REPAIR=1`: `5.905s`,
    clean final geometry, `future_parallel_overlaps=0`; no benefit over strict commit repair and
    much slower;
  - `PHOTONIC_ROUTER_ENFORCE_CROSSING_INSERTION_WINDOWS=1`: `6.844s`, clean final geometry,
    `future_parallel_overlaps=8`; detects the issue but still feeds it into the wrong broad
    repair shape;
  - plus `PHOTONIC_ROUTER_CROSSING_INSERTION_WINDOW_HARD_KEEPOUT=1`: fails at route 15 with no
    crossing-compliant route; still too broad as a global hard keepout;
  - `PHOTONIC_ROUTER_ENFORCE_CROSSING_SPACING=1`: `0.704s`, clean final geometry,
    `future_parallel_overlaps=8`; no visible effect on this benchmark.
  Interpretation: yes, the old validation rejected or disfavored useful ideas. With the corrected
  physical `8um` footprint rule, the future-host candidate-rejection plus commit/strict repair
  path is promising on `benes_8x8`. The next check should be whether the same gate remains stable
  and reasonably fast on `benes_16x16`; if not, make that mechanism more targeted instead of
  continuing with broad insertion-window hard keepouts.
- `benes_16x16 --foreign-port-keepout-cells 6` transfer check after the same correction:
  - corrected baseline passes with footprint validation clean (`initial_illegal=0`,
    `remaining_illegal=0`) but is currently slow in this working tree: about `95s`, with
    `future_parallel_overlaps=34`. The biggest remaining diagnostic is still the late 105/106
    host pair at about `1.414um`, seen by consumers 99 through 104.
  - the best `benes_8x8` gate set
    (`PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION=1`,
    `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_CONGESTION_CROSSING_ROUTES=1`,
    `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_REJECT_CROSSING_CANDIDATES=1`,
    `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_COMMIT_REPAIR=1`,
    `PHOTONIC_ROUTER_FUTURE_PARALLEL_HOST_STRICT_REPAIR=1`) does not transfer as a broad
    default. It fails at route 31 (`n_s0_7_o0_to_s1_3_i1`) with no crossing-compliant repair
    route found after trying blockers `[18, 20, 22, 28, 29, 30]`.
  - A batched route-31 variant matrix was aborted because captured subprocess output hid progress
    while one run became long-running. Do future 16x16 isolation with one live command at a time,
    not a buffered Python matrix.
  Interpretation: the idea is still the best corrected 8x8 mechanism, but it is too broad for
  16x16. The next implementation should narrow candidate rejection/commit repair to the specific
  severe shared-host groups or late/deep crossing sections instead of applying it to all future
  host/crossing-route situations.

Current conclusion:

- The user's spacing model is right: routes that are known future crossing targets need to
  reserve spacing before the crossing route arrives.
- A* memory alone is not enough, even when the cost is large.
- The first native prevention gate confirms that current-net retry is not enough.
- The corrected 8x8 matrix says the most promising old path is future-host congestion for crossing
  routes plus crossing-candidate rejection plus commit/strict repair. It reaches zero
  future-host spacing diagnostics while keeping final footprint validation clean.
- The 16x16 transfer check says that promising path must be targeted; broad application fails
  early at route 31.
- Rechecked the LiDAR implementation for the crossing baseline. Its core crossing strategy is
  closer to normal A* plus a legal-neighbor crossing exception than to our current
  topology-window crossing search. In LiDAR, `AstarSearch.findNeighbors()` asks
  `DrcManager.bViolateDRC()` whether a neighbor collides. If the neighbor is free, it is accepted.
  If it collides with a waveguide and crossing is enabled, the DRC check validates orientation,
  crossing budget, crossing compatibility, available crossing footprint around the hit, and
  straight continuation after the crossing. A valid collision becomes a `"crossing_0"` or
  `"crossing_45"` neighbor. The A* cost then adds `loss_crossing` for that crossing neighbor,
  while normal moves use propagation and bend costs. Separately, final evaluation computes
  insertion loss as `wirelength * propagation_loss + bending * bending_loss + crossing_num *
  crossing_loss`.
- New baseline idea to implement here: a LiDAR-style collision-driven crossing mode in our Rust
  A*. Keep the existing normal A* machinery and dynamic obstacle map. When a primitive footprint
  hits another net, validate it as a possible crossing instead of switching to the current
  specialized crossing-window search. The acceptance check should use the same physical rule as
  final validation: expected/allowed pair, straight primitive, perpendicular owner segment,
  `8um` crossing footprint fits at the actual hit/intersection, no unrelated route geometry in
  the footprint, no overlapping crossing footprint, and a crossing budget/count update. Add
  `crossing_loss` to the step cost and require the net's expected crossings to be satisfied by
  the time it reaches the target. This should become the baseline to compare against current
  topology-window crossing search.
- Plain cell history is still too coarse as a final design, but the corrected result says it is a
  useful control signal when combined with candidate rejection and commit validation.
- Hard or broad tagged pressure is too expensive as a default.
- Endpoint correction should be a constrained terminal-tail operation, not a whole-route
  geometry rewrite. The guard now enforces this, but the generator should be changed to propose
  only terminal-tail edits for crossing-bearing routes.
- Broadly increasing crossing margin is too blunt for dense early stages.
- The main runtime fix is to avoid unproductive whole-net repair combinations. The repair loop
  needs better candidate pruning and/or window-local victim reroute around the actual crossing
  footprint conflict.
- The next refinement, if whole-net victim ripup is still too weak or too slow, should be
  true window-local splicing: cut the victim around the conflict window and reroute only that
  segment while keeping the outside prefix/suffix fixed.
