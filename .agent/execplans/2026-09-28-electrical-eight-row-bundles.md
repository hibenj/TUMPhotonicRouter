# Electrical router beyond four-row bundles: lane pitch from the requirements, escape outside the pad span

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-28 -- written after the evidence; Milestone 1 running.**


## Purpose / Big Picture

The cascade benchmarks (`.agent/execplans/2026-09-28-cascade-benchmarks.md`) route optically clean at 8, 12 and 16 modes, but the electrical router loses pad wires on the 12-mode folded (1), the 16-mode folded (11) and the 16-mode diagonal (7, plus a disconnected bus branch, an overlap and a spacing error). The 16-mode layouts have eight-row heater groups. After this plan every cascade verifies clean electrically, and the two router rules that broke are stated: the lane pitch follows the requirements (wire 20 um, spacing equal to the 10 um cross-net clearance the verifier enforces, so eight lanes need 240 um), and the common bus escape rises outside the individual pad span.


## Progress

- [x] (2026-09-28) Evidence (lane): on `cascade_16_folded` the post group's row 7 collides with the lone cascade heater's straight bus drop and that heater's terminal opening 275 um right of the group's exit -- the corridor holds 6.75 lanes at the 40 um pitch, eight are needed; the rightmost group's row 8 collides with the bus escape's realized 400 um metal, whose vertical leg rises from the trimmed stripe's end inside the pad row span (the group's last pad lies right of it). No obstacle-map cell is involved in either refusal. The four-row and six-row groups have corridors of 80 to 135 lanes because nothing stands beside them.
- [~] Milestone 1, first slice (a6b97ee): the two rules landed. Pitch 30 um (spacing 10 = the enforced clearance); the trimmed stripe reaches under the bus pad (re-trimmed after the pad plan and once more inside the reconciler with the recomputed plan's pad x, since the swap can move the bus pad by one slot), the escape is a straight vertical beside its pad with one residual 10 um jog (`_choose_dogleg_target` aims at the pad column's midpoint while the widened stripe ends on the adjacent cell; left as is because it would move the bus guardrails just pinned). Suite: four cases green, guardrails re-derived (`pad_channel_height_um` 180 -> 140 on the lane cases, wire area down, min spacing 10). Cascades: failures 19 -> 7; `cascade_12_folded` 1 wire, `cascade_16_folded` 5, `cascade_16_diagonal` 1 plus a disconnected bus branch; the refusals are now "pad column inward of its corridor lane" on the small cascade-heater bundles and one "row 8 collides" on the rightmost eight-row group. Second slice pending the evidence.
- [ ] Milestone 2: records.


## Surprises & Discoveries

- 2026-09-28: "corridor width" in the Milestone 2 record of the electrical plan (about 5 lanes on `heater_s_mod`) was the width to the neighbouring instance; on the 16-mode layouts the neighbour is a lone heater whose bus drop and terminal opening sit at 275 um, and the pad wires of an eight-row group need 320 um at the old pitch.

- 2026-09-28 (second evidence pass): three separate causes remained after the pitch and stripe rules. (1) `pad_slots._interval_preferred_start` anchors a bundle's pads at the terminal's own exit cell, one lane inward of the river's first corridor lane, so every small bundle is refused as "pad column inward"; nine single-heater bundles on `cascade_16_diagonal` fail this way with empty slots on both sides. (2) The escape is drawn at the bus width, 400 um, and the pad plan puts the last individual pad 130 um from the bus pad, inside that band: the rightmost eight-row group's row 8 collides with the escape metal, not with anything in the corridor. (3) The pad-side swap discards a heater's old bus branch although another heater's greedy branch had joined it (`heater_extra_3:r` on `cascade_16_diagonal`), which the connectivity contract of 2026-09-28 reports. Rules adopted: pads start outward of the first lane and the river serves an inward pad for row 1 only (its inward channel run crosses no lane of its own bundle: every other lane is outward of c_1 and ends below h_1; rows >= 2 would cross the inner lanes); the bus pad keeps enough empty slots for the escape band plus clearance; a heater whose branch carries other branches is not swapped.


## Decision Log

- 2026-09-28 (owner): fix the electrical router for the eight-row bundles first, before the compact diagonal, the length labels and the other circuits.
- 2026-09-28 (lead): the default `individual_route_spacing_um` becomes 10 um (equal to `obstacle_clearance_um`, which is the clearance the verifier enforces between nets; the requirements document's minimum between wires is 5 um), so the track pitch is 30 um and eight lanes need 240 um. Not a per-benchmark flag: the old 20 um had no source. The suite guardrails and baseline are re-derived on the new pitch.
- 2026-09-28 (lead): the trimmed bus stripe spans from the leftmost branch to the bus pad's column, so the escape leaves the stripe directly below its pad; the trim therefore runs once more after the pad plan (and again inside the reconciler, as today), and the escape becomes a straight vertical wherever the pad column is free.


## Outcomes & Retrospective

Empty until work ends.


## Context and Orientation

`translation/electrical/river_routing.try_river_route_bundle` builds the lanes at `track_pitch_cells = ceil((wire_width_um + individual_route_spacing_um) / grid)`; `common_bus_router.trim_common_bus_to_connections` (stage 4, re-run by `pad_side_reconciliation.reconcile_pad_sides`) limits the stripe to the span of the branch connection cells; `escape_router.route_common_bus_escape` (stage 8) routes from the tree to the bus pad. Evidence script: scratchpad `evidence5/analyze.py` (session-local). Gate: `scripts/benchmark_electrical.py --suite --check --compare-baseline` on the four cases plus the six cascades through `run_electrical_benchmark`, and the full test baseline.


## Milestone 1: the two rules

1. `types.py`: `individual_route_spacing_um` default 10.0 with a comment naming the requirement and the verifier's clearance; docs regenerated.
2. The stripe under the bus pad: `trim_common_bus_to_connections` gains a keyword `keep_span_to_x_um: float | None` that widens the trimmed span to include that x (plus the half overlap); `route_electrical_heaters` calls the trim again after the pad plan with the bus pad slot's centre x (and the reconciler passes the same when it re-trims). Then `route_common_bus_escape` finds the straight path; if it still doglegs, report why.
3. Guardrails: measured on the four suite cases after the change, re-pinned by the rule of the electrical plan (routing quantities rounded up; zero-tolerance ones untouched); baseline re-pinned.
4. Tests: the trim test (a synthetic bus with a pad column beyond the last branch keeps the stripe under the pad); a river test that an eight-row bundle in a 250 um corridor is served (synthetic obstacle map, or `cascade_16_folded` through the fixture builder with the refusal reasons asserted empty for the two groups).

Acceptance: the six cascades electrically clean (`run_electrical_benchmark` success, 0 failed wires); the suite check green; test baseline OK; gate not needed (no optical change).


## Milestone 2: records

Plan Outcomes, `.agent/REPOSITORY_STATE.md`, the cascade plan's Milestone 2 numbers updated, `docs/CONFIGURATION.md` regenerated.
