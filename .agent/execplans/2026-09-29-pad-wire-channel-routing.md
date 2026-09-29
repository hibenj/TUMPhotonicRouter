# Pad wires as channel routing: one track assignment over all bundles

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-29 -- written; Milestone 1 running.**


## Purpose / Big Picture

The pad wires are routed bundle by bundle (`translation/electrical/river_routing.py`): row i of a bundle takes corridor lane c_i on the exit side and channel height h_i by its row index, nested so that the wires of one bundle never cross. Two bundles whose channel runs overlap in x get the same heights and collide; on `cascade_16_folded` the lone heater of column A runs under column B's exit to its pad and blocks column B's wires, although three distinct heights would carry all three (the owner's observation on the render, 2026-09-29). The general rule is channel routing: every pad wire is one horizontal run between its corridor lane and its pad column; the runs of all bundles are assigned tracks jointly under the vertical constraints (at its corridor column a wire lies below every run crossing that column, at its pad column above), with the constrained left-edge algorithm. The per-bundle nesting and the inward nesting become consequences of the constraints, not rules. After this plan `river_routing.py` is that channel router, the six cascades verify clean, and the pad-choice rules are the next item (owner: pads are not yet chosen to minimize the routing; that comes after).


## Progress

- [x] (2026-09-29) Design from the owner's observation and the measured collision (bundle 2's run at h_1 from x 1854 to 2334 against bundle 3's runs at the same heights from 2260).
- [ ] Milestone 1: the channel router replaces the per-bundle construction; six cascades clean; suite re-pinned.
- [ ] Milestone 2: records.


## Surprises & Discoveries

(none yet)


## Decision Log

- 2026-09-29 (owner): fix the routing generally, not with special cases; the pad-choice rules are the next item after that.
- 2026-09-29 (lead): the formulation. Wire W: corridor lane x c_W (as today, per bundle: exit column plus i times the track pitch outward), pad column p_W, span [min(c_W, p_W), max(c_W, p_W)], path port -> (c_W, row) -> (c_W, t_W) -> (p_W, t_W) -> pad. Vertical constraints: for every other wire R whose span contains c_W, R lies above W; for every R whose span contains p_W, W lies above R (a straight L, p_W = c_W, therefore tolerates no crossing run at all; if one exists the constraints form a cycle and the wire goes to the per-wire fallback, which is where the pad choice of the next item comes in). Tracks from the bottom (farthest from the pads) with the constrained left-edge algorithm: wires sorted by left edge; a wire joins the current track when it clears the track's last wire by one pitch in x and every wire that must lie below it is already placed; a round without progress means a cycle: those wires fall back. Track t at pad access edge minus (T + 1 - t) pitches, T the number of tracks used, which must fit above the highest terminal row plus clearance. Every placed wire is then checked against obstacles, other terminals and other pads as today and committed; the fallback search runs last for the rest.


## Outcomes & Retrospective

Empty until work ends.


## Context and Orientation

`bundle_detail_router.route_detailed_bundles` (stage 9) loops over bundles: `try_river_route_bundle` builds a bundle's wires or refuses it, then the per-wire `pad_wire_search.route_full_grid_pad_wire` fallback. After this plan the stage collects every bundle's members first, assigns corridor lanes per bundle, assigns tracks globally, commits, and falls back per wire. Contracts (`stages.PadWireRouter`, `DetailedBundleRoutingResult`, the per-wire table, `bundle_fallback_reasons`) stay; `PadAssignment.nesting` becomes unnecessary (delete with its detection in `pad_slots` and its tests, or keep only as a reported fact). Gate: `scripts/benchmark_electrical.py --suite --check --compare-baseline` (drift expected, re-pin by the rule), the six cascades through `tests/fixtures/sessions.py::route_electrical_benchmark_result`, the test baseline.


## Milestone 1: the channel router

`river_routing.py` -> `channel_routing.py` (or keep the file name and rewrite; say which): `assign_corridor_lanes(bundle, ...)`, `assign_tracks(wires, pitch_cells) -> (placed, unplaced)` (pure, unit-tested on small synthetic wire sets: nesting of one bundle, the three-wire case of the folded 16-mode, a cycle), `build_wire_path(wire, track_y)`, and the dispatcher changes. Tests: the synthetic cases; `cascade_16_folded` clean with the two column-B wires and the lone heater on three tracks; every bundle of `heater_s_mod` still detour 0 (its nesting emerges); the suite cases.

Acceptance: six cascades electrically clean; suite green; test baseline OK; `docs/ARCHITECTURE.md`'s electrical subsection updated for the module (Milestone 2).


## Milestone 2: records

Plan Outcomes, `docs/ARCHITECTURE.md`, `.agent/REPOSITORY_STATE.md`, the cascade plan's numbers.
