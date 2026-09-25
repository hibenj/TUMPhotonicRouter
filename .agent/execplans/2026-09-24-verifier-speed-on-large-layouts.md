# Make the photonic verifier fast on the largest layouts without changing a verdict

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-25 05:00 -- Milestones 1 to 4 done (Benes 64x64 contribution 2 verification 526 s to 8 s, reports byte-identical on every checked cell); Milestone 5, the full 27-cell reproduction, running.**


## Purpose / Big Picture

After routing, the flow verifies the routed layout: no two nets' waveguides overlap, no waveguide overlaps a static obstacle illegally, no route self-intersects, every route stays inside the die. On the largest paper layout (Benes 128x128 with contribution 2: 20,480 routed records, 7,680 pre-placed crossing tiles) that verification takes 71 of the run's 86 minutes, while the routing loop the paper measures takes 15. The full 27-cell reproduction of the paper therefore costs 4.5 hours, of which about 75 minutes are this one verifier call. The verifier's time is outside every paper metric, so making it fast changes no published number; what it must not change is a single verdict or a single byte of the verification reports, which the reproduction comparator and the tests read.

After this plan the same call takes minutes instead of an hour on that layout, the reports are byte-identical, and a full reproduction runs in about 3.3 hours.


## Progress

- [x] (2026-09-24 23:20) Evidence gathered (structure of the verifier, the 2026-09-07 prefilter plan, phase timings from the Milestone 8 reproduction logs); plan written.
- [x] (2026-09-25 00:45) Milestone 1: attribution by cProfile (Benes 64x64 contribution 2, 5,824 records, verification 551 s under the profiler) and by an instrumented copy of the verifier in a scratch working directory (Benes 32x32 contribution 2, 1,728 records, 64 s). Both scratch runs produced reports byte-identical to the `results_m8_check` archives, so the harness is sound. Benes 64x64: `_verify_route_obstacle_overlaps` 510 s, `_realized_record_region` 26 s (5,824 calls), `_verify_cross_net_route_overlaps` 12 s, everything else under 1 s. Benes 32x32 inside the obstacle check, per obstacle layer: the union of routes 0.01 s, the union-vs-obstacle boolean 0.15 s, the residue minus the global legal region 0.00 s, then the per-route loop 9 to 14 s on each of five crossing-tile layers (residue of 576 or 2,880 polygons, every one a legal per-key overlap, 576 touching routes, zero issues). The loop does one boolean of each route against the whole residue: 1,728 routes x 5 layers on 32x32, 5,824 x 6 on 64x64, and the residue grows with the layout, so the cost is quadratic. cProfile cannot see it because klayout's operators are not attributed to a Python frame (508 s of "own time" in the loop's frame).
- [x] (2026-09-25 02:20) Milestone 2: `_PolygonBucketIndex` over the residue's polygons, cell = max(median extent, sqrt(bbox area / n)); the loop intersects each route with its candidate polygons only; four tests (two oracle comparisons against the full-residue loop, the index's conservativeness, the query-cost bound); the dead duplicate in `_polyline_self_intersects_um` removed. Reports byte-identical to the archives on Benes 64x64 contribution 2 (verification 526 s to 38 s) and ADEPT 128x128 contribution 2 (143 s to 70 s). Rust 567, Python 526; gate 9/9 exact, verified independently.
- [x] (2026-09-25 03:30) Milestone 3: the index generalized to `_BoxBucketIndex` over boxes; the pair check queries it per route and keeps the ascending (i, j) order and the bbox test, so the issue sequence is unchanged; two oracle tests against the old double loop. Reports byte-identical on Benes 64x64 contribution 2 (verification 38 s to 32 s), ADEPT 128x128 contribution 2 (70 s to 67 s) and Benes 16x16 baseline with its crossing plan (1.7 s). Rust 567, Python 528; gate 9/9 exact, verified independently.
- [x] (2026-09-25 05:00) Milestone 4: `build_realization_router` builds the backend router once per verification pass (all seven realization methods verified `&self` with no interior-mutability access); the per-record polygon computation extracted into `_record_realized_polygon`, shared by `realize_routed_net_records` and the new `realized_record_polygons` (micron polygon to `kdb.DPolygon.to_itype(dbu)`, clipped by the crossing clip region exactly as `_add_route_polygon`), so `_realized_record_region` builds no `Component`; two exactness tests (region XOR empty against the component path on the fixtures and heater_s_mod, and a 200-polygon conversion check). Reports byte-identical on Benes 64x64 contribution 2 (verification 32 s to 8.4 s), ADEPT 128x128 contribution 2 (67 s to 24 s) and Benes 16x16 baseline with 88 legal crossings (1.7 s to 1.3 s). Rust 567, Python 530; gate 9/9 exact, verified independently.
- [ ] Milestone 5: acceptance on the archived reports and the full reproduction.


## Surprises & Discoveries

- 2026-09-25: the 2026-09-24 survey (and this plan's first version) blamed the pair check's quadratic Python loop. Measured, the pair check is 12 s of 551 s on Benes 64x64; the obstacle check's per-route loop against the residue is 510 s. The 2026-09-07 pass made the residue small "while the obstacle layer is the whole chip", but under contribution 2 the residue is the whole set of legal crossing-tile overlaps (2,880 polygons on 32x32) and every route is intersected with all of it on every crossing layer. Lesson: profile before planning; a survey of the code's shape found the wrong quadratic loop.
- 2026-09-25: the first Milestone 2 spec set the grid cell from the residue polygons' median extent alone. On Benes that is the crossing-tile pitch and the check dropped from 526 s to about 60 s on 64x64 with a byte-identical report; on ADEPT 64x64 a residue of 42 slivers of 0.25 um on a whole-chip layer gave a 250 dbu cell under chip-length routes, one query walked 2.4 million cells, and the 128x128 run was killed after 38 minutes (the implementer stopped and reported instead of improvising). Fix: the cell is at least the square root of the residue's bounding-box area per polygon, so a query never covers more cells than the residue has polygons.
- 2026-09-24: `translation/photonic_verification.py::_polyline_self_intersects_um` carries an unreachable duplicate of its own loop after a `return` (lines about 928-959). Dead code; removed in Milestone 2 with a note.


## Decision Log

- 2026-09-24 (owner): do the verifier speed-up as its own plan after the restructure, with identical verdicts and byte-identical reports as the acceptance.


## Outcomes & Retrospective

Empty until work starts.


## Context and Orientation

Where the time goes (Milestone 8 reproduction logs, `results_m8_check/`): Benes 128x128 contribution 2: verification 4,265 s of a 5,161 s run (routing loop 810 s); Benes 64x64 contribution 2 (5,824 records): verification 526 s of 632 s; ADEPT 128x128 contribution 2 (about 3,700 records): verification 143 s of 622 s. In all three the crossing-verification modules are inactive (contribution 2 pre-places its crossings; the logs show zero native crossing events and no crossing report written), so the whole "Verification time" is one call, `translation/photonic_verification.py::verify_photonic_routing`, made from `routing_flow_verification.py`. That function has no internal timing.

Inside it, two costs grow with the record count N: (1) `verify_photonic_routing` builds `route_regions_by_key` with one call of `_realized_record_region` per record (lines about 164-211 and 349-372), and each call creates a fresh `gdsfactory.Component()` and realizes a one-record list into it, so 20,480 components are built for one verification; (2) `_verify_cross_net_route_overlaps` (about 502-557) runs a plain Python double loop over all record pairs, 209 million pairs at N = 20,480, and applies the 2026-09-07 bounding-box prefilter inside the loop before the klayout region boolean, so the boolean is rare but the Python iteration itself is quadratic. The 2026-09-07 plan measured and fixed at N of about 1,000 (Benes 32x32: 93 s to 2.7 s) and never re-measured at 20,000. The obstacle check (`_verify_route_obstacle_overlaps`) is linear since that plan; the crossing-component checks are quadratic in the footprint count but see zero footprints under contribution 2; the self-intersection and die checks are linear in records. Region operations use `klayout.db` (`kdb.Region` booleans, `bbox()`), the self-intersection check uses shapely on candidate segment pairs.

What must stay identical: the reports `build/verification/<bench>_photonic_verification.json` (and `_crossing_verification.json` when a crossing plan is enabled) are written by `routing_flow_verification.py` with `json.dumps(..., sort_keys=True)`, which orders dictionary keys but not list elements; the `issues` list is in the order the verifier appends them, which follows the insertion order of `route_regions_by_key` (the records' order) and, inside the pair check, the order of the double loop (i before j, both in record order). Any change to the iteration order changes the file. The reproduction comparator reads `metrics.json` (crossing count, GDS length, error counts) and the archives keep the verification JSONs, so the archived reports of every cell are the reference for byte identity.

Terms. A *record* is one routed net's realized geometry with its net name; a *region* is a klayout polygon set; a *pair check* is the boolean intersection of two nets' regions; a *spatial index* here means a uniform grid of buckets over the die keyed by cell coordinates, where each region is registered in the buckets its bounding box covers, so candidate pairs are those sharing a bucket.


## Milestone 1: attribution

Goal: know, not guess, how the 4,265 s split between the region construction, the pair loop and the rest.

Work: add timing inside `verify_photonic_routing` (a small `dict[str, float]` of step names, returned in the result's `metrics` under a new key `verification_step_times_s` and printed by the flow under the existing "Verification time" line; adding a metrics key changes the JSON, so this instrumentation is added, measured, and then either kept with the acceptance reference regenerated once, or dropped: decision in Milestone 4, default drop). Measure on Benes 64x64 contribution 2 (10 minutes, `scripts/results/run_and_archive.sh contribution2 benes_64x64_flat --preplaced-crossing-grids true --verbose-routes` into a scratch `RESULTS_ROOT`) and on ADEPT 128x128 contribution 2; extrapolate to 128x128 from the record counts, and run the 128x128 once only if the two do not agree on which step dominates.

Acceptance: a table step by step for the two cells in this plan's Progress, and the extrapolation.


## Milestone 2: the residue index

Goal: the obstacle check intersects each route only with the residue polygons whose bounding boxes overlap or touch the route's bounding box, found through a grid index, so the cost is linear in routes plus residue; the results of every boolean are the same point sets as today, so the issues are identical.

Why it is exact: `route & residue` equals `route & subset` when `subset` holds every residue polygon whose bounding box meets the route's bounding box, because a polygon whose bounding box is disjoint from the route's contributes nothing to the intersection. The issue is built from the emptiness, the area and the bounding box of that intersection minus the per-key legal region, all properties of the point set. (Both a route and a residue polygon are merged, non-overlapping polygons from klayout's boolean output.)

Work: a small index class in `translation/photonic_verification.py` (a dict from grid cell to polygon indices, cell size derived once from the residue's polygons' median bounding-box extent with a floor, registration by each polygon's bounding box; a query by a bounding box returns the polygon indices in sorted order), used only in `_verify_route_obstacle_overlaps`: `touching = route_region & candidates_region` where `candidates_region` holds the polygons the query returned, and the `continue` when the query is empty. Everything from `touching` on is unchanged. A test in `tests/test_photonic_verification.py` builds a layout on which several routes have real illegal overlaps with obstacles on two layers (and some legal per-key ones) and compares the issue list with an oracle written in the test that runs the pre-change per-route boolean against the full residue; a second test does the same on randomly placed rectangles with a fixed seed.

Acceptance: both tests; `pytest -q tests`, `scripts/test_baseline.sh` and `scripts/results/gate_short.sh` green; the report of Benes 64x64 contribution 2 byte-identical to `results_m8_check/benes_64x64_flat/contribution2/*/benes_64x64_flat_photonic_verification.json` (run in the scratch harness of Milestone 1); the obstacle check's time on that cell reported (expected: under 10 s from 510 s).


## Milestone 4: one router per pass, regions without a component per record

Goal: the per-record region construction stops paying for a router and a gdsfactory component per record, with bit-identical regions.

Evidence (Milestone 1 profile, Benes 64x64): `_realized_record_region` 26 s for 5,824 calls: 13 s of own time inside `realize_routed_net_records`, which constructs a `PyPhotonicRouter` per call (the constructor allocates an obstacle map of the whole grid and a primitive library: `src/engine/mod.rs::construct`), and 12 s in `_component_layer_region` (one `Component`, one `add_polygon`, one `get_polygons` per record).

Work, step 1 (the router): `realize_routed_net_records` accepts an optional pre-built backend router (`router=None` keyword; when given, it is used instead of constructing one; a helper `build_realization_router(realization_grid_spec, allow_45_degree_turns, bend_radius_cells)` in `translation/route_rust_realization.py` builds it the way the function does today). The verifier builds one router per `verify_photonic_routing` call and passes it to every `_realized_record_region`. Precondition to verify in the Rust bindings before relying on it: every method the realization path calls on the router (`realize_route_polygon`, `realize_centerline_polygon_with_terminal_tangents`, the meander variants, and whatever `_physical_port_centerline` calls) must leave the router's state unchanged (or its result must not depend on state a previous call could have changed); if any of them mutates state that a later call reads, step 1 is not exact and must be reported instead of done.

Work, step 2 (the component): build the record's region straight from the realized polygon(s) instead of through a scratch `Component`: the polygon the router returns is converted to a `kdb.Polygon` in database units the way `Component.add_polygon` plus `get_polygons(merge=False, by="tuple")` does, and clipped by the crossing clip region exactly as `_add_route_polygon` does. Exactness is proven, not argued: a test on the synthetic fixtures and on `heater_s_mod` compares, for every record, the new region with the old one (`(new ^ old).is_empty()`), and the scratch harness checks byte identity of the reports on the three cells of Milestone 3. If the conversion cannot be made bit-identical (rounding of the micron coordinates to database units differs between the two paths), step 2 is dropped and the reason recorded here.

Acceptance: the equality test; `pytest -q tests`, `scripts/test_baseline.sh` and `scripts/results/gate_short.sh` green; reports byte-identical on Benes 64x64 contribution 2, ADEPT 128x128 contribution 2 and Benes 16x16 baseline; the verification time on Benes 64x64 contribution 2 reported (32 s after Milestone 3, region construction 26 s of it).


## Milestone 5: acceptance on the archived reports and the full reproduction

Goal: every cell's verification report is byte-identical to its archive, and the full run is measurably shorter.

Work: no instrumentation key was added (Milestone 1 measured through cProfile and a scratch copy, so the report format is untouched); run the full 27-cell reproduction into a fresh root; compare with `scripts/results/compare_date2027.py` (crossings, GDS length, error counts) and, in addition, `cmp` every `<bench>_photonic_verification.json` and `<bench>_crossing_verification.json` against `results_m8_check`; record the verification time per cell and the run's total.

Acceptance: 27/27 exact in the comparator, every verification JSON byte-identical, the Benes 128x128 contribution 2 verification under five minutes, the full run under 3.5 hours; the plan's Outcomes written; `docs/ARCHITECTURE.md` section 8 updated with the new run time and `docs/DATE2027_REPRODUCTION.md` with the new entry.
