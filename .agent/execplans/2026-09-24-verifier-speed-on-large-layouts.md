# Make the photonic verifier fast on the largest layouts without changing a verdict

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-24 -- plan written; starts after Milestone 8 Slice D of the restructure plan merges.**


## Purpose / Big Picture

After routing, the flow verifies the routed layout: no two nets' waveguides overlap, no waveguide overlaps a static obstacle illegally, no route self-intersects, every route stays inside the die. On the largest paper layout (Benes 128x128 with contribution 2: 20,480 routed records, 7,680 pre-placed crossing tiles) that verification takes 71 of the run's 86 minutes, while the routing loop the paper measures takes 15. The full 27-cell reproduction of the paper therefore costs 4.5 hours, of which about 75 minutes are this one verifier call. The verifier's time is outside every paper metric, so making it fast changes no published number; what it must not change is a single verdict or a single byte of the verification reports, which the reproduction comparator and the tests read.

After this plan the same call takes minutes instead of an hour on that layout, the reports are byte-identical, and a full reproduction runs in about 3.3 hours.


## Progress

- [x] (2026-09-24 23:20) Evidence gathered (structure of the verifier, the 2026-09-07 prefilter plan, phase timings from the Milestone 8 reproduction logs); plan written.
- [ ] Milestone 1: attribution (per-step timing inside the verifier on the three large contribution 2 cells).
- [ ] Milestone 2: regions without a component per record.
- [ ] Milestone 3: candidate pairs from a spatial index, issues in the original order.
- [ ] Milestone 4: acceptance on the archived reports and the full reproduction.


## Surprises & Discoveries

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


## Milestone 2: regions without a component per record

Goal: build every record's region from its realized polygons without constructing a gdsfactory component per record, producing bit-identical regions.

Work: `_realized_record_region` today realizes a one-record list into a fresh component and reads the polygons back from the layer. Replace it by one realization of the whole record list into one component (the flow already realizes the routed layout once; if the realized polygons per record are retrievable from that pass, by record name or by a per-record polygon list that `realize_routed_net_records` can return, use that; otherwise realize once into one scratch component with a per-record layer or name tag and split the polygons by tag). The region per record must be equal as a klayout region (`region_new ^ region_old` empty) for every record of the gate cells and of Benes 64x64 contribution 2; write that equality as a test on the synthetic fixtures and as a one-off assertion script run on the large cell during this milestone. Remove the dead duplicate loop in `_polyline_self_intersects_um`.

Acceptance: the equality assertion holds on every record of the cells above; `pytest -q tests`, `scripts/test_baseline.sh` and `scripts/results/gate_short.sh` green; the verification JSON of Benes 64x64 contribution 2 byte-identical to `results_m8_check/benes_64x64_flat/contribution2/*/benes_64x64_flat_photonic_verification.json`; the step time of the region construction reported.


## Milestone 3: candidate pairs from a spatial index, issues in the original order

Goal: the pair check visits only pairs whose bounding boxes can overlap, found through a grid index instead of a quadratic Python loop, and emits issues in exactly today's order.

Work: in `_verify_cross_net_route_overlaps`, register each record's bounding box in a uniform grid (bucket size on the order of the median bounding-box extent, computed once), collect candidate pairs (i, j) with i < j in record order from shared buckets into a set, then sort the candidates by (i, j) and run today's exact test on each (the bbox overlap-or-touch test, then the region boolean, then the issue construction, all unchanged), so that the sequence of issues appended equals today's because today's loop visits (i, j) in that same order and skips non-candidates. Prove the equivalence with a test that runs both implementations on a synthetic layout with many touching and overlapping routes and compares the issue lists element by element; keep the old implementation behind a private flag only during this milestone for that test, then delete it. Do the same for `_verify_crossing_component_route_overlaps` and `_verify_crossing_component_overlaps` only if Milestone 1 shows them on the path for any paper cell (they see zero footprints under contribution 2; under contributions 0 and 1 the crossing plan path may populate them: check with the ADEPT 64x64 baseline log).

Acceptance: the equivalence test; the JSONs of Benes 64x64 contribution 2 and ADEPT 128x128 contribution 2 byte-identical to their `results_m8_check` archives; the gate; the step time of the pair check reported.


## Milestone 4: acceptance on the archived reports and the full reproduction

Goal: every cell's verification report is byte-identical to its archive, and the full run is measurably shorter.

Work: decide on the instrumentation key (drop it, or keep it and regenerate the reference reports of `results_m8_check` in a documented step; default drop, since byte identity with the archives is the cleanest evidence); run the full 27-cell reproduction into a fresh root; compare with `scripts/results/compare_date2027.py` (crossings, GDS length, error counts) and, in addition, `cmp` every `<bench>_photonic_verification.json` and `<bench>_crossing_verification.json` against `results_m8_check`; record the verification time per cell and the run's total.

Acceptance: 27/27 exact in the comparator, every verification JSON byte-identical, the Benes 128x128 contribution 2 verification under five minutes, the full run under 3.5 hours; the plan's Outcomes written; `docs/ARCHITECTURE.md` section 8 updated with the new run time and `docs/DATE2027_REPRODUCTION.md` with the new entry.
