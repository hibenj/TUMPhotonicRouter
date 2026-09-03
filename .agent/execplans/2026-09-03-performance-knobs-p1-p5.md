# Performance knobs from the 2026-09-03 timing pass (P1, P5)

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Both 32x32 baselines route verification-clean (multiportmmi_32x32 490 s, benes_32x32 945 s wall). The seven-benchmark timing pass in `.agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md` found no per-net anomaly in the search engine (uniform 165-270 k expansions/s); the obvious, rule-free knobs are:

- **P1** -- endpoint correction clones the whole `ObstacleMap` per candidate per job to run a trial commit as a check (`py_router.rs`, four `check_map` sites in `centerline_port_corrected_checked_native` and its bump/merge siblings). Cost: heater_s_mod 4.8 s of an 8 s run (60 %), multiportmmi_16x16 7.3 s (11 %), multiportmmi_32x32 66.6 s of 488 s (14 %), benes_32x32 14.7 s.
- **P5** -- layout + verification + GDS: 78 s of 488 s on mm32 (16 %), 41 s on benes32, 19 s on mm16; not itemised by the flow's timing print.

After this plan: the trial commit is replaced by a check-only `can_commit_route_with_clearance_and_allowed_core_overlap_cells` (same rule, one implementation), pinned by a unit test that compares its verdict with clone+commit on accept, reject (occupied core), clearance-exempt and allowed-crossing-overlap cases; the seven benchmarks are re-measured and the table updated; P5 is itemised and, if one phase dominates, addressed in a second slice. No routing rule changes; every benchmark keeps attempts/failures/repairs and verification results.

## Progress

- [ ] Milestone 1 (P1): `ObstacleMap::plan_commit_...` (check phase, returns the prepared key lists) + `can_commit_...`; `commit_...` calls `plan_` then mutates; unit test in `obstacle_map.rs`; the four `check_map` clone sites use `can_commit_`; cargo suite; ladder; seven-benchmark re-measure (endpoint correction column and totals).
- [ ] Milestone 2 (P5): itemise layout / verification / GDS-write time per benchmark (timing prints exist for the routing stage only); record; decide with the owner.

## Surprises & Discoveries

(none yet)

## Decision Log

- (2026-09-03, owner) "erst Tagesstand konsolidieren, dann mit P1 anfangen" -- consolidation done (`39ecac4`), P1 started.

## Outcomes & Retrospective

Not started.

## Validation and Acceptance

- cargo lib suite green; pytest baseline (10 failed / 357 passed); five-benchmark ladder + both 32x32 with identical attempts/failures/repairs and 0 verification errors; endpoint-correction time on mm32 drops from 66.6 s to well under 10 s.
