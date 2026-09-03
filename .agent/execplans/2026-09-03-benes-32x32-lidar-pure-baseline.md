# benes_32x32 under lidar-pure: the baseline must route

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

The repository owner's standing decision (2026-08-31, reaffirmed 2026-09-03): the 32x32 benchmarks under the stable defaults (`--crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs`) are a BASELINE MUST, not a contribution. `multiportmmi_32x32` reached that bar on 2026-09-03 (447/447, 0 failures, 0 repairs, both verifications clean, 469 s; see `.agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md`). `benes_32x32` (320 nets) is the remaining one: on 2026-08-31 it reached net 59 of 320 in 90 minutes (repair loop grinding on adjacent-diagonal contention, "pending-straight repair signature, victim=57"); on 2026-09-01 the 57/58 cliff fell but net 59 ground in the same class. Since then four crossing-kernel fixes landed (`a6dc419`, `947e76d`, `4c623ae`, predicate 2) plus the diagnostics with a per-search `seq`.

After this plan: `benes_32x32` routes completely under the stable defaults with both verification JSONs at 0 errors, on branch `crossings/verification-foundation`, and that run is recorded as the lidar-pure baseline (time, attempts/failures/repairs, GDS snapshot). No PR yet (owner: "erst mal so belassen, kein PR").

## Working rules (owner, carried over from the previous plan)

- Every finding goes into this plan BEFORE the next run. Corrections are recorded, not overwritten.
- Evidence before conclusions: confirm which attempt (per-search `seq`, partner-set size) produced a failure line before analysing it; "rejected" is only the reason if no other route exists.
- Unit test first for every kernel fix; roles run sequentially in-session; no subagents for complex tasks.
- Stop-after-route N includes routes 1..N -- for a GDS "before the failing net" use N-1.

## Progress

- [ ] Milestone 0: fresh full run of `benes_32x32` on the current build (`8e4df3f`) with `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG=1` and `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, 3 h timeout. Record: how far it gets, per-net time profile, the first net that grinds and its failure kind (search cap? open_set_exhausted? repair thrash?), the partner-set size of the failing search.
- [ ] Milestone 1: diagnosis of the first blocker by exclusion (same tooling as for mm32: failure kind, best-crossing-path, probe cells, validation sub-conditions), written here before any fix.
- [ ] Milestone 2: fix with unit test, ladder, mm32 full re-check (must stay 447/447 clean), benes_32x32 re-run.
- [ ] Milestone 3: baseline record -- benes_32x32 complete and clean; numbers and GDS snapshot recorded here and in `.agent/REPOSITORY_STATE.md`.

## Surprises & Discoveries

- (2026-09-03 12:16-12:20, Milestone 0 first evidence, run still in flight) The first failing net is NOT in the old net-57/59 class. Net 41 (index 41, stage-2, source (364,1269,0) -> target (634,1489,0), identified via the 2026-09-01 `native_route_start` log where it routed in 43.9 s after a pending-straight repair with victim=40) now fails every attempt: seq 4..37, all `kind=open_set_exhausted`, windows escalating from [350..648,1257..1501] to [330..668,1241..1517] and cycling, explored_bbox always capped at y <= 1377 and x <= 512 -- a wall ~112 rows below the target; best-crossing-path has 0 crossings (`(364,1269)->(369,1269)`), target ring: 9 cells static+opened (the stub), 10 static+unopened, gen=0 everywhere (the search never gets near). `native_repair_probe net=41 allowed_partners=0 crossing_events=0 grid_violations=0 realized_violations=0 keepout_keys=0 candidate_blockers=[]` -- repair finds nothing to rip up. The flow then moves to the next net (source (364,719,0) -> target (634,1268,0)), which exhausts its ENTIRE window (explored_bbox == window, 4-7M expansions) with blocker lines naming static cells (635..637,1269)/(634,1270) around the target -- the adjacent port row. Stage-2 nets with small vertical offset (e.g. 33: 1709->1709) routed without failure lines. Open questions for exclusion: what is at y=1377..1489, x=330..512 (static keepout vs committed route)? why 0 crossing candidates? did net 41's target opening change between 2026-09-01 and today? Evidence being gathered: `--debug-stop-after-route 40` GDS (`build/routed_benes_32x32_M0_net40.gds`), then a probe-cell run across the wall. No hypothesis adopted yet.
- (2026-09-03 12:25) CORRECTION -- the Milestone 0 run above was NOT lidar-pure. `benchmarks/benes_32x32.py` has never carried `STABLE_ROUTING_FLAGS`/`STABLE_ROUTING_ENV` (unlike benes_16x16 and multiportmmi_32x32), so a bare `routing_flow.py benes_32x32` runs the flow default `--crossing-mode window`. Evidence: no "Benchmark stable defaults applied" line in the benes stdout; the extended `collision-crossing start` trace for net 41 shows `lookup_partner_ids=[34]` while `committed_center_routes` holds all 40 nets and `net_route_entries=40` -- i.e. the caller passes single-partner sets ([34],[36],[37],[38],[39],[40], all six with identical iteration counts 268765/307525/339407/378456 -> the partner set does not even matter for what walls the search), which is the window/expected-pair strategy, not the lidar-pure full-map attempt. The 2026-08-31/09-01 probes had passed the flags explicitly (`--crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --foreign-port-keepout-cells 0 --proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3`, the benes_16x16 block). Consequence: every net-41 observation above is window-mode behaviour and irrelevant to the goal; Milestone 0 restarts with the stable flags. Fix applied: benes_32x32.py gets the benes_16x16 stable block (owner may adjust the proactive-congestion values later).
- (2026-09-03 12:23) Process-hygiene incident: the Milestone 0 process died with exit 135 (SIGBUS) at 12:23:03 because a rebuilt `_rust.abi3.so` was copied over the file while that process had it mapped. Rule: never deploy the .so while a flow process is running (check `ps aux | grep routing_flow.py` first).

## Decision Log

- (2026-09-03, owner) No PR; stay on `crossings/verification-foundation`. Goal on this branch: benes_32x32 runs too -- that is the lidar-pure baseline, and it is to be recorded as such.

## Outcomes & Retrospective

Not started.

## Context and Orientation

- Flow: `.venv/bin/python routing_flow.py benes_32x32` (stable defaults applied by `main()`); `--debug-stop-after-route N` for partial GDS.
- History: `.agent/execplans/2026-09-01-forced-90-degree-route-degradation.md` (2026-08-31/09-01 probes, artifacts `build/routed_benes_32x32_partial_net56.gds`, known tooling gap: no attempt SVGs for nets grinding inside repair).
- Crossing rules and diagnostics: the Rule audit and Retrospective in `.agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md`.

## Validation and Acceptance

- Acceptance: `benes_32x32` rc=0, `route search: ... failures=0` or at least every net routed, `build/verification/benes_32x32_{crossing,photonic}_verification.json` with `success=True`, `error_count=0`, `missing_route_count=0`.
- Regression guard for every kernel change: cargo lib suite, pytest baseline (10 failed / 357 passed), five-benchmark ladder, `multiportmmi_32x32` full (447/447 clean).

## Artifacts and Notes

- Logs in the session scratchpad: `b32_m0.stdout.log`, `b32_m0.stderr.log`, `b32_m0.status`.
