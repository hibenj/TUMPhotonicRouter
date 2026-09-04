# multiportmmi_32x32 first layer: why do we cross where LiDAR does not?

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

Owner (2026-09-04): compare the LiDAR reference layout (`~/Documents/Repositories/working/LiDAR/build/gds/picroute_run.oas`, same coordinate frame as ours) with our routed multiportmmi_32x32 (baseline `34c59a0`, `build/routed_multiportmmi_32x32_baseline.gds`) for everything up to and including the routes into the first heater layer: `gc1 -> fanout_yb_0..4 -> mol_array_0_mzi (32) -> mmi0_ps_array_0_heater (32)`, nets n_0..n_94, x < ~2200 um. LiDAR needs no crossings there (easy geometry); we place crossings. Find out why -- parallel-spacing halos, congestion costs, net ordering, stub lanes, search cost model -- and make our router not need crossings for this geometry either.

## Progress

- [x] Localize (2026-09-04 10:20): baseline run 447/447, 149 crossings; in the region x < 2200 um only FOUR, all in the fanout->MZI fan, and they come in PAIRS on the same two nets: n_33 x n_34 at (1013.5, 2443.1) and (1335.5, 2027.1); n_58 x n_59 at (907.5, 3837.1) and (1323.5, 4337.1). A pair of crossings on the same two nets = the routes swap sides and swap back: never necessary.
- [x] Mapping check: the fan is monotonic (yb_4_k:o2 upper output -> mzi_{2k+1}, o3 lower -> mzi_{2k}); a crossing-free fan exists (LiDAR).
- [x] Route geometry (stop-62 with `PHOTONIC_ROUTER_TRACE_CROSSING=1`): n_33 (rust id 34, (451,493)->(689,143)): east, one long "\\\\" diagonal to (677,279), vertical down x=677 to 143, port. n_34 (id 35, (451,494) -- ONE row above n_33's source -- -> (689,193), 50 rows above n_33's target): east to (472,494), "\\\\" diagonal to (521,445), a HAIRPIN back SW to (511,435) crossing n_33's diagonal at cell (516,440), vertical down to (511,399), its own "\\\\" diagonal BELOW n_33's to (672,238), east across n_33's vertical at x=677 (second crossing), down x=683 to 193, port. n_34 deliberately goes under n_33 and comes back.
- [ ] Experiment X1 (in flight): route the fan layer with a prohibitive crossing search loss (`PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM=100000`): does n_34 find the crossing-free route above n_33 (parallel diagonal +2 rows, then down at x~683)? If yes -> the cost model prefers two crossings (2 x 200 um search loss + a hairpin) over that detour; if no -> something blocks the upper path (halo at n_33's diagonal->vertical corner, target stub).
- [ ] Then, per exclusion: net ordering (route the upper output first?), LSC/proactive congestion halos, stub lanes, search loss.

## Surprises & Discoveries

(none yet)

## Decision Log

- (2026-09-04, owner) Investigate the first layer against LiDAR; goal: no crossings for this easy geometry.

## Outcomes & Retrospective

Not started.
