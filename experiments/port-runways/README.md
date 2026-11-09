# Port runways

Status: not adopted (owner decision 2026-11-09). Base commit of the patches:
`89fe1b1` (the three patches are the commits `b05bffd`, `b45479b`,
`d3a62fe` without their `CHANGELOG.md` hunks); the code left the main tree
with the commit that added this folder.

## Idea

A port is not a grid point. `port_to_grid_state` floor-snaps the port centre
plus one cell outward into the 2 um cell, so the search endpoint sits up to
1 um beside the port axis and the stage closes the gap afterwards: the
endpoint correction (`engine/endpoint_correction.rs`,
`verify/route_rust_endpoint_correction.py`, most of `routing/finalize.py`,
the two pre-search snaps in `handoff.py`), about 3,000 lines that bend the
realized centerline after the search. The dense fan-out groups already do
it the other way round (`fanout_access_mode = static-stubs`): a static stub
from the port centre to a lattice-exact anchor, committed before the search,
the search starting at the anchor.

The experiment gives every cardinal port such a stub, a *runway*: a 0.5 um
straight lead-in along the port heading, two shallow arcs of the bend radius
that shift the centerline by the port's sub-cell lateral residue
(theta = acos(1 - |d| / 2R), 3 to 17 degrees at R = 10 um), and a straight
run to the first cell centre at or beyond the arcs, at least one cell
outward. The anchor is that cell centre with the port's heading; the search
starts and ends at the anchors exactly as it does for the fan-out stubs; the
stub is committed static; the realized route is stub plus search path, so
the correction has nothing left to move. With the runways as default, the
correction machinery could go (the planned slice 5B).

## What was built

Three patches, all behind `PHOTONIC_ROUTER_PORT_RUNWAYS=1`
(`FanoutAccessConfig.port_runways`), switch off exact on the nine-cell gate:

1. `0001-port-runways-behind-a-switch.patch`: `route_jobs._port_runway_anchor`
   and `_build_port_runway_anchors`; anchors merged into
   `fanout_anchor_by_port_spec` after the fan-out builders (fan-out stubs
   win); a net whose two ports are closer than their two runways (the
   pre-placed crossing grids' tile links) keeps the old path on both ports;
   the contribution-2 planner's anchor probe (`static_fanout_anchors_um`)
   reports fan-out anchors only (with the runways in it the ADEPT 8x8
   planner lost a column grid and fell back to the guided router);
   `finalize._append_target_fanout_stubs_after_correction` appends the
   reversed stub of any shape; seven unit tests.
2. `0002-runway-anchors-are-not-dense-fan-out-stubs.patch`: the first
   27-cell table failed on six cells because the runway anchors had entered
   every consumer of `fanout_anchor_by_port_spec`, including the five that
   carry dense fan-out *semantics* (port footprint shrunk to the stub lane
   size 0, dense clusters, long-straight exemption, dense target
   reservations, footprint centre). `SessionState.fanout_stub_anchor` hides
   runway anchors from those; the geometry consumers (search endpoints, stub
   prepend, static cells, endpoint correction) still take every anchor.
3. `0003-dense-target-stubs-lattice-exact-under-the-switch.patch`: ADEPT
   128x128 contribution 2 failed verification on two dead-straight nets
   (heater to multiport MMI, 0.875 um residue on both ports): the dense
   *target* stub was straight at the port's true cross-axis coordinate and
   left its sub-cell shift to the checked target correction, which had no
   free axis-aligned segment once the source side carried a runway. Under the
   switch the dense target stubs take the runway geometry too
   (`_port_runway_anchor(min_forward_cells=<staggered length>)`). A runway on
   one side needs lattice-exact anchors on the other side too.

## Measurement

Before any code, `scripts/measure_runways.py` ran the flow in-process on the
nine gate cells and read every realized centerline (2026-11-06, head
`89fe1b1`):

- No port lies on a cell-centre axis against the real grid origin: Benes
  has three residue classes (0.027, 0.348, 0.402 um), ADEPT five (0.25 to
  0.875 um).
- The correction today realizes the first run along the port axis, off the
  lattice by the residue, and absorbs it later (inside a later straight or
  at the first bend). The length it adds is almost entirely the
  port-to-grid gap stub: Benes 1.9 to 3.2 um per net (0.8 to 1.6 % of the
  cell), ADEPT 3.9 to 4.3 um on nets without a fan-out stub.
- A minimal runway S at R = 10 um needs 1.0 to 6.2 um forward extent and
  collides with the route's first real turn on 0 % of the Benes and 2 to 4 %
  of the ADEPT ports. Reading at the time: runways are length-neutral.
- The visible downside: a port-to-port straight (Benes 5 to 10 % of the
  nets, ADEPT 15 %) gets two shallow S, four arcs, where it is one line today.

Three 27-cell tables with the switch on (cda-server-4, 2026-11-06,
`tables/b05bffd.json`, `tables/b45479b.json`, `tables/d3a62fe.json`;
rendering against the pins in `tables/against_pins.md`):

| table | routes | crossings against the pins | lengths against the pins |
| --- | --- | --- | --- |
| `b05bffd` (patch 1) | 21 of 27: Benes 16x16 lidar-pure, 32x32 lidar-pure and contribution 1 fail on crossing legality, ADEPT 64x64 lidar-pure and contribution 2, 128x128 contribution 2 do not converge | ADEPT 32x32 lidar-pure 121 to 123 | Benes +0.5 to +0.6 %, ADEPT 8x8 lidar-pure -0.24 % (the collapsed footprints gave the lidar-pure cells a shortcut) |
| `b45479b` (patches 1 + 2) | 26 of 27: ADEPT 128x128 contribution 2 fails verification on two nets | Benes 32x32 lidar-pure 416 to 418 | Benes lidar-pure +1.0 to +1.1 %, Benes others +0.4 to +0.6 %, ADEPT -0.04 to +0.69 % |
| `d3a62fe` (patches 1 to 3) | 27 of 27, 0 verifier errors | Benes 32x32 lidar-pure 416 to 418, 26 cells equal | Benes lidar-pure +0.98 to +1.12 %, Benes contribution 1 / 2 +0.44 to +0.59 %, Benes 64x64 / 128x128 contribution 2 +0.28 / +0.16 %, ADEPT -0.02 to +0.61 % (8x8 pays most, 64x64 and 128x128 +0.07 to +0.12 %) |

Per cell for the final table: `tables/d3a62fe_against_pins.md`. Negotiation
rounds with the runways: ADEPT 64x64 6 / 4 / 5 against the pinned 3 / 3 / 1,
128x128 contribution 2 9 against 8; the wall ratios against the paper's loop
times stay below 1 on every cell. Arcs per net (turn runs of the realized
centerline, nine cells): Benes 8x8 2.2 to 6.6, ADEPT 8x8 2.9 to 5.1.

## Result

The runways do what the design said: lattice entry before the search, the
correction has nothing to move, crossings untouched on 26 of 27 cells,
27 of 27 cells verify. They are not length-neutral: the price is +0.5 to
+1.1 % GDS length on Benes, +0.1 to +0.6 % on ADEPT, +2 crossings on one
cell, more negotiation rounds on the dense 64x64 cells, and four shallow
arcs per net (optically negligible at R = 10 um, visible in the GDS). The
gain would have been the deletion of the correction machinery.

## Decision

Not adopted (2026-11-09): the paper numbers would carry a measurable
percent of length and more bends for code simplicity. The correction
machinery stays. A middle way, runways only where the correction inserts a
gap stub today, is a new experiment, not a default candidate.

## Reproduce

```bash
git checkout 89fe1b1
git am experiments/port-runways/patches/*.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_PORT_RUNWAYS=1 RESULTS_ROOT=$PWD/results_runways scripts/results/reproduce_paper.sh all
.venv/bin/python scripts/results/compare_paper.py results_runways docs/paper_table_sources.json
```

The measurement scripts run from the repository root with the venv active
and the flow's own arguments after the benchmark name:
`scripts/measure_runways.py <bench> <out.json> [flow flags]` (the slice 5-0
numbers above), `scripts/diag_anchor.py <bench> <net> [flow flags]` (anchors,
stub cells, foreign static cells and siblings of one net, then stops before
dispatch), `scripts/dump_nets.py <bench> <out.json> <net,net> [flow flags]`
(route cells and corrected centerlines of named nets before the verification
raise; the ADEPT 128x128 diagnosis). `diag_anchor.py` reads the state fields
the patches add and needs them applied.
