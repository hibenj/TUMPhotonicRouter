# Crossing hook on the kernel's grid

Status: not adopted (2026-11-06). Base commit of the patch: `418558f`
(the patch is the commit `fd70232`, reverted in the main tree by `89fe1b1`).

## Idea

The live crossing hook (`search/astar/crossing_rules.rs`,
`CrossingHookContext::build`) builds its own dense grid from the obstacle
map: physical occupancy only, no port-open cells, dynamic clearance
expansion radius 0, no clearance-exempt cells. The kernel's search grid gets
all three. Speed pass A8 found the difference when a halo dedupe that read
the kernel grid moved three gate cells: the hook's extra-halo test is a
different predicate from the kernel's (port stubs count as blocked there,
the extra dynamic clearance does not), and the two checks only look like
duplicates. The experiment lets the hook grid take the kernel's inputs
(anchor open cells, dynamic radius, exempt cells), behind
`PHOTONIC_ROUTER_CROSSING_HOOK_KERNEL_GRID=1` (read once per process,
experiment-only, not in the config overlay).

## Measurement

Switch off: nine-cell gate identical to the previous commit in every metric.
Switch on, nine cells: crossings and errors unchanged, three lengths shorter
(Benes 8x8 lidar-pure 31418.224 to 31409.94 um, ADEPT 8x8 lidar-pure
25171.624 to 24921.086, contribution 1 25090.058 to 24969.384).

27-cell table with the switch on (cda-server-4, 2026-11-06 01:04 to 02:46,
`tables/switch_on.json`, rendering against the pins in
`tables/against_pins.md`): eleven cells shorter by up to one percent
(ADEPT 16x16 lidar-pure -0.86 %, ADEPT 8x8 lidar-pure -1.0 %), fourteen
identical, and two cells fail: Benes 32x32 contribution 1 with a
self-intersecting route (1 verifier error, 430 crossings) and ADEPT 64x64
lidar-pure loses net 575 (no legal LiDAR crossing route through ten repair
rounds). Benes 32x32 lidar-pure gains six crossings (416 to 422).

## Result and decision

Port-open cells, the dynamic clearance expansion and the exempt cells are
search freedoms of the kernel's grid, not free silicon: a hook that treats
them as free places crossing candidates over port-open cells. The hook keeps
its physical-occupancy grid; the comment at `CrossingHookContext::build`
points here. The shorter lengths on eleven cells are real and say the hook's
grid is conservative; a hook that reads a third grid (kernel freedoms minus
the port-open cells) would be the next experiment.

## Follow-up: the radius alone (2026-11-09, not adopted)

The proposed split of the three inputs: a switch that gives the hook's grid
only the kernel's dynamic clearance expansion radius (`radius`), or the
radius with its clearance-exempt cells (`radius-exempt`), never the
port-open cells (`patches/0002-hook-grid-radius-only-switch.patch`, against
`1377ad6`, env `PHOTONIC_ROUTER_CROSSING_HOOK_GRID`). The paper cells run
with a block radius of one cell, so the switch changes the hook's grid.
Nine-cell gate with either mode: every cell identical to the pins in
crossings, length and errors (`tables/gate_radius_variants.md`), where the
full kernel grid had moved three of the nine. So the shorter lengths of the
kernel-grid table came from the port-open cells, the same input that made
two cells fail; the dynamic expansion contributes nothing on these cells.
Closed without a 27-cell table (the rule set at launch: the table only if
the nine-cell lengths move); the switch left the tree with the record.

## Reproduce

```bash
# the kernel-grid table
git checkout 418558f
git am experiments/crossing-hook-kernel-grid/patches/0001-crossing-hook-on-the-kernel-grid.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_CROSSING_HOOK_KERNEL_GRID=1 RESULTS_ROOT=$PWD/results_hook scripts/results/reproduce_paper.sh all
.venv/bin/python scripts/results/compare_paper.py results_hook docs/paper_table_sources.json
# the radius-only follow-up (a plain diff, not a mail patch)
git checkout 1377ad6
git apply experiments/crossing-hook-kernel-grid/patches/0002-hook-grid-radius-only-switch.patch
.venv/bin/maturin develop --release
PHOTONIC_ROUTER_CROSSING_HOOK_GRID=radius scripts/results/gate_short.sh
```
