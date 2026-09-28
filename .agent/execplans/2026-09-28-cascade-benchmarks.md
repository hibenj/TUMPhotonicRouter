# Cascade benchmarks: 8, 12 and 16 modes, folded and diagonal, from one builder

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

Status: **2026-09-28 -- written; Milestone 1 (the builder) running.**


## Purpose / Big Picture

`benchmarks/heater_s_mod.py` is the 8-mode cascade of the owner's confidential quantum-photonic circuits: state preparation (N/2 units, each MMI, phase shifter, MMI, phase shifter), a chain of N-3 MZIs each on the next mode pair (MMI, phase shifter, MMI), and tomography (N/2 units, each phase shifter, MMI, phase shifter, MMI): 3N-3 heaters, 21 for N = 8, which is the pad count of the handcrafted reference layout. The 12- and 16-mode cascades exist only as diagrams (in the owner's project dump; confidential, read in place, never copied). After this plan the repository has one parametric builder and six benchmarks, `cascade_{8,12,16}_{folded,diagonal}`, routed with the `heater_s_mod` configuration (90-degree routing, 10 um bends, path-length matching of gates and outputs, heater obstacles, electrical routing, 3 um clearance, no crossings), with their routing, length-matching and electrical results recorded. The other five circuits of the dump (flat GDS from three tape-outs, no netlists) and a per-waveguide length label in the SVGs are later items, not this plan.


## Progress

- [x] (2026-09-28) Analysis: the three blocks and their counts read off the 12- and 16-mode diagrams; `heater_s_mod` mapped instance by instance (lanes `mmi_a/heater/mmi_b/heater_post`, cascade `mmi_extra_{2j}/heater_extra_j/mmi_extra_{2j+1}` on modes (j+1, j+2), tomography `heater_extra_{C+k}/mmi_extra_{2C+k}/heater_extra_{C+L+k}/mmi_extra_{2C+L+k}` with C = N-3, L = N/2); the fold identified (column A holds MZI 0, every later column two MZIs stacked, the lower one fed by a loop-back route); the handcrafted layout's MZI length (560 um) and pad row (110 x 610 um pads, 150 um pitch, above) confirmed as the benchmark's source.
- [ ] Milestone 1: `benchmarks/cascade.py` and the six modules; the folded 8-mode output equal to `heater_s_mod` up to net order.
- [ ] Milestone 2: route the six with the stable configuration; results table; renders.
- [ ] Milestone 3: records and tests.


## Surprises & Discoveries

- 2026-09-28: `NODE_TYPES` and `INTERNAL_DELAYS_UM` in the heater benchmarks are read by nothing in `python/` or `translation/` (the flow reads `build_schematic` and `STABLE_ROUTING_FLAGS` only); kept in the generated modules for the moment, marked as such.


## Decision Log

- 2026-09-28 (owner): two versions of every size, folded (the handcrafted fold) and pure diagonal; the diagonal keeps the folded layout's 1000 um column spacing (a compact diagonal may follow); the other circuits of the dump come after the cascades; per-waveguide length labels in the route SVGs are a separate later item.
- 2026-09-28 (lead): `heater_s_mod.py` stays untouched as the hand-written original (the electrical suite, its baseline and the tests keep their case); `cascade_8_folded` is the generated twin, proven equal to it in instances, placements and net endpoint pairs (net names and order may differ). Module names `cascade_<N>_<layout>` because a Python module name cannot start with a digit.
- 2026-09-28 (lead): the heater of cascade MZI j sits on the upper arm for even j and on the lower arm for odd j, as in `heater_s_mod` (in a stacked column the two heaters face each other between the two MZIs); the diagonal keeps the same rule so the two layouts differ only in x.


## Outcomes & Retrospective

Empty until work ends.


## Context and Orientation

Geometry of `heater_s_mod` (all um): input couplers at x = 0, pitch 127, lane k = couplers 2k and 2k+1, lane centre = their mean; per lane `mmi_a` x 160, `heater` x 300 on the upper arm (centre + 40), `mmi_b` x 720, `heater_post` x 820; MMI port offsets +-0.625 (o1/o2 inputs lower/upper, o3/o4 outputs upper/lower). Cascade columns: column 0 at x 1340 (MZI 0 alone), column c >= 1 at x 2200 + (c-1) * 1000; inside a column the first MMI is at the column x, the heater at +140, the second MMI at +560. Folded: MZI j lives in column ceil(j/2); diagonal: MZI j in column j. MZI centre y: j even -> lane_centre[j/2 + 1] + 40 + 0.625 (its lower input straight from `heater_post_{j/2+1}`); j odd -> previous MZI's centre - 1.25 (its upper input straight from the previous MZI's lower output). Heater y: centre + 40 (even j) or centre - 40 (odd j). Tomography, with X = the last column's second-MMI x: pre-heater at X + 740 (lane centre + 40), first MMI X + 1160, heater X + 1300 (centre + 40), second MMI X + 1720, output couplers X + 2140 at the coupler rows. Nets: mode m (0 = top) ends at tomography lane m // 2, upper input via the pre-heater if m is even, lower input if odd; mode 0 comes from `heater_post_0`, mode N-1 from `mmi_b_{L-1}.o4`, mode m in 1..N-3 from MZI (m-1)'s upper output, mode N-2 from MZI (N-4)'s lower output; MZI j's upper input is `mmi_b_0.o4` for j = 0 and MZI (j-1)'s lower output otherwise, its lower input `heater_post_{(j+2)/2}.o2` for even j+2 and `mmi_b_{(j+1)/2}.o4` for odd.

Configuration: `heater_s_mod.STABLE_ROUTING_FLAGS`, applied by `python -m photonic_router route <benchmark>` by default. Sizes: folded 8-mode 5.9 mm long; 12-mode about 8 mm folded and 12 mm diagonal; 16-mode about 10 mm folded and 16 mm diagonal, against the AMF die of 12 x 3.25 mm in the requirements document (benchmarks need not fit a die; recorded for the owner).


## Milestone 1: the builder and the six modules

`benchmarks/cascade.py`: `build_cascade_schematic(n_modes: int, layout: Literal["folded", "diagonal"]) -> Schematic` with the geometry and nets above, `n_modes` even and >= 4; helpers `node_types(n_modes)` and `internal_delays_um(n_modes)`. Six modules `benchmarks/cascade_{8,12,16}_{folded,diagonal}.py` after the `benes_NxN.py` pattern: `STABLE_ROUTING_FLAGS` copied from `heater_s_mod`, `NODE_TYPES`, `INTERNAL_DELAYS_UM`, `build_schematic()`. Test `tests/test_cascade_benchmarks.py`: the folded 8-mode schematic has the same instances (name, component, placement x, y, rotation, mirror) as `heater_s_mod` and the same set of net endpoint pairs; for every module the counts (3N-3 heaters, N-3 cascade MZIs, 2N couplers, 2 * (N/2) + 2 * (N-3) + 2 * (N/2) MMIs, nets); every port referenced by a net exists on its instance's component.

Acceptance: the test green; `python -m photonic_router route cascade_8_folded` completes with the same verification verdict and the same routed GDS length as `heater_s_mod`.


## Milestone 2: routing runs

Route all six with the stable configuration and no debug SVGs (owner, 2026-09-28: the SVGs are for visual debugging only, not to be produced by default); record per benchmark: nets, verifier errors, path-length matching outcome, routing time, electrical heaters / pads / verification, and the layout size. A render is produced only when the owner asks to look at a specific benchmark.


## Milestone 3: records and tests

`docs/ARCHITECTURE.md` benchmark list, `.agent/REPOSITORY_STATE.md`, this plan's Outcomes; the compact-diagonal question and the length-label SVG item handed to the owner as the next candidates.
