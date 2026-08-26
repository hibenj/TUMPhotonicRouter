# Pre-place topology-derived crossing grids for Benes benchmarks and route only the stubs

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md` (the standing policy for all ExecPlans in this repository).

## Purpose / Big Picture

Today the router treats every optical crossing as something to be *discovered* during A* search (`--crossings true --crossing-mode lidar-pure`). For a Benes network that is wasteful: a Benes network is a pure permutation structure, and the exact set of crossings in every layer -- which pairs of waveguides cross, and in what order -- is fully determined by the graph before any routing happens. This repository already computes that information (`benchmarks/benes.py` and `python/photonic_router/crossing_plan.py`, see Context and Orientation). What it does not yet do is *use* it to build geometry up front.

After this plan, running

    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8 --preplaced-crossing-grids true

will, before any net is routed, place one pre-built, fully-wired "crossing grid" component into each interstage layer of the layout. Each grid contains every crossing that layer needs, already connected internally with straights and bends, with N input ports on its left edge and N output ports on its right edge. Every interstage net is then routed as two crossing-free stubs: switch output port to grid input port, and grid output port to next switch input port. The routing stage runs with crossings **disabled**, because the grids have already resolved every crossing. Final verification must report `error_count=0`, the number of crossing components in the layout must equal the topology-predicted crossing count (2 for `benes_4x4`, 16 for `benes_8x8`, 88 for `benes_16x16`), and the run should be dramatically faster than the current crossing-discovery baseline (`benes_16x16` currently ~215 s).

This is scoped to the three currently-working Benes benchmarks only: `benes_4x4`, `benes_8x8`, `benes_16x16`. It is a separate, opt-in comparison/optimization mode, exactly the kind `.agent/PROJECT_GOAL.md` already reserves ("Topology-precomputed crossings ... should be included later as a separate comparison or optimization mode"). It must not touch the `lidar-pure` router-discovered path, and the benchmark files themselves (`benchmarks/benes_*.py`) must not be rewritten -- all derivation happens inside the routing flow.

## Progress

- [x] (2026-08-26) Milestone 0: characterized. Substitution point is `routing_flow.py::run_routing_flow` right after `_layout_from_schematic_stage` (both `schematic` and `unrouted_layout` flow from there into `run_photonic_routing_stage` and `verify_and_attach_photonic_reports`); `benchmark_metadata.load_benchmark_metadata` tolerates unknown grid instances (`NodeType.GATE`, delay 0); the verifier's crossing-component checks only see *realized* crossings, so grid-internal crossings are invisible to them; grid waveguides on the WG layer become ordinary static obstacles (bounding-box mode) with port-open cells like any instance; `codex` is available (not used -- every slice here needed design iteration against geometry, see Decision Log).
- [x] (2026-08-26) Milestone 1: `benes_4x4` end to end in grid mode, `Verdict: PASS` (`status=complete, success=True, error_count=0, warning_count=0`, 20/20 routes, 2 grids / 2 crossings, GDS contains exactly 2 PDK crossing cells).
- [x] (2026-08-26) Milestone 2: general generator, three design iterations (see Surprises & Discoveries): (a) uniform compact pitch with all lanes pulled inward -- rejected by the repository owner on the 4x4 GDS; (b) diamond lattice (column pitch = lane pitch) so every lane is one straight diagonal through all its crossings, plus per-run grids so non-crossing lanes bypass the grid entirely; (c) grid ports on the lanes' natural rows with a deterministic in-grid fan-in/fan-out (90-degree jogs, one x-column per lane). `benes_8x8` `Verdict: PASS` (68/68, 16 crossings, 6 grids).
- [x] (2026-08-26) Milestone 3: `benes_16x16` `Verdict: PASS` (196/196, 88 crossings, 14 grids, widest grid 224 um in a ~535 um band; routing stage 2.3 s, wall 27.5 s, vs. `lidar-pure` stable baseline 215 s at `3c2b2af` -- re-measured today, see Artifacts and Notes). Geometry cross-checked beyond the verifier: 0 grid-internal piece overlaps, 0 route-route overlaps, 0 route-grid overlaps for all three sizes (scratch shapely/KLayout checks, now `tests/test_preplaced_crossing_grids.py` for the grid-internal part).
- [ ] Milestone 4: PLM pass-through is only *prepared* (`grid_internal_length_um_by_original_net` exists on `DerivedCrossingLayout`; not yet wired into the PLM graph); validation ladder run (`pytest`: 11 failed / 351 passed / 1 skipped, the 11 being the pre-existing set -- confirmed for the four not-listed-by-name ones by rerunning them at the pre-change HEAD in a worktree; `benes_4x4` plain and `benes_8x8` lidar-pure flag-off paths clean); `.agent/REPOSITORY_STATE.md` update and retrospective pending; repository owner visual GDS review of the final geometry pending.

## Surprises & Discoveries

- Observation (2026-08-26, Milestone 2): a uniform compact lane pitch pulls *every* lane -- crossing or not -- into the grid core, so the 4x4 outer lanes (which cross nothing) visibly jog inward. Rejected by the repository owner from the GDS ("unaffected edges ... should not be affected by the crossings"). Fix: per-stage runs of participating lanes (a non-crossing lane is never passed, so it partitions the schedule) each get their own grid; non-participating nets stay ordinary single nets.

- Observation (2026-08-26, Milestone 2): with bends back to horizontal after every crossing, consecutive crossings of one lane were not collinear (repository owner: "why are they not aligned in a way that we only need straights ... between the crossings?"). Analysis of the actual Benes plans: **no lane ever reverses direction or moves again after pausing** in any stage of 4x4/8x8/16x16 (checked by simulation, now enforced by `_lane_movement`, which raises otherwise). So with column pitch = lane pitch the crossing centers form a diamond lattice and each lane is `[wait] -> 45 bend -> one straight diagonal through k crossings -> bend -> [wait]`.

- Observation (2026-08-26, Milestone 3): the compact grid made `benes_16x16` unroutable at the *stubs*, not in the grid: `FAILED.txt` showed `corridor_clearance_source_region_size=1` / `target_region_size=1`. A Benes switch's two ports are 1.25 um apart -- inside one 2 um routing cell -- and a compact grid 640 um away forces both stubs to leave (or arrive) in the same direction; whichever routes first seals the other. In the normal benchmark this never happens because a Benes pair always diverges (one up, one down). Pitch/keepout knobs only moved the failing net. Fix: grid ports on the natural rows (siblings spread 2 um apart so each stub has its own cell row) and the fan-in done as real geometry inside the grid, where the 1.25 um pair is fine (the PDK switch itself does it).

- Observation (2026-08-26, Milestone 3): first fan-in ordering (by travel distance) produced real 0.25 um^2 perpendicular collisions inside 8x8/16x16 grids -- an outer lane can travel *less* than an inner one when its slot is also farther out, so it turned first and its vertical run cut through the inner lane's bend. Correct rule: order by the far row (closest-to-core first on entry, farthest first on exit). Note the diagnostic that found this initially used `inst.trans` (integer, 90-degree only) and mis-placed every 45-degree piece; use `inst.cplx_trans`.

- Observation (2026-08-26, Milestone 3, verification blind spot): with sibling grid ports spread exactly 4 um (= 2 routing cells) apart, the two `__from_grid` stubs into one switch were realized as S-curves that *cross each other* ~200 um before the port (centerline distance 0.07 um, overlap 1.09 um^2), and `verify_photonic_routing` did not flag it: its `min_route_overlap_area_um2` default is 2.0 um^2, larger than a full waveguide-width crossing (0.25 um^2 for a perpendicular hit; ~1 um^2 for a shallow one). Spreads of 2, 6 and 10 um all give zero overlaps; 2 um is now the default. The threshold itself was **not** changed (it would alter pass/fail on other benchmarks) -- flagged to the repository owner as a separate decision; this is very plausibly the "overlapping geometry from routes" they saw in their GDS.

- Observation (2026-08-26, plan drafting): the Benes switch component's two output ports are only 1.25 um apart (`benes_mmi_heater_switch` ports `o3` at `(445.5, +0.625)` and `o4` at `(445.5, -0.625)`), while switches are placed on a 220 um pitch. So the two lanes leaving one switch are almost coincident, and the grid's input ports cannot simply sit on the switch-port Y values -- the stubs from a switch to the grid must fan the two lanes apart to the grid's lane pitch. This is normal routing work (the current router already does this between switches) but it means the grid's lane pitch is a real design parameter, not something inherited from the switch.
  Evidence: `PYTHONPATH=. .venv/bin/python -c "import gdsfactory as gf; gf.gpdk.PDK.activate(); from benchmarks.benes import *; register_benes_cells(); s=gf.get_component(SWITCH_COMPONENT); print(s.dbbox(), [(p.name, tuple(p.dcenter), p.orientation) for p in s.ports])"` prints `(-10,-40.5;445.5,40.5) [('o1', (-10.0, -0.625), 180.0), ('o2', (-10.0, 0.625), 180.0), ('o3', (445.5, 0.625), 0.0), ('o4', (445.5, -0.625), 0.0)]`.

- Observation (2026-08-26, plan drafting): the generic-PDK crossing component (`gf.components.crossing()`, the one `translation/route_rust_crossing_components.py::_active_crossing_component` already uses) is an 8 um x 8 um square with four axis-aligned ports: `o1` left (180 deg), `o3` right (0 deg), `o2` top (90 deg), `o4` bottom (270 deg). A swap of two adjacent lanes realized with +-45 degree diagonals therefore needs the crossing rotated by 45 degrees so its ports line up with the diagonals. This is the "rotation" question the repository owner flagged; it is resolved by construction in Milestone 2's generator, but must be verified against the realized-crossing verifier's own perpendicularity/straight-margin rules.

## Decision Log

- Decision: do not modify any `benchmarks/benes_*.py` file. Instead, a new in-flow stage (see Plan of Work) derives a *second* schematic and layout from the benchmark's own schematic -- adding one grid instance per interstage layer and splitting each interstage net into two stub nets -- and hands those derived objects to the existing, unchanged routing/verification stages.
  Rationale: repository owner's instruction ("it should not rewrite the benchmark. it should still be part of the flow"). Deriving rather than editing keeps the benchmark the single source of truth for the network, lets the same benchmark run in both modes for comparison, and means routing, endpoint correction, and verification see ordinary schematic nets and ordinary placed instances with no special cases.
  Date/Author: 2026-08-26, repository owner + plan drafting.

- Decision: the routing stage runs with `enable_crossings=False` in this mode.
  Rationale: repository owner's instruction ("passed through the normal stage basically with crossings turned off, since they should all be resolved"). Every crossing lives inside a grid; the stubs are crossing-free by construction (the grid's input port order equals the layer's `initial_edge_order` and its output port order equals `final_edge_order`, so stubs on each side connect monotonically ordered lanes). If a stub run ever *needs* a crossing, that is a bug in the grid's port ordering, and the "no crossings allowed" routing failure is the correct, loud signal.
  Date/Author: 2026-08-26, repository owner.

- Decision: grid internal wiring may use straights of any length, 45 degree bends, and 90 degree bends -- the same primitive vocabulary the router itself uses -- built as a gdsfactory component from the active PDK's straight/bend/crossing cells.
  Rationale: repository owner's instruction. Using the same vocabulary keeps the grid's geometry within what `translation/photonic_verification.py` already knows how to classify.
  Date/Author: 2026-08-26, repository owner.

- Decision: path-length matching is not a gate for this plan, but the design must not preclude it: every stub net's record must carry, or be joinable to, the fixed internal length its lane travels inside the grid, so PLM can later treat a split net as one edge of known total length.
  Rationale: repository owner ("actually this is not that important here. but it generally makes sense that this can be enabled and should be thought of here"). Handled in Milestone 4 as a pass-through with reported lengths, not as full PLM re-validation.
  Date/Author: 2026-08-26, repository owner.

- Decision: grids are built per contiguous run of participating lanes; nets that cross nothing are not split and are routed as ordinary nets. Crossing centers form a diamond lattice (column pitch = lane pitch) so a lane needs bends only where it starts/stops moving. Grid ports sit on the lanes' natural rows (siblings spread 2 um), and the move to the lattice slot is done inside the grid with 90-degree jogs in per-lane x-columns.
  Rationale: repository owner's three corrections on the GDS (unaffected lanes must be untouched; crossings must connect by straights; dense 45-degree lattice is fine for 16x16) plus the stub-routing failure analysis in Surprises & Discoveries. `benes_16x16` widest grid is 224 um in a ~535 um band.
  Date/Author: 2026-08-26, repository owner + this implementation.

- Decision: implemented directly rather than via Codex.
  Rationale: every slice changed shape after looking at real geometry (three grid designs in one day, two of them overturned by GDS inspection and a routing failure); none was stable enough to hand over as a fully specified task, which is the boundary `.agent/CLAUDE_CODEX_FLOW.md` and the repository's own Codex experience set.
  Date/Author: 2026-08-26, this implementation.

- Decision: the mode is exposed as a new boolean flag `--preplaced-crossing-grids` (Python keyword `preplaced_crossing_grids: bool = False` on `run_routing_flow`), not as a fourth value of `--crossing-mode`.
  Rationale: `--crossing-mode` selects *how A* discovers crossings*; this mode turns A* crossing discovery off entirely and does its work before routing. Making it a separate flag keeps that distinction honest and lets the flow reject the combination `--crossings true --preplaced-crossing-grids true` explicitly. This is a plan-drafting assumption, not a repository-owner decision; revisit if the owner prefers a mode value.
  Date/Author: 2026-08-26, plan drafting (assumption).

## Outcomes & Retrospective

Not yet applicable -- no milestone complete.

## Context and Orientation

This repository routes photonic waveguides with a Python orchestration layer calling a Rust A* core. The relevant flow for this plan is entirely on the Python side:

`routing_flow.py::run_routing_flow` loads the benchmark schematic (`benchmarks/<name>.py::build_schematic`), builds the unrouted layout with `translation/layout_from_schematic.py::layout_from_schematic(schematic)` (one `add_ref` per schematic instance, positioned from `schematic.placements`), then calls `routing_flow_optical.py::run_photonic_routing_stage(benchmark_name=..., schematic=..., unrouted_layout=..., config=OpticalRoutingStageConfig(...))`, which loads benchmark metadata (`node_depths`/`node_ranks`/`edge_ranks`) and calls `translation/route_rust.py::route_nets_rust(unrouted_layout, schematic, ..., enable_crossings=..., crossing_mode=..., node_depths=..., edge_ranks=...)`. The route session enumerates nets from `schematic.netlist.routes` (`translation/route_rust.py`, `_build_route_jobs_and_fanout_clustering`, around line 6728). Afterward `routing_flow.py::verify_and_attach_photonic_reports` runs `translation/photonic_verification.py`, whose coverage check compares routed records against `_expected_route_keys(schematic)` -- i.e. against whatever schematic it is handed. The important consequence: if the derived schematic (with split stub nets and grid instances) is what gets passed to routing *and* verification, everything downstream works unmodified, because to those stages a grid is just another placed instance with ports and a stub is just another net.

A "Benes network" here is `benchmarks/benes.py::build_benes_schematic(size)`: `stage_count` columns of 2x2 switches (`benes_mmi_heater_switch`, bbox x from -10 to 445.5 um, so about 455 um wide; ports `o1`/`o2` on the left at 180 deg, `o3`/`o4` on the right at 0 deg, the pair only 1.25 um apart vertically), placed at `x = 220 + stage * 1000` um and `y = (switches_per_stage - 1 - index) * 220` um. The free horizontal band between one stage's right edge and the next stage's left edge is therefore about `1000 - 455 - 10 = 535` um wide; this band is where a grid goes. Every interstage net `n_sA_i_oP_to_sB_j_iQ` connects switch `sA_i` output port `oP` (0 = top `o3`, 1 = bottom `o4`) to switch `sB_j` input port `iQ` (0 = top `o2`, 1 = bottom `o1`).

The precomputed crossing information already exists in two equivalent forms. `benchmarks/benes.py::benes_topology_metadata(size)` returns `crossings_by_stage` (interstage layer index -> list of crossing pairs). Counts, confirmed by running it: `benes_4x4` 2 crossings `{0: 1, 1: 1}`; `benes_8x8` 16 `{0: 6, 1: 2, 2: 2, 3: 6}`; `benes_16x16` 88 `{0: 28, 1: 12, 2: 4, 3: 4, 4: 12, 5: 28}`. The router-facing form is `python/photonic_router/crossing_plan.py::build_crossing_plan(analyze_schematic_topology(schematic, node_depths=..., node_ranks=..., edge_ranks=...))`, a `CrossingPlan` whose `stages[(source_depth, target_depth)]` is a `CrossingStagePlan` with `initial_edge_order` (lanes top-to-bottom entering the layer), `final_edge_order` (lanes leaving it), and `events`: a sequence of `CrossingEvent`s, each an *adjacent swap* of two lanes tagged with a `level` (a column index; all swaps at one level are disjoint and can happen side by side) and `order_index`. `CrossingStagePlan.validate()` already proves that applying the swaps in order turns `initial_edge_order` into `final_edge_order`. This is precisely a bubble-sort permutation network, and it is the blueprint for a grid: one column per level, one X-shaped swap per event in that column, straight pass-through for every lane not swapping at that level. Example for `benes_8x8` stage 1->2: 6 crossings across 3 levels (3, 2, 1 swaps).

A "crossing grid" (this plan's term) is one gdsfactory `Component` per interstage layer with N input ports `in_0..in_{N-1}` on its left edge (top to bottom, in `initial_edge_order`) and N output ports `out_0..out_{N-1}` on its right edge (in `final_edge_order`), containing every swap of that layer already wired. The physical crossing is the same PDK component realization already uses (`gf.components.crossing()`, 8 x 8 um, axis-aligned ports). An adjacent swap between two lanes of pitch `p` is built as: each lane bends 45 deg toward the other, runs a diagonal, and bends back 45 deg; the two diagonals are perpendicular and meet at the column's midpoint, where a crossing rotated 45 deg is placed with its ports on the diagonals. The column's width is fixed by the bend radius and the pitch; the swap's internal length per lane is deterministic and must be recorded for PLM. Non-swapping lanes at that level are straights of the column width. For `benes_4x4`, each layer has exactly one swap and one level, so the grid degenerates to one crossing at the layer's X/Y center, as the repository owner described.

Terms: "stub" -- one of the two crossing-free routed nets a split interstage net becomes (switch-to-grid or grid-to-switch). "layer" / "stage" -- the interstage band between switch column d and d+1, identified by `(source_depth, target_depth)` in `CrossingPlan` and by index `stage` in `crossings_by_stage`. "derived schematic" -- the in-memory schematic this plan constructs from the benchmark's schematic, never written back to `benchmarks/`.

## Plan of Work

Milestone 0 (characterize, read-only except this file). Confirm by reading, and record here with line references: (a) exactly which `OpticalRoutingStageConfig` fields and which `run_photonic_routing_stage` lines pass `schematic`/`unrouted_layout` onward, so the derived pair can be substituted at one point; (b) whether `load_benchmark_metadata` (`routing_flow_optical.py`) needs the derived schematic or the original (it derives `node_depths`/`edge_ranks` from benchmark metadata keyed by instance names, so the added grid instances need either no entry or a harmless one -- check what happens with unknown instances); (c) how `translation/photonic_verification.py` treats a placed instance's ports and geometry as obstacles and whether a grid's waveguides inside its own bbox would be flagged as unexpected route geometry (they should not: they are instance geometry, not routed records, but confirm the crossing-component-overlap checks `_verify_crossing_component_overlaps` do not fire on grid-internal crossings); (d) which layer the grid's waveguides must be drawn on to match `route_layer=(1, 0)` and `route_width_um=0.5`; (e) how PLM's `build_graph_from_schematic` would see split nets (Milestone 4 input). Also check `which codex` and decide which slices below are well-specified enough to dispatch per `.agent/CLAUDE_CODEX_FLOW.md`.

Milestone 1 (`benes_4x4`, one crossing per layer). Create a new module `translation/preplaced_crossing_grids.py` with: `build_crossing_grid_component(stage_plan: CrossingStagePlan, *, lane_pitch_um, bend_radius_um, route_layer, route_width_um) -> Component` (for this milestone only the single-event case needs to work, but write the signature for the general case); `derive_preplaced_crossing_schematic(schematic, crossing_plan, *, stage_geometry) -> DerivedCrossingLayout` returning the derived schematic (original instances + one `crossing_grid_<stage>` instance per layer placed at the band's center, original non-interstage nets unchanged, every interstage net replaced by `<net>__to_grid` and `<net>__from_grid` stub nets) plus a mapping `stub_nets_by_original_net` and `grid_internal_length_um_by_original_net`. Register the grid components so `layout_from_schematic`'s `gf.get_component(instance.component)` can resolve them (either register a factory with the active PDK the way `benchmarks/benes.py::register_benes_cells` does, or build the derived layout directly by `add_ref` of the built component -- decide during implementation and record it). Wire the flag through `routing_flow.py` (`--preplaced-crossing-grids`, `run_routing_flow(preplaced_crossing_grids=...)`) so that when set: the crossing plan is built from the benchmark metadata, the derived schematic/layout replace the originals for `run_photonic_routing_stage` and `verify_and_attach_photonic_reports`, and `enable_crossings` is forced `False` (raise a clear error if the user also passed `--crossings true`). Add a verification-side counter: number of placed crossing-grid instances and total crossing components inside them, written into the photonic verification JSON `metrics`, and a hard check that it equals `len(crossing_plan.events)`. Acceptance: `benes_4x4` routes all stub nets, both verification JSONs `error_count=0`, metrics show 2 grid instances / 2 crossings.

Milestone 2 (general generator, `benes_8x8`). Generalize `build_crossing_grid_component` to the multi-level bubble-sort layout described in Context and Orientation. This is where the rotation/spacing experiments live: parameterize column spacing, lane pitch, and the diagonal length, and try at least (i) the minimum geometry the bend radius allows and (ii) a looser spacing, recording which passes the realized-crossing rules (`min_straight_cells_per_crossing`, `crossing_half_size_cells`, perpendicularity) and how wide the resulting grid is versus the ~535 um band. Add a focused unit test (`tests/test_preplaced_crossing_grids.py`) that builds the grid for every `benes_8x8` stage and asserts: port counts, port ordering matches `initial_edge_order`/`final_edge_order`, crossing-ref count equals event count, no two waveguide polygons overlap except at crossing refs, and grid bbox fits the band. Acceptance: `benes_8x8` end to end `error_count=0`, 16 crossings.

Milestone 3 (`benes_16x16`, density, timing). Run the 16x16 case (28 crossings in 7 levels for the outer layers); tune spacing so the widest grid still fits the band, or record why it cannot and what pitch would be needed. Record wall-clock time against the documented `lidar-pure` stable baseline (215.2 s at `3c2b2af`; re-measure on the same day rather than trusting the number). Acceptance: 88 crossings, `error_count=0`, timing table in Artifacts and Notes.

Milestone 4 (PLM pass-through, ladder, close-out). Make each original interstage net's total length recoverable: stub records plus `grid_internal_length_um_by_original_net`, reported in the routing summary; if cheap, teach the PLM graph builder to collapse the two stubs and the grid lane into one edge of that total length (do not gate the plan on full PLM validation -- see Decision Log). Run the full ladder: `cargo test --lib` (no Rust change expected; confirm 401/401), `PYTHONPATH=. .venv/bin/pytest -q` (335 passed + the same 11 documented failures, plus the new tests), the three Benes runs in both modes, and `multiportmmi_8x8` stable baseline to prove the flag-off path is byte-identical. Update `.agent/REPOSITORY_STATE.md` and write the retrospective.

## Concrete Steps

All commands run from the repository root with the project virtualenv. No Rust change is expected; if one becomes necessary, rebuild with

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$(pwd)/.venv/bin/python3" maturin develop --release

Print the crossing plan for a benchmark (works today):

    PYTHONPATH=. .venv/bin/python -c "
    from photonic_router.topology_analysis import analyze_schematic_topology
    from photonic_router.crossing_plan import build_crossing_plan
    from benchmarks.benes_8x8 import build_schematic, NODE_DEPTHS, NODE_RANKS, EDGE_RANKS
    t = analyze_schematic_topology(build_schematic(), node_depths=NODE_DEPTHS, node_ranks=NODE_RANKS, edge_ranks=EDGE_RANKS)
    print(build_crossing_plan(t).to_text())
    "

Expected head of output: `CrossingPlan: 16 crossing(s), 6 stage(s)` then per-stage `source order`, `target order`, and `level k:` swap lines.

Current crossing-discovery baselines for comparison (each benchmark file's own `STABLE_ROUTING_FLAGS`):

    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8 --crossings true --crossing-mode lidar-pure --proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16 --crossings true --crossing-mode lidar-pure --proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3

(Check the exact flags in `benchmarks/benes_8x8.py`/`benes_16x16.py` before running; the 16x16 run takes ~4 minutes -- state the cap before starting, per `.agent/WORKFLOW.md`.)

New mode, once Milestone 1 lands:

    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4 --preplaced-crossing-grids true

Expected: routing completes, `build/verification/benes_4x4_photonic_verification.json` has `error_count: 0` and a `preplaced_crossing_grids` metric block with `grid_count: 2, crossing_component_count: 2, expected_crossing_count: 2`.

## Validation and Acceptance

Per milestone, the acceptance is stated in Plan of Work. Globally: the flag-off path must be byte-identical to today (re-run `multiportmmi_8x8` stable baseline and `benes_8x8` `lidar-pure` baseline and compare verification JSON counters); every Benes run in the new mode must reach `Verdict: PASS` from an explicit harness pass that opens the verification JSON files and reads `status`, `error_count`, `issues`, and the new grid metrics, not just the exit code; crossing component count must equal the topology count; and a visual GDS inspection by the repository owner of at least one 8x8 grid is expected, since "passes verification" has repeatedly not been the same as "looks right" in this repository (see `2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`'s Surprises & Discoveries).

## Idempotence and Recovery

All steps are ordinary reversible source edits plus generated files under `build/` (gitignored). The mode is opt-in behind a flag defaulting to off, so no existing benchmark behavior changes unless the flag is passed. Commit one coherent step at a time per `.agent/GIT_WORKFLOW.md`.

## Artifacts and Notes

Grid-mode results, 2026-08-26, default geometry (`lane_pitch_um=20`, `bend_radius_um=5`, `fan_column_pitch_um=4`, `port_pair_spread_um=2`, all overridable via `PHOTONIC_ROUTER_CROSSING_GRID_*` environment variables):

    benchmark    routes  grids  crossings  widest grid  routing stage  wall   verifier
    benes_4x4     20/20    2       2          56 um        0.24 s      3.1 s  error_count=0
    benes_8x8     68/68    6      16         112 um        0.60 s      6.6 s  error_count=0
    benes_16x16  196/196  14      88         224 um        2.32 s     27.5 s  error_count=0

Reference (`lidar-pure` stable baselines, same day, same machine): `benes_8x8` 40.6 s wall; `benes_16x16` routing stage 217.1 s, wall 228.8 s (partly overlapping a pytest run; 215.2 s recorded at `3c2b2af`). Grid mode is ~95x faster on the routing stage for 16x16. The 16x16 grid-mode wall time is dominated by verification/GDS write, not routing.

Geometry cross-checks (scratch scripts, KLayout regions with `cplx_trans` + shapely on `corrected_centerline_um`): grid-internal piece overlaps 0/0/0; route-route overlaps 0/0/0; route-grid overlaps 0/0/0. Sibling spread sweep on 8x8/16x16: 0 um -> stubs share a cell row, router fails; 2, 6, 10 um -> clean; 4 um -> crossing S-curves (see Surprises & Discoveries).

Verification JSON gains a `preplaced_crossing_grids` block (top level and `metrics`) with `grid_count`, `expected_crossing_count`, `crossing_component_count`, `split_net_count`, per-grid widths/heights; the flow raises before routing if placed != expected.

## Interfaces and Dependencies

Implemented in `translation/preplaced_crossing_grids.py`: `CrossingGridGeometry` (+ `crossing_grid_geometry_from_env`), `split_stage_into_participating_runs`, `build_crossing_grid_component(stage_plan, geometry, *, entry_row_by_net, exit_row_by_net) -> CrossingGridBuildResult`, `derive_preplaced_crossing_layout(schematic, unrouted_layout, crossing_plan, *, geometry) -> DerivedCrossingLayout`, `preplaced_crossing_grid_metrics`, `build_crossing_plan_for_benchmark`. Flow wiring: `routing_flow.py::_preplaced_crossing_grids_stage` and the `preplaced_crossing_grids` keyword / `--preplaced-crossing-grids` flag; `routing_flow_verification.verify_and_attach_photonic_reports(extra_report_metadata=...)`. Tests: `tests/test_preplaced_crossing_grids.py`. The original sketch follows for the record:

    @dataclass(frozen=True)
    class CrossingGridGeometry:
        lane_pitch_um: float
        column_pitch_um: float
        bend_radius_um: float
        route_layer: tuple[int, int]
        route_width_um: float

    def build_crossing_grid_component(stage_plan: CrossingStagePlan, geometry: CrossingGridGeometry) -> gf.Component
        # ports: in_0..in_{N-1} (left, 180 deg, top-to-bottom = stage_plan.initial_edge_order)
        #        out_0..out_{N-1} (right, 0 deg, top-to-bottom = stage_plan.final_edge_order)
        # info: crossing_count, lane_length_um_by_net_name

    @dataclass(frozen=True)
    class DerivedCrossingLayout:
        schematic: Schematic            # original instances + grid instances; interstage nets split
        unrouted_layout: gf.Component
        stub_nets_by_original_net: dict[str, tuple[str, str]]
        grid_internal_length_um_by_original_net: dict[str, float]
        expected_crossing_count: int

    def derive_preplaced_crossing_layout(schematic: Schematic, crossing_plan: CrossingPlan, *, geometry: CrossingGridGeometry) -> DerivedCrossingLayout

Consumers: `routing_flow.py` (flag plumbing and substitution of the derived pair), `routing_flow_optical.py` (unchanged if the substitution happens before it is called), `translation/photonic_verification.py` (new metrics block only). Depends on `python/photonic_router/crossing_plan.py::CrossingPlan`/`CrossingStagePlan` and the active PDK's `straight`, `bend_euler`/`bend_euler_all_angle` (the same factories `python/photonic_router/primitive_library.py` uses), and `gf.components.crossing`.
