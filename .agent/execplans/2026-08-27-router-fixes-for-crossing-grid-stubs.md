# Router fixes exposed by pre-placed crossing grids: diagonal halo in plain search, switch-pair fan-out, port run-in

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

`.agent/execplans/2026-08-26-preplaced-crossing-grids-for-benes.md` made `benes_4x4` and `benes_8x8` route with all crossings pre-placed as lattice components and the router only bridging switch ports to grid ports ("stubs"). `benes_16x16` still fails, and the repository owner's GDS review of the partial result showed two things that are **router** defects, not grid defects:

1. Two stub routes whose centerlines sit in *adjacent* cells as parallel 45-degree diagonals are realized as waveguides that physically cross twice (an enclosed loop in the GDS). The router's plain (crossings-disabled) search never sees this because the compact diagonal halo that `.agent/WORKFLOW.md` prescribes for one-cell-wide diagonals is only evaluated by the crossing-aware kernel (`LiveCrossingHook::evaluate`, `src/astar.rs`), not by the plain kernel.
2. A Benes switch's two ports are 1.25 um apart -- inside one 2 um routing cell -- and when both must connect toward the same side (which pre-placed grids force, and which never happens in the plain benchmark), the first committed route seals the second. The router has a mechanism for exactly this at multiport MMIs (staggered static stubs for dense port groups), but its group threshold is hard-coded `> 2` ports and its stub builder also grabs the grid's own ports.

After this plan: no two routes may occupy adjacent diagonal cells in any search mode (a unit test proves the plain kernel rejects it), `benes_16x16 --preplaced-crossing-grids true` completes with `error_count=0`, and every existing benchmark baseline is unchanged or explained.

## Progress

- [x] (2026-08-27) Milestone 1 done, commit `8facc36`: the unified kernel's Tier-1 fast path is now gated on the diagonal halo (`halo_free`) for both hooks; halo contact drops to the hook (crossing mode legalizes-or-rejects, plain mode rejects). New stat `diagonal_halo_contacts`, new test `plain_search_rejects_diagonal_adjacent_to_committed_diagonal`. Ladder: `cargo test --lib` 402/402; pytest 11 failed (same set)/351 passed; `benes_4x4` plain, `benes_8x8` lidar-pure stable, `multiportmmi_8x8` stable (111/111, 46.7 s) unchanged and clean; grid-mode 4x4/8x8 clean; 16x16 partial: 0 enclosed loops (was 2), now proceeds to net 136 (switch input pair). `benes_16x16` lidar-pure regression: see Artifacts.
- [ ] (superseded) Milestone 1: diagonal halo in the plain kernel. Make the no-crossing search reject a primitive move whose diagonal halo witnesses hit a *dynamic* obstacle (another committed route), exactly as `LiveCrossingHook` does when crossings are enabled. Unit test with two adjacent diagonals. Validate the full ladder (this changes every benchmark's plain searches).
- [ ] Milestone 2 (partial, 2026-08-27): switch-pair fan-out via the existing static-stub mechanism. Done: `_dense_fanout_min_ports()` (env `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`, default 3), `_dense_fanout_group_size()` excluding ports with an explicit access rule (looked up directly, so it works before `port_access_rule_by_spec` exists -- grid ports no longer get automatic stubs, which removed yesterday's `static-stubs` failure on `benes_8x8`), the four hard-coded `<= 2` guards made threshold-driven, a 2-port target group ranked 1/2 so its anchors differ in x, and two real bugs in the plain (crossings-off) fanout-stub corrector fixed: the target side of a target-stubbed net was never corrected to its anchor (`target_port=None if target_has_fanout_stub`, the old eager-stitch assumption; mirror of yesterday's crossing-aware `correct_target=True` fix), and both-stubbed nets were filed as `ALREADY_CORRECTED_NO_OP` and skipped entirely. With threshold 2 + `static-stubs`: pair anchors exist and are staggered (`sw_s1_1,o1` at x=1192, `o2` at 1198), `benes_16x16` reaches net 138, IO nets now correct; STILL FAILING: switch-to-switch nets whose route reaches the pair anchor via a bend plus a short (~7.5 um) straight -- see Surprises. Default-threshold behaviour unchanged (`multiportmmi_8x8`, `benes_8x8` lidar-pure clean, pytest at baseline). Decide (see Decision Log questions) between (a) dense-group threshold 2 with the existing static two-bend stubs at switch ports, keeping the router's stub builder off grid ports, or (b) a router-native allowance for two port-adjacent routes to share the port cell. Implement, then `benes_16x16` grid mode end to end.
- [ ] Milestone 3: straight run-in reservation for grid ports (zero width, ~16 um) so stubs approach along the row instead of hooking in from the side; measure hook count on the 16x16 partial (was 12 routes with an x-direction reversal).
- [ ] Milestone 4: full ladder (`cargo test --lib`, `pytest`, `benes_*` in both modes, `multiportmmi_8x8` stable baseline), `.agent/REPOSITORY_STATE.md`, retrospective.

## Surprises & Discoveries

- Observation (2026-08-27): in the 16x16 partial (`build/routed_benes_16x16_before_failure.gds`, stop after route 133) no record's centerline self-intersects and no two centerlines cross, yet the merged GDS has enclosed 17.5 x 11.5 um loops at (860.5, 646) and its mirror -- the bottom/top edge-lane grid ports. Cell map of the committed routes shows the neighbouring edge lanes' westward 45-degree jogs in adjacent cells (A at (459,518)->(453,512), B at (456,518)->(450,512)). Centerlines 2 um apart, waveguides 0.5 um wide, but a 6 um Euler bend deviates ~1.8 um from the cell-center polyline: the realized polygons overlap. Only final photonic verification would catch it, and 16x16 never reaches it.

- Observation (2026-08-27): `effective_collision_witness_offsets` (`src/astar.rs`) already computes halo cells for every diagonal segment of a primitive, but they are consumed only via `PrimitiveCrossingMetadata.extra_witness_offsets` inside `LiveCrossingHook::evaluate`. `primitive_footprint_free` (the plain path) tests `primitive.footprint` only.

- Observation (2026-08-27, Milestone 2): the last blocker is in the endpoint corrector, isolated by replaying the exact failing job: for a target anchor 0.4 um off the cell row, `shift_target_existing_axis_run_y` only accepts a **vertical** axis run to absorb the y delta (the "adjust the y-parallel straight" strategy the repository owner described); a route that arrives at the anchor as `... bend, 7.5 um horizontal` has none, the 45-degree strategy has no usable diagonal, bump candidates need a full straight, and the last resort (`absorb_endpoint_delta_into_axis_runs`) tries to *insert a meander* into the 7.5 um run, which is too short for two 6 um bends -> `NoMeanderCandidateSegment` ("no axis-aligned centerline segment is suitable for meander insertion"); the correction is skipped, the fixed stub gets spliced onto the raw cell center, and realization rejects the slanted last segment. The same net without stubs has a 20+ um run-in and corrects fine. Two ways out, both real work: (a) a corrector strategy "translate the final axis run by the delta and absorb it in the preceding bend/diagonal" (dogleg absorption already exists as a concept, `dogleg_absorber_for_angle`), or (b) guarantee a multi-cell straight run-in before anchors in the search (`require_terminal_straights` only forbids a bend as the *last* primitive; `TRACKS_STRAIGHT_RUN` bookkeeping exists only for the crossing hook). Anchor lane reservations (`PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS` 4/8) do not force a straight approach and did not help.

## Decision Log

- Decision (open question for the repository owner): should halo-based rejection in the plain kernel be unconditional (recommended: it is a physical-overlap correctness rule, independent of crossing mode) or behind a flag?
- Decision (open question): switch-pair fan-out via (a) existing static-stub mechanism at threshold 2, or (b) a router-native shared-port-cell allowance? Recommendation: (a) -- the mechanism exists and is validated on multiport MMIs; it needs the anchor pass to run after port access rules are resolved so grid ports are excluded.
- Decision: the crossing-grid module itself is not changed by this plan except for the port access rule's run-in length (Milestone 3).

## Outcomes & Retrospective

Not yet applicable.

## Context and Orientation

Rust side: `src/astar.rs` holds the unified A* kernel (`unified_kernel` module) with a pluggable `CrossingLegalityHook`; the crossing-aware hook is `LiveCrossingHook` (search for `impl CrossingLegalityHook for LiveCrossingHook`), the plain search uses a no-op hook. Primitive footprints come from `src/primitives.rs`; the halo cells for a diagonal step are `compact_diagonal_halo_cells(start, end, dx, dy)` (`src/astar.rs`). Dynamic obstacles (committed routes) are queried through the dense grid (`relative_offsets_free_with_profile`) and `obstacle_map.dynamic_owners_at`.

Python side: `translation/route_rust.py` decides dense port groups in `_is_dense_source_fanout_group` / `_is_dense_target_fanout_group` (`> 2` ports of one instance and angle), builds static stubs in `_build_static_fanout_anchors` (source) and `_build_static_fanout_target_anchors` (target), and resolves per-port access openings in `_resolve_port_footprint_cells` / `_keyed_port_access_rule` (component-keyed `ComponentPortAccessRule`s, plus the run-time registry added on 2026-08-26 in `python/photonic_router/routing_layers.py`, which the crossing grids use to give their ports a zero-size opening). A temporary experiment on 2026-08-26 (reverted) showed that the anchor passes run before `port_access_rule_by_spec` is populated, so an exclusion based on access rules must move the population earlier or the anchor passes later.

Reproduction: `PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16 --crossings false --preplaced-crossing-grids true` fails at `n_s0_3_o0_to_s1_1_i1__from_grid` (`out_2 -> sw_s1_1,o1`, target sealed by sibling `o2`'s route). Partial artifact: add `--debug-stop-after-route 133`.

## Plan of Work

Milestone 1: read the no-op hook and the plain kernel's move acceptance; add halo evaluation there (reuse `extra_witness_offsets`/`extra_witness_profile` from the primitive metadata so nothing is recomputed per move); when a halo witness hits a dynamic obstacle, reject the move (no crossing legalization is possible with crossings disabled). Add a Rust unit test next to `crossing_move_detects_offset_diagonal_halo_contact` that commits one diagonal route and asserts a plain search cannot place a parallel diagonal in the adjacent cell. Rebuild (`maturin develop --release` with the Linux toolchain overrides from the grid plan), then run the ladder.

Milestone 2: per the owner's answer. For (a): make the threshold configurable (`PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`, default 3, Benes grid mode sets 2 through the flow), exclude ports with an explicit access rule from group detection, and move `port_access_rule_by_spec` population ahead of the anchor passes. Verify the router's stub builder no longer touches grid ports (the 8x8 `static-stubs` failure `port endpoint correction would require an unsupported terminal stub` at `in_0` must disappear), then run 16x16.

Milestone 3: change the grid port access rule to `access_length_um=16, access_width_um=0` (env-overridable), measure hooks/overlaps on the partial, keep or revert on evidence.

## Concrete Steps

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$(pwd)/.venv/bin/python3" maturin develop --release
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu cargo test --release --lib
    PYTHONPATH=. .venv/bin/pytest -q
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16 --crossings false --preplaced-crossing-grids true
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 <STABLE_ROUTING_FLAGS from benchmarks/multiportmmi_8x8.py>

## Validation and Acceptance

Milestone 1 is accepted when the new Rust test passes, `cargo test --lib` is 402/402 (401 + 1), and the benchmark ladder is byte-identical or every difference is explained (a plain search that previously placed adjacent diagonals will now route differently; such cases must be listed with net names). The plan as a whole is accepted when `benes_16x16` grid mode reports `error_count=0` on the photonic verification JSON with 88 crossing components, and `benes_4x4`/`benes_8x8` grid mode plus all lidar-pure baselines and `multiportmmi_8x8` are still clean. `Verdict:` lines are written per run.

## Idempotence and Recovery

Ordinary reversible source edits; Rust changes need a rebuild before Python tests see them. One commit per milestone.

## Artifacts and Notes

(to be filled)

## Interfaces and Dependencies

`src/astar.rs`: the plain-search hook gains halo evaluation; no public signature change intended. `translation/route_rust.py`: `_dense_fanout_min_ports()` (env-backed), `_dense_fanout_group_size()` excluding rule-governed ports; `routing_flow.py` passes the threshold for grid mode.
