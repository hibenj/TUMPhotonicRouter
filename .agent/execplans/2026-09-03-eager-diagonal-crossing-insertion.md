# Eager insertion of perpendicular diagonal-x-diagonal crossings (replace the defer)

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md` (repository root).

## Purpose / Big Picture

Today the crossing-aware A\* kernel cannot reliably cross a fan of parallel 45-degree waveguides with a single straight 135-degree line, even though every such crossing is perpendicular, has ample margin, and is legal by the crossing ruleset. The concrete symptom: in `multiportmmi_32x32`, net `n_286` (Rust net id 287, `mmi0_multiport_2_4,o2 -> mol_array_1_mzi_16,o1`) must cross a fan of ~9 parallel diagonals spaced 22 um (11 grid cells) apart; the search crosses the top few and then the crossings stop being recorded, the search exhausts (`open_set_exhausted`, not a budget cap), and the net fails.

After this plan: a perpendicular crossing of a committed diagonal is recorded the moment the kernel detects it, even when the geometric intersection lies half a cell past the end of the current 1-cell move (the "X between cells" case). A unit test pins that behavior. `multiportmmi_32x32 --debug-stop-after-route 287` routes `n_286`, the full validation ladder stays at 0 verification errors, and pytest stays at its documented baseline. The stop-gap "defer" introduced on 2026-09-02 (commit `579aed2`) is removed, because deferring hands the crossing to the next move whose halo re-contacts the same partner in the same window and the crossing never matures.

## Progress

- [ ] Milestone 0: unit tests that fail today (`perpendicular_diagonal_x_crossing_is_recorded_eagerly`, `consecutive_perpendicular_diagonal_crossings_on_one_line`).
- [ ] Milestone 1: eager insertion in `crossing_move_outcome_with_segments` (src/astar.rs); tests pass.
- [ ] Milestone 2: remove the defer path and its `defer_contact_intersection_ahead` trace event.
- [ ] Milestone 3: sequential Harness pass (cargo test, pytest baseline, five-benchmark ladder, then `multiportmmi_32x32` stop-287 and full run) recorded here with a verifier packet and a PASS/FAIL verdict.
- [ ] Milestone 4: retrospective; `.agent/REPOSITORY_STATE.md` and `.agent/ORCHESTRATOR.md` active-plan pointer updated.

## Surprises & Discoveries

(None yet in this plan. The discoveries that motivated it are summarized in Context and Orientation so this document stays self-contained.)

## Decision Log

- Decision: The crossing detection for diagonal-x-diagonal must insert the crossing cells eagerly -- no defer, no pending -- exactly the way the post-crossing straight is already inserted eagerly by the completion chain.
  Rationale: Repository owner, 2026-09-03: "bei der diagonalen crossing erkennung [hat man] dann eben nicht pending, sondern man muss diese Zellen einfuegen wie auch normalerweise, weil sonst evtl das Halo danach nochmal im selben Fenster kollidiert." Evidence that the defer is only a stop-gap: on net 287 the fan partners 280/279/278/277 now accept (16.9k/12.2k/11.5k/6.5k accepts) but 276 -> 1.7k, 275 -> 26, 274 -> 0 accepts with 371 defers and zero legitimate candidate rejects (no `partner_margin`, no `reservation_footprint`, no `not_perpendicular` candidate) -- the deferred crossing never matures.
  Date/Author: 2026-09-03 / repository owner, recorded by Claude.

- Decision: Unit test first. No implementation lands without a Rust unit test that fails before and passes after; benchmark runs are supporting evidence, not the acceptance criterion.
  Rationale: The 2026-09-02 defer commit (`579aed2`) shipped with no unit test and turned out to be partial. The repository's stated priority is readability and testing over benchmark passing.
  Date/Author: 2026-09-03 / repository owner, recorded by Claude.

- Decision: No subagents for complex tasks in this session; the orchestrator runs the Explorer, Implementation, and Harness roles sequentially in one session and labels each pass here (the fallback `.agent/ORCHESTRATOR.md` prescribes when real subagents are unavailable).
  Rationale: Repository owner, 2026-09-03: subagent lanes have repeatedly stalled ("die oft stehen geblieben sind").
  Date/Author: 2026-09-03 / repository owner.

- Decision: Consolidate findings into this plan before every further diagnostic run.
  Rationale: 2026-09-02 stacked ~10 diagnostic runs without consolidation and the owner had to call it out.
  Date/Author: 2026-09-03 / repository owner.

## Outcomes & Retrospective

Not started.

## Context and Orientation

The router is a Rust library (`src/`) driven from Python (`routing_flow.py` -> `translation/route_rust.py`). Build and deploy the Rust side with, from the repository root:

    PYO3_PYTHON=.venv/bin/python cargo +stable build --release
    cp target/release/libphotonic_router.so python/photonic_router/_rust.abi3.so

(`rust-toolchain.toml` pins a Windows channel, hence `+stable`. The Python package loads the copied `.so`; forgetting the `cp` runs the old binary.)

Terms used below:

- **Grid / cell**: routing happens on a 2.0 um grid (`SCRIPT_GRID_SIZE_UM = 2.0` in `routing_flow_config.py`). A cell is `(x, y)` in grid units. Physical micrometers = `origin + (cell + 0.5) * 2.0`; the origin is NOT the layout bounding-box minimum (for `multiportmmi_32x32` it is about `(-19.5, 1562.125)`), so never hand-convert -- read `grid_centerline_um`/`realized_centerline_um` from the `committed-centerline-compare` diagnostic (`PHOTONIC_ROUTER_TRACE_PARTNER_NET=<id>`) or read the GDS directly with `klayout.db`.
- **Angle**: 0..7 in eighths of a turn; `crate::primitives::DIRECTIONS[angle]` gives the unit step. Angle 1 = `(1, 1)`, angle 7 = `(1, -1)`; these two are perpendicular diagonals.
- **Primitive**: one A\* move -- a straight of 1 or 4 cells, or a 45/90-degree bend -- with a `footprint` (cells swept) and `start_angle`/`end_angle`. See `src/primitives.rs` and `PrimitiveLibrary`.
- **Unified kernel**: `route_single_net_with_bounds_unified` in `src/astar.rs`. Tier-1 = dense, contact-free moves; Tier-2 = moves carrying crossing state (`CrossingExtension`: `pending_after_crossing_cells`, `straight_run_cells`, `crossed_mask`).
- **Diagonal halo**: for every diagonal segment of a move, the two "corner" cells beside each diagonal step are added as extra collision witnesses (`effective_collision_witness_offsets` -> `compact_diagonal_halo_cells`, `src/astar.rs`). Purpose (2026-08-27 rule, correct and required): two parallel diagonals in adjacent cells realize as physically overlapping waveguides, so adjacency must be detected even with no shared cell. Horizontal/vertical segments carry no halo.
- **Contact**: a witness cell owned by another committed net (`dynamic_core_owners.owner_at`). A contact sends the move to `crossing_move_outcome_with_segments` (`src/astar.rs`, ~line 6600 onward), which builds the move's route segments (`translate_primitive_path_segment`), intersects each with the contacted partner's committed centerline segments (`partner_segments[partner_idx]`, built from `committed_center_routes`) via `grid_segment_intersection_with_params`, and for every intersection checks, in order: perpendicularity (`grid_axes_are_perpendicular`), partner margin (`required_margin` cells of straight partner on each side of the point; 5 in the benchmarks), the reservation window (`crossing_reservation_window_is_clear`: a box of `crossing_half_size_cells` = 2 around the point must contain no third net), and the terminal-bump guard. A passing intersection is pushed to `route_intersections` as a `CrossingRouteIntersection`; downstream this increments `crossing_count`, sets `pending_after_crossing_cells` (the minimum straight run after the crossing) and the **eager completion chain** in the kernel (search for `EAGER_CHAIN_MAX_STEPS`) immediately appends straight primitives to pay that run off, legalizing any further crossings the run hits.
- **The gap this plan closes**: when the route (angle 7) and the partner (angle 1) are perpendicular diagonals, their grid paths form an X through a 2x2 block and the centerline intersection is at a half-cell point, e.g. `(3364.5, 898.5)`. The 1-cell move that ends one cell before the X already has a halo witness on the partner's cell, so contact fires -- but the intersection lies half a cell past the move's segment, `grid_segment_intersection_with_params` returns `None` for this move, and `route_intersections` stays empty for the partner. Before 2026-09-02 that branch hard-rejected the move ("contact without a same-move intersection" = grazing). Commit `579aed2` changed it to a defer: if `contact_intersection_lies_ahead(state, primitive, partner_segments, margin + 6)` (the partner's segment intersects the route's forward ray from the move end) the code `continue`s to the next contacted partner and the move survives without recording a crossing, on the assumption that the next move will contain the intersection and record it. That assumption fails in practice (see Decision Log evidence), which is what this plan fixes.

Relevant existing unit tests (pattern to copy), all in `src/astar.rs` `mod tests`: `crossing_move_detects_offset_diagonal_halo_contact` (~line 11319) builds an `ObstacleMap::new(800, 300)`, commits a diagonal partner from grid waypoints with `rasterize_waypoints_for_test` + `commit_route_with_clearance_and_allowed_core_overlaps`, builds a `CrossingSearchConfig { partners: vec![CrossingSearchPartner { net_id, waypoints, target_terminal_bump_guard: None }], ... }`, a one-cell angle-7 `Primitive { dx: 1, dy: -1, footprint: vec![(0,0),(1,-1)], ... }`, and calls `crossing_move_outcome(...)` directly. `crossing_move_accepts_perpendicular_shared_core_cell` (~line 11247) is the neighbouring accepted case where the X does share a cell.

## Plan of Work

**Milestone 0 -- pin the bug.** Add two tests to `src/astar.rs` `mod tests`, next to `crossing_move_detects_offset_diagonal_halo_contact`:

1. `perpendicular_diagonal_x_crossing_is_recorded_eagerly`: partner net 32 committed as the diagonal `(700,150) -> (760,210)` (angle 1, line `y = x - 550`; it passes through `(729,179)` and `(730,180)`). Route state `(728, 181, 7)`; the 1-cell angle-7 move goes to `(729, 180)`. The route line is `y = -x + 909`, so the centerline intersection with the partner is at `x = (909 + 550) / 2 = 729.5`, `y = 179.5` -- half a cell PAST the move's segment end `x = 729` (this is the "X between cells" case; `grid_segment_intersection_with_params` returns `None` for this move by construction). The move still makes contact through its diagonal halo: the halo of a diagonal step reaches one cell past the step end along the x direction (observed in the real failure: state `(3363,900,7)`, move end `(3364,899)`, witness `(3365,899)` = `end + (dx, 0)`), so here the witness `(730, 180)` is a cell of partner 32. The test must first assert that `effective_collision_witness_offsets` for this primitive, translated to the state, contains at least one cell owned by net 32 -- so that a different halo shape shows up as a fixture error, never as a false pass. Then: today `crossing_move_outcome` returns `Some` with `crossing_count == 0` (the defer keeps the move alive but records nothing); after Milestone 1 it returns `Some` with `crossing_count == 1`, exactly one `crossing_events` entry with `partner_net_id == 32`, its point within 0.6 cells of `(729.5, 179.5)`, and `pending_after_crossing_cells == min_straight_cells`. Use `min_straight_cells: 2`, `crossing_half_size_cells: 2`, and the `required_margin` the kernel derives from those (`crossing_required_margin_cells`). The partner's 60-cell diagonal gives it ample margin on both sides of the point, so the only thing that can fail is the recording itself.
2. `consecutive_perpendicular_diagonal_crossings_on_one_line`: two partners (32 and 33) as parallel angle-1 diagonals whose line offsets differ by 11 (so one X shares a cell and the next is between cells), crossed by a full unified search (`route_single_net_with_bounds_unified` via the same fixture style as `crossing_search_fixture`) from a source above-left to a target below-right on the perpendicular line. Assert: a route is found, `crossing_events.len() == 2`, one per partner, and `crossing_violations_for_realized_centerline`-style realized validation reports no violation (use the existing test helper the neighbouring kernel tests use for realized checks; if none fits, assert on `crossing_events` only and note it in Surprises).

Run `PYO3_PYTHON=.venv/bin/python cargo +stable test --release perpendicular_diagonal_x_crossing_is_recorded_eagerly` and expect the crossing-count assertion to FAIL before Milestone 1 (the test must be written so the pre-fix failure is the `crossing_count == 1` assertion, not a fixture error).

**Milestone 1 -- eager insertion.** In `src/astar.rs`, `crossing_move_outcome_with_segments`, at the branch `if route_intersections.len() == intersection_count_before { ... }`:

- Replace the boolean helper `contact_intersection_lies_ahead(state, primitive, partner_segments, lookahead) -> bool` with `eager_forward_crossing_intersection(state, primitive, partner_segments, lookahead) -> Option<ForwardIntersection>` where

        struct ForwardIntersection {
            x: f64,
            y: f64,
            distance_beyond_move_end: f64, // along the forward ray, in cells
            partner_segment_idx: usize,
            partner_u: f64,                 // parametric position on the partner segment
        }

  computed exactly as the boolean was: forward ray from the move end `(state.x + primitive.dx, state.y + primitive.dy)` along `DIRECTIONS[primitive.end_angle]` for `ceil(lookahead)` cells, intersected with each `partner_segments[partner_idx]` element via `grid_segment_intersection_with_params`; return the nearest hit.
- When it returns `Some(fi)`: run the SAME legality checks the in-move path runs, on the forward point: `grid_axes_are_perpendicular(primitive.end_angle, partner_segment.angle)` (else record `not_perpendicular` via `trace_crossing_candidate` and `return None`); partner margin `min(u, 1-u) * partner_segment.length >= required_margin` (else `partner_margin` reject, `return None`); `crossing_reservation_window_is_clear(...)` at `(fi.x, fi.y)` (else `reservation_footprint` reject, `return None`); `terminal_bump_guard_satisfied(...)`. If all pass, push a `CrossingRouteIntersection` with `distance_from_primitive_start = primitive_path_length + fi.distance_beyond_move_end`, `distance_before_on_segment` computed like the in-move terminal-segment case (`current_key.straight_run_cells + last segment length + fi.distance_beyond_move_end` when the last route segment does not start after a kink), `distance_after_on_segment = 0.0`, `segment_is_terminal = true`, `route_angle = primitive.end_angle`, `partner_angle`, `partner_idx`, `bit`. Do NOT `continue`; fall through so the existing downstream code (crossing count, `pending_after_crossing_cells`, reservation keys, the eager completion chain) treats it as a recorded crossing.
- The eager completion chain's first straight step will sweep through the X cells and re-contact the same partner. Verify (read the chain code around `EAGER_CHAIN_MAX_STEPS`, the "dense-bundle case" comment) that a re-contact with the partner just recorded is treated as the already-crossed partner and not rejected again; if it is not, extend the chain's per-step outcome to skip intersections for `pending_after_crossing_partner_index` within `crossing_half_size_cells + 1` of the recorded point. Record what you found in Surprises.
- If `eager_forward_crossing_intersection` returns `None` (no intersection ahead): keep today's reject (`classify_unresolved_crossing_contact` + `return None`). Nothing is deferred anymore.

**Milestone 2 -- remove the defer.** Delete the `"defer_contact_intersection_ahead"` trace event and any code path that can `continue` past a contact without either a recorded crossing or a reject. `grep -n defer_contact_intersection_ahead src/astar.rs` must return nothing.

**Milestone 3 -- sequential Harness pass.** Run, in this order, from the repository root, and paste the results into Concrete Steps / Artifacts:

    PYO3_PYTHON=.venv/bin/python cargo +stable test --release
    timeout 580 .venv/bin/python -m pytest tests -q
    for b in heater_s_mod multiportmmi_8x8 benes_8x8 multiportmmi_16x16; do timeout 300 .venv/bin/python routing_flow.py $b > /tmp/claude-1000/.../tol_$b.log 2>&1; echo "$b exit=$?"; done
    timeout 580 .venv/bin/python routing_flow.py benes_16x16
    timeout 1500 .venv/bin/python routing_flow.py multiportmmi_32x32 --debug-stop-after-route 287
    (then the full 447-net run, detached with a status file)

Expected: cargo tests all green including the two new ones; pytest `10 failed, 357 passed` (the 10 are the documented pre-existing failures; if `test_multiportmmi_8x8_routes_cleanly_through_first_mmi_fanin_boundary` moves, re-pin its counts with a derivation comment as was done on 2026-09-02); every ladder benchmark `photonic error_count 0` and `crossing error_count 0` in `build/verification/<benchmark>_{photonic,crossing}_verification.json`; `multiportmmi_32x32 --debug-stop-after-route 287` exits 0 with 287 routed records (this is the acceptance test for the bug); full run result recorded honestly whatever it is.

Label the pass in Progress as "Harness role pass (sequential, in-session)" with verdict PASS / FAIL / BLOCKED / INCONCLUSIVE.

**Milestone 4 -- close out.** Fill Outcomes & Retrospective; update `.agent/REPOSITORY_STATE.md` (Current Snapshot, Next Engineering Step) and the active-plan pointer paragraph in `.agent/ORCHESTRATOR.md` (it still says "no single active ExecPlan (2026-08-28)").

## Concrete Steps

Working directory for every command: the repository root `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`.

Build + test cycle:

    PYO3_PYTHON=.venv/bin/python cargo +stable build --release
    PYO3_PYTHON=.venv/bin/python cargo +stable test --release 2>&1 | grep -E "^test result|FAILED|panicked"
    cp target/release/libphotonic_router.so python/photonic_router/_rust.abi3.so

Expected `cargo test` transcript today (before this plan): three `test result: ok` lines (419 + 12 + 0 tests). After Milestone 0 and before Milestone 1: one `FAILED` naming `perpendicular_diagonal_x_crossing_is_recorded_eagerly`. After Milestone 1: all green, 421 in the first group.

Focused reproduction of the bug on the real benchmark (diagnosis only; the unit test is the acceptance criterion):

    PHOTONIC_ROUTER_TRACE_CROSSING_NET=287 PHOTONIC_ROUTER_TRACE_CROSSING_LEVEL1=1 \
      timeout 1000 .venv/bin/python routing_flow.py multiportmmi_32x32 --debug-stop-after-route 287 \
      > /tmp/claude-1000/<scratchpad>/l1.stdout.log 2> /tmp/claude-1000/<scratchpad>/l1.stderr.log
    for p in 280 279 278 277 276 275 274; do echo "partner $p accepts=$(grep -c "crossing-level1 net=287 partner=$p event=accept" .../l1.stderr.log)"; done

Before this plan that prints `280: 16905, 279: 12233, 278: 11528, 277: 6540, 276: 1682, 275: 26, 274: 0`. After Milestone 1 every fan partner the straight line crosses must show accepts, and the run must end with 287 routed (no `search-failure kind=` line for `source=(3040,1593`).

Note: `--debug-stop-after-route N` routes flow routes 1..N; Rust net id 287 is `n_286` and is flow route 287, so `287` INCLUDES the failing net (needed here) while a pre-failure GDS needs `286`.

## Validation and Acceptance

- `perpendicular_diagonal_x_crossing_is_recorded_eagerly` fails before Milestone 1 on the `crossing_count == 1` assertion and passes after.
- `consecutive_perpendicular_diagonal_crossings_on_one_line` passes after Milestone 1.
- `grep -c defer_contact_intersection_ahead src/astar.rs` prints `0` after Milestone 2.
- Ladder: five benchmarks with 0/0 verification errors; times within the 2026-09-02 range (heater ~8 s, mm8 ~16 s, benes8 ~35 s, mm16 ~95-105 s, benes16 ~180-210 s).
- `multiportmmi_32x32 --debug-stop-after-route 287` routes 287 records with 0 verification errors.
- pytest at baseline (10 failed / 357 passed) or re-pinned with a derivation.
- The full `multiportmmi_32x32` run's outcome (routed count, first failing net if any, wall time) is recorded in Outcomes whatever it is.

## Idempotence and Recovery

All build/test/benchmark commands are safe to repeat. Benchmark runs overwrite `build/routed_multiportmmi_32x32.gds` and `build/verification/*.json`; copy any GDS you want to keep to a distinct name first. To roll back the implementation: `git revert <commit>` then rebuild and re-copy the `.so`. Never leave a routing process running when you stop: `ps aux | grep "python routing_flow.py"` and kill by PID (do not `pkill -f` a pattern that matches your own shell).

## Artifacts and Notes

(To be filled with the failing-then-passing test transcript, the ladder table, and the stop-287 result.)

## Interfaces and Dependencies

In `src/astar.rs` (private, same module as the kernel):

    struct ForwardIntersection { x: f64, y: f64, distance_beyond_move_end: f64, partner_segment_idx: usize, partner_u: f64 }
    fn eager_forward_crossing_intersection(
        state: State,
        primitive: &Primitive,
        partner_segments: &[PartnerPathSegment],
        lookahead_cells: f64,
    ) -> Option<ForwardIntersection>;

`contact_intersection_lies_ahead` is deleted. `crossing_move_outcome_with_segments`'s signature is unchanged. No Python-side or pyo3 interface changes. Dependencies: none new; uses `crate::primitives::DIRECTIONS`, `grid_segment_intersection_with_params`, `grid_axes_are_perpendicular`, `crossing_reservation_window_is_clear`, `terminal_bump_guard_satisfied`, `CrossingRouteIntersection`, all already in `src/astar.rs`.

---
Revision note (2026-09-03, Claude): plan created at the repository owner's direction after the 2026-09-03 audit of the 2026-09-02 session; it supersedes the defer introduced by commit `579aed2` and records the owner's eager-insertion design as the required shape.
