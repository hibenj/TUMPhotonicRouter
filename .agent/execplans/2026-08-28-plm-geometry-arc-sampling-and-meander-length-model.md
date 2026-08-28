# PLM geometry: smooth arc sampling everywhere, and the meander length model that books (amplitude - r) too little per meander

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

The repository owner looked at a path-length-matched `heater_s_mod` GDS and reported two things: the inserted meander bumps look "edgy", and the lengths measured in the GDS are not actually matched. After this plan, (1) every bend in the realized layout -- primitive bends, endpoint-correction jogs, meander connectors and U-turns -- is sampled finely enough that no chord deviates more than 5 nm from its arc, and the GDS stays inside the GDSII record limits however long a meander gets; and (2) the second observation is explained down to the formula, with the fix spelled out for the owner's decision.

## Progress

- [x] (2026-08-28) Milestone 1: one shared sampler. `arc_samples_per_90_deg(radius_um)` (`src/geometry_realization.rs`, from `MAX_ARC_SAGITTA_UM = 0.005`, floor `DEFAULT_BEND_SAMPLES_PER_90_DEG = 16`) now drives `append_circular_bend_centerline` (primitive bends, was a fixed 16), `append_arc_samples` (endpoint-correction offset jogs, was a fixed **4** per arc = 22.5-degree chords) and `append_quarter_arc_local` in `src/meander.rs` (meander connectors and U-turns, was a fixed **8** = 11.25-degree chords). At r = 10 um that is 25 samples per quarter turn (3.6 degrees). Unit test `arc_samples_keep_every_chord_within_the_sagitta_tolerance`; `cargo test --lib` 411/411. The `heater_s_mod` 90-degree GDS: max vertex turn inside arcs 22.5 -> 5.3 degrees (the remaining 5-degree kinks are straight-to-arc junctions in endpoint-corrected tails, pre-existing, see Surprises); rendered U-turn is a clean curve. Side effect fixed in the same milestone: finely sampled long meanders exceeded the GDSII XY record length (KLayout: "Record length larger than 0x8000"); `translation/gds_write_options.py` now caps `gds2_max_vertex_count` at 4000 for both GDS writers (`routing_flow_reporting.py`, `translation/route_rust_debug_artifacts.py`) so KLayout splits such polygons (heater GDS: 81 -> 86 polygons, max 2769 vertices, reads warning-free). GDS size for that benchmark 160 KB -> 850 KB.
- [x] (2026-08-28) Milestone 1 validation: `heater_s_mod` 45-degree PLM+electrical 81/81 (17 s, GDS 727 KB), `benes_8x8` stable 48/48 (45 s, 230 KB), `multiportmmi_8x8` stable 111/111 (53 s, 479 KB), `benes_16x16` stable 128/128 (254 s, 737 KB) -- all `error_count=0, warning_count=0`. `pytest`: the known 11 plus two tests that pinned the old sampling, both updated: `test_mmi_heater_pass0_characterizes_current_port_alignment` pinned realized lengths to 1e-4 and they moved by 7.4 nm (140.4033 -> 140.4149 um at 1 nm, closer to the true arc); `test_run_routing_flow_can_append_electrical_routing` stubbed `write_gds` with a positional-only lambda that rejected the new `save_options` keyword. `Verdict: PASS`. Uncommitted.
- [x] (2026-08-28) Milestone 1b, owner direction ("I really don't care about the GDS file size"): `MAX_ARC_SAGITTA_UM` 0.005 -> 0.001, i.e. 56 samples per 90 degrees at r = 10 um, the density gdsfactory's `bend_circular` uses (measured: ~62 at r = 10, one vertex per 20 nm of arc). Heater GDS 1.3 MB. Performance: routing untouched; realization 0.03 -> 0.19 s; but `verify_photonic_routing`'s self-intersection check (`_polyline_self_intersects_um`, `translation/photonic_verification.py`) was an O(n^2) shapely pair loop and went from ~8 s to ~15 s on `heater_s_mod` with the denser centerlines (found by profiling the untimed remainder of the flow). Replaced its inner loop with a vectorized bounding-box prefilter, exact shapely check only on candidate pairs, same semantics: `heater_s_mod` stable config total 15.7 s (old sampling) -> 22.8 s (1 nm) -> **7.8 s** (1 nm + prefilter). `tests/test_photonic_verification.py` 22/22.
- [x] (2026-08-28) Milestone 2, investigation only (per the owner's instruction): root cause of the unmatched lengths found and quantified -- see Surprises. **Fix not applied; owner decision pending** (Decision Log).

## Surprises & Discoveries

- Observation: the PLM matches on realized polyline length, not grid length. `RoutedNetRecord.total_length_um` is the length of the sampled `corrected_centerline_um` (81/81 records equal to 1e-12); the grid length (`base_total_length_um`, e.g. 208 vs 205.69) is not what the group arrival uses. So the grid primitive's `length_um` for a bend being the two legs (2r, not pi*r/2 -- `make_turn`, `src/primitives.rs`) only affects A* costs, not matching.
- Observation: nothing measures the realized meander. `physical_inserted_extra_length_um` in the meander report is the planner's own analytic `inserted_extra_length_um` copied through (`translation/route_rust_meanders.py` ~2850), the records passed to realization (with `meander_auto_plan`) are a local of `_realize_and_assemble_debug_artifacts`, and `debug_artifacts.routed_net_records` are the pre-meander records. The measurement below had to capture the realization call to get at the spliced centerlines.
- **Observation: the fill-box multi-bump length model under-books every meander by exactly (amplitude - r).** Measured on the `heater_s_mod` 90-degree / 0 um run by summing the planned `selected_meander_centerline` minus the straight run it replaces: 1 U-turn, A = 68.725, r = 10: realized extra 120.17 um vs booked 61.56 (+58.62 = A - r - 0.1 chord loss); 25 U-turns: +63.9 (A = 75.2); 100 quarter turns: +237.8. Derivation from the geometry `plan_fill_box_multi_bump_meander` actually builds (entry straight r, entry quarter arc, (u+1) vertical legs of length A - 2r, u U-turns of two quarter arcs, exit quarter arc, replaced baseline r(2u+3)):
  true extra(u, A) = (u + 1) * (A - 4r + pi*r)
  booked (`MeanderTurnModel::inserted_extra_length_um` = u*A + `fixed_extra_length_term_um` = r*(2u+2)*pi/2 - r*(4u+3)):
  booked(u, A) = u*A + (u+1)*pi*r - (4u+3)*r
  true - booked = A - r, for every u. The planner inverts the booked formula to choose A from the requested extra (`plan_fill_box_multi_bump_footprint`), so every meander it places is physically (A - r) longer than the group needs. Group-level check on the same run: every group whose meander stays on its own edge shows a realized arrival spread of exactly that amount (58.616 um for the four 1-U-turn groups, 63.9 / 66.0 / 237.8 for the larger ones). Chord shortening from the old 8-sample arcs is only 0.1-2.5 um on top and has the opposite sign.
  Evidence: scratch scripts `capture.py`/`confirm.py` (this session), plan keys `selected_meander_centerline`, `selected_run_start_index`, `selected_run_end_index`; meander report `u_turns`, `quarter_turns`, `effective_bend_radius_um`; the planner's `insertion_width_um = r(4*visual_bumps + 1) = r(2u+3)` already matches the geometry, so it is the length term that is wrong, not the footprint.
- Observation: a per-edge comparison is not a group comparison. The planner may move a requirement's extra length to an upstream edge of the same path (the `auto-meander-move` Codex run of 2026-08-20); in the same table those show as "booked on X, realized on Y" and are not evidence of anything.
- Observation: after resampling, the largest remaining vertex turns (5.29 degrees, 50 vertices) are where a 10.6 um straight meets the first 0.65 um chord of an arc in endpoint-corrected tails -- a true tangent discontinuity of ~3.5 degrees in the corrected centerline, not a sampling artifact. Pre-existing; not pursued here.
- Observation: the GDSII limit that matters is the 16-bit record length (4095 points of 8 bytes), not the 8191-point convention; KLayout's default split at 8000 does not protect against it.

## Decision Log

- Decision: sampling is sagitta-driven (1 nm after the owner's direction; first landed at 5 nm) with the old 16/90-degree as the floor, shared by all three arc builders, rather than raising three separate constants.
  Rationale: one rule, one place, correct for every radius; 5 nm is a few database units, invisible in the GDS, and keeps polyline length within ~1e-5 of the true arc so the analytic length model and the geometry agree on arcs.
  Date/Author: 2026-08-28, Claude.
- Decision: cap GDS polygons at 4000 vertices at write time (KLayout splits) instead of sampling more coarsely.
  Rationale: the geometry is right; the file format has a record limit; splitting is the writer's job and the split pieces are the same shape.
  Date/Author: 2026-08-28, Claude.
- Decision (pending, repository owner): fix the meander length model. Proposed change, `src/meander.rs`: make `MeanderTurnModel::inserted_extra_length_um(r, A)` return `(u_turns + 1) * (A - 4r + pi*r)` (equivalently: fix `fixed_extra_length_term_um` and the leg term together), and invert accordingly in `plan_fill_box_multi_bump_footprint`: `A = requested / (u_turns + 1) + 4r - pi*r`. Add a Rust test that builds a plan and asserts the planned centerline's polyline length minus its endpoint distance equals `requested` within the chord tolerance, and a Python test on `heater_s_mod` asserting the realized group arrival spread (computed from the spliced centerlines, as in this investigation) is below 1 um. Every meander then gets shorter by (A - r) at the same footprint width; requirements that only fit today because of the over-length may need one more bump, and the minimum insertable extra length (`minimum_four_bend_extra_length_um`, currently 13.83 um at r = 10) changes to `2*(pi - 4)*r + 2*A_min ...` -- to be re-derived when implementing. Not started.

## Outcomes & Retrospective

(Milestone 1 pending its ladder; Milestone 2 delivered as an investigation.)

## Context and Orientation

Routes are searched on a grid and then *realized*: `route_to_primitive_centerline` (`src/geometry_realization.rs`) replays each primitive as a physical polyline (straights, sampled circular bends), endpoint correction may replace the tails, path-length matching may splice a meander centerline into a straight run (`splice_meander_into_centerline_range`), and `generate_waveguide_polygon` extrudes the polyline into the GDS polygon with miter joins. The path-length matcher (`translation/route_rust_meanders.py`, Rust `src/meander.rs`/`src/auto_meander.rs`) computes, per matching group, how much extra length each incoming edge needs and asks the planner for a "fill-box multi-bump" meander of that extra length inside an available box; the planner returns both the centerline and the extra length it believes that centerline adds.

## Plan of Work

Milestone 1 (done): `arc_samples_per_90_deg` and its three call sites; `translation/gds_write_options.py`.

Milestone 2 (if the owner says go): the formula and inversion in `src/meander.rs` as in the Decision Log, tests as described, then the PLM-affected ladder (`heater_s_mod` 45/90-degree, `mmi_heater_8x4` PLM, perf-smoke PLM benchmarks) and a re-run of this investigation's group-spread measurement as the acceptance check.

## Concrete Steps

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$(pwd)/.venv/bin/python3" .venv/bin/maturin develop --release
    cargo test --lib 2>&1 | tail -1
    .venv/bin/python routing_flow.py heater_s_mod --allow-45-degree-turns false --bend-radius-um 10 \
      --path-length-matching true --path-length-match-outputs true --include-heater-obstacles true
    # then read build/routed_heater_s_mod.gds with klayout.db: no "Record length" warnings, arcs sampled

## Validation and Acceptance

Milestone 1: `cargo test --lib` green with the new test; the heater GDS reads without KLayout warnings; max vertex turn inside sampled arcs <= 90/25 degrees plus the pre-existing tail kinks; the realization-affected ladder verifies `error_count=0` everywhere. Milestone 2 (when done): realized group arrival spread < 1 um on `heater_s_mod`.

## Idempotence and Recovery

Plain source edits; `git checkout` restores. GDS files under `build/` are regenerated by every run.

## Artifacts and Notes

- Rendered U-turn after resampling: scratch `bump.png` (this session).
- Group spread table before the Milestone 2 fix: scratch `confirm.py` output (this session), summarized in Surprises.

## Interfaces and Dependencies

`arc_samples_per_90_deg` is `pub` in `geometry_realization`; `meander.rs` calls it. `translation/gds_write_options.gds_save_options()` is imported by both GDS writers; it is a leaf module (only `klayout.db`) to avoid the `routing_flow_reporting` <-> `translation` import cycle.
