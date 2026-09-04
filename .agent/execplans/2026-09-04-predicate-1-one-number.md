# Predicate 1: one number (`half_size`) and real straight cells

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`.

## Purpose / Big Picture

The crossing ruleset (owner, 2026-09-03): the GDS crossing gives `half_size`; inside the +-half_size window only straight cells may lie, on both nets; missing straights are inserted eagerly; windows are disjoint (predicate 2, done). The kernel still carries `required_margin = half_size + bend_runout` (= 5) as the straight requirement BEFORE a crossing and for the PARTNER's straight -- a number that silently compensates "a bend arm counted as straight" (exact for 90-degree arms = arc, ~1-2 cells over-strict at 45-degree corners), caps the Tier-2 state key (`straight_run_cells`, `pending` in 0..5) and is duplicated in the post-search grid check. The 2026-09-03 rule audit and performance pass both point here: the same number is three rules, and the 0..5 key multiplicity is a large share of the ~2.3x re-expansion per cell.

After this plan: every straight requirement is `half_size` on REAL straight cells -- the kernel counts a turn primitive's terminal arm as `arm - trim` (trim = `arm * tan(theta/2)`, rounded up: 90 degrees -> the whole arm, 45 degrees -> 2 of 3 cells), partner polyline margins subtract the trim at corner ends, the post-search grid check applies the same arithmetic, the Tier-2 key caps `straight_run_cells`/`pending` at `half_size`, the pre-hook "arm pays" gate is gone, `required_margin`/`bend_runout_cells` exist only as the explicit repair-keepout radius. Unit tests pin each rule with 90-degree and 45-degree geometry, H/V and diagonal; the ladder and both 32x32 stay verification-clean; timing is re-measured (expected: fewer expansions).

Owner decisions taken: 2026-09-03 "Punkt 2" (debt = half_size after the crossing), "mach das so" for the trim formulation, 2026-09-04 "mach Prädikat 1 als nächsten Meilenstein". Open (collected, not blocking): whether `min_straight_cells_per_crossing` (CLI/API knob, ignored since its introduction) becomes the `half_size` override or is deleted -- left untouched here.

## Progress

- [x] S1 (`85944aa`) kernel: real straight run after turn primitives (`primitive_terminal_straight_run_cells` minus trim); route-before requirement = `half_size`. Tests: crossing 1/2 cells after a 90-degree turn (reject/accept), 1 cell after a 45-degree turn (accept: 1 real + 1), H/V and diagonal.
- [x] S2 (`6d9d613`) kernel: partner margin = polyline margin minus corner trim (bend radius from the crossing config), requirement `half_size`. Tests: partner 90-degree corner at 4/5 cells (reject/accept), 45-degree corner at 3/4 cells (reject/accept).
- [x] S3 (`a68a7fd`) kernel: key cap `half_size` (straight_run, pending), pre-hook arm-pays gate removed, `required_margin`/`capped_required_margin` plumbing replaced by `half_size`; traces print `half_size`.
- [x] S4 (`5b92b30`) post-search: `invalid_crossing_intersections_for_route` uses the same trims and `half_size` on both sides; repair keepout radius stays `half_size + bend_radius` explicitly.
- [ ] S5 validation: cargo suite, pytest baseline, ladder, both 32x32 full (verification-clean, attempts/failures/repairs recorded), timing table vs the 2026-09-03 final guard.

## Surprises & Discoveries

- (2026-09-04, S1) The Tier-1/Tier-2 invariant "default extension <=> stored densely" broke: after a 90-degree turn a Tier-2 node now carries 0 real straight cells (default extension) and hit the Tier-1 fast path's `unreachable!`. Fix: the fast path additionally requires a dense parent; Tier-2 nodes with a default extension stay on the Tier-2 path (as they did before with straight_run 3). No semantic change.
- (2026-09-04, S3) `vertical_descent_refuses_two_crossings_four_cells_apart` was pinned to the OLD over-strict rule: with half_size real straight cells the search finds the legal alternative -- crossing partner A at x=30 and partner B at x=35, a lateral jog of 5 cells, so both +-2 windows are disjoint. Re-pinned as `vertical_descent_four_cells_apart_jogs_so_the_windows_are_disjoint` (route exists, crossings >= 5 cells apart laterally, same-x attempts still refused). The 3-cell case stays unroutable (no room for the jog).
- (2026-09-04, S4) Yesterday's grid-check test assumed 3 grid cells before a 45-degree corner suffice; with the fillet trim (1.24 cells at radius 3) that is 1.76 real cells -- the realized validator's verdict too. Re-pinned: 45-degree corner at 4/3 cells (pass/fail), 90-degree at 5/4, terminal 2/1. Kernel routes always satisfy this (debt half_size + a full turn arm before any corner).
- Not changed (by design): `crossing_required_margin_cells` (= half_size + bend radius) survives only as a DISTANCE for repair keepout radius, partner lookup bbox expansion, candidate-key/spacing-history heuristics -- never as a straight requirement. `min_straight_cells_per_crossing` untouched (open question for the owner).

## Decision Log

- (2026-09-04) Trim of a turn arm in cells: `ceil(arm_cells * tan(theta/2) - 1e-9)` with theta = 45 or 90 degrees; for make_turn primitives arm = bend radius, so 90 degrees -> arm (0 real cells), 45 degrees with radius 3 -> 2 (1 real cell). Rounding up keeps the kernel conservative against the realized validator (which measures the exact fillet).

## Outcomes & Retrospective

Not started.

## Validation and Acceptance

- Every test names its geometry; the realized validator remains the authority (`realized_violations=[]` on the benchmark runs).
- Ladder + multiportmmi_32x32 + benes_32x32 verification-clean; timing table recorded.
