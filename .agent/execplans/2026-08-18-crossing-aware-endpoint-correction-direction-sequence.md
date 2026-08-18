# Fix crossing-aware endpoint correction rejecting a valid port-exit correction

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository (`TUMPhotonicRouter`) routes optical waveguides on a grid, then runs a "geometry realization" pass that turns the grid-based route into exact physical coordinates, and finally corrects the very start and end of each route so its centerline lands exactly on the real, physical port location it is supposed to connect to (a port rarely sits exactly on a router grid point, so this correction is normal and expected, not a sign anything is wrong). For most nets this works perfectly. For a net whose route crosses another route, a separate, more careful version of this correction runs instead, because that version must avoid disturbing the route geometry right around the crossing point while still being free to fix the two ends. This plan fixes a bug in that separate, crossing-aware version: for three real nets in the `multiportmmi_8x8` benchmark (`n_40`, `n_41`, `n_42`), it rejects an exact, correct fix that was already computed, and falls back to a worse approximation that ends up `2.0014` micrometers off the true port location, tripping a hard verification error (`source_endpoint_mismatch`) that stops the benchmark from finishing.

After this plan is complete, the correction is no longer rejected, and all three nets' corrected centerlines start exactly on their real source ports (confirmed to floating-point precision, not just "close enough"). A person can see this working by running `multiportmmi_8x8` end to end (see Concrete Steps) and observing that `routing_flow.py` no longer raises `RuntimeError: Photonic geometry verification failed before GDS write: 3 error(s). source_endpoint_mismatch n_40: ...` -- the whole benchmark, including GDS write, should complete.

## Progress

- [x] (2026-08-18) Root cause fully diagnosed via direct instrumentation of the live pipeline (see Surprises & Discoveries for the exact evidence). Fix designed and hand-verified against both observed failure shapes before writing any code.
- [x] (2026-08-18) Fix applied to `translation/route_rust_endpoint_correction.py` and validated. `multiportmmi_8x8` now completes end to end for the first time, including GDS write, with both `build/verification/*.json` reports fully clean. Full pytest suite unchanged (`21 failed, 316 passed, 1 skipped`, identical failure names to baseline). Plan complete.

## Surprises & Discoveries

- Observation: the failure is not in the "exact correction" logic itself -- an exact, correct answer is already being computed, every time, for all three failing nets. It is being thrown away by a separate compatibility check that has a real logic bug.
  Evidence: instrumenting `_terminal_anchor_matches` (`translation/route_rust_endpoint_correction.py`, called from `_spliced_crossing_endpoint_centerline`) showed that for `n_40`, the rich, whole-route Rust correction (`route_port_corrected_centerline`, `src/py_router.rs:11813`, backed by `route_to_port_corrected_centerline_with_options` in `src/geometry_realization.rs`, which tries several strategies including diagonal corrections and axis-run absorption before giving up) produces a prefix centerline `((1717.7, 550.0), (1728.375, 550.0), (1736.375, 550.0), (1741.5, 549.125))` whose first point is `(1717.7, 550.0)` -- exactly the real port center, `_terminal_anchor_matches` confirms `True` -- and whose last point, `(1741.5, 549.125)`, exactly matches the fixed boundary point the crossing-aware logic requires it to reconnect at. This is a fully correct answer with zero residual error.
- Observation: this exact answer is rejected by `_compatible_terminal_direction_sequence` (`translation/route_rust_endpoint_correction.py`, line 621), which has a real bug: it assumes the single new segment a correction inserts (beyond whatever segments the uncorrected baseline already had) always appears at a fixed position in the sequence -- index `0` when `allow_extra_at_start=True` (the value used for source-port prefix corrections). For `n_40`, the baseline's own direction sequence is `((1.0, 0.0),)` (one straight horizontal run) and the correct candidate's direction sequence is `((1.0, 0.0), (0.9857364255104072, -0.16829646289202074))` -- the *first* direction already matches the baseline's own single direction (both are the port's own facing direction, which is the common, expected case: routes normally already depart a port correctly), and the genuinely new segment -- the short jog that absorbs the residual offset before reconnecting -- is the *second* one, at index `-1`, not index `0`. The function only ever checks index `0` for `allow_extra_at_start=True`, so it looks at the wrong position, finds the reconnection-jog direction where it expected the baseline's original direction, and rejects a fully valid correction.
  Evidence: direct instrumentation dump for all three failing nets: `n_40` candidate `((1.0, 0.0), (0.9857..., -0.1683...))` vs baseline `((1.0, 0.0),)`; `n_41` candidate `((1.0, 0.0), (0.9920..., -0.1263...))` vs baseline `((1.0, 0.0),)`; `n_42` candidate `((1.0, 0.0), (0.0, -1.0))` vs baseline `((1.0, 0.0),)`. All three show the identical shape: the extra segment is the *last* one, not the first, even though `allow_extra_at_start=True` is passed for all of them (they are all source-side prefix corrections).
  This traces the failure fully: `_compatible_terminal_direction_sequence` returns `False` -> `_spliced_crossing_endpoint_centerline` (line ~786) falls back to `_absorbed_terminal_centerline`, a separate, weaker Python-only solver limited to the small "guard window" immediately around the crossing point -> that solver also fails, because for these three nets the crossing point sits deep inside a single long straight run with no bend anywhere near it (the real first bend is roughly 35 micrometers further along, well outside the guard window) -> `_spliced_crossing_endpoint_centerline` falls back a second time, to the *uncorrected* baseline prefix -> the final corrected centerline starts `2.0014` micrometers away from the real port, tripping the `source_endpoint_mismatch` verification error.

## Decision Log

- Decision: implement this fix directly, not through `.agent/scripts/codex_task.sh`, even though the fix is small (roughly 15 changed lines in one function) and would otherwise be a clean fit for Codex per `.agent/CLAUDE_CODEX_FLOW.md`'s normal guidance.
  Rationale: this repository's own process docs (`.agent/CLAUDE_CODEX_FLOW.md`, "The diagnosis/implementation boundary") require an explicit choice at this point, not a silent default -- this decision entry is that choice, made under exception 3 listed there ("the fix is small but its correctness depends on interactive intermediate output from the diagnosis in a way that would not survive being restated as a static prompt"). The correct fix depends on a subtle geometric fact -- which *position* in a direction sequence the newly inserted correction segment lands at, which depends on whether the baseline's own port-adjacent segment happens to already match the port's facing direction -- that was only established by directly instrumenting the live pipeline and reading real numbers from three real nets, then hand-simulating the proposed replacement function against those exact numbers before writing it. Restating this as a static Codex prompt would either have to include all of that derivation (at which point Codex is transcribing an already-solved answer, not solving anything) or risk a fresh agent re-deriving the same kind of position-indexing logic incorrectly, which is exactly the class of bug this plan exists to fix -- getting it wrong a second time in a different way is a real, non-hypothetical risk given how easy the original bug was to introduce.
  Date/Author: 2026-08-18, Claude.

## Outcomes & Retrospective

Complete. A roughly 15-line change to one function's body (`_compatible_terminal_direction_sequence`, `translation/route_rust_endpoint_correction.py`) fixed a bug that was silently discarding an already-correct answer and falling back, twice, to progressively worse approximations -- first a guard-limited Python solver with no bend available to work with, then the raw uncorrected baseline. The fix did not need to make anything smarter; it needed to stop throwing away a correct result that was already being computed. This is worth remembering alongside this repository's other recent lesson about not inventing bolt-on mechanisms (`.agent/CLAUDE_CODEX_FLOW.md`): here, the existing "rich Rust correction, sliced to the crossing-safe region" mechanism was already doing exactly the right thing, and the only bug was a single overly-narrow compatibility check standing in its way. Finding that required tracing the actual call chain with real instrumented values rather than assuming which of the several fallback layers was actually responsible.

`multiportmmi_8x8` now completes cleanly end to end -- all 111 nets routed, zero crossing-verification issues, zero photonic-verification issues, GDS written -- closing out the last known blocker for that benchmark from this session's work (following the unify-port-access-region and dense-port-runway-clearance-reach plans). `multiportmmi_16x16` was not re-checked in this plan; its own `n_102` crossing-legality finding (recorded in `.agent/execplans/2026-08-18-dense-port-runway-clearance-reach.md`) is unrelated to this bug and remains open.

## Context and Orientation

This section assumes no knowledge of any prior conversation or prior ExecPlan. Every fact needed to understand and execute this plan is repeated here.

### Endpoint correction, in general

After the router finds a path on its internal grid, a separate "geometry realization" step (`src/geometry_realization.rs`, `translation/route_rust_geometry.py`) turns that grid path into a smooth, physically exact sequence of `(x, y)` points in micrometers -- a "centerline." The very first and very last points of that centerline are then adjusted so they land exactly on the real port each end of the net is supposed to connect to; a port's physical center is an arbitrary micrometer coordinate set by the chip's component layout, not something constrained to fall on a router grid point, so some small adjustment right at each end is normal and expected. This adjustment is called "endpoint correction," and it is verified afterward: `translation/photonic_verification.py`, function `_verify_one_port_connection` (around line 424), checks that the corrected centerline's first point (for the source role) or last point (for the target role) is within `port_contact_radius_um` (a tolerance, `2.0` micrometers for this benchmark) of the port's real center; if not, it records a `source_endpoint_mismatch` or `target_endpoint_mismatch` issue, which `routing_flow.py` treats as a hard error that stops the run before writing the final GDS chip-layout file.

### Two different correction code paths

There are two separate implementations of this correction, used in different situations:

1. For a net whose route does not cross any other route, `translation/route_rust_records.py`'s `apply_port_endpoint_corrections` calls a Rust method, `route_port_corrected_centerline` (`src/py_router.rs`, line 11813), which is backed by `route_to_port_corrected_centerline_with_options` (`src/geometry_realization.rs`, line 1094). This Rust function tries several strategies in order (an exact straight-line correction, an exact diagonal correction, a shared-axis shift, a 45-degree delta correction, absorbing the needed change into an existing straight run, and a couple of further fallbacks) until one succeeds, and is generally reliable.

2. For a net whose route crosses another route, a separate, Python-side mechanism in `translation/route_rust_endpoint_correction.py` runs instead, because touching the route geometry too close to a crossing point could invalidate the crossing itself (this repository's `.agent/WORKFLOW.md` states this constraint explicitly: "for crossed nets it may only modify the source-to-first-crossing and last-crossing-to-target terminal regions"). The relevant function is `_spliced_crossing_endpoint_centerline` (line 664). It first calls the *same* rich Rust correction from path 1 above, applied to the whole route (temporarily ignoring the crossing), then slices out just the portion from the route's start up to the crossing point (`_corrected_prefix_to_crossing`) -- the idea being that the rich Rust correction's own answer, restricted to the region this crossing-aware path is allowed to touch, is usually the best available answer, and this plan's diagnosis confirms that idea is correct for the cases investigated. Before accepting that sliced prefix, two checks must pass: `_terminal_anchor_matches` (does it actually start exactly at the port?) and `_compatible_terminal_direction_sequence` (does its sequence of straight-line directions look like a legitimate, minimal correction of the uncorrected baseline's own direction sequence, rather than something that wandered unexpectedly?). If either check fails, the code falls back to a second, weaker, Python-only solver, `_absorbed_terminal_centerline` (line 439), which is only given a small "guard window" of centerline near the crossing point (at most 4 micrometers back from the crossing, via `_guarded_cut`, line 695) to work with, specifically to avoid disturbing anything further away. If that solver also fails (returns an empty tuple), the code falls back a second time, silently, to the *uncorrected* baseline centerline for that whole prefix or suffix region (lines 786-787 and 807-808) -- which is not guaranteed to be anywhere near the real port, and is exactly what happens for the three nets this plan fixes.

### The exact bug

`_compatible_terminal_direction_sequence` (line 621) takes a `candidate` centerline (the rich correction's sliced prefix or suffix) and a `baseline` centerline (the uncorrected version of that same region), reduces each to its sequence of unit-vector directions (one entry per straight run, via `_segment_direction_sequence`, line 299, which uses `_compress_centerline` to merge consecutive collinear points into one run first), and checks whether `candidate`'s direction sequence is exactly `baseline`'s sequence with exactly one extra direction inserted -- reasonable in principle, since a minimal correction should not reshape the whole path, just add one small jog. The bug is in *where* it assumes that one extra direction is: when `allow_extra_at_start` is `True` (the value passed for a source-port prefix correction, since the port is at the very start of that region), the function unconditionally treats `candidate_dirs[0]` (the very first direction) as the new one, and requires everything after it (`candidate_dirs[1:]`) to exactly match `baseline_dirs` unchanged. It never considers the opposite arrangement -- that `candidate_dirs[0]` might already equal `baseline_dirs[0]` (both being the port's own facing direction, since a route almost always already departs a port in the port's own facing direction, correction or not) and the genuinely new segment is actually the *last* one, appended just before reconnecting to the fixed boundary point. When that is what actually happened -- confirmed directly for all three failing nets -- the function looks at the wrong position, compares the reconnection jog's direction (which has nothing to do with the baseline) against what it expected to be an unchanged copy of the baseline, finds a mismatch, and incorrectly rejects an exact, correct answer.

The symmetric branch (`allow_extra_at_start=False`, used for target-port suffix corrections) has the mirror-image version of the same unconditional assumption (always checking `candidate_dirs[-1]` as the new one), and while no currently-failing net exercises that exact branch, the same underlying reasoning applies to it and the fix below covers both directions uniformly rather than only patching the one branch with an observed failure.

## Plan of Work

In `translation/route_rust_endpoint_correction.py`, replace the body of `_compatible_terminal_direction_sequence` (currently lines 621-647) so that:

1. It still requires `len(candidate_dirs) == len(baseline_dirs) + 1` (exactly one new segment; anything else is rejected exactly as before) and still requires `expected_port_dir` to be provided (unchanged).
2. It still enforces the real physical constraint that the segment immediately adjacent to the port -- `candidate_dirs[0]` when `allow_extra_at_start` is `True` (source prefix: the port is at the start), or `candidate_dirs[-1]` when it is `False` (target suffix: the port is at the end) -- matches `expected_port_dir`. This is unconditional and does not depend on which segment turns out to be the "new" one; a route must depart or arrive at a port along the port's own facing direction regardless of where the correction's extra jog ends up.
3. Instead of assuming the extra segment is always at a fixed position, it checks *both* possible placements -- `candidate_dirs[1:]` matching `baseline_dirs` exactly (extra segment first), or `candidate_dirs[:-1]` matching `baseline_dirs` exactly (extra segment last) -- using the same per-element `_same_direction` comparison the original code already used (not raw tuple equality, since directions are floating-point unit vectors), and accepts the candidate if *either* placement matches. Both placements are geometrically and physically legitimate ways for a minimal correction to look; which one actually occurs depends on whether the baseline's own port-adjacent segment already happened to match the port's facing direction, which the router's model makes the common case.

The rest of the file (`_absorbed_terminal_centerline`, `_spliced_crossing_endpoint_centerline`, the guard-window logic, everything upstream and downstream of this one function) is untouched -- the bug is isolated entirely to this one function's placement assumption, and every other piece of the crossing-aware correction pipeline already does the right thing once this function stops rejecting a correct answer.

## Concrete Steps

All commands below are run from the repository root, `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`, using `.venv/bin/python`, with `PYTHONPATH=.` set for direct module imports.

Reproduce the failure (before the fix):

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8

This fails with `RuntimeError: Photonic geometry verification failed before GDS write: 3 error(s). source_endpoint_mismatch n_40: ...; source_endpoint_mismatch n_41: ...; source_endpoint_mismatch n_42: ...` before this plan's fix.

No Rust rebuild is needed (this is a pure Python change in `translation/route_rust_endpoint_correction.py`).

Full Python test suite, before and after:

    PYTHONPATH=. .venv/bin/pytest -q

## Validation and Acceptance

Accepted when: the same `multiportmmi_8x8` run above completes without the `source_endpoint_mismatch` error (ideally completing the full benchmark through GDS write, though a different, unrelated failure further in the pipeline would not itself indicate this plan's fix is wrong -- read whatever new failure appears, if any, on its own merits rather than assuming success or failure of this fix from it alone); `build/verification/multiportmmi_8x8_photonic_verification.json`, read directly (not inferred from console output), shows no `source_endpoint_mismatch`/`target_endpoint_mismatch` issues; and the full pytest suite shows no new failures beyond whatever this fix intentionally changes.

Verdict: PASS. Evidence: `build/verification/multiportmmi_8x8_crossing_verification.json` and `build/verification/multiportmmi_8x8_photonic_verification.json`, both read directly -- `status=complete`, `success=true`, `error_count=0`, `warning_count=0`, `issues=0`, `routed_record_count=111`, `expected_route_count=111`, `partial=false` for both. No `source_endpoint_mismatch` or any other issue remains; this is a fully clean run, not merely "the three known nets are fixed." The `multiportmmi_8x8` CLI run completed through `Write GDS...` with no exception, for the first time in this repository's history at this code state. `PYTHONPATH=. .venv/bin/pytest -q`: `21 failed, 316 passed, 1 skipped`, identical failure names to the pre-fix baseline -- no regressions.

## Idempotence and Recovery

All commands in Concrete Steps are read-only with respect to source files, or write only to `build/` (untracked, gitignored) -- safe to re-run at any time. This is a pure Python change requiring no extension rebuild; if the replacement has a bug, `PYTHONPATH=. .venv/bin/pytest -q` and the `multiportmmi_8x8` reproduction above will surface it immediately, since `_compatible_terminal_direction_sequence` and its callers are exercised by both.

## Artifacts and Notes

Key evidence from this plan's investigation (via disposable Python scripts that monkeypatch functions in `translation/route_rust_endpoint_correction.py` and `translation/photonic_verification.py` to dump real intermediate values from a live `multiportmmi_8x8` run):

    Verification error (before fix), from build/verification/multiportmmi_8x8_photonic_verification.json:
      source_endpoint_mismatch, net n_40, distance_um=2.0014057559625025, tolerance_um=2.0
      (n_41 and n_42: identical shape, different net names)

    For n_40's source-side correction:
      real port center: (1717.7, 550.0)
      rich Rust correction's own prefix (correct, rejected anyway):
        ((1717.7, 550.0), (1728.375, 550.0), (1736.375, 550.0), (1741.5, 549.125))
      _terminal_anchor_matches(prefix, port_center, at_start=True) -> True
      candidate direction sequence: ((1.0, 0.0), (0.9857364255104072, -0.16829646289202074))
      baseline direction sequence:  ((1.0, 0.0),)
      _compatible_terminal_direction_sequence(..., allow_extra_at_start=True) -> False (the bug)
      Final (wrong) corrected centerline after both fallbacks: starts at (1719.5, 549.125),
      2.0014057559625025 um from the real port center (1717.7, 550.0).

    Same shape confirmed for n_41 and n_42, each with a different-looking "extra" jog
    direction (n_41: (0.9919979117236188, -0.12625427967391514); n_42: (0.0, -1.0)),
    but in every case the extra segment is the *last* one in the candidate direction
    sequence, not the first, contradicting what allow_extra_at_start=True currently checks.

## Interfaces and Dependencies

One existing private function's body changes: `_compatible_terminal_direction_sequence` in `translation/route_rust_endpoint_correction.py`. Its signature (parameter names, types, return type `bool`) is unchanged, so every existing call site (both in `_absorbed_terminal_centerline`'s two call sites via `_spliced_crossing_endpoint_centerline`, lines 774 and 795) continues to work without modification. No other function, file, or public interface is touched.
