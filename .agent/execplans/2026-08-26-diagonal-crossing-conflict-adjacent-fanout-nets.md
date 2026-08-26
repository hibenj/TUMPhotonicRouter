# Fix unnecessary orthogonal detours between adjacent-target diagonal nets

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md` (the standing policy for all ExecPlans in this repository).

## Purpose / Big Picture

`multiportmmi_16x16` (one of this repository's LiDAR routing benchmarks, defined in `benchmarks/multiportmmi_16x16.py`) has a known, separate, already-understood failure at net `n_49` (documented in `.agent/REPOSITORY_STATE.md`'s Current Findings entry 2) -- that failure is explicitly OUT OF SCOPE for this plan and must not be conflated with it. This plan is about a different, earlier problem: nets that route BEFORE `n_49` ever gets attempted are taking a visibly worse path than they should. Specifically, in the region where 16 "Y-branch" fan-out nets connect to 16 Mach-Zehnder interferometer ("MZI") array elements (nets `n_15` through `n_30`, each net named `n_X` where X is a 0-based index baked into the benchmark's netlist), adjacent nets going to adjacent, nearly-identical targets end up forced into a blocky, right-angle "comb" detour instead of a shorter diagonal path -- even though this router's A* search is fully capable of diagonal (45-degree) moves and even though nothing in the configuration disables them. The repository owner visually inspected this (rendered GDS screenshots) and judged, correctly on inspection, that "there should be a better routing solution here."

After this plan's investigation phase (see Progress and Surprises & Discoveries below), someone picking this up next will be able to: (1) reproduce the exact bad-geometry case with a single command that stops routing right after the interesting nets, (2) see the precise, already-diagnosed root cause with supporting evidence (real illegal-crossing violations between two near-parallel diagonal candidate paths), and (3) either continue root-causing why a tested workaround (`--proactive-congestion-weight`) correlates with better geometry, or implement a more targeted fix. The plan is not yet closed: the root cause of the *forced detour* is confirmed, but the mechanism behind the workaround that improves it has not been proven, and no code fix has been written yet.

## Progress

- [x] (2026-08-26) Reproduced the bad geometry on a partial (`--debug-stop-after-route`-style) run of `multiportmmi_16x16`, confirmed via direct GDS/vector-SVG inspection with the repository owner that nets `n_15`/`n_16` (adjacent Y-branch-to-MZI connections) end up as an orthogonal "comb" detour instead of the diagonal path either net could take in isolation.
- [x] (2026-08-26) Root-caused the forced detour: `n_16`'s diagonal search candidate is rejected by the router's own crossing-legality check against `n_15`'s already-committed route, with reasons `not_perpendicular` and `collinear_route_overlap` (see Surprises & Discoveries for the exact trace). This is a real, correct rejection, not a bug -- two nets whose diagonal shortcuts run nearly parallel toward adjacent targets cannot legally share that diagonal corridor in this router's crossing model (crossings must be perpendicular; two waveguides cannot overlap/run collinear).
- [x] (2026-08-26) Confirmed (with the repository owner) that the router's `lidar-pure` crossing-discovery model works exactly as intended/documented: crossing legality is discovered from actual per-cell collisions during search, not from a pre-planned partner whitelist. This was re-verified directly against `.agent/execplans/2026-08-19-restructure-crossing-partner-discovery.md`'s own Purpose section and the code comment on `lidar_pure_owner_lookup_partner_set` (`src/py_router.rs`) -- both state the collision-discovery model explicitly. An earlier, imprecise description of "required partner nets" during this same investigation was corrected; see Decision Log.
- [x] (2026-08-26) Landed a *safe but ultimately unrelated* improvement while investigating: `route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout` (`src/py_router.rs`) now tries the normal, diagonal-capable repair search before the orthogonal-only fallback (previously the reverse), so `prefer_orthogonal_repair` no longer forces an orthogonal detour when the normal search would already find something legal. Verified empirically NOT to change `n_15`/`n_16`'s outcome (traced precisely; see Surprises & Discoveries) -- kept anyway because it is independently correct and is confirmed safe (see Validation and Acceptance).
- [ ] Not yet done: prove *why* `--proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3` changes `n_15`/`n_16` (and the rest of the `n_15`-`n_30` block) from a mix of orthogonal-forced/repaired nets to all-diagonal, cleanly, with no rip-up cycle. This was observed empirically (see Surprises & Discoveries) but the repository owner explicitly pushed back on attributing it to "congestion" without proof -- the mechanism must be traced through the actual cost function before it is trusted or proposed as a fix.
- [ ] Not yet done: decide on and implement an actual fix (which may or may not be proactive-congestion tuning) for the forced-orthogonal-detour pattern, then validate it does not regress the rest of the benchmark ladder (`multiportmmi_8x8`, `benes_8x8`, `benes_16x16`, and the full `pytest`/`cargo test` suites), matching the validation depth already established earlier this session for the `max_iterations` cap and stub-port-lane changes (see `.agent/REPOSITORY_STATE.md` Resolved Findings entries 3 and 4 for the expected depth of validation).

## Surprises & Discoveries

- Observation: the exact, direct proof that `n_16`'s diagonal candidate is rejected for a real crossing-legality reason, not silently avoided by some other mechanism.
  Evidence (captured via `PHOTONIC_ROUTER_TRACE_CROSSING=1 PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, see Concrete Steps for the exact reproduction command):

      collision-crossing start net=17 partners=[16] block_radius=0 min_straight=2 half_size=2
      native_repair_probe net=17 allowed_partners=1 crossing_events=0 grid_violations=0
        realized_violations=3 realized_reasons=[(16, "not_perpendicular"), (16, "collinear_route_overlap"), (16, "not_perpendicular")]
        keepout_keys=219 candidate_blockers=[16]
      native_repair_keepout net=17 ripup=[16] victim_first=true reverse=false keys=239

  (Net IDs in these Rust-level traces are 1-based against the 0-based net name: `net=17` is net name `n_16`, `net=16` is net name `n_15`, i.e. Rust `net_id` = the integer suffix in the net name, plus 1.) This is the moment `n_16` (net_id 17) attempts a diagonal path that intersects `n_15`'s (net_id 16) already-committed route illegally, gets rejected, and then rips `n_15` up as a victim to retry -- exactly the rip-up-then-orthogonal-fallback behavior the repository owner observed visually.

- Observation: `n_15` and `n_16` are adjacent Y-branch fan-out ports connecting to adjacent MZI array elements (`mol_array_0_mzi_0` and `mol_array_0_mzi_1`), so their unobstructed diagonal shortcuts are nearly identical in direction -- they would need to run alongside (collinear/overlapping), not cross, each other. That is why the crossing-legality check (which requires a clean, perpendicular intersection, and forbids collinear overlap) rejects it, and why this is a genuinely different failure shape than a normal "two nets cross at some point" case this router handles routinely.

- Observation: enabling `--proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3` (an existing mechanism, `src/astar.rs`'s `primitive_lateral_congestion`, already used as-default on 3 other benchmarks -- see `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestone 7) on a partial run of `multiportmmi_16x16` stopped after route 49 changes every net in the `n_15`-`n_30` block from a mix of `has_diagonal=False`/repaired-orthogonal outcomes to 100% `has_diagonal=True`, with no rip-up needed for `n_15`/`n_16` specifically (lengths changed only slightly: `n_15` `634.2um -> 629.1um`, `n_16` `535.5um -> 554.3um`).
  Evidence (`debug_stop_after_route_index=49`, `proactive_congestion_weight=4.0`, `proactive_congestion_radius_cells=3`, full per-net dump in this session's transcript): every net `n_15` through `n_30` reports `has_diagonal=True`; `n_15`/`n_16`/`n_17`/`n_18`/`n_29`/`n_30` still show `bucket=reroute_victims`/`repair_failed_net` (meaning some repair activity still happens), but `n_19`-`n_28` (10 of 16 nets) now resolve cleanly via `bucket=normal_route`/`probe_route` with no repair at all, versus several of those same nets needing repair in the baseline (no-proactive-congestion) run.
  Caveat (repository owner's explicit correction, 2026-08-26): this is a correlation, not a proven mechanism. `--proactive-congestion-weight` is a *lateral* congestion cost (`primitive_lateral_congestion` in `src/astar.rs`, penalizing moves with blocked cells to their sides within a radius) -- it is not obvious why a lateral-spacing cost would resolve a *collinear-overlap-with-a-diagonal-neighbor* problem, and no one has traced the actual cost landscape change step by step yet to confirm it is really the causal mechanism versus an incidental side effect (e.g. perturbing which of several equal-or-near-equal-cost candidate paths A* happens to expand first). Do not present this as "the fix" until that tracing is done.

- Observation: a real risk of function-name confusion in `src/py_router.rs`'s repair code was hit and corrected during this investigation. There are at least three similarly-named functions in the victim/current-net repair call chain: `route_single_net_and_commit_native_with_repair_keepout` (delegates to the plain `route_single_net_and_commit_native`), `route_single_net_and_commit_repair_native_with_repair_keepout` (delegates to `route_single_net_and_commit_repair_native`, a distinct function with its own `history_weight` parameter), and `route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout` (the try-order-flipping wrapper this plan's landed fix touches). An initial diagnostic was accidentally added to the wrong one of these (`route_single_net_and_commit_repair_native`) and printed nothing for the traced net, which looked like a dead end until the call graph was re-read line by line. Any future contributor adding diagnostics here should grep for the *exact* function name at the exact call site, not assume based on nearby line numbers -- multiple call sites with near-identical parameter lists exist within ~30 lines of each other after prior edits shifted line numbers.

## Decision Log

- Decision: correct the description of `lidar-pure` crossing partner discovery from "achieves a crossing with a required partner" to "the crossing search is only invoked as a last resort after a plain, crossing-free search already failed, and its job is then to find a route that legally crosses *something* -- the partner set is a whole-map lookup table for identifying who owns a collided-with cell, not a pre-planned target list."
  Rationale: the repository owner directly challenged the "required partner" framing, citing their own documented mental model in `.agent/execplans/2026-08-19-restructure-crossing-partner-discovery.md` ("A* just routes, and when it collides with another net's cell, it asks 'which net is this and is crossing it legal here'"). Re-reading that execplan and the `lidar_pure_owner_lookup_partner_set` code comment confirmed the owner's model is the documented, intended design, and the original phrasing in this investigation was imprecise (though the underlying trace evidence and conclusion were unaffected).
  Date/Author: 2026-08-26, this investigation.

- Decision: flip the try-order in `route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout` (`src/py_router.rs`) to attempt the normal, diagonal-capable repair search before the orthogonal-only fallback, keeping the orthogonal-only search only as a fallback when the normal search finds no legal route at all.
  Rationale: the pre-existing code tried the orthogonal-only search first whenever `prefer_orthogonal_repair` was set, even though the normal search enforces the exact same crossing-legality checks and could find an equally-legal but shorter/better route. Confirmed safe: `cargo test --lib` 401/401, `pytest` 335 passed with the same 11 pre-existing unrelated failures, and clean (`error_count=0`) verification on `multiportmmi_8x8`, `benes_8x8`, and `benes_16x16` stable baselines.
  Date/Author: 2026-08-26, this investigation.
  Correction/caveat: subsequently proven (via the `stage=first_attempt`/`has_diagonal_state=false` trace in Surprises & Discoveries) that this specific fix does NOT change `n_15`/`n_16`'s outcome -- their forced-orthogonal result comes from a genuinely unrestricted search finding orthogonal legally necessary/cheapest in that obstacle state, not from `prefer_orthogonal_repair`. The fix is kept because it is independently correct and safe, not because it solves this plan's actual problem.

- Decision: do not treat the `--proactive-congestion-weight` correlation (see Surprises & Discoveries) as a proposed fix yet.
  Rationale: repository owner's explicit instruction ("i think htis has nohting to do with congestion") -- the mechanism is unproven, and this plan's job is to trace it before proposing it, not to declare victory on a correlation.
  Date/Author: 2026-08-26, this investigation.

- Decision: stop generating SVG/PNG visual renders as part of this investigation's default workflow; work from GDS files and textual/numeric per-net data (route_attempt_records, corrected_centerline_um diagonal detection, crossing-trace logs) instead.
  Rationale: repository owner's explicit instruction ("stop making the svg"). Visual renders remain available on request but are no longer produced proactively at each step.
  Date/Author: 2026-08-26, this investigation.

## Outcomes & Retrospective

Not yet applicable -- this plan is still in its investigation phase. No fix has been implemented or validated for the actual forced-orthogonal-detour problem yet.

## Context and Orientation

This repository implements a photonic waveguide router with a Python orchestration layer (`translation/route_rust.py` and friends) calling into a Rust routing core (`src/py_router.rs`, `src/astar.rs`) via PyO3 bindings. The Rust core does the actual A* pathfinding, on-grid crossing legality checking, and net-by-net commit/repair loop.

`multiportmmi_16x16` is one of several fixed benchmark netlists (`benchmarks/multiportmmi_16x16.py`, using placement data in `benchmarks/data/multiportmmi_16x16.yml`) used to exercise the router end to end. Its "stable baseline" command (documented at the top of that benchmark file) is:

    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --routing-window-scale 0.35 --foreign-port-keepout-cells 0

This currently fails partway through (at net `n_49`) for a different, already-understood reason (`.agent/REPOSITORY_STATE.md` Current Findings entry 2) -- NOT this plan's concern. This plan concerns nets `n_15` through `n_30`, which route successfully (the run completes past them) but with worse geometry than necessary.

Key terms used throughout this plan and the wider codebase:

"Net": one point-to-point optical connection to be routed, named `n_0`, `n_1`, ... in netlist declaration order. The Rust-side internal `net_id` used in trace output is 1-based against this 0-based suffix (net name `n_X` has Rust `net_id = X + 1`); this off-by-one is a frequent source of confusion and must be accounted for explicitly whenever reading Rust-level trace output against net names.

"45-degree turn" / "diagonal": this router's grid is 8-directional (0, 45, 90, ... 315 degrees, stored internally as an angle index 0-7 where even indices are the four cardinal/axis-aligned directions and odd indices are the four diagonal directions). "Orthogonal" means a route uses only cardinal (even-angle) segments; a route with any odd-angle (45-degree-multiple) segment is "diagonal" for the purposes of this plan's analysis.

"Crossing" / "crossing legality": when one net's route needs to physically intersect another already-committed net's route (the two waveguides cross at a point), this router requires that intersection to be geometrically legal: a clean, perpendicular (90-degree) intersection with adequate straight clearance on each side (`min_straight_cells_per_crossing`), and never a collinear/overlapping run (two routes cannot share the same cells or run parallel atop each other). This benchmark uses `crossing-mode lidar-pure`, meaning legality is discovered by actually colliding with obstacle cells during the search (not pre-planned) -- see `.agent/execplans/2026-08-19-restructure-crossing-partner-discovery.md` for the full architectural rationale, and `src/py_router.rs`'s `lidar_pure_owner_lookup_partner_set` function and its comment for the precise implementation-level statement of this model.

"Repair" / "victim reroute": when a net cannot be routed without illegally overlapping/crossing another already-committed net, the router's repair loop can rip up ("victimize") the blocking net and reroute both. This is orchestrated by a large set of functions in `src/py_router.rs` in the `PyPhotonicRouter` impl block, roughly in the range of lines 6200-10000 as of this writing -- names to know for this plan specifically: `reroute_victim_with_plain_fallback` (net_id-keyed victim reroute, the function actually responsible for `n_15`/`n_16`'s outcome), `try_crossing_aware_victim_reroute` (a separate, three-strategy crossing-aware cascade tried before the plain fallback), and `route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout` (the try-order-flipped function from this plan's landed, but ultimately tangential, fix).

"Proactive congestion" (`--proactive-congestion-weight`, `--proactive-congestion-radius-cells`): an existing, opt-in A* cost term (`primitive_lateral_congestion` in `src/astar.rs`) that adds cost to a candidate move proportional to how many cells to its sides (within the given radius) are already blocked/occupied. Already the default for `multiportmmi_8x8`, `benes_8x8`, and `benes_16x16` (their own `STABLE_ROUTING_FLAGS` in `benchmarks/*.py`) but NOT yet part of `multiportmmi_16x16`'s stable flags. Its relevance to this plan is an open, unproven correlation (see Surprises & Discoveries) -- it is not yet established that it is the right fix, or even that it works for the reason it appears to.

## Plan of Work

The next contributor's job is threefold, in order:

First, trace the actual mechanism by which `--proactive-congestion-weight` changes `n_15`/`n_16`'s search outcome. This means instrumenting (temporarily, following this session's established discipline of adding `eprintln!` diagnostics gated by an environment variable, verifying with `git diff`, and reverting before landing real changes -- see `.agent/REPOSITORY_STATE.md`'s "Temporary diagnostic discipline" note if present, or simply follow the pattern used throughout this plan's own Surprises & Discoveries evidence) the actual A* cost comparison between the diagonal and orthogonal candidate paths for `n_16`, with and without proactive congestion enabled, to see concretely which specific move's cost changes and why that changes which path wins. Do not skip this step and do not propose enabling proactive congestion for `multiportmmi_16x16` without it -- the repository owner has explicitly required proof, not correlation.

Second, based on what that tracing shows, decide whether proactive congestion is really the right lever, or whether a more targeted fix is needed -- for example, a fix that specifically detects the "two candidate diagonal paths would run collinear toward adjacent targets" case and offsets one of them by a small lateral amount, rather than forcing a full orthogonal fallback. Record this decision, with rationale, in the Decision Log before implementing it.

Third, implement the chosen fix and validate it at the same depth already established this session for other router changes: `cargo test --lib` (expect the existing 401 passed, 0 failed baseline, or note exactly what changed and why), `PYTHONPATH=. .venv/bin/pytest -q` (expect 335 passed with the same 11 pre-existing, already-documented failures -- see `.agent/REPOSITORY_STATE.md` Current Findings for that list), and a full re-route of `multiportmmi_8x8`, `benes_8x8`, and `benes_16x16` stable baselines (commands in each benchmark file's own header comment) confirming `error_count=0`/`warning_count=0` on both crossing and photonic verification, with no meaningful regression in route count or timing. Then update this plan's Outcomes & Retrospective and `.agent/REPOSITORY_STATE.md` with the final result.

## Concrete Steps

All commands below are run from the repository root (`/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`), with the project virtualenv activated (`source .venv/bin/activate`) and, for any Rust code change, rebuilt first with:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$(pwd)/.venv/bin/python3" maturin develop --release

(The `RUSTUP_TOOLCHAIN`/`PYO3_PYTHON` overrides are required in this environment because the repository's checked-in `rust-toolchain.toml`/`.cargo/config.toml` target a Windows cross-compile setup that does not match this Linux development environment; without the overrides, `cargo`/`maturin` fail outright.)

To reproduce the exact bad-geometry case and its root-cause trace, run (as a Python one-liner, not the `routing_flow.py` CLI directly -- the CLI's `--debug-stop-after-route` path was observed during this session to diverge from this exact invocation in verification behavior, though not in the underlying routing decisions this plan cares about):

    PHOTONIC_ROUTER_TRACE_CROSSING=1 PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1 PYTHONPATH=. .venv/bin/python3 -c "
    import os
    os.environ['PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT'] = '0.05'
    os.environ['PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES'] = '90'
    import routing_flow
    captured = {}
    routing_flow.verify_and_attach_photonic_reports = lambda **kw: captured.update(kw)
    routed_layout = routing_flow.run_routing_flow(
        'multiportmmi_16x16', enable_crossings=True, crossing_mode='lidar-pure',
        fanout_access_mode='static-stubs', routing_window_scale=0.35,
        foreign_port_keepout_cells=0, debug_stop_after_route_index=17,
        debug_timing=False,
    )
    routed_layout.write_gds('build/routed_multiportmmi_16x16_before_n16.gds')
    "

This writes a GDS containing every net up through `n_16` and prints the crossing/repair trace shown in Surprises & Discoveries to stderr. (`verify_and_attach_photonic_reports` is stubbed out here because the strict verification gate raises on this partial, intentionally-incomplete slice for reasons unrelated to what this plan is investigating -- routing coverage is genuinely incomplete since routing was told to stop early, and downstream endpoint-correction/self-intersection checks are not yet meaningful for the not-yet-routed remainder.)

To re-run the proactive-congestion correlation check (not yet a proven fix -- see Plan of Work for what must happen before this becomes a proposal), add `proactive_congestion_weight=4.0, proactive_congestion_radius_cells=3` to the `run_routing_flow(...)` call above (and to any per-net comparison script) and re-inspect `has_diagonal`/`bucket` per net via `debug_artifacts.routed_net_records` (each record has `.net_name`, `.corrected_centerline_um`, and `.net_id`) and `stats.route_attempt_records` (requires also passing `collect_route_stats=True, stats=RoutingFlowStats()` into `run_routing_flow`; each attempt record has `net_id`, `bucket_name`, `failed`, `used_simple_route`, and detailed A* counters including `primitive_generated_by_class`/`primitive_accepted_by_class`, keyed by primitive class index -- see `PRIMITIVE_STRAIGHT_SHORT`/`PRIMITIVE_STRAIGHT_LONG`/`PRIMITIVE_BEND_45`/`PRIMITIVE_BEND_90` constants near `src/astar.rs`'s `primitive_transition_class` function for the class-to-index mapping).

## Validation and Acceptance

For the landed-but-tangential try-order fix (`route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout`), validation already performed and passing, as of 2026-08-26:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$(pwd)/.venv/bin/python3" cargo test --release --lib
    -> test result: ok. 401 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

    PYTHONPATH=. .venv/bin/pytest -q
    -> 11 failed, 335 passed, 1 skipped (same 11 pre-existing, already-documented failures as the established baseline)

    multiportmmi_8x8, benes_8x8, benes_16x16 stable baselines (commands in each benchmark file's header)
    -> all three: error_count=0, warning_count=0 on both crossing and photonic verification JSON output

For this plan's actual, not-yet-implemented fix, acceptance is: re-running the reproduction command in Concrete Steps and observing that `n_15` and `n_16` (and ideally the rest of the `n_15`-`n_30` block) route diagonally without needing a rip-up/repair cycle, with the same validation depth listed above showing no regression, AND a traced, evidence-backed explanation (not a correlation) for why the fix works, recorded in this plan's Surprises & Discoveries or Decision Log before being considered done.

## Idempotence and Recovery

All commands in this plan are read/observe-only except the one already-landed Rust code change (the try-order flip), which is a normal, reversible source edit already committed to the repository's git history (find it via `git log --oneline -- src/py_router.rs` around 2026-08-26; revert with a normal `git revert` if it is ever found to cause a problem, though as of this writing it has not). Any future temporary diagnostic `eprintln!` additions used for the tracing in "Plan of Work" step one must be reverted (verify with `git diff` showing no unintended leftover changes) before committing the eventual real fix, per this session's established discipline.

## Artifacts and Notes

The exact trace transcript proving the `n_16`/`n_15` illegal-diagonal-crossing root cause is reproduced in full in Surprises & Discoveries above; it is the single most important piece of evidence in this plan and should not be re-derived from scratch by a future reader without first trying the reproduction command in Concrete Steps, which is known (as of 2026-08-26) to reproduce it exactly.

No GDS, SVG, or PNG artifacts from this investigation are checked into the repository (the `build/` directory is gitignored and treated as disposable scratch output throughout this codebase's existing conventions) -- regenerate them using the Concrete Steps command if visual inspection is needed again.

## Interfaces and Dependencies

This plan does not yet define any new function, type, or trait -- it is purely an investigation-and-fix plan against existing code. The most relevant existing interfaces a future fix will likely need to touch or call are all in `src/py_router.rs`'s `impl PyPhotonicRouter` block: `reroute_victim_with_plain_fallback`, `try_crossing_aware_victim_reroute`, `route_single_net_and_commit_native` (the plain, full-primitive-library search-and-commit function ultimately responsible for `n_15`'s successful-but-orthogonal outcome), and the crossing-legality checking functions reachable from `crossing_violations_for_route_with_ports`/`invalid_crossing_intersections_for_route`. The A* cost function itself, including `primitive_lateral_congestion` (the proactive-congestion mechanism under investigation), lives in `src/astar.rs`.
