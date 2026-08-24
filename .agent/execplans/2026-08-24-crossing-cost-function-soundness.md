# Make crossing-aware A* search cost-sound: crossings pay their own cost, terminal-straight launch/landing is honored everywhere

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository, TUMPhotonicRouter, routes optical waveguides for photonic chips on a discrete grid. A route is built out of "primitives" (short pre-defined path segments -- straight runs, 45-degree bends, 90-degree bends, etc., defined in `src/primitives.rs`) chained together by an A* pathfinding search (`src/astar.rs`). Two of the eight compass-style directions on the grid are "cardinal" (horizontal/vertical, 90-degree turns between them) and the other six are "diagonal" (45-degree turns); the grid uses an angle index 0-7 for these eight directions, called "eighths of a circle" in this repo's own vocabulary.

Two configuration values control how the search chooses between path shapes: `bend_weight` (how much extra cost is charged per unit of turning, in `src/astar.rs`'s `AStarConfig`) and `heuristic_weight` (how aggressively the search trusts its distance estimate to the target, trading optimality for speed). A third, unrelated value, `crossing_loss`, is the cost charged when a route's search decides to place a legal "crossing" -- a point where two different nets' waveguides physically cross paths on the chip, which is allowed under certain geometric conditions when `enable_crossings` is on.

Today, whenever a benchmark is run with `crossing_mode` set to `"collision"` or `"lidar-pure"` (an internal flag this repo calls `collision_crossing_mode`, computed in `translation/route_rust.py`), `bend_weight` and `heuristic_weight` silently change for every net in that benchmark, including nets that end up never crossing anything at all. This was discovered and confirmed empirically during this session's investigation of `build/routed_multiportmmi_8x8.gds`: two nets (`n_7` and `n_8` in the `multiportmmi_8x8` stable-baseline benchmark) route with unexpected, cost-suboptimal 90-degree bends instead of diagonals, and a direct check of the router's own `crossing_events()` output showed these two nets have **zero** crossing events anywhere in the final routed layout of 33 total events -- meaning they never had any crossing-legality reason to be treated any differently from a plain, non-crossing-aware route, yet they were, because `crossing_mode` is a single global setting applied to the entire benchmark rather than a per-net decision.

Separately, a second, independently confirmed gap: `require_terminal_straights` (an `AStarConfig` field that, when true, forces a route to launch and land using a straight-line primitive right at the port face, instead of bending immediately at the port) is fully implemented and honored in the router's plain search function (`route_single_net_with_bounds_dynamic_expansion` in `src/astar.rs`), but is never read at all by the router's separate crossing-aware search function (`route_single_net_with_bounds_crossing`, same file). Several call sites in `src/py_router.rs` explicitly set `require_terminal_straights = true` on a config and then pass that same config into the crossing-aware function expecting the constraint to hold -- it silently does not.

After this plan: (1) `bend_weight` and `heuristic_weight` are the same regardless of `crossing_mode` -- crossing-awareness only changes whether the search is allowed to reason about placing a legal crossing and what a crossing costs, not how it prices ordinary bends and distance; (2) `crossing_loss` (the per-crossing cost penalty) is re-validated and, if needed, re-tuned against the corrected weight scale so that crossings remain meaningfully "expensive" relative to a bend-heavy detour, exactly as intended; (3) a route that asks for a straight port launch/landing gets one whether or not its search happens to escalate into the crossing-aware code path. A person can see this working by re-running the `multiportmmi_8x8` stable-baseline benchmark (exact command in Concrete Steps) and observing that `n_7` and `n_8` (net ids 8 and 9) now route with a genuinely cost-minimal path shape (diagonal-then-straight, not all-90-degree), while `build/verification/*.json` for every benchmark in the validation ladder still reports `error_count=0`, and the router's `crossing_events()` output for a crossing-heavy benchmark shows crossing counts that are unchanged or explicable, not silently inflated by cheapened bends.

## Progress

- [x] (2026-08-24) Milestone 1: `require_terminal_straights` is now honored by the crossing-aware search kernel. Implemented via `.agent/scripts/codex_task.sh` (Codex CLI, Implementation Engineer role) against a fully-specified task derived directly from this plan's own Milestone 1 text; reviewed line-by-line before acceptance. Both guards inserted exactly as specified: source-launch guard after `stats.primitive_generated_by_class[primitive_class] += 1;` (now `src/astar.rs:4563-4568`), target-landing guard after `next_angle` is computed (now `src/astar.rs:4625-4632`). New test `terminal_straight_requirement_rejects_immediate_port_bends_in_crossing_kernel` added, mirroring the plain-kernel test exactly (same source/target `State::new(1,1,0)` -> `State::new(3,3,2)`, same first/last-primitive-is-straight assertions), using a minimal `CrossingSearchConfig` with one deliberately-unreachable dummy partner so `route_single_net_with_crossing_config`'s early `partners.is_empty()` return does not short-circuit the test. `cargo test --lib`: `394 passed; 0 failed` (up from 392 pre-change plus the 2 filtered `terminal_straight_requirement` tests both passing). No pre-existing test regressed.
- [ ] Milestone 2: remove the `collision_crossing_mode`-gated override of `bend_weight`/`heuristic_weight` in `translation/route_rust.py` so both are uniform regardless of crossing mode, delete the resulting dead `PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT` environment variable, and re-validate/re-tune the `crossing_loss` default against the corrected weight scale.
- [ ] Milestone 3: run the full validation ladder, confirm no regression, and record findings (including the corrected `n_7`/`n_8` route shape and any `crossing_loss` retuning decision) in `.agent/REPOSITORY_STATE.md`.

## Surprises & Discoveries

- Observation: the `bend_weight`/`heuristic_weight` divergence is not an accidental bug or an inverted condition -- it was added deliberately in commit `56e8a1d` ("routing: checkpoint crossing verification foundation", a large squashed checkpoint commit), which bolted a `collision_crossing_mode` gate onto weight-tuning code that previously (commit `5c8a00b`, "Enable tuned 45-degree A star routing") applied unconditionally to every 45-degree-turn-enabled route. The squash commit's message gives no per-line rationale for the split.
  Evidence: `git log --all --oneline -S "collision_crossing_mode = bool" -- translation/route_rust.py` returns only `56e8a1d`; `git show 56e8a1d -- translation/route_rust.py` shows the diff introducing the `collision_crossing_mode` conditional directly on top of the previously-unconditional weight-boost lines.

- Observation: a dedicated "make a crossing cost extra" knob already exists and is exactly the right mechanism the repository owner asked for -- it does not need to be invented, only re-validated. `crossing_loss` is added once per crossing actually encountered during search, as a flat additive term in the A* cost function.
  Evidence: `src/astar.rs:4815`: `... + f64::from(crossing_outcome.crossing_count) * crossing.crossing_loss;`. Its default value, when no explicit override is supplied and `enable_crossings` is true with `crossing_mode` in `{"collision", "lidar-pure"}`, is `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 50.0` (`translation/route_rust_crossing_plan.py:29`), and it is already independently overridable at runtime via the environment variable named by `COLLISION_CROSSING_SEARCH_LOSS_ENV = "PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM"` (`translation/route_rust_crossing_plan.py:30`), read inside `_effective_crossing_search_loss` (`translation/route_rust_crossing_plan.py:109-129`).

- Observation: unifying `bend_weight` changes the relative cost of "bend a lot to avoid a crossing" versus "just cross" as a side effect, because `crossing_loss` is denominated in the same additive cost units as bend cost and path length. `primitive.bend_cost` is stored in angle-eighths (`src/primitives.rs:292`: `bend_cost: angle_delta.unsigned_abs() as f64`), so a 90-degree bend has `bend_cost = 2`. Today, under `collision_crossing_mode`, `bend_weight` stays at its un-boosted default of `1.0` (`src/astar.rs:90`), so a 90-degree bend costs `2.0` against a `crossing_loss` of `50.0` -- a crossing is about 25 times as expensive as one 90-degree bend. If `bend_weight` becomes uniformly `12.0` (the plain 45-degree-tuned value, `translation/route_rust.py:6441`), the same 90-degree bend costs `24.0` against the same `crossing_loss` of `50.0` -- a crossing becomes only about 2 times as expensive as one bend. This is a real, evidence-backed reason `crossing_loss` needs re-validation as part of this same plan, not a separate follow-up; Milestone 2 must not be considered complete on the weight change alone.
  Evidence: computed directly from the constants above; `50.0 / 2.0 = 25.0` versus `50.0 / 24.0 ≈ 2.08`.

- Observation: `max_iterations` is a third value gated by the same `collision_crossing_mode` conditional in `translation/route_rust.py:6419-6424` (capped to `50_000` outside collision/lidar-pure mode, left at its caller-supplied value, `500_000` in the stable-baseline configuration, inside it) -- this is a search-budget knob, not a cost-function term, and the repository owner's explicit direction so far (see Decision Log) was scoped to `bend_weight`/`heuristic_weight` and `crossing_loss` only. `max_iterations` is deliberately left untouched by this plan; note it here so a future soundness pass does not have to rediscover it.
  Evidence: `translation/route_rust.py:6419-6424`.

- Observation: a `CrossingSearchConfig` test partner that is deliberately placed far from the route under test (so it is never actually crossed) must be paired with `require_all_partners: false`, not `true`. `require_all_partners: true` requires every listed partner to actually be crossed for the search to succeed at all, so a non-interacting dummy partner combined with `require_all_partners: true` would make `route_single_net_with_crossing_config` fail to find any route.
  Evidence: Codex's Milestone 1 implementation report; confirmed by the passing test using `require_all_partners: false`.

## Decision Log

- Decision: unify `bend_weight` and `heuristic_weight` so they no longer vary with `crossing_mode`; keep `crossing_loss` as the sole crossing-specific cost term, and re-tune its default as part of the same change rather than as a separate follow-up.
  Rationale: repository owner's direct statement, verbatim: "1. idk why the weight for bends should vary for the crossing mode. in the crossings mode only the cost function should add cost for crossings and crossings should be 'quite expensive', whatever that means. this should be tuned as well then." Confirmed by the repository owner replying "Yes, write the ExecPlan" immediately after this plan's scope (including the `crossing_loss` re-tuning) was proposed back to them.
  Date/Author: 2026-08-24, repository owner (via direct chat) and Claude (proposal, confirmed by owner).

- Decision: implement `require_terminal_straights` in the crossing-aware search kernel rather than removing the field or the call sites that set it to `true`.
  Rationale: the constraint is conceptually sound (forcing a straight launch/landing segment at a port is a real, intentional geometric requirement used elsewhere in the router) and several call sites already explicitly opt into it expecting it to apply; the gap is that the crossing kernel silently ignores it, not that the constraint itself is wrong. The repository owner asked "I dont even know if require_terminal_straights makes any sense tbh. Is it always active?" -- investigation (recorded in the chat, not repeated in full here) showed it defaults to `false` and is explicitly set `true` only for primary-attempt searches and `false` for repair/fallback searches, a deliberate and defensible strict-first-relaxed-later pattern; the owner did not object to this pattern once it was explained, only to the inconsistency between kernels.
  Date/Author: 2026-08-24, Claude (investigation and proposal), repository owner (implicit confirmation via "Yes, write the ExecPlan").

- Decision: `max_iterations`'s divergence between crossing and non-crossing mode is out of scope for this plan.
  Rationale: not part of what the repository owner asked to be fixed; recorded in Surprises & Discoveries instead so it is not silently lost.
  Date/Author: 2026-08-24, Claude.

## Outcomes & Retrospective

Not started.

## Context and Orientation

### Files this plan touches

`src/astar.rs` -- contains both search kernels (`route_single_net_with_bounds_dynamic_expansion`, the "plain kernel", and `route_single_net_with_bounds_crossing`, the "crossing kernel"), their shared `AStarConfig` struct (fields include `bend_weight`, `heuristic_weight`, `require_terminal_straights`, all documented inline where they are declared, near line 55-120), and their Rust unit tests (in the `#[cfg(test)] mod tests` block near the end of the file).

`translation/route_rust.py` -- the Python orchestration layer that builds the `AStarConfig` object (called `self.astar_cfg` on the `_RouteNetsRustSession` class) before handing routing jobs to the compiled Rust extension. The `collision_crossing_mode` conditional block that this plan removes lives at lines 6415-6441 of this file as of this writing; treat line numbers throughout this plan as approximate anchors to re-locate with `grep -n`, not as guaranteed-exact after other edits.

`translation/route_rust_crossing_plan.py` -- defines `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` (line 29) and `COLLISION_CROSSING_SEARCH_LOSS_ENV` (line 30), and the `_effective_crossing_search_loss` function (lines 109-129) that decides what `crossing_loss` value actually reaches the Rust search when the caller has not supplied an explicit non-zero value.

`.agent/REPOSITORY_STATE.md` -- the living snapshot document this repository keeps of overall project state; Milestone 3 updates it with this plan's outcome, following the same pattern used by every other ExecPlan completed this session (see, for example, `.agent/execplans/2026-08-24-modular-routing-strategies.md`'s own closing updates to that file).

### Key terms, defined for a reader new to this repository

"Net": one logical point-to-point waveguide connection to be routed, identified by a `net_id` integer and a human-readable name like `n_7`.

"Primitive": a short, pre-defined path segment (straight run, 45-degree bend, 90-degree bend, etc.) that the A* search chains together to build a full route; primitives are enumerated per starting angle in `src/primitives.rs`.

"Angle" / "eighths of a circle": this repository represents direction as an integer 0-7, where even values (0, 2, 4, 6) are the four cardinal (horizontal/vertical) directions and odd values (1, 3, 5, 7) are the four 45-degree diagonal directions.

"Crossing": a point where two different nets' physical waveguide paths cross each other on the chip. Some crossings are geometrically legal (for example, two waveguides crossing at a near-90-degree angle with enough separation); the router's crossing-aware search exists to find and validate these.

"`collision_crossing_mode`" (Python-only name, not a Rust concept): a boolean computed in `translation/route_rust.py` as `bool(self.enable_crossings) and self.crossing_mode in {"collision", "lidar-pure"}`. It is a single value computed once per routing session (i.e., once per benchmark run), not per net.

"Plain kernel" (this plan's shorthand, not an existing repository term): `route_single_net_with_bounds_dynamic_expansion` in `src/astar.rs`, the search function used when a net is not currently being searched for a legal crossing placement.

"Crossing kernel" (this plan's shorthand): `route_single_net_with_bounds_crossing` in `src/astar.rs`, the search function used when a net's search is actively reasoning about a candidate crossing against one or more specific "partner" nets (represented by `CrossingSearchConfig.partners`, a list of `CrossingSearchPartner` values each carrying the partner net's `net_id` and the waypoints of its already-committed path).

"`AStarConfig`": the Rust struct (`src/astar.rs`, declared near line 40-120) carrying every tunable knob for a single search call, including `bend_weight` (default `1.0`, `src/astar.rs:90`), `heuristic_weight`, `require_terminal_straights` (default `false`, `src/astar.rs:117`), and `max_iterations`.

"`CrossingSearchConfig`": the Rust struct carrying crossing-specific search parameters, including `net_id`, `partners`, `min_straight_cells`, `crossing_half_size_cells`, `bend_runout_cells`, `crossing_loss`, `require_all_partners`, and `terminal_bump_guard`. An example fully-populated literal (from an existing test) appears in this plan's Milestone 1 section.

### How the per-net decision to use the crossing kernel already works (and why it is not the problem)

This repository already has a correct, per-net (not global) decision about which search kernel to actually run for a given net: a cheap local lookup first checks whether any other already-committed net's path is geometrically near enough to be a plausible crossing partner, and only escalates to the more expensive crossing kernel if it finds one (`src/py_router.rs:2938` and the `lidar_pure_owner_lookup_partner_set` logic near `src/py_router.rs:3467`, both covered by existing unit tests near `src/py_router.rs:16583`). This part of the system already matches the repository owner's stated mental model ("A* explores, and when crossings are enabled it can find collisions and decide whether or not to insert a crossing") and this plan does not change it.

The problem this plan fixes is narrower: even when that per-net decision correctly resolves a net through the cheap, no-crossing-nearby path, the *cost function weights* that net's search runs with were still decided once, globally, from the benchmark-level `crossing_mode` flag -- not from that same per-net decision. This plan makes the weights uniform (removing the discrepancy entirely) rather than trying to thread the existing per-net decision through to weight selection, because the repository owner's own framing ("in the crossings mode only the cost function should add cost for crossings") calls for the simpler fix: stop varying bend/heuristic cost by mode at all, and rely on `crossing_loss` (which *is* naturally zero-effect for a net that never encounters a crossing candidate, since `crossing_outcome.crossing_count` is `0` for it) to carry 100% of the crossing-specific cost signal.

### Toolchain and safety notes (identical to every other ExecPlan completed in this repository this session)

Any `src/*.rs` change requires rebuilding the compiled Python extension before Python-level testing will see it. From the repository root:

    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo check --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" maturin develop --release

Do not run the `--debug-svgs` CLI flag with no value (or `all`) on `multiportmmi_8x8` or `multiportmmi_16x16` -- it triggers an expensive full-SVG export; always pass a specific numeric net selector if SVG debugging is needed.

This plan is independent of the currently-parked `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` plan (which is investigating a specific `n_50` repair-exhaustion failure in `multiportmmi_16x16`, unrelated to this plan's cost-function scope per that plan's own findings so far). Either plan can proceed without waiting on the other; if both are active at once, validate each independently before assuming a shared benchmark run covers both.

## Plan of Work

**Milestone 1** adds the two missing guard checks to the crossing kernel, `route_single_net_with_bounds_crossing` in `src/astar.rs` (function begins at line 4320 as of this writing; re-locate with `grep -n "fn route_single_net_with_bounds_crossing" src/astar.rs` if it has moved). The plain kernel already implements this correctly, in its own neighbor-expansion loop, as two `continue`-based guards (`src/astar.rs:3651-3667` as of this writing):

    if config.require_terminal_straights
        && state == source
        && !primitive_class_is_straight(primitive_class)
    {
        continue;
    }
    let next_x = state.x.checked_add(primitive.dx)?;
    let next_y = state.y.checked_add(primitive.dy)?;
    let next_angle = primitive.end_angle % 8;
    if config.require_terminal_straights
        && (next_x - target.x).abs() <= target_tolerance
        && (next_y - target.y).abs() <= target_tolerance
        && accepted_target_angles[next_angle as usize]
        && !primitive_class_is_straight(primitive_class)
    {
        continue;
    }

The first guard rejects any candidate primitive at the very first expansion step (when the search is standing at `source`) that is not a "straight" class primitive, forcing the route to leave the port going straight. The second guard rejects any candidate primitive whose destination would land within `target_tolerance` cells of `target` at an accepted target angle, unless that primitive is also straight-class, forcing the route to arrive at the port straight. Both reuse variables (`source`, `target`, `target_tolerance`, `accepted_target_angles`, `primitive_class_is_straight`) that already exist in both kernels under the same names -- confirm this with `grep -n "target_tolerance\|accepted_target_angles" src/astar.rs` before editing, since the crossing kernel computes `target_tolerance` and `accepted_target_angles` near its own line 4427-4428.

Insert the equivalent two guards into the crossing kernel's own neighbor-expansion loop, which begins near line 4555 (`for primitive_idx in primitive_order.into_iter().take(primitive_order_len) {`) as of this writing. Place the first guard immediately after the existing `stats.primitive_generated_by_class[primitive_class] += 1;` line (near 4562) and before the existing crossing-specific "pending straight run after a crossing" logic that follows it -- the order relative to that unrelated logic does not matter functionally, since both are independent `continue`-based rejections, but placing the terminal-straight check first keeps it visually adjacent to the plain kernel's equivalent placement for future readers. Place the second guard immediately after the crossing kernel computes `next_angle` (near line 4618, `let next_angle = primitive.end_angle % 8;`) and before its own bounds check (`if !bounds.contains(next_x, next_y) {`), mirroring the plain kernel's placement exactly.

Add a new Rust unit test named `terminal_straight_requirement_rejects_immediate_port_bends_in_crossing_kernel` in the `#[cfg(test)] mod tests` block of `src/astar.rs`, directly modeled on the existing plain-kernel test `terminal_straight_requirement_rejects_immediate_port_bends` (locate it with `grep -n "fn terminal_straight_requirement_rejects_immediate_port_bends" src/astar.rs`). The existing test builds an `AStarConfig` with `require_terminal_straights: true` and calls `route_single_net_with_config`; the new test must instead call `route_single_net_with_crossing_config` (the public wrapper around the crossing kernel, declared at line 2359 as of this writing), which additionally requires a `CrossingSearchConfig` argument with at least one entry in `partners` (an empty `partners` list makes the function return `None` immediately, at line 2370-2374, without exercising the search at all -- this is a real early-return in the production code, not a test artifact, so the test's partner does not need to actually interact with the route being tested; it only needs to exist). Use this pattern, adapted from an existing crossing-kernel test in the same file (`grep -n "fn crossing_move_accepts_benes8_route15_diagonal_sequence" src/astar.rs` to find the full original for reference):

    let crossing = CrossingSearchConfig {
        net_id: <the source net's id used in the test>,
        partners: vec![CrossingSearchPartner {
            net_id: <any other id>,
            waypoints: vec![/* any short path far away from the test route, so it is never actually reached */],
            target_terminal_bump_guard: None,
        }],
        min_straight_cells: 2,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: true,
        terminal_bump_guard: None,
    };

The test should route between a source and target chosen so that, absent `require_terminal_straights`, the shortest path would bend immediately at the source or target (the existing plain-kernel test's own source/target pair, `State::new(1, 1, 0)` to `State::new(3, 3, 2)`, is a reasonable starting point to adapt, adjusted only if the crossing kernel's additional bounds/partner requirements force a different obstacle map size). Assert, as the existing plain-kernel test does, that the first primitive used is straight-class and the last primitive's start angle equals its own end angle (i.e., it did not bend on its final step).

Validate Milestone 1 with:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib terminal_straight_requirement

Expect two passing tests: the pre-existing plain-kernel one and the new crossing-kernel one. Then run the full Rust suite (`cargo test --lib`, no filter) to confirm nothing else broke, since narrowing the crossing kernel's accepted primitives at source/target could in principle change which routes some other existing crossing-kernel test expects to find -- if any pre-existing crossing-kernel test starts failing, read its failure message and its `AStarConfig` construction: if that test does not set `require_terminal_straights: true` explicitly (checked with `grep -n "require_terminal_straights" src/astar.rs` around the failing test), then the new guards should not affect it at all, since both new guards are gated on `config.require_terminal_straights` being true, and any failure would indicate a mistake in the guard placement, not an intended consequence, and must be fixed before proceeding.

**Milestone 2** removes the `collision_crossing_mode`-gated weight override in `translation/route_rust.py` and re-validates `crossing_loss`. Locate the block with `grep -n "collision_crossing_mode = bool" translation/route_rust.py` (lines 6415-6441 as of this writing):

    collision_crossing_mode = bool(self.enable_crossings) and self.crossing_mode in {
        "collision",
        "lidar-pure",
    }
    if (
        self.allow_45_degree_turns
        and not collision_crossing_mode
        and hasattr(self.astar_cfg, "max_iterations")
    ):
        self.astar_cfg.max_iterations = min(int(self.astar_cfg.max_iterations), 50_000)
    if collision_crossing_mode and hasattr(self.astar_cfg, "heuristic_weight"):
        collision_heuristic_weight = os.environ.get(
            "PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT"
        )
        self.astar_cfg.heuristic_weight = (
            float(collision_heuristic_weight)
            if collision_heuristic_weight
            else 1.0
        )
    elif self.allow_45_degree_turns and hasattr(self.astar_cfg, "heuristic_weight"):
        self.astar_cfg.heuristic_weight = max(float(self.astar_cfg.heuristic_weight), 1.25)
    if collision_crossing_mode and hasattr(self.astar_cfg, "bend_weight"):
        self.astar_cfg.bend_weight = float(self.astar_cfg.bend_weight)
    elif self.allow_45_degree_turns and hasattr(self.astar_cfg, "bend_weight"):
        # LiDAR heavily penalizes bends relative to propagation. Matching that
        # scale keeps 45-degree A* from spending work on short zig-zag variants.
        self.astar_cfg.bend_weight = max(float(self.astar_cfg.bend_weight), 12.0)

Replace it with a version that keeps the `max_iterations` cap logic completely unchanged (per this plan's Decision Log, that divergence is out of scope) but removes `collision_crossing_mode` from the `heuristic_weight` and `bend_weight` decisions entirely, so both always use the plain, 45-degree-tuned path when `self.allow_45_degree_turns` is true, regardless of crossing mode:

    collision_crossing_mode = bool(self.enable_crossings) and self.crossing_mode in {
        "collision",
        "lidar-pure",
    }
    if (
        self.allow_45_degree_turns
        and not collision_crossing_mode
        and hasattr(self.astar_cfg, "max_iterations")
    ):
        self.astar_cfg.max_iterations = min(int(self.astar_cfg.max_iterations), 50_000)
    if self.allow_45_degree_turns and hasattr(self.astar_cfg, "heuristic_weight"):
        self.astar_cfg.heuristic_weight = max(float(self.astar_cfg.heuristic_weight), 1.25)
    if self.allow_45_degree_turns and hasattr(self.astar_cfg, "bend_weight"):
        # A bend costs bend_weight per angle-eighth of turn (see
        # src/primitives.rs's bend_cost field); boosting it keeps 45-degree
        # A* from spending work on short zig-zag variants. This applies
        # uniformly regardless of crossing_mode -- crossing-awareness only
        # changes whether/what a *crossing* costs (see crossing_loss,
        # translation/route_rust_crossing_plan.py), not how bends are priced.
        self.astar_cfg.bend_weight = max(float(self.astar_cfg.bend_weight), 12.0)

`collision_crossing_mode` remains defined and used (by the still-unchanged `max_iterations` logic immediately above it), so do not delete the variable itself -- only its use in the two weight blocks.

Delete the now-fully-unused environment variable read. Confirm it truly has no other reader before deleting, with:

    grep -rn "PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT" --include="*.py" --include="*.rs" --include="*.md" .

As of this writing this returns exactly one match, the deleted line itself, confirming it is safe to remove outright (no test, benchmark script, or documentation file references it).

Next, re-validate `crossing_loss`. This does not require any code change to explore, because the existing `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM` environment variable (`translation/route_rust_crossing_plan.py:30`) already overrides the default at runtime. Rebuild the extension first (the weight-unification change above must be compiled in), then run a crossing-heavy benchmark -- `multiportmmi_8x8` under its stable-baseline crossing configuration is the fastest one available and was the benchmark used to originally discover this whole issue -- at a small sweep of `crossing_loss` values including the current default, and compare crossing counts and route quality using this diagnostic pattern (adapted from a disposable scratch script used earlier in this investigation; write it to a temporary file, e.g. `/tmp/check_crossing_loss_sweep.py`, and delete it when done since it is not part of the repository):

    import sys, os
    sys.path.insert(0, ".")
    os.environ["PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT"] = "0.05"
    os.environ["PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES"] = "90"
    # Set this to each value being swept before running, e.g.:
    # os.environ["PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM"] = "50.0"

    import translation.route_rust as route_rust_mod
    captured = {}
    _orig_dispatch = route_rust_mod._RouteNetsRustSession._dispatch_native_routing
    def _dispatch_tap(self, route_jobs):
        result = _orig_dispatch(self, route_jobs)
        captured["router"] = self.router
        return result
    route_rust_mod._RouteNetsRustSession._dispatch_native_routing = _dispatch_tap

    from routing_flow import run_routing_flow
    run_routing_flow(
        "multiportmmi_8x8",
        enable_crossings=True,
        crossing_mode="lidar-pure",
        fanout_access_mode="static-stubs",
        routing_window_scale=0.35,
        foreign_port_keepout_cells=0,
    )
    events = captured["router"].crossing_events()
    print(f"crossing_loss={os.environ.get('PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM', 'default')} total_crossing_events={len(events)}")

Run this at, at minimum, the current default (`50.0`), a value close to what the un-boosted-`bend_weight` era implied it should feel like relative to the new bend cost scale (roughly `12x` the old effective ratio, i.e. in the `500-600` range, since a 90-degree bend now costs `24.0` instead of `2.0` and the goal is to preserve roughly the same "crossing is about 25x one bend" relationship the un-tuned default happened to produce), and at least one point in between (for example `150.0` or `200.0`). For each value, record the total crossing-event count and, separately, confirm via `build/verification/*.json` that `multiportmmi_8x8`'s stable-baseline run still completes with `error_count=0` (a `crossing_loss` set too high could in principle make the search prefer an illegal-crossing-avoiding detour that fails elsewhere, though this has not been observed and is not expected). Choose the smallest value that keeps crossing-event counts and route legality consistent with the pre-Milestone-2 baseline (the actual pre-change crossing-event count for this exact benchmark and config, captured once before making any Milestone 2 edit, is the number to compare against) -- prefer the smallest value that preserves that baseline rather than the largest one tried, since a needlessly large `crossing_loss` would bias the search away from using crossings even when a crossing is legitimately the best available option, which is a real design goal of the crossing feature itself (crossings should be avoided when a clean same-net route exists, not avoided unconditionally). Record the chosen value, the sweep data that justified it, and update `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` in `translation/route_rust_crossing_plan.py:29` to that value (or leave it at `50.0` with a recorded rationale, if the sweep shows `50.0` already behaves acceptably at the new weight scale -- do not assume either outcome in advance).

Validate Milestone 2 with:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" maturin develop --release
    PYTHONPATH=. .venv/bin/python -m pytest -q
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

Expect `pytest -q` to report all tests passing (no test in the current suite asserts on the specific pre-change `bend_weight`/`heuristic_weight` values under collision/lidar-pure mode, confirmed by `grep -rn "collision_crossing_mode\|bend_weight\|heuristic_weight" tests/` returning no matches as of this writing, so this change is not expected to require any test updates beyond the new Rust unit test added in Milestone 1). Expect the `multiportmmi_8x8` run to complete with `build/verification/multiportmmi_8x8_verification.json` (or the equivalently-named file this benchmark produces; confirm the exact filename with `ls build/verification/` after the run) reporting `error_count=0`.

**Milestone 3** confirms the fix's user-visible effect and runs the complete validation ladder this repository has used for every change this session, then records the outcome. First, confirm `n_7`/`n_8` (net ids 8 and 9) now route without the anomalous 90-degree-only bend shape, reusing the diagnostic pattern above but capturing route geometry instead of crossing events (adapt the `realize_routed_net_records` monkeypatch tap used earlier in this same investigation, or inspect `build/routed_multiportmmi_8x8.gds` visually as the repository owner originally did). Then run the full ladder:

    cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --lib
    PYTHONPATH=. .venv/bin/python -m pytest -q
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_4x4
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_8x8 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16
    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

The last command is expected to still fail with the pre-existing, separately-tracked `n_50` error documented in `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` -- that failure is out of scope for this plan and its presence or absence must not be treated as this plan's success criterion; only confirm that its error text is unchanged (still names `n_50` with the same `candidate_blockers=[49, 50]` signature), which would indicate this plan's changes did not alter that unrelated failure.

Update `.agent/REPOSITORY_STATE.md` with this plan's outcome (both the code changes and the final chosen `crossing_loss` value with its rationale), following the structure already used there for every other completed ExecPlan this session.

## Concrete Steps

See the exact commands embedded in each milestone's description above in Plan of Work; they are the authoritative, exact commands for this plan and are not repeated redundantly here. Run every command from the repository root, `/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter` (do not hard-code this absolute path in any script or test; it is stated here only to orient a reader unfamiliar with the checkout).

## Validation and Acceptance

Milestone 1: `cargo test --lib` reports all tests passing, including a new test demonstrating that `require_terminal_straights: true` is honored by `route_single_net_with_crossing_config` the same way it already is by `route_single_net_with_config`.

Milestone 2: `pytest -q` reports all tests passing; the `multiportmmi_8x8` stable-baseline crossing-configuration run completes with `error_count=0`; a recorded `crossing_loss` sweep (values tried, crossing-event counts observed, legality outcome observed) justifies the final chosen default, whether that default changes from `50.0` or is confirmed to stay at `50.0`.

Milestone 3: the full validation ladder (`cargo test --lib`, `pytest -q`, `benes_4x4`, `multiportmmi_8x8` both configurations, `benes_8x8`, `benes_16x16`, `multiportmmi_16x16` stable-baseline) completes with every benchmark except `multiportmmi_16x16` reporting `error_count=0`, `multiportmmi_16x16` still failing with the unchanged, separately-tracked `n_50` error, and a direct observation (geometry inspection or a route-record diagnostic) confirming `n_7`/`n_8` no longer exhibit the anomalous all-90-degree bend shape that originally motivated this investigation.

## Idempotence and Recovery

All diagnostic scripts described in this plan are read-only with respect to the repository; they only read `translation/route_rust.py` and write to the gitignored `build/` directory or to `/tmp`. The `crossing_loss` sweep in Milestone 2 is safe to re-run any number of times with different environment-variable values; it does not require reverting anything between runs. If a Rust change in Milestone 1 causes an unexpected pre-existing test failure, do not proceed to Milestone 2 until it is understood and fixed (or the guard placement is revised) -- Milestone 2's weight-unification change is independent of Milestone 1's terminal-straight fix and could be implemented and validated first if that proves easier, but this plan sequences terminal-straights first because it is the smaller, more mechanically-verified change of the two, and de-risks Milestone 2's larger weight/cost re-tuning by not having two unvalidated changes in flight at once.

## Artifacts and Notes

(to be filled in as Milestone 2's `crossing_loss` sweep and Milestone 3's final validation runs produce concrete numbers worth preserving)

## Interfaces and Dependencies

None prescribed beyond what is already described in Context and Orientation. This plan does not depend on `.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` completing first, and that plan does not depend on this one; both may proceed independently, though re-running the full validation ladder after both are complete (rather than only after each individually) is good practice before considering the repository's benchmark baseline fully re-stabilized.
