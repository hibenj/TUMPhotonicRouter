# Heuristics and tunable-constant inventory (evidence for the engine-performance plan's Milestone 1c)

Companion artifact to `.agent/execplans/2026-08-31-engine-performance-baseline.md`.
Produced 2026-08-31 by an evidence lane (sonnet), verified spot-wise by the lead.
INVENTORY ONLY -- classification (principled / evidenced tuning / unjustified)
and any change decisions happen in the main plan and belong to the repository
owner. Line numbers are as of commit 5a0bf0e-era HEAD and will drift.

Format: `name/literal | value | file:line | effect | justification`

## 1. src/astar.rs -- AStarConfig fields (struct ~line 51, Default impl ~84-118)

Production searches are NOT built from `AStarConfig::default()` --
`translation/route_rust.py` calls the pyo3 constructor `PyAStarConfig::new`
(py_router.rs:238-290), whose signature defaults differ from the Rust struct
in one place (flagged below).

    max_iterations | 100_000 | astar.rs:87 | hard cap on search expansions | none found
    bend_weight | 1.0 | astar.rs:88 | cost per eighth-turn of bend | none found
    target_tolerance_cells | 0 | astar.rs:89 | arrival radius; pads window margin | none found
    require_target_angle | true | astar.rs:90 | exact arrival heading required | none found
    allowed_target_angles_mask | None | astar.rs:91 | acceptable arrival headings mask | none found
    use_routing_window | true | astar.rs:92 | bound search to window vs full grid | none found
    routing_window_min_margin_cells | 12 | astar.rs:93 | floor on window margin | none found
    routing_window_scale | 0.35 | astar.rs:94 | fraction of span added as margin | none found
    routing_window_max_expansions | 3 | astar.rs:95 | window-growth retries | none found
    routing_window_fallback_full_grid | false (Rust) vs true (pyo3 ctor, py_router.rs:239) | astar.rs:96 | full-grid retry after window exhaustion | none found -- DEFAULT DISCREPANCY
    routing_window_growth | 0.5 | astar.rs:97 | window margin growth per expansion | none found
    max_dense_states | 20_000_000 | astar.rs:98 | dense state storage cap | none found
    max_dense_obstacle_cells | 10_000_000 | astar.rs:99 | dense obstacle-cell cap | none found
    enable_simple_routes | true | astar.rs:100 | simple-route fast path first | none found
    simple_route_max_offset_cells | 96 (hardcoded ctor body) | astar.rs:101, py_router.rs:271 | fast-path max lateral offset | none found
    simple_route_min_leg_len_cells | 1 (hardcoded ctor body) | astar.rs:102, py_router.rs:272 | fast-path min leg | none found
    ignore_dynamic_obstacles | false | astar.rs:103 | probe mode toggle | none found
    history_weight | 0.0 | astar.rs:104 | rip-up history cost multiplier | none found
    long_straight_congestion_weight | 0.0; settable ONLY via env read Rust-side (py_router.rs:2560) | astar.rs:105 | long-straight congestion multiplier | none found -- unreachable via Python ctor kwargs
    proactive_congestion_weight | 0.0 | astar.rs:106 | lateral congestion multiplier (straights) | none found
    proactive_congestion_radius_cells | 0 | astar.rs:107 | lateral congestion scan radius | none found
    collect_detailed_timing | false | astar.rs:108 | stats only | n/a
    enable_jps4 | false (hardcoded ctor body) | astar.rs:109, py_router.rs:277 | JPS acceleration | none found
    use_indexed_heap | false | astar.rs:110 | indexed open-set heap | none found
    primitive_ordering | Library | astar.rs:111 | primitive try order | none found
    heuristic_mode | HeadingAware | astar.rs:112 | heuristic formula variant | none found
    heuristic_weight | 1.0 | astar.rs:113 | heuristic multiplier; >1 inadmissible | justified: 2026-08-31 admissible plan
    heap_tie_breaker | SmallerG (hardcoded ctor body) | astar.rs:114, py_router.rs:280 | equal-f tie-break rule | none found
    require_terminal_straights | false | astar.rs:115 | straight run at path terminals | none found
    max_search_time_ms | 0 disabled; env PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS/_S (py_router.rs:2489) | astar.rs:116 | wall-clock cutoff, polled every 4096 iters (SEARCH_TIMEOUT_CHECK_INTERVAL astar.rs:28) | none found

Kernel literals: JPS4 cardinal-only directions (astar.rs:2356, none found);
window margin formula scale*span*growth floored at min_margin plus tolerance
(astar.rs:6885-6894, none found).

## 2. src/py_router.rs -- repair/commit thresholds, margins, factors

    PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD | 100 | py_router.rs:4650-4655 | partner pending-straight event count before repair-victim hint; 0 disables | none found
    CROSSING_SPACING_HISTORY_AMOUNT | 1 | py_router.rs:979 | history bump near realized crossings | none found
    LONG_STRAIGHT_CONGESTION_MIN_UM | 200.0 | py_router.rs:980 | min straight length before congestion booked | none found
    LONG_STRAIGHT_CONGESTION_LATERAL_RADIUS_CELLS | 5 | py_router.rs:981 | congestion lateral spread | none found
    LONG_STRAIGHT_CONGESTION_AMOUNT | 1 | py_router.rs:982 | congestion increment per cell | none found
    SOURCE_LAYER_CENTER_OUT_MIN_JOBS | 8 | py_router.rs:983 | min layer size for center-out ordering | none found
    middle-window fraction 0.25/0.75 | py_router.rs:3338-3339 | congestion only on middle 50% of run | none found
    DEFAULT_MEANDER_DEPTH_CANDIDATES_UM [40..2] | py_router.rs:2862-2864 | meander depth ladder | none found
    meander endpoint-inset ladder [1.0,0.75,0.5,0.25,0]*r | py_router.rs:2909-2913 | inset candidates | none found
    cross_sin threshold 0.5 (sin 30 deg) | py_router.rs:1750,5589 | near-perpendicular exemption in parallel-overlap checks | justified: admissible plan
    realized_crossing_margin_um = grid*half_size | py_router.rs:2184-2186 | crossing keepout margin | none found
    repair defaults max_rounds=4, max_victims=8, history_weight=2.0, increment=1 | py_router.rs:12547 | plain repair loop bounds | none found
    negotiated repair max_rounds=8, history_weight=2.0 | py_router.rs:13133 | negotiated loop bounds | none found
    victim ordering (static=0, reservation=1, expected-pair-mismatch=2, then insertion) | py_router.rs:8786-8808 | who is ripped first | none found
    max_anchor_search_cells=8 | py_router.rs:13625,13654 | port-to-anchor snap radius | none found
    block/commit/core radius pyo3 defaults (0/None/None) | e.g. py_router.rs:12388,12420,12547 | keepout radii absent explicit clearance policy | none found

## 3. translation/route_rust.py -- production astar_cfg overrides and sizing (~6671-6800)

    use_indexed_heap forced ON when 45-degree | 6674 | heap swap | none found
    heuristic_mode heading_aware -> diagonal_aware when 45-degree | 6679-6682 | formula swap | none found
    max_iterations capped 50k when 45-degree, non-crossing, min weight > 1.0 | 6690-6700 | budget cap for weighted search only | justified: inline (heater_s_mod evidence)
    heuristic_weight = max(cur, env PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT, default 1.0) | 6687-6712 | THE clamp | justified: admissible plan
    bend_weight = max(cur, 12.0) when 45-degree | 6713-6720 | 12x bend cost to suppress zig-zags | mechanism commented, magnitude 12.0: none found
    heap_tie_breaker smaller_g -> larger_g when 45-degree | 6721-6724 | tie-break flip for diagonals | none found
    routing_window_min_margin_cells = max(cur, 2*bend_radius+commit_radius+2) | 6748-6755 | geometric floor | "+2": none found
    simple_route_max_offset_cells = max(cur, 12*bend_radius+2*commit_radius) | 6756-6759 | fast-path reach floor | coefficients: none found
    port_lane_length_cells = max(3, 2*bend_radius+2) | 6769 | port runway length | none found
    port_lane_half_width_cells = max(1, bend_radius+commit_radius+1) | 6770-6772 | port runway half width | none found
    stub port lane 0/0 via PHOTONIC_ROUTER_STUB_PORT_LANE_* | 6787-6800 | dense-fanout-scoped override | justified: inline comment + regression evidence
    dense_fanout_min_ports default 3 (env, >=2) | 1357-1371 | dense fanout group size | justified: docstring (2x2 switch case)
    stub forward = max(3, bend_radius+3); lane_spacing=11; x_offset=1 (env-overridable) | 2117-2130 | static stub geometry | none found
    protected lane spacing default 3 (3-level env fallback) | 2320-2328, 2582-2588 | fanout target lane spacing | none found
    FANOUT_STUB_BEND_DEGREES default 90 (45|90) | 1503-1520 | stub bend angle | none found

## 4. routing_flow.py / routing_flow_config.py CLI defaults (config:22-62)

    SCRIPT_BEND_RADIUS_UM=5.0 | bend geometry everywhere | none found
    SCRIPT_GRID_SIZE_UM=2.0 | grid pitch | none found
    SCRIPT_WAVEGUIDE_CLEARANCE_UM=0.0 | optical clearance (keepouts derive via OpticalRouteClearancePolicy, route_rust_types.py:50-94) | none found
    SCRIPT_HEATER_CLEARANCE_UM=10.0 | heater clearance | none found
    SCRIPT_MAX_ITERATIONS=5_000_000 | CLI-level cap (vs Rust default 100k, vs 50k weighted cap) | none found
    SCRIPT_ROUTING_WINDOW_SCALE=0.05 | CLI window scale -- MISMATCHES AStarConfig built-in 0.35; benchmarks re-pin 0.35 | none found, mismatch unexplained
    SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS=6 | foreign-port keepout | none found
    SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING=2 (route_rust.py:211) | straight run around crossings | none found
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM=80.0 (route_rust_types.py:25) | max meander height; Rust pyfunction default 20.0 (py_router.rs:3130) is dead code -- every call passes explicit | none found
    SCRIPT_RIPUP_* 4/8/2.0/1 | mirrors py_router repair defaults | none found
    SCRIPT_ELECTRICAL_GRID_PITCH_UM=10.0, _OBSTACLE_CLEARANCE_UM=10.0 | electrical grid | none found
    SCRIPT_CROSSING_MODE="lidar-pure", SCRIPT_FANOUT_ACCESS_MODE="legacy-runway" | mode selectors gating the above | n/a

## 5. Benchmark STABLE pins

    benes_16x16/benes_8x8: LONG_STRAIGHT=0.05 env; proactive 4.0/3; window scale stays CLI 0.05 | 0.05 value: none found
    multiportmmi_16x16/8x8: LONG_STRAIGHT=0.05, FANOUT_STUB_BEND_DEGREES=90; --routing-window-scale 0.35 | pins: none found
    heater_s_mod: 90-degree config (r=10, clearance 3.0, crossings off, PLM on, electrical on) | n/a (clamps do not apply)
    no pins: benes_4x4, benes_32x32, multiportmmi_32x32, clements_8x8, mmi_heater*, heater_s, heater_s_compact, TOY
    (the temporary benes_16x16 MIN_HEURISTIC_WEIGHT=1.25 pin is confirmed removed at HEAD)

## 6. Env vars carrying numeric weights/thresholds

    PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT | 1.0 | route_rust.py:6688 | justified (plan)
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT | unset=0.0 | py_router.rs:2560 | none found; only path to that config field
    PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD | 100 | py_router.rs:4654 | none found
    PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS/_S | unset | py_router.rs:2539-2558 | none found
    PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM | 200.0 | route_rust_crossing_plan.py:29,128 | search-time crossing penalty | none found
    PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS | 3 | route_rust.py:1365 | justified (docstring)
    PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS | max(3,r+3) | route_rust.py:2117-2121 | none found
    PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS | 11 | route_rust.py:2124-2126 | none found
    PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS | 1 | route_rust.py:2128-2130 | none found
    PHOTONIC_ROUTER_*PROTECTED_LANE_SPACING_CELLS chain | 3 | route_rust.py:2320-2328 | none found
    PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES | 90 | route_rust.py:1503 | none found
    PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH/_HALF_WIDTH_CELLS | 0/0 | route_rust.py:6796-6801 | justified (inline + regression)
    PHOTONIC_ROUTER_CROSSING_GRID_* (7 values: 20.0/5.0/4.0/4.0/2.0/6.0/0.0) | preplaced_crossing_grids.py:83-98 | preplaced-grid geometry | none found (contribution territory anyway)
    PHOTONIC_ROUTER_NEGOTIATED_REPAIR | toggle | route_rust.py:4359 | selects 8-round negotiated path | none found
    PHOTONIC_ROUTER_LAYER_ORDER | "span" toggle | route_rust.py:2774 | justified (commit c01b17a, experiment)
    (~18 pure trace/debug toggles excluded as non-behavioral)

## Count summary

99 individually inventoried behavioral knobs; 13 with some in-repo
justification, 86 with none found. Two internal default discrepancies:
`routing_window_fallback_full_grid` (Rust false vs pyo3-ctor true) and
`long_straight_congestion_weight` (unreachable through the Python ctor,
env-only). Reported as-is; judgement happens in the main plan.
