"""The photonic routing flow: two entry points over the stage modules.

`route_benchmark(config, options)` loads the benchmark named in
`options.loading` and hands the schematic to `route_schematic(schematic,
config, options)`, which runs every stage after the load:

1. Translate the schematic to an unrouted layout
2. Optionally place topology-derived crossing grids (contribution 2)
3. Route the optical nets with the Rust router backend
4. Verify, report path-length matching, optionally route heater metal
5. Write (or show) the routed layout

Both take the two objects the flow is configured with and nothing else: the
`RoutingConfig` (the router's own settings, Milestone 1) and the `FlowOptions`
(the flow-level choices, `photonic_router.flow_options`). Neither reads the
process environment and neither knows about a command line - the command line
is `photonic_router.cli`, which builds both objects and calls
`route_benchmark`. `run_routing_flow` at the bottom of this module is the
compatibility wrapper that keeps the old keyword API working.
"""

import importlib
import time
from dataclasses import replace

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RoutingConfig
from photonic_router.flow_options import (
    DebugArtifactOptions,
    ElectricalOptions,
    FlowOptions,
    LayoutOptions,
    LoadingOptions,
    ObstacleOptions,
    OpticalStageOptions,
    OutputOptions,
    PathLengthMatchingOptions,
    PreplacedCrossingGridOptions,
    StatsOptions,
)
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow_config import (
    DebugSvgSelector,
    SCRIPT_ALLOW_45_DEGREE_TURNS,
    SCRIPT_BEND_RADIUS_UM,
    SCRIPT_CHIP_ADD_X_UM,
    SCRIPT_CHIP_ADD_Y_UM,
    SCRIPT_FANOUT_ACCESS_MODE,
    SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    SCRIPT_GRID_SIZE_UM,
    SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
    build_optical_routing_stage_config,
    build_static_obstacle_config,
    debug_artifact_routing_options,
    parse_debug_svg_selector,
    resolve_legacy_display_options,
)
from routing_flow_electrical import run_electrical_routing_step
from routing_flow_optical import run_photonic_routing_stage
from routing_flow_plm import attach_and_report_path_length_matching
from routing_flow_reporting import (
    cleanup_debug_artifacts,
    report_and_open_debug_svgs,
    write_or_show_routed_layout,
)
from routing_flow_stats import (
    RoutingFlowStats,
    populate_route_stats,
    record_initial_route_stats,
)
from routing_flow_verification import verify_and_attach_photonic_reports
from translation.crossing_modes import is_guided_mode
from translation.electrical import ElectricalRoutingConfig, ElectricalRoutingResult
from translation.layout_from_schematic import layout_from_schematic
from translation.route_order import default_net_order
from translation.route_rust import RipupRerouteConfig


def _depth_by_node_from_schematic(schematic: Schematic) -> dict[str, int]:
    """Instance depth (hops from a true source) on the benchmark's own
    netlist, before any contribution 2 splitting."""
    from types import SimpleNamespace

    from translation.route_order import depth_by_node_from_jobs

    jobs = []
    for bundle in schematic.netlist.routes.values():
        for port1_spec, port2_spec in bundle.links.items():
            inst1 = str(port1_spec).split(",")[0]
            inst2 = str(port2_spec).split(",")[0]
            jobs.append(SimpleNamespace(inst1=inst1, inst2=inst2))
    return depth_by_node_from_jobs(jobs)


def load_benchmark(benchmark_name: str) -> Schematic:
    """Load a benchmark schematic from the benchmarks directory.

    Parameters:
        benchmark_name: The name of the benchmark module (e.g., 'heater_s_mod').
                       The module must have a `build_schematic()` function.

    Returns:
        A gdsfactory Schematic object.

    Raises:
        ModuleNotFoundError: If the benchmark module is not found.
        AttributeError: If the benchmark doesn't have a `build_schematic()` function.
    """
    try:
        benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
        schematic = benchmark_module.build_schematic()

        if not isinstance(schematic, Schematic):
            raise TypeError(
                f"Expected Schematic from {benchmark_name}.build_schematic(), got {type(schematic)}"
            )

        return schematic
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            f"Benchmark '{benchmark_name}' not found in benchmarks/ directory"
        ) from e
    except AttributeError as e:
        raise AttributeError(
            f"Benchmark '{benchmark_name}' must have a 'build_schematic()' function"
        ) from e


def _load_benchmark_stage(
    *,
    benchmark_name: str,
    total_steps: int,
    stats: RoutingFlowStats | None,
    debug_meanders: bool,
) -> Schematic:
    """Load the schematic and record the load-stage stats/output."""
    step_load_start = time.perf_counter()
    print(f"\n[1/{total_steps}] Loading benchmark: {benchmark_name}...")
    schematic = load_benchmark(benchmark_name)
    step_load_end = time.perf_counter()
    if stats is not None:
        stats.instance_count = len(schematic.netlist.instances)
        stats.net_count = len(schematic.netlist.routes)
        stats.step_times_s["load_benchmark"] = step_load_end - step_load_start
    print("      \u2713 Schematic loaded")
    print(f"      - Load time: {step_load_end - step_load_start:.4f} s")
    if debug_meanders:
        print(f"      - Instances: {list(schematic.netlist.instances.keys())}")
        print(f"      - Placements: {list(schematic.placements.keys())}")
    else:
        print(f"      - Instances: {len(schematic.netlist.instances)}")
        print(f"      - Placements: {len(schematic.placements)}")
    return schematic


def _layout_from_schematic_stage(
    *,
    schematic: Schematic,
    total_steps: int,
    stats: RoutingFlowStats | None,
    debug_timing: bool,
) -> Component:
    """Translate the schematic to an unrouted layout and report layout metadata."""
    step_layout_start = time.perf_counter()
    print(f"\n[2/{total_steps}] Translating schematic to layout...")
    unrouted_layout = layout_from_schematic(schematic)
    step_layout_end = time.perf_counter()
    if stats is not None:
        stats.step_times_s["layout_from_schematic"] = step_layout_end - step_layout_start
    print(f"      \u2713 Layout generated: {unrouted_layout.name}")
    bbox = unrouted_layout.bbox
    if callable(bbox):
        bbox = bbox()
    print(f"      - Bounding box: {bbox}")
    if debug_timing:
        print(f"      - Translation time: {step_layout_end - step_layout_start:.4f} s")
    return unrouted_layout


def _print_flow_header(benchmark_name: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'=' * 60}")


def _print_flow_footer() -> None:
    print(f"\n{'=' * 60}\n")


def _preplaced_crossing_grids_stage(
    *,
    benchmark_name: str,
    schematic: Schematic,
    unrouted_layout: Component,
    total_steps: int,
    stats: RoutingFlowStats | None,
    router_probe_kwargs: dict[str, object] | None = None,
    config: RoutingConfig,
) -> tuple[Schematic, Component, dict[str, object], frozenset[str], frozenset[str] | None]:
    """Replace the schematic/layout with the crossing-grid-derived pair.

    The fourth element names the nets of layers without a feasible pre-placed
    structure; the flow routes them with the guided crossing search. The
    fifth element is the dense-fanout-instance set the stage computed for its
    own static-fanout probe (`None` when none), which `run_routing_flow`
    folds into the routing-stage config so the actual routing run's fan-out
    access agrees with the probe's.

    Every interstage crossing of the benchmark is computed from its topology
    metadata, realized as a pre-wired crossing grid instance placed in the
    layer's free band, and the interstage nets are split into crossing-free
    stubs. Downstream stages see an ordinary schematic and layout.
    """
    from benchmark_metadata import load_benchmark_metadata
    from translation.preplaced_crossing_grids import (
        build_crossing_plan_for_benchmark,
        derive_preplaced_crossing_layout,
        preplaced_crossing_grid_metrics,
    )

    print(f"\n[2b/{total_steps}] Placing topology-derived crossing grids...")
    t_start = time.perf_counter()
    metadata = load_benchmark_metadata(benchmark_name, schematic=schematic)
    crossing_plan = build_crossing_plan_for_benchmark(schematic, metadata)
    derived = derive_preplaced_crossing_layout(
        schematic,
        unrouted_layout,
        crossing_plan,
        router_probe_kwargs=router_probe_kwargs,
        config=config.crossing_grid,
    )
    metrics = preplaced_crossing_grid_metrics(derived)
    if derived.placed_crossing_count != derived.expected_crossing_count:
        raise RuntimeError(
            "pre-placed crossing grids realized "
            f"{derived.placed_crossing_count} crossing component(s) but the topology "
            f"expects {derived.expected_crossing_count}"
        )
    elapsed = time.perf_counter() - t_start
    if stats is not None:
        stats.step_times_s["preplaced_crossing_grids"] = elapsed
    print(
        f"      - Grids: {metrics['grid_count']} placed, "
        f"{metrics['crossing_component_count']} crossing component(s) "
        f"(topology expects {metrics['expected_crossing_count']}), "
        f"{metrics['split_net_count']} interstage net(s) split into stubs; "
        f"{len(derived.schematic.netlist.routes)} route(s) to route ({elapsed:.2f}s)"
    )
    return (
        derived.schematic,
        derived.unrouted_layout,
        {"preplaced_crossing_grids": metrics},
        frozenset(derived.router_fallback_net_names),
        derived.dense_fanout_instances,
    )


def route_benchmark(config: RoutingConfig, options: FlowOptions) -> Component:
    """Load the benchmark named in `options.loading` and route it.

    This is the human entry point underneath the command line (D6): the flow
    header, the benchmark load, the flow timer and the footer, with every
    routing stage in `route_schematic`.
    """
    benchmark_name = options.loading.benchmark_name
    stats = options.stats.stats
    debug_svgs, _show_klayout = resolve_legacy_display_options(
        debug_svgs=options.debug.debug_svgs,
        show_klayout=options.output.show_klayout,
        show_unrouted=options.output.show_unrouted,
        show_routed=options.output.show_routed,
        show_debug_svgs=options.debug.show_debug_svgs,
        show_static_obstacles_svg=options.debug.show_static_obstacles_svg,
    )
    debug_svgs_enabled, _debug_route_indices = parse_debug_svg_selector(debug_svgs)
    total_steps = 4 if options.electrical.enable_electrical_routing else 3

    _print_flow_header(benchmark_name)

    if stats is not None:
        stats.benchmark_name = benchmark_name

    t_flow_start = time.perf_counter()

    if debug_svgs_enabled:
        cleanup_debug_artifacts(benchmark_name)

    schematic = _load_benchmark_stage(
        benchmark_name=benchmark_name,
        total_steps=total_steps,
        stats=stats,
        debug_meanders=options.debug.debug_meanders,
    )
    routed_layout = route_schematic(schematic, config, options)

    if options.debug.debug_timing:
        t_end = time.perf_counter()
        total = t_end - t_flow_start
        print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")
    if stats is not None:
        total = time.perf_counter() - t_flow_start
        stats.total_time_s = float(total)

    _print_flow_footer()

    return routed_layout


def route_schematic(
    schematic: Schematic, config: RoutingConfig, options: FlowOptions
) -> Component:
    """Route one already-loaded schematic: every stage after the load.

    Keep the body below as the stage outline; detailed behavior belongs in the
    stage modules and the small helpers above.
    """
    benchmark_name = options.loading.benchmark_name
    stats = options.stats.stats
    debug = options.debug
    optical = options.optical
    obstacles = options.obstacles
    debug_svgs, show_klayout = resolve_legacy_display_options(
        debug_svgs=debug.debug_svgs,
        show_klayout=options.output.show_klayout,
        show_unrouted=options.output.show_unrouted,
        show_routed=options.output.show_routed,
        show_debug_svgs=debug.show_debug_svgs,
        show_static_obstacles_svg=debug.show_static_obstacles_svg,
    )
    debug_svgs_enabled, debug_route_indices = parse_debug_svg_selector(debug_svgs)
    total_steps = 4 if options.electrical.enable_electrical_routing else 3

    route_static_obstacle_config, waveguide_clearance_um, heater_clearance_um = (
        build_static_obstacle_config(
            static_obstacle_config=obstacles.static_obstacle_config,
            grid_size_um=obstacles.grid_size_um,
            waveguide_clearance_um=obstacles.waveguide_clearance_um,
            heater_clearance_um=obstacles.heater_clearance_um,
            obstacle_clearance_um=obstacles.obstacle_clearance_um,
            chip_add_x_um=obstacles.chip_add_x_um,
            chip_add_y_um=obstacles.chip_add_y_um,
        )
    )
    debug_dir, route_debug_indices = debug_artifact_routing_options(
        debug_svgs_enabled=debug_svgs_enabled,
        debug_route_indices=debug_route_indices,
        collect_attempt_diagnostics=debug.collect_attempt_diagnostics,
    )

    unrouted_layout = _layout_from_schematic_stage(
        schematic=schematic,
        total_steps=total_steps,
        stats=stats,
        debug_timing=debug.debug_timing,
    )
    # The stages below negotiate the crossing configuration between them, so
    # these four start as the option values and are then rebound.
    enable_crossings = optical.enable_crossings
    crossing_mode = optical.crossing_mode
    preplaced_crossing_grids = options.preplaced_grids.preplaced_crossing_grids
    net_order = optical.net_order
    if net_order is None:
        net_order = default_net_order(
            preplaced_crossing_grids=preplaced_crossing_grids,
            guided=bool(enable_crossings) and is_guided_mode(crossing_mode),
        )
    if preplaced_crossing_grids:
        # Contribution 2 (2026-09-17, multiportmmi_128x128): the fan-in /
        # fan-out nets of dense multi-port instances route without the
        # long-straight congestion penalty so that their lanes can pack in
        # parallel; every other net keeps the benchmark's weight. Part of the
        # configuration since the ADEPT 128x128 run that only converged this
        # way; `PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT=0` in the
        # shell keeps the old behaviour.
        if config.search.long_straight_exempt_dense_fanout is None:
            config = replace(
                config,
                search=replace(config.search, long_straight_exempt_dense_fanout=True),
            )
    # Contribution 2 splits nets into tile stubs; keep the net order's depth
    # layers those of the original netlist (see route_rust.net_order_depth_by_node).
    net_order_depth_by_node: dict[str, int] | None = (
        _depth_by_node_from_schematic(schematic) if preplaced_crossing_grids else None
    )
    preplaced_report_metadata: dict[str, object] | None = None
    crossing_guidance_net_names: frozenset[str] | None = None
    if preplaced_crossing_grids:
        if is_guided_mode(crossing_mode):
            raise ValueError(
                "preplaced_crossing_grids (contribution 2) and crossing_mode "
                "'lidar-guided' (contribution 1) are alternative contributions; "
                "run exactly one of lidar-pure / lidar-guided / preplaced grids."
            )
        if enable_crossings:
            raise ValueError(
                "preplaced_crossing_grids and enable_crossings are mutually exclusive: "
                "pre-placed grids resolve every crossing before routing, so the router "
                "must run with crossings disabled."
            )
        (
            schematic,
            unrouted_layout,
            preplaced_report_metadata,
            crossing_guidance_net_names,
            dense_fanout_instances,
        ) = _preplaced_crossing_grids_stage(
            benchmark_name=benchmark_name,
            schematic=schematic,
            unrouted_layout=unrouted_layout,
            total_steps=total_steps,
            stats=stats,
            # The column grid puts its entry tiles on the router's static
            # fan-out anchors, so the probe must see the routing run's
            # configuration.
            router_probe_kwargs={
                "obstacle_config": route_static_obstacle_config,
                "fanout_access_mode": optical.fanout_access_mode,
                "bend_radius_um": options.layout.bend_radius_um,
                "include_heater_obstacles": obstacles.include_heater_obstacles,
                "config": config,
            },
            config=config,
        )
        # The stage's own static-fanout probe computed which source
        # instances get the 2-port dense-fanout threshold (see
        # `_derive_crossing_tiles`); the actual routing run must agree, so
        # PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES's old self-set/self-clear
        # environment mutation becomes this explicit config fold-in.
        config = replace(
            config, fanout=replace(config.fanout, dense_fanout_instances=dense_fanout_instances)
        )
        if crossing_guidance_net_names:
            # Mixed run: tiled layers stay crossing-free, the fallback layers
            # get contribution 1's guided search restricted to their nets.
            enable_crossings = True
            crossing_mode = "lidar-guided"
            print(
                "      - crossing structure: router fallback for "
                f"{len(crossing_guidance_net_names)} net(s) -> crossings enabled "
                "(lidar-guided, guidance restricted to those nets)"
            )
    record_initial_route_stats(stats)
    optical_config = build_optical_routing_stage_config(
        enable_path_length_matching=options.path_length.enable_path_length_matching,
        path_length_match_outputs=options.path_length.path_length_match_outputs,
        path_length_meander_height_um=options.path_length.path_length_meander_height_um,
        enable_crossings=enable_crossings,
        crossing_mode=crossing_mode,
        crossing_guidance_net_names=crossing_guidance_net_names,
        crossing_half_size_cells=optical.crossing_half_size_cells,
        min_straight_cells_per_crossing=optical.min_straight_cells_per_crossing,
        foreign_port_keepout_cells=optical.foreign_port_keepout_cells,
        fanout_access_mode=optical.fanout_access_mode,
        proactive_congestion_weight=optical.proactive_congestion_weight,
        proactive_congestion_radius_cells=optical.proactive_congestion_radius_cells,
        allow_45_degree_turns=options.layout.allow_45_degree_turns,
        bend_radius_um=options.layout.bend_radius_um,
        enable_jps4=optical.enable_jps4,
        use_indexed_heap=optical.use_indexed_heap,
        enable_simple_routes=optical.enable_simple_routes,
        primitive_ordering=optical.primitive_ordering,
        heuristic_mode=optical.heuristic_mode,
        net_order=net_order,
        net_order_depth_by_node=net_order_depth_by_node,
        heap_tie_breaker=optical.heap_tie_breaker,
        max_iterations=optical.max_iterations,
        routing_window_scale=optical.routing_window_scale,
        include_heater_obstacles=obstacles.include_heater_obstacles,
        ripup_reroute_config=optical.ripup_reroute_config,
        route_static_obstacle_config=route_static_obstacle_config,
        debug_dir=debug_dir,
        route_debug_indices=route_debug_indices,
        debug_stop_after_route_index=debug.debug_stop_after_route_index,
        debug_timing=debug.debug_timing,
        debug_meanders=debug.debug_meanders,
        verbose_routes=debug.verbose_routes,
        debug_svgs_enabled=debug_svgs_enabled,
        collect_route_stats=options.stats.collect_route_stats,
        collect_attempt_diagnostics=debug.collect_attempt_diagnostics,
        stats=stats,
        config=config,
    )
    optical_result = run_photonic_routing_stage(
        benchmark_name=benchmark_name,
        schematic=schematic,
        unrouted_layout=unrouted_layout,
        total_steps=total_steps,
        config=optical_config,
    )
    route_result = optical_result.route_result
    routed_layout = optical_result.routed_layout
    debug_artifacts = optical_result.debug_artifacts
    if stats is not None:
        populate_route_stats(
            stats,
            route_result=route_result,
            debug_artifacts=debug_artifacts,
            route_summary=optical_result.route_summary,
            route_attempt_records=optical_result.route_attempt_records,
            route_time_s=optical_result.route_time_s,
        )
    electrical_result: ElectricalRoutingResult | None = None

    t_verify_start = time.perf_counter()
    verify_and_attach_photonic_reports(
        benchmark_name=benchmark_name,
        schematic=schematic,
        unrouted_layout=unrouted_layout,
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        include_heater_obstacles=obstacles.include_heater_obstacles,
        debug_stop_after_route_index=debug.debug_stop_after_route_index,
        extra_report_metadata=preplaced_report_metadata,
        write_gds_on_photonic_verification_failure=(
            config.write_gds_on_photonic_verification_failure
        ),
    )
    if debug.debug_timing:
        # Performance pass 2026-09-03 (P5): the non-routing phases were an
        # un-itemised 16 % of a multiportmmi_32x32 run.
        print(f"      - Verification time: {time.perf_counter() - t_verify_start:.4f} s")

    t_plm_start = time.perf_counter()
    attach_and_report_path_length_matching(
        routed_layout=routed_layout,
        route_result=route_result,
        debug_meanders=debug.debug_meanders,
    )
    if debug.debug_timing:
        print(
            f"      - Path-length-matching report time: {time.perf_counter() - t_plm_start:.4f} s"
        )

    if options.electrical.enable_electrical_routing:
        routed_layout, electrical_result = run_electrical_routing_step(
            benchmark_name=benchmark_name,
            schematic=schematic,
            routed_layout=routed_layout,
            electrical_config=options.electrical.electrical_config,
            debug_dir=debug_dir,
            total_steps=total_steps,
            stats=stats,
            debug_timing=debug.debug_timing,
            debug_svgs_enabled=debug_svgs_enabled,
        )

    if debug_svgs_enabled:
        report_and_open_debug_svgs(
            debug_artifacts=debug_artifacts,
            electrical_result=electrical_result,
            debug_route_indices=debug_route_indices,
        )

    t_write_start = time.perf_counter()
    write_or_show_routed_layout(
        benchmark_name=benchmark_name,
        routed_layout=routed_layout,
        show_klayout=show_klayout,
    )
    if debug.debug_timing:
        print(f"      - GDS write time: {time.perf_counter() - t_write_start:.4f} s")

    return routed_layout


def run_routing_flow(
    benchmark_name: str,
    *,
    debug_svgs: DebugSvgSelector = False,
    show_unrouted: bool | None = None,
    show_routed: bool | None = None,
    show_debug_svgs: DebugSvgSelector | None = None,
    show_static_obstacles_svg: bool | None = None,
    debug_timing: bool = False,
    debug_stop_after_route_index: int | None = None,
    debug_meanders: bool = False,
    verbose_routes: bool = False,
    show_klayout: bool = False,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    path_length_meander_height_um: float = SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    enable_crossings: bool = False,
    crossing_mode: str = "lidar-pure",
    preplaced_crossing_grids: bool = False,
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    fanout_access_mode: str | None = SCRIPT_FANOUT_ACCESS_MODE,
    proactive_congestion_weight: float = SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
    proactive_congestion_radius_cells: int = SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    allow_45_degree_turns: bool = SCRIPT_ALLOW_45_DEGREE_TURNS,
    bend_radius_um: float = SCRIPT_BEND_RADIUS_UM,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    net_order: str | None = None,
    heap_tie_breaker: str = "smaller_g",
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    include_heater_obstacles: bool = False,
    grid_size_um: float = SCRIPT_GRID_SIZE_UM,
    waveguide_clearance_um: float | None = None,
    heater_clearance_um: float | None = None,
    obstacle_clearance_um: float | None = None,
    chip_add_x_um: float = SCRIPT_CHIP_ADD_X_UM,
    chip_add_y_um: float = SCRIPT_CHIP_ADD_Y_UM,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    static_obstacle_config: StaticObstacleMapConfig | None = None,
    enable_electrical_routing: bool = False,
    electrical_config: ElectricalRoutingConfig | None = None,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    stats: RoutingFlowStats | None = None,
    config: RoutingConfig | None = None,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'heater_s_mod').
        debug_svgs: If True or "all", generate all debug SVGs into build/.
                    If a selector such as "5-10", "5", or "2,5-10" is
                    provided, generate only matching per-route SVGs by
                    1-based net order. Static obstacle SVGs are still
                    generated when debug SVGs are enabled.
        debug_timing: If True, print timing information for each stage.
        debug_meanders: If True, print verbose path-length and meander
                      insertion details when path-length matching is enabled.
        verbose_routes: If True, print per-net routing progress and whether
                      each route used the simple router or A*.
        show_klayout: If True, open the final routed layout in KLayout via
                      `Component.show()`.
        show_unrouted: Legacy alias. Currently unused (kept for compatibility).
        show_routed: Legacy alias for `show_klayout`.
        show_debug_svgs: Legacy alias for `debug_svgs`.
        show_static_obstacles_svg: Legacy alias for enabling debug SVG output.
        stats: Optional legacy stats collector. If provided, step metrics are
               populated in-place.
        enable_path_length_matching: If True, run post-route path-length
                      analysis and compute per-edge missing lengths.
        path_length_match_outputs: If True, add output-arrival equalization
                      requirements after local path-length matching.
        path_length_meander_height_um: Maximum meander height used when
                      inserting path-length matching meanders.
        crossing_half_size_cells: Crossing keepout half-size in grid cells.
                      The default 0 derives it from the crossing component bbox.
        crossing_mode: Crossing routing mode. "lidar-pure" uses dynamic
                      DRC-style crossing permission against any committed route
                      (the baseline); "lidar-guided" is contribution 1:
                      lidar-pure mechanics plus the precomputed topology
                      crossings as soft search guidance.
                      Exactly one of lidar-pure / lidar-guided /
                      preplaced_crossing_grids (contribution 2) runs at a time.
        preplaced_crossing_grids: If True, derive every interstage layer's
                      crossings from the benchmark topology, place one
                      pre-wired crossing grid per layer into the layout before
                      routing, split each interstage net into two crossing-free
                      stubs to/from the grid, and route with crossings disabled.
                      Mutually exclusive with enable_crossings. See
                      translation/preplaced_crossing_grids.py.
        net_order: Net routing order within the topological layers (see
                      translation/route_order.py). None selects the
                      configuration's default: "topological-span" with
                      preplaced_crossing_grids (planar stubs), "topological"
                      otherwise.
        min_straight_cells_per_crossing: Minimum straight access length on each
                      side of a crossing in grid cells.
        fanout_access_mode: Dense multi-port access strategy passed to the Rust
                      routing bridge. Use "static-stubs" to pre-spread
                      same-element fanout ports as fixed static breakout
                      geometry and route from virtual anchor ports.
        proactive_congestion_weight: Soft A* cost per blocked side-neighbor
                      cell beside straight moves.
        proactive_congestion_radius_cells: Sideways grid radius used for
                      proactive congestion counting.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        bend_radius_um: Minimum optical waveguide bend radius. Rounded up to
                      the active routing grid before primitive generation.
        use_indexed_heap: Benchmark-only indexed-heap experiment. Pass 8E
            measured it slower than duplicate-entry BinaryHeap queueing, so
            the default remains False.
        enable_simple_routes: If False, force optical nets through A* by
            disabling straight/L/Z simple-route candidates.
        primitive_ordering: Benchmark-only dense A* primitive ordering
            experiment. Pass 8F keeps "library" as the default.
        heuristic_mode: Dense A* heuristic experiment.
        heap_tie_breaker: Benchmark-only dense A* heap tie-breaker experiment.
            "smaller_g" preserves historical behavior; "larger_g" favors
            deeper states on equal f-score plateaus.
        max_iterations: Maximum A* state expansions per route attempt.
        routing_window_scale: Optional A* routing-window margin scale. If None,
                      the Rust AStarConfig default is used.
        include_heater_obstacles: If True, include configured heater/metal
                      layers as static optical-routing obstacles. Component-
                      specific optical port openings still apply when their
                      component rules match.
        grid_size_um: Optical routing grid resolution in micrometers.
        waveguide_clearance_um: Static clearance in micrometers for existing
                      optical/waveguide obstacles.
        heater_clearance_um: Static clearance in micrometers for heater/metal
                      obstacles. Defaults to the waveguide clearance.
        obstacle_clearance_um: Deprecated alias for waveguide_clearance_um.
        chip_add_x_um: Extra horizontal chip margin added to both left and
                      right when the die bbox is computed automatically.
        chip_add_y_um: Extra vertical chip margin added to both bottom and top
                      when the die bbox is computed automatically.
        static_obstacle_config: Optional obstacle builder config. If omitted,
            strict bounding-box static obstacles are used.
        enable_electrical_routing: If True, run the electrical heater-metal
            routing stage after optical routing and return/write/show the
            electrically routed component.
        electrical_config: Optional electrical routing configuration. If
            omitted, `ElectricalRoutingConfig()` defaults are used.
        collect_route_stats: If True, collect route-search counters without
            printing debug timing. This is enabled automatically when stats is
            provided.
        collect_attempt_diagnostics: If True, collect extra per-attempt window,
            obstacle-density, and ripup diagnostics for slow/failed attempts.
        config: Milestone 1's typed Python-side configuration tree
            (`photonic_router.config.RoutingConfig`), replacing every
            `PHOTONIC_ROUTER_*` variable `translation/` and this module used
            to read directly. When omitted, `RoutingConfig.from_environment()`
            is used, so every caller that passes nothing keeps today's
            behavior.

    Returns:
        The routed layout component.
    """
    # Compatibility wrapper (Milestone 5, Slice 3): group the keywords into the
    # option tree and call the entry point. The one piece of behaviour that
    # lives here and not in `route_benchmark` is the historical default for a
    # caller that passes no configuration at all, which still comes from the
    # environment overlay (D4); `route_benchmark` itself is handed a built one.
    return route_benchmark(
        config if config is not None else RoutingConfig.from_environment(),
        FlowOptions(
            loading=LoadingOptions(benchmark_name=benchmark_name),
            layout=LayoutOptions(
                allow_45_degree_turns=allow_45_degree_turns,
                bend_radius_um=bend_radius_um,
            ),
            obstacles=ObstacleOptions(
                static_obstacle_config=static_obstacle_config,
                grid_size_um=grid_size_um,
                waveguide_clearance_um=waveguide_clearance_um,
                heater_clearance_um=heater_clearance_um,
                obstacle_clearance_um=obstacle_clearance_um,
                chip_add_x_um=chip_add_x_um,
                chip_add_y_um=chip_add_y_um,
                include_heater_obstacles=include_heater_obstacles,
            ),
            preplaced_grids=PreplacedCrossingGridOptions(
                preplaced_crossing_grids=preplaced_crossing_grids,
            ),
            optical=OpticalStageOptions(
                enable_crossings=enable_crossings,
                crossing_mode=crossing_mode,
                crossing_half_size_cells=crossing_half_size_cells,
                min_straight_cells_per_crossing=min_straight_cells_per_crossing,
                foreign_port_keepout_cells=foreign_port_keepout_cells,
                fanout_access_mode=fanout_access_mode,
                proactive_congestion_weight=proactive_congestion_weight,
                proactive_congestion_radius_cells=proactive_congestion_radius_cells,
                enable_jps4=enable_jps4,
                use_indexed_heap=use_indexed_heap,
                enable_simple_routes=enable_simple_routes,
                primitive_ordering=primitive_ordering,
                heuristic_mode=heuristic_mode,
                net_order=net_order,
                heap_tie_breaker=heap_tie_breaker,
                max_iterations=max_iterations,
                routing_window_scale=routing_window_scale,
                ripup_reroute_config=ripup_reroute_config,
            ),
            path_length=PathLengthMatchingOptions(
                enable_path_length_matching=enable_path_length_matching,
                path_length_match_outputs=path_length_match_outputs,
                path_length_meander_height_um=path_length_meander_height_um,
            ),
            electrical=ElectricalOptions(
                enable_electrical_routing=enable_electrical_routing,
                electrical_config=electrical_config,
            ),
            debug=DebugArtifactOptions(
                debug_svgs=debug_svgs,
                show_debug_svgs=show_debug_svgs,
                show_static_obstacles_svg=show_static_obstacles_svg,
                debug_timing=debug_timing,
                debug_stop_after_route_index=debug_stop_after_route_index,
                debug_meanders=debug_meanders,
                verbose_routes=verbose_routes,
                collect_attempt_diagnostics=collect_attempt_diagnostics,
            ),
            output=OutputOptions(
                show_klayout=show_klayout,
                show_routed=show_routed,
                show_unrouted=show_unrouted,
            ),
            stats=StatsOptions(collect_route_stats=collect_route_stats, stats=stats),
        ),
    )
