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
`route_benchmark`.
"""

from __future__ import annotations

import importlib
import time
from dataclasses import dataclass, replace
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RoutingConfig
from photonic_router.crossings.crossing_modes import is_guided_mode
from photonic_router.electrical import ElectricalRoutingResult
from photonic_router.flow_config import (
    build_static_obstacle_config,
    debug_artifact_routing_options,
    parse_debug_svg_selector,
    resolve_legacy_display_options,
)
from photonic_router.flow_electrical import run_electrical_routing_step
from photonic_router.flow_optical import (
    OpticalStageInputs,
    build_optical_routing_stage_config,
    run_photonic_routing_stage,
)
from photonic_router.flow_options import (
    FlowOptions,
)
from photonic_router.flow_plm import attach_and_report_path_length_matching
from photonic_router.flow_reporting import (
    cleanup_debug_artifacts,
    report_and_open_debug_svgs,
    write_or_show_routed_layout,
)
from photonic_router.flow_stats import (
    RoutingFlowStats,
    format_flow_summary_line,
    populate_route_stats,
    record_initial_route_stats,
)
from photonic_router.flow_verification import verify_and_attach_photonic_reports
from photonic_router.gds.route_overview_svg import (
    layout_overview_shapes,
    realized_extra_um_by_net,
    write_routes_overview_svg,
)
from photonic_router.placement_analysis import PlacementAnalysis, analyze_placement
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.records.route_order import default_net_order
from photonic_router.routing.options import (
    CrossingOptions,
    PortAccessOptions,
    RealizationOptions,
    SessionOptions,
)


@dataclass(frozen=True)
class _SchematicLink:
    """One link of the benchmark's netlist, as the instances it joins (for the depth)."""

    inst1: str
    inst2: str


def _depth_by_node_from_schematic(schematic: Schematic) -> dict[str, int]:
    """Instance depth (hops from a true source) on the benchmark's own
    netlist, before any contribution 2 splitting."""
    from photonic_router.records.route_order import depth_by_node_from_jobs

    jobs = []
    for bundle in schematic.netlist.routes.values():
        for port1_spec, port2_spec in bundle.links.items():
            inst1 = str(port1_spec).split(",")[0]
            inst2 = str(port2_spec).split(",")[0]
            jobs.append(_SchematicLink(inst1=inst1, inst2=inst2))
    return depth_by_node_from_jobs(jobs)


def _component_bbox_um(component: object) -> tuple[float, float, float, float] | None:
    """A gdsfactory component's bounding box as (xmin, ymin, xmax, ymax) in um, or None."""
    try:
        box = component.bbox()  # type: ignore[attr-defined]
    except Exception:
        return None
    return (float(box.left), float(box.bottom), float(box.right), float(box.top))


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

    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            f"Benchmark '{benchmark_name}' not found in benchmarks/ directory"
        ) from e
    except AttributeError as e:
        raise AttributeError(
            f"Benchmark '{benchmark_name}' must have a 'build_schematic()' function"
        ) from e
    else:
        return schematic


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
    print(f"      - Bounding box: {bbox() if callable(bbox) else bbox}")
    if debug_timing:
        print(f"      - Translation time: {step_layout_end - step_layout_start:.4f} s")
    return unrouted_layout


def _placement_analysis_stage(
    *,
    schematic: Schematic,
    unrouted_layout: Component,
    total_steps: int,
    stats: RoutingFlowStats | None,
    design: str = "placed design",
) -> tuple[PlacementAnalysis, float]:
    """Analyse a placed design once; the stages below read the result (`placement_analysis.py`).

    Returns the analysis and the seconds it took.
    """
    print(f"\n[2a/{total_steps}] Analysing the {design}...")
    t_start = time.perf_counter()
    analysis = analyze_placement(schematic, unrouted_layout)
    elapsed = time.perf_counter() - t_start
    if stats is not None:
        stats.step_times_s["placement_analysis"] = (
            stats.step_times_s.get("placement_analysis", 0.0) + elapsed
        )
    lines = [analysis.summary(), analysis.passage_summary()]
    print("\n".join(f"      - {line}" for line in lines if line is not None))
    return analysis, elapsed


def _print_flow_header(benchmark_name: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'=' * 60}")


def _print_flow_footer(stats: RoutingFlowStats | None = None) -> None:
    print(format_flow_summary_line(stats))
    print(f"\n{'=' * 60}\n")


def _preplaced_crossing_grids_stage(
    *,
    schematic: Schematic,
    unrouted_layout: Component,
    analysis: PlacementAnalysis,
    analysis_seconds: float,
    total_steps: int,
    stats: RoutingFlowStats | None,
    router_probe_options: SessionOptions | None = None,
    config: RoutingConfig,
) -> tuple[Schematic, Component, dict[str, object], frozenset[str], frozenset[str] | None]:
    """Replace the schematic/layout with the crossing-grid-derived pair.

    The fourth element names the nets of layers without a feasible pre-placed
    structure; the flow routes them with the guided crossing search. The
    fifth element is the dense-fanout-instance set the stage computed for its
    own static-fanout probe (`None` when none), which `route_schematic`
    folds into the routing-stage config so the actual routing run's fan-out
    access agrees with the probe's.

    Every interstage crossing of the benchmark comes from its placement
    (`analysis.crossing_demand`, the band demand), is realized as a pre-wired
    crossing grid instance placed in the layer's free band, and the interstage nets are split into
    crossing-free stubs. Downstream stages see an ordinary schematic and
    layout.

    The stage needs a band-layered design: every net confined to the band
    between two adjacent component columns. A design with nets outside that
    model (`CrossingDemand.require_band_layered`) is refused here with the
    nets and reasons, before anything is placed; those designs route with
    `--crossing-mode lidar-pure` or `lidar-guided`.
    """
    from photonic_router.crossings.preplaced_crossing_grids import (
        derive_preplaced_crossing_layout,
        preplaced_crossing_grid_metrics,
    )

    print(f"\n[2b/{total_steps}] Placing placement-derived crossing grids...")
    t_start = time.perf_counter()
    demand = analysis.crossing_demand
    try:
        demand.require_band_layered()
    except ValueError as exc:
        raise RuntimeError(
            f"pre-placed crossing grids need a band-layered design: {exc}; "
            "route with --crossing-mode lidar-pure or lidar-guided"
        ) from exc
    derived = derive_preplaced_crossing_layout(
        schematic,
        unrouted_layout,
        demand.stage_schedule(),
        router_probe_options=router_probe_options,
        config=config.crossing_grid,
    )
    metrics = preplaced_crossing_grid_metrics(derived)
    if derived.placed_crossing_count != derived.expected_crossing_count:
        raise RuntimeError(
            "pre-placed crossing grids realized "
            f"{derived.placed_crossing_count} crossing component(s) but the topology "
            f"expects {derived.expected_crossing_count}"
        )
    # The reported build time (`preplaced_build_s` of the result tables) has
    # always included the demand the tiles are placed from.
    elapsed = time.perf_counter() - t_start + analysis_seconds
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
    if options.stats.stats is None:
        # Every run ends with the SUMMARY line, so every run has a stats
        # record; per-route collection stays behind `collect_route_stats`.
        options = replace(options, stats=replace(options.stats, stats=RoutingFlowStats()))
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
    try:
        routed_layout = route_schematic(schematic, config, options)
        if options.debug.debug_timing:
            total = time.perf_counter() - t_flow_start
            print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")
    finally:
        # Every run ends with the SUMMARY line, a failed one too: the stages
        # that did not run print `-` (merge review 2026-10-05).
        if stats is not None:
            stats.total_time_s = float(time.perf_counter() - t_flow_start)
        _print_flow_footer(stats)

    return routed_layout


def route_schematic(schematic: Schematic, config: RoutingConfig, options: FlowOptions) -> Component:
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

    route_static_obstacle_config, _waveguide_clearance_um, _heater_clearance_um = (
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
    if preplaced_crossing_grids:  # noqa: SIM102 (the comment below is the inner condition's reason)
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
    # layers those of the original netlist (see `SearchOptions.net_order_depth_by_node`).
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
        original_analysis, original_analysis_seconds = _placement_analysis_stage(
            schematic=schematic,
            unrouted_layout=unrouted_layout,
            total_steps=total_steps,
            stats=stats,
        )
        (
            schematic,
            unrouted_layout,
            preplaced_report_metadata,
            crossing_guidance_net_names,
            dense_fanout_instances,
        ) = _preplaced_crossing_grids_stage(
            schematic=schematic,
            unrouted_layout=unrouted_layout,
            analysis=original_analysis,
            analysis_seconds=original_analysis_seconds,
            total_steps=total_steps,
            stats=stats,
            # The column grid puts its entry tiles on the router's static
            # fan-out anchors, so the probe must see the routing run's
            # configuration.
            router_probe_options=SessionOptions(
                port_access=PortAccessOptions(fanout_access_mode=optical.fanout_access_mode),
                realization=RealizationOptions(bend_radius_um=options.layout.bend_radius_um),
                obstacle_config=route_static_obstacle_config,
                include_heater_obstacles=obstacles.include_heater_obstacles,
                config=config,
            ),
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
    if options.path_length.enable_path_length_matching and enable_crossings:
        # Decision 10 (2026-11-05): not a target; every stable configuration
        # runs crossings off, the combination is unverified.
        print(
            "      - WARNING: path-length matching with the crossing-aware search on is "
            "not a supported combination; the stable configurations run --crossings false"
        )
    # The guided search (contribution 1, and contribution 2's router fallback
    # on its derived design) is the only reader of the analysis below this
    # point; the baseline must not see a plan and gets none.
    placement_analysis: PlacementAnalysis | None = None
    if enable_crossings and is_guided_mode(crossing_mode):
        placement_analysis, _seconds = _placement_analysis_stage(
            schematic=schematic,
            unrouted_layout=unrouted_layout,
            total_steps=total_steps,
            stats=stats,
            design="design with crossing tiles" if preplaced_crossing_grids else "placed design",
        )
    record_initial_route_stats(stats)
    optical_config = build_optical_routing_stage_config(
        options,
        config,
        OpticalStageInputs(
            crossings=CrossingOptions(
                enable_crossings=enable_crossings,
                crossing_mode=crossing_mode,
                crossing_half_size_cells=int(optical.crossing_half_size_cells),
                crossing_guidance_net_names=crossing_guidance_net_names,
                placement_analysis=placement_analysis,
            ),
            net_order=net_order,
            net_order_depth_by_node=net_order_depth_by_node,
            obstacle_config=route_static_obstacle_config,
            debug_dir=debug_dir,
            debug_route_indices=route_debug_indices,
            debug_svgs_enabled=debug_svgs_enabled,
        ),
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
        stats=stats,
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
        stats=stats,
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
        # The routes overview (route-overview plan, 2026-10-05): every routed
        # waveguide with its realized length (routed plus the meander drawn
        # on it, the bookkeeping by physical edge) and a legend with the
        # totals and the chip dimensions, next to the per-route SVGs.
        overview_path = Path("build") / "routes" / f"{benchmark_name.lower()}_overview.svg"
        instances, waveguide_polygons = layout_overview_shapes(optical_result.routed_layout)
        overview = write_routes_overview_svg(
            overview_path,
            benchmark_name=benchmark_name,
            records=list(getattr(debug_artifacts, "routed_net_records", []) or []),
            realized_extra_um_by_net=realized_extra_um_by_net(
                getattr(route_result, "meander_insertion_report_info", None)
            ),
            routable_bbox_um=getattr(debug_artifacts, "routable_bbox_um", None),
            layout_bbox_um=_component_bbox_um(optical_result.routed_layout),
            instances=instances,
            waveguide_polygons=waveguide_polygons,
        )
        print(
            f"      - Routes overview SVG: {overview_path} "
            f"({overview.route_count} routes, routed {overview.routed_length_um:.1f} um, "
            f"meander {overview.meander_length_um:.1f} um, "
            f"total {overview.total_length_um:.1f} um)"
        )
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
