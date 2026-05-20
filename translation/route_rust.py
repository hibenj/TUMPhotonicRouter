"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance
from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    PathLengthAnalysisResult,
    PortRef,
    RoutedEdgeKey,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]
    routed_edge_lengths_um: dict[RoutedEdgeKey, float]
    routed_net_records: list["RoutedNetRecord"] = field(default_factory=list)
    realization_grid_spec: tuple[int, int, float, float, float] | None = None
    realization_allow_45_degree_turns: bool = True
    realization_bend_radius_cells: int = 4


@dataclass(frozen=True)
class RoutedNetRecord:
    net_name: str
    source: PortRef
    target: PortRef
    route_obj: object
    total_length_um: float


@dataclass(frozen=True)
class RouteRustPipelineResult:
    routed_layout: Component
    debug_artifacts: RustRouteDebugArtifacts
    path_length_analysis_info: dict[str, object] | None = None
    meander_requirements_info: list[dict[str, object]] | None = None


def routed_net_records_to_edge_lengths(
    records: list[RoutedNetRecord],
) -> dict[RoutedEdgeKey, float]:
    """Convert routed net records into edge-length annotations."""
    return {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): float(record.total_length_um)
        for record in records
    }


def analyze_path_length_matching(
    schematic: Schematic,
    *,
    routed_net_records: list[RoutedNetRecord],
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
) -> tuple[PathLengthAnalysisResult, list]:
    """Phase M1: compute per-edge missing lengths before polygon realization."""
    graph = build_graph_from_schematic(
        schematic,
        node_types=node_types,
        internal_delays_um=internal_delays_um,
    )
    annotate_edge_lengths(graph, routed_net_records_to_edge_lengths(routed_net_records))
    analysis = graph.analyze_missing_lengths()
    return analysis, list(list_edges_requiring_meander(analysis))


def edge_key_to_dict(edge_key: RoutedEdgeKey) -> dict[str, object]:
    return {
        "net_name": edge_key.net_name,
        "source": {"instance": edge_key.source.instance, "port": edge_key.source.port},
        "target": {"instance": edge_key.target.instance, "port": edge_key.target.port},
    }


def requirement_to_dict(req: MissingLengthRequirement) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(req.edge_key),
        "missing_length_um": float(req.missing_length_um),
    }


def analysis_to_info_dict(analysis: PathLengthAnalysisResult) -> dict[str, object]:
    return {
        "topological_order": list(analysis.topological_order),
        "node_arrival_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_um.items()
        },
        "edge_missing_lengths_um": [
            {
                "edge": edge_key_to_dict(edge_key),
                "missing_length_um": float(missing),
            }
            for edge_key, missing in analysis.edge_missing_lengths_um.items()
        ],
        "requirements": [requirement_to_dict(req) for req in analysis.requirements],
    }


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    enable_path_length_matching: bool = False,
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    debug_timing: bool = False,
) -> RouteRustPipelineResult:
    """Run Phase A->(optional M1)->B entirely in route_rust."""
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        obstacle_config=obstacle_config,
        debug_dir=debug_dir,
        debug_prefix=debug_prefix,
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        max_iterations=max_iterations,
        debug_timing=debug_timing,
        defer_realization=True,
    )

    analysis_info = None
    requirements_info = None
    if enable_path_length_matching:
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=debug_artifacts.routed_net_records,
            node_types=node_types,
            internal_delays_um=internal_delays_um,
        )
        analysis_info = analysis_to_info_dict(analysis)
        requirements_info = [requirement_to_dict(req) for req in requirements]

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    realize_routed_net_records(
        routed_layout,
        debug_artifacts.routed_net_records,
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )

    return RouteRustPipelineResult(
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        path_length_analysis_info=analysis_info,
        meander_requirements_info=requirements_info,
    )


def realize_routed_net_records(
    routed_layout: Component,
    routed_net_records: list[RoutedNetRecord],
    *,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool = True,
    bend_radius_cells: int = 4,
) -> None:
    """Phase B: realize routed records into polygons on the target layout."""
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    width, height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    for record in routed_net_records:
        polygon = router.realize_route_polygon(record.route_obj, float(route_width_um))
        routed_layout.add_polygon(polygon, layer=route_layer)


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    debug_timing: bool = False,
    defer_realization: bool = False,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets using Rust A* and add one polygon per routed net.

    This function routes each net by:
    1. Building a static obstacle map from the unrouted layout.
    2. Calling the Rust A* router for each net.
    3. Realizing one closed polygon in Rust and inserting it with add_polygon.
    4. Updating blocked cells for subsequent nets using width-aware inflation.

    Parameters:
        unrouted_layout: Component with placed instances but no routes.
        schematic: Schematic with net definitions.
        obstacle_config: Optional obstacle-map configuration.
        debug_dir: Directory where debug SVGs are written when provided.
        debug_prefix: Prefix used for debug SVG filenames.
        route_width_um: Realized waveguide width in micrometers.
        route_layer: Target GDS layer/datatype tuple for route polygons.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        max_iterations: Maximum A* state expansions per route attempt.
        defer_realization: If True, keep routed RouteResult objects but skip
            polygon realization. This is used for pre-realization transforms
            such as path-length matching/meander insertion.

    Returns:
        A tuple of (routed_layout, debug_artifacts).
        :param debug_timing:
    """
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    t_obstacle_start = 0.0
    if debug_timing:
        t_obstacle_start = time.perf_counter()
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=obstacle_config)
    if debug_timing:
        t_obstacle_end = time.perf_counter()
        print(f"      - Obstacle Map time: {t_obstacle_end - t_obstacle_start:.4f} s")
    grid = obstacle_map.grid

    debug_path = Path(debug_dir) if debug_dir is not None else None
    obstacle_svg = None
    route_svgs: list[Path] = []
    routed_edge_lengths_um: dict[RoutedEdgeKey, float] = {}
    routed_net_records: list[RoutedNetRecord] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)

    nets = schematic.netlist.routes
    print(f"\nRouting {len(nets)} nets using Rust router...")

    if not hasattr(rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    origin_x_um, origin_y_um = _grid_origin_xy(grid)
    grid_spec = rust_backend.GridSpec(
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid.grid_size_um),
        bend_radius_cells=4,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(max_iterations))
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    block_radius_cells = max(
        0, math.ceil((float(route_width_um) / 2.0) / float(grid.grid_size_um))
    )
    router.set_static_cells(sorted(obstacle_map.blocked_cells))
    net_id = 0

    def _orientation_to_angle(orientation: float | None, *, flip: bool = False) -> int:
        if orientation is None:
            angle = 0
        else:
            angle = int(round((float(orientation) % 360.0) / 45.0)) % 8

        if flip:
            angle = (angle + 4) % 8

        return angle


    def _angle_to_step(angle: int) -> tuple[int, int]:
        steps = [
            (1, 0),    # 0 east
            (1, 1),    # 1 northeast
            (0, 1),    # 2 north
            (-1, 1),   # 3 northwest
            (-1, 0),   # 4 west
            (-1, -1),  # 5 southwest
            (0, -1),   # 6 south
            (1, -1),   # 7 southeast
        ]
        return steps[angle % 8]


    def port_to_grid_state(
            port: Port,
            grid_origin_x_um: float,
            grid_origin_y_um: float,
            grid_size_um: float,
            *,
            as_target: bool = False,
            outward_cells: int = 1,
    ):
        port_angle = _orientation_to_angle(port.orientation, flip=False)

        # For choosing the grid cell, always move outward from the physical port.
        # This avoids starting inside the real component/port geometry.
        sx, sy = _angle_to_step(port_angle)

        x = float(port.center[0]) + sx * outward_cells * grid_size_um
        y = float(port.center[1]) + sy * outward_cells * grid_size_um

        gx = int((x - grid_origin_x_um) // grid_size_um)
        gy = int((y - grid_origin_y_um) // grid_size_um)

        # For the route state angle:
        # - source: route leaves the port outward
        # - target: route approaches the port, so flip direction
        route_angle = _orientation_to_angle(port.orientation, flip=as_target)

        return rust_backend.State(gx, gy, route_angle)

    t_astar_start = 0.0
    if debug_timing:
        t_astar_start = time.perf_counter()

    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")

            source_port = get_port_from_instance(routed_layout, inst1, port1)
            target_port = get_port_from_instance(routed_layout, inst2, port2)

            source_state = port_to_grid_state(
                source_port,
                origin_x_um,
                origin_y_um,
                float(grid.grid_size_um),
                as_target=False,
            )
            target_state = port_to_grid_state(
                target_port,
                origin_x_um,
                origin_y_um,
                float(grid.grid_size_um),
                as_target=True,
            )

            print(f"  Routing {net_name}: {port1_spec} -> {port2_spec}...", end=" ")

            net_id += 1
            opened_cells = sorted(
                {
                    (int(source_state.x), int(source_state.y)),
                    (int(target_state.x), int(target_state.y)),
                }
            )
            try:
                route_obj = router.route_single_net_and_commit(
                    net_id,
                    source_state,
                    target_state,
                    block_radius_cells,
                    opened_cells,
                )
            except RuntimeError as exc:
                raise RuntimeError(
                    f"No route found for {net_name}: {port1_spec} -> {port2_spec}. "
                    f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                    f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                    f"allow_45_degree_turns={allow_45_degree_turns}"
                ) from exc

            edge_key = RoutedEdgeKey(
                net_name=net_name,
                source=PortRef(instance=inst1, port=port1),
                target=PortRef(instance=inst2, port=port2),
            )
            routed_net_records.append(
                RoutedNetRecord(
                    net_name=net_name,
                    source=edge_key.source,
                    target=edge_key.target,
                    route_obj=route_obj,
                    total_length_um=float(route_obj.total_length_um),
                )
            )
            routed_edge_lengths_um[edge_key] = float(route_obj.total_length_um)

            if debug_path is not None:
                route_dir = debug_path / "routes"
                _ensure_dir(route_dir)
                route_svg = route_dir / f"{debug_prefix}_{net_name}.svg"
                route_svg.write_text(router.export_debug_svg(route_obj), encoding="utf-8")
                route_svgs.append(route_svg)

            print("ok")

    if debug_timing:
        t_astar_end = time.perf_counter()
        print(f"      - Astar time: {t_astar_end - t_astar_start:.4f} s")

    realization_grid_spec = (
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    if not defer_realization:
        realize_routed_net_records(
            routed_layout,
            routed_net_records,
            route_width_um=route_width_um,
            route_layer=route_layer,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=4,
        )

    return routed_layout, RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        routed_edge_lengths_um=routed_edge_lengths_um,
        routed_net_records=routed_net_records,
        realization_grid_spec=realization_grid_spec,
        realization_allow_45_degree_turns=allow_45_degree_turns,
        realization_bend_radius_cells=4,
    )
