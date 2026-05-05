"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
build_static_obstacle_map = _sob.build_static_obstacle_map
grid_cell_center = _sob.grid_cell_center
physical_to_grid = _sob.physical_to_grid
_load_rust_backend = _sob._load_rust_backend

_primitive_lib_mod = importlib.import_module("photonic_router.primitive_library")
get_primitive_library = _primitive_lib_mod.get_primitive_library


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]


def _orientation_to_angle(orientation: float | None, *, flip: bool = False) -> int:
    if orientation is None:
        return 0
    normalized = orientation + (180.0 if flip else 0.0)
    normalized %= 360.0
    return int(round(normalized / 45.0)) % 8


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _place_primitives_from_result(
    layout: Component,
    grid: GridSpec,
    result: dict,
    abs_port1: Port,
    abs_port2: Port,
) -> None:
    """Place routed primitives into the layout using 1:1 mapping.

    Each primitive returned by Rust is placed directly as a gdsfactory component
    at the correct position and orientation, reconstructing the full waveguide path.

    Parameters:
        layout: Component to add routed waveguides to
        grid: GridSpec for coordinate conversion
        result: Rust routing result with "primitives" and "states"
        abs_port1: Source port
        abs_port2: Target port
    """
    primitives_used = result.get("primitives", [])
    states = result.get("states", [])

    if not primitives_used or not states:
        return

    prim_lib = get_primitive_library()

    # Place each primitive at corresponding position/orientation
    for i, prim_id in enumerate(primitives_used):
        if i >= len(states):
            break

        state = states[i]
        x_grid, y_grid, angle = int(state[0]), int(state[1]), int(state[2])

        # Convert grid coordinate to physical coordinate (cell center)
        x_um, y_um = grid_cell_center(x_grid, y_grid, grid)

        # Get primitive component and metadata
        prim_component = prim_lib.get_component(prim_id)
        if prim_component is None:
            continue

        # Create a rotated reference at the state position
        # Angle from Rust is 0-7 (45° increments), convert to degrees
        rotation_deg = angle * 45.0

        # Add reference with rotation and translation.
        # All-angle bend components may require off-grid placement in gdsfactory.
        add_ref = getattr(layout, "add_ref_off_grid", None)
        if add_ref is None:
            ref = layout.add_ref(prim_component)
        else:
            ref = add_ref(prim_component)
        ref.rotate(rotation_deg, center=(0, 0))
        ref.move((x_um, y_um))


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets using the Rust A* router with primitive library.

    This function routes each net by:
    1. Building a static obstacle map from the unrouted layout
    2. For each net, calling the Rust A* router
    3. Placing routed primitives (1:1 mapping) into the final layout

    Parameters:
        unrouted_layout: Component with placed instances but no routes.
        schematic: Schematic with net definitions.
        obstacle_config: Optional obstacle-map configuration.
        debug_dir: Directory where debug SVGs are written when provided.
        debug_prefix: Prefix used for debug SVG filenames.

    Returns:
        A tuple of (routed_layout, debug_artifacts).
    """
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    obstacle_map = build_static_obstacle_map(unrouted_layout, config=obstacle_config)
    grid = obstacle_map.grid
    blocked_cells = set(obstacle_map.blocked_cells)
    port_open_cells = set(obstacle_map.port_open_cells)

    debug_path = Path(debug_dir) if debug_dir is not None else None
    obstacle_svg = None
    route_svgs: list[Path] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)

    nets = schematic.netlist.routes
    print(f"\nRouting {len(nets)} nets using Rust router...")

    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")

            abs_port1 = get_port_from_instance(routed_layout, inst1, port1)
            abs_port2 = get_port_from_instance(routed_layout, inst2, port2)

            source = _port_to_state(abs_port1, grid, is_target=False)
            target = _port_to_state(abs_port2, grid, is_target=True)

            print(f"  Routing {net_name}: {port1_spec} -> {port2_spec}...", end=" ")

            result = rust_backend.route_single_net_rs(
                grid.width,
                grid.height,
                sorted(blocked_cells),
                sorted(port_open_cells),
                grid.grid_size_um,
                source,
                target,
                export_svg=debug_path is not None,
            )

            # Place primitives directly into layout using 1:1 mapping
            _place_primitives_from_result(
                routed_layout,
                grid,
                result,
                abs_port1,
                abs_port2,
            )

            blocked_cells.update(result["cells"])

            if debug_path is not None and "svg" in result:
                route_dir = debug_path / "routes"
                _ensure_dir(route_dir)
                route_svg = route_dir / f"{debug_prefix}_{net_name}.svg"
                route_svg.write_text(result["svg"], encoding="utf-8")
                route_svgs.append(route_svg)

            print("ok")

    return routed_layout, RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
    )


def _port_to_state(port: Port, grid: GridSpec, *, is_target: bool) -> tuple[int, int, int]:
    gx, gy = physical_to_grid(port.center[0], port.center[1], grid)
    angle = _orientation_to_angle(port.orientation, flip=is_target)
    return gx, gy, angle



