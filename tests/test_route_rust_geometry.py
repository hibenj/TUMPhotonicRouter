from routing_flow import load_benchmark, run_routing_flow
from translation.route_rust_geometry import _physical_point_to_grid_cell
from translation.route_rust_records import _grid_cell_center_um
from typing import Any, Iterable, Protocol


class _RouteWithCompressedWaypoints(Protocol):
    compressed_waypoints: Iterable[Any]


def test_physical_point_to_grid_cell_preserves_floor_snap_and_invalid_guards():
    assert _physical_point_to_grid_cell(
        (-0.51, 2.49),
        grid_size_um=0.5,
        origin_x_um=-1.0,
        origin_y_um=2.0,
    ) == (0, 0)
    assert _physical_point_to_grid_cell(
        (-5.01, -3.01),
        grid_size_um=2.0,
        origin_x_um=-5.0,
        origin_y_um=-3.0,
    ) == (-1, -1)

    assert (
        _physical_point_to_grid_cell(
            (0.0, 0.0),
            grid_size_um=0.0,
            origin_x_um=0.0,
            origin_y_um=0.0,
        )
        is None
    )
    assert (
        _physical_point_to_grid_cell(
            (float("nan"), 0.0),
            grid_size_um=1.0,
            origin_x_um=0.0,
            origin_y_um=0.0,
        )
        is None
    )


def test_grid_cell_center_um_preserves_none_and_negative_origin_cases():
    assert _grid_cell_center_um(
        (2, 2),
        grid_size_um=0.5,
        origin_x_um=-1.0,
        origin_y_um=2.0,
    ) == (0.25, 3.25)
    assert _grid_cell_center_um(
        (2, 2),
        grid_size_um=2.0,
        origin_x_um=-5.0,
        origin_y_um=-3.0,
    ) == (0.0, 2.0)
    assert (
        _grid_cell_center_um(
            None,
            grid_size_um=2.0,
            origin_x_um=-5.0,
            origin_y_um=-3.0,
        )
        is None
    )


def test_rust_routed_layout_uses_waveguide_geometry():
    # heater_s_mod routes under the current rules; TOY does not (placement
    # clearance shortage at an MMI input, 2026-08-18). Retargeted 2026-09-22.
    schematic = load_benchmark("heater_s_mod")
    layout = run_routing_flow(
        "heater_s_mod",
        show_unrouted=False,
        show_routed=False,
        show_debug_svgs=False,
    )

    # The old square-cell renderer produced hundreds of references per route.
    # The waveguide translation adds no instance per route: the routed layout
    # holds the schematic's instances and the routes as polygons.
    assert len(layout.insts) <= len(schematic.netlist.instances)
    assert len(layout.get_polygons(merge=False, by="tuple")) > 0


