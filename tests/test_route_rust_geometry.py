from routing_flow import load_benchmark, run_routing_flow
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust_geometry import _physical_point_to_grid_cell
from translation.route_rust import route_match_and_realize
from translation.route_rust_records import _grid_cell_center_um
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from typing import Any, Iterable, Protocol, cast


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
    layout = run_routing_flow(
        "TOY",
        show_unrouted=False,
        show_routed=False,
        show_debug_svgs=False,
    )

    # The old square-cell renderer produced hundreds of references.
    # The new waveguide translation should keep the routed layout compact.
    assert len(layout.insts) <= 10
    assert len(layout.get_polygons(merge=False, by="tuple")) > 0


def test_toy_ten_um_bend_radius_does_not_backtrack_on_one_cell_short_s_bend():
    schematic = load_benchmark("TOY")
    layout = layout_from_schematic(schematic)

    result = route_match_and_realize(
        layout,
        schematic,
        enable_path_length_matching=False,
        debug_dir=None,
        allow_45_degree_turns=False,
        bend_radius_um=10.0,
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="bounding_boxes",
            clearance_um=3.0,
            heater_clearance_um=5.0,
            clear_port_open_cells_from_static=False,
        ),
    )

    records_by_name = {
        record.net_name: record for record in result.debug_artifacts.routed_net_records
    }
    for net_name in ("gc1_to_mmi_in2", "mmi_out2_to_gc3"):
        record = records_by_name[net_name]
        route_obj = cast(_RouteWithCompressedWaypoints, record.route_obj)
        waypoints = list(route_obj.compressed_waypoints)
        xs = [int(point[0]) for point in waypoints]
        assert xs == sorted(xs)
        assert len(waypoints) == 4
        assert record.corrected_centerline_um
