from routing_flow import load_benchmark, run_routing_flow
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from tests.fixtures.synthetic_layouts import (
    TIGHT_S_BEND_BEND_RADIUS_UM,
    TIGHT_S_BEND_OFFSET_UM,
    two_instance_s_bend_schematic,
)
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import route_match_and_realize
from translation.route_rust_geometry import _physical_point_to_grid_cell
from translation.route_rust_records import _grid_cell_center_um
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


def test_ten_um_bend_radius_does_not_backtrack_on_a_one_cell_short_s_bend():
    """The S-bend regression the TOY benchmark used to cover.

    Milestone 0 deleted `test_toy_ten_um_bend_radius_does_not_backtrack_on_one_cell_short_s_bend`
    with TOY because TOY cannot be routed; Milestone 6 Slice 3 rebuilds its
    subject as a synthetic two-instance layout. At a 10 um bend radius on the
    default 0.5 um grid, two 90-degree arcs consume the whole 20 um lateral
    offset, so the S-bend has no room for a straight between them -- the case
    where a router that mis-sizes the second arc backtracks in x. The route's
    compressed waypoints must stay monotone in x.
    """

    schematic = two_instance_s_bend_schematic()
    layout = layout_from_schematic(schematic)

    result = route_match_and_realize(
        layout,
        schematic,
        enable_path_length_matching=False,
        debug_dir=None,
        allow_45_degree_turns=False,
        bend_radius_um=TIGHT_S_BEND_BEND_RADIUS_UM,
        max_iterations=5_000_000,
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
    record = records_by_name["left_to_right"]
    route_obj = cast(_RouteWithCompressedWaypoints, record.route_obj)
    waypoints = [(int(point[0]), int(point[1])) for point in route_obj.compressed_waypoints]

    xs = [x for x, _ in waypoints]
    assert xs == sorted(xs), f"the S-bend backtracks in x: {waypoints}"
    # Two arcs and the two straights around them: start, first bend, second
    # bend, end.
    assert len(waypoints) == 4
    # The lateral travel really is the tight case: 20.0 um of offset at a
    # 0.5 um grid pitch is 40 cells, exactly twice the 20-cell bend radius,
    # so the two arcs meet with no straight between them.
    lateral_cells = waypoints[-1][1] - waypoints[0][1]
    assert lateral_cells == 40
    assert lateral_cells == round(TIGHT_S_BEND_OFFSET_UM / 0.5)
    assert record.corrected_centerline_um
