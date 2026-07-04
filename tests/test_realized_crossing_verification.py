from types import SimpleNamespace

from photonic_router.path_length_graph import PortRef
from translation.route_rust import _verify_realized_route_intersections
from translation.route_rust_types import RoutedNetRecord


def _record(name: str, centerline: tuple[tuple[float, float], ...]) -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name=name,
        source=PortRef(instance=f"{name}_src", port="o1"),
        target=PortRef(instance=f"{name}_dst", port="o1"),
        route_obj=SimpleNamespace(),
        total_length_um=0.0,
        corrected_centerline_um=centerline,
    )


def _plan_info() -> dict[str, object]:
    return {
        "enabled": True,
        "allow_only_expected_crossings": True,
        "crossing_half_size_cells": 2,
        "min_straight_cells_per_crossing": 2,
        "events": [
            {
                "loaded": True,
                "net_id_a": 1,
                "net_id_b": 2,
                "net_name_a": "a",
                "net_name_b": "b",
            }
        ],
    }


def test_realized_crossing_verifier_accepts_straight_expected_crossing():
    info = _plan_info()
    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info["illegal_realized_crossing_count"] == 0
    assert info["realized_intersections"][0]["classification"] == "legal_expected_crossing"


def test_realized_crossing_verifier_compresses_collinear_points():
    info = _plan_info()
    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (2.0, 5.0), (5.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 2.0), (5.0, 5.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info["illegal_realized_crossing_count"] == 0
    assert info["realized_intersections"][0]["segment_a_margin_um"] == 5.0
    assert info["realized_intersections"][0]["segment_b_margin_um"] == 5.0


def test_realized_crossing_verifier_rejects_expected_bend_crossing():
    info = _plan_info()
    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (5.0, 5.0), (5.0, 10.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["reason"] == "crossing_footprint_contains_bend"
    assert illegal[0]["classification"] == "illegal_unexpected_crossing"


def test_realized_crossing_verifier_uses_footprint_not_astar_straight_margin():
    info = _plan_info()
    info["min_straight_cells_per_crossing"] = 10
    info["crossing_device"] = {"component_bbox_um": [8.0, 8.0]}

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((1.0, 5.0), (9.0, 5.0))),
            2: _record("b", ((5.0, 1.0), (5.0, 9.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    crossing = info["realized_intersections"][0]
    assert crossing["classification"] == "legal_expected_crossing"
    assert crossing["segment_a_margin_um"] == 4.0
    assert crossing["segment_b_margin_um"] == 4.0
    assert crossing["crossing_footprint_um"] == 8.0
    assert crossing["required_margin_um"] == 4.0


def test_realized_crossing_verifier_rejects_overlapping_crossing_footprints():
    info: dict[str, object] = {
        "enabled": True,
        "allow_only_expected_crossings": True,
        "crossing_half_size_cells": 4,
        "min_straight_cells_per_crossing": 2,
        "crossing_device": {"component_bbox_um": [8.0, 8.0]},
        "events": [
            {
                "loaded": True,
                "net_id_a": 1,
                "net_id_b": 2,
                "net_name_a": "a",
                "net_name_b": "b",
            },
            {
                "loaded": True,
                "net_id_a": 3,
                "net_id_b": 4,
                "net_name_a": "c",
                "net_name_b": "d",
            },
        ],
    }

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((-5.0, 0.0), (5.0, 0.0))),
            2: _record("b", ((0.0, -5.0), (0.0, 5.0))),
            3: _record("c", ((1.0, 6.0), (11.0, 6.0))),
            4: _record("d", ((6.0, 1.0), (6.0, 11.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 2
    assert {item["reason"] for item in illegal} == {"crossing_footprint_overlap"}


def test_realized_crossing_verifier_rejects_route_inside_crossing_footprint():
    info = _plan_info()
    info["crossing_device"] = {"component_bbox_um": [8.0, 8.0]}

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
            3: _record("c", ((1.0, 7.0), (4.0, 7.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["reason"] == "crossing_footprint_contains_route_geometry"
    assert illegal[0]["crossing_footprint_blockers"][0]["net_id"] == 3
