from __future__ import annotations

from types import SimpleNamespace

import pytest
from gdsfactory.component import Component
from photonic_router.path_length_graph import PortRef

from translation.crossing_verification_report import build_crossing_verification_report
from translation.route_rust import (
    _augment_insertion_loss_report_from_realized_intersections,
    _place_realized_crossing_components,
    _verify_realized_route_intersections,
)
from translation.route_rust_types import RoutedNetRecord
from translation.routing.crossing_plan_info import CrossingPlanInfo


def _record(
    name: str,
    centerline: tuple[tuple[float, float], ...],
    *,
    opened_cells: tuple[tuple[int, int], ...] = (),
    source_port_center_um: tuple[float, float] | None = None,
    target_port_center_um: tuple[float, float] | None = None,
) -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name=name,
        source=PortRef(instance=f"{name}_src", port="o1"),
        target=PortRef(instance=f"{name}_dst", port="o1"),
        route_obj=SimpleNamespace(),
        total_length_um=0.0,
        corrected_centerline_um=centerline,
        opened_cells=opened_cells,
        source_port_center_um=source_port_center_um,
        target_port_center_um=target_port_center_um,
    )


def _plan_info() -> CrossingPlanInfo:
    return CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "crossing_half_size_cells": 2,
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
    )


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
    assert info.illegal_realized_crossing_count == 0
    assert info.realized_intersections[0]["classification"] == "legal_expected_crossing"


def test_realized_crossing_verifier_allows_lidar_pure_unexpected_crossing():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 10.0), (20.0, 10.0))),
            2: _record("b", ((10.0, 0.0), (10.0, 20.0))),
        },
        realization_grid_spec=(24, 24, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    crossing = info.realized_intersections[0]
    assert crossing["expected_pair"] is False
    assert crossing["classification"] == "legal_unexpected_crossing"


def test_realized_crossing_verifier_rejects_lidar_pure_non_perpendicular_crossing():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 10.0), (20.0, 10.0))),
            2: _record("b", ((10.0, 0.0), (14.0, 20.0))),
        },
        realization_grid_spec=(24, 24, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "illegal_unexpected_crossing"
    assert crossing["reason"] == "not_perpendicular"
    assert crossing["perpendicular"] is False


def test_realized_crossing_verifier_uses_route_waypoint_fallback():
    info = _plan_info()
    chord_only = RoutedNetRecord(
        net_name="a",
        source=PortRef(instance="a_src", port="o1"),
        target=PortRef(instance="a_dst", port="o1"),
        route_obj=SimpleNamespace(compressed_waypoints=((0, 5), (10, 5))),
        total_length_um=10.0,
        net_id=1,
        corrected_centerline_um=(),
    )

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: chord_only,
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info.illegal_realized_crossing_count == 0
    assert info.routes_missing_corrected_centerline_count == 0
    assert info.realized_intersections[0]["classification"] == "legal_expected_crossing"


def test_realized_crossing_verifier_rejects_collinear_route_overlap():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (12.0, 5.0))),
            2: _record("b", ((4.0, 5.0), (16.0, 5.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["reason"] == "collinear_route_overlap"
    assert illegal[0]["classification"] == "illegal_unexpected_crossing"
    assert illegal[0]["overlap_start_um"] == [4.0, 5.0]
    assert illegal[0]["overlap_end_um"] == [12.0, 5.0]


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
    assert info.illegal_realized_crossing_count == 0
    assert info.realized_intersections[0]["segment_a_margin_um"] == 5.0
    assert info.realized_intersections[0]["segment_b_margin_um"] == 5.0


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

    reasons = {item["reason"] for item in illegal}
    assert reasons == {"crossing_footprint_contains_bend", "collinear_route_overlap"}
    assert {item["classification"] for item in illegal} == {"illegal_unexpected_crossing"}


def test_realized_crossing_verifier_reports_search_margin_from_half_size_and_runout():
    info = _plan_info()
    info.bend_runout_cells_per_crossing = 3
    info.crossing_device = {"component_bbox_um": [8.0, 8.0]}

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((1.0, 5.0), (9.0, 5.0))),
            2: _record("b", ((5.0, 1.0), (5.0, 9.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert illegal == []
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "legal_expected_crossing"
    assert "reason" not in crossing
    assert crossing["segment_a_margin_um"] == 4.0
    assert crossing["segment_b_margin_um"] == 4.0
    assert crossing["crossing_footprint_um"] == 8.0
    assert crossing["required_margin_um"] == 4.0
    assert crossing["search_required_margin_um"] == 7.0


def test_realized_crossing_verifier_rejects_overlapping_crossing_footprints():
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "crossing_half_size_cells": 4,
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
    )

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((-8.0, 0.0), (8.0, 0.0))),
            2: _record("b", ((0.0, -8.0), (0.0, 8.0))),
            3: _record("c", ((-2.0, 6.0), (14.0, 6.0))),
            4: _record("d", ((6.0, -2.0), (6.0, 14.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    reasons = {item["reason"] for item in illegal}
    assert "crossing_footprint_overlap" in reasons


def test_realized_crossing_verifier_rejects_lidar_pure_overlapping_crossing_footprints():
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "crossing_mode": "lidar-pure",
            "crossing_half_size_cells": 4,
            "crossing_device": {"component_bbox_um": [8.0, 8.0]},
            "events": [],
        }
    )

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((-8.0, 0.0), (8.0, 0.0))),
            2: _record("b", ((0.0, -8.0), (0.0, 8.0))),
            3: _record("c", ((1.0, 7.0), (15.0, 7.0))),
            4: _record("d", ((7.0, 1.0), (7.0, 15.0))),
        },
        realization_grid_spec=(30, 30, 1.0, 0.0, 0.0),
    )

    reasons = {item["reason"] for item in illegal}
    assert "crossing_footprint_overlap" in reasons
    assert {item.get("footprint_overlap_policy") for item in info.realized_intersections} == {None}


def test_realized_crossing_verifier_rejects_route_inside_crossing_footprint():
    info = _plan_info()
    info.crossing_device = {"component_bbox_um": [8.0, 8.0]}

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((-5.0, 5.0), (15.0, 5.0))),
            2: _record("b", ((5.0, -5.0), (5.0, 15.0))),
            3: _record("c", ((1.0, 7.0), (4.0, 7.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["reason"] == "crossing_footprint_contains_route_geometry"
    assert illegal[0]["crossing_footprint_blockers"][0]["net_id"] == 3


def test_realized_crossing_verifier_rejects_lidar_pure_footprint_blocker():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.crossing_device = {"component_bbox_um": [8.0, 8.0]}
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((-5.0, 5.0), (15.0, 5.0))),
            2: _record("b", ((5.0, -5.0), (5.0, 15.0))),
            3: _record("c", ((1.0, 7.0), (4.0, 7.0))),
        },
        realization_grid_spec=(20, 20, 2.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "illegal_unexpected_crossing"
    assert crossing["reason"] == "crossing_footprint_contains_route_geometry"
    assert crossing["crossing_footprint_blockers"][0]["net_id"] == 3


def test_realized_crossing_verifier_ignores_route_endpoint_access():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 0.0), (10.0, 10.0))),
            2: _record(
                "b",
                ((5.0, 5.0), (5.0, 12.0)),
            ),
        },
        realization_grid_spec=(20, 20, 2.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info.ignored_endpoint_access_intersection_count == 1
    assert info.ignored_endpoint_access_intersections[0]["reason"] == ("route_endpoint_access")


def test_realized_crossing_verifier_does_not_treat_a_midroute_opened_cell_as_endpoint_access():
    """A perpendicular, straight crossing of two nets without a planned event is
    legal (every pair may cross since the whitelist went); an opened cell in
    the middle of a route is not an endpoint access and excuses nothing."""
    info = _plan_info()
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record(
                "a",
                ((0.0, 50.0), (100.0, 50.0)),
                opened_cells=((25, 25),),
            ),
            2: _record("b", ((50.0, 0.0), (50.0, 100.0))),
        },
        realization_grid_spec=(80, 80, 2.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info.ignored_endpoint_access_intersection_count == 0
    assert [item["classification"] for item in info.realized_intersections] == [
        "legal_unexpected_crossing"
    ]


def test_realized_crossing_verifier_rejects_opened_cell_crossing_near_endpoint():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.crossing_device = {"component_bbox_um": [8.0, 8.0]}
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record(
                "a",
                ((0.0, 0.0), (20.0, 20.0)),
                opened_cells=((4, 4),),
            ),
            2: _record("b", ((6.0, 0.0), (12.0, 20.0))),
        },
        realization_grid_spec=(30, 30, 2.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["classification"] == "illegal_unexpected_crossing"
    assert illegal[0]["reason"] == "not_perpendicular"
    assert info.ignored_endpoint_access_intersection_count == 0


def test_realized_crossing_verifier_rejects_opened_cell_crossing_near_internal_bend():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.crossing_device = {"component_bbox_um": [8.0, 8.0]}
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record(
                "a",
                ((0.0, 20.0), (40.0, -20.0)),
                opened_cells=((5, 4),),
            ),
            2: _record("b", ((0.0, 0.0), (9.9, 9.9), (20.0, 14.0))),
        },
        realization_grid_spec=(30, 30, 2.0, 0.0, 0.0),
    )

    assert len(illegal) == 1
    assert illegal[0]["classification"] == "illegal_unexpected_crossing"
    assert illegal[0]["perpendicular"] is False
    assert illegal[0]["reason"] == "crossing_footprint_contains_bend"
    assert info.ignored_endpoint_access_intersection_count == 0


def test_realized_crossing_verifier_ignores_port_center_access():
    info = _plan_info()
    info.crossing_mode = "lidar-pure"
    info.events = []

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 0.0), (10.0, 10.0))),
            2: _record(
                "b",
                ((5.0, 3.5), (5.0, 5.0), (5.0, 12.0)),
                source_port_center_um=(5.0, 5.0),
            ),
        },
        realization_grid_spec=(20, 20, 2.0, 0.0, 0.0),
    )

    assert illegal == []
    assert info.ignored_endpoint_access_intersections[0]["reason"] == ("route_endpoint_access")


def test_places_active_crossing_components_for_legal_realized_intersections():
    layout = Component("crossing_component_placement")
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "realized_intersections": [
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [10.0, 20.0],
                    "segment_a_um": [[0.0, 10.0], [20.0, 30.0]],
                    "segment_b_um": [[0.0, 30.0], [20.0, 10.0]],
                    "net_id_a": 1,
                    "net_id_b": 2,
                    "net_name_a": "a",
                    "net_name_b": "b",
                }
            ],
        }
    )

    placements = _place_realized_crossing_components(layout, info)

    assert len(layout.insts) == 1
    assert len(placements) == 1
    placement = placements[0]
    assert placement["point_um"] == [10.0, 20.0]
    assert placement["center_um"] == [10.0, 20.0]
    assert placement["optical_center_um"] == [10.0, 20.0]
    assert placement["rotation_deg"] == pytest.approx(45.0)
    assert placement["component_name"]
    assert placement["instance_name"]
    ref = next(iter(layout.insts))
    port_center_x = sum(float(port.center[0]) for port in ref.ports) / len(ref.ports)
    port_center_y = sum(float(port.center[1]) for port in ref.ports) / len(ref.ports)
    assert port_center_x == pytest.approx(10.0)
    assert port_center_y == pytest.approx(20.0)
    assert info.realized_crossing_components == placements
    assert info.realized_crossing_component_count == 1
    assert layout.info["realized_crossing_components"] == placements

    info.crossing_device = {
        "component_name": placement["component_name"],
        "component_bbox_um": placement.get("component_bbox_um"),
    }
    report = build_crossing_verification_report(
        crossing_plan_info=info.to_dict(),
        realized_crossing_components=placements,
    )
    assert report.success, report.as_dict()


def test_places_one_component_for_allowed_degraded_overlap_cluster():
    layout = Component("crossing_component_cluster_placement")
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "realized_intersections": [
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [10.0, 20.0],
                    "segment_a_um": [[0.0, 20.0], [20.0, 20.0]],
                    "segment_b_um": [[10.0, 10.0], [10.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 2,
                    "net_name_a": "a",
                    "net_name_b": "b",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [1, 2],
                },
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [12.0, 20.0],
                    "segment_a_um": [[2.0, 20.0], [22.0, 20.0]],
                    "segment_b_um": [[12.0, 10.0], [12.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 3,
                    "net_name_a": "a",
                    "net_name_b": "c",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [0],
                },
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [14.0, 20.0],
                    "segment_a_um": [[4.0, 20.0], [24.0, 20.0]],
                    "segment_b_um": [[14.0, 10.0], [14.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 4,
                    "net_name_a": "a",
                    "net_name_b": "d",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [0],
                },
            ],
        }
    )

    placements = _place_realized_crossing_components(layout, info)

    assert len(layout.insts) == 1
    assert len(placements) == 1
    assert placements[0]["source_crossing_index"] == 0
    assert placements[0]["shared_crossing_indices"] == [0, 1, 2]


def test_places_one_component_for_transitive_degraded_overlap_cluster():
    layout = Component("crossing_component_transitive_cluster")
    footprint = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "realized_intersections": [
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [10.0, 20.0],
                    "segment_a_um": [[0.0, 20.0], [20.0, 20.0]],
                    "segment_b_um": [[10.0, 10.0], [10.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 2,
                    "net_name_a": "a",
                    "net_name_b": "b",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [1],
                    "crossing_footprint_polygon_um": footprint,
                },
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [12.0, 20.0],
                    "segment_a_um": [[2.0, 20.0], [22.0, 20.0]],
                    "segment_b_um": [[12.0, 10.0], [12.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 3,
                    "net_name_a": "a",
                    "net_name_b": "c",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [0, 2],
                },
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [14.0, 20.0],
                    "segment_a_um": [[4.0, 20.0], [24.0, 20.0]],
                    "segment_b_um": [[14.0, 10.0], [14.0, 30.0]],
                    "net_id_a": 1,
                    "net_id_b": 4,
                    "net_name_a": "a",
                    "net_name_b": "d",
                    "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                    "overlapping_crossing_indices": [1],
                },
            ],
        }
    )

    placements = _place_realized_crossing_components(layout, info)

    assert len(layout.insts) == 1
    assert len(placements) == 1
    assert placements[0]["source_crossing_index"] == 0
    assert placements[0]["shared_crossing_indices"] == [0, 1, 2]
    assert placements[0]["crossing_footprint_polygon_um"] == footprint


def test_insertion_loss_report_uses_final_realized_intersections():
    info = CrossingPlanInfo.from_dict(
        {
            "enabled": True,
            "crossing_loss": 0.2,
            "realized_intersections": [
                {
                    "classification": "legal_unexpected_crossing",
                    "point_um": [5.0, 5.0],
                    "net_id_a": 1,
                    "net_id_b": 2,
                    "net_name_a": "a",
                    "net_name_b": "b",
                }
            ],
        }
    )

    _augment_insertion_loss_report_from_realized_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
            3: _record("c", ((0.0, 0.0), (1.0, 0.0))),
        },
    )

    by_net_id = {int(entry["net_id"]): entry for entry in info.insertion_loss_by_net}
    assert info.insertion_loss_model["crossing_count_source"] == ("realized_intersections")
    assert info.insertion_loss_summary["total_crossing_count"] == 2
    assert by_net_id[1]["crossing_count"] == 1
    assert by_net_id[2]["crossing_count"] == 1
    assert by_net_id[3]["crossing_count"] == 0
    assert by_net_id[1]["crossing_loss"] == 0.2


# --- phase 4 slice B: the verifier reports the Rust verdicts under the switch ---


def _rust_verdict(**overrides: object) -> dict[str, object]:
    """A `PyPhotonicRouter.crossing_verdicts()` record: net 2 (the route,
    vertical) crossing net 1 (the partner, horizontal) at (5, 5) um on a
    1 um grid, half size 2 and run-out 1 (margin 3, after 2), every straight
    run 5 cells, footprint clear, event registered."""
    verdict: dict[str, object] = {
        "net_id": 2,
        "partner_net_id": 1,
        "point": (5.0, 5.0),
        "route_segment": ((5, 0), (5, 10)),
        "partner_segment": ((0, 5), (10, 5)),
        "legal": True,
        "reason": "legal",
        "margin_side": None,
        "perpendicular": True,
        "route_before_cells": 5.0,
        "route_after_cells": 5.0,
        "partner_before_cells": 5.0,
        "partner_after_cells": 5.0,
        "required_margin_cells": 3,
        "required_after_cells": 2,
        "required_spacing_cells": 5,
        "nearest_same_route_cells": None,
        "nearest_registered_cells": None,
        "too_close_other": None,
        "footprint_blocked": False,
        "event_registered": True,
        "margins_from_grid": True,
        "grid_size_um": 1.0,
    }
    verdict.update(overrides)
    return verdict


def _unified_plan_info() -> CrossingPlanInfo:
    info = _plan_info()
    info.bend_runout_cells_per_crossing = 1
    return info


def test_unified_verifier_reports_the_rust_verdict_instead_of_deciding():
    info = _unified_plan_info()
    verdict = _rust_verdict()

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[verdict],
        unified_legality=True,
    )

    assert illegal == []
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "legal_expected_crossing"
    assert "reason" not in crossing
    assert crossing["legality_source"] == "rust_crossing_verdict"
    assert crossing["perpendicular"] is True
    # One margin, (half_size + bend_runout) * grid, reported as both fields.
    assert crossing["required_margin_um"] == 3.0
    assert crossing["search_required_margin_um"] == 3.0
    # Net a (1) is the verdict's partner, net b (2) its route: 5 cells each.
    assert crossing["segment_a_margin_um"] == 5.0
    assert crossing["segment_b_margin_um"] == 5.0
    assert crossing["rust_verdict"]["reason"] == "legal"
    assert crossing["rust_verdict"]["event_registered"] is True
    # Slice B-2: the spacing inputs travel with the verdict.
    assert crossing["rust_verdict"]["required_spacing_cells"] == 5
    assert crossing["rust_verdict"]["too_close_other"] is None
    # The footprint polygon is still built from the geometry for the
    # component placement.
    assert len(crossing["crossing_footprint_polygon_um"]) == 4
    assert crossing["crossing_footprint_blockers"] == []


@pytest.mark.parametrize(
    ("reason", "margin_side"),
    [
        ("not_perpendicular", None),
        ("margin_short_route_before", "route_before"),
        ("margin_short_route_after", "route_after"),
        ("margin_short_partner", "partner"),
        ("too_close_same_route", None),
        ("too_close_registered", None),
        ("footprint_blocked", None),
        ("missing_event", None),
    ],
)
def test_unified_verifier_classification_equals_the_rust_verdict(reason, margin_side):
    info = _unified_plan_info()
    verdict = _rust_verdict(legal=False, reason=reason, margin_side=margin_side)

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[verdict],
        unified_legality=True,
    )

    assert len(illegal) == 1
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "illegal_unexpected_crossing"
    assert crossing["reason"] == reason
    assert crossing["rust_verdict"]["margin_side"] == margin_side
    assert crossing["legality_source"] == "rust_crossing_verdict"


def test_unified_verifier_does_not_retest_the_geometry():
    # A visibly non-perpendicular pair in the Python geometry, but Rust's
    # verdict (matched by pair and point, within 1.5 cells) says legal and
    # perpendicular: the verifier reports Rust's verdict.
    info = _unified_plan_info()
    verdict = _rust_verdict(point=(5.4, 5.0))

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (7.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[verdict],
        unified_legality=True,
    )

    assert illegal == []
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "legal_expected_crossing"
    assert crossing["perpendicular"] is True

    # Switch off: the same inputs are re-decided from the geometry, as before.
    legacy_info = _unified_plan_info()
    legacy_illegal = _verify_realized_route_intersections(
        crossing_plan_info=legacy_info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (7.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[verdict],
        unified_legality=False,
    )
    assert len(legacy_illegal) == 1
    legacy_crossing = legacy_info.realized_intersections[0]
    assert legacy_crossing["reason"] == "not_perpendicular"
    assert "legality_source" not in legacy_crossing


def test_unified_verifier_calls_an_intersection_without_a_rust_verdict_missing_event():
    info = _unified_plan_info()

    illegal = _verify_realized_route_intersections(
        crossing_plan_info=info,
        routed_records_by_net_id={
            1: _record("a", ((0.0, 5.0), (10.0, 5.0))),
            2: _record("b", ((5.0, 0.0), (5.0, 10.0))),
        },
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[],
        unified_legality=True,
    )

    assert len(illegal) == 1
    crossing = info.realized_intersections[0]
    assert crossing["classification"] == "illegal_unexpected_crossing"
    assert crossing["reason"] == "missing_event"
    assert crossing["legality_source"] == "no_rust_crossing_verdict"


def test_unified_native_events_path_reports_the_verdict_and_unmatched_verdicts():
    from translation.route_rust_crossing_verification import (
        _populate_realized_intersections_from_native_crossing_events,
    )

    info = _unified_plan_info()
    event = {
        "net_id": 2,
        "partner_net_id": 1,
        "point": (5.0, 5.0),
        "route_segment": ((5, 0), (5, 10)),
        "partner_segment": ((0, 5), (10, 5)),
        "route_angle": 2,
        "partner_angle": 0,
        "reservation_cells": [],
    }
    short = _rust_verdict(
        net_id=3,
        partner_net_id=1,
        point=(12.0, 5.0),
        legal=False,
        reason="margin_short_partner",
        margin_side="partner",
        partner_after_cells=1.0,
        event_registered=False,
    )

    illegal = _populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=info,
        routed_records_by_net_id={},
        native_crossing_events=[event],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[_rust_verdict(), short],
        unified_legality=True,
    )

    assert len(info.realized_intersections) == 2
    matched, unmatched = info.realized_intersections
    assert matched["classification"] == "legal_native_crossing"
    assert matched["legality_source"] == "rust_crossing_verdict"
    assert matched["required_margin_um"] == 3.0
    assert matched["search_required_margin_um"] == 3.0
    assert matched["rust_verdict"]["reason"] == "legal"
    assert unmatched["source"] == "native_crossing_verdicts"
    assert unmatched["classification"] == "illegal_unexpected_crossing"
    assert unmatched["reason"] == "margin_short_partner"
    assert unmatched["segment_a_margin_um"] == 5.0  # net 3 is the route: 5 cells
    assert unmatched["segment_b_margin_um"] == 1.0  # net 1 is the partner: 1 cell
    assert illegal == [unmatched]

    # Switch off: the events are trusted as before and the verdicts ignored.
    legacy_info = _unified_plan_info()
    legacy_illegal = _populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=legacy_info,
        routed_records_by_net_id={},
        native_crossing_events=[event],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[_rust_verdict(), short],
        unified_legality=False,
    )
    assert legacy_illegal == []
    assert len(legacy_info.realized_intersections) == 1
    assert "legality_source" not in legacy_info.realized_intersections[0]
