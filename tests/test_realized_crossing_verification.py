from __future__ import annotations

from types import SimpleNamespace

import pytest
from gdsfactory.component import Component
from photonic_router.crossings.crossing_verification_report import (
    build_crossing_verification_report,
)
from photonic_router.crossings.route_rust_crossing_components import (
    place_realized_crossing_components,
)
from photonic_router.crossings.route_rust_crossing_plan import (
    augment_insertion_loss_report_from_realized_intersections,
)
from photonic_router.records.crossing_plan_info import CrossingPlanInfo
from photonic_router.records.route_keys import PortRef
from photonic_router.records.route_rust_types import RoutedNetRecord


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

    placements = place_realized_crossing_components(layout, info)

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

    placements = place_realized_crossing_components(layout, info)

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

    placements = place_realized_crossing_components(layout, info)

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

    augment_insertion_loss_report_from_realized_intersections(
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


# --- the verifier reports the Rust verdicts ---


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


def test_unified_native_events_path_reports_the_verdict_and_unmatched_verdicts():
    from photonic_router.crossings.route_rust_crossing_verification import (
        populate_realized_intersections_from_native_crossing_events,
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

    illegal = populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=info,
        routed_records_by_net_id={},
        native_crossing_events=[event],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        native_crossing_verdicts=[_rust_verdict(), short],
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
