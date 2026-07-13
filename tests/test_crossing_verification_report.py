import json
import math

from translation.crossing_verification_report import (
    RouteCostTerms,
    build_crossing_verification_report,
    route_cost_terms_from_mapping,
    write_crossing_verification_report,
)


def _plan_with_legal_crossing() -> dict[str, object]:
    return {
        "enabled": True,
        "crossing_device": {
            "component_name": "crossing",
            "component_bbox_um": [10.0, 10.0],
        },
        "realized_intersections": [
            {
                "classification": "legal_unexpected_crossing",
                "point_um": [5.0, 5.0],
                "segment_a_um": [[0.0, 5.0], [10.0, 5.0]],
                "segment_b_um": [[5.0, 0.0], [5.0, 10.0]],
                "net_id_a": 1,
                "net_id_b": 2,
                "net_name_a": "a",
                "net_name_b": "b",
            }
        ],
    }


def test_report_accepts_legal_crossing_with_matching_component(tmp_path):
    report = build_crossing_verification_report(
        crossing_plan_info=_plan_with_legal_crossing(),
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.01, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 0.0,
            }
        ],
        route_cost_terms=[
            RouteCostTerms(
                net_id=1,
                net_name="a",
                length_um=100.0,
                length_loss=0.1,
                bend_loss=0.02,
                crossing_loss=0.03,
                total_search_cost=3.15,
            )
        ],
    )

    assert report.success is True
    assert report.metrics["legal_crossing_count"] == 1
    assert report.metrics["matched_crossing_component_count"] == 1
    assert math.isclose(report.metrics["total_physical_insertion_loss"], 0.15)
    assert math.isclose(report.metrics["total_search_guidance_penalty"], 3.0)

    path = write_crossing_verification_report(
        report,
        tmp_path / "verification" / "crossings.json",
    )
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["success"] is True
    assert payload["crossings"][0]["classification"] == "legal_unexpected_crossing"
    assert math.isclose(payload["route_costs"][0]["search_guidance_penalty"], 3.0)


def test_report_flags_missing_crossing_component():
    report = build_crossing_verification_report(
        crossing_plan_info=_plan_with_legal_crossing(),
        realized_crossing_components=[],
    )

    assert report.success is False
    assert [issue.code for issue in report.issues] == ["missing_crossing_component"]
    assert report.metrics["intended_crossing_component_count"] == 1
    assert report.metrics["matched_crossing_component_count"] == 0


def test_report_allows_component_shared_by_allowed_degraded_cluster():
    plan = _plan_with_legal_crossing()
    first = plan["realized_intersections"][0]
    first["source_crossing_index"] = 0
    first["footprint_overlap_policy"] = "allowed_lidar_pure_degraded_cluster"
    first["overlapping_crossing_index"] = 1
    plan["realized_intersections"].append(
        {
            "classification": "legal_unexpected_crossing",
            "point_um": [7.0, 5.0],
            "segment_a_um": [[0.0, 5.0], [10.0, 5.0]],
            "segment_b_um": [[7.0, 0.0], [7.0, 10.0]],
            "net_id_a": 1,
            "net_id_b": 3,
            "net_name_a": "a",
            "net_name_b": "c",
            "source_crossing_index": 1,
            "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
            "overlapping_crossing_index": 0,
        }
    )

    report = build_crossing_verification_report(
        crossing_plan_info=plan,
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.0, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 0.0,
                "source_crossing_index": 0,
            }
        ],
    )

    assert report.success is True
    assert report.metrics["intended_crossing_component_count"] == 2
    assert report.metrics["matched_crossing_component_count"] == 1


def test_report_allows_component_shared_by_transitive_degraded_cluster():
    plan = _plan_with_legal_crossing()
    first = plan["realized_intersections"][0]
    first["source_crossing_index"] = 0
    first["footprint_overlap_policy"] = "allowed_lidar_pure_degraded_cluster"
    first["overlapping_crossing_indices"] = [1]
    plan["realized_intersections"].extend(
        [
            {
                "classification": "legal_unexpected_crossing",
                "point_um": [7.0, 5.0],
                "segment_a_um": [[0.0, 5.0], [10.0, 5.0]],
                "segment_b_um": [[7.0, 0.0], [7.0, 10.0]],
                "net_id_a": 1,
                "net_id_b": 3,
                "net_name_a": "a",
                "net_name_b": "c",
                "source_crossing_index": 1,
                "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                "overlapping_crossing_indices": [0, 2],
            },
            {
                "classification": "legal_unexpected_crossing",
                "point_um": [9.0, 5.0],
                "segment_a_um": [[0.0, 5.0], [10.0, 5.0]],
                "segment_b_um": [[9.0, 0.0], [9.0, 10.0]],
                "net_id_a": 1,
                "net_id_b": 4,
                "net_name_a": "a",
                "net_name_b": "d",
                "source_crossing_index": 2,
                "footprint_overlap_policy": "allowed_lidar_pure_degraded_cluster",
                "overlapping_crossing_indices": [1],
            },
        ]
    )

    report = build_crossing_verification_report(
        crossing_plan_info=plan,
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.0, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 0.0,
                "source_crossing_index": 0,
                "shared_crossing_indices": [0, 1, 2],
            }
        ],
    )

    assert report.success is True
    assert report.metrics["intended_crossing_component_count"] == 3
    assert report.metrics["matched_crossing_component_count"] == 1


def test_report_flags_misaligned_crossing_component_rotation():
    report = build_crossing_verification_report(
        crossing_plan_info=_plan_with_legal_crossing(),
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.0, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 90.0,
            }
        ],
    )

    assert report.success is False
    assert [issue.code for issue in report.issues] == [
        "crossing_component_rotation_mismatch"
    ]


def test_report_rejects_degraded_non_perpendicular_component_crossing():
    plan = _plan_with_legal_crossing()
    crossing = plan["realized_intersections"][0]
    crossing["segment_b_um"] = [[5.0, 0.0], [9.0, 10.0]]
    crossing["perpendicular"] = False
    crossing["degraded_reason"] = "not_perpendicular"

    report = build_crossing_verification_report(
        crossing_plan_info=plan,
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.0, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 0.0,
            }
        ],
    )

    assert report.success is False
    assert [issue.code for issue in report.issues] == ["illegal_realized_crossing"]
    assert report.issues[0].details["reason"] == "not_perpendicular"


def test_report_flags_protected_segment_movement():
    report = build_crossing_verification_report(
        crossing_plan_info={"enabled": True},
        protected_segments=[
            {
                "net_id": 7,
                "net_name": "protected",
                "start_um": [0.0, 0.0],
                "end_um": [10.0, 0.0],
                "realized_start_um": [0.0, 0.0],
                "realized_end_um": [10.5, 0.0],
                "tolerance_um": 0.01,
            }
        ],
    )

    assert report.success is False
    assert report.issues[0].code == "protected_segment_moved"
    assert report.metrics["protected_segment_issue_count"] == 1
    assert math.isclose(report.issues[0].details["max_endpoint_error_um"], 0.5)


def test_report_preserves_illegal_realized_crossing_reason():
    report = build_crossing_verification_report(
        crossing_plan_info={
            "enabled": True,
            "realized_intersections": [
                {
                    "classification": "illegal_unexpected_crossing",
                    "reason": "crossing_footprint_contains_bend",
                    "point_um": [4.0, 4.0],
                    "net_name_a": "a",
                    "net_name_b": "b",
                }
            ],
        },
    )

    assert report.success is False
    assert report.metrics["illegal_crossing_count"] == 1
    assert report.issues[0].code == "illegal_realized_crossing"
    assert report.issues[0].details["reason"] == "crossing_footprint_contains_bend"


def test_route_cost_terms_separate_physical_loss_from_guidance():
    terms = route_cost_terms_from_mapping(
        {
            "net_id": 3,
            "net_name": "n3",
            "length_um": 200.0,
            "length_loss": 0.2,
            "bend_loss": 0.05,
            "crossing_loss": 0.1,
            "history_cost": 1.0,
            "congestion_cost": 2.0,
            "total_search_cost": 3.35,
        }
    )

    assert math.isclose(terms.total_physical_insertion_loss, 0.35)
    assert math.isclose(terms.search_guidance_penalty, 3.0)
    assert terms.as_dict()["history_cost"] == 1.0


def test_report_counts_final_repair_attempts():
    plan = _plan_with_legal_crossing()
    plan["final_crossing_repair_attempts"] = [
        {"status": "routed"},
        {"status": "skipped"},
    ]
    plan["final_photonic_repair_attempts"] = [
        {"status": "routed"},
        {"status": "routed"},
        {"status": "routed", "endpoint_correction_failed_net_ids": [9]},
    ]

    report = build_crossing_verification_report(
        crossing_plan_info=plan,
        realized_crossing_components=[
            {
                "component_name": "crossing",
                "point_um": [5.0, 5.0],
                "component_bbox_um": [10.0, 10.0],
                "rotation_deg": 0.0,
            }
        ],
    )

    assert report.metrics["final_crossing_repair_attempt_count"] == 2
    assert report.metrics["final_crossing_repair_success_count"] == 1
    assert report.metrics["final_photonic_repair_attempt_count"] == 3
    assert report.metrics["final_photonic_repair_success_count"] == 2
