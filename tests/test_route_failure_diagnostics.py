from __future__ import annotations

import json

from translation import route_rust


def test_failed_route_log_extracts_illegal_crossing_root_causes() -> None:
    terminal_error = (
        "No repair route found; candidate_blockers=[36, 31]; recent_errors=["
        "\"repair_failed_net:net33:roundSome(4):rip[]:Illegal realized "
        "crossing: net 33 intersects net 36 at (1491.500, 571.125) "
        "(not_perpendicular)\", "
        "\"lidar_pure_probe_commit:net33:roundSome(4):rip[]:Illegal realized "
        "crossing: net 33 intersects net 36 at (1491.500, 571.125) "
        "(not_perpendicular)\"]"
    )
    later_attempt_error = (
        "Illegal realized crossing: net 36 intersects net 33 at "
        "(1488.470, 562.095) (not_perpendicular)"
    )

    line = route_rust._format_illegal_crossing_root_causes_line(
        [terminal_error, later_attempt_error]
    )

    assert line is not None
    key, raw_payload = line.split("=", 1)
    assert key == "root_cause_illegal_crossings"
    assert json.loads(raw_payload) == [
        {
            "net_a": 33,
            "net_b": 36,
            "reason": "not_perpendicular",
            "x_um": 1491.5,
            "y_um": 571.125,
        },
        {
            "net_a": 36,
            "net_b": 33,
            "reason": "not_perpendicular",
            "x_um": 1488.47,
            "y_um": 562.095,
        },
    ]


def test_failed_route_log_omits_root_cause_line_without_illegal_crossing() -> None:
    assert (
        route_rust._format_illegal_crossing_root_causes_line(
            ["No repair route found; candidate_blockers=[36, 31]"]
        )
        is None
    )


def test_failed_route_log_formats_native_repair_trace_tail() -> None:
    lines = route_rust._format_native_repair_trace_lines(
        [
            {
                "event": "repair_mode_start",
                "route_order": "current_first",
                "net_id": 33,
                "repair_round": 2,
                "repair_set_index": 4,
                "ripup_ids": [36, 31, 30],
                "victim_order": [36, 31, 30],
                "victim_first": False,
                "reverse_victim_order": False,
            },
            {
                "event": "victim_reroute",
                "route_order": "current_first",
                "action": "normal_route",
                "net_id": 36,
                "repair_round": 2,
                "repair_set_index": 4,
                "ripup_ids": [36, 31, 30],
                "victim_order": [36, 31, 30],
                "success": True,
            },
            {
                "event": "repair_mode_result",
                "route_order": "current_first",
                "net_id": 33,
                "repair_round": 2,
                "repair_set_index": 4,
                "ripup_ids": [36, 31, 30],
                "success": False,
                "error": "mode_failed",
            },
        ],
        tail=2,
    )

    assert lines[0] == "native_repair_trace_count=3"
    key, raw_payload = lines[1].split("=", 1)
    assert key == "native_repair_trace_tail"
    trace_tail = json.loads(raw_payload)
    assert [event["event"] for event in trace_tail] == [
        "victim_reroute",
        "repair_mode_result",
    ]
    assert trace_tail[0]["route_order"] == "current_first"
    assert trace_tail[0]["ripup_ids"] == [36, 31, 30]
    assert trace_tail[1]["success"] is False
