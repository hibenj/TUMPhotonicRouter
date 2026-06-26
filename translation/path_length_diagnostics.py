"""Report and acceptance diagnostics for path-length matching."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeTiming,
    PathLengthAnalysisResult,
)

from translation.path_length_candidates import (
    edge_identity,
    edge_identity_from_info,
    edge_key_to_dict,
    requirement_to_dict,
)
from translation.path_length_requirements import PATH_LENGTH_MATCH_TOLERANCE_UM


def object_to_float(value: object, default: float = 0.0) -> float:
    if isinstance(value, (str, bytes, bytearray, int, float)):
        return float(value)
    return default


def object_to_int(value: object, default: int = 0) -> int:
    if isinstance(value, (str, bytes, bytearray, int, float)):
        return int(value)
    return default


def node_timing_to_dict(timing: NodeTiming) -> dict[str, object]:
    return {
        "node_name": timing.node_name,
        "node_type": timing.node_type.value,
        "internal_delay_um": float(timing.internal_delay_um),
        "input_arrival_um": float(timing.input_arrival_um),
        "output_arrival_um": float(timing.output_arrival_um),
        "incoming_edges": [
            _incoming_edge_timing_to_dict(edge_timing)
            for edge_timing in timing.incoming_edges
        ],
    }


def matching_groups_to_info(analysis: PathLengthAnalysisResult) -> list[dict[str, object]]:
    groups: list[dict[str, object]] = []
    for node_name in analysis.topological_order:
        timing = analysis.node_timings.get(node_name)
        if timing is None or len(timing.incoming_edges) < 2:
            continue
        missing_values = [float(edge.missing_length_um) for edge in timing.incoming_edges]
        groups.append(
            {
                "node_name": timing.node_name,
                "node_type": timing.node_type.value,
                "incoming_count": len(timing.incoming_edges),
                "target_input_arrival_um": float(timing.input_arrival_um),
                "max_missing_length_um": max(missing_values, default=0.0),
                "total_missing_length_um": float(sum(missing_values)),
                "edges_requiring_meander": sum(1 for value in missing_values if value > 0.0),
                "incoming_edges": [
                    _incoming_edge_timing_to_dict(edge_timing)
                    for edge_timing in timing.incoming_edges
                ],
            }
        )
    return groups


def matching_group_diagnostics_to_info(
    analysis: PathLengthAnalysisResult,
    meander_report: dict[str, object] | None,
    *,
    adjusted_requirements: list[MissingLengthRequirement] | None = None,
    lifted_groups: list[dict[str, object]] | None = None,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> list[dict[str, object]]:
    results = _meander_results(meander_report)
    inserted_by_edge, status_by_edge, unmatched_by_edge, reason_by_edge = (
        _meander_result_maps(results)
    )

    adjusted_missing_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    if adjusted_requirements is not None:
        adjusted_missing_by_edge = {
            edge_identity(req.edge_key): float(req.missing_length_um)
            for req in adjusted_requirements
        }

    base_groups = lifted_groups if lifted_groups is not None else matching_groups_to_info(analysis)
    diagnostics: list[dict[str, object]] = []
    for group in base_groups:
        edges: list[dict[str, object]] = []
        max_unmatched = 0.0
        max_physical_residual = 0.0
        max_disregarded = 0.0
        raw_edges = group.get("incoming_edges", [])
        if not isinstance(raw_edges, list):
            raw_edges = []
        timing_edges_by_id = {
            edge_identity(edge_timing.edge_key): edge_timing
            for edge_timing in analysis.node_timings[str(group["node_name"])].incoming_edges
        }
        for raw_edge in raw_edges:
            if not isinstance(raw_edge, dict):
                continue
            edge_id = edge_identity_from_info(raw_edge.get("edge"))
            if edge_id is None:
                continue
            edge_timing = timing_edges_by_id.get(edge_id)
            if edge_timing is None:
                continue
            edge_id = edge_identity(edge_timing.edge_key)
            missing = float(
                raw_edge.get(
                    "adjusted_missing_length_um",
                    adjusted_missing_by_edge.get(edge_id, edge_timing.missing_length_um),
                )
            )
            inserted = float(inserted_by_edge.get(edge_id, 0.0))
            status = status_by_edge.get(edge_id, "not_required" if missing == 0.0 else "missing")
            reason = reason_by_edge.get(edge_id, "")
            physical_residual = max(0.0, missing - inserted)
            accepted_unmatched = max(0.0, unmatched_by_edge.get(edge_id, physical_residual))
            disregarded = physical_residual if status == "below_minimum_bump" else 0.0
            if status == "below_minimum_bump":
                accepted_unmatched = 0.0
            max_unmatched = max(max_unmatched, accepted_unmatched)
            max_physical_residual = max(max_physical_residual, physical_residual)
            max_disregarded = max(max_disregarded, disregarded)
            edges.append(
                {
                    **raw_edge,
                    "inserted_extra_length_um": inserted,
                    "physical_residual_um": physical_residual,
                    "accepted_unmatched_um": accepted_unmatched,
                    "disregarded_residual_um": disregarded,
                    "meander_status": status,
                    "meander_reason": reason,
                }
            )
        diagnostics.append(
            {
                **{key: value for key, value in group.items() if key != "incoming_edges"},
                "tolerance_um": float(tolerance_um),
                "max_accepted_unmatched_um": float(max_unmatched),
                "max_physical_residual_um": float(max_physical_residual),
                "max_disregarded_residual_um": float(max_disregarded),
                "within_tolerance": bool(max_unmatched <= tolerance_um),
                "has_disregarded_residual": bool(max_disregarded > tolerance_um),
                "incoming_edges": edges,
            }
        )
    return diagnostics


def output_matching_diagnostics_to_info(
    output_matching_info: dict[str, object] | None,
    meander_report: dict[str, object] | None,
    *,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> list[dict[str, object]]:
    """Build acceptance diagnostics for optional global output matching."""
    if not output_matching_info or not bool(output_matching_info.get("enabled", False)):
        return []

    results = _meander_results(meander_report)
    inserted_by_edge, status_by_edge, unmatched_by_edge, reason_by_edge = (
        _meander_result_maps(results)
    )
    raw_outputs = output_matching_info.get("outputs", [])
    if not isinstance(raw_outputs, list):
        raw_outputs = []

    edges: list[dict[str, object]] = []
    max_unmatched = 0.0
    max_physical_residual = 0.0
    max_disregarded = 0.0
    for raw_output in raw_outputs:
        if not isinstance(raw_output, dict):
            continue
        missing = object_to_float(raw_output.get("missing_length_um", 0.0))
        if missing <= tolerance_um:
            continue
        edge_info = raw_output.get("edge", {})
        edge_id = edge_identity_from_info(edge_info)
        inserted = inserted_by_edge.get(edge_id, 0.0) if edge_id is not None else 0.0
        physical_residual = max(0.0, missing - inserted)
        status = (
            status_by_edge.get(edge_id, "missing")
            if edge_id is not None
            else str(raw_output.get("status", "missing"))
        )
        reason = reason_by_edge.get(edge_id, "") if edge_id is not None else ""
        accepted_unmatched = (
            max(0.0, unmatched_by_edge.get(edge_id, physical_residual))
            if edge_id is not None
            else physical_residual
        )
        disregarded = physical_residual if status == "below_minimum_bump" else 0.0
        if status == "below_minimum_bump":
            accepted_unmatched = 0.0
        max_unmatched = max(max_unmatched, accepted_unmatched)
        max_physical_residual = max(max_physical_residual, physical_residual)
        max_disregarded = max(max_disregarded, disregarded)
        edges.append(
            {
                "edge": edge_info,
                "node_name": raw_output.get("node_name", ""),
                "arrival_um": object_to_float(raw_output.get("arrival_um", 0.0)),
                "adjusted_missing_length_um": missing,
                "missing_length_um": missing,
                "inserted_extra_length_um": inserted,
                "physical_residual_um": physical_residual,
                "accepted_unmatched_um": accepted_unmatched,
                "disregarded_residual_um": disregarded,
                "meander_status": status,
                "meander_reason": reason,
            }
        )

    if not edges:
        return []

    return [
        {
            "node_name": "output_arrivals",
            "node_type": "output",
            "incoming_count": len(edges),
            "target_input_arrival_um": object_to_float(
                output_matching_info.get("target_output_arrival_um", 0.0)
            ),
            "target_lift_um": 0.0,
            "tolerance_um": float(tolerance_um),
            "max_accepted_unmatched_um": float(max_unmatched),
            "max_physical_residual_um": float(max_physical_residual),
            "max_disregarded_residual_um": float(max_disregarded),
            "within_tolerance": bool(max_unmatched <= tolerance_um),
            "has_disregarded_residual": bool(max_disregarded > tolerance_um),
            "incoming_edges": edges,
        }
    ]


def path_length_acceptance_summary(
    diagnostics: list[dict[str, object]],
    *,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> dict[str, object]:
    """Summarize whether realized PLM exactly satisfies every group target."""
    failed_groups: list[dict[str, object]] = []
    max_physical_residual = 0.0
    max_accepted_unmatched = 0.0
    max_disregarded_residual = 0.0

    for group in diagnostics:
        if not isinstance(group, dict):
            continue
        group_failures: list[dict[str, object]] = []
        raw_edges = group.get("incoming_edges", [])
        if not isinstance(raw_edges, list):
            raw_edges = []
        for edge in raw_edges:
            if not isinstance(edge, dict):
                continue
            physical_residual = object_to_float(edge.get("physical_residual_um", 0.0))
            accepted_unmatched = object_to_float(edge.get("accepted_unmatched_um", 0.0))
            disregarded_residual = object_to_float(edge.get("disregarded_residual_um", 0.0))
            max_physical_residual = max(max_physical_residual, physical_residual)
            max_accepted_unmatched = max(max_accepted_unmatched, accepted_unmatched)
            max_disregarded_residual = max(max_disregarded_residual, disregarded_residual)
            if physical_residual <= tolerance_um:
                continue
            group_failures.append(
                {
                    "edge": edge.get("edge", {}),
                    "meander_status": edge.get("meander_status", "unknown"),
                    "meander_reason": edge.get("meander_reason", ""),
                    "requested_extra_length_um": object_to_float(
                        edge.get(
                            "adjusted_missing_length_um",
                            edge.get("missing_length_um", 0.0),
                        )
                    ),
                    "inserted_extra_length_um": object_to_float(
                        edge.get("inserted_extra_length_um", 0.0)
                    ),
                    "physical_residual_um": physical_residual,
                    "accepted_unmatched_um": accepted_unmatched,
                    "disregarded_residual_um": disregarded_residual,
                }
            )
        if group_failures:
            failed_groups.append(
                {
                    "node_name": group.get("node_name", ""),
                    "node_type": group.get("node_type", ""),
                    "target_input_arrival_um": object_to_float(
                        group.get("target_input_arrival_um", 0.0)
                    ),
                    "target_lift_um": object_to_float(group.get("target_lift_um", 0.0)),
                    "max_physical_residual_um": object_to_float(
                        group.get("max_physical_residual_um", 0.0)
                    ),
                    "failures": group_failures,
                }
            )

    return {
        "passed": not failed_groups,
        "tolerance_um": float(tolerance_um),
        "failed_group_count": len(failed_groups),
        "failed_edge_count": sum(_list_length(group.get("failures", [])) for group in failed_groups),
        "max_physical_residual_um": float(max_physical_residual),
        "max_accepted_unmatched_um": float(max_accepted_unmatched),
        "max_disregarded_residual_um": float(max_disregarded_residual),
        "failed_groups": failed_groups,
    }


def format_path_length_acceptance_failure(summary: dict[str, object]) -> str:
    """Build a compact user-facing PLM failure message."""
    failed_groups = summary.get("failed_groups", [])
    if not isinstance(failed_groups, list):
        failed_groups = []
    lines = [
        "Path-length matching failed: "
        f"{object_to_int(summary.get('failed_edge_count', 0))} edge(s) in "
        f"{object_to_int(summary.get('failed_group_count', 0))} group(s) retain physical "
        f"residual above {object_to_float(summary.get('tolerance_um', 0.0)):.6g} um "
        f"(max {object_to_float(summary.get('max_physical_residual_um', 0.0)):.6g} um)."
    ]
    for group in failed_groups[:5]:
        if not isinstance(group, dict):
            continue
        node_name = str(group.get("node_name", "<unknown>"))
        failures = group.get("failures", [])
        if not isinstance(failures, list):
            failures = []
        for failure in failures[:3]:
            if not isinstance(failure, dict):
                continue
            edge_info = failure.get("edge", {})
            lines.append(
                "  - "
                f"{node_name} {_edge_label_from_info(edge_info)}: "
                f"requested={float(failure.get('requested_extra_length_um', 0.0)):.6g} um, "
                f"inserted={float(failure.get('inserted_extra_length_um', 0.0)):.6g} um, "
                f"residual={float(failure.get('physical_residual_um', 0.0)):.6g} um, "
                f"status={failure.get('meander_status', 'unknown')}"
            )
            reason = str(failure.get("meander_reason", ""))
            if reason:
                lines.append(f"    reason={reason}")
    if len(failed_groups) > 5:
        lines.append(f"  - ... {len(failed_groups) - 5} more failed group(s)")
    return "\n".join(lines)


def analysis_to_info_dict(analysis: PathLengthAnalysisResult) -> dict[str, object]:
    return {
        "topological_order": list(analysis.topological_order),
        "node_arrival_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_um.items()
        },
        "node_arrival_input_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_input_um.items()
        },
        "node_arrival_output_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_output_um.items()
        },
        "node_timings_um": {
            str(node): node_timing_to_dict(timing)
            for node, timing in analysis.node_timings.items()
        },
        "edge_missing_lengths_um": [
            {
                "edge": edge_key_to_dict(edge_key),
                "missing_length_um": float(missing),
            }
            for edge_key, missing in analysis.edge_missing_lengths_um.items()
        ],
        "requirements": [requirement_to_dict(req) for req in analysis.requirements],
        "matching_groups": matching_groups_to_info(analysis),
    }


def _incoming_edge_timing_to_dict(
    edge_timing: NodeIncomingEdgeTiming,
) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(edge_timing.edge_key),
        "routed_length_um": float(edge_timing.routed_length_um),
        "edge_arrival_um": float(edge_timing.edge_arrival_um),
        "missing_length_um": float(edge_timing.missing_length_um),
    }


def _meander_results(meander_report: dict[str, object] | None) -> list[dict[str, object]]:
    if not isinstance(meander_report, dict):
        return []
    raw_results = meander_report.get("results", [])
    if not isinstance(raw_results, list):
        return []
    return [item for item in raw_results if isinstance(item, dict)]


def _meander_result_maps(
    results: list[dict[str, object]],
) -> tuple[
    dict[tuple[str, str, str, str, str], float],
    dict[tuple[str, str, str, str, str], str],
    dict[tuple[str, str, str, str, str], float],
    dict[tuple[str, str, str, str, str], str],
]:
    inserted_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    status_by_edge: dict[tuple[str, str, str, str, str], str] = {}
    unmatched_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    reason_by_edge: dict[tuple[str, str, str, str, str], str] = {}
    for item in results:
        edge_id = edge_identity_from_info(item.get("edge"))
        if edge_id is None:
            continue
        inserted_by_edge[edge_id] = inserted_by_edge.get(edge_id, 0.0) + object_to_float(
            item.get("inserted_extra_length_um", 0.0)
        )
        status = str(item.get("status", "unknown"))
        previous_status = status_by_edge.get(edge_id)
        if previous_status is None or previous_status != "planned":
            status_by_edge[edge_id] = status
            reason_by_edge[edge_id] = str(item.get("reason", ""))
        unmatched_by_edge[edge_id] = unmatched_by_edge.get(edge_id, 0.0) + object_to_float(
            item.get("unmatched_length_um", 0.0)
        )
    return inserted_by_edge, status_by_edge, unmatched_by_edge, reason_by_edge


def _edge_label_from_info(edge_info: object) -> str:
    if not isinstance(edge_info, dict):
        return "<unknown edge>"
    source = edge_info.get("source", {})
    target = edge_info.get("target", {})
    if not isinstance(source, dict):
        source = {}
    if not isinstance(target, dict):
        target = {}
    return (
        f"{edge_info.get('net_name', '<unknown>')} "
        f"{source.get('instance', '?')},{source.get('port', '?')} -> "
        f"{target.get('instance', '?')},{target.get('port', '?')}"
    )


def _list_length(value: object) -> int:
    return len(value) if isinstance(value, list) else 0
