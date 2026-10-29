"""Report and acceptance diagnostics for path-length matching."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeTiming,
    PathLengthAnalysisResult,
)

from translation.path_length_requirements import (
    PATH_LENGTH_MATCH_TOLERANCE_UM,
    edge_identity,
    edge_identity_from_info,
    edge_key_to_dict,
    requirement_to_dict,
)
from translation.route_rust_types import MeanderEdgeReport, MeanderInsertionReport

#: Tolerance for the realized (bookkept-by-physical-edge) arrival spread of a
#: group, in um: one nanometre, the database unit; the planned check keeps
#: its 1e-6 um.
REALIZED_PATH_LENGTH_TOLERANCE_UM = 1.0e-3


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
            _incoming_edge_timing_to_dict(edge_timing) for edge_timing in timing.incoming_edges
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
    meander_report: MeanderInsertionReport | None,
    *,
    adjusted_requirements: list[MissingLengthRequirement] | None = None,
    lifted_groups: list[dict[str, object]] | None = None,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> list[dict[str, object]]:
    results = _meander_results(meander_report)
    inserted_by_edge, status_by_edge, unmatched_by_edge, reason_by_edge = _meander_result_maps(
        results
    )

    adjusted_missing_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    if adjusted_requirements is not None:
        adjusted_missing_by_edge = {
            edge_identity(req.edge_key): float(req.missing_length_um)
            for req in adjusted_requirements
        }

    base_groups = lifted_groups if lifted_groups is not None else matching_groups_to_info(analysis)
    realized_by_edge = realized_extra_by_edge_identity(meander_report)
    _, realized_arrival_by_edge = realized_arrivals(analysis, realized_by_edge)
    diagnostics: list[dict[str, object]] = []
    for group in base_groups:
        realized_group_arrivals: list[float] = []
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
            realized_extra = float(realized_by_edge.get(edge_id, 0.0))
            realized_arrival = float(realized_arrival_by_edge.get(edge_id, 0.0))
            realized_group_arrivals.append(realized_arrival)
            edges.append(
                {
                    **raw_edge,
                    "inserted_extra_length_um": inserted,
                    "physical_residual_um": physical_residual,
                    "accepted_unmatched_um": accepted_unmatched,
                    "disregarded_residual_um": disregarded,
                    "meander_status": status,
                    "meander_reason": reason,
                    # the bookkeeping by physical edge (2026-10-05): the
                    # meander actually drawn on this edge, and the edge's
                    # realized length and arrival with it
                    "realized_inserted_extra_length_um": realized_extra,
                    "realized_length_um": float(edge_timing.routed_length_um) + realized_extra,
                    "realized_arrival_um": realized_arrival,
                }
            )
        realized_spread = (
            max(realized_group_arrivals) - min(realized_group_arrivals)
            if realized_group_arrivals
            else 0.0
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
                "realized_arrival_spread_um": float(realized_spread),
                "realized_within_tolerance": bool(
                    realized_spread <= REALIZED_PATH_LENGTH_TOLERANCE_UM
                ),
                "incoming_edges": edges,
            }
        )
    return diagnostics


def output_matching_diagnostics_to_info(
    output_matching_info: dict[str, object] | None,
    meander_report: MeanderInsertionReport | None,
    *,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
    analysis: PathLengthAnalysisResult | None = None,
) -> list[dict[str, object]]:
    """Build acceptance diagnostics for optional global output matching.

    With ``analysis`` given, the group also carries the realized output
    arrivals (routed lengths plus the meanders actually drawn, walked over
    the whole graph) and their spread, the bookkeeping's own check.
    """
    if not output_matching_info or not bool(output_matching_info.get("enabled", False)):
        return []

    results = _meander_results(meander_report)
    inserted_by_edge, status_by_edge, unmatched_by_edge, reason_by_edge = _meander_result_maps(
        results
    )
    realized_output_arrival: dict[str, float] = {}
    if analysis is not None:
        output_arrival, _ = realized_arrivals(
            analysis, realized_extra_by_edge_identity(meander_report)
        )
        realized_output_arrival = {
            node_name: output_arrival[node_name]
            for node_name, timing in analysis.node_timings.items()
            if timing.node_type.value == "output" and node_name in output_arrival
        }
    raw_outputs = output_matching_info.get("outputs", [])
    if not isinstance(raw_outputs, list):
        raw_outputs = []
    # the realized spread within each group of outputs that are matched with
    # each other (scope ``all``: one group; ``depth``: one per depth)
    group_of_output = {
        str(raw.get("node_name")): raw.get("depth") for raw in raw_outputs if isinstance(raw, dict)
    }
    arrivals_by_group: dict[object, list[float]] = {}
    for node_name, arrival in realized_output_arrival.items():
        arrivals_by_group.setdefault(group_of_output.get(node_name), []).append(arrival)
    realized_output_spread = max(
        (max(values) - min(values) for values in arrivals_by_group.values()),
        default=0.0,
    )

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
            "realized_arrival_spread_um": float(realized_output_spread),
            "realized_within_tolerance": bool(
                realized_output_spread <= REALIZED_PATH_LENGTH_TOLERANCE_UM
            ),
            "realized_output_arrivals_um": dict(sorted(realized_output_arrival.items())),
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
    realized_failed_groups: list[dict[str, object]] = []
    max_physical_residual = 0.0
    max_accepted_unmatched = 0.0
    max_disregarded_residual = 0.0
    max_realized_spread = 0.0

    for group in diagnostics:
        if not isinstance(group, dict):
            continue
        realized_spread = object_to_float(group.get("realized_arrival_spread_um", 0.0))
        max_realized_spread = max(max_realized_spread, realized_spread)
        if realized_spread > REALIZED_PATH_LENGTH_TOLERANCE_UM:
            realized_failed_groups.append(
                {
                    "node_name": group.get("node_name", ""),
                    "node_type": group.get("node_type", ""),
                    "realized_arrival_spread_um": realized_spread,
                }
            )
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
        "passed": not failed_groups and not realized_failed_groups,
        "tolerance_um": float(tolerance_um),
        "realized_tolerance_um": float(REALIZED_PATH_LENGTH_TOLERANCE_UM),
        "max_realized_arrival_spread_um": float(max_realized_spread),
        "realized_failed_group_count": len(realized_failed_groups),
        "realized_failed_groups": realized_failed_groups,
        "failed_group_count": len(failed_groups),
        "failed_edge_count": sum(
            _list_length(group.get("failures", [])) for group in failed_groups
        ),
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
        (
            "Path-length matching failed: "
            f"{object_to_int(summary.get('failed_edge_count', 0))} edge(s) in "
            f"{object_to_int(summary.get('failed_group_count', 0))} group(s) retain physical "
            f"residual above {object_to_float(summary.get('tolerance_um', 0.0)):.6g} um "
            f"(max {object_to_float(summary.get('max_physical_residual_um', 0.0)):.6g} um)."
        )
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
    realized_failed_groups = summary.get("realized_failed_groups", [])
    if isinstance(realized_failed_groups, list) and realized_failed_groups:
        lines.append(
            f"  realized bookkeeping: {len(realized_failed_groups)} group(s) whose incoming "
            "arrivals (routed length plus the meander actually drawn on each edge) differ "
            f"by more than {object_to_float(summary.get('realized_tolerance_um', 0.0)):.6g} um "
            f"(max {object_to_float(summary.get('max_realized_arrival_spread_um', 0.0)):.6g} um):"
        )
        for group in realized_failed_groups[:5]:
            if isinstance(group, dict):
                lines.append(
                    f"  - {group.get('node_name', '<unknown>')}: spread "
                    f"{object_to_float(group.get('realized_arrival_spread_um', 0.0)):.6g} um"
                )
    return "\n".join(lines)


def analysis_to_info_dict(analysis: PathLengthAnalysisResult) -> dict[str, object]:
    return {
        "topological_order": list(analysis.topological_order),
        "node_arrival_um": {
            str(node): float(arrival) for node, arrival in analysis.node_arrival_um.items()
        },
        "node_arrival_input_um": {
            str(node): float(arrival) for node, arrival in analysis.node_arrival_input_um.items()
        },
        "node_arrival_output_um": {
            str(node): float(arrival) for node, arrival in analysis.node_arrival_output_um.items()
        },
        "node_timings_um": {
            str(node): node_timing_to_dict(timing) for node, timing in analysis.node_timings.items()
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


def realized_extra_by_edge_identity(
    meander_report: MeanderInsertionReport | None,
) -> dict[tuple[str, str, str, str, str], float]:
    """The meander report's realized extra length per physical edge, by edge identity."""
    if meander_report is None:
        return {}
    return {
        edge_identity(edge_key): float(extra)
        for edge_key, extra in meander_report.realized_extra_by_edge.items()
    }


def realized_arrivals(
    analysis: PathLengthAnalysisResult,
    realized_by_edge: dict[tuple[str, str, str, str, str], float],
) -> tuple[dict[str, float], dict[tuple[str, str, str, str, str], float]]:
    """Walk the graph with realized edge lengths (routed plus the meander drawn on the edge).

    Returns ``(output_arrival_by_node, arrival_by_edge)``: every node's
    output arrival (its realized input arrival, the latest incoming edge,
    plus its internal delay) and every edge's realized arrival at its
    target. This is the bookkeeping's own check that the delay-insertion
    candidates kept every gate balanced: a gate's incoming realized
    arrivals must agree, whichever edges the meanders were drawn on.
    """
    output_arrival: dict[str, float] = {}
    arrival_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    for node_name in analysis.topological_order:
        timing = analysis.node_timings[node_name]
        arrivals: list[float] = []
        for edge_timing in timing.incoming_edges:
            edge_id = edge_identity(edge_timing.edge_key)
            source_out = output_arrival.get(edge_timing.source_instance, 0.0)
            arrival = (
                source_out
                + float(edge_timing.routed_length_um)
                + realized_by_edge.get(edge_id, 0.0)
            )
            arrival_by_edge[edge_id] = arrival
            arrivals.append(arrival)
        input_arrival = max(arrivals) if arrivals else 0.0
        output_arrival[node_name] = input_arrival + float(timing.internal_delay_um)
    return output_arrival, arrival_by_edge


def _meander_results(meander_report: MeanderInsertionReport | None) -> list[MeanderEdgeReport]:
    return list(meander_report.results) if meander_report is not None else []


def _meander_result_maps(
    results: list[MeanderEdgeReport],
) -> tuple[
    dict[tuple[str, str, str, str, str], float],
    dict[tuple[str, str, str, str, str], str],
    dict[tuple[str, str, str, str, str], float],
    dict[tuple[str, str, str, str, str], str],
]:
    """Per requiring edge: the booked extra, the status (a planned entry wins), the unmatched, the reason."""
    inserted_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    status_by_edge: dict[tuple[str, str, str, str, str], str] = {}
    unmatched_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    reason_by_edge: dict[tuple[str, str, str, str, str], str] = {}
    for item in results:
        edge_id = edge_identity(item.edge)
        inserted_by_edge[edge_id] = inserted_by_edge.get(edge_id, 0.0) + float(
            item.inserted_extra_length_um
        )
        previous_status = status_by_edge.get(edge_id)
        if previous_status is None or previous_status != "planned":
            status_by_edge[edge_id] = item.status
            reason_by_edge[edge_id] = item.reason
        unmatched_by_edge[edge_id] = unmatched_by_edge.get(edge_id, 0.0) + float(
            item.unmatched_length_um
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
