"""Path-length matching metadata and reporting for routing_flow."""

from __future__ import annotations

from gdsfactory.component import Component

from translation.route_rust_types import (
    MeanderEdgeReport,
    MeanderInsertionReport,
    MeanderPlanningProfile,
)


def attach_and_report_path_length_matching(
    *,
    routed_layout: Component,
    route_result: object,
    debug_meanders: bool,
    stats: object | None = None,
) -> None:
    """Attach PLM metadata to the layout and print the PLM summary.

    The SUMMARY tokens (``plm_groups``, ``plm_over_tolerance``,
    ``meander_inserted_um``) come from the group diagnostics and the meander
    report's totals; the profile printout needs the profile the planner
    builds only under ``--debug-meanders``.
    """
    analysis_info = getattr(route_result, "path_length_analysis_info", None)
    if analysis_info is None:
        return

    meander_report = getattr(route_result, "meander_insertion_report", None)
    meander_report_info = getattr(route_result, "meander_insertion_report_info", None)
    routed_layout.info["path_length_analysis"] = analysis_info
    routed_layout.info["meander_requirements"] = (
        getattr(route_result, "meander_requirements_info", None) or []
    )
    if meander_report_info is not None:
        routed_layout.info["meander_insertion_report"] = meander_report_info
    print(
        "      - Path-length matching: "
        f"{len(routed_layout.info['meander_requirements'])} edge(s) require extra length"
    )
    group_diagnostics = analysis_info.get(
        "matching_group_diagnostics",
        analysis_info.get("matching_groups", []),
    )
    if isinstance(group_diagnostics, list):
        groups_over_tolerance = sum(
            1
            for group in group_diagnostics
            if isinstance(group, dict) and group.get("within_tolerance") is False
        )
        max_residual = max(
            (
                float(group.get("max_accepted_unmatched_um", 0.0))
                for group in group_diagnostics
                if isinstance(group, dict)
            ),
            default=0.0,
        )
        if stats is not None:
            stats.path_length_groups = len(group_diagnostics)
            stats.path_length_groups_over_tolerance = int(groups_over_tolerance)
        print(
            "      - Path-length groups: "
            f"{len(group_diagnostics)} group(s), "
            f"over_tolerance={groups_over_tolerance}, "
            f"max_residual={max_residual:.6f}um"
        )
    if debug_meanders:
        _report_path_length_node_timings(analysis_info)
    if isinstance(meander_report, MeanderInsertionReport):
        _report_meander_insertion(meander_report, debug_meanders=debug_meanders)
        if stats is not None:
            stats.meander_inserted_um = float(meander_report.total_inserted_extra_length_um)


def _report_path_length_node_timings(path_length_analysis_info: dict[object, object]) -> None:
    node_timings = path_length_analysis_info.get("node_timings_um", {})
    if not isinstance(node_timings, dict):
        return
    for node_name, node_info in node_timings.items():
        if not isinstance(node_info, dict):
            continue
        incoming = node_info.get("incoming_edges")
        if incoming is None:
            incoming = []
        print(
            f"        • node={node_name}, "
            f"type={node_info.get('node_type')}, "
            f"internal={float(node_info.get('internal_delay_um', 0.0)):.3f}um, "
            f"input={float(node_info.get('input_arrival_um', 0.0)):.3f}um, "
            f"output={float(node_info.get('output_arrival_um', 0.0)):.3f}um"
        )
        for incoming_entry in incoming:
            if not isinstance(incoming_entry, dict):
                continue
            edge = incoming_entry.get("edge", {})
            edge_name = (
                f"{edge.get('source', {}).get('instance', '?')}->"
                f"{edge.get('target', {}).get('instance', '?')} "
                f"({edge.get('net_name', '?')})"
            )
            print(
                "          - "
                f"{edge_name}: edge_len={float(incoming_entry.get('routed_length_um', 0.0)):.3f}um, "
                f"edge_arrival={float(incoming_entry.get('edge_arrival_um', 0.0)):.3f}um, "
                f"missing={float(incoming_entry.get('missing_length_um', 0.0)):.3f}um"
            )


def _report_meander_insertion(
    report: MeanderInsertionReport,
    *,
    debug_meanders: bool,
) -> None:
    print(
        "      - Meander insertion: "
        f"requested={report.total_requested_extra_length_um:.3f}um, "
        f"inserted={report.total_inserted_extra_length_um:.3f}um, "
        f"unmatched={report.unmatched_length_um:.3f}um"
    )
    if not debug_meanders:
        return
    if report.profile is not None:
        _report_meander_profile(report.profile)
    for entry in report.results:
        print("        • " + _meander_result_line(entry))


def _f(value: object) -> float:
    return float(value) if isinstance(value, (int, float, str)) else 0.0


def _i(value: object) -> int:
    return int(_f(value))


def _report_meander_profile(profile: MeanderPlanningProfile) -> None:
    """The planner's timers and counters, one line per split."""
    print(
        "        Meander overhead profile: "
        f"planner={profile.planner_elapsed_s:.4f}s "
        f"({profile.planner_calls} edge plans, "
        f"final pass {profile.final_planner_elapsed_s:.4f}s / "
        f"{profile.final_planner_calls} edge plans), "
        f"commit={profile.commit_elapsed_s:.4f}s"
    )
    _print_split("Meander setup profile", profile.setup_profile)
    _print_split("Rust meander planner split", profile.rust_planner_profile)
    _print_split("Rust meander wrapper split", profile.rust_wrapper_profile)
    _print_split("Meander commit split", profile.commit_profile)
    if profile.candidate_profile:
        print("        Candidate planner profile:")
        for reason, raw_profile in sorted(
            profile.candidate_profile.items(),
            key=lambda item: -_f(item[1].get("elapsed_s", 0.0)),
        ):
            print(
                "          - "
                f"{reason}: candidates={_i(raw_profile.get('candidate_attempts', 0))}, "
                f"edge_calls={_i(raw_profile.get('edge_calls', 0))}, "
                f"planned={_i(raw_profile.get('planned', 0))}, "
                f"no_candidate={_i(raw_profile.get('no_candidate', 0))}, "
                f"already_satisfied={_i(raw_profile.get('already_satisfied', 0))}, "
                f"elapsed={_f(raw_profile.get('elapsed_s', 0.0)):.4f}s"
            )
    for attempt in profile.requirement_attempts:
        edge = attempt.get("edge", {})
        net_name = edge.get("net_name", "<unknown>") if isinstance(edge, dict) else "<unknown>"
        candidates = attempt.get("candidates", [])
        tried = ", ".join(
            _candidate_attempt_text(candidate)
            for candidate in (candidates if isinstance(candidates, list) else [])
            if isinstance(candidate, dict)
        )
        print(
            f"        Requirement {net_name}: "
            f"requested={_f(attempt.get('requested_extra_length_um', 0.0)):.3f}um, "
            f"selected={attempt.get('selected_candidate_reason')}, "
            f"edge_plans={_i(attempt.get('planner_edge_calls', 0))}, "
            f"elapsed={_f(attempt.get('planning_elapsed_s', 0.0)):.4f}s; {tried}"
        )


def _candidate_attempt_text(candidate: dict[str, object]) -> str:
    edges = candidate.get("edges", [])
    requests = candidate.get("requested_extra_lengths_um", [])
    edge_count = len(edges) if isinstance(edges, list) else 0
    request_text = ", ".join(
        f"{_f(value):.3f}" for value in (requests if isinstance(requests, list) else [])
    )
    return (
        f"{candidate.get('candidate_reason')}[{edge_count} edge(s): {request_text}]"
        f"={candidate.get('status')}"
    )


def _print_split(title: str, split: dict[str, float]) -> None:
    if not split:
        return
    parts = [
        f"{key.removesuffix('_s')}={float(value):.4f}s"
        if key.endswith("_s")
        else f"{key}={int(float(value))}"
        for key, value in sorted(split.items(), key=lambda item: -float(item[1]))
    ]
    print(f"        {title}: " + ", ".join(parts))


def _meander_result_line(entry: MeanderEdgeReport) -> str:
    plan = entry.plan or {}
    planned_on = ", ".join(edge_key.net_name for edge_key in entry.planned_edges)
    return (
        f"{entry.edge.net_name}: status={entry.status}, "
        f"requested={entry.requested_extra_length_um:.3f}um, "
        f"inserted={entry.inserted_extra_length_um:.3f}um, "
        f"physical={entry.physical_inserted_extra_length_um:.3f}um, "
        f"unmatched={entry.unmatched_length_um:.3f}um, "
        f"candidate={entry.candidate_reason}, on=[{planned_on}], "
        f"side={plan.get('side')}, bumps={plan.get('bumps')}, "
        f"visual_bumps={plan.get('visual_bumps')}, "
        f"endpoint_inset_um={plan.get('endpoint_inset_um')}, "
        f"selected_grid_rect={plan.get('selected_grid_rect')}, "
        f"reason={entry.reason}"
    )
