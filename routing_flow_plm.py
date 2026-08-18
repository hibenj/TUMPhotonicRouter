"""Path-length matching metadata and reporting for routing_flow."""

from gdsfactory.component import Component


def attach_and_report_path_length_matching(
    *,
    routed_layout: Component,
    route_result: object,
    debug_meanders: bool,
) -> None:
    """Attach PLM metadata to the layout and print the existing PLM summary."""
    if route_result.path_length_analysis_info is None:
        return

    meander_report_info = getattr(route_result, "meander_insertion_report_info", None)
    routed_layout.info["path_length_analysis"] = route_result.path_length_analysis_info
    routed_layout.info["meander_requirements"] = (
        route_result.meander_requirements_info or []
    )
    if meander_report_info is not None:
        routed_layout.info["meander_insertion_report"] = meander_report_info
    print(
        "      - Path-length matching: "
        f"{len(routed_layout.info['meander_requirements'])} edge(s) require extra length"
    )
    group_diagnostics = route_result.path_length_analysis_info.get(
        "matching_group_diagnostics",
        route_result.path_length_analysis_info.get("matching_groups", []),
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
        print(
            "      - Path-length groups: "
            f"{len(group_diagnostics)} group(s), "
            f"over_tolerance={groups_over_tolerance}, "
            f"max_residual={max_residual:.6f}um"
        )
    if debug_meanders and route_result.path_length_analysis_info is not None:
        _report_path_length_node_timings(route_result.path_length_analysis_info)
    if meander_report_info is not None:
        _report_meander_insertion(meander_report_info, debug_meanders=debug_meanders)


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
            f"        \u2022 node={node_name}, "
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
    report: dict[object, object],
    *,
    debug_meanders: bool,
) -> None:
    total_requested = float(report.get("total_requested_extra_length_um", 0.0))
    total_inserted = float(report.get("total_inserted_extra_length_um", 0.0))
    unmatched = float(report.get("unmatched_length_um", 0.0))
    print(
        "      - Meander insertion: "
        f"requested={total_requested:.3f}um, "
        f"inserted={total_inserted:.3f}um, "
        f"unmatched={unmatched:.3f}um"
    )
    if not debug_meanders:
        return
    _report_meander_setup_profile(report)
    print(
        "        Meander overhead profile: "
        f"planner={float(report.get('planner_elapsed_s', 0.0)):.4f}s, "
        f"candidate_setup={float(report.get('candidate_overhead_s', 0.0)):.4f}s, "
        f"commit={float(report.get('commit_elapsed_s', 0.0)):.4f}s"
    )
    _report_rust_planner_profile(report)
    _report_rust_wrapper_profile(report)
    _report_meander_commit_profile(report)
    _report_meander_candidate_execution(report)
    _report_candidate_engine_counts(report)
    _report_candidate_setup_profile(report)
    _report_candidate_profile(report)
    _report_meander_results(report)


def _report_meander_setup_profile(report: dict[object, object]) -> None:
    setup_profile = report.get("setup_profile", {})
    if not isinstance(setup_profile, dict) or not setup_profile:
        return
    print(
        "        Meander setup profile: "
        f"total={float(setup_profile.get('total_s', 0.0)):.4f}s, "
        f"router_init={float(setup_profile.get('router_init_s', 0.0)):.4f}s, "
        f"by_edge={float(setup_profile.get('by_edge_s', 0.0)):.4f}s, "
        f"base_static_collect={float(setup_profile.get('base_static_collect_s', 0.0)):.4f}s, "
        f"base_static_reused={int(float(setup_profile.get('base_static_reused', 0.0)))}, "
        f"static_handle={int(float(setup_profile.get('combined_static_route_registration_handle', 0.0)))}, "
        f"set_static={float(setup_profile.get('set_static_cells_s', 0.0)):.4f}s, "
        f"register_routes={float(setup_profile.get('register_route_cells_s', 0.0)):.4f}s, "
        f"register_geometry={float(setup_profile.get('register_route_geometry_s', 0.0)):.4f}s, "
        f"registered_records={int(float(setup_profile.get('registered_record_count', 0.0)))}, "
        f"unregistered_records={int(float(setup_profile.get('unregistered_record_count', 0.0)))}, "
        f"unregistered_route_static={int(float(setup_profile.get('unregistered_route_static_cell_count', 0.0)))}, "
        f"route_occupancy_radius={int(float(setup_profile.get('route_occupancy_radius_cells', 0.0)))}, "
        f"box_clearance_radius={int(float(setup_profile.get('meander_box_clearance_radius_cells', 0.0)))}, "
        f"unique_route_cells={int(float(setup_profile.get('unique_route_cell_count', 0.0)))}"
    )
    print(
        "        Meander route-registration setup split: "
        f"edge_order={float(setup_profile.get('edge_order_s', 0.0)):.4f}s, "
        f"route_objects={float(setup_profile.get('route_object_list_s', 0.0)):.4f}s, "
        f"base_static_list={float(setup_profile.get('base_static_registration_list_s', 0.0)):.4f}s, "
        f"rust_call={float(setup_profile.get('register_route_cells_call_s', 0.0)):.4f}s, "
        f"result_map={float(setup_profile.get('registration_result_map_s', 0.0)):.4f}s"
    )
    print(
        "        Meander geometry-registration setup split: "
        f"prepare={float(setup_profile.get('geometry_prepare_s', 0.0)):.4f}s, "
        f"centerline_copy={float(setup_profile.get('geometry_centerline_copy_s', 0.0)):.4f}s, "
        f"max_bumps={float(setup_profile.get('geometry_max_bumps_s', 0.0)):.4f}s, "
        f"rust_call={float(setup_profile.get('geometry_call_s', 0.0)):.4f}s, "
        f"result_map={float(setup_profile.get('geometry_result_map_s', 0.0)):.4f}s"
    )
    if any(key.startswith("rust_registration_") for key in setup_profile):
        print(
            "        Rust route-registration split: "
            f"total={float(setup_profile.get('rust_registration_total_s', 0.0)):.4f}s, "
            f"reset={float(setup_profile.get('rust_registration_reset_s', 0.0)):.4f}s, "
            f"base_pack={float(setup_profile.get('rust_registration_base_static_pack_s', 0.0)):.4f}s, "
            f"base_obstacles={float(setup_profile.get('rust_registration_base_static_obstacle_add_s', 0.0)):.4f}s, "
            f"base_prefix={float(setup_profile.get('rust_registration_base_prefix_build_s', 0.0)):.4f}s, "
            f"route_extract={float(setup_profile.get('rust_registration_route_extract_s', 0.0)):.4f}s, "
            f"route_cells={float(setup_profile.get('rust_registration_route_cell_collect_s', 0.0)):.4f}s, "
            f"open_sets={float(setup_profile.get('rust_registration_open_set_build_s', 0.0)):.4f}s, "
            f"route_list={float(setup_profile.get('rust_registration_route_cell_list_s', 0.0)):.4f}s, "
            f"route_static={float(setup_profile.get('rust_registration_route_static_add_s', 0.0)):.4f}s, "
            f"store={float(setup_profile.get('rust_registration_registered_store_s', 0.0)):.4f}s, "
            f"routes={int(float(setup_profile.get('rust_registration_route_count', 0.0)))}, "
            f"base_static={int(float(setup_profile.get('rust_registration_base_static_cell_count', 0.0)))}, "
            f"unique_route={int(float(setup_profile.get('rust_registration_unique_route_cell_count', 0.0)))}, "
            f"open_cells={int(float(setup_profile.get('rust_registration_registered_open_cell_count', 0.0)))}"
        )


def _report_rust_planner_profile(report: dict[object, object]) -> None:
    rust_planner_profile = report.get("rust_planner_profile", {})
    if not isinstance(rust_planner_profile, dict) or not rust_planner_profile:
        return
    print(
        "        Rust meander planner split: "
        f"total={float(rust_planner_profile.get('total_s', 0.0)):.4f}s, "
        f"free_interval={float(rust_planner_profile.get('free_interval_s', 0.0)):.4f}s, "
        f"box_check={float(rust_planner_profile.get('box_check_s', 0.0)):.4f}s, "
        f"analytic_plan={float(rust_planner_profile.get('analytic_plan_s', 0.0)):.4f}s, "
        f"replacement_check={float(rust_planner_profile.get('replacement_check_s', 0.0)):.4f}s, "
        f"footprint={float(rust_planner_profile.get('footprint_s', 0.0)):.4f}s, "
        f"run_extraction={float(rust_planner_profile.get('run_extraction_s', 0.0)):.4f}s, "
        f"plan_calls={int(float(rust_planner_profile.get('plan_calls', 0.0)))}, "
        f"depths={int(float(rust_planner_profile.get('depth_count', 0.0)))}, "
        f"run_side_checks={int(float(rust_planner_profile.get('run_side_checks', 0.0)))}, "
        f"box_checks={int(float(rust_planner_profile.get('box_checks', 0.0)))}, "
        f"analytic_calls={int(float(rust_planner_profile.get('analytic_plan_calls', 0.0)))}"
    )


def _report_rust_wrapper_profile(report: dict[object, object]) -> None:
    rust_wrapper_profile = report.get("rust_wrapper_profile", {})
    if not isinstance(rust_wrapper_profile, dict) or not rust_wrapper_profile:
        return
    print(
        "        Rust meander wrapper split: "
        f"planner_call={float(rust_wrapper_profile.get('planner_call_s', 0.0)):.4f}s, "
        f"reserved_snapshot={float(rust_wrapper_profile.get('reserved_snapshot_s', 0.0)):.4f}s, "
        f"rect_cells={float(rust_wrapper_profile.get('selected_rect_cells_s', 0.0)):.4f}s, "
        f"reserved_update={float(rust_wrapper_profile.get('candidate_reserved_update_s', 0.0)):.4f}s, "
        f"py_plan={float(rust_wrapper_profile.get('py_plan_conversion_s', 0.0)):.4f}s, "
        f"py_candidate_result={float(rust_wrapper_profile.get('py_candidate_result_build_s', 0.0)):.4f}s, "
        f"py_result={float(rust_wrapper_profile.get('py_result_build_s', 0.0)):.4f}s, "
        f"prepare_calls={int(float(rust_wrapper_profile.get('extra_blocked_prepare_calls', 0.0)))}, "
        f"rect_cells_count={int(float(rust_wrapper_profile.get('selected_rect_cell_count', 0.0)))}, "
        f"py_plans={int(float(rust_wrapper_profile.get('py_plan_count', 0.0)))}, "
        f"candidate_results={int(float(rust_wrapper_profile.get('candidate_result_count', 0.0)))}"
    )


def _report_meander_commit_profile(report: dict[object, object]) -> None:
    commit_profile = report.get("commit_profile", {})
    if not isinstance(commit_profile, dict) or not commit_profile:
        return
    sorted_commit = sorted(
        commit_profile.items(),
        key=lambda item: -float(item[1]),
    )
    commit_parts = [
        f"{key[:-2] if key.endswith('_s') else key}={float(value):.4f}s"
        for key, value in sorted_commit
    ]
    print("        Meander commit split: " + ", ".join(commit_parts))


def _report_meander_candidate_execution(report: dict[object, object]) -> None:
    print(
        "        Meander candidate execution: "
        f"requirement_batches={int(report.get('requirement_batch_calls', 0))}, "
        f"requirement_batch_candidates={int(report.get('requirement_batch_candidate_calls', 0))}, "
        f"requirement_batch_edge_calls={int(report.get('requirement_batch_edge_calls', 0))}, "
        f"bundle_candidates={int(report.get('bundle_candidate_calls', 0))}, "
        f"bundle_edge_calls={int(report.get('bundle_edge_calls', 0))}, "
        f"bundle_planned={int(report.get('bundle_planned', 0))}, "
        f"bundle_no_candidate={int(report.get('bundle_no_candidate', 0))}"
    )


def _report_candidate_engine_counts(report: dict[object, object]) -> None:
    candidate_engine_counts = report.get("candidate_engine_counts", {})
    if not isinstance(candidate_engine_counts, dict) or not candidate_engine_counts:
        return
    formatted_engine_counts = ", ".join(
        f"{key}={int(value)}"
        for key, value in sorted(candidate_engine_counts.items())
        if isinstance(key, str) and isinstance(value, (int, float))
    )
    if formatted_engine_counts:
        print(f"        Meander candidate engines: {formatted_engine_counts}")


def _report_candidate_setup_profile(report: dict[object, object]) -> None:
    candidate_setup_profile = report.get("candidate_setup_profile", {})
    if not isinstance(candidate_setup_profile, dict) or not candidate_setup_profile:
        return
    sorted_setup = sorted(
        candidate_setup_profile.items(),
        key=lambda item: -float(item[1]),
    )
    setup_parts = [
        f"{key[:-2] if key.endswith('_s') else key}={float(value):.4f}s"
        for key, value in sorted_setup
    ]
    print("        Candidate setup split: " + ", ".join(setup_parts))


def _report_candidate_profile(report: dict[object, object]) -> None:
    candidate_profile = report.get("candidate_profile", {})
    if not isinstance(candidate_profile, dict) or not candidate_profile:
        return
    print("        Candidate planner profile:")
    sorted_profile = sorted(
        candidate_profile.items(),
        key=lambda item: (
            -float(item[1].get("elapsed_s", 0.0))
            if isinstance(item[1], dict)
            else 0.0
        ),
    )
    for reason, raw_profile in sorted_profile:
        if not isinstance(raw_profile, dict):
            continue
        print(
            "          - "
            f"{reason}: candidates={int(raw_profile.get('candidate_attempts', 0))}, "
            f"edge_calls={int(raw_profile.get('edge_calls', 0))}, "
            f"planned={int(raw_profile.get('planned', 0))}, "
            f"no_candidate={int(raw_profile.get('no_candidate', 0))}, "
            f"elapsed={float(raw_profile.get('elapsed_s', 0.0)):.4f}s"
        )


def _report_meander_results(report: dict[object, object]) -> None:
    for entry in report.get("results", []):
        edge = entry.get("edge", {})
        net_name = edge.get("net_name", "<unknown>")
        status = entry.get("status", "<unknown>")
        reason = entry.get("reason", "")
        req = float(entry.get("requested_extra_length_um", 0.0))
        ins = float(entry.get("inserted_extra_length_um", 0.0))
        unmatched = float(entry.get("unmatched_length_um", max(0.0, req - ins)))
        planning_mode = entry.get("planning_mode", None)
        effective_radius = entry.get("effective_bend_radius_um", None)
        primitive_radius = entry.get("primitive_bend_radius_um", None)
        selected_box = entry.get("selected_box", None)
        selected_grid_rect = entry.get("selected_grid_rect", None)
        bumps = entry.get("bumps", None)
        visual_bumps = entry.get("visual_bumps", None)
        u_turns = entry.get("u_turns", None)
        quarter_turns = entry.get("quarter_turns", None)
        side = entry.get("side", None)
        reserved_cells_count = entry.get("reserved_cells_count", None)
        print(
            f"        \u2022 {net_name}: status={status}, requested={req:.3f}um, "
            f"inserted={ins:.3f}um, unmatched={unmatched:.3f}um, "
            f"planning_mode={planning_mode}, side={side}, bumps={bumps}, "
            f"visual_bumps={visual_bumps}, u_turns={u_turns}, "
            f"quarter_turns={quarter_turns}, "
            f"effective_bend_radius_um={effective_radius}, "
            f"primitive_bend_radius_um={primitive_radius}, "
            f"selected_box={selected_box}, selected_grid_rect={selected_grid_rect}, "
            f"reserved_cells_count={reserved_cells_count}, reason={reason}"
        )
