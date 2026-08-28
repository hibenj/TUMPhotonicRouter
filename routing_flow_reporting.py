"""Console reporting helpers for routing_flow.

This module keeps timing and debug-output formatting out of the top-level flow
orchestrator. It should not change routing state.
"""

from pathlib import Path
import webbrowser

from gdsfactory.component import Component

from translation.electrical import ElectricalRoutingResult
from translation.gds_write_options import gds_save_options


def route_attempt_as_dict(record: object) -> dict[str, object]:
    as_dict = getattr(record, "as_dict", None)
    if callable(as_dict):
        result = as_dict()
        if isinstance(result, dict):
            return dict(result)
    if isinstance(record, dict):
        return dict(record)
    return {}


def _route_attempt_float(record: dict[str, object], key: str) -> float:
    value = record.get(key, 0.0)
    try:
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return float(value)
    except (TypeError, ValueError):
        pass
    return 0.0


def _route_attempt_int(record: dict[str, object], key: str) -> int:
    value = record.get(key, 0)
    try:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return int(value)
    except (TypeError, ValueError):
        pass
    return 0


def _route_attempt_duration_s(record: dict[str, object]) -> float:
    route_search_s = _route_attempt_float(record, "route_search_total_time_s")
    if route_search_s > 0.0:
        return route_search_s
    return _route_attempt_float(record, "elapsed_s")


def _format_slowest_route_attempt_lines(
    records: list[dict[str, object]],
    *,
    limit: int = 8,
) -> list[str]:
    timed_records = [record for record in records if _route_attempt_duration_s(record) > 0.0]
    if not timed_records:
        return []

    slowest = sorted(
        timed_records,
        key=_route_attempt_duration_s,
        reverse=True,
    )[: max(1, int(limit))]
    lines: list[str] = []
    for record in slowest:
        elapsed_s = _route_attempt_duration_s(record)
        route_index = _route_attempt_int(record, "route_index")
        attempt_index = _route_attempt_int(record, "attempt_index")
        expanded_states = _route_attempt_int(record, "expanded_states")
        generated_neighbors = _route_attempt_int(record, "generated_neighbors")
        window_attempts = _route_attempt_int(record, "window_attempts")
        dense_grid_cells = _route_attempt_int(record, "dense_grid_cells")
        search_loop_time_s = _route_attempt_float(record, "search_loop_time_s")
        simple_route_time_s = _route_attempt_float(record, "simple_route_time_s")
        commit_time_s = _route_attempt_float(record, "commit_time_s")
        full_grid = bool(record.get("used_full_grid_fallback", False))
        status = "failed" if bool(record.get("failed", False)) else "ok"
        route_kind = "simple" if bool(record.get("used_simple_route", False)) else "astar"
        parts = [
            f"#{attempt_index}",
            f"route[{route_index}]",
            str(record.get("net_name", "<unknown>")),
            str(record.get("bucket_name", "<unknown>")),
            f"{elapsed_s:.4f}s",
            status,
            route_kind,
            f"expanded={expanded_states}",
            f"generated={generated_neighbors}",
            f"windows={window_attempts}",
        ]
        if dense_grid_cells:
            parts.append(f"dense_cells={dense_grid_cells}")
        if search_loop_time_s > 0.0:
            parts.append(f"search_loop={search_loop_time_s:.4f}s")
        if simple_route_time_s > 0.0:
            parts.append(f"simple_probe={simple_route_time_s:.4f}s")
        if commit_time_s > 0.0:
            parts.append(f"commit={commit_time_s:.4f}s")
        if full_grid:
            parts.append("full_grid")
        lines.append("            - " + ", ".join(parts))
    return lines


def _format_slowest_route_net_lines(
    records: list[dict[str, object]],
    *,
    limit: int = 8,
) -> list[str]:
    grouped: dict[tuple[int, str], dict[str, object]] = {}
    for record in records:
        elapsed_s = _route_attempt_duration_s(record)
        if elapsed_s <= 0.0:
            continue
        route_index = _route_attempt_int(record, "route_index")
        net_name = str(record.get("net_name", "<unknown>"))
        key = (route_index, net_name)
        group = grouped.setdefault(
            key,
            {
                "route_index": route_index,
                "net_name": net_name,
                "elapsed_s": 0.0,
                "attempts": 0,
                "failures": 0,
                "expanded_states": 0,
                "generated_neighbors": 0,
                "buckets": set(),
            },
        )
        group["elapsed_s"] = _route_attempt_float(group, "elapsed_s") + elapsed_s
        group["attempts"] = _route_attempt_int(group, "attempts") + 1
        group["failures"] = _route_attempt_int(group, "failures") + int(
            bool(record.get("failed", False))
        )
        group["expanded_states"] = _route_attempt_int(
            group,
            "expanded_states",
        ) + _route_attempt_int(
            record,
            "expanded_states",
        )
        group["generated_neighbors"] = _route_attempt_int(
            group,
            "generated_neighbors",
        ) + _route_attempt_int(
            record,
            "generated_neighbors",
        )
        buckets = group["buckets"]
        if isinstance(buckets, set):
            buckets.add(str(record.get("bucket_name", "<unknown>")))

    if not grouped:
        return []

    slowest = sorted(
        grouped.values(),
        key=lambda group: _route_attempt_float(group, "elapsed_s"),
        reverse=True,
    )[: max(1, int(limit))]
    lines: list[str] = []
    for group in slowest:
        buckets = group.get("buckets", set())
        bucket_text = (
            "/".join(sorted(buckets)) if isinstance(buckets, set) and buckets else "<unknown>"
        )
        parts = [
            f"route[{_route_attempt_int(group, 'route_index')}]",
            str(group["net_name"]),
            f"{_route_attempt_float(group, 'elapsed_s'):.4f}s",
            f"attempts={_route_attempt_int(group, 'attempts')}",
            f"failures={_route_attempt_int(group, 'failures')}",
            f"expanded={_route_attempt_int(group, 'expanded_states')}",
            f"generated={_route_attempt_int(group, 'generated_neighbors')}",
            f"buckets={bucket_text}",
        ]
        lines.append("            - " + ", ".join(parts))
    return lines


def cleanup_debug_artifacts(benchmark_name: str) -> None:
    """Remove stale debug artifacts for a benchmark before writing fresh ones."""
    prefix = benchmark_name.lower()
    for pattern in (
        f"build/static_obstacles/{prefix}_*.svg",
        f"build/routes/{prefix}_*.svg",
        f"build/routes/{prefix}_*_diagnostics.txt",
        f"build/routes/{prefix}_*_FAILED.txt",
        f"build/crossings/{prefix}_*.json",
        f"build/crossings/{prefix}_*.txt",
        f"build/verification/{prefix}_*.json",
        f"build/electrical/{prefix}_*.svg",
    ):
        for path in Path(".").glob(pattern):
            try:
                path.unlink()
            except OSError:
                pass


def report_partial_debug_artifacts(
    benchmark_name: str,
    *,
    debug_svgs_enabled: bool,
) -> None:
    """Print and open any debug artifacts that exist after a failed run."""
    if not debug_svgs_enabled:
        return
    prefix = benchmark_name.lower()
    build_dir = Path("build")
    obstacle_dir = build_dir / "static_obstacles"
    routes_dir = build_dir / "routes"
    electrical_dir = build_dir / "electrical"
    obstacle_svgs = sorted(obstacle_dir.glob(f"{prefix}_*.svg")) if obstacle_dir.exists() else []
    route_svgs = sorted(routes_dir.glob(f"{prefix}_*.svg")) if routes_dir.exists() else []
    electrical_svgs = (
        sorted(electrical_dir.glob(f"{prefix}_*.svg")) if electrical_dir.exists() else []
    )
    failed_logs = sorted(routes_dir.glob(f"{prefix}_*_FAILED.txt")) if routes_dir.exists() else []

    print("      - Partial debug artifacts:")
    print(f"        static obstacle SVGs: {len(obstacle_svgs)}")
    print(f"        route SVGs: {len(route_svgs)}")
    print(f"        electrical SVGs: {len(electrical_svgs)}")
    print(f"        failure logs: {len(failed_logs)}")
    for failed_log in failed_logs:
        print(f"        failure log: {failed_log}")

    try:
        for svg_path in obstacle_svgs:
            webbrowser.open_new_tab(svg_path.resolve().as_uri())
        for svg_path in route_svgs:
            webbrowser.open_new_tab(svg_path.resolve().as_uri())
        for svg_path in electrical_svgs:
            webbrowser.open_new_tab(svg_path.resolve().as_uri())
    except Exception as e:
        print(f"      - Warning: failed to open partial SVGs automatically: {e}")


def report_optical_timing(
    *,
    route_result: object,
    route_summary: object,
    route_attempt_records: list[dict[str, object]],
    route_time_s: float,
) -> None:
    """Print the detailed optical routing timing report."""
    timings = getattr(route_result, "pipeline_timings_s", {})
    route_nets_time = float(timings.get("route_nets", 0.0))
    plm_analysis_time = float(timings.get("path_length_analysis", 0.0))
    plm_obstacle_time = float(timings.get("meander_obstacle_map", 0.0))
    plm_planning_time = float(timings.get("meander_planning", 0.0))
    route_endpoint_correction_time = float(timings.get("route_endpoint_correction", 0.0))
    realization_time = float(timings.get("route_realization", 0.0))
    plm_total = plm_analysis_time + plm_obstacle_time + plm_planning_time
    known_substage_time = (
        route_nets_time + route_endpoint_correction_time + plm_total + realization_time
    )
    overhead_time = max(0.0, route_time_s - known_substage_time)
    print(
        "      - Optical routing stage time "
        f"(net routing + PLM + realization): {route_time_s:.4f} s"
    )
    print(f"        - net routing phase (obstacles + A* + repairs): {route_nets_time:.4f} s")
    _report_route_nets_subtimings(timings, route_nets_time)
    print(
        "          route search: "
        f"astar_loop={float(route_summary.astar_elapsed_s):.4f}s, "
        f"attempts={int(route_summary.route_attempts)}, "
        f"failures={int(route_summary.route_failures)}, "
        f"simple={int(route_summary.simple_route_count)}/"
        f"{int(route_summary.route_count)}, "
        f"repairs={int(route_summary.repair_count)}"
    )
    endpoint_correction_time_s = float(getattr(route_summary, "endpoint_correction_time_s", 0.0))
    if endpoint_correction_time_s > 0.0:
        print(
            "          endpoint correction: "
            f"time={endpoint_correction_time_s:.4f}s, "
            f"calls={int(getattr(route_summary, 'endpoint_correction_calls', 0))}, "
            f"failures={int(getattr(route_summary, 'endpoint_correction_failures', 0))}"
        )
    print(
        "          A* counters: "
        f"expanded={int(route_summary.expanded_states)}, "
        f"generated={int(route_summary.generated_neighbors)}, "
        f"heap_pushes={int(route_summary.heap_pushes)}, "
        f"heap_pops={int(route_summary.heap_pops)}, "
        f"footprint_checks={int(route_summary.footprint_checks)}, "
        f"rect_checks={int(route_summary.footprint_rect_checks)}, "
        f"full_grid_fallbacks={int(route_summary.full_grid_fallbacks)}"
    )
    print(
        "          Crossing hot path: "
        f"no_contact={int(getattr(route_summary, 'crossing_hotpath_no_contact', 0))}, "
        f"contact_checks={int(getattr(route_summary, 'crossing_hotpath_contact_checks', 0))}, "
        f"static_rejects={int(getattr(route_summary, 'crossing_hotpath_static_rejects', 0))}, "
        f"no_owner={int(getattr(route_summary, 'crossing_hotpath_no_owner_contacts', 0))}, "
        f"single_owner={int(getattr(route_summary, 'crossing_hotpath_single_owner_contacts', 0))}, "
        f"multi_owner={int(getattr(route_summary, 'crossing_hotpath_multi_owner_contacts', 0))}, "
        f"candidate_checks={int(route_summary.crossing_candidate_checks)}, "
        f"accepted={int(route_summary.crossing_accepted)}"
    )
    print(
        "          Crossing hot path detail: "
        f"witness_cells={int(getattr(route_summary, 'crossing_hotpath_witness_cells_scanned', 0))}, "
        f"partner_segments={int(getattr(route_summary, 'crossing_hotpath_partner_segment_checks', 0))}, "
        f"bbox_rejects={int(getattr(route_summary, 'crossing_hotpath_partner_segment_bbox_rejects', 0))}, "
        f"intersections={int(getattr(route_summary, 'crossing_hotpath_intersection_hits', 0))}, "
        f"total={float(getattr(route_summary, 'crossing_hotpath_total_time_us', 0)) / 1_000_000.0:.4f}s, "
        f"owner_scan={float(getattr(route_summary, 'crossing_hotpath_owner_scan_time_us', 0)) / 1_000_000.0:.4f}s, "
        f"segments={float(getattr(route_summary, 'crossing_hotpath_segment_time_us', 0)) / 1_000_000.0:.4f}s, "
        f"reservation={float(getattr(route_summary, 'crossing_hotpath_reservation_time_us', 0)) / 1_000_000.0:.4f}s"
    )
    _report_astar_timed_ops(route_summary)
    _report_slowest_routes(route_attempt_records)
    if plm_total > 0.0:
        print(
            "        - path-length matching phase: "
            f"{plm_total:.4f} s "
            f"(analysis={plm_analysis_time:.4f}s, "
            f"meander_obstacles={plm_obstacle_time:.4f}s, "
            f"meander_planning={plm_planning_time:.4f}s)"
        )
    if route_endpoint_correction_time > 0.0:
        print(f"        - route endpoint correction phase: {route_endpoint_correction_time:.4f} s")
    print(f"        - route realization phase: {realization_time:.4f} s")
    if overhead_time > 1.0e-3:
        print(f"        - stage overhead/reporting: {overhead_time:.4f} s")


def _report_route_nets_subtimings(timings: object, route_nets_time: float) -> None:
    route_nets_subtimings = {
        str(name).removeprefix("route_nets."): float(elapsed_s)
        for name, elapsed_s in dict(timings).items()
        if str(name).startswith("route_nets.")
    }
    if not route_nets_subtimings:
        return
    ordered_subtiming_names = (
        "obstacle_map",
        "router_setup",
        "route_job_build",
        "port_opening_prep",
        "port_opening_batch",
        "static_map_handoff",
        "state_opening_precompute",
        "clearance_exempt_batch",
        "batch_job_pack",
        "native_route_batch",
        "batch_result_processing",
        "endpoint_correction_pack",
        "endpoint_correction_native",
        "endpoint_correction_processing",
        "record_assembly",
        "realized_crossing_overlap_augment",
        "realized_crossing_native_events",
        "realized_crossing_insertion_loss",
        "realized_crossing_verify_intersections",
        "realized_crossing_realized_loss",
        "realized_crossing_refresh_total",
        "photonic_probe_copy",
        "photonic_probe_realize",
        "photonic_probe_crossing_place",
        "photonic_probe_layout_total",
        "photonic_probe_verify",
        "photonic_refresh_total",
        "final_verification_block",
        "direct_realization",
        "debug_artifact_assembly",
    )
    known_route_nets_s = sum(route_nets_subtimings.values())
    parts = [
        f"{name}={route_nets_subtimings[name]:.4f}s"
        for name in ordered_subtiming_names
        if route_nets_subtimings.get(name, 0.0) > 0.0
    ]
    route_nets_other_s = max(0.0, route_nets_time - known_route_nets_s)
    if route_nets_other_s > 1.0e-4:
        parts.append(f"other={route_nets_other_s:.4f}s")
    print("          route_nets split: " + ", ".join(parts))
    _report_native_repair_timings(route_nets_subtimings)


def _report_native_repair_timings(route_nets_subtimings: dict[str, float]) -> None:
    native_repair_timing_names = (
        "ripup",
        "repair_failed_net_wall",
        "reroute_victims_wall",
        "repair_probe_victim_selection",
        "repair_state_reset",
    )
    native_repair_timings = {
        name: float(route_nets_subtimings.get(f"native_batch_{name}", 0.0))
        for name in native_repair_timing_names
    }
    native_repair_total_s = sum(native_repair_timings.values())
    if native_repair_total_s <= 0.0:
        return
    native_search_s = float(route_nets_subtimings.get("native_batch_route_search_total", 0.0))
    native_dense_astar_s = float(route_nets_subtimings.get("native_batch_dense_astar", 0.0))
    print(
        "          native repair profile: "
        f"ripup={native_repair_timings['ripup']:.4f}s, "
        f"current={native_repair_timings['repair_failed_net_wall']:.4f}s, "
        f"victims={native_repair_timings['reroute_victims_wall']:.4f}s, "
        f"selection={native_repair_timings['repair_probe_victim_selection']:.4f}s, "
        f"reset={native_repair_timings['repair_state_reset']:.4f}s, "
        f"repair_total={native_repair_total_s:.4f}s, "
        f"native_search={native_search_s:.4f}s, "
        f"dense_astar={native_dense_astar_s:.4f}s"
    )


def _report_astar_timed_ops(route_summary: object) -> None:
    print(
        "          A* timed ops: "
        f"dense_build={float(route_summary.dense_grid_build_time_us) / 1_000_000.0:.4f}s, "
        f"search_loop={float(route_summary.search_loop_time_us) / 1_000_000.0:.4f}s, "
        f"obstacle_prepare={float(route_summary.obstacle_map_prepare_time_us) / 1_000_000.0:.4f}s, "
        f"simple_probe={float(route_summary.simple_route_time_us) / 1_000_000.0:.4f}s, "
        f"commit_prepare={float(route_summary.commit_prepare_time_us) / 1_000_000.0:.4f}s, "
        f"commit={float(route_summary.commit_time_us) / 1_000_000.0:.4f}s, "
        f"neighbor={float(route_summary.neighbor_generation_time_us) / 1_000_000.0:.4f}s, "
        f"heap={float(route_summary.heap_operation_time_us) / 1_000_000.0:.4f}s, "
        f"legality={float(route_summary.legality_check_time_us) / 1_000_000.0:.4f}s, "
        f"reconstruction={float(route_summary.reconstruction_time_us) / 1_000_000.0:.4f}s"
    )
    timed_search_s = float(route_summary.search_loop_time_us) / 1_000_000.0
    measured_inner_s = (
        float(route_summary.neighbor_generation_time_us)
        + float(route_summary.heap_operation_time_us)
        + float(route_summary.legality_check_time_us)
    ) / 1_000_000.0
    route_overhead_s = (
        float(route_summary.obstacle_map_prepare_time_us)
        + float(route_summary.simple_route_time_us)
        + float(route_summary.commit_prepare_time_us)
        + float(route_summary.commit_time_us)
    ) / 1_000_000.0
    if timed_search_s > 0.0:
        print(
            "          A* loop attribution: "
            f"measured_inner={measured_inner_s:.4f}s, "
            f"other={max(0.0, timed_search_s - measured_inner_s):.4f}s, "
            f"route_overhead={route_overhead_s:.4f}s"
        )


def _report_slowest_routes(route_attempt_records: list[dict[str, object]]) -> None:
    slowest_net_lines = _format_slowest_route_net_lines(
        route_attempt_records,
        limit=8,
    )
    if slowest_net_lines:
        print("          slowest route nets:")
        for line in slowest_net_lines:
            print(line)
    slowest_attempt_lines = _format_slowest_route_attempt_lines(
        route_attempt_records,
        limit=8,
    )
    if slowest_attempt_lines:
        print("          slowest route attempts:")
        for line in slowest_attempt_lines:
            print(line)


def report_and_open_debug_svgs(
    *,
    debug_artifacts: object,
    electrical_result: ElectricalRoutingResult | None,
    debug_route_indices: set[int] | None,
) -> None:
    """Report generated debug SVGs and open them in the default browser."""
    if getattr(debug_artifacts, "obstacle_svg", None) is not None:
        print(f"      - Obstacle SVG: {debug_artifacts.obstacle_svg}")
    if debug_route_indices is None:
        if getattr(debug_artifacts, "route_svgs", None):
            print(f"      - Route SVGs: {len(debug_artifacts.route_svgs)} files")
    else:
        selected = _format_debug_route_indices(debug_route_indices)
        print(
            f"      - Route SVGs: {len(debug_artifacts.route_svgs)} "
            f"selected file(s), route indices: {selected}"
        )

    try:
        if getattr(debug_artifacts, "obstacle_svg", None) is not None:
            obs_path = Path(debug_artifacts.obstacle_svg)
            if obs_path.exists():
                webbrowser.open_new_tab(obs_path.resolve().as_uri())
        for svg in getattr(debug_artifacts, "route_svgs", ()) or ():
            svg_path = Path(svg)
            if svg_path.exists():
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
        if electrical_result is not None:
            for svg in electrical_result.debug_artifacts.values():
                svg_path = Path(svg)
                if svg_path.exists():
                    webbrowser.open_new_tab(svg_path.resolve().as_uri())
    except Exception as e:
        print(f"      - Warning: failed to open SVGs automatically: {e}")


def write_or_show_routed_layout(
    *,
    benchmark_name: str,
    routed_layout: Component,
    show_klayout: bool,
) -> None:
    """Show the final layout in KLayout or write the benchmark GDS."""
    if show_klayout:
        try:
            print("      - Opening routed layout in KLayout...")
            routed_layout.show()
        except Exception as e:
            print(f"      - Warning: failed to open layout in KLayout: {e}")
        return

    print("      - Write GDS...")
    routed_layout.write_gds(f"build/routed_{benchmark_name}.gds", save_options=gds_save_options())


def _format_debug_route_indices(indices: set[int]) -> str:
    if not indices:
        return "<none>"

    ranges: list[str] = []
    sorted_indices = sorted(indices)
    start = sorted_indices[0]
    previous = start
    for index in sorted_indices[1:]:
        if index == previous + 1:
            previous = index
            continue
        ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
        start = index
        previous = index
    ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
    return ",".join(ranges)
