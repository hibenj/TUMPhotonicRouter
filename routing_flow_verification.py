"""Crossing and photonic verification stage for routing_flow."""

from collections.abc import Mapping
import json
import os
from pathlib import Path
from typing import Any, cast

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.routing_layers import get_routing_obstacle_layers
from routing_flow_component_info import component_info
from translation.crossing_verification_report import (
    RouteCostTerms,
    build_crossing_verification_report,
    route_cost_terms_from_mapping,
)
from translation.photonic_verification import verify_photonic_routing


def verify_and_attach_photonic_reports(
    *,
    benchmark_name: str,
    schematic: Schematic,
    unrouted_layout: Component,
    routed_layout: Component,
    debug_artifacts: object,
    include_heater_obstacles: bool,
    debug_stop_after_route_index: int | None,
) -> None:
    """Run crossing and photonic final-geometry gates and attach report metadata."""
    crossing_plan_info = getattr(debug_artifacts, "crossing_plan_info", None)
    routed_records = tuple(getattr(debug_artifacts, "routed_net_records", ()) or ())
    route_coverage_check_enabled = debug_stop_after_route_index is None
    verification_status = _verification_status_metadata(
        debug_stop_after_route_index=debug_stop_after_route_index,
        expected_route_count=_schematic_route_count(schematic),
        routed_record_count=len(routed_records),
        route_coverage_check_enabled=route_coverage_check_enabled,
    )
    routed_info = component_info(routed_layout)
    if isinstance(crossing_plan_info, Mapping):
        routed_info["crossing_plan"] = dict(crossing_plan_info)
        if bool(crossing_plan_info.get("enabled", False)):
            crossing_report = _write_crossing_verification_report(
                benchmark_name=benchmark_name,
                crossing_plan_info=crossing_plan_info,
                status_metadata=verification_status,
            )
            routed_info["crossing_verification"] = crossing_report
            print(f"      - Crossing verification JSON: {crossing_report['path']}")
            if crossing_report.get("success") is False:
                raise RuntimeError(
                    "Crossing verification failed before GDS write: "
                    f"{crossing_report.get('error_count', 0)} error(s). "
                    f"{_crossing_verification_failure_preview(crossing_report)}"
                )
    elif crossing_plan_info is not None:
        routed_info["crossing_plan"] = crossing_plan_info

    realization_grid_spec = getattr(debug_artifacts, "realization_grid_spec", None)
    if not routed_records or realization_grid_spec is None:
        return

    # This Python geometry verifier is the normal final gate for the realized
    # routed layout. Internal photonic probe verification in
    # `translation.route_rust` should be treated as diagnostic/mismatch
    # debugging support; this pass is the authoritative check before the GDS is
    # accepted.
    photonic_verification = verify_photonic_routing(
        routed_layout,
        schematic,
        routed_net_records=routed_records,
        unrouted_layout=unrouted_layout,
        obstacle_layers=get_routing_obstacle_layers(
            include_heaters=include_heater_obstacles,
        ),
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=bool(
            getattr(debug_artifacts, "realization_allow_45_degree_turns", True)
        ),
        bend_radius_cells=int(
            getattr(debug_artifacts, "realization_bend_radius_cells", 4)
        ),
        legal_overlap_polygons_by_net_id_pair_um=(
            _legal_crossing_overlap_polygons(crossing_plan_info)
            if isinstance(crossing_plan_info, Mapping)
            else {}
        ),
        crossing_component_footprints_um=(
            _legal_crossing_component_footprints(crossing_plan_info)
            if isinstance(crossing_plan_info, Mapping)
            else ()
        ),
        check_route_coverage=route_coverage_check_enabled,
        check_endpoint_connectivity=True,
    )
    photonic_report = _write_photonic_verification_report(
        benchmark_name=benchmark_name,
        verification=photonic_verification,
        status_metadata=verification_status,
    )
    routed_info["photonic_verification"] = photonic_report
    print(f"      - Photonic verification JSON: {photonic_report['path']}")
    if photonic_verification.success:
        return
    if os.environ.get(
        "PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE",
        "",
    ).strip().lower() in {"1", "true", "yes", "on"}:
        print(
            "      - WARNING: Photonic geometry verification failed; "
            "writing GDS anyway because "
            "PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE is set."
        )
        return
    raise RuntimeError(
        "Photonic geometry verification failed before GDS write: "
        f"{photonic_verification.error_count} error(s). "
        f"{_photonic_verification_failure_preview(photonic_verification)}"
    )


def _crossing_report_route_cost_terms(
    crossing_plan_info: Mapping[str, object],
) -> list[RouteCostTerms]:
    raw_entries = crossing_plan_info.get("insertion_loss_by_net", ())
    if not isinstance(raw_entries, (list, tuple)):
        return []

    terms: list[RouteCostTerms] = []
    for entry in raw_entries:
        if not isinstance(entry, Mapping):
            continue
        terms.append(
            route_cost_terms_from_mapping(
                {
                    "net_id": entry.get("net_id"),
                    "net_name": entry.get("net_name"),
                    "length_um": entry.get("length_um"),
                    "length_loss": entry.get("propagation_loss"),
                    "bend_loss": entry.get("bend_loss"),
                    "crossing_loss": entry.get("crossing_loss"),
                    "physical_insertion_loss": entry.get("insertion_loss"),
                }
            )
        )
    return terms


def _write_crossing_verification_report(
    *,
    benchmark_name: str,
    crossing_plan_info: Mapping[str, object],
    status_metadata: Mapping[str, object] | None = None,
    output_dir: Path = Path("build") / "verification",
) -> dict[str, object]:
    realized_crossing_components = crossing_plan_info.get(
        "realized_crossing_components",
        (),
    )
    if not isinstance(realized_crossing_components, (list, tuple)):
        realized_crossing_components = ()
    report = build_crossing_verification_report(
        crossing_plan_info=crossing_plan_info,
        realized_crossing_components=realized_crossing_components,
        route_cost_terms=_crossing_report_route_cost_terms(crossing_plan_info),
    )
    output_path = output_dir / f"{benchmark_name.lower()}_crossing_verification.json"
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = report.as_dict()
    _apply_verification_status_metadata(payload, status_metadata)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    payload["path"] = str(output_path)
    return payload


def _crossing_verification_failure_preview(payload: Mapping[str, object]) -> str:
    issues = payload.get("issues", ())
    if not isinstance(issues, list):
        return ""
    lines: list[str] = []
    for raw_issue in issues[:5]:
        if not isinstance(raw_issue, Mapping):
            continue
        code = raw_issue.get("code", "unknown")
        net_name = raw_issue.get("net_name")
        message = raw_issue.get("message", "")
        details = raw_issue.get("details", {})
        reason = None
        if isinstance(details, Mapping):
            reason = details.get("reason")
            if reason is None:
                nested = details.get("details")
                if isinstance(nested, Mapping):
                    reason = nested.get("reason")
        suffix = f" ({reason})" if reason else ""
        lines.append(f"{code} {net_name or '<unknown>'}: {message}{suffix}")
    if len(issues) > 5:
        lines.append(f"... {len(issues) - 5} more")
    return "; ".join(lines)


def _legal_crossing_overlap_polygons(
    crossing_plan_info: Mapping[str, object] | None,
) -> dict[tuple[int, int], tuple[tuple[tuple[float, float], ...], ...]]:
    if crossing_plan_info is None:
        return {}
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, (list, tuple)):
        return {}

    polygons_by_pair: dict[tuple[int, int], list[tuple[tuple[float, float], ...]]] = {}
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        try:
            net_id_a = int(raw_crossing["net_id_a"])
            net_id_b = int(raw_crossing["net_id_b"])
        except (KeyError, TypeError, ValueError):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if not isinstance(raw_polygon, (list, tuple)) or len(raw_polygon) < 3:
            continue
        points: list[tuple[float, float]] = []
        for raw_point in raw_polygon:
            if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
                points = []
                break
            try:
                points.append((float(raw_point[0]), float(raw_point[1])))
            except (TypeError, ValueError):
                points = []
                break
        if len(points) >= 3:
            pair = (net_id_a, net_id_b) if net_id_a <= net_id_b else (net_id_b, net_id_a)
            polygons_by_pair.setdefault(pair, []).append(tuple(points))
    return {pair: tuple(polygons) for pair, polygons in polygons_by_pair.items()}


def _legal_crossing_component_footprints(
    crossing_plan_info: Mapping[str, object] | None,
) -> tuple[dict[str, object], ...]:
    if crossing_plan_info is None:
        return ()
    raw_components = crossing_plan_info.get("realized_crossing_components", ())
    if isinstance(raw_components, (list, tuple)):
        components = [
            dict(component)
            for component in raw_components
            if isinstance(component, Mapping)
        ]
        if components:
            return tuple(components)
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, (list, tuple)):
        return ()

    footprints: list[dict[str, object]] = []
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if not isinstance(raw_polygon, (list, tuple)) or len(raw_polygon) < 3:
            continue
        footprints.append(dict(raw_crossing))
    return tuple(footprints)


def _schematic_route_count(schematic: object) -> int:
    routes = getattr(getattr(schematic, "netlist", None), "routes", {})
    if not isinstance(routes, Mapping):
        return 0
    count = 0
    for bundle in routes.values():
        links = getattr(bundle, "links", None)
        if isinstance(links, Mapping):
            count += len(links)
        else:
            count += 1
    return count


def _verification_status_metadata(
    *,
    debug_stop_after_route_index: int | None,
    expected_route_count: int,
    routed_record_count: int,
    route_coverage_check_enabled: bool,
) -> dict[str, object]:
    partial = debug_stop_after_route_index is not None
    missing_route_count = max(0, int(expected_route_count) - int(routed_record_count))
    return {
        "status": "partial_debug_stop" if partial else "complete",
        "partial": partial,
        "debug_stop_after_route_index": debug_stop_after_route_index,
        "expected_route_count": int(expected_route_count),
        "routed_record_count": int(routed_record_count),
        "missing_route_count": missing_route_count,
        "route_coverage_check_enabled": bool(route_coverage_check_enabled),
    }


def _apply_verification_status_metadata(
    payload: dict[str, object],
    status_metadata: Mapping[str, object] | None,
) -> None:
    if not status_metadata:
        return
    payload.update(dict(status_metadata))
    metrics = payload.setdefault("metrics", {})
    if isinstance(metrics, dict):
        metrics.update(dict(status_metadata))


def _write_photonic_verification_report(
    *,
    benchmark_name: str,
    verification: object,
    status_metadata: Mapping[str, object] | None = None,
    output_dir: Path = Path("build") / "verification",
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{benchmark_name.lower()}_photonic_verification.json"
    payload = cast(Any, verification).as_dict()
    _apply_verification_status_metadata(payload, status_metadata)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    payload["path"] = str(output_path)
    return payload


def _photonic_verification_failure_preview(verification: object) -> str:
    issues = tuple(getattr(verification, "issues", ()) or ())
    lines: list[str] = []
    for issue in issues[:5]:
        code = getattr(issue, "code", "unknown")
        net_name = getattr(issue, "net_name", None)
        message = getattr(issue, "message", "")
        details = getattr(issue, "details", {}) or {}
        bbox = details.get("overlap_bbox_um") if isinstance(details, Mapping) else None
        area = details.get("overlap_area_um2") if isinstance(details, Mapping) else None
        suffix_parts = []
        if area is not None:
            suffix_parts.append(f"area={area}")
        if bbox is not None:
            suffix_parts.append(f"bbox={bbox}")
        suffix = f" ({', '.join(suffix_parts)})" if suffix_parts else ""
        lines.append(f"{code} {net_name or '<unknown>'}: {message}{suffix}")
    if len(issues) > 5:
        lines.append(f"... {len(issues) - 5} more")
    return "; ".join(lines)
