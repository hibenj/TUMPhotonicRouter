"""Crossing and photonic verification stage for routing_flow."""

from __future__ import annotations

import json
import logging
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from gdsfactory.component import Component

from photonic_router.crossings.crossing_verification_report import (
    RouteCostTerms,
    build_crossing_verification_report,
    route_cost_terms_from_mapping,
)
from photonic_router.crossings.route_rust_crossing_components import (
    legal_crossing_component_footprints,
    legal_crossing_overlap_polygons,
)
from photonic_router.flow_component_info import component_info
from photonic_router.flow_stats import FlowRun
from photonic_router.routing.products import LoadedDesign
from photonic_router.routing_layers import get_routing_obstacle_layers
from photonic_router.verify.photonic_verification import (
    LegalCrossings,
    RealizedRouteGeometry,
    VerificationChecks,
    verify_photonic_routing,
)

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class FinalCheckSettings:
    """What the final check of the routed layout is run with."""

    include_heater_obstacles: bool
    #: set when the run was stopped after one route for debugging: the
    #: check then does not ask for every net
    debug_stop_after_route_index: int | None = None
    #: write the layout although it failed, so that it can be looked at
    write_gds_on_failure: bool = False


def verify_and_attach_photonic_reports(
    *,
    run: FlowRun,
    design: LoadedDesign,
    routed_layout: Component,
    debug_artifacts: object,
    settings: FinalCheckSettings,
    extra_report_metadata: Mapping[str, object] | None = None,
) -> None:
    """Run crossing and photonic final-geometry gates and attach report metadata.

    ``extra_report_metadata`` is merged into both verification reports' top
    level and ``metrics`` (e.g. pre-placed crossing grid counters).
    """
    benchmark_name, stats = run.benchmark_name, run.stats
    schematic, unrouted_layout = design.schematic, design.unrouted_layout
    include_heater_obstacles = settings.include_heater_obstacles
    debug_stop_after_route_index = settings.debug_stop_after_route_index
    crossing_plan_info = getattr(debug_artifacts, "crossing_plan_info", None)
    routed_records = tuple(getattr(debug_artifacts, "routed_net_records", ()) or ())
    route_coverage_check_enabled = debug_stop_after_route_index is None
    verification_status = _verification_status_metadata(
        debug_stop_after_route_index=debug_stop_after_route_index,
        expected_route_count=_schematic_route_count(schematic),
        routed_record_count=len(routed_records),
        route_coverage_check_enabled=route_coverage_check_enabled,
    )
    if extra_report_metadata:
        verification_status = {**verification_status, **dict(extra_report_metadata)}
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
            _LOGGER.info(f"      - Crossing verification JSON: {crossing_report['path']}")
            if crossing_report.get("success") is False:
                raise RuntimeError(
                    "Crossing verification failed before GDS write: "
                    f"{crossing_report.get('error_count', 0)} error(s). "
                    f"{_crossing_verification_failure_preview(crossing_report)}"
                )
            _check_mixed_run_crossing_budget(crossing_report, extra_report_metadata)
    elif crossing_plan_info is not None:
        routed_info["crossing_plan"] = crossing_plan_info

    realization_grid_spec = getattr(debug_artifacts, "realization_grid_spec", None)
    if not routed_records or realization_grid_spec is None:
        return
    crossing_plan: Mapping[str, object] = (
        crossing_plan_info if isinstance(crossing_plan_info, Mapping) else {}
    )

    # The check of the realized layout before the GDS is accepted. The router
    # itself checks only the legality of its crossings (`routing.verify_repair`).
    photonic_verification = verify_photonic_routing(
        routed_layout,
        schematic,
        routed_net_records=routed_records,
        unrouted_layout=unrouted_layout,
        geometry=RealizedRouteGeometry(
            obstacle_layers=get_routing_obstacle_layers(
                include_heaters=include_heater_obstacles,
            ),
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=bool(
                getattr(debug_artifacts, "realization_allow_45_degree_turns", True)
            ),
            bend_radius_cells=int(getattr(debug_artifacts, "realization_bend_radius_cells", 4)),
            routable_bbox_um=getattr(debug_artifacts, "routable_bbox_um", None),
        ),
        crossings=LegalCrossings(
            overlap_polygons_by_net_id_pair_um=legal_crossing_overlap_polygons(
                crossing_plan.get("realized_intersections")
            ),
            component_footprints_um=legal_crossing_component_footprints(
                crossing_plan.get("realized_crossing_components"),
                crossing_plan.get("realized_intersections"),
            ),
        ),
        checks=VerificationChecks(
            check_route_coverage=route_coverage_check_enabled, check_endpoint_connectivity=True
        ),
    )
    photonic_report = _write_photonic_verification_report(
        benchmark_name=benchmark_name,
        verification=photonic_verification,
        status_metadata=verification_status,
    )
    routed_info["photonic_verification"] = photonic_report
    if stats is not None:
        stats.verification_error_count = int(getattr(photonic_verification, "error_count", 0))
        stats.verification_warning_count = int(getattr(photonic_verification, "warning_count", 0))
    _LOGGER.info(f"      - Photonic verification JSON: {photonic_report['path']}")
    if photonic_verification.success:
        return
    if settings.write_gds_on_failure:
        _LOGGER.info(
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


def _check_mixed_run_crossing_budget(
    crossing_report: Mapping[str, object],
    extra_report_metadata: Mapping[str, object] | None,
) -> None:
    """Contribution 2 with router fallback: the pre-placed tiles plus the
    crossings the guided search realized in the fallback layers must equal
    the plan's crossing count -- every planned crossing resolved exactly once."""
    grids = (extra_report_metadata or {}).get("preplaced_crossing_grids")
    if not isinstance(grids, Mapping) or not grids.get("router_fallback_net_count"):
        return
    metrics = crossing_report.get("metrics")
    realized = int(metrics.get("crossing_count", 0)) if isinstance(metrics, Mapping) else 0
    tiles = int(grids.get("crossing_component_count", 0))
    plan = int(grids.get("plan_event_count", 0))
    _LOGGER.info(
        f"      - mixed run: {tiles} pre-placed tile(s) + {realized} router crossing(s) "
        f"= {tiles + realized} of {plan} planned"
    )
    if tiles + realized != plan:
        raise RuntimeError(
            f"mixed run resolved {tiles + realized} crossings ({tiles} tiles + {realized} "
            f"router crossings) but the plan has {plan}"
        )


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
