"""Structured crossing verification reports.

This module is intentionally small and data-oriented. It does not call the
router or gdsfactory, so it can be used by focused fixtures and later wired into
benchmark runs without adding work to the A* hot path.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any

Point = tuple[float, float]


@dataclass(frozen=True)
class CrossingVerificationIssue:
    code: str
    message: str
    severity: str = "error"
    net_name: str | None = None
    details: dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict[str, object]:
        return {
            "code": self.code,
            "message": self.message,
            "severity": self.severity,
            "net_name": self.net_name,
            "details": dict(self.details),
        }


@dataclass(frozen=True)
class CrossingRecord:
    classification: str
    point_um: Point | None = None
    net_id_a: int | None = None
    net_id_b: int | None = None
    net_name_a: str | None = None
    net_name_b: str | None = None
    reason: str | None = None
    details: dict[str, object] = field(default_factory=dict)

    @property
    def legal(self) -> bool:
        return self.classification.startswith("legal_")

    @property
    def illegal(self) -> bool:
        return self.classification.startswith("illegal_")

    def as_dict(self) -> dict[str, object]:
        return {
            "classification": self.classification,
            "point_um": list(self.point_um) if self.point_um is not None else None,
            "net_id_a": self.net_id_a,
            "net_id_b": self.net_id_b,
            "net_name_a": self.net_name_a,
            "net_name_b": self.net_name_b,
            "reason": self.reason,
            "details": dict(self.details),
        }


@dataclass(frozen=True)
class RouteCostTerms:
    net_id: int | None = None
    net_name: str | None = None
    length_um: float = 0.0
    length_loss: float = 0.0
    bend_loss: float = 0.0
    crossing_loss: float = 0.0
    total_search_cost: float | None = None
    history_cost: float = 0.0
    congestion_cost: float = 0.0
    other_search_guidance_cost: float = 0.0
    physical_insertion_loss: float | None = None

    @property
    def total_physical_insertion_loss(self) -> float:
        if self.physical_insertion_loss is not None:
            return float(self.physical_insertion_loss)
        return float(self.length_loss + self.bend_loss + self.crossing_loss)

    @property
    def search_guidance_penalty(self) -> float:
        if self.total_search_cost is not None:
            return max(0.0, float(self.total_search_cost) - self.total_physical_insertion_loss)
        return float(
            self.history_cost + self.congestion_cost + self.other_search_guidance_cost
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "net_id": self.net_id,
            "net_name": self.net_name,
            "length_um": float(self.length_um),
            "length_loss": float(self.length_loss),
            "bend_loss": float(self.bend_loss),
            "crossing_loss": float(self.crossing_loss),
            "total_physical_insertion_loss": self.total_physical_insertion_loss,
            "total_search_cost": self.total_search_cost,
            "history_cost": float(self.history_cost),
            "congestion_cost": float(self.congestion_cost),
            "other_search_guidance_cost": float(self.other_search_guidance_cost),
            "search_guidance_penalty": self.search_guidance_penalty,
        }


@dataclass(frozen=True)
class CrossingVerificationReport:
    issues: tuple[CrossingVerificationIssue, ...]
    crossings: tuple[CrossingRecord, ...]
    route_costs: tuple[RouteCostTerms, ...] = ()
    crossing_component: dict[str, object] = field(default_factory=dict)
    metrics: dict[str, object] = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return self.error_count == 0

    @property
    def error_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "error")

    @property
    def warning_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "warning")

    def as_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "error_count": self.error_count,
            "warning_count": self.warning_count,
            "metrics": dict(self.metrics),
            "crossing_component": dict(self.crossing_component),
            "crossings": [crossing.as_dict() for crossing in self.crossings],
            "route_costs": [cost.as_dict() for cost in self.route_costs],
            "issues": [issue.as_dict() for issue in self.issues],
        }


def build_crossing_verification_report(
    *,
    crossing_plan_info: Mapping[str, object] | None,
    realized_crossing_components: Iterable[Mapping[str, object]] = (),
    protected_segments: Iterable[Mapping[str, object]] = (),
    route_cost_terms: Iterable[RouteCostTerms | Mapping[str, object]] = (),
    component_match_tolerance_um: float = 0.25,
    component_rotation_tolerance_deg: float = 1.0,
) -> CrossingVerificationReport:
    """Build a structured report from already-collected crossing diagnostics."""

    plan_info = dict(crossing_plan_info or {})
    crossings = _crossing_records_from_plan(plan_info)
    issues: list[CrossingVerificationIssue] = []
    issues.extend(_issues_from_crossings(crossings))

    component_info = dict(_as_mapping(plan_info.get("crossing_device")))
    component_issues, component_metrics = _component_placement_issues(
        intended_crossings=[crossing for crossing in crossings if crossing.legal],
        realized_components=tuple(realized_crossing_components),
        component_info=component_info,
        tolerance_um=float(component_match_tolerance_um),
        rotation_tolerance_deg=float(component_rotation_tolerance_deg),
    )
    issues.extend(component_issues)

    protected_issues = _protected_segment_issues(protected_segments)
    issues.extend(protected_issues)

    costs = tuple(_coerce_cost_terms(item) for item in route_cost_terms)
    metrics = _build_metrics(
        plan_info=plan_info,
        crossings=crossings,
        issues=issues,
        protected_issue_count=len(protected_issues),
        component_metrics=component_metrics,
        route_costs=costs,
    )

    return CrossingVerificationReport(
        issues=tuple(issues),
        crossings=tuple(crossings),
        route_costs=costs,
        crossing_component=component_info,
        metrics=metrics,
    )


def route_cost_terms_from_mapping(values: Mapping[str, object]) -> RouteCostTerms:
    return _coerce_cost_terms(values)


def write_crossing_verification_report(
    report: CrossingVerificationReport,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return output


def _crossing_records_from_plan(plan_info: Mapping[str, object]) -> list[CrossingRecord]:
    raw_records = []
    raw_records.extend(_iter_mappings(plan_info.get("realized_intersections")))
    raw_records.extend(_iter_mappings(plan_info.get("illegal_realized_crossings")))

    seen: set[tuple[object, ...]] = set()
    records: list[CrossingRecord] = []
    for source_index, raw in enumerate(raw_records):
        classification = str(raw.get("classification", "") or "")
        if not classification:
            reason = str(raw.get("reason", "") or "")
            classification = "illegal_unexpected_crossing" if reason else "unknown"
        point = _as_point(raw.get("point_um", raw.get("point")))
        key = (
            classification,
            point,
            _as_int(raw.get("net_id_a")),
            _as_int(raw.get("net_id_b")),
            str(raw.get("reason", "") or ""),
        )
        if key in seen:
            continue
        seen.add(key)
        details = dict(raw)
        details.setdefault("source_crossing_index", source_index)
        records.append(
            CrossingRecord(
                classification=classification,
                point_um=point,
                net_id_a=_as_int(raw.get("net_id_a")),
                net_id_b=_as_int(raw.get("net_id_b")),
                net_name_a=_as_str_or_none(raw.get("net_name_a")),
                net_name_b=_as_str_or_none(raw.get("net_name_b")),
                reason=_as_str_or_none(raw.get("reason")),
                details=details,
            )
        )
    return records


def _issues_from_crossings(
    crossings: Iterable[CrossingRecord],
) -> list[CrossingVerificationIssue]:
    issues: list[CrossingVerificationIssue] = []
    for crossing in crossings:
        if crossing.illegal:
            issues.append(
                CrossingVerificationIssue(
                    code="illegal_realized_crossing",
                    message=_crossing_message(crossing),
                    net_name=crossing.net_name_a,
                    details=crossing.as_dict(),
                )
            )
            continue

        if not crossing.legal:
            continue

        illegal_reason = _illegal_legal_crossing_reason(crossing)
        if illegal_reason is None:
            continue
        issues.append(
            CrossingVerificationIssue(
                code="illegal_realized_crossing",
                message=(
                    "A realized crossing was marked legal but cannot be "
                    f"represented by the physical crossing component: {illegal_reason}."
                ),
                net_name=crossing.net_name_a,
                details={
                    **crossing.as_dict(),
                    "reason": illegal_reason,
                },
            )
        )
    return issues


def _illegal_legal_crossing_reason(crossing: CrossingRecord) -> str | None:
    degraded_reason = _as_str_or_none(crossing.details.get("degraded_reason"))
    if degraded_reason:
        return degraded_reason

    perpendicular = crossing.details.get("perpendicular")
    if perpendicular is False:
        return "not_perpendicular"

    segment_a = _segment_from_details(crossing.details.get("segment_a_um"))
    segment_b = _segment_from_details(crossing.details.get("segment_b_um"))
    if segment_a is None or segment_b is None:
        return None
    if not _segments_are_perpendicular(segment_a, segment_b):
        return "not_perpendicular"
    return None


def _component_placement_issues(
    *,
    intended_crossings: list[CrossingRecord],
    realized_components: tuple[Mapping[str, object], ...],
    component_info: Mapping[str, object],
    tolerance_um: float,
    rotation_tolerance_deg: float,
) -> tuple[list[CrossingVerificationIssue], dict[str, object]]:
    issues: list[CrossingVerificationIssue] = []
    matched_component_indices: set[int] = set()
    expected_name = _as_str_or_none(component_info.get("component_name"))
    expected_bbox = _as_float_pair(component_info.get("component_bbox_um"))

    for crossing in intended_crossings:
        if crossing.point_um is None:
            continue
        shared_cluster_match = False
        match_index, match = _nearest_component(
            crossing.point_um,
            realized_components,
            already_matched=matched_component_indices,
            tolerance_um=tolerance_um,
        )
        if match is None and _allows_shared_cluster_component(crossing):
            match_index, match = _cluster_component_match(
                crossing,
                realized_components,
            )
            shared_cluster_match = match is not None
        if match is None:
            issues.append(
                CrossingVerificationIssue(
                    code="missing_crossing_component",
                    message=(
                        "No realized crossing component was found at legal "
                        f"crossing {crossing.point_um}."
                    ),
                    net_name=crossing.net_name_a,
                    details=crossing.as_dict(),
                )
            )
            continue
        matched_component_indices.add(match_index)
        actual_name = _as_str_or_none(match.get("component_name"))
        if expected_name and actual_name and actual_name != expected_name:
            issues.append(
                CrossingVerificationIssue(
                    code="crossing_component_name_mismatch",
                    message=(
                        f"Crossing component {actual_name!r} does not match "
                        f"expected {expected_name!r}."
                    ),
                    net_name=crossing.net_name_a,
                    details={
                        "expected_component_name": expected_name,
                        "actual_component_name": actual_name,
                        "crossing": crossing.as_dict(),
                    },
                )
            )
        actual_bbox = _as_float_pair(match.get("component_bbox_um"))
        if expected_bbox is not None and actual_bbox is not None:
            bbox_error = max(
                abs(expected_bbox[0] - actual_bbox[0]),
                abs(expected_bbox[1] - actual_bbox[1]),
            )
            if bbox_error > tolerance_um:
                issues.append(
                    CrossingVerificationIssue(
                        code="crossing_component_bbox_mismatch",
                        message="Realized crossing component bbox differs from expected bbox.",
                        net_name=crossing.net_name_a,
                        details={
                            "expected_component_bbox_um": list(expected_bbox),
                            "actual_component_bbox_um": list(actual_bbox),
                            "max_error_um": bbox_error,
                            "tolerance_um": tolerance_um,
                            "crossing": crossing.as_dict(),
                        },
                    )
                )
        expected_rotation = (
            None if shared_cluster_match else _crossing_axis_rotation_deg(crossing)
        )
        actual_rotation = _as_float_or_none(match.get("rotation_deg"))
        if expected_rotation is not None and actual_rotation is None:
            issues.append(
                CrossingVerificationIssue(
                    code="crossing_component_rotation_missing",
                    message="Realized crossing component is missing rotation metadata.",
                    net_name=crossing.net_name_a,
                    details={
                        "expected_rotation_deg": expected_rotation,
                        "crossing": crossing.as_dict(),
                        "component": dict(match),
                    },
                )
            )
        elif expected_rotation is not None and actual_rotation is not None:
            rotation_error = _axis_angle_error_deg(actual_rotation, expected_rotation)
            if rotation_error > rotation_tolerance_deg:
                issues.append(
                    CrossingVerificationIssue(
                        code="crossing_component_rotation_mismatch",
                        message="Realized crossing component rotation does not match crossing axis.",
                        net_name=crossing.net_name_a,
                        details={
                            "expected_rotation_deg": expected_rotation,
                            "actual_rotation_deg": actual_rotation,
                            "axis_rotation_error_deg": rotation_error,
                            "tolerance_deg": rotation_tolerance_deg,
                            "crossing": crossing.as_dict(),
                        },
                    )
                )

    for index, component in enumerate(realized_components):
        if index in matched_component_indices:
            continue
        point = _component_point(component)
        issues.append(
            CrossingVerificationIssue(
                code="unexpected_crossing_component",
                message=f"Unexpected realized crossing component at {point}.",
                details=dict(component),
            )
        )

    metrics = {
        "intended_crossing_component_count": len(
            [crossing for crossing in intended_crossings if crossing.point_um is not None]
        ),
        "realized_crossing_component_count": len(realized_components),
        "matched_crossing_component_count": len(matched_component_indices),
    }
    return issues, metrics


def _allows_shared_cluster_component(crossing: CrossingRecord) -> bool:
    return str(crossing.details.get("footprint_overlap_policy", "") or "") in {
        "allowed_lidar_pure_cluster",
        "allowed_lidar_pure_degraded_cluster",
    }


def _cluster_peer_indices(crossing: CrossingRecord) -> set[int]:
    indices: set[int] = set()
    raw_indices = crossing.details.get("overlapping_crossing_indices")
    if isinstance(raw_indices, (list, tuple, set)):
        for raw_index in raw_indices:
            value = _as_int(raw_index)
            if value is not None:
                indices.add(value)
    value = _as_int(crossing.details.get("overlapping_crossing_index"))
    if value is not None:
        indices.add(value)
    value = _as_int(crossing.details.get("source_crossing_index"))
    if value is not None:
        indices.add(value)
    return indices


def _cluster_component_match(
    crossing: CrossingRecord,
    realized_components: tuple[Mapping[str, object], ...],
) -> tuple[int, Mapping[str, object] | None]:
    peer_indices = _cluster_peer_indices(crossing)
    if not peer_indices:
        return -1, None
    for index, component in enumerate(realized_components):
        source_index = _as_int(component.get("source_crossing_index"))
        if source_index is not None and source_index in peer_indices:
            return index, component
        raw_shared_indices = component.get("shared_crossing_indices")
        if isinstance(raw_shared_indices, (list, tuple, set)):
            for raw_shared_index in raw_shared_indices:
                shared_index = _as_int(raw_shared_index)
                if shared_index is not None and shared_index in peer_indices:
                    return index, component
    return -1, None


def _protected_segment_issues(
    protected_segments: Iterable[Mapping[str, object]],
) -> list[CrossingVerificationIssue]:
    issues: list[CrossingVerificationIssue] = []
    for index, segment in enumerate(protected_segments):
        intended_start = _as_point(segment.get("start_um"))
        intended_end = _as_point(segment.get("end_um"))
        realized_start = _as_point(segment.get("realized_start_um"))
        realized_end = _as_point(segment.get("realized_end_um"))
        if None in (intended_start, intended_end, realized_start, realized_end):
            issues.append(
                CrossingVerificationIssue(
                    code="protected_segment_invalid",
                    message=f"Protected segment {index} is missing endpoint data.",
                    net_name=_as_str_or_none(segment.get("net_name")),
                    details=dict(segment),
                )
            )
            continue

        tolerance_um = _as_float(segment.get("tolerance_um"), 1.0e-6)
        direct_error = max(
            _distance(intended_start, realized_start),
            _distance(intended_end, realized_end),
        )
        reversed_error = max(
            _distance(intended_start, realized_end),
            _distance(intended_end, realized_start),
        )
        best_error = min(direct_error, reversed_error)
        if best_error <= tolerance_um:
            continue
        issues.append(
            CrossingVerificationIssue(
                code="protected_segment_moved",
                message=(
                    "Endpoint correction or realization moved a protected "
                    f"crossing segment by {best_error:.6g}um."
                ),
                net_name=_as_str_or_none(segment.get("net_name")),
                details={
                    **dict(segment),
                    "max_endpoint_error_um": best_error,
                    "tolerance_um": tolerance_um,
                },
            )
        )
    return issues


def _build_metrics(
    *,
    plan_info: Mapping[str, object],
    crossings: Iterable[CrossingRecord],
    issues: Iterable[CrossingVerificationIssue],
    protected_issue_count: int,
    component_metrics: Mapping[str, object],
    route_costs: Iterable[RouteCostTerms],
) -> dict[str, object]:
    crossing_list = list(crossings)
    issue_list = list(issues)
    cost_list = list(route_costs)
    metrics: dict[str, object] = {
        "crossing_enabled": bool(plan_info.get("enabled", False)),
        "crossing_count": len(crossing_list),
        "legal_crossing_count": sum(1 for crossing in crossing_list if crossing.legal),
        "illegal_crossing_count": sum(1 for crossing in crossing_list if crossing.illegal),
        "issue_count": len(issue_list),
        "error_count": sum(1 for issue in issue_list if issue.severity == "error"),
        "warning_count": sum(1 for issue in issue_list if issue.severity == "warning"),
        "protected_segment_issue_count": protected_issue_count,
        "route_cost_count": len(cost_list),
        "total_physical_insertion_loss": sum(
            cost.total_physical_insertion_loss for cost in cost_list
        ),
        "total_search_guidance_penalty": sum(
            cost.search_guidance_penalty for cost in cost_list
        ),
    }
    metrics.update(component_metrics)
    total_search_costs = [
        float(cost.total_search_cost)
        for cost in cost_list
        if cost.total_search_cost is not None
    ]
    if total_search_costs:
        metrics["total_search_cost"] = sum(total_search_costs)
    final_crossing_repair_attempts = tuple(
        _iter_mappings(plan_info.get("final_crossing_repair_attempts"))
    )
    final_photonic_repair_attempts = tuple(
        _iter_mappings(plan_info.get("final_photonic_repair_attempts"))
    )
    metrics["final_crossing_repair_attempt_count"] = len(
        final_crossing_repair_attempts
    )
    metrics["final_photonic_repair_attempt_count"] = len(
        final_photonic_repair_attempts
    )
    metrics["final_crossing_repair_success_count"] = sum(
        1
        for attempt in final_crossing_repair_attempts
        if str(attempt.get("status", "") or "") == "routed"
    )
    metrics["final_photonic_repair_success_count"] = sum(
        1
        for attempt in final_photonic_repair_attempts
        if str(attempt.get("status", "") or "") == "routed"
        and not attempt.get("endpoint_correction_failed_net_ids")
    )
    return metrics


def _nearest_component(
    point: Point,
    components: tuple[Mapping[str, object], ...],
    *,
    already_matched: set[int],
    tolerance_um: float,
) -> tuple[int, Mapping[str, object] | None]:
    best_index = -1
    best_component: Mapping[str, object] | None = None
    best_distance = math.inf
    for index, component in enumerate(components):
        if index in already_matched:
            continue
        component_point = _component_point(component)
        if component_point is None:
            continue
        distance = _distance(point, component_point)
        if distance < best_distance:
            best_index = index
            best_component = component
            best_distance = distance
    if best_distance <= tolerance_um:
        return best_index, best_component
    return -1, None


def _component_point(component: Mapping[str, object]) -> Point | None:
    return _as_point(component.get("point_um", component.get("center_um")))


def _crossing_axis_rotation_deg(crossing: CrossingRecord) -> float | None:
    segment = _segment_from_details(crossing.details.get("segment_a_um"))
    if segment is None:
        return None
    start, end = segment
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if math.hypot(dx, dy) <= 1.0e-9:
        return None
    return float(math.degrees(math.atan2(dy, dx)) % 360.0)


def _segment_from_details(value: object) -> tuple[Point, Point] | None:
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    start = _as_point(value[0])
    end = _as_point(value[1])
    if start is None or end is None:
        return None
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if math.hypot(dx, dy) <= 1.0e-9:
        return None
    return start, end


def _segments_are_perpendicular(
    segment_a: tuple[Point, Point],
    segment_b: tuple[Point, Point],
    *,
    tolerance: float = 1.0e-6,
) -> bool:
    (ax0, ay0), (ax1, ay1) = segment_a
    (bx0, by0), (bx1, by1) = segment_b
    adx = ax1 - ax0
    ady = ay1 - ay0
    bdx = bx1 - bx0
    bdy = by1 - by0
    len_a = math.hypot(adx, ady)
    len_b = math.hypot(bdx, bdy)
    if len_a <= 1.0e-9 or len_b <= 1.0e-9:
        return False
    dot = (adx * bdx + ady * bdy) / (len_a * len_b)
    return abs(dot) <= float(tolerance)


def _axis_angle_error_deg(actual_deg: float, expected_deg: float) -> float:
    return abs((float(actual_deg) - float(expected_deg) + 90.0) % 180.0 - 90.0)


def _crossing_message(crossing: CrossingRecord) -> str:
    names = " x ".join(
        name for name in (crossing.net_name_a, crossing.net_name_b) if name
    )
    suffix = f" for {names}" if names else ""
    reason = f": {crossing.reason}" if crossing.reason else ""
    return f"Illegal realized crossing{suffix}{reason}."


def _coerce_cost_terms(item: RouteCostTerms | Mapping[str, object]) -> RouteCostTerms:
    if isinstance(item, RouteCostTerms):
        return item
    return RouteCostTerms(
        net_id=_as_int(item.get("net_id")),
        net_name=_as_str_or_none(item.get("net_name")),
        length_um=_as_float(item.get("length_um"), 0.0),
        length_loss=_as_float(item.get("length_loss"), 0.0),
        bend_loss=_as_float(item.get("bend_loss"), 0.0),
        crossing_loss=_as_float(item.get("crossing_loss"), 0.0),
        total_search_cost=_as_float_or_none(item.get("total_search_cost")),
        history_cost=_as_float(item.get("history_cost"), 0.0),
        congestion_cost=_as_float(item.get("congestion_cost"), 0.0),
        other_search_guidance_cost=_as_float(
            item.get("other_search_guidance_cost"), 0.0
        ),
        physical_insertion_loss=_as_float_or_none(
            item.get("physical_insertion_loss")
        ),
    )


def _iter_mappings(value: object) -> Iterable[Mapping[str, object]]:
    if not isinstance(value, Iterable) or isinstance(value, (str, bytes, bytearray)):
        return ()
    return tuple(item for item in value if isinstance(item, Mapping))


def _as_mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _as_point(value: object) -> Point | None:
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    try:
        x = float(value[0])
        y = float(value[1])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return x, y


def _as_float_pair(value: object) -> tuple[float, float] | None:
    point = _as_point(value)
    if point is None:
        return None
    return point


def _as_float(value: object, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _as_float_or_none(value: object) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _as_int(value: object) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_str_or_none(value: object) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None


def _distance(a: Point, b: Point) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))
