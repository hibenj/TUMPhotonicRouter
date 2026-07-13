"""Post-route verification for photonic waveguide routing."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from collections.abc import Mapping
from typing import Any, Iterable

from gdsfactory.component import Component
import klayout.db as kdb

from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_types import RoutedNetRecord

Layer = tuple[int, int]
PortEndpoint = tuple[str, str]
RouteKey = tuple[str, PortEndpoint, PortEndpoint]
NetIdPair = tuple[int, int]


@dataclass(frozen=True)
class PhotonicVerificationIssue:
    code: str
    message: str
    severity: str = "error"
    net_name: str | None = None
    details: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class PhotonicVerificationResult:
    issues: tuple[PhotonicVerificationIssue, ...]
    metrics: dict[str, object]

    @property
    def success(self) -> bool:
        return self.error_count == 0

    @property
    def error_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "error")

    @property
    def warning_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "warning")

    def raise_for_errors(self) -> None:
        if self.success:
            return
        formatted = "\n".join(
            f"- {issue.code}: {issue.message}" for issue in self.issues[:10]
        )
        suffix = "" if len(self.issues) <= 10 else f"\n... {len(self.issues) - 10} more"
        raise AssertionError(f"Photonic verification failed:\n{formatted}{suffix}")

    def as_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "error_count": self.error_count,
            "warning_count": self.warning_count,
            "metrics": dict(self.metrics),
            "issues": [
                {
                    "code": issue.code,
                    "message": issue.message,
                    "severity": issue.severity,
                    "net_name": issue.net_name,
                    "details": dict(issue.details),
                }
                for issue in self.issues
            ],
        }


def verify_photonic_routing(
    routed_layout: Component,
    schematic: Any,
    *,
    routed_net_records: Iterable[RoutedNetRecord],
    unrouted_layout: Component | None = None,
    route_width_um: float = 0.5,
    route_layer: Layer = (1, 0),
    obstacle_layers: Iterable[Layer] = ((1, 0),),
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool = True,
    bend_radius_cells: int = 4,
    port_contact_radius_um: float | None = None,
    port_obstacle_exemption_radius_um: float = 25.0,
    legal_overlap_polygons_um: Iterable[Iterable[tuple[float, float]]] = (),
    legal_overlap_polygons_by_net_id_pair_um: Mapping[
        NetIdPair,
        Iterable[Iterable[tuple[float, float]]],
    ]
    | None = None,
    crossing_component_footprints_um: Iterable[Mapping[str, object] | Iterable[tuple[float, float]]] = (),
    check_route_coverage: bool = True,
    min_route_overlap_area_um2: float = 2.0,
    min_obstacle_overlap_area_um2: float = 2.0,
    min_crossing_component_route_overlap_area_um2: float = 0.25,
    min_crossing_component_overlap_area_um2: float = 0.25,
    check_endpoint_connectivity: bool = True,
) -> PhotonicVerificationResult:
    """Verify routed optical topology and realized waveguide geometry.

    The topology check compares the routed records against
    ``schematic.netlist.routes``. Geometry checks are performed on the realized
    polygons for each route, so post-processing such as endpoint correction and
    path-length meanders is covered.
    """
    route_width_um = float(route_width_um)
    if route_width_um <= 0.0:
        raise ValueError("route_width_um must be > 0")
    if port_contact_radius_um is None:
        port_contact_radius_um = max(route_width_um, float(realization_grid_spec[2]))

    records = tuple(routed_net_records)
    issues: list[PhotonicVerificationIssue] = []
    expected_keys = _expected_route_keys(schematic)
    actual_keys = [_record_key(record) for record in records]

    if check_route_coverage:
        _verify_record_coverage(issues, expected_keys, actual_keys)

    dbu = _component_dbu(routed_layout)
    all_port_contact_windows = kdb.Region()
    route_obstacle_windows_by_key: dict[RouteKey, kdb.Region] = {}
    non_route_obstacle_windows_by_key: dict[RouteKey, kdb.Region] = {}
    legal_route_overlap_region = _polygons_region_um(legal_overlap_polygons_um, dbu)
    legal_route_overlap_regions_by_pair = _polygon_regions_by_pair_um(
        legal_overlap_polygons_by_net_id_pair_um or {},
        dbu,
    )
    route_regions_by_key: dict[RouteKey, kdb.Region] = {}
    record_by_key: dict[RouteKey, RoutedNetRecord] = {}
    net_id_by_key: dict[RouteKey, int] = {}
    crossing_component_footprints = tuple(crossing_component_footprints_um)
    crossing_clip_intersections: list[dict[str, object]] = []
    for pair, polygons in (legal_overlap_polygons_by_net_id_pair_um or {}).items():
        try:
            net_id_a = int(pair[0])
            net_id_b = int(pair[1])
        except (TypeError, ValueError, IndexError):
            continue
        for polygon in polygons:
            crossing_clip_intersections.append(
                {
                    "classification": "legal_verification_clip",
                    "net_id_a": net_id_a,
                    "net_id_b": net_id_b,
                    "crossing_footprint_polygon_um": polygon,
                }
            )
    crossing_clip_plan_info = (
        {"enabled": True, "realized_intersections": crossing_clip_intersections}
        if crossing_clip_intersections
        else None
    )

    for record in records:
        key = _record_key(record)
        if key in record_by_key:
            continue
        record_by_key[key] = record
        if record.net_id is not None:
            net_id_by_key[key] = int(record.net_id)
        route_region = _realized_record_region(
            record,
            route_layer=route_layer,
            route_width_um=route_width_um,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
            crossing_plan_info=crossing_clip_plan_info,
            enable_endpoint_correction=check_endpoint_connectivity,
        )
        route_regions_by_key[key] = route_region
        _verify_record_connectivity(
            issues,
            record,
            route_region,
            dbu=dbu,
            port_contact_radius_um=float(port_contact_radius_um),
            check_endpoint_connectivity=check_endpoint_connectivity,
        )
        all_port_contact_windows += _record_port_windows_region(
            record,
            dbu=dbu,
            radius_um=float(port_contact_radius_um),
        )
        obstacle_port_contact_radius_um = min(
            float(port_contact_radius_um),
            float(port_obstacle_exemption_radius_um),
        )
        route_obstacle_windows_by_key[key] = _record_port_windows_region(
            record,
            dbu=dbu,
            radius_um=max(0.0, obstacle_port_contact_radius_um),
        )
        non_route_obstacle_windows_by_key[key] = _record_port_windows_region(
            record,
            dbu=dbu,
            radius_um=max(
                float(port_contact_radius_um),
                float(port_obstacle_exemption_radius_um),
            ),
        )

    cross_net_overlap_count = _verify_cross_net_route_overlaps(
        issues,
        route_regions_by_key,
        dbu=dbu,
        legal_overlap_region=_combined_region(
            all_port_contact_windows,
            legal_route_overlap_region,
        ),
        net_id_by_key=net_id_by_key,
        legal_overlap_regions_by_net_id_pair=legal_route_overlap_regions_by_pair,
        min_overlap_area_um2=float(min_route_overlap_area_um2),
    )
    normalized_obstacle_layers = tuple(
        _normalize_layer(layer) for layer in obstacle_layers
    )
    normalized_route_layer = _normalize_layer(route_layer)
    obstacle_overlap_count = _verify_route_obstacle_overlaps(
        issues,
        route_regions_by_key,
        obstacle_component=unrouted_layout,
        routed_layout=routed_layout,
        route_layer=route_layer,
        obstacle_layers=normalized_obstacle_layers,
        dbu=dbu,
        legal_overlap_region=legal_route_overlap_region,
        legal_overlap_regions_by_key=route_obstacle_windows_by_key,
        legal_overlap_regions_by_layer={
            layer: non_route_obstacle_windows_by_key
            for layer in normalized_obstacle_layers
            if layer != normalized_route_layer
        },
        min_overlap_area_um2=float(min_obstacle_overlap_area_um2),
    )
    crossing_component_route_overlap_count = _verify_crossing_component_route_overlaps(
        issues,
        route_regions_by_key,
        crossing_component_footprints,
        dbu=dbu,
        min_overlap_area_um2=float(min_crossing_component_route_overlap_area_um2),
    )
    crossing_component_overlap_count = _verify_crossing_component_overlaps(
        issues,
        crossing_component_footprints,
        dbu=dbu,
        min_overlap_area_um2=float(min_crossing_component_overlap_area_um2),
    )

    return PhotonicVerificationResult(
        issues=tuple(issues),
        metrics={
            "expected_route_count": len(expected_keys),
            "routed_record_count": len(records),
            "unique_routed_record_count": len(set(actual_keys)),
            "cross_net_waveguide_overlap_count": cross_net_overlap_count,
            "waveguide_obstacle_overlap_count": obstacle_overlap_count,
            "crossing_component_route_overlap_count": (
                crossing_component_route_overlap_count
            ),
            "crossing_component_overlap_count": crossing_component_overlap_count,
        },
    )


def _expected_route_keys(schematic: Any) -> set[RouteKey]:
    routes = getattr(getattr(schematic, "netlist", None), "routes", {})
    expected: set[RouteKey] = set()
    for net_name, bundle in routes.items():
        links = getattr(bundle, "links", {})
        for port1_spec, port2_spec in links.items():
            expected.add(
                (
                    str(net_name),
                    _parse_port_spec(str(port1_spec)),
                    _parse_port_spec(str(port2_spec)),
                )
            )
    return expected


def _parse_port_spec(port_spec: str) -> PortEndpoint:
    instance, port = port_spec.split(",", 1)
    return instance, port


def _record_key(record: RoutedNetRecord) -> RouteKey:
    return (
        record.net_name,
        (record.source.instance, record.source.port),
        (record.target.instance, record.target.port),
    )


def _verify_record_coverage(
    issues: list[PhotonicVerificationIssue],
    expected_keys: set[RouteKey],
    actual_keys: list[RouteKey],
) -> None:
    actual_key_set = set(actual_keys)
    duplicate_keys = sorted(
        key for key in actual_key_set if actual_keys.count(key) > 1
    )
    for key in sorted(expected_keys - actual_key_set):
        issues.append(
            PhotonicVerificationIssue(
                code="missing_route_record",
                message=f"No routed record was produced for {key[0]} {key[1]} -> {key[2]}.",
                net_name=key[0],
                details={"source": key[1], "target": key[2]},
            )
        )
    for key in sorted(actual_key_set - expected_keys):
        issues.append(
            PhotonicVerificationIssue(
                code="extra_route_record",
                message=f"Unexpected routed record for {key[0]} {key[1]} -> {key[2]}.",
                net_name=key[0],
                details={"source": key[1], "target": key[2]},
            )
        )
    for key in duplicate_keys:
        issues.append(
            PhotonicVerificationIssue(
                code="duplicate_route_record",
                message=f"Duplicate routed records were produced for {key[0]}.",
                net_name=key[0],
                details={"source": key[1], "target": key[2]},
            )
        )


def _realized_record_region(
    record: RoutedNetRecord,
    *,
    route_layer: Layer,
    route_width_um: float,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    crossing_plan_info: Mapping[str, object] | None = None,
    enable_endpoint_correction: bool = True,
) -> kdb.Region:
    temp = Component()
    realize_routed_net_records(
        temp,
        [record],
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
        crossing_plan_info=crossing_plan_info,
        enable_endpoint_correction=enable_endpoint_correction,
    )
    return _component_layer_region(temp, route_layer)


def _verify_record_connectivity(
    issues: list[PhotonicVerificationIssue],
    record: RoutedNetRecord,
    route_region: kdb.Region,
    *,
    dbu: float,
    port_contact_radius_um: float,
    check_endpoint_connectivity: bool = True,
) -> None:
    if route_region.is_empty():
        issues.append(
            PhotonicVerificationIssue(
                code="empty_route_geometry",
                message=f"Route geometry for {record.net_name} is empty.",
                net_name=record.net_name,
            )
        )
        return
    if not check_endpoint_connectivity:
        return
    if record.endpoint_correction_error is not None:
        issues.append(
            PhotonicVerificationIssue(
                code="endpoint_correction_error",
                message=record.endpoint_correction_error,
                net_name=record.net_name,
            )
        )
    if not record.corrected_centerline_um:
        issues.append(
            PhotonicVerificationIssue(
                code="missing_corrected_centerline",
                message=f"Route {record.net_name} has no corrected centerline.",
                net_name=record.net_name,
            )
        )

    _verify_one_port_connection(
        issues,
        record,
        route_region,
        dbu=dbu,
        center_um=record.source_port_center_um,
        endpoint=record.source,
        role="source",
        port_contact_radius_um=port_contact_radius_um,
    )
    _verify_one_port_connection(
        issues,
        record,
        route_region,
        dbu=dbu,
        center_um=record.target_port_center_um,
        endpoint=record.target,
        role="target",
        port_contact_radius_um=port_contact_radius_um,
    )


def _verify_one_port_connection(
    issues: list[PhotonicVerificationIssue],
    record: RoutedNetRecord,
    route_region: kdb.Region,
    *,
    dbu: float,
    center_um: tuple[float, float] | None,
    endpoint: Any,
    role: str,
    port_contact_radius_um: float,
) -> None:
    if center_um is None:
        issues.append(
            PhotonicVerificationIssue(
                code=f"{role}_port_center_missing",
                message=f"{role.title()} port center is missing for {record.net_name}.",
                net_name=record.net_name,
                details={"instance": endpoint.instance, "port": endpoint.port},
            )
        )
        return
    port_box = _box_region_around(center_um, port_contact_radius_um, dbu)
    if (route_region & port_box).is_empty():
        issues.append(
            PhotonicVerificationIssue(
                code=f"{role}_port_not_connected",
                message=(
                    f"Route {record.net_name} does not touch {role} port "
                    f"{endpoint.instance},{endpoint.port}."
                ),
                net_name=record.net_name,
                details={
                    "instance": endpoint.instance,
                    "port": endpoint.port,
                    "center_um": center_um,
                    "contact_radius_um": port_contact_radius_um,
                },
            )
        )
    if record.corrected_centerline_um:
        expected_point = (
            record.corrected_centerline_um[0]
            if role == "source"
            else record.corrected_centerline_um[-1]
        )
        distance = math.hypot(
            float(expected_point[0]) - float(center_um[0]),
            float(expected_point[1]) - float(center_um[1]),
        )
        if distance > max(port_contact_radius_um, 1.0e-6):
            issues.append(
                PhotonicVerificationIssue(
                    code=f"{role}_endpoint_mismatch",
                    message=(
                        f"Corrected centerline endpoint for {record.net_name} is "
                        f"{distance:.3f}um away from {role} port."
                    ),
                    net_name=record.net_name,
                    details={
                        "instance": endpoint.instance,
                        "port": endpoint.port,
                        "distance_um": distance,
                        "tolerance_um": port_contact_radius_um,
                    },
                )
            )


def _verify_cross_net_route_overlaps(
    issues: list[PhotonicVerificationIssue],
    route_regions_by_key: dict[RouteKey, kdb.Region],
    *,
    dbu: float,
    legal_overlap_region: kdb.Region,
    net_id_by_key: Mapping[RouteKey, int] | None = None,
    legal_overlap_regions_by_net_id_pair: Mapping[NetIdPair, kdb.Region] | None = None,
    min_overlap_area_um2: float = 0.0,
) -> int:
    overlap_count = 0
    items = list(route_regions_by_key.items())
    for index, (left_key, left_region) in enumerate(items):
        if left_region.is_empty():
            continue
        for right_key, right_region in items[index + 1:]:
            if right_region.is_empty():
                continue
            allowed_region = legal_overlap_region
            if (
                net_id_by_key is not None
                and legal_overlap_regions_by_net_id_pair is not None
                and left_key in net_id_by_key
                and right_key in net_id_by_key
            ):
                allowed_region = _combined_region(
                    legal_overlap_region,
                    legal_overlap_regions_by_net_id_pair.get(
                        _net_id_pair(net_id_by_key[left_key], net_id_by_key[right_key]),
                        kdb.Region(),
                    ),
                )
            overlap = (left_region & right_region) - allowed_region
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="cross_net_waveguide_overlap",
                    message=(
                        f"Waveguide for {left_key[0]} overlaps waveguide for "
                        f"{right_key[0]}."
                    ),
                    net_name=left_key[0],
                    details={
                        "other_net_name": right_key[0],
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


def _verify_route_obstacle_overlaps(
    issues: list[PhotonicVerificationIssue],
    route_regions_by_key: dict[RouteKey, kdb.Region],
    *,
    obstacle_component: Component | None,
    routed_layout: Component,
    route_layer: Layer,
    obstacle_layers: tuple[Layer, ...],
    dbu: float,
    legal_overlap_region: kdb.Region,
    legal_overlap_regions_by_key: Mapping[RouteKey, kdb.Region] | None = None,
    legal_overlap_regions_by_layer: Mapping[
        Layer,
        Mapping[RouteKey, kdb.Region],
    ]
    | None = None,
    min_overlap_area_um2: float = 0.0,
) -> int:
    overlap_count = 0
    for layer in obstacle_layers:
        source_component = obstacle_component
        if source_component is None:
            if layer == route_layer:
                continue
            source_component = routed_layout
        obstacle_region = _component_layer_region(source_component, layer)
        if obstacle_region.is_empty():
            continue
        for key, route_region in route_regions_by_key.items():
            allowed_region = legal_overlap_region
            per_key_regions = legal_overlap_regions_by_key
            if legal_overlap_regions_by_layer is not None:
                per_key_regions = legal_overlap_regions_by_layer.get(
                    layer,
                    per_key_regions,
                )
            if per_key_regions is not None:
                allowed_region = _combined_region(
                    legal_overlap_region,
                    per_key_regions.get(key, kdb.Region()),
                )
            overlap = (route_region & obstacle_region) - allowed_region
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="waveguide_obstacle_overlap",
                    message=f"Waveguide for {key[0]} overlaps obstacle layer {layer}.",
                    net_name=key[0],
                    details={
                        "obstacle_layer": layer,
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


@dataclass(frozen=True)
class _CrossingFootprint:
    region: kdb.Region
    details: dict[str, object]


def _verify_crossing_component_route_overlaps(
    issues: list[PhotonicVerificationIssue],
    route_regions_by_key: Mapping[RouteKey, kdb.Region],
    crossing_component_footprints_um: Iterable[
        Mapping[str, object] | Iterable[tuple[float, float]]
    ],
    *,
    dbu: float,
    min_overlap_area_um2: float = 0.0,
) -> int:
    overlap_count = 0
    footprints = _crossing_footprints_from_metadata(
        crossing_component_footprints_um,
        dbu=dbu,
    )
    for footprint_index, footprint in enumerate(footprints):
        owner_net_names = _crossing_footprint_owner_net_names(footprint.details)
        for key, route_region in route_regions_by_key.items():
            if route_region.is_empty():
                continue
            if key[0] in owner_net_names:
                continue
            overlap = route_region & footprint.region
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="crossing_component_route_overlap",
                    message=(
                        f"Waveguide for {key[0]} overlaps a realized crossing "
                        "component footprint."
                    ),
                    net_name=key[0],
                    details={
                        "crossing_index": footprint_index,
                        "crossing": dict(footprint.details),
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


def _crossing_footprint_owner_net_names(
    details: Mapping[str, object],
) -> set[str]:
    names: set[str] = set()
    for key in ("net_name_a", "net_name_b"):
        value = details.get(key)
        if isinstance(value, str) and value:
            names.add(value)
    raw_shared_names = details.get("shared_owner_net_names")
    if isinstance(raw_shared_names, Iterable) and not isinstance(
        raw_shared_names,
        (str, bytes, bytearray),
    ):
        for value in raw_shared_names:
            if isinstance(value, str) and value:
                names.add(value)
    return names


def _verify_crossing_component_overlaps(
    issues: list[PhotonicVerificationIssue],
    crossing_component_footprints_um: Iterable[
        Mapping[str, object] | Iterable[tuple[float, float]]
    ],
    *,
    dbu: float,
    min_overlap_area_um2: float = 0.0,
) -> int:
    overlap_count = 0
    footprints = _crossing_footprints_from_metadata(
        crossing_component_footprints_um,
        dbu=dbu,
    )
    for left_index, left in enumerate(footprints):
        if left.region.is_empty():
            continue
        for right_index, right in enumerate(footprints[left_index + 1 :], start=left_index + 1):
            if right.region.is_empty():
                continue
            overlap = left.region & right.region
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="crossing_component_overlap",
                    message="Realized crossing component footprints overlap.",
                    net_name=_footprint_net_name(left.details),
                    details={
                        "left_crossing_index": left_index,
                        "right_crossing_index": right_index,
                        "left_crossing": dict(left.details),
                        "right_crossing": dict(right.details),
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


def _crossing_footprints_from_metadata(
    crossing_component_footprints_um: Iterable[
        Mapping[str, object] | Iterable[tuple[float, float]]
    ],
    *,
    dbu: float,
) -> tuple[_CrossingFootprint, ...]:
    footprints: list[_CrossingFootprint] = []
    for raw in crossing_component_footprints_um:
        details = dict(raw) if isinstance(raw, Mapping) else {}
        region = _crossing_footprint_region(raw, dbu=dbu)
        if region.is_empty():
            continue
        footprints.append(_CrossingFootprint(region=region, details=details))
    return tuple(footprints)


def _crossing_footprint_region(
    raw: Mapping[str, object] | Iterable[tuple[float, float]],
    *,
    dbu: float,
) -> kdb.Region:
    if isinstance(raw, Mapping):
        polygon = raw.get(
            "crossing_footprint_polygon_um",
            raw.get("footprint_polygon_um", raw.get("polygon_um")),
        )
        if isinstance(polygon, Iterable) and not isinstance(
            polygon,
            (str, bytes, bytearray),
        ):
            region = _polygons_region_um((polygon,), dbu)
            if not region.is_empty():
                return region
        center = _as_point(raw.get("point_um", raw.get("center_um")))
        bbox = _as_point(raw.get("component_bbox_um"))
        if center is None or bbox is None:
            return kdb.Region()
        half_width = max(0.0, float(bbox[0]) / 2.0)
        half_height = max(0.0, float(bbox[1]) / 2.0)
        return kdb.Region(
            kdb.Box(
                _um_to_dbu(float(center[0]) - half_width, dbu),
                _um_to_dbu(float(center[1]) - half_height, dbu),
                _um_to_dbu(float(center[0]) + half_width, dbu),
                _um_to_dbu(float(center[1]) + half_height, dbu),
            )
        )
    return _polygons_region_um((raw,), dbu)


def _footprint_net_name(details: Mapping[str, object]) -> str | None:
    for key in ("net_name_a", "net_name_b", "net_name"):
        value = details.get(key)
        if value is not None:
            return str(value)
    return None


def _record_port_windows_region(
    record: RoutedNetRecord,
    *,
    dbu: float,
    radius_um: float,
) -> kdb.Region:
    region = kdb.Region()
    for center in (record.source_port_center_um, record.target_port_center_um):
        if center is None:
            continue
        region += _box_region_around(center, radius_um, dbu)
    return region


def _component_layer_region(component: Component, layer: Layer) -> kdb.Region:
    region = kdb.Region()
    for polygon in component.get_polygons(merge=False, by="tuple").get(layer, []):
        region.insert(polygon)
    return region


def _combined_region(*regions: kdb.Region) -> kdb.Region:
    combined = kdb.Region()
    for region in regions:
        combined += region
    return combined


def _polygons_region_um(
    polygons_um: Iterable[Iterable[tuple[float, float]]],
    dbu: float,
) -> kdb.Region:
    region = kdb.Region()
    for polygon in polygons_um:
        points: list[kdb.Point] = []
        for raw_point in polygon:
            if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
                points = []
                break
            points.append(
                kdb.Point(
                    _um_to_dbu(float(raw_point[0]), dbu),
                    _um_to_dbu(float(raw_point[1]), dbu),
                )
            )
        if len(points) < 3:
            continue
        region.insert(kdb.Polygon(points))
    return region


def _polygon_regions_by_pair_um(
    polygons_by_pair: Mapping[
        NetIdPair,
        Iterable[Iterable[tuple[float, float]]],
    ],
    dbu: float,
) -> dict[NetIdPair, kdb.Region]:
    return {
        _net_id_pair(pair[0], pair[1]): _polygons_region_um(polygons, dbu)
        for pair, polygons in polygons_by_pair.items()
    }


def _net_id_pair(net_id_a: int, net_id_b: int) -> NetIdPair:
    a = int(net_id_a)
    b = int(net_id_b)
    return (a, b) if a <= b else (b, a)


def _box_region_around(
    center_um: tuple[float, float],
    radius_um: float,
    dbu: float,
) -> kdb.Region:
    x, y = float(center_um[0]), float(center_um[1])
    radius = float(radius_um)
    return kdb.Region(
        kdb.Box(
            _um_to_dbu(x - radius, dbu),
            _um_to_dbu(y - radius, dbu),
            _um_to_dbu(x + radius, dbu),
            _um_to_dbu(y + radius, dbu),
        )
    )


def _component_dbu(component: Component) -> float:
    kcl = getattr(component, "kcl", None)
    dbu = getattr(kcl, "dbu", None)
    if dbu is None:
        return 0.001
    return float(dbu)


def _um_to_dbu(value_um: float, dbu: float) -> int:
    return int(round(float(value_um) / float(dbu)))


def _dbu_to_um(value_dbu: int | float, dbu: float) -> float:
    return float(value_dbu) * float(dbu)


def _region_area_um2(region: kdb.Region, dbu: float) -> float:
    return float(region.area()) * float(dbu) * float(dbu)


def _region_bbox_um(region: kdb.Region, dbu: float) -> tuple[float, float, float, float] | None:
    if region.is_empty():
        return None
    bbox = region.bbox()
    return (
        _dbu_to_um(bbox.left, dbu),
        _dbu_to_um(bbox.bottom, dbu),
        _dbu_to_um(bbox.right, dbu),
        _dbu_to_um(bbox.top, dbu),
    )


def _as_point(value: object) -> tuple[float, float] | None:
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


def _normalize_layer(layer: Iterable[int]) -> Layer:
    layer_number, datatype = tuple(layer)
    return int(layer_number), int(datatype)
