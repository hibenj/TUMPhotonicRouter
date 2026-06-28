"""Post-route verification for photonic waveguide routing."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable

from gdsfactory.component import Component
import klayout.db as kdb

from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_types import RoutedNetRecord

Layer = tuple[int, int]
PortEndpoint = tuple[str, str]
RouteKey = tuple[str, PortEndpoint, PortEndpoint]


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

    _verify_record_coverage(issues, expected_keys, actual_keys)

    dbu = _component_dbu(routed_layout)
    all_port_windows = kdb.Region()
    route_regions_by_key: dict[RouteKey, kdb.Region] = {}
    record_by_key: dict[RouteKey, RoutedNetRecord] = {}

    for record in records:
        key = _record_key(record)
        if key in record_by_key:
            continue
        record_by_key[key] = record
        route_region = _realized_record_region(
            record,
            route_layer=route_layer,
            route_width_um=route_width_um,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
        )
        route_regions_by_key[key] = route_region
        _verify_record_connectivity(
            issues,
            record,
            route_region,
            dbu=dbu,
            port_contact_radius_um=float(port_contact_radius_um),
        )
        all_port_windows += _record_port_windows_region(
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
        legal_overlap_region=all_port_windows,
    )
    obstacle_overlap_count = _verify_route_obstacle_overlaps(
        issues,
        route_regions_by_key,
        obstacle_component=unrouted_layout,
        routed_layout=routed_layout,
        route_layer=route_layer,
        obstacle_layers=tuple(_normalize_layer(layer) for layer in obstacle_layers),
        dbu=dbu,
        legal_overlap_region=all_port_windows,
    )

    return PhotonicVerificationResult(
        issues=tuple(issues),
        metrics={
            "expected_route_count": len(expected_keys),
            "routed_record_count": len(records),
            "unique_routed_record_count": len(set(actual_keys)),
            "cross_net_waveguide_overlap_count": cross_net_overlap_count,
            "waveguide_obstacle_overlap_count": obstacle_overlap_count,
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
    )
    return _component_layer_region(temp, route_layer)


def _verify_record_connectivity(
    issues: list[PhotonicVerificationIssue],
    record: RoutedNetRecord,
    route_region: kdb.Region,
    *,
    dbu: float,
    port_contact_radius_um: float,
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
) -> int:
    overlap_count = 0
    items = list(route_regions_by_key.items())
    for index, (left_key, left_region) in enumerate(items):
        if left_region.is_empty():
            continue
        for right_key, right_region in items[index + 1:]:
            if right_region.is_empty():
                continue
            overlap = (left_region & right_region) - legal_overlap_region
            if overlap.is_empty():
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
                        "overlap_area_um2": _region_area_um2(overlap, dbu),
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
            overlap = (route_region & obstacle_region) - legal_overlap_region
            if overlap.is_empty():
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="waveguide_obstacle_overlap",
                    message=f"Waveguide for {key[0]} overlaps obstacle layer {layer}.",
                    net_name=key[0],
                    details={
                        "obstacle_layer": layer,
                        "overlap_area_um2": _region_area_um2(overlap, dbu),
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


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


def _normalize_layer(layer: Iterable[int]) -> Layer:
    layer_number, datatype = tuple(layer)
    return int(layer_number), int(datatype)
