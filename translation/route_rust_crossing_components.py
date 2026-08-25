"""Realized PDK crossing component placement for the Rust-backed photonic router."""

from __future__ import annotations

import math
from collections.abc import Iterable as IterableABC
from typing import Any, Mapping, cast

import gdsfactory as gf
from gdsfactory.component import Component

from translation.route_rust_geometry import _rounded_point


def _bbox_size_um(component: Component) -> tuple[float, float] | None:
    try:
        bbox = component.dbbox() if callable(component.dbbox) else component.dbbox
    except (AttributeError, TypeError):
        try:
            bbox = component.bbox() if callable(component.bbox) else component.bbox
        except (AttributeError, TypeError):
            return None
    try:
        width = float(bbox.right) - float(bbox.left)
        height = float(bbox.top) - float(bbox.bottom)
    except AttributeError:
        try:
            left, bottom, right, top = cast(Any, bbox)
            width = float(right) - float(left)
            height = float(top) - float(bottom)
        except (TypeError, ValueError):
            return None
    if not math.isfinite(width) or not math.isfinite(height) or width <= 0 or height <= 0:
        return None
    return width, height


def _bbox_center_um(ref: Any) -> tuple[float, float] | None:
    bounds = _bbox_bounds_um(ref)
    if bounds is None:
        return None
    left, bottom, right, top = bounds
    return (left + right) / 2.0, (bottom + top) / 2.0


def _bbox_bounds_um(ref: Any) -> tuple[float, float, float, float] | None:
    try:
        bbox = ref.dbbox() if callable(ref.dbbox) else ref.dbbox
    except (AttributeError, TypeError):
        try:
            bbox = ref.bbox() if callable(ref.bbox) else ref.bbox
        except (AttributeError, TypeError):
            return None
    try:
        left = float(bbox.left)
        right = float(bbox.right)
        bottom = float(bbox.bottom)
        top = float(bbox.top)
    except AttributeError:
        try:
            left, bottom, right, top = cast(Any, bbox)
            left = float(left)
            right = float(right)
            bottom = float(bottom)
            top = float(top)
        except (TypeError, ValueError):
            return None
    if not all(math.isfinite(value) for value in (left, right, bottom, top)):
        return None
    if right < left or top < bottom:
        return None
    return left, bottom, right, top


def _ports_optical_center_um(obj: Any) -> tuple[float, float] | None:
    """Return the optical center implied by port positions, when available."""

    try:
        ports = list(obj.ports)
    except (AttributeError, TypeError, ValueError):
        return None
    points: list[tuple[float, float]] = []
    for port in ports:
        try:
            x = float(port.center[0])
            y = float(port.center[1])
        except (AttributeError, IndexError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    if len(points) < 2:
        return None
    return (
        sum(point[0] for point in points) / float(len(points)),
        sum(point[1] for point in points) / float(len(points)),
    )


def _active_crossing_component() -> Component | None:
    try:
        return gf.components.crossing()
    except Exception:
        try:
            from gdsfactory.gpdk import get_generic_pdk

            get_generic_pdk().activate()
            return gf.components.crossing()
        except Exception:
            return None


def _crossing_component_bbox_size_um() -> tuple[str, float, float] | None:
    component = _active_crossing_component()
    if component is None:
        return None
    size = _bbox_size_um(component)
    if size is None:
        return None
    return str(component.name), float(size[0]), float(size[1])


def _point_um_from_mapping(value: object) -> tuple[float, float] | None:
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


def _segment_um_from_mapping(
    value: object,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    start = _point_um_from_mapping(value[0])
    end = _point_um_from_mapping(value[1])
    if start is None or end is None:
        return None
    if math.hypot(end[0] - start[0], end[1] - start[1]) <= 1.0e-9:
        return None
    return start, end


def _crossing_component_rotation_deg(crossing: Mapping[str, object]) -> float:
    segment = _segment_um_from_mapping(crossing.get("segment_a_um"))
    if segment is None:
        return 0.0
    (x0, y0), (x1, y1) = segment
    angle = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return float(angle % 360.0)


_SHARED_CROSSING_COMPONENT_POLICIES = {
    "allowed_lidar_pure_cluster",
    "allowed_lidar_pure_degraded_cluster",
}


def _shared_crossing_peer_indices(raw_crossing: Mapping[str, object]) -> set[int]:
    indices: set[int] = set()
    raw_peer_indices = raw_crossing.get("overlapping_crossing_indices")
    if isinstance(raw_peer_indices, IterableABC) and not isinstance(
        raw_peer_indices,
        (str, bytes, bytearray),
    ):
        for raw_peer_index in raw_peer_indices:
            try:
                indices.add(int(cast(object, raw_peer_index)))
            except (TypeError, ValueError):
                continue
    try:
        indices.add(int(cast(object, raw_crossing.get("overlapping_crossing_index"))))
    except (TypeError, ValueError):
        pass
    return indices


def _shared_crossing_component_clusters(
    raw_crossings: list[Mapping[str, object]],
) -> dict[int, set[int]]:
    shared_indices = {
        index
        for index, raw_crossing in enumerate(raw_crossings)
        if str(raw_crossing.get("footprint_overlap_policy", "") or "")
        in _SHARED_CROSSING_COMPONENT_POLICIES
    }
    if not shared_indices:
        return {}

    parent = {index: index for index in shared_indices}

    def find(index: int) -> int:
        root = parent[index]
        while root != parent[root]:
            root = parent[root]
        while index != root:
            next_index = parent[index]
            parent[index] = root
            index = next_index
        return root

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        root = min(left_root, right_root)
        other = max(left_root, right_root)
        parent[other] = root

    for index in shared_indices:
        for peer_index in _shared_crossing_peer_indices(raw_crossings[index]):
            if peer_index in shared_indices:
                union(index, peer_index)

    clusters_by_root: dict[int, set[int]] = {}
    for index in shared_indices:
        clusters_by_root.setdefault(find(index), set()).add(index)
    return {index: set(cluster) for cluster in clusters_by_root.values() for index in cluster}


def _crossing_footprint_polygon_metadata(
    raw_crossing: Mapping[str, object],
) -> list[list[float]] | None:
    raw_polygon = raw_crossing.get("crossing_footprint_polygon_um")
    if not isinstance(raw_polygon, IterableABC) or isinstance(
        raw_polygon,
        (str, bytes, bytearray),
    ):
        return None
    polygon: list[list[float]] = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
            return None
        try:
            polygon.append([float(raw_point[0]), float(raw_point[1])])
        except (TypeError, ValueError):
            return None
    return polygon if len(polygon) >= 3 else None


def _place_realized_crossing_components(
    routed_layout: Component,
    crossing_plan_info: dict[str, object],
) -> list[dict[str, object]]:
    """Place active PDK crossing refs for legal realized route crossings."""

    if not crossing_plan_info.get("enabled"):
        crossing_plan_info["realized_crossing_components"] = []
        crossing_plan_info["realized_crossing_component_count"] = 0
        return []

    component = _active_crossing_component()
    if component is None:
        crossing_plan_info["realized_crossing_components"] = []
        crossing_plan_info["realized_crossing_component_count"] = 0
        crossing_plan_info["realized_crossing_component_error"] = "crossing_component_unavailable"
        return []

    component_size = _bbox_size_um(component)
    component_bbox_um = (
        [float(component_size[0]), float(component_size[1])] if component_size is not None else None
    )
    component_name = str(component.name)
    placements: list[dict[str, object]] = []
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        raw_crossings = ()
    raw_crossing_list = [
        raw_crossing for raw_crossing in raw_crossings if isinstance(raw_crossing, Mapping)
    ]
    shared_clusters_by_index = _shared_crossing_component_clusters(raw_crossing_list)

    for index, raw_crossing in enumerate(raw_crossing_list):
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        cluster_indices = shared_clusters_by_index.get(index)
        if cluster_indices:
            representative_index = min(cluster_indices)
            if index != representative_index:
                continue
        point_um = _point_um_from_mapping(raw_crossing.get("point_um"))
        if point_um is None:
            continue
        rotation_deg = _crossing_component_rotation_deg(raw_crossing)
        instance_name = (
            f"crossing_{len(placements):04d}_"
            f"{raw_crossing.get('net_id_a', 'na')}_"
            f"{raw_crossing.get('net_id_b', 'nb')}"
        )
        try:
            ref = routed_layout.add_ref(component, name=instance_name)
        except TypeError:
            ref = routed_layout.add_ref(component)
        if abs(rotation_deg) > 1.0e-9:
            ref.drotate(rotation_deg)
        ref_center_um = _ports_optical_center_um(ref) or _bbox_center_um(ref)
        if ref_center_um is None:
            ref.dmove(point_um)
        else:
            ref.dmove(
                (
                    point_um[0] - ref_center_um[0],
                    point_um[1] - ref_center_um[1],
                )
            )

        placement: dict[str, object] = {
            "component_name": component_name,
            "instance_name": str(getattr(ref, "name", instance_name)),
            "point_um": _rounded_point(point_um),
            "center_um": _rounded_point(point_um),
            "optical_center_um": _rounded_point(point_um),
            "rotation_deg": round(float(rotation_deg), 6),
            "source_crossing_index": int(index),
            "classification": classification,
            "net_id_a": raw_crossing.get("net_id_a"),
            "net_id_b": raw_crossing.get("net_id_b"),
            "net_name_a": raw_crossing.get("net_name_a"),
            "net_name_b": raw_crossing.get("net_name_b"),
        }
        if cluster_indices:
            placement["shared_crossing_indices"] = sorted(int(i) for i in cluster_indices)
            shared_owner_names: set[str] = set()
            shared_owner_ids: set[int] = set()
            for shared_index in cluster_indices:
                if shared_index < 0 or shared_index >= len(raw_crossing_list):
                    continue
                shared_crossing = raw_crossing_list[shared_index]
                for key in ("net_name_a", "net_name_b"):
                    value = shared_crossing.get(key)
                    if isinstance(value, str) and value:
                        shared_owner_names.add(value)
                for key in ("net_id_a", "net_id_b"):
                    try:
                        shared_owner_ids.add(int(cast(object, shared_crossing.get(key))))
                    except (TypeError, ValueError):
                        continue
            if shared_owner_names:
                placement["shared_owner_net_names"] = sorted(shared_owner_names)
            if shared_owner_ids:
                placement["shared_owner_net_ids"] = sorted(shared_owner_ids)
        footprint_polygon = _crossing_footprint_polygon_metadata(raw_crossing)
        if footprint_polygon is not None:
            placement["crossing_footprint_polygon_um"] = footprint_polygon
        if component_bbox_um is not None:
            placement["component_bbox_um"] = list(component_bbox_um)
        placements.append(placement)

    crossing_plan_info["realized_crossing_components"] = placements
    crossing_plan_info["realized_crossing_component_count"] = len(placements)
    crossing_plan_info.pop("realized_crossing_component_error", None)
    try:
        routed_layout.info["realized_crossing_components"] = placements
    except (AttributeError, TypeError, ValueError):
        pass
    return placements


def _legal_crossing_overlap_polygons_for_verification(
    crossing_plan_info: Mapping[str, object] | None,
) -> dict[tuple[int, int], tuple[tuple[tuple[float, float], ...], ...]]:
    if crossing_plan_info is None:
        return {}
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return {}

    polygons_by_pair: dict[tuple[int, int], list[tuple[tuple[float, float], ...]]] = {}
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        try:
            net_id_a = int(cast(object, raw_crossing["net_id_a"]))
            net_id_b = int(cast(object, raw_crossing["net_id_b"]))
        except (KeyError, TypeError, ValueError):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if not isinstance(raw_polygon, IterableABC) or isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            continue
        points: list[tuple[float, float]] = []
        for raw_point in raw_polygon:
            if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
                points = []
                break
            try:
                point = (float(raw_point[0]), float(raw_point[1]))
            except (TypeError, ValueError):
                points = []
                break
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                points = []
                break
            points.append(point)
        if len(points) >= 3:
            pair = (net_id_a, net_id_b) if net_id_a <= net_id_b else (net_id_b, net_id_a)
            polygons_by_pair.setdefault(pair, []).append(tuple(points))
    return {pair: tuple(polygons) for pair, polygons in polygons_by_pair.items()}


def _legal_crossing_component_footprints_for_verification(
    crossing_plan_info: Mapping[str, object] | None,
) -> tuple[dict[str, object], ...]:
    if crossing_plan_info is None:
        return ()
    raw_components = crossing_plan_info.get("realized_crossing_components", ())
    if isinstance(raw_components, IterableABC) and not isinstance(
        raw_components,
        (str, bytes, bytearray),
    ):
        components = [
            dict(component) for component in raw_components if isinstance(component, Mapping)
        ]
        if components:
            return tuple(components)

    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return ()
    footprints: list[dict[str, object]] = []
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if isinstance(raw_polygon, IterableABC) and not isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            raw_points = list(raw_polygon)
            if len(raw_points) >= 3:
                footprint = dict(raw_crossing)
                footprint["crossing_footprint_polygon_um"] = raw_points
                footprints.append(footprint)
    return tuple(footprints)
