"""Realized PDK crossing component placement for the Rust-backed photonic router."""

from __future__ import annotations

import contextlib
import math
from collections.abc import Iterable as IterableABC
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, cast

import gdsfactory as gf
from gdsfactory.component import Component

from photonic_router.records.route_rust_geometry import rounded_point
from photonic_router.records.route_rust_types import int_or_none

if TYPE_CHECKING:
    from kfactory.typings import MetaData

    from photonic_router.records.crossing_plan_info import CrossingPlanInfo


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
    bounds = bbox_bounds_um(ref)
    if bounds is None:
        return None
    left, bottom, right, top = bounds
    return (left + right) / 2.0, (bottom + top) / 2.0


def bbox_bounds_um(ref: Any) -> tuple[float, float, float, float] | None:
    """Return the bounding box of a reference as (left, bottom, right, top) in um, or None.

    Reads ``dbbox`` and falls back to ``bbox``; each may be an attribute or a method,
    and the box may be an object with ``left``/``right``/``bottom``/``top`` or a
    4-tuple in the returned order. None if there is no box, a value is not finite or
    the box is inverted.
    """
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


def crossing_component_bbox_size_um() -> tuple[str, float, float] | None:
    """Return (name, width, height) of the active PDK's crossing component, sizes in um.

    None if the crossing component cannot be built or has no usable bounding box.
    """
    component = _active_crossing_component()
    if component is None:
        return None
    size = _bbox_size_um(component)
    if size is None:
        return None
    return str(component.name), float(size[0]), float(size[1])


def point_um_from_mapping(value: object) -> tuple[float, float] | None:
    """Coerce a raw point of a record to a finite (x, y) float pair, or None.

    Accepts a list or tuple; only the first two entries are read. None for any other
    type, for fewer than two entries, and for non-numeric or non-finite values.
    """
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
    start = point_um_from_mapping(value[0])
    end = point_um_from_mapping(value[1])
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
            peer_index = int_or_none(raw_peer_index)
            if peer_index is not None:
                indices.add(peer_index)
    overlapping_index = int_or_none(raw_crossing.get("overlapping_crossing_index"))
    if overlapping_index is not None:
        indices.add(overlapping_index)
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


def crossing_footprint_polygon_metadata(
    raw_crossing: Mapping[str, object],
) -> list[list[float]] | None:
    """Read ``crossing_footprint_polygon_um`` of a raw crossing record as a list of [x, y] in um.

    None if the key is missing or not a sequence of points, if a point is not a pair of
    numbers, or if there are fewer than three points.
    """
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


def _place_crossing_ref(
    routed_layout: Component,
    component: Component,
    instance_name: str,
    point_um: tuple[float, float],
    rotation_deg: float,
) -> Any:
    """Add a reference of the crossing component, turned and with its optical centre at the point.

    The centre is the one the reference's port positions imply, or the centre
    of its bounding box without any; a reference with neither is moved by
    ``point_um`` as it is.
    """

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
    return ref


def _shared_crossing_owners(
    raw_crossing_list: list[Mapping[str, object]], cluster_indices: IterableABC[int]
) -> tuple[set[str], set[int]]:
    """The net names and net ids of the crossings of one cluster that share a component."""

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
            shared_owner_id = int_or_none(shared_crossing.get(key))
            if shared_owner_id is not None:
                shared_owner_ids.add(shared_owner_id)
    return shared_owner_names, shared_owner_ids


def _realized_crossing_records(crossing_plan_info: CrossingPlanInfo) -> list[Mapping[str, object]]:
    """The realized intersections of the plan that are mappings, in order; [] when the plan holds no sequence there."""

    raw_crossings: object = crossing_plan_info.realized_intersections or ()
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return []
    return [raw_crossing for raw_crossing in raw_crossings if isinstance(raw_crossing, Mapping)]


def place_realized_crossing_components(
    routed_layout: Component,
    crossing_plan_info: CrossingPlanInfo,
) -> list[dict[str, object]]:
    """Place active PDK crossing refs for legal realized route crossings."""

    if not crossing_plan_info.enabled:
        crossing_plan_info.realized_crossing_components = []
        crossing_plan_info.realized_crossing_component_count = 0
        return []

    component = _active_crossing_component()
    if component is None:
        crossing_plan_info.realized_crossing_components = []
        crossing_plan_info.realized_crossing_component_count = 0
        crossing_plan_info.realized_crossing_component_error = "crossing_component_unavailable"
        return []

    component_size = _bbox_size_um(component)
    component_bbox_um = (
        [float(component_size[0]), float(component_size[1])] if component_size is not None else None
    )
    component_name = str(component.name)
    placements: list[dict[str, object]] = []
    raw_crossing_list = _realized_crossing_records(crossing_plan_info)
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
        point_um = point_um_from_mapping(raw_crossing.get("point_um"))
        if point_um is None:
            continue
        rotation_deg = _crossing_component_rotation_deg(raw_crossing)
        instance_name = (
            f"crossing_{len(placements):04d}_"
            f"{raw_crossing.get('net_id_a', 'na')}_"
            f"{raw_crossing.get('net_id_b', 'nb')}"
        )
        ref = _place_crossing_ref(routed_layout, component, instance_name, point_um, rotation_deg)

        placement: dict[str, object] = {
            "component_name": component_name,
            "instance_name": str(getattr(ref, "name", instance_name)),
            "point_um": rounded_point(point_um),
            "center_um": rounded_point(point_um),
            "optical_center_um": rounded_point(point_um),
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
            shared_owner_names, shared_owner_ids = _shared_crossing_owners(
                raw_crossing_list, cluster_indices
            )
            if shared_owner_names:
                placement["shared_owner_net_names"] = sorted(shared_owner_names)
            if shared_owner_ids:
                placement["shared_owner_net_ids"] = sorted(shared_owner_ids)
        footprint_polygon = crossing_footprint_polygon_metadata(raw_crossing)
        if footprint_polygon is not None:
            placement["crossing_footprint_polygon_um"] = footprint_polygon
        if component_bbox_um is not None:
            placement["component_bbox_um"] = list(component_bbox_um)
        placements.append(placement)

    crossing_plan_info.realized_crossing_components = placements
    crossing_plan_info.realized_crossing_component_count = len(placements)
    crossing_plan_info.realized_crossing_component_error = None
    with contextlib.suppress(AttributeError, TypeError, ValueError):
        # The placements hold JSON values only, read from untyped crossing
        # records; kfactory checks the value when it is assigned.
        routed_layout.info["realized_crossing_components"] = cast("MetaData", placements)
    return placements


def legal_crossing_overlap_polygons_for_verification(
    crossing_plan_info: CrossingPlanInfo | None,
) -> dict[tuple[int, int], tuple[tuple[tuple[float, float], ...], ...]]:
    """Collect the footprint polygons of the legal realized crossings per pair of nets.

    Reads ``realized_intersections`` of the crossing plan. The key is the pair of net
    ids, smaller id first; each polygon is a tuple of (x, y) in um. An intersection is
    skipped if its classification does not start with ``legal_``, if it has no integer
    net ids, or if its polygon is not at least three finite points. Returns an empty
    dict without plan info.
    """
    if crossing_plan_info is None:
        return {}
    raw_crossings: object = crossing_plan_info.realized_intersections or ()
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
        net_id_a = int_or_none(raw_crossing.get("net_id_a"))
        net_id_b = int_or_none(raw_crossing.get("net_id_b"))
        if net_id_a is None or net_id_b is None:
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


def legal_crossing_component_footprints_for_verification(
    crossing_plan_info: CrossingPlanInfo | None,
) -> tuple[dict[str, object], ...]:
    """Return the records of the realized crossing components as a tuple of dicts.

    Uses ``realized_crossing_components`` of the crossing plan when it holds at least
    one mapping. Otherwise falls back to the ``realized_intersections`` whose
    classification starts with ``legal_`` and whose ``crossing_footprint_polygon_um``
    has at least three points. Returns an empty tuple without plan info.
    """
    if crossing_plan_info is None:
        return ()
    raw_components: object = crossing_plan_info.realized_crossing_components or ()
    if isinstance(raw_components, IterableABC) and not isinstance(
        raw_components,
        (str, bytes, bytearray),
    ):
        components = [
            dict(component) for component in raw_components if isinstance(component, Mapping)
        ]
        if components:
            return tuple(components)

    raw_crossings = crossing_plan_info.realized_intersections or ()
    footprints: list[dict[str, object]] = []
    for raw_crossing in raw_crossings:
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
