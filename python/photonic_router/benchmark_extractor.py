"""Extract fixed geometry and ports from gdsfactory benchmark components."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

Point = tuple[float, float]
Polygon = list[Point]
BBox = tuple[float, float, float, float]


@dataclass(frozen=True)
class Port:
    """Router-facing port metadata in physical micrometer coordinates."""

    name: str
    position: Point
    orientation: float | None
    width: float | None = None
    layer: Any | None = None
    port_type: str | None = None


@dataclass(frozen=True)
class ExtractedBenchmark:
    """Geometry extracted from a gdsfactory component.

    Attributes:
        polygons: Flattened fixed layout polygons in micrometers.
        ports: Top-level and direct component-reference ports.
        bbox: Bounding box `(xmin, ymin, xmax, ymax)` in micrometers.
    """

    polygons: list[Polygon]
    ports: list[Port]
    bbox: BBox


def extract_benchmark(
    component: Any,
    layers: Iterable[Any] | None = None,
    *,
    as_bounding_boxes: bool = False,
) -> ExtractedBenchmark:
    """Extract flattened geometry, reference ports, and bbox from a component.

    Args:
        component: A `gdsfactory.Component`.
        layers: Optional layer filter passed to `component.get_polygons`.
        as_bounding_boxes: If true, represent each polygon by its physical
            bounding box. This is equivalent for bounding-box obstacle mode and
            avoids expensive hull-point conversion.

    Returns:
        Extracted benchmark data in physical micrometer units.
    """

    polygons = _extract_polygons(
        component,
        layers=layers,
        as_bounding_boxes=as_bounding_boxes,
    )
    ports = _extract_ports(component)
    bbox = _compute_bbox(polygons, ports)
    return ExtractedBenchmark(polygons=polygons, ports=ports, bbox=bbox)


def _extract_polygons(
    component: Any,
    layers: Iterable[Any] | None = None,
    *,
    as_bounding_boxes: bool = False,
) -> list[Polygon]:
    polygon_map = component.get_polygons(merge=False, by="tuple", layers=layers)
    dbu = float(getattr(component.kcl, "dbu", 1.0))

    polygons: list[Polygon] = []
    for layer_polygons in polygon_map.values():
        for polygon in layer_polygons:
            points = (
                _polygon_bbox_points_um(polygon, dbu)
                if as_bounding_boxes
                else _polygon_points_um(polygon, dbu)
            )
            if len(points) >= 3:
                polygons.append(points)
    return polygons


def _polygon_bbox_points_um(polygon: Any, dbu: float) -> Polygon:
    """Convert a KLayout/kfactory polygon bbox to a rectangle in micrometers."""

    if not hasattr(polygon, "bbox"):
        return _polygon_points_um(polygon, dbu)

    bbox = polygon.bbox()
    left = float(bbox.left) * dbu
    right = float(bbox.right) * dbu
    bottom = float(bbox.bottom) * dbu
    top = float(bbox.top) * dbu
    return [(left, bottom), (right, bottom), (right, top), (left, top)]


def _polygon_points_um(polygon: Any, dbu: float) -> Polygon:
    """Convert a KLayout/kfactory polygon hull to micrometer points."""

    if hasattr(polygon, "to_dtype"):
        dpolygon = polygon.to_dtype(dbu)
        return [(float(point.x), float(point.y)) for point in dpolygon.each_point_hull()]

    return [(float(point.x) * dbu, float(point.y) * dbu) for point in polygon.each_point_hull()]


def _extract_ports(component: Any) -> list[Port]:
    ports: list[Port] = []

    for port in component.ports:
        ports.append(_port_from_gdsfactory_port(port))

    for instance in getattr(component, "insts", []):
        instance_name = str(getattr(instance, "name", "") or "")
        for port in instance.ports:
            extracted = _port_from_gdsfactory_port(port)
            name = f"{instance_name},{extracted.name}" if instance_name else extracted.name
            ports.append(
                Port(
                    name=name,
                    position=extracted.position,
                    orientation=extracted.orientation,
                    width=extracted.width,
                    layer=extracted.layer,
                    port_type=extracted.port_type,
                )
            )

    return ports


def _port_from_gdsfactory_port(port: Any) -> Port:
    center = getattr(port, "dcenter", None)
    if center is None:
        center = port.center

    return Port(
        name=str(getattr(port, "name", "")),
        position=(float(center[0]), float(center[1])),
        orientation=_optional_float(getattr(port, "orientation", None)),
        width=_optional_float(getattr(port, "width", None)),
        layer=getattr(port, "layer", None),
        port_type=getattr(port, "port_type", None),
    )


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _compute_bbox(polygons: list[Polygon], ports: list[Port]) -> BBox:
    xs: list[float] = []
    ys: list[float] = []

    for polygon in polygons:
        for x, y in polygon:
            xs.append(x)
            ys.append(y)

    for port in ports:
        xs.append(port.position[0])
        ys.append(port.position[1])

    if not xs:
        return (0.0, 0.0, 0.0, 0.0)

    return (min(xs), min(ys), max(xs), max(ys))
