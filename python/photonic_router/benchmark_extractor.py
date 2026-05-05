"""Extract fixed geometry and ports from gdsfactory benchmark components."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Tuple

Point = Tuple[float, float]
Polygon = List[Point]
BBox = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Port:
    """Router-facing port metadata in physical micrometer coordinates."""

    name: str
    position: Point
    orientation: Optional[float]
    width: Optional[float] = None
    layer: Optional[Any] = None
    port_type: Optional[str] = None


@dataclass(frozen=True)
class ExtractedBenchmark:
    """Geometry extracted from a gdsfactory component.

    Attributes:
        polygons: Flattened fixed layout polygons in micrometers.
        ports: Top-level and direct component-reference ports.
        bbox: Bounding box `(xmin, ymin, xmax, ymax)` in micrometers.
    """

    polygons: List[Polygon]
    ports: List[Port]
    bbox: BBox


def extract_benchmark(component: Any, layers: Optional[Iterable[Any]] = None) -> ExtractedBenchmark:
    """Extract flattened geometry, reference ports, and bbox from a component.

    Args:
        component: A `gdsfactory.Component`.
        layers: Optional layer filter passed to `component.get_polygons`.

    Returns:
        Extracted benchmark data in physical micrometer units.
    """

    polygons = _extract_polygons(component, layers=layers)
    ports = _extract_ports(component)
    bbox = _compute_bbox(polygons, ports)
    return ExtractedBenchmark(polygons=polygons, ports=ports, bbox=bbox)


def _extract_polygons(component: Any, layers: Optional[Iterable[Any]] = None) -> List[Polygon]:
    polygon_map = component.get_polygons(merge=False, by="tuple", layers=layers)
    dbu = float(getattr(component.kcl, "dbu", 1.0))

    polygons: List[Polygon] = []
    for layer_polygons in polygon_map.values():
        for polygon in layer_polygons:
            points = _polygon_points_um(polygon, dbu)
            if len(points) >= 3:
                polygons.append(points)
    return polygons


def _polygon_points_um(polygon: Any, dbu: float) -> Polygon:
    """Convert a KLayout/kfactory polygon hull to micrometer points."""

    if hasattr(polygon, "to_dtype"):
        dpolygon = polygon.to_dtype(dbu)
        return [(float(point.x), float(point.y)) for point in dpolygon.each_point_hull()]

    return [
        (float(point.x) * dbu, float(point.y) * dbu)
        for point in polygon.each_point_hull()
    ]


def _extract_ports(component: Any) -> List[Port]:
    ports: List[Port] = []

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
        center = getattr(port, "center")

    return Port(
        name=str(getattr(port, "name", "")),
        position=(float(center[0]), float(center[1])),
        orientation=_optional_float(getattr(port, "orientation", None)),
        width=_optional_float(getattr(port, "width", None)),
        layer=getattr(port, "layer", None),
        port_type=getattr(port, "port_type", None),
    )


def _optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    return float(value)


def _compute_bbox(polygons: List[Polygon], ports: List[Port]) -> BBox:
    xs: List[float] = []
    ys: List[float] = []

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
