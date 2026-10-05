"""The routes overview SVG: the routed layout with every waveguide's length, and a legend.

Written under ``--debug-svgs`` next to the per-route SVGs
(``build/routes/<prefix>_overview.svg``; route-overview plan, 2026-10-05).
The picture is the routed layout: the benchmark instances as boxes with
their names, the routed waveguides as their real polygons (meanders
included) from the routed component's own waveguide shapes, and the die
and layout bounding boxes. Each route's label carries its realized length
(routed plus the meander actually drawn on it, the bookkeeping by physical
edge of the meander report), placed at the middle of its routed centerline.
The legend is a band below the drawing, never over it: the benchmark name,
the route count, the total routed length without meanders, the total
meander length, the total with meanders, the die area and the layout box.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol
from xml.sax.saxutils import escape

BBox = tuple[float, float, float, float]
Point = tuple[float, float]

CANVAS_WIDTH_PX = 2400.0
MARGIN_PX = 30.0
LABEL_FONT_PX = 8.0
LEGEND_FONT_PX = 15.0
INSTANCE_FONT_PX = 7.0
ROUTE_COLOR = "#1f5fbf"
MEANDER_ROUTE_COLOR = "#c2410c"
INSTANCE_FILL = "#e5e7eb"
INSTANCE_STROKE = "#9ca3af"


class RouteLike(Protocol):
    """What the writer reads from a ``RoutedNetRecord``."""

    net_name: str
    total_length_um: float
    corrected_centerline_um: tuple[Point, ...]


@dataclass(frozen=True)
class InstanceBox:
    """One benchmark instance: its name and bounding box in um."""

    name: str
    bbox: BBox


@dataclass(frozen=True)
class RoutesOverviewSummary:
    """The numbers the legend shows, returned so the caller can print and test them."""

    route_count: int
    drawn_count: int
    routed_length_um: float
    meander_length_um: float
    total_length_um: float
    routable_bbox_um: BBox | None
    layout_bbox_um: BBox | None

    @property
    def legend_lines(self) -> tuple[str, ...]:
        not_drawn = self.route_count - self.drawn_count
        lines = [
            f"routes: {self.route_count}" + (f" ({not_drawn} not drawn)" if not_drawn else ""),
            f"routed length without meanders: {self.routed_length_um:.1f} um",
            f"meander length: {self.meander_length_um:.1f} um",
            f"total length with meanders: {self.total_length_um:.1f} um",
        ]
        if self.routable_bbox_um is not None:
            lines.append(f"chip (routable die area): {_bbox_size(self.routable_bbox_um)}")
        if self.layout_bbox_um is not None:
            lines.append(f"layout bounding box: {_bbox_size(self.layout_bbox_um)}")
        return tuple(lines)


def realized_extra_um_by_net(meander_report: object) -> dict[str, float]:
    """The meander report's realized extra length per net (bookkeeping by physical edge).

    From ``realized_extra_length_um_by_edge`` (2026-10-05); a report without
    it falls back to the results' booked ``inserted_extra_length_um`` per
    requiring edge.
    """

    if not isinstance(meander_report, dict):
        return {}
    realized: dict[str, float] = {}
    entries = meander_report.get("realized_extra_length_um_by_edge")
    if isinstance(entries, list):
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            edge = entry.get("edge", {})
            net_name = str(edge.get("net_name", "")) if isinstance(edge, dict) else ""
            if net_name:
                realized[net_name] = realized.get(net_name, 0.0) + float(
                    entry.get("extra_length_um", 0.0)
                )
        return realized
    for item in meander_report.get("results", []) or []:
        if not isinstance(item, dict):
            continue
        edge = item.get("edge", {})
        net_name = str(edge.get("net_name", "")) if isinstance(edge, dict) else ""
        if net_name:
            realized[net_name] = realized.get(net_name, 0.0) + float(
                item.get("inserted_extra_length_um", 0.0)
            )
    return realized


def layout_overview_shapes(component: object) -> tuple[list[InstanceBox], list[list[Point]]]:
    """Instances (name, bbox in um) and the routed waveguide polygons of a routed component.

    The routed waveguides are the component's own shapes on the waveguide
    layer (1, 0); the benchmark instances are references, read as boxes.
    Anything the component cannot provide is returned empty.
    """

    instances: list[InstanceBox] = []
    try:
        for inst in component.insts:  # type: ignore[attr-defined]
            box = inst.dbbox()
            instances.append(
                InstanceBox(
                    name=str(getattr(inst, "name", "") or ""),
                    bbox=(float(box.left), float(box.bottom), float(box.right), float(box.top)),
                )
            )
    except Exception:
        instances = []
    polygons: list[list[Point]] = []
    try:
        import klayout.db as kdb

        layer_index = component.kcl.layer(kdb.LayerInfo(1, 0))  # type: ignore[attr-defined]
        for shape in component.shapes(layer_index).each():  # type: ignore[attr-defined]
            polygon = shape.dpolygon
            polygons.append([(float(p.x), float(p.y)) for p in polygon.each_point_hull()])
    except Exception:
        polygons = []
    return instances, polygons


def write_routes_overview_svg(
    path: Path,
    *,
    benchmark_name: str,
    records: list[RouteLike],
    realized_extra_um_by_net: dict[str, float],
    routable_bbox_um: BBox | None,
    layout_bbox_um: BBox | None,
    instances: list[InstanceBox] = (),  # type: ignore[assignment]
    waveguide_polygons: list[list[Point]] = (),  # type: ignore[assignment]
) -> RoutesOverviewSummary:
    """Write the overview SVG to ``path`` and return the legend's numbers.

    ``realized_extra_um_by_net`` is the meander length actually drawn on
    each net, so a label is the net's realized length. ``instances`` and
    ``waveguide_polygons`` (``layout_overview_shapes``) make the picture the
    layout; without them only the routed centerlines are drawn.
    """

    drawn = [record for record in records if len(record.corrected_centerline_um) >= 2]
    routed_length_um = sum(float(record.total_length_um) for record in records)
    meander_length_um = sum(
        float(realized_extra_um_by_net.get(record.net_name, 0.0)) for record in records
    )
    summary = RoutesOverviewSummary(
        route_count=len(records),
        drawn_count=len(drawn),
        routed_length_um=routed_length_um,
        meander_length_um=meander_length_um,
        total_length_um=routed_length_um + meander_length_um,
        routable_bbox_um=routable_bbox_um,
        layout_bbox_um=layout_bbox_um,
    )

    xmin, ymin, xmax, ymax = _drawing_extent(
        drawn, routable_bbox_um, layout_bbox_um, instances, waveguide_polygons
    )
    width_um = max(xmax - xmin, 1.0)
    height_um = max(ymax - ymin, 1.0)
    scale = (CANVAS_WIDTH_PX - 2 * MARGIN_PX) / width_um
    drawing_height_px = height_um * scale + 2 * MARGIN_PX
    legend_lines = (escape(benchmark_name), *summary.legend_lines)
    line_height = LEGEND_FONT_PX * 1.5
    legend_height_px = line_height * len(legend_lines) + 2 * MARGIN_PX
    canvas_height_px = drawing_height_px + legend_height_px

    def to_px(point: Point) -> tuple[float, float]:
        x, y = point
        return (MARGIN_PX + (x - xmin) * scale, MARGIN_PX + (ymax - y) * scale)

    size = f'width="{CANVAS_WIDTH_PX:.0f}" height="{canvas_height_px:.0f}"'
    parts: list[str] = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" {size} '
            f'viewBox="0 0 {CANVAS_WIDTH_PX:.0f} {canvas_height_px:.0f}">'
        ),
        f"<title>{escape(benchmark_name)} routes overview</title>",
        f'<rect x="0" y="0" {size} fill="white"/>',
    ]
    for bbox, dash, color, label in (
        (routable_bbox_um, "8 4", "#6b7280", "routable die area"),
        (layout_bbox_um, "2 3", "#9ca3af", "layout bounding box"),
    ):
        if bbox is None:
            continue
        (x0, y0), (x1, y1) = to_px((bbox[0], bbox[3])), to_px((bbox[2], bbox[1]))
        parts.append(
            f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{x1 - x0:.1f}" height="{y1 - y0:.1f}" '
            f'fill="none" stroke="{color}" stroke-width="1" stroke-dasharray="{dash}">'
            f"<title>{label}: {_bbox_size(bbox)}</title></rect>"
        )

    parts.append(
        f'<g id="instances" fill="{INSTANCE_FILL}" stroke="{INSTANCE_STROKE}" stroke-width="0.5">'
    )
    for instance in instances:
        (x0, y0), (x1, y1) = (
            to_px((instance.bbox[0], instance.bbox[3])),
            to_px((instance.bbox[2], instance.bbox[1])),
        )
        parts.append(
            f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{max(x1 - x0, 0.5):.1f}" '
            f'height="{max(y1 - y0, 0.5):.1f}"><title>{escape(instance.name)}</title></rect>'
        )
    parts.append("</g>")
    parts.append(
        f'<g id="instance-names" font-family="sans-serif" font-size="{INSTANCE_FONT_PX:.0f}" '
        'fill="#4b5563">'
    )
    for instance in instances:
        x0, y0 = to_px((instance.bbox[0], instance.bbox[3]))
        parts.append(f'<text x="{x0:.1f}" y="{y0 - 1.5:.1f}">{escape(instance.name)}</text>')
    parts.append("</g>")

    # the routed waveguides: real polygons, each colored by the route it belongs to
    meander_nets = {
        record.net_name
        for record in records
        if float(realized_extra_um_by_net.get(record.net_name, 0.0)) > 0.0
    }
    polygon_owner = _polygon_owners(waveguide_polygons, drawn)
    # a 0.5 um waveguide is well under one pixel at this scale: the stroke
    # keeps every route visible without hiding the meander shapes
    parts.append('<g id="waveguides" stroke-width="0.8" stroke-linejoin="round">')
    for index, polygon in enumerate(waveguide_polygons):
        owner = polygon_owner.get(index)
        color = MEANDER_ROUTE_COLOR if owner in meander_nets else ROUTE_COLOR
        points = " ".join(f"{x:.1f},{y:.1f}" for x, y in (to_px(point) for point in polygon))
        title = f"<title>{escape(owner)}</title>" if owner else ""
        parts.append(
            f'<polygon points="{points}" fill="{color}" stroke="{color}">{title}</polygon>'
        )
    parts.append("</g>")
    if not waveguide_polygons:
        parts.append('<g id="routes" fill="none" stroke-width="1">')
        for record in drawn:
            color = MEANDER_ROUTE_COLOR if record.net_name in meander_nets else ROUTE_COLOR
            points = " ".join(
                f"{x:.1f},{y:.1f}" for x, y in (to_px(p) for p in record.corrected_centerline_um)
            )
            parts.append(
                f'<polyline points="{points}" stroke="{color}">'
                f"<title>{escape(record.net_name)}</title></polyline>"
            )
        parts.append("</g>")

    parts.append(
        f'<g id="labels" font-family="sans-serif" font-size="{LABEL_FONT_PX:.0f}" '
        'paint-order="stroke" stroke="white" stroke-width="2.5" fill="#111827">'
    )
    for record in drawn:
        points = record.corrected_centerline_um
        mx, my = to_px(points[len(points) // 2])
        label = escape(_route_label(record, realized_extra_um_by_net))
        parts.append(f'<text x="{mx + 2:.1f}" y="{my - 2:.1f}">{label}</text>')
    parts.append("</g>")

    # the legend band below the drawing
    band_top = drawing_height_px
    parts.append(
        f'<g id="legend" font-family="sans-serif" font-size="{LEGEND_FONT_PX:.0f}" fill="#111827">'
    )
    parts.append(
        f'<rect x="0" y="{band_top:.0f}" width="{CANVAS_WIDTH_PX:.0f}" '
        f'height="{legend_height_px:.0f}" fill="#f9fafb" stroke="#111827" stroke-width="1"/>'
    )
    for index, line in enumerate(legend_lines):
        y = band_top + MARGIN_PX + line_height * (index + 1) - 5
        weight = ' font-weight="bold"' if index == 0 else ""
        parts.append(f'<text x="{MARGIN_PX:.0f}" y="{y:.0f}"{weight}>{line}</text>')
    swatch_x = CANVAS_WIDTH_PX - 420
    for row, (color, text) in enumerate(
        ((ROUTE_COLOR, "route without meander"), (MEANDER_ROUTE_COLOR, "route with meander"))
    ):
        y = band_top + MARGIN_PX + line_height * (row + 1) - 5
        parts.append(
            f'<rect x="{swatch_x:.0f}" y="{y - 11:.0f}" width="24" height="12" fill="{color}"/>'
            f'<text x="{swatch_x + 32:.0f}" y="{y:.0f}">{text}</text>'
        )
    parts.append("</g></svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")
    return summary


def _route_label(record: RouteLike, realized_extra_um_by_net: dict[str, float]) -> str:
    meander_um = float(realized_extra_um_by_net.get(record.net_name, 0.0))
    if meander_um > 0.0:
        return f"{float(record.total_length_um) + meander_um:.1f} um (+{meander_um:.1f} meander)"
    return f"{float(record.total_length_um):.1f} um"


def _bbox_size(bbox: BBox) -> str:
    return f"{bbox[2] - bbox[0]:.1f} x {bbox[3] - bbox[1]:.1f} um"


def _drawing_extent(
    drawn: list[RouteLike],
    routable_bbox_um: BBox | None,
    layout_bbox_um: BBox | None,
    instances: list[InstanceBox],
    waveguide_polygons: list[list[Point]],
) -> BBox:
    xs: list[float] = []
    ys: list[float] = []
    for bbox in (routable_bbox_um, layout_bbox_um, *(inst.bbox for inst in instances)):
        if bbox is not None:
            xs.extend((bbox[0], bbox[2]))
            ys.extend((bbox[1], bbox[3]))
    for record in drawn:
        for x, y in record.corrected_centerline_um:
            xs.append(x)
            ys.append(y)
    for polygon in waveguide_polygons:
        for x, y in polygon:
            xs.append(x)
            ys.append(y)
    if not xs:
        return (0.0, 0.0, 1.0, 1.0)
    return (min(xs), min(ys), max(xs), max(ys))


def _polygon_owners(polygons: list[list[Point]], drawn: list[RouteLike]) -> dict[int, str]:
    """Which route each waveguide polygon belongs to.

    The route whose sampled centerline points the polygon contains.
    """

    if not polygons:
        return {}
    boxes = [
        (
            min(x for x, _ in polygon),
            min(y for _, y in polygon),
            max(x for x, _ in polygon),
            max(y for _, y in polygon),
        )
        for polygon in polygons
    ]
    owners: dict[int, str] = {}
    for record in drawn:
        points = record.corrected_centerline_um
        samples = [points[int(len(points) * f)] for f in (0.1, 0.3, 0.5, 0.7, 0.9)]
        for sample in samples:
            hit = next(
                (
                    index
                    for index, (polygon, box) in enumerate(zip(polygons, boxes, strict=True))
                    if index not in owners
                    and box[0] <= sample[0] <= box[2]
                    and box[1] <= sample[1] <= box[3]
                    and _point_in_polygon(sample, polygon)
                ),
                None,
            )
            if hit is not None:
                owners[hit] = record.net_name
                break
    return owners


def _point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    x, y = point
    inside = False
    count = len(polygon)
    for index in range(count):
        x0, y0 = polygon[index]
        x1, y1 = polygon[(index + 1) % count]
        if (y0 > y) != (y1 > y):
            x_cross = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x_cross > x:
                inside = not inside
    return inside
