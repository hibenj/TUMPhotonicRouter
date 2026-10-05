"""The routes overview SVG: every routed waveguide with its length, and a legend.

Written under ``--debug-svgs`` next to the per-route SVGs
(``build/routes/<prefix>_overview.svg``; route-overview plan, 2026-10-05).
Drawn from the flow's own records, not from the GDS: a route's centerline
and routed length come from its ``RoutedNetRecord`` (the corrected
centerline, before meanders), the meander length the path-length matching
inserted on it from the meander insertion report (by net name). The
meander geometry is not redrawn; its length is in the label. The legend
carries the benchmark name, the route count, the total routed length
without meanders, the total meander length, the total with meanders, the
routable die area and the layout's bounding box.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol
from xml.sax.saxutils import escape

BBox = tuple[float, float, float, float]

CANVAS_WIDTH_PX = 2000.0
MARGIN_PX = 40.0
LABEL_FONT_PX = 9.0
LEGEND_FONT_PX = 14.0
ROUTE_COLOR = "#1f5fbf"
MEANDER_ROUTE_COLOR = "#c2410c"


class RouteLike(Protocol):
    """What the writer reads from a ``RoutedNetRecord``."""

    net_name: str
    total_length_um: float
    corrected_centerline_um: tuple[tuple[float, float], ...]


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
        lines = [
            f"routes: {self.route_count}"
            + (
                f" ({self.route_count - self.drawn_count} not drawn)"
                if self.drawn_count != self.route_count
                else ""
            ),
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


def write_routes_overview_svg(
    path: Path,
    *,
    benchmark_name: str,
    records: list[RouteLike],
    realized_extra_um_by_net: dict[str, float],
    routable_bbox_um: BBox | None,
    layout_bbox_um: BBox | None,
) -> RoutesOverviewSummary:
    """Write the overview SVG to ``path`` and return the legend's numbers.

    ``realized_extra_um_by_net`` is the meander length actually drawn on
    each net (``realized_extra_um_by_net`` of the meander report), so a
    label is the net's realized length.
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

    extent = _drawing_extent(drawn, routable_bbox_um, layout_bbox_um)
    xmin, ymin, xmax, ymax = extent
    width_um = max(xmax - xmin, 1.0)
    height_um = max(ymax - ymin, 1.0)
    scale = (CANVAS_WIDTH_PX - 2 * MARGIN_PX) / width_um
    canvas_height_px = height_um * scale + 2 * MARGIN_PX

    def to_px(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        return (MARGIN_PX + (x - xmin) * scale, MARGIN_PX + (ymax - y) * scale)

    parts: list[str] = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_WIDTH_PX:.0f}" '
            f'height="{canvas_height_px:.0f}" '
            f'viewBox="0 0 {CANVAS_WIDTH_PX:.0f} {canvas_height_px:.0f}">'
        ),
        f"<title>{escape(benchmark_name)} routes overview</title>",
        (
            f'<rect x="0" y="0" width="{CANVAS_WIDTH_PX:.0f}" '
            f'height="{canvas_height_px:.0f}" fill="white"/>'
        ),
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
    parts.append('<g id="routes" fill="none" stroke-width="1">')
    for record in drawn:
        points = [to_px(point) for point in record.corrected_centerline_um]
        meander_um = float(realized_extra_um_by_net.get(record.net_name, 0.0))
        color = MEANDER_ROUTE_COLOR if meander_um > 0.0 else ROUTE_COLOR
        path_data = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
        parts.append(
            f'<polyline points="{path_data}" stroke="{color}">'
            f"<title>{escape(record.net_name)}</title></polyline>"
        )
    parts.append("</g>")
    parts.append(
        f'<g id="labels" font-family="sans-serif" font-size="{LABEL_FONT_PX:.0f}" '
        'paint-order="stroke" stroke="white" stroke-width="3" fill="#111827">'
    )
    for record in drawn:
        points = record.corrected_centerline_um
        mx, my = to_px(points[len(points) // 2])
        label = escape(_route_label(record, realized_extra_um_by_net))
        parts.append(f'<text x="{mx + 2:.1f}" y="{my - 2:.1f}">{label}</text>')
    parts.append("</g>")
    legend_lines = (escape(benchmark_name), *summary.legend_lines)
    line_height = LEGEND_FONT_PX * 1.4
    box_height = line_height * len(legend_lines) + 12
    box_width = max(len(line) for line in legend_lines) * LEGEND_FONT_PX * 0.62 + 20
    parts.append(
        f'<g id="legend" font-family="sans-serif" font-size="{LEGEND_FONT_PX:.0f}" fill="#111827">'
    )
    parts.append(
        f'<rect x="{MARGIN_PX:.0f}" y="{MARGIN_PX:.0f}" width="{box_width:.0f}" '
        f'height="{box_height:.0f}" fill="white" fill-opacity="0.9" stroke="#111827" '
        'stroke-width="1"/>'
    )
    for index, line in enumerate(legend_lines):
        y = MARGIN_PX + 8 + line_height * (index + 1) - 4
        parts.append(f'<text x="{MARGIN_PX + 10:.0f}" y="{y:.0f}">{line}</text>')
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
    drawn: list[RouteLike], routable_bbox_um: BBox | None, layout_bbox_um: BBox | None
) -> BBox:
    xs: list[float] = []
    ys: list[float] = []
    for bbox in (routable_bbox_um, layout_bbox_um):
        if bbox is not None:
            xs.extend((bbox[0], bbox[2]))
            ys.extend((bbox[1], bbox[3]))
    for record in drawn:
        for x, y in record.corrected_centerline_um:
            xs.append(x)
            ys.append(y)
    if not xs:
        return (0.0, 0.0, 1.0, 1.0)
    return (min(xs), min(ys), max(xs), max(ys))
