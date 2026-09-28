"""The physical-coordinate debug SVG: the realized electrical metal snapshot."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Any, Iterable, cast

from .svg_support import _all_port_accesses
from .terminal_contacts import terminal_contact_bboxes
from .types import (
    BBox,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    PadPlan,
    TerminalPairGroup,
)


class _SnapshotContext:
    def __init__(self, *, xmin: float, ymax: float) -> None:
        self.xmin = xmin
        self.ymax = ymax


def export_electrical_metal_snapshot_svg(
    path: str | Path,
    routed_component: object,
    obstacle_map: ElectricalObstacleMap,
    terminal_groups: tuple[TerminalPairGroup, ...],
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
    *,
    max_px: int = 1400,
) -> None:
    """Write a physical-coordinate SVG of the realized electrical metal."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        electrical_metal_snapshot_svg(
            routed_component,
            obstacle_map,
            terminal_groups,
            pad_plan,
            config,
            max_px=max_px,
        ),
        encoding="utf-8",
    )


def electrical_metal_snapshot_svg(
    routed_component: object,
    obstacle_map: ElectricalObstacleMap,
    terminal_groups: tuple[TerminalPairGroup, ...],
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
    *,
    max_px: int = 1400,
) -> str:
    metal_polygons = _component_layer_polygons_um(routed_component, config.metal_layer)
    heater_polygons = tuple(
        polygon
        for layer in config.heater_layers
        for polygon in _component_layer_polygons_um(routed_component, layer)
    )
    terminals = tuple(terminal for group in terminal_groups for terminal in group.terminals)
    pad_bboxes = (
        tuple(assignment.slot.bbox for assignment in pad_plan.assignments)
        if pad_plan is not None
        else ()
    )
    terminal_bboxes = tuple(terminal.bbox for terminal in terminals)
    contact_bboxes = tuple(
        bbox
        for terminal in terminals
        for bbox in _terminal_contact_bboxes(terminal, config.terminal_contact_width_um)
    )
    port_accesses = _all_port_accesses(obstacle_map)

    view_bbox = _expanded_bbox(
        (
            obstacle_map.die_bbox,
            obstacle_map.layout_bbox,
            *obstacle_map.raw_obstacle_bboxes,
            *pad_bboxes,
            *terminal_bboxes,
            *contact_bboxes,
            *(
                _point_bbox(point)
                for access in port_accesses
                for point in access.access_centerline_um
            ),
            *(_polygon_bbox(polygon) for polygon in metal_polygons),
            *(_polygon_bbox(polygon) for polygon in heater_polygons),
        ),
        margin_um=20.0,
    )
    if view_bbox is None:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />\n'

    xmin, ymin, xmax, ymax = view_bbox
    width_um = max(xmax - xmin, 1.0)
    height_um = max(ymax - ymin, 1.0)
    scale = min(max_px / width_um, max_px / height_um)
    width_px = max(1, round(width_um * scale))
    height_px = max(1, round(height_um * scale))
    ctx = _SnapshotContext(xmin=xmin, ymax=ymax)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" '
        f'viewBox="0 0 {width_um:.6g} {height_um:.6g}">',
        "<title>electrical metal snapshot</title>",
        '<rect width="100%" height="100%" fill="#fbfbf8" />',
    ]
    _append_snapshot_rect(
        parts,
        ctx,
        obstacle_map.die_bbox,
        fill="none",
        opacity=1.0,
        stroke="#6c757d",
        stroke_width=1.0,
        title="electrical die bbox",
    )
    _append_snapshot_rect(
        parts,
        ctx,
        obstacle_map.layout_bbox,
        fill="none",
        opacity=1.0,
        stroke="#adb5bd",
        stroke_width=0.7,
        title="source layout bbox",
    )
    for bbox in sorted(obstacle_map.raw_obstacle_bboxes):
        _append_snapshot_rect(
            parts,
            ctx,
            bbox,
            fill="#ced4da",
            opacity=0.34,
            stroke="#868e96",
            stroke_width=0.35,
            title="metal obstacle",
        )
    for polygon in sorted(heater_polygons):
        _append_snapshot_polygon(
            parts,
            ctx,
            polygon,
            fill="#f59f00",
            opacity=0.28,
            stroke="#e67700",
            stroke_width=0.35,
            title="heater layer",
        )
    for bbox in sorted(terminal_bboxes):
        _append_snapshot_rect(
            parts,
            ctx,
            bbox,
            fill="none",
            opacity=1.0,
            stroke="#d9480f",
            stroke_width=0.7,
            title="terminal bbox",
        )
    for bbox in sorted(contact_bboxes):
        _append_snapshot_rect(
            parts,
            ctx,
            bbox,
            fill="#15aabf",
            opacity=0.55,
            stroke="#0b7285",
            stroke_width=0.45,
            title="terminal contact",
        )
    for bbox in sorted(pad_bboxes):
        _append_snapshot_rect(
            parts,
            ctx,
            bbox,
            fill="#ffd43b",
            opacity=0.35,
            stroke="#e67700",
            stroke_width=0.5,
            title="assigned pad",
        )
    for access in port_accesses:
        _append_snapshot_polyline(
            parts,
            ctx,
            access.access_centerline_um,
            stroke="#7b2cbf" if access.purpose == "common_bus" else "#0081a7",
            stroke_width=0.6,
            opacity=0.9,
            title=(f"{access.purpose} access {access.terminal_id} anchor={access.anchor_cell}"),
        )
    for polygon in sorted(metal_polygons):
        _append_snapshot_polygon(
            parts,
            ctx,
            polygon,
            fill="#4263eb",
            opacity=0.72,
            stroke="#1c3faa",
            stroke_width=0.45,
            title="realized metal",
        )
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _component_layer_polygons_um(
    component: object,
    layer: tuple[int, int],
) -> tuple[tuple[tuple[float, float], ...], ...]:
    get_polygons = getattr(component, "get_polygons", None)
    if not callable(get_polygons):
        return ()
    polygons_by_layer = get_polygons(merge=False, by="tuple")
    if not isinstance(polygons_by_layer, dict):
        return ()
    dbu = _component_dbu_um(component)
    polygons = []
    for polygon in polygons_by_layer.get(layer, []):
        points = _polygon_points_um(polygon, dbu)
        if len(points) >= 3:
            polygons.append(points)
    return tuple(sorted(polygons))


def _component_dbu_um(component: object) -> float:
    kcl = getattr(component, "kcl", None)
    dbu = getattr(kcl, "dbu", 0.001)
    return float(dbu or 0.001)


def _polygon_points_um(
    polygon: object,
    dbu: float,
) -> tuple[tuple[float, float], ...]:
    each_point_hull = getattr(polygon, "each_point_hull", None)
    if callable(each_point_hull):
        return tuple(
            (float(point.x) * dbu, float(point.y) * dbu)
            for point in cast(Iterable[Any], each_point_hull())
        )
    if isinstance(polygon, (tuple, list)):
        points: list[tuple[float, float]] = []
        for item in polygon:
            if (
                isinstance(item, (tuple, list))
                and len(item) >= 2
                and isinstance(item[0], (int, float))
                and isinstance(item[1], (int, float))
            ):
                points.append((float(item[0]), float(item[1])))
        return tuple(points)
    return ()


def _terminal_contact_bboxes(
    terminal: ElectricalTerminal,
    fallback_width_um: float,
) -> tuple[BBox, ...]:
    return terminal_contact_bboxes(terminal, fallback_width_um)


def _point_bbox(point: tuple[float, float]) -> BBox:
    x, y = point
    return (x, y, x, y)


def _expanded_bbox(
    bboxes: Iterable[BBox | None],
    *,
    margin_um: float,
) -> BBox | None:
    valid = tuple(bbox for bbox in bboxes if bbox is not None)
    if not valid:
        return None
    xmin = min(bbox[0] for bbox in valid) - margin_um
    ymin = min(bbox[1] for bbox in valid) - margin_um
    xmax = max(bbox[2] for bbox in valid) + margin_um
    ymax = max(bbox[3] for bbox in valid) + margin_um
    return (xmin, ymin, xmax, ymax)


def _polygon_bbox(polygon: tuple[tuple[float, float], ...]) -> BBox:
    xs = tuple(point[0] for point in polygon)
    ys = tuple(point[1] for point in polygon)
    return (min(xs), min(ys), max(xs), max(ys))


def _append_snapshot_rect(
    parts: list[str],
    ctx: _SnapshotContext,
    bbox: BBox,
    *,
    fill: str,
    opacity: float,
    stroke: str,
    stroke_width: float,
    title: str,
) -> None:
    xmin, ymin, xmax, ymax = bbox
    x = xmin - ctx.xmin
    y = ctx.ymax - ymax
    width = xmax - xmin
    height = ymax - ymin
    if width <= 0.0 or height <= 0.0:
        return
    parts.append(
        f'<rect x="{x:.6g}" y="{y:.6g}" width="{width:.6g}" height="{height:.6g}" '
        f'fill="{fill}" opacity="{opacity:.6g}" stroke="{stroke}" '
        f'stroke-width="{stroke_width:.6g}" vector-effect="non-scaling-stroke">'
        f"<title>{escape(title)}</title></rect>"
    )


def _append_snapshot_polygon(
    parts: list[str],
    ctx: _SnapshotContext,
    polygon: tuple[tuple[float, float], ...],
    *,
    fill: str,
    opacity: float,
    stroke: str,
    stroke_width: float,
    title: str,
) -> None:
    if len(polygon) < 3:
        return
    points = " ".join(f"{x - ctx.xmin:.6g},{ctx.ymax - y:.6g}" for x, y in polygon)
    parts.append(
        f'<polygon points="{points}" fill="{fill}" opacity="{opacity:.6g}" '
        f'stroke="{stroke}" stroke-width="{stroke_width:.6g}" '
        f'vector-effect="non-scaling-stroke"><title>{escape(title)}</title></polygon>'
    )


def _append_snapshot_polyline(
    parts: list[str],
    ctx: _SnapshotContext,
    points_um: tuple[tuple[float, float], ...],
    *,
    stroke: str,
    stroke_width: float,
    opacity: float,
    title: str,
) -> None:
    if len(points_um) < 2:
        return
    points = " ".join(f"{x - ctx.xmin:.6g},{ctx.ymax - y:.6g}" for x, y in points_um)
    parts.append(
        f'<polyline points="{points}" fill="none" stroke="{stroke}" '
        f'stroke-width="{stroke_width:.6g}" opacity="{opacity:.6g}" '
        f'stroke-linecap="round" stroke-linejoin="round" '
        f'vector-effect="non-scaling-stroke"><title>{escape(title)}</title></polyline>'
    )
