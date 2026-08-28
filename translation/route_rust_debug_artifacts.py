"""Debug probe SVG and failure artifact writing for the Rust-backed photonic router."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import cast

from gdsfactory.component import Component

from translation.gds_write_options import gds_save_options
from translation.photonic_verification import PhotonicVerificationResult
from translation.route_rust_records import EndpointCorrectionRouter
from translation.route_rust_types import RoutedNetRecord


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _as_point_list(raw_points: Iterable[object]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for raw_point in raw_points:
        if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
            continue
        try:
            x = float(raw_point[0])
            y = float(raw_point[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def _centerline_bbox_um(
    centerlines: Iterable[Iterable[tuple[float, float]]],
) -> tuple[float, float, float, float] | None:
    xs: list[float] = []
    ys: list[float] = []
    for centerline in centerlines:
        for x, y in centerline:
            xs.append(float(x))
            ys.append(float(y))
    if not xs or not ys:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def _normalize_um_bbox(raw_bbox: object) -> tuple[float, float, float, float] | None:
    if not isinstance(raw_bbox, (tuple, list)) or len(raw_bbox) != 4:
        return None
    try:
        min_x, min_y, max_x, max_y = (float(value) for value in raw_bbox)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (min_x, min_y, max_x, max_y)):
        return None
    if max_x < min_x:
        min_x, max_x = max_x, min_x
    if max_y < min_y:
        min_y, max_y = max_y, min_y
    return min_x, min_y, max_x, max_y


def _expanded_bbox_um(
    bbox: tuple[float, float, float, float],
    *,
    padding_um: float,
) -> tuple[float, float, float, float]:
    min_x, min_y, max_x, max_y = bbox
    padding = max(0.0, float(padding_um))
    return min_x - padding, min_y - padding, max_x + padding, max_y + padding


def _write_centerline_probe_svg(
    path: Path,
    *,
    centerlines_by_net: Mapping[str, Mapping[str, list[tuple[float, float]]]],
    overlap_bboxes_um: Iterable[tuple[float, float, float, float]],
    view_bbox_um: tuple[float, float, float, float],
) -> None:
    min_x, min_y, max_x, max_y = view_bbox_um
    width = max(1.0, max_x - min_x)
    height = max(1.0, max_y - min_y)
    colors = [
        ("#0b57d0", "#7baaf7"),
        ("#d93025", "#f28b82"),
        ("#188038", "#81c995"),
        ("#b06000", "#fbbc04"),
    ]

    def sx(x: float) -> float:
        return float(x) - min_x

    def sy(y: float) -> float:
        return max_y - float(y)

    def polyline(points: list[tuple[float, float]]) -> str:
        return " ".join(f"{sx(x):.6g},{sy(y):.6g}" for x, y in points)

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="1100" height="800" viewBox="0 0 {width:.6g} {height:.6g}">',
        '<rect width="100%" height="100%" fill="#ffffff" />',
        '<g stroke="#d0d7de" stroke-width="0.15" opacity="0.55">',
    ]
    grid_step = 2.0
    gx = math.floor(min_x / grid_step) * grid_step
    while gx <= max_x:
        parts.append(f'<path d="M {sx(gx):.6g} 0 V {height:.6g}" fill="none" />')
        gx += grid_step
    gy = math.floor(min_y / grid_step) * grid_step
    while gy <= max_y:
        parts.append(f'<path d="M 0 {sy(gy):.6g} H {width:.6g}" fill="none" />')
        gy += grid_step
    parts.append("</g>")

    for bbox in overlap_bboxes_um:
        bx0, by0, bx1, by1 = bbox
        parts.append(
            f'<rect x="{sx(bx0):.6g}" y="{sy(by1):.6g}" '
            f'width="{max(0.1, bx1 - bx0):.6g}" '
            f'height="{max(0.1, by1 - by0):.6g}" '
            'fill="#ff00aa" opacity="0.32" stroke="#9c0069" stroke-width="0.25" />'
        )

    for index, (net_name, net_centerlines) in enumerate(centerlines_by_net.items()):
        solid, dashed = colors[index % len(colors)]
        primitive = net_centerlines.get("primitive_centerline_um", [])
        corrected = net_centerlines.get("corrected_centerline_um", [])
        if primitive:
            parts.append(
                f'<polyline points="{polyline(primitive)}" fill="none" '
                f'stroke="{dashed}" stroke-width="0.55" stroke-dasharray="2 1" '
                'stroke-linecap="round" stroke-linejoin="round" />'
            )
        if corrected:
            parts.append(
                f'<polyline points="{polyline(corrected)}" fill="none" '
                f'stroke="{solid}" stroke-width="0.85" '
                'stroke-linecap="round" stroke-linejoin="round" />'
            )
        label_point = (corrected or primitive or [(min_x, max_y)])[0]
        parts.append(
            f'<text x="{sx(label_point[0]):.6g}" y="{sy(label_point[1]):.6g}" '
            f'font-size="3" fill="{solid}">{net_name}</text>'
        )

    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _dump_photonic_probe_failure_artifacts(
    *,
    debug_path: Path,
    debug_prefix: str,
    probe_layout: Component | None,
    verification: PhotonicVerificationResult,
    records: Iterable[RoutedNetRecord],
    router: EndpointCorrectionRouter,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_unchecked_bumps: bool,
) -> dict[str, str]:
    probe_dir = debug_path / "photonic_probe_failures"
    _ensure_dir(probe_dir)
    stem = f"{debug_prefix}_photonic_probe_failure"
    artifacts: dict[str, str] = {}

    if probe_layout is not None:
        gds_path = probe_dir / f"{stem}.gds"
        probe_layout.write_gds(str(gds_path), save_options=gds_save_options())
        artifacts["probe_gds"] = str(gds_path)

    issues_path = probe_dir / f"{stem}.json"
    issues_path.write_text(
        json.dumps(verification.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifacts["issues_json"] = str(issues_path)

    issue_net_names: set[str] = set()
    overlap_bboxes: list[tuple[float, float, float, float]] = []
    for issue in verification.issues:
        if issue.net_name:
            issue_net_names.add(str(issue.net_name))
        other = (issue.details or {}).get("other_net_name")
        if isinstance(other, str):
            issue_net_names.add(other)
        bbox = _normalize_um_bbox((issue.details or {}).get("overlap_bbox_um"))
        if bbox is not None:
            overlap_bboxes.append(bbox)

    records_by_name = {record.net_name: record for record in records}
    centerlines_by_net: dict[str, dict[str, object]] = {}
    for net_name in sorted(issue_net_names):
        record = records_by_name.get(net_name)
        if record is None:
            continue
        item: dict[str, object] = {
            "net_id": record.net_id,
            "source": str(record.source),
            "target": str(record.target),
            "source_port_center_um": record.source_port_center_um,
            "target_port_center_um": record.target_port_center_um,
            "endpoint_correction_error": record.endpoint_correction_error,
        }
        try:
            primitive = _as_point_list(router.route_primitive_centerline(record.route_obj))
        except Exception as exc:
            primitive = []
            item["primitive_centerline_error"] = str(exc)
        corrected = _as_point_list(record.corrected_centerline_um)
        if not corrected:
            try:
                corrected = _as_point_list(
                    router.route_port_corrected_centerline(
                        record.route_obj,
                        source_port_um=record.source_port_center_um,
                        target_port_um=record.target_port_center_um,
                        allow_unchecked_bumps=allow_unchecked_bumps,
                    )
                )
            except Exception as exc:
                corrected = []
                item["corrected_centerline_error"] = str(exc)
        item["primitive_centerline_um"] = primitive
        item["corrected_centerline_um"] = corrected
        centerlines_by_net[net_name] = item

    centerline_path = probe_dir / f"{stem}_centerlines.json"
    centerline_path.write_text(
        json.dumps(centerlines_by_net, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifacts["centerlines_json"] = str(centerline_path)

    svg_centerlines: dict[str, Mapping[str, list[tuple[float, float]]]] = {}
    for net_name, item in centerlines_by_net.items():
        svg_centerlines[net_name] = {
            "primitive_centerline_um": cast(
                list[tuple[float, float]], item.get("primitive_centerline_um", [])
            ),
            "corrected_centerline_um": cast(
                list[tuple[float, float]], item.get("corrected_centerline_um", [])
            ),
        }
    focus_bbox = (
        overlap_bboxes[0]
        if overlap_bboxes
        else _centerline_bbox_um(
            line for lines in svg_centerlines.values() for line in lines.values()
        )
    )
    if focus_bbox is not None:
        svg_path = probe_dir / f"{stem}_centerlines.svg"
        _write_centerline_probe_svg(
            svg_path,
            centerlines_by_net=svg_centerlines,
            overlap_bboxes_um=overlap_bboxes,
            view_bbox_um=_expanded_bbox_um(focus_bbox, padding_um=40.0),
        )
        artifacts["centerlines_svg"] = str(svg_path)

    summary_path = probe_dir / f"{stem}.txt"
    lines = [
        "Photonic probe failure artifacts",
        f"debug_prefix={debug_prefix}",
        f"realization_grid_spec={realization_grid_spec}",
    ]
    for key, value in sorted(artifacts.items()):
        lines.append(f"{key}={value}")
    for issue in verification.issues[:10]:
        lines.append(
            f"issue={issue.code} net={issue.net_name} message={issue.message} "
            f"details={issue.details}"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    artifacts["summary_txt"] = str(summary_path)
    return artifacts


def _cells_bbox(cells: set[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), max(xs), min(ys), max(ys)


def _rect_cell_count(
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    width = max(0, max_x - min_x + 1)
    height = max(0, max_y - min_y + 1)
    return width * height


def _rect_overlap_cell_count(
    rect: tuple[int, int, int, int],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    rect_min_x, rect_min_y, rect_max_x, rect_max_y = rect
    overlap_min_x = max(min_x, rect_min_x)
    overlap_max_x = min(max_x, rect_max_x)
    overlap_min_y = max(min_y, rect_min_y)
    overlap_max_y = min(max_y, rect_max_y)
    return _rect_cell_count(
        min_x=overlap_min_x,
        max_x=overlap_max_x,
        min_y=overlap_min_y,
        max_y=overlap_max_y,
    )


def _route_cells_bbox(cells: Iterable[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    normalized = [(int(cell[0]), int(cell[1])) for cell in cells]
    if not normalized:
        return None
    xs = [cell[0] for cell in normalized]
    ys = [cell[1] for cell in normalized]
    return min(xs), max(xs), min(ys), max(ys)


def _format_cells_preview(cells: set[tuple[int, int]], *, limit: int = 120) -> str:
    if not cells:
        return "[]"
    ordered = sorted(cells)
    if len(ordered) <= limit:
        return str(ordered)
    head = ordered[:limit]
    return f"{head} ... (+{len(ordered) - limit} more)"
