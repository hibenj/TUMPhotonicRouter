"""Tests for translation/route_overview_svg.py (route-overview plan, 2026-10-05)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from translation.route_overview_svg import InstanceBox, write_routes_overview_svg


@dataclass(frozen=True)
class _Record:
    net_name: str
    total_length_um: float
    corrected_centerline_um: tuple[tuple[float, float], ...]


def test_overview_svg_labels_every_route_and_sums_the_legend(tmp_path: Path):
    records = [
        _Record("straight", 100.0, ((0.0, 0.0), (100.0, 0.0))),
        _Record("ell", 150.0, ((0.0, 50.0), (100.0, 50.0), (100.0, 100.0))),
        _Record("with_meander", 80.0, ((0.0, 200.0), (80.0, 200.0))),
        _Record("not_drawn", 10.0, ()),
    ]
    path = tmp_path / "routes" / "demo_overview.svg"

    summary = write_routes_overview_svg(
        path,
        benchmark_name="demo",
        records=records,
        realized_extra_um_by_net={"with_meander": 25.5},
        routable_bbox_um=(-10.0, -10.0, 210.0, 210.0),
        layout_bbox_um=(0.0, 0.0, 100.0, 200.0),
        instances=[InstanceBox("mmi_0", (-5.0, -2.0, 0.0, 2.0))],
        # one polygon strip per drawn route (the "with_meander" strip carries a bump)
        waveguide_polygons=[
            [(0.0, -0.25), (100.0, -0.25), (100.0, 0.25), (0.0, 0.25)],
            [
                (0.0, 49.75),
                (100.25, 49.75),
                (100.25, 100.0),
                (99.75, 100.0),
                (99.75, 50.25),
                (0.0, 50.25),
            ],
            [(0.0, 199.75), (80.0, 199.75), (80.0, 200.25), (0.0, 200.25)],
        ],
    )

    assert summary.route_count == 4
    assert summary.drawn_count == 3
    assert summary.routed_length_um == pytest.approx(340.0)
    assert summary.meander_length_um == pytest.approx(25.5)
    assert summary.total_length_um == pytest.approx(365.5)

    text = path.read_text(encoding="utf-8")
    assert text.startswith("<svg")
    assert "100.0 um" in text
    assert "150.0 um" in text
    assert "105.5 um (+25.5 meander)" in text
    assert text.count("<polygon") == 3
    assert text.count("<polyline") == 0
    # the polygon of the meander route is colored as such and titled with its net
    assert '<polygon points="' in text and "<title>with_meander</title>" in text
    assert "<title>mmi_0</title>" in text
    # the legend band sits below the drawing: its rect starts past every route point
    legend_band = text.split('<g id="legend"')[1]
    legend_y = float(legend_band.split('<rect x="0" y="')[1].split('"')[0])
    drawing_ys = [
        float(token.split(",")[1])
        for token in text.split('<polygon points="')[1].split('"')[0].split()
    ]
    assert legend_y > max(drawing_ys)
    for line in summary.legend_lines:
        assert line in text
    assert "routes: 4 (1 not drawn)" in text
    assert "chip (routable die area): 220.0 x 220.0 um" in text
    assert "layout bounding box: 100.0 x 200.0 um" in text
    assert "demo" in text
