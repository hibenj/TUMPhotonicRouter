"""Tests for translation/route_overview_svg.py (route-overview plan, 2026-10-05)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from translation.route_overview_svg import (
    MEANDER_ROUTE_COLOR,
    ROUTE_COLOR,
    InstanceBox,
    realized_extra_um_by_net,
    write_routes_overview_svg,
)


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
    # every polygon is owned by its route and colored by it: the meander
    # route's strip in MEANDER_ROUTE_COLOR with its net as the title, the
    # others in ROUTE_COLOR
    polygons = [part.split("</polygon>")[0] for part in text.split("<polygon ")[1:]]
    meander_polygons = [p for p in polygons if "<title>with_meander</title>" in p]
    assert len(meander_polygons) == 1 and f'fill="{MEANDER_ROUTE_COLOR}"' in meander_polygons[0]
    assert all(f'fill="{ROUTE_COLOR}"' in p for p in polygons if p not in meander_polygons)
    assert {p.split("<title>")[1].split("</title>")[0] for p in polygons} == {
        "straight",
        "ell",
        "with_meander",
    }
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


def test_overview_svg_owns_every_piece_of_a_split_route(tmp_path: Path):
    """A route clipped by a crossing is realized as several polygons; each
    piece is owned by the route and colored by it (merge review 2026-10-05).
    """

    records = [_Record("split_meander", 100.0, ((0.0, 0.0), (100.0, 0.0)))]
    pieces = [
        [(0.0, -0.25), (40.0, -0.25), (40.0, 0.25), (0.0, 0.25)],
        [(60.0, -0.25), (100.0, -0.25), (100.0, 0.25), (60.0, 0.25)],
    ]
    path = tmp_path / "split_overview.svg"

    write_routes_overview_svg(
        path,
        benchmark_name="split",
        records=records,
        realized_extra_um_by_net={"split_meander": 10.0},
        routable_bbox_um=None,
        layout_bbox_um=None,
        waveguide_polygons=pieces,
    )

    text = path.read_text(encoding="utf-8")
    polygons = [part.split("</polygon>")[0] for part in text.split("<polygon ")[1:]]
    assert len(polygons) == 2
    assert all("<title>split_meander</title>" in p for p in polygons)
    assert all(f'fill="{MEANDER_ROUTE_COLOR}"' in p for p in polygons)


def test_overview_svg_falls_back_to_centerlines_without_layout_shapes(tmp_path: Path):
    records = [_Record("only", 50.0, ((0.0, 0.0), (50.0, 0.0)))]
    path = tmp_path / "centerline_overview.svg"

    summary = write_routes_overview_svg(
        path,
        benchmark_name="bare",
        records=records,
        realized_extra_um_by_net={},
        routable_bbox_um=None,
        layout_bbox_um=None,
    )

    text = path.read_text(encoding="utf-8")
    assert summary.drawn_count == 1
    assert text.count("<polyline") == 1 and text.count("<polygon") == 0
    assert "50.0 um" in text

    # and an empty picture still writes a valid SVG
    empty = write_routes_overview_svg(
        tmp_path / "empty.svg",
        benchmark_name="empty",
        records=[],
        realized_extra_um_by_net={},
        routable_bbox_um=None,
        layout_bbox_um=None,
    )
    assert empty.route_count == 0
    assert (tmp_path / "empty.svg").read_text(encoding="utf-8").startswith("<svg")


def test_realized_extra_by_net_reads_the_physical_table_or_falls_back_to_the_booking():
    edge = {"net_name": "n", "source": {}, "target": {}}
    with_table = {
        "results": [{"edge": edge, "inserted_extra_length_um": 99.0}],
        "realized_extra_length_um_by_edge": [{"edge": edge, "extra_length_um": 12.5}],
    }
    assert realized_extra_um_by_net(with_table) == {"n": 12.5}
    booked_only = {"results": [{"edge": edge, "inserted_extra_length_um": 99.0}]}
    assert realized_extra_um_by_net(booked_only) == {"n": 99.0}
    assert realized_extra_um_by_net(None) == {}
