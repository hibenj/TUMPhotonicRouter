"""The path-length bookkeeping against the routed GDS (2026-10-05, owner's question).

Every routed waveguide is one merged polygon strip in the routed layout's
top cell (the benchmark components are instances), so a net's realized
length is its strip's area on the waveguide layer divided by the 0.5 um
width -- exact for arcs. The meander report books a meander on its
requiring edge but may draw it on a delay-insertion candidate upstream;
``realized_extra_length_um_by_edge`` is the bookkeeping by physical edge.
This test pins that the GDS agrees with that bookkeeping net by net, and
that the realized arrivals balance every gate and every output coupler.
"""

from __future__ import annotations

import contextlib
import io

import pytest
from photonic_router import flow as routing_flow
from photonic_router.cli import main

WAVEGUIDE_WIDTH_UM = 0.5
PER_NET_TOLERANCE_UM = 1.0  # polygonized arcs against the area measure
GATE_TOLERANCE_UM = 0.5

pytestmark = [pytest.mark.integration, pytest.mark.e2e]


def _route(benchmark_name: str, monkeypatch):
    captured = {}
    original = routing_flow.run_photonic_routing_stage

    def capture(**kwargs):
        result = original(**kwargs)
        captured["optical"] = result
        return result

    monkeypatch.setattr(routing_flow, "run_photonic_routing_stage", capture)
    with contextlib.redirect_stdout(io.StringIO()):
        layout = main([benchmark_name])
    return layout, captured["optical"]


def _strip_lengths_by_net(gds_path: str, records) -> dict[str, float]:
    kdb = pytest.importorskip("klayout.db")
    layout = kdb.Layout()
    layout.read(gds_path)
    top = next(cell for cell in layout.each_cell() if cell.is_top())
    dbu = layout.dbu
    waveguide = next(
        index for index in layout.layer_indexes() if str(layout.get_info(index)) == "1/0"
    )
    region = kdb.Region(top.shapes(waveguide))
    region.merge()
    polygons = list(region.each())
    lengths: dict[str, float] = {}
    for record in records:
        points = record.corrected_centerline_um
        if not points:
            continue
        for fraction in (0.1, 0.3, 0.5, 0.7, 0.9):
            x, y = points[int(len(points) * fraction)]
            point = kdb.Point(round(x / dbu), round(y / dbu))
            polygon = next((polygon for polygon in polygons if polygon.inside(point)), None)
            if polygon is not None:
                lengths[record.net_name] = polygon.area() * dbu * dbu / WAVEGUIDE_WIDTH_UM
                break
    return lengths


@pytest.mark.parametrize("benchmark_name", ["heater_s_mod", "cascade_12_compact"])
def test_realized_bookkeeping_matches_the_routed_gds(benchmark_name, monkeypatch):
    layout, optical = _route(benchmark_name, monkeypatch)
    records = optical.debug_artifacts.routed_net_records
    analysis_info = layout.info["path_length_analysis"]
    report = layout.info["meander_insertion_report"]

    realized = report["realized_extra_length_um_by_edge"]
    assert realized
    assert sum(entry["extra_length_um"] for entry in realized) == pytest.approx(
        report["total_physical_inserted_extra_length_um"], abs=1e-6
    )
    realized_by_net = {entry["edge"]["net_name"]: entry["extra_length_um"] for entry in realized}

    gds_lengths = _strip_lengths_by_net(f"build/routed_{benchmark_name}.gds", records)
    assert len(gds_lengths) == len(records)
    worst = 0.0
    for record in records:
        bookkept = float(record.total_length_um) + realized_by_net.get(record.net_name, 0.0)
        worst = max(worst, abs(gds_lengths[record.net_name] - bookkept))
    assert worst <= PER_NET_TOLERANCE_UM

    groups = analysis_info["matching_group_diagnostics"]
    assert groups
    assert all(group["realized_within_tolerance"] for group in groups)
    outputs = analysis_info["output_matching_diagnostics"]
    assert outputs and outputs[0]["realized_within_tolerance"]
    assert analysis_info["path_length_acceptance"]["passed"]
