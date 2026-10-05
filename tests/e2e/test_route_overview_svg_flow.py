"""The routes overview SVG the flow writes under ``--debug-svgs``.

Route-overview plan, 2026-10-05.
"""

from __future__ import annotations

import contextlib
import io
from pathlib import Path

import pytest
from photonic_router.cli import main

pytestmark = [pytest.mark.integration, pytest.mark.e2e]


def test_flow_writes_the_routes_overview_with_realized_lengths_and_a_legend(monkeypatch):
    import webbrowser

    opened: list[str] = []
    monkeypatch.setattr(webbrowser, "open_new_tab", lambda url: opened.append(url) or True)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        layout = main(["heater_s_mod", "--debug-svgs", "1"])
    overview = Path("build/routes/heater_s_mod_overview.svg")
    assert overview.is_file()
    text = overview.read_text(encoding="utf-8")
    assert text.count("<polygon") == 81  # the routed waveguide polygons, meanders included
    assert text.count('<g id="instances"') == 1 and "<title>heater_extra_0</title>" in text
    report = layout.info["meander_insertion_report"]
    meander_total = float(report["total_physical_inserted_extra_length_um"])
    assert f"meander length: {meander_total:.1f} um" in text
    assert "routes: 81" in text
    assert "chip (routable die area):" in text
    assert "layout bounding box:" in text
    # a net whose meander was drawn on it carries the realized length, not
    # the booking: mmi_extra_1 -> mmi_extra_10 carries the 3,012.6 um of the
    # two output requirements booked on mmi_extra_14's routes
    realized = {
        entry["edge"]["net_name"]: entry["extra_length_um"]
        for entry in report["realized_extra_length_um_by_edge"]
    }
    carrier = "mmi_extra_1_upper_to_mmi_extra_10_lower_in"
    assert realized[carrier] > 3000.0
    assert f"(+{realized[carrier]:.1f} meander)" in text
    # the per-route SVG of route 1 carries its name and routed length
    route_svgs = sorted(Path("build/routes").glob("heater_s_mod_gc_in_0_to_mmi_a_0_upper_in*.svg"))
    assert route_svgs
    assert 'id="route-length-label"' in route_svgs[0].read_text(encoding="utf-8")
    printed = out.getvalue()
    assert "Routes overview SVG: build/routes/heater_s_mod_overview.svg" in printed
    # the run's SUMMARY line, with the counters the stages filled in
    summary = next(line for line in printed.splitlines() if line.startswith("SUMMARY "))
    assert "benchmark=heater_s_mod nets=81 " in summary
    assert "verifier_errors=0 verifier_warnings=0 plm_groups=26 plm_over_tolerance=0" in summary
    assert (
        "electrical_heaters=21 electrical_pads=22 electrical_failed_wires=0 electrical_ok=yes"
        in (summary)
    )
    # the overview is not a selected per-route SVG: the report counts one
    # selected route file and the browser is asked for the obstacle and
    # route SVGs only (stubbed above)
    assert "Route SVGs: 1 selected file(s)" in printed
    assert opened and not any(url.endswith("_overview.svg") for url in opened)
