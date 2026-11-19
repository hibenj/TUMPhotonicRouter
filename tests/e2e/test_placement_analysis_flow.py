"""The flow analyses a placed design once and only for the stages that read the analysis."""

from __future__ import annotations

import pytest
from photonic_router import cli, flow
from photonic_router.crossings import crossing_demand, route_rust_crossing_plan


@pytest.mark.parametrize(
    ("configuration", "analyses"),
    [
        # the search discovers its crossings, no plan exists
        ("baseline", 0),
        # the guided session reads the flow's analysis
        ("contribution1", 1),
        # the tiles are placed from it; the derived design routes
        # crossing-free and needs none
        ("contribution2", 1),
    ],
)
def test_one_analysis_per_design_that_needs_one(configuration, analyses, monkeypatch) -> None:
    calls = {"flow": 0, "session": 0, "demand": 0}

    def counted(module, name: str, key: str) -> None:
        original = getattr(module, name)

        def wrapper(*args, **kwargs):
            calls[key] += 1
            return original(*args, **kwargs)

        monkeypatch.setattr(module, name, wrapper)

    counted(crossing_demand, "build_crossing_demand", "demand")
    counted(flow, "analyze_placement", "flow")
    counted(route_rust_crossing_plan, "analyze_placement", "session")

    routed = cli.main(["benes_4x4_flat", "--configuration", configuration])

    assert routed.info["photonic_verification"]["error_count"] == 0
    assert calls == {"flow": analyses, "session": 0, "demand": 0}
