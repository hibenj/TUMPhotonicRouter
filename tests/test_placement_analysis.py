"""`photonic_router.placement_analysis`: one analysis per placed design, read by the stages below."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from photonic_router.crossings import route_rust_crossing_plan
from photonic_router.crossings.route_rust_crossing_plan import (
    CrossingPlanRequest,
    build_crossing_plan_info,
)
from photonic_router.placement_analysis import PlacementAnalysis, analyze_placement
from photonic_router.records.layout_from_schematic import layout_from_schematic

from benchmarks.benes_8x8 import build_schematic


class _Router:
    """The two router calls the crossing plan makes, recorded."""

    def __init__(self) -> None:
        self.guidance_pairs: object = None

    def set_crossing_config(self, config: object) -> None:
        pass

    def set_crossing_guidance(self, pairs: object, loss: float, single_per_pair: bool) -> None:
        self.guidance_pairs = pairs


_BACKEND = SimpleNamespace(CrossingConfig=lambda **kwargs: kwargs)


def _plan_info(crossing_mode: str, analysis: PlacementAnalysis | None, design: tuple) -> dict:
    schematic, layout = design
    return build_crossing_plan_info(
        rust_backend=_BACKEND,
        router=_Router(),
        schematic=schematic,
        unrouted_layout=layout,
        route_jobs=[],
        placement_analysis=analysis,
        request=CrossingPlanRequest(
            enable_crossings=True,
            crossing_mode=crossing_mode,
            crossing_loss=0.0,
            crossing_search_loss=1.0,
            crossing_half_size_cells=3,
        ),
    )


@pytest.fixture(scope="module")
def design() -> tuple:
    schematic = build_schematic()
    return schematic, layout_from_schematic(schematic)


def test_the_analysis_holds_the_region_graph_and_the_crossing_demand(design) -> None:
    analysis = analyze_placement(*design)
    assert analysis.regions is analysis.crossing_demand.regions
    assert analysis.crossing_demand.lower_bound() == 16
    assert analysis.summary() == (
        "7 column(s), 6 band(s), 39 passage(s); 16 planned crossing(s), "
        "0 net(s) outside the band model"
    )


def test_the_guided_session_reads_the_flows_analysis_and_does_not_analyse_again(
    design, monkeypatch
) -> None:
    analysis = analyze_placement(*design)

    def refuse(*_args: object) -> None:
        raise AssertionError("the session analysed a design the flow had analysed")

    monkeypatch.setattr(route_rust_crossing_plan, "analyze_placement", refuse)
    info = _plan_info("lidar-guided", analysis, (SimpleNamespace(), SimpleNamespace()))
    assert info["event_count"] == 16
    assert info["plan_text"] == analysis.crossing_demand.to_text()


def test_a_session_entered_without_an_analysis_analyses_its_own_design(design) -> None:
    info = _plan_info("lidar-guided", None, design)
    assert info["event_count"] == 16


def test_the_baseline_never_reads_an_analysis(design, monkeypatch) -> None:
    def refuse(*_args: object) -> None:
        raise AssertionError("lidar-pure must not be informed by a plan")

    monkeypatch.setattr(route_rust_crossing_plan, "analyze_placement", refuse)
    info = _plan_info("lidar-pure", analyze_placement(*design), design)
    assert info["reason"] == "lidar_pure_mode_ignores_topology_plan"
    assert info["event_count"] == 0 and "plan_text" not in info
