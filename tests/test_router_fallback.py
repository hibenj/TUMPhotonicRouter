"""Contribution 2 router fallback: a crossing layer without a feasible
pre-placed structure keeps its nets and is routed by the guided crossing
search while the other layers keep their tiles; tiles plus router-found
crossings equal the plan."""

from __future__ import annotations

import json
from pathlib import Path

from routing_flow import run_routing_flow


def test_mixed_run_routes_a_forced_fallback_layer_with_the_guided_search(monkeypatch):
    monkeypatch.setenv("PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS", "6")
    run_routing_flow(
        "multiportmmi_8x8",
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        enable_path_length_matching=False,
        path_length_match_outputs=False,
        enable_crossings=False,
        crossing_mode="lidar-pure",
        preplaced_crossing_grids=True,
        fanout_access_mode="static-stubs",
        foreign_port_keepout_cells=0,
        proactive_congestion_weight=4.0,
        proactive_congestion_radius_cells=3,
    )
    photonic = json.loads(
        Path("build/verification/multiportmmi_8x8_photonic_verification.json").read_text()
    )
    crossing = json.loads(
        Path("build/verification/multiportmmi_8x8_crossing_verification.json").read_text()
    )
    assert photonic["error_count"] == 0 and crossing["error_count"] == 0
    grids = photonic["metrics"]["preplaced_crossing_grids"]
    assert grids["router_fallback_net_count"] == 8  # the eight lanes of layer 6
    tiles = grids["crossing_component_count"]
    realized = crossing["metrics"]["crossing_count"]
    assert tiles + realized == grids["plan_event_count"] == 33
    assert realized == 7  # layer 6's planned crossings, found by the router
