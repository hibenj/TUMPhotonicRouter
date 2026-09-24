"""`RouterConfig.search.engine`, the Rust-side `NetSearch` selection
(Milestone 3 Slice 4 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`): the
second engine routes a whole benchmark end to end, and an unknown engine
name is refused where the configuration crosses into Rust."""

from __future__ import annotations

import dataclasses

import pytest

from photonic_router.config import RouterConfig, RoutingConfig, SearchOverrides
from routing_flow import run_routing_flow


def _config_with_engine(engine: str) -> RoutingConfig:
    base = RoutingConfig()
    return dataclasses.replace(
        base,
        router=dataclasses.replace(
            base.router, search=dataclasses.replace(base.router.search, engine=engine)
        ),
    )


def test_grid_dijkstra_engine_routes_benes_4x4_flat_end_to_end():
    """Contribution 2 (pre-placed crossing structures) routes every net
    crossing-free, so the Dijkstra engine -- which serves no crossing
    request -- sees only requests it can answer, and the flow's own
    verifier has to accept the result."""
    routed = run_routing_flow(
        "benes_4x4_flat",
        config=_config_with_engine("grid-dijkstra"),
        preplaced_crossing_grids=True,
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        enable_crossings=False,
        crossing_mode="lidar-pure",
        enable_path_length_matching=False,
        path_length_match_outputs=False,
    )

    photonic = routed.info["photonic_verification"]
    assert photonic["error_count"] == 0
    grids = photonic["metrics"]["preplaced_crossing_grids"]
    assert grids["plan_event_count"] == 2
    assert grids["crossing_component_count"] == 2
    assert grids["router_fallback_net_count"] == 0


def test_unknown_search_engine_is_rejected_naming_both_valid_values():
    import photonic_router._rust as rust_backend

    cfg = RouterConfig(search=SearchOverrides(engine="nonsense"))
    with pytest.raises(ValueError) as excinfo:
        cfg.to_rust(rust_backend)
    message = str(excinfo.value)
    assert "astar" in message
    assert "grid-dijkstra" in message
    assert "nonsense" in message
