from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.static_obstacle_builder import _load_rust_backend


def test_rust_backend_exposes_router_class():
    backend = _load_rust_backend()
    assert backend is not None
    assert hasattr(backend, "PyPhotonicRouter")
    assert hasattr(backend, "GridSpec")
    assert hasattr(backend, "CrossingConfig")
    assert hasattr(backend, "build_static_obstacle_map_rs")


def test_astar_config_exposes_proactive_congestion_fields():
    backend = _load_rust_backend()
    assert backend is not None
    cfg = backend.AStarConfig()

    assert cfg.proactive_congestion_weight == 0.0
    assert cfg.proactive_congestion_radius_cells == 0
    cfg.proactive_congestion_weight = 2.5
    cfg.proactive_congestion_radius_cells = 4
    assert cfg.proactive_congestion_weight == 2.5
    assert cfg.proactive_congestion_radius_cells == 4


def test_router_crossing_context_is_disabled_by_default():
    backend = _load_rust_backend()
    assert backend is not None
    router = backend.PyPhotonicRouter(
        backend.GridSpec(20, 20, 1.0, 0.0, 0.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(),
    )

    cfg = router.crossing_config()
    assert cfg.enabled is False
    assert cfg.crossing_half_size_cells == 0


def _collision_crossing_router(
    *, crossing_half_size_cells: int = 0, unified_legality: bool = False
):
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        straight_short_cells=1,
        straight_long_cells=4,
        bend_radius_cells=1,
        allow_45_degree_turns=False,
    )
    router = backend.PyPhotonicRouter(
        backend.GridSpec(20, 14, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(
            max_iterations=10_000,
            require_target_angle=False,
            use_routing_window=False,
            heuristic_mode="distance",
        ),
        backend.RouterConfig(crossing_unified_legality=unified_legality),
    )
    router.set_crossing_config(
        backend.CrossingConfig(
            enabled=True,
            crossing_loss=3.0,
            crossing_half_size_cells=crossing_half_size_cells,
        )
    )
    return backend, router


def test_collision_crossing_route_uses_local_dynamic_intersection():
    backend, router = _collision_crossing_router()

    router.route_single_net_and_commit(
        1,
        backend.State(8, 2, 2),
        backend.State(8, 10, 2),
        block_radius_cells=0,
    )
    router.add_static_cells([(x, 5) for x in range(3, 14)] + [(x, 7) for x in range(3, 14)])

    route = router.route_single_net_and_commit(
        2,
        backend.State(2, 6, 0),
        backend.State(14, 6, 0),
        block_radius_cells=0,
    )

    net1_core = {tuple(cell) for cell in router.get_net_core_cells(1)}
    net2_core = {tuple(cell) for cell in router.get_net_core_cells(2)}
    assert net1_core & net2_core == {(8, 6)}
    assert route.compressed_waypoints == [(2, 6), (14, 6)]
    events = router.crossing_events()
    assert len(events) == 1
    assert events[0]["net_id"] == 2
    assert events[0]["partner_net_id"] == 1
    assert tuple(events[0]["point"]) == (8.5, 6.5)
    assert tuple(events[0]["route_segment"]) == ((2, 6), (14, 6))
    assert tuple(events[0]["partner_segment"]) == ((8, 2), (8, 10))


def test_unified_legality_verdicts_match_the_events_and_the_python_verifier():
    """Phase 4 slice B: with `crossing_unified_legality` the router records
    one verdict per realized crossing, and the Python verifier's
    classification is that verdict, not a re-decision."""
    from types import SimpleNamespace

    from photonic_router.crossings.route_rust_crossing_verification import (
        _populate_realized_intersections_from_native_crossing_events,
        _verify_realized_route_intersections,
    )
    from photonic_router.plm.path_length_graph import PortRef
    from photonic_router.records.route_rust_types import RoutedNetRecord
    from photonic_router.routing.crossing_plan_info import CrossingPlanInfo

    backend, router = _collision_crossing_router(crossing_half_size_cells=1, unified_legality=True)
    assert router.crossing_verdicts() == []

    router.route_single_net_and_commit(
        1,
        backend.State(8, 2, 2),
        backend.State(8, 10, 2),
        block_radius_cells=0,
    )
    router.add_static_cells([(x, 4) for x in range(3, 14)] + [(x, 8) for x in range(3, 14)])
    router.route_single_net_and_commit(
        2,
        backend.State(2, 6, 0),
        backend.State(14, 6, 0),
        block_radius_cells=0,
    )

    events = router.crossing_events()
    verdicts = router.crossing_verdicts()
    assert len(events) == 1
    assert len(verdicts) == 1
    verdict = verdicts[0]
    assert (verdict["net_id"], verdict["partner_net_id"]) == (2, 1)
    assert verdict["legal"] is True
    assert verdict["reason"] == "legal"
    assert verdict["perpendicular"] is True
    assert verdict["event_registered"] is True
    assert verdict["footprint_blocked"] is False
    assert verdict["margins_from_grid"] is True
    # half_size 1 + bend_radius 1: one margin of two cells, one cell after.
    assert verdict["required_margin_cells"] == 2
    assert verdict["required_after_cells"] == 1
    assert tuple(verdict["point"]) == tuple(events[0]["point"])
    assert tuple(verdict["route_segment"]) == tuple(events[0]["route_segment"])
    assert tuple(verdict["partner_segment"]) == tuple(events[0]["partner_segment"])
    assert verdict["route_before_cells"] == 6.0  # (2,6)->(14,6) crosses x = 8
    assert verdict["partner_before_cells"] == 4.0  # (8,2)->(8,10) crosses y = 6

    info = CrossingPlanInfo.from_dict(
        {"enabled": True, "crossing_half_size_cells": 1, "events": []}
    )
    info.bend_runout_cells_per_crossing = 1
    grid_spec = (20, 14, 1.0, 0.0, 0.0)
    illegal = _populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=info,
        routed_records_by_net_id={},
        native_crossing_events=events,
        realization_grid_spec=grid_spec,
        native_crossing_verdicts=verdicts,
        unified_legality=True,
    )
    assert illegal == []
    native = info.realized_intersections[0]
    assert native["classification"] == "legal_native_crossing"
    assert native["legality_source"] == "rust_crossing_verdict"
    assert native["required_margin_um"] == 2.0
    assert native["rust_verdict"]["reason"] == "legal"

    def _record(name: str, centerline) -> RoutedNetRecord:
        return RoutedNetRecord(
            net_name=name,
            source=PortRef(instance=f"{name}_src", port="o1"),
            target=PortRef(instance=f"{name}_dst", port="o1"),
            route_obj=SimpleNamespace(),
            total_length_um=0.0,
            corrected_centerline_um=centerline,
        )

    # The realized centerlines run through the cell centers.
    full_info = CrossingPlanInfo.from_dict(
        {"enabled": True, "crossing_half_size_cells": 1, "events": []}
    )
    full_info.bend_runout_cells_per_crossing = 1
    full_info.crossing_mode = "lidar-pure"
    illegal = _verify_realized_route_intersections(
        crossing_plan_info=full_info,
        routed_records_by_net_id={
            1: _record("a", ((8.5, 2.5), (8.5, 10.5))),
            2: _record("b", ((2.5, 6.5), (14.5, 6.5))),
        },
        realization_grid_spec=grid_spec,
        native_crossing_verdicts=verdicts,
        unified_legality=True,
    )
    assert illegal == []
    crossing = full_info.realized_intersections[0]
    assert crossing["classification"] == "legal_unexpected_crossing"
    assert crossing["legality_source"] == "rust_crossing_verdict"
    assert "reason" not in crossing
    assert crossing["required_margin_um"] == 2.0
    assert crossing["search_required_margin_um"] == 2.0

    # A rip-up drops the verdict with the event.
    assert router.ripup_route(2) is True
    assert router.crossing_events() == []
    assert router.crossing_verdicts() == []


def test_collision_crossing_route_reserves_crossing_footprint():
    backend, router = _collision_crossing_router(crossing_half_size_cells=2)

    router.route_single_net_and_commit(
        1,
        backend.State(8, 1, 2),
        backend.State(8, 11, 2),
        block_radius_cells=0,
    )
    router.add_static_cells([(x, 3) for x in range(3, 14)] + [(x, 9) for x in range(3, 14)])

    route = router.route_single_net_and_commit(
        2,
        backend.State(2, 6, 0),
        backend.State(14, 6, 0),
        block_radius_cells=0,
    )

    static_cells = {tuple(cell) for cell in router.raw_static_obstacle_cells()}
    expected_reserved = {(x, y) for x in range(6, 11) for y in range(4, 9)}
    assert expected_reserved <= static_cells
    events = router.crossing_events()
    assert len(events) == 1
    assert events[0]["net_id"] == 2
    assert events[0]["partner_net_id"] == 1
    assert tuple(events[0]["point"]) == (8.5, 6.5)
    assert expected_reserved <= {tuple(cell) for cell in events[0]["reservation_cells"]}
    svg = router.export_debug_svg(route)
    assert 'id="crossing-events"' in svg
    assert 'class="crossing-event"' in svg
    assert 'data-net-id="2"' in svg
    assert 'data-partner-net-id="1"' in svg
    assert 'fill="#8a8a8a"' in svg
    assert 'stroke="#d81b60"' not in svg
    assert 'stroke="#00acc1"' not in svg

    assert router.ripup_route(2) is True
    assert router.crossing_events() == []
    static_cells_after_ripup = {tuple(cell) for cell in router.raw_static_obstacle_cells()}
    assert expected_reserved.isdisjoint(static_cells_after_ripup)


def test_collision_crossing_route_rejects_invalid_local_crossing_move():
    backend, router = _collision_crossing_router()

    router.route_single_net_and_commit(
        1,
        backend.State(8, 6, 2),
        backend.State(8, 7, 2),
        block_radius_cells=0,
    )
    router.add_static_cells([(x, 5) for x in range(3, 14)] + [(x, 7) for x in range(3, 14)])

    with pytest.raises(RuntimeError, match="No legal LiDAR crossing route found"):
        router.route_single_net_and_commit(
            2,
            backend.State(2, 6, 0),
            backend.State(14, 6, 0),
            block_radius_cells=0,
        )

    assert router.get_net_core_cells(2) == []


def test_route_debug_svg_can_use_obstacle_snapshot():
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig()
    router = backend.PyPhotonicRouter(
        backend.GridSpec(32, 32, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(max_iterations=10_000),
    )

    first = router.route_single_net_and_commit(
        1,
        backend.State(3, 8, 0),
        backend.State(12, 8, 0),
        block_radius_cells=0,
    )
    router.route_single_net_and_commit(
        2,
        backend.State(3, 14, 0),
        backend.State(12, 14, 0),
        block_radius_cells=0,
    )
    router.add_port_open_cells([(4, 8)])

    final_svg = router.export_debug_svg(first)
    snapshot_svg = router.export_debug_svg_with_obstacle_cells(first, [])

    assert 'fill="#000000"' in final_svg
    assert 'fill="#000000"' not in snapshot_svg
    assert (
        '<rect class="port-access" x="4" y="23" width="1" height="1" '
        'fill="#d93025" opacity="0.38" />'
    ) in final_svg
    assert (
        '<rect class="port-access" x="4" y="23" width="1" height="1" '
        'fill="#d93025" opacity="0.38" />'
    ) in snapshot_svg
    assert '<rect x="3" y="23" width="1" height="1" fill="#1a73e8" />' in snapshot_svg
