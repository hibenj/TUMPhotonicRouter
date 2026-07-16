import math
from dataclasses import replace
from types import SimpleNamespace
from typing import Any, cast

import pytest

from photonic_router.path_length_graph import PortRef
from photonic_router.static_obstacle_builder import _load_rust_backend
from translation.route_rust import (
    _absorbed_terminal_centerline,
    _apply_crossing_aware_endpoint_correction_to_record,
)
from translation.route_rust_records import (
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
)
from translation.route_rust_types import RoutedNetRecord


def _centerline_length_um(centerline: tuple[tuple[float, float], ...]) -> float:
    return sum(
        math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        for p0, p1 in zip(centerline, centerline[1:])
    )


def _unit_from_orientation_deg(
    orientation_deg: float | None,
    *,
    as_target: bool,
) -> tuple[float, float] | None:
    if orientation_deg is None:
        return None
    angle_rad = math.radians(float(orientation_deg) + (180.0 if as_target else 0.0))
    return (math.cos(angle_rad), math.sin(angle_rad))


def _assert_segment_aligned_with_dir(
    p0: tuple[float, float],
    p1: tuple[float, float],
    direction: tuple[float, float],
) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.hypot(dx, dy)
    assert length > 0.0
    dot = dx * direction[0] + dy * direction[1]
    cross = dx * direction[1] - dy * direction[0]
    assert dot > 0.0
    assert abs(cross) <= 1.0e-6 * max(length, 1.0)


def _assert_record_uses_corrected_centerline(record: RoutedNetRecord) -> None:
    centerline = record.corrected_centerline_um
    assert len(centerline) >= 2
    assert record.endpoint_correction_error is None
    assert record.source_port_center_um is not None
    assert record.target_port_center_um is not None
    assert centerline[0] == pytest.approx(record.source_port_center_um)
    assert centerline[-1] == pytest.approx(record.target_port_center_um)
    assert record.total_length_um == pytest.approx(_centerline_length_um(centerline))

    source_dir = _unit_from_orientation_deg(
        record.source_port_orientation_deg,
        as_target=False,
    )
    if source_dir is not None:
        _assert_segment_aligned_with_dir(centerline[0], centerline[1], source_dir)

    target_dir = _unit_from_orientation_deg(
        record.target_port_orientation_deg,
        as_target=True,
    )
    if target_dir is not None:
        _assert_segment_aligned_with_dir(centerline[-2], centerline[-1], target_dir)


def test_port_alignment_diagnostics_reports_endpoint_mu_values():
    route_obj = SimpleNamespace(
        states=[
            SimpleNamespace(x=10, y=20),
            SimpleNamespace(x=30, y=40),
        ]
    )
    record = RoutedNetRecord(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o2"),
        route_obj=route_obj,
        total_length_um=12.5,
        source_port_center_um=(5.2, 10.4),
        target_port_center_um=(15.8, 20.1),
        source_port_orientation_deg=0.0,
        target_port_orientation_deg=180.0,
    )

    diagnostics = build_port_alignment_diagnostics(
        [record],
        realization_grid_spec=(100, 100, 0.5, 0.0, 0.0),
    )

    assert len(diagnostics) == 1
    entry = diagnostics[0]
    source = cast(dict[str, Any], entry["source"])
    target = cast(dict[str, Any], entry["target"])
    assert entry["net_name"] == "n0"
    assert entry["route_total_length_um"] == 12.5
    assert source["route_cell"] == [10, 20]
    assert source["route_grid_center_um"] == [5.25, 10.25]
    assert source["mu_x_um"] == pytest.approx(-0.05)
    assert source["mu_y_um"] == pytest.approx(0.15)
    assert target["route_cell"] == [30, 40]
    assert target["route_grid_center_um"] == [15.25, 20.25]
    assert target["mu_x_um"] == pytest.approx(0.55)
    assert target["mu_y_um"] == pytest.approx(-0.15)


def test_endpoint_correction_failure_prints_net_and_endpoints(capsys):
    class FailingRouter:
        def route_port_corrected_centerline(self, route, **kwargs):
            raise ValueError("port endpoint correction would require an unsupported terminal stub")

        def centerline_length_um(self, centerline):
            return 0.0

    route_obj = SimpleNamespace(
        states=[
            SimpleNamespace(x=10, y=20),
            SimpleNamespace(x=30, y=40),
        ]
    )
    record = RoutedNetRecord(
        net_name="bad_net",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o2"),
        route_obj=route_obj,
        total_length_um=12.5,
        source_port_center_um=(5.2, 10.4),
        target_port_center_um=(15.8, 20.1),
    )

    updated = apply_port_endpoint_corrections(
        [record],
        router=FailingRouter(),
        realization_grid_spec=(100, 100, 0.5, 0.0, 0.0),
    )

    assert len(updated) == 1
    assert updated[0].endpoint_correction_error is not None
    message = capsys.readouterr().out
    assert "ERROR:" in message
    assert "Grid-to-port endpoint correction failed" in message
    assert "bad_net" in message
    assert "src.o1 -> dst.o2" in message
    assert "source_port_um=(5.2, 10.4)" in message
    assert "target_port_um=(15.8, 20.1)" in message
    assert "source_route_cell=(10, 20)" in message
    assert "target_route_cell=(30, 40)" in message
    assert "source_route_center_um=(5.25, 10.25)" in message
    assert "target_route_center_um=(15.25, 20.25)" in message


def test_mmi_heater_pass0_characterizes_current_port_alignment():
    pytest.importorskip("gdsfactory")
    if _load_rust_backend() is None:
        pytest.skip("Rust backend unavailable for mmi_heater alignment diagnostics.")

    from benchmarks.mmi_heater import build_schematic
    from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
    from translation.layout_from_schematic import layout_from_schematic
    from translation.route_rust import route_nets_rust

    schematic = build_schematic()
    layout = layout_from_schematic(schematic)

    _, artifacts = route_nets_rust(
        layout,
        schematic,
        obstacle_config=StaticObstacleMapConfig(clearance_um=0.0),
        defer_realization=True,
        include_heater_obstacles=True,
        collect_route_stats=True,
        allow_45_degree_turns=False,
        enable_checked_endpoint_correction=False,
    )

    records_by_name = {record.net_name: record for record in artifacts.routed_net_records}
    first = records_by_name["gc0_to_mmi0_in1"]
    second = records_by_name["gc1_to_mmi0_in2"]
    assert first.base_total_length_um == pytest.approx(149.0)
    assert second.base_total_length_um == pytest.approx(149.5)
    assert first.total_length_um == pytest.approx(140.40331156954753)
    assert second.total_length_um == pytest.approx(140.9033115695475)
    assert (
        second.total_length_um - first.total_length_um
    ) == pytest.approx(0.5)
    assert len(first.corrected_centerline_um) >= 2
    assert len(second.corrected_centerline_um) >= 2
    assert first.endpoint_correction_error is None
    assert second.endpoint_correction_error is None

    diagnostics_by_name = {
        str(entry["net_name"]): entry
        for entry in artifacts.port_alignment_diagnostics
    }
    assert set(diagnostics_by_name) == set(records_by_name)
    for entry in diagnostics_by_name.values():
        max_offset = entry["max_endpoint_offset_abs_um"]
        assert isinstance(max_offset, (int, float))
        assert max_offset > 0.0
        for endpoint_name in ("source", "target"):
            endpoint = cast(dict[str, Any], entry[endpoint_name])
            assert endpoint["port_center_um"] is not None
            assert endpoint["route_grid_center_um"] is not None
            assert endpoint["mu_x_um"] is not None
            assert endpoint["mu_y_um"] is not None


def test_mmi_heater_route_match_uses_corrected_records_for_realization():
    pytest.importorskip("gdsfactory")
    if _load_rust_backend() is None:
        pytest.skip("Rust backend unavailable for mmi_heater endpoint correction.")

    from benchmark_metadata import resolve_internal_delays_for_instances
    from benchmarks.mmi_heater import INTERNAL_DELAYS_UM, NODE_TYPES, build_schematic
    from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
    from translation.layout_from_schematic import layout_from_schematic
    from translation.route_rust import route_match_and_realize

    schematic = build_schematic()
    layout = layout_from_schematic(schematic)
    result = route_match_and_realize(
        layout,
        schematic,
        enable_path_length_matching=False,
        node_types=NODE_TYPES,
        internal_delays_um=resolve_internal_delays_for_instances(
            schematic,
            INTERNAL_DELAYS_UM,
        ),
        obstacle_config=StaticObstacleMapConfig(clearance_um=0.0),
        include_heater_obstacles=True,
        collect_route_stats=True,
        allow_45_degree_turns=False,
        enable_grid_endpoint_correction=False,
    )

    records = result.debug_artifacts.routed_net_records
    assert records
    records_by_name = {record.net_name: record for record in records}
    assert any(record.corrected_centerline_um for record in records)
    assert any(record.base_total_length_um is not None for record in records)
    assert any(
        record.base_total_length_um is not None
        and abs(float(record.total_length_um) - float(record.base_total_length_um)) > 1.0e-6
        for record in records
    )
    for net_name in (
        "gc0_to_mmi0_in1",
        "gc1_to_mmi0_in2",
        "mmi1_out1_to_gc2",
        "mmi1_out2_to_gc3",
    ):
        record = records_by_name[net_name]
        assert len(record.corrected_centerline_um) >= 2
        assert record.endpoint_correction_error is None
    assert result.path_length_analysis_info is None
    assert result.meander_insertion_report_info is None


def test_py_router_exposes_endpoint_corrected_centerline_and_polygon():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    grid = rust_backend.GridSpec(64, 64, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=False,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)

    route = router.route_single_net(
        rust_backend.State(1, 2, 0),
        rust_backend.State(5, 6, 2),
    )
    centerline = router.route_port_corrected_centerline(
        route,
        source_port_um=(1.2, 2.1),
        target_port_um=(5.8, 6.4),
    )
    polygon = router.realize_route_polygon_with_endpoint_correction(
        route,
        0.5,
        source_port_um=(1.2, 2.1),
        target_port_um=(5.8, 6.4),
    )

    assert centerline[0] == pytest.approx((1.2, 2.1))
    assert centerline[-1] == pytest.approx((5.8, 6.4))
    assert centerline[1][1] == pytest.approx(centerline[0][1])
    assert centerline[-2][0] == pytest.approx(centerline[-1][0])
    assert polygon[0] == pytest.approx(polygon[-1])
    assert min(point[0] for point in polygon) <= 1.2 + 1.0e-9
    assert max(point[0] for point in polygon) >= 5.8 - 1.0e-9


def test_py_router_corrects_full_diagonal_45_degree_centerline():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    grid = rust_backend.GridSpec(64, 64, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=True,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)

    route = router.route_single_net(
        rust_backend.State(1, 1, 1),
        rust_backend.State(9, 9, 1),
    )
    centerline = router.route_port_corrected_centerline(
        route,
        source_port_um=(1.2, 1.2),
        target_port_um=(10.0, 10.0),
    )

    assert centerline[0] == pytest.approx((1.2, 1.2))
    assert centerline[-1] == pytest.approx((10.0, 10.0))
    for p0, p1 in zip(centerline, centerline[1:]):
        assert (p1[0] - p0[0]) == pytest.approx(p1[1] - p0[1])
        assert p1[0] > p0[0]


def test_py_router_safe_endpoint_correction_rejects_unchecked_bump():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    grid = rust_backend.GridSpec(64, 16, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=True,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    route = router.route_single_net(
        rust_backend.State(1, 2, 0),
        rust_backend.State(13, 2, 0),
    )

    with pytest.raises(ValueError):
        router.route_port_corrected_centerline(
            route,
            source_port_um=(1.0, 2.5),
            target_port_um=(13.5, 2.0),
            allow_unchecked_bumps=False,
        )


def _checked_case4_test_router_and_route(rust_backend, *, allow_45_degree_turns: bool = False):
    grid = rust_backend.GridSpec(40, 12, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)

    route = router.route_single_net_and_commit(
        7,
        rust_backend.State(1, 1, 0),
        rust_backend.State(21, 1, 0),
        0,
        [],
        0,
        [],
        0,
    )
    return router, route


def test_checked_case4_bump_allows_local_static_port_opening():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    router.add_static_cells([(2, 3), (3, 3)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        0,
        0,
        [(2, 3), (3, 3)],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 0


def test_checked_case4_bump_uses_clear_mirrored_side_when_static_blocks_first_candidate():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    router.add_static_cells([(2, 3), (3, 3)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        0,
        0,
        [],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
        allow_unchecked_fallback=False,
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]


def test_checked_case4_bump_rejects_static_without_active_port_opening():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    router.add_static_cells(
        [(x, y) for x in range(2, 22) for y in range(0, 4)]
    )

    with pytest.raises(RuntimeError, match="static_overlap"):
        router.route_port_corrected_centerline_checked_and_commit(
            7,
            route,
            0.5,
            0,
            0,
            [],
            [],
            source_port_um=(1.5, 1.5),
            target_port_um=(21.5, 2.0),
            allow_unchecked_fallback=False,
        )


def test_checked_case4_bump_skips_dynamic_blocked_start_top_candidate():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        0,
        0,
        [(2, 3), (3, 3)],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 1
    centerline = tuple((float(x), float(y)) for x, y in result["centerline"])
    assert centerline[0] == pytest.approx((1.5, 1.5))
    assert centerline[-1] == pytest.approx((21.5, 2.0))


def test_checked_case4_bump_allows_dynamic_only_in_inflated_footprint():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 4), (3, 4)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        1,
        1,
        [],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 0
    assert "top" in result["candidate_label"]


def test_checked_case4_bump_rejects_dynamic_in_core_footprint():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        1,
        1,
        [],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]


def test_batch_checked_endpoint_correction_returns_per_net_metadata():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])

    results = router.apply_checked_endpoint_corrections_and_commit(
        [
            (
                7,
                route,
                [(2, 3), (3, 3)],
                [],
                (1.5, 1.5),
                (21.5, 2.0),
            )
        ],
        0.5,
        0,
        0,
        False,
    )

    assert len(results) == 1
    result = dict(results[0])
    assert result["net_id"] == 7
    assert result["error"] is None
    assert result["committed_bump"] is True
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]
    assert result["total_length_um"] > 0.0
    centerline = tuple((float(x), float(y)) for x, y in result["centerline"])
    assert centerline[0] == pytest.approx((1.5, 1.5))
    assert centerline[-1] == pytest.approx((21.5, 2.0))


def test_checked_endpoint_correction_dry_run_does_not_mutate_router_state():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])
    before_cells = {tuple(cell) for cell in router.get_net_cells(7)}
    before_core_cells = {tuple(cell) for cell in router.get_net_core_cells(7)}
    before_blocker_cells = {tuple(cell) for cell in router.get_net_cells(8)}

    result = router.route_port_corrected_centerline_checked(
        7,
        route,
        0.5,
        0,
        0,
        [(2, 3), (3, 3)],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
        allow_unchecked_fallback=False,
    )

    assert result["committed_bump"] is False
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]
    centerline = tuple((float(x), float(y)) for x, y in result["centerline"])
    assert centerline[0] == pytest.approx((1.5, 1.5))
    assert centerline[-1] == pytest.approx((21.5, 2.0))
    assert {tuple(cell) for cell in router.get_net_cells(7)} == before_cells
    assert {tuple(cell) for cell in router.get_net_core_cells(7)} == before_core_cells
    assert {tuple(cell) for cell in router.get_net_cells(8)} == before_blocker_cells


def test_batch_checked_endpoint_correction_dry_run_returns_metadata_without_commit():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(rust_backend)
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])
    before_cells = {tuple(cell) for cell in router.get_net_cells(7)}

    results = router.apply_checked_endpoint_corrections(
        [
            (
                7,
                route,
                [(2, 3), (3, 3)],
                [],
                (1.5, 1.5),
                (21.5, 2.0),
            )
        ],
        0.5,
        0,
        0,
        False,
    )

    assert len(results) == 1
    result = dict(results[0])
    assert result["net_id"] == 7
    assert result["error"] is None
    assert result["committed_bump"] is False
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]
    assert result["total_length_um"] > 0.0
    assert {tuple(cell) for cell in router.get_net_cells(7)} == before_cells


def test_checked_case4_bump_with_45_enabled_uses_clear_mirrored_side():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_case4_test_router_and_route(
        rust_backend,
        allow_45_degree_turns=True,
    )
    assert router.commit_route_cells(8, [(2, 3), (3, 3)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        0,
        0,
        [(2, 3), (3, 3)],
        [],
        source_port_um=(1.5, 1.5),
        target_port_um=(21.5, 2.0),
        allow_unchecked_fallback=False,
    )

    assert result["committed_bump"] is True
    assert result["candidate_index"] == 1
    assert "bottom" in result["candidate_label"]


def _checked_no_bump_diagonal_test_router_and_route(rust_backend):
    grid = rust_backend.GridSpec(32, 32, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=True,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    route = router.route_single_net_and_commit(
        7,
        rust_backend.State(1, 1, 1),
        rust_backend.State(9, 9, 1),
        0,
        [],
        0,
        [],
        0,
    )
    return router, route


class _FakeEndpointRouter:
    def route_primitive_centerline(self, route: object) -> list[tuple[float, float]]:
        return [(1.0, 0.0), (4.0, 0.0), (5.0, 0.0), (6.0, 0.0), (10.0, 0.0)]

    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: object,
    ) -> list[tuple[float, float]]:
        return [(0.0, 1.0), (2.0, 1.0), (4.0, 1.0), (6.0, 1.0), (9.0, 1.0), (11.0, 1.0)]

    def centerline_length_um(self, centerline: list[tuple[float, float]]) -> float:
        return _centerline_length_um(tuple(centerline))


class _FakeStraightEndpointRouter(_FakeEndpointRouter):
    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: object,
    ) -> list[tuple[float, float]]:
        return [(0.0, 0.0), (4.0, 0.0), (5.0, 0.0), (6.0, 0.0), (11.0, 0.0)]


class _FakePortStubEndpointRouter(_FakeEndpointRouter):
    def route_primitive_centerline(self, route: object) -> list[tuple[float, float]]:
        return [(0.0, 1.0), (0.0, 4.0), (0.0, 5.0), (0.0, 6.0), (0.0, 10.0)]

    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: object,
    ) -> list[tuple[float, float]]:
        return [(1.0, 1.0), (0.0, 1.0), (0.0, 4.0), (0.0, 5.0), (0.0, 6.0), (0.0, 11.0)]


class _FakeSplitEndpointRouter(_FakeEndpointRouter):
    def route_primitive_centerline(self, route: object) -> list[tuple[float, float]]:
        return [(1.0, 0.0), (9.0, 0.0), (20.0, 0.0)]

    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: object,
    ) -> list[tuple[float, float]]:
        return [(0.0, 1.0), (9.0, 1.0), (21.0, 1.0)]


class _FakeUnanchoredEndpointRouter(_FakeSplitEndpointRouter):
    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: object,
    ) -> list[tuple[float, float]]:
        return [(1.0, 0.0), (9.0, 0.0), (20.0, 0.0)]


def _simple_record() -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name="n_crossed",
        source=PortRef("src", "o1"),
        target=PortRef("dst", "o1"),
        route_obj=object(),
        total_length_um=9.0,
        net_id=7,
        source_port_center_um=(0.0, 0.0),
        target_port_center_um=(11.0, 0.0),
    )


def _split_record() -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name="n_split",
        source=PortRef("src", "o1"),
        target=PortRef("dst", "o1"),
        route_obj=object(),
        total_length_um=19.0,
        net_id=9,
        source_port_center_um=(0.0, 0.0),
        target_port_center_um=(21.0, 0.0),
    )


def _port_stub_record() -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name="n_stub",
        source=PortRef("src", "o1"),
        target=PortRef("dst", "o1"),
        route_obj=object(),
        total_length_um=9.0,
        net_id=8,
        source_port_center_um=(1.0, 1.0),
        target_port_center_um=(0.0, 11.0),
        source_port_orientation_deg=180.0,
        target_port_orientation_deg=90.0,
    )


def test_crossing_aware_endpoint_correction_rejects_inserted_mid_access_geometry():
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        _simple_record(),
        router=_FakeEndpointRouter(),
        crossing_points=[(4.5, 0.0), (5.5, 0.0)],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.endpoint_correction_error is None
    assert updated.corrected_centerline_um == (
        (0.0, 0.0),
        (4.0, 0.0),
        (5.0, 0.0),
        (6.0, 0.0),
        (11.0, 0.0),
    )
    assert updated.base_total_length_um == 9.0


def test_crossing_aware_endpoint_correction_absorbs_both_split_terminal_sides():
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        _split_record(),
        router=_FakeSplitEndpointRouter(),
        crossing_points=[(7.0, 0.0), (12.0, 0.0)],
        realization_grid_spec=(24, 4, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.endpoint_correction_error is None
    assert updated.corrected_centerline_um == (
        (0.0, 0.0),
        (3.0, 0.0),
        (9.0, 0.0),
        (16.0, 0.0),
        (21.0, 0.0),
    )


def test_crossing_aware_endpoint_correction_rejects_unanchored_source_prefix():
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        _split_record(),
        router=_FakeUnanchoredEndpointRouter(),
        crossing_points=[(7.0, 0.0), (12.0, 0.0)],
        realization_grid_spec=(24, 4, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.endpoint_correction_error is None
    assert updated.corrected_centerline_um == (
        (0.0, 0.0),
        (3.0, 0.0),
        (9.0, 0.0),
        (16.0, 0.0),
        (21.0, 0.0),
    )


def test_crossing_aware_endpoint_correction_keeps_required_port_straight():
    corrected = _absorbed_terminal_centerline(
        ((1.0, 0.0), (2.0, -1.0), (4.0, -1.0)),
        desired_start=(0.0, 0.0),
        extra_start_dir=(1.0, 0.0),
    )

    assert corrected == (
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, -1.0),
        (4.0, -1.0),
    )


def test_crossing_aware_endpoint_correction_does_not_modify_bend_samples():
    baseline = (
        (1.0, 0.5),
        (1.6, 0.4),
        (1.6, -1.6),
        (4.6, 1.4),
    )
    corrected = _absorbed_terminal_centerline(
        baseline,
        desired_start=(0.0, 0.0),
        extra_start_dir=(1.0, 0.0),
    )

    assert corrected
    assert corrected[0] == (0.0, 0.0)
    assert corrected[-1] == pytest.approx(baseline[-1])
    assert corrected[1][1] == pytest.approx(0.0)
    original_bend_sample_length = math.hypot(
        baseline[1][0] - baseline[0][0],
        baseline[1][1] - baseline[0][1],
    )
    corrected_bend_sample_length = math.hypot(
        corrected[2][0] - corrected[1][0],
        corrected[2][1] - corrected[1][1],
    )
    assert corrected_bend_sample_length == pytest.approx(
        original_bend_sample_length
    )


def test_crossing_aware_endpoint_correction_allows_oriented_port_stub():
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        _port_stub_record(),
        router=_FakePortStubEndpointRouter(),
        crossing_points=[(0.0, 4.5), (0.0, 5.5)],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.endpoint_correction_error is None
    assert updated.corrected_centerline_um == (
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 4.0),
        (0.0, 5.0),
        (0.0, 6.0),
        (0.0, 11.0),
    )


def test_crossing_aware_endpoint_correction_lengthens_existing_terminal_straights():
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        _simple_record(),
        router=_FakeStraightEndpointRouter(),
        crossing_points=[(4.5, 0.0), (5.5, 0.0)],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.endpoint_correction_error is None
    assert updated.corrected_centerline_um == (
        (0.0, 0.0),
        (4.0, 0.0),
        (5.0, 0.0),
        (6.0, 0.0),
        (11.0, 0.0),
    )
    assert updated.base_total_length_um == 9.0


def test_crossing_free_endpoint_correction_uses_normal_corrected_centerline():
    record = replace(
        _simple_record(),
        corrected_centerline_um=((1.0, 0.0), (4.0, 0.0), (10.0, 0.0)),
    )
    updated = _apply_crossing_aware_endpoint_correction_to_record(
        record,
        router=_FakeEndpointRouter(),
        crossing_points=[],
        realization_grid_spec=(20, 20, 1.0, 0.0, 0.0),
        route_width_um=0.5,
        allow_unchecked_bumps=True,
        log_failures=True,
    )

    assert updated.corrected_centerline_um == (
        (0.0, 1.0),
        (2.0, 1.0),
        (4.0, 1.0),
        (6.0, 1.0),
        (9.0, 1.0),
        (11.0, 1.0),
    )


def test_checked_no_bump_endpoint_correction_allows_active_endpoint_static_contact():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_no_bump_diagonal_test_router_and_route(rust_backend)
    router.add_static_cells([(10, 10)])

    result = router.route_port_corrected_centerline_checked_and_commit(
        7,
        route,
        0.5,
        0,
        0,
        [(10, 10)],
        [],
        source_port_um=(1.2, 1.2),
        target_port_um=(10.0, 10.0),
        allow_unchecked_fallback=False,
    )

    assert result["committed_bump"] is False
    assert result["candidate_index"] is None
    centerline = tuple((float(x), float(y)) for x, y in result["centerline"])
    assert centerline[0] == pytest.approx((1.2, 1.2))
    assert centerline[-1] == pytest.approx((10.0, 10.0))


def test_checked_no_bump_endpoint_correction_rejects_middle_static_contact():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    router, route = _checked_no_bump_diagonal_test_router_and_route(rust_backend)
    router.add_static_cells([(5, 5)])

    with pytest.raises(RuntimeError, match="static_overlap"):
        router.route_port_corrected_centerline_checked_and_commit(
            7,
            route,
            0.5,
            0,
            0,
            [],
            [],
            source_port_um=(1.2, 1.2),
            target_port_um=(10.0, 10.0),
            allow_unchecked_fallback=False,
        )
