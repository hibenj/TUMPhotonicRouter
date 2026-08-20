from __future__ import annotations

import ast
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from gdsfactory.component import Component
import gdsfactory as gf
from gdsfactory.gpdk import get_generic_pdk
from photonic_router.static_obstacle_builder import GridSpec, StaticObstacleMapConfig
from translation import route_rust

get_generic_pdk().activate()


@dataclass
class _DummyBundle:
    links: dict[str, str]


@dataclass
class _DummyNetlist:
    routes: dict[str, _DummyBundle]
    instances: dict[str, Any] = field(default_factory=dict)


@dataclass
class _DummySchematic:
    netlist: _DummyNetlist


class _DummyObstacleData:
    def __init__(
        self,
        blocked_cells: set[tuple[int, int]],
        raw_blocked_cells: set[tuple[int, int]] | None = None,
        port_open_cells: set[tuple[int, int]] | None = None,
        width: int = 30,
        height: int = 20,
    ) -> None:
        self.grid = GridSpec(
            width=width,
            height=height,
            grid_size_um=1.0,
            origin=(0.0, 0.0),
            die_bbox=(0.0, 0.0, float(width), float(height)),
        )
        self.blocked_cells = blocked_cells
        if raw_blocked_cells is not None:
            self.raw_blocked_cells = raw_blocked_cells
        if port_open_cells is not None:
            self.port_open_cells = port_open_cells

    def export_debug_svg(self, path: Any) -> None:
        path.write_text("<svg/>", encoding="utf-8")


def _make_dummy_layout() -> Component:
    return Component(f"dummy_layout_{uuid4().hex}")


def _diagnostic_value(text: str, key: str) -> str:
    prefix = f"{key}="
    return next(line[len(prefix):] for line in text.splitlines() if line.startswith(prefix))


def _diagnostic_opened_cells(text: str) -> set[tuple[int, int]]:
    return {
        (int(x), int(y))
        for x, y in ast.literal_eval(_diagnostic_value(text, "opened_cells"))
    }


def _corridor_session(width: int, height: int, bend_radius_cells: int = 2):
    return SimpleNamespace(
        grid_width=width,
        grid_height=height,
        bend_radius_cells=bend_radius_cells,
    )


def _state(x: int, y: int, angle: int = 0):
    return SimpleNamespace(x=x, y=y, angle=angle)


class _PortOpeningCaptured(Exception):
    pass


def test_port_lane_half_width_scales_with_bend_radius(monkeypatch):
    captured_lane_widths: list[int] = []

    class FakeBackendGridSpec:
        def __init__(
            self,
            width: int,
            height: int,
            grid_size_um: float,
            origin_x_um: float,
            origin_y_um: float,
        ) -> None:
            self.width = width
            self.height = height
            self.grid_size_um = grid_size_um
            self.origin_x_um = origin_x_um
            self.origin_y_um = origin_y_um

    class FakePrimitiveLibraryConfig:
        def __init__(
            self,
            *,
            grid_size_um: float,
            bend_radius_cells: int,
            allow_45_degree_turns: bool,
        ) -> None:
            self.grid_size_um = grid_size_um
            self.bend_radius_cells = bend_radius_cells
            self.allow_45_degree_turns = allow_45_degree_turns

    class FakeAStarConfig:
        def __init__(self, max_iterations: int) -> None:
            self.max_iterations = max_iterations

    class CapturingRouter:
        def __init__(
            self,
            _grid_spec: FakeBackendGridSpec,
            _primitive_cfg: FakePrimitiveLibraryConfig,
            _astar_cfg: FakeAStarConfig,
        ) -> None:
            pass

        def build_port_footprint_cells(
            self,
            ports: list[tuple[str, float, float, float | None, int, int]],
        ):
            for _spec, _x_um, _y_um, _orientation, _length_cells, half_width_cells in ports:
                captured_lane_widths.append(int(half_width_cells))
            raise _PortOpeningCaptured

    fake_backend = SimpleNamespace(
        GridSpec=FakeBackendGridSpec,
        PrimitiveLibraryConfig=FakePrimitiveLibraryConfig,
        AStarConfig=FakeAStarConfig,
        PyPhotonicRouter=CapturingRouter,
    )

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(blocked_cells=set(), width=30, height=20)

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(2.0, 10.0), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(24.0, 10.0), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "_load_rust_backend", lambda: fake_backend)
    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "port_lane_width": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    for bend_radius_um in (1.0, 3.0):
        with pytest.raises(_PortOpeningCaptured):
            route_rust.route_nets_rust(
                _make_dummy_layout(),
                schematic,  # type: ignore[arg-type]
                obstacle_config=StaticObstacleMapConfig(
                    obstacle_mode="rasterized_polygons",
                    grid_size_um=1.0,
                    security_margin_um=0.0,
                    clearance_um=0.0,
                    port_open_radius_um=0.0,
                    die_bbox=(0.0, 0.0, 30.0, 20.0),
                ),
                route_width_um=0.5,
                allow_45_degree_turns=False,
                bend_radius_um=bend_radius_um,
                max_iterations=100_000,
                defer_realization=True,
            )

    assert captured_lane_widths == [2, 2, 4, 4]


def _footprint_resolver_session(
    *,
    bend_radius_cells: int,
    commit_radius_cells: int = 0,
    grid_size_um: float = 1.0,
    access_rule: tuple[float | None, float | None, str | None] = (None, None, None),
) -> SimpleNamespace:
    def keyed_port_access_rule(**_kwargs: object) -> tuple[float | None, float | None, str | None]:
        return access_rule

    return SimpleNamespace(
        grid=SimpleNamespace(grid_size_um=grid_size_um),
        port_lane_length_cells=max(3, 2 * bend_radius_cells + 2),
        port_lane_half_width_cells=max(1, bend_radius_cells + commit_radius_cells + 1),
        _keyed_port_access_rule=keyed_port_access_rule,
    )


def test_resolve_port_footprint_cells_uses_lane_sized_optical_default():
    small_bend_session = _footprint_resolver_session(bend_radius_cells=1)
    large_bend_session = _footprint_resolver_session(bend_radius_cells=3)
    port = SimpleNamespace(port_type="optical")

    small_footprint = route_rust._RouteNetsRustSession._resolve_port_footprint_cells(
        small_bend_session,
        instance_name="gc_0",
        port_name="o1",
        port=port,
    )
    large_footprint = route_rust._RouteNetsRustSession._resolve_port_footprint_cells(
        large_bend_session,
        instance_name="gc_0",
        port_name="o1",
        port=port,
    )

    assert small_footprint == (
        small_bend_session.port_lane_length_cells,
        small_bend_session.port_lane_half_width_cells,
    )
    assert large_footprint == (
        large_bend_session.port_lane_length_cells,
        large_bend_session.port_lane_half_width_cells,
    )
    assert large_footprint[1] > small_footprint[1]

    custom_session = _footprint_resolver_session(
        bend_radius_cells=3,
        grid_size_um=2.0,
        access_rule=(6.1, 5.0, "gc"),
    )
    assert route_rust._RouteNetsRustSession._resolve_port_footprint_cells(
        custom_session,
        instance_name="gc_0",
        port_name="o1",
        port=port,
    ) == (4, 2)


def test_corridor_clearance_reports_no_bare_centerline_path():
    session = _corridor_session(width=12, height=6)
    blocked_cells = {(5, y) for y in range(6)}

    diagnostic = route_rust._RouteNetsRustSession._corridor_clearance_diagnostic(
        session,
        _state(2, 3),
        _state(9, 3),
        blocked_cells,
        max_radius=3,
    )

    assert diagnostic["max_radius_checked"] == 3
    assert diagnostic["last_connected_radius"] is None
    assert diagnostic["first_disconnected_radius"] == 0
    assert diagnostic["source_region_size"] == 30
    assert diagnostic["target_region_size"] == 36


def test_corridor_clearance_reports_narrow_gap_closed_by_one_cell_clearance():
    session = _corridor_session(width=12, height=6)
    blocked_cells = {(8, y) for y in range(6) if y != 3}

    diagnostic = route_rust._RouteNetsRustSession._corridor_clearance_diagnostic(
        session,
        _state(2, 3),
        _state(10, 3),
        blocked_cells,
        max_radius=3,
    )

    assert diagnostic["last_connected_radius"] == 0
    assert diagnostic["first_disconnected_radius"] == 1
    assert diagnostic["source_region_size"] == 42
    assert diagnostic["target_region_size"] == 12
    assert diagnostic["source_region_min_distance_to_target"] == 4
    assert diagnostic["target_region_min_distance_to_source"] == 8


def test_corridor_clearance_reports_open_grid_stays_connected():
    session = _corridor_session(width=12, height=6)

    diagnostic = route_rust._RouteNetsRustSession._corridor_clearance_diagnostic(
        session,
        _state(2, 3),
        _state(10, 3),
        set(),
        max_radius=3,
    )

    assert diagnostic["max_radius_checked"] == 3
    assert diagnostic["last_connected_radius"] == 3
    assert diagnostic["first_disconnected_radius"] is None
    assert diagnostic["source_region_size"] is None
    assert diagnostic["target_region_size"] is None


def test_route_nets_rust_does_not_open_static_geometry(monkeypatch, tmp_path):
    wall_cells = {(16, y) for y in range(20)}

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(blocked_cells=wall_cells)

    ports = {
        ("left", "o1"): SimpleNamespace(center=(4.0, 10.0), orientation=0.0),
        ("right", "o1"): SimpleNamespace(center=(26.0, 10.0), orientation=180.0),
    }

    def fake_get_port_from_instance(_layout, inst, port):
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "wall_cross": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    try:
        route_rust.route_nets_rust(
            _make_dummy_layout(),
            schematic,  # type: ignore[arg-type]
            debug_dir=tmp_path,
            debug_prefix="opened_cells_regression",
            route_width_um=0.5,
            allow_45_degree_turns=False,
            max_iterations=100_000,
            defer_realization=True,
        )
    except RuntimeError as exc:
        assert "No route found" in str(exc)
    else:
        raise AssertionError("Expected net to fail: wall must remain blocked.")

    diag_path = tmp_path / "routes" / "opened_cells_regression_wall_cross_diagnostics.txt"
    diag_text = diag_path.read_text(encoding="utf-8")

    overlap_line = next(
        line for line in diag_text.splitlines()
        if line.startswith("opened_candidate_static_overlap_count=")
    )
    overlap_count = int(overlap_line.split("=", 1)[1])
    assert overlap_count > 0

    route_static_overlap_line = next(
        line for line in diag_text.splitlines()
        if line.startswith("route_static_blocked_overlap_count=")
    )
    route_static_overlap_count = int(route_static_overlap_line.split("=", 1)[1])
    assert route_static_overlap_count == 0

    route_opened_static_line = next(
        line for line in diag_text.splitlines()
        if line.startswith("route_overlap_effective_opened_static_count=")
    )
    route_opened_static_count = int(route_opened_static_line.split("=", 1)[1])
    assert route_opened_static_count == 0


def test_rust_dynamic_clearance_exempt_batch_uses_endpoint_contact_cells_only():
    rust_backend = route_rust._load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend is not available")

    grid = rust_backend.GridSpec(40, 30, 1.0, 0.0, 0.0)
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=True,
    )
    router = rust_backend.PyPhotonicRouter(
        grid,
        primitive_cfg,
        rust_backend.AStarConfig(max_iterations=100),
    )
    source = rust_backend.State(10, 10, 0)
    target = rust_backend.State(24, 12, 4)

    ninety_result = router.build_dynamic_clearance_exempt_cells_for_routes(
        [(7, source, target)],
        False,
        2,
        1,
    )
    diagonal_result = router.build_dynamic_clearance_exempt_cells_for_routes(
        [(7, source, target)],
        True,
        2,
        1,
    )

    ninety_cells = set(ninety_result[0][1])
    diagonal_cells = set(diagonal_result[0][1])
    assert ninety_result[0][0] == 7
    assert diagonal_result[0][0] == 7
    assert (10, 10) in diagonal_cells
    assert (24, 12) in diagonal_cells
    assert ninety_cells == diagonal_cells
    assert (11, 10) in diagonal_cells
    assert (23, 12) in diagonal_cells
    assert (12, 12) not in diagonal_cells


def test_route_nets_rust_applies_heater_opening_without_heater_obstacle_layers(
    monkeypatch,
    tmp_path,
):
    def fake_build_static_obstacle_map(_component, config=None):
        return _DummyObstacleData(blocked_cells=set(), width=80, height=40)

    ports = {
        ("heater", "o2"): SimpleNamespace(
            center=(20.0, 10.0),
            orientation=0.0,
            port_type="optical",
        ),
        ("sink", "o1"): SimpleNamespace(
            center=(75.0, 10.0),
            orientation=180.0,
            port_type="optical",
        ),
        ("left", "o1"): SimpleNamespace(
            center=(2.0, 30.0),
            orientation=0.0,
            port_type="optical",
        ),
        ("right", "o1"): SimpleNamespace(
            center=(60.0, 30.0),
            orientation=180.0,
            port_type="optical",
        ),
    }

    def fake_get_port_from_instance(_layout, inst, port):
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "heater_net": _DummyBundle(links={"heater,o2": "sink,o1"}),
                "unrelated_net": _DummyBundle(links={"left,o1": "right,o1"}),
            },
            instances={
                "heater": SimpleNamespace(component="straight_heater_metal"),
                "sink": SimpleNamespace(component="straight"),
                "left": SimpleNamespace(component="straight"),
                "right": SimpleNamespace(component="straight"),
            },
        )
    )

    _routed_layout, debug_artifacts = route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        debug_dir=tmp_path,
        debug_prefix="heater_keyed_openings",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        include_heater_obstacles=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    heater_diag = (
        tmp_path / "routes" / "heater_keyed_openings_heater_net_diagnostics.txt"
    ).read_text(encoding="utf-8")
    unrelated_diag = (
        tmp_path / "routes" / "heater_keyed_openings_unrelated_net_diagnostics.txt"
    ).read_text(encoding="utf-8")

    assert _diagnostic_value(heater_diag, "source_access_rule") == "straight_heater_metal*"
    assert (35, 10) in _diagnostic_opened_cells(heater_diag)
    assert _diagnostic_value(unrelated_diag, "source_access_rule") == "None"
    assert _diagnostic_value(unrelated_diag, "target_access_rule") == "None"
    assert (35, 10) not in _diagnostic_opened_cells(unrelated_diag)
    records_by_net = {record.net_name: record for record in debug_artifacts.routed_net_records}
    assert (35, 10) in records_by_net["heater_net"].opened_cells
    assert (35, 10) not in records_by_net["unrelated_net"].opened_cells


def test_route_nets_rust_does_not_apply_heater_rule_to_electrical_port(
    monkeypatch,
    tmp_path,
):
    def fake_build_static_obstacle_map(_component, config=None):
        return _DummyObstacleData(blocked_cells=set(), width=80, height=40)

    ports = {
        ("heater", "l_e1"): SimpleNamespace(
            center=(20.0, 10.0),
            orientation=0.0,
            port_type="electrical",
        ),
        ("sink", "o1"): SimpleNamespace(
            center=(75.0, 10.0),
            orientation=180.0,
            port_type="optical",
        ),
    }

    def fake_get_port_from_instance(_layout, inst, port):
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "electrical_endpoint_net": _DummyBundle(links={"heater,l_e1": "sink,o1"}),
            },
            instances={
                "heater": SimpleNamespace(component="straight_heater_metal"),
                "sink": SimpleNamespace(component="straight"),
            },
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        debug_dir=tmp_path,
        debug_prefix="heater_electrical_opening",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        include_heater_obstacles=True,
        max_iterations=100_000,
        defer_realization=True,
    )

    diag_text = (
        tmp_path
        / "routes"
        / "heater_electrical_opening_electrical_endpoint_net_diagnostics.txt"
    ).read_text(encoding="utf-8")

    assert _diagnostic_value(diag_text, "source_access_rule") == "None"
    assert (35, 10) not in _diagnostic_opened_cells(diag_text)


def test_route_nets_rust_defaults_to_strict_bounding_box_mode(monkeypatch, tmp_path):
    captured: dict[str, StaticObstacleMapConfig | None] = {}

    def fake_build_static_obstacle_map(_component, config=None):
        captured["config"] = config
        return _DummyObstacleData(blocked_cells=set())

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(27.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "default_mode_net": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        debug_dir=tmp_path,
        debug_prefix="strict_default_mode",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    resolved = captured["config"]
    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_mode == "bounding_boxes"
    assert resolved.clear_port_open_cells_from_static is False


def test_route_nets_rust_does_not_open_dynamic_geometry(monkeypatch, tmp_path):
    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(blocked_cells=set())

    ports = {
        ("left_a", "o1"): SimpleNamespace(center=(2.0, 10.0), orientation=0.0),
        ("right_a", "o1"): SimpleNamespace(center=(28.0, 10.0), orientation=180.0),
        ("left_b", "o1"): SimpleNamespace(center=(2.0, 14.0), orientation=0.0),
        ("right_b", "o1"): SimpleNamespace(center=(28.0, 14.0), orientation=180.0),
    }

    def fake_get_port_from_instance(_layout, inst, port):
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "net_a": _DummyBundle(links={"left_a,o1": "right_a,o1"}),
                "net_b": _DummyBundle(links={"left_b,o1": "right_b,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        debug_dir=tmp_path,
        debug_prefix="opened_dynamic_regression",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    diag_path = tmp_path / "routes" / "opened_dynamic_regression_net_b_diagnostics.txt"
    diag_text = diag_path.read_text(encoding="utf-8")

    route_dynamic_line = next(
        line for line in diag_text.splitlines()
        if line.startswith("route_dynamic_overlap_count=")
    )
    route_dynamic_overlap_count = int(route_dynamic_line.split("=", 1)[1])
    assert route_dynamic_overlap_count == 0

    route_opened_dynamic_line = next(
        line for line in diag_text.splitlines()
        if line.startswith("route_overlap_effective_opened_dynamic_count=")
    )
    route_opened_dynamic_count = int(route_opened_dynamic_line.split("=", 1)[1])
    assert route_opened_dynamic_count == 0


def test_route_nets_rust_opened_port_cell_in_bounding_box_mode(monkeypatch, tmp_path):
    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(4.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    layout = gf.Component("bounding_box_opened_cells")
    layout.add_polygon([(1.0, 10.0), (2.0, 10.0), (2.0, 11.0), (1.0, 11.0)], layer=(1, 0))

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "boxed_port_net": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        layout,
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="bounding_boxes",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 30.0, 20.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="bounding_box_opened",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    diag_path = tmp_path / "routes" / "bounding_box_opened_boxed_port_net_diagnostics.txt"
    assert diag_path.exists()
    diag_text = diag_path.read_text(encoding="utf-8")
    assert "status=ok" in diag_text


def test_route_nets_rust_route_to_blocked_port_with_opened_cells(monkeypatch, tmp_path):
    blocked_cells = {(26, 10)}

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=set(blocked_cells),
            raw_blocked_cells=None,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(27.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "blocked_port_with_opened_cells": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            clear_port_open_cells_from_static=False,
            die_bbox=(0.0, 0.0, 30.0, 20.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="strict_mode_blocked_port",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    diag_path = tmp_path / "routes" / "strict_mode_blocked_port_blocked_port_with_opened_cells_diagnostics.txt"
    diag_text = diag_path.read_text(encoding="utf-8")
    assert "status=ok" in diag_text
    assert "route_static_blocked_overlap_count=1" in diag_text
    assert "route_overlap_effective_opened_static_count=1" in diag_text


def test_route_nets_rust_foreign_port_keepout_blocks_unrelated_net(monkeypatch, tmp_path):
    corridor_y = 10
    blocked_cells = {
        (x, y)
        for x in range(30)
        for y in range(21)
        if y != corridor_y
    }

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=30,
            height=21,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(28.5, 10.5), orientation=180.0),
            ("mid", "o1"): SimpleNamespace(center=(14.5, 10.5), orientation=0.0),
            ("future", "o1"): SimpleNamespace(center=(20.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "unrelated": _DummyBundle(links={"left,o1": "right,o1"}),
                "future_mid": _DummyBundle(links={"mid,o1": "future,o1"}),
            }
        )
    )

    with pytest.raises(RuntimeError, match="No route found for unrelated"):
        route_rust.route_nets_rust(
            _make_dummy_layout(),
            schematic,  # type: ignore[arg-type]
            obstacle_config=StaticObstacleMapConfig(
                obstacle_mode="rasterized_polygons",
                grid_size_um=1.0,
                security_margin_um=0.0,
                clearance_um=0.0,
                port_open_radius_um=0.0,
                die_bbox=(0.0, 0.0, 30.0, 21.0),
            ),
            debug_dir=tmp_path,
            debug_prefix="foreign_keepout_blocks",
            route_width_um=0.5,
            allow_45_degree_turns=False,
            bend_radius_um=3.0,
            foreign_port_keepout_cells=3,
            max_iterations=10_000,
            defer_realization=True,
        )


def test_route_nets_rust_foreign_port_keepout_does_not_open_sibling_port(
    monkeypatch,
    tmp_path,
):
    corridor_y = 10
    blocked_cells = {
        (x, y)
        for x in range(30)
        for y in range(21)
        if y != corridor_y
    }

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=30,
            height=21,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("multi", "o1"): SimpleNamespace(center=(28.5, 10.5), orientation=180.0),
            ("multi", "o2"): SimpleNamespace(center=(14.5, 10.5), orientation=180.0),
            ("future", "o1"): SimpleNamespace(center=(20.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "to_active_port": _DummyBundle(links={"left,o1": "multi,o1"}),
                "sibling_reservation": _DummyBundle(links={"multi,o2": "future,o1"}),
            }
        )
    )

    with pytest.raises(RuntimeError, match="No route found for to_active_port"):
        route_rust.route_nets_rust(
            _make_dummy_layout(),
            schematic,  # type: ignore[arg-type]
            obstacle_config=StaticObstacleMapConfig(
                obstacle_mode="rasterized_polygons",
                grid_size_um=1.0,
                security_margin_um=0.0,
                clearance_um=0.0,
                port_open_radius_um=0.0,
                die_bbox=(0.0, 0.0, 30.0, 21.0),
            ),
            debug_dir=tmp_path,
            debug_prefix="foreign_keepout_sibling_closed",
            route_width_um=0.5,
            allow_45_degree_turns=False,
            bend_radius_um=3.0,
            foreign_port_keepout_cells=3,
            max_iterations=10_000,
            defer_realization=True,
        )


def test_route_nets_rust_dense_same_instance_keepout_does_not_open_raw_static_sibling(
    monkeypatch,
    tmp_path,
):
    corridor_y = 10
    sibling_static_cell = (14, corridor_y)
    blocked_cells = {
        (x, y)
        for x in range(30)
        for y in range(21)
        if y != corridor_y
    }
    blocked_cells.add(sibling_static_cell)

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=30,
            height=21,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("multi", "o1"): SimpleNamespace(center=(28.5, 10.5), orientation=180.0),
            ("multi", "o2"): SimpleNamespace(center=(14.5, 10.5), orientation=180.0),
            ("multi", "o3"): SimpleNamespace(center=(20.5, 10.5), orientation=180.0),
            ("future_a", "o1"): SimpleNamespace(center=(4.5, 10.5), orientation=0.0),
            ("future_b", "o1"): SimpleNamespace(center=(6.5, 10.5), orientation=0.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "to_active_port": _DummyBundle(links={"left,o1": "multi,o1"}),
                "sibling_reservation_a": _DummyBundle(links={"multi,o2": "future_a,o1"}),
                "sibling_reservation_b": _DummyBundle(links={"multi,o3": "future_b,o1"}),
            }
        )
    )

    with pytest.raises(RuntimeError, match="No route found for to_active_port"):
        route_rust.route_nets_rust(
            _make_dummy_layout(),
            schematic,  # type: ignore[arg-type]
            obstacle_config=StaticObstacleMapConfig(
                obstacle_mode="rasterized_polygons",
                grid_size_um=1.0,
                security_margin_um=0.0,
                clearance_um=0.0,
                port_open_radius_um=0.0,
                die_bbox=(0.0, 0.0, 30.0, 21.0),
            ),
            debug_dir=tmp_path,
            debug_prefix="dense_same_instance_raw_static_closed",
            route_width_um=0.5,
            allow_45_degree_turns=False,
            bend_radius_um=2.0,
            foreign_port_keepout_cells=3,
            max_iterations=10_000,
            defer_realization=True,
        )

    diag_path = (
        tmp_path
        / "routes"
        / "dense_same_instance_raw_static_closed_to_active_port_diagnostics.txt"
    )
    diag_text = diag_path.read_text(encoding="utf-8")
    opened_cells = _diagnostic_opened_cells(diag_text)
    assert sibling_static_cell not in opened_cells
    assert (13, corridor_y) not in opened_cells
    assert "foreign_port_keepout_cells=3" in diag_text


def test_route_nets_rust_foreign_port_keepout_uses_unified_self_opening_for_same_instance(
    monkeypatch,
    tmp_path,
):
    corridor_y = 10
    blocked_cells = {
        (x, y)
        for x in range(30)
        for y in range(21)
        if y != corridor_y
    }

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=30,
            height=21,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("mid", "o1"): SimpleNamespace(center=(14.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "to_mid": _DummyBundle(links={"left,o1": "mid,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 30.0, 21.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="foreign_keepout_opens",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        bend_radius_um=3.0,
        foreign_port_keepout_cells=3,
        max_iterations=10_000,
        defer_realization=True,
    )

    diag_path = tmp_path / "routes" / "foreign_keepout_opens_to_mid_diagnostics.txt"
    diag_text = diag_path.read_text(encoding="utf-8")
    assert "status=ok" in diag_text
    assert "foreign_port_keepout_cells=3" in diag_text
    assert int(_diagnostic_value(diag_text, "foreign_port_keepout_open_count")) == 0


def test_route_nets_rust_removes_foreign_keepout_after_port_is_routed(
    monkeypatch,
    tmp_path,
):
    routed_lane_y = 10
    released_lane_y = 12
    blocked_cells = {
        (x, y)
        for x in range(30)
        for y in range(21)
        if y not in {routed_lane_y, released_lane_y}
    }

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=30,
            height=21,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left_a", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("mid", "o1"): SimpleNamespace(center=(14.5, 10.5), orientation=180.0),
            ("left_b", "o1"): SimpleNamespace(center=(1.5, 12.5), orientation=0.0),
            ("right_b", "o1"): SimpleNamespace(center=(28.5, 12.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "connect_mid": _DummyBundle(links={"left_a,o1": "mid,o1"}),
                "through_released_keepout": _DummyBundle(
                    links={"left_b,o1": "right_b,o1"}
                ),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 30.0, 21.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="foreign_keepout_cleanup",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        foreign_port_keepout_cells=3,
        max_iterations=10_000,
        defer_realization=True,
    )

    diag_path = (
        tmp_path
        / "routes"
        / "foreign_keepout_cleanup_through_released_keepout_diagnostics.txt"
    )
    diag_text = diag_path.read_text(encoding="utf-8")
    assert "status=ok" in diag_text
    assert "route_static_blocked_overlap_count=0" in diag_text


def test_route_nets_rust_static_stub_fanout_uses_virtual_source_anchor(
    monkeypatch,
    tmp_path,
):
    blocked_cells: set[tuple[int, int]] = set()

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=45,
            height=20,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("mmi0_multiport", "o1"): SimpleNamespace(center=(2.5, 6.5), orientation=0.0),
            ("mmi0_multiport", "o2"): SimpleNamespace(center=(2.5, 8.5), orientation=0.0),
            ("mmi0_multiport", "o3"): SimpleNamespace(center=(2.5, 10.5), orientation=0.0),
            ("right0", "o1"): SimpleNamespace(center=(35.5, 3.5), orientation=180.0),
            ("right1", "o1"): SimpleNamespace(center=(35.5, 8.5), orientation=180.0),
            ("right2", "o1"): SimpleNamespace(center=(35.5, 13.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "spread_0": _DummyBundle(links={"mmi0_multiport,o1": "right0,o1"}),
                "spread_1": _DummyBundle(links={"mmi0_multiport,o2": "right1,o1"}),
                "spread_2": _DummyBundle(links={"mmi0_multiport,o3": "right2,o1"}),
            },
            instances={
                "mmi0_multiport": SimpleNamespace(component="mmi_3x3"),
            },
        )
    )

    _routed_layout, debug_artifacts = route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 45.0, 20.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="static_stub_fanout",
        debug_stop_after_route_index=1,
        route_width_um=0.5,
        allow_45_degree_turns=True,
        bend_radius_um=1.0,
        foreign_port_keepout_cells=0,
        fanout_access_mode="static-stubs",
        max_iterations=20_000,
        defer_realization=True,
    )

    diag_path = tmp_path / "routes" / "static_stub_fanout_spread_0_diagnostics.txt"
    diag_text = diag_path.read_text(encoding="utf-8")
    assert "status=ok" in diag_text
    assert "fanout_access_mode=static-stubs" in diag_text
    assert "source_fanout_anchor=True" in diag_text
    # Recalibrated 2026-08-20: this scenario's actual runway-cells value is 6,
    # not the previously-asserted 9. Confirmed by direct reproduction and
    # cross-referenced against .agent/execplans/2026-08-18-dense-port-runway-clearance-reach.md,
    # which recalibrated _dense_source_port_runway_lengths's static-stubs
    # branch (the exact code path this scenario exercises via
    # fanout_access_mode="static-stubs") -- this assertion was never updated
    # for that change.
    assert "source_dense_port_runway_cells=6" in diag_text
    assert int(_diagnostic_value(diag_text, "fanout_anchor_port_count")) == 3
    assert int(_diagnostic_value(diag_text, "fanout_stub_static_cell_count")) > 0

    crossing_info = debug_artifacts.crossing_plan_info
    assert crossing_info["fanout_access_mode"] == "static-stubs"
    assert crossing_info["fanout_anchor_port_count"] == 3
    assert crossing_info["fanout_anchor_net_ids"] == [1, 2, 3]
    assert len(crossing_info["fanout_stub_centerlines_um"]) == 3
    first_stub = crossing_info["fanout_stub_centerlines_um"][0]
    assert first_stub["port_spec"] == "mmi0_multiport,o1"
    assert len(first_stub["centerline_um"]) > 1

    record = debug_artifacts.routed_net_records[0]
    assert record.corrected_centerline_um
    assert record.corrected_centerline_um[0] == pytest.approx((2.5, 6.5))
    assert record.corrected_centerline_um[-1] == pytest.approx((35.5, 3.5))
    assert len(record.corrected_centerline_um) > 2


def test_apply_checked_fanout_stub_endpoint_corrections_skips_both_sides_fanout_net():
    # Characterizes the currently-implicit both_fanout_stub fall-through
    # (2026-08-19-restructure-port-endpoint-correction ExecPlan, Milestone 0):
    # a net with fanout stubs on both its source and target sides is skipped
    # entirely by this pass, via the explicit
    # `if source_has_fanout_stub and target_has_fanout_stub: continue` guard
    # at translation/route_rust.py:4705, before it ever reaches port
    # resolution or the native corrector.
    #
    # A real, end-to-end benchmark layout could not be built to exercise this
    # case with an actual successful route (investigated directly, not
    # assumed): fanout anchors are only ever built from a net's *source*
    # side (`_build_static_fanout_anchors` only reads
    # `source_port_specs_by_instance`), so the only way for a net's *target*
    # port to also be a fanout anchor is for that exact port to be reused as
    # a different net's source port. Constructing that (two mmi_3x3-style
    # hubs, the second hub's port used both as one net's target and a
    # second net's source) makes the router reject one of the two nets
    # outright: the shared anchor grid cell cannot simultaneously be one
    # route's commit-destination and a different route's commit-origin in
    # the router's dynamic occupancy model ("No route found ...
    # candidate_blockers=[<the other net's id>]"). So this test instead
    # constructs the minimal session state needed to exercise the guard
    # directly, and proves via spies that control flow never reaches port
    # resolution (i.e. the explicit both-sides guard is what skips the net,
    # not some other, later fallback that happens to produce the same
    # empty result).
    net_id = 1
    record = SimpleNamespace(corrected_centerline_um=((0.0, 0.0), (1.0, 0.0)))
    job = SimpleNamespace()

    def _unexpected_call(name: str):
        def _raise(*_args: Any, **_kwargs: Any) -> Any:
            raise AssertionError(f"{name} must not be called for a both-fanout-stub net")

        return _raise

    session = object.__new__(route_rust._RouteNetsRustSession)
    session.enable_checked_endpoint_correction = True
    session.fanout_anchor_net_ids = {net_id}
    session.fanout_anchor_source_net_ids = {net_id}
    session.fanout_anchor_target_net_ids = {net_id}
    session.enable_crossings = False
    session.router = SimpleNamespace(
        apply_checked_endpoint_corrections=_unexpected_call(
            "apply_checked_endpoint_corrections"
        ),
    )
    session.route_bookkeeping = SimpleNamespace(records_by_id={net_id: record})
    session.route_jobs_by_id = {net_id: job}
    session._pipeline_timer_start = lambda: 0.0
    session._record_pipeline_timing = lambda *_args, **_kwargs: None
    session._routing_endpoint_center_um = _unexpected_call("_routing_endpoint_center_um")
    session._state_openings_for_job = _unexpected_call("_state_openings_for_job")

    corrected_net_ids = session._apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
        [net_id]
    )

    assert corrected_net_ids == []


def test_classify_net_for_endpoint_correction_covers_every_category():
    # Milestone 1 of the 2026-08-19-restructure-port-endpoint-correction
    # ExecPlan: direct unit coverage of the new shared classification
    # function, one net per named EndpointCorrectionCategory plus the
    # missing-record case. crossing_net_ids is passed in explicitly, the
    # same way both callers (_apply_checked_endpoint_corrections_for_net_ids,
    # _apply_checked_fanout_stub_endpoint_corrections_for_net_ids) compute
    # it once via _endpoint_correction_crossing_net_ids and reuse it.
    plain_net_id = 1
    crossing_net_id = 2
    source_stub_net_id = 3
    target_stub_net_id = 4
    both_stub_net_id = 5
    missing_net_id = 6

    def _record(corrected_centerline_um=()):
        return SimpleNamespace(corrected_centerline_um=corrected_centerline_um)

    session = object.__new__(route_rust._RouteNetsRustSession)
    session.enable_crossings = True
    session.fanout_anchor_source_net_ids = {source_stub_net_id, both_stub_net_id}
    session.fanout_anchor_target_net_ids = {target_stub_net_id, both_stub_net_id}
    session.fanout_anchor_net_ids = (
        session.fanout_anchor_source_net_ids | session.fanout_anchor_target_net_ids
    )
    session.route_bookkeeping = SimpleNamespace(
        records_by_id={
            plain_net_id: _record(),
            crossing_net_id: _record(),
            source_stub_net_id: _record(corrected_centerline_um=((0.0, 0.0), (1.0, 0.0))),
            target_stub_net_id: _record(corrected_centerline_um=((0.0, 0.0), (1.0, 0.0))),
            both_stub_net_id: _record(corrected_centerline_um=((0.0, 0.0), (1.0, 0.0))),
        }
    )
    session.route_jobs_by_id = {
        net_id: SimpleNamespace()
        for net_id in (
            plain_net_id,
            crossing_net_id,
            source_stub_net_id,
            target_stub_net_id,
            both_stub_net_id,
        )
    }
    crossing_net_ids = {crossing_net_id}

    def classify(net_id: int):
        return session._classify_net_for_endpoint_correction(
            net_id, crossing_net_ids=crossing_net_ids
        )

    assert classify(missing_net_id) is None

    plain = classify(plain_net_id)
    assert plain.category is route_rust.EndpointCorrectionCategory.UNRESTRICTED
    assert not plain.has_crossing
    assert not plain.source_has_fanout_stub
    assert not plain.target_has_fanout_stub

    crossing = classify(crossing_net_id)
    assert crossing.category is route_rust.EndpointCorrectionCategory.CROSSING_AWARE
    assert crossing.has_crossing

    source_only = classify(source_stub_net_id)
    assert source_only.category is route_rust.EndpointCorrectionCategory.FANOUT_STUB_SOURCE_ONLY
    assert source_only.source_has_fanout_stub
    assert not source_only.target_has_fanout_stub

    target_only = classify(target_stub_net_id)
    assert target_only.category is route_rust.EndpointCorrectionCategory.FANOUT_STUB_TARGET_ONLY
    assert not target_only.source_has_fanout_stub
    assert target_only.target_has_fanout_stub

    both = classify(both_stub_net_id)
    assert both.category is route_rust.EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP
    assert both.source_has_fanout_stub
    assert both.target_has_fanout_stub

    # A net flagged as a fanout anchor that has *not* yet been given a
    # corrected centerline falls through to UNRESTRICTED, matching the
    # real pass 1 today (see _classify_net_for_endpoint_correction's own
    # docstring) -- not a bug, an explicit, tested edge case.
    session.route_bookkeeping.records_by_id[source_stub_net_id] = _record()
    not_yet_precorrected = classify(source_stub_net_id)
    assert not_yet_precorrected.category is route_rust.EndpointCorrectionCategory.UNRESTRICTED


def test_endpoint_correction_crossing_net_ids_propagates_real_failures():
    # Milestone 2 of the 2026-08-19-restructure-port-endpoint-correction
    # ExecPlan: confirmed via git history that the old
    # `except Exception: crossing_net_ids = set()` swallow around this
    # exact call had never legitimately fired, so it was removed rather
    # than preserved. This pins down the resulting behavior: a real
    # failure in router.crossing_events() must propagate, not be silently
    # treated as "no crossings" (which would have let every net fall
    # through to the unrestricted corrector as if crossings were off).
    class _BrokenCrossingEventsRouter:
        def crossing_events(self):
            raise RuntimeError("crossing_events backend failure")

    session = object.__new__(route_rust._RouteNetsRustSession)
    session.enable_crossings = True
    session.router = _BrokenCrossingEventsRouter()

    with pytest.raises(RuntimeError, match="crossing_events backend failure"):
        session._endpoint_correction_crossing_net_ids()


def test_route_nets_rust_same_instance_port_access_does_not_open_sibling_lane(
    monkeypatch,
    tmp_path,
):
    blocked_cells: set[tuple[int, int]] = set()

    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(
            blocked_cells=blocked_cells,
            raw_blocked_cells=blocked_cells,
            width=40,
            height=25,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("future", "o1"): SimpleNamespace(center=(1.5, 14.5), orientation=0.0),
            ("multi", "o1"): SimpleNamespace(center=(30.5, 10.5), orientation=180.0),
            ("multi", "o2"): SimpleNamespace(center=(30.5, 14.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "to_active_port": _DummyBundle(links={"left,o1": "multi,o1"}),
                "sibling_later": _DummyBundle(links={"future,o1": "multi,o2"}),
            }
        )
    )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 40.0, 25.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="same_instance_sibling_lane",
        debug_stop_after_route_index=1,
        route_width_um=0.5,
        allow_45_degree_turns=False,
        bend_radius_um=2.0,
        foreign_port_keepout_cells=0,
        max_iterations=10_000,
        defer_realization=True,
    )

    diag_path = (
        tmp_path
        / "routes"
        / "same_instance_sibling_lane_to_active_port_diagnostics.txt"
    )
    diag_text = diag_path.read_text(encoding="utf-8")
    opened_cells = _diagnostic_opened_cells(diag_text)
    assert "status=ok" in diag_text
    assert (29, 14) not in opened_cells


def test_route_nets_rust_clear_port_opening_flag_controls_global_crossing_blocking(monkeypatch, tmp_path):
    def blocked_cells_for_corridor(remove_middle: bool) -> set[tuple[int, int]]:
        blocked = set()
        for x in range(2, 27):
            for y in range(20):
                if y != 10 or (x == 15 and not remove_middle):
                    blocked.add((x, y))
        return blocked

    def fake_build_static_obstacle_map(_component, config=None):
        config = config or StaticObstacleMapConfig()
        if config.clear_port_open_cells_from_static:
            blocked = blocked_cells_for_corridor(remove_middle=True)
        else:
            blocked = blocked_cells_for_corridor(remove_middle=False)
        return _DummyObstacleData(
            blocked_cells=blocked,
            raw_blocked_cells=None,
        )

    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left", "o1"): SimpleNamespace(center=(1.5, 10.5), orientation=0.0),
            ("right", "o1"): SimpleNamespace(center=(27.5, 10.5), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "build_static_obstacle_map", fake_build_static_obstacle_map)
    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "strict_mode_compare": _DummyBundle(links={"left,o1": "right,o1"}),
            }
        )
    )

    with pytest.raises(RuntimeError, match="No route found"):
        route_rust.route_nets_rust(
            _make_dummy_layout(),
            schematic,  # type: ignore[arg-type]
            obstacle_config=StaticObstacleMapConfig(
                obstacle_mode="rasterized_polygons",
                grid_size_um=1.0,
                security_margin_um=0.0,
                clearance_um=0.0,
                port_open_radius_um=0.0,
                clear_port_open_cells_from_static=False,
                die_bbox=(0.0, 0.0, 30.0, 20.0),
            ),
            debug_dir=tmp_path,
            debug_prefix="strict_mode_compare_false",
            route_width_um=0.5,
            allow_45_degree_turns=False,
            max_iterations=100_000,
            defer_realization=True,
        )

    route_rust.route_nets_rust(
        _make_dummy_layout(),
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="rasterized_polygons",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            clear_port_open_cells_from_static=True,
            die_bbox=(0.0, 0.0, 30.0, 20.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="strict_mode_compare_true",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )


def test_route_nets_rust_multi_net_with_bounding_box_mode(monkeypatch, tmp_path):
    def fake_get_port_from_instance(_layout, inst, port):
        ports = {
            ("left_a", "o1"): SimpleNamespace(center=(2.0, 10.0), orientation=0.0),
            ("right_a", "o1"): SimpleNamespace(center=(28.0, 10.0), orientation=180.0),
            ("left_b", "o1"): SimpleNamespace(center=(2.0, 14.0), orientation=0.0),
            ("right_b", "o1"): SimpleNamespace(center=(28.0, 14.0), orientation=180.0),
        }
        return ports[(inst, port)]

    monkeypatch.setattr(route_rust, "get_port_from_instance", fake_get_port_from_instance)

    layout = gf.Component("bounding_box_multi_net_routing")
    layout.add_polygon(
        [(10.0, 1.0), (11.0, 1.0), (11.0, 3.0), (10.0, 3.0)],
        layer=(1, 0),
    )

    schematic = _DummySchematic(
        netlist=_DummyNetlist(
            routes={
                "bbox_net_a": _DummyBundle(links={"left_a,o1": "right_a,o1"}),
                "bbox_net_b": _DummyBundle(links={"left_b,o1": "right_b,o1"}),
            }
        )
    )

    route_rust.route_nets_rust(
        layout,
        schematic,  # type: ignore[arg-type]
        obstacle_config=StaticObstacleMapConfig(
            obstacle_mode="bounding_boxes",
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 30.0, 20.0),
        ),
        debug_dir=tmp_path,
        debug_prefix="bounding_box_multi_net",
        route_width_um=0.5,
        allow_45_degree_turns=False,
        max_iterations=100_000,
        defer_realization=True,
    )

    for net_name in ("bbox_net_a", "bbox_net_b"):
        diag_path = tmp_path / "routes" / f"bounding_box_multi_net_{net_name}_diagnostics.txt"
        diag_text = diag_path.read_text(encoding="utf-8")
        assert "status=ok" in diag_text
        assert "route_dynamic_overlap_count=0" in diag_text
