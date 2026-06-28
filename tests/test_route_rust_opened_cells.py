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


def test_rust_port_opening_batch_filters_raw_static_rects():
    rust_backend = route_rust._load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend is not available")

    grid = rust_backend.GridSpec(30, 20, 1.0, 0.0, 0.0)
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=False,
    )
    router = rust_backend.PyPhotonicRouter(
        grid,
        primitive_cfg,
        rust_backend.AStarConfig(max_iterations=100),
    )

    result = router.build_route_port_openings(
        [
            ("optical", 2.0, 10.0, 0.0, "optical", None, None),
            ("electrical", 2.0, 14.0, 0.0, "electrical", None, None),
        ],
        raw_static_cells=[],
        raw_static_rects=[(5, 8, 8, 12)],
        route_clearance_um=3.0,
        port_open_radius_um=1.0,
        bend_radius_cells=2,
        commit_radius_cells=1,
        port_entry_length_cells=4,
        port_entry_half_width_cells=2,
        port_lane_length_cells=6,
        port_lane_half_width_cells=1,
    )

    by_spec = {
        spec: (set(cells), set(candidates), set(runway))
        for spec, cells, candidates, runway in result
    }
    effective, candidates, runway = by_spec["optical"]
    assert (5, 10) in candidates
    assert (5, 10) in runway
    assert (5, 10) not in effective
    assert (3, 10) in effective
    assert by_spec["electrical"] == (set(), set(), set())


def test_route_nets_rust_applies_heater_opening_only_to_connected_endpoint(
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
        include_heater_obstacles=True,
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
