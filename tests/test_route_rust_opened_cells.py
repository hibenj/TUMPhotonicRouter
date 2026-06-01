from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from gdsfactory.component import Component
from photonic_router.static_obstacle_builder import GridSpec
from translation import route_rust


@dataclass
class _DummyBundle:
    links: dict[str, str]


@dataclass
class _DummyNetlist:
    routes: dict[str, _DummyBundle]


@dataclass
class _DummySchematic:
    netlist: _DummyNetlist


class _DummyObstacleData:
    def __init__(self, blocked_cells: set[tuple[int, int]]) -> None:
        self.grid = GridSpec(
            width=30,
            height=20,
            grid_size_um=1.0,
            origin=(0.0, 0.0),
            die_bbox=(0.0, 0.0, 30.0, 20.0),
        )
        self.blocked_cells = blocked_cells

    def export_debug_svg(self, path: Any) -> None:
        path.write_text("<svg/>", encoding="utf-8")


def _make_dummy_layout() -> Component:
    return Component(f"dummy_layout_{uuid4().hex}")


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


def test_route_nets_rust_does_not_open_dynamic_geometry(monkeypatch, tmp_path):
    def fake_build_static_obstacle_map(_component, config=None):
        _ = config
        return _DummyObstacleData(blocked_cells=set())

    ports = {
        ("left_a", "o1"): SimpleNamespace(center=(2.0, 10.0), orientation=0.0),
        ("right_a", "o1"): SimpleNamespace(center=(28.0, 10.0), orientation=180.0),
        ("left_b", "o1"): SimpleNamespace(center=(2.0, 12.0), orientation=0.0),
        ("right_b", "o1"): SimpleNamespace(center=(28.0, 12.0), orientation=180.0),
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
