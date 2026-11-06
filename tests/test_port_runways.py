"""Phase 5 slice 5A: port runways (`PHOTONIC_ROUTER_PORT_RUNWAYS=1`).

A runway is a static stub from a port centre onto the cell-centre lattice:
two shallow arcs of the bend radius absorb the sub-cell lateral residue, a
straight run reaches a cell-centre anchor with the port's heading.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from photonic_router.config import FanoutAccessConfig, RoutingConfig
from photonic_router.routing import route_jobs
from photonic_router.static_obstacle_builder import GridSpec

from tests.fixtures.sessions import settings_for_test, state_for_test


def _state(width: int = 40, height: int = 20, bend_radius_cells: int = 2):
    return state_for_test(
        grid=GridSpec(
            width=width,
            height=height,
            grid_size_um=1.0,
            origin=(0.0, 0.0),
            die_bbox=(0.0, 0.0, float(width), float(height)),
        ),
        grid_width=width,
        grid_height=height,
        origin_x_um=0.0,
        origin_y_um=0.0,
        bend_radius_cells=bend_radius_cells,
    )


def _settings(port_runways: bool | None = True):
    return settings_for_test(
        config=RoutingConfig(fanout=FanoutAccessConfig(port_runways=port_runways))
    )


def _port(center, orientation):
    return SimpleNamespace(center=center, orientation=float(orientation))


def _heading(points, index_a, index_b):
    dx = points[index_b][0] - points[index_a][0]
    dy = points[index_b][1] - points[index_a][1]
    norm = math.hypot(dx, dy)
    return (dx / norm, dy / norm)


def test_on_axis_port_gets_a_straight_runway_to_the_first_outward_cell_centre():
    anchor = route_jobs._port_runway_anchor(_settings(), _state(), "a,o1", _port((2.5, 6.5), 0.0))
    assert anchor is not None
    assert anchor.stub_centerline_um == ((2.5, 6.5), (3.5, 6.5))
    assert (anchor.state_x, anchor.state_y) == (3, 6)
    assert anchor.center_um == (3.5, 6.5)
    assert anchor.physical_angle == 0


def test_off_axis_port_gets_two_arcs_onto_the_lattice_and_a_cell_centre_anchor():
    anchor = route_jobs._port_runway_anchor(_settings(), _state(), "a,o1", _port((2.5, 6.3), 0.0))
    assert anchor is not None
    points = anchor.stub_centerline_um
    assert points[0] == (2.5, 6.3)
    assert points[-1] == anchor.center_um
    # the anchor is a cell centre with the port's heading, on the lattice axis y = 6.5
    assert anchor.center_um == (anchor.state_x + 0.5, anchor.state_y + 0.5)
    assert anchor.center_um[1] == 6.5
    # tangent along +x at both ends, monotone forward, lateral shift = the residue
    assert _heading(points, 0, 1)[1] == pytest_approx(0.0, abs=0.05)
    assert _heading(points, -2, -1) == (1.0, 0.0)
    xs = [p[0] for p in points]
    assert xs == sorted(xs)
    ys = [p[1] for p in points]
    assert min(ys) >= 6.3 - 1e-9 and max(ys) <= 6.5 + 1e-9
    # the arcs' forward extent: 2 R sin(acos(1 - d / (2 R))) with R = 2, d = 0.2
    theta = math.acos(1.0 - 0.1 / 2.0)
    assert points[-1][0] >= 2.5 + 2 * 2.0 * math.sin(theta) - 1e-9
    assert anchor.stub_center_cells  # committed static like a fan-out stub


def test_west_facing_port_runs_westward():
    anchor = route_jobs._port_runway_anchor(
        _settings(), _state(), "b,o1", _port((35.5, 8.8), 180.0)
    )
    assert anchor is not None
    assert anchor.stub_centerline_um[0] == (35.5, 8.8)
    assert anchor.center_um[0] < 35.5
    assert anchor.center_um == (anchor.state_x + 0.5, anchor.state_y + 0.5)
    assert anchor.center_um[1] == 8.5
    assert anchor.physical_angle == 4


def test_north_facing_port_shifts_in_x():
    anchor = route_jobs._port_runway_anchor(_settings(), _state(), "c,o1", _port((10.2, 3.5), 90.0))
    assert anchor is not None
    assert anchor.center_um == (10.5, anchor.state_y + 0.5)
    assert anchor.center_um[1] >= 4.5
    assert anchor.physical_angle == 2


def test_diagonal_ports_keep_the_old_path():
    assert (
        route_jobs._port_runway_anchor(_settings(), _state(), "d,o1", _port((5.5, 5.5), 45.0))
        is None
    )


def _job(net_id, spec1, port1, spec2, port2):
    inst1, name1 = spec1.split(",")
    inst2, name2 = spec2.split(",")
    return SimpleNamespace(
        net_id=net_id,
        inst1=inst1,
        port1=name1,
        inst2=inst2,
        port2=name2,
        source_port=port1,
        target_port=port2,
    )


def test_builder_is_off_by_default_and_skips_fan_out_anchored_ports():
    state = _state()
    a = _port((2.5, 6.3), 0.0)
    b = _port((35.5, 8.8), 180.0)
    state.endpoint_ports_by_spec = {"a,o1": ("a", "o1", a), "b,o1": ("b", "o1", b)}
    jobs = [_job(1, "a,o1", a, "b,o1", b)]
    assert route_jobs._build_port_runway_anchors(_settings(None), state, {}, jobs) == {}
    built = route_jobs._build_port_runway_anchors(_settings(True), state, {"a,o1": object()}, jobs)
    assert set(built) == {"b,o1"}


def test_ports_closer_than_their_runways_keep_the_old_path():
    # two tile ports facing each other one cell apart (a pre-placed crossing
    # grid link): no runway on either side
    state = _state()
    a = _port((10.5, 6.3), 0.0)
    b = _port((11.5, 6.3), 180.0)
    far = _port((35.5, 8.8), 180.0)
    state.endpoint_ports_by_spec = {
        "a,o1": ("a", "o1", a),
        "b,o1": ("b", "o1", b),
        "c,o1": ("c", "o1", far),
    }
    jobs = [_job(1, "a,o1", a, "b,o1", b), _job(2, "a,o2", a, "c,o1", far)]
    built = route_jobs._build_port_runway_anchors(_settings(True), state, {}, jobs)
    assert set(built) == {"c,o1"}


def pytest_approx(value, abs):  # small local helper, keeps the module free of pytest at import
    import pytest

    return pytest.approx(value, abs=abs)


def test_runway_anchors_keep_the_ordinary_port_footprint_and_dense_semantics_off():
    # A runway is the ordinary port's own approach: it must not inherit the
    # dense fan-out stub semantics (stub lane sizing, dense clusters, the
    # long-straight exemption). The first 27-cell table with runways
    # (b05bffd) did, the port footprints collapsed to the stub lane size 0
    # and six cells failed on crossing legality.
    from photonic_router.routing import crossing_plan_stage

    settings = _settings(True)
    state = _state()
    state.port_lane_length_cells = 12
    state.port_lane_half_width_cells = 6
    state.stub_port_lane_length_cells = 0
    state.stub_port_lane_half_width_cells = 0
    runway = route_jobs._port_runway_anchor(settings, state, "a,o1", _port((2.5, 6.3), 0.0))
    stub = route_jobs._port_runway_anchor(settings, state, "b,o1", _port((2.5, 9.3), 0.0))
    assert runway is not None and stub is not None
    state.fanout_anchor_by_port_spec = {"a,o1": runway, "b,o1": stub}
    state.port_runway_anchor_specs = frozenset({"a,o1"})
    assert state.fanout_stub_anchor("a,o1") is None
    assert state.fanout_stub_anchor("b,o1") is stub
    assert crossing_plan_stage._resolve_port_footprint_cells(
        settings, state, instance_name="a", port_name="o1", port=_port((2.5, 6.3), 0.0)
    ) == (12, 6)
    assert crossing_plan_stage._resolve_port_footprint_cells(
        settings, state, instance_name="b", port_name="o1", port=_port((2.5, 9.3), 0.0)
    ) == (0, 0)
