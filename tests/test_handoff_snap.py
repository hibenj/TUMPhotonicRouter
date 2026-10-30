"""The handoff's collinear snap: a snapped endpoint stays inside its port's dense-port opening.

Private-circuits plan, 2026-10-06: in the sender-and-receiver cut-out a
coupler sits 1 um above an MMI input whose sibling input is 2 um below it.
Snapping the MMI input onto the coupler's row put the target into the
sibling's half of the runway, which this net never opens; the route could
not reach it. The source is snapped onto the target's row instead.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from photonic_router import _rust
from photonic_router.routing.handoff import _snap_nearly_collinear_states
from photonic_router.static_obstacle_builder import GridSpec

pytestmark = pytest.mark.integration


def _state(windows: dict | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        grid=GridSpec(width=400, height=600, grid_size_um=1.0, origin=(0.0, 0.0), die_bbox=None),
        rust_backend=_rust,
        dense_port_lateral_windows=windows or {},
        dense_port_lateral_owner_groups={},
    )


def _port(x: float, y: float) -> SimpleNamespace:
    return SimpleNamespace(center=(x, y))


def _cells(*states) -> list[tuple[int, int, int]]:
    return [(int(s.x), int(s.y), int(s.angle)) for s in states]


def test_target_snaps_onto_the_source_row_without_a_dense_window():
    state = _state()
    source, target = _rust.State(163, 450, 0), _rust.State(334, 449, 0)
    new_source, new_target, _ = _snap_nearly_collinear_states(
        None, state, source, target, _port(163.5, 450.5), _port(334.5, 449.5), "gc,o1", "mmi,o1"
    )
    assert _cells(new_source, new_target) == [(163, 450, 0), (334, 450, 0)]


def test_source_snaps_instead_when_the_target_row_belongs_to_the_sibling_input():
    # the MMI's lower input opens y up to 450.4 (rows up to 449), the
    # upper one from 450.4 on; the source row 450 is the upper input's
    windows = {
        "mmi,o1": (0.0, -1.0, -450.4, -448.4, 0.0),
        "mmi,o2": (0.0, -1.0, -452.4, -450.4, 0.0),
    }
    state = _state(windows)
    source, target = _rust.State(163, 450, 0), _rust.State(334, 449, 0)
    new_source, new_target, cells = _snap_nearly_collinear_states(
        None, state, source, target, _port(163.5, 450.5), _port(334.5, 449.5), "gc,o1", "mmi,o1"
    )
    assert _cells(new_source, new_target) == [(163, 449, 0), (334, 449, 0)]
    assert cells == {(163, 450), (334, 449)}  # the original cells are still opened


def test_nothing_snaps_when_both_rows_are_owned_by_other_ports():
    windows = {
        "mmi,o1": (0.0, -1.0, -450.4, -448.4, 0.0),
        "gc,o1": (0.0, -1.0, -451.4, -450.4, 0.0),
    }
    state = _state(windows)
    source, target = _rust.State(163, 450, 0), _rust.State(334, 449, 0)
    new_source, new_target, _ = _snap_nearly_collinear_states(
        None, state, source, target, _port(163.5, 450.5), _port(334.5, 449.5), "gc,o1", "mmi,o1"
    )
    assert _cells(new_source, new_target) == [(163, 450, 0), (334, 449, 0)]


def test_vertical_headings_snap_the_column_the_same_way():
    windows = {"mmi,o1": (-1.0, 0.0, -200.4, -198.4, 0.0)}
    state = _state(windows)
    source, target = _rust.State(200, 10, 2), _rust.State(199, 300, 2)
    new_source, new_target, _ = _snap_nearly_collinear_states(
        None, state, source, target, _port(200.5, 10.5), _port(199.5, 300.5), "gc,o1", "mmi,o1"
    )
    assert _cells(new_source, new_target) == [(199, 10, 2), (199, 300, 2)]
