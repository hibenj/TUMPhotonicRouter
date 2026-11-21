"""`photonic_router.logic_checks.route_jobs`: each statement finds what it names.

The state is a real one: `heater_single` run through the flow up to the
static hand-off. Each test breaks one thing in a copy.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from photonic_router import cli
from photonic_router.logic_checks import count_by_check
from photonic_router.logic_checks.route_jobs import (
    _connected,
    _label_free_areas,
    check_route_jobs,
)
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

pytestmark = pytest.mark.e2e


@pytest.fixture(scope="module")
def session():
    seen = {}

    def observer(stage: str, settings: object, product: object) -> None:
        if stage == "handoff":
            seen["session"] = (settings, product)
            raise StopAfterStage

    with (
        pytest.raises(StopAfterStage),
        observe_stages(observer),
        contextlib.redirect_stdout(io.StringIO()),
    ):
        cli.main(["heater_single"])
    settings, state = seen["session"]
    return settings.schematic, state


def _counts(schematic, state) -> dict[str, int]:
    return count_by_check(check_route_jobs(schematic, state))


def test_the_real_hand_off_is_sound(session) -> None:
    schematic, state = session
    assert len(state.run.route_jobs_by_id) == 7
    assert _counts(schematic, state) == {}


def test_the_jobs_are_the_links_and_the_order_holds_each_once(session) -> None:
    schematic, state = session
    jobs = dict(state.run.route_jobs_by_id)
    del jobs[1]
    assert _counts(schematic, replace(state, run=replace(state.run, route_jobs_by_id=jobs))) == {
        "route_jobs.link_without_job": 1,
        "route_jobs.net_ids_not_consecutive": 1,
        "route_jobs.order_not_a_permutation": 1,
    }
    renamed = dict(state.run.route_jobs_by_id)
    renamed[1] = replace(renamed[1], port2="no_such_port")
    assert _counts(schematic, replace(state, run=replace(state.run, route_jobs_by_id=renamed))) == {
        "route_jobs.job_without_link": 1,
        "route_jobs.link_without_job": 1,
    }


def test_every_job_has_its_states(session) -> None:
    schematic, state = session
    openings = dict(state.run.state_openings_by_id)
    del openings[2]
    broken = replace(state, run=replace(state.run, state_openings_by_id=openings))
    assert _counts(schematic, broken) == {"route_jobs.job_without_openings": 1}


def test_a_state_heads_the_way_its_port_points(session) -> None:
    schematic, state = session
    openings = dict(state.run.state_openings_by_id)
    source = openings[1].source_state
    turned = SimpleNamespace(x=source.x, y=source.y, angle=(int(source.angle) + 2) % 8)
    openings[1] = openings[1]._replace(source_state=turned)
    broken = replace(state, run=replace(state.run, state_openings_by_id=openings))
    (finding,) = check_route_jobs(schematic, broken)
    assert finding.check == "route_jobs.state_heading_differs"
    assert finding.subject.endswith(" source")


def test_a_state_off_the_grid_is_found(session) -> None:
    schematic, state = session
    openings = dict(state.run.state_openings_by_id)
    target = openings[1].target_state
    openings[1] = openings[1]._replace(
        target_state=SimpleNamespace(x=-1, y=target.y, angle=target.angle)
    )
    broken = replace(state, run=replace(state.run, state_openings_by_id=openings))
    assert _counts(schematic, broken) == {"route_jobs.state_off_grid": 1}


def test_a_net_walled_in_is_blocked_and_not_connected(session) -> None:
    schematic, state = session
    grid = state.obstacles.obstacle_map.grid
    everything = [(0, 0, grid.width - 1, grid.height - 1)]
    openings = {
        net_id: opening._replace(opened_cells_set=set(), opened_cells=[])
        for net_id, opening in state.run.state_openings_by_id.items()
    }
    broken = replace(
        state,
        static_cells=replace(state.static_cells, blocked_rects_for_diagnostics=everything),
        run=replace(state.run, state_openings_by_id=openings),
    )
    assert _counts(schematic, broken) == {
        "route_jobs.ends_not_connected": 7,
        "route_jobs.state_blocked": 14,
    }


def test_opened_cells_join_two_free_areas() -> None:
    static = np.zeros((7, 3), dtype=bool)
    static[3, :] = True  # a wall between x 0..2 and x 4..6
    areas = _label_free_areas(static)
    left, right = (1, 1), (5, 1)
    assert not _connected(left, right, set(), areas)
    assert _connected(left, right, {(3, 1)}, areas)
    assert _connected(left, (3, 1), {(3, 1)}, areas)  # an end on an opened cell
    assert not _connected(left, (3, 0), {(3, 1)}, areas)  # an end in the wall
    assert not _connected(left, right, {(3, 1)}, _label_free_areas(_with_wall(static, 4)))


def _with_wall(static: np.ndarray, x: int) -> np.ndarray:
    thicker = static.copy()
    thicker[x, :] = True
    return thicker
