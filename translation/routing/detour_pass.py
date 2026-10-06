"""Phase 7b: rip up every routed net that detoured, and route those nets again together.

Detour-aware routing plan (`.agent/execplans/2026-10-06-detour-aware-routing.md`).
The negotiated loop (`src/engine/negotiation/loop_.rs`) accepts any legal
path, however long, and never revisits a routed net unless a later net rips
it up as a blocker. A net that had to go around a route which a later repair
round moved away keeps its detour; two nets of one MZI arm can even end up
detouring around each other, each a legal "success". This pass measures every
route's length against a free-space lower bound computed from its port
centers alone, rips
every net above ``factor`` times that bound at once, and routes them again in
one negotiated batch, shortest first, against everything else committed. It
repeats while the number of detoured nets shrinks. It is the same batch the
first routing and the crossing repair of `verify_repair.py` run, applied once
more to a chosen set of nets; there is no geometry post-processing.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from translation.route_rust_types import RouteJob
from translation.routing.dispatch import (
    NEGOTIATED_MAX_ROUNDS,
    _clearance_exempt_cells_for_job,
    _foreign_keepout_cleanup_cells_for_job,
    _record_route,
    _routing_endpoint_center_um,
    _state_openings_for_job,
)
from translation.routing.finalize import (
    _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids,
)
from translation.routing.settings import SessionSettings
from translation.routing.stages import RoutedRecords
from translation.routing.state import SessionState

#: a pass stops after this many rounds even if the detoured set still shrinks
MAX_PASSES = 3


def free_lower_bound_um(
    source_center_um: tuple[float, float],
    target_center_um: tuple[float, float],
    bend_radius_um: float,
) -> float:
    """The shortest conceivable route between two port centers, in micrometres.

    The Manhattan distance of the two centers plus the length two 90-degree
    bends take at this radius (`2 * bend_radius_um`, a generous allowance for
    the S-bend a port beside the line of travel needs): a straight shot is
    shorter than this bound only by that allowance, so a route under it is
    never a detour.
    """

    dx = abs(float(target_center_um[0]) - float(source_center_um[0]))
    dy = abs(float(target_center_um[1]) - float(source_center_um[1]))
    return dx + dy + 2.0 * max(0.0, float(bend_radius_um))


def select_detoured_net_ids(
    length_by_net_id: Mapping[int, float],
    bound_by_net_id: Mapping[int, float],
    factor: float,
) -> list[int]:
    """The nets whose route is longer than ``factor`` times their lower bound.

    Sorted by the bound ascending so that a batch of them routes the
    shortest first, ties by net id; a net missing a bound is skipped.
    """

    if factor <= 0:
        return []
    selected = [
        net_id
        for net_id, length_um in length_by_net_id.items()
        if net_id in bound_by_net_id and length_um > factor * bound_by_net_id[net_id]
    ]
    return sorted(selected, key=lambda net_id: (float(bound_by_net_id[net_id]), int(net_id)))


def _route_lengths_um(state: SessionState, net_ids: Iterable[int]) -> dict[int, float]:
    """The recorded route lengths (the bookkeeping's, after endpoint correction)."""

    lengths = state.route_bookkeeping.lengths_by_id
    return {
        int(net_id): float(lengths[int(net_id)]) for net_id in net_ids if int(net_id) in lengths
    }


def _bend_radius_um(settings: SessionSettings, state: SessionState) -> float:
    if settings.bend_radius_um is not None:
        return float(settings.bend_radius_um)
    grid = state.grid
    return float(state.bend_radius_cells) * (float(grid.grid_size_um) if grid is not None else 1.0)


def run_detour_pass(
    settings: SessionSettings, state: SessionState, routed: RoutedRecords
) -> RoutedRecords:
    """Rip up and re-route the detoured nets; returns the refreshed records.

    Raises `RuntimeError` when the re-routing batch does not route every
    ripped net: a pass must never lose a net silently.
    """

    factor = float(settings.detour_pass_factor)
    if factor <= 0:
        return routed
    jobs_by_id: dict[int, RouteJob] = {int(k): v for k, v in state.route_jobs_by_id.items()}
    routed_ids = [
        int(record.net_id) for record in routed.routed_net_records if record.net_id is not None
    ]
    bend_radius_um = _bend_radius_um(settings, state)
    bounds: dict[int, float] = {}
    for net_id in routed_ids:
        job = jobs_by_id.get(net_id)
        if job is None:
            continue
        source_center = _routing_endpoint_center_um(settings, state, job, source=True)
        target_center = _routing_endpoint_center_um(settings, state, job, source=False)
        if source_center is None or target_center is None:
            continue
        bounds[net_id] = free_lower_bound_um(source_center, target_center, bend_radius_um)
    total_reroutes = 0
    previous_count: int | None = None
    for pass_index in range(1, MAX_PASSES + 1):
        lengths = _route_lengths_um(state, bounds)
        detoured = select_detoured_net_ids(lengths, bounds, factor)
        if not detoured or (previous_count is not None and len(detoured) >= previous_count):
            break
        previous_count = len(detoured)
        before_um = sum(lengths[net_id] for net_id in detoured)
        names = ", ".join(
            f"{jobs_by_id[net_id].net_name} ({lengths[net_id]:.1f}/{bounds[net_id]:.1f} um)"
            for net_id in detoured
        )
        print(
            f"      - detour pass {pass_index}: {len(detoured)} net(s) over "
            f"{factor:g}x their free lower bound: {names}; ripping up and re-routing"
        )
        for net_id in detoured:
            state.router.ripup_route(int(net_id))
            state.route_bookkeeping.clear_route(int(net_id))
        batch_jobs = []
        opened_by_id: dict[int, list[tuple[int, int]]] = {}
        for net_id in detoured:
            job = jobs_by_id[net_id]
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(
                settings, state, job
            )
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    _clearance_exempt_cells_for_job(settings, state, job),
                    _foreign_keepout_cleanup_cells_for_job(settings, state, job),
                    _routing_endpoint_center_um(settings, state, job, source=True),
                    _routing_endpoint_center_um(settings, state, job, source=False),
                )
            )
            opened_by_id[int(job.net_id)] = opened_cells
        raw_result = state.router.route_many_with_negotiated_repair_and_commit(
            batch_jobs,
            state.block_radius_cells,
            state.commit_radius_cells,
            state.core_commit_radius_cells,
            NEGOTIATED_MAX_ROUNDS,
            float(state.repair_config.history_weight),
            int(state.repair_config.history_increment),
        )
        result = dict(raw_result)
        if str(result.get("status", "")) != "routed":
            failed = result.get("failed_net_id")
            raise RuntimeError(
                f"detour pass {pass_index} did not route every ripped net "
                f"(status {result.get('status')!r}, failed net id {failed!r}, "
                f"error {result.get('error')!r})"
            )
        rerouted_ids: list[int] = []
        for raw_entry in result.get("routes", []):
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            _record_route(settings, state, jobs_by_id[net_id], entry["route"], opened_by_id[net_id])
            rerouted_ids.append(net_id)
        if settings.enable_checked_endpoint_correction and rerouted_ids:
            _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
                settings, state, rerouted_ids, record_pipeline_timing=False
            )
        after_um = sum(_route_lengths_um(state, detoured).values())
        total_reroutes += len(rerouted_ids)
        print(
            f"      - detour pass {pass_index}: re-routed {len(rerouted_ids)} net(s), "
            f"{before_um:.1f} -> {after_um:.1f} um"
        )
    state.detour_pass_reroutes = total_reroutes
    return RoutedRecords(state.route_bookkeeping.ordered_records(), routed.astar_elapsed_s)
