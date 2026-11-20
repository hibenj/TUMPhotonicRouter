"""What the first two routing phases produce, as frozen records.

Phase 1 (`obstacle_context.py`) returns an `ObstacleContext` and phase 2
(`router_setup.py`) a `KernelSetup`. Neither phase writes to the session
state: the session builds the state from the two products
(`SessionState.after_setup`), and the later phases read them as
`state.obstacles` and `state.kernel`.

Phase 3 (`route_jobs.py`) runs in three steps, each returning one record:
`EndpointIndex`, `FanoutStubs`, `DenseRunways`. A later step reads the
earlier records through the state, so the phase stores each one in its slot
(`state.endpoints`, `state.fanout`, `state.runways`) as soon as the step
returned. A slot is assigned once and as a whole; until then it holds the
record's `empty()` value.

Phase 4 (`crossing_plan_stage.py`) returns the crossing plan and one
`PortFootprints`, which it stores as `state.ports`.

Phase 5 (`handoff.py`) stores `StaticHandoffCells` (`state.static_cells`)
and `RunPlan` (`state.run`).

Nothing rebinds a field of these records after its step returned.

The records are frozen, their members are not all immutable: the kernel
objects are PyO3 handles that the later phases call, and the cell sets are
plain sets because their consumers take sets.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import TYPE_CHECKING, Any

from photonic_router.records.route_rust_types import (
    OpticalRouteClearancePolicy,
    RipupRerouteConfig,
    RouteJob,
)
from photonic_router.static_obstacle_builder import GridSpec, StaticObstacleMapConfig

if TYPE_CHECKING:  # annotation-only: `route_jobs` imports this module
    from gdsfactory.typings import Port

    from photonic_router.routing.route_jobs import FanoutAnchor


@dataclass(frozen=True)
class ObstacleContext:
    """Phase 1's product: the static obstacle map and what was resolved with it."""

    #: the obstacle builder's result object
    obstacle_map: Any
    #: how the crossing footprint was resolved, for the crossing plan's record
    crossing_device_info: dict[str, Any]
    #: the debug SVG of the static obstacles; None without a debug directory
    obstacle_svg: Path | None
    #: the routing grid of the static obstacle map
    grid: GridSpec
    #: the obstacle configuration actually used, its obstacle layers resolved
    resolved_obstacle_config: StaticObstacleMapConfig
    #: the crossing footprint's half size in cells after resolution
    resolved_crossing_half_size_cells: int
    #: the debug artifact directory; None writes no artifacts
    debug_path: Path | None

    @property
    def diagnostics_enabled(self) -> bool:
        """Whether debug diagnostics are collected, which a debug directory decides."""
        return self.debug_path is not None


@dataclass(frozen=True)
class GridFrame:
    """The routing grid as the kernel was given it."""

    grid_width: int
    grid_height: int
    origin_x_um: float
    origin_y_um: float


@dataclass(frozen=True)
class ClearanceRadii:
    """The radii, in cells, derived from the route width, the clearance and the grid."""

    #: the policy the four radii below are read from
    clearance_policy: OpticalRouteClearancePolicy
    #: the clearance of the resolved obstacle configuration
    route_clearance_um: float
    #: the bend radius after the primitive library rounded it
    bend_radius_cells: int
    #: keep-out radius committed around a routed net
    commit_radius_cells: int
    #: the waveguide's own radius within a committed route
    core_commit_radius_cells: int
    #: how far the search stays away from dynamic obstacles
    block_radius_cells: int


@dataclass(frozen=True)
class PortLanes:
    """The size of the access lane reserved in front of a port, in cells."""

    port_lane_length_cells: int
    port_lane_half_width_cells: int
    #: the two values for the stub ports of a dense fan-out instance
    stub_port_lane_length_cells: int
    stub_port_lane_half_width_cells: int


@dataclass(frozen=True)
class RawStaticGeometry:
    """The static blockers of the obstacle map before any port was opened."""

    #: blocked cells before clearance inflation
    raw_static_cells: set[tuple[int, int]]
    #: the same geometry as rectangles, for port openings
    raw_static_rects_for_openings: list[tuple[int, int, int, int]]
    #: raw plus heater-layer rectangles, for heater openings
    heater_opening_rects_for_openings: list[tuple[int, int, int, int]]


def _rect_ranges_by_y(
    rects: Iterable[tuple[int, int, int, int]], grid_width: int, grid_height: int
) -> dict[int, list[tuple[int, int]]]:
    """Index rectangles by row: per y the merged, sorted x ranges inside the grid."""
    ranges_by_y: dict[int, list[tuple[int, int]]] = {}
    for rect_min_x, rect_min_y, rect_max_x, rect_max_y in rects:
        min_x = max(0, rect_min_x)
        max_x = min(grid_width - 1, rect_max_x)
        min_y = max(0, rect_min_y)
        max_y = min(grid_height - 1, rect_max_y)
        if min_x > max_x or min_y > max_y:
            continue
        for y in range(min_y, max_y + 1):
            ranges_by_y.setdefault(y, []).append((min_x, max_x))
    for y, ranges in list(ranges_by_y.items()):
        ranges.sort()
        merged_ranges: list[tuple[int, int]] = []
        for min_x, max_x in ranges:
            if not merged_ranges or min_x > merged_ranges[-1][1] + 1:
                merged_ranges.append((min_x, max_x))
            else:
                prev_min_x, prev_max_x = merged_ranges[-1]
                merged_ranges[-1] = (prev_min_x, max(prev_max_x, max_x))
        ranges_by_y[y] = merged_ranges
    return ranges_by_y


@dataclass(frozen=True)
class KernelSetup:
    """Phase 2's product: the kernel's router and everything derived for it."""

    #: the kernel's `PyPhotonicRouter` (PyO3)
    router: Any
    #: the kernel's `PrimitiveLibraryConfig` (PyO3)
    primitive_cfg: Any
    #: the kernel's `AStarConfig` (PyO3)
    astar_cfg: Any
    frame: GridFrame
    radii: ClearanceRadii
    lanes: PortLanes
    static: RawStaticGeometry

    # The three row indexes below are derived from `static` and `frame` alone.
    # Phase 4 needs them for some designs only, so each is built on first use.

    @cached_property
    def raw_static_cells_by_y(self) -> dict[int, set[int]]:
        """The raw static cells as the set of x per row."""
        cells_by_y: dict[int, set[int]] = {}
        for cell_x, cell_y in self.static.raw_static_cells:
            cells_by_y.setdefault(int(cell_y), set()).add(int(cell_x))
        return cells_by_y

    @cached_property
    def raw_static_rect_ranges_by_y(self) -> dict[int, list[tuple[int, int]]]:
        """The raw static rectangles as merged x ranges per row."""
        return _rect_ranges_by_y(
            self.static.raw_static_rects_for_openings,
            self.frame.grid_width,
            self.frame.grid_height,
        )

    @cached_property
    def heater_opening_rect_ranges_by_y(self) -> dict[int, list[tuple[int, int]]]:
        """The raw and heater-layer rectangles as merged x ranges per row."""
        return _rect_ranges_by_y(
            self.static.heater_opening_rects_for_openings,
            self.frame.grid_width,
            self.frame.grid_height,
        )


@dataclass(frozen=True)
class EndpointIndex:
    """Phase 3, step 1: the ports the nets end on, indexed for the fan-out rules."""

    #: `"inst,port"` -> (instance, port name, port)
    ports_by_spec: dict[str, tuple[str, str, Port]]
    source_specs_by_instance: dict[str, set[str]]
    target_specs_by_instance: dict[str, set[str]]
    #: source port specs per (instance, outgoing angle)
    source_specs_by_instance_angle: dict[tuple[str, int], set[str]]
    #: target port specs per (instance, incoming angle)
    target_specs_by_instance_angle: dict[tuple[str, int], set[str]]

    @classmethod
    def empty(cls) -> EndpointIndex:
        """The index of a session whose phase 3 has not run."""
        return cls({}, {}, {}, {}, {})


@dataclass(frozen=True)
class FanoutStubs:
    """Phase 3, step 2: the static fan-out stubs and the nets that use them."""

    #: the stub's anchor per port spec; a route to that port starts there
    anchor_by_port_spec: dict[str, FanoutAnchor]
    #: nets with a stub on either side, on the source side, on the target side
    anchor_net_ids: set[int]
    anchor_source_net_ids: set[int]
    anchor_target_net_ids: set[int]
    #: all cells the stubs occupy, inflated by the commit radius
    stub_static_cells: set[tuple[int, int]]
    #: the centerline cells of the stubs
    stub_center_cells: set[tuple[int, int]]
    #: the inflated cells per port spec
    stub_static_cells_by_spec: dict[str, set[tuple[int, int]]]

    @classmethod
    def empty(cls) -> FanoutStubs:
        """The stubs of a session whose phase 3 has not run."""
        return cls({}, set(), set(), set(), set(), set(), {})


@dataclass(frozen=True)
class DenseRunways:
    """Phase 3, step 3: the staggered runways of the dense ports and their clusters."""

    #: runway length in cells per dense source port spec
    source_length_by_spec: dict[str, int]
    #: runway length in cells per dense target port spec
    target_length_by_spec: dict[str, int]
    #: the dense fan-out cluster each source port belongs to
    source_cluster_specs_by_port_spec: dict[str, set[str]]

    @classmethod
    def empty(cls) -> DenseRunways:
        """The runways of a session whose phase 3 has not run."""
        return cls({}, {}, {})


@dataclass(frozen=True)
class PortFootprints:
    """Phase 4's record: what every port opens, keeps clear and is offset by."""

    #: cells opened for each port's access (the footprint minus static geometry)
    access_cells_by_spec: dict[str, set[tuple[int, int]]]
    #: cells a port's access rule may open (the whole footprint)
    access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]]
    #: the name of the access rule applied per port spec; None for the default
    access_rule_by_spec: dict[str, str | None]
    #: the approach lane of each port
    runway_cells_by_spec: dict[str, set[tuple[int, int]]]
    #: the union of all runways
    normal_runway_cells: set[tuple[int, int]]
    #: cells kept clear of foreign nets per port; empty when the keep-out is off
    foreign_keepout_cells_by_spec: dict[str, set[tuple[int, int]]]
    #: per dense port: (lateral x, lateral y, lower, upper, lane margin in um)
    dense_lateral_windows: dict[str, tuple[float, float, float, float, float]]
    #: per dense port: the lateral direction and the ports sharing its window
    dense_lateral_owner_groups: dict[str, tuple[float, float, tuple[tuple[str, float], ...]]]
    #: the lane offset of each (port spec, as_target) that shares a grid state
    state_lane_offsets: dict[tuple[str, bool], tuple[int, int]]

    @classmethod
    def empty(cls) -> PortFootprints:
        """The footprints of a session whose phase 4 has not run."""
        return cls({}, {}, {}, {}, set(), {}, {}, {}, {})


#: per net, as `handoff.states_and_openings` returns it: the source state, the
#: target state (kernel `State` handles), the candidate cells the net may
#: open, the cells it opens, and those cells as a sorted list
StateOpenings = tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]


@dataclass(frozen=True)
class StaticHandoffCells:
    """Phase 5: the static cells the kernel was given beyond the obstacle map."""

    #: the foreign-port keep-out cells of all instances
    foreign_keepout_cells: set[tuple[int, int]]
    #: runways, foreign keep-outs and fan-out stubs, for the debug artifacts
    debug_port_keepout_cells: set[tuple[int, int]]
    #: the raw static cells plus those reservations
    blocked_cells_before_port_reservations: set[tuple[int, int]]
    #: the static blocker rectangles and the die keep-out, for diagnostics
    blocked_rects_for_diagnostics: list[tuple[int, int, int, int]]

    @classmethod
    def empty(cls) -> StaticHandoffCells:
        """The cells of a session whose phase 5 has not run."""
        return cls(set(), set(), set(), [])


@dataclass(frozen=True)
class RunPlan:
    """Phase 5: the jobs of this run and what was precomputed for them."""

    #: net ids in the order the kernel routes them
    route_order: list[int]
    #: the jobs of this run by net id
    route_jobs_by_id: dict[int, RouteJob]
    #: every job by route index, including those a debug limit cut off
    full_route_jobs_by_route_index: dict[int, RouteJob]
    state_openings_by_id: dict[int, StateOpenings]
    #: the clearance-exempt cells the kernel computed per net
    clearance_exempt_cells_by_id: dict[int, list[tuple[int, int]]]
    #: (width, height, cell size, origin x, origin y) for realization; a grid
    #: of no cells until phase 5 ran
    realization_grid_spec: tuple[int, int, float, float, float]
    #: the rip-up and reroute configuration the loop runs with
    repair_config: RipupRerouteConfig
    #: whether per-net timing buckets are filled
    collect_timing: bool
    #: whether committed dynamic cells are tracked per net
    track_dynamic_cells: bool

    @classmethod
    def empty(cls) -> RunPlan:
        """The plan of a session whose phase 5 has not run."""
        return cls([], {}, {}, {}, {}, (0, 0, 0.0, 0.0, 0.0), RipupRerouteConfig(), False, False)
