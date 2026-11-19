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

Nothing rebinds a field of these records after its step returned.

The records are frozen, their members are not all immutable: the kernel
objects are PyO3 handles that the later phases call, and the cell sets are
plain sets because their consumers take sets.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from photonic_router.records.route_rust_types import OpticalRouteClearancePolicy
from photonic_router.static_obstacle_builder import GridSpec

if TYPE_CHECKING:  # annotation-only: `route_jobs` imports this module
    from gdsfactory.typings import Port

    from photonic_router.routing.route_jobs import _FanoutAnchor


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
    #: the obstacle configuration actually used: a `StaticObstacleMapConfig`,
    #: or the caller's own configuration object when it passed one
    resolved_obstacle_config: object
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
    anchor_by_port_spec: dict[str, _FanoutAnchor]
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
