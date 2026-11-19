"""What the first two routing phases produce, as frozen records.

Phase 1 (`obstacle_context.py`) returns an `ObstacleContext` and phase 2
(`router_setup.py`) a `KernelSetup`. Neither phase writes to the session
state: the session builds the state from the two products
(`SessionState.after_setup`), and the later phases read them as
`state.obstacles` and `state.kernel`. Nothing rebinds a field of these
records after its phase returned.

The records are frozen, their members are not all immutable: the kernel
objects are PyO3 handles that the later phases call, and the cell sets are
plain sets because their consumers take sets.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from photonic_router.records.route_rust_types import OpticalRouteClearancePolicy
from photonic_router.static_obstacle_builder import GridSpec


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
