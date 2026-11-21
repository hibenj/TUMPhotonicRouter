"""Stage "obstacle map": phase 1's static obstacle map of a routing session.

`check_obstacle_map` reads the `StaticObstacleMapData` of the session
(`ObstacleContext.obstacle_map`), its resolved configuration and the links
of the schematic. Cells are grid cells `(x, y)`, rectangles are
`(x_min, y_min, x_max, y_max)` in cells with both ends included; lengths
are in um.

A cell is *blocked* when it is in `blocked_cells` or under one of the
`blocked_static_rects`; the session may run without materialized rectangle
cells, so the set alone is not the blocked area. The *keep-out band* is the
`chip_boundary_rects`.

Errors (the map contradicts itself or the design):

- `obstacle_map.grid_misses_die`: the grid does not cover its own die box
- `obstacle_map.die_misses_design`: the die box does not hold the extracted
  design (polygons and ports)
- `obstacle_map.routable_box_outside_die`
- `obstacle_map.cell_outside_grid`: a blocked cell, an opened cell or a
  rectangle lies outside the grid
- `obstacle_map.obstacle_not_blocked`: a cell under the bounding box of an
  obstacle polygon is not blocked (bounding-box mode only)
- `obstacle_map.keep_out_inside_routable_box`: the keep-out band covers a
  cell whose centre is inside the routable box
- `obstacle_map.linked_port_not_extracted`: a link names a port the
  obstacle extraction does not list
- `obstacle_map.port_exit_blocked`: the cell in front of a linked port is
  off the grid, in the keep-out band, or blocked without being one of the
  port-open cells
- `obstacle_map.stats_disagree`: a count of `build_stats` is not the count
  of the data it describes (ports, port-open cells, polygons, rectangles,
  blocked cells)

Notes:

- `obstacle_map.routable_cell_outside_routable_box`: cells whose centre is
  outside the routable box and that the keep-out band does not cover (the
  band is rounded outward, so a cell straddling the box edge stays
  routable); one finding with the number of cells
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding
from photonic_router.logic_checks.reading import schematic_links

TOLERANCE_UM = 1e-6

#: `build_stats` keys and what they are compared with
_STAT_KEYS = (
    "port_count",
    "port_open_cell_count",
    "polygon_count",
    "static_box_count",
    "blocked_cell_count",
)


class _Grid:
    """The grid of an obstacle map with its masks."""

    def __init__(self, obstacle_map: Any) -> None:
        grid = obstacle_map.grid
        self.width = int(grid.width)
        self.height = int(grid.height)
        self.size_um = float(grid.grid_size_um)
        self.origin_x, self.origin_y = (float(value) for value in grid.origin)
        self.outside: list[str] = []
        self.blocked = self._mask(obstacle_map.blocked_cells, obstacle_map.blocked_static_rects)
        self.opened = self._mask(obstacle_map.port_open_cells, ())
        self.keep_out = self._mask((), obstacle_map.chip_boundary_rects)

    def _mask(self, cells: Any, rects: Any) -> np.ndarray:
        mask = np.zeros((self.width, self.height), dtype=bool)
        for x, y in cells:
            if self.contains((x, y)):
                mask[x, y] = True
            else:
                self.outside.append(f"cell ({x}, {y})")
        for x_min, y_min, x_max, y_max in rects:
            if self.contains((x_min, y_min)) and self.contains((x_max, y_max)):
                mask[x_min : x_max + 1, y_min : y_max + 1] = True
            else:
                self.outside.append(f"rectangle ({x_min}, {y_min}, {x_max}, {y_max})")
        return mask

    def contains(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cell_at(self, x_um: float, y_um: float) -> tuple[int, int]:
        """The cell that holds a point; a point on a cell edge belongs to the upper cell."""
        return (
            math.floor((x_um - self.origin_x) / self.size_um),
            math.floor((y_um - self.origin_y) / self.size_um),
        )

    @property
    def extent(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x,
            self.origin_y,
            self.origin_x + self.width * self.size_um,
            self.origin_y + self.height * self.size_um,
        )


def check_obstacle_map(schematic: Any, obstacle_map: Any, config: Any) -> list[Finding]:
    """The statements about one session's static obstacle map.

    `config` is the resolved `StaticObstacleMapConfig` the map was built
    with (`ObstacleContext.resolved_obstacle_config`).
    """
    grid = _Grid(obstacle_map)
    findings = [
        Finding("obstacle_map.cell_outside_grid", ERROR, subject) for subject in grid.outside
    ]
    findings.extend(_box_findings(obstacle_map, grid))
    if str(config.obstacle_mode) == "bounding_boxes":
        findings.extend(_unblocked_obstacles(obstacle_map, grid))
    findings.extend(_keep_out_findings(obstacle_map, grid))
    findings.extend(_port_findings(schematic, obstacle_map, grid))
    findings.extend(_stats_findings(obstacle_map, grid))
    return findings


def _box_findings(obstacle_map: Any, grid: _Grid) -> list[Finding]:
    findings: list[Finding] = []
    die = tuple(float(value) for value in obstacle_map.grid.die_bbox)
    if not _box_holds(grid.extent, die):
        findings.append(
            Finding("obstacle_map.grid_misses_die", ERROR, "grid", f"{grid.extent} / {die}")
        )
    design = tuple(float(value) for value in obstacle_map.benchmark.bbox)
    if not _box_holds(die, design):
        findings.append(
            Finding("obstacle_map.die_misses_design", ERROR, "die", f"{die} / {design}")
        )
    routable = obstacle_map.routable_bbox
    if routable is not None and not _box_holds(die, tuple(float(v) for v in routable)):
        findings.append(
            Finding("obstacle_map.routable_box_outside_die", ERROR, "die", f"{die} / {routable}")
        )
    return findings


def _unblocked_obstacles(obstacle_map: Any, grid: _Grid) -> list[Finding]:
    findings: list[Finding] = []
    for index, polygon in enumerate(obstacle_map.benchmark.polygons):
        xs = [float(point[0]) for point in polygon]
        ys = [float(point[1]) for point in polygon]
        low = grid.cell_at(min(xs), min(ys))
        # the upper edge of the box belongs to the cell below it
        high = grid.cell_at(max(xs) - TOLERANCE_UM, max(ys) - TOLERANCE_UM)
        x_min, y_min = max(low[0], 0), max(low[1], 0)
        if not grid.blocked[x_min : high[0] + 1, y_min : high[1] + 1].all():
            findings.append(
                Finding(
                    "obstacle_map.obstacle_not_blocked",
                    ERROR,
                    f"polygon {index}",
                    f"cells ({x_min}, {y_min})..({high[0]}, {high[1]})",
                )
            )
    return findings


def _keep_out_findings(obstacle_map: Any, grid: _Grid) -> list[Finding]:
    routable = obstacle_map.routable_bbox
    if routable is None:
        return []
    centres_x = grid.origin_x + (np.arange(grid.width) + 0.5) * grid.size_um
    centres_y = grid.origin_y + (np.arange(grid.height) + 0.5) * grid.size_um
    inside = np.outer(
        (centres_x >= routable[0]) & (centres_x <= routable[2]),
        (centres_y >= routable[1]) & (centres_y <= routable[3]),
    )
    findings: list[Finding] = []
    covered_inside = int((grid.keep_out & inside).sum())
    if covered_inside:
        findings.append(
            Finding(
                "obstacle_map.keep_out_inside_routable_box",
                ERROR,
                "keep-out band",
                f"{covered_inside} cell(s)",
            )
        )
    free_outside = int((~grid.keep_out & ~inside).sum())
    if free_outside:
        findings.append(
            Finding(
                "obstacle_map.routable_cell_outside_routable_box",
                NOTE,
                "keep-out band",
                f"{free_outside} cell(s)",
            )
        )
    return findings


def _port_findings(schematic: Any, obstacle_map: Any, grid: _Grid) -> list[Finding]:
    findings: list[Finding] = []
    ports = {str(port.name): port for port in obstacle_map.benchmark.ports}
    for net, first, second in schematic_links(schematic):
        for end in (first, second):
            port = ports.get(end)
            if port is None or port.orientation is None:
                findings.append(Finding("obstacle_map.linked_port_not_extracted", ERROR, net, end))
                continue
            angle = math.radians(float(port.orientation))
            ahead = grid.cell_at(
                float(port.position[0]) + round(math.cos(angle)) * grid.size_um,
                float(port.position[1]) + round(math.sin(angle)) * grid.size_um,
            )
            reason = _exit_block(grid, ahead)
            if reason is not None:
                findings.append(
                    Finding("obstacle_map.port_exit_blocked", ERROR, end, f"{ahead}: {reason}")
                )
    return findings


def _exit_block(grid: _Grid, cell: tuple[int, int]) -> str | None:
    if not grid.contains(cell):
        return "off the grid"
    if grid.keep_out[cell]:
        return "in the keep-out band"
    if grid.blocked[cell] and not grid.opened[cell]:
        return "blocked and not opened"
    return None


def _stats_findings(obstacle_map: Any, grid: _Grid) -> list[Finding]:
    stats = obstacle_map.build_stats
    if not isinstance(stats, dict):
        return []
    actual = {
        "port_count": len(obstacle_map.benchmark.ports),
        "port_open_cell_count": len(obstacle_map.port_open_cells),
        "polygon_count": len(obstacle_map.benchmark.polygons),
        "static_box_count": len(obstacle_map.blocked_static_rects),
        "blocked_cell_count": int(grid.blocked.sum()),
    }
    return [
        Finding(
            "obstacle_map.stats_disagree",
            ERROR,
            key,
            f"build_stats says {stats[key]}, the map has {actual[key]}",
        )
        for key in _STAT_KEYS
        if key in stats and int(stats[key]) != actual[key]
    ]


def _box_holds(outer: tuple[float, ...], inner: tuple[float, ...]) -> bool:
    return (
        outer[0] <= inner[0] + TOLERANCE_UM
        and outer[1] <= inner[1] + TOLERANCE_UM
        and outer[2] >= inner[2] - TOLERANCE_UM
        and outer[3] >= inner[3] - TOLERANCE_UM
    )
