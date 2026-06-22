"""Grid helpers for electrical routing."""

from __future__ import annotations

import math
from typing import Iterable

from photonic_router.benchmark_extractor import BBox
from photonic_router.static_obstacle_builder import GridSpec, physical_to_grid

from .types import GridCell


def bbox_to_grid_cells(bbox: BBox, grid: GridSpec) -> frozenset[GridCell]:
    """Return all grid cells touched by a physical bbox."""

    xmin, ymin, xmax, ymax = bbox
    gx_min = math.floor((xmin - grid.origin[0]) / grid.grid_size_um)
    gy_min = math.floor((ymin - grid.origin[1]) / grid.grid_size_um)
    gx_max = math.ceil((xmax - grid.origin[0]) / grid.grid_size_um) - 1
    gy_max = math.ceil((ymax - grid.origin[1]) / grid.grid_size_um) - 1
    gx_min = max(gx_min, 0)
    gy_min = max(gy_min, 0)
    gx_max = min(gx_max, grid.width - 1)
    gy_max = min(gy_max, grid.height - 1)
    if gx_min > gx_max or gy_min > gy_max:
        return frozenset()
    return frozenset(
        (gx, gy)
        for gx in range(gx_min, gx_max + 1)
        for gy in range(gy_min, gy_max + 1)
    )


def disk_cells(center: tuple[float, float], radius_um: float, grid: GridSpec) -> frozenset[GridCell]:
    """Return a clipped Chebyshev disk around a physical point."""

    gx, gy = physical_to_grid(center[0], center[1], grid)
    radius_cells = max(0, math.ceil(radius_um / grid.grid_size_um))
    cells: set[GridCell] = set()
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            nx = gx + dx
            ny = gy + dy
            if 0 <= nx < grid.width and 0 <= ny < grid.height:
                cells.add((nx, ny))
    return frozenset(cells)


def cells_bbox(cells: Iterable[GridCell]) -> tuple[int, int, int, int] | None:
    cells = tuple(cells)
    if not cells:
        return None
    return (
        min(x for x, _ in cells),
        min(y for _, y in cells),
        max(x for x, _ in cells),
        max(y for _, y in cells),
    )
