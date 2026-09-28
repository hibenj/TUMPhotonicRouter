"""Shared private helpers for tests/electrical/*.

Factored out of the pre-split ``tests/test_electrical_routing.py`` (Milestone
4, slice 5) so no test module under ``tests/electrical/`` needs to define a
helper another module also uses.
"""

from __future__ import annotations

from types import SimpleNamespace

import klayout.db as kdb
from gdsfactory.component import Component

from translation.electrical.types import (
    BusStripe,
    ElectricalObstacleMap,
    ElectricalPortRef,
    ElectricalTerminal,
)


def _dbu_bbox(bbox_um):
    return tuple(round(value * 1000) for value in bbox_um)


def _polygon_bboxes_by_layer(component: Component, layer: tuple[int, int]):
    return {
        (
            polygon.bbox().left,
            polygon.bbox().bottom,
            polygon.bbox().right,
            polygon.bbox().top,
        )
        for polygon in component.get_polygons(by="tuple").get(layer, [])
    }


def _kdb_region(*args):
    return getattr(kdb, "Region")(*args)


def _kdb_box(*args):
    return getattr(kdb, "Box")(*args)


def _polygon_region_by_layer(component: Component, layer: tuple[int, int]):
    region = _kdb_region()
    for polygon in component.get_polygons(merge=False, by="tuple").get(layer, []):
        region.insert(polygon)
    return region


def _box_region(bbox_um):
    return _kdb_region(_kdb_box(*_dbu_bbox(bbox_um)))


def _region_covers_bbox(region, bbox_um) -> bool:
    box = _box_region(bbox_um)
    return (box - region).is_empty()


def _bbox_contains(outer, inner) -> bool:
    return (
        outer[0] <= inner[0]
        and outer[1] <= inner[1]
        and outer[2] >= inner[2]
        and outer[3] >= inner[3]
    )


def _verification_obstacle_map(
    *,
    raw_obstacle_bboxes: tuple[tuple[float, float, float, float], ...] = (),
) -> ElectricalObstacleMap:
    grid = SimpleNamespace(
        width=40,
        height=40,
        grid_size_um=10.0,
        origin=(0.0, 0.0),
    )
    bus = BusStripe(
        side="bottom",
        bbox=(0.0, 0.0, 40.0, 10.0),
        cells=frozenset({(0, 0), (1, 0), (2, 0), (3, 0)}),
    )
    return ElectricalObstacleMap(
        grid=grid,
        raw_blocked_cells=frozenset(),
        blocked_cells=frozenset(),
        terminal_open_cells={},
        bus=bus,
        die_bbox=(0.0, 0.0, 400.0, 400.0),
        layout_bbox=(0.0, 0.0, 200.0, 200.0),
        raw_obstacle_bboxes=raw_obstacle_bboxes,
    )


def _terminal(terminal_id: str, center: tuple[float, float]) -> ElectricalTerminal:
    port = ElectricalPortRef(
        name="e1",
        center=center,
        orientation=None,
        width=4.0,
        layer=(49, 0),
    )
    return ElectricalTerminal(
        id=terminal_id,
        heater_id=terminal_id.split(":", 1)[0],
        side_key=terminal_id.split(":", 1)[1],
        center=center,
        bbox=(
            center[0] - 2.0,
            center[1] - 2.0,
            center[0] + 2.0,
            center[1] + 2.0,
        ),
        ports=(port,),
        layer=(49, 0),
    )


def _path_direction_changes(path: tuple[tuple[int, int], ...]) -> int:
    """Count direction changes along a grid-cell path (0 for a straight run)."""

    steps = [(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(path, path[1:])]
    return sum(1 for previous, current in zip(steps, steps[1:]) if previous != current)


def _column_trunk_vertical_cells(path: tuple[tuple[int, int], ...]) -> set[tuple[int, int]]:
    """The path's vertical-run cells: every cell sharing the path's last x.

    By construction (a per-member stub joining a shared trunk column) the
    path's last cell is always on the trunk column, so this isolates the
    vertical run from any horizontal stub.
    """

    trunk_x = path[-1][0]
    return {cell for cell in path if cell[0] == trunk_x}
