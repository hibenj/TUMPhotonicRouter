"""Phase 3: the route jobs, the port-access tables, the dense fan-out anchors and their runways."""

from __future__ import annotations

import math
import sys

from dataclasses import dataclass
from typing import Any, Iterable, Mapping

from gdsfactory.typings import Port

from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    find_component_port_access_rule,
)
from photonic_router.static_obstacle_builder import grid_cell_center

from translation.route_gds import get_port_from_instance
from translation.route_rust_crossing_plan import _port_center_um
from translation.route_rust_geometry import _compress_centerline, _physical_point_to_grid_cell
from translation.route_rust_obstacle_config import (
    _port_type_name,
    _schematic_instance_component_name,
)
from translation.route_rust_types import RouteJob

@dataclass(frozen=True)
class _FanoutAnchor:
    port_spec: str
    state_x: int
    state_y: int
    physical_angle: int
    center_um: tuple[float, float]
    stub_center_cells: tuple[tuple[int, int], ...]
    stub_centerline_um: tuple[tuple[float, float], ...]


def _orientation_to_angle(session, orientation: float | None, *, flip: bool = False) -> int:
    if orientation is None:
        angle = 0
    else:
        angle = int(round((float(orientation) % 360.0) / 45.0)) % 8

    if flip:
        angle = (angle + 4) % 8

    return angle


def _angle_to_step(session, angle: int) -> tuple[int, int]:
    steps = [
        (1, 0),  # 0 east
        (1, 1),  # 1 northeast
        (0, 1),  # 2 north
        (-1, 1),  # 3 northwest
        (-1, 0),  # 4 west
        (-1, -1),  # 5 southwest
        (0, -1),  # 6 south
        (1, -1),  # 7 southeast
    ]
    return steps[angle % 8]


def _in_bounds(session, gx: int, gy: int) -> bool:
    return 0 <= gx < int(session.grid.width) and 0 <= gy < int(session.grid.height)


def port_to_grid_state(
    session,
    port: Port,
    grid_origin_x_um: float,
    grid_origin_y_um: float,
    grid_size_um: float,
    *,
    as_target: bool = False,
    outward_cells: int = 1,
):
    port_angle = session._orientation_to_angle(port.orientation, flip=False)

    # For choosing the grid cell, always move outward from the physical port.
    # This avoids starting inside the real component/port geometry.
    sx, sy = session._angle_to_step(port_angle)

    x = float(port.center[0]) + sx * outward_cells * grid_size_um
    y = float(port.center[1]) + sy * outward_cells * grid_size_um

    gx = int((x - grid_origin_x_um) // grid_size_um)
    gy = int((y - grid_origin_y_um) // grid_size_um)

    # For the route state angle:
    # - source: route leaves the port outward
    # - target: route approaches the port, so flip direction
    route_angle = session._orientation_to_angle(port.orientation, flip=as_target)

    return session.rust_backend.State(gx, gy, route_angle)


def _port_access_rule_for(
    session,
    *,
    instance_name: str,
    port_name: str,
    port: Port,
) -> ComponentPortAccessRule | None:
    """The component access rule governing this port, if any.

    Returns the rule itself (not a projection of it) so every consumer --
    runway sizing, dense-group exclusion, the instance-geometry opening --
    reads the same declaration.
    """
    return find_component_port_access_rule(
        component_name=_schematic_instance_component_name(session.schematic, instance_name),
        port_name=port_name,
        port_type=_port_type_name(port),
    )


def _dense_fanout_min_ports(session) -> int:
    """Smallest same-instance, same-angle port group treated as a dense fanout.

    Default 3 (the historical `> 2`). `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`
    overrides it; the pre-placed crossing-grid flow uses 2 so a 2x2
    switch's port pair (1.25 um apart, inside one routing cell) gets the
    same staggered static stubs a multiport MMI's port row gets.
    """
    value = session.config.fanout.dense_fanout_min_ports
    return 3 if value is None else value


def _dense_fanout_group_size(session, port_specs: set[str]) -> int:
    """Ports of a group that qualify for automatic dense-fanout handling.

    A port with an explicit component access rule (see
    `_port_access_rule_for`) has had its access geometry decided
    deliberately and is left out, so components that manage their own
    approach (e.g. pre-placed crossing grids) never get automatic stubs
    or runways. Looked up directly rather than through
    `port_access_rule_by_spec`, which is only populated after the anchor
    passes that call this.
    """
    count = 0
    for spec in port_specs:
        endpoint = session.endpoint_ports_by_spec.get(spec)
        if endpoint is None:
            count += 1
            continue
        instance_name, port_name, port = endpoint
        rule = session._port_access_rule_for(
            instance_name=instance_name, port_name=port_name, port=port
        )
        if rule is None:
            count += 1
    return count


def _dense_fanout_min_ports_for(session, instance_name: str) -> int:
    """Per-instance threshold: instances listed in
    `PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES` (comma separated) get static
    stubs from two ports up -- the pre-placed crossing column grid needs
    every crossing lane of a multiport MMI on a spread row, also when
    only two of its outputs cross in a layer -- while everything else
    (e.g. the 1x2 splitter tree) keeps the global threshold."""
    instances = session.config.fanout.dense_fanout_instances
    if instances and instance_name in instances:
        return 2
    return session._dense_fanout_min_ports()


def _is_dense_source_fanout_instance(session, instance_name: str) -> bool:
    return any(
        session._dense_fanout_group_size(port_specs)
        >= session._dense_fanout_min_ports_for(instance_name)
        for (
            group_instance,
            _angle,
        ), port_specs in session.source_port_specs_by_instance_angle.items()
        if group_instance == instance_name
    )


def _is_dense_source_fanout_group(session, instance_name: str, angle: int) -> bool:
    return (
        session._dense_fanout_group_size(
            session.source_port_specs_by_instance_angle.get((instance_name, int(angle)), set())
        )
        >= session._dense_fanout_min_ports_for(instance_name)
    )


def _is_dense_target_fanout_instance(session, instance_name: str) -> bool:
    return any(
        session._dense_fanout_group_size(port_specs) >= session._dense_fanout_min_ports()
        for (
            group_instance,
            _angle,
        ), port_specs in session.target_port_specs_by_instance_angle.items()
        if group_instance == instance_name
    )


def _is_dense_target_fanout_group(session, instance_name: str, angle: int) -> bool:
    return (
        session._dense_fanout_group_size(
            session.target_port_specs_by_instance_angle.get((instance_name, int(angle)), set())
        )
        >= session._dense_fanout_min_ports()
    )


def _grid_cell_center_um(session, cell_x: int, cell_y: int) -> tuple[float, float]:
    return grid_cell_center(cell_x, cell_y, session.grid)


def _centerline_grid_cells(
    session,
    centerline_um: Iterable[tuple[float, float]],
) -> tuple[tuple[int, int], ...]:
    points = [
        (float(point[0]), float(point[1]))
        for point in centerline_um
        if math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
    ]
    if not points:
        return ()
    cells: list[tuple[int, int]] = []

    def append_point(point: tuple[float, float]) -> None:
        cell = _physical_point_to_grid_cell(
            point,
            grid_size_um=float(session.grid.grid_size_um),
            origin_x_um=float(session.origin_x_um),
            origin_y_um=float(session.origin_y_um),
        )
        if cell is None:
            return
        if not session._in_bounds(cell[0], cell[1]):
            return
        if cells and cells[-1] == cell:
            return
        cells.append(cell)

    append_point(points[0])
    sample_step_um = max(float(session.grid.grid_size_um) / 4.0, 1.0e-6)
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            append_point(end)
            continue
        steps = max(1, int(math.ceil(length / sample_step_um)))
        for index in range(1, steps + 1):
            t = float(index) / float(steps)
            append_point((start[0] + dx * t, start[1] + dy * t))
    return tuple(dict.fromkeys(cells))


def _fanout_stub_bend_steps(session) -> int:
    raw_value = session.config.fanout.fanout_stub_bend_degrees
    if raw_value is None:
        raw_value = "90"
    normalized = raw_value.strip().lower().replace("_", "-")
    aliases = {
        "45": 1,
        "45deg": 1,
        "45-degree": 1,
        "45-deg": 1,
        "diagonal": 1,
        "diag": 1,
        "90": 2,
        "90deg": 2,
        "90-degree": 2,
        "90-deg": 2,
        "orthogonal": 2,
        "orthogonal-u": 2,
    }
    if normalized not in aliases:
        raise ValueError("PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES must be 45 or 90")
    return aliases[normalized]


def _append_grid_step(
    session,
    path: list[tuple[int, int]],
    step_x: int,
    step_y: int,
    count: int,
) -> None:
    if count <= 0:
        return
    cell_x, cell_y = path[-1]
    for _ in range(count):
        cell_x += step_x
        cell_y += step_y
        if session._in_bounds(cell_x, cell_y):
            path.append((cell_x, cell_y))


def _inflated_cells(
    session,
    cells: Iterable[tuple[int, int]],
    radius: int,
) -> set[tuple[int, int]]:
    radius = max(0, int(radius))
    inflated: set[tuple[int, int]] = set()
    for cell_x, cell_y in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = int(cell_x) + dx
                ny = int(cell_y) + dy
                if session._in_bounds(nx, ny):
                    inflated.add((nx, ny))
    return inflated


def _angle_to_unit_vector(session, angle: int) -> tuple[float, float]:
    radians = (int(angle) % 8) * (math.pi / 4.0)
    return (math.cos(radians), math.sin(radians))


def _rotate_left_vector(session, vector: tuple[float, float]) -> tuple[float, float]:
    return (-vector[1], vector[0])


def _rotate_right_vector(session, vector: tuple[float, float]) -> tuple[float, float]:
    return (vector[1], -vector[0])


def _cross2(
    session,
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    return float(a[0]) * float(b[1]) - float(a[1]) * float(b[0])


def _append_stub_point(
    session,
    out: list[tuple[float, float]],
    point: tuple[float, float],
) -> None:
    point = (float(point[0]), float(point[1]))
    if out:
        last_x, last_y = out[-1]
        if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
            return
    out.append(point)


def _append_circular_stub_bend(
    session,
    out: list[tuple[float, float]],
    *,
    start_point: tuple[float, float],
    start_angle: int,
    end_point: tuple[float, float],
    end_angle: int,
    angle_delta: int,
) -> None:
    radius_um = float(session.bend_radius_cells) * float(session.grid.grid_size_um)
    if radius_um <= 0.0 or not math.isfinite(radius_um):
        session._append_stub_point(out, end_point)
        return
    start_dir = session._angle_to_unit_vector(start_angle)
    end_dir = session._angle_to_unit_vector(end_angle)
    chord = (
        float(end_point[0]) - float(start_point[0]),
        float(end_point[1]) - float(start_point[1]),
    )
    denom = session._cross2(start_dir, end_dir)
    if abs(denom) <= 1.0e-9:
        session._append_stub_point(out, end_point)
        return
    in_len = session._cross2(chord, end_dir) / denom
    out_len = session._cross2(start_dir, chord) / denom
    if (
        not math.isfinite(in_len)
        or not math.isfinite(out_len)
        or in_len <= 1.0e-9
        or out_len <= 1.0e-9
    ):
        session._append_stub_point(out, end_point)
        return
    corner = (
        float(start_point[0]) + start_dir[0] * in_len,
        float(start_point[1]) + start_dir[1] * in_len,
    )
    turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
    trim = radius_um * math.tan(turn_abs / 2.0)
    trim_eff = min(trim, in_len, out_len)
    if not math.isfinite(trim_eff) or trim_eff <= 1.0e-9:
        session._append_stub_point(out, end_point)
        return
    t_in = (
        corner[0] - start_dir[0] * trim_eff,
        corner[1] - start_dir[1] * trim_eff,
    )
    t_out = (
        corner[0] + end_dir[0] * trim_eff,
        corner[1] + end_dir[1] * trim_eff,
    )
    session._append_stub_point(out, t_in)
    left_turn = int(angle_delta) > 0
    n_start = (
        session._rotate_left_vector(start_dir)
        if left_turn
        else session._rotate_right_vector(start_dir)
    )
    n_end = (
        session._rotate_left_vector(end_dir) if left_turn else session._rotate_right_vector(end_dir)
    )
    c0 = (
        t_in[0] + n_start[0] * radius_um,
        t_in[1] + n_start[1] * radius_um,
    )
    c1 = (
        t_out[0] + n_end[0] * radius_um,
        t_out[1] + n_end[1] * radius_um,
    )
    center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
    a0 = math.atan2(t_in[1] - center[1], t_in[0] - center[0])
    a1 = math.atan2(t_out[1] - center[1], t_out[0] - center[0])
    if left_turn:
        while a1 <= a0:
            a1 += math.tau
    else:
        while a1 >= a0:
            a1 -= math.tau
    arc_span = abs(a1 - a0)
    steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
    for index in range(1, steps):
        t = float(index) / float(steps)
        angle = a0 + (a1 - a0) * t
        session._append_stub_point(
            out,
            (
                center[0] + radius_um * math.cos(angle),
                center[1] + radius_um * math.sin(angle),
            ),
        )
    session._append_stub_point(out, t_out)
    session._append_stub_point(out, end_point)


def _append_arc_from_tangencies(
    session,
    out: list[tuple[float, float]],
    *,
    t_in: tuple[float, float],
    t_out: tuple[float, float],
    start_angle: int,
    end_angle: int,
    angle_delta: int,
) -> None:
    radius_um = float(session.bend_radius_cells) * float(session.grid.grid_size_um)
    if radius_um <= 0.0 or not math.isfinite(radius_um):
        session._append_stub_point(out, t_out)
        return
    start_dir = session._angle_to_unit_vector(start_angle)
    end_dir = session._angle_to_unit_vector(end_angle)
    session._append_stub_point(out, t_in)
    left_turn = int(angle_delta) > 0
    n_start = (
        session._rotate_left_vector(start_dir)
        if left_turn
        else session._rotate_right_vector(start_dir)
    )
    n_end = (
        session._rotate_left_vector(end_dir) if left_turn else session._rotate_right_vector(end_dir)
    )
    c0 = (
        float(t_in[0]) + n_start[0] * radius_um,
        float(t_in[1]) + n_start[1] * radius_um,
    )
    c1 = (
        float(t_out[0]) + n_end[0] * radius_um,
        float(t_out[1]) + n_end[1] * radius_um,
    )
    center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
    a0 = math.atan2(float(t_in[1]) - center[1], float(t_in[0]) - center[0])
    a1 = math.atan2(float(t_out[1]) - center[1], float(t_out[0]) - center[0])
    if left_turn:
        while a1 <= a0:
            a1 += math.tau
    else:
        while a1 >= a0:
            a1 -= math.tau
    arc_span = abs(a1 - a0)
    steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
    for index in range(1, steps):
        t = float(index) / float(steps)
        angle = a0 + (a1 - a0) * t
        session._append_stub_point(
            out,
            (
                center[0] + radius_um * math.cos(angle),
                center[1] + radius_um * math.sin(angle),
            ),
        )
    session._append_stub_point(out, t_out)


def _append_realized_stub_bend(
    session,
    out: list[tuple[float, float]],
    start_point_um: tuple[float, float],
    start_angle: int,
    angle_delta: int,
) -> tuple[float, float]:
    arm_um = float(session.bend_radius_cells) * float(session.grid.grid_size_um)
    radius_um = arm_um
    turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
    trim = radius_um * math.tan(turn_abs / 2.0)
    end_angle = (int(start_angle) + int(angle_delta)) % 8
    start_dir = session._angle_to_unit_vector(int(start_angle) % 8)
    end_dir = session._angle_to_unit_vector(end_angle)
    start_step = session._angle_to_step(int(start_angle) % 8)
    end_step = session._angle_to_step(end_angle)
    start_point = (float(start_point_um[0]), float(start_point_um[1]))
    corner = (
        start_point[0] + float(start_step[0]) * arm_um,
        start_point[1] + float(start_step[1]) * arm_um,
    )
    end_point = (
        corner[0] + float(end_step[0]) * arm_um,
        corner[1] + float(end_step[1]) * arm_um,
    )
    t_in = (
        corner[0] - start_dir[0] * trim,
        corner[1] - start_dir[1] * trim,
    )
    t_out = (
        corner[0] + end_dir[0] * trim,
        corner[1] + end_dir[1] * trim,
    )
    session._append_stub_point(out, t_in)
    session._append_arc_from_tangencies(
        out,
        t_in=t_in,
        t_out=t_out,
        start_angle=int(start_angle) % 8,
        end_angle=end_angle,
        angle_delta=int(angle_delta),
    )
    session._append_stub_point(out, end_point)
    return end_point


def _two_bend_static_stub_centerline_um(
    session,
    port_center_um: tuple[float, float],
    physical_angle: int,
    lateral_sign: int,
    target_anchor_y_cell: int | None = None,
    min_forward_cells: int = 0,
    initial_forward_cells: int = 0,
    extra_final_forward_cells: int = 0,
) -> tuple[tuple[tuple[float, float], ...], tuple[int, int]] | None:
    start_angle = int(physical_angle) % 8
    bend_delta = int(lateral_sign) * session._fanout_stub_bend_steps()
    intermediate_angle = (start_angle + bend_delta) % 8
    intermediate_step = session._angle_to_step(intermediate_angle)
    final_step = session._angle_to_step(start_angle)
    trace_fanout_stubs = session.config.diagnostics.trace_fanout_stubs

    def fail(reason: str, extra: str = "") -> None:
        if trace_fanout_stubs:
            print(
                "fanout_stub_failed "
                f"reason={reason} "
                f"port={port_center_um} "
                f"angle={start_angle} lateral_sign={lateral_sign} "
                f"target_anchor_y_cell={target_anchor_y_cell} "
                f"min_forward_cells={min_forward_cells} "
                f"initial_forward_cells={initial_forward_cells} "
                f"extra_final_forward_cells={extra_final_forward_cells}"
                f"{extra}",
                file=sys.stderr,
            )

    if abs(final_step[0]) + abs(final_step[1]) != 1:
        fail("non_cardinal_final")
        return None
    if intermediate_step[1] == 0:
        fail("intermediate_has_no_y")
        return None
    port_point = (float(port_center_um[0]), float(port_center_um[1]))

    def _next_grid_axis_value(
        value: float,
        axis: str,
        direction: int,
    ) -> float | None:
        if direction == 0:
            return None
        origin = session.origin_x_um if axis == "x" else session.origin_y_um
        rel = (float(value) - float(origin)) / float(session.grid.grid_size_um) - 0.5
        eps = 1.0e-9
        if direction > 0:
            index = math.ceil(rel - eps)
        else:
            index = math.floor(rel + eps)
        center = grid_cell_center(
            int(index) if axis == "x" else 0,
            int(index) if axis == "y" else 0,
            session.grid,
        )
        return center[0] if axis == "x" else center[1]

    points: list[tuple[float, float]] = [port_point]
    bend_start = port_point
    initial_forward_um = float(max(0, int(initial_forward_cells))) * float(
        session.grid.grid_size_um
    )
    if initial_forward_um > 1.0e-9:
        bend_start = (
            port_point[0] + float(final_step[0]) * initial_forward_um,
            port_point[1] + float(final_step[1]) * initial_forward_um,
        )
        session._append_stub_point(points, bend_start)
    first_end = session._append_realized_stub_bend(
        points,
        bend_start,
        start_angle,
        bend_delta,
    )
    if target_anchor_y_cell is None:
        target_intermediate_y = _next_grid_axis_value(
            first_end[1],
            "y",
            int(intermediate_step[1]),
        )
    else:
        target_intermediate_y = session._grid_cell_center_um(
            0,
            int(target_anchor_y_cell) - int(intermediate_step[1]) * int(session.bend_radius_cells),
        )[1]
    if target_intermediate_y is None:
        fail("no_target_intermediate_y")
        return None
    intermediate_delta_y = float(target_intermediate_y) - float(first_end[1])
    if intermediate_delta_y * float(intermediate_step[1]) < -1.0e-9:
        fail("intermediate_moves_backward")
        return None
    intermediate_delta_x = intermediate_delta_y * (
        float(intermediate_step[0]) / float(intermediate_step[1])
    )
    intermediate_end = (
        first_end[0] + intermediate_delta_x,
        float(target_intermediate_y),
    )
    session._append_stub_point(points, intermediate_end)
    second_end = session._append_realized_stub_bend(
        points,
        intermediate_end,
        intermediate_angle,
        angle_delta=-bend_delta,
    )
    if final_step[0] != 0:
        target_final_x = _next_grid_axis_value(
            second_end[0],
            "x",
            int(final_step[0]),
        )
        if target_final_x is None:
            fail("no_target_final_x")
            return None
        min_forward_x = port_point[0] + float(final_step[0]) * float(
            max(0, int(min_forward_cells))
            + max(0, int(initial_forward_cells))
            + max(0, int(extra_final_forward_cells))
        ) * float(session.grid.grid_size_um)
        if int(final_step[0]) > 0:
            if float(target_final_x) < float(min_forward_x):
                snapped_min_forward_x = _next_grid_axis_value(
                    float(min_forward_x),
                    "x",
                    int(final_step[0]),
                )
                if snapped_min_forward_x is None:
                    fail("no_snapped_min_forward_x")
                    return None
                target_final_x = float(snapped_min_forward_x)
        else:
            if float(target_final_x) > float(min_forward_x):
                snapped_min_forward_x = _next_grid_axis_value(
                    float(min_forward_x),
                    "x",
                    int(final_step[0]),
                )
                if snapped_min_forward_x is None:
                    fail("no_snapped_min_forward_x")
                    return None
                target_final_x = float(snapped_min_forward_x)
        final_delta_x = float(target_final_x) - float(second_end[0])
        if final_delta_x * float(final_step[0]) < -1.0e-9:
            fail("final_moves_backward_x")
            return None
        anchor_point = (float(target_final_x), float(second_end[1]))
    else:
        target_final_y = _next_grid_axis_value(
            second_end[1],
            "y",
            int(final_step[1]),
        )
        if target_final_y is None:
            fail("no_target_final_y")
            return None
        final_delta_y = float(target_final_y) - float(second_end[1])
        if final_delta_y * float(final_step[1]) < -1.0e-9:
            fail("final_moves_backward_y")
            return None
        anchor_point = (float(second_end[0]), float(target_final_y))
    session._append_stub_point(points, anchor_point)
    anchor_x = int(
        round((anchor_point[0] - session.origin_x_um) / float(session.grid.grid_size_um) - 0.5)
    )
    anchor_y = int(
        round((anchor_point[1] - session.origin_y_um) / float(session.grid.grid_size_um) - 0.5)
    )
    snapped_anchor = session._grid_cell_center_um(anchor_x, anchor_y)
    snap_error_um = math.hypot(
        float(snapped_anchor[0]) - float(anchor_point[0]),
        float(snapped_anchor[1]) - float(anchor_point[1]),
    )
    if snap_error_um > max(1.0e-6, 0.05 * float(session.grid.grid_size_um)):
        fail(
            f"snap_error:{snap_error_um:.6g}",
            " "
            f"anchor_point=({anchor_point[0]:.6g},{anchor_point[1]:.6g}) "
            f"anchor_cell=({anchor_x},{anchor_y}) "
            f"snapped=({snapped_anchor[0]:.6g},{snapped_anchor[1]:.6g}) "
            f"origin=({session.origin_x_um:.6g},{session.origin_y_um:.6g}) "
            f"grid={float(session.grid.grid_size_um):.6g} "
            f"bend_radius_cells={session.bend_radius_cells}",
        )
        return None
    if not session._in_bounds(anchor_x, anchor_y):
        fail("anchor_out_of_bounds")
        return None
    return _compress_centerline(tuple(points)), (anchor_x, anchor_y)


def _straight_static_stub_centerline_um(
    session,
    port_center_um: tuple[float, float],
    physical_angle: int,
    forward_cells: int,
) -> tuple[tuple[tuple[float, float], ...], tuple[int, int]] | None:
    """Build a straight stub extending `forward_cells` grid cells forward
    from a port along its own physical orientation, absorbing any
    sub-cell port-to-grid misalignment with a small real-bend-radius
    curve rather than a lateral offset.

    Used for dense TARGET port stubs. Unlike source stubs (which need
    to laterally redistribute a tight component pitch out to a wider
    lane spacing via `_two_bend_static_stub_centerline_um`, since many
    source nets fan out from adjacent ports toward widely separated
    destinations), a target port just needs a short, protected,
    straight approach directly in front of itself -- any lateral
    movement a route still needs happens in the main A* search after
    it reaches this anchor, not baked into the stub geometry.

    Two things were tried and rejected before this one, both recorded
    in `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
    Surprises & Discoveries: (1) reusing the two-bend, laterally
    offsetting source-stub geometry for target ports too caused a real
    regression (nearby target stubs ended up on different lateral
    lanes, producing near-duplicate diagonal candidate paths for
    adjacent nets that then illegally collided); (2) keeping the
    anchor's geometric Y exactly at the port's own Y (a perfectly
    straight, unbent line) and deferring the resulting sub-cell offset
    to the ordinary checked-endpoint-correction pass did not work --
    that pass left the route's raw grid endpoint essentially untouched
    rather than reconciling it, for reasons not yet root-caused. This
    version absorbs the offset locally instead, using
    `_fanout_stub_centerline_um`, which is a plain straight line when
    the port and the grid-snapped anchor are already aligned, and a
    small circular arc (built by `_append_circular_stub_bend`, using
    `self.bend_radius_cells` -- the exact same physical bend radius
    every other primitive in this router uses, not an approximation)
    only when they are not.
    """
    step_x, step_y = session._angle_to_step(int(physical_angle) % 8)
    if abs(step_x) + abs(step_y) != 1:
        return None
    forward_cells = max(0, int(forward_cells))
    if forward_cells <= 0:
        return None
    port_cell = _physical_point_to_grid_cell(
        port_center_um,
        grid_size_um=float(session.grid.grid_size_um),
        origin_x_um=session.origin_x_um,
        origin_y_um=session.origin_y_um,
    )
    if port_cell is None:
        return None
    anchor_x = int(port_cell[0]) + step_x * forward_cells
    anchor_y = int(port_cell[1]) + step_y * forward_cells
    if not session._in_bounds(anchor_x, anchor_y):
        return None
    # Deliberately NOT grid-snapped: a port's exact physical position
    # generally does not fall exactly on a grid cell center, and a
    # pure straight run along one axis cannot itself correct the other
    # axis. Keep the port's exact Y throughout (a truly straight line,
    # X-only) and let this exact point be what standard, ordinary
    # checked-endpoint-correction resolves against -- see
    # `RouteBookkeeping.record_route`'s `target_port_center_um_override`
    # and `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
    anchor_point_um = (
        float(port_center_um[0])
        + float(step_x) * float(forward_cells) * float(session.grid.grid_size_um),
        float(port_center_um[1])
        + float(step_y) * float(forward_cells) * float(session.grid.grid_size_um),
    )
    centerline = _compress_centerline((tuple(port_center_um), anchor_point_um))
    if len(centerline) < 2:
        return None
    return centerline, (anchor_x, anchor_y)


def _fanout_stub_centerline_um(
    session,
    port_center_um: tuple[float, float] | None,
    anchor_center_um: tuple[float, float],
    physical_angle: int,
) -> tuple[tuple[float, float], ...]:
    if port_center_um is None:
        return (anchor_center_um,)
    forward_x, forward_y = session._angle_to_step(int(physical_angle) % 8)
    lateral_x, lateral_y = -forward_y, forward_x
    port_x, port_y = (float(port_center_um[0]), float(port_center_um[1]))
    anchor_x, anchor_y = (float(anchor_center_um[0]), float(anchor_center_um[1]))
    delta_x = anchor_x - port_x
    delta_y = anchor_y - port_y
    forward_delta = delta_x * forward_x + delta_y * forward_y
    lateral_delta = delta_x * lateral_x + delta_y * lateral_y
    if forward_delta <= 1.0e-9:
        return _compress_centerline((port_center_um, anchor_center_um))
    lateral_abs = abs(lateral_delta)
    available_straight = forward_delta - lateral_abs
    if available_straight <= 1.0e-9:
        return _compress_centerline((port_center_um, anchor_center_um))

    preferred_first_straight_um = max(
        float(session.grid.grid_size_um),
        float(session.bend_radius_cells) * float(session.grid.grid_size_um),
    )
    first_straight_um = min(preferred_first_straight_um, available_straight)
    points: list[tuple[float, float]] = [
        (port_x, port_y),
        (
            port_x + forward_x * first_straight_um,
            port_y + forward_y * first_straight_um,
        ),
    ]
    if lateral_abs > 1.0e-9:
        lateral_sign = 1 if lateral_delta > 0.0 else -1
        diagonal_angle = (int(physical_angle) + lateral_sign) % 8
        diagonal_end = (
            points[-1][0] + forward_x * lateral_abs + lateral_x * lateral_delta,
            points[-1][1] + forward_y * lateral_abs + lateral_y * lateral_delta,
        )
        smoothed: list[tuple[float, float]] = [points[0]]
        session._append_circular_stub_bend(
            smoothed,
            start_point=points[0],
            start_angle=physical_angle,
            end_point=diagonal_end,
            end_angle=diagonal_angle,
            angle_delta=lateral_sign,
        )
        session._append_circular_stub_bend(
            smoothed,
            start_point=diagonal_end,
            start_angle=diagonal_angle,
            end_point=(anchor_x, anchor_y),
            end_angle=physical_angle,
            angle_delta=-lateral_sign,
        )
        return _compress_centerline(tuple(smoothed))
    points.append((anchor_x, anchor_y))
    return _compress_centerline(tuple(points))


def _build_static_fanout_anchors(session) -> dict[str, _FanoutAnchor]:
    if session.fanout_access_mode_normalized != "static-stubs":
        return {}
    default_forward_cells = max(3, int(session.bend_radius_cells) + 3)
    default_lane_spacing_cells = 11
    forward_cells = session._fanout_int_or_default(
        session.config.fanout.fanout_stub_forward_cells,
        default_forward_cells,
    )
    lane_spacing_cells = session._fanout_int_or_default(
        session.config.fanout.fanout_lane_spacing_cells,
        default_lane_spacing_cells,
    )
    stub_x_offset_cells = session._fanout_int_or_default(
        session.config.fanout.fanout_stub_x_offset_cells,
        1,
    )
    if forward_cells <= 0 or lane_spacing_cells <= 0:
        return {}

    anchors: dict[str, _FanoutAnchor] = {}
    for instance_name, port_specs in session.source_port_specs_by_instance.items():
        if not session._is_dense_source_fanout_instance(instance_name):
            continue
        by_angle: dict[int, list[str]] = {}
        for port_spec in port_specs:
            _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
            angle = session._orientation_to_angle(getattr(port, "orientation", None), flip=False)
            step_x, step_y = session._angle_to_step(angle)
            # The first static-stub implementation intentionally handles
            # cardinal MMI port rows. Diagonal component ports fall back to
            # the normal endpoint behavior until a safe breakout is defined.
            if abs(step_x) + abs(step_y) != 1:
                continue
            by_angle.setdefault(angle, []).append(port_spec)

        for angle, group_specs in by_angle.items():
            if not session._is_dense_source_fanout_group(instance_name, angle):
                continue
            step_x, step_y = session._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x
            ordered_items: list[tuple[str, int, Any]] = []
            for port_spec in group_specs:
                _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
                state = session.port_to_grid_state(
                    port,
                    session.origin_x_um,
                    session.origin_y_um,
                    float(session.grid.grid_size_um),
                    as_target=False,
                )
                lateral_cell = int(state.x) * lateral_x + int(state.y) * lateral_y
                ordered_items.append((port_spec, lateral_cell, state))
            ordered_items.sort(key=lambda item: (item[1], item[0]))
            count = len(ordered_items)
            if count < session._dense_fanout_min_ports_for(instance_name) or step_y != 0:
                continue

            def add_two_bend_anchor(
                item: tuple[str, int, Any],
                lateral_sign: int,
                target_anchor_y_cell: int | None,
                initial_forward_cells: int = 0,
                extra_final_forward_cells: int = 0,
            ) -> tuple[int, int] | None:
                port_spec, _current_lateral, state = item
                _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
                real_center = _port_center_um(port)
                if real_center is None:
                    return None
                stub_result = session._two_bend_static_stub_centerline_um(
                    real_center,
                    angle,
                    lateral_sign,
                    target_anchor_y_cell=target_anchor_y_cell,
                    min_forward_cells=int(forward_cells),
                    initial_forward_cells=max(0, int(initial_forward_cells)),
                    extra_final_forward_cells=max(0, int(extra_final_forward_cells)),
                )
                if stub_result is None:
                    return None
                centerline, (anchor_x, anchor_y) = stub_result
                anchor_center = session._grid_cell_center_um(anchor_x, anchor_y)
                anchors[port_spec] = session._FanoutAnchor(
                    port_spec=port_spec,
                    state_x=anchor_x,
                    state_y=anchor_y,
                    physical_angle=angle,
                    center_um=anchor_center,
                    stub_center_cells=session._centerline_grid_cells(centerline),
                    stub_centerline_um=centerline,
                )
                return anchor_x, anchor_y

            lower_items = ordered_items[: count // 2]
            upper_items = ordered_items[count // 2 :]
            if not lower_items or not upper_items:
                continue
            stub_bend_steps = session._fanout_stub_bend_steps()
            stagger_forward_cells = int(stub_x_offset_cells) if int(stub_bend_steps) >= 2 else 0

            lower_inner = lower_items[-1]
            lower_count = len(lower_items)
            lower_inner_anchor = add_two_bend_anchor(
                lower_inner,
                -1,
                target_anchor_y_cell=None,
                initial_forward_cells=(lower_count - 1) * stagger_forward_cells,
                extra_final_forward_cells=0,
            )
            if lower_inner_anchor is not None:
                lower_base_y = int(lower_inner_anchor[1])
                for rank, item in enumerate(reversed(lower_items[:-1]), start=1):
                    initial_rank = lower_count - 1 - int(rank)
                    final_rank = int(rank)
                    _min_anchor = add_two_bend_anchor(
                        item,
                        -1,
                        target_anchor_y_cell=None,
                        initial_forward_cells=initial_rank * stagger_forward_cells,
                        extra_final_forward_cells=final_rank * stagger_forward_cells,
                    )
                    desired_y = lower_base_y - int(rank) * int(lane_spacing_cells)
                    if _min_anchor is not None:
                        desired_y = min(desired_y, int(_min_anchor[1]))
                    add_two_bend_anchor(
                        item,
                        -1,
                        target_anchor_y_cell=desired_y,
                        initial_forward_cells=initial_rank * stagger_forward_cells,
                        extra_final_forward_cells=final_rank * stagger_forward_cells,
                    )

            upper_inner = upper_items[0]
            upper_count = len(upper_items)
            upper_inner_anchor = add_two_bend_anchor(
                upper_inner,
                1,
                target_anchor_y_cell=None,
                initial_forward_cells=(upper_count - 1) * stagger_forward_cells,
                extra_final_forward_cells=0,
            )
            if upper_inner_anchor is not None:
                upper_base_y = int(upper_inner_anchor[1])
                for rank, item in enumerate(upper_items[1:], start=1):
                    initial_rank = upper_count - 1 - int(rank)
                    final_rank = int(rank)
                    _min_anchor = add_two_bend_anchor(
                        item,
                        1,
                        target_anchor_y_cell=None,
                        initial_forward_cells=initial_rank * stagger_forward_cells,
                        extra_final_forward_cells=final_rank * stagger_forward_cells,
                    )
                    desired_y = upper_base_y + int(rank) * int(lane_spacing_cells)
                    if _min_anchor is not None:
                        desired_y = max(desired_y, int(_min_anchor[1]))
                    add_two_bend_anchor(
                        item,
                        1,
                        target_anchor_y_cell=desired_y,
                        initial_forward_cells=initial_rank * stagger_forward_cells,
                        extra_final_forward_cells=final_rank * stagger_forward_cells,
                    )
    return anchors


def _build_static_fanout_target_anchors(session) -> dict[str, _FanoutAnchor]:
    """Build real, pre-committed straight stubs for dense TARGET ports.

    Unlike `_build_static_fanout_anchors` (the source-side equivalent),
    this deliberately does NOT reuse the two-bend, laterally-offsetting
    stub geometry: a first implementation attempt did reuse it, and that
    turned out to actively cause a regression on `multiportmmi_8x8`
    (`n_24` newly failed to route) -- see
    `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
    Surprises & Discoveries for the full trace evidence. Laterally
    offsetting each target port onto a different lane produced
    near-duplicate diagonal candidate paths for adjacent nets, which
    then illegally collided (non-perpendicular / collinear-overlap
    crossings) with each other. A target port does not need lateral
    redistribution the way a dense source fanout does (source nets fan
    out from adjacent ports toward widely separated destinations;
    target nets converge from widely separated sources onto adjacent
    ports, which is not the same problem) -- it only needs a short,
    protected, straight approach directly in front of itself, staggered
    in LENGTH only (middle ports reaching further, matching this
    benchmark's own physical layout intent), never in lateral position.
    Any lateral movement a route still needs happens in the main A*
    search after it reaches this anchor, exactly as it already does for
    every non-stubbed port.

    `_states_and_openings` and `_fanout_stubbed_centerline` already
    consume `fanout_anchor_by_port_spec` symmetrically for source and
    target anchors (confirmed by reading both before writing this
    function), so populating target entries in that same dict is all
    that is required for the rest of the routing pipeline to pick them
    up correctly -- no other call site needs to change.
    """
    if session.fanout_access_mode_normalized != "static-stubs":
        return {}
    default_forward_cells = max(3, int(session.bend_radius_cells) + 3)
    forward_cells = session._fanout_int_or_default(
        session.config.fanout.fanout_stub_forward_cells,
        default_forward_cells,
    )
    spacing_cells = session._fanout_int_or_default(
        session.config.fanout.target_protected_lane_spacing_cells,
        session._fanout_int_or_default(
            session.config.fanout.fanout_protected_lane_spacing_cells,
            session._fanout_int_or_default(
                session.config.fanout.fanout_lane_spacing_cells,
                3,
            ),
        ),
    )
    if forward_cells <= 0:
        return {}

    anchors: dict[str, _FanoutAnchor] = {}
    for instance_name, port_specs in session.target_port_specs_by_instance.items():
        if not session._is_dense_target_fanout_instance(instance_name):
            continue
        by_angle: dict[int, list[str]] = {}
        for port_spec in port_specs:
            _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
            angle = session._orientation_to_angle(getattr(port, "orientation", None), flip=False)
            step_x, step_y = session._angle_to_step(angle)
            # Mirrors the source-side restriction: static stub geometry
            # is only defined for cardinal (axis-aligned) port rows
            # today. Diagonal target ports fall back to the normal
            # (non-stub) endpoint behavior, exactly like diagonal
            # source ports already do.
            if abs(step_x) + abs(step_y) != 1:
                continue
            by_angle.setdefault(angle, []).append(port_spec)

        for angle, group_specs in by_angle.items():
            if not session._is_dense_target_fanout_group(instance_name, angle):
                continue
            step_x, step_y = session._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x

            def _lateral_position(port_spec: str) -> float:
                _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
                center = _port_center_um(port)
                if center is None:
                    return 0.0
                return float(center[0]) * lateral_x + float(center[1]) * lateral_y

            ordered = sorted(group_specs, key=lambda spec: (_lateral_position(spec), spec))
            count = len(ordered)
            if count < session._dense_fanout_min_ports():
                continue
            lower_specs = ordered[: count // 2]
            upper_specs = ordered[count // 2 :]
            # Rank 1 = shortest (edge of the group), highest rank =
            # longest (middle of the group) -- the "middle ones go out
            # the furthest" staggering, in length only.
            ranked: list[tuple[str, int]] = [
                (port_spec, port_index + 1) for port_index, port_spec in enumerate(lower_specs)
            ]
            upper_count = len(upper_specs)
            ranked.extend(
                (port_spec, upper_count - port_index)
                for port_index, port_spec in enumerate(upper_specs)
            )
            if count == 2:
                # A bare pair (e.g. a 2x2 switch's inputs, 1.25 um apart
                # inside one routing cell) has no "middle": the symmetric
                # ranking gives both the same length and the two
                # approaches would still share the port cell. Ranks 1
                # and 2 put their anchors at different x instead.
                ranked = [(ordered[0], 1), (ordered[1], 2)]

            for port_spec, rank in ranked:
                _inst, _port_name, port = session.endpoint_ports_by_spec[port_spec]
                real_center = _port_center_um(port)
                if real_center is None:
                    continue
                length_cells = int(forward_cells) + int(spacing_cells) * (int(rank) - 1)
                stub_result = session._straight_static_stub_centerline_um(
                    real_center,
                    angle,
                    length_cells,
                )
                if stub_result is None:
                    continue
                centerline, (anchor_x, anchor_y) = stub_result
                # The anchor's own exact geometric position (the
                # straight stub's far end, keeping the port's true Y),
                # NOT the grid cell's snapped center -- see
                # `_straight_static_stub_centerline_um`.
                anchor_center = centerline[-1]
                anchors[port_spec] = session._FanoutAnchor(
                    port_spec=port_spec,
                    state_x=anchor_x,
                    state_y=anchor_y,
                    physical_angle=angle,
                    center_um=anchor_center,
                    stub_center_cells=session._centerline_grid_cells(centerline),
                    stub_centerline_um=centerline,
                )
    return anchors


def _dense_source_port_runway_lengths(
    session,
    jobs: list[RouteJob],
) -> dict[str, int]:
    """Reserve staggered source-port access in dense MMI fanout runs."""
    lengths_by_spec: dict[str, int] = {}
    if session.fanout_access_mode_normalized == "static-stubs":
        grouped_specs: dict[tuple[str, int], set[str]] = {}
        source_specs = {
            f"{run_job.inst1},{run_job.port1}"
            for run_job in jobs
            if f"{run_job.inst1},{run_job.port1}" in session.fanout_anchor_by_port_spec
        }
        for port_spec in source_specs:
            anchor = session.fanout_anchor_by_port_spec[port_spec]
            instance_name = port_spec.split(",", 1)[0]
            grouped_specs.setdefault(
                (instance_name, int(anchor.physical_angle) % 8),
                set(),
            ).add(port_spec)

        for (_instance_name, angle), specs in grouped_specs.items():
            if len(specs) < session._dense_fanout_min_ports():
                continue
            step_x, step_y = session._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x

            def anchor_lateral_position(port_spec: str) -> int:
                anchor = session.fanout_anchor_by_port_spec[port_spec]
                return int(anchor.state_x) * lateral_x + int(anchor.state_y) * lateral_y

            ordered_specs = sorted(
                specs,
                key=lambda port_spec: (
                    anchor_lateral_position(port_spec),
                    port_spec,
                ),
            )
            count = len(ordered_specs)
            spacing_cells = session._fanout_int_or_default(
                session.config.fanout.fanout_protected_lane_spacing_cells,
                session._fanout_int_or_default(
                    session.config.fanout.fanout_lane_spacing_cells,
                    3,
                ),
            )
            spacing_cells = max(1, int(spacing_cells))
            lower_specs = ordered_specs[: count // 2]
            upper_specs = ordered_specs[count // 2 :]
            for port_index, port_spec in enumerate(lower_specs):
                runway_rank = int(port_index) + 1
                lengths_by_spec[port_spec] = spacing_cells * runway_rank
            upper_count = len(upper_specs)
            for port_index, port_spec in enumerate(upper_specs):
                runway_rank = upper_count - int(port_index)
                lengths_by_spec[port_spec] = spacing_cells * runway_rank
            session._equalize_dense_runway_reach(lengths_by_spec, list(ordered_specs))
        return lengths_by_spec

    if session.fanout_access_mode_normalized != "legacy-runway":
        return {}

    index = 0
    while index < len(jobs):
        job = jobs[index]
        if not session._is_dense_source_fanout_instance(job.inst1):
            index += 1
            continue
        run_end = index + 1
        while (
            run_end < len(jobs)
            and jobs[run_end].inst1 == job.inst1
            and session._is_dense_source_fanout_instance(jobs[run_end].inst1)
        ):
            run_end += 1

        run = jobs[index:run_end]
        by_angle: dict[int, list[RouteJob]] = {}
        for run_job in run:
            angle = session._orientation_to_angle(
                getattr(run_job.source_port, "orientation", None),
                flip=False,
            )
            by_angle.setdefault(angle, []).append(run_job)

        for angle, angle_jobs in by_angle.items():
            if not session._is_dense_source_fanout_group(job.inst1, angle):
                continue
            step_x, step_y = session._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x

            def _lateral_position_for_dense_source_runway(run_job: RouteJob) -> float:
                center = _port_center_um(run_job.source_port)
                if center is None:
                    return float(run_job.route_index)
                return float(center[0]) * lateral_x + float(center[1]) * lateral_y

            ordered = sorted(
                angle_jobs,
                key=lambda run_job: (
                    _lateral_position_for_dense_source_runway(run_job),
                    int(run_job.route_index),
                ),
            )
            count = len(ordered)
            group_port_specs: list[str] = []
            for port_index, run_job in enumerate(ordered):
                port_spec = f"{run_job.inst1},{run_job.port1}"
                lengths_by_spec[port_spec] = 3 + 3 * (count - 1 - port_index)
                group_port_specs.append(port_spec)
            session._equalize_dense_runway_reach(lengths_by_spec, group_port_specs)

        index = run_end
    return lengths_by_spec


def _equalize_dense_runway_reach(
    session,
    lengths_by_spec: dict[str, int],
    port_specs: list[str],
) -> None:
    """Extend every port in one dense group to match the group's own
    furthest forward reach, in place.

    A port's staggered length only controls how far its own raw
    footprint extends. That raw footprint (before any per-port lateral
    narrowing) is reserved globally, in `port_runway_cells_by_spec`,
    which becomes part of `static_blocked_cells_before_port_reservations`
    for every OTHER net's routing (see `run()`, where
    `port_runway_static_cells` is unioned into it). If ports in the same
    dense group are given different lengths, the longest-reaching one's
    raw footprint -- which is a wide box, not just that port's own
    narrow lane, since half_width_cells is not reduced by staggering --
    can end up blocking a shorter neighbor's own narrower lane in the
    gap between them, even though nothing physically stands in that gap.
    Equalizing every port in the group to the same forward reach as its
    longest sibling removes this self-inflicted gap: by definition, no
    sibling's own raw footprint reserves anything beyond the group's
    current maximum reach, so no sibling can block another sibling
    beyond that point once every port shares the same reach.
    """
    if not port_specs:
        return
    half_width_cells = int(session.port_lane_half_width_cells)
    max_reach = max(int(lengths_by_spec[spec]) + half_width_cells - 1 for spec in port_specs)
    equalized_length = max_reach - half_width_cells + 1
    for spec in port_specs:
        lengths_by_spec[spec] = max(int(lengths_by_spec[spec]), equalized_length)


def _dense_target_port_runway_lengths(
    session,
    jobs: list[RouteJob],
) -> dict[str, int]:
    """Reserve staggered target-port access for dense same-instance sinks."""
    lengths_by_spec: dict[str, int] = {}
    grouped: dict[tuple[str, int], list[RouteJob]] = {}
    for run_job in jobs:
        angle = session._orientation_to_angle(
            getattr(run_job.target_port, "orientation", None),
            flip=False,
        )
        grouped.setdefault((run_job.inst2, int(angle)), []).append(run_job)

    base_cells = max(1, int(session.bend_radius_cells) + 1)
    spacing_cells = session._fanout_int_or_default(
        session.config.fanout.target_protected_lane_spacing_cells,
        session._fanout_int_or_default(
            session.config.fanout.fanout_protected_lane_spacing_cells,
            session._fanout_int_or_default(
                session.config.fanout.fanout_lane_spacing_cells,
                3,
            ),
        ),
    )
    spacing_cells = max(1, int(spacing_cells))

    for (_instance_name, angle), angle_jobs in grouped.items():
        if len(angle_jobs) < 4:
            continue
        step_x, step_y = session._angle_to_step(angle)
        lateral_x, lateral_y = -step_y, step_x

        def _lateral_position_for_dense_target_runway(run_job: RouteJob) -> float:
            center = _port_center_um(run_job.target_port)
            if center is None:
                return float(run_job.route_index)
            return float(center[0]) * lateral_x + float(center[1]) * lateral_y

        ordered = sorted(
            angle_jobs,
            key=lambda run_job: (
                _lateral_position_for_dense_target_runway(run_job),
                int(run_job.route_index),
            ),
        )
        count = len(ordered)
        lower_jobs = ordered[: count // 2]
        upper_jobs = ordered[count // 2 :]
        group_port_specs: list[str] = []
        for port_index, run_job in enumerate(lower_jobs):
            port_spec = f"{run_job.inst2},{run_job.port2}"
            if port_spec in session.fanout_anchor_by_port_spec:
                # A real, pre-committed static stub already exists for
                # this port (see `_build_static_fanout_target_anchors`):
                # the abstract reservation this function computes is
                # redundant for it and would just reintroduce the
                # flattened-staggering behavior this mechanism exists to
                # avoid, so skip it entirely (0 reservation, excluded
                # from equalization) rather than reserving anything.
                lengths_by_spec[port_spec] = 0
                continue
            lengths_by_spec[port_spec] = base_cells + spacing_cells * int(port_index)
            group_port_specs.append(port_spec)
        upper_count = len(upper_jobs)
        for port_index, run_job in enumerate(upper_jobs):
            port_spec = f"{run_job.inst2},{run_job.port2}"
            if port_spec in session.fanout_anchor_by_port_spec:
                lengths_by_spec[port_spec] = 0
                continue
            lengths_by_spec[port_spec] = base_cells + spacing_cells * (
                upper_count - 1 - int(port_index)
            )
            group_port_specs.append(port_spec)
        session._equalize_dense_runway_reach(lengths_by_spec, group_port_specs)
    return lengths_by_spec


def build_route_jobs_and_fanout_clustering(
    session, nets: Mapping[str, Any]
) -> tuple[
    list[RouteJob],
    dict[str, set[str]],
    dict[str, int],
]:
    """Build the per-net `RouteJob` list from the schematic netlist and compute fanout/dense-port clustering.

    Populates `self.endpoint_ports_by_spec`, `self.source_port_specs_by_instance`
    (and its angle-keyed variant), `self.fanout_anchor_by_port_spec` and the related
    fanout-stub cell sets, and `self.dense_source_cluster_specs_by_port_spec` (which
    source ports on a dense multi-port instance share a lateral cluster). Returns the
    route jobs plus two locals the next phase (port-opening/footprint computation)
    still needs: `endpoint_port_specs_by_instance` and
    `dense_port_runway_length_by_spec` (the merged source+target runway-length map).
    """
    route_jobs: list[RouteJob] = []
    session.endpoint_ports_by_spec: dict[str, tuple[str, str, Port]] = {}
    endpoint_port_specs_by_instance: dict[str, set[str]] = {}
    session.port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    session.port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    session.port_runway_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    session.port_access_rule_by_spec: dict[str, str | None] = {}
    next_net_id = 1
    t_route_job_build_start = session._pipeline_timer_start()
    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")
            source_port = get_port_from_instance(session.routed_layout, inst1, port1)
            target_port = get_port_from_instance(session.routed_layout, inst2, port2)
            route_jobs.append(
                RouteJob(
                    net_id=next_net_id,
                    route_index=next_net_id,
                    net_name=net_name,
                    inst1=inst1,
                    port1=port1,
                    inst2=inst2,
                    port2=port2,
                    source_port=source_port,
                    target_port=target_port,
                )
            )
            next_net_id += 1
            session.endpoint_ports_by_spec.setdefault(port1_spec, (inst1, port1, source_port))
            session.endpoint_ports_by_spec.setdefault(port2_spec, (inst2, port2, target_port))
            endpoint_port_specs_by_instance.setdefault(inst1, set()).add(port1_spec)
            endpoint_port_specs_by_instance.setdefault(inst2, set()).add(port2_spec)
    session._record_pipeline_timing("route_job_build", t_route_job_build_start)

    session.source_port_specs_by_instance: dict[str, set[str]] = {}
    session.source_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = {}
    session.target_port_specs_by_instance: dict[str, set[str]] = {}
    session.target_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = {}
    for run_job in route_jobs:
        port_spec = f"{run_job.inst1},{run_job.port1}"
        session.source_port_specs_by_instance.setdefault(run_job.inst1, set()).add(port_spec)
        angle = session._orientation_to_angle(
            getattr(run_job.source_port, "orientation", None),
            flip=False,
        )
        session.source_port_specs_by_instance_angle.setdefault(
            (run_job.inst1, int(angle)), set()
        ).add(port_spec)
        target_port_spec = f"{run_job.inst2},{run_job.port2}"
        session.target_port_specs_by_instance.setdefault(run_job.inst2, set()).add(
            target_port_spec
        )
        target_angle = session._orientation_to_angle(
            getattr(run_job.target_port, "orientation", None),
            flip=False,
        )
        session.target_port_specs_by_instance_angle.setdefault(
            (run_job.inst2, int(target_angle)), set()
        ).add(target_port_spec)

    session.fanout_anchor_by_port_spec = {
        **session._build_static_fanout_anchors(),
        **session._build_static_fanout_target_anchors(),
    }
    traced_instance = session.config.diagnostics.trace_runway_instance
    if traced_instance:
        for port_spec, anchor in sorted(session.fanout_anchor_by_port_spec.items()):
            if port_spec.startswith(f"{traced_instance},"):
                print(
                    f"anchor_trace {port_spec} state=({anchor.state_x},{anchor.state_y}) "
                    f"angle={anchor.physical_angle} "
                    f"centerline={anchor.stub_centerline_um}"
                )
    session.fanout_stub_static_cells_by_spec: dict[str, set[tuple[int, int]]] = {
        port_spec: session._inflated_cells(anchor.stub_center_cells, int(session.commit_radius_cells))
        for port_spec, anchor in session.fanout_anchor_by_port_spec.items()
    }
    session.fanout_stub_center_cells: set[tuple[int, int]] = set()
    for anchor in session.fanout_anchor_by_port_spec.values():
        session.fanout_stub_center_cells.update(anchor.stub_center_cells)
    session.fanout_stub_static_cells: set[tuple[int, int]] = set()
    for cells in session.fanout_stub_static_cells_by_spec.values():
        session.fanout_stub_static_cells.update(cells)
    session.fanout_anchor_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst1},{job.port1}" in session.fanout_anchor_by_port_spec
        or f"{job.inst2},{job.port2}" in session.fanout_anchor_by_port_spec
    }
    session.fanout_anchor_source_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst1},{job.port1}" in session.fanout_anchor_by_port_spec
    }
    session.fanout_anchor_target_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst2},{job.port2}" in session.fanout_anchor_by_port_spec
    }

    session.dense_source_port_runway_length_by_spec = session._dense_source_port_runway_lengths(
        route_jobs
    )
    session.dense_target_port_runway_length_by_spec = session._dense_target_port_runway_lengths(
        route_jobs
    )
    traced_instance = session.config.diagnostics.trace_runway_instance
    if traced_instance:
        for port_spec, runway_length in sorted(
            session.dense_target_port_runway_length_by_spec.items()
        ):
            if port_spec.startswith(f"{traced_instance},"):
                print(f"runway_trace target {port_spec} length={runway_length}")
        for port_spec, runway_length in sorted(
            session.dense_source_port_runway_length_by_spec.items()
        ):
            if port_spec.startswith(f"{traced_instance},"):
                print(f"runway_trace source {port_spec} length={runway_length}")
    dense_port_runway_length_by_spec: dict[str, int] = dict(
        session.dense_source_port_runway_length_by_spec
    )
    for port_spec, runway_length in session.dense_target_port_runway_length_by_spec.items():
        existing_length = dense_port_runway_length_by_spec.get(port_spec)
        dense_port_runway_length_by_spec[port_spec] = max(
            int(existing_length) if existing_length is not None else 0,
            int(runway_length),
        )
    session.dense_source_cluster_specs_by_port_spec: dict[str, set[str]] = {}
    index = 0
    while index < len(route_jobs):
        job = route_jobs[index]
        if f"{job.inst1},{job.port1}" not in session.dense_source_port_runway_length_by_spec:
            index += 1
            continue
        run_end = index + 1
        while (
            run_end < len(route_jobs)
            and route_jobs[run_end].inst1 == job.inst1
            and f"{route_jobs[run_end].inst1},{route_jobs[run_end].port1}"
            in session.dense_source_port_runway_length_by_spec
        ):
            run_end += 1
        cluster_specs = {
            f"{run_job.inst1},{run_job.port1}"
            for run_job in route_jobs[index:run_end]
            if f"{run_job.inst1},{run_job.port1}"
            in session.dense_source_port_runway_length_by_spec
        }
        if len(cluster_specs) > 1:
            for port_spec in cluster_specs:
                session.dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)
        index = run_end
    if session.fanout_anchor_by_port_spec:
        static_stub_groups: dict[tuple[str, int], set[str]] = {}
        for port_spec, anchor in session.fanout_anchor_by_port_spec.items():
            instance_name = port_spec.split(",", 1)[0]
            static_stub_groups.setdefault(
                (instance_name, int(anchor.physical_angle) % 8),
                set(),
            ).add(port_spec)
        for cluster_specs in static_stub_groups.values():
            if len(cluster_specs) <= 1:
                continue
            for port_spec in cluster_specs:
                session.dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)

    return route_jobs, endpoint_port_specs_by_instance, dense_port_runway_length_by_spec
