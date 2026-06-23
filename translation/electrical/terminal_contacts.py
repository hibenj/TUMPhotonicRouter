"""Physical contact helpers for logical heater electrical terminals."""

from __future__ import annotations

from dataclasses import dataclass

from .types import BBox, ElectricalPortRef, ElectricalTerminal, Point, Side


@dataclass(frozen=True)
class TerminalAccessPath:
    """Continuous terminal adapter geometry before the gridded route tail."""

    contact_center: Point
    contact_bbox: BBox
    access_width_um: float
    adapter_points: tuple[Point, ...]
    route_tail_points: tuple[Point, ...]


def terminal_contact_seed_points(terminal: ElectricalTerminal) -> tuple[Point, ...]:
    """Return physical points where routing may enter a logical terminal."""

    if terminal.ports:
        return tuple(port.center for port in terminal.ports)
    return (terminal.center,)


def terminal_contact_seed_point_for_side(
    terminal: ElectricalTerminal,
    side: Side,
) -> Point:
    """Return the physical port center that faces a top/bottom access side."""

    port = select_terminal_port_for_side(terminal, side)
    return port.center if port is not None else terminal.center


def select_terminal_port_for_side(
    terminal: ElectricalTerminal,
    side: Side,
) -> ElectricalPortRef | None:
    """Choose the terminal port whose orientation faces the access side."""

    if not terminal.ports:
        return None
    target_orientation = 90.0 if side == "top" else 270.0
    return min(
        terminal.ports,
        key=lambda port: (
            _orientation_distance(port.orientation, target_orientation),
            port.name,
        ),
    )


def select_terminal_contact(
    terminal: ElectricalTerminal,
    route_start_um: Point | None,
    fallback_width_um: float,
) -> tuple[Point, BBox]:
    """Choose the physical terminal port that best matches a route start."""

    contacts = terminal_contact_bboxes(terminal, fallback_width_um)
    if not terminal.ports:
        return (terminal.center, contacts[0])
    if route_start_um is None:
        port = terminal.ports[0]
        return (port.center, contacts[0])
    route_start = route_start_um

    ranked_ports = sorted(
        zip(terminal.ports, contacts),
        key=lambda item: (
            _distance_sq(item[0].center, route_start),
            item[0].name,
        ),
    )
    port, bbox = ranked_ports[0]
    return (port.center, bbox)


def terminal_access_path(
    terminal: ElectricalTerminal,
    route_points_um: tuple[Point, ...],
    fallback_width_um: float,
) -> TerminalAccessPath:
    """Build a port-exact adapter from one physical port to the route tail.

    The discrete router may start inside the terminal opening because terminals
    are not grid-aligned. Realization should not draw that snapped portion over
    the heater terminal body; instead it exits the selected physical port first
    and joins the first useful route point outside the logical terminal bbox.
    """

    route_start_um = route_points_um[0] if route_points_um else None
    contact, contact_bbox, port = _select_terminal_contact_with_port(
        terminal,
        route_start_um,
        fallback_width_um,
    )
    trimmed_tail = _trim_route_points_inside_terminal(
        route_points_um,
        terminal,
        fallback_width_um,
    )
    join_point = trimmed_tail[0] if trimmed_tail else route_start_um
    anchor = _port_anchor_point(
        terminal,
        port,
        contact,
        fallback_width_um,
        join_point,
    )
    adapter_points = _dedupe_points(
        _manhattan_adapter_points(contact, anchor, join_point)
    )
    return TerminalAccessPath(
        contact_center=contact,
        contact_bbox=contact_bbox,
        access_width_um=_contact_access_width(port, fallback_width_um),
        adapter_points=adapter_points,
        route_tail_points=trimmed_tail,
    )


def terminal_contact_bboxes(
    terminal: ElectricalTerminal,
    fallback_width_um: float,
) -> tuple[BBox, ...]:
    """Return physical port contact boxes for a logical terminal."""

    if terminal.ports:
        return tuple(
            port_contact_bbox(port, fallback_width_um)
            for port in terminal.ports
        )
    return (_fallback_terminal_bbox(terminal, fallback_width_um),)


def terminal_access_keepout_bbox(
    terminal: ElectricalTerminal,
    fallback_width_um: float,
) -> BBox:
    """Return the local bbox where snapped route starts should be suppressed."""

    margin = max(fallback_width_um / 2.0, 0.0)
    return (
        terminal.bbox[0] - margin,
        terminal.bbox[1] - margin,
        terminal.bbox[2] + margin,
        terminal.bbox[3] + margin,
    )


def port_contact_bbox(
    port: ElectricalPortRef,
    fallback_width_um: float,
) -> BBox:
    """Return a compact contact box centered on one physical port."""

    width = float(port.width or 0.0)
    if width <= 0.0:
        width = fallback_width_um
    half_width = max(width / 2.0, 0.0)
    x, y = port.center
    return (x - half_width, y - half_width, x + half_width, y + half_width)


def _select_terminal_contact_with_port(
    terminal: ElectricalTerminal,
    route_start_um: Point | None,
    fallback_width_um: float,
) -> tuple[Point, BBox, ElectricalPortRef | None]:
    contacts = terminal_contact_bboxes(terminal, fallback_width_um)
    if not terminal.ports:
        return (terminal.center, contacts[0], None)
    if route_start_um is None:
        port = terminal.ports[0]
        return (port.center, contacts[0], port)
    route_start = route_start_um

    ranked_ports = sorted(
        zip(terminal.ports, contacts),
        key=lambda item: (
            _distance_sq(item[0].center, route_start),
            item[0].name,
        ),
    )
    port, bbox = ranked_ports[0]
    return (port.center, bbox, port)


def _trim_route_points_inside_terminal(
    route_points_um: tuple[Point, ...],
    terminal: ElectricalTerminal,
    fallback_width_um: float,
) -> tuple[Point, ...]:
    keepout = terminal_access_keepout_bbox(terminal, fallback_width_um)
    first_outside_index = 0
    for index, point in enumerate(route_points_um):
        if not _point_in_bbox(point, keepout):
            first_outside_index = index
            break
    else:
        return route_points_um[-1:] if route_points_um else ()
    return route_points_um[first_outside_index:]


def _port_anchor_point(
    terminal: ElectricalTerminal,
    port: ElectricalPortRef | None,
    contact: Point,
    fallback_width_um: float,
    join_point: Point | None,
) -> Point:
    direction = _port_direction(port, contact, terminal, join_point)
    margin = max(fallback_width_um, float(getattr(port, "width", 0.0) or 0.0))
    xmin, ymin, xmax, ymax = terminal_access_keepout_bbox(terminal, fallback_width_um)
    if direction == (1, 0):
        return (max(contact[0], xmax), contact[1])
    if direction == (-1, 0):
        return (min(contact[0], xmin), contact[1])
    if direction == (0, 1):
        return (contact[0], max(contact[1], ymax))
    if direction == (0, -1):
        return (contact[0], min(contact[1], ymin))
    if join_point is None:
        return contact
    return (
        contact[0] + direction[0] * margin,
        contact[1] + direction[1] * margin,
    )


def _port_direction(
    port: ElectricalPortRef | None,
    contact: Point,
    terminal: ElectricalTerminal,
    join_point: Point | None,
) -> tuple[int, int]:
    orientation = port.orientation if port is not None else None
    if orientation is not None:
        normalized = round(float(orientation) / 90.0) % 4
        if normalized == 0:
            return (1, 0)
        if normalized == 1:
            return (0, 1)
        if normalized == 2:
            return (-1, 0)
        return (0, -1)

    dx = contact[0] - terminal.center[0]
    dy = contact[1] - terminal.center[1]
    if abs(dx) >= abs(dy) and dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    if join_point is None:
        return (0, 0)
    route_dx = join_point[0] - contact[0]
    route_dy = join_point[1] - contact[1]
    if abs(route_dx) >= abs(route_dy) and route_dx != 0:
        return (1 if route_dx > 0 else -1, 0)
    if route_dy != 0:
        return (0, 1 if route_dy > 0 else -1)
    return (0, 0)


def _manhattan_adapter_points(
    contact: Point,
    anchor: Point,
    join_point: Point | None,
) -> tuple[Point, ...]:
    if join_point is None:
        return (contact, anchor)
    if anchor[0] == join_point[0] or anchor[1] == join_point[1]:
        return (contact, anchor, join_point)
    via = (anchor[0], join_point[1])
    return (contact, anchor, via, join_point)


def _dedupe_points(points: tuple[Point, ...]) -> tuple[Point, ...]:
    deduped: list[Point] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def _point_in_bbox(point: Point, bbox: BBox) -> bool:
    return bbox[0] <= point[0] <= bbox[2] and bbox[1] <= point[1] <= bbox[3]


def _fallback_terminal_bbox(
    terminal: ElectricalTerminal,
    fallback_width_um: float,
) -> BBox:
    xmin, ymin, xmax, ymax = terminal.bbox
    min_half_size = fallback_width_um / 2.0
    center_x, center_y = terminal.center
    return (
        min(xmin, center_x - min_half_size),
        min(ymin, center_y - min_half_size),
        max(xmax, center_x + min_half_size),
        max(ymax, center_y + min_half_size),
    )


def _distance_sq(left: Point, right: Point) -> float:
    dx = left[0] - right[0]
    dy = left[1] - right[1]
    return dx * dx + dy * dy


def _contact_access_width(
    port: ElectricalPortRef | None,
    fallback_width_um: float,
) -> float:
    if port is None or port.width is None or port.width <= 0.0:
        return fallback_width_um
    return min(float(port.width), fallback_width_um)


def _orientation_distance(left: float | None, right: float) -> float:
    if left is None:
        return 360.0
    delta = abs((float(left) - right) % 360.0)
    return min(delta, 360.0 - delta)
