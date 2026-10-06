"""Rebuild an extracted circuit with generic-PDK components and write it as a benchmark.

Private-circuits plan, 2026-10-05. ``generic_schematic`` replaces every
extracted component by a generic gdsfactory component parameterized to
reproduce the original port geometry (``GenericMapping``), rotated and moved so
that its ports coincide with the original ones; ``write_benchmark_module``
writes the result as a benchmark module with ``build_schematic``. The
extraction itself is ``gds_circuit_extraction``.
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .gds_circuit_components import register_generic_components
from .gds_circuit_extraction import ExtractedCircuit, ExtractedComponent, ExtractedPort


@dataclass(frozen=True)
class GenericMapping:
    """How a kind becomes a generic component.

    ``port_names_by_side`` maps the original port's side in the component
    frame (``left``/``right``/``top``/``bottom``, with ``_lower``/``_upper``
    suffixes when two ports share a side) to the generic port name;
    ``settings`` derives the component settings from the extracted
    component (its ports and measured geometry; ``None``: none);
    ``generic_ports`` gives the generic component's port
    positions and orientations for those settings (name -> (x, y, angle)).
    ``match_ports_by_position`` replaces the side mapping for a device whose
    ports do not sit on its sides (a two-port grating coupler's leave at 45
    degrees): the device faces the mean of its port normals and every
    original port gets the name of the nearest generic port (see
    `_ports_matched_by_position`).
    """

    #: the generic component's name, or a callable of the extracted ports
    #: choosing it (a ring with one bus or two)
    component: Any
    port_names_by_side: dict[str, str]
    settings: Any = None
    generic_ports: Any = None
    match_ports_by_position: bool = False

    def component_name(self, ports: tuple[ExtractedPort, ...]) -> str:
        return self.component(ports) if callable(self.component) else self.component


def side_labels(ports: tuple[ExtractedPort, ...]) -> list[str]:
    """``left``, ``right_lower``, ``right_upper``, ...: the side of each port, disambiguated."""

    by_side: dict[str, list[int]] = {}
    for index, port in enumerate(ports):
        by_side.setdefault(port.local_side, []).append(index)
    labels = [""] * len(ports)
    for side, indexes in by_side.items():
        if len(indexes) == 1:
            labels[indexes[0]] = side
            continue
        axis = 1 if side in ("left", "right") else 0
        ordered = sorted(indexes, key=lambda i: ports[i].local_position[axis])
        for rank, index in enumerate(ordered):
            suffix = "lower" if rank == 0 else "upper" if rank == len(ordered) - 1 else f"mid{rank}"
            labels[index] = f"{side}_{suffix}"
    return labels


#: a net shorter than this joins two abutting components and is not emitted
ABUT_UM = 0.5

#: how ``generic_schematic`` orders the nets: by increasing port distance
#: (``chord``), decreasing (``chord_desc``), or as extracted (``extraction``)
NetOrder = str


def generic_schematic(
    circuit: ExtractedCircuit,
    mappings: dict[str, GenericMapping],
    net_order: NetOrder = "chord",
) -> Any:
    """Rebuild the circuit as a gdsfactory Schematic of generic components placed by their ports.

    The flow routes nets in declaration order within their topological
    layers and is sensitive to it on these circuits (a short connection
    routed after a long one across its corridor may find its port walled
    in, a long one routed after many short ones may find no corridor), so
    the order is a parameter; ``chord`` (shortest first) is the default.
    """

    from gdsfactory.gpdk import get_generic_pdk
    from gdsfactory.schematic import Instance, Net, Placement, Schematic

    get_generic_pdk().activate()  # the Instance validation resolves the component names
    register_generic_components()
    schematic = Schematic()
    port_name_of: dict[tuple[str, int], str] = {}
    for component in circuit.components:
        mapping = mappings.get(component.kind)
        if mapping is None:
            raise ValueError(f"no generic mapping for kind {component.kind!r} ({component.name})")
        labels = side_labels(component.ports)
        settings = mapping.settings(component) if mapping.settings else {}
        component_name = mapping.component_name(component.ports)
        generic_ports = _generic_component_ports(component_name, settings)
        if mapping.generic_ports is not None:
            generic_ports = {**generic_ports, **mapping.generic_ports(settings)}
        if mapping.match_ports_by_position:
            names, rotation = _ports_matched_by_position(component, generic_ports)
        else:
            names = _port_names_by_side(mapping, component, labels)
            # the first port's generic orientation must point along the original normal
            rotation = _rotation_onto(
                component.ports[0].normal, generic_ports.get(names[0], (0.0, 0.0, 180.0))[2]
            )
        first_name = names[0]
        gx, gy, _ = generic_ports.get(first_name, (0.0, 0.0, 180.0))
        rad = math.radians(rotation)
        rx = gx * math.cos(rad) - gy * math.sin(rad)
        ry = gx * math.sin(rad) + gy * math.cos(rad)
        px, py = component.ports[0].position
        placement = Placement(x=px - rx, y=py - ry, rotation=rotation, mirror=False)
        schematic.add_instance(
            component.name, Instance(component=component_name, settings=settings), placement
        )
        for index, name in enumerate(names):
            port_name_of[(component.name, index)] = name
    ports_by_name = {component.name: component.ports for component in circuit.components}

    def chord_um(net: Any) -> float:
        (a_name, a_index), (b_name, b_index) = net.endpoints
        ax, ay = ports_by_name[a_name][a_index].position
        bx, by = ports_by_name[b_name][b_index].position
        return math.hypot(bx - ax, by - ay)

    two_ported = [net for net in circuit.nets if len(net.endpoints) == 2]
    if net_order == "chord":
        two_ported.sort(key=lambda net: (chord_um(net), net.name))
    elif net_order == "chord_desc":
        two_ported.sort(key=lambda net: (-chord_um(net), net.name))
    elif net_order != "extraction":
        raise ValueError(f"unknown net order {net_order!r}")
    for net in two_ported:
        if chord_um(net) < ABUT_UM:
            continue  # the components abut; nothing to route
        (a_name, a_index), (b_name, b_index) = net.endpoints
        p1 = f"{a_name},{port_name_of[(a_name, a_index)]}"
        p2 = f"{b_name},{port_name_of[(b_name, b_index)]}"
        schematic.add_net(
            Net(
                p1=p1,
                p2=p2,
                name=f"{a_name}_{port_name_of[(a_name, a_index)]}"
                f"_to_{b_name}_{port_name_of[(b_name, b_index)]}",
            )
        )
    return schematic


_GENERIC_PORT_CACHE: dict[
    tuple[str, tuple[tuple[str, Any], ...]], dict[str, tuple[float, float, float]]
] = {}


def _port_names_by_side(
    mapping: GenericMapping, component: ExtractedComponent, labels: list[str]
) -> list[str]:
    names = []
    for label in labels:
        name = mapping.port_names_by_side.get(
            label, mapping.port_names_by_side.get(label.split("_")[0])
        )
        if name is None:
            raise ValueError(f"{component.name}: no generic port for side {label!r}")
        names.append(name)
    return names


def _rotation_onto(normal: tuple[float, float], generic_angle: float) -> float:
    """The 90-degree multiple turning a generic port of ``generic_angle`` onto ``normal``.

    A generic port with orientation a faces direction a (its normal into the
    waveguide); the rotation turns that onto the original normal.
    """

    original_angle = math.degrees(math.atan2(normal[1], normal[0]))
    return float(round(((original_angle - generic_angle) % 360.0) / 90.0) * 90 % 360)


def _ports_matched_by_position(
    component: ExtractedComponent, generic_ports: dict[str, tuple[float, float, float]]
) -> tuple[list[str], float]:
    """Generic port names, and the rotation, for a device whose ports leave it obliquely.

    A two-port grating coupler's ports leave at 45 degrees, so neither their
    sides nor their order says which generic port they are. The device faces
    the mean of its port normals; the generic component is turned so that its
    first optical port faces that way, and every original port, taken relative
    to the ports' centroid and turned back into the generic frame, gets the
    name of the nearest generic port not taken yet.
    """

    ports = component.ports
    mean_normal = (
        sum(port.normal[0] for port in ports),
        sum(port.normal[1] for port in ports),
    )
    generic_names = sorted(generic_ports)
    rotation = _rotation_onto(mean_normal, generic_ports[generic_names[0]][2])
    centroid = (
        sum(port.position[0] for port in ports) / len(ports),
        sum(port.position[1] for port in ports) / len(ports),
    )
    generic_centroid = (
        sum(generic_ports[name][0] for name in generic_names) / len(generic_names),
        sum(generic_ports[name][1] for name in generic_names) / len(generic_names),
    )
    rad = math.radians(-rotation)
    names: list[str] = []
    for port in ports:
        dx, dy = port.position[0] - centroid[0], port.position[1] - centroid[1]
        x = dx * math.cos(rad) - dy * math.sin(rad)
        y = dx * math.sin(rad) + dy * math.cos(rad)
        free = [name for name in generic_names if name not in names]
        if not free:
            raise ValueError(f"{component.name}: more ports than the generic component has")
        names.append(
            min(
                free,
                key=lambda name: math.hypot(
                    generic_ports[name][0] - generic_centroid[0] - x,
                    generic_ports[name][1] - generic_centroid[1] - y,
                ),
            )
        )
    return names, rotation


def overlapping_instances(layout: Any, tolerance_um: float = 0.01) -> list[tuple[str, str]]:
    """The pairs of instances whose bounding boxes overlap (elements must never overlap).

    The owner's rule (2026-10-06, the second Clements mesh): a rebuilt circuit
    may not place two elements over each other. Bounding boxes over all
    layers, so a pair that touches only through a heater's metal shows up
    too; a shared edge within ``tolerance_um`` does not count.
    """

    boxes = []
    for instance in layout.insts:
        box = instance.dbbox()
        boxes.append((str(instance.name), box.left, box.bottom, box.right, box.top))
    pairs: list[tuple[str, str]] = []
    for index, (name_a, left_a, bottom_a, right_a, top_a) in enumerate(boxes):
        for name_b, left_b, bottom_b, right_b, top_b in boxes[index + 1 :]:
            overlap_x = min(right_a, right_b) - max(left_a, left_b)
            overlap_y = min(top_a, top_b) - max(bottom_a, bottom_b)
            if overlap_x > tolerance_um and overlap_y > tolerance_um:
                pairs.append((name_a, name_b))
    return pairs


def _generic_component_ports(
    component: str, settings: dict[str, Any]
) -> dict[str, tuple[float, float, float]]:
    """The generic component's optical ports (name -> (x, y, orientation)), read from gdsfactory."""

    key = (component, tuple(sorted(settings.items())))
    cached = _GENERIC_PORT_CACHE.get(key)
    if cached is not None:
        return cached
    import gdsfactory as gf

    built = gf.get_component(component, **settings)
    ports = {
        port.name: (float(port.center[0]), float(port.center[1]), float(port.orientation))
        for port in built.ports
        if port.port_type == "optical"
    }
    _GENERIC_PORT_CACHE[key] = ports
    return ports


def write_benchmark_module(
    schematic: Any, path: str | Path, *, title: str, flags: tuple[str, ...]
) -> None:
    """Write a benchmark module (``build_schematic``, ``STABLE_ROUTING_FLAGS``)."""

    lines = [
        f'"""{title}',
        "",
        "Generated by translation/gds_circuit_extraction.py from a confidential tape-out",
        "GDS (private-circuits plan, 2026-10-05); never versioned.",
        '"""',
        "",
        "from __future__ import annotations",
        "",
        "from gdsfactory.gpdk import get_generic_pdk",
        "from gdsfactory.schematic import Instance, Net, Placement, Schematic",
        "",
        "from translation.gds_circuit_components import register_generic_components",
        "",
        "STABLE_ROUTING_FLAGS: tuple[str, ...] = (",
    ]
    for flag in flags:
        lines.append(f"    {flag!r},")
    lines += [")", "", "INSTANCES = ("]
    for name, instance in schematic.netlist.instances.items():
        placement = schematic.placements[name]
        lines.append(
            f"    ({name!r}, {instance.component!r}, {dict(instance.settings)!r}, "
            f"{placement.x!r}, {placement.y!r}, {placement.rotation!r}),"
        )
    lines += [")", "", "NETS = ("]
    for net_name, bundle in schematic.netlist.routes.items():
        for p1, p2 in bundle.links.items():
            lines.append(f"    ({net_name!r}, {p1!r}, {p2!r}),")
    lines += [
        ")",
        "",
        "",
        "def build_schematic() -> Schematic:",
        "    get_generic_pdk().activate()",
        "    register_generic_components()",
        "    schematic = Schematic()",
        "    for name, component, settings, x, y, rotation in INSTANCES:",
        "        schematic.add_instance(",
        "            name,",
        "            Instance(component=component, settings=dict(settings)),",
        "            Placement(x=x, y=y, rotation=rotation, mirror=False),",
        "        )",
        "    for net_name, p1, p2 in NETS:",
        "        schematic.add_net(Net(p1=p1, p2=p2, name=net_name))",
        "    return schematic",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def _port_span_and_separation(ports: tuple[ExtractedPort, ...]) -> tuple[float, float]:
    xs = [port.local_position[0] for port in ports]
    span = max(xs) - min(xs)
    left = sorted(port.local_position[1] for port in ports if port.local_side == "left")
    separation = (left[-1] - left[0]) if len(left) >= 2 else 0.0
    return span, separation


def _mmi_settings(component: ExtractedComponent) -> dict[str, float]:
    span, separation = _port_span_and_separation(component.ports)
    length_taper = 10.0
    width_taper = 1.0
    gap = max(separation - width_taper, 0.25)
    return {
        "length_taper": length_taper,
        "length_mmi": max(span - 2 * length_taper, 1.0),
        "gap_mmi": gap,
        "width_taper": width_taper,
        "width_mmi": max(2.5, gap + 2 * width_taper + 1.0),
    }


def _mmi_ports(settings: dict[str, float]) -> dict[str, tuple[float, float, float]]:
    half = (settings["gap_mmi"] + settings["width_taper"]) / 2
    x_out = settings["length_mmi"] + settings["length_taper"]
    x_in = -settings["length_taper"]
    return {
        "o1": (x_in, -half, 180.0),
        "o2": (x_in, half, 180.0),
        "o3": (x_out, half, 0.0),
        "o4": (x_out, -half, 0.0),
    }


def _heater_settings(component: ExtractedComponent) -> dict[str, Any]:
    """The heater's length from its port span, without the generic via stacks.

    The generic heater's contact via stacks stick out 16 um past both
    waveguide ends and are obstacles; the foundry heaters of the owner's
    circuits keep their metal along the waveguide, and an MMI output often
    sits 20 um before a heater input. Without the stacks the heater has no
    electrical ports, which these photonic-only benchmarks do not need.
    """

    span, _ = _port_span_and_separation(component.ports)
    return {"length": round(span, 3), "via_stack": None}


def _heater_ports(settings: dict[str, float]) -> dict[str, tuple[float, float, float]]:
    return {"o1": (0.0, 0.0, 180.0), "o2": (settings["length"], 0.0, 0.0)}


def _ring_settings(component: ExtractedComponent) -> dict[str, float]:
    """A one-bus ring's coupling length so that its bus ports span the original ones.

    ``ring_single`` puts ``o2`` at x = radius + 3 and ``o1`` at
    -(radius + 3) - length_x for the default radius of 10 um (measured on
    the generic PDK: o1 at -13 - length_x, o2 at 13); a two-bus ring keeps
    the generic defaults.
    """

    if len(component.ports) != 2:
        return {}
    span, _separation = _port_span_and_separation(component.ports)
    return {"length_x": max(span - 26.0, 0.0)}


def _waveguide_settings(component: ExtractedComponent) -> dict[str, float]:
    span, _separation = _port_span_and_separation(component.ports)
    return {"length": round(span, 3)}


def _arm_pair_settings(component: ExtractedComponent, bend_radius_um: float) -> dict[str, float]:
    """An arm pair's arms from the extractor's geometry, its pitches from the ports."""

    by_side: dict[str, list[float]] = {}
    for port in component.ports:
        by_side.setdefault(port.local_side, []).append(port.local_position[1])
    left, right = by_side.get("left", [0.0]), by_side.get("right", [0.0])
    geometry = component.geometry
    return {
        "arm_length": geometry["arm_length_um"],
        "arm_separation": geometry["arm_separation_um"],
        "extension": geometry["extension_um"],
        "pitch_in": round(max(left) - min(left), 3) if len(left) == 2 else 2.0,
        "pitch_out": round(max(right) - min(right), 3) if len(right) == 2 else 2.0,
        "bend_radius": bend_radius_um,
    }


def _stub_settings(component: ExtractedComponent) -> dict[str, float]:
    return {"length": STUB_LENGTH_UM}


def _grating_coupler_2d_settings(component: ExtractedComponent) -> dict[str, float]:
    """The pitch of the two ports; the device's size is generic."""

    (ax, ay), (bx, by) = (port.position for port in component.ports[:2])
    return {"port_pitch": round(math.hypot(bx - ax, by - ay), 3)}


#: an end cap or an open end is rebuilt as a straight of this length whose
#: ``o1`` sits at the waveguide's end; its ``o2`` stays unconnected
STUB_LENGTH_UM = 10.0


def default_generic_mappings(bend_radius_um: float = 5.0) -> dict[str, GenericMapping]:
    """The generic replacements: MMI2x2, heater, bare arm, arm pair, couplers, ring, stubs.

    ``bend_radius_um`` is the routing bend radius of the benchmark; the arm
    pair is drawn with the same 90-degree bends the router uses.
    """

    any_side_o1 = dict.fromkeys(("left", "right", "top", "bottom"), "o1")
    stub = GenericMapping(
        component="straight", port_names_by_side=any_side_o1, settings=_stub_settings
    )
    return {
        "endcap": stub,
        "open_end": stub,
        # a bare waveguide arm kept as a fixed component (a heater cell
        # without metal, a delay arm): a straight of its span
        "waveguide": GenericMapping(
            component="straight",
            port_names_by_side={"left": "o1", "right": "o2"},
            settings=_waveguide_settings,
        ),
        # an MZI arm pair with its S-bends (translation/gds_circuit_components.py)
        "arm_pair": GenericMapping(
            component="arm_pair",
            port_names_by_side={
                "left_lower": "o1",
                "left_upper": "o2",
                "right_upper": "o3",
                "right_lower": "o4",
            },
            settings=functools.partial(_arm_pair_settings, bend_radius_um=bend_radius_um),
        ),
        "mmi2x2": GenericMapping(
            component="mmi2x2",
            port_names_by_side={
                "left_lower": "o1",
                "left_upper": "o2",
                "right_upper": "o3",
                "right_lower": "o4",
            },
            settings=_mmi_settings,
        ),
        "heater": GenericMapping(
            component="straight_heater_metal",
            port_names_by_side={"left": "o1", "right": "o2"},
            settings=_heater_settings,
        ),
        "gc": GenericMapping(
            component="grating_coupler_te",
            port_names_by_side=any_side_o1,
        ),
        "gc_2d": GenericMapping(
            component="grating_coupler_2d",
            port_names_by_side={},
            settings=_grating_coupler_2d_settings,
            match_ports_by_position=True,
        ),
        "edge_coupler": GenericMapping(
            component="edge_coupler_silicon",
            port_names_by_side=any_side_o1,
        ),
        "ring": GenericMapping(
            component=lambda ports: "ring_single" if len(ports) == 2 else "ring_double",
            port_names_by_side={
                "left": "o1",
                "right": "o2",
                "left_lower": "o1",
                "right_lower": "o2",
                "left_upper": "o3",
                "right_upper": "o4",
            },
            settings=_ring_settings,
        ),
    }
