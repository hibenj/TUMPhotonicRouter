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

from photonic_router.port_roles import INPUT, component_port_role

from .gds_circuit_components import arm_pair_path_length_um, register_generic_components
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
    """

    #: the generic component's name, or a callable of the extracted ports
    #: choosing it (a ring with one bus or two)
    component: Any
    port_names_by_side: dict[str, str]
    settings: Any = None
    generic_ports: Any = None

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
        # rotation: the first port's generic orientation must point along the original normal
        first_label = labels[0]
        first_name = mapping.port_names_by_side.get(
            first_label, mapping.port_names_by_side.get(first_label.split("_")[0])
        )
        if first_name is None:
            raise ValueError(f"{component.name}: no generic port for side {first_label!r}")
        gx, gy, g_angle = generic_ports.get(first_name, (0.0, 0.0, 180.0))
        # a generic port with orientation a faces direction a (its normal into
        # the waveguide); the rotation turns that onto the original normal
        original_angle = math.degrees(
            math.atan2(component.ports[0].normal[1], component.ports[0].normal[0])
        )
        rotation = float(round(((original_angle - g_angle) % 360.0) / 90.0) * 90 % 360)
        rad = math.radians(rotation)
        rx = gx * math.cos(rad) - gy * math.sin(rad)
        ry = gx * math.sin(rad) + gy * math.cos(rad)
        px, py = component.ports[0].position
        placement = Placement(x=px - rx, y=py - ry, rotation=rotation, mirror=False)
        schematic.add_instance(
            component.name, Instance(component=component_name, settings=settings), placement
        )
        for index, label in enumerate(labels):
            name = mapping.port_names_by_side.get(
                label, mapping.port_names_by_side.get(label.split("_")[0])
            )
            if name is None:
                raise ValueError(f"{component.name}: no generic port for side {label!r}")
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


def overlapping_instances(layout: Any, tolerance_um: float = 0.01) -> list[tuple[str, str]]:
    """The pairs of instances whose bounding boxes overlap (elements must never overlap).

    The owner's rule (2026-10-06, the second Clements mesh): a rebuilt circuit
    may not place two elements over each other, even where moving them apart
    departs from the original layout. Bounding boxes over all layers; a
    shared edge within ``tolerance_um`` does not count.
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


#: the extracted kinds that are a circuit's couplers (one optical port each)
COUPLER_KINDS = frozenset({"gc", "edge_coupler"})
#: the extracted kinds whose instance is a dead end: a sink of the circuit
STUB_NODE_KINDS = frozenset({"endcap", "open_end"})


def benchmark_metadata(
    schematic: Any,
    circuit: ExtractedCircuit,
    node_type_overrides: dict[str, str] | None = None,
) -> tuple[dict[str, str], dict[str, float]]:
    """``NODE_TYPES`` and ``INTERNAL_DELAYS_UM`` of the generated module, for the path-length
    analysis (`benchmark_metadata.load_benchmark_metadata`).

    A coupler whose net ends on the input side of the component it feeds
    (`port_roles.INPUT_PORTS_BY_COMPONENT`, the components' fan-in) is an
    ``input`` of the circuit, one on an output side an ``output``; an end
    cap or open end is an ``output`` too (a dead end is a sink: its stub
    hangs on an unused input port, where fan-in alone cannot tell the two
    ends apart); ``node_type_overrides`` (the private configuration) has the
    last word.
    The internal delays are the optical lengths through the components the
    light passes: the MMI's body
    and tapers, every straight's and heater's length, the arm pair's arm;
    couplers and rings carry 0 (a ring circuit is not matched).
    """

    kind_of = {component.name: component.kind for component in circuit.components}
    component_of = {name: inst.component for name, inst in schematic.netlist.instances.items()}
    node_types: dict[str, str] = {
        name: "output" for name, kind in kind_of.items() if kind in STUB_NODE_KINDS
    }
    for bundle in schematic.netlist.routes.values():
        for p1, p2 in bundle.links.items():
            for coupler_spec, other_spec in ((p1, p2), (p2, p1)):
                coupler, _ = coupler_spec.split(",")
                other, other_port = other_spec.split(",")
                if kind_of.get(coupler) not in COUPLER_KINDS or kind_of.get(other) in COUPLER_KINDS:
                    continue
                role = component_port_role(component_of.get(other, ""), other_port)
                if role is not None:
                    node_types[coupler] = "input" if role == INPUT else "output"
    node_types.update(node_type_overrides or {})
    delays: dict[str, float] = {}
    for name, instance in schematic.netlist.instances.items():
        settings = dict(instance.settings)
        if instance.component == "mmi2x2":
            delay = float(settings["length_mmi"]) + 2 * float(settings["length_taper"])
        elif instance.component == "arm_pair":
            delay = arm_pair_path_length_um(**settings)
        elif "length" in settings:
            delay = float(settings["length"])
        else:
            delay = 0.0
        delays[str(name)] = round(delay, 3)
    return node_types, delays


def write_benchmark_module(
    schematic: Any,
    path: str | Path,
    *,
    title: str,
    flags: tuple[str, ...],
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
) -> None:
    """Write a benchmark module (``build_schematic``, ``STABLE_ROUTING_FLAGS``, and the
    path-length metadata ``NODE_TYPES`` and ``INTERNAL_DELAYS_UM`` when given)."""

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
    lines.append(")")
    if node_types is not None:
        lines += ["", "NODE_TYPES: dict[str, str] = {"]
        lines += [f"    {name!r}: {kind!r}," for name, kind in node_types.items()]
        lines.append("}")
    if internal_delays_um is not None:
        lines += ["", "INTERNAL_DELAYS_UM: dict[str, float] = {"]
        lines += [f"    {name!r}: {delay!r}," for name, delay in internal_delays_um.items()]
        lines.append("}")
    lines += [
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
