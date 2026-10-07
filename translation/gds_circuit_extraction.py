"""Extract a circuit (components, placements, nets) from a hierarchical GDS.

Private-circuits plan, 2026-10-05. The owner's tape-out circuits are IPKISS-style
hierarchies: foundry component cells, sub-circuit containers and one leaf cell
per waveguide. ``extract_circuit`` classifies the leaf cells by name
(``ExtractionConfig.rules``, ordered, first match wins; a container named by a
``CompositeRule`` is one component as a whole), merges every waveguide cell's
shapes on the core layers into one region so that each connected polygon is one
net, and finds a net's ports as the polygon's short edges (one waveguide width)
that touch a component's shapes, or that end within a tolerance of an empty
placeholder cell's origin, or (``open_ends``) that touch nothing at all: an
I/O cell missing from the file. ``gds_circuit_rebuild`` rebuilds the result
with generic-PDK components. Nothing here knows any foundry cell name: those
live in the private configurations outside the repository.
"""

from __future__ import annotations

import dataclasses
import math
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import klayout.db as kdb

Point = tuple[float, float]


@dataclass(frozen=True)
class ClassificationRule:
    """A leaf cell whose name contains any of ``substrings`` (case-insensitive) is ``kind``."""

    kind: str
    substrings: tuple[str, ...]


@dataclass(frozen=True)
class CompositeRule:
    """A container cell whose name contains ``substring`` is one component of ``kind``."""

    kind: str
    substring: str


@dataclass(frozen=True)
class ArmPairRule:
    """Two arms of ``kinds`` (one each or both of one kind) with the same x-span,
    at most ``max_separation_um`` apart, are one component of ``kind`` together
    with the ``extension_um`` of S-bends on both sides: an MZI arm pair that
    enters and leaves as a parallel pair (the owner's 2026-10-06 review of the
    beam-splitter circuit). The nets are cut at the pair's box; a component
    port lying on that box (an MMI feeding the pair directly) becomes an
    abutting net of zero length."""

    kinds: tuple[str, ...]
    max_separation_um: float
    extension_um: float
    kind: str = "arm_pair"
    #: the two arms must be of different kinds (a heater and a bare arm); two
    #: stacked single-arm paths of one kind are not a pair
    distinct_kinds: bool = True


@dataclass(frozen=True)
class ExtractionConfig:
    core_layers: tuple[tuple[int, int], ...]
    rules: tuple[ClassificationRule, ...]
    composites: tuple[CompositeRule, ...] = ()
    #: kinds whose cells may have no geometry; their port is the instance origin
    placeholder_kinds: frozenset[str] = frozenset()
    #: kinds that terminate a waveguide (end caps); such a net is reported as
    #: a stub and rebuilt with a short straight at that end
    stub_kinds: frozenset[str] = frozenset({"endcap", "open_end"})
    #: a facet that touches nothing becomes a one-port component of kind
    #: ``open_end`` (an I/O cell that is not in the file) instead of leaving
    #: the net one-ported
    open_ends: bool = False
    #: when set, a cell classified ``heater`` that draws nothing on these
    #: layers is a bare arm of the same length and becomes
    #: ``heater_without_metal_kind``: a fixed waveguide with two ports (the
    #: owner's 2026-10-06 review of the beam-splitter circuit, whose MZIs
    #: pair a metal heater with a bare arm of the same cell family)
    heater_metal_layers: tuple[tuple[int, int], ...] = ()
    heater_without_metal_kind: str = "waveguide"
    #: arm pairs kept as one component with two inputs and two outputs
    arm_pairs: tuple[ArmPairRule, ...] = ()
    #: kinds recorded but neither components of the benchmark nor nets
    skip_kinds: frozenset[str] = frozenset({"bondpad", "heater_metal"})
    #: an unmatched leaf cell is a waveguide piece of this kind
    default_kind: str = "net"
    #: kinds whose components are split into one single-port component per port
    #: (a two-port polarization-splitting coupler becomes two couplers of the
    #: mapped kind), kind -> kind of the parts
    split_kinds: dict[str, str] = field(default_factory=dict)
    #: the parts of a split two-port cell are moved apart, symmetrically about
    #: their centre along the line between their ports, until their ports are
    #: this far apart (0: left where they are). The generic replacement of a
    #: part is wider than the original pitch (a generic TE grating coupler is
    #: 26 um tall, a 2D coupler's ports 9.9 um apart), and elements must never
    #: overlap (owner's rule, 2026-10-06), even where the rebuilt layout then
    #: departs from the original
    split_pitch_um: float = 0.0
    facet_width_um: tuple[float, float] = (0.3, 2.0)
    touch_tolerance_um: float = 0.03
    placeholder_tolerance_um: float = 5.0
    #: gaps up to this width between net pieces are closed before the nets
    #: are read off (a layer transition drawn only on a layer that is not
    #: the core); 0 keeps the pieces apart. Keep it below the smallest
    #: coupling gap between distinct waveguides of the file.
    net_gap_um: float = 0.0

    def kind_of(self, cell_name: str) -> str:
        lowered = cell_name.lower()
        for rule in self.rules:
            if any(token.lower() in lowered for token in rule.substrings):
                return rule.kind
        return self.default_kind

    def composite_kind(self, cell_name: str) -> str | None:
        lowered = cell_name.lower()
        for rule in self.composites:
            if rule.substring.lower() in lowered:
                return rule.kind
        return None


@dataclass(frozen=True)
class ExtractedPort:
    position: Point
    #: unit vector from the component into the waveguide
    normal: Point
    #: the position in the component's own frame (before its placement)
    local_position: Point
    local_normal: Point

    @property
    def local_side(self) -> str:
        nx, ny = self.local_normal
        if abs(nx) >= abs(ny):
            return "left" if nx < 0 else "right"
        return "bottom" if ny < 0 else "top"


@dataclass(frozen=True)
class ExtractedComponent:
    name: str
    kind: str
    cell_name: str
    origin: Point
    rotation_deg: float
    mirror: bool
    ports: tuple[ExtractedPort, ...]
    #: measured dimensions the generic replacement needs beyond the ports
    #: (an arm pair's arm length, separation and S-bend extension)
    geometry: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ExtractedNet:
    name: str
    endpoints: tuple[tuple[str, int], ...]
    stub: bool = False
    note: str = ""
    #: the original waveguide's length: its polygon area over its waveguide
    #: width, the narrowest end facet (0 for an abutting net); what the design
    #: routed, for comparing the original's path lengths with the rebuild's
    length_um: float = 0.0


@dataclass
class ExtractedCircuit:
    components: list[ExtractedComponent]
    nets: list[ExtractedNet]
    skipped: Counter = field(default_factory=Counter)
    fragments: int = 0

    def report(self) -> str:
        kinds = Counter(component.kind for component in self.components)
        routed = [net for net in self.nets if len(net.endpoints) == 2 and not net.stub]
        stubs = [net for net in self.nets if net.stub]
        odd = [net for net in self.nets if len(net.endpoints) != 2 and not net.stub]
        lines = [
            f"components: {dict(sorted(kinds.items()))}",
            (
                f"nets: {len(routed)} two-ported, {len(stubs)} ending on a stub, "
                f"{len(odd)} to account for, {self.fragments} port-less fragments"
            ),
            f"skipped cells: {dict(sorted(self.skipped.items()))}",
        ]
        for net in odd[:20]:
            lines.append(f"  {net.name}: {net.endpoints} {net.note}")
        return "\n".join(lines)


@dataclass(frozen=True)
class _Leaf:
    path: str
    cell: Any  # None for a synthetic open end
    trans: Any
    kind: str
    composite: bool

    @property
    def cell_name(self) -> str:
        return self.cell.name if self.cell is not None else self.path


@dataclass(frozen=True)
class _Component:
    """A placed component cell with its absolute shapes (see ``_leaf_region``)."""

    leaf: _Leaf
    region: Any
    bbox: Any  # kdb.DBox, None when the cell has no shapes at all
    #: no shapes on the core layers: a foundry black box drawn elsewhere
    black_box: bool
    geometry: dict[str, float] = field(default_factory=dict)


def _flatten(layout: Any, top: Any, config: ExtractionConfig) -> list[_Leaf]:
    leaves: list[_Leaf] = []

    def walk(cell: Any, trans: Any, path: str) -> None:
        for inst in cell.each_inst():
            child = layout.cell(inst.cell_index)
            child_trans = trans * inst.dcplx_trans
            child_path = f"{path}/{child.name}"
            composite = config.composite_kind(child.name)
            if composite is not None:
                leaves.append(_Leaf(child_path, child, child_trans, composite, True))
            elif child.child_instances() == 0:
                leaves.append(
                    _Leaf(child_path, child, child_trans, config.kind_of(child.name), False)
                )
            else:
                walk(child, child_trans, child_path)

    walk(top, kdb.DCplxTrans(), "")
    return leaves


def _cell_region(layout: Any, cell: Any, trans: Any, layers: list[int] | None) -> Any:
    """All shapes of ``cell`` (recursively) on ``layers`` (every layer if None), absolute."""

    region = kdb.Region()
    dbu = layout.dbu
    for layer_index in layers if layers is not None else layout.layer_indexes():
        for shape in cell.begin_shapes_rec(layer_index):
            polygon = shape.shape().dpolygon
            if polygon is None:
                continue
            region.insert(polygon.transformed(shape.dtrans()).transformed(trans).to_itype(dbu))
    return region


def _leaf_region(layout: Any, leaf: _Leaf, core_indexes: list[int], fallback: bool) -> Any:
    """The leaf's shapes on the first core layer it draws on, in absolute coordinates.

    The core layers are tried in the configured order and only one is used:
    merging a waveguide's core with a wider layer drawn along it (a rib slab,
    a cladding window) would swallow the sub-micron end facets the port
    detection looks for and fuse neighbouring nets. A leaf without shapes on
    any core layer gets its other layers when ``fallback`` is set (foundry
    black boxes drawn elsewhere) and an empty region otherwise.
    """

    for layer_index in core_indexes:
        region = _cell_region(layout, leaf.cell, leaf.trans, [layer_index])
        if not region.is_empty():
            return region
    if fallback:
        return _cell_region(layout, leaf.cell, leaf.trans, None)
    return kdb.Region()


def extract_circuit(gds_path: str | Path, config: ExtractionConfig) -> ExtractedCircuit:
    layout = kdb.Layout()
    layout.read(str(gds_path))
    top = next(cell for cell in layout.each_cell() if cell.is_top())
    dbu = layout.dbu
    core_indexes = [
        layout.layer(kdb.LayerInfo(layer, datatype)) for layer, datatype in config.core_layers
    ]
    leaves = _flatten(layout, top, config)
    if config.heater_metal_layers:
        metal_indexes = [
            layout.layer(kdb.LayerInfo(layer, datatype))
            for layer, datatype in config.heater_metal_layers
        ]
        leaves = [
            dataclasses.replace(leaf, kind=config.heater_without_metal_kind)
            if leaf.kind == "heater"
            and leaf.cell is not None
            and _cell_region(layout, leaf.cell, leaf.trans, metal_indexes).is_empty()
            else leaf
            for leaf in leaves
        ]

    skipped: Counter = Counter()
    components: list[_Component] = []
    net_region = kdb.Region()
    for leaf in leaves:
        if leaf.kind in config.skip_kinds:
            skipped[leaf.kind] += 1
            continue
        if leaf.kind == config.default_kind:
            net_region += _leaf_region(layout, leaf, core_indexes, fallback=False)
            continue
        # a component's ports are where waveguides end on its own waveguide
        # shapes; its other layers (heater metal, trenches) only when it has
        # no shapes on the core layers (foundry black boxes drawn elsewhere)
        region = _leaf_region(layout, leaf, core_indexes, fallback=True)
        components.append(
            _Component(
                leaf=leaf,
                region=region,
                bbox=region.bbox().to_dtype(dbu) if not region.is_empty() else None,
                black_box=_leaf_region(layout, leaf, core_indexes, fallback=False).is_empty(),
            )
        )
    net_region.merge()
    if config.net_gap_um > 0:
        half = round(config.net_gap_um / 2 / dbu)
        net_region = net_region.sized(half).sized(-half)
        net_region.merge()
    tol = config.touch_tolerance_um
    low, high = config.facet_width_um
    abutting: list[tuple[int, int, Point, Point]] = []
    if config.arm_pairs:
        components, pair_boxes = _group_arm_pairs(
            components, config, net_region, tol, low, high, dbu
        )
        abutting = _abutting_ports(net_region, components, pair_boxes, tol, low, high, dbu)
        for box in pair_boxes:
            net_region -= kdb.Region(box.to_itype(dbu))
        net_region.merge()

    ports_by_component: dict[int, list[ExtractedPort]] = {
        index: [] for index in range(len(components))
    }
    nets: list[ExtractedNet] = []
    fragments = 0
    seen_ports: dict[tuple[int, int, int], int] = {}

    def register_port(component_index: int, position: Point, normal: Point) -> int:
        key = (component_index, round(position[0] * 100), round(position[1] * 100))
        if key in seen_ports:
            return seen_ports[key]
        leaf = components[component_index].leaf
        # the local frame is the instance's rotation-only frame: a mirrored
        # instance keeps its absolute geometry, the cell's y is folded in, so
        # the generic replacement is placed with a rotation and no mirror
        frame = kdb.DCplxTrans(1.0, float(leaf.trans.angle), False, leaf.trans.disp)
        inverse = frame.inverted()
        local = inverse * kdb.DPoint(*position)
        local_normal = inverse * kdb.DVector(*normal)
        ports_by_component[component_index].append(
            ExtractedPort(
                position=position,
                normal=normal,
                local_position=(local.x, local.y),
                local_normal=(local_normal.x, local_normal.y),
            )
        )
        seen_ports[key] = len(ports_by_component[component_index]) - 1
        return seen_ports[key]

    for polygon_index, polygon in enumerate(net_region.each()):
        dpolygon = polygon.to_dtype(dbu)
        points = list(dpolygon.each_point_hull())
        count = len(points)
        endpoints: list[tuple[int, int]] = []
        touched: set[int] = set()
        facet_widths: list[float] = []
        for index in range(count):
            a, b = points[index], points[(index + 1) % count]
            length = a.distance(b)
            if length < low or length > high:
                continue
            mid = kdb.DPoint((a.x + b.x) / 2, (a.y + b.y) / 2)
            # the waveguide runs perpendicular to the facet, into the polygon
            px, py = (b.y - a.y) / length, -(b.x - a.x) / length
            probe = kdb.DPoint(mid.x + px * 0.3, mid.y + py * 0.3)
            inside = dpolygon.inside(probe)
            normal = (px, py) if inside else (-px, -py)
            cap = _is_end_cap(points, index)
            component_index = _touching_component(components, mid, tol, dbu, cap)
            if component_index is None and cap:
                component_index = _placeholder_near(components, config, mid)
            if component_index is None and cap and config.open_ends:
                component_index = len(components)
                components.append(_open_end_component(mid))
                ports_by_component[component_index] = []
            if component_index is None or component_index in touched:
                continue
            touched.add(component_index)
            port_index = register_port(component_index, (mid.x, mid.y), normal)
            endpoints.append((component_index, port_index))
            facet_widths.append(length)
        if not endpoints:
            fragments += 1
            continue
        kinds = [components[ci].leaf.kind for ci, _ in endpoints]
        stub = len(endpoints) == 2 and any(kind in config.stub_kinds for kind in kinds)
        note = ""
        if len(endpoints) == 1:
            note = "one port only: the other end touches nothing recognized"
        elif len(endpoints) > 2:
            note = "more than two ports: a crossing, a junction, or two nets that touch"
        # the narrowest end facet is the waveguide's width; a wider one is the
        # face of the component port the net ends on (a 2 um MMI port)
        width = min(facet_widths)
        nets.append(
            ExtractedNet(
                name=f"net_{polygon_index}",
                endpoints=tuple((ci, pi) for ci, pi in endpoints),
                stub=stub,
                note=note,
                length_um=round(dpolygon.area() / width, 3),
            )
        )

    for component_index, pair_index, position, normal in abutting:
        # an MMI output lying on an arm pair's box: both get the port, the net
        # between them has no length and is not routed
        a = register_port(component_index, position, normal)
        b = register_port(pair_index, position, (-normal[0], -normal[1]))
        nets.append(
            ExtractedNet(
                name=f"abut_{len(nets)}",
                endpoints=((component_index, a), (pair_index, b)),
                note="abutting: the arm pair starts at the component's port",
            )
        )

    # components with ports (unconnected show cells are dropped), named by kind
    named: list[ExtractedComponent] = []
    index_to_name: dict[int, str] = {}
    counters: Counter = Counter()
    split_names: dict[tuple[int, int], str] = {}
    for component_index, component in enumerate(components):
        leaf = component.leaf
        ports = ports_by_component[component_index]
        if not ports:
            skipped[f"{leaf.kind} (no net)"] += 1
            continue
        ordered = sorted(ports, key=lambda port: (port.local_position[0], port.local_position[1]))
        trans = leaf.trans
        part_kind = config.split_kinds.get(leaf.kind)
        if part_kind is not None:
            # one single-port component per port, the parts spread to the configured pitch
            spread = _spread_ports(ordered, config.split_pitch_um)
            for port in ordered:
                counters[part_kind] += 1
                part_name = f"{part_kind}_{counters[part_kind] - 1}"
                split_names[(component_index, ports.index(port))] = part_name
                named.append(
                    ExtractedComponent(
                        name=part_name,
                        kind=part_kind,
                        cell_name=leaf.cell_name,
                        origin=(trans.disp.x, trans.disp.y),
                        rotation_deg=float(trans.angle),
                        mirror=bool(trans.is_mirror()),
                        ports=(spread[id(port)],),
                    )
                )
            continue
        counters[leaf.kind] += 1
        name = f"{leaf.kind}_{counters[leaf.kind] - 1}"
        index_to_name[component_index] = name
        named.append(
            ExtractedComponent(
                name=name,
                kind=leaf.kind,
                cell_name=leaf.cell_name,
                origin=(trans.disp.x, trans.disp.y),
                rotation_deg=float(trans.angle),
                mirror=bool(trans.is_mirror()),
                ports=tuple(ordered),
                geometry=dict(component.geometry),
            )
        )
        # remap port indexes to the ordered tuple
        order = {id(port): new_index for new_index, port in enumerate(ordered)}
        ports_by_component[component_index] = [None] * len(ordered)  # type: ignore[list-item]
        for port in ordered:
            ports_by_component[component_index][order[id(port)]] = port
        for net_index, net in enumerate(nets):
            nets[net_index] = ExtractedNet(
                name=net.name,
                endpoints=tuple(
                    (ci, order[id(ports[pi])] if ci == component_index else pi)
                    for ci, pi in net.endpoints
                ),
                stub=net.stub,
                note=net.note,
                length_um=net.length_um,
            )

    def resolve(ci: int, pi: int) -> tuple[str, int]:
        if (ci, pi) in split_names:
            return (split_names[(ci, pi)], 0)
        return (index_to_name[ci], pi)

    resolved_nets = [
        ExtractedNet(
            name=net.name,
            endpoints=tuple(resolve(ci, pi) for ci, pi in net.endpoints),  # type: ignore[misc]
            stub=net.stub,
            note=net.note,
            length_um=net.length_um,
        )
        for net in nets
    ]
    return ExtractedCircuit(
        components=named, nets=resolved_nets, skipped=skipped, fragments=fragments
    )


def _spread_ports(ports: list, pitch_um: float) -> dict[int, ExtractedPort]:
    """Two ports moved apart about their centre to ``pitch_um``; others, or 0, unchanged.

    Keyed by the identity of the original port. Only the chip-frame position
    moves: the normal and the local frame are the cell's and stay.
    """

    spread = {id(port): port for port in ports}
    if len(ports) != 2 or pitch_um <= 0:
        return spread
    (ax, ay), (bx, by) = ports[0].position, ports[1].position
    distance = math.hypot(bx - ax, by - ay)
    if distance <= 0 or distance >= pitch_um:
        return spread
    ux, uy = (bx - ax) / distance, (by - ay) / distance
    shift = (pitch_um - distance) / 2
    spread[id(ports[0])] = dataclasses.replace(
        ports[0], position=(ax - ux * shift, ay - uy * shift)
    )
    spread[id(ports[1])] = dataclasses.replace(
        ports[1], position=(bx + ux * shift, by + uy * shift)
    )
    return spread


def _group_arm_pairs(
    components: list[_Component],
    config: ExtractionConfig,
    net_region: Any,
    tol: float,
    low: float,
    high: float,
    dbu: float,
) -> tuple[list[_Component], list[Any]]:
    """Replace every two arms an ``ArmPairRule`` matches by one arm-pair component.

    Returns the new component list (the arms removed, the pairs appended)
    and the pairs' boxes: the arms' x-span extended by the rule's S-bend
    length on both sides, the arms' y-extent plus a waveguide width. Two
    arms are a pair only when the waveguides enter and leave that box as
    two on the left and two on the right (``_box_crossings``): arms fed
    from another side (one from above over a bend) stay separate components.
    """

    absorbed: set[int] = set()
    pairs: list[_Component] = []
    boxes: list[Any] = []
    for rule in config.arm_pairs:
        candidates = [
            (index, component)
            for index, component in enumerate(components)
            if component.leaf.kind in rule.kinds and component.bbox is not None
        ]
        for i, (index_a, a) in enumerate(candidates):
            if index_a in absorbed:
                continue
            for index_b, b in candidates[i + 1 :]:
                if index_b in absorbed:
                    continue
                if abs(a.bbox.left - b.bbox.left) > 1.0 or abs(a.bbox.right - b.bbox.right) > 1.0:
                    continue
                if rule.distinct_kinds and a.leaf.kind == b.leaf.kind:
                    continue
                separation = abs(a.bbox.center().y - b.bbox.center().y)
                if separation == 0 or separation > rule.max_separation_um:
                    continue
                absorbed.update((index_a, index_b))
                box = kdb.DBox(
                    a.bbox.left - rule.extension_um,
                    min(a.bbox.bottom, b.bbox.bottom) - 1.0,
                    a.bbox.right + rule.extension_um,
                    max(a.bbox.top, b.bbox.top) + 1.0,
                )
                if _box_crossings(net_region, box, tol, low, high, dbu) != (2, 2):
                    absorbed.difference_update((index_a, index_b))
                    continue
                trans = kdb.DCplxTrans(1.0, 0.0, False, kdb.DVector(box.center().x, box.center().y))
                leaf = _Leaf(
                    path="<arm pair>", cell=None, trans=trans, kind=rule.kind, composite=False
                )
                pairs.append(
                    _Component(
                        leaf=leaf,
                        region=kdb.Region(),
                        bbox=box,
                        black_box=True,
                        geometry={
                            "arm_length_um": round(a.bbox.width(), 3),
                            "arm_separation_um": round(separation, 3),
                            "extension_um": rule.extension_um,
                        },
                    )
                )
                boxes.append(box)
                break
    kept = [component for index, component in enumerate(components) if index not in absorbed]
    return kept + pairs, boxes


def _box_crossings(
    net_region: Any, box: Any, tol: float, low: float, high: float, dbu: float
) -> tuple[int, int]:
    """How many waveguides enter ``box`` through its left and its right edge.

    Counts the end caps on those edges after cutting the box out of a copy
    of the net region, plus the caps that already lie on them (a component
    port feeding the box directly).
    """

    cut = net_region - kdb.Region(box.to_itype(dbu))
    cut.merge()
    left = right = 0
    seen: set[tuple[int, int]] = set()
    for region in (net_region, cut):
        for polygon in region.interacting(kdb.Region(box.enlarged(tol, tol).to_itype(dbu))).each():
            dpolygon = polygon.to_dtype(dbu)
            points = list(dpolygon.each_point_hull())
            count = len(points)
            for index in range(count):
                a, b = points[index], points[(index + 1) % count]
                length = a.distance(b)
                if length < low or length > high or not _is_end_cap(points, index):
                    continue
                mid = kdb.DPoint((a.x + b.x) / 2, (a.y + b.y) / 2)
                if not (box.bottom - tol <= mid.y <= box.top + tol):
                    continue
                key = (round(mid.x * 100), round(mid.y * 100))
                if key in seen:
                    continue
                if abs(mid.x - box.left) <= tol:
                    seen.add(key)
                    left += 1
                elif abs(mid.x - box.right) <= tol:
                    seen.add(key)
                    right += 1
    return left, right


def _abutting_ports(
    net_region: Any,
    components: list[_Component],
    pair_boxes: list[Any],
    tol: float,
    low: float,
    high: float,
    dbu: float,
) -> list[tuple[int, int, Point, Point]]:
    """Net end caps that touch a component and lie on an arm pair's box boundary.

    Such a net disappears when the box is cut out of the net region (an
    MMI output feeding the pair's S-bend directly), so it is recorded here
    as (component, pair, position, normal into the net) before the cut.
    """

    found: list[tuple[int, int, Point, Point]] = []
    pair_indexes = [index for index, c in enumerate(components) if c.leaf.path == "<arm pair>"]
    for polygon in net_region.each():
        dpolygon = polygon.to_dtype(dbu)
        points = list(dpolygon.each_point_hull())
        count = len(points)
        for index in range(count):
            a, b = points[index], points[(index + 1) % count]
            length = a.distance(b)
            if length < low or length > high or not _is_end_cap(points, index):
                continue
            mid = kdb.DPoint((a.x + b.x) / 2, (a.y + b.y) / 2)
            on_box = None
            for pair_index, box in zip(pair_indexes, pair_boxes, strict=True):
                if (
                    box.bottom - tol <= mid.y <= box.top + tol
                    and min(abs(mid.x - box.left), abs(mid.x - box.right)) <= tol
                ):
                    on_box = pair_index
            if on_box is None:
                continue
            px, py = (b.y - a.y) / length, -(b.x - a.x) / length
            inside = dpolygon.inside(kdb.DPoint(mid.x + px * 0.3, mid.y + py * 0.3))
            normal = (px, py) if inside else (-px, -py)
            component_index = _touching_component(components, mid, tol, dbu, True)
            if component_index is None or component_index in pair_indexes:
                continue
            found.append((component_index, on_box, (mid.x, mid.y), normal))
    return found


def _is_end_cap(points: list, index: int) -> bool:
    """Whether hull edge ``index`` closes a waveguide: both neighbours run perpendicular to it.

    A short hull edge is also what a bend's polygonized arc is made of; the
    waveguide's sides next to a real end run straight away from it. The
    neighbours are taken as chords of at least ``_CHORD_UM`` so that the tiny
    steps a polygonization or a sizing leaves behind do not decide.
    """

    count = len(points)
    a, b = points[index], points[(index + 1) % count]
    facet = (b.x - a.x, b.y - a.y)
    facet_length = math.hypot(*facet)
    for origin, start, step in ((a, index, -1), (b, (index + 1) % count, 1)):
        chord = _chord(points, origin, start, step)
        if chord is None:
            return False
        chord_length = math.hypot(*chord)
        cosine = abs(facet[0] * chord[0] + facet[1] * chord[1]) / (facet_length * chord_length)
        if cosine > 0.2:
            return False
    return True


_CHORD_UM = 0.2


def _chord(points: list, origin: Any, start: int, step: int) -> tuple[float, float] | None:
    """The vector from ``origin`` to the first hull point at least ``_CHORD_UM`` away."""

    count = len(points)
    index = start
    for _ in range(count - 1):
        index = (index + step) % count
        vector = (points[index].x - origin.x, points[index].y - origin.y)
        if math.hypot(*vector) >= _CHORD_UM:
            return vector
    return None


def _open_end_component(mid: Any) -> _Component:
    """A synthetic one-port component at a facet that touches nothing (``open_ends``)."""

    trans = kdb.DCplxTrans(1.0, 0.0, False, kdb.DVector(mid.x, mid.y))
    leaf = _Leaf(path="<open end>", cell=None, trans=trans, kind="open_end", composite=False)
    return _Component(leaf=leaf, region=kdb.Region(), bbox=None, black_box=True)


def _touching_component(
    components: list[_Component], mid: Any, tol: float, dbu: float, cap: bool
) -> int | None:
    """The component whose shapes a facet midpoint touches, else None.

    A black box (no shapes on the core layers) is often drawn as a frame with
    label fragments on a marker layer that do not fill its outline, so its
    probe finds nothing; for those every end cap (``cap``) inside or on the
    component's bounding box counts: a foundry cell's ports sit on its
    boundary, and a heater drawn as a show cell has its waveguide ends under
    its metal, where a bend's arc segments may lie too.
    """

    probe = kdb.Region(kdb.DBox(mid.x - tol, mid.y - tol, mid.x + tol, mid.y + tol).to_itype(dbu))
    for index, component in enumerate(components):
        bbox = component.bbox
        if bbox is None:
            continue
        if not (
            bbox.left - tol <= mid.x <= bbox.right + tol
            and bbox.bottom - tol <= mid.y <= bbox.top + tol
        ):
            continue
        if component.black_box:
            if cap:
                return index
            continue  # its metal or marker shapes say nothing about waveguide ends
        region = component.region
        if not (region & probe).is_empty() or not region.interacting(probe).is_empty():
            return index
    return None


def _placeholder_near(components: list, config: ExtractionConfig, mid: Any) -> int | None:
    best: tuple[float, int] | None = None
    for index, component in enumerate(components):
        leaf = component.leaf
        if leaf.kind not in config.placeholder_kinds or component.bbox is not None:
            continue
        origin = leaf.trans.disp
        distance = math.hypot(origin.x - mid.x, origin.y - mid.y)
        if distance <= config.placeholder_tolerance_um and (best is None or distance < best[0]):
            best = (distance, index)
    return best[1] if best else None


# --- rebuilding with generic components ----------------------------------------
