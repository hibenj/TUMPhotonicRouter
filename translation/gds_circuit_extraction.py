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
    #: kinds recorded but neither components of the benchmark nor nets
    skip_kinds: frozenset[str] = frozenset({"bondpad", "heater_metal"})
    #: an unmatched leaf cell is a waveguide piece of this kind
    default_kind: str = "net"
    #: kinds whose components are split into one single-port component per port
    #: (a two-port polarization-splitting coupler becomes two couplers of the
    #: mapped kind), kind -> kind of the parts
    split_kinds: dict[str, str] = field(default_factory=dict)
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


@dataclass(frozen=True)
class ExtractedNet:
    name: str
    endpoints: tuple[tuple[str, int], ...]
    stub: bool = False
    note: str = ""


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
        nets.append(
            ExtractedNet(
                name=f"net_{polygon_index}",
                endpoints=tuple((ci, pi) for ci, pi in endpoints),
                stub=stub,
                note=note,
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
            # one single-port component per port
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
                        ports=(port,),
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
        )
        for net in nets
    ]
    return ExtractedCircuit(
        components=named, nets=resolved_nets, skipped=skipped, fragments=fragments
    )


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
