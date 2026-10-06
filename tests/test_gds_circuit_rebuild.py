"""The rebuild of an extracted circuit: the two-port grating coupler, by-position port matching
and the rule that rebuilt elements never overlap (owner's review of the second Clements mesh,
2026-10-06)."""

from __future__ import annotations

import gdsfactory as gf
import pytest

from translation.gds_circuit_components import grating_coupler_2d, register_generic_components
from translation.gds_circuit_extraction import (
    ExtractedCircuit,
    ExtractedComponent,
    ExtractedNet,
    ExtractedPort,
)
from translation.gds_circuit_rebuild import (
    default_generic_mappings,
    generic_schematic,
    overlapping_instances,
)
from translation.layout_from_schematic import layout_from_schematic

_DIAG = 0.7071067811865476


def _port(position, normal, local_position, local_normal) -> ExtractedPort:
    return ExtractedPort(
        position=position,
        normal=normal,
        local_position=local_position,
        local_normal=local_normal,
    )


def _two_port_coupler_circuit() -> ExtractedCircuit:
    """A 2D coupler as the extractor sees it: a 45-degree-rotated square whose two ports
    leave at +45 and -45 degrees, side by side 9.9 um apart, feeding one waveguide."""

    coupler = ExtractedComponent(
        name="gc_2d_0",
        kind="gc_2d",
        cell_name="coupler",
        origin=(20555.0, 13335.0),
        rotation_deg=315.0,
        mirror=False,
        ports=(
            _port((20559.95, 13339.95), (_DIAG, _DIAG), (0.0, 7.0), (0.0, 1.0)),
            _port((20559.95, 13330.05), (_DIAG, -_DIAG), (7.0, 0.0), (1.0, 0.0)),
        ),
    )
    waveguide = ExtractedComponent(
        name="waveguide_0",
        kind="waveguide",
        cell_name="arm",
        origin=(20700.0, 13339.95),
        rotation_deg=0.0,
        mirror=False,
        ports=(
            _port((20700.0, 13339.95), (-1.0, 0.0), (0.0, 0.0), (-1.0, 0.0)),
            _port((20750.0, 13339.95), (1.0, 0.0), (50.0, 0.0), (1.0, 0.0)),
        ),
    )
    nets = [
        ExtractedNet(name="feed", endpoints=(("gc_2d_0", 0), ("waveguide_0", 0))),
        ExtractedNet(name="back", endpoints=(("waveguide_0", 1), ("gc_2d_0", 1))),
    ]
    return ExtractedCircuit(components=[coupler, waveguide], nets=nets)


def test_two_port_coupler_keeps_its_body_behind_the_port_plane():
    register_generic_components()
    coupler = gf.get_component("grating_coupler_2d", port_pitch=9.9)
    ports = {port.name: port for port in coupler.ports if port.port_type == "optical"}
    assert sorted(ports) == ["o1", "o2"]
    assert tuple(ports["o1"].center) == pytest.approx((0.0, -4.95), abs=1e-3)
    assert tuple(ports["o2"].center) == pytest.approx((0.0, 4.95), abs=1e-3)
    assert ports["o1"].orientation == 180 and ports["o2"].orientation == 180
    box = coupler.dbbox()
    # nothing left of the ports; the tapers and the 20 um grating to the right
    assert box.left == pytest.approx(0.0, abs=1e-3)
    assert box.right == pytest.approx(30.0, abs=1e-3)
    assert (box.bottom, box.top) == pytest.approx((-10.0, 10.0), abs=1e-3)
    with pytest.raises(ValueError):
        grating_coupler_2d(port_pitch=3.0, taper_width=4.0)
    # ports far apart (the beam splitter's coupler cell): one grating per port
    far = gf.get_component("grating_coupler_2d", port_pitch=482.104)
    box = far.dbbox()
    assert box.left == pytest.approx(0.0, abs=1e-3)
    assert (box.bottom, box.top) == pytest.approx((-251.052, 251.052), abs=1e-3)
    core = far.get_polygons(by="name")["WG"] if hasattr(far, "get_polygons") else None
    assert core is None or len(core) == 4  # two tapers, two slabs


def test_rebuild_places_one_two_port_coupler_on_both_original_ports():
    circuit = _two_port_coupler_circuit()
    schematic = generic_schematic(circuit, default_generic_mappings())
    instance = schematic.netlist.instances["gc_2d_0"]
    assert instance.component == "grating_coupler_2d"
    assert instance.settings["port_pitch"] == pytest.approx(9.9)
    placement = schematic.placements["gc_2d_0"]
    # the device faces +x (the mean of the two 45-degree normals): turned by 180
    assert float(placement.rotation) == pytest.approx(180.0)
    layout = layout_from_schematic(schematic)
    placed = next(ref for ref in layout.insts if ref.name == "gc_2d_0")
    # the upper original port is the generic o1 after the half turn, the lower o2
    assert tuple(placed.ports["o1"].center) == pytest.approx((20559.95, 13339.95), abs=1e-3)
    assert tuple(placed.ports["o2"].center) == pytest.approx((20559.95, 13330.05), abs=1e-3)
    links = {
        f"{p1}->{p2}"
        for bundle in schematic.netlist.routes.values()
        for p1, p2 in bundle.links.items()
    }
    assert len(links) == 2  # two nets, none lost in the renaming
    assert "gc_2d_0,o1->waveguide_0,o1" in links
    assert "waveguide_0,o2->gc_2d_0,o2" in links
    assert overlapping_instances(layout) == []


def test_overlapping_instances_reports_two_couplers_at_the_old_pitch():
    gf.gpdk.PDK.activate()
    layout = gf.Component()
    for name, y in (("gc_0", 0.0), ("gc_1", 9.9)):
        ref = layout.add_ref(gf.components.grating_coupler_te(), name=name)
        ref.move((0.0, y))
    far = layout.add_ref(gf.components.grating_coupler_te(), name="gc_2")
    far.move((0.0, 100.0))
    assert overlapping_instances(layout) == [("gc_0", "gc_1")]
