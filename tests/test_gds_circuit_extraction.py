"""Tests for the GDS circuit extraction and rebuild on a synthetic GDS of generic components.

Private-circuits plan, 2026-10-05: the circuit is built here from the generic
PDK (two MMI2x2, one heater, four grating couplers, routed waveguides), written
to a GDS, and the extractor must recover the instances, their placements and
the nets; the rebuilt schematic must place the generic replacements exactly
where the originals stand.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from translation.gds_circuit_extraction import (
    ClassificationRule,
    CompositeRule,
    ExtractionConfig,
    extract_circuit,
)
from translation.gds_circuit_rebuild import (
    default_generic_mappings,
    generic_schematic,
    write_benchmark_module,
)

pytestmark = pytest.mark.integration


def _synthetic_gds(path: Path) -> dict[str, tuple[float, float, float]]:
    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component("synthetic_circuit")
    placements: dict[str, tuple[float, float, float]] = {}

    def place(name: str, component: str, x: float, y: float, rotation: float = 0.0, **settings):
        ref = c.add_ref(gf.get_component(component, **settings), name=name)
        ref.rotate(rotation)
        ref.move((x, y))
        placements[name] = (x, y, rotation)
        return ref

    mmi_a = place("mmi_a", "mmi2x2", 100.0, 0.0)
    heater = place("heater", "straight_heater_metal", 200.0, 40.0, length=120.0)
    mmi_b = place("mmi_b", "mmi2x2", 400.0, 0.0)
    gc_in_0 = place("gc_in_0", "grating_coupler_te", -60.0, 60.0, rotation=180.0)
    gc_in_1 = place("gc_in_1", "grating_coupler_te", -60.0, -60.0, rotation=180.0)
    gc_out_0 = place("gc_out_0", "grating_coupler_te", 560.0, 60.0)
    gc_out_1 = place("gc_out_1", "grating_coupler_te", 560.0, -60.0)
    gf.routing.route_single(c, gc_in_0.ports["o1"], mmi_a.ports["o2"], cross_section="strip")
    gf.routing.route_single(c, gc_in_1.ports["o1"], mmi_a.ports["o1"], cross_section="strip")
    gf.routing.route_single(c, mmi_a.ports["o3"], heater.ports["o1"], cross_section="strip")
    gf.routing.route_single(c, heater.ports["o2"], mmi_b.ports["o2"], cross_section="strip")
    gf.routing.route_single(c, mmi_a.ports["o4"], mmi_b.ports["o1"], cross_section="strip")
    gf.routing.route_single(c, mmi_b.ports["o3"], gc_out_0.ports["o1"], cross_section="strip")
    gf.routing.route_single(c, mmi_b.ports["o4"], gc_out_1.ports["o1"], cross_section="strip")
    c.write_gds(str(path))
    return placements


def _config() -> ExtractionConfig:
    return ExtractionConfig(
        core_layers=((1, 0),),
        rules=(
            ClassificationRule("heater", ("straight_heater_metal",)),
            ClassificationRule("mmi2x2", ("mmi2x2",)),
            ClassificationRule("gc", ("grating_coupler",)),
            ClassificationRule("net", ("straight", "bend")),
        ),
        composites=(
            CompositeRule("heater", "straight_heater_metal"),
            CompositeRule("mmi2x2", "mmi2x2"),
            CompositeRule("gc", "grating_coupler"),
        ),
    )


def test_extractor_recovers_components_nets_and_placements(tmp_path: Path):
    gds = tmp_path / "synthetic.gds"
    placements = _synthetic_gds(gds)

    circuit = extract_circuit(gds, _config())

    kinds = {component.kind for component in circuit.components}
    assert kinds == {"mmi2x2", "heater", "gc"}
    assert len(circuit.components) == 7
    routed = [net for net in circuit.nets if len(net.endpoints) == 2 and not net.stub]
    assert len(routed) == 7, circuit.report()
    assert all(len(net.endpoints) == 2 for net in circuit.nets), circuit.report()
    mmis = [component for component in circuit.components if component.kind == "mmi2x2"]
    assert all(len(mmi.ports) == 4 for mmi in mmis)
    # the MMI ports in the component frame: the generic mmi2x2's own geometry
    local = sorted(
        (round(p.local_position[0], 2), round(p.local_position[1], 2)) for p in mmis[0].ports
    )
    assert local == [(-10.0, -0.62), (-10.0, 0.62), (15.5, -0.62), (15.5, 0.62)]
    heater = next(component for component in circuit.components if component.kind == "heater")
    assert sorted(round(p.local_position[0], 2) for p in heater.ports) == [0.0, 120.0]

    schematic = generic_schematic(circuit, default_generic_mappings())
    assert len(schematic.netlist.instances) == 7
    assert len(schematic.netlist.routes) == 7
    # the rebuilt components stand where the originals stand (matched by position)
    originals = sorted(placements.values())
    rebuilt = sorted(
        (round(placement.x, 3), round(placement.y, 3), float(placement.rotation))
        for placement in schematic.placements.values()
    )
    assert [(x, y) for x, y, _ in rebuilt] == pytest.approx(
        [(x, y) for x, y, _ in originals], abs=1e-3
    )
    # gdsfactory resolves a component alias to the underlying cell function
    # on validation (straight_heater_metal -> straight_heater_metal_undercut),
    # so the instances are found by the extractor's kind-derived names
    heater_instance = schematic.netlist.instances["heater_0"]
    assert heater_instance.settings["length"] == pytest.approx(120.0)
    mmi_instance = schematic.netlist.instances["mmi2x2_0"]
    assert mmi_instance.settings["length_mmi"] == pytest.approx(5.5)
    assert mmi_instance.settings["gap_mmi"] == pytest.approx(0.25)

    module_path = tmp_path / "synthetic_benchmark.py"
    write_benchmark_module(
        schematic, module_path, title="synthetic", flags=("--crossings", "false")
    )
    namespace: dict = {}
    exec(module_path.read_text(encoding="utf-8"), namespace)
    rebuilt_again = namespace["build_schematic"]()
    assert len(rebuilt_again.netlist.instances) == 7
    assert len(rebuilt_again.netlist.routes) == 7


def _synthetic_stub_gds(path: Path) -> tuple[float, float]:
    """An MMI whose outputs end on an empty end-cap cell and in free space; returns the ends."""
    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component("synthetic_stubs")
    mmi = c.add_ref(gf.get_component("mmi2x2"), name="mmi")
    mmi.move((100.0, 0.0))
    for index, port in enumerate(("o1", "o2")):
        gc = c.add_ref(gf.get_component("grating_coupler_te"), name=f"gc_{index}")
        gc.rotate(180.0)
        gc.move((-60.0, 60.0 if port == "o2" else -60.0))
        gf.routing.route_single(c, gc.ports["o1"], mmi.ports[port], cross_section="strip")
    # the capped arm: a waveguide piece, then a cell whose name says heater
    # but that draws no metal (a bare arm, kept as a fixed waveguide), then
    # a waveguide piece ending on the empty end-cap cell
    lead_in = c.add_ref(gf.get_component("straight", length=20.0), name="lead_in")
    lead_in.connect("o1", mmi.ports["o3"])
    arm = gf.Component("straight_heater_metal_bare_arm")
    arm_ref = arm.add_ref(gf.get_component("straight", length=50.0))
    arm.add_ports(arm_ref.ports)
    bare_arm = c.add_ref(arm, name="bare_arm")
    bare_arm.connect("o1", lead_in.ports["o2"])
    capped = c.add_ref(gf.get_component("straight", length=20.0), name="capped")
    capped.connect("o1", bare_arm.ports["o2"])
    marker = c.add_ref(gf.Component("endcap_marker"), name="endcap")
    marker.move(tuple(capped.ports["o2"].center))
    # longer than the capped arm: its end must lie outside the marker's placeholder tolerance
    open_end = c.add_ref(gf.get_component("straight", length=100.0), name="open")
    open_end.connect("o1", mmi.ports["o4"])
    c.write_gds(str(path))
    return tuple(capped.ports["o2"].center), tuple(open_end.ports["o2"].center)


def test_extractor_terminates_nets_on_end_caps_and_open_ends(tmp_path: Path):
    gds_path = tmp_path / "stubs.gds"
    cap_end, open_end = _synthetic_stub_gds(gds_path)
    config = ExtractionConfig(
        core_layers=((1, 0),),
        rules=(
            ClassificationRule("endcap", ("endcap_marker",)),
            ClassificationRule("heater", ("straight_heater_metal",)),
            ClassificationRule("mmi2x2", ("mmi2x2",)),
            ClassificationRule("gc", ("grating_coupler",)),
            ClassificationRule("net", ("straight", "bend")),
        ),
        composites=(
            CompositeRule("heater", "straight_heater_metal"),
            CompositeRule("mmi2x2", "mmi2x2"),
            CompositeRule("gc", "grating_coupler"),
        ),
        placeholder_kinds=frozenset({"endcap"}),
        open_ends=True,
        heater_metal_layers=((47, 0),),  # the generic PDK's HEATER layer
    )
    circuit = extract_circuit(gds_path, config)
    kinds = sorted(component.kind for component in circuit.components)
    # the metal-less heater cell is a bare arm: a waveguide with two ports
    assert kinds == ["endcap", "gc", "gc", "mmi2x2", "open_end", "waveguide"]
    assert all(len(net.endpoints) == 2 for net in circuit.nets)
    assert sum(net.stub for net in circuit.nets) == 2
    assert len(circuit.nets) == 5  # gc, gc, mmi -> arm, arm -> end cap, mmi -> open end
    by_kind = {component.kind: component for component in circuit.components}
    assert by_kind["endcap"].ports[0].position == pytest.approx(cap_end, abs=1e-3)
    assert by_kind["open_end"].ports[0].position == pytest.approx(open_end, abs=1e-3)
    # the waveguide runs into both ends from the left
    assert by_kind["endcap"].ports[0].normal == pytest.approx((-1.0, 0.0))
    assert by_kind["open_end"].ports[0].normal == pytest.approx((-1.0, 0.0))

    schematic = generic_schematic(circuit, default_generic_mappings())
    assert len(schematic.netlist.routes) == 5  # the stubs are routed to as well
    arm_instance = schematic.netlist.instances["waveguide_0"]
    assert arm_instance.settings["length"] == pytest.approx(50.0)
    for name, end in (("endcap_0", cap_end), ("open_end_0", open_end)):
        instance = schematic.netlist.instances[name]
        assert instance.settings["length"] == pytest.approx(10.0)
        placement = schematic.placements[name]
        # a straight's o1 sits at its origin: the stub starts at the waveguide's end
        assert (placement.x, placement.y) == pytest.approx(end, abs=1e-3)
        assert float(placement.rotation) == pytest.approx(0.0)
