"""Tests for the GDS circuit extraction and rebuild on a synthetic GDS of generic components.

Private-circuits plan, 2026-10-05: the circuit is built here from the generic
PDK (two MMI2x2, one heater, four grating couplers, routed waveguides), written
to a GDS, and the extractor must recover the instances, their placements and
the nets; the rebuilt schematic must place the generic replacements exactly
where the originals stand.
"""

from __future__ import annotations

from pathlib import Path
from uuid import uuid4

import pytest

from translation.gds_circuit_components import (
    directional_coupler,
    directional_coupler_path_length_um,
    register_generic_components,
)
from translation.gds_circuit_extraction import (
    ArmPairRule,
    ClassificationRule,
    CompositeRule,
    DirectionalCouplerRule,
    ExtractionConfig,
    extract_circuit,
)
from translation.gds_circuit_rebuild import (
    benchmark_metadata,
    default_generic_mappings,
    generic_schematic,
    overlapping_instances,
    write_benchmark_module,
)
from translation.layout_from_schematic import layout_from_schematic

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
    suffix = uuid4().hex[:6]  # the fixture is built more than once per session
    c = gf.Component(f"synthetic_stubs_{suffix}")
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
    arm = gf.Component(f"straight_heater_metal_bare_arm_{suffix}")
    arm_ref = arm.add_ref(gf.get_component("straight", length=50.0))
    arm.add_ports(arm_ref.ports)
    bare_arm = c.add_ref(arm, name="bare_arm")
    bare_arm.connect("o1", lead_in.ports["o2"])
    capped = c.add_ref(gf.get_component("straight", length=20.0), name="capped")
    capped.connect("o1", bare_arm.ports["o2"])
    marker = c.add_ref(gf.Component(f"endcap_marker_{suffix}"), name="endcap")
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
    # the original waveguides' lengths (polygon area over facet width): the two
    # 20 um pieces around the bare arm and the 100 um open end are straights,
    # the coupler nets are longer routed bends
    lengths = sorted(round(net.length_um, 1) for net in circuit.nets)
    assert lengths[:2] == [20.0, 20.0] and 100.0 in lengths and lengths[-1] > 100.0
    by_kind = {component.kind: component for component in circuit.components}
    assert by_kind["endcap"].ports[0].position == pytest.approx(cap_end, abs=1e-3)
    assert by_kind["open_end"].ports[0].position == pytest.approx(open_end, abs=1e-3)
    # the waveguide runs into both ends from the left
    assert by_kind["endcap"].ports[0].normal == pytest.approx((-1.0, 0.0))
    assert by_kind["open_end"].ports[0].normal == pytest.approx((-1.0, 0.0))

    schematic = generic_schematic(circuit, default_generic_mappings())
    assert len(schematic.netlist.routes) == 5  # the stubs are routed to as well
    # the couplers feed the MMI's input side: they are the circuit's inputs; the
    # end cap and the open end are its sinks
    node_types, _delays = benchmark_metadata(schematic, circuit)
    # the open end stands for a missing I/O cell on the MMI's output side
    assert node_types == {
        "gc_0": "input",
        "gc_1": "input",
        "endcap_0": "sink",
        "open_end_0": "output",
    }
    assert benchmark_metadata(schematic, circuit, {"gc_1": "output"})[0]["gc_1"] == "output"
    arm_instance = schematic.netlist.instances["waveguide_0"]
    assert arm_instance.settings["length"] == pytest.approx(50.0)
    for name, end in (("endcap_0", cap_end), ("open_end_0", open_end)):
        instance = schematic.netlist.instances[name]
        assert instance.settings["length"] == pytest.approx(10.0)
        placement = schematic.placements[name]
        # a straight's o1 sits at its origin: the stub starts at the waveguide's end
        assert (placement.x, placement.y) == pytest.approx(end, abs=1e-3)
        assert float(placement.rotation) == pytest.approx(0.0)


def _synthetic_arm_pair_gds(path: Path) -> dict[str, tuple[float, float]]:
    """Two MMIs joined by a metal heater arm and a bare arm over 60 um S-bends."""
    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component("synthetic_arm_pair")
    mmi_a = c.add_ref(gf.get_component("mmi2x2"), name="mmi_a")
    mmi_a.move((100.0, 0.0))
    bare = gf.Component("straight_heater_metal_bare_arm_of_pair")
    bare_ref = bare.add_ref(gf.get_component("straight", length=120.0))
    bare.add_ports(bare_ref.ports)
    arms = {}
    for sign, out_port, arm_component in ((1.0, "o3", "straight_heater_metal"), (-1.0, "o4", None)):
        pitch = abs(mmi_a.ports["o3"].center[1] - mmi_a.ports["o4"].center[1])
        fan_out = c.add_ref(gf.get_component("bend_s", size=(60.0, sign * (58.0 - pitch) / 2)))
        fan_out.connect("o1", mmi_a.ports[out_port])
        arm = c.add_ref(
            gf.get_component(arm_component, length=120.0) if arm_component else bare,
            name=f"arm_{out_port}",
        )
        arm.connect("o1", fan_out.ports["o2"])
        fan_in = c.add_ref(gf.get_component("bend_s", size=(60.0, -sign * (58.0 - pitch) / 2)))
        fan_in.connect("o1", arm.ports["o2"])
        arms[out_port] = fan_in
    mmi_b = c.add_ref(gf.get_component("mmi2x2"), name="mmi_b")
    mmi_b.connect("o2", arms["o3"].ports["o2"])
    c.write_gds(str(path))
    return {
        "mmi_a_o3": tuple(mmi_a.ports["o3"].center),
        "mmi_a_o4": tuple(mmi_a.ports["o4"].center),
        "mmi_b_o1": tuple(mmi_b.ports["o1"].center),
        "mmi_b_o2": tuple(mmi_b.ports["o2"].center),
    }


def test_extractor_keeps_an_arm_pair_as_one_component(tmp_path: Path):
    gds_path = tmp_path / "arm_pair.gds"
    ends = _synthetic_arm_pair_gds(gds_path)
    config = ExtractionConfig(
        core_layers=((1, 0),),
        rules=(
            ClassificationRule("heater", ("straight_heater_metal",)),
            ClassificationRule("mmi2x2", ("mmi2x2",)),
            ClassificationRule("net", ("straight", "bend")),
        ),
        composites=(
            CompositeRule("heater", "straight_heater_metal"),
            CompositeRule("mmi2x2", "mmi2x2"),
        ),
        heater_metal_layers=((47, 0),),
        arm_pairs=(
            ArmPairRule(("heater", "waveguide"), max_separation_um=70.0, extension_um=60.0),
        ),
    )
    circuit = extract_circuit(gds_path, config)
    kinds = sorted(component.kind for component in circuit.components)
    assert kinds == ["arm_pair", "mmi2x2", "mmi2x2"]
    pair = next(component for component in circuit.components if component.kind == "arm_pair")
    assert pair.geometry["arm_length_um"] == pytest.approx(120.0)
    assert pair.geometry["arm_separation_um"] == pytest.approx(58.0)
    # its four ports sit on the MMIs' ports: the S-bends start right there
    positions = sorted(tuple(round(v, 3) for v in port.position) for port in pair.ports)
    assert positions == sorted(tuple(round(v, 3) for v in end) for end in ends.values())
    # four abutting nets of zero length, nothing to route
    assert len(circuit.nets) == 4
    assert all(net.note.startswith("abutting") for net in circuit.nets)

    schematic = generic_schematic(circuit, default_generic_mappings())
    assert len(schematic.netlist.routes) == 0
    pair_instance = schematic.netlist.instances["arm_pair_0"]
    assert pair_instance.settings["arm_length"] == pytest.approx(120.0)
    assert pair_instance.settings["bend_radius"] == pytest.approx(5.0)
    assert pair_instance.settings["pitch_in"] == pytest.approx(
        abs(ends["mmi_a_o3"][1] - ends["mmi_a_o4"][1]), abs=1e-3
    )
    placement = schematic.placements["arm_pair_0"]
    assert (placement.x, placement.y) == pytest.approx(
        (ends["mmi_a_o4"][0], (ends["mmi_a_o3"][1] + ends["mmi_a_o4"][1]) / 2), abs=1e-3
    )
    # the written module rebuilds, and the layout places the pair between the MMIs;
    # its path-length metadata carries the MMI's and the arm pair's optical lengths
    module_path = tmp_path / "arm_pair_benchmark.py"
    node_types, delays = benchmark_metadata(schematic, circuit)
    write_benchmark_module(
        schematic,
        module_path,
        title="synthetic",
        flags=(),
        node_types=node_types,
        internal_delays_um=delays,
    )
    namespace: dict = {}
    exec(compile(module_path.read_text(), str(module_path), "exec"), namespace)
    rebuilt = namespace["build_schematic"]()
    assert set(rebuilt.netlist.instances) == set(schematic.netlist.instances)
    assert namespace["NODE_TYPES"] == {}  # no couplers here
    mmi_settings = schematic.netlist.instances["mmi2x2_0"].settings
    assert namespace["INTERNAL_DELAYS_UM"]["mmi2x2_0"] == pytest.approx(
        mmi_settings["length_mmi"] + 2 * mmi_settings["length_taper"]
    )
    # one arm: two leads, two rises, the 120 um arm and four quarter arcs of 5 um
    pair = pair_instance.settings
    assert namespace["INTERNAL_DELAYS_UM"]["arm_pair_0"] == pytest.approx(
        2 * (pair["extension"] - 10.0)
        + (pair["arm_separation"] - pair["pitch_in"]) / 2
        - 10.0
        + (pair["arm_separation"] - pair["pitch_out"]) / 2
        - 10.0
        + 120.0
        + 2 * 3.141592653589793 * 5.0,
        abs=1e-3,
    )


def _synthetic_split_coupler_gds(path: Path) -> tuple[tuple[float, float], tuple[float, float]]:
    """A two-port coupler cell whose ports sit 10 um apart, each feeding an MMI input;
    returns the two port positions."""

    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component("synthetic_split")
    dual = gf.Component("dual_coupler_cell")
    for index, y in enumerate((-5.0, 5.0)):
        stub = dual.add_ref(gf.get_component("straight", length=10.0))
        stub.move((0.0, y))
        dual.add_port(f"o{index + 1}", port=stub.ports["o2"])
    coupler = c.add_ref(dual, name="dual")
    mmi = c.add_ref(gf.get_component("mmi2x2"), name="mmi")
    mmi.move((100.0, 0.0))
    for coupler_port, mmi_port in (("o1", "o1"), ("o2", "o2")):
        gf.routing.route_single(
            c, coupler.ports[coupler_port], mmi.ports[mmi_port], cross_section="strip"
        )
    c.write_gds(str(path))
    return tuple(coupler.ports["o1"].center), tuple(coupler.ports["o2"].center)


def test_split_coupler_parts_are_moved_apart_to_the_configured_pitch(tmp_path: Path):
    gds_path = tmp_path / "split.gds"
    lower, upper = _synthetic_split_coupler_gds(gds_path)
    base: dict = {
        "core_layers": ((1, 0),),
        "rules": (
            ClassificationRule("gc_2d", ("dual_coupler",)),
            ClassificationRule("mmi2x2", ("mmi2x2",)),
            ClassificationRule("net", ("straight", "bend")),
        ),
        "composites": (
            CompositeRule("gc_2d", "dual_coupler"),
            CompositeRule("mmi2x2", "mmi2x2"),
        ),
        "split_kinds": {"gc_2d": "gc"},
    }
    # without a pitch the parts stay on the original ports, 10 um apart
    circuit = extract_circuit(gds_path, ExtractionConfig(**base))
    parts = sorted(
        (component for component in circuit.components if component.kind == "gc"),
        key=lambda component: component.ports[0].position[1],
    )
    assert [part.ports[0].position for part in parts] == pytest.approx([lower, upper], abs=1e-3)
    # two generic TE couplers (26 um tall) at that pitch lie over each other
    overlaps = overlapping_instances(
        layout_from_schematic(generic_schematic(circuit, default_generic_mappings()))
    )
    assert overlaps == [("gc_0", "gc_1")]

    # with the pitch the parts move apart symmetrically about their centre; the
    # ports' normals, the nets and the MMI stay where they were
    circuit = extract_circuit(gds_path, ExtractionConfig(**base, split_pitch_um=30.0))
    parts = sorted(
        (component for component in circuit.components if component.kind == "gc"),
        key=lambda component: component.ports[0].position[1],
    )
    assert parts[0].ports[0].position == pytest.approx((lower[0], lower[1] - 10.0), abs=1e-3)
    assert parts[1].ports[0].position == pytest.approx((upper[0], upper[1] + 10.0), abs=1e-3)
    assert all(part.ports[0].normal == pytest.approx((1.0, 0.0)) for part in parts)
    assert len(circuit.nets) == 2
    schematic = generic_schematic(circuit, default_generic_mappings())
    layout = layout_from_schematic(schematic)
    assert overlapping_instances(layout) == []
    placed = {ref.name: ref for ref in layout.insts}
    assert tuple(placed["gc_0"].ports["o1"].center) == pytest.approx(
        parts[0].ports[0].position, abs=1e-3
    )
    assert (schematic.placements["mmi2x2_0"].x, schematic.placements["mmi2x2_0"].y) == (
        pytest.approx(100.0, abs=1e-3),
        pytest.approx(0.0, abs=1e-3),
    )


def test_net_length_uses_the_waveguide_width_not_a_wide_port_face(tmp_path: Path):
    """A net that widens into a 2 um port face (the beam splitter's MMI ports) is measured
    over its waveguide width: area over the narrowest end facet."""

    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component("synthetic_wide_port")
    gc = c.add_ref(gf.get_component("grating_coupler_te"), name="gc")
    gc.rotate(180.0)
    straight = c.add_ref(gf.get_component("straight", length=50.0), name="lead")
    straight.connect("o1", gc.ports["o1"])
    taper = c.add_ref(gf.get_component("taper", length=10.0, width1=0.5, width2=2.0), name="widen")
    taper.connect("o1", straight.ports["o2"])
    block = gf.Component("mmi2x2_wide_block")
    block.add_polygon([(0.0, -1.0), (20.0, -1.0), (20.0, 1.0), (0.0, 1.0)], layer="WG")
    block.add_port("o1", center=(0.0, 0.0), width=2.0, orientation=180, layer="WG")
    mmi = c.add_ref(block, name="mmi")
    mmi.connect("o1", taper.ports["o2"])
    gds_path = tmp_path / "wide_port.gds"
    c.write_gds(str(gds_path))
    config = ExtractionConfig(
        core_layers=((1, 0),),
        rules=(
            ClassificationRule("mmi2x2", ("mmi2x2",)),
            ClassificationRule("gc", ("grating_coupler",)),
            ClassificationRule("net", ("straight", "taper")),
        ),
        composites=(CompositeRule("gc", "grating_coupler"),),
    )
    circuit = extract_circuit(gds_path, config)
    (net,) = circuit.nets
    # 50 um of 0.5 um waveguide plus the taper's 12.5 um2, all over 0.5 um
    assert net.length_um == pytest.approx(75.0, abs=0.5)


def test_light_direction_decides_the_components_input_side(tmp_path: Path):
    """With the light flowing leftward (the beam splitter: in at its coupler array on the
    right, out through the couplers on the left) the MMI's right-hand ports are its inputs
    o1/o2, its left-hand ports the outputs, and the couplers on the left are the outputs."""

    gds_path = tmp_path / "stubs.gds"
    _synthetic_stub_gds(gds_path)
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
        heater_metal_layers=((47, 0),),
    )
    circuit = extract_circuit(gds_path, config)
    forward = generic_schematic(circuit, default_generic_mappings())
    backward = generic_schematic(circuit, default_generic_mappings(), light_direction=(-1.0, 0.0))

    def mmi_ports_fed_by_couplers(schematic) -> set[str]:
        ports = set()
        for bundle in schematic.netlist.routes.values():
            for p1, p2 in bundle.links.items():
                for coupler, other in ((p1, p2), (p2, p1)):
                    if coupler.startswith("gc_") and other.startswith("mmi2x2_0"):
                        ports.add(other)
        return ports

    # the couplers feed the MMI's left ports: o1/o2 with the light from the left,
    # o4/o3 (the outputs, lower and upper swapped by the half turn) against it
    assert mmi_ports_fed_by_couplers(forward) == {"mmi2x2_0,o1", "mmi2x2_0,o2"}
    assert mmi_ports_fed_by_couplers(backward) == {"mmi2x2_0,o3", "mmi2x2_0,o4"}
    assert float(forward.placements["mmi2x2_0"].rotation) == pytest.approx(0.0)
    assert float(backward.placements["mmi2x2_0"].rotation) == pytest.approx(180.0)
    # the MMI's ports still land on the original ones, both ways
    original_mmi = next(c for c in circuit.components if c.kind == "mmi2x2")
    original = sorted(tuple(round(v, 3) for v in port.position) for port in original_mmi.ports)
    for schematic in (forward, backward):
        layout = layout_from_schematic(schematic)
        mmi = next(ref for ref in layout.insts if ref.name == "mmi2x2_0")
        centers = sorted(
            tuple(round(v, 3) for v in mmi.ports[p].center) for p in ("o1", "o2", "o3", "o4")
        )
        assert centers == original
    # the couplers are the inputs one way and the outputs the other
    assert benchmark_metadata(forward, circuit)[0]["gc_0"] == "input"
    assert benchmark_metadata(backward, circuit)[0]["gc_0"] == "output"


def test_directional_coupler_component_has_mmi_like_ports_and_equal_waveguides():
    import gdsfactory as gf

    register_generic_components()
    coupler = gf.get_component(
        "directional_coupler",
        coupling_length=29.0,
        gap_pitch=0.75,
        extension=19.0,
        pitch_in=127.0,
        pitch_out=60.75,
        bend_radius=5.0,
        extension_out=19.0,
    )
    ports = {port.name: port for port in coupler.ports if port.port_type == "optical"}
    assert sorted(ports) == ["o1", "o2", "o3", "o4"]
    assert tuple(ports["o1"].center) == pytest.approx((0.0, -63.5), abs=1e-3)
    assert tuple(ports["o2"].center) == pytest.approx((0.0, 63.5), abs=1e-3)
    assert tuple(ports["o3"].center) == pytest.approx((67.0, 30.375), abs=1e-3)
    assert tuple(ports["o4"].center) == pytest.approx((67.0, -30.375), abs=1e-3)
    assert ports["o1"].orientation == 180 and ports["o4"].orientation == 0
    # both waveguides are the same length: leads, drops, the coupling section, four arcs
    length = directional_coupler_path_length_um(
        coupling_length=29.0,
        gap_pitch=0.75,
        extension=19.0,
        pitch_in=127.0,
        pitch_out=60.75,
        bend_radius=5.0,
        extension_out=19.0,
    )
    assert length == pytest.approx(9.0 + 9.0 + 53.125 + 20.0 + 29.0 + 2 * 3.141592653589793 * 5.0)
    with pytest.raises(ValueError):
        directional_coupler(pitch_in=10.0)  # no room for two bends per side


def _synthetic_coupler_mzi_gds(path: Path) -> dict[str, tuple[float, float]]:
    """Two heaters 60 um apart with a directional coupler on each side (gdsfactory's
    ``coupler``: S-bends to a 0.3 um gap over 29 um) and four couplers feeding them."""

    import gdsfactory as gf
    from gdsfactory.gpdk import get_generic_pdk

    get_generic_pdk().activate()
    c = gf.Component(f"synthetic_coupler_mzi_{uuid4().hex[:6]}")
    heaters = {}
    for name, y in (("heater_lower", -30.0), ("heater_upper", 30.0)):
        heater = c.add_ref(gf.get_component("straight_heater_metal", length=216.0), name=name)
        heater.move((100.0, y))
        heaters[name] = heater
    left = c.add_ref(
        gf.get_component("coupler", gap=0.3, length=29.0, dy=60.0, dx=40.0), name="dc_left"
    )
    left.connect("o4", heaters["heater_lower"].ports["o1"])
    right = c.add_ref(
        gf.get_component("coupler", gap=0.3, length=29.0, dy=60.0, dx=40.0), name="dc_right"
    )
    right.connect("o1", heaters["heater_lower"].ports["o2"])
    ends = {}
    for index, (ref, port, dx, dy) in enumerate(
        (
            (left, "o1", -120.0, -40.0),
            (left, "o2", -120.0, 40.0),
            (right, "o3", 120.0, 40.0),
            (right, "o4", 120.0, -40.0),
        )
    ):
        gc = c.add_ref(gf.get_component("grating_coupler_te"), name=f"gc_{index}")
        if dx < 0:
            gc.rotate(180.0)
        gc.move((ref.ports[port].center[0] + dx, ref.ports[port].center[1] + dy))
        gf.routing.route_single(c, gc.ports["o1"], ref.ports[port], cross_section="strip")
        ends[f"gc_{index}"] = tuple(gc.ports["o1"].center)
    c.write_gds(str(path))
    return {
        **ends,
        **{f"{name},o1": tuple(h.ports["o1"].center) for name, h in heaters.items()},
        **{f"{name},o2": tuple(h.ports["o2"].center) for name, h in heaters.items()},
    }


def test_extractor_keeps_a_directional_coupler_as_one_component(tmp_path: Path):
    gds_path = tmp_path / "coupler_mzi.gds"
    ends = _synthetic_coupler_mzi_gds(gds_path)
    config = ExtractionConfig(
        core_layers=((1, 0),),
        rules=(
            ClassificationRule("heater", ("straight_heater_metal",)),
            ClassificationRule("gc", ("grating_coupler",)),
            ClassificationRule("net", ("straight", "bend", "coupler")),
        ),
        composites=(
            CompositeRule("heater", "straight_heater_metal"),
            CompositeRule("gc", "grating_coupler"),
        ),
        heater_metal_layers=((47, 0),),
        directional_couplers=(DirectionalCouplerRule(("heater",), max_separation_um=70.0),),
    )
    circuit = extract_circuit(gds_path, config)
    kinds = sorted(component.kind for component in circuit.components)
    assert kinds == ["directional_coupler"] * 2 + ["gc"] * 4 + ["heater"] * 2
    couplers = [c for c in circuit.components if c.kind == "directional_coupler"]
    for coupler in couplers:
        geometry = coupler.geometry
        assert geometry["gap_pitch_um"] == pytest.approx(0.8, abs=0.05)
        assert geometry["coupling_length_um"] == pytest.approx(29.0, abs=2.0)
        assert geometry["pitch_um"] == pytest.approx(60.0, abs=0.1)
        assert geometry["outer_pitch_um"] == pytest.approx(60.0, abs=0.1)
    # four nets to route (the gcs to the couplers' outer ports), four abutting
    # ones of zero length (the couplers' inner ports on the heaters)
    assert sum(net.note.startswith("abutting") for net in circuit.nets) == 4
    assert len(circuit.nets) == 8

    schematic = generic_schematic(circuit, default_generic_mappings())
    assert len(schematic.netlist.routes) == 4
    layout = layout_from_schematic(schematic)
    assert overlapping_instances(layout) == []
    placed = {ref.name: ref for ref in layout.insts}
    # the couplers' inner ports sit on the heaters' ports
    inner = sorted(
        tuple(round(v, 2) for v in placed[name].ports[port].center)
        for name in ("directional_coupler_0", "directional_coupler_1")
        for port in ("o1", "o2", "o3", "o4")
        if abs(placed[name].ports[port].center[0] - 100.0) < 0.5
        or abs(placed[name].ports[port].center[0] - 316.0) < 0.5
    )
    heater_ports = sorted(
        tuple(round(v, 2) for v in ends[key]) for key in ends if key.startswith("heater")
    )
    assert inner == heater_ports
