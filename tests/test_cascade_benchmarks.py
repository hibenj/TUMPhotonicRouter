"""Tests for the parametric cascade benchmark builder (`benchmarks/cascade.py`)
and its six generated modules `benchmarks/cascade_{8,12,16}_{folded,diagonal}.py`.

These tests only build schematics (and, for the largest module, an unrouted
layout); none of them route waveguides, so they follow the same
`test_benes_benchmark.py` / `test_clements_benchmark_alignment.py` pattern of
schematic-only benchmark tests and are marked `integration` accordingly
(`tests/conftest.py` marks those files by stem via `_INTEGRATION_FILE_STEMS`;
this file is out of this lane's edit scope, so the marker is applied directly
here instead -- see the verifier packet for this discrepancy).
"""

from __future__ import annotations

import importlib

import gdsfactory as gf
import pytest

from benchmarks import cascade, heater_s_mod

pytestmark = pytest.mark.integration

CASCADE_MODULES: tuple[tuple[str, int, str], ...] = (
    ("cascade_8_folded", 8, "folded"),
    ("cascade_8_diagonal", 8, "diagonal"),
    ("cascade_12_folded", 12, "folded"),
    ("cascade_12_diagonal", 12, "diagonal"),
    ("cascade_16_folded", 16, "folded"),
    ("cascade_16_diagonal", 16, "diagonal"),
)


def _net_pairs(schematic) -> set[frozenset[str]]:
    pairs: set[frozenset[str]] = set()
    for bundle in schematic.netlist.routes.values():
        for p1, p2 in bundle.links.items():
            pairs.add(frozenset((p1, p2)))
    return pairs


def _link_count(schematic) -> int:
    return sum(len(bundle.links) for bundle in schematic.netlist.routes.values())


def test_folded_8_mode_equals_heater_s_mod():
    reference = heater_s_mod.build_schematic()
    generated = cascade.build_cascade_schematic(8, "folded")

    ref_instances = set(reference.netlist.instances.keys())
    gen_instances = set(generated.netlist.instances.keys())
    assert ref_instances == gen_instances

    for name in ref_instances:
        ref_inst = reference.netlist.instances[name]
        gen_inst = generated.netlist.instances[name]
        assert ref_inst.component == gen_inst.component, name

        ref_placement = reference.placements[name]
        gen_placement = generated.placements[name]
        assert ref_placement.x == pytest.approx(gen_placement.x), name
        assert ref_placement.y == pytest.approx(gen_placement.y), name
        assert ref_placement.rotation == pytest.approx(gen_placement.rotation), name
        assert ref_placement.mirror == gen_placement.mirror, name

    assert _net_pairs(reference) == _net_pairs(generated)


@pytest.mark.parametrize("module_name,n_modes,layout", CASCADE_MODULES)
def test_cascade_counts(module_name: str, n_modes: int, layout: str):
    module = importlib.import_module(f"benchmarks.{module_name}")
    schematic = module.build_schematic()
    instances = schematic.netlist.instances

    couplers = [n for n in instances if n.startswith("gc_in_") or n.startswith("gc_out_")]
    heaters = [n for n in instances if n.startswith("heater_")]
    mmis = [n for n in instances if n.startswith("mmi_")]
    cascade_count = n_modes - 3
    cascade_mzi_names = {f"heater_extra_{j}" for j in range(cascade_count)}

    assert len(couplers) == 2 * n_modes
    assert len(heaters) == 3 * n_modes - 3
    assert len(mmis) == 4 * n_modes - 6
    assert cascade_mzi_names <= set(instances)
    assert len(cascade_mzi_names) == cascade_count

    # Every port referenced by a net exists on its instance's component, and
    # every instance optical port is referenced by exactly one net endpoint.
    port_usage: dict[str, dict[str, int]] = {}
    for name, inst in instances.items():
        component = gf.get_component(inst.component)
        optical_ports = {p.name for p in component.ports if p.port_type == "optical"}
        port_usage[name] = dict.fromkeys(optical_ports, 0)

    total_port_references = 0
    for bundle in schematic.netlist.routes.values():
        for p1, p2 in bundle.links.items():
            for endpoint in (p1, p2):
                inst_name, port_name = endpoint.split(",")
                assert port_name in port_usage[inst_name], endpoint
                port_usage[inst_name][port_name] += 1
                total_port_references += 1

    for inst_name, ports in port_usage.items():
        for port_name, count in ports.items():
            assert count == 1, (inst_name, port_name, count)

    assert len(schematic.netlist.routes) == total_port_references // 2
    assert _link_count(schematic) == total_port_references // 2


@pytest.mark.parametrize("n_modes", (8, 12, 16))
def test_diagonal_differs_only_in_x(n_modes: int):
    folded = cascade.build_cascade_schematic(n_modes, "folded")
    diagonal = cascade.build_cascade_schematic(n_modes, "diagonal")

    assert set(folded.netlist.instances) == set(diagonal.netlist.instances)
    assert _net_pairs(folded) == _net_pairs(diagonal)

    lane_prefixes = ("gc_in_", "mmi_a_", "heater_", "mmi_b_", "heater_post_")
    for name in folded.netlist.instances:
        folded_placement = folded.placements[name]
        diagonal_placement = diagonal.placements[name]
        assert folded_placement.y == pytest.approx(diagonal_placement.y), name
        assert folded_placement.rotation == pytest.approx(diagonal_placement.rotation), name
        assert folded_placement.mirror == diagonal_placement.mirror, name

        is_input_or_lane = name.startswith(lane_prefixes) and not name.startswith("heater_extra_")
        if is_input_or_lane:
            assert folded_placement.x == pytest.approx(diagonal_placement.x), name


def test_layout_from_schematic_builds():
    from translation.layout_from_schematic import layout_from_schematic

    module = importlib.import_module("benchmarks.cascade_16_diagonal")
    schematic = module.build_schematic()
    layout = layout_from_schematic(schematic)

    bbox = layout.bbox()
    width = bbox.right - bbox.left
    height = bbox.top - bbox.bottom
    print(f"cascade_16_diagonal bbox: width={width} height={height}")
    assert width > 0
    assert height > 0
