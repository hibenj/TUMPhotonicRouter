"""`photonic_router.logic_checks.reading`: each statement finds what it names."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from photonic_router.logic_checks import ERROR, NOTE, count_by_check
from photonic_router.logic_checks.reading import check_layout, check_schematic
from photonic_router.records.layout_from_schematic import layout_from_schematic

from benchmarks.heater_single import build_schematic


@pytest.fixture
def schematic():
    return build_schematic()


def _first_link(schematic) -> tuple[str, str, str]:
    net, bundle = next(iter(schematic.netlist.routes.items()))
    first, second = next(iter(bundle.links.items()))
    return net, first, second


def test_a_sound_design_has_no_error(schematic) -> None:
    findings = check_schematic(schematic) + check_layout(
        schematic, layout_from_schematic(schematic)
    )
    assert [finding for finding in findings if finding.severity == ERROR] == []


def test_an_instance_and_its_placement_belong_together(schematic) -> None:
    name = next(iter(schematic.placements))
    schematic.placements["ghost"] = schematic.placements.pop(name)
    assert count_by_check(check_schematic(schematic)) == {
        "reading.instance_without_placement": 1,
        "reading.placement_without_instance": 1,
    }


def test_a_link_end_names_an_instance_of_the_netlist(schematic) -> None:
    net, first, _second = _first_link(schematic)
    schematic.netlist.routes[net].links[first] = "nobody,o1"
    (finding,) = check_schematic(schematic)
    assert (finding.check, finding.subject, finding.detail) == (
        "reading.link_end_without_instance",
        net,
        "nobody,o1",
    )


def test_a_port_carries_one_link_and_no_link_to_itself(schematic) -> None:
    net, first, second = _first_link(schematic)
    links = schematic.netlist.routes[net].links
    links[second] = first  # a second link on both ports
    assert count_by_check(check_schematic(schematic)) == {"reading.port_in_several_links": 2}
    links.clear()
    links[first] = first
    assert count_by_check(check_schematic(schematic)) == {
        "reading.link_to_itself": 1,
        "reading.port_in_several_links": 1,
    }
    links.clear()
    assert "reading.net_without_link" in count_by_check(check_schematic(schematic))


def test_a_link_end_names_a_port_of_its_placed_instance(schematic) -> None:
    layout = layout_from_schematic(schematic)
    net, first, _second = _first_link(schematic)
    instance = first.split(",")[0]
    schematic.netlist.routes[net].links[first] = f"{instance},no_such_port"
    findings = [f for f in check_layout(schematic, layout) if f.severity == ERROR]
    assert [(f.check, f.detail) for f in findings] == [
        ("reading.link_end_without_port", f"{instance},no_such_port")
    ]


def _port(name: str, x: float, y: float, orientation: float, **changes: object):
    values = {"port_type": "optical", "width": 0.5, "layer_info": None, **changes}
    return SimpleNamespace(name=name, center=(x, y), orientation=orientation, **values)


def _design(ports_a: list, ports_b: list, links: dict[str, str]):
    """Two 10 x 10 um instances, `a` at x 0..10 and `b` at x 20..30, with the given ports."""

    def instance(name: str, left: float, ports: list):
        box = SimpleNamespace(left=left, bottom=0.0, right=left + 10.0, top=10.0)
        return SimpleNamespace(name=name, ports=ports, dbbox=lambda: box)

    schematic = SimpleNamespace(
        netlist=SimpleNamespace(
            instances={"a": object(), "b": object()},
            routes={"n": SimpleNamespace(links=links)},
        ),
        placements={},
    )
    layout = SimpleNamespace(insts=[instance("a", 0.0, ports_a), instance("b", 20.0, ports_b)])
    return schematic, layout


def test_ports_on_facing_walls_are_sound() -> None:
    design = _design([_port("o", 10.0, 5.0, 0.0)], [_port("i", 20.0, 5.0, 180.0)], {"a,o": "b,i"})
    assert check_layout(*design) == []


def test_a_port_short_of_the_face_it_points_at_is_noted() -> None:
    design = _design([_port("o", 6.0, 5.0, 0.0)], [_port("i", 20.0, 5.0, 180.0)], {"a,o": "b,i"})
    (finding,) = check_layout(*design)
    assert (finding.check, finding.severity, finding.subject) == (
        "reading.port_recessed",
        NOTE,
        "a,o",
    )
    assert finding.detail == "4.000 um behind the face"


def test_a_port_outside_its_instance_or_off_axis_is_an_error() -> None:
    design = _design([_port("o", 12.0, 5.0, 0.0)], [_port("i", 20.0, 5.0, 135.0)], {"a,o": "b,i"})
    assert count_by_check(check_layout(*design)) == {
        "reading.port_not_axis_aligned": 1,
        "reading.port_outside_instance": 1,
    }


def test_the_two_ports_of_a_link_match() -> None:
    design = _design(
        [_port("o", 10.0, 5.0, 0.0, width=0.5)],
        [_port("i", 20.0, 5.0, 180.0, width=1.0, port_type="electrical")],
        {"a,o": "b,i"},
    )
    assert count_by_check(check_layout(*design)) == {
        "reading.link_width_mismatch": 1,
        "reading.port_not_optical": 1,
    }


def test_a_link_inside_one_instance_and_overlapping_boxes_are_noted() -> None:
    schematic, layout = _design(
        [_port("o", 10.0, 5.0, 0.0), _port("p", 10.0, 8.0, 0.0)], [], {"a,o": "a,p"}
    )
    box = SimpleNamespace(left=5.0, bottom=5.0, right=25.0, top=6.0)
    layout.insts.append(SimpleNamespace(name="c", ports=[], dbbox=lambda: box))
    schematic.netlist.instances["c"] = object()
    findings = check_layout(schematic, layout)
    assert {finding.severity for finding in findings} == {NOTE}
    assert count_by_check(findings) == {
        "reading.instances_overlap": 2,
        "reading.link_within_instance": 1,
    }
