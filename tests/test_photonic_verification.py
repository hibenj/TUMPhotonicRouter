from types import SimpleNamespace

import klayout.db as kdb
from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk

from translation.photonic_verification import (
    PhotonicVerificationIssue,
    _component_layer_region,
    _verify_cross_net_route_overlaps,
    _verify_record_coverage,
    _verify_route_obstacle_overlaps,
    verify_photonic_routing,
)

get_generic_pdk().activate()


def _schematic_with_one_net():
    return SimpleNamespace(
        netlist=SimpleNamespace(
            routes={
                "n1": SimpleNamespace(
                    links={
                        "src,o1": "dst,o2",
                    }
                )
            }
        )
    )


def _box_region(xmin: int, ymin: int, xmax: int, ymax: int) -> kdb.Region:
    return kdb.Region(kdb.Box(xmin, ymin, xmax, ymax))


def test_photonic_verifier_reports_missing_routed_record():
    result = verify_photonic_routing(
        Component(),
        _schematic_with_one_net(),
        routed_net_records=[],
        realization_grid_spec=(100, 100, 1.0, 0.0, 0.0),
    )

    assert result.success is False
    assert result.error_count == 1
    assert result.metrics["expected_route_count"] == 1
    assert result.metrics["routed_record_count"] == 0
    assert [issue.code for issue in result.issues] == ["missing_route_record"]


def test_photonic_verifier_record_coverage_reports_duplicate_and_extra_records():
    issues: list[PhotonicVerificationIssue] = []
    expected = {
        ("n1", ("src", "o1"), ("dst", "o2")),
    }
    actual = [
        ("n1", ("src", "o1"), ("dst", "o2")),
        ("n1", ("src", "o1"), ("dst", "o2")),
        ("extra", ("a", "o1"), ("b", "o2")),
    ]

    _verify_record_coverage(issues, expected, actual)

    assert {issue.code for issue in issues} == {
        "duplicate_route_record",
        "extra_route_record",
    }


def test_photonic_verifier_reports_cross_net_waveguide_overlap():
    issues: list[PhotonicVerificationIssue] = []

    overlap_count = _verify_cross_net_route_overlaps(
        issues,
        {
            ("n1", ("a", "o1"), ("b", "o2")): _box_region(0, 0, 10_000, 2_000),
            ("n2", ("c", "o1"), ("d", "o2")): _box_region(5_000, 0, 15_000, 2_000),
        },
        dbu=0.001,
        legal_overlap_region=kdb.Region(),
    )

    assert overlap_count == 1
    assert [issue.code for issue in issues] == ["cross_net_waveguide_overlap"]
    assert issues[0].details["overlap_area_um2"] == 10.0


def test_photonic_verifier_ignores_cross_net_overlap_inside_port_window():
    issues: list[PhotonicVerificationIssue] = []

    overlap_count = _verify_cross_net_route_overlaps(
        issues,
        {
            ("n1", ("a", "o1"), ("b", "o2")): _box_region(0, 0, 10_000, 2_000),
            ("n2", ("c", "o1"), ("d", "o2")): _box_region(5_000, 0, 15_000, 2_000),
        },
        dbu=0.001,
        legal_overlap_region=_box_region(4_000, -1_000, 11_000, 3_000),
    )

    assert overlap_count == 0
    assert issues == []


def test_photonic_verifier_reports_waveguide_obstacle_overlap():
    obstacle_layout = Component()
    obstacle_layout.add_polygon(
        [
            (5.0, 0.0),
            (15.0, 0.0),
            (15.0, 2.0),
            (5.0, 2.0),
        ],
        layer=(47, 0),
    )
    issues: list[PhotonicVerificationIssue] = []

    overlap_count = _verify_route_obstacle_overlaps(
        issues,
        {
            ("n1", ("a", "o1"), ("b", "o2")): _box_region(0, 0, 10_000, 2_000),
        },
        obstacle_component=obstacle_layout,
        routed_layout=Component(),
        route_layer=(1, 0),
        obstacle_layers=((47, 0),),
        dbu=0.001,
        legal_overlap_region=kdb.Region(),
    )

    assert overlap_count == 1
    assert [issue.code for issue in issues] == ["waveguide_obstacle_overlap"]
    assert issues[0].details["overlap_area_um2"] == 10.0
    assert not _component_layer_region(obstacle_layout, (47, 0)).is_empty()
