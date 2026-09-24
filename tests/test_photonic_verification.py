import math
import random
import time
from dataclasses import replace
from types import SimpleNamespace

import klayout.db as kdb
from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from photonic_router.path_length_graph import PortRef
from photonic_router.static_obstacle_builder import _load_rust_backend

import translation.photonic_verification as photonic_verification_module
from translation.photonic_verification import (
    PhotonicVerificationIssue,
    _component_layer_region,
    _polygon_regions_by_pair_um,
    _PolygonBucketIndex,
    _region_area_um2,
    _region_bbox_um,
    _verify_crossing_component_overlaps,
    _verify_crossing_component_route_overlaps,
    _verify_cross_net_route_overlaps,
    _verify_record_coverage,
    _verify_route_obstacle_overlaps,
    _verify_routes_inside_routable_bbox,
    _verify_self_intersecting_routes,
    _polyline_self_intersects_um,
    verify_photonic_routing,
)
from translation.route_rust_types import RoutedNetRecord

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


def _schematic_with_two_nets():
    return SimpleNamespace(
        netlist=SimpleNamespace(
            routes={
                "n1": SimpleNamespace(links={"src,o1": "dst,o2"}),
                "n2": SimpleNamespace(links={"src,o1": "dst,o2"}),
            }
        )
    )


def _routed_record(
    *,
    net_name: str = "n1",
    centerline: tuple[tuple[float, float], ...],
    source_port_center_um: tuple[float, float] | None = None,
    target_port_center_um: tuple[float, float] | None = None,
    route_obj: object | None = None,
) -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name=net_name,
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o2"),
        route_obj=route_obj if route_obj is not None else SimpleNamespace(),
        total_length_um=0.0,
        corrected_centerline_um=centerline,
        source_port_center_um=source_port_center_um,
        target_port_center_um=target_port_center_um,
    )


def _box_region(xmin: int, ymin: int, xmax: int, ymax: int) -> kdb.Region:
    return kdb.Region(kdb.Box(xmin, ymin, xmax, ymax))


def _box_polygon_um(
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> tuple[tuple[float, float], ...]:
    return ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))


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


def _perpendicular_crossing_route_regions(record, **_kwargs) -> kdb.Region:
    """Two 0.5 um wide waveguides crossing perpendicularly at the origin,
    with no legal-overlap region: n1 runs along x, n2 along y, so their
    route polygons overlap on exactly a 0.5 x 0.5 um square (0.25 um2) --
    the case B2 of
    `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    says the old `min_route_overlap_area_um2 = 2.0` default hid.
    """
    if record.net_name == "n1":
        return _box_region(-10_000, -250, 10_000, 250)
    return _box_region(-250, -10_000, 250, 10_000)


def test_photonic_verifier_reports_perpendicular_crossing_overlap_at_default_threshold(
    monkeypatch,
):
    monkeypatch.setattr(
        photonic_verification_module,
        "_realized_record_region",
        _perpendicular_crossing_route_regions,
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_two_nets(),
        routed_net_records=[
            _routed_record(
                net_name="n1",
                centerline=((-10.0, 0.0), (10.0, 0.0)),
                source_port_center_um=(-10.0, 0.0),
                target_port_center_um=(10.0, 0.0),
            ),
            _routed_record(
                net_name="n2",
                centerline=((0.0, -10.0), (0.0, 10.0)),
                source_port_center_um=(0.0, -10.0),
                target_port_center_um=(0.0, 10.0),
            ),
        ],
        route_width_um=0.5,
        realization_grid_spec=(40, 40, 1.0, -20.0, -20.0),
        check_endpoint_connectivity=False,
    )

    assert [issue.code for issue in result.issues] == ["cross_net_waveguide_overlap"]
    assert result.issues[0].details["overlap_area_um2"] == 0.25


def test_photonic_verifier_old_threshold_hid_the_perpendicular_crossing_overlap(
    monkeypatch,
):
    monkeypatch.setattr(
        photonic_verification_module,
        "_realized_record_region",
        _perpendicular_crossing_route_regions,
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_two_nets(),
        routed_net_records=[
            _routed_record(
                net_name="n1",
                centerline=((-10.0, 0.0), (10.0, 0.0)),
                source_port_center_um=(-10.0, 0.0),
                target_port_center_um=(10.0, 0.0),
            ),
            _routed_record(
                net_name="n2",
                centerline=((0.0, -10.0), (0.0, 10.0)),
                source_port_center_um=(0.0, -10.0),
                target_port_center_um=(0.0, 10.0),
            ),
        ],
        route_width_um=0.5,
        realization_grid_spec=(40, 40, 1.0, -20.0, -20.0),
        check_endpoint_connectivity=False,
        min_route_overlap_area_um2=2.0,
    )

    assert result.issues == ()


def test_photonic_verifier_allows_pair_specific_legal_crossing_overlap():
    issues: list[PhotonicVerificationIssue] = []
    key_a = ("n1", ("a", "o1"), ("b", "o2"))
    key_b = ("n2", ("c", "o1"), ("d", "o2"))

    overlap_count = _verify_cross_net_route_overlaps(
        issues,
        {
            key_a: _box_region(0, 0, 10_000, 2_000),
            key_b: _box_region(5_000, 0, 15_000, 2_000),
        },
        dbu=0.001,
        legal_overlap_region=kdb.Region(),
        net_id_by_key={key_a: 1, key_b: 2},
        legal_overlap_regions_by_net_id_pair=_polygon_regions_by_pair_um(
            {(1, 2): (_box_polygon_um(5.0, 0.0, 10.0, 2.0),)},
            dbu=0.001,
        ),
    )

    assert overlap_count == 0
    assert issues == []


def test_photonic_verifier_reports_legal_crossing_spillover():
    issues: list[PhotonicVerificationIssue] = []
    key_a = ("n1", ("a", "o1"), ("b", "o2"))
    key_b = ("n2", ("c", "o1"), ("d", "o2"))

    overlap_count = _verify_cross_net_route_overlaps(
        issues,
        {
            key_a: _box_region(0, 0, 15_000, 2_000),
            key_b: _box_region(5_000, 0, 20_000, 2_000),
        },
        dbu=0.001,
        legal_overlap_region=kdb.Region(),
        net_id_by_key={key_a: 1, key_b: 2},
        legal_overlap_regions_by_net_id_pair=_polygon_regions_by_pair_um(
            {(1, 2): (_box_polygon_um(5.0, 0.0, 10.0, 2.0),)},
            dbu=0.001,
        ),
    )

    assert overlap_count == 1
    assert [issue.code for issue in issues] == ["cross_net_waveguide_overlap"]
    assert issues[0].details["overlap_area_um2"] == 10.0
    assert issues[0].details["overlap_bbox_um"] == (10.0, 0.0, 15.0, 2.0)


def test_photonic_verifier_does_not_apply_legal_crossing_mask_to_third_net():
    issues: list[PhotonicVerificationIssue] = []
    key_a = ("n1", ("a", "o1"), ("b", "o2"))
    key_b = ("n3", ("e", "o1"), ("f", "o2"))

    overlap_count = _verify_cross_net_route_overlaps(
        issues,
        {
            key_a: _box_region(0, 0, 10_000, 2_000),
            key_b: _box_region(5_000, 0, 10_000, 2_000),
        },
        dbu=0.001,
        legal_overlap_region=kdb.Region(),
        net_id_by_key={key_a: 1, key_b: 3},
        legal_overlap_regions_by_net_id_pair=_polygon_regions_by_pair_um(
            {(1, 2): (_box_polygon_um(5.0, 0.0, 10.0, 2.0),)},
            dbu=0.001,
        ),
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


def test_photonic_verifier_accepts_stubbed_centerline_port_connections(monkeypatch):
    monkeypatch.setattr(
        photonic_verification_module,
        "_realized_record_region",
        lambda *args, **kwargs: _box_region(-500, -500, 20_500, 5_500),
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_one_net(),
        routed_net_records=[
            _routed_record(
                centerline=((0.0, 0.0), (4.0, 0.0), (8.0, 4.0), (20.0, 5.0)),
                source_port_center_um=(0.0, 0.0),
                target_port_center_um=(20.0, 5.0),
            )
        ],
        route_width_um=1.0,
        realization_grid_spec=(40, 20, 1.0, -5.0, -5.0),
        check_endpoint_connectivity=True,
    )

    assert result.success is True


def test_photonic_verifier_reports_unconnected_port_with_crossings_enabled(monkeypatch):
    monkeypatch.setattr(
        photonic_verification_module,
        "_realized_record_region",
        lambda *args, **kwargs: _box_region(-500, -500, 20_500, 5_500),
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_one_net(),
        routed_net_records=[
            _routed_record(
                centerline=((0.0, 0.0), (4.0, 0.0), (8.0, 4.0), (20.0, 5.0)),
                source_port_center_um=(0.0, 0.0),
                target_port_center_um=(30.0, 5.0),
            )
        ],
        route_width_um=1.0,
        realization_grid_spec=(40, 20, 1.0, -5.0, -5.0),
        legal_overlap_polygons_by_net_id_pair_um={(1, 2): ()},
        check_endpoint_connectivity=True,
    )

    issue_codes = {issue.code for issue in result.issues}
    assert result.success is False
    assert "target_port_not_connected" in issue_codes
    assert "target_endpoint_mismatch" in issue_codes


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


def test_photonic_verifier_reports_static_overlap_inside_broad_port_window(monkeypatch):
    obstacle_layout = Component()
    obstacle_layout.add_polygon(
        [
            (5.0, -1.0),
            (8.0, -1.0),
            (8.0, 1.0),
            (5.0, 1.0),
        ],
        layer=(1, 0),
    )
    monkeypatch.setattr(
        photonic_verification_module,
        "_realized_record_region",
        lambda *args, **kwargs: _box_region(0, -500, 20_000, 500),
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_one_net(),
        routed_net_records=[
            _routed_record(
                centerline=((0.0, 0.0), (20.0, 0.0)),
                source_port_center_um=(0.0, 0.0),
                target_port_center_um=(20.0, 0.0),
            )
        ],
        unrouted_layout=obstacle_layout,
        route_width_um=1.0,
        route_layer=(1, 0),
        obstacle_layers=((1, 0),),
        realization_grid_spec=(40, 20, 1.0, -5.0, -5.0),
        port_obstacle_exemption_radius_um=25.0,
    )

    assert result.success is False
    assert result.metrics["waveguide_obstacle_overlap_count"] == 1
    assert [issue.code for issue in result.issues] == ["waveguide_obstacle_overlap"]


def test_photonic_verifier_reports_realized_bend_static_overlap():
    backend = _load_rust_backend()
    assert backend is not None
    router = backend.PyPhotonicRouter(
        backend.GridSpec(40, 40, 1.0, -15.0, -20.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(max_iterations=10_000),
    )
    route_obj = router.route_single_net_and_commit(
        1,
        backend.State(15, 20, 0),
        backend.State(25, 10, 6),
        block_radius_cells=0,
    )

    obstacle_layout = Component()
    obstacle_layout.add_polygon(
        [
            (9.4, -7.0),
            (10.6, -7.0),
            (10.6, -3.0),
            (9.4, -3.0),
        ],
        layer=(1, 0),
    )

    result = verify_photonic_routing(
        Component(),
        _schematic_with_one_net(),
        routed_net_records=[
            _routed_record(
                centerline=((0.0, 0.0), (10.0, 0.0), (10.0, -10.0)),
                source_port_center_um=(0.0, 0.0),
                target_port_center_um=(10.0, -10.0),
                route_obj=route_obj,
            )
        ],
        unrouted_layout=obstacle_layout,
        route_width_um=1.0,
        route_layer=(1, 0),
        obstacle_layers=((1, 0),),
        realization_grid_spec=(40, 40, 1.0, -15.0, -20.0),
    )

    assert result.success is False
    assert result.metrics["waveguide_obstacle_overlap_count"] >= 1
    assert "waveguide_obstacle_overlap" in {issue.code for issue in result.issues}


def test_photonic_verifier_reports_crossing_component_route_overlap():
    issues: list[PhotonicVerificationIssue] = []
    key = ("n3", ("a", "o1"), ("b", "o2"))

    overlap_count = _verify_crossing_component_route_overlaps(
        issues,
        {
            key: _box_region(0, 0, 10_000, 2_000),
        },
        [
            {
                "point_um": [5.0, 1.0],
                "component_bbox_um": [4.0, 4.0],
                "net_name_a": "n1",
                "net_name_b": "n2",
            }
        ],
        dbu=0.001,
        min_overlap_area_um2=0.25,
    )

    assert overlap_count == 1
    assert [issue.code for issue in issues] == ["crossing_component_route_overlap"]
    assert issues[0].net_name == "n3"
    assert issues[0].details["overlap_area_um2"] == 8.0
    assert issues[0].details["overlap_bbox_um"] == (3.0, 0.0, 7.0, 2.0)


def test_photonic_verifier_allows_crossing_component_owner_route_overlap():
    issues: list[PhotonicVerificationIssue] = []
    key = ("n1", ("a", "o1"), ("b", "o2"))

    overlap_count = _verify_crossing_component_route_overlaps(
        issues,
        {
            key: _box_region(0, 0, 10_000, 2_000),
        },
        [
            {
                "point_um": [5.0, 1.0],
                "component_bbox_um": [4.0, 4.0],
                "net_name_a": "n1",
                "net_name_b": "n2",
            }
        ],
        dbu=0.001,
        min_overlap_area_um2=0.25,
    )

    assert overlap_count == 0
    assert issues == []


def test_photonic_verifier_allows_shared_crossing_component_owner_route_overlap():
    issues: list[PhotonicVerificationIssue] = []
    shared_key = ("n4", ("a", "o1"), ("b", "o2"))
    third_party_key = ("n5", ("c", "o1"), ("d", "o2"))

    overlap_count = _verify_crossing_component_route_overlaps(
        issues,
        {
            shared_key: _box_region(0, 0, 10_000, 2_000),
            third_party_key: _box_region(0, 0, 10_000, 2_000),
        },
        [
            {
                "point_um": [5.0, 1.0],
                "component_bbox_um": [4.0, 4.0],
                "net_name_a": "n1",
                "net_name_b": "n2",
                "shared_owner_net_names": ["n1", "n2", "n4"],
            }
        ],
        dbu=0.001,
        min_overlap_area_um2=0.25,
    )

    assert overlap_count == 1
    assert [issue.net_name for issue in issues] == ["n5"]


def test_photonic_verifier_reports_crossing_component_overlap():
    issues: list[PhotonicVerificationIssue] = []

    overlap_count = _verify_crossing_component_overlaps(
        issues,
        [
            {
                "crossing_footprint_polygon_um": _box_polygon_um(0.0, 0.0, 4.0, 4.0),
                "net_name_a": "n1",
                "net_name_b": "n2",
            },
            {
                "crossing_footprint_polygon_um": _box_polygon_um(2.0, 2.0, 6.0, 6.0),
                "net_name_a": "n3",
                "net_name_b": "n4",
            },
        ],
        dbu=0.001,
        min_overlap_area_um2=0.25,
    )

    assert overlap_count == 1
    assert [issue.code for issue in issues] == ["crossing_component_overlap"]
    assert issues[0].details["overlap_area_um2"] == 4.0
    assert issues[0].details["overlap_bbox_um"] == (2.0, 2.0, 4.0, 4.0)


def test_polyline_self_intersects_um_accepts_simple_paths():

    assert _polyline_self_intersects_um(()) is None
    assert _polyline_self_intersects_um(((0.0, 0.0),)) is None
    assert _polyline_self_intersects_um(((0.0, 0.0), (5.0, 0.0), (5.0, 5.0))) is None
    assert (
        _polyline_self_intersects_um(((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (8.0, 4.0), (8.0, 8.0)))
        is None
    )


def test_polyline_self_intersects_um_rejects_a_genuine_crossing():

    point = _polyline_self_intersects_um(((0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)))
    assert point is not None
    assert point == (5.0, 5.0)


def test_polyline_self_intersects_um_rejects_collinear_overlap():

    point = _polyline_self_intersects_um(
        (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 5.0),
            (5.0, 5.0),
            (5.0, 0.0),
            (2.0, 0.0),
        )
    )
    assert point is not None


def test_polyline_self_intersects_um_accepts_a_terminal_heading_overshoot_and_return():

    # Mirrors the exact real regression this same permissiveness fixed on
    # the Rust side (src/astar.rs's polyline_self_intersects): a one-cell
    # overshoot-and-return to satisfy a required terminal heading revisits
    # an exact earlier vertex, which must not be flagged.
    assert (
        _polyline_self_intersects_um(
            (
                (2.0, 10.0),
                (2.0, 14.0),
                (47.0, 14.0),
                (47.0, 10.0),
                (46.0, 10.0),
                (47.0, 10.0),
            )
        )
        is None
    )


def test_photonic_verifier_reports_self_intersecting_route():
    issues: list[PhotonicVerificationIssue] = []

    record = _routed_record(
        net_name="n_31",
        centerline=(
            (0.0, 0.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 10.0),
        ),
    )

    count = _verify_self_intersecting_routes(issues, [record])

    assert count == 1
    assert [issue.code for issue in issues] == ["self_intersecting_route"]
    assert issues[0].net_name == "n_31"
    assert issues[0].severity == "error"
    assert issues[0].details["intersection_point_um"] == (5.0, 5.0)


def test_photonic_verifier_allows_clean_route_with_no_self_intersection():
    issues: list[PhotonicVerificationIssue] = []

    record = _routed_record(
        net_name="n1",
        centerline=((0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (10.0, 5.0)),
    )

    count = _verify_self_intersecting_routes(issues, [record])

    assert count == 0
    assert issues == []


def test_photonic_verifier_reports_route_outside_the_routable_die():
    # Shape of the multiportmmi_32x32 finding: the chip ends at x = 11424.164 um,
    # the route detours through the padding east of the output couplers.
    issues: list[PhotonicVerificationIssue] = []
    bbox = (0.0, 1622.125, 11424.164, 4778.125)
    record = _routed_record(
        net_name="n_430",
        centerline=((11000.0, 3300.0), (11425.25, 3300.0), (11425.25, 3200.0), (11400.0, 3200.0)),
    )

    count = _verify_routes_inside_routable_bbox(
        issues, [record], routable_bbox_um=bbox, tolerance_um=1.0
    )

    assert count == 1
    assert [issue.code for issue in issues] == ["route_outside_chip"]
    assert issues[0].net_name == "n_430"
    assert issues[0].severity == "error"
    assert issues[0].details["point_um"] == (11425.25, 3300.0)


def test_photonic_verifier_allows_centerline_in_the_cell_holding_the_bound():
    # A centerline on the center of the cell that holds the bound (at most half
    # a cell beyond it) is legal; without a routable bbox nothing is checked.
    issues: list[PhotonicVerificationIssue] = []
    bbox = (0.0, 0.0, 100.0, 50.0)
    inside = _routed_record(centerline=((0.5, 25.0), (100.9, 25.0), (100.9, 50.9)))
    outside = _routed_record(net_name="n2", centerline=((0.5, 25.0), (101.1, 25.0)))

    assert (
        _verify_routes_inside_routable_bbox(
            issues, [inside], routable_bbox_um=bbox, tolerance_um=1.0
        )
        == 0
    )
    assert issues == []
    assert (
        _verify_routes_inside_routable_bbox(
            issues, [outside], routable_bbox_um=None, tolerance_um=1.0
        )
        == 0
    )
    assert (
        _verify_routes_inside_routable_bbox(
            issues, [outside], routable_bbox_um=bbox, tolerance_um=1.0
        )
        == 1
    )


def _oracle_verify_route_obstacle_overlaps(
    issues,
    route_regions_by_key,
    *,
    obstacle_component,
    routed_layout,
    route_layer,
    obstacle_layers,
    dbu,
    legal_overlap_region,
    legal_overlap_regions_by_key=None,
    legal_overlap_regions_by_layer=None,
    min_overlap_area_um2=0.0,
):
    """Verbatim copy of the pre-Milestone-2 `_verify_route_obstacle_overlaps`
    per-route loop: intersects each route directly with the FULL residue,
    with no bounding-box index. Used as the correctness oracle for
    `_PolygonBucketIndex`.
    """
    overlap_count = 0
    for layer in obstacle_layers:
        source_component = obstacle_component
        if source_component is None:
            if layer == route_layer:
                continue
            source_component = routed_layout
        obstacle_region = _component_layer_region(source_component, layer)
        if obstacle_region.is_empty():
            continue
        all_routes_region = kdb.Region()
        for route_region in route_regions_by_key.values():
            all_routes_region.insert(route_region)
        residue = (all_routes_region & obstacle_region) - legal_overlap_region
        if residue.is_empty():
            continue
        for key, route_region in route_regions_by_key.items():
            touching = route_region & residue
            if touching.is_empty():
                continue
            per_key_regions = legal_overlap_regions_by_key
            if legal_overlap_regions_by_layer is not None:
                per_key_regions = legal_overlap_regions_by_layer.get(
                    layer,
                    per_key_regions,
                )
            overlap = touching
            if per_key_regions is not None:
                overlap = touching - per_key_regions.get(key, kdb.Region())
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="waveguide_obstacle_overlap",
                    message=f"Waveguide for {key[0]} overlaps obstacle layer {layer}.",
                    net_name=key[0],
                    details={
                        "obstacle_layer": layer,
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


def test_obstacle_overlap_issues_match_full_residue_oracle():
    obstacle_layout = Component()
    layer_a = (1, 0)
    layer_b = (2, 0)
    # Layer A: two illegal overlaps (n1, n2) and one overlap fully inside a
    # per-key legal window (n5, so no issue).
    obstacle_layout.add_polygon([(5.0, 0.0), (15.0, 0.0), (15.0, 2.0), (5.0, 2.0)], layer=layer_a)
    obstacle_layout.add_polygon(
        [(25.0, -1.0), (35.0, -1.0), (35.0, 3.0), (25.0, 3.0)], layer=layer_a
    )
    obstacle_layout.add_polygon(
        [(45.0, -1.0), (55.0, -1.0), (55.0, 3.0), (45.0, 3.0)], layer=layer_a
    )
    # Layer B: two illegal overlaps (n3, n4) and one legal-window overlap (n6).
    obstacle_layout.add_polygon(
        [(5.0, 10.0), (15.0, 10.0), (15.0, 12.0), (5.0, 12.0)], layer=layer_b
    )
    obstacle_layout.add_polygon(
        [(25.0, 9.0), (35.0, 9.0), (35.0, 13.0), (25.0, 13.0)], layer=layer_b
    )
    obstacle_layout.add_polygon(
        [(45.0, 9.0), (55.0, 9.0), (55.0, 13.0), (45.0, 13.0)], layer=layer_b
    )

    key1 = ("n1", ("a", "o1"), ("b", "o2"))
    key2 = ("n2", ("a", "o1"), ("b", "o2"))
    key3 = ("n3", ("a", "o1"), ("b", "o2"))
    key4 = ("n4", ("a", "o1"), ("b", "o2"))
    key5 = ("n5", ("a", "o1"), ("b", "o2"))
    key6 = ("n6", ("a", "o1"), ("b", "o2"))
    key7 = ("n7", ("a", "o1"), ("b", "o2"))

    route_regions_by_key = {
        key1: _box_region(0, 0, 10_000, 2_000),
        key2: _box_region(20_000, 0, 30_000, 2_000),
        key3: _box_region(0, 10_000, 10_000, 12_000),
        key4: _box_region(20_000, 10_000, 30_000, 12_000),
        key5: _box_region(40_000, 0, 50_000, 2_000),
        key6: _box_region(40_000, 10_000, 50_000, 12_000),
        # Far from every obstacle on both layers.
        key7: _box_region(1_000_000, 1_000_000, 1_010_000, 1_002_000),
    }
    legal_overlap_regions_by_key = {
        key5: _box_region(44_000, -1_000, 51_000, 3_000),
        key6: _box_region(44_000, 9_000, 51_000, 13_000),
    }

    kwargs = {
        "route_regions_by_key": route_regions_by_key,
        "obstacle_component": obstacle_layout,
        "routed_layout": Component(),
        "route_layer": (99, 0),
        "obstacle_layers": (layer_a, layer_b),
        "dbu": 0.001,
        "legal_overlap_region": kdb.Region(),
        "legal_overlap_regions_by_key": legal_overlap_regions_by_key,
    }

    shipped_issues: list[PhotonicVerificationIssue] = []
    shipped_count = _verify_route_obstacle_overlaps(shipped_issues, **kwargs)
    oracle_issues: list[PhotonicVerificationIssue] = []
    oracle_count = _oracle_verify_route_obstacle_overlaps(oracle_issues, **kwargs)

    assert shipped_count == oracle_count
    assert len(shipped_issues) >= 4
    assert shipped_issues == oracle_issues
    assert [issue.net_name for issue in shipped_issues] == ["n1", "n2", "n3", "n4"]


def test_obstacle_overlap_index_is_exact_on_random_rectangles():
    rng = random.Random(20260925)
    obstacle_layout = Component()
    layer = (7, 0)

    def _random_box_um(rng: random.Random) -> tuple[float, float, float, float]:
        x0 = float(rng.randint(0, 190))
        y0 = float(rng.randint(0, 190))
        w = float(rng.randint(1, 15))
        h = float(rng.randint(1, 15))
        return (x0, y0, x0 + w, y0 + h)

    route_boxes_um = [_random_box_um(rng) for _ in range(60)]
    obstacle_boxes_um = [_random_box_um(rng) for _ in range(80)]

    # Force a few deterministic edge cases the pure randomness might miss.
    route_boxes_um[0] = (0.0, 0.0, 10.0, 10.0)
    obstacle_boxes_um[0] = (10.0, 0.0, 20.0, 10.0)  # touches route 0 at x=10
    route_boxes_um[1] = (50.0, 50.0, 80.0, 80.0)
    obstacle_boxes_um[1] = (60.0, 60.0, 65.0, 65.0)  # fully inside route 1
    route_boxes_um[2] = (110.0, 110.0, 115.0, 115.0)
    obstacle_boxes_um[2] = (100.0, 100.0, 140.0, 140.0)  # fully contains route 2

    route_keys = [(f"n{i}", (f"src{i}", "o1"), (f"dst{i}", "o2")) for i in range(60)]
    route_regions_by_key = {
        key: _box_region(
            round(xmin * 1000),
            round(ymin * 1000),
            round(xmax * 1000),
            round(ymax * 1000),
        )
        for key, (xmin, ymin, xmax, ymax) in zip(route_keys, route_boxes_um, strict=True)
    }
    for xmin, ymin, xmax, ymax in obstacle_boxes_um:
        obstacle_layout.add_polygon(
            [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)],
            layer=layer,
        )

    legal_overlap_regions_by_key = {
        route_keys[0]: _box_region(0, 0, 10_000, 10_000),
        route_keys[5]: _box_region(0, 0, 5_000, 5_000),
        route_keys[10]: _box_region(0, 0, 1_000, 1_000),
    }

    kwargs = {
        "route_regions_by_key": route_regions_by_key,
        "obstacle_component": obstacle_layout,
        "routed_layout": Component(),
        "route_layer": (98, 0),
        "obstacle_layers": (layer,),
        "dbu": 0.001,
        "legal_overlap_region": kdb.Region(),
        "legal_overlap_regions_by_key": legal_overlap_regions_by_key,
    }

    shipped_issues: list[PhotonicVerificationIssue] = []
    shipped_count = _verify_route_obstacle_overlaps(shipped_issues, **kwargs)
    oracle_issues: list[PhotonicVerificationIssue] = []
    oracle_count = _oracle_verify_route_obstacle_overlaps(oracle_issues, **kwargs)

    assert oracle_count > 0
    assert shipped_issues != []
    assert shipped_count == oracle_count
    assert shipped_issues == oracle_issues


def test_polygon_bucket_index_candidates_are_conservative_and_exact():
    # 20 boxes on a 5x4 grid, each 10x10 dbu, spaced 20 dbu apart, so
    # adjacent boxes never touch or overlap unless a query is built to do so.
    region = kdb.Region()
    for row in range(4):
        for col in range(5):
            box = kdb.Box(col * 20, row * 20, col * 20 + 10, row * 20 + 10)
            region.insert(box)
    polygons = list(region.each())
    assert len(polygons) == 20
    index = _PolygonBucketIndex(region)

    def _exact_candidates(query: kdb.Box) -> set[int]:
        return {
            i
            for i, polygon in enumerate(polygons)
            if query.overlaps(polygon.bbox()) or query.touches(polygon.bbox())
        }

    equal_count = 0
    queries = [
        # Fully inside the box at col=1, row=0 (x 20-30, y 0-10): overlap.
        kdb.Box(22, 2, 28, 8),
        # Straddles the shared edge between col=2 (x 40-50) and col=3
        # (x 60-70) at y in [20, 30) (row=1): touches both, no overlap.
        kdb.Box(50, 20, 60, 30),
        # Far from every box: miss.
        kdb.Box(10_000, 10_000, 10_010, 10_010),
        # In a gap cell whose only registered box (col=0, row=0) does not
        # actually meet the query: the index may over-report (conservative).
        kdb.Box(15, 15, 19, 19),
    ]
    for query in queries:
        exact = _exact_candidates(query)
        candidates = set(index.candidates(query))
        assert candidates.issuperset(exact)
        if candidates == exact:
            equal_count += 1

    assert equal_count >= 3


def test_polygon_bucket_index_query_cost_is_bounded_by_polygon_count():
    # 40 small (250 x 250 dbu) polygons scattered over a huge 3,000,000 x
    # 5,000,000 dbu area. Without the area-based cell-size term, the
    # median-extent term alone (250, the polygon size) would force a
    # whole-area query to visit (3e6/250) * (5e6/250) = 240,000 cells; the
    # area term instead bounds a query over the whole indexed area to
    # roughly the polygon count.
    rng = random.Random(20260925)
    area_width = 3_000_000
    area_height = 5_000_000
    polygon_extent = 250
    n = 40
    region = kdb.Region()
    # Anchor two polygons at the extreme corners so the region's bbox is
    # exactly (0, 0, area_width, area_height), matching the nominal area
    # the cell-size formula below is checked against exactly.
    region.insert(kdb.Box(0, 0, polygon_extent, polygon_extent))
    region.insert(
        kdb.Box(
            area_width - polygon_extent,
            area_height - polygon_extent,
            area_width,
            area_height,
        )
    )
    for _ in range(n - 2):
        x0 = rng.randint(0, area_width - polygon_extent)
        y0 = rng.randint(0, area_height - polygon_extent)
        region.insert(kdb.Box(x0, y0, x0 + polygon_extent, y0 + polygon_extent))
    assert region.count() == n
    assert region.bbox() == kdb.Box(0, 0, area_width, area_height)

    index = _PolygonBucketIndex(region)
    expected_min_cell = math.ceil(math.sqrt((area_width * area_height) / n))
    assert index._cell >= expected_min_cell

    query = kdb.Box(0, 0, area_width, area_height)
    # The number of grid cells a whole-area query iterates, computed the
    # same way `candidates()` does internally: bounded by (a small
    # constant beyond) the polygon count, not by (area / median-extent^2).
    cells_visited = (query.right // index._cell - query.left // index._cell + 1) * (
        query.top // index._cell - query.bottom // index._cell + 1
    )
    assert cells_visited <= n + 8

    start = time.perf_counter()
    candidates = index.candidates(query)
    elapsed = time.perf_counter() - start

    assert sorted(candidates) == list(range(n))
    assert elapsed < 1.0
