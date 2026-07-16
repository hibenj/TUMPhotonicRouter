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
    _verify_crossing_component_overlaps,
    _verify_crossing_component_route_overlaps,
    _verify_cross_net_route_overlaps,
    _verify_record_coverage,
    _verify_route_obstacle_overlaps,
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
