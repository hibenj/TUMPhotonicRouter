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
    _BoxBucketIndex,
    _combined_region,
    _component_layer_region,
    _net_id_pair,
    _polygon_regions_by_pair_um,
    _region_area_um2,
    _region_bbox_um,
    _verify_crossing_component_overlaps,
    _verify_crossing_component_route_overlaps,
    _verify_cross_net_route_overlaps,
    _verify_record_coverage,
    _realized_record_region,
    _verify_route_obstacle_overlaps,
    _verify_routes_inside_routable_bbox,
    _verify_self_intersecting_routes,
    _polyline_self_intersects_um,
    verify_photonic_routing,
)
from translation.route_rust_realization import (
    build_realization_router,
    realize_routed_net_records,
    realized_record_polygons,
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


def _oracle_verify_cross_net_route_overlaps(
    issues,
    route_regions_by_key,
    *,
    dbu,
    legal_overlap_region,
    net_id_by_key=None,
    legal_overlap_regions_by_net_id_pair=None,
    min_overlap_area_um2=0.0,
):
    """Verbatim copy of the pre-Milestone-3 `_verify_cross_net_route_overlaps`
    double loop: visits every pair (i, j), i < j, in record order with a
    bounding-box prefilter, and no spatial index. Used as the correctness
    oracle for the `_BoxBucketIndex`-based candidate query.
    """
    overlap_count = 0
    items = [(key, region) for key, region in route_regions_by_key.items() if not region.is_empty()]
    bboxes = [region.bbox() for _key, region in items]
    for index, (left_key, left_region) in enumerate(items):
        left_bbox = bboxes[index]
        for offset, (right_key, right_region) in enumerate(items[index + 1 :], start=index + 1):
            if not left_bbox.overlaps(bboxes[offset]) and not left_bbox.touches(bboxes[offset]):
                continue
            allowed_region = legal_overlap_region
            if (
                net_id_by_key is not None
                and legal_overlap_regions_by_net_id_pair is not None
                and left_key in net_id_by_key
                and right_key in net_id_by_key
            ):
                allowed_region = _combined_region(
                    legal_overlap_region,
                    legal_overlap_regions_by_net_id_pair.get(
                        _net_id_pair(net_id_by_key[left_key], net_id_by_key[right_key]),
                        kdb.Region(),
                    ),
                )
            overlap = (left_region & right_region) - allowed_region
            if overlap.is_empty():
                continue
            overlap_area_um2 = _region_area_um2(overlap, dbu)
            if overlap_area_um2 <= float(min_overlap_area_um2):
                continue
            overlap_count += 1
            issues.append(
                PhotonicVerificationIssue(
                    code="cross_net_waveguide_overlap",
                    message=(f"Waveguide for {left_key[0]} overlaps waveguide for {right_key[0]}."),
                    net_name=left_key[0],
                    details={
                        "other_net_name": right_key[0],
                        "overlap_area_um2": overlap_area_um2,
                        "overlap_bbox_um": _region_bbox_um(overlap, dbu),
                    },
                )
            )
    return overlap_count


def test_cross_net_overlap_issues_match_double_loop_oracle():
    def _route(net_name, xmin, ymin, xmax, ymax):
        key = (net_name, (f"{net_name}_src", "o1"), (f"{net_name}_dst", "o2"))
        return key, _box_region(xmin, ymin, xmax, ymax)

    # Lane 0 (y 0-2000): n0 illegally overlaps both n1 and n2 (same route
    # in two issues, so the order of issues matters); n1 and n2 do not
    # overlap each other.
    r0 = _route("n0", 0, 0, 30_000, 2_000)
    r1 = _route("n1", 5_000, 0, 15_000, 2_000)
    r2 = _route("n2", 20_000, 0, 28_000, 2_000)
    # Lane 1 (y 20000-22000): n3 illegally overlaps both n4 and n5.
    r3 = _route("n3", 0, 20_000, 30_000, 22_000)
    r4 = _route("n4", 5_000, 20_000, 15_000, 22_000)
    r5 = _route("n5", 20_000, 20_000, 28_000, 22_000)
    # Lane 2 (y 40000-42000): n6 illegally overlaps n7 (the 5th illegal pair).
    r6 = _route("n6", 0, 40_000, 10_000, 42_000)
    r7 = _route("n7", 5_000, 40_000, 15_000, 42_000)
    # Lanes 3 and 4: pairs that only touch at an edge -- zero-area overlap,
    # no issue.
    r8 = _route("n8", 0, 60_000, 10_000, 62_000)
    r9 = _route("n9", 10_000, 60_000, 20_000, 62_000)
    r10 = _route("n10", 0, 80_000, 5_000, 82_000)
    r11 = _route("n11", 5_000, 80_000, 12_000, 82_000)
    # Lanes 5 and 6: overlaps fully inside the global legal_overlap_region.
    r12 = _route("n12", 0, 100_000, 10_000, 102_000)
    r13 = _route("n13", 5_000, 100_000, 15_000, 102_000)
    r14 = _route("n14", 0, 120_000, 10_000, 122_000)
    r15 = _route("n15", 5_000, 120_000, 15_000, 122_000)
    # Lane 7: an overlap that is legal only under its net-id-pair's own
    # legal region (not covered by the global legal_overlap_region).
    r16 = _route("n16", 0, 140_000, 10_000, 142_000)
    r17 = _route("n17", 5_000, 140_000, 15_000, 142_000)
    # Far from every other route.
    r18 = _route("n18", 0, 10_000_000, 5_000, 10_002_000)
    r19 = _route("n19", 0, 20_000_000, 5_000, 20_002_000)

    # Deliberate, non-sequential insertion order (so a correct
    # implementation must not depend on insertion order matching the
    # geometric layout above).
    route_regions_by_key = {
        route[0]: route[1]
        for route in (
            r9,
            r0,
            r16,
            r3,
            r12,
            r18,
            r1,
            r6,
            r14,
            r10,
            r4,
            r17,
            r2,
            r19,
            r13,
            r7,
            r11,
            r5,
            r15,
            r8,
        )
    }

    net_id_by_key = {r16[0]: 1, r17[0]: 2}
    legal_overlap_regions_by_net_id_pair = {
        _net_id_pair(1, 2): _box_region(5_000, 140_000, 10_000, 142_000),
    }
    legal_overlap_region = kdb.Region()
    legal_overlap_region.insert(kdb.Box(0, 95_000, 20_000, 107_000))
    legal_overlap_region.insert(kdb.Box(0, 115_000, 20_000, 127_000))

    kwargs = {
        "route_regions_by_key": route_regions_by_key,
        "dbu": 0.001,
        "legal_overlap_region": legal_overlap_region,
        "net_id_by_key": net_id_by_key,
        "legal_overlap_regions_by_net_id_pair": legal_overlap_regions_by_net_id_pair,
    }

    shipped_issues: list[PhotonicVerificationIssue] = []
    shipped_count = _verify_cross_net_route_overlaps(shipped_issues, **kwargs)
    oracle_issues: list[PhotonicVerificationIssue] = []
    oracle_count = _oracle_verify_cross_net_route_overlaps(oracle_issues, **kwargs)

    assert shipped_count == oracle_count
    assert len(shipped_issues) >= 5
    assert shipped_issues == oracle_issues


def test_cross_net_overlap_index_is_exact_on_random_rectangles():
    rng = random.Random(20260925)
    n = 80
    chip_span = 200_000  # 200 um chip, in dbu at dbu=0.001 um.

    def _random_box(rng: random.Random) -> tuple[int, int, int, int]:
        x0 = rng.randint(0, chip_span - 5_000)
        y0 = rng.randint(0, chip_span - 5_000)
        w = rng.randint(100, 5_000)
        h = rng.randint(100, 5_000)
        return (x0, y0, x0 + w, y0 + h)

    boxes = [_random_box(rng) for _ in range(n)]
    # A few chip-spanning routes.
    boxes[0] = (0, 0, chip_span, 50_000)
    boxes[1] = (0, chip_span - 50_000, chip_span, chip_span)
    boxes[2] = (0, 0, 50_000, chip_span)
    # Forced touching pairs: share an edge, zero-area overlap.
    boxes[3] = (10_000, 10_000, 15_000, 12_000)
    boxes[4] = (15_000, 10_000, 20_000, 12_000)  # touches box 3 at x=15000
    boxes[5] = (30_000, 30_000, 35_000, 32_000)
    boxes[6] = (30_000, 32_000, 35_000, 34_000)  # touches box 5 at y=32000
    # A forced overlap below the min-overlap threshold.
    boxes[7] = (50_000, 50_000, 50_100, 50_100)
    boxes[8] = (50_050, 50_050, 50_150, 50_150)  # 0.0025 um^2 overlap
    # A forced overlap comfortably above the threshold.
    boxes[9] = (60_000, 60_000, 70_000, 62_000)
    boxes[10] = (65_000, 60_000, 75_000, 62_000)  # 10 um^2 overlap

    route_keys = [(f"n{i}", (f"src{i}", "o1"), (f"dst{i}", "o2")) for i in range(n)]
    order = list(range(n))
    rng.shuffle(order)  # deliberate, non-sequential insertion order
    route_regions_by_key = {route_keys[i]: _box_region(*boxes[i]) for i in order}

    kwargs = {
        "route_regions_by_key": route_regions_by_key,
        "dbu": 0.001,
        "legal_overlap_region": kdb.Region(),
        "min_overlap_area_um2": 0.05,
    }

    shipped_issues: list[PhotonicVerificationIssue] = []
    shipped_count = _verify_cross_net_route_overlaps(shipped_issues, **kwargs)
    oracle_issues: list[PhotonicVerificationIssue] = []
    oracle_count = _oracle_verify_cross_net_route_overlaps(oracle_issues, **kwargs)

    assert oracle_count > 0
    assert shipped_issues != []
    assert shipped_count == oracle_count
    assert shipped_issues == oracle_issues


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
    `_BoxBucketIndex`.
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
    boxes = [
        kdb.Box(col * 20, row * 20, col * 20 + 10, row * 20 + 10)
        for row in range(4)
        for col in range(5)
    ]
    assert len(boxes) == 20
    index = _BoxBucketIndex(boxes)

    def _exact_candidates(query: kdb.Box) -> set[int]:
        return {i for i, box in enumerate(boxes) if query.overlaps(box) or query.touches(box)}

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
    # Anchor two boxes at the extreme corners so the indexed bbox is
    # exactly (0, 0, area_width, area_height), matching the nominal area
    # the cell-size formula below is checked against exactly.
    boxes = [
        kdb.Box(0, 0, polygon_extent, polygon_extent),
        kdb.Box(
            area_width - polygon_extent,
            area_height - polygon_extent,
            area_width,
            area_height,
        ),
    ]
    for _ in range(n - 2):
        x0 = rng.randint(0, area_width - polygon_extent)
        y0 = rng.randint(0, area_height - polygon_extent)
        boxes.append(kdb.Box(x0, y0, x0 + polygon_extent, y0 + polygon_extent))
    assert len(boxes) == n
    overall_bbox = kdb.Box()
    for box in boxes:
        overall_bbox += box
    assert overall_bbox == kdb.Box(0, 0, area_width, area_height)

    index = _BoxBucketIndex(boxes)
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


def _component_path_region(
    record: RoutedNetRecord,
    *,
    route_layer: tuple[int, int],
    route_width_um: float,
    realization_grid_spec: tuple[int, int, float, float, float],
    router: object,
    crossing_plan_info: dict[str, object] | None = None,
    enable_endpoint_correction: bool = True,
) -> kdb.Region:
    """Oracle for `test_realized_record_region_equals_component_path`.

    A verbatim copy of `_realized_record_region`
    (`translation/photonic_verification.py`) as it read before Milestone 4
    step 2 of `.agent/execplans/2026-09-24-verifier-speed-on-large-layouts.md`:
    realize the record into a scratch `Component` and read the layer back as
    a region. `realize_routed_net_records`'s outward behaviour for a given
    router is unchanged by Milestone 4 (step 1 only adds an optional
    pre-built router; step 2 only extracts the per-record polygon
    computation into a helper it already called inline), so calling today's
    `realize_routed_net_records` here still exercises the pre-Milestone-4
    Component/add_polygon/get_polygons path byte for byte.
    """
    temp = Component()
    realize_routed_net_records(
        temp,
        [record],
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=realization_grid_spec,
        crossing_plan_info=crossing_plan_info,
        enable_endpoint_correction=enable_endpoint_correction,
        router=router,
    )
    return _component_layer_region(temp, route_layer)


def test_realized_record_region_equals_component_path():
    """Milestone 4 step 2 (2026-09-25) exactness: the region built directly
    from the router's polygon (`realized_record_polygons`, no scratch
    `Component`) must be bit-identical to the region built the old way
    (`_component_path_region`, above), for every kind of record this module
    exercises for real (not monkeypatched) plus every routed record of
    `heater_s_mod`.

    `benes_4x4_flat --crossing-mode lidar-pure` (suggested by the plan for a
    real crossing-clip record) took over 120 s to route inside this process
    and was dropped as too slow for this unit test; the crossing-clip branch
    is covered here by a synthetic clip carved out of a real router-realized
    bend polygon (below), and end to end by the byte-identical
    `benes_16x16_flat --crossing-mode lidar-pure` verification report checked
    by this plan's validation harness.
    """
    route_width_um = 0.5
    route_layer = (1, 0)
    realization_grid_spec = (40, 40, 1.0, -15.0, -20.0)

    backend = _load_rust_backend()
    assert backend is not None
    router = backend.PyPhotonicRouter(
        backend.GridSpec(40, 40, 1.0, -15.0, -20.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(max_iterations=10_000),
    )
    bend_route_obj = router.route_single_net_and_commit(
        1,
        backend.State(15, 20, 0),
        backend.State(25, 10, 6),
        block_radius_cells=0,
    )

    # A real router-realized bend (terminal-tangent centerline branch).
    record_bend = _routed_record(
        centerline=((0.0, 0.0), (10.0, 0.0), (10.0, -10.0)),
        source_port_center_um=(0.0, 0.0),
        target_port_center_um=(10.0, -10.0),
        route_obj=bend_route_obj,
    )
    # The same route, forced through the plain `realize_route_polygon`
    # branch (no corrected centerline, no ports).
    record_plain = replace(
        record_bend,
        corrected_centerline_um=(),
        source_port_center_um=None,
        target_port_center_um=None,
    )
    # The same route, forced through `route_port_corrected_centerline` (only
    # one port set, no corrected centerline yet).
    record_port_corrected = replace(
        record_bend,
        corrected_centerline_um=(),
        source_port_center_um=(0.0, 0.0),
        target_port_center_um=None,
    )

    # A crossing clip region cut out of `record_bend`'s own realized
    # polygon, so the clip subtraction runs on real (not box) geometry.
    unclipped_region = _component_path_region(
        record_bend,
        route_layer=route_layer,
        route_width_um=route_width_um,
        realization_grid_spec=realization_grid_spec,
        router=router,
    )
    bbox = unclipped_region.bbox()
    dbu = 0.001
    mid_x_um = ((bbox.left + bbox.right) / 2.0) * dbu
    ymin_um = bbox.bottom * dbu - 1.0
    ymax_um = bbox.top * dbu + 1.0
    record_clip = replace(record_bend, net_id=42)
    crossing_plan_info = {
        "enabled": True,
        "realized_intersections": [
            {
                "classification": "legal_test_clip",
                "net_id_a": 42,
                "net_id_b": 43,
                "crossing_footprint_polygon_um": [
                    (mid_x_um - 1.0, ymin_um),
                    (mid_x_um + 1.0, ymin_um),
                    (mid_x_um + 1.0, ymax_um),
                    (mid_x_um - 1.0, ymax_um),
                ],
            }
        ],
    }
    # The clip must actually cut something, or this record is not testing
    # what it claims to.
    clip_region_um = kdb.Region(
        kdb.DPolygon(
            [
                kdb.DPoint(mid_x_um - 1.0, ymin_um),
                kdb.DPoint(mid_x_um + 1.0, ymin_um),
                kdb.DPoint(mid_x_um + 1.0, ymax_um),
                kdb.DPoint(mid_x_um - 1.0, ymax_um),
            ]
        ).to_itype(dbu)
    )
    assert not (unclipped_region & clip_region_um).is_empty()
    assert not (unclipped_region - clip_region_um).is_empty()

    fixtures: list[
        tuple[RoutedNetRecord, tuple[int, int, float, float, float], dict[str, object] | None]
    ] = [
        (record_bend, realization_grid_spec, None),
        (record_plain, realization_grid_spec, None),
        (record_port_corrected, realization_grid_spec, None),
        (record_clip, realization_grid_spec, crossing_plan_info),
    ]

    for record, grid_spec, plan_info in fixtures:
        old_region = _component_path_region(
            record,
            route_layer=route_layer,
            route_width_um=route_width_um,
            realization_grid_spec=grid_spec,
            router=router,
            crossing_plan_info=plan_info,
        )
        new_region = _realized_record_region(
            record,
            route_width_um=route_width_um,
            realization_grid_spec=grid_spec,
            dbu=dbu,
            router=router,
            crossing_plan_info=plan_info,
        )
        assert (new_region ^ old_region).is_empty(), record.net_name
        assert new_region.count() == old_region.count(), record.net_name

    # heater_s_mod, routed with its own stable configuration (90-degree
    # routing, 10 um bends, path-length matching, heater obstacles --
    # `benchmarks/heater_s_mod.py`'s `STABLE_ROUTING_FLAGS`, the way
    # `tests/e2e/test_routing_flow_stats.py::test_heater_s_mod_90_degree_plm_regression`
    # loads and routes it), every routed record.
    from photonic_router.static_obstacle_builder import StaticObstacleMapConfig

    import benchmark_metadata
    from routing_flow import load_benchmark
    from translation.layout_from_schematic import layout_from_schematic
    from translation.routing.api import route_match_and_realize

    schematic = load_benchmark("heater_s_mod")
    unrouted_layout = layout_from_schematic(schematic)
    metadata = benchmark_metadata.load_benchmark_metadata("heater_s_mod", schematic=schematic)
    result = route_match_and_realize(
        unrouted_layout,
        schematic,
        enable_path_length_matching=True,
        path_length_match_outputs=True,
        node_types=metadata.get("node_types"),
        internal_delays_um=metadata.get("internal_delays_um"),
        debug_dir=None,
        debug_prefix="heater_s_mod",
        allow_45_degree_turns=False,
        bend_radius_um=10.0,
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        collect_route_stats=True,
        include_heater_obstacles=True,
        obstacle_config=StaticObstacleMapConfig(
            grid_size_um=2.0,
            obstacle_mode="bounding_boxes",
            clearance_um=0.0,
            heater_clearance_um=10.0,
            chip_add_x_um=0.0,
            chip_add_y_um=40.0,
            clear_port_open_cells_from_static=False,
        ),
    )
    heater_debug_artifacts = result.debug_artifacts
    heater_records = heater_debug_artifacts.routed_net_records
    assert len(heater_records) > 0
    heater_grid_spec = heater_debug_artifacts.realization_grid_spec
    heater_router = build_realization_router(
        realization_grid_spec=heater_grid_spec,
        allow_45_degree_turns=heater_debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=heater_debug_artifacts.realization_bend_radius_cells,
    )
    for record in heater_records:
        old_region = _component_path_region(
            record,
            route_layer=route_layer,
            route_width_um=route_width_um,
            realization_grid_spec=heater_grid_spec,
            router=heater_router,
        )
        new_region = _realized_record_region(
            record,
            route_width_um=route_width_um,
            realization_grid_spec=heater_grid_spec,
            dbu=dbu,
            router=heater_router,
        )
        assert (new_region ^ old_region).is_empty(), record.net_name
        assert new_region.count() == old_region.count(), record.net_name


def test_realized_record_polygons_returns_dbu_polygons_matching_component_add_polygon():
    """Narrower unit check backing the region-equality test above: the
    micron-to-dbu conversion `realized_record_polygons` uses must match what
    `Component.add_polygon` does (`gdsfactory/component.py`: a `kdb.DPolygon`
    converted with `DPolygon.to_itype(dbu)`), point for point, not just as
    equal regions.
    """
    route_width_um = 0.5
    realization_grid_spec = (40, 40, 1.0, -15.0, -20.0)
    dbu = 0.001

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
    record = _routed_record(
        centerline=((0.0, 0.0), (10.0, 0.0), (10.0, -10.0)),
        source_port_center_um=(0.0, 0.0),
        target_port_center_um=(10.0, -10.0),
        route_obj=route_obj,
    )

    polygons = realized_record_polygons(
        record,
        route_width_um=route_width_um,
        realization_grid_spec=realization_grid_spec,
        dbu=dbu,
        router=router,
    )
    assert len(polygons) == 1

    temp = Component()
    realize_routed_net_records(
        temp,
        [record],
        route_width_um=route_width_um,
        route_layer=(1, 0),
        realization_grid_spec=realization_grid_spec,
        router=router,
    )
    component_polygons = temp.get_polygons(merge=False, by="tuple").get((1, 0), [])
    assert len(component_polygons) == 1
    assert list(polygons[0].each_point_hull()) == list(component_polygons[0].each_point_hull())
