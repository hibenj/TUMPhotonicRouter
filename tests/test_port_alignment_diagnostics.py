from types import SimpleNamespace
from typing import Any, cast

import pytest

from photonic_router.path_length_graph import PortRef
from photonic_router.static_obstacle_builder import _load_rust_backend
from translation.route_rust_records import build_port_alignment_diagnostics
from translation.route_rust_types import RoutedNetRecord


def test_port_alignment_diagnostics_reports_endpoint_mu_values():
    route_obj = SimpleNamespace(
        states=[
            SimpleNamespace(x=10, y=20),
            SimpleNamespace(x=30, y=40),
        ]
    )
    record = RoutedNetRecord(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o2"),
        route_obj=route_obj,
        total_length_um=12.5,
        source_port_center_um=(5.2, 10.4),
        target_port_center_um=(15.8, 20.1),
        source_port_orientation_deg=0.0,
        target_port_orientation_deg=180.0,
    )

    diagnostics = build_port_alignment_diagnostics(
        [record],
        realization_grid_spec=(100, 100, 0.5, 0.0, 0.0),
    )

    assert len(diagnostics) == 1
    entry = diagnostics[0]
    source = cast(dict[str, Any], entry["source"])
    target = cast(dict[str, Any], entry["target"])
    assert entry["net_name"] == "n0"
    assert entry["route_total_length_um"] == 12.5
    assert source["route_cell"] == [10, 20]
    assert source["route_grid_center_um"] == [5.25, 10.25]
    assert source["mu_x_um"] == pytest.approx(-0.05)
    assert source["mu_y_um"] == pytest.approx(0.15)
    assert target["route_cell"] == [30, 40]
    assert target["route_grid_center_um"] == [15.25, 20.25]
    assert target["mu_x_um"] == pytest.approx(0.55)
    assert target["mu_y_um"] == pytest.approx(-0.15)


def test_mmi_heater_pass0_characterizes_current_port_alignment():
    pytest.importorskip("gdsfactory")
    if _load_rust_backend() is None:
        pytest.skip("Rust backend unavailable for mmi_heater alignment diagnostics.")

    from benchmarks.mmi_heater import build_schematic
    from translation.layout_from_schematic import layout_from_schematic
    from translation.route_rust import route_nets_rust

    schematic = build_schematic()
    layout = layout_from_schematic(schematic)

    _, artifacts = route_nets_rust(
        layout,
        schematic,
        defer_realization=True,
        include_heater_obstacles=True,
        collect_route_stats=True,
    )

    records_by_name = {record.net_name: record for record in artifacts.routed_net_records}
    assert records_by_name["gc0_to_mmi0_in1"].total_length_um == pytest.approx(149.0)
    assert records_by_name["gc1_to_mmi0_in2"].total_length_um == pytest.approx(149.5)
    assert (
        records_by_name["gc1_to_mmi0_in2"].total_length_um
        - records_by_name["gc0_to_mmi0_in1"].total_length_um
    ) == pytest.approx(0.5)

    diagnostics_by_name = {
        str(entry["net_name"]): entry
        for entry in artifacts.port_alignment_diagnostics
    }
    assert set(diagnostics_by_name) == set(records_by_name)
    for entry in diagnostics_by_name.values():
        max_offset = entry["max_endpoint_offset_abs_um"]
        assert isinstance(max_offset, (int, float))
        assert max_offset > 0.0
        for endpoint_name in ("source", "target"):
            endpoint = cast(dict[str, Any], entry[endpoint_name])
            assert endpoint["port_center_um"] is not None
            assert endpoint["route_grid_center_um"] is not None
            assert endpoint["mu_x_um"] is not None
            assert endpoint["mu_y_um"] is not None


def test_mmi_heater_route_match_uses_corrected_records_for_realization():
    pytest.importorskip("gdsfactory")
    if _load_rust_backend() is None:
        pytest.skip("Rust backend unavailable for mmi_heater endpoint correction.")

    from benchmark_metadata import resolve_internal_delays_for_instances
    from benchmarks.mmi_heater import INTERNAL_DELAYS_UM, NODE_TYPES, build_schematic
    from translation.layout_from_schematic import layout_from_schematic
    from translation.route_rust import route_match_and_realize

    schematic = build_schematic()
    layout = layout_from_schematic(schematic)
    result = route_match_and_realize(
        layout,
        schematic,
        enable_path_length_matching=False,
        node_types=NODE_TYPES,
        internal_delays_um=resolve_internal_delays_for_instances(
            schematic,
            INTERNAL_DELAYS_UM,
        ),
        include_heater_obstacles=True,
        collect_route_stats=True,
    )

    records = result.debug_artifacts.routed_net_records
    assert records
    records_by_name = {record.net_name: record for record in records}
    assert any(record.corrected_centerline_um for record in records)
    assert any(record.base_total_length_um is not None for record in records)
    assert any(
        record.base_total_length_um is not None
        and abs(float(record.total_length_um) - float(record.base_total_length_um)) > 1.0e-6
        for record in records
    )
    assert records_by_name["gc0_to_mmi0_in1"].total_length_um == pytest.approx(
        records_by_name["gc1_to_mmi0_in2"].total_length_um
    )
    assert records_by_name["mmi1_out1_to_gc2"].total_length_um == pytest.approx(
        records_by_name["mmi1_out2_to_gc3"].total_length_um
    )
    assert result.path_length_analysis_info is None
    assert result.meander_insertion_report_info is None


def test_py_router_exposes_endpoint_corrected_centerline_and_polygon():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for endpoint correction API test.")

    grid = rust_backend.GridSpec(64, 64, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=1,
        allow_45_degree_turns=True,
    )
    astar = rust_backend.AStarConfig(max_iterations=10_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)

    route = router.route_single_net(
        rust_backend.State(1, 2, 0),
        rust_backend.State(5, 6, 2),
    )
    centerline = router.route_port_corrected_centerline(
        route,
        source_port_um=(1.2, 2.1),
        target_port_um=(5.8, 6.4),
    )
    polygon = router.realize_route_polygon_with_endpoint_correction(
        route,
        0.5,
        source_port_um=(1.2, 2.1),
        target_port_um=(5.8, 6.4),
    )

    assert centerline[0] == pytest.approx((1.2, 2.1))
    assert centerline[-1] == pytest.approx((5.8, 6.4))
    assert centerline[1][1] == pytest.approx(centerline[0][1])
    assert centerline[-2][0] == pytest.approx(centerline[-1][0])
    assert polygon[0] == pytest.approx(polygon[-1])
    assert min(point[0] for point in polygon) <= 1.2 + 1.0e-9
    assert max(point[0] for point in polygon) >= 5.8 - 1.0e-9
