"""The path-length diagnostics: matching-group and output residuals after the
meanders, the acceptance summary and its failure message, and the realized
bookkeeping by physical edge."""

from __future__ import annotations

import pytest
from photonic_router.plm.path_length_diagnostics import (
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
)
from photonic_router.plm.path_length_graph import (
    PortRef,
    RoutedEdgeKey,
)
from photonic_router.records.route_rust_types import MeanderEdgeReport
from photonic_router.routing.route_rust import (
    RoutedNetRecord,
    analyze_path_length_matching,
)

from tests.fixtures.path_length import (
    meander_report,
)
from tests.fixtures.synthetic_layouts import (
    PATH_LENGTH_INTERNAL_DELAYS_UM,
    PATH_LENGTH_NODE_TYPES,
    path_length_schematic,
)


def test_matching_group_diagnostics_reports_post_meander_residuals():
    schematic = path_length_schematic()
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=edge_short,
                requested_extra_length_um=20.0,
                status="planned",
                inserted_extra_length_um=20.0,
                planned_edges=(edge_short,),
            )
        ],
        realized={edge_short: 20.0},
    )

    groups = matching_group_diagnostics_to_info(result, report)

    assert len(groups) == 1
    assert groups[0]["within_tolerance"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.0)


def test_matching_group_diagnostics_tracks_disregarded_small_residual():
    schematic = path_length_schematic()
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=edge_short,
                requested_extra_length_um=0.5,
                status="below_minimum_bump",
            )
        ]
    )

    groups = matching_group_diagnostics_to_info(result, report)

    assert groups[0]["within_tolerance"] is True
    assert groups[0]["has_disregarded_residual"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.5)
    assert groups[0]["max_disregarded_residual_um"] == pytest.approx(0.5)


def test_path_length_acceptance_uses_physical_residual_not_accepted_unmatched():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 100.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 25.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 0.0,
                    "physical_residual_um": 25.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 25.0,
                    "meander_status": "below_minimum_bump",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)
    message = format_path_length_acceptance_failure(summary)

    assert summary["passed"] is False
    assert summary["failed_group_count"] == 1
    assert summary["failed_edge_count"] == 1
    assert summary["max_physical_residual_um"] == pytest.approx(25.0)
    assert "below_minimum_bump" in message
    assert "residual=25" in message


def test_path_length_acceptance_passes_exactly_realized_group():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 125.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 0.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 25.0,
                    "physical_residual_um": 0.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 0.0,
                    "meander_status": "planned",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)

    assert summary["passed"] is True
    assert summary["failed_group_count"] == 0
    assert summary["failed_edge_count"] == 0
    assert summary["max_physical_residual_um"] == pytest.approx(0.0)


def test_output_matching_diagnostics_fail_unrealized_output_requirement():
    edge_info = {
        "net_name": "out_short",
        "source": {"instance": "mmi0", "port": "o3"},
        "target": {"instance": "gc_out0", "port": "o1"},
    }
    output_info = {
        "enabled": True,
        "target_output_arrival_um": 100.0,
        "output_count": 2,
        "outputs": [
            {
                "node_name": "gc_out0",
                "arrival_um": 80.0,
                "missing_length_um": 20.0,
                "incoming_count": 1,
                "status": "requires_delay",
                "edge": edge_info,
            },
            {
                "node_name": "gc_out1",
                "arrival_um": 100.0,
                "missing_length_um": 0.0,
                "incoming_count": 1,
                "status": "not_required",
            },
        ],
    }
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=RoutedEdgeKey(
                    net_name="out_short",
                    source=PortRef(instance="mmi0", port="o3"),
                    target=PortRef(instance="gc_out0", port="o1"),
                ),
                requested_extra_length_um=20.0,
                status="no_candidate",
                unmatched_length_um=20.0,
            )
        ]
    )

    diagnostics = output_matching_diagnostics_to_info(output_info, report)
    summary = path_length_acceptance_summary(diagnostics)

    assert diagnostics[0]["node_name"] == "output_arrivals"
    assert diagnostics[0]["max_physical_residual_um"] == pytest.approx(20.0)
    assert summary["passed"] is False
    assert summary["failed_edge_count"] == 1


def test_matching_group_diagnostics_use_the_realized_placement_of_a_meander():
    """Bookkeeping by physical edge (2026-10-05): a meander booked on one
    incoming edge of the gate but drawn on the other (a delay-insertion
    candidate) shows up in ``realized_inserted_extra_length_um`` of the
    edge it was drawn on; the realized arrivals are walked over the graph,
    so the gate's realized spread reports the physical imbalance -- here
    the planned view is balanced (booked on the short edge) while the
    realized view is off by twice the meander, which the acceptance summary
    turns into a failure.
    """

    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    booked = MeanderEdgeReport(
        edge=short_edge,
        requested_extra_length_um=20.0,
        status="planned",
        inserted_extra_length_um=20.0,
    )
    # realized where booked: balanced in both views
    report = meander_report([booked], realized={short_edge: 20.0})
    groups = matching_group_diagnostics_to_info(result, report)
    assert len(groups) == 1
    by_net = {edge["edge"]["net_name"]: edge for edge in groups[0]["incoming_edges"]}
    assert by_net[short_edge.net_name]["realized_inserted_extra_length_um"] == pytest.approx(20.0)
    assert by_net[short_edge.net_name]["realized_length_um"] == pytest.approx(100.0)
    assert groups[0]["realized_arrival_spread_um"] == pytest.approx(0.0)
    assert groups[0]["realized_within_tolerance"] is True
    assert path_length_acceptance_summary(groups)["passed"] is True

    # realized on the other edge: the planned view still says balanced, the
    # realized view reports the imbalance and the acceptance fails on it
    report_misplaced = meander_report([booked], realized={long_edge: 20.0})
    groups = matching_group_diagnostics_to_info(result, report_misplaced)
    assert groups[0]["within_tolerance"] is True
    assert groups[0]["realized_arrival_spread_um"] == pytest.approx(40.0)
    assert groups[0]["realized_within_tolerance"] is False
    summary = path_length_acceptance_summary(groups)
    assert summary["passed"] is False
    assert summary["realized_failed_group_count"] == 1
    assert "realized bookkeeping" in format_path_length_acceptance_failure(summary)

    # the GDS meta info form carries the realized table per edge
    info = report_misplaced.to_info_dict()
    assert info["realized_extra_length_um_by_edge"] == [
        {
            "edge": {
                "net_name": long_edge.net_name,
                "source": {"instance": "gc_1", "port": "o1"},
                "target": {"instance": "mmi_0", "port": "o1"},
            },
            "extra_length_um": 20.0,
        }
    ]
    assert info["results"][0]["inserted_extra_length_um"] == pytest.approx(20.0)
