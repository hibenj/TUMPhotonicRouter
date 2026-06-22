from types import SimpleNamespace
from typing import cast

from photonic_router.path_length_graph import PortRef, RoutedEdgeKey
from gdsfactory.typings import Port

from translation.route_rust_records import (
    RouteBookkeeping,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import RouteJob, RoutedNetRecord


def _job(net_id: int, net_name: str) -> RouteJob:
    return RouteJob(
        net_id=net_id,
        route_index=net_id,
        net_name=net_name,
        inst1=f"src{net_id}",
        port1="o1",
        inst2=f"dst{net_id}",
        port2="o2",
        source_port=cast(Port, cast(object, SimpleNamespace())),
        target_port=cast(Port, cast(object, SimpleNamespace())),
    )


def test_route_bookkeeping_preserves_route_order_and_edge_lengths():
    bookkeeping = RouteBookkeeping(route_order=[2, 1], diagnostics_enabled=True)

    bookkeeping.record_route(
        _job(1, "n1"),
        SimpleNamespace(total_length_um=11.5),
        [(1, 1), (2, 2)],
        route_cells={(10, 10)},
    )
    bookkeeping.record_route(
        _job(2, "n2"),
        SimpleNamespace(total_length_um=22.5),
        [(3, 3)],
        route_cells={(20, 20)},
    )

    ordered = bookkeeping.ordered_records()
    assert [record.net_name for record in ordered] == ["n2", "n1"]
    assert bookkeeping.routed_edge_lengths() == routed_edge_lengths_from_records(ordered)
    assert bookkeeping.committed_dynamic_cells() == {(10, 10), (20, 20)}
    assert bookkeeping.committed_dynamic_cells(exclude_net_id=1) == {(20, 20)}


def test_route_bookkeeping_clear_and_restore_records():
    bookkeeping = RouteBookkeeping(route_order=[1], diagnostics_enabled=False)
    job = _job(1, "n1")
    route_obj = SimpleNamespace(total_length_um=11.5)

    bookkeeping.record_route(job, route_obj, [(1, 1)], route_cells={(10, 10)})
    snapshot_records = dict(bookkeeping.records_by_id)
    snapshot_lengths = dict(bookkeeping.lengths_by_id)

    bookkeeping.clear_route(1)
    assert bookkeeping.ordered_records() == []
    assert bookkeeping.committed_dynamic_cells() == set()

    bookkeeping.restore_records(snapshot_records, snapshot_lengths)
    edge = RoutedEdgeKey(
        net_name="n1",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="dst1", port="o2"),
    )
    assert bookkeeping.routed_edge_lengths() == {edge: 11.5}


def test_routed_edge_lengths_from_records_uses_record_identity_fields():
    record = RoutedNetRecord(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o2"),
        route_obj=object(),
        total_length_um=42,
    )

    assert routed_edge_lengths_from_records([record]) == {
        RoutedEdgeKey(
            net_name="n0",
            source=PortRef(instance="src", port="o1"),
            target=PortRef(instance="dst", port="o2"),
        ): 42.0
    }
