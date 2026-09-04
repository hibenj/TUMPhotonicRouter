"""Net routing order (contribution 1, S3): pure ordering rules on synthetic jobs."""

from typing import cast

import pytest

from translation.route_order import (
    NET_ORDERS,
    depth_by_node_from_jobs,
    order_route_jobs,
)
from translation.route_rust_types import RouteJob


def _job(net_id: int, inst1: str, inst2: str) -> RouteJob:
    return RouteJob(
        net_id=net_id,
        route_index=net_id,
        net_name=f"n_{net_id}",
        inst1=inst1,
        port1="o1",
        inst2=inst2,
        port2="o1",
        source_port=cast(object, None),  # type: ignore[arg-type]
        target_port=cast(object, None),  # type: ignore[arg-type]
    )


# Two stages: in -> a/b (depth 0), a/b -> out1/out2 (depth 1). Declaration
# order puts the depth-1 nets first on purpose.
JOBS = [
    _job(1, "a", "out1"),
    _job(2, "b", "out2"),
    _job(3, "a", "out2"),
    _job(4, "in", "a"),
    _job(5, "in", "b"),
]


def _ids(jobs: list[RouteJob]) -> list[int]:
    return [int(job.net_id) for job in jobs]


def test_depth_by_node_counts_hops_from_true_sources():
    depths = depth_by_node_from_jobs(JOBS)
    assert depths == {"in": 0, "a": 1, "b": 1, "out1": 2, "out2": 2}


def test_topological_order_is_depth_then_declaration_order():
    ordered = order_route_jobs(
        JOBS, net_order="topological", depth_by_node=depth_by_node_from_jobs(JOBS)
    )
    assert _ids(ordered) == [4, 5, 1, 2, 3]


def test_topological_span_order_routes_shortest_nets_first_within_a_layer():
    span = {4: 10, 5: 5, 1: 30, 2: 20, 3: 25}
    ordered = order_route_jobs(
        JOBS,
        net_order="topological-span",
        depth_by_node=depth_by_node_from_jobs(JOBS),
        span_by_net_id=span,
    )
    assert _ids(ordered) == [5, 4, 2, 3, 1]


def test_plan_crossing_orders_use_planned_crossing_counts_within_a_layer():
    planned = {1: 2, 2: 0, 3: 1}  # depth-1 nets; depth-0 nets have none
    depths = depth_by_node_from_jobs(JOBS)
    desc = order_route_jobs(
        JOBS,
        net_order="plan-crossings-desc",
        depth_by_node=depths,
        planned_crossings_by_net_id=planned,
    )
    assert _ids(desc) == [4, 5, 1, 3, 2]
    asc = order_route_jobs(
        JOBS,
        net_order="plan-crossings-asc",
        depth_by_node=depths,
        planned_crossings_by_net_id=planned,
    )
    assert _ids(asc) == [4, 5, 2, 3, 1]


def test_plan_crossing_orders_tiebreak_by_declaration_order():
    depths = depth_by_node_from_jobs(JOBS)
    ordered = order_route_jobs(
        JOBS, net_order="plan-crossings-desc", depth_by_node=depths, planned_crossings_by_net_id={}
    )
    assert _ids(ordered) == [4, 5, 1, 2, 3]


def test_orders_require_their_inputs_and_reject_unknown_names():
    depths = depth_by_node_from_jobs(JOBS)
    with pytest.raises(ValueError, match="span"):
        order_route_jobs(JOBS, net_order="topological-span", depth_by_node=depths)
    with pytest.raises(ValueError, match="plan"):
        order_route_jobs(JOBS, net_order="plan-crossings-asc", depth_by_node=depths)
    with pytest.raises(ValueError, match="net_order"):
        order_route_jobs(JOBS, net_order="random", depth_by_node=depths)
    assert NET_ORDERS == (
        "topological",
        "topological-span",
        "plan-crossings-asc",
        "plan-crossings-desc",
    )
