"""Net routing order rules (contribution 1, S3 -- see
`.agent/execplans/2026-09-04-crossing-guided-search.md`).

Sequential routing pays a crossing on the LATER net of a pair, and the
earlier net's greedy shortest path can seal a sibling's target pocket
(2026-09-04 braid finding). The order is therefore a knob:

- ``topological`` (default, the baseline): source-instance depth, then
  declaration order -- LiDAR's ``comp_dist`` primary key with declaration
  tiebreak.
- ``topological-span``: depth, then grid Manhattan span ascending (shortest
  first) -- the former ``PHOTONIC_ROUTER_LAYER_ORDER=span`` experiment
  (2026-08-27), now an option.
- ``plan-crossings-desc`` / ``plan-crossings-asc``: depth, then the number
  of planned crossings of the net (topology plan; needs ``lidar-guided``),
  most/fewest first, declaration tiebreak.
- ``plan-crossings-hybrid``: ``plan-crossings-desc`` inside depth layers of
  at most ``HYBRID_DESC_MAX_LAYER_NETS`` nets, ``topological`` in larger
  layers (the 32x32 cases).

Depth is derived from the batch's own net graph so every benchmark and
every crossing mode has it (benchmark ``NODE_DEPTHS`` metadata is optional
and the plan is withheld in lidar-pure).

The default depends on the configuration (``default_net_order``): the
baseline routes in ``topological`` order, contribution 1 (guided crossing
search) in ``plan-crossings-hybrid`` order; contribution 2
(pre-placed crossing grids, crossings off) routes in ``topological-span``
order, because its stubs must fan into the grid planarly and planar nesting
needs the widest span routed last (2026-08-27 finding: ``benes_16x16`` and
``benes_32x32`` grid mode fail under declaration order and complete under
span order). An explicit ``--net-order`` always wins.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from translation.route_rust_types import RouteJob

NET_ORDERS: tuple[str, ...] = (
    "topological",
    "topological-span",
    "plan-crossings-asc",
    "plan-crossings-desc",
    "plan-crossings-hybrid",
)

# plan-crossings-hybrid: most planned crossings first inside a depth layer
# with at most this many nets, declaration order in larger layers. The S3
# ladder (2026-09-04) showed desc winning on every layer of up to 16 nets
# (benes8 -95 %, mm16 -23 % expansions, the mm16 braid gone) and stalling
# benes_32x32's 32-net middle stage, where the widest-span nets committed
# first form a dense pack of steep diagonals the shorter nets cannot cross.
HYBRID_DESC_MAX_LAYER_NETS = 16


def default_net_order(*, preplaced_crossing_grids: bool, guided: bool = False) -> str:
    """The net order a configuration uses when none is given explicitly.

    Contribution 2 (pre-placed crossing grids) routes crossing-free stubs
    that must nest planarly around the grids: shortest span first.
    Contribution 1 (guided crossing search, ``guided=True``) routes the
    planned-crossing-rich nets first inside small layers
    (``plan-crossings-hybrid``; owner decision 2026-09-08: strictly better
    or identical on every benchmark). The baseline keeps the declaration
    order, which Benes crossing discovery under lidar-pure depends on
    (widest span first).
    """
    if preplaced_crossing_grids:
        return "topological-span"
    if guided:
        return "plan-crossings-hybrid"
    return "topological"


def normalize_net_order(net_order: object) -> str:
    value = str(net_order).strip().lower()
    if value not in NET_ORDERS:
        raise ValueError(
            "net_order must be one of "
            + ", ".join(repr(name) for name in NET_ORDERS)
            + f", got {net_order!r}"
        )
    return value


def depth_by_node_from_jobs(jobs: Sequence[RouteJob]) -> dict[str, int]:
    """Hops from a true source (an instance with no incoming net in `jobs`).

    A cycle member (impossible for a real photonic netlist's signal flow, a
    DAG) is treated as depth 0 rather than recursing forever.
    """
    incoming: dict[str, set[str]] = {}
    all_nodes: set[str] = set()
    for job in jobs:
        all_nodes.add(job.inst1)
        all_nodes.add(job.inst2)
        incoming.setdefault(job.inst2, set()).add(job.inst1)

    depth_by_node: dict[str, int] = {}

    def resolve_depth(node: str, visiting: set[str]) -> int:
        if node in depth_by_node:
            return depth_by_node[node]
        sources = incoming.get(node)
        if not sources or node in visiting:
            depth_by_node[node] = 0
            return 0
        visiting.add(node)
        depth = 1 + max(resolve_depth(source, visiting) for source in sources)
        visiting.discard(node)
        depth_by_node[node] = depth
        return depth

    for node in sorted(all_nodes):
        resolve_depth(node, set())
    return depth_by_node


def order_route_jobs(
    jobs: Sequence[RouteJob],
    *,
    net_order: str,
    depth_by_node: Mapping[str, int],
    span_by_net_id: Mapping[int, int] | None = None,
    planned_crossings_by_net_id: Mapping[int, int] | None = None,
) -> list[RouteJob]:
    """Sort `jobs` by the named rule; every rule starts with source depth and
    ends with declaration order (`route_index`)."""

    order = normalize_net_order(net_order)

    def depth(job: RouteJob) -> int:
        return int(depth_by_node.get(job.inst1, 0))

    if order == "topological":
        return sorted(jobs, key=lambda job: (depth(job), int(job.route_index)))
    if order == "topological-span":
        if span_by_net_id is None:
            raise ValueError("net_order 'topological-span' needs span_by_net_id")
        return sorted(
            jobs,
            key=lambda job: (
                depth(job),
                int(span_by_net_id[int(job.net_id)]),
                int(job.route_index),
            ),
        )
    if planned_crossings_by_net_id is None:
        raise ValueError(
            f"net_order {order!r} needs the topology plan (planned_crossings_by_net_id)"
        )
    sign = 1 if order == "plan-crossings-asc" else -1
    nets_per_depth: dict[int, int] = {}
    for job in jobs:
        nets_per_depth[depth(job)] = nets_per_depth.get(depth(job), 0) + 1

    def planned_key(job: RouteJob) -> int:
        large_layer = nets_per_depth[depth(job)] > HYBRID_DESC_MAX_LAYER_NETS
        if order == "plan-crossings-hybrid" and large_layer:
            return 0  # large layer: declaration order decides
        return sign * int(planned_crossings_by_net_id.get(int(job.net_id), 0))

    return sorted(
        jobs,
        key=lambda job: (
            depth(job),
            planned_key(job),
            int(job.route_index),
        ),
    )
