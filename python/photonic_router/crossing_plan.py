"""Router-facing crossing event plans derived from topology annotations."""

from __future__ import annotations

from dataclasses import dataclass

from photonic_router.path_length_graph import RoutedEdgeKey
from photonic_router.topology_analysis import TopologyAnalysisResult, TopologyEdgeRank


@dataclass(frozen=True)
class CrossingEvent:
    edge_a: RoutedEdgeKey
    edge_b: RoutedEdgeKey
    source_depth: int
    target_depth: int
    level: int
    order_index: int
    edge_a_source_rank: int
    edge_a_target_rank: int
    edge_b_source_rank: int
    edge_b_target_rank: int

    @property
    def edge_pair(self) -> frozenset[RoutedEdgeKey]:
        return frozenset((self.edge_a, self.edge_b))

    def __str__(self) -> str:
        return (
            f"level {self.level}: {self.edge_a.net_name}"
            f"[{self.edge_a_source_rank}->{self.edge_a_target_rank}] x "
            f"{self.edge_b.net_name}"
            f"[{self.edge_b_source_rank}->{self.edge_b_target_rank}]"
        )


@dataclass(frozen=True)
class CrossingStagePlan:
    source_depth: int
    target_depth: int
    initial_edge_order: tuple[RoutedEdgeKey, ...]
    final_edge_order: tuple[RoutedEdgeKey, ...]
    events: tuple[CrossingEvent, ...]

    def apply_events(self) -> tuple[RoutedEdgeKey, ...]:
        current = list(self.initial_edge_order)
        for event in self.events:
            index_a = current.index(event.edge_a)
            index_b = current.index(event.edge_b)
            if abs(index_a - index_b) != 1:
                raise ValueError(
                    "Crossing event edges are not adjacent in current order: "
                    f"{event.edge_a.net_name} x {event.edge_b.net_name}"
                )
            left = min(index_a, index_b)
            current[left], current[left + 1] = current[left + 1], current[left]
        return tuple(current)

    def validate(self) -> None:
        realized = self.apply_events()
        if realized != self.final_edge_order:
            realized_names = [edge.net_name for edge in realized]
            expected_names = [edge.net_name for edge in self.final_edge_order]
            raise ValueError(
                "Crossing stage event order does not realize target edge order: "
                f"{realized_names} != {expected_names}"
            )

    def to_text(self, *, include_orders: bool = True) -> str:
        lines = [
            (
                f"stage {self.source_depth}->{self.target_depth}: "
                f"{len(self.events)} crossing(s)"
            )
        ]
        if include_orders:
            initial = ", ".join(edge.net_name for edge in self.initial_edge_order)
            final = ", ".join(edge.net_name for edge in self.final_edge_order)
            lines.append(f"  source order: {initial}")
            lines.append(f"  target order: {final}")
        for event in self.events:
            lines.append(f"  - {event}")
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.to_text()


@dataclass(frozen=True)
class CrossingPlan:
    stages: dict[tuple[int, int], CrossingStagePlan]
    events_by_edge: dict[RoutedEdgeKey, tuple[CrossingEvent, ...]]
    events_by_pair: dict[frozenset[RoutedEdgeKey], CrossingEvent]

    @property
    def events(self) -> tuple[CrossingEvent, ...]:
        return tuple(
            event
            for stage_key in sorted(self.stages)
            for event in self.stages[stage_key].events
        )

    def events_for_edge(self, edge_key: RoutedEdgeKey) -> tuple[CrossingEvent, ...]:
        return self.events_by_edge.get(edge_key, ())

    def event_for_pair(
        self,
        edge_a: RoutedEdgeKey,
        edge_b: RoutedEdgeKey,
    ) -> CrossingEvent | None:
        return self.events_by_pair.get(frozenset((edge_a, edge_b)))

    def to_text(
        self,
        *,
        include_empty_stages: bool = False,
        include_orders: bool = True,
    ) -> str:
        lines = [
            (
                "CrossingPlan: "
                f"{len(self.events)} crossing(s), {len(self.stages)} stage(s)"
            )
        ]
        for stage_key in sorted(self.stages):
            stage = self.stages[stage_key]
            if not include_empty_stages and not stage.events:
                continue
            lines.append(stage.to_text(include_orders=include_orders))
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.to_text()


def build_crossing_plan(topology: TopologyAnalysisResult) -> CrossingPlan:
    stages: dict[tuple[int, int], CrossingStagePlan] = {}
    events_by_edge_mut: dict[RoutedEdgeKey, list[CrossingEvent]] = {}
    events_by_pair: dict[frozenset[RoutedEdgeKey], CrossingEvent] = {}

    for stage_key, edge_ranks in _edge_ranks_by_stage(topology).items():
        stage_plan = _build_stage_plan(stage_key, edge_ranks)
        stage_plan.validate()
        stages[stage_key] = stage_plan
        for event in stage_plan.events:
            events_by_pair[event.edge_pair] = event
            events_by_edge_mut.setdefault(event.edge_a, []).append(event)
            events_by_edge_mut.setdefault(event.edge_b, []).append(event)

    events_by_edge = {
        edge_key: tuple(sorted(events, key=lambda event: (event.source_depth, event.level, event.order_index)))
        for edge_key, events in events_by_edge_mut.items()
    }
    return CrossingPlan(
        stages=stages,
        events_by_edge=events_by_edge,
        events_by_pair=events_by_pair,
    )


def _edge_ranks_by_stage(
    topology: TopologyAnalysisResult,
) -> dict[tuple[int, int], list[TopologyEdgeRank]]:
    by_stage: dict[tuple[int, int], list[TopologyEdgeRank]] = {}
    for edge_rank in topology.edge_ranks.values():
        by_stage.setdefault((edge_rank.source_depth, edge_rank.target_depth), []).append(
            edge_rank
        )
    return by_stage


def _build_stage_plan(
    stage_key: tuple[int, int],
    edge_ranks: list[TopologyEdgeRank],
) -> CrossingStagePlan:
    source_depth, target_depth = stage_key
    ordered_by_source = sorted(
        edge_ranks,
        key=lambda edge_rank: (edge_rank.source_rank, edge_rank.target_rank, edge_rank.edge_key.net_name),
    )
    initial_order = tuple(edge_rank.edge_key for edge_rank in ordered_by_source)
    final_order = tuple(
        edge_rank.edge_key
        for edge_rank in sorted(
            edge_ranks,
            key=lambda edge_rank: (edge_rank.target_rank, edge_rank.source_rank, edge_rank.edge_key.net_name),
        )
    )
    current = list(initial_order)
    edge_rank_by_key = {edge_rank.edge_key: edge_rank for edge_rank in edge_ranks}
    event_levels_by_edge: dict[RoutedEdgeKey, int] = {}
    events: list[CrossingEvent] = []
    order_index = 0

    while tuple(current) != final_order:
        progress = False
        for index in range(len(current) - 1):
            left = current[index]
            right = current[index + 1]
            left_rank = edge_rank_by_key[left]
            right_rank = edge_rank_by_key[right]
            if left_rank.target_rank <= right_rank.target_rank:
                continue

            level = max(
                event_levels_by_edge.get(left, 0),
                event_levels_by_edge.get(right, 0),
            )
            event = CrossingEvent(
                edge_a=left,
                edge_b=right,
                source_depth=source_depth,
                target_depth=target_depth,
                level=level,
                order_index=order_index,
                edge_a_source_rank=left_rank.source_rank,
                edge_a_target_rank=left_rank.target_rank,
                edge_b_source_rank=right_rank.source_rank,
                edge_b_target_rank=right_rank.target_rank,
            )
            events.append(event)
            event_levels_by_edge[left] = level + 1
            event_levels_by_edge[right] = level + 1
            current[index], current[index + 1] = current[index + 1], current[index]
            order_index += 1
            progress = True
            break

        if not progress:
            current_names = [edge.net_name for edge in current]
            target_names = [edge.net_name for edge in final_order]
            raise ValueError(
                "Unable to produce crossing event sequence for stage "
                f"{stage_key}: {current_names} -> {target_names}"
            )

    ordered_events = tuple(sorted(events, key=lambda event: (event.level, event.order_index)))
    return CrossingStagePlan(
        source_depth=source_depth,
        target_depth=target_depth,
        initial_edge_order=initial_order,
        final_edge_order=final_order,
        events=ordered_events,
    )
