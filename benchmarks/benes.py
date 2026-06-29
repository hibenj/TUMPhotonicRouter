"""Schematic-first Benes network benchmark generation.

The generated schematic intentionally leaves all inter-stage waveguides unrouted.
Topology metadata records stage depth, lane ranks, and unavoidable crossings so
crossing-aware routing can use the benchmark as a deterministic oracle.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import log2
from typing import Any

import gdsfactory as gf
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

SWITCH_COMPONENT = "benes_mmi_heater_switch"
IO_COMPONENT = "grating_coupler_te"

SWITCH_TOP_INPUT_PORT = "o2"
SWITCH_BOTTOM_INPUT_PORT = "o1"
SWITCH_TOP_OUTPUT_PORT = "o3"
SWITCH_BOTTOM_OUTPUT_PORT = "o4"
IO_PORT = "o1"


@dataclass(frozen=True)
class BenesLayerEdge:
    """One logical edge between adjacent Benes switch stages."""

    net_name: str
    source_instance: str
    source_port: str
    target_instance: str
    target_port: str
    source_rank: int
    target_rank: int
    source_depth: int
    target_depth: int


@dataclass(frozen=True)
class BenesCrossing:
    """A pair of inter-stage edges whose rank order is inverted."""

    stage: int
    edge_a: str
    edge_b: str
    edge_a_source_rank: int
    edge_a_target_rank: int
    edge_b_source_rank: int
    edge_b_target_rank: int


def switch_name(stage: int, index: int) -> str:
    return f"sw_s{stage}_{index}"


def input_name(index: int) -> str:
    return f"in_{index}"


def output_name(index: int) -> str:
    return f"out_{index}"


def validate_benes_size(size: int) -> None:
    if size < 4 or size & (size - 1):
        raise ValueError(f"Benes size must be a power of two >= 4. Got {size}.")


@gf.cell
def benes_mmi_heater_switch(
    heater_separation_um: float = 35.0,
    heater_input_x_um: float = 70.0,
    second_mmi_x_um: float = 430.0,
) -> gf.Component:
    """LiDAR-style 2x2 Benes switch cell with two MMIs and two heater arms."""
    get_generic_pdk().activate()
    c = gf.Component()
    mmi = gf.get_component("mmi2x2")
    heater = gf.get_component("straight_heater_metal")

    mmi_in = c.add_ref(mmi, name="mmi_in")
    mmi_out = c.add_ref(mmi, name="mmi_out")
    heater_top = c.add_ref(heater, name="heater_top")
    heater_bottom = c.add_ref(heater, name="heater_bottom")

    mmi_out.dmovex(second_mmi_x_um)
    heater_top.dmove((heater_input_x_um, heater_separation_um))
    heater_bottom.dmove((heater_input_x_um, -heater_separation_um))

    gf.routing.route_single(
        c,
        mmi_in.ports["o3"],
        heater_top.ports["o1"],
        port_type="optical",
        cross_section="strip",
    )
    gf.routing.route_single(
        c,
        heater_top.ports["o2"],
        mmi_out.ports["o2"],
        port_type="optical",
        cross_section="strip",
    )
    gf.routing.route_single(
        c,
        mmi_in.ports["o4"],
        heater_bottom.ports["o1"],
        port_type="optical",
        cross_section="strip",
    )
    gf.routing.route_single(
        c,
        heater_bottom.ports["o2"],
        mmi_out.ports["o1"],
        port_type="optical",
        cross_section="strip",
    )

    c.add_port("o1", port=mmi_in.ports["o1"])
    c.add_port("o2", port=mmi_in.ports["o2"])
    c.add_port("o3", port=mmi_out.ports["o3"])
    c.add_port("o4", port=mmi_out.ports["o4"])
    c.info["optical_length_um"] = 320.0
    return c


def register_benes_cells() -> None:
    pdk = get_generic_pdk()
    pdk.activate()
    if SWITCH_COMPONENT not in pdk.cells:
        pdk.register_cells(benes_mmi_heater_switch=benes_mmi_heater_switch)


def benes_connection_pattern(size: int) -> tuple[tuple[tuple[int, int], ...], ...]:
    """Return destination switch pairs for every inter-stage transition.

    The result is indexed as connections[stage][source_switch] = (dst_a, dst_b).
    For size 4 this yields the canonical 3-stage Benes with two shuffle layers.
    """
    validate_benes_size(size)
    address_bits = int(log2(size))
    stage_count = 2 * address_bits - 1
    switches_per_stage = size // 2
    connections: list[list[tuple[int, int]]] = [
        [] for _ in range(stage_count - 1)
    ]

    forward_stage_count = (stage_count - 1) // 2
    for stage in range(forward_stage_count):
        step = switches_per_stage >> stage
        half = step >> 1
        for group in range(switches_per_stage // step):
            for offset in range(half):
                a = group * step + offset
                b = a + half
                connections[stage].append((a, b))
                connections[stage].append((a, b))

    def build_inverse_stage(
        forward: list[tuple[int, int]],
        backward: list[tuple[int, int]],
    ) -> None:
        backward.clear()
        positions: list[list[int]] = [[] for _ in range(len(forward))]
        for source_index, (dst_a, dst_b) in enumerate(forward):
            positions[dst_a].append(source_index)
            positions[dst_b].append(source_index)
        for source_positions in positions:
            if len(source_positions) >= 2:
                ordered = sorted(source_positions)
                backward.append((ordered[0], ordered[1]))

    for stage in range(forward_stage_count):
        mirror_stage = stage_count - 2 - stage
        build_inverse_stage(connections[stage], connections[mirror_stage])

    return tuple(tuple(stage) for stage in connections)


def _iter_interstage_edges(size: int) -> tuple[BenesLayerEdge, ...]:
    connections = benes_connection_pattern(size)
    edges: list[BenesLayerEdge] = []

    for stage, stage_connections in enumerate(connections):
        incoming_count_by_switch = {index: 0 for index in range(size // 2)}
        for source_switch, destinations in enumerate(stage_connections):
            source_instance = switch_name(stage, source_switch)
            for output_index, target_switch in enumerate(destinations):
                target_input_index = incoming_count_by_switch[target_switch]
                incoming_count_by_switch[target_switch] += 1

                source_port = (
                    SWITCH_TOP_OUTPUT_PORT
                    if output_index == 0
                    else SWITCH_BOTTOM_OUTPUT_PORT
                )
                target_port = (
                    SWITCH_TOP_INPUT_PORT
                    if target_input_index == 0
                    else SWITCH_BOTTOM_INPUT_PORT
                )
                source_rank = 2 * source_switch + output_index
                target_rank = 2 * target_switch + target_input_index
                target_instance = switch_name(stage + 1, target_switch)
                edges.append(
                    BenesLayerEdge(
                        net_name=(
                            f"n_s{stage}_{source_switch}_o{output_index}_"
                            f"to_s{stage + 1}_{target_switch}_i{target_input_index}"
                        ),
                        source_instance=source_instance,
                        source_port=source_port,
                        target_instance=target_instance,
                        target_port=target_port,
                        source_rank=source_rank,
                        target_rank=target_rank,
                        source_depth=stage + 1,
                        target_depth=stage + 2,
                    )
                )

    return tuple(edges)


def find_rank_crossings(edges: tuple[BenesLayerEdge, ...]) -> tuple[BenesCrossing, ...]:
    """Find pairwise rank inversions between edges in the same stage transition."""
    crossings: list[BenesCrossing] = []
    by_stage: dict[int, list[BenesLayerEdge]] = {}
    for edge in edges:
        by_stage.setdefault(edge.source_depth - 1, []).append(edge)

    for stage, stage_edges in by_stage.items():
        ordered = sorted(stage_edges, key=lambda edge: edge.source_rank)
        for left_index, edge_a in enumerate(ordered):
            for edge_b in ordered[left_index + 1 :]:
                source_delta = edge_a.source_rank - edge_b.source_rank
                target_delta = edge_a.target_rank - edge_b.target_rank
                if source_delta * target_delta >= 0:
                    continue
                crossings.append(
                    BenesCrossing(
                        stage=stage,
                        edge_a=edge_a.net_name,
                        edge_b=edge_b.net_name,
                        edge_a_source_rank=edge_a.source_rank,
                        edge_a_target_rank=edge_a.target_rank,
                        edge_b_source_rank=edge_b.source_rank,
                        edge_b_target_rank=edge_b.target_rank,
                    )
                )

    return tuple(crossings)


def benes_topology_metadata(size: int) -> dict[str, Any]:
    validate_benes_size(size)
    address_bits = int(log2(size))
    stage_count = 2 * address_bits - 1
    switches_per_stage = size // 2
    interstage_edges = _iter_interstage_edges(size)
    crossings = find_rank_crossings(interstage_edges)

    node_depths: dict[str, int] = {}
    node_ranks: dict[str, int] = {}
    for index in range(size):
        node_depths[input_name(index)] = 0
        node_ranks[input_name(index)] = index
        node_depths[output_name(index)] = stage_count + 1
        node_ranks[output_name(index)] = index
    for stage in range(stage_count):
        for switch_index in range(switches_per_stage):
            name = switch_name(stage, switch_index)
            node_depths[name] = stage + 1
            node_ranks[name] = switch_index

    edge_ranks = {
        edge.net_name: {
            "source_rank": edge.source_rank,
            "target_rank": edge.target_rank,
            "source_depth": edge.source_depth,
            "target_depth": edge.target_depth,
        }
        for edge in interstage_edges
    }
    crossings_by_stage: dict[int, list[dict[str, Any]]] = {}
    for crossing in crossings:
        crossings_by_stage.setdefault(crossing.stage, []).append(
            {
                "edge_a": crossing.edge_a,
                "edge_b": crossing.edge_b,
                "edge_a_source_rank": crossing.edge_a_source_rank,
                "edge_a_target_rank": crossing.edge_a_target_rank,
                "edge_b_source_rank": crossing.edge_b_source_rank,
                "edge_b_target_rank": crossing.edge_b_target_rank,
            }
        )

    return {
        "network": "benes",
        "size": size,
        "address_bits": address_bits,
        "stage_count": stage_count,
        "switches_per_stage": switches_per_stage,
        "connections": benes_connection_pattern(size),
        "node_depths": node_depths,
        "node_ranks": node_ranks,
        "interstage_edges": tuple(edge.__dict__ for edge in interstage_edges),
        "edge_ranks": edge_ranks,
        "crossings": tuple(crossing.__dict__ for crossing in crossings),
        "crossings_by_stage": crossings_by_stage,
    }


def benes_node_types(size: int) -> dict[str, str]:
    metadata = benes_topology_metadata(size)
    stage_count = int(metadata["stage_count"])
    switches_per_stage = int(metadata["switches_per_stage"])
    node_types = {
        input_name(index): "input"
        for index in range(size)
    } | {
        output_name(index): "output"
        for index in range(size)
    }
    for stage in range(stage_count):
        for switch_index in range(switches_per_stage):
            node_types[switch_name(stage, switch_index)] = "gate"
    return node_types


def benes_internal_delays_um(size: int) -> dict[str, float | str]:
    metadata = benes_topology_metadata(size)
    stage_count = int(metadata["stage_count"])
    switches_per_stage = int(metadata["switches_per_stage"])
    internal_delays: dict[str, float | str] = {
        input_name(index): 0.0
        for index in range(size)
    } | {
        output_name(index): 0.0
        for index in range(size)
    }
    for stage in range(stage_count):
        for switch_index in range(switches_per_stage):
            internal_delays[switch_name(stage, switch_index)] = "auto"
    return internal_delays


def build_benes_schematic(
    size: int,
    *,
    stage_pitch_um: float = 1000.0,
    switch_pitch_um: float = 220.0,
    io_dx_um: float = 220.0,
    lane_separation_um: float = 90.0,
) -> Schematic:
    """Build an unrouted size x size Benes schematic."""
    validate_benes_size(size)
    register_benes_cells()

    metadata = benes_topology_metadata(size)
    stage_count = int(metadata["stage_count"])
    switches_per_stage = int(metadata["switches_per_stage"])
    schematic = Schematic()
    switch_instance = Instance(component=SWITCH_COMPONENT)
    io_instance = Instance(component=IO_COMPONENT)

    def switch_y(index: int) -> float:
        return (switches_per_stage - 1 - index) * switch_pitch_um

    def lane_y(index: int) -> float:
        switch_index = index // 2
        is_top_lane = index % 2 == 0
        lane_offset = 0.5 * lane_separation_um
        return switch_y(switch_index) + (lane_offset if is_top_lane else -lane_offset)

    for index in range(size):
        schematic.add_instance(
            input_name(index),
            io_instance,
            Placement(x=0.0, y=lane_y(index), mirror=True),
        )
        schematic.add_instance(
            output_name(index),
            io_instance,
            Placement(
                x=io_dx_um + stage_pitch_um * stage_count,
                y=lane_y(index),
            ),
        )

    for stage in range(stage_count):
        for switch_index in range(switches_per_stage):
            schematic.add_instance(
                switch_name(stage, switch_index),
                switch_instance,
                Placement(
                    x=io_dx_um + stage * stage_pitch_um,
                    y=switch_y(switch_index),
                ),
            )

    for index in range(size):
        switch_index = index // 2
        target_port = (
            SWITCH_TOP_INPUT_PORT if index % 2 == 0 else SWITCH_BOTTOM_INPUT_PORT
        )
        schematic.add_net(
            Net(
                p1=f"{input_name(index)},{IO_PORT}",
                p2=f"{switch_name(0, switch_index)},{target_port}",
                name=f"n_{input_name(index)}_to_s0_{switch_index}",
            )
        )

    for edge in _iter_interstage_edges(size):
        schematic.add_net(
            Net(
                p1=f"{edge.source_instance},{edge.source_port}",
                p2=f"{edge.target_instance},{edge.target_port}",
                name=edge.net_name,
            )
        )

    last_stage = stage_count - 1
    for index in range(size):
        switch_index = index // 2
        source_port = (
            SWITCH_TOP_OUTPUT_PORT if index % 2 == 0 else SWITCH_BOTTOM_OUTPUT_PORT
        )
        schematic.add_net(
            Net(
                p1=f"{switch_name(last_stage, switch_index)},{source_port}",
                p2=f"{output_name(index)},{IO_PORT}",
                name=f"n_s{last_stage}_{switch_index}_to_{output_name(index)}",
            )
        )

    return schematic
