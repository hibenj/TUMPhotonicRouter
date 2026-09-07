"""Synthetic permutation layer for generality tests of contribution 2.

Two columns: ``m`` source bars on the left, each with ``k`` east-facing
ports at pitch ``sp`` (a multiport-MMI-like dense group when ``sp`` is
small), and ``m * k`` single-port target bars on the right at pitch ``tp``,
``band`` um of free space between the columns, and a permutation between
source ports and targets. No metadata: the topology analysis derives the
layers and lane order from the placements, exactly as for the real
benchmarks.

Configured through ``PHOTONIC_ROUTER_SYNTH`` (comma-separated ``key=value``):
``k`` ports per source bar (default 4), ``m`` source bars (2), ``sp`` source
port pitch um (5), ``tp`` target pitch um (100), ``band`` um (110),
``mode`` = ``random`` | ``swap`` | ``reverse`` | ``identity`` (random),
``seed`` (1).
"""

from __future__ import annotations

import os
import random

from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

# The multiportmmi stable block: static stubs for dense port groups and the
# lidar-pure crossing search for the baseline control; grid mode drops the
# crossing flags itself.
STABLE_ROUTING_ENV: dict[str, str] = {
    "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "0.05",
    "PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES": "90",
}

STABLE_ROUTING_FLAGS: tuple[str, ...] = (
    "--crossings",
    "true",
    "--crossing-mode",
    "lidar-pure",
    "--fanout-access-mode",
    "static-stubs",
    # --routing-window-scale 0.35 removed 2026-08-31: the CLI default
    # 0.05 routes identically clean and 15-30% faster (heuristics-audit
    # experiments, .agent/execplans/2026-08-31-engine-performance-baseline.md).
    "--foreign-port-keepout-cells",
    "0",
    "--proactive-congestion-weight",
    "4.0",
    "--proactive-congestion-radius-cells",
    "3",
)

BAR_WIDTH_UM = 20.0
BAR_MARGIN_UM = 5.0


def synth_config() -> dict[str, object]:
    raw = os.environ.get("PHOTONIC_ROUTER_SYNTH", "")
    cfg: dict[str, object] = {
        "k": 4,
        "m": 2,
        "sp": 5.0,
        "tp": 100.0,
        "band": 110.0,
        "mode": "random",
        "seed": 1,
    }
    for item in raw.split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key in ("k", "m", "seed"):
            cfg[key] = int(value)
        elif key in ("sp", "tp", "band"):
            cfg[key] = float(value)
        elif key == "mode":
            cfg[key] = value
    return cfg


def _bar(name: str, port_count: int, pitch_um: float, orientation: float) -> Component:
    component = Component(name=name)
    height = max(port_count - 1, 0) * pitch_um + 2 * BAR_MARGIN_UM
    component.add_polygon(
        [
            (0, -height / 2),
            (BAR_WIDTH_UM, -height / 2),
            (BAR_WIDTH_UM, height / 2),
            (0, height / 2),
        ],
        layer=(1, 0),
    )
    for index in range(port_count):
        y = (index - (port_count - 1) / 2.0) * pitch_um
        x = BAR_WIDTH_UM if orientation == 0 else 0.0
        component.add_port(
            name=f"o{index + 1}", center=(x, y), width=0.5, orientation=orientation, layer=(1, 0)
        )
    return component


def register_synth_cells(k: int, sp: float) -> tuple[str, str]:
    pdk = get_generic_pdk()
    pdk.activate()
    source_name = f"synth_source_k{k}_p{sp:g}".replace(".", "_")
    target_name = "synth_target"
    if source_name not in pdk.cells:
        source = _bar(source_name, k, sp, 0.0)
        pdk.register_cells(**{source_name: (lambda c=source: c)})
    if target_name not in pdk.cells:
        target = _bar(target_name, 1, 0.0, 180.0)
        pdk.register_cells(**{target_name: (lambda c=target: c)})
    return source_name, target_name


def permutation(n: int, mode: str, seed: int) -> list[int]:
    order = list(range(n))
    if mode == "random":
        random.Random(seed).shuffle(order)
    elif mode == "reverse":
        order.reverse()
    elif mode == "swap":
        for i in range(0, n - 1, 2):
            order[i], order[i + 1] = order[i + 1], order[i]
    elif mode != "identity":
        raise ValueError(f"unknown synth mode {mode!r}")
    return order


def build_schematic() -> Schematic:
    cfg = synth_config()
    k = int(cfg["k"])
    m = int(cfg["m"])
    sp = float(cfg["sp"])
    tp = float(cfg["tp"])
    band = float(cfg["band"])
    n = k * m
    source_name, target_name = register_synth_cells(k, sp)
    schematic = Schematic()
    # targets top to bottom at pitch tp, centered on 0
    target_ys = [((n - 1) / 2.0 - i) * tp for i in range(n)]
    target_x = BAR_WIDTH_UM + band
    for i, y in enumerate(target_ys):
        schematic.add_instance(
            f"tgt_{i}", Instance(component=target_name), Placement(x=target_x, y=y)
        )
    # source bars: centered on the rows of their k targets' block (like MMI groups)
    perm = permutation(n, str(cfg["mode"]), int(cfg["seed"]))
    lanes: list[tuple[str, str, str]] = []
    for b in range(m):
        block = target_ys[b * k : (b + 1) * k]
        center = sum(block) / len(block)
        schematic.add_instance(
            f"src_{b}", Instance(component=source_name), Placement(x=0.0, y=center)
        )
        for j in range(k):
            lane = b * k + j
            # port o1 is the lowest; lane index counts top to bottom
            port = f"o{k - j}"
            lanes.append((f"src_{b},{port}", f"tgt_{perm[lane]},o1", f"n_{lane}"))
    for p1, p2, name in lanes:
        schematic.add_net(Net(p1=p1, p2=p2, name=name))
    return schematic
