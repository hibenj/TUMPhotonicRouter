"""Run the flow from a test with a few named options."""

from __future__ import annotations

from typing import Any

from gdsfactory.component import Component
from photonic_router.config import RoutingConfig
from photonic_router.flow import route_benchmark
from photonic_router.flow_options import FlowOptions


def run_flow(
    benchmark_name: str, *, config: RoutingConfig | None = None, **names: Any
) -> Component:
    """Route a benchmark with the default flow and the named options.

    `names` are leaf field names of `FlowOptions` (`FlowOptions.from_names`).
    Without a `config` the configuration comes from the environment, as on
    the command line. A test that hands in a `stats` record wants it filled,
    so that implies `collect_route_stats`.
    """
    if names.get("stats") is not None:
        names["collect_route_stats"] = True
    return route_benchmark(
        config if config is not None else RoutingConfig.from_environment(),
        FlowOptions.from_names(benchmark_name=benchmark_name, **names),
    )
