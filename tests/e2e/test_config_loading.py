"""`photonic_router.config_loading.build_config`: defaults, then the
benchmark's STABLE_ROUTING_ENV block, then the process environment; nothing is
written to os.environ (Milestone 1, Slice 3)."""

from __future__ import annotations

import os

from photonic_router import cli
from photonic_router.config import RoutingConfig
from photonic_router.config_loading import build_config

WEIGHT = "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT"
STUB = "PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES"


def test_defaults_when_nothing_is_set():
    assert build_config(None, {}) == RoutingConfig()


def test_stable_block_applies_when_the_environment_is_silent():
    config = build_config({WEIGHT: "0.05", STUB: "45"}, {})
    assert config.router.search.long_straight_congestion_weight == 0.05
    assert config.fanout.fanout_stub_bend_degrees == "45"


def test_environment_beats_the_stable_block():
    config = build_config({WEIGHT: "0.05"}, {WEIGHT: "1.0"})
    assert config.router.search.long_straight_congestion_weight == 1.0


def test_a_name_absent_from_both_keeps_its_default():
    config = build_config({WEIGHT: "0.05"}, {"UNRELATED": "x"})
    assert config.router.negotiation == RoutingConfig().router.negotiation


def test_main_hands_the_built_config_to_the_flow_without_touching_os_environ(monkeypatch):
    monkeypatch.delenv(WEIGHT, raising=False)
    monkeypatch.setattr(
        cli,
        "_benchmark_stable_defaults",
        lambda argv: (["--crossing-mode", "lidar-pure"], {WEIGHT: "0.05"}),
    )
    captured: dict[str, object] = {}

    def fake_route_benchmark(config, options):
        captured["benchmark"] = options.loading.benchmark_name
        captured["config"] = config
        captured["crossing_mode"] = options.optical.crossing_mode
        return object()

    # Milestone 5, Slice 3: the entry point the command line calls is
    # `photonic_router.flow.route_benchmark(config, options)`; `main` is
    # `photonic_router.cli.main` (and `routing_flow.main` is that same object).
    monkeypatch.setattr(cli, "route_benchmark", fake_route_benchmark)
    cli.main(["benes_4x4"])

    assert captured["benchmark"] == "benes_4x4"
    assert captured["crossing_mode"] == "lidar-pure"
    assert captured["config"].router.search.long_straight_congestion_weight == 0.05
    assert WEIGHT not in os.environ
