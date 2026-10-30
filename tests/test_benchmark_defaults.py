"""`benchmarks/<name>.toml`: the benchmark defaults and their loader (phase 8 slice C)."""

from __future__ import annotations

from pathlib import Path

import pytest
from photonic_router.cli import load_benchmark_defaults

ROOT = Path(__file__).resolve().parents[1]


def test_benes_32x32_flat_defaults_are_the_papers() -> None:
    flags, env = load_benchmark_defaults("benes_32x32_flat")
    assert flags == [
        "--crossings",
        "true",
        "--crossing-mode",
        "lidar-pure",
        "--fanout-access-mode",
        "static-stubs",
        "--foreign-port-keepout-cells",
        "0",
        "--proactive-congestion-weight",
        "4.0",
        "--proactive-congestion-radius-cells",
        "3",
        "--max-iterations",
        "20000000",
    ]
    assert env == {"PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "1.0"}


def test_benchmark_without_a_file_has_no_defaults() -> None:
    assert load_benchmark_defaults("benes_4x4_flat") == ([], {})
    assert load_benchmark_defaults("no_such_benchmark") == ([], {})
    assert load_benchmark_defaults("../pyproject") == ([], {})


def test_flag_values_keep_their_spelling() -> None:
    flags, _ = load_benchmark_defaults("heater_s_mod")
    assert flags[:6] == [
        "--allow-45-degree-turns",
        "false",
        "--bend-radius-um",
        "10",
        "--path-length-matching",
        "true",
    ]
    assert flags[-4:] == ["--waveguide-clearance-um", "3.0", "--crossings", "false"]


@pytest.mark.parametrize(
    "toml_path",
    sorted(p for p in (ROOT / "benchmarks").glob("*.toml")),
    ids=lambda p: p.stem,
)
def test_every_benchmark_toml_loads_through_the_cli_helper(toml_path: Path) -> None:
    assert toml_path.with_suffix(".py").is_file(), "a TOML without its benchmark module"
    flags, env = load_benchmark_defaults(toml_path.stem)
    assert len(flags) % 2 == 0
    assert all(flags[i].startswith("--") for i in range(0, len(flags), 2))
    assert all(name.startswith("PHOTONIC_ROUTER_") for name in env)
    assert flags or env
