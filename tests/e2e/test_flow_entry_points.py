"""The entry points of Milestone 5, Slice 3: `FlowOptions`, `route_benchmark`,
`photonic_router.cli` and `python -m photonic_router`.

The flow itself is covered by tests/e2e/test_routing_flow_stats.py; these
tests cover the boundary the CLI and the option tree add on top of it.
"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import fields
from pathlib import Path

import pytest
from photonic_router import cli
from photonic_router.flow import route_benchmark
from photonic_router.flow_options import FlowOptions, LoadingOptions

REPO_ROOT = Path(__file__).resolve().parents[2]


def _flow_option_defaults() -> dict[str, object]:
    """Every `FlowOptions()` leaf field by name, across the groups."""
    options = FlowOptions()
    defaults: dict[str, object] = {}
    for group in fields(options):
        group_value = getattr(options, group.name)
        for leaf in fields(group_value):
            assert leaf.name not in defaults, f"{leaf.name} appears in two groups"
            defaults[leaf.name] = getattr(group_value, leaf.name)
    return defaults


def test_from_names_puts_each_name_into_the_group_that_declares_it():
    options = FlowOptions.from_names(
        benchmark_name="heater_s_mod", debug_timing=True, max_iterations=7, show_klayout=True
    )

    assert options.loading.benchmark_name == "heater_s_mod"
    assert options.debug.debug_timing is True
    assert options.optical.max_iterations == 7
    assert options.output.show_klayout is True
    # what is not named keeps the default
    assert options.path_length == FlowOptions().path_length


def test_from_names_without_names_is_the_default_flow():
    assert FlowOptions.from_names() == FlowOptions()


def test_from_names_refuses_a_name_no_group_declares():
    with pytest.raises(TypeError, match="not_an_option"):
        FlowOptions.from_names(not_an_option=1)


def test_every_leaf_name_is_declared_by_one_group():
    """`from_names` relies on it; `_flow_option_defaults` asserts it."""
    assert "benchmark_name" in _flow_option_defaults()


def test_flow_options_groups_are_frozen():
    """The option tree is immutable; a stage that needs a changed value builds
    a new tree (`dataclasses.replace`) instead of writing into this one."""
    options = FlowOptions()
    for group in fields(options):
        assert getattr(options, group.name).__dataclass_params__.frozen
    assert options.__dataclass_params__.frozen


def test_configuration_contribution2_expands_to_the_paper_flags():
    """`--configuration contribution2` is exactly the flags
    scripts/results/reproduce_paper.sh passes for a contribution 2 row."""
    assert cli.configuration_flags(["benes_8x8_flat", "--configuration", "contribution2"]) == [
        "--preplaced-crossing-grids",
        "true",
        "--verbose-routes",
    ]
    assert cli.configuration_flags(["benes_8x8_flat", "--configuration", "baseline"]) == [
        "--crossing-mode",
        "lidar-pure",
    ]
    assert cli.configuration_flags(["benes_8x8_flat", "--configuration", "contribution1"]) == [
        "--crossing-mode",
        "lidar-guided",
    ]
    assert cli.configuration_flags(["benes_8x8_flat"]) == []


def test_a_user_flag_after_the_configuration_wins():
    """The expansion is inserted before the caller's own flags, so an explicit
    flag overrides the named configuration."""
    user_argv = [
        "benes_8x8_flat",
        "--configuration",
        "contribution2",
        "--preplaced-crossing-grids",
        "false",
    ]
    argv = cli.configuration_flags(user_argv) + user_argv
    args = cli._build_arg_parser().parse_args(argv)
    assert args.preplaced_crossing_grids is False
    assert args.verbose_routes is True

    guided = ["benes_8x8_flat", "--configuration", "contribution1", "--crossing-mode", "lidar-pure"]
    args = cli._build_arg_parser().parse_args(cli.configuration_flags(guided) + guided)
    assert args.crossing_mode == "lidar-pure"


def test_module_without_a_subcommand_prints_the_usage():
    """`python -m photonic_router` prints the usage and exits with argparse's
    usage-error status (2)."""
    completed = subprocess.run(
        [sys.executable, "-m", "photonic_router"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 2
    assert "usage: python -m photonic_router" in completed.stderr
    assert "route" in completed.stderr


def test_route_benchmark_routes_heater_s_mod_from_named_options():
    """The explicit option tree and `from_names` are the same flow."""
    options = FlowOptions(loading=LoadingOptions(benchmark_name="heater_s_mod"))
    assert FlowOptions.from_names(benchmark_name="heater_s_mod") == options

    routed = route_benchmark(cli.build_config(None), options)

    assert routed.insts
