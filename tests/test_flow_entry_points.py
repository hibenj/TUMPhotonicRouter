"""The entry points of Milestone 5, Slice 3: `FlowOptions`, `route_benchmark`,
`photonic_router.cli` and `python -m photonic_router`.

The flow itself is covered by tests/test_routing_flow_stats.py through
`run_routing_flow`; these tests cover the boundary the CLI and the option tree
add on top of it.
"""

from __future__ import annotations

import inspect
import subprocess
import sys
from dataclasses import fields
from pathlib import Path

from photonic_router import cli
from photonic_router.flow import route_benchmark, run_routing_flow
from photonic_router.flow_options import FlowOptions, LoadingOptions

REPO_ROOT = Path(__file__).resolve().parents[1]

# `run_routing_flow`'s two parameters that are not FlowOptions fields: the
# benchmark is `loading.benchmark_name` (positional, so it has no signature
# default to compare against) and the configuration is `route_benchmark`'s own
# first argument.
_NOT_FLOW_OPTIONS = frozenset({"benchmark_name", "config"})


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


def test_flow_options_defaults_equal_the_run_routing_flow_signature():
    """`FlowOptions()` is today's default flow: every default is the one
    `run_routing_flow` declares, so the wrapper and the option tree cannot
    drift apart."""
    signature = inspect.signature(run_routing_flow)
    defaults = _flow_option_defaults()
    for name, parameter in signature.parameters.items():
        if name in _NOT_FLOW_OPTIONS:
            continue
        assert name in defaults, f"{name} is a run_routing_flow keyword with no FlowOptions field"
        assert defaults[name] == parameter.default, f"{name} default differs"


def test_every_flow_option_field_is_a_run_routing_flow_keyword():
    """The other direction: no field was invented that the wrapper cannot set."""
    keywords = set(inspect.signature(run_routing_flow).parameters) - _NOT_FLOW_OPTIONS
    fields_by_name = set(_flow_option_defaults()) - {"benchmark_name"}
    assert fields_by_name == keywords


def test_flow_options_groups_are_frozen():
    """The option tree is immutable; a stage that needs a changed value builds
    a new tree (`dataclasses.replace`) instead of writing into this one."""
    options = FlowOptions()
    for group in fields(options):
        assert getattr(options, group.name).__dataclass_params__.frozen
    assert options.__dataclass_params__.frozen


def test_configuration_contribution2_expands_to_the_paper_flags():
    """`--configuration contribution2` is exactly the flags
    scripts/results/reproduce_date2027.sh passes for a contribution 2 row."""
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


def test_route_benchmark_matches_run_routing_flow_on_heater_s_mod():
    """`run_routing_flow` is a wrapper over `route_benchmark`, so the two route
    the same benchmark to the same layout."""
    options = FlowOptions(loading=LoadingOptions(benchmark_name="heater_s_mod"))
    through_entry_point = route_benchmark(cli.build_config(None), options)
    through_wrapper = run_routing_flow("heater_s_mod")
    assert len(through_entry_point.insts) == len(through_wrapper.insts)
    assert through_entry_point.name == through_wrapper.name
