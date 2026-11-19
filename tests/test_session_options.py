"""The options of a routing session (`photonic_router.routing.options`).

What a caller hands a session is one `SessionOptions`; `SessionSettings`
validates it, and the flow builds it from its `FlowOptions` and from what the
stages before the optical one decided. These tests pin that contract: the
defaults, what is refused, and that the flow's builder puts every value where
the session reads it.
"""

from __future__ import annotations

from dataclasses import fields, replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from photonic_router.config import RoutingConfig
from photonic_router.flow_optical import OpticalStageInputs, build_optical_routing_stage_config
from photonic_router.flow_options import (
    DebugArtifactOptions,
    FlowOptions,
    LayoutOptions,
    LoadingOptions,
    OpticalStageOptions,
    PathLengthMatchingOptions,
)
from photonic_router.routing.options import (
    CrossingOptions,
    RealizationOptions,
    SearchOptions,
    SessionOptions,
)
from photonic_router.routing.settings import SessionSettings
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig

from tests.fixtures.sessions import session_options


def _settings(options: SessionOptions) -> SessionSettings:
    return SessionSettings.from_options(SimpleNamespace(), SimpleNamespace(), options)


def test_default_options_are_the_default_session() -> None:
    settings = _settings(SessionOptions(config=RoutingConfig()))

    assert settings.max_iterations == 500_000
    assert settings.net_order == "topological"
    assert settings.enable_crossings is False
    assert settings.crossing_mode == "lidar-pure"
    assert settings.placement_analysis is None
    assert settings.fanout_access_mode_normalized == "legacy-runway"
    assert settings.route_width_um == 0.5
    assert settings.defer_realization is False
    assert settings.enable_checked_endpoint_correction is True
    assert settings.collect_pipeline_timing is False


def test_no_option_group_exceeds_the_record_size_the_standards_question() -> None:
    groups = [
        SessionOptions,
        *(type(getattr(SessionOptions(), f.name)) for f in fields(SessionOptions)),
    ]

    sizes = {
        group.__name__: len(fields(group))
        for group in groups
        if hasattr(group, "__dataclass_fields__")
    }

    assert max(sizes.values()) <= 15, sizes


@pytest.mark.parametrize(
    ("options", "message"),
    [
        (SessionOptions(realization=RealizationOptions(route_width_um=0.0)), "route_width_um"),
        (SessionOptions(search=SearchOptions(max_iterations=0)), "max_iterations"),
        (SessionOptions(crossings=CrossingOptions(crossing_loss=-1.0)), "crossing_loss"),
        (
            SessionOptions(crossings=CrossingOptions(crossing_half_size_cells=-1)),
            "crossing_half_size_cells",
        ),
    ],
)
def test_an_option_outside_its_range_is_refused(options: SessionOptions, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _settings(replace(options, config=RoutingConfig()))


def test_the_test_helper_refuses_a_name_that_is_not_an_option() -> None:
    assert session_options(max_iterations=7, debug_prefix="x").search.max_iterations == 7
    with pytest.raises(TypeError, match="not an option"):
        session_options(max_iteratoins=7)


def test_the_flow_builder_puts_each_value_where_the_session_reads_it() -> None:
    config = RoutingConfig()
    obstacle_config = StaticObstacleMapConfig(grid_size_um=2.0)
    crossings = CrossingOptions(enable_crossings=True, crossing_mode="lidar-guided")
    flow_options = FlowOptions(
        loading=LoadingOptions(benchmark_name="Benes_4x4"),
        layout=LayoutOptions(allow_45_degree_turns=False, bend_radius_um=7.0),
        optical=OpticalStageOptions(max_iterations=123, fanout_access_mode="static-stubs"),
        path_length=PathLengthMatchingOptions(
            enable_path_length_matching=True, path_length_output_scope="depth"
        ),
        debug=DebugArtifactOptions(debug_meanders=True),
    )

    stage = build_optical_routing_stage_config(
        flow_options,
        config,
        OpticalStageInputs(
            crossings=crossings,
            net_order="topological-span",
            net_order_depth_by_node={"a": 1},
            obstacle_config=obstacle_config,
            debug_dir=Path("build"),
            debug_route_indices={3},
            debug_svgs_enabled=True,
        ),
    )

    session = stage.session
    assert session.config is config
    assert session.obstacle_config is obstacle_config
    assert session.crossings is crossings
    assert session.search.max_iterations == 123
    assert session.search.allow_45_degree_turns is False
    assert session.search.net_order == "topological-span"
    assert session.search.net_order_depth_by_node == {"a": 1}
    assert session.port_access.fanout_access_mode == "static-stubs"
    assert session.realization.bend_radius_um == 7.0
    assert session.debug.debug_dir == Path("build")
    assert session.debug.debug_prefix == "benes_4x4"
    assert session.debug.debug_route_indices == {3}
    # the meander profile needs the per-net diagnostics of the session
    assert session.debug.verbose_route_diagnostics is True
    assert stage.matching.enable_path_length_matching is True
    assert stage.matching.path_length_output_scope == "depth"
    assert stage.matching.debug_meanders is True
    assert stage.debug_svgs_enabled is True
