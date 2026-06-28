"""Polygon realization for Rust-routed photonic nets."""

from __future__ import annotations

import importlib
from collections.abc import Iterable
from typing import Protocol

from gdsfactory.component import Component

from translation.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    RoutedNetRecord,
    _as_float,
    _as_int,
)
from translation.route_rust_records import format_port_endpoint_correction_error

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
_load_rust_backend = _sob._load_rust_backend


class _EndpointCorrectionRouter(Protocol):
    def route_port_corrected_centerline(
        self,
        route: object,
        *,
        source_port_um: tuple[float, float] | None = None,
        target_port_um: tuple[float, float] | None = None,
    ) -> Iterable[object]:
        ...


def _physical_port_centerline(
    router: _EndpointCorrectionRouter,
    record: RoutedNetRecord,
    *,
    realization_grid_spec: tuple[int, int, float, float, float] | None = None,
    enable_endpoint_correction: bool = True,
) -> list[tuple[float, float]]:
    corrected_centerline = [
        (float(p[0]), float(p[1]))
        for p in record.corrected_centerline_um
    ]
    if corrected_centerline:
        return corrected_centerline
    if not enable_endpoint_correction:
        return []

    source_port_um = record.source_port_center_um
    target_port_um = record.target_port_center_um
    if source_port_um is None and target_port_um is None:
        return []
    if record.endpoint_correction_error is not None:
        return []

    try:
        raw_centerline = router.route_port_corrected_centerline(
            record.route_obj,
            source_port_um=source_port_um,
            target_port_um=target_port_um,
        )
    except (TypeError, ValueError) as exc:
        print(
            "ERROR: "
            + format_port_endpoint_correction_error(
                record,
                exc,
                realization_grid_spec=realization_grid_spec,
            )
        )
        return []

    centerline: list[tuple[float, float]] = []
    for point in raw_centerline:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            print(
                "ERROR: "
                + format_port_endpoint_correction_error(
                    record,
                    "router returned an invalid corrected centerline point",
                    realization_grid_spec=realization_grid_spec,
                )
            )
            return []
        centerline.append((float(point[0]), float(point[1])))
    if not centerline:
        print(
            "ERROR: "
            + format_port_endpoint_correction_error(
                record,
                "router returned an empty corrected centerline",
                realization_grid_spec=realization_grid_spec,
            )
        )
        return []
    return centerline


def realize_routed_net_records(
    routed_layout: Component,
    routed_net_records: list[RoutedNetRecord],
    *,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool = True,
    bend_radius_cells: int = 4,
) -> None:
    """Phase B: realize routed records into polygons on the target layout."""
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    width, height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    for record in routed_net_records:
        corrected_centerline = _physical_port_centerline(
            router,
            record,
            realization_grid_spec=realization_grid_spec,
            enable_endpoint_correction=not allow_45_degree_turns,
        )
        if record.meander_auto_plan is not None:
            plan = record.meander_auto_plan
            selected_side = plan.get("selected_side")
            selected_box = plan.get("selected_box")
            selected_run_start_index = plan.get("selected_run_start_index")
            selected_run_end_index = plan.get("selected_run_end_index")
            selected_meander_centerline = plan.get("selected_meander_centerline")
            if (
                isinstance(selected_run_start_index, (int, float))
                and isinstance(selected_run_end_index, (int, float))
                and isinstance(selected_meander_centerline, list)
                and len(selected_meander_centerline) >= 2
            ):
                meander_centerline = [
                    (_as_float(p[0], 0.0), _as_float(p[1], 0.0))
                    for p in selected_meander_centerline
                    if isinstance(p, (tuple, list)) and len(p) == 2
                ]
                if corrected_centerline:
                    realize_with_tangents = getattr(
                        router,
                        "realize_centerline_polygon_from_planned_auto_meander_with_terminal_tangents",
                    )
                    try:
                        polygon = realize_with_tangents(
                            corrected_centerline,
                            float(route_width_um),
                            record.route_obj,
                            selected_run_start_index=_as_int(selected_run_start_index, 0),
                            selected_run_end_index=_as_int(selected_run_end_index, 0),
                            meander_centerline=meander_centerline,
                            source_enabled=record.source_port_center_um is not None,
                            target_enabled=record.target_port_center_um is not None,
                        )
                    except ValueError as exc:
                        raise RuntimeError(
                            format_port_endpoint_correction_error(
                                record,
                                exc,
                                realization_grid_spec=realization_grid_spec,
                            )
                        ) from exc
                else:
                    polygon = router.realize_route_polygon_from_planned_auto_meander(
                        record.route_obj,
                        float(route_width_um),
                        selected_run_start_index=_as_int(selected_run_start_index, 0),
                        selected_run_end_index=_as_int(selected_run_end_index, 0),
                        meander_centerline=meander_centerline,
                    )
            elif (
                isinstance(selected_side, str)
                and selected_side in {"left", "right"}
                and isinstance(selected_box, (tuple, list))
                and len(selected_box) == 4
            ):
                box_tuple = (
                    _as_float(selected_box[0], 0.0),
                    _as_float(selected_box[1], 0.0),
                    _as_float(selected_box[2], 0.0),
                    _as_float(selected_box[3], 0.0),
                )
                polygon = router.realize_route_polygon_with_analytic_meander(
                    record.route_obj,
                    float(route_width_um),
                    requested_extra_length_um=_as_float(plan["requested_extra_length_um"], 0.0),
                    min_bend_radius_um=plan["min_bend_radius_um"],
                    min_straight_um=_as_float(plan["min_straight_um"], 0.0),
                    max_bumps=_as_int(plan["max_bumps"], 8),
                    side=selected_side,
                    available_box=box_tuple,
                    planning_mode=str(plan["planning_mode"]),
                )
            else:
                # Backward-compatible fallback for older records that lack
                # persisted selected_side/selected_box metadata.
                polygon = router.realize_route_polygon_with_auto_checked_analytic_meander(
                    record.route_obj,
                    float(route_width_um),
                    requested_extra_length_um=_as_float(plan["requested_extra_length_um"], 0.0),
                    min_bend_radius_um=plan["min_bend_radius_um"],
                    min_straight_um=_as_float(plan["min_straight_um"], 0.0),
                    max_bumps=_as_int(plan["max_bumps"], 8),
                    max_meander_height_um=_as_float(
                        plan.get("max_meander_height_um", DEFAULT_MEANDER_MAX_HEIGHT_UM),
                        DEFAULT_MEANDER_MAX_HEIGHT_UM,
                    ),
                    box_depth_um=_as_float(plan["box_depth_um"], 20.0),
                    min_segment_length_um=_as_float(plan["min_segment_length_um"], 1.0),
                    clearance_radius_cells=_as_int(plan["clearance_radius_cells"], 0),
                    side_policy=str(plan["side_policy"]),
                    opened_cells=[],
                    planning_mode=str(plan["planning_mode"]),
                )
            routed_layout.add_polygon(polygon, layer=route_layer)
            continue
        if corrected_centerline:
            try:
                polygon = router.realize_centerline_polygon_with_terminal_tangents(
                    corrected_centerline,
                    float(route_width_um),
                    record.route_obj,
                    source_enabled=record.source_port_center_um is not None,
                    target_enabled=record.target_port_center_um is not None,
                )
            except ValueError as exc:
                raise RuntimeError(
                    format_port_endpoint_correction_error(
                        record,
                        exc,
                        realization_grid_spec=realization_grid_spec,
                    )
                ) from exc
            routed_layout.add_polygon(polygon, layer=route_layer)
            continue
        polygon = router.realize_route_polygon(record.route_obj, float(route_width_um))
        routed_layout.add_polygon(polygon, layer=route_layer)
