"""Polygon realization for Rust-routed photonic nets."""

from __future__ import annotations

import importlib
import math
from collections.abc import Iterable, Mapping
from typing import Protocol

from gdsfactory.component import Component
import klayout.db as kdb

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
        allow_unchecked_bumps: bool = True,
    ) -> Iterable[object]:
        ...


def _physical_port_centerline(
    router: _EndpointCorrectionRouter,
    record: RoutedNetRecord,
    *,
    realization_grid_spec: tuple[int, int, float, float, float] | None = None,
    enable_endpoint_correction: bool = True,
    allow_unchecked_bumps: bool = True,
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
            allow_unchecked_bumps=allow_unchecked_bumps,
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
    crossing_plan_info: Mapping[str, object] | None = None,
    enable_endpoint_correction: bool = True,
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
    dbu = _component_dbu(routed_layout)
    crossing_clip_regions_by_net_id = _crossing_clip_regions_by_net_id(
        crossing_plan_info,
        dbu=dbu,
    )

    for record in routed_net_records:
        clip_region = (
            crossing_clip_regions_by_net_id.get(int(record.net_id))
            if record.net_id is not None
            else None
        )
        corrected_centerline = _physical_port_centerline(
            router,
            record,
            realization_grid_spec=realization_grid_spec,
            enable_endpoint_correction=enable_endpoint_correction,
            allow_unchecked_bumps=not allow_45_degree_turns,
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
            _add_route_polygon(
                routed_layout,
                polygon,
                route_layer=route_layer,
                clip_region=clip_region,
            )
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
            _add_route_polygon(
                routed_layout,
                polygon,
                route_layer=route_layer,
                clip_region=clip_region,
            )
            continue
        polygon = router.realize_route_polygon(record.route_obj, float(route_width_um))
        _add_route_polygon(
            routed_layout,
            polygon,
            route_layer=route_layer,
            clip_region=clip_region,
        )


def _add_route_polygon(
    routed_layout: Component,
    polygon: object,
    *,
    route_layer: tuple[int, int],
    clip_region: kdb.Region | None = None,
) -> None:
    if clip_region is None or clip_region.is_empty():
        routed_layout.add_polygon(polygon, layer=route_layer)
        return

    temp = Component()
    temp.add_polygon(polygon, layer=route_layer)
    route_region = _component_layer_region(temp, route_layer)
    if route_region.is_empty():
        return
    clipped = route_region - clip_region
    for clipped_polygon in clipped.each():
        routed_layout.add_polygon(clipped_polygon, layer=route_layer)


def _crossing_clip_regions_by_net_id(
    crossing_plan_info: Mapping[str, object] | None,
    *,
    dbu: float,
) -> dict[int, kdb.Region]:
    if not isinstance(crossing_plan_info, Mapping) or not crossing_plan_info.get(
        "enabled",
    ):
        return {}
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, Iterable) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return {}

    regions_by_net_id: dict[int, kdb.Region] = {}
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        footprint = _polygon_region_um(
            raw_crossing.get("crossing_footprint_polygon_um"),
            dbu=dbu,
        )
        if footprint.is_empty():
            continue
        for key in ("net_id_a", "net_id_b"):
            net_id = _as_int_or_none(raw_crossing.get(key))
            if net_id is None:
                continue
            regions_by_net_id.setdefault(net_id, kdb.Region())
            regions_by_net_id[net_id] += footprint
    return regions_by_net_id


def _polygon_region_um(raw_polygon: object, *, dbu: float) -> kdb.Region:
    region = kdb.Region()
    if not isinstance(raw_polygon, Iterable) or isinstance(
        raw_polygon,
        (str, bytes, bytearray),
    ):
        return region

    points: list[kdb.Point] = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
            return kdb.Region()
        try:
            x = float(raw_point[0])
            y = float(raw_point[1])
        except (TypeError, ValueError):
            return kdb.Region()
        if not math.isfinite(x) or not math.isfinite(y):
            return kdb.Region()
        points.append(kdb.Point(_um_to_dbu(x, dbu), _um_to_dbu(y, dbu)))
    if len(points) < 3:
        return region
    region.insert(kdb.Polygon(points))
    return region


def _component_layer_region(component: Component, layer: tuple[int, int]) -> kdb.Region:
    region = kdb.Region()
    for polygon in component.get_polygons(merge=False, by="tuple").get(layer, []):
        region.insert(polygon)
    return region


def _component_dbu(component: Component) -> float:
    kcl = getattr(component, "kcl", None)
    dbu = getattr(kcl, "dbu", None)
    if dbu is None:
        return 0.001
    return float(dbu)


def _um_to_dbu(value_um: float, dbu: float) -> int:
    return int(round(float(value_um) / float(dbu)))


def _as_int_or_none(value: object) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None
