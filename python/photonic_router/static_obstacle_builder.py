"""Build a static grid obstacle map from extracted gdsfactory benchmark geometry."""

from __future__ import annotations

import math
import sys
from importlib import machinery, util
from importlib import import_module
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence, Set, Tuple

from photonic_router.benchmark_extractor import BBox, ExtractedBenchmark, Point, Polygon, extract_benchmark
from photonic_router.routing_layers import HEATER_METAL_OBSTACLE_LAYERS

GridCell = Tuple[int, int]
GridRect = Tuple[int, int, int, int]


@dataclass(frozen=True)
class StaticObstacleMapConfig:
    """Configuration for converting physical layout geometry into grid cells."""

    grid_size_um: float = 0.5
    security_margin_um: float = 20.0
    clearance_um: float = 0.5
    heater_clearance_um: Optional[float] = None
    clearance_metric: str = "chebyshev"
    port_open_radius_um: float = 0.5
    obstacle_mode: str = "bounding_boxes"
    clear_port_open_cells_from_static: bool = False
    materialize_bbox_cells: bool = True
    populate_obstacle_map: bool = True
    die_bbox: Optional[BBox] = None
    obstacle_layers: Optional[Tuple[Tuple[int, int], ...]] = None
    heater_obstacle_layers: Optional[Tuple[Tuple[int, int], ...]] = (
        HEATER_METAL_OBSTACLE_LAYERS
    )


@dataclass(frozen=True)
class GridSpec:
    """Rectangular grid definition for the Rust router."""

    width: int
    height: int
    grid_size_um: float
    origin: Point
    die_bbox: BBox


@dataclass(frozen=True)
class StaticObstacleMapData:
    """Static obstacle-map payload ready to pass to Rust later."""

    grid: GridSpec
    raw_blocked_cells: Set[GridCell]
    blocked_cells: Set[GridCell]
    port_open_cells: Set[GridCell]
    benchmark: ExtractedBenchmark
    raw_static_rects: tuple[GridRect, ...] = ()
    blocked_static_rects: tuple[GridRect, ...] = ()
    backend: str = "python"
    build_stats: dict[str, Any] | None = None

    def rust_static_cells(self) -> List[GridCell]:
        """Return deterministic `(x, y)` cells suitable for `ObstacleMap.add_static_cells`."""

        return sorted(self.blocked_cells)

    def rust_open_cells(self) -> List[GridCell]:
        """Return deterministic port-opening cells for temporary free-space queries."""

        return sorted(self.port_open_cells)

    def export_debug_svg(
        self,
        path: str | Path,
        *,
        max_cell_px: int = 8,
        show_ports: bool = True,
    ) -> None:
        """Export a simple SVG view of blocked cells and optional port openings."""

        path = Path(path)
        path.write_text(
            grid_to_svg(
                self.grid,
                self.blocked_cells,
                self.port_open_cells if show_ports else set(),
                max_cell_px=max_cell_px,
            ),
            encoding="utf-8",
        )


def build_static_obstacle_map(
    component: object,
    config: StaticObstacleMapConfig | None = None,
) -> StaticObstacleMapData:
    """Extract and rasterize fixed geometry from a gdsfactory component.

    The default path uses the Rust backend when the PyO3 extension is installed.
    If the extension is not importable, this falls back to the pure-Python
    implementation so tests and development flows still work.
    """

    config = config or StaticObstacleMapConfig()
    split_layers = _split_obstacle_layers_for_heater_clearance(config)
    if split_layers is not None:
        return _build_static_obstacle_map_split_by_heater_clearance(
            component,
            config,
            split_layers=split_layers,
        )

    benchmark = extract_benchmark(component, layers=config.obstacle_layers)

    rust_backend = _load_rust_backend()
    if rust_backend is not None:
        return _build_static_obstacle_map_rust(benchmark, config, rust_backend)

    return build_static_obstacle_map_python_from_extracted(benchmark, config)


def _split_obstacle_layers_for_heater_clearance(
    config: StaticObstacleMapConfig,
) -> tuple[tuple[tuple[int, int], ...], tuple[tuple[int, int], ...]] | None:
    if config.heater_clearance_um is None:
        return None
    if config.obstacle_layers is None or config.heater_obstacle_layers is None:
        return None

    obstacle_layers = _normalize_layers(config.obstacle_layers)
    heater_layers = _normalize_layers(config.heater_obstacle_layers)
    selected_heater_layers = tuple(layer for layer in obstacle_layers if layer in heater_layers)
    if not selected_heater_layers:
        return None

    waveguide_layers = tuple(layer for layer in obstacle_layers if layer not in heater_layers)
    return waveguide_layers, selected_heater_layers


def _build_static_obstacle_map_split_by_heater_clearance(
    component: object,
    config: StaticObstacleMapConfig,
    *,
    split_layers: tuple[tuple[tuple[int, int], ...], tuple[tuple[int, int], ...]],
) -> StaticObstacleMapData:
    waveguide_layers, heater_layers = split_layers
    heater_clearance_um = config.heater_clearance_um
    if heater_clearance_um is None:
        raise ValueError("heater_clearance_um is required when splitting heater obstacles")

    all_benchmark = extract_benchmark(component, layers=config.obstacle_layers)
    shared_die_bbox = config.die_bbox
    if shared_die_bbox is None:
        shared_die_bbox = make_grid_spec(
            all_benchmark,
            grid_size_um=config.grid_size_um,
            security_margin_um=config.security_margin_um,
            die_bbox=None,
        ).die_bbox

    rust_backend = _load_rust_backend()
    waveguide_data = _build_layer_group_static_obstacles(
        component,
        all_benchmark,
        replace(
            config,
            clearance_um=float(config.clearance_um),
            heater_clearance_um=None,
            obstacle_layers=waveguide_layers,
            die_bbox=shared_die_bbox,
        ),
        rust_backend,
    )
    heater_data = _build_layer_group_static_obstacles(
        component,
        all_benchmark,
        replace(
            config,
            clearance_um=0.0,
            heater_clearance_um=None,
            obstacle_layers=heater_layers,
            die_bbox=shared_die_bbox,
        ),
        rust_backend,
    )
    heater_data = _apply_perpendicular_heater_clearance(
        heater_data,
        clearance_um=float(heater_clearance_um),
    )
    return _merge_static_obstacle_data(
        all_benchmark,
        waveguide_data,
        heater_data,
        backend="rust-split" if rust_backend is not None else "python-split",
    )


def _build_layer_group_static_obstacles(
    component: object,
    all_benchmark: ExtractedBenchmark,
    config: StaticObstacleMapConfig,
    rust_backend: Any | None,
) -> StaticObstacleMapData:
    if config.obstacle_layers:
        benchmark = extract_benchmark(component, layers=config.obstacle_layers)
    else:
        benchmark = ExtractedBenchmark(
            polygons=[],
            ports=all_benchmark.ports,
            bbox=all_benchmark.bbox,
        )

    if not benchmark.polygons:
        return build_static_obstacle_map_python_from_extracted(benchmark, config)

    if rust_backend is not None:
        return _build_static_obstacle_map_rust(benchmark, config, rust_backend)
    return build_static_obstacle_map_python_from_extracted(benchmark, config)


def _apply_perpendicular_heater_clearance(
    data: StaticObstacleMapData,
    *,
    clearance_um: float,
) -> StaticObstacleMapData:
    radius = math.ceil(max(0.0, clearance_um) / data.grid.grid_size_um)
    raw_rects = data.raw_static_rects or tuple(
        rect
        for polygon in data.benchmark.polygons
        if (rect := polygon_to_grid_rect(polygon, data.grid)) is not None
    )
    if radius <= 0 or not raw_rects:
        return StaticObstacleMapData(
            grid=data.grid,
            raw_blocked_cells=data.raw_blocked_cells,
            blocked_cells=data.blocked_cells,
            port_open_cells=data.port_open_cells,
            raw_static_rects=raw_rects,
            blocked_static_rects=data.blocked_static_rects or raw_rects,
            benchmark=data.benchmark,
            backend=data.backend,
            build_stats=data.build_stats,
        )

    blocked_rects = tuple(
        _expand_rect_perpendicular_to_long_axis(rect, radius)
        for rect in raw_rects
    )
    blocked_cells = (
        set(_materialize_grid_rects(blocked_rects, data.grid))
        if data.blocked_cells
        else set()
    )
    return StaticObstacleMapData(
        grid=data.grid,
        raw_blocked_cells=data.raw_blocked_cells,
        blocked_cells=blocked_cells,
        port_open_cells=data.port_open_cells,
        raw_static_rects=raw_rects,
        blocked_static_rects=blocked_rects,
        benchmark=data.benchmark,
        backend=data.backend,
        build_stats=data.build_stats,
    )


def _expand_rect_perpendicular_to_long_axis(rect: GridRect, radius: int) -> GridRect:
    x_min, y_min, x_max, y_max = rect
    width = x_max - x_min + 1
    height = y_max - y_min + 1
    if width >= height:
        return (x_min, y_min - radius, x_max, y_max + radius)
    return (x_min - radius, y_min, x_max + radius, y_max)


def _materialize_grid_rects(rects: Iterable[GridRect], grid: GridSpec) -> Iterable[GridCell]:
    for x_min, y_min, x_max, y_max in rects:
        for y in range(max(0, y_min), min(grid.height - 1, y_max) + 1):
            for x in range(max(0, x_min), min(grid.width - 1, x_max) + 1):
                yield (x, y)


def _merge_static_obstacle_data(
    benchmark: ExtractedBenchmark,
    *data_items: StaticObstacleMapData,
    backend: str,
) -> StaticObstacleMapData:
    if not data_items:
        raise ValueError("at least one static obstacle map is required")

    grid = data_items[0].grid
    for data in data_items[1:]:
        if data.grid != grid:
            raise ValueError("split static obstacle maps produced different grids")

    build_stats = _merge_build_stats(data_items)
    return StaticObstacleMapData(
        grid=grid,
        raw_blocked_cells=set().union(*(data.raw_blocked_cells for data in data_items)),
        blocked_cells=set().union(*(data.blocked_cells for data in data_items)),
        port_open_cells=set().union(*(data.port_open_cells for data in data_items)),
        raw_static_rects=tuple(
            rect for data in data_items for rect in data.raw_static_rects
        ),
        blocked_static_rects=tuple(
            rect for data in data_items for rect in data.blocked_static_rects
        ),
        benchmark=benchmark,
        backend=backend,
        build_stats=build_stats,
    )


def _merge_build_stats(data_items: Iterable[StaticObstacleMapData]) -> dict[str, Any] | None:
    merged: dict[str, Any] = {}
    saw_stats = False
    for data in data_items:
        stats = data.build_stats
        if not stats:
            continue
        saw_stats = True
        for key, value in stats.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                merged[key] = merged.get(key, 0) + value
            elif key not in merged:
                merged[key] = value
    return merged if saw_stats else None


def _normalize_layers(layers: Iterable[Any]) -> tuple[tuple[int, int], ...]:
    normalized: list[tuple[int, int]] = []
    for layer in layers:
        if not isinstance(layer, tuple) or len(layer) < 2:
            raise ValueError(f"layer must be a (layer, datatype) tuple, got {layer!r}")
        normalized.append((int(layer[0]), int(layer[1])))
    return tuple(normalized)


def build_static_obstacle_map_python_from_extracted(
    benchmark: ExtractedBenchmark,
    config: StaticObstacleMapConfig | None = None,
) -> StaticObstacleMapData:
    """Pure-Python fallback builder from already extracted benchmark geometry."""

    config = config or StaticObstacleMapConfig()
    grid = make_grid_spec(
        benchmark,
        grid_size_um=config.grid_size_um,
        security_margin_um=config.security_margin_um,
        die_bbox=config.die_bbox,
    )

    raw_blocked: Set[GridCell] = set()
    if config.obstacle_mode == "rasterized_polygons":
        for polygon in benchmark.polygons:
            raw_blocked.update(rasterize_polygon(polygon, grid))
    elif config.obstacle_mode == "bounding_boxes":
        for polygon in benchmark.polygons:
            raw_blocked.update(polygon_to_bbox_cells(polygon, grid))
    else:
        raise ValueError("obstacle_mode must be 'rasterized_polygons' or 'bounding_boxes'")

    clearance_radius = math.ceil(config.clearance_um / config.grid_size_um)
    blocked = inflate_cells(
        raw_blocked,
        grid.width,
        grid.height,
        clearance_radius,
        metric=config.clearance_metric,
    )

    port_radius = math.ceil(config.port_open_radius_um / config.grid_size_um)
    port_open_cells = build_port_open_cells(benchmark, grid, port_radius)

    if config.clear_port_open_cells_from_static:
        blocked.difference_update(port_open_cells)

    return StaticObstacleMapData(
        grid=grid,
        raw_blocked_cells=raw_blocked,
        blocked_cells=blocked,
        port_open_cells=port_open_cells,
        raw_static_rects=(),
        blocked_static_rects=(),
        benchmark=benchmark,
        backend="python",
        build_stats=None,
    )


def _build_static_obstacle_map_rust(
    benchmark: ExtractedBenchmark,
    config: StaticObstacleMapConfig,
    rust_backend: Any,
) -> StaticObstacleMapData:
    ports = [
        (port.name, port.position[0], port.position[1], port.orientation)
        for port in benchmark.ports
    ]
    fallback_without_clear_port_flag = False
    used_legacy_builder_signature = False

    def _call_rust_builder() -> dict[str, Any]:
        positional_compat_message = (
            "clear_port_open_cells_from_static",
            "positional argument",
            "positional arguments",
            "takes from",
            "unexpected argument count",
        )

        def _is_compat_error(message: str) -> bool:
            return any(token in message for token in positional_compat_message)

        try:
            result_obj = rust_backend.build_static_obstacle_map_rs(
                benchmark.polygons,
                ports,
                config.grid_size_um,
                config.security_margin_um,
                config.clearance_um,
                config.clearance_metric,
                config.port_open_radius_um,
                config.die_bbox,
                config.obstacle_mode,
                config.clear_port_open_cells_from_static,
                config.materialize_bbox_cells,
                config.populate_obstacle_map,
            )
            return result_obj
        except TypeError as exc:
            # Backward compatibility with older local wheels/extensions.
            msg = str(exc)
            if not _is_compat_error(msg):
                raise
            nonlocal used_legacy_builder_signature
            used_legacy_builder_signature = True
            try:
                return rust_backend.build_static_obstacle_map_rs(
                    benchmark.polygons,
                    ports,
                    config.grid_size_um,
                    config.security_margin_um,
                    config.clearance_um,
                    config.clearance_metric,
                    config.port_open_radius_um,
                    config.die_bbox,
                    config.obstacle_mode,
                    config.clear_port_open_cells_from_static,
                    config.materialize_bbox_cells,
                )
            except TypeError as exc_without_populate:
                if not _is_compat_error(str(exc_without_populate)):
                    raise
                try:
                    return rust_backend.build_static_obstacle_map_rs(
                        benchmark.polygons,
                        ports,
                        config.grid_size_um,
                        config.security_margin_um,
                        config.clearance_um,
                        config.clearance_metric,
                        config.port_open_radius_um,
                        config.die_bbox,
                        config.obstacle_mode,
                        config.clear_port_open_cells_from_static,
                    )
                except TypeError as exc_without_materialize:
                    if not _is_compat_error(str(exc_without_materialize)):
                        raise
                    nonlocal fallback_without_clear_port_flag
                    fallback_without_clear_port_flag = True
                    try:
                        return rust_backend.build_static_obstacle_map_rs(
                            benchmark.polygons,
                            ports,
                            config.grid_size_um,
                            config.security_margin_um,
                            config.clearance_um,
                            config.clearance_metric,
                            config.port_open_radius_um,
                            config.die_bbox,
                            config.obstacle_mode,
                        )
                    except TypeError as exc_without_mode:
                        if not _is_compat_error(str(exc_without_mode)):
                            raise
                        try:
                            return rust_backend.build_static_obstacle_map_rs(
                                benchmark.polygons,
                                ports,
                                config.grid_size_um,
                                config.security_margin_um,
                                config.clearance_um,
                                config.clearance_metric,
                                config.port_open_radius_um,
                                config.die_bbox,
                            )
                        except TypeError as exc_without_die_bbox:
                            if not _is_compat_error(str(exc_without_die_bbox)):
                                raise
                            return rust_backend.build_static_obstacle_map_rs(
                                benchmark.polygons,
                                ports,
                                config.grid_size_um,
                                config.security_margin_um,
                                config.clearance_um,
                                config.clearance_metric,
                                config.port_open_radius_um,
                            )

    result = _call_rust_builder()

    width_raw, height_raw, grid_size_raw, origin_raw, die_bbox_raw = result["grid"]
    width = int(width_raw)
    height = int(height_raw)
    grid_size_um = float(grid_size_raw)
    origin = (float(origin_raw[0]), float(origin_raw[1]))
    die_bbox = (
        float(die_bbox_raw[0]),
        float(die_bbox_raw[1]),
        float(die_bbox_raw[2]),
        float(die_bbox_raw[3]),
    )
    blocked_cells = set(map(tuple, result["blocked_cells"]))
    port_open_cells = set(map(tuple, result["port_open_cells"]))

    def _coerce_grid_rects(rects: Iterable[Sequence[Any]]) -> tuple[GridRect, ...]:
        coerced: list[GridRect] = []
        for rect in rects:
            if len(rect) != 4:
                raise ValueError(f"grid rect must contain exactly 4 coordinates, got {rect!r}")
            coerced.append((int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])))
        return tuple(coerced)

    raw_static_rects = _coerce_grid_rects(result.get("raw_static_rects", ()))
    blocked_static_rects = _coerce_grid_rects(result.get("blocked_static_rects", ()))
    compact_bbox_requested = (
        config.obstacle_mode == "bounding_boxes" and not config.materialize_bbox_cells
    )
    if compact_bbox_requested and used_legacy_builder_signature:
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        router_cls = getattr(rust_backend, "PyPhotonicRouter", None)
        has_static_rect_router = bool(
            router_cls is not None and hasattr(router_cls, "set_static_rects")
        )
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not support compact "
            "bounding-box obstacle payloads. "
            f"Loaded extension: {extension_path}. "
            f"PyPhotonicRouter.set_static_rects available: {has_static_rect_router}. "
            "Rebuild the extension with `python3 -m maturin develop --release` "
            "using the same Python interpreter that runs routing_flow.py; "
            "otherwise bbox mode falls back to materialized cells and will be slow."
        )
    if compact_bbox_requested and not blocked_static_rects:
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "Compact bounding-box mode requested, but the Rust obstacle builder "
            "returned no blocked_static_rects. "
            f"Loaded extension: {extension_path}. "
            "Rebuild photonic_router._rust with "
            "`python3 -m maturin develop --release` and verify the active extension path."
        )

    if fallback_without_clear_port_flag and not config.clear_port_open_cells_from_static:
        # Older builders always clear port opening cells from static blockers.
        # Emulate the strict-per-net mode by re-adding those cells on the Python
        # side to keep routing behavior usable with stale backends.
        blocked_cells |= port_open_cells

    stats_raw = result.get("stats")
    if fallback_without_clear_port_flag:
        if isinstance(stats_raw, dict):
            stats_raw["clear_port_open_cells_from_static"] = False
            stats_raw["blocked_cells_after_port_opening"] = len(blocked_cells)
    build_stats = (
        {str(key): value for key, value in stats_raw.items()}
        if isinstance(stats_raw, dict)
        else None
    )
    return StaticObstacleMapData(
        grid=GridSpec(
            width=width,
            height=height,
            grid_size_um=grid_size_um,
            origin=origin,
            die_bbox=die_bbox,
        ),
        raw_blocked_cells=set(map(tuple, result["raw_blocked_cells"])),
        blocked_cells=blocked_cells,
        port_open_cells=port_open_cells,
        raw_static_rects=raw_static_rects,
        blocked_static_rects=blocked_static_rects,
        benchmark=benchmark,
        backend="rust",
        build_stats=build_stats,
    )


def _load_rust_backend() -> Any | None:
    try:
        return import_module("photonic_router._rust")
    except ImportError:
        pass

    # Development fallback: `cargo build` creates the PyO3 cdylib under
    # target/debug/deps, but does not install it into python/photonic_router.
    # Loading it directly keeps IDE runs fast without requiring `maturin develop`
    # after every local Rust edit.
    project_root = Path(__file__).resolve().parents[2]
    candidates = [
        project_root / "target" / "release" / "deps" / "libphotonic_router.so",
        project_root / "target" / "release" / "libphotonic_router.so",
        project_root / "target" / "debug" / "deps" / "libphotonic_router.so",
        project_root / "target" / "debug" / "libphotonic_router.so",
    ]

    for candidate in candidates:
        if not candidate.exists():
            continue
        try:
            module_name = "photonic_router._rust"
            loader = machinery.ExtensionFileLoader(module_name, str(candidate))
            spec = util.spec_from_file_location(module_name, candidate, loader=loader)
            if spec is None:
                continue
            module = util.module_from_spec(spec)
            loader.exec_module(module)
            sys.modules[module_name] = module
            return module
        except ImportError:
            continue

    return None


def make_grid_spec(
    benchmark: ExtractedBenchmark,
    *,
    grid_size_um: float,
    security_margin_um: float,
    die_bbox: Optional[BBox] = None,
) -> GridSpec:
    """Compute die bounds, grid dimensions, and grid origin."""

    if grid_size_um <= 0:
        raise ValueError("grid_size_um must be positive")

    bbox = die_bbox if die_bbox is not None else expand_bbox(benchmark.bbox, security_margin_um)
    xmin, ymin, xmax, ymax = bbox
    if xmax < xmin or ymax < ymin:
        raise ValueError(f"invalid die bbox: {bbox}")

    width = math.ceil((xmax - xmin) / grid_size_um)
    height = math.ceil((ymax - ymin) / grid_size_um)

    return GridSpec(
        width=max(width, 0),
        height=max(height, 0),
        grid_size_um=grid_size_um,
        origin=(xmin, ymin),
        die_bbox=bbox,
    )


def expand_bbox(bbox: BBox, margin_um: float) -> BBox:
    """Expand a physical bbox by `margin_um` on every side."""

    xmin, ymin, xmax, ymax = bbox
    return (
        xmin - margin_um,
        ymin - margin_um,
        xmax + margin_um,
        ymax + margin_um,
    )


def physical_to_grid(x: float, y: float, grid: GridSpec) -> GridCell:
    """Convert physical micrometer coordinates to integer grid coordinates."""

    xmin, ymin = grid.origin
    gx = math.floor((x - xmin) / grid.grid_size_um)
    gy = math.floor((y - ymin) / grid.grid_size_um)
    return gx, gy


def grid_cell_center(gx: int, gy: int, grid: GridSpec) -> Point:
    """Return the physical center of a grid cell in micrometers."""

    xmin, ymin = grid.origin
    return (
        xmin + (gx + 0.5) * grid.grid_size_um,
        ymin + (gy + 0.5) * grid.grid_size_um,
    )


def rasterize_polygon(polygon: Polygon, grid: GridSpec) -> Set[GridCell]:
    """Rasterize a polygon by testing grid-cell centers inside its bbox."""

    if len(polygon) < 3:
        return set()

    min_x = min(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_x = max(point[0] for point in polygon)
    max_y = max(point[1] for point in polygon)

    gx_min, gy_min = physical_to_grid(min_x, min_y, grid)
    gx_max, gy_max = physical_to_grid(max_x, max_y, grid)

    gx_min = max(gx_min, 0)
    gy_min = max(gy_min, 0)
    gx_max = min(gx_max, grid.width - 1)
    gy_max = min(gy_max, grid.height - 1)

    cells: Set[GridCell] = set()
    for gx in range(gx_min, gx_max + 1):
        for gy in range(gy_min, gy_max + 1):
            center = grid_cell_center(gx, gy, grid)
            if point_in_polygon(center, polygon):
                cells.add((gx, gy))

    return cells


def polygon_to_bbox_cells(polygon: Polygon, grid: GridSpec) -> Set[GridCell]:
    """Conservatively mark polygon bounding-box cell range."""

    rect = polygon_to_grid_rect(polygon, grid)
    if rect is None:
        return set()
    gx_min, gy_min, gx_max, gy_max = rect
    return {
        (gx, gy)
        for gx in range(gx_min, gx_max + 1)
        for gy in range(gy_min, gy_max + 1)
    }


def polygon_to_grid_rect(polygon: Polygon, grid: GridSpec) -> GridRect | None:
    """Conservatively map a polygon bbox to an inclusive grid rectangle."""

    if len(polygon) < 3:
        return None

    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)

    gx_min = math.floor((min_x - grid.origin[0]) / grid.grid_size_um)
    gy_min = math.floor((min_y - grid.origin[1]) / grid.grid_size_um)
    gx_max = math.ceil((max_x - grid.origin[0]) / grid.grid_size_um) - 1
    gy_max = math.ceil((max_y - grid.origin[1]) / grid.grid_size_um) - 1

    gx_min = max(gx_min, 0)
    gy_min = max(gy_min, 0)
    gx_max = min(gx_max, grid.width - 1)
    gy_max = min(gy_max, grid.height - 1)

    if gx_min > gx_max or gy_min > gy_max:
        return None

    return gx_min, gy_min, gx_max, gy_max


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Return true when `point` is inside or on the boundary of `polygon`."""

    x, y = point
    inside = False
    count = len(polygon)

    for i in range(count):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % count]

        if _point_on_segment(x, y, x1, y1, x2, y2):
            return True

        crosses = (y1 > y) != (y2 > y)
        if crosses:
            x_intersection = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x <= x_intersection:
                inside = not inside

    return inside


def inflate_cells(
    cells: Iterable[GridCell],
    width: int,
    height: int,
    radius: int,
    *,
    metric: str,
) -> Set[GridCell]:
    """Expand cells by Manhattan or Chebyshev distance, clipped to grid bounds."""

    if radius < 0:
        raise ValueError("radius must be non-negative")

    normalized_metric = metric.lower()
    if normalized_metric not in {"manhattan", "chebyshev"}:
        raise ValueError("metric must be 'manhattan' or 'chebyshev'")

    inflated: Set[GridCell] = set()
    for gx, gy in cells:
        if not in_bounds(gx, gy, width, height):
            continue
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if normalized_metric == "manhattan":
                    include = abs(dx) + abs(dy) <= radius
                else:
                    include = max(abs(dx), abs(dy)) <= radius

                if not include:
                    continue

                nx = gx + dx
                ny = gy + dy
                if in_bounds(nx, ny, width, height):
                    inflated.add((nx, ny))

    return inflated


def build_port_open_cells(
    benchmark: ExtractedBenchmark,
    grid: GridSpec,
    radius: int,
) -> Set[GridCell]:
    """Generate temporary opening cells around every extracted port."""

    base_cells = [
        physical_to_grid(port.position[0], port.position[1], grid)
        for port in benchmark.ports
    ]
    return inflate_cells(base_cells, grid.width, grid.height, radius, metric="chebyshev")


def in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    return 0 <= gx < width and 0 <= gy < height


def grid_to_svg(
    grid: GridSpec,
    blocked_cells: Iterable[GridCell],
    port_open_cells: Iterable[GridCell] = (),
    *,
    max_cell_px: int = 8,
) -> str:
    """Render die, grid, obstacle cells, and port openings as a compact SVG string."""

    if grid.width <= 0 or grid.height <= 0:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />\n'

    cell_px = max(1, min(max_cell_px, 1200 // max(grid.width, grid.height, 1)))
    width_px = grid.width * cell_px
    height_px = grid.height * cell_px

    parts: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" '
        f'viewBox="0 0 {grid.width} {grid.height}">',
        '<rect width="100%" height="100%" fill="#eeeeee" />',
        '<path d="'
        + " ".join(f"M {x} 0 V {grid.height}" for x in range(grid.width + 1))
        + " "
        + " ".join(f"M 0 {y} H {grid.width}" for y in range(grid.height + 1))
        + '" stroke="#3fa34d" stroke-width="0.025" vector-effect="non-scaling-stroke" '
        + 'opacity="0.75" fill="none" />',
    ]

    for gx, gy in sorted(blocked_cells):
        svg_y = grid.height - gy - 1
        parts.append(f'<rect x="{gx}" y="{svg_y}" width="1" height="1" fill="#000000" />')

    for gx, gy in sorted(port_open_cells):
        svg_y = grid.height - gy - 1
        parts.append(f'<rect x="{gx}" y="{svg_y}" width="1" height="1" fill="#fbbc04" opacity="0.85" />')

    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _point_on_segment(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    *,
    eps: float = 1e-9,
) -> bool:
    cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
    if abs(cross) > eps:
        return False

    return (
        min(x1, x2) - eps <= px <= max(x1, x2) + eps
        and min(y1, y2) - eps <= py <= max(y1, y2) + eps
    )
