"""Fixtures of the path-length-matching tests (phase 6 slice C of the 2026-10
do-over).

The link-based netlists the graph, requirements, diagnostics and flow tests
build (`schematic_from_links`; four copies of `_Bundle`/`_Netlist`/`_Schematic`
lived in `test_path_length_graph.py` before), the records and edge keys made
from them, the fake Rust backend, router and `MeanderPlanner` the planner
tests drive the Python side with (`fake_backend`, `FakeRouter`,
`FakePlanner`; seven fake routers before), the real-backend helpers the
other planner tests need, and the small report/config builders around them.
The placed four-instance layout `path_length_schematic` stays in
`synthetic_layouts` beside the other placed layouts, since the electrical
and e2e tests use it too.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from types import SimpleNamespace
from typing import Any, Protocol

import pytest
from photonic_router.plm.path_length_graph import PortRef, RoutedEdgeKey
from photonic_router.plm.route_rust_meanders import _MeanderSearchConfig
from photonic_router.records.route_rust_types import (
    MeanderEdgeReport,
    MeanderInsertionReport,
    RoutedNetRecord,
)
from photonic_router.static_obstacle_builder import _load_rust_backend

GridCell = tuple[int, int]
Centerline = list[tuple[float, float]]


class SchematicLike(Protocol):
    netlist: Any
    placements: dict[str, object]


# --- netlists from links ----------------------------------------------------


class _Bundle:
    def __init__(self, links: dict[str, str]):
        self.links = links


class _Netlist:
    def __init__(self, instances: dict[str, object], routes: dict[str, _Bundle]):
        self.instances = instances
        self.routes = routes


class _Schematic:
    def __init__(self, netlist: _Netlist):
        self.netlist = netlist
        self.placements: dict[str, object] = {}


def schematic_from_links(
    instances: list[str] | Mapping[str, str],
    links_by_net: Mapping[str, Mapping[str, str]],
) -> SchematicLike:
    """A netlist-only schematic: the instances by name (with a `component`
    name each when a mapping is given, for the tests that tell nodes by
    structure rather than by name) and one bundle per net, its links
    `{"inst,port": "inst,port"}`."""
    if isinstance(instances, Mapping):
        instance_objects: dict[str, object] = {
            name: SimpleNamespace(component=component, settings={})
            for name, component in instances.items()
        }
    else:
        instance_objects = {name: object() for name in instances}
    routes = {net_name: _Bundle(dict(links)) for net_name, links in links_by_net.items()}
    return _Schematic(_Netlist(instance_objects, routes))


def edge_key(net_name: str, p1: str, p2: str) -> RoutedEdgeKey:
    """The edge key of net `net_name` from `"inst,port"` to `"inst,port"`."""
    a, pa = p1.split(",")
    b, pb = p2.split(",")
    return RoutedEdgeKey(net_name=net_name, source=PortRef(a, pa), target=PortRef(b, pb))


def records_from_links(
    nets: Mapping[str, tuple[str, str]],
    lengths: Mapping[str, float],
) -> list[RoutedNetRecord]:
    """One unrouted record per net (`route_obj=None`) at the given length."""
    return [
        RoutedNetRecord(
            net_name=name,
            source=edge_key(name, *ends).source,
            target=edge_key(name, *ends).target,
            route_obj=None,
            total_length_um=lengths[name],
        )
        for name, ends in nets.items()
    ]


# --- the fake Rust backend --------------------------------------------------


class FakeRouter:
    """The router the fake backend builds: it keeps the static cells it is
    given and answers port-corrected centerlines from `centerline` (none,
    so no record registers, when it is not given)."""

    def __init__(
        self,
        *_args: object,
        centerline: Callable[[Any], Centerline] | None = None,
        **_kwargs: object,
    ) -> None:
        self.static_cells: list[GridCell] = []
        self._centerline = centerline

    def set_static_cells(self, cells: Any) -> None:
        self.static_cells = list(cells)

    def add_static_cells(self, _cells: object) -> None:
        return None

    def route_port_corrected_centerline(self, route_obj: Any) -> Centerline:
        return [] if self._centerline is None else self._centerline(route_obj)


class FakePlanner:
    """The `MeanderPlanner` stand-in: registration answers index lists,
    `reserve_rect` records the rectangles, the two planning calls record
    their arguments and answer from `plan_requests`/`plan_final`."""

    def __init__(
        self,
        router: Any,
        *,
        plan_requests: Callable[..., dict[str, object]] | None = None,
        plan_final: Callable[..., dict[str, object]] | None = None,
    ) -> None:
        self.router = router
        self.calls: list[dict[str, object]] = []
        self.reserved_rects: list[tuple[int, int, int, int]] = []
        self._plan_requests = plan_requests
        self._plan_final = plan_final

    def register(
        self,
        routes: list[Any],
        base_static_cells: list[GridCell] | None = None,
        base_static_cell_handle: object | None = None,
        route_clearance_radius_cells: int = 0,
        reset: bool = True,
    ) -> tuple[list[int], list[int], int]:
        return list(range(len(routes))), [len(route.cells) for route in routes], 0

    def profile(self) -> dict[str, float]:
        return {}

    def register_geometries(
        self,
        centerlines: list[Centerline],
        _registered_opened_cell_indices: list[int],
        _max_bumps_by_edge: list[int],
    ) -> list[int]:
        return list(range(len(centerlines)))

    def reserve_rect(self, min_x: int, max_x: int, min_y: int, max_y: int) -> int:
        self.reserved_rects.append((min_x, max_x, min_y, max_y))
        return (max_x - min_x + 1) * (max_y - min_y + 1)

    def plan_requests(
        self,
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[list[float]],
        **kwargs: object,
    ) -> dict[str, object]:
        self.calls.append(
            {
                "candidate_geometry_indices": candidate_geometry_indices,
                "candidate_requested_extra_lengths_um": candidate_requested_extra_lengths_um,
                **kwargs,
            }
        )
        assert self._plan_requests is not None, "the fake planner has no plan_requests answer"
        return self._plan_requests(
            candidate_geometry_indices, candidate_requested_extra_lengths_um, **kwargs
        )

    def plan_final(
        self,
        geometry_indices: list[int],
        requested_extra_lengths_um: list[float],
        min_insertable_extra_length_um: float,
        **kwargs: object,
    ) -> dict[str, object]:
        assert self._plan_final is not None, "the fake planner has no plan_final answer"
        return self._plan_final(
            geometry_indices, requested_extra_lengths_um, min_insertable_extra_length_um, **kwargs
        )


def fake_backend(
    *,
    centerline: Callable[[Any], Centerline] | None = None,
    plan_requests: Callable[..., dict[str, object]] | None = None,
    plan_final: Callable[..., dict[str, object]] | None = None,
) -> Any:
    """A class standing in for the loaded Rust module: the three config
    constructors record their arguments, `PyPhotonicRouter` builds a
    `FakeRouter` answering `centerline`, `MeanderPlanner` a `FakePlanner`
    answering `plan_requests`/`plan_final`, and the meander search policy
    is the real backend's. The routers and planners built are kept on the
    class (`routers`, `planners`) for the assertions."""

    class _FakeBackend:
        routers: list[FakeRouter] = []
        planners: list[FakePlanner] = []
        GridSpec = staticmethod(lambda *args, **kwargs: ("grid", args, kwargs))
        PrimitiveLibraryConfig = staticmethod(lambda *args, **kwargs: ("primitive", args, kwargs))
        AStarConfig = staticmethod(lambda *args, **kwargs: ("astar", args, kwargs))

        @staticmethod
        def PyPhotonicRouter(*args: object, **kwargs: object) -> FakeRouter:
            router = FakeRouter(*args, centerline=centerline, **kwargs)
            _FakeBackend.routers.append(router)
            return router

        @staticmethod
        def MeanderPlanner(router: Any) -> FakePlanner:
            planner = FakePlanner(router, plan_requests=plan_requests, plan_final=plan_final)
            _FakeBackend.planners.append(planner)
            return planner

        @staticmethod
        def auto_meander_search_config_rs(*args: object, **kwargs: object) -> dict[str, object]:
            return real_search_config_rs()(*args, **kwargs)

    return _FakeBackend


def planned_meander(requested: float, grid_rect: tuple[int, int, int, int]) -> dict[str, object]:
    """The plan dict a fake planner answers for one edge: `requested` inserted
    exactly, one bump on the left in a 10 um box at `grid_rect`."""
    return {
        "inserted_extra_length_um": float(requested),
        "effective_bend_radius_um": 4.0,
        "primitive_bend_radius_um": 4.0,
        "selected_box": (0.0, 10.0, 0.0, 10.0),
        "selected_grid_rect": grid_rect,
        "bumps": 1,
        "side": "left",
        "box_depth_um": 10.0,
        "box_depths_um": [10.0],
        "endpoint_inset_um": 4.0,
        "endpoint_insets_um": [4.0],
        "selected_run_start_index": 0,
        "selected_run_end_index": 1,
        "centerline": [(0.0, 0.0), (1.0, 0.0)],
        "planning_mode": "fill_box_multi_bump",
    }


def planned_result(
    plans: list[dict[str, object]],
    *,
    planning_mode: str,
    plan_input_indices: list[int] | None = None,
) -> dict[str, object]:
    """A planned result of the fake planner: candidate 0 planned `plans`
    (`split_part_count` for the selection pass, `plan_input_indices` for the
    final pass)."""
    result: dict[str, object] = {
        "status": "planned",
        "planning_mode": planning_mode,
        "selected_candidate_index": 0,
        "plans": plans,
        "candidate_results": [
            {
                "candidate_index": 0,
                "status": "planned",
                "reason": "",
                "failed_edge_index": None,
                "plans": plans,
            }
        ],
        "endpoint_inset_um": 4.0,
        "endpoint_insets_um": [4.0],
        "box_depths_um": [10.0],
    }
    if plan_input_indices is None:
        result["split_part_count"] = None
    else:
        result["plan_input_indices"] = plan_input_indices
    return result


class FakeLayout:
    """The layout `layout_from_schematic` is patched to return in the flow
    tests: a name, a bbox and the `info` dict the flow fills."""

    def __init__(self, name: str = "dummy_layout") -> None:
        self.name = name
        self.bbox = (0, 0, 1, 1)
        self.info: dict[str, object] = {}

    def show(self) -> None:
        return None


# --- the real backend -------------------------------------------------------


def real_backend() -> Any:
    backend = _load_rust_backend()
    if backend is None:
        pytest.skip("Rust backend unavailable.")
    return backend


def real_search_config_rs() -> Any:
    """The real backend's search policy for the fake backends."""
    return staticmethod(real_backend().auto_meander_search_config_rs)


def real_route_obj(
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    *,
    grid_size_um: float = 1.0,
    bend_radius_cells: int = 4,
    allow_45_degree_turns: bool = True,
) -> Any:
    """One net routed by the real router on an empty 256x256 grid."""
    rust_backend = real_backend()
    grid = rust_backend.GridSpec(256, 256, grid_size_um, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=grid_size_um,
        bend_radius_cells=bend_radius_cells,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    source = rust_backend.State(x0, y0, 0)
    target = rust_backend.State(x1, y1, 0)
    return router.route_single_net(source, target)


# --- reports and configs ----------------------------------------------------


def meander_report(
    results: list[MeanderEdgeReport],
    *,
    realized: dict[RoutedEdgeKey, float] | None = None,
) -> MeanderInsertionReport:
    """A meander report with the given results and realized placement."""
    requested = sum(entry.requested_extra_length_um for entry in results)
    inserted = sum(entry.inserted_extra_length_um for entry in results)
    return MeanderInsertionReport(
        results=results,
        total_requested_extra_length_um=requested,
        total_inserted_extra_length_um=inserted,
        total_physical_inserted_extra_length_um=inserted,
        total_disregarded_extra_length_um=0.0,
        unmatched_length_um=max(0.0, requested - inserted),
        realized_extra_by_edge=dict(realized or {}),
        minimum_insertable_extra_length_um=0.0,
    )


def search_config() -> _MeanderSearchConfig:
    return _MeanderSearchConfig(
        min_straight_um=1.0,
        min_segment_um=1.0,
        max_height_um=80.0,
        box_depths_um=[80.0],
        endpoint_inset_um=0.0,
        endpoint_insets_um=[0.0],
    )
