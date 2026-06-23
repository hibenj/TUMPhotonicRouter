"""Data structures for the electrical heater-routing stage."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from gdsfactory.component import Component

from photonic_router.routing_layers import HEATER_METAL_OBSTACLE_LAYERS, Layer

Point = tuple[float, float]
BBox = tuple[float, float, float, float]
GridCell = tuple[int, int]
GridPoint = tuple[float, float]
Side = Literal["top", "bottom"]


@dataclass(frozen=True)
class ElectricalRoutingConfig:
    """Configuration for the first electrical common-bus routing milestone.

    ``pad_side`` is the only user-facing side choice. The common bus is placed
    on the opposite side and later escapes back to the pad side.
    """

    pad_side: Side = "top"
    wire_width_um: float = 20.0
    bondpad_width_um: float = 80.0
    bondpad_length_um: float = 300.0
    bondpad_spacing_um: float = 50.0
    pad_pitch_um: float = 130.0
    pad_offset_um: float = 40.0
    pad_access_depth_um: float = 20.0
    pad_origin_x_um: float | None = None
    pad_empty_slots_between_assignments: int = 0
    pad_extra_slots_left: int = 0
    pad_extra_slots_right: int = 0
    common_bus_pad_position: Literal["left", "right"] = "right"
    routing_grid_pitch_um: float = 10.0
    metal_layer: Layer = (49, 0)
    heater_layers: tuple[Layer, ...] = ((47, 0),)
    metal_obstacle_layers: tuple[Layer, ...] = HEATER_METAL_OBSTACLE_LAYERS
    obstacle_clearance_um: float = 10.0
    terminal_open_radius_um: float = 15.0
    layout_margin_um: float = 80.0
    bus_offset_um: float = 60.0
    bus_width_um: float = 20.0
    bus_x_margin_um: float = 80.0
    common_bus_routing_strategy: Literal["greedy_tree", "local_trunk_then_greedy"] = (
        "local_trunk_then_greedy"
    )
    common_bus_terminal_selection: Literal[
        "path_cost",
        "median_x_biased",
        "local_pair_median_x_biased",
    ] = (
        "local_pair_median_x_biased"
    )
    common_bus_median_bias_weight: float = 1.0
    common_bus_local_pair_y_tolerance_um: float = 30.0
    common_bus_local_pair_max_gap_um: float = 800.0
    individual_route_spacing_um: float = 20.0
    heater_component_patterns: tuple[str, ...] = ("straight_heater_metal*",)
    heater_instance_prefixes: tuple[str, ...] = ("heater",)
    obstacle_mode: Literal["bounding_boxes", "rasterized_polygons"] = "bounding_boxes"
    clearance_metric: Literal["manhattan", "chebyshev"] = "chebyshev"

    @property
    def bus_side(self) -> Side:
        return opposite_side(self.pad_side)

    def validate(self) -> None:
        if self.pad_side not in {"top", "bottom"}:
            raise ValueError("pad_side must be 'top' or 'bottom'")
        if self.wire_width_um <= 0:
            raise ValueError("wire_width_um must be positive")
        if self.bondpad_width_um <= 0 or self.bondpad_length_um <= 0:
            raise ValueError("bondpad dimensions must be positive")
        if self.bondpad_spacing_um < 0:
            raise ValueError("bondpad_spacing_um must be non-negative")
        if self.pad_pitch_um < self.bondpad_width_um + self.bondpad_spacing_um:
            raise ValueError(
                "pad_pitch_um must be at least bondpad_width_um + bondpad_spacing_um"
            )
        if self.pad_offset_um < 0:
            raise ValueError("pad_offset_um must be non-negative")
        if self.pad_access_depth_um <= 0:
            raise ValueError("pad_access_depth_um must be positive")
        if self.pad_empty_slots_between_assignments < 0:
            raise ValueError("pad_empty_slots_between_assignments must be non-negative")
        if self.pad_extra_slots_left < 0 or self.pad_extra_slots_right < 0:
            raise ValueError("pad_extra_slots_left/right must be non-negative")
        if self.common_bus_pad_position not in {"left", "right"}:
            raise ValueError("common_bus_pad_position must be 'left' or 'right'")
        if self.routing_grid_pitch_um <= 0:
            raise ValueError("routing_grid_pitch_um must be positive")
        if self.obstacle_clearance_um < 0 or self.terminal_open_radius_um < 0:
            raise ValueError("clearances must be non-negative")
        if self.layout_margin_um < 0 or self.bus_offset_um < 0:
            raise ValueError("layout margins and offsets must be non-negative")
        if self.bus_width_um <= 0:
            raise ValueError("bus_width_um must be positive")
        if self.common_bus_routing_strategy not in {"greedy_tree", "local_trunk_then_greedy"}:
            raise ValueError(
                "common_bus_routing_strategy must be 'greedy_tree' or "
                "'local_trunk_then_greedy'"
            )
        if self.common_bus_terminal_selection not in {
            "path_cost",
            "median_x_biased",
            "local_pair_median_x_biased",
        }:
            raise ValueError(
                "common_bus_terminal_selection must be 'path_cost', "
                "'median_x_biased', or 'local_pair_median_x_biased'"
            )
        if self.common_bus_median_bias_weight < 0:
            raise ValueError("common_bus_median_bias_weight must be non-negative")
        if self.common_bus_local_pair_y_tolerance_um < 0:
            raise ValueError("common_bus_local_pair_y_tolerance_um must be non-negative")
        if self.common_bus_local_pair_max_gap_um < 0:
            raise ValueError("common_bus_local_pair_max_gap_um must be non-negative")
        if self.individual_route_spacing_um < 0:
            raise ValueError("individual_route_spacing_um must be non-negative")


@dataclass(frozen=True)
class ElectricalPortRef:
    """One physical electrical port belonging to a logical terminal."""

    name: str
    center: Point
    orientation: float | None
    width: float | None
    layer: Any


@dataclass(frozen=True)
class ElectricalTerminal:
    """Logical heater terminal, potentially represented by several gf ports."""

    id: str
    heater_id: str
    side_key: str
    center: Point
    bbox: BBox
    ports: tuple[ElectricalPortRef, ...]
    layer: Any = None


@dataclass(frozen=True)
class TerminalPairGroup:
    """The two interchangeable electrical terminals of one heater."""

    heater_id: str
    terminal_a: ElectricalTerminal
    terminal_b: ElectricalTerminal

    @property
    def terminals(self) -> tuple[ElectricalTerminal, ElectricalTerminal]:
        return (self.terminal_a, self.terminal_b)


@dataclass(frozen=True)
class BusStripe:
    """Fixed same-net bus stripe used as the common-bus root."""

    side: Side
    bbox: BBox
    cells: frozenset[GridCell]


@dataclass(frozen=True)
class ElectricalObstacleMap:
    """Layer-filtered obstacle grid for electrical routing."""

    grid: Any
    raw_blocked_cells: frozenset[GridCell]
    blocked_cells: frozenset[GridCell]
    terminal_open_cells: dict[str, frozenset[GridCell]]
    bus: BusStripe
    die_bbox: BBox
    layout_bbox: BBox
    raw_obstacle_bboxes: tuple[BBox, ...] = ()
    common_bus_terminal_open_cells: dict[str, frozenset[GridCell]] = field(
        default_factory=dict
    )
    individual_terminal_open_cells: dict[str, frozenset[GridCell]] = field(
        default_factory=dict
    )


@dataclass(frozen=True)
class TerminalBusRoute:
    heater_id: str
    terminal: ElectricalTerminal
    path: tuple[GridCell, ...]
    cost: int


@dataclass(frozen=True)
class CommonBusRoutingResult:
    bus_side: Side
    bus: BusStripe
    selected_terminals: dict[str, ElectricalTerminal]
    unselected_terminals: dict[str, ElectricalTerminal]
    routes: tuple[TerminalBusRoute, ...]
    tree_cells: frozenset[GridCell]
    failed_heaters: tuple[str, ...] = ()

    @property
    def success(self) -> bool:
        return not self.failed_heaters

    @property
    def total_cost(self) -> int:
        return sum(route.cost for route in self.routes)


@dataclass(frozen=True)
class PadSlot:
    """One legal abstract pad location on the configured pad-side pitch grid."""

    index: int
    center: Point
    bbox: BBox
    side: Side


@dataclass(frozen=True)
class PadAssignment:
    """Abstract assignment of one electrical net target to a pad slot."""

    slot: PadSlot
    net_id: str
    kind: Literal["common_bus", "individual"]
    terminal: ElectricalTerminal | None = None
    heater_id: str | None = None
    topology_bundle_id: int | None = None
    topology_rank: int | None = None


@dataclass(frozen=True)
class PadPlan:
    """Abstract pad-slot plan. Only assigned slots become physical pads later."""

    side: Side
    pitch_um: float
    origin_x_um: float
    slots: tuple[PadSlot, ...]
    assignments: tuple[PadAssignment, ...]
    empty_slots: tuple[PadSlot, ...]

    @property
    def assigned_slots(self) -> tuple[PadSlot, ...]:
        return tuple(assignment.slot for assignment in self.assignments)

    @property
    def common_bus_assignment(self) -> PadAssignment | None:
        for assignment in self.assignments:
            if assignment.kind == "common_bus":
                return assignment
        return None


@dataclass(frozen=True)
class CommonBusEscapeResult:
    """Abstract path from the common-bus tree to its assigned pad slot."""

    pad_assignment: PadAssignment | None
    path: tuple[GridCell, ...]
    target_cells: frozenset[GridCell]
    success: bool
    reason: str | None = None

    @property
    def cost(self) -> int:
        return max(0, len(self.path) - 1)


@dataclass(frozen=True)
class EscapeTopologyRoute:
    """Coarse path from one individual terminal to the configured pad side."""

    terminal: ElectricalTerminal
    path: tuple[GridCell, ...]
    exit_cell: GridCell | None
    success: bool
    reason: str | None = None

    @property
    def cost(self) -> int:
        return max(0, len(self.path) - 1)


@dataclass(frozen=True)
class EscapeBundle:
    """Topological one-sided escape bundle inferred before pad assignment."""

    bundle_id: int
    routes: tuple[EscapeTopologyRoute, ...]
    cells: frozenset[GridCell]
    shared_cells: frozenset[GridCell]
    exit_interval: tuple[int, int]
    ordered_terminals: tuple[ElectricalTerminal, ...]
    order_axis: Literal["x", "y"]
    required_tracks: int
    required_width_um: float


@dataclass(frozen=True)
class IndividualEscapeTopologyResult:
    """Coarse individual escape topology used for order-aware pad assignment."""

    routes: tuple[EscapeTopologyRoute, ...]
    failed_routes: tuple[EscapeTopologyRoute, ...]
    bundles: tuple[EscapeBundle, ...]
    terminal_order: tuple[ElectricalTerminal, ...] = ()
    cell_usage: dict[GridCell, int] = field(default_factory=dict)
    shared_cells: frozenset[GridCell] = frozenset()
    debug_info: dict[str, Any] = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return not self.failed_routes


@dataclass(frozen=True)
class DetailedBundleRoute:
    """Detailed centerline path for one topology-assigned individual route."""

    bundle_id: int
    rank: int
    terminal: ElectricalTerminal
    pad_assignment: PadAssignment | None
    path: tuple[GridCell, ...]
    target_cells: frozenset[GridCell]
    track_cell: GridCell | None
    lane_cell: GridCell | None
    offset_um: float
    offset_axis: Literal["x", "y"]
    offset_path: tuple[GridPoint, ...]
    success: bool
    reason: str | None = None
    source_stub_path: tuple[GridPoint, ...] = ()
    bundle_track_path: tuple[GridPoint, ...] = ()
    pad_stub_path: tuple[GridPoint, ...] = ()

    @property
    def cost(self) -> int:
        return max(0, len(self.path) - 1)


@dataclass(frozen=True)
class DetailedBundleRoutingResult:
    """Detailed non-overlapping centerline routes for topology bundles."""

    routes: tuple[DetailedBundleRoute, ...]
    failed_routes: tuple[DetailedBundleRoute, ...]
    committed_cells: frozenset[GridCell]
    cell_usage: dict[GridCell, int] = field(default_factory=dict)
    track_pitch_cells: int = 1

    @property
    def success(self) -> bool:
        return not self.failed_routes


@dataclass(frozen=True)
class ElectricalVerificationIssue:
    """One electrical routing verification issue."""

    code: str
    message: str
    severity: Literal["error", "warning"] = "error"
    net_id: str | None = None
    details: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ElectricalVerificationResult:
    """Post-route electrical correctness checks."""

    issues: tuple[ElectricalVerificationIssue, ...] = ()
    metrics: dict[str, Any] = field(default_factory=dict)

    @property
    def error_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "error")

    @property
    def warning_count(self) -> int:
        return sum(1 for issue in self.issues if issue.severity == "warning")

    @property
    def success(self) -> bool:
        return self.error_count == 0


@dataclass(frozen=True)
class ElectricalRoutingResult:
    """Electrical routing milestone result payload."""

    terminal_groups: tuple[TerminalPairGroup, ...]
    obstacle_map: ElectricalObstacleMap
    common_bus: CommonBusRoutingResult
    pad_plan: PadPlan | None = None
    common_bus_escape: CommonBusEscapeResult | None = None
    individual_topology: IndividualEscapeTopologyResult | None = None
    detailed_bundle_routes: DetailedBundleRoutingResult | None = None
    routed_component: Component | None = None
    verification: ElectricalVerificationResult | None = None
    debug_artifacts: dict[str, str] = field(default_factory=dict)


def opposite_side(side: Side) -> Side:
    if side == "top":
        return "bottom"
    if side == "bottom":
        return "top"
    raise ValueError("side must be 'top' or 'bottom'")
