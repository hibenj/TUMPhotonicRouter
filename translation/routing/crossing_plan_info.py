"""`CrossingPlanInfo`: the crossing plan and its realized-crossing record, typed.

Milestone 5 Slice 2c. This was the one untyped boundary left in the session --
`SessionState.crossing_plan_info` used to be a `dict[str, Any]` that four phases
wrote into and five read from, with the key names themselves as the only
contract. Every key the pipeline ever sets is a field here, grouped by the phase
that writes it and named in the field docstring.

Absence is part of that contract: readers used `dict.get(key, default)` and the
crossing debug JSON / the verification reports only carry the keys that were
actually written, so a key that was never set must not appear. `None` is that
"not written yet" marker for every optional field, `to_dict()` drops those
fields, and `from_dict()` restores exactly what it was given. The thirteen
fields `translation.route_rust_crossing_plan._build_crossing_plan_info` always
writes are therefore not optional and are always emitted.

`to_dict()` emits the keys in this class's declaration order, which is the
pipeline's phase order. The dict it replaces was ordered by whichever write
happened first, and that order was not one order: whether the realized-crossing
keys precede the reconciliation and insertion-loss ones depends on whether
phase 7's endpoint correction found crossing events at all. Nothing observes the
order -- every artifact that carries the plan (the crossing debug JSON, the
crossing and photonic verification reports, `metrics.json`) is dumped with
`sort_keys=True` -- so the keys and their values are the identity that is kept.

`to_dict()` is what the pipeline hands to the boundaries that still take a plain
mapping: `RustRouteDebugArtifacts.crossing_plan_info` (and through it
`routing_flow_verification` and `translation.crossing_verification_report`), the
`{prefix}_crossings.json` debug artifact, and the realization/endpoint-correction
readers. Those dumps are unchanged by this slice, key for key.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, fields
from typing import Any


@dataclass
class CrossingPlanInfo:
    """The crossing plan, its realized crossings and its per-phase diagnostics.

    Mutable on purpose: phase 4 builds it, phases 7, 8 and 9 add to it (the
    `# shared writer` entry in `SessionState`). `w:` in each field docstring is
    the phase that writes the field.
    """

    # ---- always written by `_build_crossing_plan_info` (phase 4) ----------
    enabled: bool = False
    """Whether crossing-aware routing is on at all. w:4"""

    constraint_count: int = 0
    """Hard window-mode crossing constraints handed to the router. w:4"""

    event_count: int = 0
    """Planned crossing events in the topology plan. w:4"""

    missing_event_count: int = 0
    """Planned events whose edges have no route job. w:4"""

    missing_events: list[dict[str, Any]] = field(default_factory=list)
    """Those events, one record each. w:4"""

    events: list[dict[str, Any]] = field(default_factory=list)
    """Every planned event, loaded or not (`loaded` flag per record). w:4"""

    expected_crossings_by_net_id: dict[int, int] = field(default_factory=dict)
    """Planned crossing count per net id (used by the plan-crossings net orders). w:4"""

    expected_crossings_by_net_name: dict[str, int] = field(default_factory=dict)
    """The same counts keyed by net name. w:4"""

    crossing_loss: float = 0.0
    """Reported per-crossing insertion loss (dB). w:4"""

    crossing_search_loss: float = 0.0
    """Per-crossing price the A* search pays. w:4"""

    crossing_half_size_cells: int = 0
    """Half the crossing footprint, in grid cells. w:4"""

    min_straight_cells_per_crossing: int = 0
    """Straight cells the router must keep on each arm of a crossing. w:4"""

    allow_only_expected_crossings: bool = True
    """Whether the search may only cross planned pairs. w:4

    `True` by default because that was the default the realized-crossing
    verifier used for a plan mapping without the key; the builder always writes
    the run's own resolved value.
    """

    # ---- optional, `_build_crossing_plan_info` (phase 4) ------------------
    reason: str | None = None
    """Why no plan was built (lidar-pure mode, missing/invalid metadata). w:4"""

    error: str | None = None
    """The topology error message when the plan could not be built. w:4"""

    stage_count: int | None = None
    """Stages in the topology plan. w:4"""

    plan_text: str | None = None
    """The plan rendered for `{prefix}_crossings.txt`. w:4"""

    guidance: dict[str, Any] | None = None
    """Contribution 1's soft guidance actually handed to the router. w:4"""

    # ---- the crossing plan stage's own additions (phase 4) ----------------
    crossing_mode: str | None = None
    """The configured crossing mode. w:4"""

    requested_allow_only_expected_crossings: bool | None = None
    """What the caller asked for, before mode-specific resolution. w:4"""

    bend_runout_cells_per_crossing: int | None = None
    """Straight run-out cells a crossing needs for its bends. w:4"""

    fanout_stub_bend_degrees: int | None = None
    """Bend angle of the static fan-out stubs. w:4"""

    required_straight_margin_cells_per_crossing: int | None = None
    """Half footprint plus bend run-out: the margin the verifier requires. w:4"""

    fanout_access_mode: str | None = None
    """Resolved fan-out access mode. w:4"""

    fanout_anchor_port_count: int | None = None
    """Ports that got a static fan-out anchor. w:4"""

    fanout_anchor_net_ids: list[int] | None = None
    """Nets with a fan-out stub on either end. w:4"""

    fanout_anchor_source_net_ids: list[int] | None = None
    """Nets with a fan-out stub on the source end. w:4"""

    fanout_anchor_target_net_ids: list[int] | None = None
    """Nets with a fan-out stub on the target end. w:4"""

    fanout_stub_center_cell_count: int | None = None
    """Centerline cells of all static stubs. w:4"""

    fanout_stub_static_cell_count: int | None = None
    """Inflated static cells of all stubs. w:4"""

    fanout_stub_centerlines_um: list[dict[str, Any]] | None = None
    """One centerline record per fan-out anchor. w:4"""

    crossing_device: dict[str, Any] | None = None
    """The PDK crossing component's name and footprint. w:4"""

    # ---- the terminal-bump diagnostics (phase 7, finalize) ----------------
    terminal_bump_target_x_offset_nets: list[dict[str, Any]] | None = None
    """Nets whose vertical target approach carries an x offset. w:7"""

    terminal_bump_target_x_offset_net_count: int | None = None
    """How many. w:7"""

    terminal_bump_target_y_offset_nets: list[dict[str, Any]] | None = None
    """Nets whose horizontal target approach carries a y offset. w:7"""

    terminal_bump_target_y_offset_net_count: int | None = None
    """How many. w:7"""

    terminal_bump_distance_checks: list[dict[str, Any]] | None = None
    """Where a terminal bump distance check would become active. w:7"""

    terminal_bump_distance_check_count: int | None = None
    """How many. w:7"""

    terminal_bump_distance_failures: list[dict[str, Any]] | None = None
    """Those checks that would not be satisfied. w:7"""

    terminal_bump_distance_failure_count: int | None = None
    """How many. w:7"""

    # ---- the realized crossings (phases 7 and 8) -------------------------
    realized_intersections: list[dict[str, Any]] | None = None
    """Every realized crossing with its physical metadata. w:7,8"""

    realized_intersection_count: int | None = None
    """How many. w:7,8"""

    routes_missing_corrected_centerline: list[dict[str, Any]] | None = None
    """Routes the verifier could not check for lack of a centerline. w:7,8"""

    routes_missing_corrected_centerline_count: int | None = None
    """How many. w:7,8"""

    ignored_endpoint_access_intersections: list[dict[str, Any]] | None = None
    """Intersections excused as endpoint access. w:7,8"""

    ignored_endpoint_access_intersection_count: int | None = None
    """How many. w:7,8"""

    illegal_realized_crossings: list[dict[str, Any]] | None = None
    """Realized crossings the geometry verifier rejects. w:7,8"""

    illegal_realized_crossing_count: int | None = None
    """How many. w:7,8"""

    # ---- the A*-native crossing events and the repair record (phase 8) ---
    native_crossing_events: list[Any] | None = None
    """The crossing moves A* accepted, as the kernel reported them. w:8"""

    native_crossing_event_count: int | None = None
    """How many. w:8"""

    final_crossing_repair_attempts: list[dict[str, Any]] | None = None
    """One record per final illegal-crossing repair attempt. w:8"""

    final_photonic_repair_attempts: list[dict[str, Any]] | None = None
    """One record per final photonic-issue repair attempt. w:8"""

    photonic_probe_failure_artifacts: dict[str, str] | None = None
    """Paths of the artifacts dumped for a failed photonic probe. w:8"""

    # ---- the insertion-loss report (phase 8) -----------------------------
    insertion_loss_model: dict[str, Any] | None = None
    """The loss formula and its coefficients. w:8"""

    insertion_loss_summary: dict[str, Any] | None = None
    """Totals over all nets. w:8"""

    insertion_loss_by_net: list[dict[str, Any]] | None = None
    """Per-net length, bends, crossings and loss. w:8"""

    # ---- plan-versus-realized reconciliation (phase 8) -------------------
    actual_crossing_count: int | None = None
    """Planned events that are realized as a crossing. w:8"""

    actual_crossing_cell_count: int | None = None
    """Core-cell overlap of those crossings. w:8"""

    actual_geometric_crossing_count: int | None = None
    """Those that are a real segment intersection. w:8"""

    actual_crossings: list[dict[str, Any]] | None = None
    """Those crossings, one record each. w:8"""

    unrealized_expected_crossings: list[dict[str, Any]] | None = None
    """Planned events that did not become a crossing. w:8"""

    unrealized_expected_crossing_count: int | None = None
    """How many. w:8"""

    actual_crossing_reason: str | None = None
    """Why the reconciliation could not run. w:8"""

    # ---- the placed crossing components (phase 9, realize) ---------------
    realized_crossing_components: list[dict[str, Any]] | None = None
    """One record per placed PDK crossing reference. w:9"""

    realized_crossing_component_count: int | None = None
    """How many. w:9"""

    realized_crossing_component_error: str | None = None
    """Why no component could be placed. w:9"""

    def to_dict(self) -> dict[str, Any]:
        """The plan as the pipeline's mapping consumers see it.

        Exactly the dict the pipeline used to carry: the thirteen always-written
        fields plus every optional field that has been written, in insertion
        order, with unwritten fields absent.
        """

        values: dict[str, Any] = {}
        for entry in fields(self):
            value = getattr(self, entry.name)
            if value is None and entry.name not in _ALWAYS_PRESENT:
                continue
            values[entry.name] = value
        return values

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> CrossingPlanInfo:
        """Adopt a plan mapping -- `_build_crossing_plan_info`'s output, or one
        round-tripped through `RustRouteDebugArtifacts`.

        Unknown keys are an error rather than silently dropped data: every key
        the pipeline writes has a field here, so a new key means a new field.
        """

        unknown = sorted(set(values) - _FIELD_NAMES)
        if unknown:
            raise ValueError(
                f"CrossingPlanInfo has no field for crossing plan key(s): {unknown}. "
                "Add a field (and its writing phase) in translation/routing/"
                "crossing_plan_info.py."
            )
        return cls(**dict(values))


_FIELD_NAMES: frozenset[str] = frozenset(entry.name for entry in fields(CrossingPlanInfo))
_ALWAYS_PRESENT: frozenset[str] = frozenset(
    entry.name
    for entry in fields(CrossingPlanInfo)
    if entry.name
    in {
        "enabled",
        "constraint_count",
        "event_count",
        "missing_event_count",
        "missing_events",
        "events",
        "expected_crossings_by_net_id",
        "expected_crossings_by_net_name",
        "crossing_loss",
        "crossing_search_loss",
        "crossing_half_size_cells",
        "min_straight_cells_per_crossing",
        "allow_only_expected_crossings",
    }
)
