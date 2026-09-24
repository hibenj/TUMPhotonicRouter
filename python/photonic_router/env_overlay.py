"""The one environment-variable overlay loader (owner decision D4 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`): every
`PHOTONIC_ROUTER_*` name from the Milestone 1 inventory -- Rust-side
(Slice 1) and Python-side (Slice 2) -- maps to exactly one field of
`photonic_router.config.RoutingConfig`, applied here and only here.
Everything else in the codebase takes a `RoutingConfig` (or, at the
Rust boundary, a `RouterConfig`) object explicitly; this module exists for
the benchmark modules' `STABLE_ROUTING_ENV` blocks, the reproduction
scripts, and ad-hoc experiments.

Each entry of `ENV_OVERLAY` is one `(name, path, parse)` triple: `path` is
an arbitrary-depth chain of group names into `RoutingConfig` ending in a
leaf field name (Slice 1's Rust-boundary entries are all rooted at
`("router", ...)`), and `parse` turns the raw string value into the field's
value. A variable is applied only when present in the environ mapping --
`apply_env_overlay` never reads an absent name -- which is how the "unset
differs from any value" trap (2026-09-07 repository memory) is respected:
an unset field keeps its dataclass default, not a parsed "empty" value.

`PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS`/`_S` (precedence + unit conversion) and
`PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT` (validated, raises) are not
simple one-name-one-parse entries and are applied by their own functions,
called after the table; `SPECIALLY_HANDLED_NAMES` names them for the
coverage test.
"""

from __future__ import annotations

import math
import os
from collections import namedtuple
from dataclasses import replace
from typing import Callable, Mapping

from photonic_router.config import ALL_NETS, RouterConfig

EnvVar = namedtuple("EnvVar", ["name", "path", "parse"])

_PREFIX = "PHOTONIC_ROUTER_"


# --- parsers -----------------------------------------------------------


def presence(_raw: str) -> bool:
    """The variable's mere presence (any value, including empty) is the
    signal; called only when the name is in the environ mapping."""
    return True


def exact_one(raw: str) -> bool:
    """`== "1"`."""
    return raw == "1"


def not_zero(raw: str) -> bool:
    """`!= "0"` (the field's own dataclass default carries the "true when
    unset" half of this rule)."""
    return raw != "0"


def enabled_words(raw: str) -> bool:
    """Lowercased value in `{1, true, on, yes, enabled}`."""
    return raw.strip().lower() in {"1", "true", "on", "yes", "enabled"}


def u64_positive_or_default(default: int) -> Callable[[str], int]:
    """Parse a positive integer; any other value (unparseable or <= 0)
    keeps `default`."""

    def parse(raw: str) -> int:
        try:
            value = int(raw.strip())
        except ValueError:
            return default
        return value if value > 0 else default

    return parse


def usize_or_default(default: int) -> Callable[[str], int]:
    """Parse an integer; an unparseable value keeps `default` (no
    positivity filter -- 0 is a legal value some of these fields use to
    mean "disabled")."""

    def parse(raw: str) -> int:
        try:
            return int(raw.strip())
        except ValueError:
            return default

    return parse


def usize_positive(raw: str) -> int | None:
    """Parse a positive integer; an unparseable or non-positive value maps
    to `None` (the field's own "no override" value, which the kernel's own
    default then fills)."""
    try:
        value = int(raw.strip())
    except ValueError:
        return None
    return value if value > 0 else None


def optional_int(raw: str) -> int | None:
    """Parse an integer; an unparseable value maps to `None`, never
    raises."""
    try:
        return int(raw.strip())
    except ValueError:
        return None


def i32(raw: str) -> int | None:
    """Alias of `optional_int` for the `i32`-typed Rust fields."""
    return optional_int(raw)


def f64_finite_nonneg(name: str) -> Callable[[str], float]:
    """Parse a finite, non-negative float; raises `ValueError` with the
    same message shape the Rust binding used to, on any invalid value."""

    def parse(raw: str) -> float:
        try:
            value = float(raw.strip())
        except ValueError as exc:
            raise ValueError(f"{name} must be a number") from exc
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    return parse


def comma_ints(raw: str) -> tuple[int, ...]:
    """Comma-separated ints; unparseable items are dropped (never raises,
    and returns an empty tuple rather than `None` when nothing parses --
    the caller only calls this when the variable is present, and presence
    alone must yield `Some([...])` on the Rust side)."""
    out: list[int] = []
    for item in raw.split(","):
        try:
            out.append(int(item.strip()))
        except ValueError:
            continue
    return tuple(out)


def cell_pair(raw: str) -> tuple[int, int] | None:
    """`"x,y"`; malformed input maps to `None`."""
    parts = raw.split(",")
    if len(parts) < 2:
        return None
    try:
        return (int(parts[0].strip()), int(parts[1].strip()))
    except ValueError:
        return None


def cell_pairs_semicolon(raw: str) -> tuple[tuple[int, int], ...]:
    """`"x,y;x,y;..."`; malformed items are dropped."""
    out: list[tuple[int, int]] = []
    for item in raw.split(";"):
        cell = cell_pair(item)
        if cell is not None:
            out.append(cell)
    return tuple(out)


def net_names_or_star(raw: str) -> str | tuple[str, ...]:
    """Comma-separated net names, or `ALL_NETS` ("*") if any item is
    exactly `"*"` (matching every net, same as the Rust site's per-item
    `item == "*" || item == net_id` -- one `"*"` anywhere makes every net
    match)."""
    items = [item.strip() for item in raw.split(",")]
    if any(item == ALL_NETS for item in items):
        return ALL_NETS
    return tuple(items)


# --- Milestone 1 Slice 2 parsers (Python-side `RoutingConfig` fields) ----


def raw_string(raw: str) -> str:
    """Identity: the field stores the raw value untouched, same as the
    site's own `os.environ.get(NAME, default)` (no `.strip()`); alias
    normalization/validation happens at the call site, unchanged."""
    return raw


def stripped_string(raw: str) -> str:
    """`raw.strip()`."""
    return raw.strip()


def string_or_unset(raw: str) -> str | None:
    """`raw.strip() or None` -- a blank value is "unset", matching sites
    that fall back to their own default with `.strip() or base.field`."""
    return raw.strip() or None


def nonempty_raw_or_none(raw: str) -> str | None:
    """`raw or None` (no `.strip()`), matching `if os.environ.get(NAME):
    ... os.environ[NAME]`."""
    return raw or None


def truthy(raw: str) -> bool:
    """`bool(raw)`: true whenever the raw string is non-empty, matching
    `if os.environ.get(NAME):`."""
    return bool(raw)


def truthy_stripped(raw: str) -> bool:
    """`bool(raw.strip())`."""
    return bool(raw.strip())


def plain_float(raw: str) -> float:
    """`float(raw)`; propagates Python's own `ValueError` on bad input,
    matching the site's unwrapped `float(os.environ.get(...))` call."""
    return float(raw)


def float_or_unset(name: str) -> Callable[[str], float | None]:
    """Parse a float; a blank (after `.strip()`) value is "unset" (`None`);
    any other unparseable value raises with the site's own message shape."""

    def parse(raw: str) -> float | None:
        text = raw.strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError as exc:
            raise ValueError(f"{name} must be a number, got {text!r}") from exc

    return parse


def nonnegative_int(name: str) -> Callable[[str], int | None]:
    """Parse a non-negative integer; a blank (after `.strip()`) value is
    "unset" (`None`); any other unparseable or negative value raises,
    matching `_env_nonnegative_int`'s message."""

    def parse(raw: str) -> int | None:
        text = raw.strip()
        if not text:
            return None
        try:
            value = int(text)
        except ValueError as exc:
            raise ValueError(f"{name} must be a non-negative integer") from exc
        if value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return value

    return parse


def dense_fanout_min_ports_override(name: str) -> Callable[[str], int | None]:
    """Parse `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`: blank (after
    `.strip()`) is "unset" (`None`); an unparseable value propagates
    Python's builtin `ValueError`; a parsed value below 2 raises."""

    def parse(raw: str) -> int | None:
        text = raw.strip()
        if not text:
            return None
        value = int(text)
        if value < 2:
            raise ValueError(f"{name} must be >= 2")
        return value

    return parse


def positive_int_or_raise(name: str) -> Callable[[str], int | None]:
    """Parse `PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT`: an empty raw string is
    "unset" (`None`); any other unparseable value or a value below 1
    raises."""

    def parse(raw: str) -> int | None:
        if not raw:
            return None
        try:
            value = int(raw)
        except ValueError as exc:
            raise ValueError(f"{name} must be an integer") from exc
        if value < 1:
            raise ValueError(f"{name} must be >= 1")
        return value

    return parse


def zero_one_crossing_budget(name: str) -> Callable[[str], bool]:
    """`"0"` -> unlimited (False), `"1"` -> one discounted crossing per
    planned pair (True); any other value raises."""

    def parse(raw: str) -> bool:
        value = raw.strip()
        if value == "1":
            return True
        if value == "0":
            return False
        raise ValueError(f"{name} must be 0 (unlimited) or 1 (one per pair)")

    return parse


def heap_tie_breaker_override(raw: str) -> str | None:
    """Only exactly `"smaller_g"` or `"larger_g"` (after `.strip()`)
    override; anything else means "no override" (`None`), matching the
    site's `if env_tie_breaker in {...}:` guard."""
    value = raw.strip()
    return value if value in {"smaller_g", "larger_g"} else None


def comma_digit_ints_frozenset(raw: str) -> frozenset[int]:
    """Comma-separated ints; non-digit tokens dropped (matches
    `v.strip().isdigit()`, so no negative numbers)."""
    return frozenset(int(v.strip()) for v in raw.split(",") if v.strip().isdigit())


def ordered_digit_ints_tuple(raw: str) -> tuple[int, ...]:
    """Comma-separated ints, order kept, non-digit tokens dropped."""
    return tuple(int(token.strip()) for token in raw.split(",") if token.strip().isdigit())


def comma_token_frozenset(raw: str) -> frozenset[str]:
    """Comma-separated tokens, stripped, empty tokens dropped."""
    return frozenset(item.strip() for item in raw.split(",") if item.strip())


def comma_names_frozenset_or_none(raw: str) -> frozenset[str] | None:
    """Comma-separated names; an empty result (blank raw, or only blank
    items) is "unset" (`None`), matching sites that guard with
    `if raw and instance_name in {...}:`."""
    names = frozenset(item.strip() for item in raw.split(",") if item.strip())
    return names or None


# --- the table -----------------------------------------------------------

ENV_OVERLAY: tuple[EnvVar, ...] = (
    # Negotiation (src/py_router.rs)
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_FIRST",
        ("router", "negotiation", "budget_first"),
        u64_positive_or_default(2_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_FIRST_RETRY",
        ("router", "negotiation", "budget_first_retry"),
        u64_positive_or_default(10_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_RETRY",
        ("router", "negotiation", "budget_retry"),
        u64_positive_or_default(30_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BRAID_ESCALATION",
        ("router", "negotiation", "braid_escalation"),
        exact_one,
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_CROSSING_FREE_UNPLANNED",
        ("router", "negotiation", "crossing_free_unplanned"),
        not_zero,
    ),
    EnvVar(
        _PREFIX + "DISABLE_BRAID_REPAIR",
        ("router", "negotiation", "disable_braid_repair"),
        presence,
    ),
    EnvVar(
        _PREFIX + "PENDING_STRAIGHT_RIPUP_THRESHOLD",
        ("router", "negotiation", "pending_straight_ripup_threshold"),
        usize_or_default(100),
    ),
    # Search overrides (src/py_router.rs) -- ASTAR_TIMEOUT_MS/S and
    # LONG_STRAIGHT_CONGESTION_WEIGHT are applied by their own functions
    # below (precedence/unit-conversion, or raising validation).
    EnvVar(
        _PREFIX + "MAX_DENSE_STATES",
        ("router", "search", "max_dense_states"),
        usize_positive,
    ),
    EnvVar(
        _PREFIX + "SEARCH_ENGINE",
        ("router", "search", "engine"),
        stripped_string,
    ),
    # Crossing engine (src/py_router.rs)
    EnvVar(
        _PREFIX + "ENABLE_GUIDED_COLLISION_CROSSING",
        ("router", "crossing", "enable_guided_collision_crossing"),
        presence,
    ),
    EnvVar(
        _PREFIX + "DISABLE_GUIDED_COLLISION_CROSSING",
        ("router", "crossing", "disable_guided_collision_crossing"),
        presence,
    ),
    EnvVar(
        _PREFIX + "DISABLE_RUST_CROSSING_VALIDATION",
        ("router", "crossing", "disable_rust_crossing_validation"),
        presence,
    ),
    # Diagnostics (src/py_router.rs)
    EnvVar(_PREFIX + "NATIVE_PROGRESS", ("router", "diagnostics", "native_progress"), presence),
    EnvVar(_PREFIX + "NATIVE_REPAIR_DIAG", ("router", "diagnostics", "native_repair_diag"), presence),
    EnvVar(
        _PREFIX + "TRACE_PLAIN_ROUTE_NET",
        ("router", "diagnostics", "trace_plain_route_net"),
        optional_int,
    ),
    EnvVar(_PREFIX + "TRACE_CROSSING", ("router", "diagnostics", "trace_crossing"), presence),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_NET", ("router", "diagnostics", "trace_crossing_net"), optional_int
    ),
    EnvVar(_PREFIX + "TRACE_PARTNER_NET", ("router", "diagnostics", "trace_partner_net"), optional_int),
    EnvVar(
        _PREFIX + "TRACE_ENDPOINT_BUMP_NETS",
        ("router", "diagnostics", "trace_endpoint_bump_nets"),
        net_names_or_star,
    ),
    EnvVar(
        _PREFIX + "TRACE_ENDPOINT_CORRECTION_NET",
        ("router", "diagnostics", "trace_endpoint_correction_net"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_DUMP",
        ("router", "diagnostics", "crossing_mismatch_dump"),
        presence,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_DUMP",
        ("router", "diagnostics", "crossing_mismatch_dump_net"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_FATAL",
        ("router", "diagnostics", "crossing_mismatch_fatal"),
        presence,
    ),
    # Diagnostics (src/astar.rs)
    EnvVar(
        _PREFIX + "ANALYSIS_CROSSING_PARTNER_COUNTERS",
        ("router", "diagnostics", "analysis_crossing_partner_counters"),
        enabled_words,
    ),
    EnvVar(_PREFIX + "CHAIN_DIAG", ("router", "diagnostics", "chain_diag"), presence),
    EnvVar(_PREFIX + "HOT_LOOP_TIMING", ("router", "diagnostics", "hot_loop_timing"), presence),
    EnvVar(_PREFIX + "MOVE_DIAG", ("router", "diagnostics", "move_diag"), presence),
    EnvVar(_PREFIX + "MOVE_DIAG", ("router", "diagnostics", "move_diag_cell"), cell_pair),
    EnvVar(_PREFIX + "POP_DIAG_BELOW_Y", ("router", "diagnostics", "pop_diag_below_y"), i32),
    EnvVar(_PREFIX + "PROBE_CELLS", ("router", "diagnostics", "probe_cells"), cell_pairs_semicolon),
    EnvVar(_PREFIX + "SEARCH_FAILURE_DIAG", ("router", "diagnostics", "search_failure_diag"), presence),
    EnvVar(
        _PREFIX + "SEARCH_FAILURE_MAP", ("router", "diagnostics", "search_failure_map"), comma_ints
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_CANDIDATE_MAX",
        ("router", "diagnostics", "trace_crossing_candidate_max"),
        usize_or_default(120),
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_CANDIDATES",
        ("router", "diagnostics", "trace_crossing_candidates"),
        presence,
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_LEVEL1", ("router", "diagnostics", "trace_crossing_level1"), presence
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PENDING", ("router", "diagnostics", "trace_crossing_pending"), presence
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PENDING_THRESHOLD",
        ("router", "diagnostics", "trace_crossing_pending_threshold"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PERP_REJECT_THRESHOLD",
        ("router", "diagnostics", "trace_crossing_perp_reject_threshold"),
        optional_int,
    ),
    # --- Milestone 1 Slice 2: Python-side RoutingConfig fields ------------
    # Crossing plan (translation/route_rust_crossing_plan.py)
    EnvVar(
        _PREFIX + "COLLISION_CROSSING_SEARCH_LOSS_UM",
        ("crossing_plan", "collision_crossing_search_loss_um"),
        f64_finite_nonneg(_PREFIX + "COLLISION_CROSSING_SEARCH_LOSS_UM"),
    ),
    EnvVar(
        _PREFIX + "PLANNED_CROSSING_SEARCH_LOSS_UM",
        ("crossing_plan", "planned_crossing_search_loss_um"),
        f64_finite_nonneg(_PREFIX + "PLANNED_CROSSING_SEARCH_LOSS_UM"),
    ),
    EnvVar(
        _PREFIX + "PLANNED_CROSSING_BUDGET",
        ("crossing_plan", "planned_crossing_budget"),
        zero_one_crossing_budget(_PREFIX + "PLANNED_CROSSING_BUDGET"),
    ),
    # Crossing grid (translation/preplaced_crossing_grids.py, crossing_structures.py)
    EnvVar(
        _PREFIX + "CROSSING_GRID_BAND_MARGIN_UM",
        ("crossing_grid", "band_margin_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_BAND_MARGIN_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_BEND_RADIUS_UM",
        ("crossing_grid", "bend_radius_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_BEND_RADIUS_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_COLUMN_LEAD_UM",
        ("crossing_grid", "column_lead_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_COLUMN_LEAD_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_COLUMN_PITCH_UM",
        ("crossing_grid", "column_pitch_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_COLUMN_PITCH_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_CORNER_MARGIN_UM",
        ("crossing_grid", "corner_margin_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_CORNER_MARGIN_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_ENTRY_STRAIGHT_UM",
        ("crossing_grid", "entry_straight_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_ENTRY_STRAIGHT_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_FAN_COLUMN_PITCH_UM",
        ("crossing_grid", "fan_column_pitch_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_FAN_COLUMN_PITCH_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_LANE_PITCH_UM",
        ("crossing_grid", "lane_pitch_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_LANE_PITCH_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_PORT_PAIR_SPREAD_UM",
        ("crossing_grid", "port_pair_spread_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_PORT_PAIR_SPREAD_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_SLOT_SPREAD_UM",
        ("crossing_grid", "slot_spread_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_SLOT_SPREAD_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_STUB_STAGGER_UM",
        ("crossing_grid", "stub_stagger_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_STUB_STAGGER_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_TILE_MIN_SPACING_UM",
        ("crossing_grid", "tile_min_spacing_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_TILE_MIN_SPACING_UM"),
    ),
    EnvVar(
        _PREFIX + "CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM",
        ("crossing_grid", "unrouted_sibling_clearance_um"),
        float_or_unset(_PREFIX + "CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM"),
    ),
    EnvVar(_PREFIX + "CROSSING_GRID_FAN_MODE", ("crossing_grid", "fan_mode"), string_or_unset),
    EnvVar(
        _PREFIX + "CROSSING_GRID_TILE_PLACEMENT",
        ("crossing_grid", "tile_placement"),
        string_or_unset,
    ),
    EnvVar(_PREFIX + "CROSSING_GRID_CORNERS", ("crossing_grid", "corners"), string_or_unset),
    EnvVar(
        _PREFIX + "CROSSING_GRID_ROUTER_LAYERS",
        ("crossing_grid", "router_layers"),
        comma_digit_ints_frozenset,
    ),
    EnvVar(
        _PREFIX + "TRACE_COLUMN_GRID", ("crossing_grid", "trace_column_grid"), truthy
    ),
    # Fan-out access (translation/route_rust.py)
    EnvVar(
        _PREFIX + "DENSE_FANOUT_INSTANCES",
        ("fanout", "dense_fanout_instances"),
        comma_names_frozenset_or_none,
    ),
    EnvVar(
        _PREFIX + "DENSE_FANOUT_MIN_PORTS",
        ("fanout", "dense_fanout_min_ports"),
        dense_fanout_min_ports_override(_PREFIX + "DENSE_FANOUT_MIN_PORTS"),
    ),
    EnvVar(_PREFIX + "FANOUT_ACCESS_MODE", ("fanout", "fanout_access_mode"), raw_string),
    EnvVar(
        _PREFIX + "FANOUT_LANE_SPACING_CELLS",
        ("fanout", "fanout_lane_spacing_cells"),
        nonnegative_int(_PREFIX + "FANOUT_LANE_SPACING_CELLS"),
    ),
    EnvVar(
        _PREFIX + "FANOUT_PROTECTED_LANE_SPACING_CELLS",
        ("fanout", "fanout_protected_lane_spacing_cells"),
        nonnegative_int(_PREFIX + "FANOUT_PROTECTED_LANE_SPACING_CELLS"),
    ),
    EnvVar(
        _PREFIX + "TARGET_PROTECTED_LANE_SPACING_CELLS",
        ("fanout", "target_protected_lane_spacing_cells"),
        nonnegative_int(_PREFIX + "TARGET_PROTECTED_LANE_SPACING_CELLS"),
    ),
    EnvVar(
        _PREFIX + "FANOUT_STUB_BEND_DEGREES", ("fanout", "fanout_stub_bend_degrees"), raw_string
    ),
    EnvVar(
        _PREFIX + "FANOUT_STUB_FORWARD_CELLS",
        ("fanout", "fanout_stub_forward_cells"),
        nonnegative_int(_PREFIX + "FANOUT_STUB_FORWARD_CELLS"),
    ),
    EnvVar(
        _PREFIX + "FANOUT_STUB_X_OFFSET_CELLS",
        ("fanout", "fanout_stub_x_offset_cells"),
        nonnegative_int(_PREFIX + "FANOUT_STUB_X_OFFSET_CELLS"),
    ),
    EnvVar(
        _PREFIX + "STUB_PORT_LANE_HALF_WIDTH_CELLS",
        ("fanout", "stub_port_lane_half_width_cells"),
        nonnegative_int(_PREFIX + "STUB_PORT_LANE_HALF_WIDTH_CELLS"),
    ),
    EnvVar(
        _PREFIX + "STUB_PORT_LANE_LENGTH_CELLS",
        ("fanout", "stub_port_lane_length_cells"),
        nonnegative_int(_PREFIX + "STUB_PORT_LANE_LENGTH_CELLS"),
    ),
    # Search tuning (translation/route_rust.py)
    EnvVar(_PREFIX + "MIN_BEND_WEIGHT", ("search", "min_bend_weight"), plain_float),
    EnvVar(_PREFIX + "MIN_HEURISTIC_WEIGHT", ("search", "min_heuristic_weight"), plain_float),
    EnvVar(
        _PREFIX + "HEAP_TIE_BREAKER", ("search", "heap_tie_breaker"), heap_tie_breaker_override
    ),
    EnvVar(
        _PREFIX + "LONG_STRAIGHT_EXEMPT_DENSE_FANOUT",
        ("search", "long_straight_exempt_dense_fanout"),
        exact_one,
    ),
    # Flow diagnostics (translation/route_rust.py, route_rust_endpoint_correction.py)
    EnvVar(
        _PREFIX + "DEBUG_EXECUTION_LIMIT",
        ("diagnostics", "debug_execution_limit"),
        positive_int_or_raise(_PREFIX + "DEBUG_EXECUTION_LIMIT"),
    ),
    EnvVar(
        _PREFIX + "DEBUG_ROUTE_FIRST_INSTANCE",
        ("diagnostics", "debug_route_first_instance"),
        stripped_string,
    ),
    EnvVar(
        _PREFIX + "DEBUG_ROUTE_FIRST_NETS",
        ("diagnostics", "debug_route_first_nets"),
        ordered_digit_ints_tuple,
    ),
    EnvVar(
        _PREFIX + "TRACE_ENDPOINT_CORRECTION_NETS",
        ("diagnostics", "trace_endpoint_correction_nets"),
        comma_token_frozenset,
    ),
    EnvVar(
        _PREFIX + "TRACE_FANOUT_STUBS", ("diagnostics", "trace_fanout_stubs"), truthy_stripped
    ),
    EnvVar(_PREFIX + "TRACE_GRID", ("diagnostics", "trace_grid"), truthy),
    EnvVar(
        _PREFIX + "TRACE_RUNWAY_INSTANCE",
        ("diagnostics", "trace_runway_instance"),
        nonempty_raw_or_none,
    ),
    EnvVar(
        _PREFIX + "TRACE_TERMINAL_BUMP_DISTANCE_CHECKS",
        ("diagnostics", "trace_terminal_bump_distance_checks"),
        comma_token_frozenset,
    ),
    # Debug artifact (routing_flow_verification.py)
    EnvVar(
        _PREFIX + "WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE",
        ("write_gds_on_photonic_verification_failure",),
        lambda raw: raw.strip().lower() in {"1", "true", "yes", "on"},
    ),
)

#: Names applied by their own function rather than a table entry
#: (precedence, unit conversion, or validation that raises).
SPECIALLY_HANDLED_NAMES: frozenset[str] = frozenset(
    {
        _PREFIX + "ASTAR_TIMEOUT_MS",
        _PREFIX + "ASTAR_TIMEOUT_S",
        _PREFIX + "LONG_STRAIGHT_CONGESTION_WEIGHT",
    }
)


def _replace_path(config: object, path: tuple[str, ...], value: object) -> object:
    """Set the field named by `path` (arbitrary depth: a leaf name, or a
    chain of group names ending in a leaf) on `config`, rebuilding every
    frozen dataclass along the way with `dataclasses.replace`."""
    head, *rest = path
    if not rest:
        return replace(config, **{head: value})
    child = getattr(config, head)
    return replace(config, **{head: _replace_path(child, tuple(rest), value)})


def _apply_astar_timeout(config: RouterConfig, environ: Mapping[str, str]) -> RouterConfig:
    """`PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS` (integer ms) takes precedence over
    `PHOTONIC_ROUTER_ASTAR_TIMEOUT_S` (finite, non-negative seconds,
    converted to ms, rounded up); either raises `ValueError` on an invalid
    value, matching the Rust binding's error messages."""
    ms_raw = environ.get(_PREFIX + "ASTAR_TIMEOUT_MS")
    if ms_raw is not None:
        try:
            ms = int(ms_raw.strip())
        except ValueError as exc:
            raise ValueError(
                f"{_PREFIX}ASTAR_TIMEOUT_MS must be an integer"
            ) from exc
        return _replace_path(config, ("search", "astar_timeout_ms"), ms)
    s_raw = environ.get(_PREFIX + "ASTAR_TIMEOUT_S")
    if s_raw is not None:
        try:
            seconds = float(s_raw.strip())
        except ValueError as exc:
            raise ValueError(f"{_PREFIX}ASTAR_TIMEOUT_S must be a number") from exc
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError(
                f"{_PREFIX}ASTAR_TIMEOUT_S must be finite and non-negative"
            )
        ms = math.ceil(seconds * 1000.0)
        return _replace_path(config, ("search", "astar_timeout_ms"), ms)
    return config


def _apply_long_straight_congestion_weight(
    config: RouterConfig, environ: Mapping[str, str]
) -> RouterConfig:
    name = _PREFIX + "LONG_STRAIGHT_CONGESTION_WEIGHT"
    raw = environ.get(name)
    if raw is None:
        return config
    value = f64_finite_nonneg(name)(raw)
    return _replace_path(config, ("search", "long_straight_congestion_weight"), value)


def apply_env_overlay(config: object, environ: Mapping[str, str] | None = None) -> object:
    """Apply every present `PHOTONIC_ROUTER_*` name of `ENV_OVERLAY` plus
    the two specially-handled ones onto `config`, returning a new config of
    the same type. A variable absent from `environ` is never applied -- its
    field keeps whatever `config` already carried.

    `ENV_OVERLAY`'s paths are rooted at `RoutingConfig` (Milestone 1 Slice
    2): the Rust-boundary entries (Slice 1) all start with `"router"`.
    Called with a `RoutingConfig`, every entry applies at its own path.
    Called with a bare `RouterConfig` (`RouterConfig.from_environment`,
    kept working for callers that only ever construct/overlay that
    subtree), only the `"router"`-rooted entries apply, with that first
    path element stripped.
    """
    if environ is None:
        environ = os.environ
    is_router_config = isinstance(config, RouterConfig)
    for entry in ENV_OVERLAY:
        if entry.name not in environ:
            continue
        path = entry.path
        if is_router_config:
            if path[0] != "router":
                continue
            path = path[1:]
        value = entry.parse(environ[entry.name])
        config = _replace_path(config, path, value)
    if is_router_config:
        config = _apply_astar_timeout(config, environ)
        config = _apply_long_straight_congestion_weight(config, environ)
    else:
        config = replace(config, router=_apply_astar_timeout(config.router, environ))
        config = replace(
            config, router=_apply_long_straight_congestion_weight(config.router, environ)
        )
    return config


__all__ = [
    "ENV_OVERLAY",
    "EnvVar",
    "SPECIALLY_HANDLED_NAMES",
    "apply_env_overlay",
    "cell_pair",
    "cell_pairs_semicolon",
    "comma_digit_ints_frozenset",
    "comma_ints",
    "comma_names_frozenset_or_none",
    "comma_token_frozenset",
    "dense_fanout_min_ports_override",
    "enabled_words",
    "exact_one",
    "f64_finite_nonneg",
    "float_or_unset",
    "heap_tie_breaker_override",
    "i32",
    "net_names_or_star",
    "nonempty_raw_or_none",
    "nonnegative_int",
    "not_zero",
    "optional_int",
    "ordered_digit_ints_tuple",
    "plain_float",
    "positive_int_or_raise",
    "presence",
    "raw_string",
    "string_or_unset",
    "stripped_string",
    "truthy",
    "truthy_stripped",
    "u64_positive_or_default",
    "usize_or_default",
    "usize_positive",
    "zero_one_crossing_budget",
]
