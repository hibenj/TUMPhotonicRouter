"""The one environment-variable overlay loader (owner decision D4 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`): every
Rust-side `PHOTONIC_ROUTER_*` name from the Milestone 1 inventory maps to
exactly one field of `photonic_router.config.RouterConfig`, applied here and
only here. Everything else in the codebase takes a `RouterConfig` object
explicitly; this module exists for the benchmark modules' `STABLE_ROUTING_ENV`
blocks, the reproduction scripts, and ad-hoc experiments.

Each entry of `ENV_OVERLAY` is one `(name, path, parse)` triple: `path` is
`(group, field)` into `RouterConfig`, and `parse` turns the raw string value
into the field's value. A variable is applied only when present in the
environ mapping -- `apply_env_overlay` never reads an absent name -- which
is how the "unset differs from any value" trap (2026-09-07 repository
memory) is respected: an unset field keeps its dataclass default, not a
parsed "empty" value.

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


# --- the table -----------------------------------------------------------

ENV_OVERLAY: tuple[EnvVar, ...] = (
    # Negotiation (src/py_router.rs)
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_FIRST",
        ("negotiation", "budget_first"),
        u64_positive_or_default(2_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_FIRST_RETRY",
        ("negotiation", "budget_first_retry"),
        u64_positive_or_default(10_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BUDGET_RETRY",
        ("negotiation", "budget_retry"),
        u64_positive_or_default(30_000_000),
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_BRAID_ESCALATION",
        ("negotiation", "braid_escalation"),
        exact_one,
    ),
    EnvVar(
        _PREFIX + "NEGOTIATED_CROSSING_FREE_UNPLANNED",
        ("negotiation", "crossing_free_unplanned"),
        not_zero,
    ),
    EnvVar(
        _PREFIX + "DISABLE_BRAID_REPAIR",
        ("negotiation", "disable_braid_repair"),
        presence,
    ),
    EnvVar(
        _PREFIX + "PENDING_STRAIGHT_RIPUP_THRESHOLD",
        ("negotiation", "pending_straight_ripup_threshold"),
        usize_or_default(100),
    ),
    EnvVar(
        _PREFIX + "ENABLE_ORTHOGONAL_REPAIR_FALLBACK",
        ("negotiation", "enable_orthogonal_repair_fallback"),
        presence,
    ),
    # Search overrides (src/py_router.rs) -- ASTAR_TIMEOUT_MS/S and
    # LONG_STRAIGHT_CONGESTION_WEIGHT are applied by their own functions
    # below (precedence/unit-conversion, or raising validation).
    EnvVar(
        _PREFIX + "MAX_DENSE_STATES",
        ("search", "max_dense_states"),
        usize_positive,
    ),
    # Crossing engine (src/py_router.rs)
    EnvVar(
        _PREFIX + "ENABLE_GUIDED_COLLISION_CROSSING",
        ("crossing", "enable_guided_collision_crossing"),
        presence,
    ),
    EnvVar(
        _PREFIX + "DISABLE_GUIDED_COLLISION_CROSSING",
        ("crossing", "disable_guided_collision_crossing"),
        presence,
    ),
    EnvVar(
        _PREFIX + "DISABLE_RUST_CROSSING_VALIDATION",
        ("crossing", "disable_rust_crossing_validation"),
        presence,
    ),
    # Diagnostics (src/py_router.rs)
    EnvVar(_PREFIX + "NATIVE_PROGRESS", ("diagnostics", "native_progress"), presence),
    EnvVar(_PREFIX + "NATIVE_REPAIR_DIAG", ("diagnostics", "native_repair_diag"), presence),
    EnvVar(
        _PREFIX + "TRACE_PLAIN_ROUTE_NET",
        ("diagnostics", "trace_plain_route_net"),
        optional_int,
    ),
    EnvVar(_PREFIX + "TRACE_CROSSING", ("diagnostics", "trace_crossing"), presence),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_NET", ("diagnostics", "trace_crossing_net"), optional_int
    ),
    EnvVar(_PREFIX + "TRACE_PARTNER_NET", ("diagnostics", "trace_partner_net"), optional_int),
    EnvVar(
        _PREFIX + "TRACE_ENDPOINT_BUMP_NETS",
        ("diagnostics", "trace_endpoint_bump_nets"),
        net_names_or_star,
    ),
    EnvVar(
        _PREFIX + "TRACE_ENDPOINT_CORRECTION_NET",
        ("diagnostics", "trace_endpoint_correction_net"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_DUMP",
        ("diagnostics", "crossing_mismatch_dump"),
        presence,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_DUMP",
        ("diagnostics", "crossing_mismatch_dump_net"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "CROSSING_MISMATCH_FATAL",
        ("diagnostics", "crossing_mismatch_fatal"),
        presence,
    ),
    # Diagnostics (src/astar.rs)
    EnvVar(
        _PREFIX + "ANALYSIS_CROSSING_PARTNER_COUNTERS",
        ("diagnostics", "analysis_crossing_partner_counters"),
        enabled_words,
    ),
    EnvVar(_PREFIX + "CHAIN_DIAG", ("diagnostics", "chain_diag"), presence),
    EnvVar(_PREFIX + "HOT_LOOP_TIMING", ("diagnostics", "hot_loop_timing"), presence),
    EnvVar(_PREFIX + "MOVE_DIAG", ("diagnostics", "move_diag"), presence),
    EnvVar(_PREFIX + "MOVE_DIAG", ("diagnostics", "move_diag_cell"), cell_pair),
    EnvVar(_PREFIX + "POP_DIAG_BELOW_Y", ("diagnostics", "pop_diag_below_y"), i32),
    EnvVar(_PREFIX + "PROBE_CELLS", ("diagnostics", "probe_cells"), cell_pairs_semicolon),
    EnvVar(_PREFIX + "SEARCH_FAILURE_DIAG", ("diagnostics", "search_failure_diag"), presence),
    EnvVar(
        _PREFIX + "SEARCH_FAILURE_MAP", ("diagnostics", "search_failure_map"), comma_ints
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_CANDIDATE_MAX",
        ("diagnostics", "trace_crossing_candidate_max"),
        usize_or_default(120),
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_CANDIDATES",
        ("diagnostics", "trace_crossing_candidates"),
        presence,
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_LEVEL1", ("diagnostics", "trace_crossing_level1"), presence
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PENDING", ("diagnostics", "trace_crossing_pending"), presence
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PENDING_THRESHOLD",
        ("diagnostics", "trace_crossing_pending_threshold"),
        optional_int,
    ),
    EnvVar(
        _PREFIX + "TRACE_CROSSING_PERP_REJECT_THRESHOLD",
        ("diagnostics", "trace_crossing_perp_reject_threshold"),
        optional_int,
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


def _replace_path(config: RouterConfig, path: tuple[str, str], value: object) -> RouterConfig:
    group_name, field_name = path
    group = getattr(config, group_name)
    new_group = replace(group, **{field_name: value})
    return replace(config, **{group_name: new_group})


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


def apply_env_overlay(
    config: RouterConfig, environ: Mapping[str, str] | None = None
) -> RouterConfig:
    """Apply every present `PHOTONIC_ROUTER_*` name of `ENV_OVERLAY` plus
    the two specially-handled ones onto `config`, returning a new
    `RouterConfig`. A variable absent from `environ` is never applied --
    its field keeps whatever `config` already carried."""
    if environ is None:
        environ = os.environ
    for entry in ENV_OVERLAY:
        if entry.name not in environ:
            continue
        value = entry.parse(environ[entry.name])
        config = _replace_path(config, entry.path, value)
    config = _apply_astar_timeout(config, environ)
    config = _apply_long_straight_congestion_weight(config, environ)
    return config


__all__ = [
    "ENV_OVERLAY",
    "EnvVar",
    "SPECIALLY_HANDLED_NAMES",
    "apply_env_overlay",
    "cell_pair",
    "cell_pairs_semicolon",
    "comma_ints",
    "enabled_words",
    "exact_one",
    "f64_finite_nonneg",
    "i32",
    "net_names_or_star",
    "not_zero",
    "optional_int",
    "presence",
    "u64_positive_or_default",
    "usize_or_default",
    "usize_positive",
]
