"""Tests for `photonic_router.env_overlay` (Milestone 1 Slice 1 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`): one
test per parser rule, a test that the overlay table covers every Rust-read
`PHOTONIC_ROUTER_*` name of the inventory, a `RouterConfig().to_rust`
round-trip, and the environment-driven examples the milestone calls out."""

from __future__ import annotations

import pytest

from photonic_router.config import ALL_NETS, RouterConfig, RoutingConfig
from photonic_router.env_overlay import (
    ENV_OVERLAY,
    SPECIALLY_HANDLED_NAMES,
    apply_env_overlay,
    cell_pair,
    cell_pairs_semicolon,
    comma_digit_ints_frozenset,
    comma_ints,
    comma_names_frozenset_or_none,
    comma_token_frozenset,
    dense_fanout_min_ports_override,
    enabled_words,
    exact_one,
    f64_finite_nonneg,
    float_or_unset,
    heap_tie_breaker_override,
    i32,
    net_names_or_star,
    nonempty_raw_or_none,
    nonnegative_int,
    not_zero,
    optional_int,
    ordered_digit_ints_tuple,
    plain_float,
    positive_int_or_raise,
    presence,
    raw_string,
    string_or_unset,
    stripped_string,
    truthy,
    truthy_stripped,
    u64_positive_or_default,
    usize_or_default,
    usize_positive,
    zero_one_crossing_budget,
)

# --- one test per parser rule -------------------------------------------


def test_presence_is_true_whenever_called():
    assert presence("") is True
    assert presence("anything") is True


def test_exact_one_matches_only_the_literal_string_one():
    assert exact_one("1") is True
    assert exact_one("true") is False
    assert exact_one("0") is False


def test_not_zero_is_false_only_for_the_literal_string_zero():
    assert not_zero("0") is False
    assert not_zero("1") is True
    assert not_zero("") is True


def test_enabled_words_matches_the_documented_set_case_insensitively():
    for word in ("1", "true", "on", "yes", "enabled", "TRUE", " On "):
        assert enabled_words(word) is True
    for word in ("0", "false", "off", "no", "disabled", ""):
        assert enabled_words(word) is False


def test_u64_positive_or_default_keeps_default_on_zero_or_unparseable():
    parse = u64_positive_or_default(2_000_000)
    assert parse("5000000") == 5_000_000
    assert parse("0") == 2_000_000
    assert parse("-3") == 2_000_000
    assert parse("abc") == 2_000_000
    assert parse("") == 2_000_000


def test_usize_or_default_allows_zero_but_not_unparseable():
    parse = usize_or_default(100)
    assert parse("0") == 0
    assert parse("42") == 42
    assert parse("abc") == 100
    assert parse("") == 100


def test_usize_positive_maps_zero_and_unparseable_to_none():
    assert usize_positive("30000000") == 30_000_000
    assert usize_positive("0") is None
    assert usize_positive("abc") is None
    assert usize_positive("") is None
    assert usize_positive(" 30000000 ") == 30_000_000


def test_optional_int_never_raises():
    assert optional_int("42") == 42
    assert optional_int("-7") == -7
    assert optional_int("abc") is None
    assert optional_int("") is None


def test_i32_is_optional_int_never_raises():
    assert i32("5") == 5
    assert i32("abc") is None


def test_f64_finite_nonneg_raises_on_invalid_and_accepts_valid():
    parse = f64_finite_nonneg("PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT")
    assert parse("0.05") == pytest.approx(0.05)
    assert parse("0") == 0.0
    with pytest.raises(ValueError):
        parse("abc")
    with pytest.raises(ValueError):
        parse("-1.0")
    with pytest.raises(ValueError):
        parse("nan")
    with pytest.raises(ValueError):
        parse("inf")


def test_comma_ints_drops_unparseable_items_and_can_be_short_or_empty():
    assert comma_ints("1,2,3,4") == (1, 2, 3, 4)
    assert comma_ints("1,2,x,4") == (1, 2, 4)
    assert comma_ints("x,y") == ()
    assert comma_ints("") == ()


def test_cell_pair_parses_x_y_and_rejects_malformed_input():
    assert cell_pair("3,4") == (3, 4)
    assert cell_pair(" 3 , 4 ") == (3, 4)
    assert cell_pair("3") is None
    assert cell_pair("x,y") is None


def test_cell_pairs_semicolon_splits_and_skips_malformed_entries():
    assert cell_pairs_semicolon("1,2;3,4") == ((1, 2), (3, 4))
    assert cell_pairs_semicolon("1,2;bad;5,6") == ((1, 2), (5, 6))
    assert cell_pairs_semicolon("") == ()


def test_net_names_or_star_returns_all_nets_sentinel_when_star_present():
    assert net_names_or_star("*") == ALL_NETS
    assert net_names_or_star("n1,*,n2") == ALL_NETS
    assert net_names_or_star("n1,n2") == ("n1", "n2")
    assert net_names_or_star(" n1 , n2 ") == ("n1", "n2")


# --- Milestone 1 Slice 2: one test per new parser rule ------------------


def test_raw_string_is_identity():
    assert raw_string("") == ""
    assert raw_string(" Legacy_Runway ") == " Legacy_Runway "


def test_stripped_string_strips_only():
    assert stripped_string("  first  ") == "first"
    assert stripped_string("") == ""


def test_string_or_unset_maps_blank_to_none():
    assert string_or_unset("tiles") == "tiles"
    assert string_or_unset("  ") is None
    assert string_or_unset("") is None


def test_nonempty_raw_or_none_does_not_strip():
    assert nonempty_raw_or_none("mmi_1") == "mmi_1"
    assert nonempty_raw_or_none("") is None
    assert nonempty_raw_or_none(" ") == " "  # whitespace-only is still non-empty


def test_truthy_is_bool_of_the_raw_value():
    assert truthy("0") is True  # unlike Python's own bool("0"), any non-empty string
    assert truthy("anything") is True
    assert truthy("") is False


def test_truthy_stripped_strips_before_the_truthiness_check():
    assert truthy_stripped("  ") is False
    assert truthy_stripped(" x ") is True
    assert truthy_stripped("") is False


def test_plain_float_raises_the_builtin_value_error():
    assert plain_float("12.5") == pytest.approx(12.5)
    with pytest.raises(ValueError):
        plain_float("abc")


def test_float_or_unset_maps_blank_to_none_and_raises_on_bad_value():
    parse = float_or_unset("PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM")
    assert parse("20") == pytest.approx(20.0)
    assert parse("  ") is None
    assert parse("") is None
    with pytest.raises(ValueError):
        parse("abc")


def test_nonnegative_int_maps_blank_to_none_and_raises_on_negative_or_bad_value():
    parse = nonnegative_int("PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS")
    assert parse("11") == 11
    assert parse("0") == 0
    assert parse("") is None
    assert parse("  ") is None
    with pytest.raises(ValueError):
        parse("-1")
    with pytest.raises(ValueError):
        parse("abc")


def test_dense_fanout_min_ports_override_enforces_the_floor_of_two():
    parse = dense_fanout_min_ports_override("PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS")
    assert parse("") is None
    assert parse("3") == 3
    with pytest.raises(ValueError):
        parse("1")
    with pytest.raises(ValueError):
        parse("abc")


def test_positive_int_or_raise_maps_empty_to_none_and_enforces_the_floor_of_one():
    parse = positive_int_or_raise("PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT")
    assert parse("") is None
    assert parse("5") == 5
    with pytest.raises(ValueError):
        parse("0")
    with pytest.raises(ValueError):
        parse("abc")


def test_zero_one_crossing_budget_accepts_only_0_or_1():
    parse = zero_one_crossing_budget("PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET")
    assert parse("0") is False
    assert parse("1") is True
    with pytest.raises(ValueError):
        parse("2")
    with pytest.raises(ValueError):
        parse("")


def test_heap_tie_breaker_override_only_accepts_the_two_literal_values():
    assert heap_tie_breaker_override("smaller_g") == "smaller_g"
    assert heap_tie_breaker_override("larger_g") == "larger_g"
    assert heap_tie_breaker_override(" smaller_g ") == "smaller_g"
    assert heap_tie_breaker_override("other") is None
    assert heap_tie_breaker_override("") is None


def test_comma_digit_ints_frozenset_drops_non_digit_tokens():
    assert comma_digit_ints_frozenset("1,2,3") == frozenset({1, 2, 3})
    assert comma_digit_ints_frozenset("1,-2,x") == frozenset({1})  # "-2" is not a digit token
    assert comma_digit_ints_frozenset("") == frozenset()


def test_ordered_digit_ints_tuple_keeps_order_and_drops_non_digit_tokens():
    assert ordered_digit_ints_tuple("3,1,2") == (3, 1, 2)
    assert ordered_digit_ints_tuple("3,x,2") == (3, 2)
    assert ordered_digit_ints_tuple("") == ()


def test_comma_token_frozenset_strips_and_drops_empty_tokens():
    assert comma_token_frozenset("a, b ,,c") == frozenset({"a", "b", "c"})
    assert comma_token_frozenset("*") == frozenset({"*"})
    assert comma_token_frozenset("") == frozenset()


def test_comma_names_frozenset_or_none_maps_empty_result_to_none():
    assert comma_names_frozenset_or_none("mmi_1,mmi_2") == frozenset({"mmi_1", "mmi_2"})
    assert comma_names_frozenset_or_none("") is None
    assert comma_names_frozenset_or_none(" , ,") is None


# --- coverage: every Rust-read name from the Milestone 1 inventory ------

# Hard-coded from the brief's Slice 1 enumeration (src/astar.rs and
# src/py_router.rs read sites), one entry per distinct PHOTONIC_ROUTER_*
# name -- not per Rust field (TRACE_CROSSING_NET and TRACE_PARTNER_NET are
# each read from both files but are one name/one field).
RUST_SIDE_NAMES: frozenset[str] = frozenset(
    {
        # Negotiation (src/py_router.rs)
        "PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST",
        "PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST_RETRY",
        "PHOTONIC_ROUTER_NEGOTIATED_BUDGET_RETRY",
        "PHOTONIC_ROUTER_NEGOTIATED_BRAID_ESCALATION",
        "PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED",
        "PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR",
        "PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD",
        "PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK",
        # Search overrides (src/py_router.rs)
        "PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS",
        "PHOTONIC_ROUTER_ASTAR_TIMEOUT_S",
        "PHOTONIC_ROUTER_MAX_DENSE_STATES",
        "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT",
        "PHOTONIC_ROUTER_SEARCH_ENGINE",
        # Crossing engine (src/py_router.rs)
        "PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING",
        "PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING",
        "PHOTONIC_ROUTER_DISABLE_RUST_CROSSING_VALIDATION",
        # Diagnostics (src/py_router.rs)
        "PHOTONIC_ROUTER_NATIVE_PROGRESS",
        "PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG",
        "PHOTONIC_ROUTER_TRACE_CROSSING",
        "PHOTONIC_ROUTER_TRACE_CROSSING_NET",
        "PHOTONIC_ROUTER_TRACE_PARTNER_NET",
        "PHOTONIC_ROUTER_TRACE_PLAIN_ROUTE_NET",
        "PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS",
        "PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NET",
        "PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP",
        "PHOTONIC_ROUTER_CROSSING_MISMATCH_FATAL",
        # Diagnostics (src/astar.rs)
        "PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS",
        "PHOTONIC_ROUTER_CHAIN_DIAG",
        "PHOTONIC_ROUTER_HOT_LOOP_TIMING",
        "PHOTONIC_ROUTER_MOVE_DIAG",
        "PHOTONIC_ROUTER_POP_DIAG_BELOW_Y",
        "PHOTONIC_ROUTER_PROBE_CELLS",
        "PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG",
        "PHOTONIC_ROUTER_SEARCH_FAILURE_MAP",
        "PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATE_MAX",
        "PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATES",
        "PHOTONIC_ROUTER_TRACE_CROSSING_LEVEL1",
        "PHOTONIC_ROUTER_TRACE_CROSSING_PENDING",
        "PHOTONIC_ROUTER_TRACE_CROSSING_PENDING_THRESHOLD",
        "PHOTONIC_ROUTER_TRACE_CROSSING_PERP_REJECT_THRESHOLD",
    }
)


# Python-side names (Milestone 1 Slice 2 of the same ExecPlan): every
# PHOTONIC_ROUTER_* variable `translation/`, `routing_flow*.py` and
# `python/photonic_router/` read outside `env_overlay.py` itself, one entry
# per distinct name. `PHOTONIC_ROUTER_SYNTH` (benchmarks/permutation_synthetic.py,
# a benchmark generator parameter, not a routing knob) and the dead
# `LAYER_ORDER` comment mention are deliberately excluded -- out of scope
# for both slices.
PYTHON_SIDE_NAMES: frozenset[str] = frozenset(
    {
        # Crossing plan (translation/route_rust_crossing_plan.py)
        "PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM",
        "PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM",
        "PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET",
        # Crossing grid (translation/preplaced_crossing_grids.py, crossing_structures.py)
        "PHOTONIC_ROUTER_CROSSING_GRID_BAND_MARGIN_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_LEAD_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_PITCH_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_CORNER_MARGIN_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_FAN_COLUMN_PITCH_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_PORT_PAIR_SPREAD_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_SLOT_SPREAD_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_STUB_STAGGER_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_TILE_MIN_SPACING_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM",
        "PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE",
        "PHOTONIC_ROUTER_CROSSING_GRID_TILE_PLACEMENT",
        "PHOTONIC_ROUTER_CROSSING_GRID_CORNERS",
        "PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS",
        "PHOTONIC_ROUTER_TRACE_COLUMN_GRID",
        # Fan-out access (translation/route_rust.py)
        "PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES",
        "PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS",
        "PHOTONIC_ROUTER_FANOUT_ACCESS_MODE",
        "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
        "PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS",
        "PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS",
        "PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES",
        "PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS",
        "PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS",
        "PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS",
        "PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS",
        # Engine selection (translation/route_rust.py)
        "PHOTONIC_ROUTER_NEGOTIATED_REPAIR",
        "PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN",
        # Search tuning (translation/route_rust.py)
        "PHOTONIC_ROUTER_MIN_BEND_WEIGHT",
        "PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT",
        "PHOTONIC_ROUTER_HEAP_TIE_BREAKER",
        "PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT",
        # Flow diagnostics (translation/route_rust.py, route_rust_endpoint_correction.py)
        "PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT",
        "PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE",
        "PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS",
        "PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS",
        "PHOTONIC_ROUTER_TRACE_FANOUT_STUBS",
        "PHOTONIC_ROUTER_TRACE_GRID",
        "PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE",
        "PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS",
        # Debug artifact (routing_flow_verification.py)
        "PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE",
    }
)


def test_env_overlay_table_covers_every_rust_side_name():
    covered = {entry.name for entry in ENV_OVERLAY} | SPECIALLY_HANDLED_NAMES
    all_names = RUST_SIDE_NAMES | PYTHON_SIDE_NAMES
    missing = all_names - covered
    extra = covered - all_names
    assert not missing, f"names read but missing from the overlay: {sorted(missing)}"
    assert not extra, f"overlay names not in the Rust+Python-side inventory: {sorted(extra)}"


# --- RouterConfig().to_rust round-trip ----------------------------------


def test_router_config_default_to_rust_round_trips_defaults():
    import photonic_router._rust as rust_backend

    cfg = RouterConfig()
    rust_cfg = cfg.to_rust(rust_backend)
    assert rust_cfg.negotiation_budget_first == cfg.negotiation.budget_first
    assert rust_cfg.negotiation_budget_first_retry == cfg.negotiation.budget_first_retry
    assert rust_cfg.negotiation_budget_retry == cfg.negotiation.budget_retry
    assert rust_cfg.negotiation_braid_escalation == cfg.negotiation.braid_escalation
    assert (
        rust_cfg.negotiation_crossing_free_unplanned == cfg.negotiation.crossing_free_unplanned
    )
    assert rust_cfg.negotiation_disable_braid_repair == cfg.negotiation.disable_braid_repair
    assert (
        rust_cfg.negotiation_pending_straight_ripup_threshold
        == cfg.negotiation.pending_straight_ripup_threshold
    )
    assert (
        rust_cfg.negotiation_enable_orthogonal_repair_fallback
        == cfg.negotiation.enable_orthogonal_repair_fallback
    )
    assert rust_cfg.search_astar_timeout_ms == cfg.search.astar_timeout_ms
    assert rust_cfg.search_max_dense_states == cfg.search.max_dense_states
    assert (
        rust_cfg.search_long_straight_congestion_weight
        == cfg.search.long_straight_congestion_weight
    )
    assert rust_cfg.search_engine == cfg.search.engine == "astar"
    assert (
        rust_cfg.crossing_enable_guided_collision_crossing
        == cfg.crossing.enable_guided_collision_crossing
    )
    assert (
        rust_cfg.crossing_disable_guided_collision_crossing
        == cfg.crossing.disable_guided_collision_crossing
    )
    assert (
        rust_cfg.crossing_disable_rust_crossing_validation
        == cfg.crossing.disable_rust_crossing_validation
    )
    assert rust_cfg.diag_native_progress == cfg.diagnostics.native_progress
    assert rust_cfg.diag_trace_crossing_candidate_max == cfg.diagnostics.trace_crossing_candidate_max
    assert rust_cfg.diag_trace_endpoint_bump_nets is None
    assert rust_cfg.diag_trace_endpoint_bump_all_nets is False


def test_router_config_to_rust_encodes_all_nets_sentinel():
    import photonic_router._rust as rust_backend

    cfg = RouterConfig(
        diagnostics=RouterConfig().diagnostics.__class__(trace_endpoint_bump_nets=ALL_NETS)
    )
    rust_cfg = cfg.to_rust(rust_backend)
    assert rust_cfg.diag_trace_endpoint_bump_all_nets is True
    assert rust_cfg.diag_trace_endpoint_bump_nets is None


def test_router_config_to_rust_encodes_specific_net_names():
    import photonic_router._rust as rust_backend

    cfg = RouterConfig(
        diagnostics=RouterConfig().diagnostics.__class__(trace_endpoint_bump_nets=("n1", "n2"))
    )
    rust_cfg = cfg.to_rust(rust_backend)
    assert rust_cfg.diag_trace_endpoint_bump_all_nets is False
    assert rust_cfg.diag_trace_endpoint_bump_nets == ["n1", "n2"]


# --- from_environment examples the milestone calls out -------------------


def test_from_environment_applies_crossing_free_unplanned_zero_and_native_repair_diag():
    cfg = RouterConfig.from_environment(
        {
            "PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED": "0",
            "PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG": "anything",
        }
    )
    assert cfg.negotiation.crossing_free_unplanned is False
    assert cfg.diagnostics.native_repair_diag is True
    # Everything else stays at its default.
    assert cfg.negotiation.budget_first == 2_000_000
    assert cfg.search.long_straight_congestion_weight is None


def test_from_environment_empty_environ_returns_all_defaults():
    cfg = RouterConfig.from_environment({})
    assert cfg == RouterConfig()


# --- coordinator-confirmed semantics: net filters never raise -----------


@pytest.mark.parametrize(
    "name,path",
    [
        ("PHOTONIC_ROUTER_TRACE_CROSSING_NET", ("diagnostics", "trace_crossing_net")),
        ("PHOTONIC_ROUTER_TRACE_PARTNER_NET", ("diagnostics", "trace_partner_net")),
        (
            "PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NET",
            ("diagnostics", "trace_endpoint_correction_net"),
        ),
    ],
)
def test_net_id_filters_parse_to_none_on_unparseable_value_never_raise(name, path):
    cfg = apply_env_overlay(RouterConfig(), {name: "not-a-number"})
    group_name, field_name = path
    assert getattr(getattr(cfg, group_name), field_name) is None


def test_crossing_mismatch_dump_net_filter_parses_to_none_on_unparseable_value():
    cfg = apply_env_overlay(
        RouterConfig(), {"PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP": "not-a-number"}
    )
    assert cfg.diagnostics.crossing_mismatch_dump is True
    assert cfg.diagnostics.crossing_mismatch_dump_net is None


def test_crossing_mismatch_dump_net_filter_parses_a_valid_value():
    cfg = apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP": "42"})
    assert cfg.diagnostics.crossing_mismatch_dump is True
    assert cfg.diagnostics.crossing_mismatch_dump_net == 42


# --- coordinator-confirmed semantics: SEARCH_FAILURE_MAP presence -------


def test_search_failure_map_present_yields_some_list_even_when_short_or_empty():
    cfg = apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_SEARCH_FAILURE_MAP": "abc,xyz"})
    assert cfg.diagnostics.search_failure_map == ()

    cfg = apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_SEARCH_FAILURE_MAP": "1,2"})
    assert cfg.diagnostics.search_failure_map == (1, 2)

    cfg = apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_SEARCH_FAILURE_MAP": "1,2,3,4,5"})
    assert cfg.diagnostics.search_failure_map == (1, 2, 3, 4, 5)


def test_search_failure_map_absent_stays_none():
    cfg = apply_env_overlay(RouterConfig(), {})
    assert cfg.diagnostics.search_failure_map is None


# --- coordinator-confirmed semantics: ASTAR_TIMEOUT_MS/S and
# LONG_STRAIGHT_CONGESTION_WEIGHT keep raising on invalid values ----------


def test_astar_timeout_ms_raises_on_invalid_value():
    with pytest.raises(ValueError):
        apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS": "not-an-int"})


def test_astar_timeout_s_raises_on_invalid_and_negative_values():
    with pytest.raises(ValueError):
        apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_ASTAR_TIMEOUT_S": "not-a-number"})
    with pytest.raises(ValueError):
        apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_ASTAR_TIMEOUT_S": "-1.0"})


def test_astar_timeout_ms_takes_precedence_over_s_and_converts_seconds_to_ms():
    cfg = apply_env_overlay(
        RouterConfig(),
        {"PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS": "500", "PHOTONIC_ROUTER_ASTAR_TIMEOUT_S": "9"},
    )
    assert cfg.search.astar_timeout_ms == 500

    cfg = apply_env_overlay(RouterConfig(), {"PHOTONIC_ROUTER_ASTAR_TIMEOUT_S": "1.5"})
    assert cfg.search.astar_timeout_ms == 1500


def test_long_straight_congestion_weight_raises_on_invalid_value():
    with pytest.raises(ValueError):
        apply_env_overlay(
            RouterConfig(), {"PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "not-a-number"}
        )
    with pytest.raises(ValueError):
        apply_env_overlay(
            RouterConfig(), {"PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "-0.5"}
        )


def test_long_straight_congestion_weight_applies_a_valid_value():
    cfg = apply_env_overlay(
        RouterConfig(), {"PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "0.05"}
    )
    assert cfg.search.long_straight_congestion_weight == pytest.approx(0.05)


# --- Milestone 1 Slice 2: RoutingConfig.from_environment examples --------


def test_routing_config_from_environment_applies_the_whole_table():
    cfg = RoutingConfig.from_environment(
        {
            "PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM": "20",
            "PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST": "123",
        }
    )
    assert cfg.crossing_grid.lane_pitch_um == pytest.approx(20.0)
    assert cfg.router.negotiation.budget_first == 123
    # Everything else stays at its default.
    assert cfg == RoutingConfig(
        crossing_grid=RoutingConfig().crossing_grid.__class__(lane_pitch_um=20.0),
        router=RouterConfig(
            negotiation=RouterConfig().negotiation.__class__(budget_first=123)
        ),
    )


def test_routing_config_from_environment_empty_environ_returns_all_defaults():
    assert RoutingConfig.from_environment({}) == RoutingConfig()


def test_planned_crossing_budget_raises_through_routing_config_from_environment():
    """0/1 are legal; every other value raises the same way `RouterConfig`'s
    other raising variables do -- validated once, at config-build time."""
    cfg = RoutingConfig.from_environment({"PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET": "1"})
    assert cfg.crossing_plan.planned_crossing_budget is True
    with pytest.raises(ValueError):
        RoutingConfig.from_environment({"PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET": "2"})


def test_debug_execution_limit_raises_through_routing_config_from_environment():
    cfg = RoutingConfig.from_environment({"PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT": "5"})
    assert cfg.diagnostics.debug_execution_limit == 5
    with pytest.raises(ValueError):
        RoutingConfig.from_environment({"PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT": "0"})
    with pytest.raises(ValueError):
        RoutingConfig.from_environment({"PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT": "abc"})


def test_long_straight_exempt_dense_fanout_optional_bool_semantics():
    """Unset stays `None` (the flow then decides); present-and-"1" is True;
    present-but-not-"1" is False -- distinguishable from unset, matching the
    site's `is True` test (not a truthiness test)."""
    assert RoutingConfig.from_environment({}).search.long_straight_exempt_dense_fanout is None
    cfg_on = RoutingConfig.from_environment(
        {"PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT": "1"}
    )
    assert cfg_on.search.long_straight_exempt_dense_fanout is True
    cfg_off = RoutingConfig.from_environment(
        {"PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT": "0"}
    )
    assert cfg_off.search.long_straight_exempt_dense_fanout is False


def test_fanout_access_mode_env_override_beats_the_constructor_argument(monkeypatch):
    """`PHOTONIC_ROUTER_FANOUT_ACCESS_MODE` overrides the `fanout_access_mode`
    constructor argument of `_RouteNetsRustSession` when set; unset lets the
    constructor argument (or its own default) through."""
    import sys
    from pathlib import Path
    from types import SimpleNamespace

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
    from dataclasses import dataclass, field
    from typing import Any

    from gdsfactory.component import Component
    from gdsfactory.gpdk import get_generic_pdk

    from translation import route_rust
    from translation.routing import session as routing_session

    get_generic_pdk().activate()

    @dataclass
    class _DummyBundle:
        links: dict[str, str] = field(default_factory=dict)

    @dataclass
    class _DummyNetlist:
        routes: dict[str, _DummyBundle] = field(default_factory=dict)
        instances: dict[str, Any] = field(default_factory=dict)

    @dataclass
    class _DummySchematic:
        netlist: _DummyNetlist = field(default_factory=_DummyNetlist)

    monkeypatch.setattr(routing_session, "_load_rust_backend", lambda: SimpleNamespace())

    from uuid import uuid4

    def _session(fanout_access_mode, config=None):
        return route_rust._RouteNetsRustSession(
            unrouted_layout=Component(f"dummy_unrouted_{uuid4().hex}"),
            schematic=_DummySchematic(),
            fanout_access_mode=fanout_access_mode,
            config=config,
        )

    # No env override: the constructor argument wins.
    session = _session("off")
    assert session.fanout_access_mode_normalized == "off"

    # Env override present: it wins over the constructor argument.
    monkeypatch.setenv("PHOTONIC_ROUTER_FANOUT_ACCESS_MODE", "static-stubs")
    session = _session("off", config=RoutingConfig.from_environment())
    assert session.fanout_access_mode_normalized == "static-stubs"
