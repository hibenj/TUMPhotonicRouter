"""Tests for `photonic_router.env_overlay` (Milestone 1 Slice 1 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`): one
test per parser rule, a test that the overlay table covers every Rust-read
`PHOTONIC_ROUTER_*` name of the inventory, a `RouterConfig().to_rust`
round-trip, and the environment-driven examples the milestone calls out."""

from __future__ import annotations

import pytest

from photonic_router.config import ALL_NETS, RouterConfig
from photonic_router.env_overlay import (
    ENV_OVERLAY,
    SPECIALLY_HANDLED_NAMES,
    apply_env_overlay,
    cell_pair,
    cell_pairs_semicolon,
    comma_ints,
    enabled_words,
    exact_one,
    f64_finite_nonneg,
    i32,
    net_names_or_star,
    not_zero,
    optional_int,
    presence,
    u64_positive_or_default,
    usize_or_default,
    usize_positive,
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


def test_env_overlay_table_covers_every_rust_side_name():
    covered = {entry.name for entry in ENV_OVERLAY} | SPECIALLY_HANDLED_NAMES
    missing = RUST_SIDE_NAMES - covered
    extra = covered - RUST_SIDE_NAMES
    assert not missing, f"names read in Rust but missing from the overlay: {sorted(missing)}"
    assert not extra, f"overlay names not in the Rust-side inventory: {sorted(extra)}"


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
