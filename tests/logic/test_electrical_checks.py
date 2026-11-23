"""The checks of stage "electrical": each statement, on a wiring it holds for and on one it fails."""

from __future__ import annotations

from types import SimpleNamespace as NS

from photonic_router.logic_checks import count_by_check
from photonic_router.logic_checks.electrical import check_electrical


def _terminal(name: str) -> NS:
    return NS(id=name)


def _wiring() -> NS:
    """Two heaters: `h0` and `h1`, terminal `a` on the bus, terminal `b` wired to a pad."""
    groups = tuple(
        NS(heater_id=h, terminal_a=_terminal(f"{h}_a"), terminal_b=_terminal(f"{h}_b"))
        for h in ("h0", "h1")
    )
    pads = [NS(slot=NS(index=index)) for index in range(3)]
    routes = [
        NS(
            terminal=group.terminal_b,
            pad_assignment=pads[index],
            path=((index * 5, 0), (index * 5, 1), (index * 5, 2)),
            target_cells=frozenset({(index * 5, 2)}),
            reason=None,
        )
        for index, group in enumerate(groups)
    ]
    return NS(
        terminal_groups=groups,
        common_bus=NS(
            failed_heaters=(),
            selected_terminals={g.heater_id: g.terminal_a for g in groups},
            unselected_terminals={g.heater_id: g.terminal_b for g in groups},
        ),
        detailed_bundle_routes=NS(routes=tuple(routes), failed_routes=()),
        pad_plan=NS(assignments=tuple(pads)),
        verification=NS(issues=()),
    )


def _checks(wiring: NS) -> dict[str, int]:
    return count_by_check(check_electrical(wiring))


def test_a_sound_wiring_has_no_finding() -> None:
    assert _checks(_wiring()) == {}


def test_a_result_with_the_common_bus_only_is_checked_as_far_as_it_goes() -> None:
    wiring = _wiring()
    wiring.detailed_bundle_routes = None
    wiring.pad_plan = None
    wiring.verification = None

    assert _checks(wiring) == {}


def test_a_heater_the_bus_router_gave_up_on_is_an_error() -> None:
    wiring = _wiring()
    wiring.common_bus.failed_heaters = ("h1",)

    assert _checks(wiring) == {"electrical.heater_not_on_bus": 1}


def test_a_heater_that_is_neither_on_the_bus_nor_failed_is_an_error() -> None:
    wiring = _wiring()
    del wiring.common_bus.selected_terminals["h0"]

    assert _checks(wiring) == {"electrical.heater_without_bus_terminal": 1}


def test_bus_and_wire_must_take_the_two_terminals_of_the_heater() -> None:
    same = _wiring()
    same.common_bus.unselected_terminals["h0"] = same.common_bus.selected_terminals["h0"]
    foreign = _wiring()
    foreign.common_bus.selected_terminals["h1"] = _terminal("h0_a")

    assert _checks(same)["electrical.bus_terminal_not_of_heater"] == 1
    assert _checks(foreign)["electrical.bus_terminal_not_of_heater"] == 1


def test_a_failed_wire_is_an_error_and_not_also_a_missing_one() -> None:
    wiring = _wiring()
    failed = wiring.detailed_bundle_routes.routes[1]
    failed.reason = "no free track"
    wiring.detailed_bundle_routes.routes = wiring.detailed_bundle_routes.routes[:1]
    wiring.detailed_bundle_routes.failed_routes = (failed,)

    assert _checks(wiring) == {"electrical.wire_not_routed": 1}


def test_every_heater_has_exactly_one_wire() -> None:
    none = _wiring()
    none.detailed_bundle_routes.routes = none.detailed_bundle_routes.routes[:1]
    two = _wiring()
    extra = NS(**vars(two.detailed_bundle_routes.routes[0]))
    extra.path = ((20, 0), (20, 1))
    extra.target_cells = frozenset({(20, 1)})
    two.detailed_bundle_routes.routes = (*two.detailed_bundle_routes.routes, extra)

    assert _checks(none) == {"electrical.heater_without_wire": 1}
    assert _checks(two) == {"electrical.heater_with_several_wires": 1}


def test_a_pad_slot_holds_one_net() -> None:
    wiring = _wiring()
    wiring.pad_plan.assignments[2].slot.index = 0

    assert _checks(wiring) == {"electrical.pad_shared": 1}


def test_a_wire_is_a_path_of_neighbouring_cells_that_ends_on_its_pad() -> None:
    gap = _wiring()
    gap.detailed_bundle_routes.routes[0].path = ((0, 0), (0, 2))
    empty = _wiring()
    empty.detailed_bundle_routes.routes[0].path = ()
    short = _wiring()
    short.detailed_bundle_routes.routes[0].target_cells = frozenset({(0, 9)})
    unassigned = _wiring()
    unassigned.detailed_bundle_routes.routes[1].pad_assignment = None

    assert _checks(gap) == {"electrical.wire_not_a_path": 1}
    assert _checks(empty) == {"electrical.wire_not_a_path": 1}
    assert _checks(short) == {"electrical.wire_misses_pad": 1}
    assert _checks(unassigned) == {"electrical.wire_without_pad": 1}


def test_two_wires_do_not_share_a_cell() -> None:
    wiring = _wiring()
    wiring.detailed_bundle_routes.routes[1].path = ((5, 0), (4, 1), (3, 1), (2, 1), (1, 1), (0, 1))
    wiring.detailed_bundle_routes.routes[1].target_cells = frozenset({(0, 1)})

    assert _checks(wiring) == {"electrical.wires_share_cell": 1}


def test_an_error_of_the_router_s_own_verification_is_a_finding() -> None:
    wiring = _wiring()
    wiring.verification.issues = (
        NS(code="cross_net_overlap", message="two nets overlap", severity="error"),
        NS(code="spacing", message="close", severity="warning"),
    )

    findings = check_electrical(wiring)

    assert [(f.check, f.subject) for f in findings] == [
        ("electrical.own_check_failed", "cross_net_overlap")
    ]
