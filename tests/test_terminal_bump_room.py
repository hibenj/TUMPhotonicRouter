"""The diagnostic record of the room a terminal bump has behind a crossing."""

from __future__ import annotations

from photonic_router.routing.finalize import _bump_room, _print_terminal_bump_checks


def test_the_room_is_the_distance_less_the_crossing_half_size():
    room = _bump_room(30.0, 4.5, 20.0, 1e-6)
    assert room == {
        "distance_to_target_um": 30.0,
        "crossing_half_um": 4.5,
        "available_um": 25.5,
        "required_bump_um": 20.0,
        "satisfies": True,
    }
    assert list(room) == [
        "distance_to_target_um",
        "crossing_half_um",
        "available_um",
        "required_bump_um",
        "satisfies",
    ]


def test_the_room_is_never_negative_and_is_judged_within_eps():
    assert _bump_room(3.0, 4.5, 20.0, 1e-6)["available_um"] == 0.0
    assert _bump_room(3.0, 4.5, 20.0, 1e-6)["satisfies"] is False
    # 24.5 - 4.5 = 20.0 exactly; a hair less still counts within eps
    assert _bump_room(24.5 - 5e-7, 4.5, 20.0, 1e-6)["satisfies"] is True
    assert _bump_room(24.5 - 5e-6, 4.5, 20.0, 1e-6)["satisfies"] is False


def test_the_trace_prints_the_target_line_and_one_line_per_crossing(capsys):
    check = {"crossing_um": [1.0, 2.0], **_bump_room(30.0, 4.5, 20.0, 1e-6)}
    _print_terminal_bump_checks("net=n1 id=7", "target_axis=horizontal", [check])
    assert capsys.readouterr().out.splitlines() == [
        "terminal_bump_distance_check net=n1 id=7 target_axis=horizontal same_axis_crossings=1",
        (
            "terminal_bump_distance_check net=n1 id=7 crossing=[1.0, 2.0] "
            "available_um=25.5 required_um=20.0 satisfies=True"
        ),
    ]
