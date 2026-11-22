"""`rect_geometry.rect_is_covered`: is a rectangle inside the union of others?"""

from __future__ import annotations

from photonic_router.electrical.rect_geometry import rect_is_covered


def test_one_cover_that_contains_the_rectangle_covers_it() -> None:
    assert rect_is_covered((1.0, 1.0, 2.0, 2.0), ((0.0, 0.0, 3.0, 3.0),))


def test_two_covers_that_only_together_contain_the_rectangle_cover_it() -> None:
    left, right = (0.0, 0.0, 2.0, 4.0), (2.0, 0.0, 5.0, 4.0)
    assert rect_is_covered((1.0, 1.0, 3.0, 2.0), (left, right))


def test_a_gap_between_the_covers_leaves_the_rectangle_uncovered() -> None:
    left, right = (0.0, 0.0, 1.9, 4.0), (2.0, 0.0, 5.0, 4.0)
    assert not rect_is_covered((1.0, 1.0, 3.0, 2.0), (left, right))


def test_no_cover_covers_nothing() -> None:
    assert not rect_is_covered((1.0, 1.0, 3.0, 2.0), ())
