"""The detour pass's pure rules: the free lower bound and the selection of detoured nets."""

from __future__ import annotations

from photonic_router.routing.detour_pass import free_lower_bound_um, select_detoured_net_ids


def test_free_lower_bound_is_manhattan_distance_plus_two_bends():
    assert free_lower_bound_um((10.0, 10.0), (30.0, 49.0), bend_radius_um=5.0) == 20 + 39 + 10
    # a straight shot still carries the bend allowance; a zero radius adds nothing
    assert free_lower_bound_um((0.0, 0.0), (100.0, 0.0), bend_radius_um=0.0) == 100
    assert free_lower_bound_um((5.0, 5.0), (5.0, 5.0), bend_radius_um=3.0) == 6
    # the second Clements mesh's feed: 120 um across, 39 um down, bends of 5 um
    assert free_lower_bound_um((20856.5, 13257.0), (20976.5, 13218.0), 5.0) == 169


def test_selection_takes_the_nets_over_the_factor_shortest_bound_first():
    lengths = {1: 457.5, 2: 54.7, 3: 329.1, 4: 120.0, 5: 70.0}
    bounds = {1: 69.0, 2: 69.0, 3: 69.0, 4: 100.0, 5: 20.0}
    # net 1 (6.6x) and 3 (4.8x) are detours, 5 (3.5x) too; 2 (0.8x) and 4 (1.2x) are not
    assert select_detoured_net_ids(lengths, bounds, factor=3.0) == [5, 1, 3]
    # a stricter factor keeps only the worst; an unknown bound is skipped; 0 disables
    assert select_detoured_net_ids(lengths, bounds, factor=5.0) == [1]
    assert select_detoured_net_ids({6: 999.0}, bounds, factor=3.0) == []
    assert select_detoured_net_ids(lengths, bounds, factor=0.0) == []


def test_selection_breaks_bound_ties_by_net_id():
    lengths = {9: 500.0, 4: 500.0}
    bounds = {9: 50.0, 4: 50.0}
    assert select_detoured_net_ids(lengths, bounds, factor=2.0) == [4, 9]
