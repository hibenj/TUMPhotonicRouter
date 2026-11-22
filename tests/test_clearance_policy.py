"""`OpticalRouteClearancePolicy.from_dimensions`: which dimensions it refuses."""

from __future__ import annotations

import math

import pytest
from photonic_router.records.route_rust_types import OpticalRouteClearancePolicy


@pytest.mark.parametrize("clearance_um", [math.nan, math.inf])
def test_a_clearance_that_is_not_a_finite_number_is_refused(clearance_um: float) -> None:
    with pytest.raises(ValueError, match="route_clearance_um must be finite"):
        OpticalRouteClearancePolicy.from_dimensions(
            route_width_um=0.5, grid_size_um=1.0, route_clearance_um=clearance_um
        )


def test_a_negative_clearance_is_taken_as_zero() -> None:
    policy = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=0.5, grid_size_um=1.0, route_clearance_um=-1.0
    )
    zero = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=0.5, grid_size_um=1.0, route_clearance_um=0.0
    )
    assert policy == zero
