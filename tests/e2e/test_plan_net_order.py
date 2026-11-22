"""The net order of contribution 1 (`plan-crossings-hybrid`) and the plan it reads.

A plan without a crossing is a plan: the design routes, in declaration
order. Only a run without a plan (the baseline) is refused.
"""

from __future__ import annotations

import pytest
from photonic_router import cli


@pytest.mark.e2e
def test_contribution_1_routes_a_design_whose_plan_has_no_crossing() -> None:
    assert cli.main(["heater_lanes_ripup", "--configuration", "contribution1"]) is not None


@pytest.mark.e2e
def test_the_baseline_refuses_the_plan_order_and_says_why_there_is_no_plan() -> None:
    with pytest.raises(ValueError, match="no plan: lidar_pure_mode_ignores_topology_plan"):
        cli.main(["heater_lanes_ripup", "--net-order", "plan-crossings-hybrid"])
