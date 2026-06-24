import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.routing_layers import (
    HEATER_METAL_OBSTACLE_LAYERS,
    HEATER_OPTICAL_PORT_ACCESS_RULES,
    OPTICAL_OBSTACLE_LAYERS,
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)


def test_get_routing_obstacle_layers_defaults_to_optical_only():
    assert get_routing_obstacle_layers(include_heaters=False) == OPTICAL_OBSTACLE_LAYERS
    assert get_routing_obstacle_layers(include_heaters=False) == ((1, 0),)


def test_get_routing_obstacle_layers_can_include_heaters():
    layers = get_routing_obstacle_layers(include_heaters=True)

    assert layers == OPTICAL_OBSTACLE_LAYERS + HEATER_METAL_OBSTACLE_LAYERS
    assert layers == ((1, 0), (47, 0), (45, 0), (49, 0), (125, 0), (44, 0), (43, 0))


def test_straight_heater_optical_ports_match_access_rule():
    for port_name in ("o1", "o2"):
        rule = find_component_port_access_rule(
            component_name="straight_heater_metal",
            port_name=port_name,
            port_type="optical",
        )

        assert rule in HEATER_OPTICAL_PORT_ACCESS_RULES
        assert rule is not None
        assert rule.access_length_um > 0.0
        assert rule.access_width_um > 0.0


def test_generated_straight_heater_component_name_matches_access_rule():
    rule = find_component_port_access_rule(
        component_name="straight_heater_metal_undercut_gdsfactorypcomponentspwa_1bc8609e",
        port_name="o1",
        port_type="optical",
    )

    assert rule in HEATER_OPTICAL_PORT_ACCESS_RULES


def test_straight_heater_electrical_ports_do_not_match_optical_rule():
    assert (
        find_component_port_access_rule(
            component_name="straight_heater_metal",
            port_name="l_e1",
            port_type="electrical",
        )
        is None
    )
    assert (
        find_component_port_access_rule(
            component_name="straight_heater_metal",
            port_name="o1",
            port_type="electrical",
        )
        is None
    )
