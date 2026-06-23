"""Electrical routing stage for heater metal routing."""

from .metal_realization import realize_electrical_metal
from .route_electrical import route_electrical_heaters
from .types import (
    CommonBusRoutingResult,
    CommonBusEscapeResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    ElectricalTerminal,
    ElectricalVerificationIssue,
    ElectricalVerificationResult,
    EscapeBundle,
    EscapeTopologyRoute,
    IndividualEscapeTopologyResult,
    PadAssignment,
    PadPlan,
    PadSlot,
    TerminalPairGroup,
)
from .verification import verify_electrical_routing

__all__ = [
    "CommonBusRoutingResult",
    "CommonBusEscapeResult",
    "DetailedBundleRoute",
    "DetailedBundleRoutingResult",
    "ElectricalObstacleMap",
    "ElectricalRoutingConfig",
    "ElectricalRoutingResult",
    "ElectricalTerminal",
    "ElectricalVerificationIssue",
    "ElectricalVerificationResult",
    "EscapeBundle",
    "EscapeTopologyRoute",
    "IndividualEscapeTopologyResult",
    "PadAssignment",
    "PadPlan",
    "PadSlot",
    "TerminalPairGroup",
    "realize_electrical_metal",
    "route_electrical_heaters",
    "verify_electrical_routing",
]
