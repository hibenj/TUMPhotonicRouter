"""Electrical routing stage for heater metal routing."""

from __future__ import annotations

from .metal_realization import realize_electrical_metal
from .route_electrical import route_electrical_heaters
from .types import (
    DEFAULT_BONDPAD_WIDTH_UM,
    DEFAULT_BUS_WIDTH_UM,
    DEFAULT_COMMON_BUS_BONDPAD_LENGTH_UM,
    DEFAULT_COMMON_BUS_BONDPAD_WIDTH_UM,
    DEFAULT_PAD_PITCH_UM,
    DEFAULT_WIRE_WIDTH_UM,
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalPortAccess,
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
    "DEFAULT_BONDPAD_WIDTH_UM",
    "DEFAULT_BUS_WIDTH_UM",
    "DEFAULT_COMMON_BUS_BONDPAD_LENGTH_UM",
    "DEFAULT_COMMON_BUS_BONDPAD_WIDTH_UM",
    "DEFAULT_PAD_PITCH_UM",
    "DEFAULT_WIRE_WIDTH_UM",
    "CommonBusEscapeResult",
    "CommonBusRoutingResult",
    "DetailedBundleRoute",
    "DetailedBundleRoutingResult",
    "ElectricalObstacleMap",
    "ElectricalPortAccess",
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
