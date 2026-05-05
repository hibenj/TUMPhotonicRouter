"""Primitive library: 1:1 mapping from Rust primitive IDs to gdsfactory components.

This module defines the waveguide primitives that Rust A* routing uses.
Each primitive is a prebuilt gdsfactory component that can be placed directly.

Structure:
- Rust primitives (straights, bends) → gdsfactory components
- Metadata: angle change, length, cost
- 1:1 mapping: Rust primitive ID → component + metadata
"""

import gdsfactory as gf
from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class PrimitiveMetadata:
    """Metadata for a routed primitive."""
    start_angle: int      # 0-7 (East, NE, N, NW, W, SW, S, SE)
    end_angle: int        # 0-7
    length_um: float      # physical length
    bend_cost: float      # A* cost contribution


class PrimitiveLibrary:
    """Maps Rust primitive IDs to gdsfactory components and metadata."""

    def __init__(self):
        """Initialize the primitive library with gdsfactory waveguides."""
        self.components: Dict[int, gf.Component] = {}
        self.metadata: Dict[int, PrimitiveMetadata] = {}
        self._build_library()

    def _build_library(self) -> None:
        """Build primitives for all angles and types."""
        # Configuration from Rust defaults
        grid_size_um = 0.5
        straight_short_cells = 1
        straight_long_cells = 4
        bend_radius_cells = 2

        # gdsfactory cross sections often require a minimum bend radius.
        # Keep the primitive library compatible with the active technology.
        bend_radius_um = max(bend_radius_cells * grid_size_um, 5.0)

        straight_short_um = straight_short_cells * grid_size_um
        straight_long_um = straight_long_cells * grid_size_um

        def make_bend(angle: float) -> gf.Component:
            """Create a bend component for any angle.

            Prefer the all-angle Euler bend factory when available because the
            library includes 45° primitives. Fall back to bend_euler only for
            right-angle-compatible environments.
            """
            bend_factory = getattr(gf.components, "bend_euler_all_angle", None)
            if bend_factory is not None:
                return bend_factory(radius=bend_radius_um, angle=angle, width=0.5)

            if abs(angle) != 90:
                raise RuntimeError(
                    "This gdsfactory version does not expose bend_euler_all_angle, "
                    "so 45° primitive bends cannot be created safely."
                )
            return gf.components.bend_euler(radius=bend_radius_um, angle=angle, width=0.5)

        primitive_id = 0

        # For each of 8 angles (E, NE, N, NW, W, SW, S, SE)
        for angle in range(8):
            # Short straight
            self.components[primitive_id] = gf.components.straight(
                length=straight_short_um,
                width=0.5,
            )
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=angle,
                length_um=straight_short_um,
                bend_cost=0.0,
            )
            primitive_id += 1

            # Long straight
            self.components[primitive_id] = gf.components.straight(
                length=straight_long_um,
                width=0.5,
            )
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=angle,
                length_um=straight_long_um,
                bend_cost=0.0,
            )
            primitive_id += 1

            # Turn left 45° (angle delta +1)
            self.components[primitive_id] = make_bend(45)
            end_angle = (angle + 1) % 8
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=end_angle,
                length_um=self._approx_bend_length(bend_radius_um, 45),
                bend_cost=1.0,
            )
            primitive_id += 1

            # Turn right 45° (angle delta -1)
            self.components[primitive_id] = make_bend(-45)
            end_angle = (angle - 1) % 8
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=end_angle,
                length_um=self._approx_bend_length(bend_radius_um, 45),
                bend_cost=1.0,
            )
            primitive_id += 1

            # Turn left 90° (angle delta +2)
            self.components[primitive_id] = make_bend(90)
            end_angle = (angle + 2) % 8
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=end_angle,
                length_um=self._approx_bend_length(bend_radius_um, 90),
                bend_cost=2.0,
            )
            primitive_id += 1

            # Turn right 90° (angle delta -2)
            self.components[primitive_id] = make_bend(-90)
            end_angle = (angle - 2) % 8
            self.metadata[primitive_id] = PrimitiveMetadata(
                start_angle=angle,
                end_angle=end_angle,
                length_um=self._approx_bend_length(bend_radius_um, 90),
                bend_cost=2.0,
            )
            primitive_id += 1

    @staticmethod
    def _approx_bend_length(radius: float, angle_deg: float) -> float:
        """Approximate arc length of a bend in micrometers."""
        import math
        angle_rad = math.radians(angle_deg)
        return radius * angle_rad

    def get_component(self, primitive_id: int) -> gf.Component | None:
        """Get the gdsfactory component for a primitive ID."""
        return self.components.get(primitive_id)

    def get_metadata(self, primitive_id: int) -> PrimitiveMetadata | None:
        """Get metadata for a primitive ID."""
        return self.metadata.get(primitive_id)

    def get_all_metadata(self) -> Dict[int, PrimitiveMetadata]:
        """Get all primitive metadata."""
        return dict(self.metadata)


# Global singleton
_instance: PrimitiveLibrary | None = None


def get_primitive_library() -> PrimitiveLibrary:
    """Get or create the global primitive library."""
    global _instance
    if _instance is None:
        _instance = PrimitiveLibrary()
    return _instance


def reset_primitive_library() -> None:
    """Reset the global primitive library (for testing)."""
    global _instance
    _instance = None

