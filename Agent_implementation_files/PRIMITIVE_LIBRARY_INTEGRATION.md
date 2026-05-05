"""Primitive Library Integration: How It Works

This document explains the 1:1 mapping approach for routing primitives.

=== ARCHITECTURE OVERVIEW ===

Before (path extrusion approach):
  Rust router → states + primitives → ignored primitives
  Python constructs path from states → path smoothing → extrude to waveguide
  ❌ Loses primitive information, reconstructed geometry may be inaccurate

After (primitive library approach):
  Python builds primitive library (gdsfactory components)
  Python passes library metadata to Rust (optional, for future optimization)
  Rust router → states + primitive IDs (4,5,12,13,...)
  Python does 1:1 lookup: prim_id → gdsfactory component → place in layout
  ✓ Exact primitives used, faithful reconstruction, modular


=== PYTHON PRIMITIVE LIBRARY ===

Location: python/photonic_router/primitive_library.py

Structure:
  class PrimitiveLibrary:
    - components: dict[int, gf.Component]    # Rust ID → gdsfactory component
    - metadata: dict[int, PrimitiveMetadata]  # Rust ID → angle/length/cost info

Total primitives: 48 (6 primitives × 8 angles)
  ├─ For each angle (0-7, representing E/NE/N/NW/W/SW/S/SE):
  │   ├─ Primitive 0: straight short (1 cell = 0.5 μm)
  │   ├─ Primitive 1: straight long (4 cells = 2.0 μm)
  │   ├─ Primitive 2: turn left 45° (radius 1.0 μm)
  │   ├─ Primitive 3: turn right 45° (radius 1.0 μm)
  │   ├─ Primitive 4: turn left 90° (radius 1.0 μm)
  │   └─ Primitive 5: turn right 90° (radius 1.0 μm)

Primitive ID mapping:
  Angle 0 (East): IDs 0-5
  Angle 1 (NE):   IDs 6-11
  Angle 2 (N):    IDs 12-17
  ... (pattern repeats for 8 angles)

Usage:
  lib = get_primitive_library()  # Singleton, lazy-loaded on first use
  comp = lib.get_component(prim_id)
  meta = lib.get_metadata(prim_id)


=== RUST SIDE (UNCHANGED FOR NOW) ===

Rust router already returns:
  result = {
    "primitives": [4, 5, 12, 13, ...],  # Primitive IDs used
    "states": [(8,20,0), (9,20,0), (10,20,0), ...],  # Grid (x,y,angle)
    "cells": [...],     # Swept cells for obstacle update
    "total_length_um": 65.2,
    "total_cost": 142.5,
    "svg": "<svg>..."   # Optional debug SVG
  }

No changes needed to Rust side for this integration!


=== PYTHON PLACEMENT LOGIC ===

Location: translation/route_rust.py, function _place_primitives_from_result()

Process for each net:
  1. Get primitives list and states from Rust result
  2. For each (primitive_id, state):
     a. Look up gdsfactory component from library
     b. Convert grid state (x_grid, y_grid, angle) to physical coords
     c. Place component at (x_um, y_um) with rotation (angle * 45°)
     d. Add as reference to layout

Pseudocode:
  for i, prim_id in enumerate(primitives_used):
    state = states[i]
    x_grid, y_grid, angle = state
    
    # Grid → physical conversion
    x_um, y_um = grid_cell_center(x_grid, y_grid, grid)
    
    # Get component
    component = primitive_lib.get_component(prim_id)
    
    # Place with rotation
    ref = layout.add_ref(component)
    ref.rotate(angle * 45.0)
    ref.move((x_um, y_um))

Result: Layout now contains placed primitive components forming the routed waveguide.


=== EXAMPLE: ROUTING gc_0 → mmi_0 ===

Assuming Rust returns:
  primitives: [0, 0, 2, 0]           # straight, straight, turn_left_45, straight
  states: [(8,20,0), (9,20,0), (10,20, 1), (11,21,1)]

Placement sequence:
  1. Primitive 0 at (8*0.5, 20*0.5) = (4.0, 10.0) with rotation 0°
     → straight (0.5 μm) heading East

  2. Primitive 0 at (4.5, 10.0) with rotation 0°
     → straight (0.5 μm) heading East

  3. Primitive 2 at (5.0, 10.0) with rotation 45°
     → bend left 45° (radius 1.0 μm), now heading NE

  4. Primitive 0 at (5.5, 10.25) with rotation 45°
     → straight (0.5 μm) heading NE

Result: 4 gdsfactory components placed end-to-end forming the waveguide path.


=== ADVANTAGES ===

1. **Fidelity**: Uses exact primitives that A* searched with
2. **Modularity**: Easy to swap primitive components later
3. **Performance**: No path smoothing/extrusion overhead
4. **Testability**: Can verify primitive placement independently
5. **Design rule compliance**: Primitives prebuilt to fab spec


=== FUTURE ENHANCEMENTS ===

1. Pass primitive library to Rust
   - Rust can use exact costs during search
   - Enable primitive-specific constraints

2. Map primitive IDs to actual mask layers
   - Different bends for different technologies
   - Bend radius optimization per layer

3. Primitive variants
   - Add S-bend, spiral, directional coupler primitives
   - Extend library to 60+ primitives for complex routing

4. Cost optimization
   - Tune bend_cost in primitive metadata
   - Reflect real loss/delay per bend

5. Validation
   - Check placed primitives don't overlap
   - Verify port connections align


=== CODE LOCATIONS ===

New files:
  - python/photonic_router/primitive_library.py

Modified files:
  - translation/route_rust.py
    • Removed: _route_result_to_layout, _route_points_from_result, _route_width
    • Added: _place_primitives_from_result
    • Changed: Call primitive placement instead of path extrusion

Expected behavior:
  - routing_flow.py works the same externally
  - Internally, routes now use placed primitives
  - final layout shows real primitive components, not extruded paths
  - debug SVGs and KLayout viewing unaffected
"""

if __name__ == "__main__":
    # Example: show all primitives in library
    import sys
    sys.path.insert(0, "python")
    from photonic_router.primitive_library import get_primitive_library
    
    lib = get_primitive_library()
    print("Primitive Library Summary:")
    print(f"Total primitives: {len(lib.get_all_metadata())}")
    print("\nPrimitives by angle (0=East, 2=North, 4=West, 6=South):")
    
    for angle in [0, 2, 4, 6]:
        print(f"\n  Angle {angle} ({['E', 'N', 'W', 'S'][angle//2]}):")
        for offset in range(6):
            prim_id = angle * 6 + offset
            meta = lib.get_metadata(prim_id)
            if meta:
                desc = [
                    "straight short",
                    "straight long",
                    "turn left 45",
                    "turn right 45",
                    "turn left 90",
                    "turn right 90",
                ][offset]
                print(f"    ID {prim_id:2d}: {desc:20s} (end angle: {meta.end_angle})")

