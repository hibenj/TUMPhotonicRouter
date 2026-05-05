"""
QUICK START EXAMPLES FOR ROUTING FLOW

This file shows practical examples of how to use the routing flow.
"""

# ============================================================================
# EXAMPLE 1: Run Benchmark with GUI Visualization
# ============================================================================

from routing_flow import run_routing_flow

# Run the TOY benchmark and display in klive viewer
layout = run_routing_flow("TOY", show_layout=True)


# ============================================================================
# EXAMPLE 2: Load Benchmark and Inspect Programmatically
# ============================================================================

from routing_flow import load_benchmark
from translation import layout_from_schematic

# Load the schematic
schematic = load_benchmark("TOY")

# Inspect instances and placements
print("Instances in schematic:")
for name, instance in schematic.netlist.instances.items():
    print(f"  {name}: {instance.component}")

print("\nPlacements:")
for name, placement in schematic.placements.items():
    print(f"  {name}: x={placement.x}, y={placement.y}, rotation={placement.rotation}")

# Convert to layout
layout = layout_from_schematic(schematic)
print(f"\nLayout bbox: {layout.bbox}")


# ============================================================================
# EXAMPLE 3: Create Custom Benchmark
# ============================================================================

# File: benchmarks/MY_CHIP.py
"""
from gdsfactory.schematic import Instance, Placement, Schematic

def build_schematic() -> Schematic:
    schematic = Schematic()
    
    # Create multiple components
    for i in range(3):
        inst = Instance(component="waveguide")
        placement = Placement(x=i*100, y=0)
        schematic.add_instance(f"wg_{i}", inst, placement)
    
    return schematic
"""

# Then run it:
# from routing_flow import run_routing_flow
# layout = run_routing_flow("MY_CHIP")


# ============================================================================
# EXAMPLE 4: Access Layout and Prepare for Routing (Future)
# ============================================================================

# Once routing is implemented, you'll do:
#
# schematic = load_benchmark("TOY")
# unrouted_layout = layout_from_schematic(schematic)
#
# # Build routing database (TODO)
# routing_db = build_routing_database(unrouted_layout)
#
# # Route connections (TODO)
# routes = route_connections(routing_db)
#
# # Generate routed layout (TODO)
# routed_layout = apply_routes(unrouted_layout, routes)
# routed_layout.show()
# routed_layout.write_gds("output.gds")


# ============================================================================
# EXAMPLE 5: Batch Process Multiple Benchmarks
# ============================================================================

# benchmarks = ["TOY", "MY_CHIP", "ANOTHER"]
# for bench in benchmarks:
#     print(f"Processing {bench}...")
#     layout = run_routing_flow(bench, show_layout=False)
#     output_path = f"outputs/{bench}_unrouted.gds"
#     layout.write_gds(output_path)
#     print(f"  Saved to {output_path}")

