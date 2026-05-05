"""PHOTONIC ROUTING FLOW ARCHITECTURE

This document describes the implementation of the first step of the photonic routing flow.

═══════════════════════════════════════════════════════════════════════════════

PROJECT STRUCTURE:

  Project1/
  ├── benchmarks/              # Benchmark definitions
  │   ├── __init__.py
  │   ├── TOY.py              # Example 2-component benchmark
  │   └── generate_toy.py     # (old, can be deprecated)
  │
  ├── translation/             # Translation layer
  │   ├── __init__.py
  │   └── layout_from_schematic.py  # Converts Schematic → Component
  │
  └── routing_flow.py          # Main orchestrator


═══════════════════════════════════════════════════════════════════════════════

CORE CONCEPTS:

1. SCHEMATIC (gdsfactory.schematic.Schematic)
   └─ Abstract blueprint describing the circuit
      ├─ Instances: which components to use
      ├─ Placements: where each component goes (x, y, rotation, mirror)
      ├─ Connections: (future) port-to-port links
      └─ Routes: (future) how signals are routed

2. LAYOUT (gdsfactory.component.Component)
   └─ Concrete GDS geometry
      ├─ Placed component instances
      ├─ (future) Routed connections
      └─ Can be visualized with .show()


═══════════════════════════════════════════════════════════════════════════════

WORKFLOW:

  1. Define Schematic in Python
     ↓
  2. Load via routing_flow.load_benchmark()
     ↓
  3. Translate to unrouted Layout via layout_from_schematic()
     ↓
  4. Visualize and inspect
     ↓
  5. (Future) Route connections
     ↓
  6. (Future) Generate final routed layout


═══════════════════════════════════════════════════════════════════════════════

QUICK START:

Run the example TOY benchmark:

  $ python routing_flow.py

This will:
  1. Load the TOY schematic from benchmarks/TOY.py
  2. Translate it to an unrouted layout
  3. Display the layout in the klive viewer


═══════════════════════════════════════════════════════════════════════════════

CREATE A NEW BENCHMARK:

1. Create a new file in benchmarks/ (e.g., benchmarks/MY_DESIGN.py)

2. Define a build_schematic() function that returns a Schematic:

   from gdsfactory.schematic import Instance, Placement, Schematic
   
   def build_schematic() -> Schematic:
       schematic = Schematic()
       
       # Add instances
       instance1 = Instance(component="component_name_1")
       instance2 = Instance(component="component_name_2")
       
       # Add placements
       placement1 = Placement(x=0, y=0, rotation=0)
       placement2 = Placement(x=100, y=50, rotation=90)
       
       # Attach to schematic
       schematic.add_instance("inst_1", instance1, placement1)
       schematic.add_instance("inst_2", instance2, placement2)
       
       return schematic

3. Run the routing flow:

   $ python routing_flow.py  # (will need to modify to use your benchmark)

   Or use the Python API:

   from routing_flow import run_routing_flow
   layout = run_routing_flow("MY_DESIGN", show_layout=True)


═══════════════════════════════════════════════════════════════════════════════

API REFERENCE:

routing_flow.py:
  Functions:
    - load_benchmark(benchmark_name: str) -> Schematic
      Load a benchmark schematic from benchmarks/ directory
      
    - run_routing_flow(benchmark_name: str, show_layout: bool = True) -> Component
      Execute the full routing flow for a benchmark (Steps 1-2 above)

translation/layout_from_schematic.py:
  Functions:
    - layout_from_schematic(schematic: Schematic) -> Component
      Convert a Schematic to an unrouted layout Component


═══════════════════════════════════════════════════════════════════════════════

FUTURE EXTENSIONS:

The architecture is designed to support these future additions:

  1. Routing Database Construction
     └─ Translate layout to routing graph/database
     └─ Extract obstacles and track information
  
  2. Routing Algorithms
     └─ A* pathfinding
     └─ Rip-up & reroute
     └─ Timing/length-aware routing
  
  3. Routed Layout Generation
     └─ Convert routed paths to geometric wires
     └─ Generate final GDS

Each step will be a separate module in the translation/ directory or a new
specialized directory, maintaining clean separation of concerns.


═══════════════════════════════════════════════════════════════════════════════

NOTES:

- The Python Schematic is the single source of truth (no YAML)
- This enables easy programmatic generation and parametric designs
- All benchmarks are pure Python functions for maximum flexibility
- The design supports adding more benchmarks without modifying routing_flow.py

═══════════════════════════════════════════════════════════════════════════════
"""

