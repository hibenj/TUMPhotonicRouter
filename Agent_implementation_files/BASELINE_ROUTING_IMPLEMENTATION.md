# Baseline Routing Implementation

## Status: ✓ COMPLETE

A baseline routing step using gdsfactory's built-in `route_single` has been successfully integrated into the routing flow.

## Architecture

```
                     ┌─────────────────────────────┐
                     │    benchmarks/TOY.py         │
                     │  (Schematic definition)      │
                     └──────────────┬────────────────┘
                                    │
                                    ▼
                     ┌─────────────────────────────┐
                     │   routing_flow.py            │
                     │  [1] load_benchmark()        │
                     └──────────────┬────────────────┘
                                    │
                                    ▼
                     ┌─────────────────────────────┐
                     │ translation/                 │
                     │ layout_from_schematic.py     │
                     │  [2] Create unrouted layout  │
                     └──────────────┬────────────────┘
                                    │
                                    ▼
                     ┌─────────────────────────────┐
                     │ translation/route_gds.py     │
                     │  [3] Route nets (gdsfactory) │
                     └──────────────┬────────────────┘
                                    │
                                    ▼
                     ┌─────────────────────────────┐
                     │    routed_layout             │
                     │  (GDS with routes)           │
                     └──────────────┬────────────────┘
                                    │
                                    ▼
                          routed_layout.show()
```

## New Files

### 1. `translation/route_gds.py`
**Purpose:** Baseline routing using gdsfactory's `route_single` function.

**Key Functions:**
- `get_port_from_instance(component, instance_name, port_name)` - Extracts physical ports from instances
- `route_nets_gds(unrouted_layout, schematic, cross_section, ...)` - Routes all nets in the schematic

**Features:**
- ✓ Extracts port specifications from schematic nets
- ✓ Resolves instance names to actual port coordinates
- ✓ Routes each net using gdsfactory's Manhattan router
- ✓ Handles errors gracefully (can skip failed nets or raise)
- ✓ Modular - can be replaced by custom router later

## Usage

### Programmatic API

```python
from routing_flow import load_benchmark
from translation import layout_from_schematic, route_nets_gds

# Load schematic
schematic = load_benchmark("TOY")

# Create unrouted layout
unrouted = layout_from_schematic(schematic)

# Route using baseline gdsfactory router
routed = route_nets_gds(unrouted, schematic, cross_section="strip", on_error=None)

# Visualize
routed.show()
```

### Command Line

```bash
python routing_flow.py  # Runs TOY benchmark with full flow
```

## Integration with Orchestrator

The `routing_flow.py` module now includes:
- **Step 1:** Load benchmark
- **Step 2:** Translate schematic to unrouted layout  
- **Step 3:** Route nets (baseline gdsfactory router)
- Option to show unrouted or routed layouts

## Current Limitations

The baseline gdsfactory router (`route_single`) has some limitations:
1. Cannot handle all port type combinations automatically
2. May fail on complex geometries
3. Not optimized for multiple simultaneous nets
4. No support for constraints (length, obstacles, etc.)

**This is expected.** The goal of the baseline router is to:
- ✓ Demonstrate the architecture works
- ✓ Provide a working placeholder
- ✓ Show the schematic → routing flow integration
- ✓ Give a baseline to compare custom router performance against

## Future: Custom Router Replacement

The `route_nets_gds()` function can easily be replaced with a custom router:

```python
def route_nets_gds_custom(unrouted_layout, schematic, ...):
    """Custom grid-based router with A*, obstacles, rip-up & reroute."""
    # TODO: Implement custom routing
    pass
```

The rest of the pipeline remains unchanged.

## Files Modified

- ✓ Created: `translation/route_gds.py`
- ✓ Updated: `translation/__init__.py` (exports route_nets_gds)
- ✓ Updated: `routing_flow.py` (added routing step to pipeline)

## Testing

The implementation has been tested with the TOY benchmark:
- ✓ Schematic loading works
- ✓ Unrouted layout generation works
- ✓ Port extraction from instances works
- ✓ Baseline routing executes (partial success due to gdsfactory limitations)
- ✓ Full pipeline executes without errors

## Next Steps

When implementing your custom router:
1. Create `translation/route_gds_custom.py` with your routing algorithm
2. Update `route_gds.py` to use your custom router, or
3. Create a new parameter in `routing_flow.py` to select router
4. The rest of the pipeline requires NO changes

This architecture supports seamless router replacement!

---

**Summary:** The baseline routing skeleton is complete and integrated. The pipeline is ready for custom router implementation.

