# Pass 18C - Metal Access Start-Cell Consolidation

## Scope

Pass 18C keeps the work metal-only. It consolidates electrical route-start
selection around explicit `ElectricalPortAccess` records introduced in 18B.
Photonic routing remains unchanged.

## Implementation Summary

- Added shared route-start helpers in `translation/electrical/port_access.py`:
  - `choose_route_start_cell()`
  - `ordered_route_start_cells()`
  - `RouteStartChoice`
- Common-bus BFS now uses `ordered_route_start_cells()` instead of local
  duplicated start ordering.
- Common-bus local-trunk routing now uses `choose_route_start_cell()` with
  target-x bias and `prefer_access_anchor=False`, making the local-trunk
  exception explicit.
- Individual topology now uses `ordered_route_start_cells()` for source-cell
  iteration.
- Detailed-route validation now accepts starts at either terminal-open cells or
  the selected access anchor.
- Route result records now carry:
  - `access_anchor_cell`
  - `route_start_cell`
  - `used_access_anchor`

## Debug And Metrics

Electrical debug SVG route titles now include route-start metadata. Verification
metrics now include:

- `port_access_route_start_count_by_purpose`
- `port_access_exact_anchor_route_count_by_purpose`
- `port_access_biased_route_count_by_purpose`
- `port_access_route_start_records`

The compact electrical benchmark JSON exposes the route-start count metrics.

## Verification

Commands run:

- `python3 -m py_compile ...`
- `.venv/bin/python -m pytest tests/test_electrical_routing.py -q`
- `.venv/bin/python -m pytest tests/test_benchmark_electrical_script.py -q`
- `.venv/bin/python scripts/benchmark_electrical.py mmi_heater --artifacts-dir build/18c_metal_access`
- `.venv/bin/python scripts/benchmark_electrical.py mmi_heater_8x4_ripup_reroute --artifacts-dir build/18c_metal_access`

Results:

- electrical routing tests: 36 passed, 1 skipped
- electrical benchmark-script tests: 15 passed
- `mmi_heater`: no guardrail violations
- `mmi_heater_8x4_ripup_reroute`: no guardrail violations

