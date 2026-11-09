# Nine-cell gate with the hook grid taking the kernel's dynamic expansion radius (1377ad6 + patch 0002, workstation, 2026-11-09)

Block radius of the paper cells: 1 cell (OpticalRouteClearancePolicy.from_dimensions, 0.5 um routes on 2 um cells).

## PHOTONIC_ROUTER_CROSSING_HOOK_GRID=radius

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9484.511 | 0 | 0.86 | ok |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.06 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 0.95 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31418.224 | 0 | 0.63 | ok |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 1.08 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 0.97 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25171.624 | 0 | 0.70 | ok (current) |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25090.058 | 0 | 0.84 | ok (current) |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26319.97 | 0 | 0.90 | ok |

## PHOTONIC_ROUTER_CROSSING_HOOK_GRID=radius-exempt

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9484.511 | 0 | 0.80 | ok |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.05 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 0.94 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31418.224 | 0 | 0.66 | ok |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 1.09 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 0.98 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25171.624 | 0 | 0.79 | ok (current) |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25090.058 | 0 | 0.92 | ok (current) |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26319.97 | 0 | 0.99 | ok |

Both: all reproduced cells match (identical to the pins in crossings, length and errors); switch off exact.
