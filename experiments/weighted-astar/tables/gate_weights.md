# Nine-cell gate with PHOTONIC_ROUTER_EXP_HEURISTIC_WEIGHT (ec84bb2 + patch 0001, workstation, 2026-11-09)

Switch off (unset = weight 1.0): all reproduced cells match.

## weight 1.05

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9484.511 | 0 | 0.75 | ok |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.03 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 0.90 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31418.196 | 0 | 0.58 | MISMATCH length |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 1.01 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 0.93 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25207.717 | 0 | 0.66 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25106.899 | 0 | 0.69 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26345.499 | 0 | 0.89 | MISMATCH length |

## weight 1.10

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9484.511 | 0 | 0.64 | ok |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.03 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 0.91 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31418.196 | 0 | 0.52 | MISMATCH length |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 0.99 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 0.93 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25351.732 | 0 | 0.71 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25068.67 | 0 | 0.84 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26345.499 | 0 | 0.87 | MISMATCH length |

## weight 1.25

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9478.951 | 0 | 0.53 | MISMATCH length |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.00 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 0.92 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31469.565 | 0 | 0.45 | MISMATCH length |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 0.97 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 0.92 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25307.401 | 0 | 0.74 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25075.006 | 0 | 0.61 | MISMATCH length (current) |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26345.499 | 0 | 0.87 | MISMATCH length |

## routing loop seconds per cell and search attempts (same machine, one run each)

```
benes_4x4_flat       contribution1  off:   0.40 s / 40 attempts  1.05:   0.41 s / 40 attempts  1.10:   0.40 s / 40 attempts  1.25:   0.39 s / 40 attempts
benes_4x4_flat       contribution2  off:   0.61 s / 44 attempts  1.05:   0.59 s / 44 attempts  1.10:   0.60 s / 44 attempts  1.25:   0.60 s / 44 attempts
benes_4x4_flat       lidar-pure     off:   1.61 s / 42 attempts  1.05:   1.54 s / 42 attempts  1.10:   1.32 s / 42 attempts  1.25:   1.08 s / 42 attempts
benes_8x8_flat       contribution1  off:   1.21 s / 128 attempts  1.05:   1.22 s / 128 attempts  1.10:   1.20 s / 128 attempts  1.25:   1.17 s / 128 attempts
benes_8x8_flat       contribution2  off:   1.71 s / 160 attempts  1.05:   1.70 s / 160 attempts  1.10:   1.70 s / 160 attempts  1.25:   1.67 s / 160 attempts
benes_8x8_flat       lidar-pure     off:  13.92 s / 132 attempts  1.05:  12.84 s / 132 attempts  1.10:  11.58 s / 132 attempts  1.25:  10.12 s / 132 attempts
multiportmmi_8x8     contribution1  off:   3.19 s / 115 attempts  1.05:   2.61 s / 115 attempts  1.10:   3.17 s / 120 attempts  1.25:   2.30 s / 120 attempts
multiportmmi_8x8     contribution2  off:   1.91 s / 224 attempts  1.05:   1.86 s / 224 attempts  1.10:   1.81 s / 224 attempts  1.25:   1.80 s / 224 attempts
multiportmmi_8x8     lidar-pure     off:   6.34 s / 119 attempts  1.05:   6.12 s / 115 attempts  1.10:   6.60 s / 120 attempts  1.25:   6.86 s / 122 attempts
total off: 30.9 s  1.05: 28.9 s  1.10: 28.4 s  1.25: 26.0 s
```
