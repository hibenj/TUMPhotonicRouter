# Photonic Routing Baseline

- Captured: 2026-06-22T11:14:58+02:00
- Git revision: `ea2e096`
- Python: `3.12.3`
- Path-length matching: `False`
- 45-degree turns: `False`
- Heater obstacles: `True`
- Obstacle mode: `bounding_boxes`
- Max iterations: `5000000`

| Benchmark | Instances | Nets | Grid | Total s | Load s | Layout s | Route s |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| TOY | 5 | 4 | 645x332 | 0.0591 | 0.0424 | 0.0030 | 0.0120 |
| mmi_heater | 7 | 7 | 1805x292 | 0.1797 | 0.1130 | 0.0025 | 0.0350 |
| mmi_heater_8x4 | 61 | 78 | 13005x1252 | 1.0004 | 0.1044 | 0.0070 | 0.8565 |

## Path-Length Matching Check

Same settings as above, with path-length matching enabled.

| Benchmark | Instances | Nets | Grid | Total s | Load s | Layout s | Route + Match s |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| mmi_heater_8x4 | 61 | 78 | 13005x1252 | 5.6325 | 0.1051 | 0.0076 | 5.4678 |
