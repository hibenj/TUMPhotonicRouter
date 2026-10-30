# TUMPhotonicRouter

A grid-based waveguide router for photonic integrated circuits. It takes a
placed [gdsfactory](https://gdsfactory.github.io/gdsfactory/) schematic
(instances, placements, nets between ports) and writes a routed GDS in which
every net is a waveguide that keeps the bend radius and the clearance, and
every crossing of two waveguides is a crossing cell.

- **Single-net search**: A* over grid states `(x, y, heading)` with eight
  headings and a primitive set of straights, 45-degree and 90-degree bends of
  one fixed bend radius (rounded up to whole grid cells); a straight/L/Z
  shortcut is tried before the search.
- **Legal crossings**: two waveguides may cross only perpendicularly, with a
  straight margin on both sides; each realized crossing becomes a crossing
  component in the GDS.
- **Negotiated rip-up and reroute**: a failing net asks who blocks it, rips
  them up, and the loop iterates with history costs until every net routes.
- **Pre-placed crossing grids**: the crossings a placement forces (band
  inversions between component columns) can be placed as crossing tiles
  before routing, so the rest routes crossing-free.
- **Path-length matching**: the routed lengths are analysed along the light
  path and meanders are inserted where a group of paths must arrive equal.
- **Heater electrical routing**: heater terminals, a common bus, pad
  planning and pad wires on a metal layer, with their own verification.

The search, the routing database and the meander planner are Rust
(`crates/`); gdsfactory integration, the stage sequence, verification and
every artifact are Python (`python/photonic_router/`).

## Install

Needs a stable Rust toolchain (`rust-toolchain.toml` selects `stable`;
install it with [rustup](https://rustup.rs)) and Python 3.12 (the version CI
runs). From the repository root:

```bash
python3.12 -m venv .venv
.venv/bin/pip install -e ".[dev]" pytest     # builds the Rust extension through maturin
.venv/bin/maturin develop --release          # rebuild after a change under crates/
```

The extension lands at `python/photonic_router/_rust.abi3.so`. Windows hosts
without Visual Studio: [`docs/WINDOWS_RUST_TOOLCHAIN.md`](docs/WINDOWS_RUST_TOOLCHAIN.md).

## Quick start

```bash
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration baseline
# the same command line through the repository-root shim:
.venv/bin/python routing_flow.py benes_8x8_flat --crossing-mode lidar-pure
```

The benchmark name is a module under `benchmarks/`; its defaults
(`benchmarks/<name>.toml`) are applied first and any flag given on the
command line overrides them. The routed layout is written to
`build/routed_<benchmark>.gds`, the verification reports to
`build/verification/`, and the run ends with one grep-able line:

```text
SUMMARY benchmark=benes_4x4_flat nets=40 route_failures=2 detours=- verifier_errors=0 verifier_warnings=0 plm_groups=- plm_over_tolerance=- meander_inserted_um=- electrical_heaters=- electrical_pads=- electrical_failed_wires=- electrical_ok=- total_s=2.1
```

`verifier_errors=0` is the verdict on the geometry; `route_failures` counts
failed search attempts that the negotiation later repaired; `-` means the
stage did not run. Path-length matching and heater routing are on by default
for `heater_s_mod` (see `benchmarks/heater_s_mod.toml`):

```bash
.venv/bin/python -m photonic_router route heater_s_mod
```

Boolean flags take a value (`--path-length-matching true`). Every flag and
setting, with its default, is in [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md);
`--debug-svgs all` writes obstacle and per-route SVGs under `build/`.

## Benchmarks and the DATE 2027 numbers

`benchmarks/` holds Benes networks (`benes_<n>x<n>`, and the `_flat`
variants with every switch expanded into its primitives), ADEPT-style
multiport-MMI meshes (`multiportmmi_<n>x<n>` and their band slices),
path-length-matching cascades (`cascade_{8,12,16}_*`), heater benchmarks
(`heater_s_mod`, `heater_single`, `heater_lanes_*`) and a Clements mesh.

The paper compares three configurations; `--configuration` names them, and
exactly one runs at a time:

| `--configuration` | expands to | what it does |
| --- | --- | --- |
| `baseline` | `--crossing-mode lidar-pure` | crossings are discovered during search |
| `contribution1` | `--crossing-mode lidar-guided` | the placement's crossing plan guides costs and order |
| `contribution2` | `--preplaced-crossing-grids true --verbose-routes` | crossing tiles are placed before routing |

The 27 cells of ours in the paper's results table: crossings (X) and GDS
waveguide length in um (L), from
[`docs/date2027_table_sources.json`](docs/date2027_table_sources.json).
"Bound" is the placement-derived lower bound on crossings, the sum of band
inversions between component columns
([`docs/date2027_crossing_bounds.json`](docs/date2027_crossing_bounds.json)):
no routing that keeps each net inside its band can have fewer.

| cell | benchmark | bound | baseline X | baseline L | contr. 1 X | contr. 1 L | contr. 2 X | contr. 2 L |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Benes 4x4 | `benes_4x4_flat` | 2 | 2 | 9484.511 | 2 | 9475.187 | 2 | 9475.255 |
| Benes 8x8 | `benes_8x8_flat` | 16 | 16 | 31418.224 | 16 | 31312.203 | 16 | 31348.890 |
| Benes 16x16 | `benes_16x16_flat` | 88 | 88 | 92323.991 | 88 | 92106.172 | 88 | 93654.793 |
| Benes 32x32 | `benes_32x32_flat` | 416 | 416 | 270827.610 | 416 | 270872.224 | 416 | 277886.703 |
| Benes 64x64 | `benes_64x64_flat` | 1824 | - | - | - | - | 1824 | 1159313.330 |
| Benes 128x128 | `benes_128x128_flat` | 7680 | - | - | - | - | 7680 | 4938323.005 |
| ADEPT 8x8 | `multiportmmi_8x8` | 33 | 33 | 25160.335 | 33 | 25073.626 | 33 | 26319.970 |
| ADEPT 16x16 | `multiportmmi_16x16` | 63 | 63 | 90200.066 | 63 | 89586.040 | 63 | 96331.277 |
| ADEPT 32x32 | `multiportmmi_32x32` | 121 | 121 | 336038.575 | 121 | 334816.567 | 121 | 360764.408 |
| ADEPT 64x64 | `multiportmmi_64x64` | 209 | 217 | 1305628.208 | 209 | 1302769.960 | 209 | 1408033.109 |
| ADEPT 128x128 | `multiportmmi_128x128` | 311 | - | - | - | - | 311 | 5573562.981 |

Every cell routes with zero verifier errors. The realized crossing count
equals the bound everywhere except ADEPT 64x64 under the baseline (+8), which
both contributions close.

## Reproducing the numbers

```bash
scripts/results/gate_short.sh                    # nine smallest cells, about two minutes
scripts/results/reproduce_date2027.sh            # all 27 cells, about 2.5 h, into results_date2027/
.venv/bin/python scripts/results/compare_date2027.py results_date2027 \
  docs/date2027_table_sources.json --with-bounds
scripts/results/plm_gate.sh                      # path-length matching: two cells
scripts/results/plm_baseline.sh                  # all thirteen PLM cells, into results_plm/
```

`gate_short.sh` ends with `all reproduced cells match` when crossings and GDS
length are exact and the verifier reports no error. Provenance, the engine
commits behind each row and the archived runs (attached to the
`date2027-results` release) are in
[`docs/DATE2027_REPRODUCTION.md`](docs/DATE2027_REPRODUCTION.md); the
path-length-matching baseline is in
[`docs/PLM_REPRODUCTION.md`](docs/PLM_REPRODUCTION.md).

## How it is built

The pipeline is: schematic, unrouted layout, static obstacle map, route
jobs, crossing plan, the negotiated loop in the Rust kernel, endpoint
correction, path-length matching, realization, verification, GDS.
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) describes each stage, the data
model, the two Rust crates (`router-core`, the engine with no Python
dependency, and `router-py`, the PyO3 bindings) and the Python package, the
configuration surface, the oracles and how to add a benchmark.
[`CONTRIBUTING.md`](CONTRIBUTING.md) has the build, test and lint commands.

## Status

Research code. It produced the results of the DATE 2027 paper and is being
restructured (see [`CHANGELOG.md`](CHANGELOG.md)); the Python and Rust APIs
are not stable and there is no versioned release yet.

**License: pending** (owner decision; there is no `LICENSE` file yet, so no
rights are granted until one is added). How to cite: [`CITATION.cff`](CITATION.cff).
