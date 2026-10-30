# Contributing

TUMPhotonicRouter is a Rust + Python photonic waveguide router. This page is
how to build it, check a change, and commit it.
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) is the map of the code and
[`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) lists every setting.

## Build

Needs a stable Rust toolchain (`rust-toolchain.toml` selects `stable`) and
Python 3.12.

```bash
python3.12 -m venv .venv
.venv/bin/pip install -e ".[dev]" pytest     # builds the Rust extension through maturin
```

The Rust side is a Cargo workspace: `crates/router-core` is the engine (no
Python dependency), `crates/router-py` the PyO3 bindings that maturin builds
into `photonic_router._rust` (`pyproject.toml`'s `manifest-path`). Rebuild
the extension after a change under `crates/`:

```bash
.venv/bin/maturin develop --release
```

Windows without Visual Studio: [`docs/WINDOWS_RUST_TOOLCHAIN.md`](docs/WINDOWS_RUST_TOOLCHAIN.md).

## Test

```bash
cargo test --release -p router-core                  # the engine; needs no Python
PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --release -p router-py   # the bindings link the Python they are built for
.venv/bin/python -m pytest -q -m "not e2e" tests     # the fast suite, under 60 s
.venv/bin/python -m pytest -q tests                  # with the end-to-end runs
scripts/test_baseline.sh                             # both suites against tests/baselines/test_baseline.txt
```

`scripts/test_baseline.sh` passes when the Rust suite has no failure, the
Python failure list equals the pinned one (empty), neither pass count has
dropped below the pin and the fast suite stays under 60 s. A change that
adds or removes tests re-pins it with `TEST_BASELINE_UPDATE=1
scripts/test_baseline.sh`.

Tests sit next to what they test: Rust tests in their module (fixtures in
`search/test_support.rs`, `engine/test_support.rs`), Python unit tests in
`tests/`, full runs in `tests/e2e/` (marker `e2e`), the electrical stages in
`tests/electrical/`, synthetic layouts in `tests/fixtures/` only.

## Lint

```bash
scripts/lint.sh          # check
scripts/lint.sh --fix    # apply ruff's and clippy's safe fixes
```

It runs `ruff format --check`, `ruff check`, the generated-file checks (below),
`mypy`, `cargo fmt --all -- --check` and `cargo clippy --workspace
--all-targets -- -D warnings`. CI (`.github/workflows/ci.yml`) runs the same
checks, the Rust tests and the fast Python suite on every push, and the paper
gate nightly.

## The two gates

A change that can move a routed result runs both; they take a few minutes
each and print one line that decides.

```bash
scripts/results/gate_short.sh    # nine paper cells -> "all reproduced cells match"
scripts/results/plm_gate.sh      # two PLM cells    -> "all PLM cells match"
```

`gate_short.sh` compares crossings, GDS length and verifier errors of the
nine smallest DATE 2027 cells with `docs/date2027_table_sources.json`;
`plm_gate.sh` compares the path-length-matching quantities of `heater_s_mod`
and `cascade_8_compact` with `docs/plm_baseline.json`. The full runs behind
them are `scripts/results/reproduce_date2027.sh` (27 cells, about 2.5 h) and
`scripts/results/plm_baseline.sh` (13 cells); see
[`docs/DATE2027_REPRODUCTION.md`](docs/DATE2027_REPRODUCTION.md) and
[`docs/PLM_REPRODUCTION.md`](docs/PLM_REPRODUCTION.md). A change to the
electrical router also runs
`.venv/bin/python scripts/benchmark_electrical.py --suite --check --compare-baseline`.

**A change that moves a pinned number re-pins it in the same commit, with the
reason.** This holds for both oracles. For the paper gate the paper's table
itself does not move: a commit that changes a reproduced crossing count or
length says so in its message, runs the full 27-cell reproduction, and
records the new numbers and their provenance in
`docs/DATE2027_REPRODUCTION.md` (and the bounds in
`docs/date2027_crossing_bounds.json` when the placement changes). For the PLM
gate the commit re-runs `plm_baseline.sh` and re-pins `docs/plm_baseline.json`
(`plm_table.py results_plm --json docs/plm_baseline.json`). The same rule
applies to `tests/baselines/`. A commit never moves a number silently.

## Generated files

Edit the generator or its sources, never the output; CI and the tests fail
when an output differs from a fresh generation.

| file | regenerate with | when |
| --- | --- | --- |
| `docs/CONFIGURATION.md` | `.venv/bin/python scripts/generate_config_reference.py` | a configuration field, overlay variable or flag changes |
| `python/photonic_router/_rust.pyi` | `.venv/bin/python scripts/generate_rust_stubs.py` (after `maturin develop`) | a binding in `crates/router-py/` changes |
| `docs/date2027_crossing_bounds.json` | `.venv/bin/python scripts/results/crossing_lower_bound.py --all-paper-cells --json docs/date2027_crossing_bounds.json` | a paper benchmark's placement changes |
| `docs/plm_baseline.json` | `.venv/bin/python scripts/results/plm_table.py results_plm --json docs/plm_baseline.json` | a PLM number moves on purpose |
| `tests/baselines/test_baseline.txt` | `TEST_BASELINE_UPDATE=1 scripts/test_baseline.sh` | tests are added or removed |
| `tests/baselines/electrical_suite_metrics.json` | `.venv/bin/python scripts/benchmark_electrical.py --suite --output tests/baselines/electrical_suite_metrics.json` | an electrical number moves on purpose |
| `benchmarks/<slice>.py`, `.toml` | `.venv/bin/python scripts/make_mesh_slice_benchmark.py` | a mesh slice is added |

Both `--check` options (`generate_config_reference.py --check`,
`generate_rust_stubs.py --check`) verify without writing.

## Style

- Python: `ruff format` and the `ruff check` rules in `pyproject.toml`. A new
  ignore carries a comment with its reason.
- Rust: `cargo fmt`, `cargo clippy -D warnings`. A crate- or workspace-level
  `allow` names the debt it stands for.
- Comments explain why, not what. Parameters that travel together become a
  dataclass or struct. One implementation per concern: before adding a
  helper, search for the one that exists.
- A configuration field is added on both sides (`crates/router-core/src/config.rs`,
  `python/photonic_router/config.py`, with an `ENV_OVERLAY` entry when it has
  a variable) and `docs/CONFIGURATION.md` is regenerated; a binding change
  regenerates `_rust.pyi`.
- No algorithm reads the environment; no stage reads the command line
  (`docs/ARCHITECTURE.md` section 10).

## Commits

One slice per commit: a change that can be reverted on its own. The message:

- an imperative subject naming the slice, e.g. `Phase 8 slice C: benchmark
  defaults as benchmarks/<name>.toml`;
- a body that says what changed and why, in prose, with the files or
  modules it touches;
- the checks it ran and their result: the gate lines (`all reproduced cells
  match`, `all PLM cells match`), the test counts, and for a behaviour change
  the numbers that moved and the re-pin;
- any attribution trailers last.

Keep `benchmarks/private/` and any foundry GDS out of the repository; the
extraction scripts work on files read in place. Results roots
(`results*/`) are not versioned.

## Where things go

| What | Where |
| --- | --- |
| A benchmark | `benchmarks/<name>.py` with `build_schematic()`, defaults in `benchmarks/<name>.toml` (`docs/ARCHITECTURE.md` section 8.2) |
| A search engine or negotiation policy | `docs/ARCHITECTURE.md` sections 4 and 5 |
| A routing phase | `python/photonic_router/routing/`, typed by a Protocol in `stages.py` |
| An electrical stage | `python/photonic_router/electrical/`, typed by a Protocol in `stages.py` |
| A configuration field | both config trees, the overlay table, then regenerate `docs/CONFIGURATION.md` |
