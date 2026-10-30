# Contributing

TUMPhotonicRouter is a Rust + Python photonic waveguide router. This page is
the short version of how to build it, check a change, and get it merged;
`docs/ARCHITECTURE.md` is the map of the code and `docs/CONFIGURATION.md`
lists every setting.

## Build

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]" pytest     # builds the Rust extension through maturin
```

Rebuild the extension after a change under `src/`:

```bash
.venv/bin/maturin develop --release
```

`rust-toolchain.toml` pins the host's stable toolchain. Windows without
Visual Studio: `docs/WINDOWS_RUST_TOOLCHAIN.md`.

## Check a change

Run the narrowest check first, then the gates. All of these run in CI.

```bash
# style and generated files
.venv/bin/ruff check . && .venv/bin/ruff format --check .
cargo fmt --all -- --check
PYO3_PYTHON="$PWD/.venv/bin/python" cargo clippy --release --lib --tests -- -D warnings
.venv/bin/python scripts/generate_config_reference.py --check
.venv/bin/python scripts/generate_rust_stubs.py --check

# tests
PYO3_PYTHON="$PWD/.venv/bin/python" cargo test --release --lib
.venv/bin/python -m pytest -q -m "not e2e" tests     # under 60 s
.venv/bin/python -m pytest -q tests                  # with the end-to-end tests

# the paper's numbers: nine smallest cells, about two minutes
scripts/results/gate_short.sh
```

A change that can touch the optical router runs `gate_short.sh`; a change
that intends to move a paper number says so in the commit message and
`docs/DATE2027_REPRODUCTION.md` records the new provenance. A change to the
electrical router runs
`scripts/benchmark_electrical.py --suite --check --compare-baseline`.

## Style

- Python: `ruff format` and the `ruff check` rules in `pyproject.toml`. A new
  ignore needs a comment with its reason next to it.
- Rust: `cargo fmt`, `cargo clippy -D warnings`. A crate-level `allow` names
  the debt it stands for and the plan that removes it.
- Comments explain why, not what. Parameters that travel together become a
  dataclass or struct. One implementation per concern: before adding a
  helper, search for the one that exists.
- A configuration field is added on both sides (`src/config.rs`,
  `python/photonic_router/config.py`) and `docs/CONFIGURATION.md` is
  regenerated; a binding change regenerates `_rust.pyi`.

## Commits and pull requests

- One slice per commit: a change that can be reverted on its own, with the
  check it passed named in the message.
- Branch from `main`; open a pull request; CI must be green. Behaviour
  changes reference the benchmark evidence (SUMMARY line, gate output).
- Keep `benchmarks/private/` and any foundry GDS out of the repository; the
  extraction scripts work on files you read in place.

## Where things go

| What | Where |
| --- | --- |
| A benchmark | `benchmarks/<name>.py` with `build_schematic()` (see `benchmarks/cascade.py`) |
| A search engine or negotiation policy | `docs/ARCHITECTURE.md` sections 4 and 5 |
| An electrical stage | `python/photonic_router/electrical/` after `stages.py` |
| Tests | `tests/` (unit), `tests/e2e/` (full runs), `tests/electrical/`; synthetic layouts in `tests/fixtures/` only; Rust tests next to their module |
