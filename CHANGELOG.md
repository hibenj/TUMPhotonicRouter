# Changelog

All notable changes to this repository. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the project has no
versioned release yet.

## Unreleased

The do-over of 2026-10 (analysis and phase plan: the owner's
`working/.agent/TUMPhotonicRouter/DO_OVER_ANALYSIS.md`). Phase 0, hygiene
and freeze, changes no routing behaviour; the nine-cell paper reproduction
(`scripts/results/gate_short.sh`) is the check for every slice.

### Changed

- `rust-toolchain.toml` pins the host's `stable` toolchain. Linux and macOS
  build with plain `cargo` and `maturin`; the Windows-without-Visual-Studio
  setup is a local, git-ignored configuration described in
  `docs/WINDOWS_RUST_TOOLCHAIN.md`.
- The Python sources are formatted by `ruff format` and pass `ruff check`
  with the rule set in `pyproject.toml`; the remaining scoped ignores are
  listed there with their reasons.
- `docs/CONFIGURATION.md` regenerated (a union type's member order).
- The simple-route (straight/L/Z) shortcut runs in one place, the engine's
  probe before each ordinary search; the second copy inside the single-net
  search body is gone (phase 2 slice B). The static-only probe keeps it
  explicitly. Routes are unchanged on the nine-cell gate; the single-net
  `route_single_net` API now always runs the kernel.

### Added

- `python/photonic_router/_rust.pyi`, the type stub of the Rust extension,
  generated from the built module by `scripts/generate_rust_stubs.py`, and
  `py.typed`.
- GitHub Actions CI (`.github/workflows/ci.yml`): rustfmt, clippy, the Rust
  tests, ruff, the generated-file checks and the fast Python suite on every
  push; the nine-cell reproduction gate nightly.
- `CONTRIBUTING.md`, `CHANGELOG.md`, `.pre-commit-config.yaml`.

### Removed

- The JPS4 accelerator (`src/search/astar/dense.rs`), the two unit-grid
  primitive libraries (`jps4_unit_grid`, `grid4_unit_grid`) and the
  `--enable-jps4` flag (phase 2 slice A1). JPS4 was eligible only for a
  plain 4-connected unit primitive library, which the photonic library never
  is, so no production run could reach it; the `jps4_*` route-stat fields go
  with it.
- The pure-Python static obstacle builder
  (`build_static_obstacle_map_python_from_extracted`, its rasterise and
  point-in-polygon helpers) and the legacy-signature retry shim around the
  Rust builder (phase 2 slice A2). Routing needs the extension anyway;
  without it `build_static_obstacle_map` now raises an `ImportError` with a
  rebuild hint. A heater layer group without geometry keeps a 20-line
  port-opening constructor so the split merge is unchanged.
- Tracked IDE settings (`.idea/`, `.vscode/`) and the per-machine
  `.cargo/config.toml`; `Agent_implementation_files/` (superseded by
  `docs/ARCHITECTURE.md`); `benes_test/` (an early experiment with three GDS
  files); `rust_router_reference/` (an unrelated pose router).
