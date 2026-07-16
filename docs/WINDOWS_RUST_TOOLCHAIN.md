# Windows Rust Toolchain

This repository builds the Python extension through PyO3/maturin. On this
Windows workspace, the normal Rust default host was `x86_64-pc-windows-msvc`,
but the Visual Studio C++ linker (`link.exe`) is not installed. That makes plain
MSVC builds fail during dependency build scripts.

The repository therefore pins the local Rust toolchain to:

```text
stable-x86_64-pc-windows-gnullvm
```

and configures Cargo to link with the `rust-lld.exe` shipped by that toolchain.
This avoids any dependency on Visual Studio Build Tools.

## Files That Make This Work

- `rust-toolchain.toml` selects `stable-x86_64-pc-windows-gnullvm`.
- `.cargo/config.toml` sets:
  - `target.x86_64-pc-windows-gnullvm.linker` to the toolchain's `rust-lld.exe`.
  - `PYO3_PYTHON` to the project virtualenv Python.

## Expected Commands

From the repository root:

```powershell
C:\Users\benja\.cargo\bin\cargo.exe check
.\.venv\Scripts\python.exe -m maturin develop --release
```

The second command rebuilds and reinstalls `photonic_router._rust` into the
project virtualenv/editable package.

## If It Fails Again

Check these first:

```powershell
C:\Users\benja\.cargo\bin\rustup.exe show
C:\Users\benja\.cargo\bin\rustup.exe component list --toolchain stable-x86_64-pc-windows-gnullvm --installed
Test-Path C:\Users\benja\.rustup\toolchains\stable-x86_64-pc-windows-gnullvm\lib\rustlib\x86_64-pc-windows-gnullvm\bin\rust-lld.exe
Test-Path .\.venv\Scripts\python.exe
```

The known bad states are:

- `link.exe not found`: Cargo/maturin is using the MSVC toolchain instead of
  the pinned gnullvm toolchain.
- `x86_64-w64-mingw32-clang not found`: Cargo is using gnullvm without the
  repository linker override.
- `no Python 3.x interpreter found`: PyO3 cannot see the virtualenv Python;
  verify `.cargo/config.toml` still sets `PYO3_PYTHON`.

Do not fix these by switching back to MSVC unless Visual Studio Build Tools with
the C++ toolchain are intentionally installed. The repo-local path is gnullvm
plus `rust-lld.exe`.
