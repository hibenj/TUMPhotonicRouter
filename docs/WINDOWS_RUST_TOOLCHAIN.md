# Building on Windows without Visual Studio

`rust-toolchain.toml` pins `stable`, which on Windows resolves to the MSVC
host toolchain. That toolchain needs the Visual Studio C++ linker
(`link.exe`). On a machine without Visual Studio Build Tools the build fails
in the first dependency's build script with `link.exe not found`.

The repository does not pin a Windows-specific toolchain or linker path, so
that Linux and macOS hosts build with plain `cargo` and `maturin`. A Windows
host without MSVC instead selects the GNU/LLVM toolchain locally, in two
files that git ignores:

1. Install the toolchain once:

   ```powershell
   rustup toolchain install stable-x86_64-pc-windows-gnullvm
   ```

2. Create `.cargo/config.toml` in the repository root (git-ignored) with
   the linker that toolchain ships and the virtualenv interpreter PyO3
   should build against. Replace `<user>` with your account name:

   ```toml
   [target.x86_64-pc-windows-gnullvm]
   linker = "C:\\Users\\<user>\\.rustup\\toolchains\\stable-x86_64-pc-windows-gnullvm\\lib\\rustlib\\x86_64-pc-windows-gnullvm\\bin\\rust-lld.exe"

   [env]
   PYO3_PYTHON = { value = ".venv/Scripts/python.exe", relative = true }
   ```

3. Build with the toolchain selected for the command:

   ```powershell
   $env:RUSTUP_TOOLCHAIN = 'stable-x86_64-pc-windows-gnullvm'
   cargo check
   .\.venv\Scripts\python.exe -m maturin develop --release
   ```

   or make it the machine default with
   `rustup default stable-x86_64-pc-windows-gnullvm`, after which the
   repository's `stable` pin resolves to it.

## If it fails

- `link.exe not found`: Cargo is using the MSVC toolchain; set
  `RUSTUP_TOOLCHAIN` as above or change the rustup default.
- `x86_64-w64-mingw32-clang not found`: the gnullvm toolchain is active but
  the local `.cargo/config.toml` linker override is missing.
- `no Python 3.x interpreter found`: PyO3 cannot see the virtualenv; check
  the `PYO3_PYTHON` entry in the local `.cargo/config.toml`.

Do not switch back to MSVC unless Visual Studio Build Tools with the C++
workload are intentionally installed.
