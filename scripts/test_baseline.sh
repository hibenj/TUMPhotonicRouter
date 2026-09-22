#!/bin/bash
# Test baseline check (ExecPlan 2026-09-22 Milestone 0). Runs the Rust library
# tests and the Python suite and compares against the pinned baseline in
# tests/baselines/test_baseline.txt (Rust pass count, Python pass count, and the
# sorted list of Python failures, which is empty once Milestone 0 is done).
# Exit 0 only when the Rust suite has no failures and the Python failure list
# equals the pinned one. Run from the repository root with the kernel built.
#   TEST_BASELINE_UPDATE=1  rewrites the pinned file from this run instead.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$R" || exit 1
export RUSTUP_TOOLCHAIN="${RUSTUP_TOOLCHAIN:-stable-x86_64-unknown-linux-gnu}"
export PYO3_PYTHON="${PYO3_PYTHON:-$R/.venv/bin/python}"
pin="$R/tests/baselines/test_baseline.txt"
tmp="$(mktemp -d "${TMPDIR:-/tmp}/test_baseline_XXXX")"
cargo test --release --lib > "$tmp/cargo.log" 2>&1
rust_line=$(grep -E "^test result:" "$tmp/cargo.log" | tail -1)
rust_passed=$(echo "$rust_line" | grep -o "[0-9]* passed" | grep -o "[0-9]*")
rust_failed=$(echo "$rust_line" | grep -o "[0-9]* failed" | grep -o "[0-9]*")
.venv/bin/python -m pytest -q tests > "$tmp/pytest.log" 2>&1
py_summary=$(grep -E "^[0-9]+ (passed|failed)|^=+ .*(passed|failed).* =+$" "$tmp/pytest.log" | tail -1 | sed 's/=//g; s/^ *//; s/ *$//')
py_passed=$(echo "$py_summary" | grep -o "[0-9]* passed" | grep -o "[0-9]*")
grep "^FAILED" "$tmp/pytest.log" | sed 's/ - .*//' | sort > "$tmp/failures.txt"
{
  echo "rust_passed=$rust_passed"
  echo "python_passed=$py_passed"
  echo "python_failures:"
  cat "$tmp/failures.txt"
} > "$tmp/current.txt"
if [ "${TEST_BASELINE_UPDATE:-0}" = 1 ]; then
  mkdir -p "$(dirname "$pin")"; cp "$tmp/current.txt" "$pin"; echo "pinned:"; cat "$pin"; rm -rf "$tmp"; exit 0
fi
echo "rust: $rust_line"
echo "python: $py_summary"
rc=0
[ "${rust_failed:-0}" = 0 ] || { echo "RUST FAILURES"; rc=1; }
if ! diff <(sed -n '/^python_failures:/,$p' "$pin") <(sed -n '/^python_failures:/,$p' "$tmp/current.txt"); then echo "PYTHON FAILURE LIST DIFFERS FROM BASELINE"; rc=1; fi
pinned_rust=$(grep "^rust_passed=" "$pin" | cut -d= -f2); pinned_py=$(grep "^python_passed=" "$pin" | cut -d= -f2)
[ "$rust_passed" -ge "$pinned_rust" ] || { echo "rust pass count dropped: $rust_passed < pinned $pinned_rust"; rc=1; }
[ "$py_passed" -ge "$pinned_py" ] || { echo "python pass count dropped: $py_passed < pinned $pinned_py"; rc=1; }
[ $rc = 0 ] && echo "test baseline: OK (rust $rust_passed, python $py_passed, failures as pinned)"
rm -rf "$tmp"; exit $rc
