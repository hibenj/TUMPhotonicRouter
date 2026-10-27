"""The checked-in stub of the Rust extension equals the one its generator writes."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_checked_in_rust_stub_equals_the_generated_one() -> None:
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "generate_rust_stubs.py"), "--check"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
