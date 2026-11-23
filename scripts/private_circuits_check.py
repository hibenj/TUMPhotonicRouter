#!/usr/bin/env python3
"""Route the owner's private circuits, if this checkout has them.

    private_circuits_check.py [--jobs N] [--timeout SECONDS]

`benchmarks/private/` is not part of the repository (`.gitignore`), so no
gate of the continuous integration can route what is in it; on 2026-11-23
its circuits had been unroutable for eighteen days without a check noticing.
`scripts/check.sh --full` runs this script: every module of
`benchmarks/private/` with a `build_schematic()` is routed with its own
defaults (`routing_flow.py private.<name>`), each in a working directory of
its own, and has to finish with exit code 0 and `verifier_errors=0` in its
`SUMMARY` line. Without the directory the script says so and passes.

Nothing about the circuits is pinned in the repository: the names appear on
the terminal only.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PRIVATE = REPO_ROOT / "benchmarks" / "private"

#: entries of the repository root a run must not share with another run
_OWN_PER_RUN = {"build", "results"}


def private_circuits(directory: Path = PRIVATE) -> list[str]:
    """The modules of `directory` that build a schematic, sorted."""
    if not directory.is_dir():
        return []
    return sorted(
        path.stem
        for path in directory.glob("*.py")
        if "def build_schematic(" in path.read_text(encoding="utf-8")
    )


def verdict(exit_code: int, output: str) -> tuple[bool, str]:
    """Whether a run passed, and the line that says why.

    A run passes when the flow ended with exit code 0 and its `SUMMARY`
    line reports no verifier error.
    """
    summary = next(
        (line for line in reversed(output.splitlines()) if line.startswith("SUMMARY ")), ""
    )
    if exit_code != 0:
        last = next((line for line in reversed(output.splitlines()) if line.strip()), "")
        return False, f"exit code {exit_code}: {last[:200]}"
    errors = re.search(r"\bverifier_errors=(\S+)", summary)
    if errors is None or errors.group(1) != "0":
        return False, f"no clean verifier result: {summary[:200] or 'no SUMMARY line'}"
    fields = dict(re.findall(r"(\w+)=(\S+)", summary))
    shown = ("nets", "plm_groups", "plm_over_tolerance", "total_s")
    return True, ", ".join(f"{key}={fields[key]}" for key in shown if key in fields)


def route(name: str, timeout_s: float) -> tuple[bool, str]:
    """Route `private.<name>` in a working directory of its own."""
    with tempfile.TemporaryDirectory(prefix=f"private_{name}_") as work:
        work_dir = Path(work)
        (work_dir / "build").mkdir()
        for entry in [*REPO_ROOT.iterdir()]:
            if entry.name not in _OWN_PER_RUN and entry.name != ".git":
                (work_dir / entry.name).symlink_to(entry)
        try:
            run = subprocess.run(
                [str(REPO_ROOT / ".venv" / "bin" / "python"), "routing_flow.py", f"private.{name}"],
                cwd=work_dir,
                capture_output=True,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return False, f"no result after {timeout_s:.0f} s"
    return verdict(run.returncode, run.stdout + run.stderr)


def main(argv: list[str] | None = None) -> int:
    """Route every private circuit and report; 1 if one of them failed."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--jobs", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=1800.0, metavar="SECONDS")
    args = parser.parse_args(argv)
    names = private_circuits()
    if not names:
        print("private circuits: none in this checkout (benchmarks/private/), skipped")
        return 0
    with ThreadPoolExecutor(max(1, args.jobs)) as pool:
        results = list(pool.map(lambda name: route(name, args.timeout), names))
    failed = 0
    for name, (passed, why) in zip(names, results, strict=True):
        print(f"private.{name}: {'ok' if passed else 'FAILED'} ({why})")
        failed += not passed
    print(
        f"private circuits: {len(names) - failed} of {len(names)} routed"
        if failed
        else f"all {len(names)} private circuits routed"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
