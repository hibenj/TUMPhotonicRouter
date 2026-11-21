"""What a logic check reports."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass

ERROR = "error"
NOTE = "note"


@dataclass(frozen=True)
class Finding:
    """One place where a stage's product fails a statement of the harness."""

    check: str
    """The statement that failed, as `stage.name`, e.g. `reading.link_end_without_port`."""
    severity: str
    """`ERROR`: the product contradicts itself. `NOTE`: legal, but a later stage must handle it."""
    subject: str
    """What it was found on: an instance, a net, a port."""
    detail: str = ""

    def __str__(self) -> str:
        text = f"{self.severity} {self.check}: {self.subject}"
        return f"{text} ({self.detail})" if self.detail else text


def count_by_check(findings: Iterable[Finding]) -> dict[str, int]:
    """How many findings each check has, sorted by check name."""
    return dict(sorted(Counter(finding.check for finding in findings).items()))
