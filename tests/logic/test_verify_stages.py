"""`photonic_router.logic_checks.stages`: the stage checks inside a run of the flow."""

from __future__ import annotations

from typing import Any, cast

import pytest
from photonic_router import cli
from photonic_router.logic_checks import ERROR, NOTE, Finding, stages
from photonic_router.logic_checks.stages import (
    STAGES,
    StageVerificationError,
    stage_findings,
    verify_stages,
)
from photonic_router.routing.stage_observer import notify

_SETTINGS = cast(Any, None)


def _findings_per_stage(monkeypatch, per_stage: dict[str, list[Finding]]) -> None:
    monkeypatch.setattr(
        stages, "stage_findings", lambda stage, _settings, _product: per_stage.get(stage, [])
    )


def test_a_stage_without_checks_has_no_findings() -> None:
    assert stage_findings("router_setup", object(), object()) == []
    assert stage_findings("no_such_stage", None, object()) == []


def test_notes_are_collected_and_reported_and_the_run_goes_on(monkeypatch) -> None:
    note = Finding("reading.port_recessed", NOTE, "heater_0,o1")
    _findings_per_stage(monkeypatch, {"loaded": [note]})
    lines: list[str] = []
    with verify_stages(lines.append) as findings:
        notify("loaded", _SETTINGS, None)
        notify("obstacle_map", _SETTINGS, None)
    assert findings == [note]
    assert lines == [
        "      - verify loaded: 0 error(s), 1 note(s)",
        "      - verify obstacle_map: 0 error(s), 0 note(s)",
    ]


def test_an_error_ends_the_run_and_names_the_stage(monkeypatch) -> None:
    errors = [Finding("routing.route_through_static", ERROR, f"net_{i}") for i in range(7)]
    _findings_per_stage(monkeypatch, {"routed": errors})
    lines: list[str] = []
    with pytest.raises(StageVerificationError) as raised, verify_stages(lines.append):
        notify("handoff", _SETTINGS, None)
        notify("routed", _SETTINGS, None)
        notify("records", _SETTINGS, None)  # not reached
    assert raised.value.stage == "routed"
    assert raised.value.errors == errors
    assert "stage 'routed': 7 error(s)" in str(raised.value)
    assert "(and 2 more)" in str(raised.value)
    assert [line.split(":")[0].strip() for line in lines] == ["- verify handoff", "- verify routed"]
    notify("routed", _SETTINGS, None)  # the verifier is gone with its block


@pytest.mark.e2e
def test_the_flag_checks_every_stage_of_a_run_and_a_run_without_it_checks_none(capsys) -> None:
    cli.main(["benes_4x4", "--verify-stages"])
    verified = capsys.readouterr().out
    for stage in STAGES:
        assert f"      - verify {stage}: 0 error(s)" in verified
    cli.main(["benes_4x4"])
    assert "verify " not in capsys.readouterr().out


@pytest.mark.e2e
def test_an_error_of_a_stage_ends_a_real_run(monkeypatch, capsys) -> None:
    def broken(stage: str, _settings: object, _product: object) -> list[Finding]:
        return [Finding("route_jobs.state_blocked", ERROR, "made up")] if stage == "handoff" else []

    monkeypatch.setattr(stages, "stage_findings", broken)
    with pytest.raises(StageVerificationError, match="stage 'handoff': 1 error"):
        cli.main(["benes_4x4", "--verify-stages"])
    out = capsys.readouterr().out
    assert "verify handoff: 1 error(s)" in out
    assert "verify routed" not in out
    assert "SUMMARY benchmark=benes_4x4" in out  # the flow's footer still closes the run
