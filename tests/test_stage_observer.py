"""`photonic_router.routing.stage_observer`: the seam the logic harness reads through."""

from __future__ import annotations

from typing import Any, cast

import pytest
from photonic_router.routing.stage_observer import StopAfterStage, notify, observe_stages


def test_notify_reaches_the_observer_only_inside_the_block() -> None:
    seen: list[tuple[str, object]] = []
    settings = cast(Any, object())
    notify("before", settings, 1)
    with observe_stages(lambda stage, _settings, product: seen.append((stage, product))):
        notify("inside", settings, 2)
    notify("after", settings, 3)
    assert seen == [("inside", 2)]


def test_an_observer_ends_the_run_and_the_block_closes() -> None:
    def stop(_stage: str, _settings: object, _product: object) -> None:
        raise StopAfterStage

    with pytest.raises(StopAfterStage), observe_stages(stop):
        notify("obstacle_map", cast(Any, object()), None)
    notify("later", cast(Any, object()), None)  # no observer is left behind
