"""The crossing-mode vocabulary (`translation/crossing_modes.py`)."""

from __future__ import annotations

import pytest

from translation.crossing_modes import (
    CROSSING_MODES,
    is_guided_mode,
    is_lidar_mode,
    normalize_crossing_mode,
)


def test_only_the_two_paper_modes_exist():
    assert CROSSING_MODES == ("lidar-pure", "lidar-guided")
    assert all(is_lidar_mode(mode) for mode in CROSSING_MODES)
    assert [mode for mode in CROSSING_MODES if is_guided_mode(mode)] == ["lidar-guided"]


def test_aliases_resolve_to_the_two_modes():
    assert normalize_crossing_mode(" Lidar ") == "lidar-pure"
    assert normalize_crossing_mode("guided") == "lidar-guided"


@pytest.mark.parametrize("removed_mode", ["window", "collision"])
def test_the_removed_modes_are_rejected(removed_mode):
    """`window` and `collision` were removed on 2026-09-24 (Milestone 8 of
    .agent/execplans/2026-09-22-modular-readable-router-restructure.md)."""

    with pytest.raises(ValueError) as excinfo:
        normalize_crossing_mode(removed_mode)
    message = str(excinfo.value)
    assert "'lidar-pure'" in message
    assert "'lidar-guided'" in message
    assert removed_mode in message
