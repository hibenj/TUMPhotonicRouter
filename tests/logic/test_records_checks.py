"""`photonic_router.logic_checks.records`: each statement finds what it names.

The records are real ones: Benes 8 x 8 routed and its ends corrected (48
nets), read after phase 7. Each test breaks one thing in one record.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace

import pytest
from photonic_router import cli
from photonic_router.logic_checks import count_by_check
from photonic_router.logic_checks.records import check_records
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

pytestmark = pytest.mark.e2e


@pytest.fixture(scope="module")
def session():
    seen = {}

    def observer(stage: str, settings: object, product: object) -> None:
        if stage == "records":
            seen["session"] = (settings, product)
            raise StopAfterStage

    with (
        pytest.raises(StopAfterStage),
        observe_stages(observer),
        contextlib.redirect_stdout(io.StringIO()),
    ):
        cli.main(["benes_8x8"])
    settings, routed = seen["session"]
    return settings.schematic, list(routed.routed_net_records)


def _counts(schematic, records, **changes: object) -> dict[str, int]:
    """The findings with the first record changed by `changes`."""
    if changes:
        records = [replace(records[0], **changes), *records[1:]]
    return count_by_check(check_records(schematic, records))


def test_the_real_records_are_sound(session) -> None:
    schematic, records = session
    assert len(records) == 48
    assert all(record.corrected_centerline_um for record in records)
    assert _counts(schematic, records) == {}


def test_the_records_are_the_links(session) -> None:
    schematic, records = session
    assert _counts(schematic, records[1:]) == {"records.link_without_record": 1}
    assert _counts(schematic, [*records, records[0]]) == {"records.record_without_link": 1}
    assert _counts(schematic, records, net_name="stray") == {
        "records.link_without_record": 1,
        "records.record_without_link": 1,
    }


def test_a_failed_or_missing_correction_is_named(session) -> None:
    schematic, records = session
    assert _counts(schematic, records, endpoint_correction_error="no room for the bump") == {
        "records.correction_failed": 1
    }
    assert _counts(schematic, records, corrected_centerline_um=()) == {
        "records.end_not_corrected": 1
    }


def test_a_centerline_runs_from_port_centre_to_port_centre(session) -> None:
    schematic, records = session
    line = records[0].corrected_centerline_um
    # the grid end instead of the port: the first point is dropped
    counts = _counts(schematic, records, corrected_centerline_um=line[1:])
    assert counts["records.centerline_misses_port"] == 1
    x, y = records[0].target_port_center_um
    assert _counts(schematic, records, target_port_center_um=(x, y + 0.01)) == {
        "records.centerline_misses_port": 1
    }
    assert _counts(schematic, records, target_port_center_um=(x, y + 0.0005)) == {}


def test_a_centerline_has_no_repeated_point_and_leaves_its_ports_straight(session) -> None:
    schematic, records = session
    line = records[0].corrected_centerline_um
    assert _counts(schematic, records, corrected_centerline_um=(line[0], *line)) == {
        "records.centerline_point_repeated": 1
    }
    turned = (records[0].source_port_orientation_deg + 90.0) % 360.0
    assert _counts(schematic, records, source_port_orientation_deg=turned) == {
        "records.centerline_leaves_port_askew": 1
    }
    turned = (records[0].target_port_orientation_deg + 10.0) % 360.0
    assert _counts(schematic, records, target_port_orientation_deg=turned) == {
        "records.centerline_leaves_port_askew": 1
    }


def test_the_lengths_are_the_centerlines_and_the_routes(session) -> None:
    schematic, records = session
    record = records[0]
    assert _counts(schematic, records, total_length_um=record.total_length_um + 0.001) == {
        "records.length_not_the_centerline": 1
    }
    assert _counts(schematic, records, base_total_length_um=None) == {
        "records.base_length_not_the_route": 1
    }
    assert _counts(
        schematic, records, base_total_length_um=record.base_total_length_um + 0.001
    ) == {"records.base_length_not_the_route": 1}
