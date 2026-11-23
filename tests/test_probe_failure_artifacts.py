"""The debug files of a failed photonic verification probe."""

from __future__ import annotations

import json
from pathlib import Path

from photonic_router.gds.route_rust_debug_artifacts import dump_photonic_probe_failure_artifacts
from photonic_router.records.route_rust_types import PortRef, RoutedNetRecord
from photonic_router.verify.photonic_verification import (
    PhotonicVerificationIssue,
    PhotonicVerificationResult,
)


class _Router:
    """Gives a centerline for the route object "ok" and fails for every other one."""

    def route_primitive_centerline(self, route):
        if route != "ok":
            raise ValueError("no primitives")
        return [(0.0, 0.0), (10.0, 0.0)]

    def route_port_corrected_centerline(
        self, route, *, source_port_um=None, target_port_um=None, allow_unchecked_bumps=True
    ):
        if route != "ok":
            raise ValueError("no correction")
        return [source_port_um, (10.0, 0.0), target_port_um]


def _record(name: str, route: str, corrected=()) -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name=name,
        source=PortRef("a", "o1"),
        target=PortRef("b", "o1"),
        route_obj=route,
        total_length_um=20.0,
        net_id=7,
        source_port_center_um=(-1.0, 0.0),
        target_port_center_um=(21.0, 0.0),
        corrected_centerline_um=corrected,
    )


def test_the_files_hold_the_centerlines_of_every_net_an_issue_names(tmp_path: Path):
    verification = PhotonicVerificationResult(
        issues=(
            PhotonicVerificationIssue(
                code="waveguide_overlap",
                message="n1 overlaps n2",
                net_name="n1",
                details={"other_net_name": "n2", "overlap_bbox_um": (4.0, -1.0, 6.0, 1.0)},
            ),
            PhotonicVerificationIssue(code="missing_route_record", message="?", net_name="gone"),
        ),
        metrics={},
    )
    records = [
        _record("n1", "ok"),
        _record("n2", "broken", corrected=((0.0, 5.0), (10.0, 5.0))),
        _record("n3", "ok"),
    ]

    artifacts = dump_photonic_probe_failure_artifacts(
        debug_path=tmp_path,
        debug_prefix="bench",
        probe_layout=None,
        verification=verification,
        records=records,
        router=_Router(),
        realization_grid_spec=(10, 10, 1.0, 0.0, 0.0),
    )

    assert sorted(artifacts) == [
        "centerlines_json",
        "centerlines_svg",
        "issues_json",
        "summary_txt",
    ]
    assert all(
        Path(path).parent == tmp_path / "photonic_probe_failures" for path in artifacts.values()
    )
    centerlines = json.loads(Path(artifacts["centerlines_json"]).read_text())
    # the nets the issues name and that have a record; n3 is in no issue
    assert sorted(centerlines) == ["n1", "n2"]
    assert centerlines["n1"]["primitive_centerline_um"] == [[0.0, 0.0], [10.0, 0.0]]
    # no corrected centerline on the record: the router's, between the port centres
    assert centerlines["n1"]["corrected_centerline_um"] == [[-1.0, 0.0], [10.0, 0.0], [21.0, 0.0]]
    # a centerline the router cannot give is an error text, not an exception
    assert centerlines["n2"]["primitive_centerline_um"] == []
    assert centerlines["n2"]["primitive_centerline_error"] == "no primitives"
    assert centerlines["n2"]["corrected_centerline_um"] == [[0.0, 5.0], [10.0, 5.0]]
    assert "corrected_centerline_error" not in centerlines["n2"]
    assert json.loads(Path(artifacts["issues_json"]).read_text())["issues"][0]["net_name"] == "n1"
    summary = Path(artifacts["summary_txt"]).read_text().splitlines()
    assert summary[:3] == [
        "Photonic probe failure artifacts",
        "debug_prefix=bench",
        "realization_grid_spec=(10, 10, 1.0, 0.0, 0.0)",
    ]
    assert sum(line.startswith("issue=") for line in summary) == 2
