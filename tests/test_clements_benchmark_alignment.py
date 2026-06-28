from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from benchmarks.clements_8x8 import SOURCE_YAML, build_schematic
from translation.layout_from_schematic import layout_from_schematic


def test_clements_8x8_layout_matches_lidar_macro_placements() -> None:
    yaml_path = Path(SOURCE_YAML)
    if not yaml_path.exists():
        pytest.skip(f"LiDAR benchmark YAML not available: {yaml_path}")

    data = yaml.load(yaml_path.read_text(encoding="utf-8"), Loader=yaml.UnsafeLoader)
    layout = layout_from_schematic(build_schematic())

    for instance_name, instance_data in data["instances"].items():
        ref = layout.insts[instance_name]
        macro_type = instance_data["settings"]["macro_type"]
        macro = data["library"][macro_type]
        x, y = instance_data["settings"]["placement"][1]
        width, height = macro["size"]

        bbox = ref.dbbox()
        assert float(bbox.left) == pytest.approx(float(x))
        assert float(bbox.bottom) == pytest.approx(float(y))
        assert float(bbox.right) == pytest.approx(float(x) + float(width))
        assert float(bbox.top) == pytest.approx(float(y) + float(height))

        orientation = instance_data["settings"]["placement"][2]
        for port_name, pin in macro["pins"].items():
            expected_x = float(pin["pin_offset_x"])
            expected_y = float(pin["pin_offset_y"])
            expected_orientation = float(pin["pin_orient"])
            if orientation == "FN":
                expected_x = float(width) - expected_x
                expected_orientation = [180.0, 90.0, 0.0, 270.0][
                    int(expected_orientation // 90.0)
                ]
            elif orientation != "N":
                continue

            port = ref.ports[port_name]
            assert float(port.center[0]) == pytest.approx(float(x) + expected_x)
            assert float(port.center[1]) == pytest.approx(float(y) + expected_y)
            assert float(port.orientation) == pytest.approx(expected_orientation)
