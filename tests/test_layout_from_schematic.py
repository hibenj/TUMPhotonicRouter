from __future__ import annotations

import pytest
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Placement, Schematic

from translation.layout_from_schematic import layout_from_schematic

get_generic_pdk().activate()


def _single_rectangle_schematic(placement: Placement) -> Schematic:
    schematic = Schematic()
    schematic.add_instance(
        "rect_0",
        Instance(component="rectangle", settings={"size": (10, 2)}),
        placement,
    )
    return schematic


def _bbox_tuple(ref) -> tuple[float, float, float, float]:
    bbox = ref.dbbox()
    return (
        float(bbox.left),
        float(bbox.bottom),
        float(bbox.right),
        float(bbox.top),
    )


def test_no_anchor_rotation_is_applied_before_translation() -> None:
    layout = layout_from_schematic(_single_rectangle_schematic(Placement(x=100, y=0, rotation=90)))

    assert _bbox_tuple(layout.insts["rect_0"]) == pytest.approx((98, 0, 100, 10))


def test_no_anchor_mirror_is_applied_before_translation() -> None:
    layout = layout_from_schematic(_single_rectangle_schematic(Placement(x=100, y=0, mirror=True)))

    assert _bbox_tuple(layout.insts["rect_0"]) == pytest.approx((90, 0, 100, 2))
