import klayout.db as kdb
from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk

from translation.route_rust_realization import (
    _add_route_polygon,
    _crossing_clip_regions_by_net_id,
)

get_generic_pdk().activate()


def _layer_polygons(component: Component, layer: tuple[int, int]):
    return component.get_polygons(merge=False, by="tuple").get(layer, [])


def test_add_route_polygon_clips_crossing_footprint_from_route_layer():
    layout = Component("clip_route_polygon")
    route_layer = (1, 0)
    route_polygon = kdb.Polygon(kdb.Box(0, 0, 10_000, 1_000))
    clip_region = kdb.Region(kdb.Box(4_000, -1_000, 6_000, 2_000))

    _add_route_polygon(
        layout,
        route_polygon,
        route_layer=route_layer,
        clip_region=clip_region,
    )

    bboxes = [polygon.bbox() for polygon in _layer_polygons(layout, route_layer)]
    assert [(box.left, box.bottom, box.right, box.top) for box in bboxes] == [
        (0, 0, 4000, 1000),
        (6000, 0, 10000, 1000),
    ]


def test_crossing_clip_regions_are_keyed_to_legal_owner_nets():
    regions = _crossing_clip_regions_by_net_id(
        {
            "enabled": True,
            "realized_intersections": [
                {
                    "classification": "legal_unexpected_crossing",
                    "net_id_a": 1,
                    "net_id_b": 2,
                    "crossing_footprint_polygon_um": [
                        (1.0, 1.0),
                        (3.0, 1.0),
                        (3.0, 3.0),
                        (1.0, 3.0),
                    ],
                },
                {
                    "classification": "illegal_unexpected_crossing",
                    "net_id_a": 3,
                    "net_id_b": 4,
                    "crossing_footprint_polygon_um": [
                        (5.0, 5.0),
                        (7.0, 5.0),
                        (7.0, 7.0),
                        (5.0, 7.0),
                    ],
                },
            ],
        },
        dbu=0.001,
    )

    assert set(regions) == {1, 2}
    assert regions[1].area() == 4_000_000
    assert regions[2].area() == 4_000_000
