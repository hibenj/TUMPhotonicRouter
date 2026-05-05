from routing_flow import run_routing_flow


def test_rust_routed_layout_uses_waveguide_geometry():
    layout = run_routing_flow(
        "TOY",
        show_unrouted=False,
        show_routed=False,
        show_debug_svgs=False,
    )

    # The old square-cell renderer produced hundreds of references.
    # The new waveguide translation should keep the routed layout compact.
    assert len(layout.insts) <= 10
    assert len(layout.get_polygons(merge=False, by="tuple")) > 0

