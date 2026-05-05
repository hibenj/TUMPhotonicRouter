import gdsfactory as gf
from gdsfactory.cross_section import CrossSectionSpec
from gdsfactory.cross_section import CrossSection, Section
from gdsfactory.samples.pdk.fab_c import LAYER
from sax.models import grating_coupler


def _get_bbox_dims(component: gf.Component):
    """Returns (width, height) of a component in microns based on its bbox."""
    bb = component.bbox() if callable(getattr(component, "bbox", None)) else component.bbox
    try:
        (xmin, ymin), (xmax, ymax) = bb
    except Exception:
        xmin, ymin, xmax, ymax = component.xmin, component.ymin, component.xmax, component.ymax
    return float(xmax - xmin), float(ymax - ymin)


def _get_bbox(c: gf.Component):
    """Returns ((xmin, ymin), (xmax, ymax)) regardless of bbox being a method or property."""
    bb = c.bbox() if callable(getattr(c, "bbox", None)) else c.bbox
    # Some versions may return a numpy array or a DBox-like object
    # Normalize to tuple of tuples
    try:
        (xmin, ymin), (xmax, ymax) = bb
    except Exception:
        # Fallback to attributes if available
        xmin, ymin, xmax, ymax = c.xmin, c.ymin, c.xmax, c.ymax
        return (xmin, ymin), (xmax, ymax)
    return (float(xmin), float(ymin)), (float(xmax), float(ymax))


@gf.cell
def switch(
        ring_component: gf.Component,
        gap: float = 0.225,
        x: float = 0.0,
        y: float = 0.0,
) -> gf.Component:
    """Builds one switch (two asymmetric rings) as a reusable component.

    Args:
        ring_component: the single-ring component to duplicate and couple.
        gap: desired vertical separation (µm) between the rings' coupling buses.
        x, y: position offset for the upper ring.
    """
    c = gf.Component("switch")

    # first (upper) ring
    r1 = c << ring_component
    r1.movex(x)
    r1.movey(y)

    # second (lower) ring
    r2 = c << ring_component
    r2.rotate(180)
    r2.movex(x)

    # compute bounding boxes (in µm)
    (_, ymin1), (_, ymax1) = _get_bbox(r1)
    (_, ymin2), (_, ymax2) = _get_bbox(r2)

    # align top of r2 + gap = bottom of r1
    dy = ymin1 - (ymax2 + gap)
    r2.movey(dy)

    num_ports = 0
    for port in r1.ports:
        c.add_port(port=port)
        num_ports += 1

    for i, p in enumerate(r2.ports, start=num_ports + 1):
        new_name = f"o{i}"
        c.add_port(
            name=new_name,
            center=p.center,
            width=p.width,
            orientation=p.orientation,
            layer=p.layer,
            port_type=p.port_type,
        )

    return c


def benes_parameters(n: int):
    """
    Computes the structural parameters of an n-bit Benes network.

    Args:
        n (int): Number of address bits (network size = 2^n x 2^n).

    Returns:
        dict: containing:
            - 'n': input parameter
            - 'N': number of inputs/outputs (2^n)
            - 'stages': number of stages (2n - 1)
            - 'elements': number of 2x2 switches per stage (N/2)
            - 'connections': list of stage connections as (a, b) pairs (1-based)
    """
    # number of inputs/outputs
    N = 1 << n
    # number of stages
    stages = 2 * n - 1
    # number of 2×2 elements per stage
    elements = N // 2

    # connections[stage] = list of (a, b) pairs (1-based)
    connections = [[] for _ in range(stages - 1)]

    # --- Forward connections ---
    for i in range((stages - 1) // 2):
        step = elements >> i  # elements / 2^i
        half = step >> 1  # step / 2

        for g in range(elements // step):  # number of blocks
            for j in range(half):  # pairs inside a block
                a = g * step + j + 1  # 1-based
                b = a + half
                p = (a, b)
                connections[i].append(p)
                connections[i].append(p)  # duplicates, like in C++

    # --- Helper for inverse stage ---
    def build_inverse_stage(forward, backward):
        backward.clear()
        m = len(forward)
        pos = [[] for _ in range(m + 1)]  # pos[node] -> entry indices (1-based)

        # Record where each node appears
        for idx, (a, b) in enumerate(forward, start=1):
            pos[a].append(idx)
            pos[b].append(idx)

        # For duplicates: each node appears twice -> connect those entry indices
        for u in range(1, m + 1):
            if len(pos[u]) >= 2:
                pos[u].sort()
                backward.append((pos[u][0], pos[u][1]))

    # --- Backward connections (mirror of forward) ---
    for i in range((stages - 1) // 2):
        build_inverse_stage(connections[i], connections[stages - 2 - i])

    return {
        "n": n,
        "N": N,
        "stages": stages,
        "elements": elements,
        "connections": connections,
    }


def add_two_edge_ports(c: gf.Component, width: float = 1.0, layer=(1, 0)):
    (xmin, ymin), (xmax, ymax) = _get_bbox(c)
    xmid = 0.5 * (xmin + xmax) - 0.5
    ymid = 0.5 * (ymin + ymax)

    c.add_port(name="W", center=(xmin - 0.2, ymid), width=width, orientation=180,
               layer=layer, port_type="optical")
    c.add_port(name="E", center=(xmax - 0.2, ymid), width=width, orientation=0,
               layer=layer, port_type="optical")


import gdsfactory as gf


@gf.cell
def dual_ring_switch(ring: gf.Component) -> gf.Component:
    """Two vertically stacked microrings acting as an optical switch."""
    c = gf.Component("dual_ring_switch")

    # first ring
    r1 = c << ring
    r1.rotate(-90)
    r1.move((0, 0))

    # second ring (vertically offset)
    r2 = c << ring
    r2.rotate(-90)
    r2.move((0, -43))  # adjust vertical distance between rings

    # optionally define where the waveguides connect
    # here we just expose the ports of each ring
    c.add_port("in_top", port=r1.ports["o4"])
    c.add_port("out_top", port=r1.ports["o1"])
    c.add_port("in_bottom", port=r2.ports["o3"])
    c.add_port("out_bottom", port=r2.ports["o2"])

    return c

def connect_inputs(c, input_refs, switch_refs, cross_section="strip"):
    """
    Connects input grating couplers to the first-stage switches.
    Two inputs per switch: top → o1, bottom → o4.
    """
    for i in range(0, len(input_refs), 2):
        switch_index = i // 2  # because 2 inputs per switch

        # top input → left-top port (o1)
        gf.routing.route_single(
            c,
            input_refs[i].ports["o1"],
            switch_refs[0][switch_index].ports["o1"],
            port_type="optical",
            cross_section=cross_section,
        )

        # bottom input → left-bottom port (o4)
        gf.routing.route_single(
            c,
            input_refs[i + 1].ports["o1"],
            switch_refs[0][switch_index].ports["o4"],
            port_type="optical",
            cross_section=cross_section,
        )

def connect_outputs(c, output_refs, switch_refs, elements, cross_section="strip"):
    """
    Connects the last-stage switches to the output grating couplers.
    Two outputs per switch: top → o2, bottom → o3.
    """
    for i in range(elements):
        # top-right port (o2) → top output coupler
        gf.routing.route_single(
            c,
            switch_refs[-1][i].ports["o2"],
            output_refs[2 * i].ports["o1"],
            port_type="optical",
            cross_section=cross_section,
        )

        # bottom-right port (o3) → bottom output coupler
        gf.routing.route_single(
            c,
            switch_refs[-1][i].ports["o3"],
            output_refs[2 * i + 1].ports["o1"],
            port_type="optical",
            cross_section=cross_section,
        )

def _init_port_usage(switch_refs):
    """Make a usage map: {(stage, elem): {'left': {'o1':False,'o4':False},
                                          'right':{'o2':False,'o3':False}}}"""
    usage = {}
    for s, stage in enumerate(switch_refs):
        for e, _ in enumerate(stage):
            usage[(s, e)] = {
                "left":  {"o1": False, "o4": False},
                "right": {"o2": False, "o3": False},
            }
    return usage

def _take_port(cref, usage, key, side, prefer):
    """
    Pick the first free port on 'side' from 'prefer' order, mark as used, return the Port.
    key = (stage_idx, elem_idx)
    side in {'left','right'}
    prefer = e.g. ['o1','o4'] or ['o2','o3']
    """
    for pname in prefer:
        if not usage[key][side][pname]:
            usage[key][side][pname] = True
            return cref.ports[pname]
    raise RuntimeError(
        f"No free {side} ports remaining on switch {key}; tried {prefer}."
    )

def connect_stages(c, switch_refs, connections, cross_section="strip"):
    """
    Route all internal Benes connections stage-by-stage using port-usage flags.

    connections[x][i] = (a,b) where:
      - source = switch_refs[x][i]
      - dest A = switch_refs[x+1][a-1]
      - dest B = switch_refs[x+1][b-1]
    Rules:
      - source (right side): prefer o2 then o3
      - dest   (left  side): prefer o1 then o4
      - error if a side runs out of ports
    """
    usage = _init_port_usage(switch_refs)
    routes = []

    for x, stage_conn in enumerate(connections):
        src_stage = switch_refs[x]
        dst_stage = switch_refs[x + 1]

        for src_i, (a, b) in enumerate(stage_conn):
            src_key = (x, src_i)
            dstA_key = (x + 1, a - 1)
            dstB_key = (x + 1, b - 1)

            src_ref = src_stage[src_i]
            dstA_ref = dst_stage[a - 1]
            dstB_ref = dst_stage[b - 1]

            # First link (prefer o2 on src-right, o1 on dst-left)
            p_src_1 = _take_port(src_ref, usage, src_key, "right", ["o2", "o3"])
            p_dst_1 = _take_port(dstA_ref, usage, dstA_key, "left",  ["o1", "o4"])
            routes.append(
                gf.routing.route_single(
                    c, p_src_1, p_dst_1, port_type="optical", cross_section=cross_section
                )
            )

            # Second link (next available on each side)
            p_src_2 = _take_port(src_ref, usage, src_key, "right", ["o2", "o3"])
            p_dst_2 = _take_port(dstB_ref, usage, dstB_key, "left",  ["o1", "o4"])
            routes.append(
                gf.routing.route_single(
                    c, p_src_2, p_dst_2, port_type="optical", cross_section=cross_section
                )
            )
            #break
        #break
    return routes

@gf.cell
def benes_array(
        n: int,
        switch_component: gf.Component,
        grating_coupler:  gf.Component,
        x_min_gap: float = 0.0,
        y_min_gap: float = 1.0,
        waveguide_width: float = 0.5,
) -> gf.Component:
    """
    Builds a rectangular array of switch components for an n-bit Benes network.
    Only placement, no routing.

    Args:
        n: Number of address bits (network size = 2^n).
        switch_component: gdsfactory component for a 2x2 switch.
        y_min_gap: Minimum vertical spacing (µm) between switches.

    Returns:
        gf.Component with all switches placed.
    """
    c = gf.Component(f"benes_array_n{n}")

    params = benes_parameters(n)

    # print(f"\nComputed Benes parameters:")
    # print(f"N = {params['N']}")
    # print(f"stages = {params['stages']}")
    # print(f"elements per stage = {params['elements']}\n")

    stages = params['stages']
    elements = params['elements']

    # Measure switch geometry
    sw_w, sw_h = _get_bbox_dims(switch_component)

    # Vertical pitch (at least 1 µm apart)
    y_pitch = sw_h + y_min_gap
    # Horizontal pitch = width (touching side by side)
    x_pitch = sw_w + x_min_gap

    input_refs = []
    output_refs = []
    switch_refs = []
    for stage in range(stages):
        stage_refs = []
        for elem in range(elements):
            sw_ref = c << switch_component
            x = stage * x_pitch
            y = -elem * y_pitch
            sw_ref.move((x, y))
            stage_refs.append(sw_ref)

            (xmin, ymin), (xmax, ymax) = _get_bbox(sw_ref)

            # Save boundary info
            if stage == 0:
                gc_ref = c << grating_coupler
                gc_ref.rotate(90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x-x_gc, ymax-(waveguide_width/2)))
                input_refs.append(gc_ref)
                gc_ref = c << grating_coupler
                gc_ref.rotate(90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x - x_gc, ymin + (waveguide_width / 2)))
                input_refs.append(gc_ref)
            if stage == stages - 1:
                gc_ref = c << grating_coupler
                gc_ref.rotate(-90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x + x_gc, ymax - (waveguide_width / 2)))
                output_refs.append(gc_ref)
                gc_ref = c << grating_coupler
                gc_ref.rotate(-90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x + x_gc, ymin + (waveguide_width / 2)))
                output_refs.append(gc_ref)

        switch_refs.append(stage_refs)

    connect_inputs(c, input_refs, switch_refs, cross_section="strip")
    connect_outputs(c, output_refs, switch_refs, elements, cross_section="strip")

    connect_stages(c, switch_refs, params['connections'])

    # gf.routing.route_bundle(
    #     c,
    #     ports1=input_refs,
    #     ports2=switch_refs[0],
    #     separation=4.0,
    #     cross_section="strip",
    #     sort_ports=False  # Don't sort to preserve our connection pattern
    # )


    return c


if __name__ == "__main__":
    # Start the test
    print("Start testing...")

    # GDS-Datei importieren
    gc = gf.read.import_gds(
        "/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/photonics/Projects/Carleton_collaboration/PDK/NanoSOI_PDK_v74/tech/libraries/ANT_PDK_Silicon_v74.gds",
        cellname="GratingCoupler_TM_Oxide_8degrees")

    # ring = gf.read.import_gds(
    # "/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/photonics/Projects/Carleton_collaboration/PDK/NanoSOI_PDK_v74/tech/libraries/ANT_PDK_Silicon_v74.gds",
    # cellname="Microring_TM_Criticallycoupled_R=20um")

    ring = gf.read.import_gds(
        "/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/photonics/Projects/Carleton_collaboration/Custom_Elements/Ring.gds",
        cellname="TOP")

    gc = gf.add_ports.add_ports_from_markers_inside(gc, pin_layer="PORT", port_layer="WG")
    ring = gf.add_ports.add_ports_from_markers_inside(ring, pin_layer="PORT", port_layer="WG")

    for port in gc.ports:
        x, y = port.center
        port.center = (x, y + 0.05)

    for port in ring.ports:
        x, y = port.center
        if x < 0:
            port.center = (x + 0.05, y)
        else:
            port.center = (x - 0.05, y)

    #gc.pprint_ports()
    #ring.pprint_ports()

    top = gf.Component("demo")
    #gc1 = top << gc
    #gc1.rotate(90)
    #gc1.move((-80, 21.2))  # place it at x=50

    sw = switch(ring, gap=0.225)
    #sw.pprint_ports()

    array = benes_array(n=2, switch_component=sw, grating_coupler=gc, x_min_gap = 40.0, y_min_gap=40.0, waveguide_width=0.5)
    ar1 = top << array
    # ar1.move((80, -21.2))

    # --- route: ring1.E → ring2.W ---
    # route = gf.routing.route_single(
    #     top,
    #     gc1.ports["o1"],
    #     ring1.ports["o4"],
    #     port_type="optical",
    #     cross_section="strip",
    # )
    #
    # top = gf.Component("demo")
    #
    # # --- Components ---
    # gc1 = top << gc
    # gc2 = top << gc
    # switch1 = top << dual_ring_switch(ring)
    # switch2 = top << dual_ring_switch(ring)
    #
    # # --- Placement ---
    # gc1.rotate(90)
    # gc1.move((0, 0))
    #
    # switch1.move((80, -21.2))  # align with GC output
    # switch2.move((140, -21.2))

    # --- Routing ---
    # route1 = gf.routing.route_single(
    #     top,
    #     gc1.ports["o1"],
    #     switch1.ports["in_top"],  # connect GC to switch input
    #     port_type="optical",
    #     cross_section="strip",
    # )
    #
    # route2 = gf.routing.route_single(
    #     top,
    #     gc2.ports["o1"],
    #     switch2.ports["in_bottom"],  # connect GC to switch input
    #     port_type="optical",
    #     cross_section="strip",
    # )
    #
    # route3 = gf.routing.route_single(
    #     top,
    #     switch1.ports["out_top"],
    #     switch2.ports["in_top"],  # connect GC to switch input
    #     port_type="optical",
    #     cross_section="strip",
    # )
    #
    # route4 = gf.routing.route_single(
    #     top,
    #     switch1.ports["out_bottom"],
    #     switch2.ports["in_bottom"],  # connect GC to switch input
    #     port_type="optical",
    #     cross_section="strip",
    # )

    top.show()

    # Print information
    # print(c)
    # print(c.name)
    # print(f"Labels: {c.get_labels((10, 0))}")
    # print(f"Labels: {c.get_labels((1, 10))}")
    # # Convert labels on (1, 10) into ports
    # for lb in c.get_labels((1, 10), recursive=True):
    #     name = lb.string
    #     x, y = lb.trans.disp.x, lb.trans.disp.y
    #     print(f"Adding port {name} at ({x}, {y}) on layer (1, 10)")
    #
    #     c.add_port(
    #         name=name,
    #         center=(x, y),
    #         width=0.5,  # microns
    #         orientation=180,  # or 0, depending on direction
    #         layer=(1, 0),
    #     )

    # # Print information
    # print(c1)
    # print(c1.name)
    # #
    # # # Use it like any gdsfactory component
    # top = gf.Component("demo")
    # c11 = top << c1
    # c12 = top << c2
    # c11.move((0, 0))  # place it at x=50
    # c12.move((150, 20))
    #
    # # --- route: ring1.E → ring2.W ---
    # # route = gf.routing.route_single(
    # #     top,
    # #     c1.ports["o1"],
    # #     c2.ports["o1"],
    # #     port_type="optical",
    # #     layer=(1, 10),
    # #     route_width=0.5
    # # )
    #
    # # print(f"Routed length: {route.length:.3f} um")
    #
    # top.show()  # or top.show()
