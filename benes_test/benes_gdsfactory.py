import gdsfactory as gf
import gdsfactory.components
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

def _get_bbox_center(component: gf.Component):
    """Return the (x_center, y_center) of a component based on its bbox."""
    (xmin, ymin), (xmax, ymax) = _get_bbox(component)
    x_center = (xmin + xmax) / 2
    y_center = (ymin + ymax) / 2
    return x_center, y_center

def rotate_and_align(ref, angle):
    # bbox before rotation
    (xmin_before, ymin_before), _ = _get_bbox(ref)

    # rotate
    ref.rotate(angle)

    # bbox after rotation
    (xmin_after, ymin_after), _ = _get_bbox(ref)

    # shift back to original position
    dx = xmin_before - xmin_after
    dy = ymin_before - ymin_after

    ref.move((dx, dy))

import gdsfactory as gf


def add_ports_nesw(comp: gf.Component, width: float = 0.5, layer=(1, 0)):
    """
    Adds 4 optical ports (N, S, E, W) to a component based on its bounding box.

    Args:
        cross: gdsfactory component (without ports yet)
        width: optical port width (µm)
        layer: layer tuple for the ports (default = (1, 0))

    Returns:
        comp: same component with ports added
    """
    (xmin, ymin), (xmax, ymax) = _get_bbox(comp)
    w, h = _get_bbox_dims(comp)

    # Compute center coordinates
    x_center = xmin + w / 2
    y_center = ymin + h / 2

    # --- Add ports (assuming comp is centered) ---
    # North (up)
    comp.add_port(
        name="n",
        center=(x_center, ymax),
        width=width,
        orientation=90,
        layer=layer,
        port_type="optical",
    )

    # South (down)
    comp.add_port(
        name="s",
        center=(x_center, ymin),
        width=width,
        orientation=-90,
        layer=layer,
        port_type="optical",
    )

    # East (right)
    comp.add_port(
        name="e",
        center=(xmax, y_center),
        width=width,
        orientation=0,
        layer=layer,
        port_type="optical",
    )

    # West (left)
    comp.add_port(
        name="w",
        center=(xmin, y_center),
        width=width,
        orientation=180,
        layer=layer,
        port_type="optical",
    )

    return comp

def add_ports_edges(comp: gf.Component, width: float = 0.5, layer=(1, 0)):
    (xmin, ymin), (xmax, ymax) = _get_bbox(comp)
    w, h = _get_bbox_dims(comp)

    # --- Add ports (assuming comp is centered) ---
    # North (up)
    comp.add_port(
        name="o1",
        center=(xmin, ymin),
        width=width,
        orientation=90,
        layer=layer,
        port_type="optical",
    )

    # South (down)
    comp.add_port(
        name="o2",
        center=(xmax, ymin),
        width=width,
        orientation=-90,
        layer=layer,
        port_type="optical",
    )

    # East (right)
    comp.add_port(
        name="o3",
        center=(xmax, ymax),
        width=width,
        orientation=0,
        layer=layer,
        port_type="optical",
    )

    # West (left)
    comp.add_port(
        name="o4",
        center=(xmin, ymax),
        width=width,
        orientation=180,
        layer=layer,
        port_type="optical",
    )

    return comp


@gf.cell
def switch(
        ring_component: gf.Component,
        gap: float = 0.25
) -> gf.Component:
    """Builds one switch (two asymmetric rings) as a reusable component.

    Args:
        ring_component: the single-ring component to duplicate and couple.
        gap: desired vertical separation (µm) between the rings' coupling buses.
        x, y: position offset for the upper ring.
    """
    c = gf.Component("switch")
    r1 = c << ring_component
    r2 = c << ring_component
    rotate_and_align(r2, 180)
    (x_dim), (y_dim) = _get_bbox_dims(ring)
    r1.movey(-(y_dim+gap))
    print(y_dim)

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
    Computes the structural parameters of an n-bit Benes network (0-based indexing).

    Returns
    -------
    dict
        {
            "n": n,
            "N": number of inputs/outputs (2^n),
            "stages": number of stages (2n - 1),
            "elements": number of 2x2 switches per stage (N/2),
            "connections": list of stage connections as (a, b) pairs (0-based)
        }
    """
    N = 1 << n  # number of inputs/outputs
    stages = 2 * n - 1
    elements = N // 2

    # connections[stage] = list of (a, b) pairs (0-based)
    connections = [[] for _ in range(stages - 1)]

    # --- Forward half ---
    for i in range((stages - 1) // 2):
        step = elements >> i  # elements / 2^i
        half = step >> 1  # step / 2

        for g in range(elements // step):  # number of blocks
            for j in range(half):  # pairs inside a block
                a = g * step + j  # 0-based
                b = a + half
                p = (a, b)
                connections[i].append(p)
                connections[i].append(p)  # duplicates, like in C++

    # --- Helper for inverse stage (0-based) ---
    def build_inverse_stage(forward, backward):
        backward.clear()
        m = len(forward)
        pos = [[] for _ in range(m)]  # pos[node] -> entry indices (0-based)

        # Record where each node appears
        for idx, (a, b) in enumerate(forward):
            pos[a].append(idx)
            pos[b].append(idx)

        # For duplicates: each node appears twice -> connect those entry indices
        for u, lst in enumerate(pos):
            if len(lst) >= 2:
                lst.sort()
                backward.append((lst[0], lst[1]))

    # --- Backward (mirror) half ---
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


def _init_port_usage_switch(switch_refs):
    """Make a usage map: {(stage, elem): {'left': {'o1':False,'o4':False},
                                          'right':{'o2':False,'o3':False}}}"""
    usage = {}
    for s, stage in enumerate(switch_refs):
        for e, _ in enumerate(stage):
            usage[(s, e)] = {
                "left": {"o1": False, "o4": False},
                "right": {"o2": False, "o3": False},
            }
    return usage

def _init_port_usage_cross(cross_refs):
    """Make a usage map: {(stage, elem): {'left': {'s':False,'w':False},
                                          'right':{'n':False,'e':False}}}"""
    usage = {}
    for s, stage in enumerate(cross_refs):
        for e, _ in enumerate(stage):
            usage[(s, e)] = {
                "left": {"n": False, "w": False},
                "right": {"s": False, "e": False},
            }
    return usage

def _take_cross_port(ref, usage, key, side, candidates):
    """Return the first available port from 'candidates' on the given side."""
    for p in candidates:
        if not usage[key][side][p]:
            usage[key][side][p] = True
            return ref.ports[p]
    raise ValueError(f"No available {side} ports left for crossing {key}")


def _take_switch_port(cref, usage, key, side, prefer):
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


def count_crossings(connections):
    """
    Counts wire crossings between consecutive stages (0-based) and records:
      - all crossing pairs per stage
      - all edges that are unaffected (not part of any crossing)
      - how many crossings occur in each sub-layer (stage_val)

    Returns
    -------
    total_crossings : int
    results_per_stage : list[dict]
        Each stage entry has:
            {
              "crossings": [((src1, dst1), (src2, dst2), stage_val), ...],
              "unaffected": [(src, dst), ...],
              "crossings_per_layer": {layer_val: count, ...}
            }
    """
    total_crossings = 0
    results_per_stage = []

    for stage_idx, stage in enumerate(connections):
        next_width = max(max(pair) for pair in stage) + 1
        penalty = [[] for _ in range(next_width)]
        max_pos = -1

        stage_crossings = []
        affected_edges = set()
        crossings_per_layer = {}  # <-- new: counts per layer

        for src_idx, (a, b) in enumerate(stage):
            for pos in (a, b):
                local_cross = 0

                # reversed iteration over later destinations
                for k in reversed(range(pos + 1, max_pos + 1)):
                    for j, (prev_edge, stage_val) in enumerate(penalty[k]):
                        layer_val = max(stage_val, local_cross)
                        stage_crossings.append(
                            ((src_idx, pos), prev_edge, layer_val)
                        )

                        # count occurrences immediately
                        crossings_per_layer[layer_val] = (
                                crossings_per_layer.get(layer_val, 0) + 1
                        )

                        affected_edges.add(prev_edge)
                        affected_edges.add((src_idx, pos))

                        if stage_val > local_cross:
                            local_cross = stage_val

                        penalty[k][j] = (prev_edge, stage_val + 1)
                        total_crossings += 1
                        local_cross += 1

            # Add current edge to penalty array (prepend)
            for pos in (a, b):
                max_pos = max(max_pos, pos)
                penalty[pos].insert(0, ((src_idx, pos), 0))

        all_edges = {(src_idx, pos) for src_idx, (a, b) in enumerate(stage) for pos in (a, b)}
        unaffected_edges = sorted(all_edges - affected_edges)

        results_per_stage.append({
            "crossings": stage_crossings,
            "unaffected": unaffected_edges,
            "crossings_per_layer": dict(sorted(crossings_per_layer.items())),
        })

    return total_crossings, results_per_stage


def connect_stages(c, switch_refs, cross_refs, connections, cross_ctns, cross_section="strip"):
    """
    Route all internal Benes connections stage-by-stage using port-usage flags.

    connections[x][i] = (a, b) where:
      - source = switch_refs[x][i]
      - dest A = switch_refs[x+1][a]
      - dest B = switch_refs[x+1][b]
    Rules:
      - source (right side): prefer o2 then o3
      - dest   (left  side): prefer o1 then o4
      - raises error if a side runs out of ports
    """
    usage_switch = _init_port_usage_switch(switch_refs)
    usage_cross = _init_port_usage_cross(cross_refs)
    routes = []

    # for s, stage in enumerate(cross_ctns):
    #     print(f"Stage {s}:")
    #     print("  Crossings:")
    #     for elem in stage["crossings"]:
    #         print("   ", elem)
    #     print("  Unaffected edges:", stage["unaffected"])
    #     print("  Crossings per layer:", stage["crossings_per_layer"])

    for x, stage_conn in enumerate(connections):
        print("stage", x)
        src_stage = switch_refs[x]
        dst_stage = switch_refs[x + 1]

        cross_ctn = cross_ctns[x]
        unaff = cross_ctn["unaffected"]
        ordered_crossings = sorted(cross_ctn["crossings"], key=lambda x: x[-1])
        # for elem in ordered_crossings:
        #     print("   ", elem)

        stage_refs = cross_refs[x]
        for src_i, (a, b) in enumerate(stage_conn):
            src_key = (x, src_i)
            dstA_key = (x + 1, a)
            dstB_key = (x + 1, b)

            src_ref = src_stage[src_i]
            dstA_ref = dst_stage[a]
            dstB_ref = dst_stage[b]

            pair_a = (src_i, a)
            pair_b = (src_i, b)
            print("pair_a", pair_a)
            print("pair_b", pair_b)

            def route_pair(pair, src_ref, src_key, dst_ref, dst_key):
                cross_key_prev = 0
                cross_ref_prev = 0

                for i, elem in enumerate(ordered_crossings):
                    if pair in (elem[0], elem[1]):
                        cross_ref = stage_refs[i]
                        cross_key = (x, i)

                        if cross_ref_prev == 0:
                            p_src = _take_switch_port(src_ref, usage_switch, src_key, "right", ["o2", "o3"])
                        else:
                            p_src = _take_cross_port(cross_ref_prev, usage_cross, cross_key_prev, "right", ["s", "e"])

                        # Destination (cross_ref) and its y
                        p_dst = _take_cross_port(cross_ref, usage_cross, cross_key, "left", ["n", "w"])
                        _, y_curr = _get_bbox_center(cross_ref)

                        # print("p_src:", p_src)
                        # print("p_dst:", p_dst)
                        # print("p_src.center:", getattr(p_src, "center", None))
                        # print("p_dst.center:", getattr(p_dst, "center", None))

                        x1, y1 = p_src.center
                        x2, y2 = p_dst.center

                        # 2-step jog: horizontal then vertical
                        steps = [
                            {"x": x2, "y": y1},
                        ]

                        routes.append(
                            gf.routing.route_single(
                                c, p_src, p_dst,
                                port_type="optical", cross_section=cross_section,
                            )
                        )

                        # route = gf.routing.route_astar(
                        #     component=c,
                        #     port1=p_src,
                        #     port2=p_dst,
                        #     cross_section=cross_section,
                        #     resolution=15,
                        #     distance=12,
                        # )

                        cross_ref_prev = cross_ref
                        cross_key_prev = cross_key

                # Connect final crossing → destination switch
                if cross_ref_prev != 0:
                    p_src_last = _take_cross_port(cross_ref_prev, usage_cross, cross_key_prev, "right", ["s", "e"])
                    p_dst_last = _take_switch_port(dst_ref, usage_switch, dst_key, "left", ["o1", "o4"])

                    routes.append(
                        gf.routing.route_single(
                            c, p_src_last, p_dst_last,
                            port_type="optical", cross_section=cross_section,
                            auto_taper=True, allow_width_mismatch=True,
                        )
                    )

            def route_pair_step(pair, src_ref, src_key, dst_ref, dst_key):
                cross_key_prev = 0
                cross_ref_prev = 0

                for i, elem in enumerate(ordered_crossings):
                    if pair in (elem[0], elem[1]):
                        cross_ref = stage_refs[i]
                        cross_key = (x, i)

                        if cross_ref_prev == 0:
                            print("Switch source")
                            p_src = _take_switch_port(src_ref, usage_switch, src_key, "right", ["o2", "o3"])
                        else:
                            print("Crossing source")
                            p_src = _take_cross_port(cross_ref_prev, usage_cross, cross_key_prev, "right", ["s", "e"])

                        # Destination (cross_ref) and its y
                        p_dst = _take_cross_port(cross_ref, usage_cross, cross_key, "left", ["n", "w"])
                        _, y_curr = _get_bbox_center(cross_ref)

                        # print("p_src:", p_src)
                        # print("p_dst:", p_dst)
                        # print("p_src.center:", getattr(p_src, "center", None))
                        # print("p_dst.center:", getattr(p_dst, "center", None))

                        if cross_ref_prev == 0:
                            x_src, y_src = _get_bbox(src_ref)
                            print("x_src, y_src", x_src, y_src)
                            print(src_ref.x)
                            print(src_ref.y)
                        else:
                            x_src, y_src = _get_bbox(src_ref)
                            print("x_src, y_src", x_src, y_src)
                            print(cross_ref_prev.x)
                            print(cross_ref_prev.y)

                        (x_dst_min, y_dst_min), (x_dst_max, y_dst_max) = _get_bbox(cross_ref)
                        print("x_dst_min, y_dst_min", x_dst_min, y_dst_min)
                        print("x_dst_max, y_dst_max", x_dst_max, y_dst_max)
                        print(cross_ref.x)
                        print(cross_ref.y)

                        x1, y1 = p_src.center
                        x2, y2 = p_dst.center

                        if cross_ref_prev != 0:
                            y1 -= 10

                        steps = [
                            {"x": x2, "y": y1},
                        ]

                        print("x1, y1:", x1, y1)
                        print("x2, y2:", x2, y2)

                        route = None

                        for i in range(100):  # try up to 10 times
                            try:
                                # decrease both x2 and y1 gradually by 10 µm each iteration
                                dx = -10 * i
                                dy = -10 * i
                                steps = [{"x": x2 + dx, "y": y1 + dy}]

                                print(f"Try {i + 1}: steps={steps}")

                                route = gf.routing.route_single(
                                    c, p_src, p_dst,
                                    port_type="optical",
                                    cross_section=cross_section,
                                    steps=steps,
                                )

                                print(f"Routing succeeded on try {i + 1}")
                                break  # ✅ success, exit loop

                            except Exception as e:
                                print(f"Routing attempt {i + 1} failed: {e}")
                                continue  # try next offset

                        # If all attempts fail, raise an error
                        if route is None:
                            raise RuntimeError("Routing failed after 10 attempts with offsets")

                        routes.append(route)

                        cross_ref_prev = cross_ref
                        cross_key_prev = cross_key

                # Connect final crossing → destination switch
                if cross_ref_prev != 0:
                    p_src_last = _take_cross_port(cross_ref_prev, usage_cross, cross_key_prev, "right", ["s", "e"])
                    p_dst_last = _take_switch_port(dst_ref, usage_switch, dst_key, "left", ["o1", "o4"])

                    routes.append(
                        gf.routing.route_single(
                            c, p_src_last, p_dst_last,
                            port_type="optical", cross_section=cross_section,
                            auto_taper=True, allow_width_mismatch=True,
                        )
                    )

            # Unaffected Nodes
            if pair_a in unaff:
                # First link (prefer o2 on src-right, o1 on dst-left)
                p_src_1 = _take_switch_port(src_ref, usage_switch, src_key, "right", ["o2", "o3"])
                p_dst_1 = _take_switch_port(dstA_ref, usage_switch, dstA_key, "left", ["o1", "o4"])
                routes.append(
                    gf.routing.route_single(
                        c, p_src_1, p_dst_1, port_type="optical", cross_section=cross_section
                    )
                )
            else:
                route_pair(pair_a, src_ref, src_key, dstA_ref, dstA_key)

            if pair_b in unaff:
                # Second link (next available on each side)
                p_src_2 = _take_switch_port(src_ref, usage_switch, src_key, "right", ["o2", "o3"])
                p_dst_2 = _take_switch_port(dstB_ref, usage_switch, dstB_key, "left", ["o1", "o4"])
                routes.append(
                    gf.routing.route_single(
                        c, p_src_2, p_dst_2, port_type="optical", cross_section=cross_section
                    )
                )
            else:
                route_pair(pair_b, src_ref, src_key, dstB_ref, dstB_key)

    return routes


@gf.cell
def benes_array(
        n: int,
        switch_component: gf.Component,
        grating_coupler: gf.Component,
        crossing: gf.Component,
        x_layer_gap: float = 0.0,
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
    x_pitch = sw_w + x_layer_gap

    connections = params['connections']

    num_cross, cross_ctns = count_crossings(connections)

    input_refs = []
    output_refs = []
    switch_refs = []
    cross_refs = []

    x_crossings = 0.0  # cumulative offset for all crossings before the current stage

    for stage in range(stages):
        # --- Place switches for this stage ---
        stage_refs_switch = []
        stage_refs_cross = []
        y_bottom = 0
        y_top = 0
        for elem in range(elements):
            sw_ref = c << switch_component
            # base x now includes accumulated crossing offset
            x = stage * x_pitch + x_crossings
            y = -elem * y_pitch
            sw_ref.move((x, y))
            stage_refs_switch.append(sw_ref)

            (xmin, ymin), (xmax, ymax) = _get_bbox(sw_ref)

            if elem == 0:
                y_bottom = ymax - (waveguide_width / 2)
            elif elem == elements - 1:
                y_top = ymin + (waveguide_width / 2)

            # --- Input couplers (first stage) ---
            if stage == 0:
                gc_ref = c << grating_coupler
                gc_ref.rotate(90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x - x_gc, ymax - (waveguide_width / 2)))
                input_refs.append(gc_ref)

                gc_ref = c << grating_coupler
                gc_ref.rotate(90)
                (x_gc), (y_gc) = _get_bbox_dims(gc_ref)
                gc_ref.move((x - x_gc, ymin + (waveguide_width / 2)))
                input_refs.append(gc_ref)

            # --- Output couplers (last stage) ---
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

        switch_refs.append(stage_refs_switch)

        # --- Place crossings after stage placement ---
        if stage < stages - 1:
            cross_ctn = cross_ctns[stage]
            cpl = cross_ctn["crossings_per_layer"]

            # Sort crossings by layer index
            ordered_crossings = sorted(cross_ctn["crossings"], key=lambda x: x[-1])

            # y-center range of this stage
            y_center = (y_top + y_bottom) / 2.0

            # Dictionary to track which edge currently ends at which crossing
            edge_to_cross = {}

            # iterate through layers of crossings
            layer_count = 0
            x_cross = 0
            x_shift_step = 200.0  # µm, adjust as needed

            for layer, count in sorted(cpl.items()):
                layer_count += (count - 1)

                # compute X for crossings, including stage base and layer offset
                x_cross = ((stage + layer + 1) * x_pitch + x_crossings) + x_shift_step * layer_count

                # iterate over crossings in this layer only
                crossings_in_layer = [cross for cross in ordered_crossings if cross[-1] == layer]

                for i, cross in enumerate(crossings_in_layer):
                    (src_a, dst_a), (src_b, dst_b), _ = cross
                    x_local = x_cross - i * x_shift_step
                    cross_ref = c << crossing

                    # --- determine predecessors for both input edges ---
                    # if the edge already has a crossing, use it; otherwise use the source switch
                    if (src_a, dst_a) in edge_to_cross:
                        pred1_ref = edge_to_cross[(src_a, dst_a)]
                    else:
                        pred1_ref = stage_refs_switch[src_a]

                    if (src_b, dst_b) in edge_to_cross:
                        pred2_ref = edge_to_cross[(src_b, dst_b)]
                    else:
                        pred2_ref = stage_refs_switch[src_b]

                    # compute new y position as midpoint of predecessor y-positions
                    _, y1 = _get_bbox_center(pred1_ref)
                    _, y2 = _get_bbox_center(pred2_ref)
                    y_cross = (y1 + y2) / 2.0

                    # place the crossing
                    cross_ref.move((round(x_local, 0), round(y_cross, 0)))
                    stage_refs_cross.append(cross_ref)

                    # update dictionary so future layers use this crossing as source
                    edge_to_cross[(src_a, dst_a)] = cross_ref
                    edge_to_cross[(src_b, dst_b)] = cross_ref

            # After placing all crossings for this stage, update total offset
            x_crossings = x_cross - stage * x_pitch
            # Append the crossing Refs for this stage
            cross_refs.append(stage_refs_cross)

    connect_inputs(c, input_refs, switch_refs, cross_section="strip")
    connect_outputs(c, output_refs, switch_refs, elements, cross_section="strip")

    # First do routing for unaffected edges
    connect_stages(c, switch_refs, cross_refs, connections, cross_ctns)

    return c


# # y-center range of this stage
# y_center = (y_top + y_bottom) / 2.0
#
# # iterate through layers of crossings
# layer_count = 0
# x_cross = 0
# x_shift_step = 200.0  # µm, adjust as needed
#
# for layer, count in sorted(cpl.items()):
#     layer_count += (count - 1)
#
#     # compute X for crossings, including stage base and layer offset
#     x_cross = ((stage + layer + 1) * x_pitch + x_crossings) + x_shift_step * layer_count
#
#     # iterate over crossings in this layer only
#     crossings_in_layer = [cross for cross in ordered_crossings if cross[-1] == layer]
#
#     for i, cross in enumerate(crossings_in_layer):
#         x_local = x_cross - i * x_shift_step
#         cross_ref = c << crossing
#
#         if layer == 0:
#             # First layer: align between source switches
#             (src_a, _), (src_b, _), _ = cross
#             sw1_ref = stage_refs_switch[src_a]
#             sw2_ref = stage_refs_switch[src_b]
#             _, y_sw1 = _get_bbox_center(sw1_ref)
#             _, y_sw2 = _get_bbox_center(sw2_ref)
#             y_cross = (y_sw1 + y_sw2) / 2.0
#         else:
#             # Later layers: evenly spaced around y_center
#             if count == 1:
#                 y_cross = y_center
#             else:
#                 total_height = (count - 1) * y_pitch
#                 y_cross = y_center + (total_height / 2) - i * y_pitch
#
#         # place the crossing
#         cross_ref.move((x_local, y_cross))
#         stage_refs_cross.append(cross_ref)


if __name__ == "__main__":
    # Start the test
    print("Start testing...")

    # GDS-Datei importieren
    gc = gf.read.import_gds(
        "/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/photonics/Projects/Carleton_collaboration/PDK/NanoSOI_PDK_v74/tech/libraries/ANT_PDK_Silicon_v74.gds",
        cellname="GratingCoupler_TM_Oxide_8degrees")

    ring = gdsfactory.components.ring_single()
    ring_single_heater = gdsfactory.components.ring_single_heater()
    ring_double_heater = gdsfactory.components.ring_double_heater()

    cross = gf.read.import_gds(
        "/home/benjamin/Documents/Repositories/cda.cit.tum.gitlab/photonics/Projects/Carleton_collaboration/Custom_Elements/Wg_crossing_flt.gds",
        cellname="PATHS")

    gc = gf.add_ports.add_ports_from_markers_inside(gc, pin_layer="PORT", port_layer="WG")
    cross = add_ports_nesw(cross, width=0.45)
    #cross.rotate(45)

    #gc.pprint_ports()
    ring.pprint_ports()
    #cross.pprint_ports()

    for port in gc.ports:
        x, y = port.center
        port.center = (x, y + 0.05)

    top = gf.Component("Benes network")

    sw = switch(ring, gap=0.2)

    sw_heater = switch(ring_single_heater, gap=0.2)

    sw.pprint_ports()

    # array = benes_array(n=1, switch_component=sw, grating_coupler=gc, crossing=cross, x_layer_gap=100.0, y_min_gap=200,
    #                     waveguide_width=0.5)

    ar1 = top << sw

    top.show()
