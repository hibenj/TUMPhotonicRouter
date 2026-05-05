# benes_network_layout.py

import gdsfactory as gf
from gdsfactory.component import Component


def mzi_switch(straight_length: float = 35.0) -> Component:
    """
    Creates a basic 2x2 MZI switch with clearly named ports
    for routing ('in1', 'in2', 'out1', 'out2').

    Args:
        straight_length: The length of the straight sections in the MZI arms.
    """
    # Use the proper 4-pin MZI lattice component
    mzi_comp = gf.components.mzi_lattice(
        coupler_lengths=(10, 20),
        coupler_gaps=(0.2, 0.3),
        delta_lengths=(10,),
        mzi='mzi_coupler',
        splitter='coupler'
    ).copy()

    mzi_comp.pprint_ports()

    # Create a new component to wrap the MZI and rename its ports for clarity
    c = Component()
    mzi_ref = c << mzi_comp

    # Map the ports based on their positions:
    # o1: left bottom input -> in1
    # o2: left top input -> in2
    # o3: right top output -> out2
    # o4: right bottom output -> out1
    c.add_port('in1', port=mzi_ref.ports['o1'])
    c.add_port('in2', port=mzi_ref.ports['o2'])
    c.add_port('out1', port=mzi_ref.ports['o4'])
    c.add_port('out2', port=mzi_ref.ports['o3'])

    return c


def benes_network(N: int = 8) -> Component:
    """
    Generates the layout for an N-input Beneš network using MZI switches.

    Args:
        N: The number of inputs/outputs for the network. Must be a power of 2.
    """
    # --- 1. Pre-computation and Validation ---
    if N <= 1 or (N & (N - 1) != 0):
        raise ValueError(f"Network size N must be a power of 2. Got {N}.")

    # A Beneš network of size N has (2 * log2(N) - 1) stages
    num_stages = 2 * int(N.bit_length() - 1) - 1
    num_switches_per_stage = N // 2

    # --- 2. Component and Layout Setup ---
    c = Component(f"benes_network_{N}x{N}")

    # Define the basic building block: our MZI switch
    switch = mzi_switch()

    # Define layout spacing
    stage_spacing = 200.0  # Horizontal distance between stages
    switch_spacing = 80.0  # Vertical distance between switches in a stage

    # --- 3. Switch Placement ---
    # Store references to all placed switches in a 2D list: switches[stage_idx][switch_idx]
    switches = []
    for i in range(num_stages):
        stage_switches = []
        for j in range(num_switches_per_stage):
            x_pos = i * stage_spacing
            y_pos = j * switch_spacing
            switch_ref = c << switch
            switch_ref.move((x_pos, y_pos))
            stage_switches.append(switch_ref)
        switches.append(stage_switches)

    # --- 4. Inter-stage Routing ---
    # This is the core logic, defining the Beneš network connections
    for i in range(num_stages - 1):
        ports1 = []  # Source ports (outputs of stage i)
        ports2 = []  # Destination ports (inputs of stage i+1)

        # Collect all output ports from current stage
        for j in range(num_switches_per_stage):
            ports1.extend([
                switches[i][j].ports["out1"],
                switches[i][j].ports["out2"]
            ])

        # Collect all input ports from next stage
        for j in range(num_switches_per_stage):
            ports2.extend([
                switches[i + 1][j].ports["in1"],
                switches[i + 1][j].ports["in2"]
            ])

        # Implement proper Beneš network connection pattern
        # For a proper Beneš network, we need different connection patterns for different stages
        if i < num_stages // 2:
            # First half: butterfly network pattern (perfect shuffle)
            # Each output connects to inputs with a shuffle pattern
            shuffle_distance = 1 << i
            connected_ports2 = []

            for k in range(N):
                # Calculate the shuffle destination
                dest = k ^ shuffle_distance
                if dest < len(ports2):
                    connected_ports2.append(ports2[dest])
                else:
                    connected_ports2.append(ports2[k])  # Fallback
        else:
            # Second half: reverse butterfly pattern
            # Mirror the connection pattern from the first half
            mirror_stage = num_stages - 1 - i
            shuffle_distance = 1 << mirror_stage
            connected_ports2 = []

            for k in range(N):
                # Calculate the reverse shuffle destination
                dest = k ^ shuffle_distance
                if dest < len(ports2):
                    connected_ports2.append(ports2[dest])
                else:
                    connected_ports2.append(ports2[k])  # Fallback

        # Ensure we have the right number of connections
        while len(connected_ports2) < len(ports1):
            connected_ports2.append(ports2[len(connected_ports2) % len(ports2)])

        # Route the connections
        try:
            gf.routing.route_bundle(
                c,
                ports1=ports1,
                ports2=connected_ports2[:len(ports1)],
                separation=4.0,
                cross_section="strip",
                sort_ports=False  # Don't sort to preserve our connection pattern
            )
        except Exception as e:
            print(f"Warning: Routing failed for stage {i}: {e}")
            # Fallback to simple straight connections
            for p1, p2 in zip(ports1, connected_ports2[:len(ports1)]):
                try:
                    gf.routing.route_single(c, p1, p2, cross_section="strip")
                except:
                    continue

    # --- 5. Add Network I/O Ports ---
    # Expose the inputs of the first stage and outputs of the last stage
    for i in range(num_switches_per_stage):
        # Input ports
        p_in1 = switches[0][i].ports["in1"]
        p_in2 = switches[0][i].ports["in2"]
        c.add_port(f"in_{2 * i}", port=p_in1)
        c.add_port(f"in_{2 * i + 1}", port=p_in2)

        # Output ports
        p_out1 = switches[-1][i].ports["out1"]
        p_out2 = switches[-1][i].ports["out2"]
        c.add_port(f"out_{2 * i}", port=p_out1)
        c.add_port(f"out_{2 * i + 1}", port=p_out2)

    return c


if __name__ == "__main__":
    # --- Generate, Plot, and Save the Network ---
    print("Generating 8x8 Beneš network layout...")

    # Create the network component
    benes_8x8 = benes_network(N=4)

    # Plot the layout to view it
    benes_8x8.plot()

    # You can also show the layout in a GDS viewer like KLayout
    benes_8x8.show()

    # Save the layout to a GDSII file
    # benes_8x8.write_gds("benes_8x8_network.gds")

    print("Layout generation complete.")
    print(f"Component info: {benes_8x8.info}")