"""Parametric N-mode cascade benchmark builder.

Generalizes `benchmarks/heater_s_mod.py` (the hand-written 8-mode "folded"
cascade) to any even mode count >= 4, in two layouts:

- "folded": cascade MZI columns fold -- column 0 holds MZI 0 alone, every
  later column holds two MZIs stacked (the lower one fed by a loop-back
  route), as in `heater_s_mod`.
- "diagonal": one cascade MZI per column, keeping the folded layout's
  1000 um column pitch.

Topology: state preparation (`N/2` lanes, each MMI, phase shifter, MMI,
phase shifter), a chain of `N-3` cascade MZIs each on the next mode pair
(MMI, phase shifter, MMI), and tomography (`N/2` lanes, each phase shifter,
MMI, phase shifter, MMI): `3N-3` heaters in total, 21 for N = 8 (the pad
count of the handcrafted `heater_s_mod` reference layout).

See `.agent/execplans/2026-09-28-cascade-benchmarks.md` for the geometry and
net specification this module implements; the folded 8-mode output of
`build_cascade_schematic(8, "folded")` is the generated twin of
`heater_s_mod.build_schematic()` (same instances, placements and net
endpoint pairs; net names and order may differ).
"""

from __future__ import annotations

from typing import Literal

from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

Layout = Literal["folded", "diagonal"]

# --- geometry constants (all um), read off `heater_s_mod` -----------------
GC_PITCH_UM = 127.0
PATH_OFFSET_Y_UM = 40.0
MMI_LOWER_INPUT_OFFSET_Y_UM = -0.625
MMI_UPPER_INPUT_OFFSET_Y_UM = 0.625

FIRST_MMI_X_UM = 160.0
HEATER_X_UM = 300.0
SECOND_MMI_X_UM = 720.0
POST_HEATER_X_UM = 820.0

CASCADE_COLUMN0_X_UM = 1340.0
CASCADE_COLUMN_BASE_X_UM = 2200.0
CASCADE_COLUMN_PITCH_UM = 1000.0
CASCADE_HEATER_DX_UM = 140.0
CASCADE_SECOND_MMI_DX_UM = 560.0
CASCADE_CENTER_Y_STEP_UM = MMI_LOWER_INPUT_OFFSET_Y_UM - MMI_UPPER_INPUT_OFFSET_Y_UM  # -1.25

TOMOGRAPHY_PRE_HEATER_DX_UM = 740.0
TOMOGRAPHY_FIRST_MMI_DX_UM = 1160.0
TOMOGRAPHY_POST_HEATER_DX_UM = 1300.0
TOMOGRAPHY_SECOND_MMI_DX_UM = 1720.0
TOMOGRAPHY_OUTPUT_GC_DX_UM = 2140.0


def _validate_n_modes(n_modes: int) -> None:
    if n_modes % 2 != 0 or n_modes < 4:
        raise ValueError(f"n_modes must be even and at least 4. Got {n_modes}.")


def node_types(n_modes: int) -> dict[str, str]:
    """Same vocabulary as `heater_s_mod._node_types`: couplers input/output,
    everything else gate."""
    _validate_n_modes(n_modes)
    lane_count = n_modes // 2
    cascade_count = n_modes - 3
    types: dict[str, str] = {}
    for idx in range(n_modes):
        types[f"gc_in_{idx}"] = "input"
        types[f"gc_out_{idx}"] = "output"
    for k in range(lane_count):
        types[f"mmi_a_{k}"] = "gate"
        types[f"heater_{k}"] = "gate"
        types[f"mmi_b_{k}"] = "gate"
        types[f"heater_post_{k}"] = "gate"
    for j in range(cascade_count):
        types[f"mmi_extra_{2 * j}"] = "gate"
        types[f"heater_extra_{j}"] = "gate"
        types[f"mmi_extra_{2 * j + 1}"] = "gate"
    for k in range(lane_count):
        types[f"heater_extra_{cascade_count + k}"] = "gate"
        types[f"mmi_extra_{2 * cascade_count + k}"] = "gate"
        types[f"heater_extra_{cascade_count + lane_count + k}"] = "gate"
        types[f"mmi_extra_{2 * cascade_count + lane_count + k}"] = "gate"
    return types


def internal_delays_um(n_modes: int) -> dict[str, float | str]:
    """ "auto" for heaters, 0.0 otherwise -- same rule as `heater_s_mod`."""
    types = node_types(n_modes)
    return {name: ("auto" if name.startswith("heater_") else 0.0) for name in types}


def _column_of(mzi_index: int, layout: Layout) -> int:
    """Cascade column holding MZI `mzi_index`."""
    if layout == "folded":
        return 0 if mzi_index == 0 else (mzi_index + 1) // 2
    return mzi_index


def _column_x(column: int) -> float:
    if column == 0:
        return CASCADE_COLUMN0_X_UM
    return CASCADE_COLUMN_BASE_X_UM + (column - 1) * CASCADE_COLUMN_PITCH_UM


def build_cascade_schematic(n_modes: int, layout: Layout) -> Schematic:
    """Build the N-mode cascade benchmark (state prep, cascade MZIs, tomography).

    `n_modes` must be even and >= 4. `layout` is "folded" (the handcrafted
    fold, the twin of `heater_s_mod` for `n_modes == 8`) or "diagonal" (one
    cascade MZI per column, same column pitch).
    """
    _validate_n_modes(n_modes)
    if layout not in ("folded", "diagonal"):
        raise ValueError(f"layout must be 'folded' or 'diagonal'. Got {layout!r}.")

    pdk = get_generic_pdk()
    pdk.activate()

    schematic = Schematic()
    gc_instance = Instance(component="grating_coupler_te")
    mmi_instance = Instance(component="mmi2x2")
    heater_instance = Instance(component="straight_heater_metal")

    lane_count = n_modes // 2
    cascade_count = n_modes - 3

    # --- shared geometry ---------------------------------------------------
    gc_y = [((n_modes - 1) / 2 - idx) * GC_PITCH_UM for idx in range(n_modes)]
    lane_centers_y = [(gc_y[2 * k] + gc_y[2 * k + 1]) / 2 for k in range(lane_count)]

    cascade_column = [_column_of(j, layout) for j in range(cascade_count)]
    cascade_first_x = [_column_x(col) for col in cascade_column]
    cascade_heater_x = [x + CASCADE_HEATER_DX_UM for x in cascade_first_x]
    cascade_second_x = [x + CASCADE_SECOND_MMI_DX_UM for x in cascade_first_x]

    cascade_center_y: list[float] = [0.0] * cascade_count
    for j in range(cascade_count):
        if j % 2 == 0:
            cascade_center_y[j] = (
                lane_centers_y[j // 2 + 1] + PATH_OFFSET_Y_UM - MMI_LOWER_INPUT_OFFSET_Y_UM
            )
        else:
            cascade_center_y[j] = cascade_center_y[j - 1] + CASCADE_CENTER_Y_STEP_UM
    cascade_heater_y = [
        cascade_center_y[j] + (PATH_OFFSET_Y_UM if j % 2 == 0 else -PATH_OFFSET_Y_UM)
        for j in range(cascade_count)
    ]

    tomography_x = cascade_second_x[cascade_count - 1]
    tomo_pre_heater_x = tomography_x + TOMOGRAPHY_PRE_HEATER_DX_UM
    tomo_first_mmi_x = tomography_x + TOMOGRAPHY_FIRST_MMI_DX_UM
    tomo_post_heater_x = tomography_x + TOMOGRAPHY_POST_HEATER_DX_UM
    tomo_second_mmi_x = tomography_x + TOMOGRAPHY_SECOND_MMI_DX_UM
    tomo_output_gc_x = tomography_x + TOMOGRAPHY_OUTPUT_GC_DX_UM

    def first_mmi(j: int) -> str:
        return f"mmi_extra_{2 * j}"

    def second_mmi(j: int) -> str:
        return f"mmi_extra_{2 * j + 1}"

    def cascade_heater(j: int) -> str:
        return f"heater_extra_{j}"

    def mode_target(mode: int) -> tuple[str, str]:
        """Instance, port that carries `mode` into tomography."""
        lane = mode // 2
        if mode % 2 == 0:
            return f"heater_extra_{cascade_count + lane}", "o1"
        return f"mmi_extra_{2 * cascade_count + lane}", "o1"

    def add_net(p1: str, p2: str) -> None:
        p1_inst, p1_port = p1.split(",")
        p2_inst, p2_port = p2.split(",")
        schematic.add_net(Net(p1=p1, p2=p2, name=f"{p1_inst}_{p1_port}_to_{p2_inst}_{p2_port}"))

    # --- instances: lanes ----------------------------------------------------
    for k in range(lane_count):
        upper_idx = 2 * k
        lower_idx = upper_idx + 1
        center_y = lane_centers_y[k]
        schematic.add_instance(
            f"gc_in_{upper_idx}", gc_instance, Placement(x=0.0, y=gc_y[upper_idx], mirror=True)
        )
        schematic.add_instance(
            f"gc_in_{lower_idx}", gc_instance, Placement(x=0.0, y=gc_y[lower_idx], mirror=True)
        )
        schematic.add_instance(
            f"mmi_a_{k}", mmi_instance, Placement(x=FIRST_MMI_X_UM, y=center_y, rotation=0)
        )
        schematic.add_instance(
            f"heater_{k}",
            heater_instance,
            Placement(x=HEATER_X_UM, y=center_y + PATH_OFFSET_Y_UM, rotation=0),
        )
        schematic.add_instance(
            f"mmi_b_{k}", mmi_instance, Placement(x=SECOND_MMI_X_UM, y=center_y, rotation=0)
        )
        schematic.add_instance(
            f"heater_post_{k}",
            heater_instance,
            Placement(x=POST_HEATER_X_UM, y=center_y + PATH_OFFSET_Y_UM, rotation=0),
        )
        schematic.add_instance(
            f"gc_out_{upper_idx}",
            gc_instance,
            Placement(x=tomo_output_gc_x, y=gc_y[upper_idx], rotation=0),
        )
        schematic.add_instance(
            f"gc_out_{lower_idx}",
            gc_instance,
            Placement(x=tomo_output_gc_x, y=gc_y[lower_idx], rotation=0),
        )

    # --- instances: cascade MZIs ----------------------------------------------
    for j in range(cascade_count):
        schematic.add_instance(
            first_mmi(j),
            mmi_instance,
            Placement(x=cascade_first_x[j], y=cascade_center_y[j], rotation=0),
        )
        schematic.add_instance(
            cascade_heater(j),
            heater_instance,
            Placement(x=cascade_heater_x[j], y=cascade_heater_y[j], rotation=0),
        )
        schematic.add_instance(
            second_mmi(j),
            mmi_instance,
            Placement(x=cascade_second_x[j], y=cascade_center_y[j], rotation=0),
        )

    # --- instances: tomography ----------------------------------------------
    for k in range(lane_count):
        center_y = lane_centers_y[k]
        schematic.add_instance(
            f"heater_extra_{cascade_count + k}",
            heater_instance,
            Placement(x=tomo_pre_heater_x, y=center_y + PATH_OFFSET_Y_UM, rotation=0),
        )
        schematic.add_instance(
            f"mmi_extra_{2 * cascade_count + k}",
            mmi_instance,
            Placement(x=tomo_first_mmi_x, y=center_y, rotation=0),
        )
        schematic.add_instance(
            f"heater_extra_{cascade_count + lane_count + k}",
            heater_instance,
            Placement(x=tomo_post_heater_x, y=center_y + PATH_OFFSET_Y_UM, rotation=0),
        )
        schematic.add_instance(
            f"mmi_extra_{2 * cascade_count + lane_count + k}",
            mmi_instance,
            Placement(x=tomo_second_mmi_x, y=center_y, rotation=0),
        )

    # --- nets: lanes ------------------------------------------------------
    for k in range(lane_count):
        upper_idx = 2 * k
        lower_idx = upper_idx + 1
        add_net(f"gc_in_{upper_idx},o1", f"mmi_a_{k},o2")
        add_net(f"gc_in_{lower_idx},o1", f"mmi_a_{k},o1")
        add_net(f"mmi_a_{k},o3", f"heater_{k},o1")
        add_net(f"heater_{k},o2", f"mmi_b_{k},o2")
        add_net(f"mmi_a_{k},o4", f"mmi_b_{k},o1")
        add_net(f"mmi_b_{k},o3", f"heater_post_{k},o1")

        if k == 0:
            target_inst, target_port = mode_target(0)
        else:
            target_inst, target_port = first_mmi(2 * (k - 1)), "o1"
        add_net(f"heater_post_{k},o2", f"{target_inst},{target_port}")

        if k == 0:
            target_inst, target_port = first_mmi(0), "o2"
        elif k == lane_count - 1:
            target_inst, target_port = mode_target(n_modes - 1)
        else:
            target_inst, target_port = first_mmi(2 * k - 1), "o1"
        add_net(f"mmi_b_{k},o4", f"{target_inst},{target_port}")

    # --- nets: cascade MZIs -------------------------------------------------
    for j in range(cascade_count):
        heater = cascade_heater(j)
        if j % 2 == 0:
            add_net(f"{first_mmi(j)},o3", f"{heater},o1")
            add_net(f"{heater},o2", f"{second_mmi(j)},o2")
            add_net(f"{first_mmi(j)},o4", f"{second_mmi(j)},o1")
        else:
            add_net(f"{first_mmi(j)},o4", f"{heater},o1")
            add_net(f"{heater},o2", f"{second_mmi(j)},o1")
            add_net(f"{first_mmi(j)},o3", f"{second_mmi(j)},o2")

        target_inst, target_port = mode_target(j + 1)
        add_net(f"{second_mmi(j)},o3", f"{target_inst},{target_port}")

        if j < cascade_count - 1:
            add_net(f"{second_mmi(j)},o4", f"{first_mmi(j + 1)},o2")
        else:
            target_inst, target_port = mode_target(n_modes - 2)
            add_net(f"{second_mmi(j)},o4", f"{target_inst},{target_port}")

    # --- nets: tomography ---------------------------------------------------
    for k in range(lane_count):
        pre_heater = f"heater_extra_{cascade_count + k}"
        tomo_first = f"mmi_extra_{2 * cascade_count + k}"
        post_heater = f"heater_extra_{cascade_count + lane_count + k}"
        tomo_second = f"mmi_extra_{2 * cascade_count + lane_count + k}"
        add_net(f"{pre_heater},o2", f"{tomo_first},o2")
        add_net(f"{tomo_first},o3", f"{post_heater},o1")
        add_net(f"{post_heater},o2", f"{tomo_second},o2")
        add_net(f"{tomo_first},o4", f"{tomo_second},o1")
        add_net(f"{tomo_second},o3", f"gc_out_{2 * k},o1")
        add_net(f"{tomo_second},o4", f"gc_out_{2 * k + 1},o1")

    return schematic


if __name__ == "__main__":
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_cascade_schematic(8, "folded")
    print("Schematic instances:", list(schematic.netlist.instances.keys()))
    print("Schematic placements:", list(schematic.placements.keys()))
    print("\nNets defined:")
    for net_name, bundle in schematic.netlist.routes.items():
        print(f"  - {net_name}: {bundle.links}")
    print(f"\nTotal nets: {len(schematic.netlist.routes)}")

    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout in KLayout: {layout.name}")
    layout.show()
