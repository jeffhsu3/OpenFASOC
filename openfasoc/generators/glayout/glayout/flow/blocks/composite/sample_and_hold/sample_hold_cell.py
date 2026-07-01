from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.blocks.elementary.transmission_gate.transmission_gate import (
    transmission_gate,
)
from glayout.flow.primitives.fet import nmos
from glayout.flow.primitives.mimcap import mimcap_array
from glayout.flow.spice.netlist import Netlist


def _sample_hold_netlist(
    tg_comp: Component, mim_comp: Component, rst_comp: Component | None = None
) -> Netlist:
    """Composite source netlist for the sample-and-hold cell."""
    nodes = ["VIN", "CLK", "CLK_B", "VSS", "VOUT", "VCC"]
    if rst_comp is not None:
        nodes.extend(["RESET", "VZERO"])

    netlist = Netlist(
        circuit_name="sample_hold_cell",
        nodes=nodes,
    )

    netlist.connect_netlist(
        tg_comp.info["netlist"],
        [
            ("VIN", "VIN"),
            ("VSS", "VSS"),
            ("VOUT", "VOUT"),
            ("VCC", "VCC"),
            ("VGP", "CLK_B"),
            ("VGN", "CLK"),
        ],
    )

    netlist.connect_netlist(
        mim_comp.info["netlist"],
        [
            ("V1", "VOUT"),
            ("V2", "VSS"),
        ],
    )

    if rst_comp is not None:
        netlist.connect_netlist(
            rst_comp.info["netlist"],
            [
                ("D", "VOUT"),
                ("G", "RESET"),
                ("S", "VZERO"),
                ("B", "VSS"),
            ],
        )

    return netlist


@cell
def sample_hold_cell(
    pdk: MappedPDK,
    switch_width: tuple[float, float] = (1.0, 1.0),
    switch_fingers: tuple[int, int] = (2, 2),
    switch_multipliers: tuple[int, int] = (1, 1),
    cap_size: tuple[float, float] = (5.0, 5.0),
    cap_rows: int = 1,
    cap_cols: int = 1,
    with_reset: bool = True,
    reset_fingers: int = 1,
) -> Component:
    """
    A reusable Sample-and-Hold (S&H) storage cell.
    Samples VIN to VOUT on CLK/CLK_B.
    Optionally resets VOUT to VZERO when RESET is high.
    """
    top_level = Component(name="sample_hold_cell")
    placement_sep = 0.0

    def place_left_of(left_ref, right_ref, separation: float = placement_sep):
        left_ref.movex(right_ref.xmin - left_ref.xmax - separation)
        left_ref.movey(right_ref.center[1] - left_ref.center[1])

    tg_comp = transmission_gate(
        pdk=pdk,
        width=switch_width,
        fingers=switch_fingers,
        multipliers=switch_multipliers,
        inter_finger_topmet="met1",
    )
    tg_ref = top_level << tg_comp

    mim_comp = mimcap_array(
        pdk=pdk,
        rows=cap_rows,
        columns=cap_cols,
        size=cap_size,
    )
    mim_ref = top_level << mim_comp

    place_left_of(mim_ref, tg_ref)

    rst_comp = None
    rst_ref = None
    if with_reset:
        if reset_fingers > 1:
            # :TODO auto-adjust to τ = Ron_reset × C_hold
            raise NotImplementedError
        rst_comp = nmos(
            pdk,
            width=1.0,
            length=pdk.get_grule("poly")["min_width"],
            fingers=1,
            multipliers=1,
            with_tie=True,
            with_dnwell=False,
            with_substrate_tap=False,
            inter_finger_topmet="met1",
            sd_route_topmet="met2",
            gate_route_topmet="met2",
        )
        rst_ref = top_level << rst_comp
        place_left_of(rst_ref, mim_ref)

    from glayout.flow.primitives.via_gen import via_stack
    from glayout.flow.routing.straight_route import straight_route

    def drop_via(layer1, layer2, port):
        v = via_stack(pdk, layer1, layer2, fulltop=True, fullbottom=True)
        v_ref = top_level << v
        v_ref.movex(port.center[0] - v_ref.center[0])
        v_ref.movey(port.center[1] - v_ref.center[1])
        return v_ref

    # The MIM cap top/bottom plate metals are PDK-dependent (both gf180 and sky130
    # use met5/met4 top/bottom, but other PDKs may differ). Derive them from the
    # cap's actual ports so this cell is PDK-agnostic rather than hardcoding a stack.
    mim_top_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_top_met_E"].layer)
    mim_bottom_layer = pdk.layer_to_glayer(
        mim_ref.ports["row0_col0_bottom_met_E"].layer
    )

    if with_reset:
        rst_drain_port = rst_ref.ports["multiplier_0_drain_E"].copy()
        mim_top_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_top_met_W"].layer)
        mim_layer_top_w = mim_ref.ports["row0_col0_top_met_W"].copy()
        rst_drain_dest = rst_drain_port.copy()
        rst_drain_dest.center = (mim_layer_top_w.center[0], rst_drain_port.center[1])
        top_level << straight_route(
            pdk,
            rst_drain_port,
            rst_drain_dest,
            glayer1=mim_top_layer,
            glayer2=mim_top_layer,
        )

    # TG VOUT (V1 = top plate). Outside the reset block so VOUT always connects to the
    # hold cap, with or without the reset switch. Single met2->met5 via at the TG drain,
    # then a straight met5 run west onto the top plate at the drain's y -- mirrors the
    # VSS routing but climbs to the TOP plate. The via sits at the drain, EAST of the
    # cap and off its footprint, so the met2->met5 stack never punches through the met4
    # bottom plate (which would short VOUT to VSS).
    #
    # Use the HIGHEST PMOS multiplier index: multiplier_0 is the topmost drain (furthest
    # from the TG center), and the cap plate is centered on the TG, so the highest index
    # (the bottommost drain, nearest center) is the one that lands within the plate
    # y-extent for multipliers > 1. switch_multipliers is (NMOS, PMOS); the drain is PMOS.
    pmos_mult = int(switch_multipliers[1])
    tg_vout_port = tg_ref.ports[f"P_multiplier_{pmos_mult - 1}_drain_W"].copy()
    tg_vout_port.center = (tg_vout_port.center[0] + 0.1, tg_vout_port.center[1])

    # The straight met5 run lands on the plate only if the drain's y is within the
    # top-plate y-extent. For tall switches with a short cap (e.g. asymmetric
    # multipliers + a small cap) even the bottommost PMOS drain can sit above the
    # plate, which would silently leave VOUT open. Fail loudly instead.
    # This happens around (4, 4) caps
    tp_s = mim_ref.ports["row0_col0_top_met_S"].center[1]
    tp_n = mim_ref.ports["row0_col0_top_met_N"].center[1]
    if not (tp_s <= tg_vout_port.center[1] <= tp_n):
        raise NotImplementedError(
            f"TG VOUT drain (y={tg_vout_port.center[1]:.2f}) falls outside the cap "
            f"top-plate y-extent [{tp_s:.2f}, {tp_n:.2f}] for "
            f"switch_multipliers={switch_multipliers}, cap_size={cap_size}: the "
            f"straight met5 route would leave VOUT open. Increase the cap height "
            f"(cap_size[1]) or rebalance the switch multipliers."
        )

    tg_vout_via = drop_via("met2", mim_top_layer, tg_vout_port)
    vout_inset = pdk.get_grule(mim_top_layer)["min_separation"]
    vout_met5_dst = tg_vout_via.ports["top_met_W"].copy()
    vout_met5_dst.center = (
        mim_ref.ports["row0_col0_top_met_E"].center[0] - vout_inset,
        vout_met5_dst.center[1],
    )
    top_level << straight_route(
        pdk,
        tg_vout_via.ports["top_met_W"],
        vout_met5_dst,
        glayer1=mim_top_layer,
        glayer2=mim_top_layer,
    )

    # VSS (V2 = bottom plate): a single straight_route from the NMOS tie-north. Because
    # the tie is on met2 but glayer1/2 are met4, straight_route auto-inserts the met2->met4
    # front-via AT the tie and runs met4 west onto the bottom plate -- no separate via call.
    # The tie-north y sits within the plate y-extent (even for the smallest cap), so the
    # met4 wire lands directly on the plate; inset the endpoint one met4 min_separation so
    # the wire overlaps the plate rather than just meeting its edge.
    plate_e = mim_ref.ports["row0_col0_bottom_met_E"]
    tie_w = tg_ref.ports["N_tie_N_top_met_W"]
    vss_dst = tie_w.copy()
    # vss_dst.center = (plate_e.center[0] - pdk.get_grule(mim_bottom_layer)["min_separation"], tie_w.center[1])
    # top_level << straight_route(pdk, tie_w, vss_dst, glayer1=mim_top_layer, glayer2=mim_top_layer)
    vss_dst.center = (plate_e.center[0], tie_w.center[1])
    top_level << straight_route(
        pdk, tie_w, vss_dst, glayer1=mim_bottom_layer, glayer2=mim_bottom_layer
    )
    # drop_via(mim_top_layer, mim_bottom_layer, vss_dst)

    # Expose Ports
    def expose(name: str, port, glayer: str | None = None):
        top_level.add_port(name=name, port=port)
        # Use port's actual layer if glayer not provided
        if glayer is None:
            glayer = pdk.layer_to_glayer(port.layer)

        # Move the label slightly inside the polygon to ensure Magic attaches it properly
        import numpy as np

        angle = port.orientation
        if angle is not None:
            dx = -0.1 * np.cos(np.radians(angle))
            dy = -0.1 * np.sin(np.radians(angle))
        else:
            dx, dy = 0, 0
        pos = (port.center[0] + dx, port.center[1] + dy)

        top_level.add_label(text=name, position=pos, layer=pdk.get_glayer(glayer))

    expose("VIN", tg_ref.ports["P_multiplier_0_source_E"])
    expose("CLK", tg_ref.ports["N_multiplier_0_gate_E"])
    expose("CLK_B", tg_ref.ports["P_multiplier_0_gate_E"])

    expose("VCC", tg_ref.ports["P_tie_S_top_met_S"])
    expose("VSS", tg_ref.ports["N_tie_S_top_met_N"])
    expose("VOUT", mim_ref.ports["row0_col0_top_met_W"])
    expose("VOUT_TAP", tg_vout_via.ports["bottom_met_N"])

    if with_reset:
        expose("RESET", rst_ref.ports["multiplier_0_gate_W"])
        expose("VZERO", rst_ref.ports["multiplier_0_source_W"])
        # Route NMOS body to VSS if needed, or assume it's global substrate

    top_level.info["netlist"] = _sample_hold_netlist(tg_comp, mim_comp, rst_comp)

    return top_level


if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk

    print("Generating sample_hold_cell...")
    comp = sample_hold_cell(
        gf180_mapped_pdk, switch_multipliers=(2, 2), cap_size=(4, 4)
    )
    comp.write_gds("sample_hold_cell.gds")
    print(f"Generated GDS successfully: {comp.name}")
