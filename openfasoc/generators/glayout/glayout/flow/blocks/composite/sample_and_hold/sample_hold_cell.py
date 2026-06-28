import sys

from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.blocks.elementary.transmission_gate.transmission_gate import transmission_gate
from glayout.flow.primitives.fet import nmos
from glayout.flow.primitives.mimcap import mimcap_array
from glayout.flow.routing.L_route import L_route
from glayout.flow.pdk.util.comp_utils import evaluate_bbox
from glayout.flow.spice.netlist import Netlist

def _sample_hold_netlist(tg_comp: Component, mim_comp: Component, rst_comp: Component | None = None) -> Netlist:
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
    cap_cols: int =  1,
    with_reset: bool = True,
    reset_fingers: int = 1,
) -> Component:
    """
    A reusable Sample-and-Hold (S&H) storage cell.
    Samples VIN to VOUT on CLK/CLK_B.
    Optionally resets VOUT to VZERO when RESET is high.
    """
    top_level = Component(name="sample_hold_cell")
    SEP_MULT = 1
    
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
    
    mim_ref.movex(tg_ref.xmin - evaluate_bbox(mim_ref)[0] - pdk.util_max_metal_seperation() * SEP_MULT)
    mim_ref.movey(tg_ref.center[1] - mim_ref.center[1])
    
    rst_comp = None
    rst_ref = None
    if with_reset:
        if reset_fingers > 1:
            # Need to handle routing
            # τ = Ron_reset × C_hold
            # Probably need to this to be automatic and also be scaled
            # depending on the PDK
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
        # Place to the left of the MIM cap
        # rst_ref.movex(mim_ref.xmin - evaluate_bbox(rst_ref)[0] - pdk.util_max_metal_seperation() * SEP_MULT)
        rst_ref.movex(mim_ref.xmin - evaluate_bbox(rst_ref)[0])
        rst_ref.movey(mim_ref.center[1] - rst_ref.center[1])
    
    # Custom Routing
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
    mim_bottom_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_bottom_met_E"].layer)
    ROUTING_SEP = 1.1

    # Top plate (V1 = VOUT) = cap top plate:
    if with_reset:
        # Route rst_port right by 1.0um on met3 to avoid crossing dummy routes on met2
        rst_drain_port = rst_ref.ports["multiplier_0_drain_E"].copy()
        rst_drain_port.center = (rst_drain_port.center[0] - 0.1, rst_drain_port.center[1])
        rst_via1 = drop_via("met2", "met3", rst_drain_port)
        
        rst_port = rst_via1.ports["top_met_E"].copy()
        rst_port.center = (rst_port.center[0] + ROUTING_SEP, rst_port.center[1])
        top_level << straight_route(pdk, rst_via1.ports["top_met_E"], rst_port, glayer1="met3", glayer2="met3")
        rst_via = drop_via("met3", mim_top_layer, rst_port)
        top_level << L_route(
            pdk,
            rst_via.ports["top_met_N"],
            mim_ref.ports["row0_col0_top_met_W"],
            hglayer=mim_top_layer,
            vglayer=mim_top_layer,
            vwidth=1.0,
            hwidth=1.0,
        )

    # TG VOUT via (M2 -> cap top plate). Outside the reset block so VOUT always
    # connects to the hold cap, with or without the reset switch.
    # TG VOUT via (M2 -> M3 -> cap top plate).
    tg_vout_port = tg_ref.ports["P_multiplier_0_drain_W"].copy()
    tg_vout_port.center = (tg_vout_port.center[0] + 0.1, tg_vout_port.center[1])
    tg_vout_via1 = drop_via("met2", "met3", tg_vout_port)
    
    tg_vout_port2 = tg_vout_via1.ports["top_met_W"].copy()
    tg_vout_port2.center = (tg_vout_port2.center[0] - 1.1, tg_vout_port2.center[1])
    top_level << straight_route(pdk, tg_vout_via1.ports["top_met_W"], tg_vout_port2, glayer1="met3", glayer2="met3")
    tg_vout_via = drop_via("met3", mim_top_layer, tg_vout_port2)
    top_level << L_route(
        pdk,
        tg_vout_via.ports["top_met_N"],
        mim_ref.ports["row0_col0_top_met_E"],
        hglayer=mim_top_layer,
        vglayer=mim_top_layer,
        vwidth=1.0,
        hwidth=1.0,
    )
    
    # Bottom plate (V2 = VSS): use the NMOS tie NORTH bar. Its y (~+2.475) falls
    # within the MIM cap bottom-plate y-extent even for the smallest (5x5) cap, so a
    # straight met2 run west lands a single met2->met4 via directly on the bottom
    # plate -- no L_route needed. (The south tie at y~-2.475 sits below the 5x5
    # plate, which is why it previously needed an L_route.)
    #
    # Reference the bottom-plate's own east port (the met4 edge) rather than the
    # component bbox, and inset the via west so its land sits fully ON the plate
    # instead of overhanging the met4 edge. Note: the met4 bottom plate legitimately
    # extends ~0.6um (the met4:capmet enclosure) beyond the visible top-plate cap,
    # so the via correctly lands on that enclosure ring.
    #
    # Derive the inset from the PDK, not a magic number: half the actual via-land
    # width keeps the land's east edge inside the plate edge, plus one bottom-metal
    # min_separation so the plate cleanly encloses the land. This matters across
    # PDKs -- the via land is 0.5um on gf180 but 1.5um on sky130, so a fixed inset
    # would leave the via overhanging on sky130.
    plate_e = mim_ref.ports["row0_col0_bottom_met_E"]
    vss_via_land = via_stack(pdk, "met2", mim_bottom_layer, fulltop=True, fullbottom=True)
    via_inset = evaluate_bbox(vss_via_land)[0] / 2.0 + pdk.get_grule(mim_bottom_layer)["min_separation"]
    tie_w = tg_ref.ports["N_tie_N_top_met_W"]
    vss_met2_dst = tie_w.copy()
    vss_met2_dst.center = (plate_e.center[0] - via_inset, tie_w.center[1])
    top_level << straight_route(pdk, tie_w, vss_met2_dst, glayer1="met2", glayer2="met2")
    drop_via("met2", mim_bottom_layer, vss_met2_dst)
    
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
    # Routable met2 tap on the VOUT net, OFF the MIM cap (the TG-drain side of the
    # hold node). Downstream assembly must connect here: you cannot drop a via
    # through a MIM cap, so the met5 VOUT port above is for labeling/probing, not
    # for routing into the next stage.
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
    comp = sample_hold_cell(gf180_mapped_pdk, cap_size=(20, 10))
    comp.write_gds("sample_hold_cell.gds")
    print(f"Generated GDS successfully: {comp.name}")
