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
    with_reset: bool = True
) -> Component:
    """
    A reusable Sample-and-Hold (S&H) storage cell.
    Samples VIN to VOUT on CLK/CLK_B.
    Optionally resets VOUT to VZERO when RESET is high.
    """
    top_level = Component(name="sample_hold_cell")
    
    # 1. Transmission Gate
    tg_comp = transmission_gate(
        pdk=pdk,
        width=switch_width,
        fingers=switch_fingers,
        multipliers=switch_multipliers,
        inter_finger_topmet="met1",
    )
    tg_ref = top_level << tg_comp
    
    # 2. MIM Capacitor
    mim_comp = mimcap_array(
        pdk=pdk,
        rows=cap_rows,
        columns=cap_cols,
        size=cap_size,
    )
    mim_ref = top_level << mim_comp
    
    # Place MIM cap to the left of TG
    # OPTIMIZE placement here
    mim_ref.movex(tg_ref.xmin - evaluate_bbox(mim_ref)[0] - pdk.util_max_metal_seperation() * 4)
    mim_ref.movey(tg_ref.center[1] - mim_ref.center[1])
    
    # 3. Optional Reset Switch (NMOS)
    rst_comp = None
    rst_ref = None
    if with_reset:
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
            # NOTE: met1 SD routing crashes glayout nmos (KeyError bottom_met_N); the
            # met2-rail M2.2a needs a primitive-level fix, not a config flag.
            sd_route_topmet="met2",
            gate_route_topmet="met2",
        )
        rst_ref = top_level << rst_comp
        # Place to the left of the MIM cap
        rst_ref.movex(mim_ref.xmin - evaluate_bbox(rst_ref)[0] - pdk.util_max_metal_seperation() * 4)
        rst_ref.movey(mim_ref.center[1] - rst_ref.center[1])
    
    # Custom Routing
    from glayout.flow.primitives.via_gen import via_stack

    def drop_via(layer1, layer2, port):
        v = via_stack(pdk, layer1, layer2, fulltop=True, fullbottom=True)
        v_ref = top_level << v
        v_ref.movex(port.center[0] - v_ref.center[0])
        v_ref.movey(port.center[1] - v_ref.center[1])
        return v_ref

    # Top plate (V1 = VOUT) = cap top plate (met5):
    if with_reset:
        # Route rst_port right by 1.0um before dropping via to avoid crossing North tie ring on met2
        rst_port = rst_ref.ports["multiplier_0_drain_E"].copy()
        rst_port.center = (rst_port.center[0] + 1.0, rst_port.center[1])
        from glayout.flow.routing.straight_route import straight_route
        top_level << straight_route(pdk, rst_ref.ports["multiplier_0_drain_E"], rst_port, glayer1="met2", glayer2="met2")
        rst_via = drop_via("met2", "met5", rst_port)
        top_level << L_route(
            pdk,
            rst_via.ports["top_met_N"],
            mim_ref.ports["row0_col0_top_met_W"],
            hglayer="met5",
            vglayer="met5",
            vwidth=1.0,
            hwidth=1.0,
        )

    # TG VOUT via (M2 -> M5)
    tg_vout_port = tg_ref.ports["P_multiplier_0_drain_W"].copy()
    tg_vout_port.center = (tg_vout_port.center[0] - 1.0, tg_vout_port.center[1])
    top_level << straight_route(pdk, tg_ref.ports["P_multiplier_0_drain_W"], tg_vout_port, glayer1="met2", glayer2="met2")
    tg_vout_via = drop_via("met2", "met5", tg_vout_port)
    top_level << L_route(
        pdk,
        tg_vout_via.ports["top_met_N"],
        mim_ref.ports["row0_col0_top_met_E"],
        hglayer="met5",
        vglayer="met5",
        vwidth=1.0,
        hwidth=1.0,
    )

    # Bottom plate (V2 = VSS) = cap bottom plate (met4):
    # TG VSS via (M2 -> M4)
    tg_vss_port = tg_ref.ports["N_tie_S_top_met_N"].copy()
    tg_vss_port.center = (tg_vss_port.center[0], tg_vss_port.center[1] - 1.0)
    top_level << straight_route(pdk, tg_ref.ports["N_tie_S_top_met_N"], tg_vss_port, glayer1="met2", glayer2="met2")
    tg_vss_via = drop_via("met2", "met4", tg_vss_port)
    top_level << L_route(
        pdk,
        tg_vss_via.ports["top_met_S"],
        mim_ref.ports["row0_col0_bottom_met_E"],
        hglayer="met4",
        vglayer="met4",
        vwidth=1.0,
        hwidth=1.0,
    )
    
    # Expose Ports
    def expose(name: str, port, glayer: str):
        top_level.add_port(name=name, port=port)
        top_level.add_label(text=name, position=port.center, layer=pdk.get_glayer(glayer))

    expose("VIN", tg_ref.ports["P_multiplier_0_source_E"], "met2")
    expose("CLK", tg_ref.ports["N_multiplier_0_gate_E"], "met2")
    # Move CLK_B via right by 1.0um to avoid met3 spacing with VOUT via
    clk_b_port = tg_ref.ports["P_multiplier_0_gate_E"].copy()
    clk_b_port.center = (clk_b_port.center[0] + 1.0, clk_b_port.center[1])
    from glayout.flow.routing.straight_route import straight_route
    top_level << straight_route(pdk, tg_ref.ports["P_multiplier_0_gate_E"], clk_b_port, glayer1="met2", glayer2="met2")
    clk_b_via = drop_via("met2", "met3", clk_b_port)
    expose("CLK_B", clk_b_via.ports["top_met_N"], "met3")
    expose("VCC", tg_ref.ports["P_tie_S_top_met_S"], "met2")
    expose("VSS", tg_ref.ports["N_tie_S_top_met_N"], "met2")
    expose("VOUT", mim_ref.ports["row0_col0_top_met_W"], "met5")
    
    if with_reset:
        expose("RESET", rst_ref.ports["multiplier_0_gate_W"], "met2")
        expose("VZERO", rst_ref.ports["multiplier_0_source_W"], "met2")
        # Route NMOS body to VSS if needed, or assume it's global substrate
    
    top_level.info["netlist"] = _sample_hold_netlist(tg_comp, mim_comp, rst_comp)
    
    return top_level

if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    print("Generating sample_hold_cell...")
    comp = sample_hold_cell(gf180_mapped_pdk)
    comp.write_gds("sample_hold_cell.gds")
    print(f"Generated GDS successfully: {comp.name}")
