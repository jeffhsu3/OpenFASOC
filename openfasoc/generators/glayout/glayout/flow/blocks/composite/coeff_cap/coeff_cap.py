"""coeff_cap — one fixed-coefficient tap of a passive switched-cap charge-domain FIR.

Topology (a near-clone of sample_hold_cell): a transmission_gate samples the
(already buffered/held) delay-line tap onto a weighting MIM-cap array whose value
encodes the coefficient MAGNITUDE (|coeff| parallel unit caps), and a reset NMOS
dumps the sample-node plate to VZERO=Vcm. The cap's OTHER plate is exposed as a
routable SUM tap that fir_mac ties to a differential summing rail; the coefficient
SIGN selects which rail (SUM_P/SUM_N) at the fir_mac level, so this leaf is
rail-agnostic and one body is reused for every tap of the same magnitude.

  - TOP plate (met5, lower substrate parasitic) = SAMPLE node: switched by the TG
    to the tap during phase A and dumped to VZERO by the reset NMOS during phase B.
  - BOTTOM plate (met4) = SUM: routed out to a met2 tap for the rail.

Coefficient -> layout: magnitude n=|coeff| -> mimcap_array(rows=1, columns=n); each
unit is the DRC-min 5x5um=25um^2=50fF MIM (gf180 MIM.8a). Sign is handled by fir_mac.
"""
import numpy as np

from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.blocks.elementary.transmission_gate.transmission_gate import transmission_gate
from glayout.flow.primitives.fet import nmos
from glayout.flow.primitives.mimcap import mimcap_array
from glayout.flow.primitives.via_gen import via_stack
from glayout.flow.routing.L_route import L_route
from glayout.flow.routing.straight_route import straight_route
from glayout.flow.pdk.util.comp_utils import evaluate_bbox
from glayout.flow.spice.netlist import Netlist


def _coeff_cap_netlist(
    tg_comp: Component,
    mim_comp: Component,
    rst_comp: Component | None = None,
) -> Netlist:
    """Source netlist for one FIR coefficient tap.

    Identical to the sample-and-hold netlist except the cap bottom plate (V2) goes
    to the SUM rail instead of VSS (this single change makes it a summing tap).
    """
    nodes = ["VIN", "CLK", "CLK_B", "VSS", "SAMPLE", "SUM", "VCC"]
    if rst_comp is not None:
        nodes.extend(["RESET", "VZERO"])

    netlist = Netlist(circuit_name="coeff_cap", nodes=nodes)

    netlist.connect_netlist(
        tg_comp.info["netlist"],
        [
            ("VIN", "VIN"),
            ("VSS", "VSS"),
            ("VOUT", "SAMPLE"),
            ("VCC", "VCC"),
            ("VGP", "CLK_B"),
            ("VGN", "CLK"),
        ],
    )

    netlist.connect_netlist(
        mim_comp.info["netlist"],
        [
            ("V1", "SAMPLE"),  # top plate = per-tap floating sample node
            ("V2", "SUM"),     # bottom plate = shared summing rail
        ],
    )

    if rst_comp is not None:
        netlist.connect_netlist(
            rst_comp.info["netlist"],
            [
                ("D", "SAMPLE"),  # dump the sample plate to VZERO in phase B
                ("G", "RESET"),
                ("S", "VZERO"),
                ("B", "VSS"),
            ],
        )

    return netlist


@cell
def coeff_cap(
    pdk: MappedPDK,
    coefficient: int = 1,
    unit_cap_size: tuple[float, float] = (5.0, 5.0),
    switch_width: tuple[float, float] = (1.0, 1.0),
    switch_fingers: tuple[int, int] = (2, 2),
    switch_multipliers: tuple[int, int] = (1, 1),
    with_reset: bool = True,
    reset_fingers: int = 1,
) -> Component:
    """One fixed-coefficient FIR tap (weight = |coefficient| unit MIM caps)."""
    n = abs(int(coefficient))
    if n < 1:
        raise ValueError("coefficient magnitude must be >= 1 (a zero tap is omitted at fir_mac level)")

    top_level = Component(name=f"coeff_cap_{coefficient}")
    SEP_MULT = 1

    tg_comp = transmission_gate(
        pdk=pdk,
        width=switch_width,
        fingers=switch_fingers,
        multipliers=switch_multipliers,
        inter_finger_topmet="met1",
    )
    tg_ref = top_level << tg_comp

    # Weight cap: n unit caps in parallel (mimcap_array already shorts all top
    # plates into one net and all bottom plates into one net -> single 2-terminal
    # cap of n*unit). One array per tap; never mix coefficients in one array.
    mim_comp = mimcap_array(pdk=pdk, rows=1, columns=n, size=unit_cap_size)
    mim_ref = top_level << mim_comp
    mim_ref.movex(tg_ref.xmin - evaluate_bbox(mim_ref)[0] - pdk.util_max_metal_seperation() * SEP_MULT)
    mim_ref.movey(tg_ref.center[1] - mim_ref.center[1])

    rst_comp = None
    rst_ref = None
    if with_reset:
        if reset_fingers > 1:
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
        rst_ref.movex(mim_ref.xmin - evaluate_bbox(rst_ref)[0])
        rst_ref.movey(mim_ref.center[1] - rst_ref.center[1])

    def drop_via(layer1, layer2, port):
        v = via_stack(pdk, layer1, layer2, fulltop=True, fullbottom=True)
        v_ref = top_level << v
        v_ref.movex(port.center[0] - v_ref.center[0])
        v_ref.movey(port.center[1] - v_ref.center[1])
        return v_ref

    # Plate glayers derived from the cap (gf180: top=met5, bottom=met4).
    mim_top_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_top_met_E"].layer)
    mim_bottom_layer = pdk.layer_to_glayer(mim_ref.ports["row0_col0_bottom_met_E"].layer)
    ROUTING_SEP = 1.1

    # ----- TOP plate (V1 = SAMPLE node) -- proven sample_hold staircase --------
    # Reset NMOS drain -> top plate (dump-to-VZERO switch).
    if with_reset:
        rst_drain_port = rst_ref.ports["multiplier_0_drain_E"].copy()
        rst_drain_port.center = (rst_drain_port.center[0] - 0.1, rst_drain_port.center[1])
        rst_via1 = drop_via("met2", "met3", rst_drain_port)
        rst_port = rst_via1.ports["top_met_E"].copy()
        rst_port.center = (rst_port.center[0] + ROUTING_SEP, rst_port.center[1])
        top_level << straight_route(pdk, rst_via1.ports["top_met_E"], rst_port, glayer1="met3", glayer2="met3")
        rst_via = drop_via("met3", mim_top_layer, rst_port)
        top_level << L_route(
            pdk, rst_via.ports["top_met_N"], mim_ref.ports["row0_col0_top_met_W"],
            hglayer=mim_top_layer, vglayer=mim_top_layer, vwidth=1.0, hwidth=1.0,
        )

    # TG drain (sampling switch output) -> top plate.
    tg_vout_port = tg_ref.ports["P_multiplier_0_drain_W"].copy()
    tg_vout_port.center = (tg_vout_port.center[0] + 0.1, tg_vout_port.center[1])
    tg_vout_via1 = drop_via("met2", "met3", tg_vout_port)
    tg_vout_port2 = tg_vout_via1.ports["top_met_W"].copy()
    tg_vout_port2.center = (tg_vout_port2.center[0] - 1.1, tg_vout_port2.center[1])
    top_level << straight_route(pdk, tg_vout_via1.ports["top_met_W"], tg_vout_port2, glayer1="met3", glayer2="met3")
    tg_vout_via = drop_via("met3", mim_top_layer, tg_vout_port2)
    top_level << L_route(
        pdk, tg_vout_via.ports["top_met_N"], mim_ref.ports["row0_col0_top_met_E"],
        hglayer=mim_top_layer, vglayer=mim_top_layer, vwidth=1.0, hwidth=1.0,
    )

    # ----- BOTTOM plate (V2 = SUM rail tap) ------------------------------------
    # Unlike sample_hold (which ties this plate to VSS), expose it as a routable
    # met2 SUM tap. Drop a met2->met4 via at the cap right edge and L_route to the
    # bottom-plate SE corner; the via's met2 side is the SUM tap (fir_mac routes
    # the differential rail here -- you cannot via through the MIM).
    sum_via_dst = mim_ref.ports["row0_col0_bottom_met_E"].copy()
    sum_via_dst.center = (mim_ref.xmax, tg_ref.ports["N_tie_S_top_met_W"].center[1])
    sum_via = drop_via("met2", mim_bottom_layer, sum_via_dst)
    se_port = mim_ref.ports["row0_col0_bottom_met_E"].copy()
    se_port.center = (mim_ref.xmax, mim_ref.ymin)
    top_level << L_route(
        pdk, sum_via.ports["top_met_S"], se_port,
        hglayer=mim_bottom_layer, vglayer=mim_bottom_layer,
    )

    # ----- Expose ports (labels nudged inside the polygon for magic) -----------
    def expose(name: str, port, glayer: str | None = None):
        top_level.add_port(name=name, port=port)
        if glayer is None:
            glayer = pdk.layer_to_glayer(port.layer)
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
    expose("SAMPLE", mim_ref.ports["row0_col0_top_met_W"])  # label/probe (floating sample node)
    expose("SUM", sum_via.ports["bottom_met_S"])            # ROUTABLE met2 rail tap

    if with_reset:
        expose("RESET", rst_ref.ports["multiplier_0_gate_W"])
        expose("VZERO", rst_ref.ports["multiplier_0_source_W"])

    top_level.info["netlist"] = _coeff_cap_netlist(tg_comp, mim_comp, rst_comp)
    return top_level


if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    import sys
    coeff = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"Generating coeff_cap(coefficient={coeff})...")
    comp = coeff_cap(gf180_mapped_pdk, coefficient=coeff)
    comp.write_gds("coeff_cap.gds")
    print(f"Generated GDS: {comp.name}")
