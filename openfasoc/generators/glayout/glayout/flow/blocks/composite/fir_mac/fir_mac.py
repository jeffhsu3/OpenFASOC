"""fir_mac — passive switched-cap charge-domain differential FIR multiply-accumulate.

N coeff_cap taps share a differential summing-rail pair (SUM_P / SUM_N). The fixed
signed integer coefficient of tap i selects the rail (h_i>0 -> SUM_P, h_i<0 -> SUM_N)
and the cap magnitude (|h_i| unit caps). Charge sharing on the floating rails in
phase B forms the weighted sum; the differential output (read BUF_OUT_N - BUF_OUT_P
to undo the redistribution inversion) is proportional to sum_i h_i (V_i - Vcm) when
the two rails carry equal total capacitance (rail balancing, enforced with ballast
dummy caps).

See memory `fir-charge-domain-scheme.md` for the locked electrical contract:
  - V_SUM = Vcm - sum_k C_k (V_k - Vcm)/C_tot   (the minus is real, inversion)
  - V_OUT = BUF_OUT_N - BUF_OUT_P  (cancels the inversion)
  - balance C_tot_P == C_tot_N via ballast (this version pads to equal unit-cap count)
  - VZERO == Vcm one shared node; CLK drives TG + rail resets (A), RESET the per-tap
    resets (B); non-overlap dead-time required (TG/rail-reset open before per-tap close)

Plate orientation here is the v1 (sample_hold) convention: per-tap SAMPLE = cap TOP
plate (met5), SUM = cap BOTTOM plate (met4, routed out as a met2 tap). Rails run on
met3, clear of the met4/met5 cap plates. (v2 refinement: swap to top-plate rails for
lower absolute attenuation -- calibrated out downstream, so deferred.)
"""
from typing import Sequence

from gdsfactory.cell import cell
from gdsfactory.component import Component
from gdsfactory.components import rectangle

from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.blocks.composite.coeff_cap.coeff_cap import coeff_cap
from glayout.flow.primitives.fet import nmos
from glayout.flow.primitives.mimcap import mimcap_array
from glayout.flow.primitives.via_gen import via_array
from glayout.flow.routing.straight_route import straight_route
from glayout.flow.pdk.util.comp_utils import evaluate_bbox
from glayout.flow.spice.netlist import Netlist


def _reset_nmos(pdk: MappedPDK) -> Component:
    """Per-rail reset switch (also the per-coeff_cap dump device's twin)."""
    return nmos(
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


def _fir_mac_netlist(
    cc_comps: list[tuple[int, int, Component]],
    rst_p: Component,
    rst_n: Component,
    ballast: Component | None,
    ballast_rail: str | None,
) -> Netlist:
    """Source netlist for the differential charge-domain MAC core."""
    tap_nodes = [f"TAP{i}" for (i, _h, _c) in cc_comps]
    sample_nodes = [f"SAMPLE{i}" for (i, _h, _c) in cc_comps]
    nodes = (
        tap_nodes
        + ["CLK", "CLK_B", "RESET", "VZERO", "VCC", "VSS", "SUM_P", "SUM_N"]
        + sample_nodes
    )
    nl = Netlist(circuit_name="fir_mac", nodes=nodes)

    for (i, h, cc) in cc_comps:
        rail = "SUM_P" if h > 0 else "SUM_N"
        nl.connect_netlist(
            cc.info["netlist"],
            [
                ("VIN", f"TAP{i}"),
                ("CLK", "CLK"),
                ("CLK_B", "CLK_B"),
                ("VSS", "VSS"),
                ("VCC", "VCC"),
                ("SAMPLE", f"SAMPLE{i}"),
                ("SUM", rail),
                ("RESET", "RESET"),
                ("VZERO", "VZERO"),
            ],
        )

    # Per-rail reset NMOS: clamp rail to VZERO=Vcm during phase A (gate = CLK).
    nl.connect_netlist(rst_p.info["netlist"], [("D", "SUM_P"), ("G", "CLK"), ("S", "VZERO"), ("B", "VSS")])
    nl.connect_netlist(rst_n.info["netlist"], [("D", "SUM_N"), ("G", "CLK"), ("S", "VZERO"), ("B", "VSS")])

    # Ballast: cap SUM-side (V2, bottom plate) -> lighter rail, other plate -> Vcm.
    if ballast is not None and ballast_rail is not None:
        nl.connect_netlist(ballast.info["netlist"], [("V1", "VZERO"), ("V2", ballast_rail)])

    return nl


@cell
def fir_mac(
    pdk: MappedPDK,
    coefficients: Sequence[int] = (1, -1, 1, -1),
    unit_cap_size: tuple[float, float] = (5.0, 5.0),
    tap_gap: float | None = None,
    rail_width: float = 1.0,
) -> Component:
    """Differential charge-domain FIR MAC for fixed integer `coefficients`."""
    coeffs = list(coefficients)
    nz = [(i, int(h)) for i, h in enumerate(coeffs) if int(h) != 0]
    if not nz:
        raise ValueError("need at least one non-zero coefficient")

    top = Component(name="fir_mac")
    SEP = pdk.util_max_metal_seperation()
    gap = tap_gap if tap_gap is not None else 4.0 * SEP

    # ---- place coeff_caps left-to-right, bottom-aligned ----------------------
    cc_comps: list[tuple[int, int, Component]] = []
    cc_refs = []
    x = 0.0
    for (i, h) in nz:
        cc = coeff_cap(pdk, coefficient=h, unit_cap_size=unit_cap_size)
        ref = top << cc
        ref.movex(x - ref.xmin)
        ref.movey(-ref.ymin)  # bottom edge at y=0
        cc_comps.append((i, h, cc))
        cc_refs.append((i, h, ref))
        x += evaluate_bbox(ref)[0] + gap
    row_xmax = max(r.xmax for (_i, _h, r) in cc_refs)
    row_xmin = min(r.xmin for (_i, _h, r) in cc_refs)

    # ---- two differential rails on met3, below the row ----------------------
    # SUM_P nearer the row, SUM_N below it. h>0 taps drop met2->met3 onto SUM_P;
    # h<0 taps run met2 PAST SUM_P (different layer, no short) down to SUM_N.
    y_p = -3.0
    y_n = -7.0
    rail_len = (row_xmax - row_xmin) + 4.0
    rail_cx = (row_xmin + row_xmax) / 2.0

    def add_rail(y):
        r = top << rectangle(size=(rail_len, rail_width), layer=pdk.get_glayer("met3"), centered=True)
        r.move((rail_cx, y))
        return r

    rail_p = add_rail(y_p)
    rail_n = add_rail(y_n)

    # ---- connect each coeff_cap SUM tap (met2, south-facing) to its rail -----
    for (i, h, ref) in cc_refs:
        sum_port = ref.ports["SUM"]
        sx = sum_port.center[0]
        ry = y_p if h > 0 else y_n
        # met2 stub straight down from the tap to the rail y
        dst = sum_port.copy()
        dst.center = (sx, ry)
        top << straight_route(pdk, sum_port, dst, glayer1="met2", glayer2="met2")
        # via up to the met3 rail at the landing point
        v = via_array(pdk, "met2", "met3", size=(rail_width, rail_width), fullbottom=True, no_exception=True)
        v_ref = top << v
        v_ref.movex(sx - v_ref.center[0])
        v_ref.movey(ry - v_ref.center[1])

    # ---- per-rail reset NMOS (matched), placed below the rails ---------------
    rst_p = _reset_nmos(pdk)
    rst_n = _reset_nmos(pdk)
    rst_p_ref = top << rst_p
    rst_n_ref = top << rst_n
    rst_p_ref.movex(row_xmin - rst_p_ref.xmin)
    rst_p_ref.movey(y_n - evaluate_bbox(rst_p_ref)[1] - SEP - rst_p_ref.ymin)
    rst_n_ref.movex(rst_p_ref.xmax + SEP - rst_n_ref.xmin)
    rst_n_ref.movey(rst_p_ref.ymin - rst_n_ref.ymin)
    # drain (met2) -> rail via: short met2 then via to met3 rail
    for (rst_ref, ry) in ((rst_p_ref, y_p), (rst_n_ref, y_n)):
        d = rst_ref.ports["multiplier_0_drain_E"]
        dst = d.copy()
        dst.center = (d.center[0], ry)
        top << straight_route(pdk, d, dst, glayer1="met2", glayer2="met2")
        v = via_array(pdk, "met2", "met3", size=(rail_width, rail_width), fullbottom=True, no_exception=True)
        v_ref = top << v
        v_ref.movex(d.center[0] - v_ref.center[0])
        v_ref.movey(ry - v_ref.center[1])

    # ---- rail balancing: ballast the lighter rail to equal unit-cap count ----
    m_p = sum(abs(h) for (_i, h) in nz if h > 0)
    m_n = sum(abs(h) for (_i, h) in nz if h < 0)
    ballast = None
    ballast_rail = None
    if m_p != m_n:
        nb = abs(m_p - m_n)
        ballast_rail = "SUM_P" if m_p < m_n else "SUM_N"
        ballast = mimcap_array(pdk, rows=1, columns=nb, size=unit_cap_size)
        bref = top << ballast
        bref.movex(row_xmax + gap - bref.xmin)
        bref.movey(-bref.ymin)
        # bottom plate (V2) -> rail ; top plate (V1) -> VZERO handled at outer level.
        # (routing of the ballast plates is added in the DRC-tuning pass.)

    # ---- expose ports + netlist ---------------------------------------------
    def expose(name, port, glayer=None):
        top.add_port(name=name, port=port)
        if glayer is None:
            glayer = pdk.layer_to_glayer(port.layer)
        top.add_label(text=name, position=port.center, layer=pdk.get_glayer(glayer))

    for (i, h, ref) in cc_refs:
        expose(f"TAP{i}", ref.ports["VIN"])
    expose("SUM_P", rail_p.ports["e3"] if "e3" in rail_p.ports else rail_p.ports[list(rail_p.ports)[0]], glayer="met3")
    expose("SUM_N", rail_n.ports[list(rail_n.ports)[0]], glayer="met3")
    expose("RESET", cc_refs[0][2].ports["RESET"])
    expose("VZERO", cc_refs[0][2].ports["VZERO"])
    expose("CLK", cc_refs[0][2].ports["CLK"])
    expose("CLK_B", cc_refs[0][2].ports["CLK_B"])
    expose("VCC", cc_refs[0][2].ports["VCC"])
    expose("VSS", cc_refs[0][2].ports["VSS"])

    top.info["netlist"] = _fir_mac_netlist(cc_comps, rst_p, rst_n, ballast, ballast_rail)
    return top


if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    import sys
    coeffs = [int(a) for a in sys.argv[1:]] or [1, -1, 2, -1]
    print(f"Generating fir_mac(coefficients={coeffs})...")
    comp = fir_mac(gf180_mapped_pdk, coefficients=coeffs)
    comp.write_gds("fir_mac.gds")
    print(f"Generated GDS: {comp.name}")
