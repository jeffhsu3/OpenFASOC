"""diff_buffer — pseudo-differential unity-gain buffer for the analog Conv1D delay line.

Two LOW-POWER OpenFASOC two-stage opamps (one per rail), each wired as a unity-gain
follower, buffer a differential tap while preserving BOTH the differential signal and the
common mode -- the fix for the delay-line CM droop (cascaded source-followers collapsed it).
Validated in ngspice (analog_ref/opamp_lp/casc.spice): diff preserved exactly, CM held at
1.649V across a 4-stage cascade, ~66uW/opamp (528uW for a 4-tap buffer). DRC-clean.

Each opamp: OPAMP_TWO_STAGE  VDD GND DIFFPAIR_BIAS VP VN CS_BIAS VOUT. Follower wiring (VP is
the INVERTING input, verified empirically): VP shorted to VOUT (feedback), signal into VN.
Output node = port `commonsource_output_E` (no pin_output when add_output_stage=False).
Bias pins are CURRENT inputs (diode-connected mirror refs); the two opamps SHARE VDP_BIAS /
VCS_BIAS (tied) for rail matching -- the top-level bias gen sources the total current.

Ports: VINP VINN VOUTP VOUTN VDP_BIAS VCS_BIAS VDD VSS.
"""
import re
from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.pdk.util.comp_utils import evaluate_bbox
from glayout.flow.pdk.util.snap_to_grid import component_snap_to_grid
from glayout.flow.routing.smart_route import smart_route
from glayout.flow.spice.netlist import Netlist
from glayout.flow.blocks.composite.opamp.opamp import opamp

# low-power sizing (matches analog_ref/opamp_lp/gen_opamp_lp.py): ~66uW/opamp.
# inter_finger_topmet="met1": keep the per-finger S/D via arrays on met1 so their met2
# patches don't sit ~0.25um from the met2 S/D rails (gf180 M2.2a needs 0.28) -- same
# opt-in as the crossbar/S&H cells (see 82b72286). Layout-only; LVS netlist unchanged.
# Verified: LP opamp gf180 magic DRC 106 -> 16 (M2.2a 186 -> 17); sky130 unchanged.
LP = dict(
    half_diffpair_params=(2, 1, 2), diffpair_bias=(2, 4, 2),
    half_common_source_params=(3, 1, 4, 2), half_common_source_bias=(2, 4, 2, 2),
    half_pload=(2, 1, 4), mim_cap_size=(6, 6), mim_cap_rows=1,
    rmult=1, with_antenna_diode_on_diffinputs=2, add_output_stage=False,
    inter_finger_topmet="met1",
)


def units_fix(netlist: str) -> str:
    """Append 'u' to bare numeric l=/w= (generator omits the unit -> ngspice reads meters)."""
    return "\n".join(re.sub(r'\b([lw])=(\d+\.?\d*)(?![u\d\.])', r'\1=\2u', ln)
                     for ln in netlist.splitlines())


def _diff_buffer_netlist(opa: Component) -> Netlist:
    nodes = ["VINP", "VINN", "VOUTP", "VOUTN", "VDP_BIAS", "VCS_BIAS", "VDD", "VSS"]
    net = Netlist(circuit_name="diff_buffer", nodes=nodes)
    # p-rail follower: VP=VOUTP (feedback), VN=VINP, VOUT=VOUTP
    net.connect_netlist(opa.info["netlist"], [
        ("VDD", "VDD"), ("GND", "VSS"), ("DIFFPAIR_BIAS", "VDP_BIAS"),
        ("VP", "VOUTP"), ("VN", "VINP"), ("CS_BIAS", "VCS_BIAS"), ("VOUT", "VOUTP")])
    # n-rail follower: VP=VOUTN (feedback), VN=VINN, VOUT=VOUTN
    net.connect_netlist(opa.info["netlist"], [
        ("VDD", "VDD"), ("GND", "VSS"), ("DIFFPAIR_BIAS", "VDP_BIAS"),
        ("VP", "VOUTN"), ("VN", "VINN"), ("CS_BIAS", "VCS_BIAS"), ("VOUT", "VOUTN")])
    return net


@cell
def diff_buffer(pdk: MappedPDK, gap: float = 10.0) -> Component:
    pdk.activate()
    opa = opamp(pdk, **LP)  # one body, placed twice (identical -> good rail matching)
    top = Component()
    pref = top << opa                          # p-rail (left)
    nref = top << opa                          # n-rail (right)
    nref.movex(pref.xmax + gap - nref.xmin)

    routed, failed = [], []

    def route(net, ra, pa, rb, pb):
        try:
            top << smart_route(pdk, ra.ports[pa], rb.ports[pb], ra, rb)
            routed.append(net)
        except Exception as e:  # noqa: BLE001
            failed.append((net, type(e).__name__))

    # per-opamp unity-follower feedback short: VP (pin_plus) -> VOUT (commonsource_output)
    route("FB_P", pref, "pin_plus_E", pref, "commonsource_output_E")
    route("FB_N", nref, "pin_plus_E", nref, "commonsource_output_E")
    # tie shared rails between the two opamps
    route("VDD", pref, "pin_vdd_E", nref, "pin_vdd_W")
    route("VSS", pref, "pin_gnd_E", nref, "pin_gnd_W")
    route("VDP_BIAS", pref, "pin_diffpairibias_E", nref, "pin_diffpairibias_W")
    route("VCS_BIAS", pref, "pin_commonsourceibias_E", nref, "pin_commonsourceibias_W")

    # expose clean-named buffer pins (composable + labelable for LVS)
    top.add_port(name="VINP", port=pref.ports["pin_minus_W"])
    top.add_port(name="VINN", port=nref.ports["pin_minus_E"])
    top.add_port(name="VOUTP", port=pref.ports["commonsource_output_E"])
    top.add_port(name="VOUTN", port=nref.ports["commonsource_output_E"])
    top.add_port(name="VDP_BIAS", port=pref.ports["pin_diffpairibias_W"])
    top.add_port(name="VCS_BIAS", port=pref.ports["pin_commonsourceibias_W"])
    top.add_port(name="VDD", port=pref.ports["pin_vdd_W"])
    top.add_port(name="VSS", port=pref.ports["pin_gnd_W"])

    comp = component_snap_to_grid(top)
    comp.info["netlist"] = _diff_buffer_netlist(opa)
    comp.info["routed"] = routed
    comp.info["route_failed"] = failed
    return comp


if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    c = diff_buffer(gf180_mapped_pdk)
    c.write_gds("diff_buffer.gds")
    open("diff_buffer.spice", "w").write(units_fix(c.info["netlist"].generate_netlist()))
    print("GDS:", c.name, "bbox=", c.bbox)
    print("routed:", c.info.get("routed"), "FAILED:", c.info.get("route_failed"))
