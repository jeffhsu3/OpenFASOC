"""differential_delay_stage — one CM-preserving tap of the analog Conv1D delay line.

Architecture validated in ngspice (analog_ref/diff_delayline_tb.spice): a SINGLE sample-hold
per rail + a `diff_buffer` (recycled-opamp unity-follower pair). Stages alternate the clock
phase (even=PHI1, odd=PHI2) so each buffer's LOW-Z output ACTIVELY DRIVES the next stage's
sampling switch -- this is what preserves the value (no passive master-slave charge sharing)
and the common mode (the source-follower buffer collapsed it). 4-stage line holds diff =
0.2000V exact + CM = 1.649V at every tap.

  VINP --[S/H_p (CLK)]-- HOLDP --\
                                  diff_buffer --> VOUTP / VOUTN
  VINN --[S/H_n (CLK)]-- HOLDN --/

Ports: VINP VINN VOUTP VOUTN CLK CLK_B VDP_BIAS VCS_BIAS VDD VSS VCC.
(VCC = S/H transmission-gate well supply; VDD = opamp supply; both 3.3V, tied at top level.)
"""
import numpy as np
import gdsfactory as gf
from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.pdk.util.comp_utils import evaluate_bbox
from glayout.flow.pdk.util.snap_to_grid import component_snap_to_grid
from glayout.flow.routing.smart_route import smart_route
from glayout.flow.routing.straight_route import straight_route
from glayout.flow.primitives.via_gen import via_stack
from glayout.flow.spice.netlist import Netlist
from glayout.flow.blocks.composite.sample_and_hold.sample_hold_cell import sample_hold_cell
from glayout.flow.blocks.composite.diff_buffer.diff_buffer import diff_buffer


def _stage_netlist(sh: Component, buf: Component) -> Netlist:
    nodes = ["VINP", "VINN", "VOUTP", "VOUTN", "CLK", "CLK_B",
             "VDP_BIAS", "VCS_BIAS", "VDD", "VSS", "VCC"]
    net = Netlist(circuit_name="differential_delay_stage", nodes=nodes)
    # per-rail single S/H: VIN -> HOLD on CLK
    net.connect_netlist(sh.info["netlist"], [
        ("VIN", "VINP"), ("CLK", "CLK"), ("CLK_B", "CLK_B"),
        ("VOUT", "HOLDP"), ("VCC", "VCC"), ("VSS", "VSS")])
    net.connect_netlist(sh.info["netlist"], [
        ("VIN", "VINN"), ("CLK", "CLK"), ("CLK_B", "CLK_B"),
        ("VOUT", "HOLDN"), ("VCC", "VCC"), ("VSS", "VSS")])
    # diff_buffer follows the held nodes
    net.connect_netlist(buf.info["netlist"], [
        ("VINP", "HOLDP"), ("VINN", "HOLDN"), ("VOUTP", "VOUTP"), ("VOUTN", "VOUTN"),
        ("VDP_BIAS", "VDP_BIAS"), ("VCS_BIAS", "VCS_BIAS"), ("VDD", "VDD"), ("VSS", "VSS")])
    return net


@cell
def differential_delay_stage(pdk: MappedPDK, gap: float = 12.0) -> Component:
    pdk.activate()
    sh = sample_hold_cell(pdk, with_reset=False)
    buf = diff_buffer(pdk)
    top = Component()

    shp = top << sh          # p-rail S/H (upper-left)
    shn = top << sh          # n-rail S/H (lower-left)
    # Mirror the S/H cells horizontally: their VIN/CLK/CLK_B ports are native EAST (the cell
    # is cap-West / TG-East), which points INTO the macro when the S/H sits at the left and
    # buries the pins. Flipping puts VIN/CLK/CLK_B on the WEST -> at the left macro edge
    # (router-accessible), and bonus: VOUT now faces EAST toward the buffer (shorter routes).
    shp.mirror_x()
    shn.mirror_x()
    bref = top << buf        # buffer (right)
    hsh = evaluate_bbox(sh)[1]
    shp.movey(hsh / 2 + gap / 2)
    shn.movey(-(hsh / 2 + gap / 2))
    bref.movex(max(shp.xmax, shn.xmax) - bref.xmin)

    routed, failed = [], []

    def route(net, ra, pa, rb, pb):
        try:
            top << smart_route(pdk, ra.ports[pa], rb.ports[pb], ra, rb)
            routed.append(net)
        except Exception as e:  # noqa: BLE001
            failed.append((net, type(e).__name__))

    # held S/H outputs -> buffer inputs. Route via VOUT_TAP (the met2 routable tap, same
    # net as VOUT) -- VOUT itself is the met5 cap plate and U-turns away from the buffer.
    route("HOLDP", shp, "VOUT_TAP", bref, "VINP")
    route("HOLDN", shn, "VOUT_TAP", bref, "VINN")
    # share clocks + supplies between the two S/H cells
    route("CLK", shp, "CLK", shn, "CLK")
    route("CLK_B", shp, "CLK_B", shn, "CLK_B")
    route("VSS_sh", shp, "VSS", shn, "VSS")
    route("VCC_sh", shp, "VCC", shn, "VCC")
    route("VSS_buf", shn, "VSS", bref, "VSS")

    # expose stage pins WITH GDS labels (so magic `port makeall` -> LEF PINs, and LVS
    # has named ports). Mirrors the coeff_cap/sample_hold_cell expose() pattern.
    def expose(name, port):
        top.add_port(name=name, port=port)
        top.add_label(text=name, position=(port.center[0], port.center[1]),
                      layer=pdk.get_glayer(pdk.layer_to_glayer(port.layer)))

    def expose_down(name, port):
        # met5 pin near an edge -> just via down to met2 in place and expose there.
        v = top << via_stack(pdk, pdk.layer_to_glayer(port.layer), "met2", centered=True)
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        top.add_port(name=name, port=v.ports["bottom_met_N"])
        top.add_label(text=name, position=(v.center[0], v.center[1]),
                      layer=pdk.get_glayer("met2"))

    def expose_up(name, port):
        # INTERIOR met5 opamp output -> the router can't reach a pin buried mid-macro.
        # Via down to met2 (free corridor: opamp upper region uses met3-5, not met2) and
        # route straight UP to above the buffer top, exposing the PIN at the (new) top edge.
        ytop = bref.ymax + 4
        v = top << via_stack(pdk, pdk.layer_to_glayer(port.layer), "met2", centered=True)
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        dest = gf.Port(name=name + "_top", center=(float(port.center[0]), float(ytop)),
                       width=float(v.ports["bottom_met_N"].width), orientation=270,
                       layer=v.ports["bottom_met_N"].layer)
        top << straight_route(pdk, v.ports["bottom_met_N"], dest, glayer1="met2", glayer2="met2")
        top.add_port(name=name, port=gf.Port(name=name, center=(float(port.center[0]), float(ytop)),
                     width=dest.width, orientation=90, layer=dest.layer))
        top.add_label(text=name, position=(float(port.center[0]), float(ytop)),
                      layer=pdk.get_glayer("met2"))

    # West landing-pad egress for the S/H pins. The S/H pins are bare ~0.5um met2 taps buried
    # in the cell's congested met2 clock/via row, so even at the left edge the router can't
    # land a via. Pop UP to met3 (free above that row), route WEST to a clean pad at the left
    # macro edge, and expose the PIN there. xedge sits just inside the S/H left edge.
    xedge = float(min(shp.xmin, shn.xmin)) + 0.5

    def expose_west(name, port):
        v = top << via_stack(pdk, pdk.layer_to_glayer(port.layer), "met3", centered=True)
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        m3 = v.ports["top_met_W"]
        dest = gf.Port(name=name + "_w", center=(xedge, float(port.center[1])),
                       width=float(m3.width), orientation=0, layer=m3.layer)
        top << straight_route(pdk, m3, dest, glayer1="met3", glayer2="met3")
        top.add_port(name=name, port=gf.Port(name=name, center=(xedge, float(port.center[1])),
                     width=float(m3.width), orientation=180, layer=m3.layer))
        top.add_label(text=name, position=(xedge, float(port.center[1])),
                      layer=pdk.get_glayer("met3"))

    expose_west("VINP", shp.ports["VIN"])          # S/H taps -> met3 pad at the left edge
    expose_west("VINN", shn.ports["VIN"])
    expose_west("CLK", shn.ports["CLK"])
    expose_west("CLK_B", shn.ports["CLK_B"])
    expose_west("VSS", shp.ports["VSS"])
    expose_west("VCC", shp.ports["VCC"])
    expose_up("VOUTP", bref.ports["VOUTP"])        # interior opamp output -> route up to top edge
    expose_up("VOUTN", bref.ports["VOUTN"])
    expose_up("VCS_BIAS", bref.ports["VCS_BIAS"])  # interior bias pin -> route up to top edge
    expose_up("VDD", bref.ports["VDD"])            # interior supply -> route up to top edge
    expose("VDP_BIAS", bref.ports["VDP_BIAS"])     # already at the bottom edge

    comp = component_snap_to_grid(top)
    comp.info["netlist"] = _stage_netlist(sh, buf)
    comp.info["routed"] = routed
    comp.info["route_failed"] = failed
    return comp


if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    c = differential_delay_stage(gf180_mapped_pdk)
    c.write_gds("differential_delay_stage.gds")
    print("GDS:", c.name, "bbox=", c.bbox)
    print("routed:", c.info.get("routed"), "\nFAILED:", c.info.get("route_failed"))
