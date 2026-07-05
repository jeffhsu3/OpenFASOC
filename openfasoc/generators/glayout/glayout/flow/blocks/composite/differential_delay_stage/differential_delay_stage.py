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
    # Problematic, are VINN and VINP 
    route("HOLDN", shn, "VOUT_TAP", bref, "VINN")

    # --- clocks + supplies: EXPLICIT west-flank routing ------------------------
    # smart_route turned CLK/CLK_B into c_routes whose met3 trunks (x~4.5) crossed
    # the expose_west met3 pad runs (VSS y9.3, VCC y-3.1, ...) -> the old stage
    # merge blob. Instead: met2 columns west of both cells (layer-free vs the met3
    # runs), east->west VSS | CLK | CLK_B | VCC at 0.8 pitch. A stub only crosses
    # columns EAST of its own; the three unavoidable crossings dive under on MET1
    # (the west flank + inter-cell gap are empty on met1, and with met1 tie rings
    # nothing else met1-level leaves the cells).
    from gdsfactory.components.rectangle import rectangle as _rectangle
    from glayout.flow.primitives.via_gen import via_stack as _via_stack

    W2 = 0.5
    def _rect(x0, y0, x1, y1, glayer="met2"):
        _r = top << _rectangle(size=(round(abs(x1 - x0), 3), round(abs(y1 - y0), 3)),
                               layer=pdk.get_glayer(glayer), centered=True)
        _r.movex((x0 + x1) / 2 - _r.center[0]).movey((y0 + y1) / 2 - _r.center[1])
    def _viaat(x, y, l1, l2):
        _v = top << _via_stack(pdk, l1, l2)
        _v.movex(round(x, 3) - _v.center[0]).movey(round(y, 3) - _v.center[1])
    def _underpass(y, x_from_east, x_col, cols):
        # met2 (port..east via) | met1 under `cols` | met2 (west via..own column)
        xe = max(cols) + 0.78
        xw_ = min(cols) - 0.78
        _rect(xe - 0.1, y - W2/2, x_from_east, y + W2/2)
        _viaat(xe, y, "met1", "met2")
        _rect(xw_ - 0.1, y - W2/2, xe + 0.1, y + W2/2, "met1")
        _viaat(xw_, y, "met1", "met2")
        _rect(x_col - W2/2, y - W2/2, xw_ + 0.1, y + W2/2)

    xw = min(shp.xmin, shn.xmin)
    xVSS, xCLK, xCKB, xVCC = xw - 1.3, xw - 2.1, xw - 2.9, xw - 3.7

    # CLK column + stubs (top stub crosses nothing; bottom stub dives under VSS col)
    _pt, _pb = shp.ports["CLK"], shn.ports["CLK"]
    _rect(xCLK - W2/2, _pb.center[1] - W2/2, xCLK + W2/2, _pt.center[1] + W2/2)
    _rect(xCLK - W2/2, _pt.center[1] - W2/2, _pt.center[0] + 0.2, _pt.center[1] + W2/2)
    _underpass(_pb.center[1], _pb.center[0] + 0.2, xCLK, [xVSS])
    routed.append("CLK")
    # CLK_B column + stubs (bottom stub dives under CLK and VSS cols)
    _pt, _pb = shp.ports["CLK_B"], shn.ports["CLK_B"]
    _rect(xCKB - W2/2, _pb.center[1] - W2/2, xCKB + W2/2, _pt.center[1] + W2/2)
    _rect(xCKB - W2/2, _pt.center[1] - W2/2, _pt.center[0] + 0.2, _pt.center[1] + W2/2)
    _underpass(_pb.center[1], _pb.center[0] + 0.2, xCKB, [xCLK, xVSS])
    routed.append("CLK_B")
    # VCC: over the top of shp, met1 jog in the gap under all three east columns
    _pt, _pb = shp.ports["VCC"], shn.ports["VCC"]
    _ytop = shp.ymax + 1.1
    _yjog = shn.ymax + 0.55
    _rect(_pt.center[0] - W2/2, _pt.center[1] - 0.2, _pt.center[0] + W2/2, _ytop + W2/2)
    _rect(xVCC - W2/2, _ytop - W2/2, _pt.center[0] + W2/2, _ytop + W2/2)
    _rect(xVCC - W2/2, _yjog - W2/2, xVCC + W2/2, _ytop + W2/2)
    _viaat(xVCC, _yjog, "met1", "met2")
    _rect(xVCC - 0.23, _yjog - W2/2, _pb.center[0] + 0.23, _yjog + W2/2, "met1")
    _viaat(_pb.center[0], _yjog, "met1", "met2")
    _rect(_pb.center[0] - W2/2, _pb.center[1] - 0.2, _pb.center[0] + W2/2, _yjog + W2/2)
    routed.append("VCC_sh")
    # VSS tree: shp.VSS -> west column -> bottom rail -> riser to shn.VSS AND east
    # into the buffer's met4 gnd pin (explicit met2->met4 via on the pin rect).
    _pt, _pb = shp.ports["VSS"], shn.ports["VSS"]
    _bpin = bref.ports["VSS"]
    _yg = shp.ymin - 0.55
    _yrail = _bpin.center[1] + 0.6
    _rect(_pt.center[0] - W2/2, _yg - W2/2, _pt.center[0] + W2/2, _pt.center[1] + 0.2)
    _rect(xVSS - W2/2, _yg - W2/2, _pt.center[0] + W2/2, _yg + W2/2)
    _rect(xVSS - W2/2, _yrail - W2/2, xVSS + W2/2, _yg + W2/2)
    _rect(xVSS - W2/2, _yrail - W2/2, _bpin.center[0] + 1.5, _yrail + W2/2)
    _viaat(_bpin.center[0] + 1.0, _yrail, "met2", "met4")
    _rect(_pb.center[0] - W2/2, _yrail - W2/2, _pb.center[0] + W2/2, _pb.center[1] + 0.2)
    routed += ["VSS_sh", "VSS_buf"]

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

    expose_west("VINP", shp.ports["VIN"])
    expose_west("VINN", shn.ports["VIN"])

    # CLK/CLK_B/VSS/VCC pins sit directly ON the new west-flank met2 columns --
    # top-level geometry on the nets themselves, at the left macro edge.
    def expose_col(name, x, y):
        _p = gf.Port(name=name, center=(float(x), float(y)), width=W2,
                     orientation=180, layer=pdk.get_glayer("met2"))
        top.add_port(name=name, port=_p)
        top.add_label(text=name, position=(float(x), float(y)), layer=pdk.get_glayer("met2"))

    expose_col("CLK", xCLK, shp.ports["CLK"].center[1])
    expose_col("CLK_B", xCKB, shp.ports["CLK_B"].center[1])
    expose_col("VCC", xVCC, _ytop)
    expose_col("VSS", xVSS, _yg)

    expose_up("VOUTP", bref.ports["VOUTP"])        # interior opamp output -> route up to top edge
    expose_up("VOUTN", bref.ports["VOUTN"])
    # VCS_BIAS: expose_up's via at the pin would put its met3 pad ON the
    # VDP_BIAS met3 rail (both sit at the same y) -> VCS/VDP short. Stub NORTH
    # on met4 first (the pin column is met4-free until the summit cap arm),
    # via down to met2 at a rail-clear y, then the usual met2 riser to the top.
    _vp = bref.ports["VCS_BIAS"]
    _xc = _vp.center[0] + 1.5                   # pin center x
    _ylift = _vp.center[1] + 8.0                # clears VDP rail AND the buffer VSS met3 run
    _rect(_xc - 0.75, _vp.center[1] - 1.5, _xc + 0.75, _ylift + 0.75, "met4")
    _vv = top << via_stack(pdk, "met2", "met4", centered=True)
    _vv.movex(_xc - _vv.center[0]).movey(_ylift - _vv.center[1])
    _ytop2 = bref.ymax + 4
    _rect(_xc - 0.5, _ylift, _xc + 0.5, _ytop2)  # met2 riser
    top.add_port(name="VCS_BIAS", port=gf.Port(name="VCS_BIAS", center=(float(_xc), float(_ytop2)),
                 width=1.0, orientation=90, layer=pdk.get_glayer("met2")))
    top.add_label(text="VCS_BIAS", position=(float(_xc), float(_ytop2) - 0.2), layer=pdk.get_glayer("met2"))
    expose_up("VDD", bref.ports["VDD"])            # interior supply -> route up to top edge
    expose("VDP_BIAS", bref.ports["VDP_BIAS"])     # already at the bottom edge

    # component_snap_to_grid strips the S/H instances' flattened-in internal
    # labels generically (labels are cell-local; duplicate texts on different
    # nets would otherwise short BY NAME in magic) -- only the stage's own pin
    # labels above survive.
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
