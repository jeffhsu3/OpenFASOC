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
from glayout.flow.blocks.composite.sample_and_hold.sample_hold_cell import (
    sample_hold_cell,
)
from glayout.flow.blocks.composite.diff_buffer.diff_buffer import diff_buffer


def _stage_netlist(sh: Component, buf: Component) -> Netlist:
    nodes = [
        "VINP",
        "VINN",
        "VOUTP",
        "VOUTN",
        "CLK",
        "CLK_B",
        "VDP_BIAS",
        "VCS_BIAS",
        "VDD",
        "VSS",
        "VCC",
    ]
    net = Netlist(circuit_name="differential_delay_stage", nodes=nodes)
    # per-rail single S/H: VIN -> HOLD on CLK
    net.connect_netlist(
        sh.info["netlist"],
        [
            ("VIN", "VINP"),
            ("CLK", "CLK"),
            ("CLK_B", "CLK_B"),
            ("VOUT", "HOLDP"),
            ("VCC", "VCC"),
            ("VSS", "VSS"),
        ],
    )
    net.connect_netlist(
        sh.info["netlist"],
        [
            ("VIN", "VINN"),
            ("CLK", "CLK"),
            ("CLK_B", "CLK_B"),
            ("VOUT", "HOLDN"),
            ("VCC", "VCC"),
            ("VSS", "VSS"),
        ],
    )
    # diff_buffer follows the held nodes
    net.connect_netlist(
        buf.info["netlist"],
        [
            ("VINP", "HOLDP"),
            ("VINN", "HOLDN"),
            ("VOUTP", "VOUTP"),
            ("VOUTN", "VOUTN"),
            ("VDP_BIAS", "VDP_BIAS"),
            ("VCS_BIAS", "VCS_BIAS"),
            ("VDD", "VDD"),
            ("VSS", "VSS"),
        ],
    )
    return net


@cell
def differential_delay_stage(pdk: MappedPDK, gap: float = 12.0) -> Component:
    pdk.activate()
    sh = sample_hold_cell(pdk, with_reset=False)
    buf = diff_buffer(pdk)
    top = Component()

    shp = top << sh  # p-rail S/H (upper-left)
    shn = top << sh  # n-rail S/H (lower-left)
    # Mirror the S/H cells horizontally: their VIN/CLK/CLK_B ports are native EAST (the cell
    # is cap-West / TG-East), which points INTO the macro when the S/H sits at the left and
    # buries the pins. Flipping puts VIN/CLK/CLK_B on the WEST -> at the left macro edge
    # (router-accessible), and bonus: VOUT now faces EAST toward the buffer (shorter routes).
    shp.mirror_x()
    shn.mirror_x()
    bref = top << buf  # buffer (right)
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

    from gdsfactory.components.rectangle import rectangle as _rectangle
    from glayout.flow.primitives.via_gen import via_stack as _via_stack

    W2 = 0.5  # This can't be hardcoded

    def _rect(x0, y0, x1, y1, glayer="met2"):
        _r = top << _rectangle(
            size=(round(abs(x1 - x0), 3), round(abs(y1 - y0), 3)),
            layer=pdk.get_glayer(glayer),
            centered=True,
        )
        _r.movex((x0 + x1) / 2 - _r.center[0]).movey((y0 + y1) / 2 - _r.center[1])

    def _viaat(x, y, l1, l2):
        _v = top << _via_stack(pdk, l1, l2)
        _v.movex(round(x, 3) - _v.center[0]).movey(round(y, 3) - _v.center[1])

    # Route VSS
    _pt, _pb = shp.ports["VSS"], shn.ports["VSS"]

    tie_offset_vss = 4.93  # This can't be hardcoded
    sh_comp = shp.parent
    for ref in sh_comp.references:
        if "transmission_gate" in ref.parent.name:
            tie_offset_vss = abs(
                ref.ports["N_tie_W_top_met_S"].center[0]
                - sh_comp.ports["VSS"].center[0]
            )
            break

    x_east_tie_vss = _pt.center[0] + tie_offset_vss
    _yrail = bref.ports["VSS"].center[1]
    # south stub: continue the column to the macro's bottom edge so the VSS
    # pin sits at the boundary (promoted-leaf pins must be edge-reachable
    # under the full-area OBS). The corridor x~11.1, y<-15 is open: south of
    # shn (ymin ~-15.0), west of the buffer (xmin 17.79).
    _ybot = min(shn.ymin, bref.ymin) + 0.1

    # Vias at the North and South east ring segments (the tie rings are
    # met1-only, so the met2 column crosses them freely elsewhere)
    _viaat(x_east_tie_vss, _pt.center[1], "met1", "met2")
    _viaat(x_east_tie_vss, _pb.center[1], "met1", "met2")

    # Straight met2 column down to the buffer yrail, up to the upper S/H N-tie
    _rect(
        x_east_tie_vss - W2 / 2,
        _yrail - W2 / 2,
        x_east_tie_vss + W2 / 2,
        _pt.center[1] + W2 / 2,
        "met2",
    )
    # south boundary stub on MET3: the HOLDN smart_route lands on a met2
    # strip along the bottom (y~-26.6) into the buffer VINN pin, which a met2
    # stub would short; hop to met3 at the rail and cross it layer-clear
    # (the HOLDN met3 riser is at x~7.5, 3.4 um west)
    _viaat(x_east_tie_vss, _yrail, "met2", "met3")
    _rect(
        x_east_tie_vss - W2 / 2,
        _ybot,
        x_east_tie_vss + W2 / 2,
        _yrail + 0.25,
        "met3",
    )

    # Horizontal connection to buffer VSS
    _rect(
        x_east_tie_vss - W2 / 2,
        _yrail - W2 / 2,
        bref.ports["VSS"].center[0] + 1.5,
        _yrail + W2 / 2,
        "met2",
    )
    _viaat(bref.ports["VSS"].center[0] + 1.0, _yrail, "met2", "met4")
    routed += ["VSS_sh", "VSS_buf"]

    # Route VCC
    _pt, _pb = shp.ports["VCC"], shn.ports["VCC"]
    tie_offset = 4.93  # This can't be an absolute number
    # There is nothing here that we could imporve

    tie_bottom_off = -4.93  # fallback: y from VCC port down to the tie-ring bottom
    for ref in sh_comp.references:
        if "transmission_gate" in ref.parent.name:
            tie_offset = abs(
                ref.ports["P_tie_W_top_met_S"].center[0]
                - sh_comp.ports["VCC"].center[0]
            )
            # bottom of the PMOS tie ring: P_tie_N is the (pfet-mirrored) bottom segment.
            # Its E/W ports sit at the segment centre (the mirrored cell drops the _N
            # edge port); use that as a cell-frame y-offset from the VCC port.
            tie_bottom_off = (
                ref.ports["P_tie_N_top_met_E"].center[1]
                - sh_comp.ports["VCC"].center[1]
            )
            break

    # Lift VCC on the WEST tie (nearer the west VCC exit). tie_offset is measured from
    # P_tie_W, so subtracting it lands directly on the west tie (no east mirror). The
    # column's TOP end connects at the BOTTOM of the top S/H's PMOS tie ring (not the VCC
    # port at the cell top) -- the ring is continuous met1, so the bar can stop there.
    x_west_tie = _pt.center[0] - tie_offset
    y_top = _pt.center[1] + tie_bottom_off
    _viaat(x_west_tie, y_top, "met1", "met2")
    _viaat(x_west_tie, _pb.center[1], "met1", "met2")
    _rect(
        x_west_tie - W2 / 2,
        _pb.center[1] - W2 / 2,
        x_west_tie + W2 / 2,
        y_top + W2 / 2,
        "met2",
    )
    vcc_pin_y = (_pb.center[1] + y_top) / 2  # centre of the (now shorter) west-tie bar

    # Route CLK and CLK_B: WEST-FLANK met3 columns + met2 stubs from the port
    # pads (ported from the measured east-tie-FIXED variant). Columns at the
    # bare port x's sat 0.8/1.7 um from the TG/VIN structures -- close AND
    # asymmetric, so the complementary CLK/CLK_B coupling no longer cancelled
    # (50.2 mV p-p output feedthrough, 30.8 mV signal-dependent pedestal ->
    # ~55 mV/hop offsets in the 8-stage line). On the flank they are ~5 um
    # out and near-equidistant (measured 8.6 mV p-p). met2 stubs crossing the
    # other net's met3 column are layer-clear by construction. A stub whose y
    # falls within the VCC west-tie met2 bar's span (the shp CLK stub) hops
    # the bar on MET3, dropping to met2 east of it -- a met2 run there is a
    # CLK-VCC short, and met1 is blocked by the VSS ring rails at that y.
    _bar_ylo = min(_pb.center[1], y_top) - W2 / 2 - 0.3
    _bar_yhi = max(_pb.center[1], y_top) + W2 / 2 + 0.3
    xw = min(shp.xmin, shn.xmin)
    x_clk_col = xw - 0.8
    x_ckb_col = xw - 1.6
    for net, _xcol in (("CLK", x_clk_col), ("CLK_B", x_ckb_col)):
        _pt, _pb = shp.ports[net], shn.ports[net]
        for _p in (_pt, _pb):
            _y = _p.center[1]
            if _bar_ylo < _y < _bar_yhi:
                # met3 over the VCC bar (merges with the column), via down
                # east of the bar, met2 into the port pad
                _rect(
                    _xcol - W2 / 2,
                    _y - W2 / 2,
                    x_west_tie + 1.0,
                    _y + W2 / 2,
                    "met3",
                )
                _viaat(x_west_tie + 0.85, _y, "met2", "met3")
                _rect(
                    x_west_tie + 0.6,
                    _y - W2 / 2,
                    _p.center[0] + 0.3,
                    _y + W2 / 2,
                    "met2",
                )
            else:
                _rect(
                    _xcol - W2 / 2,
                    _y - W2 / 2,
                    _p.center[0] + 0.3,
                    _y + W2 / 2,
                    "met2",
                )
                _viaat(_xcol, _y, "met2", "met3")
        _rect(
            _xcol - W2 / 2,
            _pb.center[1] - W2 / 2,
            _xcol + W2 / 2,
            _pt.center[1] + W2 / 2,
            "met3",
        )
        routed.append(net)

    # expose stage pins WITH GDS labels (so magic `port makeall` -> LEF PINs, and LVS
    # has named ports). Mirrors the coeff_cap/sample_hold_cell expose() pattern.
    def expose(name, port):
        top.add_port(name=name, port=port)
        top.add_label(
            text=name,
            position=(port.center[0], port.center[1]),
            layer=pdk.get_glayer(pdk.layer_to_glayer(port.layer)),
        )

    def expose_down(name, port):
        # met5 pin near an edge -> just via down to met2 in place and expose there.
        v = top << via_stack(
            pdk, pdk.layer_to_glayer(port.layer), "met2", centered=True
        )
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        top.add_port(name=name, port=v.ports["bottom_met_N"])
        top.add_label(
            text=name, position=(v.center[0], v.center[1]), layer=pdk.get_glayer("met2")
        )

    def expose_up(name, port):
        # INTERIOR met5 opamp output -> the router can't reach a pin buried mid-macro.
        # Via down to met2 (free corridor: opamp upper region uses met3-5, not met2) and
        # route straight UP to above the buffer top, exposing the PIN at the (new) top edge.
        ytop = bref.ymax + 4
        v = top << via_stack(
            pdk, pdk.layer_to_glayer(port.layer), "met2", centered=True
        )
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        dest = gf.Port(
            name=name + "_top",
            center=(float(port.center[0]), float(ytop)),
            width=float(v.ports["bottom_met_N"].width),
            orientation=270,
            layer=v.ports["bottom_met_N"].layer,
        )
        top << straight_route(
            pdk, v.ports["bottom_met_N"], dest, glayer1="met2", glayer2="met2"
        )
        top.add_port(
            name=name,
            port=gf.Port(
                name=name,
                center=(float(port.center[0]), float(ytop)),
                width=dest.width,
                orientation=90,
                layer=dest.layer,
            ),
        )
        # nudge the label INSIDE the riser metal: an exactly-on-edge label is
        # missed by magic `port makeall` (no LEF PIN -> unroutable macro pin)
        top.add_label(
            text=name,
            position=(float(port.center[0]), float(ytop) - 0.2),
            layer=pdk.get_glayer("met2"),
        )

    # West landing-pad egress for the S/H pins. The S/H pins are bare ~0.5um met2 taps buried
    # in the cell's congested met2 clock/via row, so even at the left edge the router can't
    # land a via. Pop UP to met3 (free above that row), route WEST to a clean pad at the left
    # macro edge, and expose the PIN there. xedge sits just inside the S/H left edge.
    xedge = float(min(shp.xmin, shn.xmin)) + 0.5

    def expose_west(name, port):
        v = top << via_stack(
            pdk, pdk.layer_to_glayer(port.layer), "met3", centered=True
        )
        v.movex(port.center[0] - v.center[0]).movey(port.center[1] - v.center[1])
        m3 = v.ports["top_met_W"]
        dest = gf.Port(
            name=name + "_w",
            center=(xedge, float(port.center[1])),
            width=float(m3.width),
            orientation=0,
            layer=m3.layer,
        )
        top << straight_route(pdk, m3, dest, glayer1="met3", glayer2="met3")
        top.add_port(
            name=name,
            port=gf.Port(
                name=name,
                center=(xedge, float(port.center[1])),
                width=float(m3.width),
                orientation=180,
                layer=m3.layer,
            ),
        )
        top.add_label(
            text=name,
            position=(xedge, float(port.center[1])),
            layer=pdk.get_glayer("met3"),
        )

    expose_west("VINP", shp.ports["VIN"])
    expose_west("VINN", shn.ports["VIN"])

    def expose_col(name, x, y):
        _p = gf.Port(
            name=name,
            center=(float(x), float(y)),
            width=W2,
            orientation=180,
            layer=pdk.get_glayer("met2"),
        )
        top.add_port(name=name, port=_p)
        top.add_label(
            text=name, position=(float(x), float(y)), layer=pdk.get_glayer("met2")
        )

    def expose_met3(name, x, y):
        _p = gf.Port(
            name=name,
            center=(float(x), float(y)),
            width=W2,
            orientation=90,
            layer=pdk.get_glayer("met3"),
        )
        top.add_port(name=name, port=_p)
        top.add_label(
            text=name, position=(float(x), float(y)), layer=pdk.get_glayer("met3")
        )

    # CLK / CLK_B pins sit directly ON the west-flank met3 columns -- top-level
    # geometry on the nets themselves, at the (new) left macro edge. Labels at
    # the shp-port y are 0.25 um inside the column top (magic port makeall
    # drops exactly-on-edge labels).
    expose_met3("CLK", x_clk_col, shp.ports["CLK"].center[1])
    expose_met3("CLK_B", x_ckb_col, shp.ports["CLK_B"].center[1])
    # VCC pin centered on the west-tie met2 bar -- lands directly on that bar's metal.
    expose_col("VCC", x_west_tie, vcc_pin_y)

    # Expose VSS at the SOUTH end of the met3 boundary stub
    def expose_vss(name, x, y):
        _p = gf.Port(
            name=name,
            center=(float(x), float(y)),
            width=W2,
            orientation=270,
            layer=pdk.get_glayer("met3"),
        )
        top.add_port(name=name, port=_p)
        top.add_label(
            text=name, position=(float(x), float(y)), layer=pdk.get_glayer("met3")
        )

    expose_vss("VSS", x_east_tie_vss, _ybot + 0.25)

    expose_up(
        "VOUTP", bref.ports["VOUTP"]
    )  # interior opamp output -> route up to top edge
    expose_up("VOUTN", bref.ports["VOUTN"])
    # VCS_BIAS: expose_up's via at the pin would put its met3 pad ON the
    # VDP_BIAS met3 rail (both sit at the same y) -> VCS/VDP short. Stub NORTH
    # on met4 first (the pin column is met4-free until the summit cap arm),
    # via down to met2 at a rail-clear y, then the usual met2 riser to the top.
    _vp = bref.ports["VCS_BIAS"]
    _xc = _vp.center[0] + 1.5  # pin center x
    _ylift = _vp.center[1] + 8.0  # clears VDP rail AND the buffer VSS met3 run
    _rect(_xc - 0.75, _vp.center[1] - 1.5, _xc + 0.75, _ylift + 0.75, "met4")
    _vv = top << via_stack(pdk, "met2", "met4", centered=True)
    _vv.movex(_xc - _vv.center[0]).movey(_ylift - _vv.center[1])
    _ytop2 = bref.ymax + 4
    _rect(_xc - 0.5, _ylift, _xc + 0.5, _ytop2)  # met2 riser
    top.add_port(
        name="VCS_BIAS",
        port=gf.Port(
            name="VCS_BIAS",
            center=(float(_xc), float(_ytop2)),
            width=1.0,
            orientation=90,
            layer=pdk.get_glayer("met2"),
        ),
    )
    top.add_label(
        text="VCS_BIAS",
        position=(float(_xc), float(_ytop2) - 0.2),
        layer=pdk.get_glayer("met2"),
    )
    expose_up("VDD", bref.ports["VDD"])  # interior supply -> route up to top edge
    # VDP_BIAS south boundary stub: the buffer pin pad (met3) sits 6 um inside
    # the south edge, past the promote_leaf trim margin. The only shape below
    # it is the buffer's HOLDN met2 strip along y~-26.6, crossed layer-clear
    # on met3 -- straight met3 rect from the pad to the boundary, pin at the
    # south end (port center = the pad's WEST edge, so nudge east into it)
    _vdp = bref.ports["VDP_BIAS"]
    _xvdp = _vdp.center[0] + 0.25
    _rect(_xvdp - W2 / 2, _ybot, _xvdp + W2 / 2, _vdp.center[1], "met3")
    expose_vss("VDP_BIAS", _xvdp, _ybot + 0.25)

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
    from glayout.flow.pdk.sky130_mapped.sky130_mapped import sky130_mapped_pdk

    for pdk_name, pdk in [("gf180", gf180_mapped_pdk), ("sky130", sky130_mapped_pdk)]:
        c = differential_delay_stage(pdk)
        gds_file = f"differential_delay_stage_{pdk_name}.gds"
        c.write_gds(gds_file)

        spice_file = f"differential_delay_stage_{pdk_name}.spice"
        with open(spice_file, "w") as f:
            f.write(c.info["netlist"].generate_netlist())

        print(f"[{pdk_name}] GDS:", c.name, "bbox=", c.bbox)
        print(
            f"[{pdk_name}] routed:",
            c.info.get("routed"),
            "FAILED:",
            c.info.get("route_failed"),
        )
