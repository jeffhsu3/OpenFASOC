from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.pdk.sky130_mapped import sky130_mapped_pdk
from gdsfactory.cell import cell
from gdsfactory.component import Component
from gdsfactory import Component
from glayout.flow.primitives.fet import nmos, pmos, multiplier
from glayout.flow.pdk.util.comp_utils import evaluate_bbox, prec_center, align_comp_to_port, movex, movey
from glayout.flow.pdk.util.snap_to_grid import component_snap_to_grid
from glayout.flow.pdk.util.port_utils import rename_ports_by_orientation
from glayout.flow.routing.straight_route import straight_route
from glayout.flow.routing.c_route import c_route
from gdsfactory.components.rectangle import rectangle
from glayout.flow.routing.L_route import L_route
from glayout.flow.primitives.guardring import tapring
from glayout.flow.pdk.util.port_utils import add_ports_perimeter
from glayout.flow.spice.netlist import Netlist
from glayout.flow.primitives.via_gen import via_stack
from gdsfactory.components import text_freetype, rectangle
from glayout.flow.pdk.util.label_utils import add_pin_labels, LabelSpec

# Port -> LVS-label mapping for the transmission gate. Layers are derived from each
# port's own metal by add_pin_labels, so this works on any PDK (not just sky130).
_TG_LABELS = [
    LabelSpec("VIN", "N_multiplier_0_source_E", size=0.27),
    LabelSpec("VOUT", "P_multiplier_0_drain_W", size=0.27),
    LabelSpec("VCC", "P_tie_S_top_met_S", size=0.5),
    LabelSpec("VSS", "N_tie_S_top_met_N", size=0.5),
    LabelSpec("VGP", "P_multiplier_0_gate_E", size=0.27),
    LabelSpec("VGN", "N_multiplier_0_gate_E", size=0.27),
]


def add_tg_labels(tg_in: Component, pdk: MappedPDK) -> Component:
    """Add LVS pin rectangles + text labels to a transmission gate (PDK-agnostic)."""
    return add_pin_labels(tg_in, pdk, _TG_LABELS)


def tg_netlist(nfet: Component, pfet: Component) -> Netlist:

         netlist = Netlist(circuit_name='Transmission_Gate', nodes=['VIN', 'VSS', 'VOUT', 'VCC', 'VGP', 'VGN'])
         netlist.connect_netlist(nfet.info['netlist'], [('D', 'VOUT'), ('G', 'VGN'), ('S', 'VIN'), ('B', 'VSS')])
         netlist.connect_netlist(pfet.info['netlist'], [('D', 'VOUT'), ('G', 'VGP'), ('S', 'VIN'), ('B', 'VCC')])

         return netlist

@cell
def  transmission_gate(
        pdk: MappedPDK,
        width: tuple[float,float] = (1,1),
        length: tuple[float,float] = (None,None),
        fingers: tuple[int,int] = (1,1),
        multipliers: tuple[int,int] = (1,1),
        substrate_tap: bool = False,
        tie_layers: tuple[str,str] = ("met2","met1"),
        **kwargs
        ) -> Component:
    """
    creates a transmission gate
    tuples are in (NMOS,PMOS) order
    **kwargs are any kwarg that is supported by nmos and pmos
    """
   
    top_level = Component(name="transmission_gate")
    # GF180 issues
    kwargs.setdefault("inter_finger_topmet", "met1")

    #two fets
    nfet = nmos(pdk, width=width[0], fingers=fingers[0], multipliers=multipliers[0], with_dummy=True, with_dnwell=False,  with_substrate_tap=False, length=length[0], tie_layers=tie_layers, **kwargs)
    pfet = pmos(pdk, width=width[1], fingers=fingers[1], multipliers=multipliers[1], with_dummy=True, with_substrate_tap=False, length=length[1], tie_layers=tie_layers, **kwargs)
    nfet_ref = top_level << nfet
    pfet_ref = top_level << pfet 
    pfet_ref = rename_ports_by_orientation(pfet_ref.mirror_y())

    #Relative move
    pfet_ref.movey(nfet_ref.ymax + evaluate_bbox(pfet_ref)[1]/2 + pdk.util_max_metal_seperation() + 0.02)
    
    #Routing
    # Inter-FET connections. The per-FET multiplier-array routing (fet.py,
    # __mult_array_macro) already shorts every multiplier of a terminal, so ONE
    # link between the CLOSEST rows suffices: the nfet's top row (multiplier_{n-1})
    # faces the mirrored pfet's bottom row (multiplier_{m-1}).
    _mn, _mp = multipliers[0] - 1, multipliers[1] - 1
    if tie_layers[0] != "met2":
        # Non-met2 ring N/S segments (e.g. tie_layers=("met1","met1")): met2 can
        # cross the rings directly, so connect on met2 with NO vias and no met3.
        # Shape = a same-layer "C": a stub from each port plus a vertical trunk
        # OFFSET past the rail ends -- on each flank the source AND drain rails
        # end at the SAME x, so a flush bar would skewer the other net's rail end
        # (this clearance is also why the legacy c_route has its `extension`).
        # BOTH links are required: a TG is two fets in PARALLEL (S<->S = VIN,
        # D<->D = VOUT; the netlist declares both, and consumers like the S/H
        # tap VOUT only at the pfet drain -- without the drain link the nfet
        # drain floats and half the switch is dead).
        def _rect(x0, y0, x1, y1):
            _r = top_level << rectangle(size=(round(x1 - x0, 3), round(y1 - y0, 3)), layer=pdk.get_glayer("met2"), centered=True)
            _r.movex((x0 + x1) / 2 - _r.center[0]).movey((y0 + y1) / 2 - _r.center[1])
        def _cbar(p1, p2, east):
            sgn = 1 if east else -1
            ext, bw = 0.5, 0.4
            x_edge = max(sgn * p1.center[0], sgn * p2.center[0]) * sgn  # outermost rail end
            xb = sorted((x_edge + sgn * ext, x_edge + sgn * (ext + bw)))
            ylo, yhi = 1e9, -1e9
            for p in (p1, p2):
                h = min(float(p.width), 0.7)
                xs = sorted((p.center[0] - sgn * 0.2, x_edge + sgn * (ext + bw)))
                _rect(xs[0], p.center[1] - h / 2, xs[1], p.center[1] + h / 2)
                ylo = min(ylo, p.center[1] - h / 2); yhi = max(yhi, p.center[1] + h / 2)
            _rect(xb[0], ylo, xb[1], yhi)
        _cbar(nfet_ref.ports[f"multiplier_{_mn}_source_E"], pfet_ref.ports[f"multiplier_{_mp}_source_E"], east=True)
        _cbar(nfet_ref.ports[f"multiplier_{_mn}_drain_W"], pfet_ref.ports[f"multiplier_{_mp}_drain_W"], east=False)
    else:
        # Legacy met2-ring path (byte-for-byte unchanged): met3 trunks hop the rings.
        # With stacked multipliers (multipliers>1) the per-FET gate trunk crowds the
        # East inter-FET source trunk on met3 (M3.2a spacing < 0.30um); extend the
        # source c-route East so its full-height trunk clears the gate trunk. The
        # single-multiplier default (extension 0.5) is unchanged and stays clean.
        src_extension = 1.0 if max(multipliers) > 1 else 0.5
        top_level << c_route(pdk, nfet_ref.ports["multiplier_0_source_E"], pfet_ref.ports["multiplier_0_source_E"], extension=src_extension)
        top_level << c_route(pdk, nfet_ref.ports["multiplier_0_drain_W"], pfet_ref.ports["multiplier_0_drain_W"], viaoffset=False)
    
    #Renaming Ports
    top_level.add_ports(nfet_ref.get_ports_list(), prefix="N_")
    top_level.add_ports(pfet_ref.get_ports_list(), prefix="P_")

    #substrate tap
    if substrate_tap:
            substrate_tap_encloses =((evaluate_bbox(top_level)[0]+pdk.util_max_metal_seperation() + 0.02), (evaluate_bbox(top_level)[1]+pdk.util_max_metal_seperation() + 0.02))
            guardring_ref = top_level << tapring(
            pdk,
            enclosed_rectangle=substrate_tap_encloses,
            sdlayer="p+s/d",
            horizontal_glayer='met2',
            vertical_glayer='met1',
        )
            guardring_ref.move(nfet_ref.center).movey(evaluate_bbox(pfet_ref)[1]/2 + (pdk.util_max_metal_seperation() + 0.02)/2)
            top_level.add_ports(guardring_ref.get_ports_list(),prefix="tap_")
    
    component = component_snap_to_grid(rename_ports_by_orientation(top_level)) 
    component.info['netlist'] = tg_netlist(nfet, pfet)


    return component

if __name__ == "__main__":
    from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk
    from glayout.flow.pdk.sky130_mapped import sky130_mapped_pdk
    
    for pdk_name, pdk in [("gf180", gf180_mapped_pdk), ("sky130", sky130_mapped_pdk)]:
        print(f"Generating transmission gate({pdk_name})...")
        comp = transmission_gate(pdk, multipliers=(2, 2), fingers=(2, 1), tie_layers=("met1", "met1"))
        out = f"tg_{pdk_name}.gds"
        comp.write_gds(out)
