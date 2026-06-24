from gdsfactory.cell import cell
from gdsfactory.component import Component
from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.blocks.composite.opamp.opamp import opamp
from glayout.flow.pdk.util.comp_utils import prec_ref_center
from glayout.flow.pdk.util.port_utils import rename_ports_by_orientation
from glayout.flow.pdk.util.snap_to_grid import component_snap_to_grid
from glayout.flow.spice import Netlist

def comparator_netlist(opamp_netlist: Netlist) -> Netlist:
    netlist = Netlist(circuit_name="comparator", nodes=['VDD', 'GND', 'VIN_P', 'VIN_N', 'VOUT', 'IBIAS1', 'IBIAS2'])
    netlist.connect_netlist(
        opamp_netlist,
        [('VDD', 'VDD'), ('GND', 'GND'), ('VP', 'VIN_P'), ('VN', 'VIN_N'), ('VOUT', 'VOUT'), ('CS_BIAS', 'IBIAS1'), ('DIFFPAIR_BIAS', 'IBIAS2')]
    )
    return netlist

@cell
def comparator(
    pdk: MappedPDK,
    **kwargs
) -> Component:
    """
    Push-button continuous-time comparator.
    It wraps the two-stage uncompensated opamp for high gain and rail-to-rail comparison.
    """
    top_level = Component("comparator")
    kwargs["add_output_stage"] = False 
    comp = opamp(pdk, **kwargs)
    comp_ref = prec_ref_center(comp)
    top_level.add(comp_ref)
    top_level.add_ports(comp_ref.get_ports_list(), prefix="cmp_")
    top_level.info['netlist'] = comparator_netlist(comp.info['netlist'])
    
    return component_snap_to_grid(rename_ports_by_orientation(top_level))
