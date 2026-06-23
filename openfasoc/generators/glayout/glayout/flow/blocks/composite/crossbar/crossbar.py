from copy import deepcopy

from gdsfactory.cell import cell
from gdsfactory.component import Component
from gdsfactory.components import rectangle

from glayout.flow.pdk.mappedpdk import MappedPDK
from glayout.flow.pdk.util.comp_utils import evaluate_bbox, prec_ref_center
from glayout.flow.pdk.util.snap_to_grid import component_snap_to_grid
from glayout.flow.primitives.fet import nmos
from glayout.flow.primitives.guardring import tapring
from glayout.flow.primitives.via_gen import via_array
from glayout.flow.spice import Netlist


def _validate_weights(
    weights: tuple[tuple[int, ...], ...], weight_bits: int
) -> tuple[tuple[int, ...], ...]:
    if not 2 <= weight_bits <= 4:
        raise ValueError("weight_bits must be between 2 and 4")
    if not weights or not weights[0]:
        raise ValueError("weights must be a non-empty rectangular matrix")

    columns = len(weights[0])
    if any(len(row) != columns for row in weights):
        raise ValueError("weights must be a non-empty rectangular matrix")

    minimum = -(1 << (weight_bits - 1))
    maximum = (1 << (weight_bits - 1)) - 1
    if any(weight < minimum or weight > maximum for row in weights for weight in row):
        raise ValueError(
            f"{weight_bits}-bit signed weights must be in [{minimum}, {maximum}]"
        )
    return tuple(tuple(row) for row in weights)


def _add_metal_bridge(
    component: Component,
    pdk: MappedPDK,
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    glayer: str,
) -> None:
    """Add an orthogonal bridge, routing vertically and then horizontally."""
    if start[1] != end[1]:
        vertical = component << rectangle(
            size=(width, abs(end[1] - start[1]) + width),
            layer=pdk.get_glayer(glayer),
            centered=True,
        )
        vertical.move((start[0], (start[1] + end[1]) / 2))
    if start[0] != end[0]:
        horizontal = component << rectangle(
            size=(abs(end[0] - start[0]) + width, width),
            layer=pdk.get_glayer(glayer),
            centered=True,
        )
        horizontal.move(((start[0] + end[0]) / 2, end[1]))


@cell
def conductance_crossbar(
    pdk: MappedPDK,
    weights: tuple[tuple[int, ...], ...],
    weight_bits: int = 4,
    unit_width: float = 1.0,
    length: float = 0.35,
    row_pitch_um: float = 3.0,
    column_pitch_um: float = 3.0,
) -> Component:
    """Create a fixed-weight differential NMOS conductance crossbar.

    ``weights`` contains signed two's-complement integers. Weight magnitude is
    encoded by identical transistor fingers; weight sign selects COL_POS or
    COL_NEG. ROW ports, differential column ports, VBIAS, and B are analog.
    """
    weights = _validate_weights(weights, weight_bits)
    if unit_width <= 0 or length <= 0:
        raise ValueError("unit_width and length must both be positive")
    if row_pitch_um <= 0 or column_pitch_um <= 0:
        raise ValueError("row and column pitches must both be positive")

    pdk.activate()
    rows = len(weights)
    columns = len(weights[0])
    maximum_magnitude = max(1, max(abs(weight) for row in weights for weight in row))

    def make_fet(fingers: int) -> Component:
        return nmos(
            pdk,
            width=unit_width,
            length=length,
            fingers=fingers,
            multipliers=1,
            with_dummy=False,
            with_tie=False,
            with_dnwell=False,
            with_substrate_tap=False,
            # Route the inter-finger source/drain vias up to met1 only. The
            # default ("met2") leaves a per-finger met2 patch 0.25um below the
            # met2 S/D rail, violating gf180 met2 spacing (M2.2a, 0.28um); the
            # rail stays met2 so the crossbar's vias/bridge still connect.
            inter_finger_topmet="met1",
        )

    fet_components = {
        magnitude: make_fet(magnitude)
        for magnitude in sorted(
            {abs(weight) for row in weights for weight in row if weight}
            | {maximum_magnitude}
        )
    }
    template = fet_components[maximum_magnitude]
    source_port = template.ports["multiplier_0_source_W"]
    drain_port = template.ports["multiplier_0_drain_E"]
    gate_port = template.ports["multiplier_0_gate_S"]

    source_via = via_array(pdk, "met1", "met3", size=(0.8, 0.8), no_exception=True)
    drain_via = via_array(pdk, "met1", "met4", size=(0.8, 0.8), no_exception=True)
    drain_met3 = drain_via.extract(layers=[pdk.get_glayer("met3")])

    max_spacing = pdk.util_max_metal_seperation()
    row_width = max(0.5, pdk.get_grule("met3")["min_width"])
    column_width = max(0.5, pdk.get_grule("met4")["min_width"])
    bias_width = max(0.5, pdk.get_grule("met2")["min_width"])

    required_drain_y = (
        source_port.center[1]
        + row_width / 2
        + evaluate_bbox(drain_met3)[1] / 2
        + pdk.get_grule("met3")["min_separation"]
    )
    drain_via_shift = max(0, required_drain_y - drain_port.center[1])

    rail_offset = template.xsize / 2 + max_spacing + drain_via.xsize / 2
    output_landing_width = max(column_width, drain_via.xsize)
    minimum_column_pitch = 2 * rail_offset + output_landing_width + max_spacing
    column_pitch = max(column_pitch_um, minimum_column_pitch)

    unit_ymin = min(template.ymin, gate_port.center[1] - bias_width / 2)
    unit_ymax = max(
        template.ymax,
        source_port.center[1] + source_via.ysize / 2,
        drain_port.center[1] + drain_via_shift + drain_via.ysize / 2,
    )
    row_pitch = max(row_pitch_um, unit_ymax - unit_ymin + max_spacing)

    positive_x = [c * column_pitch - rail_offset for c in range(columns)]
    negative_x = [c * column_pitch + rail_offset for c in range(columns)]
    bias_x = positive_x[0] - drain_via.xsize / 2 - max_spacing - bias_width / 2
    route_xmax = negative_x[-1] + drain_via.xsize / 2
    top_level = Component(name="conductance_crossbar")
    refs: dict[tuple[int, int], Component] = {}

    for r, weight_row in enumerate(weights):
        for c, weight in enumerate(weight_row):
            if weight == 0:
                continue
            magnitude = abs(weight)
            fet_ref = prec_ref_center(fet_components[magnitude])
            fet_ref.move((c * column_pitch, r * row_pitch))
            top_level.add(fet_ref)
            refs[(r, c)] = fet_ref

            source_via_ref = prec_ref_center(source_via)
            source_via_ref.move(fet_ref.ports["multiplier_0_source_W"].center)
            top_level.add(source_via_ref)

            output_x = positive_x[c] if weight > 0 else negative_x[c]
            drain_start = tuple(fet_ref.ports["multiplier_0_drain_E"].center)
            drain_end = (
                output_x,
                drain_start[1] + drain_via_shift,
            )
            drain_via_ref = prec_ref_center(drain_via)
            drain_via_ref.move(drain_end)
            top_level.add(drain_via_ref)
            _add_metal_bridge(
                top_level,
                pdk,
                drain_start,
                drain_end,
                float(fet_ref.ports["multiplier_0_drain_E"].width),
                "met2",
            )

    row_xmin = bias_x
    row_xmax = route_xmax
    for r in range(rows):
        y_center = r * row_pitch + source_port.center[1]
        row_ref = top_level << rectangle(
            size=(row_xmax - row_xmin, row_width),
            layer=pdk.get_glayer("met3"),
            centered=True,
        )
        row_ref.move(((row_xmin + row_xmax) / 2, y_center))
        row_name = f"ROW_{r}"
        top_level.add_port(
            name=row_name,
            center=(row_xmin, y_center),
            width=row_width,
            orientation=180,
            layer=pdk.get_glayer("met3"),
        )
        top_level.add_label(
            text=row_name,
            position=(row_xmin, y_center),
            layer=pdk.get_glayer("met3"),
        )

    drain_ymin = drain_port.center[1] + drain_via_shift - 1.0
    drain_ymax = (rows - 1) * row_pitch + drain_port.center[1] + drain_via_shift + 1.0
    for c in range(columns):
        for sign, x_center in (("POS", positive_x[c]), ("NEG", negative_x[c])):
            column_ref = top_level << rectangle(
                size=(column_width, drain_ymax - drain_ymin),
                layer=pdk.get_glayer("met4"),
                centered=True,
            )
            column_ref.move((x_center, (drain_ymin + drain_ymax) / 2))
            column_name = f"COL_{sign}_{c}"
            top_level.add_port(
                name=column_name,
                center=(x_center, drain_ymin),
                width=column_width,
                orientation=270,
                layer=pdk.get_glayer("met4"),
            )
            top_level.add_label(
                text=column_name,
                position=(x_center, drain_ymin),
                layer=pdk.get_glayer("met4"),
            )

    gate_ymin = gate_port.center[1]
    gate_ymax = (rows - 1) * row_pitch + gate_port.center[1]
    gate_xmax = (columns - 1) * column_pitch + template.xsize / 2
    for r in range(rows):
        gate_y = r * row_pitch + gate_port.center[1]
        gate_row_ref = top_level << rectangle(
            size=(gate_xmax - bias_x, bias_width),
            layer=pdk.get_glayer("met2"),
            centered=True,
        )
        gate_row_ref.move(((bias_x + gate_xmax) / 2, gate_y))
    bias_spine_ref = top_level << rectangle(
        size=(bias_width, gate_ymax - gate_ymin + bias_width),
        layer=pdk.get_glayer("met2"),
        centered=True,
    )
    bias_spine_ref.move((bias_x, (gate_ymin + gate_ymax) / 2))
    top_level.add_port(
        name="VBIAS",
        center=(bias_x, gate_ymin),
        width=bias_width,
        orientation=180,
        layer=pdk.get_glayer("met2"),
    )
    top_level.add_label(
        text="VBIAS",
        position=(bias_x, gate_ymin),
        layer=pdk.get_glayer("met2"),
    )

    array_center = (
        (top_level.xmin + top_level.xmax) / 2,
        (top_level.ymin + top_level.ymax) / 2,
    )
    tap_separation = (
        max(
            max_spacing,
            pdk.get_grule("active_diff", "active_tap")["min_separation"],
        )
        + pdk.get_grule("p+s/d", "active_tap")["min_enclosure"]
    )
    body_tap = tapring(
        pdk,
        enclosed_rectangle=(
            top_level.xsize + 2 * tap_separation,
            top_level.ysize + 2 * tap_separation,
        ),
        sdlayer="p+s/d",
        horizontal_glayer="met1",
        vertical_glayer="met1",
    )
    body_tap_ref = prec_ref_center(body_tap)
    body_tap_ref.move(array_center)
    top_level.add(body_tap_ref)

    pwell_enclosure = pdk.get_grule("pwell", "active_tap")["min_enclosure"]
    pwell_ref = top_level << rectangle(
        size=(
            body_tap.xsize + 2 * pwell_enclosure,
            body_tap.ysize + 2 * pwell_enclosure,
        ),
        layer=pdk.get_glayer("pwell"),
        centered=True,
    )
    pwell_ref.move(array_center)

    top_level.add_port(name="B", port=body_tap_ref.ports["S_top_met_S"])
    top_level.add_label(
        text="B",
        position=body_tap_ref.ports["S_top_met_S"].center,
        layer=pdk.get_glayer("met1"),
    )

    nodes = (
        [f"ROW_{r}" for r in range(rows)]
        + [f"COL_{sign}_{c}" for c in range(columns) for sign in ("POS", "NEG")]
        + ["VBIAS", "B"]
    )
    netlist = Netlist(circuit_name="conductance_crossbar", nodes=nodes)
    for r, weight_row in enumerate(weights):
        for c, weight in enumerate(weight_row):
            if weight == 0:
                continue
            fet_netlist = deepcopy(fet_components[abs(weight)].info["netlist"])
            fet_netlist.parameters["mult"] = abs(weight)
            netlist.connect_netlist(
                fet_netlist,
                [
                    ("D", f"COL_{'POS' if weight > 0 else 'NEG'}_{c}"),
                    ("G", "VBIAS"),
                    ("S", f"ROW_{r}"),
                    ("B", "B"),
                ],
            )
    if not netlist.sub_netlists:
        netlist.source_netlist = ".subckt {circuit_name} {nodes}\n.ends {circuit_name}"

    top_level.info["netlist"] = netlist
    top_level.info["weights"] = weights
    top_level.info["weight_bits"] = weight_bits
    return component_snap_to_grid(top_level)
