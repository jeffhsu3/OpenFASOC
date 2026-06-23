import pytest

from glayout.flow.blocks.composite.crossbar.crossbar import conductance_crossbar
from glayout.flow.pdk.gf180_mapped.gf180_mapped import gf180_mapped_pdk


def test_fixed_crossbar_ports_and_netlist() -> None:
    crossbar = conductance_crossbar(
        gf180_mapped_pdk,
        weights=((7, -3), (0, 1)),
        weight_bits=4,
    )

    assert list(crossbar.ports) == [
        "ROW_0",
        "ROW_1",
        "COL_POS_0",
        "COL_NEG_0",
        "COL_POS_1",
        "COL_NEG_1",
        "VBIAS",
        "B",
    ]
    assert crossbar.info["netlist"].nodes == [
        "ROW_0",
        "ROW_1",
        "COL_POS_0",
        "COL_NEG_0",
        "COL_POS_1",
        "COL_NEG_1",
        "VBIAS",
        "B",
    ]

    spice = crossbar.info["netlist"].generate_netlist()
    assert "X0 COL_POS_0 VBIAS ROW_0 B NMOS l=0.35 w=1.0 m=7" in spice
    assert "X1 COL_NEG_1 VBIAS ROW_0 B NMOS_1 l=0.35 w=1.0 m=3" in spice
    assert "X2 COL_POS_1 VBIAS ROW_1 B NMOS_2 l=0.35 w=1.0 m=1" in spice
    assert "COL_NEG_0 VBIAS ROW_1" not in spice


def test_all_zero_crossbar_has_no_transistors() -> None:
    crossbar = conductance_crossbar(
        gf180_mapped_pdk,
        weights=((0, 0), (0, 0)),
        weight_bits=4,
    )

    spice = crossbar.info["netlist"].generate_netlist()
    assert ".subckt conductance_crossbar" in spice
    assert "X0 " not in spice


@pytest.mark.parametrize("weight", (-9, 8))
def test_fixed_crossbar_rejects_out_of_range_4bit_weights(weight: int) -> None:
    with pytest.raises(ValueError, match="4-bit signed weights"):
        conductance_crossbar(
            gf180_mapped_pdk,
            weights=((weight,),),
            weight_bits=4,
        )


def test_fixed_crossbar_rejects_ragged_weights() -> None:
    with pytest.raises(ValueError, match="rectangular matrix"):
        conductance_crossbar(
            gf180_mapped_pdk,
            weights=((1, 2), (3,)),
            weight_bits=4,
        )
