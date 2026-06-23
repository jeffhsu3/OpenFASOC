import os
import shutil
import subprocess
from pathlib import Path

import pytest


DESIGN_NAME = "fixed_crossbar_pex"
WEIGHT_CODES = tuple(range(-8, 8))
EXPECTED_PINS = (
    ["ROW_0"]
    + [
        f"COL_{sign}_{column}"
        for column in range(len(WEIGHT_CODES))
        for sign in ("POS", "NEG")
    ]
    + ["VBIAS", "B"]
)


def _require_tool(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        pytest.skip(f"{name} is required for the crossbar PEX regression")
    return executable


def _require_sky130_file(relative_path: str) -> Path:
    pdk_root = os.getenv("PDK_ROOT")
    if not pdk_root:
        pytest.skip("PDK_ROOT is required for the crossbar PEX regression")
    path = Path(pdk_root) / "sky130A" / relative_path
    if not path.is_file():
        pytest.skip(f"required SKY130 file is unavailable: {path}")
    return path


def _subcircuit_pins(pex: str) -> list[str]:
    lines = pex.splitlines()
    for index, line in enumerate(lines):
        if line.startswith(f".subckt {DESIGN_NAME} "):
            pins = line.split()[2:]
            for continuation in lines[index + 1 :]:
                if not continuation.startswith("+"):
                    break
                pins.extend(continuation[1:].split())
            return pins
    raise AssertionError(f"{DESIGN_NAME} subcircuit was not extracted")


@pytest.fixture(scope="module")
def extracted_crossbar(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    magic = _require_tool("magic")
    magicrc = _require_sky130_file("libs.tech/magic/sky130A.magicrc")

    from glayout.flow.blocks.composite.crossbar.crossbar import conductance_crossbar
    from glayout.flow.pdk.sky130_mapped import sky130_mapped_pdk

    work_dir = tmp_path_factory.mktemp("fixed_crossbar_pex")
    gds_path = work_dir / f"{DESIGN_NAME}.gds"
    pex_path = work_dir / f"{DESIGN_NAME}.spice"
    magic_script = work_dir / "extract.tcl"

    crossbar = conductance_crossbar(
        sky130_mapped_pdk,
        weights=(WEIGHT_CODES,),
        weight_bits=4,
    )
    crossbar.name = DESIGN_NAME
    crossbar.write_gds(gds_path)

    magic_script.write_text(
        "\n".join(
            (
                f"gds read {gds_path}",
                f"load {DESIGN_NAME}",
                "select top cell",
                "port makeall",
                "extract do local",
                "extract all",
                "ext2spice lvs",
                "ext2spice cthresh 0",
                f"ext2spice -o {pex_path}",
                "quit -noprompt",
            )
        )
        + "\n",
        encoding="ascii",
    )
    result = subprocess.run(
        [
            magic,
            "-rcfile",
            str(magicrc),
            "-noconsole",
            "-dnull",
            str(magic_script),
        ],
        cwd=work_dir,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert pex_path.is_file(), result.stdout + result.stderr

    return {"pex": pex_path, "work_dir": work_dir}


def test_crossbar_pex_pin_and_weight_topology(
    extracted_crossbar: dict[str, Path],
) -> None:
    pex = extracted_crossbar["pex"].read_text(encoding="ascii")
    assert _subcircuit_pins(pex) == EXPECTED_PINS

    devices = [line.split() for line in pex.splitlines() if line.startswith("X")]
    assert len(devices) == sum(abs(code) for code in WEIGHT_CODES)

    for column, code in enumerate(WEIGHT_CODES):
        expected_sign = "POS" if code > 0 else "NEG"
        expected_output = f"COL_{expected_sign}_{column}"
        expected_devices = [
            device
            for device in devices
            if {device[1], device[3]} == {"ROW_0", expected_output}
        ]
        assert len(expected_devices) == abs(code)

        opposite_sign = "NEG" if expected_sign == "POS" else "POS"
        opposite_output = f"COL_{opposite_sign}_{column}"
        assert not any(
            {device[1], device[3]} == {"ROW_0", opposite_output} for device in devices
        )

    assert all(device[2] == "VBIAS" and device[4] == "B" for device in devices)
    assert all("w=1" in device and "l=0.35" in device for device in devices)
    assert any(line.startswith("C") for line in pex.splitlines())


def test_crossbar_pex_four_bit_weight_and_input_linearity(
    extracted_crossbar: dict[str, Path],
) -> None:
    ngspice = _require_tool("ngspice")
    models = _require_sky130_file("libs.tech/ngspice/sky130.lib.spice")
    work_dir = extracted_crossbar["work_dir"]
    testbench = work_dir / "crossbar_pex_tb.sp"
    sweep_data = work_dir / "crossbar_input_sweep.csv"
    column_nodes = " ".join(EXPECTED_PINS[1:-2])
    sense_sources = "\n".join(
        f"V{sign[0]}{column} COL_{sign}_{column} 0 0"
        for column in range(len(WEIGHT_CODES))
        for sign in ("POS", "NEG")
    )
    current_expressions = " ".join(
        f"i(V{sign}{column})"
        for column in range(len(WEIGHT_CODES))
        for sign in ("P", "N")
    )
    testbench.write_text(
        f""".lib {models} tt
.include {extracted_crossbar["pex"]}

XCB ROW_0 {column_nodes} VBIAS B {DESIGN_NAME}
VROW ROW_0 0 0
{sense_sources}
VBIAS VBIAS 0 1.2
VB B 0 0

.control
set wr_singlescale
dc VROW 0.01 0.10 0.01
wrdata {sweep_data} {current_expressions}
.endc
.end
""",
        encoding="ascii",
    )
    result = subprocess.run(
        [ngspice, "-b", str(testbench)],
        cwd=work_dir,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert sweep_data.is_file(), output

    sweep_rows = [
        [float(value) for value in line.split()]
        for line in sweep_data.read_text(encoding="ascii").splitlines()
        if line.strip()
    ]
    assert len(sweep_rows) == 10
    assert all(len(row) == 1 + 2 * len(WEIGHT_CODES) for row in sweep_rows)

    fitted_weight_lsbs = []
    maximum_weight_inl = 0.0
    maximum_weight_dnl = 0.0
    input_voltages = []
    for sweep_row in sweep_rows:
        input_voltage = sweep_row[0]
        input_voltages.append(input_voltage)
        sense_currents = sweep_row[1:]
        measured_currents = []
        for column, code in enumerate(WEIGHT_CODES):
            positive = sense_currents[2 * column]
            negative = sense_currents[2 * column + 1]
            measured_currents.append(positive - negative)
            if code > 0:
                assert positive > 0
                assert negative == pytest.approx(0, abs=1e-12)
            elif code < 0:
                assert negative > 0
                assert positive == pytest.approx(0, abs=1e-12)
            else:
                assert positive == pytest.approx(0, abs=1e-12)
                assert negative == pytest.approx(0, abs=1e-12)

        assert all(
            current_b > current_a
            for current_a, current_b in zip(
                measured_currents[:-1], measured_currents[1:], strict=True
            )
        )
        fitted_lsb = sum(
            code * current
            for code, current in zip(WEIGHT_CODES, measured_currents, strict=True)
        ) / sum(code * code for code in WEIGHT_CODES)
        fitted_weight_lsbs.append(fitted_lsb)
        weight_inl_lsb = [
            (current - fitted_lsb * code) / abs(fitted_lsb)
            for code, current in zip(WEIGHT_CODES, measured_currents, strict=True)
        ]
        weight_dnl_lsb = [
            (current_b - current_a) / fitted_lsb - 1
            for current_a, current_b in zip(
                measured_currents[:-1], measured_currents[1:], strict=True
            )
        ]
        maximum_weight_inl = max(
            maximum_weight_inl, max(abs(error) for error in weight_inl_lsb)
        )
        maximum_weight_dnl = max(
            maximum_weight_dnl, max(abs(error) for error in weight_dnl_lsb)
        )

    assert maximum_weight_inl < 0.5, (
        f"maximum weight INL was {maximum_weight_inl:.3f} LSB"
    )
    assert maximum_weight_dnl < 0.5, (
        f"maximum weight DNL was {maximum_weight_dnl:.3f} LSB"
    )

    input_slope = sum(
        voltage * fitted_lsb
        for voltage, fitted_lsb in zip(input_voltages, fitted_weight_lsbs, strict=True)
    ) / sum(voltage * voltage for voltage in input_voltages)
    maximum_magnitude = max(abs(code) for code in WEIGHT_CODES)
    full_scale_unit_current = input_slope * max(input_voltages)
    input_inl_lsb = [
        maximum_magnitude
        * (fitted_lsb - input_slope * voltage)
        / full_scale_unit_current
        for voltage, fitted_lsb in zip(input_voltages, fitted_weight_lsbs, strict=True)
    ]
    maximum_input_inl = max(abs(error) for error in input_inl_lsb)
    assert maximum_input_inl < 0.5, (
        f"maximum input INL at weight magnitude {maximum_magnitude} was "
        f"{maximum_input_inl:.3f} LSB"
    )


def test_crossbar_pex_tia_settling_linearity_and_headroom(
    extracted_crossbar: dict[str, Path],
) -> None:
    ngspice = _require_tool("ngspice")
    models = _require_sky130_file("libs.tech/ngspice/sky130.lib.spice")
    work_dir = extracted_crossbar["work_dir"]
    testbench = work_dir / "crossbar_pex_tia_tb.sp"
    transient_data = work_dir / "crossbar_tia_transient.csv"
    column_nodes = " ".join(EXPECTED_PINS[1:-2])
    tia_instances = "\n".join(
        f"XTIA_{sign}_{column} COL_{sign}_{column} OUT_{sign[0]}{column} "
        "VCM VSS VDD TIA"
        for column in range(len(WEIGHT_CODES))
        for sign in ("POS", "NEG")
    )
    sensed_expressions = " ".join(
        f"v(OUT_{sign[0]}{column}) v(COL_{sign}_{column})"
        for column in range(len(WEIGHT_CODES))
        for sign in ("POS", "NEG")
    )
    testbench.write_text(
        f""".lib {models} tt
.include {extracted_crossbar["pex"]}

.subckt TIA IN OUT VREF VSS VDD
GERR NINT 0 value={{0.01 * (V(IN) - V(VREF))}}
RDOM NINT 0 1MEG
CDOM NINT 0 159.155p
BOUT OUT 0 V=min(max(V(VREF) + V(NINT), V(VSS) + 0.05), V(VDD) - 0.05)
RFB OUT IN 1K
CFB OUT IN 100f
.ends TIA

XCB ROW_0 {column_nodes} VBIAS B {DESIGN_NAME}
VROW ROW_0 0 PWL(0 0.41 1u 0.41 1.1u 0.50 4u 0.50 4.1u 0.41 8u 0.41)
{tia_instances}
VBIAS VBIAS 0 1.2
VB B 0 0
VVCM VCM 0 0.4
VDD VDD 0 1.8
VSS VSS 0 0

.control
set wr_singlescale
tran 5n 8u
wrdata {transient_data} v(ROW_0) {sensed_expressions}
.endc
.end
""",
        encoding="ascii",
    )
    result = subprocess.run(
        [ngspice, "-b", str(testbench)],
        cwd=work_dir,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert transient_data.is_file(), output

    transient_rows = [
        [float(value) for value in line.split()]
        for line in transient_data.read_text(encoding="ascii").splitlines()
        if line.strip()
    ]
    assert len(transient_rows) > 1500
    assert all(len(row) == 2 + 4 * len(WEIGHT_CODES) for row in transient_rows)

    def tia_values(row: list[float], column: int) -> tuple[float, float, float, float]:
        offset = 2 + 4 * column
        return tuple(row[offset : offset + 4])

    def differential_output(row: list[float], column: int) -> float:
        out_positive, _, out_negative, _ = tia_values(row, column)
        return out_negative - out_positive

    positive_seven_column = WEIGHT_CODES.index(7)
    zero_column = WEIGHT_CODES.index(0)
    high_steady_rows = [row for row in transient_rows if 3.8e-6 <= row[0] <= 3.95e-6]
    low_steady_rows = [row for row in transient_rows if 7.8e-6 <= row[0] <= 8.0e-6]
    assert high_steady_rows and low_steady_rows

    high_target = sum(
        differential_output(row, positive_seven_column) for row in high_steady_rows
    ) / len(high_steady_rows)
    low_target = sum(
        differential_output(row, positive_seven_column) for row in low_steady_rows
    ) / len(low_steady_rows)
    output_lsb = abs(high_target) / 7
    tolerance = output_lsb / 2
    assert output_lsb > 0

    def settling_time(
        transition_end: float,
        hold_end: float,
        target: float,
    ) -> float:
        hold_rows = [
            row for row in transient_rows if transition_end <= row[0] <= hold_end
        ]
        for index, row in enumerate(hold_rows):
            if all(
                abs(differential_output(later, positive_seven_column) - target)
                <= tolerance
                for later in hold_rows[index:]
            ):
                return row[0] - transition_end
        raise AssertionError("output did not settle within the half-LSB band")

    rise_settling = settling_time(1.1e-6, 3.95e-6, high_target)
    fall_settling = settling_time(4.1e-6, 7.95e-6, low_target)
    assert rise_settling <= 1.0e-6, (
        f"rising output settling time was {rise_settling * 1e9:.1f} ns"
    )
    assert fall_settling <= 1.0e-6, (
        f"falling output settling time was {fall_settling * 1e9:.1f} ns"
    )

    steady_outputs = [
        sum(differential_output(row, column) for row in high_steady_rows)
        / len(high_steady_rows)
        for column in range(len(WEIGHT_CODES))
    ]
    fitted_output_lsb = sum(
        code * voltage
        for code, voltage in zip(WEIGHT_CODES, steady_outputs, strict=True)
    ) / sum(code * code for code in WEIGHT_CODES)
    output_inl = [
        (voltage - fitted_output_lsb * code) / abs(fitted_output_lsb)
        for code, voltage in zip(WEIGHT_CODES, steady_outputs, strict=True)
    ]
    maximum_output_inl = max(abs(error) for error in output_inl)
    assert maximum_output_inl < 0.5, f"TIA output INL was {maximum_output_inl:.3f} LSB"

    virtual_ground_error = max(
        abs(column_voltage - 0.4)
        for row in high_steady_rows
        for column in range(len(WEIGHT_CODES))
        for column_voltage in (tia_values(row, column)[1], tia_values(row, column)[3])
    )
    assert virtual_ground_error < 1e-3, (
        f"maximum virtual-ground error was {virtual_ground_error * 1e3:.3f} mV"
    )

    output_voltages = [
        output_voltage
        for row in transient_rows
        for column in range(len(WEIGHT_CODES))
        for output_voltage in (tia_values(row, column)[0], tia_values(row, column)[2])
    ]
    assert min(output_voltages) > 0.1
    assert max(output_voltages) < 1.7

    transition_rows = [row for row in transient_rows if 1.0e-6 <= row[0] <= 1.5e-6]
    maximum_zero_column_coupling = max(
        abs(differential_output(row, zero_column)) for row in transition_rows
    )
    assert maximum_zero_column_coupling <= tolerance, (
        "zero-weight column coupling was "
        f"{maximum_zero_column_coupling / output_lsb:.3f} LSB"
    )
