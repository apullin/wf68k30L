"""
Unit test for WF68K30L_DATA_REGISTERS module.

Tests the data register file independently:
  - Write/read back in LONG, WORD, BYTE sizes
  - Verify partial writes preserve upper bits
  - Hazard detection (DR_IN_USE)
  - Reset clears all registers

Set TOPLEVEL=WF68K30L_DATA_REGISTERS when running this test.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles


# OP_SIZE encoding from wf68k30L_pkg.svh:
#   LONG = 2'b00 = 0
#   WORD = 2'b01 = 1
#   BYTE = 2'b10 = 2
LONG = 0
WORD = 1
BYTE = 2


def init_signals(dut):
    """Initialize all input signals to safe defaults."""
    dut.RESET.value = 0
    dut.DR_IN_1.value = 0
    dut.DR_IN_2.value = 0
    dut.DR_SEL_WR_1.value = 0
    dut.DR_SEL_WR_2.value = 0
    dut.DR_SEL_RD_1.value = 0
    dut.DR_SEL_RD_2.value = 0
    dut.DR_WR_1.value = 0
    dut.DR_WR_2.value = 0
    dut.DR_MARK_USED.value = 0
    dut.USE_DPAIR.value = 0
    dut.UNMARK.value = 0
    dut.OP_SIZE.value = LONG


async def do_reset(dut, cycles=5):
    """Assert RESET for the given number of cycles."""
    dut.RESET.value = 1
    await ClockCycles(dut.CLK, cycles)
    dut.RESET.value = 0
    await RisingEdge(dut.CLK)


async def write_reg(dut, port, reg_num, value, op_size=LONG):
    """Write a value to a data register via write port 1 or 2.

    The register file latches DR_SEL_WR when DR_MARK_USED is asserted,
    then performs the actual write when DR_WR is asserted.
    """
    dut.OP_SIZE.value = op_size

    # Step 1: Mark the register as used (latches the write select)
    if port == 1:
        dut.DR_SEL_WR_1.value = reg_num
    else:
        dut.DR_SEL_WR_2.value = reg_num
    dut.DR_MARK_USED.value = 1
    await RisingEdge(dut.CLK)
    dut.DR_MARK_USED.value = 0

    # Step 2: Drive data and assert write enable
    if port == 1:
        dut.DR_IN_1.value = value
        dut.DR_WR_1.value = 1
    else:
        dut.DR_IN_2.value = value
        dut.DR_WR_2.value = 1
    await RisingEdge(dut.CLK)

    # Deassert write
    dut.DR_WR_1.value = 0
    dut.DR_WR_2.value = 0
    await RisingEdge(dut.CLK)


def read_reg(dut, port, reg_num):
    """Read a register value via read port 1 or 2 (combinational).

    Returns the integer value.
    """
    if port == 1:
        dut.DR_SEL_RD_1.value = reg_num
        return int(dut.DR_OUT_1.value)
    else:
        dut.DR_SEL_RD_2.value = reg_num
        return int(dut.DR_OUT_2.value)


@cocotb.test()
async def test_reset_clears_all(dut):
    """Verify that RESET clears all 8 data registers to zero."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)

    await do_reset(dut)

    # Check all 8 registers are zero via read port 1
    for reg in range(8):
        dut.DR_SEL_RD_1.value = reg
        await RisingEdge(dut.CLK)  # Allow combinational output to settle
        val = int(dut.DR_OUT_1.value)
        assert val == 0, f"D{reg} should be 0 after reset, got 0x{val:08X}"

    dut._log.info("PASS: All registers cleared after reset")


@cocotb.test()
async def test_write_long_read_back(dut):
    """Write a LONG value to D0, read it back."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    test_val = 0xDEADBEEF
    await write_reg(dut, 1, 0, test_val, LONG)

    dut.DR_SEL_RD_1.value = 0
    await RisingEdge(dut.CLK)
    result = int(dut.DR_OUT_1.value)
    assert result == test_val, f"D0 should be 0x{test_val:08X}, got 0x{result:08X}"
    dut._log.info(f"PASS: D0 = 0x{result:08X}")


@cocotb.test()
async def test_write_word_preserves_upper(dut):
    """Write D3 with LONG, then overwrite lower WORD, verify upper half unchanged."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Write full long to D3
    await write_reg(dut, 1, 3, 0xAABBCCDD, LONG)

    # Overwrite only the lower word
    await write_reg(dut, 1, 3, 0x00001234, WORD)

    dut.DR_SEL_RD_1.value = 3
    await RisingEdge(dut.CLK)
    result = int(dut.DR_OUT_1.value)

    expected = 0xAABB1234
    assert result == expected, f"D3 should be 0x{expected:08X}, got 0x{result:08X}"
    dut._log.info(f"PASS: D3 = 0x{result:08X} (upper half preserved)")


@cocotb.test()
async def test_write_byte_preserves_upper24(dut):
    """Write D7 with LONG, then overwrite lowest BYTE, verify upper 24 bits unchanged."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Write full long to D7
    await write_reg(dut, 1, 7, 0x11223344, LONG)

    # Overwrite only the lowest byte
    await write_reg(dut, 1, 7, 0x000000FF, BYTE)

    dut.DR_SEL_RD_1.value = 7
    await RisingEdge(dut.CLK)
    result = int(dut.DR_OUT_1.value)

    expected = 0x112233FF
    assert result == expected, f"D7 should be 0x{expected:08X}, got 0x{result:08X}"
    dut._log.info(f"PASS: D7 = 0x{result:08X} (upper 24 bits preserved)")


@cocotb.test()
async def test_hazard_detection(dut):
    """Test DR_IN_USE signal: marks a register as in-use, then checks if reading it triggers the hazard."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Mark register D2 as in-use via write port 1
    dut.DR_SEL_WR_1.value = 2
    dut.DR_MARK_USED.value = 1
    await RisingEdge(dut.CLK)
    dut.DR_MARK_USED.value = 0
    await RisingEdge(dut.CLK)

    # Now try to read D2 -- DR_IN_USE should be asserted
    dut.DR_SEL_RD_1.value = 2
    await RisingEdge(dut.CLK)

    in_use = int(dut.DR_IN_USE.value)
    assert in_use == 1, f"DR_IN_USE should be 1 when reading a marked register, got {in_use}"
    dut._log.info("PASS: DR_IN_USE asserted for marked register")

    # Unmark and verify DR_IN_USE clears
    dut.UNMARK.value = 1
    await RisingEdge(dut.CLK)
    dut.UNMARK.value = 0
    await RisingEdge(dut.CLK)

    in_use = int(dut.DR_IN_USE.value)
    assert in_use == 0, f"DR_IN_USE should be 0 after UNMARK, got {in_use}"
    dut._log.info("PASS: DR_IN_USE cleared after UNMARK")


@cocotb.test()
async def test_dual_port_write(dut):
    """Write two different registers simultaneously via both write ports."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Mark both write targets
    dut.DR_SEL_WR_1.value = 4
    dut.DR_SEL_WR_2.value = 5
    dut.USE_DPAIR.value = 1
    dut.DR_MARK_USED.value = 1
    await RisingEdge(dut.CLK)
    dut.DR_MARK_USED.value = 0
    dut.USE_DPAIR.value = 0

    # Write both
    dut.OP_SIZE.value = LONG
    dut.DR_IN_1.value = 0xAAAAAAAA
    dut.DR_IN_2.value = 0x55555555
    dut.DR_WR_1.value = 1
    dut.DR_WR_2.value = 1
    await RisingEdge(dut.CLK)
    dut.DR_WR_1.value = 0
    dut.DR_WR_2.value = 0
    await RisingEdge(dut.CLK)

    # Read back
    dut.DR_SEL_RD_1.value = 4
    dut.DR_SEL_RD_2.value = 5
    await RisingEdge(dut.CLK)

    val4 = int(dut.DR_OUT_1.value)
    val5 = int(dut.DR_OUT_2.value)

    assert val4 == 0xAAAAAAAA, f"D4 should be 0xAAAAAAAA, got 0x{val4:08X}"
    assert val5 == 0x55555555, f"D5 should be 0x55555555, got 0x{val5:08X}"
    dut._log.info(f"PASS: Dual port write -- D4=0x{val4:08X}, D5=0x{val5:08X}")
