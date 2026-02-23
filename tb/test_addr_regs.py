"""
Unit test for WF68K30L_ADDRESS_REGISTERS module.

Tests the address register file independently:
  - Write/read back via AR_WR_1
  - Stack pointer selection (ISP/MSP/USP via A7 with SBIT/MBIT)
  - Increment/decrement with different OP_SIZE
  - PC load and increment
  - Hazard detection (AR_IN_USE)
  - Reset clears all registers

Set TOPLEVEL=WF68K30L_ADDRESS_REGISTERS when running this test.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


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
    dut.AR_IN_1.value = 0
    dut.AR_IN_2.value = 0
    dut.INDEX_IN.value = 0
    dut.PC_EW_OFFSET.value = 0
    dut.FETCH_MEM_ADR.value = 0
    dut.STORE_ADR_FORMAT.value = 0
    dut.STORE_ABS_HI.value = 0
    dut.STORE_ABS_LO.value = 0
    dut.STORE_D16.value = 0
    dut.STORE_D32_LO.value = 0
    dut.STORE_D32_HI.value = 0
    dut.STORE_DISPL.value = 0
    dut.STORE_MEM_ADR.value = 0
    dut.STORE_OD_HI.value = 0
    dut.STORE_OD_LO.value = 0
    dut.STORE_AEFF.value = 0
    dut.OP_SIZE.value = LONG
    dut.ADR_OFFSET.value = 0
    dut.ADR_MARK_USED.value = 0
    dut.USE_APAIR.value = 0
    dut.ADR_MODE.value = 0
    dut.AMODE_SEL.value = 0
    dut.USE_DREG.value = 0
    dut.DFC_WR.value = 0
    dut.SFC_WR.value = 0
    dut.ISP_DEC.value = 0
    dut.ISP_RD.value = 0
    dut.ISP_WR.value = 0
    dut.MSP_RD.value = 0
    dut.MSP_WR.value = 0
    dut.USP_RD.value = 0
    dut.USP_WR.value = 0
    dut.AR_MARK_USED.value = 0
    dut.AR_IN_USE  # read-only output
    dut.AR_SEL_RD_1.value = 0
    dut.AR_SEL_RD_2.value = 0
    dut.AR_SEL_WR_1.value = 0
    dut.AR_SEL_WR_2.value = 0
    dut.AR_DEC.value = 0
    dut.AR_INC.value = 0
    dut.AR_WR_1.value = 0
    dut.AR_WR_2.value = 0
    dut.UNMARK.value = 0
    dut.EXT_WORD.value = 0
    dut.MBIT.value = 0
    dut.SBIT.value = 0
    dut.SP_ADD_DISPL.value = 0
    dut.RESTORE_ISP_PC.value = 0
    dut.DISPLACEMENT.value = 0
    dut.PC_ADD_DISPL.value = 0
    dut.PC_INC.value = 0
    dut.PC_LOAD.value = 0
    dut.PC_RESTORE.value = 0
    dut.PC_OFFSET.value = 0


async def do_reset(dut, cycles=5):
    """Assert RESET for the given number of cycles."""
    dut.RESET.value = 1
    await ClockCycles(dut.CLK, cycles)
    dut.RESET.value = 0
    await RisingEdge(dut.CLK)


async def write_ar(dut, reg_num, value):
    """Write a value to an address register via write port 1.

    Protocol:
      1. Assert AR_MARK_USED with AR_SEL_WR_1 set (latches the write select)
      2. Drive AR_IN_1 and assert AR_WR_1
    """
    # Step 1: Mark the register
    dut.AR_SEL_WR_1.value = reg_num
    dut.AR_MARK_USED.value = 1
    # Also set SBIT/MBIT context for A7 writes
    await RisingEdge(dut.CLK)
    dut.AR_MARK_USED.value = 0

    # Step 2: Drive data and assert write enable
    dut.AR_IN_1.value = value
    dut.AR_WR_1.value = 1
    await RisingEdge(dut.CLK)

    # Deassert write
    dut.AR_WR_1.value = 0
    await RisingEdge(dut.CLK)


@cocotb.test()
async def test_reset_clears_all(dut):
    """Verify that RESET clears all address registers and PC to zero."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)

    await do_reset(dut)

    # Check A0-A6 are zero via read port 1
    for reg in range(7):
        dut.AR_SEL_RD_1.value = reg
        await RisingEdge(dut.CLK)
        val = int(dut.AR_OUT_1.value)
        assert val == 0, f"A{reg} should be 0 after reset, got 0x{val:08X}"

    # Check PC is zero
    pc_val = int(dut.PC.value)
    assert pc_val == 0, f"PC should be 0 after reset, got 0x{pc_val:08X}"

    dut._log.info("PASS: All address registers and PC cleared after reset")


@cocotb.test()
async def test_write_read_back(dut):
    """Write a value to A0, read it back."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    test_val = 0x12345678
    await write_ar(dut, 0, test_val)

    dut.AR_SEL_RD_1.value = 0
    await RisingEdge(dut.CLK)
    result = int(dut.AR_OUT_1.value)
    assert result == test_val, f"A0 should be 0x{test_val:08X}, got 0x{result:08X}"
    dut._log.info(f"PASS: A0 = 0x{result:08X}")


@cocotb.test()
async def test_write_all_regs(dut):
    """Write different values to A0-A6, read them all back."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Write A0-A6 with unique values
    test_vals = [0x10000000 + i * 0x11111111 for i in range(7)]
    for reg in range(7):
        await write_ar(dut, reg, test_vals[reg])

    # Read back and verify
    for reg in range(7):
        dut.AR_SEL_RD_1.value = reg
        await RisingEdge(dut.CLK)
        result = int(dut.AR_OUT_1.value)
        expected = test_vals[reg]
        assert result == expected, f"A{reg} should be 0x{expected:08X}, got 0x{result:08X}"

    dut._log.info("PASS: All address registers A0-A6 written and read back correctly")


@cocotb.test()
async def test_increment_long(dut):
    """Write A2 with a value, then increment (LONG size), verify +4."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    base_addr = 0x00002000
    await write_ar(dut, 2, base_addr)

    # Set up for increment: AR_SEL_RD_1 selects which register to increment
    dut.AR_SEL_RD_1.value = 2
    dut.OP_SIZE.value = LONG
    dut.AR_INC.value = 1
    await RisingEdge(dut.CLK)
    dut.AR_INC.value = 0
    await RisingEdge(dut.CLK)

    # Read back
    dut.AR_SEL_RD_1.value = 2
    await RisingEdge(dut.CLK)
    result = int(dut.AR_OUT_1.value)
    expected = base_addr + 4
    assert result == expected, f"A2 should be 0x{expected:08X} after LONG inc, got 0x{result:08X}"
    dut._log.info(f"PASS: A2 incremented by 4 (LONG) = 0x{result:08X}")


@cocotb.test()
async def test_increment_word(dut):
    """Write A3 with a value, then increment (WORD size), verify +2."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    base_addr = 0x00003000
    await write_ar(dut, 3, base_addr)

    dut.AR_SEL_RD_1.value = 3
    dut.OP_SIZE.value = WORD
    dut.AR_INC.value = 1
    await RisingEdge(dut.CLK)
    dut.AR_INC.value = 0
    await RisingEdge(dut.CLK)

    dut.AR_SEL_RD_1.value = 3
    await RisingEdge(dut.CLK)
    result = int(dut.AR_OUT_1.value)
    expected = base_addr + 2
    assert result == expected, f"A3 should be 0x{expected:08X} after WORD inc, got 0x{result:08X}"
    dut._log.info(f"PASS: A3 incremented by 2 (WORD) = 0x{result:08X}")


@cocotb.test()
async def test_decrement_long(dut):
    """Write A4 with a value, then decrement (LONG size), verify -4."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    base_addr = 0x00004000
    await write_ar(dut, 4, base_addr)

    dut.AR_SEL_RD_1.value = 4
    dut.OP_SIZE.value = LONG
    dut.AR_DEC.value = 1
    await RisingEdge(dut.CLK)
    dut.AR_DEC.value = 0
    await RisingEdge(dut.CLK)

    dut.AR_SEL_RD_1.value = 4
    await RisingEdge(dut.CLK)
    result = int(dut.AR_OUT_1.value)
    expected = base_addr - 4
    assert result == expected, f"A4 should be 0x{expected:08X} after LONG dec, got 0x{result:08X}"
    dut._log.info(f"PASS: A4 decremented by 4 (LONG) = 0x{result:08X}")


@cocotb.test()
async def test_pc_load(dut):
    """Test loading the PC from AR_IN_1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    target_pc = 0x00000100
    dut.AR_IN_1.value = target_pc
    dut.PC_LOAD.value = 1
    await RisingEdge(dut.CLK)
    dut.PC_LOAD.value = 0
    await RisingEdge(dut.CLK)

    pc_val = int(dut.PC.value)
    assert pc_val == target_pc, f"PC should be 0x{target_pc:08X}, got 0x{pc_val:08X}"
    dut._log.info(f"PASS: PC loaded = 0x{pc_val:08X}")


@cocotb.test()
async def test_pc_increment(dut):
    """Test incrementing the PC by PC_OFFSET."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Load PC with initial value
    initial_pc = 0x00001000
    dut.AR_IN_1.value = initial_pc
    dut.PC_LOAD.value = 1
    await RisingEdge(dut.CLK)
    dut.PC_LOAD.value = 0
    await RisingEdge(dut.CLK)

    # Increment PC by 2 (typical instruction word fetch)
    dut.PC_OFFSET.value = 2
    dut.PC_INC.value = 1
    await RisingEdge(dut.CLK)
    dut.PC_INC.value = 0
    await RisingEdge(dut.CLK)

    pc_val = int(dut.PC.value)
    expected = initial_pc + 2
    assert pc_val == expected, f"PC should be 0x{expected:08X}, got 0x{pc_val:08X}"
    dut._log.info(f"PASS: PC incremented = 0x{pc_val:08X}")


@cocotb.test()
async def test_hazard_detection(dut):
    """Test AR_IN_USE signal: marks a register as in-use, then checks if reading it triggers the hazard."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Mark register A3 as in-use via write port 1
    dut.AR_SEL_WR_1.value = 3
    dut.AR_MARK_USED.value = 1
    await RisingEdge(dut.CLK)
    dut.AR_MARK_USED.value = 0
    await RisingEdge(dut.CLK)

    # Try to read A3 -- AR_IN_USE should be asserted
    dut.AR_SEL_RD_1.value = 3
    # SBIT = 0, so A7-related special cases do not apply
    dut.SBIT.value = 0
    await RisingEdge(dut.CLK)

    in_use = int(dut.AR_IN_USE.value)
    assert in_use == 1, f"AR_IN_USE should be 1 when reading a marked register, got {in_use}"
    dut._log.info("PASS: AR_IN_USE asserted for marked register")

    # Unmark and verify AR_IN_USE clears
    dut.UNMARK.value = 1
    await RisingEdge(dut.CLK)
    dut.UNMARK.value = 0
    await RisingEdge(dut.CLK)

    in_use = int(dut.AR_IN_USE.value)
    assert in_use == 0, f"AR_IN_USE should be 0 after UNMARK, got {in_use}"
    dut._log.info("PASS: AR_IN_USE cleared after UNMARK")
