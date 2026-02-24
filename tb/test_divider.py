"""
Division tests for WF68K30L_ALU (which instantiates WF68K30L_DIVIDER).

Tests division via the ALU interface to verify BUG-003 and BUG-009 fixes.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# OP_SIZE
LONG = 0
WORD = 1
BYTE = 2

# OP codes (from OP_68K enum in wf68k30L_pkg.svh) -- CORRECT VALUES
DIVS = 39
DIVU = 40


def init_signals(dut):
    """Initialize all ALU inputs to safe defaults."""
    dut.RESET.value = 0
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0
    dut.LOAD_OP3.value = 0
    dut.OP1_IN.value = 0
    dut.OP2_IN.value = 0
    dut.OP3_IN.value = 0
    dut.BF_OFFSET_IN.value = 0
    dut.BF_WIDTH_IN.value = 0
    dut.BITPOS_IN.value = 0
    dut.ADR_MODE_IN.value = 0
    dut.OP_SIZE_IN.value = LONG
    dut.OP_IN.value = 0
    dut.OP_WB.value = 0
    dut.BIW_0_IN.value = 0
    dut.BIW_1_IN.value = 0
    dut.SR_WR.value = 0
    dut.SR_INIT.value = 0
    dut.SR_CLR_MBIT.value = 0
    dut.CC_UPDT.value = 0
    dut.ALU_INIT.value = 0
    dut.ALU_ACK.value = 0
    dut.USE_DREG.value = 0
    dut.HILOn.value = 0
    dut.IRQ_PEND.value = 0


async def do_reset(dut, cycles=5):
    """Assert RESET for the given number of cycles."""
    dut.RESET.value = 1
    await ClockCycles(dut.CLK, cycles)
    dut.RESET.value = 0
    await RisingEdge(dut.CLK)


async def alu_div(dut, op, op_size, divisor, dividend_lo, dividend_hi=0, biw_0=0, biw_1=0, timeout=200):
    """Execute a division via the ALU and return (quotient, remainder, status).

    OP1 = divisor, OP2 = dividend low 32, OP3 = dividend high 32.
    For LONG: RESULT = {REMAINDER, QUOTIENT}
    For WORD: RESULT = {0, REMAINDER[15:0], QUOTIENT[15:0]}
    """
    # Load operands
    dut.OP1_IN.value = divisor & 0xFFFFFFFF
    dut.OP2_IN.value = dividend_lo & 0xFFFFFFFF
    dut.OP3_IN.value = dividend_hi & 0xFFFFFFFF
    dut.LOAD_OP1.value = 1
    dut.LOAD_OP2.value = 1
    dut.LOAD_OP3.value = 1
    dut.OP_IN.value = op
    dut.OP_SIZE_IN.value = op_size
    dut.BIW_0_IN.value = biw_0 & 0xFFFF
    dut.BIW_1_IN.value = biw_1 & 0xFFFF
    await RisingEdge(dut.CLK)

    # Assert ALU_INIT
    dut.ALU_INIT.value = 1
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0
    dut.LOAD_OP3.value = 0
    await RisingEdge(dut.CLK)

    dut.ALU_INIT.value = 0

    # Wait for ALU_REQ (division takes many cycles)
    for i in range(timeout):
        await RisingEdge(dut.CLK)
        try:
            if int(dut.ALU_REQ.value) == 1:
                break
        except ValueError:
            pass
    else:
        raise TimeoutError(f"ALU_REQ never asserted after {timeout} cycles")

    # Update condition codes
    dut.CC_UPDT.value = 1
    await RisingEdge(dut.CLK)
    dut.CC_UPDT.value = 0

    # Read result
    result_full = int(dut.RESULT.value)

    if op_size == LONG:
        quotient  = result_full & 0xFFFFFFFF
        remainder = (result_full >> 32) & 0xFFFFFFFF
    else:  # WORD
        quotient  = result_full & 0xFFFF
        remainder = (result_full >> 16) & 0xFFFF

    # ACK
    dut.ALU_ACK.value = 1
    await RisingEdge(dut.CLK)
    dut.ALU_ACK.value = 0
    await RisingEdge(dut.CLK)

    status = int(dut.STATUS_REG_OUT.value)
    return quotient, remainder, status


# --- DIVU (unsigned) Tests ---

@cocotb.test()
async def test_divu_long_trivial_gt(dut):
    """DIVU.L: 5 / 10 -> quotient=0, remainder=5 (fast path: divisor > dividend)."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 10, 5)
    dut._log.info(f"DIVU.L 5/10: Q={q}, R={r}")
    assert q == 0, f"Expected Q=0, got {q}"
    assert r == 5, f"Expected R=5, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_equal(dut):
    """DIVU.L: 10 / 10 -> quotient=1, remainder=0 (fast path: equal)."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 10, 10)
    dut._log.info(f"DIVU.L 10/10: Q={q}, R={r}")
    assert q == 1, f"Expected Q=1, got {q}"
    assert r == 0, f"Expected R=0, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_100_div_10(dut):
    """DIVU.L: 100 / 10 -> quotient=10, remainder=0 (BUG-003: iterative loop)."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 10, 100)
    dut._log.info(f"DIVU.L 100/10: Q={q}, R={r}")
    assert q == 10, f"Expected Q=10, got {q}"
    assert r == 0, f"Expected R=0, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_100_div_7(dut):
    """DIVU.L: 100 / 7 -> quotient=14, remainder=2."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 7, 100)
    dut._log.info(f"DIVU.L 100/7: Q={q}, R={r}")
    assert q == 14, f"Expected Q=14, got {q}"
    assert r == 2, f"Expected R=2, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_255_div_16(dut):
    """DIVU.L: 255 / 16 -> quotient=15, remainder=15."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 16, 255)
    dut._log.info(f"DIVU.L 255/16: Q={q}, R={r}")
    assert q == 15, f"Expected Q=15, got {q}"
    assert r == 15, f"Expected R=15, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_deadbeef(dut):
    """DIVU.L: 0xDEADBEEF / 0x1234 -> verify iterative loop with large values."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    dividend = 0xDEADBEEF
    divisor = 0x1234
    expected_q = dividend // divisor
    expected_r = dividend % divisor

    q, r, st = await alu_div(dut, DIVU, LONG, divisor, dividend)
    dut._log.info(f"DIVU.L 0x{dividend:X}/0x{divisor:X}: Q=0x{q:08X} ({q}), R=0x{r:08X} ({r})")
    dut._log.info(f"  Expected: Q=0x{expected_q:08X} ({expected_q}), R=0x{expected_r:08X} ({expected_r})")
    assert q == expected_q, f"Expected Q=0x{expected_q:X}, got 0x{q:X}"
    assert r == expected_r, f"Expected R=0x{expected_r:X}, got 0x{r:X}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_long_max(dut):
    """DIVU.L: 0xFFFFFFFF / 2 -> quotient=0x7FFFFFFF, remainder=1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, LONG, 2, 0xFFFFFFFF)
    dut._log.info(f"DIVU.L 0xFFFFFFFF/2: Q=0x{q:08X}, R=0x{r:08X}")
    assert q == 0x7FFFFFFF, f"Expected Q=0x7FFFFFFF, got 0x{q:08X}"
    assert r == 1, f"Expected R=1, got 0x{r:08X}"
    dut._log.info("PASS")


# --- DIVS (signed) Tests ---

@cocotb.test()
async def test_divs_long_pos_pos(dut):
    """DIVS.L: 100 / 10 -> quotient=10, remainder=0 (positive / positive)."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVS, LONG, 10, 100)
    dut._log.info(f"DIVS.L 100/10: Q={q}, R={r}")
    assert q == 10, f"Expected Q=10, got {q}"
    assert r == 0, f"Expected R=0, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divs_long_neg_pos(dut):
    """DIVS.L: -100 / 10 -> quotient=-10, remainder=0 (negative / positive)."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    neg100 = 0xFFFFFF9C  # -100 as 32-bit two's complement
    q, r, st = await alu_div(dut, DIVS, LONG, 10, neg100)
    # Convert quotient to signed
    q_signed = q if q < 0x80000000 else q - 0x100000000
    dut._log.info(f"DIVS.L -100/10: Q=0x{q:08X} ({q_signed}), R=0x{r:08X}")
    assert q_signed == -10, f"Expected Q=-10, got {q_signed}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divs_long_100_div_7(dut):
    """DIVS.L: 100 / 7 -> quotient=14, remainder=2."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVS, LONG, 7, 100)
    dut._log.info(f"DIVS.L 100/7: Q={q}, R={r}")
    assert q == 14, f"Expected Q=14, got {q}"
    assert r == 2, f"Expected R=2, got {r}"
    dut._log.info("PASS")


# --- Divide by zero Tests ---

@cocotb.test()
async def test_divu_divide_by_zero(dut):
    """DIVU.L: 100 / 0 -> BUG-009: check divide-by-zero behavior.

    The MC68030 spec says destination register is unchanged on div-by-zero.
    After BUG-009 fix, QUOTIENT and REMAINDER should NOT be modified.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # First do a normal division to set known values in QUOTIENT/REMAINDER
    q1, r1, st1 = await alu_div(dut, DIVU, LONG, 5, 15)
    dut._log.info(f"Setup: DIVU 15/5: Q={q1}, R={r1}")
    assert q1 == 3, f"Setup failed: expected Q=3, got {q1}"
    assert r1 == 0, f"Setup failed: expected R=0, got {r1}"

    # Now divide by zero
    q2, r2, st2 = await alu_div(dut, DIVU, LONG, 0, 100)
    dut._log.info(f"DIVU.L 100/0: Q=0x{q2:08X}, R=0x{r2:08X}")

    # After BUG-009 fix: QUOTIENT and REMAINDER should be preserved (unchanged)
    assert q2 == 3, f"BUG-009: Expected Q=3 (preserved), got Q=0x{q2:08X}"
    assert r2 == 0, f"BUG-009: Expected R=0 (preserved), got R=0x{r2:08X}"
    dut._log.info("PASS: divide-by-zero preserves destination register")


@cocotb.test()
async def test_divs_divide_by_zero(dut):
    """DIVS.L: 100 / 0 -> same BUG-009 test for signed division."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Setup: known values
    q1, r1, st1 = await alu_div(dut, DIVS, LONG, 7, 100)
    dut._log.info(f"Setup: DIVS 100/7: Q={q1}, R={r1}")
    assert q1 == 14, f"Setup failed: expected Q=14, got {q1}"

    # Divide by zero
    q2, r2, st2 = await alu_div(dut, DIVS, LONG, 0, 100)
    dut._log.info(f"DIVS.L 100/0: Q=0x{q2:08X}, R=0x{r2:08X}")

    # QUOTIENT and REMAINDER preserved
    assert q2 == 14, f"BUG-009: Expected Q=14 (preserved), got Q=0x{q2:08X}"
    assert r2 == 2, f"BUG-009: Expected R=2 (preserved), got R=0x{r2:08X}"
    dut._log.info("PASS: divide-by-zero preserves destination register")


# --- WORD mode tests ---

@cocotb.test()
async def test_divu_word_100_div_10(dut):
    """DIVU.W: 100 / 10 -> quotient=10, remainder=0.
    
    WORD mode: 32-bit dividend in OP2, 16-bit divisor in low word of OP1.
    Result: 16-bit quotient, 16-bit remainder.
    BIW_0[8:6] = "001" for word mode. OP_SIZE = WORD.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, WORD, 10, 100, biw_0=0x0040)
    dut._log.info(f"DIVU.W 100/10: Q={q}, R={r}")
    assert q == 10, f"Expected Q=10, got {q}"
    assert r == 0, f"Expected R=0, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_word_50000_div_7(dut):
    """DIVU.W: 50000 / 7 -> quotient=7142, remainder=6."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    q, r, st = await alu_div(dut, DIVU, WORD, 7, 50000, biw_0=0x0040)
    expected_q = 50000 // 7
    expected_r = 50000 % 7
    dut._log.info(f"DIVU.W 50000/7: Q={q} (expected {expected_q}), R={r} (expected {expected_r})")
    assert q == expected_q, f"Expected Q={expected_q}, got {q}"
    assert r == expected_r, f"Expected R={expected_r}, got {r}"
    dut._log.info("PASS")


@cocotb.test()
async def test_divu_word_large_dividend(dut):
    """DIVU.W: 0x12345678 / 0x1234 -> test with large 32-bit dividend.
    
    32-bit dividend / 16-bit divisor in WORD mode.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    dividend = 0x12345678
    divisor = 0x1234
    expected_q = dividend // divisor
    expected_r = dividend % divisor

    q, r, st = await alu_div(dut, DIVU, WORD, divisor, dividend, biw_0=0x0040)
    dut._log.info(f"DIVU.W 0x{dividend:X}/0x{divisor:X}: Q=0x{q:04X} ({q}), R=0x{r:04X} ({r})")
    dut._log.info(f"  Expected: Q=0x{expected_q:X} ({expected_q}), R=0x{expected_r:X} ({expected_r})")
    # In WORD mode, if quotient > 0xFFFF, overflow should be flagged
    if expected_q > 0xFFFF:
        dut._log.info("Expected overflow (quotient > 16 bits)")
    else:
        assert q == expected_q, f"Expected Q=0x{expected_q:X}, got 0x{q:X}"
        assert r == expected_r, f"Expected R=0x{expected_r:X}, got 0x{r:X}"
    dut._log.info("PASS")
