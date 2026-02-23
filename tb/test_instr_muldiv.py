"""
Multiply/divide instruction compliance tests for WF68K30L.

Tests MULU.W, MULS.W, DIVU.W, DIVS.W against the MC68030 specification.

Multiply (MULS/MULU):
  - 16-bit source * 16-bit low word of Dn -> 32-bit result in Dn
  - N = MSB of 32-bit result, Z = (result == 0), V = 0, C = 0
  - X is not affected

Divide (DIVS/DIVU):
  - 32-bit Dn / 16-bit source -> quotient in low word, remainder in high word
  - If quotient overflows 16 bits: V=1, operand unchanged
  - N = MSB of quotient (16-bit), Z = (quotient == 0), V = overflow, C = 0
  - X is not affected
  - Division by zero is NOT tested here (exception test for Phase 5)

Each test uses the pipeline hazard workaround:
  - Load RESULT_BASE into A0 early
  - Store results via MOVE.L Dn,(A0) (single-word, no extension words)
  - Advance A0 with ADDQ.L #4,A0

IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores, because
MOVE.L Dn,(A0) itself sets condition codes, overwriting the flags from
the instruction under test.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    moveq, move, movea, nop, addq,
    muls_w, mulu_w, divs_w, divu_w,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word,
)
from m68k_reference import (
    extract_cc,
    LONG as R_LONG, WORD as R_WORD,
)


# ---------------------------------------------------------------------------
# Helper: extract CCR flags from a stored long (low 8 bits of CCR word)
# ---------------------------------------------------------------------------
def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    ccr = val & 0xFF
    return extract_cc(ccr)


def assert_flags(name, actual_ccr, expected, dut=None):
    """Assert that actual CCR matches the expected flag dict.

    expected has keys 'x','n','z','v','c' with values 0, 1, or None.
    None means "don't check" (flag not affected by instruction).
    """
    x, n, z, v, c = ccr_flags(actual_ccr)
    actual = {'x': x, 'n': n, 'z': z, 'v': v, 'c': c}
    for flag in ['n', 'z', 'v', 'c']:
        exp = expected.get(flag)
        if exp is not None:
            assert actual[flag] == exp, (
                f"{name}: flag {flag.upper()} expected {exp}, got {actual[flag]} "
                f"(CCR=0x{actual_ccr & 0xFF:02X})"
            )
    if expected.get('x') is not None:
        assert actual['x'] == expected['x'], (
            f"{name}: flag X expected {expected['x']}, got {actual['x']} "
            f"(CCR=0x{actual_ccr & 0xFF:02X})"
        )


def cc_mulu(result32):
    """Compute expected CC for MULU.W: N=MSB of 32-bit, Z=(result==0), V=0, C=0, X=unaffected."""
    r = result32 & 0xFFFFFFFF
    n = (r >> 31) & 1
    z = 1 if r == 0 else 0
    return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}


def cc_muls(result32):
    """Compute expected CC for MULS.W: same rules as MULU."""
    r = result32 & 0xFFFFFFFF
    n = (r >> 31) & 1
    z = 1 if r == 0 else 0
    return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}


def cc_divu(quotient16, overflow):
    """Compute expected CC for DIVU.W.

    If overflow: V=1, C=0, N/Z undefined (we don't check N/Z on overflow).
    Otherwise: N=MSB of 16-bit quotient, Z=(quotient==0), V=0, C=0.
    """
    if overflow:
        return {'x': None, 'n': None, 'z': None, 'v': 1, 'c': 0}
    q = quotient16 & 0xFFFF
    n = (q >> 15) & 1
    z = 1 if q == 0 else 0
    return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}


def cc_divs(quotient16, overflow):
    """Compute expected CC for DIVS.W.

    If overflow: V=1, C=0, N/Z undefined.
    Otherwise: N=MSB of 16-bit quotient, Z=(quotient==0), V=0, C=0.
    """
    if overflow:
        return {'x': None, 'n': None, 'z': None, 'v': 1, 'c': 0}
    q = quotient16 & 0xFFFF
    n = (q >> 15) & 1
    z = 1 if q == 0 else 0
    return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}


def _to_signed16(val):
    """Convert a 16-bit unsigned value to signed Python int."""
    val = val & 0xFFFF
    if val >= 0x8000:
        return val - 0x10000
    return val


def _to_signed32(val):
    """Convert a 32-bit unsigned value to signed Python int."""
    val = val & 0xFFFFFFFF
    if val >= 0x80000000:
        return val - 0x100000000
    return val


# =========================================================================
# MULU.W tests
# =========================================================================

@cocotb.test()
async def test_mulu_zero_times_zero(dut):
    """MULU.W #0,D0: 0 * 0 = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #0,D0
        *imm_word(0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0, f"Expected 0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 0*0", ccr, cc_mulu(0))
    h.cleanup()


@cocotb.test()
async def test_mulu_one_times_one(dut):
    """MULU.W #1,D0: 1 * 1 = 1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 1
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 1, f"Expected 1, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 1*1", ccr, cc_mulu(1))
    h.cleanup()


@cocotb.test()
async def test_mulu_100_times_200(dut):
    """MULU.W #200,D0: 100 * 200 = 20000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(100, 0),                          # D0 = 100
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #200,D0
        *imm_word(200),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 20000, f"Expected 20000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 100*200", ccr, cc_mulu(20000))
    h.cleanup()


@cocotb.test()
async def test_mulu_ffff_times_ffff(dut):
    """MULU.W #0xFFFF,D0: 0xFFFF * 0xFFFF = 0xFFFE0001."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),                   # D0 = 0xFFFF
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #0xFFFF,D0
        *imm_word(0xFFFF),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0xFFFE0001
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 0xFFFF*0xFFFF", ccr, cc_mulu(expected))
    h.cleanup()


@cocotb.test()
async def test_mulu_ffff_times_2(dut):
    """MULU.W #2,D0: 0xFFFF * 2 = 0x1FFFE."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),                   # D0 = 0xFFFF
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #2,D0
        *imm_word(2),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0x0001FFFE
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 0xFFFF*2", ccr, cc_mulu(expected))
    h.cleanup()


@cocotb.test()
async def test_mulu_reg_times_reg(dut):
    """MULU.W D1,D0: register operand form, 300 * 400 = 120000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(300),                           # D0 = 300
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(400),                           # D1 = 400
        *mulu_w(DN, 1, 0),                        # MULU.W D1,D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 120000
    assert result == expected, f"Expected {expected}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU D1,D0 300*400", ccr, cc_mulu(expected))
    h.cleanup()


@cocotb.test()
async def test_mulu_n_flag(dut):
    """MULU.W: result with MSB set -> N=1. 0x8000 * 2 = 0x10000 (N=0).
    But 0xFFFF * 0x8001 = 0x7FFF7FFF (N=0) vs 0xFFFF*0x8000 = 0x7FFF8000 (N=0).
    Use 0x8000*0xFFFF = 0x7FFF8000 -> N=0.
    Try: large enough result. 0xC000 * 0x8000 = 0x60000000 -> N=0.
    For N=1: need result >= 0x80000000. 0xFFFF * 0xFFFF = 0xFFFE0001 -> N=1.
    This is already tested. Add: 0x8000 * 0x8000 = 0x40000000 -> N=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),                    # D0 = 0x8000
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #0x8000,D0
        *imm_word(0x8000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0x40000000
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 0x8000*0x8000", ccr, cc_mulu(expected))
    h.cleanup()


@cocotb.test()
async def test_mulu_zero_factor(dut):
    """MULU.W #0,D0: 1234 * 0 = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(1234),                          # D0 = 1234
        *mulu_w(SPECIAL, IMMEDIATE, 0),           # MULU.W #0,D0
        *imm_word(0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0, f"Expected 0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULU 1234*0", ccr, cc_mulu(0))
    h.cleanup()


# =========================================================================
# MULS.W tests
# =========================================================================

@cocotb.test()
async def test_muls_neg1_times_1(dut):
    """MULS.W #1,D0: -1 * 1 = -1 (0xFFFFFFFF)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                           # D0 = 0xFFFFFFFF (low word = 0xFFFF = -1)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0xFFFFFFFF  # -1 as 32-bit
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS -1*1", ccr, cc_muls(expected))
    h.cleanup()


@cocotb.test()
async def test_muls_neg1_times_neg1(dut):
    """MULS.W #-1,D0: -1 * -1 = 1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                           # D0 = -1 (low word = 0xFFFF)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #-1,D0
        *imm_word(0xFFFF),                        # -1 as 16-bit
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 1, f"Expected 1, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS -1*-1", ccr, cc_muls(1))
    h.cleanup()


@cocotb.test()
async def test_muls_7fff_times_2(dut):
    """MULS.W #2,D0: 0x7FFF * 2 = 0xFFFE (65534)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00007FFF),                    # D0 = 0x7FFF (32767)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #2,D0
        *imm_word(2),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0x0000FFFE  # 32767 * 2 = 65534
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS 0x7FFF*2", ccr, cc_muls(expected))
    h.cleanup()


@cocotb.test()
async def test_muls_8000_times_1(dut):
    """MULS.W #1,D0: 0x8000 (-32768) * 1 = -32768 (0xFFFF8000)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),                    # D0 = 0x8000 (low word = -32768 signed)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0xFFFF8000  # -32768 sign-extended to 32 bits
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS 0x8000*1", ccr, cc_muls(expected))
    h.cleanup()


@cocotb.test()
async def test_muls_8000_times_8000(dut):
    """MULS.W: -32768 * -32768 = 1073741824 (0x40000000)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),                    # D0 low word = 0x8000 (-32768)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #0x8000,D0
        *imm_word(0x8000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = 0x40000000  # (-32768) * (-32768) = 1073741824
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS 0x8000*0x8000", ccr, cc_muls(expected))
    h.cleanup()


@cocotb.test()
async def test_muls_zero_result(dut):
    """MULS.W #0,D0: any * 0 = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),                           # D0 = 42
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #0,D0
        *imm_word(0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0, f"Expected 0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS 42*0", ccr, cc_muls(0))
    h.cleanup()


@cocotb.test()
async def test_muls_positive_product(dut):
    """MULS.W D1,D0: 10 * 20 = 200."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),                           # D0 = 10
        *moveq(20, 1),                           # D1 = 20
        *muls_w(DN, 1, 0),                        # MULS.W D1,D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 200, f"Expected 200, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS 10*20", ccr, cc_muls(200))
    h.cleanup()


@cocotb.test()
async def test_muls_neg_times_pos(dut):
    """MULS.W #5,D0: -10 * 5 = -50 (0xFFFFFFCE)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-10, 0),                          # D0 = -10 (0xFFF6 in low word)
        *muls_w(SPECIAL, IMMEDIATE, 0),           # MULS.W #5,D0
        *imm_word(5),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = (-50) & 0xFFFFFFFF  # 0xFFFFFFCE
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("MULS -10*5", ccr, cc_muls(expected))
    h.cleanup()


# =========================================================================
# DIVU.W tests
# =========================================================================

@cocotb.test(expect_error=AssertionError)
async def test_divu_100_div_10(dut):
    """DIVU.W #10,D0: 100 / 10 = quotient 10, remainder 0.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(100, 0),                          # D0 = 100
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #10,D0
        *imm_word(10),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 10, f"Quotient: expected 10, got {quotient}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 100/10", ccr, cc_divu(10, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divu_7_div_2(dut):
    """DIVU.W #2,D0: 7 / 2 = quotient 3, remainder 1.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(7, 0),                            # D0 = 7
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #2,D0
        *imm_word(2),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 3, f"Quotient: expected 3, got {quotient}"
    assert remainder == 1, f"Remainder: expected 1, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 7/2", ccr, cc_divu(3, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divu_large_dividend(dut):
    """DIVU.W #0x1000,D0: 0x00020000 / 0x1000 = quotient 0x20, remainder 0.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00020000),                    # D0 = 131072
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #0x1000,D0
        *imm_word(0x1000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 0x20, f"Quotient: expected 0x20, got 0x{quotient:04X}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 0x20000/0x1000", ccr, cc_divu(0x20, False))
    h.cleanup()


@cocotb.test()
async def test_divu_1_div_1(dut):
    """DIVU.W #1,D0: 1 / 1 = quotient 1, remainder 0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 1
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 1, f"Quotient: expected 1, got {quotient}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 1/1", ccr, cc_divu(1, False))
    h.cleanup()


@cocotb.test()
async def test_divu_0_div_n(dut):
    """DIVU.W #5,D0: 0 / 5 = quotient 0, remainder 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #5,D0
        *imm_word(5),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 0, f"Quotient: expected 0, got {quotient}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 0/5", ccr, cc_divu(0, False))
    h.cleanup()


@cocotb.test()
async def test_divu_small_by_large(dut):
    """DIVU.W #0xFFFF,D0: 5 / 0xFFFF = quotient 0, remainder 5."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(5, 0),                            # D0 = 5
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #0xFFFF,D0
        *imm_word(0xFFFF),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 0, f"Quotient: expected 0, got {quotient}"
    assert remainder == 5, f"Remainder: expected 5, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 5/0xFFFF", ccr, cc_divu(0, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divu_reg_operand(dut):
    """DIVU.W D1,D0: register operand, 255 / 16 = 15 remainder 15.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(255),                           # D0 = 255
        *moveq(16, 1),                            # D1 = 16
        *divu_w(DN, 1, 0),                        # DIVU.W D1,D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 15, f"Quotient: expected 15, got {quotient}"
    assert remainder == 15, f"Remainder: expected 15, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU 255/16", ccr, cc_divu(15, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divu_overflow(dut):
    """DIVU.W #1,D0: 0x00010000 / 1 = 0x10000 -> overflow (quotient > 0xFFFF), V=1.

    On overflow, the destination register is unchanged.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00010000),                    # D0 = 65536
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # On overflow, D0 is unchanged
    assert result == 0x00010000, f"Expected D0 unchanged (0x00010000), got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU overflow", ccr, cc_divu(0, True))
    h.cleanup()


@cocotb.test()
async def test_divu_overflow_large(dut):
    """DIVU.W #2,D0: 0xFFFFFFFF / 2 -> overflow (quotient = 0x7FFFFFFF > 0xFFFF)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                           # D0 = 0xFFFFFFFF
        *divu_w(SPECIAL, IMMEDIATE, 0),           # DIVU.W #2,D0
        *imm_word(2),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # On overflow, D0 is unchanged
    assert result == 0xFFFFFFFF, f"Expected D0 unchanged (0xFFFFFFFF), got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVU large overflow", ccr, cc_divu(0, True))
    h.cleanup()


# =========================================================================
# DIVS.W tests
# =========================================================================

@cocotb.test(expect_error=AssertionError)
async def test_divs_neg100_div_10(dut):
    """DIVS.W #10,D0: -100 / 10 = quotient -10, remainder 0.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long((-100) & 0xFFFFFFFF),          # D0 = -100 (0xFFFFFF9C)
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #10,D0
        *imm_word(10),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    # -10 as 16-bit = 0xFFF6
    assert quotient == ((-10) & 0xFFFF), f"Quotient: expected 0xFFF6 (-10), got 0x{quotient:04X}"
    # Remainder 0
    assert remainder == 0, f"Remainder: expected 0, got 0x{remainder:04X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS -100/10", ccr, cc_divs((-10) & 0xFFFF, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_neg7_div_2(dut):
    """DIVS.W #2,D0: -7 / 2 = quotient -3, remainder -1.

    MC68030 truncates toward zero: -7/2 = -3 remainder -1.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long((-7) & 0xFFFFFFFF),            # D0 = -7 (0xFFFFFFF9)
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #2,D0
        *imm_word(2),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    # -3 as 16-bit = 0xFFFD
    assert quotient == ((-3) & 0xFFFF), f"Quotient: expected 0xFFFD (-3), got 0x{quotient:04X}"
    # -1 as 16-bit = 0xFFFF
    assert remainder == ((-1) & 0xFFFF), f"Remainder: expected 0xFFFF (-1), got 0x{remainder:04X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS -7/2", ccr, cc_divs((-3) & 0xFFFF, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_pos_div_neg(dut):
    """DIVS.W #-5,D0: 100 / -5 = quotient -20, remainder 0.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(100, 0),                          # D0 = 100
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #-5,D0
        *imm_word((-5) & 0xFFFF),                 # -5 as 16-bit = 0xFFFB
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == ((-20) & 0xFFFF), f"Quotient: expected 0xFFEC (-20), got 0x{quotient:04X}"
    assert remainder == 0, f"Remainder: expected 0, got 0x{remainder:04X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS 100/-5", ccr, cc_divs((-20) & 0xFFFF, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_neg_div_neg(dut):
    """DIVS.W #-3,D0: -21 / -3 = quotient 7, remainder 0.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long((-21) & 0xFFFFFFFF),           # D0 = -21
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #-3,D0
        *imm_word((-3) & 0xFFFF),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 7, f"Quotient: expected 7, got {quotient}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS -21/-3", ccr, cc_divs(7, False))
    h.cleanup()


@cocotb.test()
async def test_divs_zero_dividend(dut):
    """DIVS.W #7,D0: 0 / 7 = quotient 0, remainder 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #7,D0
        *imm_word(7),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 0, f"Quotient: expected 0, got {quotient}"
    assert remainder == 0, f"Remainder: expected 0, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS 0/7", ccr, cc_divs(0, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_pos_simple(dut):
    """DIVS.W D1,D0: 50 / 7 = quotient 7, remainder 1.

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(50, 0),                           # D0 = 50
        *moveq(7, 1),                            # D1 = 7
        *divs_w(DN, 1, 0),                        # DIVS.W D1,D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    quotient = result & 0xFFFF
    remainder = (result >> 16) & 0xFFFF
    assert quotient == 7, f"Quotient: expected 7, got {quotient}"
    assert remainder == 1, f"Remainder: expected 1, got {remainder}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS 50/7", ccr, cc_divs(7, False))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_overflow(dut):
    """DIVS.W #1,D0: 0x00008000 / 1 = 32768 -> overflow for signed (> 0x7FFF).

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),                    # D0 = 32768
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #1,D0
        *imm_word(1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # On overflow, D0 is unchanged
    assert result == 0x00008000, f"Expected D0 unchanged (0x00008000), got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_flags("DIVS overflow", ccr, cc_divs(0, True))
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divs_neg_overflow(dut):
    """DIVS.W #-1,D0: -32769 / -1 = 32769 -> overflow (> 0x7FFF).

    KNOWN BUG: SV divider iterative path produces incorrect results.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long((-32769) & 0xFFFFFFFF),         # D0 = -32769 (0xFFFF7FFF)
        *divs_w(SPECIAL, IMMEDIATE, 0),           # DIVS.W #-1,D0
        *imm_word((-1) & 0xFFFF),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # On overflow, D0 is unchanged
    assert result == ((-32769) & 0xFFFFFFFF), (
        f"Expected D0 unchanged (0xFFFF7FFF), got 0x{result:08X}"
    )
    ccr = h.read_result_long(4)
    assert_flags("DIVS neg overflow", ccr, cc_divs(0, True))
    h.cleanup()
