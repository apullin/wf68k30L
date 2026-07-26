"""
Multiply/divide instruction compliance tests for WF68K30L.

Tests MULU.W, MULS.W, DIVU.W, DIVS.W and the MC68020+ 32-bit forms
MULU.L, MULS.L, DIVU.L, DIVS.L, DIVUL.L and DIVSL.L against the MC68030
specification.

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

Each test uses a consistent result-store pattern:
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
    moveq, move, movea, move_to_abs_long, nop, addq,
    muls_w, mulu_w, divs_w, divu_w,
    muls_l, mulu_l, divs_l, divu_l,
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

@cocotb.test()
async def test_divu_100_div_10(dut):
    """DIVU.W #10,D0: 100 / 10 = quotient 10, remainder 0."""
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


@cocotb.test()
async def test_divu_7_div_2(dut):
    """DIVU.W #2,D0: 7 / 2 = quotient 3, remainder 1."""
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


@cocotb.test()
async def test_divu_large_dividend(dut):
    """DIVU.W #0x1000,D0: 0x00020000 / 0x1000 = quotient 0x20, remainder 0."""
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


@cocotb.test()
async def test_divu_reg_operand(dut):
    """DIVU.W D1,D0: register operand, 255 / 16 = 15 remainder 15."""
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


@cocotb.test()
async def test_divu_overflow(dut):
    """DIVU.W #1,D0: 0x00010000 / 1 = 0x10000 -> overflow (quotient > 0xFFFF), V=1.

    On overflow, the destination register is unchanged."""
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

@cocotb.test()
async def test_divs_neg100_div_10(dut):
    """DIVS.W #10,D0: -100 / 10 = quotient -10, remainder 0."""
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


@cocotb.test()
async def test_divs_neg7_div_2(dut):
    """DIVS.W #2,D0: -7 / 2 = quotient -3, remainder -1.

    MC68030 truncates toward zero: -7/2 = -3 remainder -1."""
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


@cocotb.test()
async def test_divs_pos_div_neg(dut):
    """DIVS.W #-5,D0: 100 / -5 = quotient -20, remainder 0."""
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


@cocotb.test()
async def test_divs_neg_div_neg(dut):
    """DIVS.W #-3,D0: -21 / -3 = quotient 7, remainder 0."""
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


@cocotb.test()
async def test_divs_pos_simple(dut):
    """DIVS.W D1,D0: 50 / 7 = quotient 7, remainder 1."""
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


@cocotb.test()
async def test_divs_overflow(dut):
    """DIVS.W #1,D0: 0x00008000 / 1 = 32768 -> overflow for signed (> 0x7FFF)."""
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


@cocotb.test()
async def test_divs_neg_overflow(dut):
    """DIVS.W #-1,D0: -32769 / -1 = 32769 -> overflow (> 0x7FFF)."""
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


# =========================================================================
# MULU.L / MULS.L / DIVU.L / DIVS.L / DIVUL.L / DIVSL.L
#
# The 0x4C00 (MUL) and 0x4C40 (DIV) decode paths and the register-pair forms
# had no instruction-level coverage: the divider and the 64-bit multiplier were
# exercised only by driving the ALU ports directly, so nothing had ever fetched
# and decoded one of these opcodes.
#
# PRM MULU/MULS: "In the long form, the multiplier and multiplicand are both
# long-word operands, and the result is either a long word or a quad word."
# "Overflow (V = 1) can occur only when multiplying 32-bit operands to yield a
# 32-bit result.  Overflow occurs if any of the high-order 32 bits of the
# quad-word product are not equal to zero."  (For MULS the test is against the
# sign extension of the low half rather than against zero.)
#
# PRM DIVS/DIVU size field: "0 - 32-bit dividend is in register Dq.  1 - 64-bit
# dividend is in Dr - Dq."  And on Dr: "After the division, this register
# contains the 32-bit remainder.  If Dr and Dq are the same register, only the
# quotient is returned."
#
# Every expected value below was cross-checked against
# `qemu-system-m68k -cpu m68030`.
# =========================================================================

def _load_long_reg(dn, value):
    return [*move(LONG, SPECIAL, IMMEDIATE, DN, dn), *imm_long(value)]


async def _run_muldiv_l(dut, setup, instr, capture_regs, budget=8000):
    """Run a long-form multiply/divide and return (register values, ccr)."""
    h = CPUTestHarness(dut)
    words = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *setup,
        *instr,
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
    ]
    for dn in capture_regs:
        words += [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    words += [*move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    words += h.sentinel_program()
    await h.setup(words)
    found = await h.run_until_sentinel(max_cycles=budget)
    assert found, "Sentinel not reached"
    values = [h.read_result_long(4 * i) for i in range(len(capture_regs))]
    ccr = h.read_result_long(4 * len(capture_regs))
    h.cleanup()
    return values, ccr


# ---- MULU.L / MULS.L, 32x32 -> 32 ----------------------------------------

@cocotb.test()
async def test_mulu_l_32x32_to_32(dut):
    """MULU.L D2,D0: 100 * 200 = 20000 in D0, no overflow."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 100) + _load_long_reg(2, 200),
        mulu_l(DN, 2, 0), [0])
    assert vals[0] == 20000, f"expected 20000, got 0x{vals[0]:08X}"
    assert_flags("MULU.L 100*200", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_mulu_l_32x32_to_32_overflow(dut):
    """MULU.L 0x10000 * 0x10000: the low 32 bits are zero and V is set.

    The high half of the quad-word product is nonzero, so V=1; the returned
    32-bit result is zero, so Z=1 as well.
    """
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x10000) + _load_long_reg(2, 0x10000),
        mulu_l(DN, 2, 0), [0])
    assert vals[0] == 0x00000000, f"expected 0, got 0x{vals[0]:08X}"
    assert_flags("MULU.L overflow", ccr,
                 {'x': None, 'n': 0, 'z': 1, 'v': 1, 'c': 0})


@cocotb.test()
async def test_mulu_l_32x32_to_32_overflow_negative_looking(dut):
    """MULU.L 0xFFFFFFFF * 2 = 0x1_FFFFFFFE: V=1, N from the returned half."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 2),
        mulu_l(DN, 2, 0), [0])
    assert vals[0] == 0xFFFFFFFE, f"expected 0xFFFFFFFE, got 0x{vals[0]:08X}"
    assert_flags("MULU.L 0xFFFFFFFF*2", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 1, 'c': 0})


@cocotb.test()
async def test_muls_l_32x32_to_32_negative(dut):
    """MULS.L D2,D0: -2 * 3 = -6, in range so V=0."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFE) + _load_long_reg(2, 3),
        muls_l(DN, 2, 0), [0])
    assert vals[0] == 0xFFFFFFFA, f"expected 0xFFFFFFFA, got 0x{vals[0]:08X}"
    assert_flags("MULS.L -2*3", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_32x32_to_32_two_negatives(dut):
    """MULS.L -1 * -1 = 1."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 0xFFFFFFFF),
        muls_l(DN, 2, 0), [0])
    assert vals[0] == 1, f"expected 1, got 0x{vals[0]:08X}"
    assert_flags("MULS.L -1*-1", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_32x32_to_32_overflow(dut):
    """MULS.L 0x40000000 * 4 = 0x1_00000000: out of signed 32-bit range, V=1."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x40000000) + _load_long_reg(2, 4),
        muls_l(DN, 2, 0), [0])
    assert vals[0] == 0x00000000, f"expected 0, got 0x{vals[0]:08X}"
    assert_flags("MULS.L overflow", ccr,
                 {'x': None, 'n': 0, 'z': 1, 'v': 1, 'c': 0})


@cocotb.test()
async def test_muls_l_32x32_to_32_zero(dut):
    """MULS.L 0 * -5 = 0 with Z=1 and no overflow."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0) + _load_long_reg(2, 0xFFFFFFFB),
        muls_l(DN, 2, 0), [0])
    assert vals[0] == 0, f"expected 0, got 0x{vals[0]:08X}"
    assert_flags("MULS.L 0*-5", ccr,
                 {'x': None, 'n': 0, 'z': 1, 'v': 0, 'c': 0})


# ---- MULU.L / MULS.L, 32x32 -> 64 ---------------------------------------

@cocotb.test()
async def test_mulu_l_64bit_product(dut):
    """MULU.L D2,D1:D0 with 0x10000 * 0x10000 = 0x00000001_00000000."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x10000) + _load_long_reg(2, 0x10000),
        mulu_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x00000000, f"Dl expected 0, got 0x{vals[0]:08X}"
    assert vals[1] == 0x00000001, f"Dh expected 1, got 0x{vals[1]:08X}"
    # The 64-bit form cannot overflow, and Z/N are taken from all 64 bits.
    assert_flags("MULU.L 64-bit", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_mulu_l_64bit_product_max(dut):
    """MULU.L 0xFFFFFFFF * 0xFFFFFFFF = 0xFFFFFFFE_00000001."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 0xFFFFFFFF),
        mulu_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x00000001, f"Dl expected 1, got 0x{vals[0]:08X}"
    assert vals[1] == 0xFFFFFFFE, f"Dh expected 0xFFFFFFFE, got 0x{vals[1]:08X}"
    assert_flags("MULU.L 64-bit max", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_product_negative(dut):
    """MULS.L D2,D1:D0 with -2 * 3 = 0xFFFFFFFF_FFFFFFFA.

    The upper half must be the sign extension, not a zero-extended product;
    audit finding A2 was exactly this case computing 0x00000002 in Dh.
    """
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFE) + _load_long_reg(2, 3),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0xFFFFFFFA, f"Dl expected 0xFFFFFFFA, got 0x{vals[0]:08X}"
    assert vals[1] == 0xFFFFFFFF, f"Dh expected 0xFFFFFFFF, got 0x{vals[1]:08X}"
    assert_flags("MULS.L 64-bit -2*3", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_product_two_negatives(dut):
    """MULS.L -1 * -1 = 0x00000000_00000001 in the 64-bit form."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 0xFFFFFFFF),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x00000001, f"Dl expected 1, got 0x{vals[0]:08X}"
    assert vals[1] == 0x00000000, f"Dh expected 0, got 0x{vals[1]:08X}"
    assert_flags("MULS.L 64-bit -1*-1", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_product_large_positive(dut):
    """MULS.L 0x7FFFFFFF * 0x7FFFFFFF = 0x3FFFFFFF_00000001."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x7FFFFFFF) + _load_long_reg(2, 0x7FFFFFFF),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x00000001, f"Dl expected 1, got 0x{vals[0]:08X}"
    assert vals[1] == 0x3FFFFFFF, f"Dh expected 0x3FFFFFFF, got 0x{vals[1]:08X}"
    assert_flags("MULS.L 64-bit large", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_product_zero_sets_z(dut):
    """A zero 64-bit product sets Z (all 64 bits are zero)."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0) + _load_long_reg(2, 3),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0 and vals[1] == 0, (
        f"expected zero, got 0x{vals[1]:08X}_{vals[0]:08X}")
    assert_flags("MULS.L 64-bit zero", ccr,
                 {'x': None, 'n': 0, 'z': 1, 'v': 0, 'c': 0})


@cocotb.test()
async def test_mulu_l_64bit_n_comes_from_bit63_not_bit31(dut):
    """MULU.L D2,D1:D0 with 0x80000000 * 1 = 0x00000000_80000000 gives N=0.

    N is the MSB of the whole quad-word product.  Bit 31 of the product is set
    here, so a core that fell back to the 32-bit test would report N=1.
    """
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x80000000) + _load_long_reg(2, 1),
        mulu_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x80000000, f"Dl expected 0x80000000, got 0x{vals[0]:08X}"
    assert vals[1] == 0x00000000, f"Dh expected 0, got 0x{vals[1]:08X}"
    assert_flags("MULU.L 64-bit N from bit 63", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_n_comes_from_bit63_not_bit31(dut):
    """MULS.L 0x40000000 * 2 = 0x00000000_80000000 is positive, so N=0."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x40000000) + _load_long_reg(2, 2),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0x80000000, f"Dl expected 0x80000000, got 0x{vals[0]:08X}"
    assert vals[1] == 0x00000000, f"Dh expected 0, got 0x{vals[1]:08X}"
    assert_flags("MULS.L 64-bit N from bit 63", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_muls_l_64bit_negative_product_sets_n(dut):
    """MULS.L -1 * 0x100000 = 0xFFFFFFFF_FFF00000 sets N from bit 63."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 0x100000),
        muls_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0xFFF00000, f"Dl expected 0xFFF00000, got 0x{vals[0]:08X}"
    assert vals[1] == 0xFFFFFFFF, f"Dh expected 0xFFFFFFFF, got 0x{vals[1]:08X}"
    assert_flags("MULS.L 64-bit negative", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_mulu_l_64bit_immediate_source(dut):
    """MULU.L #$10,D1:D0 with D0 = 0x12345678 -> 0x00000001_23456780.

    Exercises the immediate operand of the 0x4C00 decode path.
    """
    words = mulu_l(SPECIAL, IMMEDIATE, 0, 1) + imm_long(0x10)
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0x12345678), words, [0, 1])
    assert vals[0] == 0x23456780, f"Dl expected 0x23456780, got 0x{vals[0]:08X}"
    assert vals[1] == 0x00000001, f"Dh expected 1, got 0x{vals[1]:08X}"
    assert_flags("MULU.L #imm 64-bit", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_mulu_l_memory_source(dut):
    """MULU.L (A1),D0 reads the multiplier from memory."""
    h = CPUTestHarness(dut)
    data = h.DATA_BASE
    words = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(data),
        *_load_long_reg(0, 1000),
        *mulu_l(AN_IND, 1, 0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(words)
    h.mem.write(data, 4, 3000)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 3000000, (
        f"expected 3000000, got 0x{h.read_result_long(0):08X}")
    assert_flags("MULU.L (A1),D0", h.read_result_long(4),
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})
    h.cleanup()


# ---- DIVU.L / DIVS.L, 32/32 -> 32 quotient -------------------------------

@cocotb.test()
async def test_divu_l_32_by_32(dut):
    """DIVU.L D2,D0: 1000 / 7 = 142; the remainder is discarded."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 1000) + _load_long_reg(2, 7),
        divu_l(DN, 2, 0), [0, 2])
    assert vals[0] == 142, f"quotient expected 142, got 0x{vals[0]:08X}"
    assert vals[1] == 7, (
        f"the divisor register must be untouched, got 0x{vals[1]:08X}")
    assert_flags("DIVU.L 1000/7", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divu_l_full_range_quotient(dut):
    """DIVU.L 0xFFFFFFFF / 1 = 0xFFFFFFFF: N follows the quotient's MSB."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFFFF) + _load_long_reg(2, 1),
        divu_l(DN, 2, 0), [0])
    assert vals[0] == 0xFFFFFFFF, f"expected 0xFFFFFFFF, got 0x{vals[0]:08X}"
    assert_flags("DIVU.L 0xFFFFFFFF/1", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divs_l_negative_dividend(dut):
    """DIVS.L D2,D0: -1000 / 7 = -142."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFC18) + _load_long_reg(2, 7),
        divs_l(DN, 2, 0), [0])
    assert vals[0] == 0xFFFFFF72, (
        f"expected 0xFFFFFF72 (-142), got 0x{vals[0]:08X}")
    assert_flags("DIVS.L -1000/7", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divs_l_negative_divisor(dut):
    """DIVS.L D2,D0: 1000 / -7 = -142."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 1000) + _load_long_reg(2, 0xFFFFFFF9),
        divs_l(DN, 2, 0), [0])
    assert vals[0] == 0xFFFFFF72, (
        f"expected 0xFFFFFF72 (-142), got 0x{vals[0]:08X}")
    assert_flags("DIVS.L 1000/-7", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divs_l_both_negative(dut):
    """DIVS.L D2,D0: -1000 / -7 = 142."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFC18) + _load_long_reg(2, 0xFFFFFFF9),
        divs_l(DN, 2, 0), [0])
    assert vals[0] == 142, f"expected 142, got 0x{vals[0]:08X}"
    assert_flags("DIVS.L -1000/-7", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


# ---- DIVUL.L / DIVSL.L, 32/32 -> 32 remainder : 32 quotient --------------

@cocotb.test()
async def test_divul_l_returns_remainder_and_quotient(dut):
    """DIVUL.L D2,D1:D0 with 1000 / 7 puts 142 in D0 (Dq) and 6 in D1 (Dr)."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 1000) + _load_long_reg(2, 7),
        divu_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 142, f"quotient expected 142, got 0x{vals[0]:08X}"
    assert vals[1] == 6, f"remainder expected 6, got 0x{vals[1]:08X}"
    assert_flags("DIVUL.L 1000/7", ccr,
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divsl_l_remainder_takes_dividend_sign(dut):
    """DIVSL.L -1000 / 7 = -142 remainder -6.

    PRM DIVS: "The sign of the remainder is the same as the sign of the
    dividend."
    """
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 0xFFFFFC18) + _load_long_reg(2, 7),
        divs_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0xFFFFFF72, f"quotient expected -142, got 0x{vals[0]:08X}"
    assert vals[1] == 0xFFFFFFFA, f"remainder expected -6, got 0x{vals[1]:08X}"
    assert_flags("DIVSL.L -1000/7", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divsl_l_positive_dividend_negative_divisor(dut):
    """DIVSL.L 1000 / -7 = -142 remainder +6 (the dividend is positive)."""
    vals, ccr = await _run_muldiv_l(
        dut, _load_long_reg(0, 1000) + _load_long_reg(2, 0xFFFFFFF9),
        divs_l(DN, 2, 0, 1), [0, 1])
    assert vals[0] == 0xFFFFFF72, f"quotient expected -142, got 0x{vals[0]:08X}"
    assert vals[1] == 6, f"remainder expected 6, got 0x{vals[1]:08X}"
    assert_flags("DIVSL.L 1000/-7", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


# ---- DIVU.L / DIVS.L, 64/32 -> 32 remainder : 32 quotient ----------------

@cocotb.test()
async def test_divu_l_64_by_32(dut):
    """DIVU.L D2,D1:D0 with the 64-bit dividend 0x00000001_00000000 / 2.

    Dr (D1) supplies the high half of the dividend and receives the remainder;
    Dq (D0) supplies the low half and receives the quotient.
    """
    vals, ccr = await _run_muldiv_l(
        dut,
        _load_long_reg(0, 0x00000000) + _load_long_reg(1, 0x00000001) +
        _load_long_reg(2, 2),
        divu_l(DN, 2, 0, 1, size64=True), [0, 1])
    assert vals[0] == 0x80000000, (
        f"quotient expected 0x80000000, got 0x{vals[0]:08X}")
    assert vals[1] == 0x00000000, f"remainder expected 0, got 0x{vals[1]:08X}"
    assert_flags("DIVU.L 64/32", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divu_l_64_by_32_odd(dut):
    """DIVU.L 0x00000002_00000001 / 3 = 0xAAAAAAAB remainder 0."""
    vals, ccr = await _run_muldiv_l(
        dut,
        _load_long_reg(0, 0x00000001) + _load_long_reg(1, 0x00000002) +
        _load_long_reg(2, 3),
        divu_l(DN, 2, 0, 1, size64=True), [0, 1])
    assert vals[0] == 0xAAAAAAAB, (
        f"quotient expected 0xAAAAAAAB, got 0x{vals[0]:08X}")
    assert vals[1] == 0x00000000, f"remainder expected 0, got 0x{vals[1]:08X}"
    assert_flags("DIVU.L 64/32 odd", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divs_l_64_by_32_negative(dut):
    """DIVS.L with the sign-extended 64-bit dividend -1000 / 7 = -142 rem -6."""
    vals, ccr = await _run_muldiv_l(
        dut,
        _load_long_reg(0, 0xFFFFFC18) + _load_long_reg(1, 0xFFFFFFFF) +
        _load_long_reg(2, 7),
        divs_l(DN, 2, 0, 1, size64=True), [0, 1])
    assert vals[0] == 0xFFFFFF72, f"quotient expected -142, got 0x{vals[0]:08X}"
    assert vals[1] == 0xFFFFFFFA, f"remainder expected -6, got 0x{vals[1]:08X}"
    assert_flags("DIVS.L 64/32 negative", ccr,
                 {'x': None, 'n': 1, 'z': 0, 'v': 0, 'c': 0})


@cocotb.test()
async def test_divu_l_64_by_32_overflow_preserves_registers(dut):
    """DIVU.L 0x00000001_00000000 / 1 overflows: V=1 and both registers survive.

    PRM: "Overflow may be detected and set before the instruction completes.
    If the instruction detects an overflow, it sets the overflow condition
    code, and the operands are unaffected."
    """
    vals, ccr = await _run_muldiv_l(
        dut,
        _load_long_reg(0, 0x00000000) + _load_long_reg(1, 0x00000001) +
        _load_long_reg(2, 1),
        divu_l(DN, 2, 0, 1, size64=True), [0, 1])
    assert vals[0] == 0x00000000, (
        f"Dq must be unaffected on overflow, got 0x{vals[0]:08X}")
    assert vals[1] == 0x00000001, (
        f"Dr must be unaffected on overflow, got 0x{vals[1]:08X}")
    assert_flags("DIVU.L 64/32 overflow", ccr,
                 {'x': None, 'n': None, 'z': None, 'v': 1, 'c': 0})


@cocotb.test()
async def test_divs_l_64_by_32_overflow_preserves_registers(dut):
    """DIVS.L 0x00000001_00000000 / 1 overflows the signed 32-bit quotient."""
    vals, ccr = await _run_muldiv_l(
        dut,
        _load_long_reg(0, 0x00000000) + _load_long_reg(1, 0x00000001) +
        _load_long_reg(2, 1),
        divs_l(DN, 2, 0, 1, size64=True), [0, 1])
    assert vals[0] == 0x00000000, (
        f"Dq must be unaffected on overflow, got 0x{vals[0]:08X}")
    assert vals[1] == 0x00000001, (
        f"Dr must be unaffected on overflow, got 0x{vals[1]:08X}")
    assert_flags("DIVS.L 64/32 overflow", ccr,
                 {'x': None, 'n': None, 'z': None, 'v': 1, 'c': 0})


@cocotb.test()
async def test_divu_l_zero_divisor_traps(dut):
    """DIVU.L by zero takes the zero-divide vector (5)."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000A00
    handler = [
        *moveq(0x05, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    program = [
        *moveq(0x33, 1),                      # marker if no trap happens
        *_load_long_reg(0, 1000),
        *_load_long_reg(2, 0),
        *divu_l(DN, 2, 0),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_long(5 * 4, handler_addr)
    h.mem.load_words(handler_addr, handler)
    found = await h.run_until_sentinel(max_cycles=9000)
    assert found, "DIVU.L zero-divide test did not complete"
    marker = h.read_result_long(0)
    assert marker == 0x05, (
        f"expected the zero-divide handler marker 0x05, got 0x{marker:08X}")
    h.cleanup()


@cocotb.test()
async def test_divs_l_zero_divisor_traps(dut):
    """DIVS.L by zero takes the zero-divide vector (5)."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000A00
    handler = [
        *moveq(0x05, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    program = [
        *moveq(0x33, 1),
        *_load_long_reg(0, 0xFFFFFC18),
        *_load_long_reg(2, 0),
        *divs_l(DN, 2, 0),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_long(5 * 4, handler_addr)
    h.mem.load_words(handler_addr, handler)
    found = await h.run_until_sentinel(max_cycles=9000)
    assert found, "DIVS.L zero-divide test did not complete"
    marker = h.read_result_long(0)
    assert marker == 0x05, (
        f"expected the zero-divide handler marker 0x05, got 0x{marker:08X}")
    h.cleanup()


@cocotb.test()
async def test_divu_l_memory_source(dut):
    """DIVU.L (A1),D0 reads the divisor from memory."""
    h = CPUTestHarness(dut)
    data = h.DATA_BASE
    words = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(data),
        *_load_long_reg(0, 1000000),
        *divu_l(AN_IND, 1, 0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(words)
    h.mem.write(data, 4, 1000)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 1000, (
        f"expected 1000, got 0x{h.read_result_long(0):08X}")
    assert_flags("DIVU.L (A1),D0", h.read_result_long(4),
                 {'x': None, 'n': 0, 'z': 0, 'v': 0, 'c': 0})
    h.cleanup()
