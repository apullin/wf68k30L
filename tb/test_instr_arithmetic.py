"""
Arithmetic instruction compliance tests for WF68K30L.

Tests ADD/ADDI/ADDQ, SUB/SUBI/SUBQ, NEG/NEGX, CLR, CMP/CMPI
against the MC68030 specification (Table 3-12 condition codes).

Each test uses a consistent result-store pattern:
  - Load RESULT_BASE into A0 early
  - Store results via MOVE.L Dn,(A0) (single-word, no extension words)
  - Advance A0 with ADDQ.L #4,A0

IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores, because
MOVE.L Dn,(A0) itself sets condition codes (cc_logic), overwriting
the flags from the instruction under test. The pattern is:
  1. instruction under test
  2. NOP NOP (pipeline drain)
  3. MOVE CCR,D6 (capture flags before any store)
  4. MOVE.L Dn,(A0) / ADDQ / MOVE.L D6,(A0) / ADDQ (store result + CCR)

Uses m68k_reference.py to compute expected condition code values.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq, addi, subi,
    add, sub, neg, negx, clr, cmpi, cmp_reg,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, imm_byte,
)
from m68k_reference import (
    cc_add, cc_sub, cc_neg, cc_negx, cc_logic, cc_cmp,
    mask_val, extract_cc, apply_cc,
    BYTE as R_BYTE, WORD as R_WORD, LONG as R_LONG,
)


# ---------------------------------------------------------------------------
# Helper: extract CCR flags from a stored long (low 8 bits of CCR word)
# ---------------------------------------------------------------------------
def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    # MOVE from CCR stores a 16-bit word; we store as .L so low 16 bits matter
    ccr = val & 0xFF
    return extract_cc(ccr)


def assert_flags(name, actual_ccr, expected_cc, dut=None):
    """Assert that actual CCR matches the expected CC dict.

    expected_cc has keys 'x','n','z','v','c' with values 0, 1, or None.
    None means "don't check" (flag not affected by instruction).
    """
    x, n, z, v, c = ccr_flags(actual_ccr)
    actual = {'x': x, 'n': n, 'z': z, 'v': v, 'c': c}
    for flag in ['n', 'z', 'v', 'c']:
        exp = expected_cc.get(flag)
        if exp is not None:
            assert actual[flag] == exp, (
                f"{name}: flag {flag.upper()} expected {exp}, got {actual[flag]} "
                f"(CCR=0x{actual_ccr & 0xFF:02X})"
            )
    # Only check X if the instruction sets it (not None)
    if expected_cc.get('x') is not None:
        assert actual['x'] == expected_cc['x'], (
            f"{name}: flag X expected {expected_cc['x']}, got {actual['x']} "
            f"(CCR=0x{actual_ccr & 0xFF:02X})"
        )


# =========================================================================
# ADD tests
# =========================================================================

@cocotb.test()
async def test_add_long_reg_reg(dut):
    """ADD.L D0,D1: 10 + 20 = 30 (register to register)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),                        # D0 = 10
        *moveq(20, 1),                        # D1 = 20
        *add(LONG, 1, 0, DN, 0),              # ADD.L D0,D1 (D1 = D1 + D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),                 # capture CCR BEFORE any store
        *move(LONG, DN, 1, AN_IND, 0),        # store D1
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),        # store CCR
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 30, f"Expected 30, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 10, 20, 30)
    assert_flags("ADD.L 10+20", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_add_word_reg_reg(dut):
    """ADD.W D0,D1: word-size addition 0x7000 + 0x1000 = 0x8000 (overflow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00007000),                 # D0 = 0x7000
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00001000),                 # D1 = 0x1000
        *add(WORD, 1, 0, DN, 0),              # ADD.W D0,D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # ADD.W only affects low 16 bits of D1; upper 16 unchanged
    assert (result & 0xFFFF) == 0x8000, f"Expected low word 0x8000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(WORD, 0x7000, 0x1000, 0x8000)
    assert_flags("ADD.W overflow", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_add_byte_reg_reg(dut):
    """ADD.B D0,D1: byte-size addition 0x60 + 0x30 = 0x90 (overflow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x60, 0),                       # D0 = 0x60
        *moveq(0x30, 1),                       # D1 = 0x30
        *add(BYTE, 1, 0, DN, 0),              # ADD.B D0,D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFF) == 0x90, f"Expected low byte 0x90, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(BYTE, 0x60, 0x30, 0x90)
    assert_flags("ADD.B 0x60+0x30", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_add_zero_plus_zero(dut):
    """ADD.L D0,D1: 0+0=0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *moveq(0, 1),
        *add(LONG, 1, 0, DN, 0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
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
    expected_cc = cc_add(LONG, 0, 0, 0)
    assert_flags("ADD.L 0+0", ccr, expected_cc)
    assert expected_cc['z'] == 1, "Z flag should be set for zero result"
    h.cleanup()


@cocotb.test()
async def test_add_positive_overflow(dut):
    """ADD.L: 0x7FFFFFFF + 1 = 0x80000000, V=1 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x7FFFFFFF),
        *moveq(1, 1),
        *add(LONG, 0, 0, DN, 1),              # ADD.L D1,D0 (D0 = D0 + D1)
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
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 1, 0x7FFFFFFF, 0x80000000)
    assert_flags("ADD.L overflow", ccr, expected_cc)
    assert expected_cc['v'] == 1, "V should be set"
    assert expected_cc['n'] == 1, "N should be set"
    h.cleanup()


@cocotb.test()
async def test_add_carry_wrap(dut):
    """ADD.L: 0xFFFFFFFF + 1 = 0, C=1 Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                         # D0 = 0xFFFFFFFF
        *moveq(1, 1),
        *add(LONG, 0, 0, DN, 1),              # ADD.L D1,D0 (D0 = D0 + D1)
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
    expected_cc = cc_add(LONG, 1, 0xFFFFFFFF, 0x100000000)
    assert_flags("ADD.L carry", ccr, expected_cc)
    assert expected_cc['c'] == 1, "C should be set"
    assert expected_cc['z'] == 1, "Z should be set"
    h.cleanup()


# =========================================================================
# ADDI tests
# =========================================================================

@cocotb.test()
async def test_addi_long(dut):
    """ADDI.L #0x100,D0: 0x200 + 0x100 = 0x300."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x200),
        *addi(LONG, DN, 0, 0x100),
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
    assert result == 0x300, f"Expected 0x300, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 0x100, 0x200, 0x300)
    assert_flags("ADDI.L", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_addi_word(dut):
    """ADDI.W #0x0001,D0 with D0=0xFFFE -> low word = 0xFFFF, N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFE),
        *addi(WORD, DN, 0, 0x0001),
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
    assert (result & 0xFFFF) == 0xFFFF, f"Expected low word 0xFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(WORD, 0x0001, 0xFFFE, 0xFFFF)
    assert_flags("ADDI.W", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_addi_byte(dut):
    """ADDI.B #0x80,D0 with D0=0x80 -> byte = 0x00, C=1 Z=1 V=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00000080),
        *addi(BYTE, DN, 0, 0x80),
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
    assert (result & 0xFF) == 0x00, f"Expected low byte 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(BYTE, 0x80, 0x80, 0x100)
    assert_flags("ADDI.B overflow", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# ADDQ tests
# =========================================================================

@cocotb.test()
async def test_addq_long_1(dut):
    """ADDQ.L #1,D0: 0 + 1 = 1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *addq(LONG, 1, DN, 0),
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
    expected_cc = cc_add(LONG, 1, 0, 1)
    assert_flags("ADDQ.L #1", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_addq_long_8(dut):
    """ADDQ.L #8,D0: 10 + 8 = 18."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),
        *addq(LONG, 8, DN, 0),
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
    assert result == 18, f"Expected 18, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 8, 10, 18)
    assert_flags("ADDQ.L #8", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_addq_long_3(dut):
    """ADDQ.L #3,D0: 50 + 3 = 53."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(50, 0),
        *addq(LONG, 3, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 53, f"Expected 53, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_addq_word(dut):
    """ADDQ.W #7,D0: word boundary test 0xFFFC + 7 = 0x10003, low word wraps to 0x0003."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFC),
        *addq(WORD, 7, DN, 0),
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
    # ADDQ.W only affects low 16 bits; upper bits unchanged
    assert (result & 0xFFFF) == 0x0003, f"Expected low word 0x0003, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(WORD, 7, 0xFFFC, 0x10003)
    assert_flags("ADDQ.W carry", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# SUB tests
# =========================================================================

@cocotb.test()
async def test_sub_long_reg_reg(dut):
    """SUB.L D0,D1: D1 = D1 - D0, 30 - 10 = 20."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),                         # D0 = 10 (source)
        *moveq(30, 1),                         # D1 = 30 (dest)
        *sub(LONG, 1, 0, DN, 0),              # SUB.L D0,D1 (D1 = D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 20, f"Expected 20, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 10, 30, 20)
    assert_flags("SUB.L 30-10", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_sub_word_reg_reg(dut):
    """SUB.W D0,D1: word subtraction."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00001000),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00003000),
        *sub(WORD, 1, 0, DN, 0),              # SUB.W D0,D1 (D1 = D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFFFF) == 0x2000, f"Expected low word 0x2000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(WORD, 0x1000, 0x3000, 0x2000)
    assert_flags("SUB.W", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_sub_byte_reg_reg(dut):
    """SUB.B D0,D1: byte subtraction 0x10 - 0x20 = 0xF0 (borrow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x20, 0),                       # D0 = 0x20 (source)
        *moveq(0x10, 1),                       # D1 = 0x10 (dest)
        *sub(BYTE, 1, 0, DN, 0),              # SUB.B D0,D1 (D1 = D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFF) == 0xF0, f"Expected low byte 0xF0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(BYTE, 0x20, 0x10, 0xF0)
    assert_flags("SUB.B borrow", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_sub_zero_minus_one(dut):
    """SUB.L: 0 - 1 = 0xFFFFFFFF, C=1 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                          # D0 = 1 (source)
        *moveq(0, 1),                          # D1 = 0 (dest)
        *sub(LONG, 1, 0, DN, 0),              # SUB.L D0,D1 (D1 = D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 1, 0, 0xFFFFFFFF)
    assert_flags("SUB.L 0-1", ccr, expected_cc)
    assert expected_cc['c'] == 1, "C should be set"
    assert expected_cc['n'] == 1, "N should be set"
    h.cleanup()


@cocotb.test()
async def test_sub_negative_overflow(dut):
    """SUB.L: 0x80000000 - 1 = 0x7FFFFFFF, V=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                          # D0 = 1 (source)
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x80000000),                 # D1 = 0x80000000 (dest)
        *sub(LONG, 1, 0, DN, 0),              # SUB.L D0,D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x7FFFFFFF, f"Expected 0x7FFFFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 1, 0x80000000, 0x7FFFFFFF)
    assert_flags("SUB.L neg overflow", ccr, expected_cc)
    assert expected_cc['v'] == 1, "V should be set"
    h.cleanup()


# =========================================================================
# SUBI tests
# =========================================================================

@cocotb.test()
async def test_subi_long(dut):
    """SUBI.L #0x100,D0: 0x500 - 0x100 = 0x400."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x500),
        *subi(LONG, DN, 0, 0x100),
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
    assert result == 0x400, f"Expected 0x400, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 0x100, 0x500, 0x400)
    assert_flags("SUBI.L", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_subi_word(dut):
    """SUBI.W #0x0001,D0 with D0=0x0000 -> word wraps to 0xFFFF, C=1 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *subi(WORD, DN, 0, 0x0001),
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
    assert (result & 0xFFFF) == 0xFFFF, f"Expected low word 0xFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(WORD, 0x0001, 0x0000, 0xFFFF)
    assert_flags("SUBI.W borrow", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_subi_byte(dut):
    """SUBI.B #0x01,D0 with D0=0x01 -> byte = 0x00, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),
        *subi(BYTE, DN, 0, 0x01),
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
    assert (result & 0xFF) == 0x00, f"Expected low byte 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(BYTE, 0x01, 0x01, 0x00)
    assert_flags("SUBI.B zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


# =========================================================================
# SUBQ tests
# =========================================================================

@cocotb.test()
async def test_subq_long_1(dut):
    """SUBQ.L #1,D0: 10 - 1 = 9."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),
        *subq(LONG, 1, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 9, f"Expected 9, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_subq_long_8(dut):
    """SUBQ.L #8,D0: 20 - 8 = 12."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(20, 0),
        *subq(LONG, 8, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 12, f"Expected 12, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_subq_zero_wrap(dut):
    """SUBQ.L #1,D0 with D0=0 -> 0xFFFFFFFF, C=1 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *subq(LONG, 1, DN, 0),
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
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 1, 0, 0xFFFFFFFF)
    assert_flags("SUBQ.L 0-1", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_subq_word(dut):
    """SUBQ.W #4,D0 with D0=0x0002 -> word wraps 0xFFFE, C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(2, 0),
        *subq(WORD, 4, DN, 0),
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
    assert (result & 0xFFFF) == 0xFFFE, f"Expected low word 0xFFFE, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(WORD, 4, 2, 0xFFFE)
    assert_flags("SUBQ.W borrow", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# NEG tests
# =========================================================================

@cocotb.test()
async def test_neg_long_zero(dut):
    """NEG.L D0 with D0=0 -> 0, Z=1 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *neg(LONG, DN, 0),
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
    expected_cc = cc_neg(LONG, 0, 0)
    assert_flags("NEG.L 0", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert expected_cc['c'] == 0
    h.cleanup()


@cocotb.test()
async def test_neg_long_one(dut):
    """NEG.L D0 with D0=1 -> 0xFFFFFFFF, N=1 C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),
        *neg(LONG, DN, 0),
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
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(LONG, 1, 0xFFFFFFFF)
    assert_flags("NEG.L 1", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert expected_cc['c'] == 1
    h.cleanup()


@cocotb.test()
async def test_neg_long_minint(dut):
    """NEG.L D0 with D0=0x80000000 -> 0x80000000, V=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),
        *neg(LONG, DN, 0),
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
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(LONG, 0x80000000, 0x80000000)
    assert_flags("NEG.L minint", ccr, expected_cc)
    assert expected_cc['v'] == 1
    h.cleanup()


@cocotb.test()
async def test_neg_word(dut):
    """NEG.W D0 with D0=0x0001 -> low word 0xFFFF, N=1 C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),
        *neg(WORD, DN, 0),
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
    assert (result & 0xFFFF) == 0xFFFF, f"Expected low word 0xFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(WORD, 1, 0xFFFF)
    assert_flags("NEG.W 1", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_neg_byte(dut):
    """NEG.B D0 with D0=0x7F -> byte 0x81, N=1 C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x7F, 0),
        *neg(BYTE, DN, 0),
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
    assert (result & 0xFF) == 0x81, f"Expected low byte 0x81, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(BYTE, 0x7F, 0x81)
    assert_flags("NEG.B 0x7F", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# NEGX tests
# =========================================================================

@cocotb.test()
async def test_negx_long_zero_x_clear(dut):
    """NEGX.L D0 with D0=0 and X=0 -> 0, Z unchanged (stays 1 if was 1)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set CCR to 0x04 (Z=1 only, X=0) to test NEGX Z behavior
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x04),
        *move_to_ccr(DN, 5),
        *moveq(0, 0),
        *negx(LONG, DN, 0),
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
    # NEGX: result = 0 - 0 - X(0) = 0; Z = old_z & (result==0) = 1 & 1 = 1
    expected_cc = cc_negx(LONG, 0, 0, 1)
    assert_flags("NEGX.L 0 X=0", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_negx_long_one_x_set(dut):
    """NEGX.L D0 with D0=1 and X=1 -> 0xFFFFFFFE."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set X flag: MOVE #$14,CCR (X=1, Z=1)
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x14),
        *move_to_ccr(DN, 5),
        *moveq(1, 0),
        *negx(LONG, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # NEGX: 0 - 1 - 1(X) = 0xFFFFFFFE
    assert result == 0xFFFFFFFE, f"Expected 0xFFFFFFFE, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# CLR tests
# =========================================================================

@cocotb.test()
async def test_clr_long(dut):
    """CLR.L D0: result=0, Z=1 N=0 V=0 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(99, 0),
        *clr(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0)
    assert_flags("CLR.L", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert expected_cc['n'] == 0
    assert expected_cc['v'] == 0
    assert expected_cc['c'] == 0
    h.cleanup()


@cocotb.test()
async def test_clr_word(dut):
    """CLR.W D0: clears only low word, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *clr(WORD, DN, 0),
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
    # CLR.W clears low 16 bits, upper 16 unaffected
    assert (result & 0xFFFF) == 0, f"Expected low word 0, got 0x{result:08X}"
    assert (result >> 16) == 0x1234, f"Expected high word 0x1234 preserved, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0)
    assert_flags("CLR.W", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_clr_byte(dut):
    """CLR.B D0: clears only low byte, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *clr(BYTE, DN, 0),
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
    assert (result & 0xFF) == 0, f"Expected low byte 0, got 0x{result:08X}"
    assert (result & 0xFFFFFF00) == 0x12345600, f"Expected upper bytes preserved, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0)
    assert_flags("CLR.B", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# CMP tests
# =========================================================================

@cocotb.test()
async def test_cmp_long_equal(dut):
    """CMP.L D0,D1 with D0=D1=42 -> Z=1 (equal)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *moveq(42, 1),
        *cmp_reg(LONG, 1, DN, 0),             # CMP.L D0,D1 (D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(LONG, 42, 42, 0)
    assert_flags("CMP.L equal", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


@cocotb.test()
async def test_cmp_long_greater(dut):
    """CMP.L D0,D1 with D1=50,D0=30 -> Z=0 C=0 (D1 > D0)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(30, 0),                         # D0 = 30 (source in EA)
        *moveq(50, 1),                         # D1 = 50 (destination Dn)
        *cmp_reg(LONG, 1, DN, 0),             # CMP.L D0,D1 (D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(LONG, 30, 50, 20)
    assert_flags("CMP.L greater", ccr, expected_cc)
    assert expected_cc['z'] == 0
    assert expected_cc['c'] == 0
    h.cleanup()


@cocotb.test()
async def test_cmp_long_less(dut):
    """CMP.L D0,D1 with D1=10,D0=50 -> C=1 N=1 (D1 < D0, unsigned borrow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(50, 0),                         # D0 = 50 (source)
        *moveq(10, 1),                         # D1 = 10 (dest)
        *cmp_reg(LONG, 1, DN, 0),             # CMP.L D0,D1 (D1 - D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    # D1 - D0 = 10 - 50 = -40 = 0xFFFFFFD8
    expected_cc = cc_cmp(LONG, 50, 10, 0xFFFFFFD8)
    assert_flags("CMP.L less", ccr, expected_cc)
    assert expected_cc['c'] == 1
    h.cleanup()


@cocotb.test()
async def test_cmp_does_not_modify_dest(dut):
    """CMP.L D0,D1 should NOT modify D1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),
        *moveq(77, 1),
        *cmp_reg(LONG, 1, DN, 0),             # CMP.L D0,D1
        *nop(), *nop(),
        *move(LONG, DN, 1, AN_IND, 0),        # D1 should still be 77
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 77, f"CMP modified D1! Expected 77, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# CMPI tests
# =========================================================================

@cocotb.test()
async def test_cmpi_long_equal(dut):
    """CMPI.L #42,D0 with D0=42 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *cmpi(LONG, DN, 0, 42),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(LONG, 42, 42, 0)
    assert_flags("CMPI.L equal", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


@cocotb.test()
async def test_cmpi_long_greater(dut):
    """CMPI.L #10,D0 with D0=50 -> Z=0 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(50, 0),
        *cmpi(LONG, DN, 0, 10),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    # CMPI: D0 - imm = 50 - 10 = 40
    expected_cc = cc_cmp(LONG, 10, 50, 40)
    assert_flags("CMPI.L greater", ccr, expected_cc)
    h.cleanup()


@cocotb.test()
async def test_cmpi_long_less(dut):
    """CMPI.L #100,D0 with D0=10 -> C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),
        *cmpi(LONG, DN, 0, 100),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    # CMPI: D0 - imm = 10 - 100 = -90 (0xFFFFFFA6)
    expected_cc = cc_cmp(LONG, 100, 10, 0xFFFFFFA6)
    assert_flags("CMPI.L less", ccr, expected_cc)
    assert expected_cc['c'] == 1
    h.cleanup()


@cocotb.test()
async def test_cmpi_does_not_modify(dut):
    """CMPI.L #99,D0 should NOT modify D0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(55, 0),
        *cmpi(LONG, DN, 0, 99),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 55, f"CMPI modified D0! Expected 55, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_cmpi_word(dut):
    """CMPI.W #0x8000,D0 with D0=0x8000 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),
        *cmpi(WORD, DN, 0, 0x8000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(WORD, 0x8000, 0x8000, 0)
    assert_flags("CMPI.W equal", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


@cocotb.test()
async def test_cmpi_byte(dut):
    """CMPI.B #0xFF,D0 with D0=0x00 -> C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *cmpi(BYTE, DN, 0, 0xFF),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    # 0x00 - 0xFF = 0x01 with borrow
    expected_cc = cc_cmp(BYTE, 0xFF, 0x00, 0x01)
    assert_flags("CMPI.B less", ccr, expected_cc)
    assert expected_cc['c'] == 1
    h.cleanup()


# =========================================================================
# ADD/SUB combined chained test
# =========================================================================

@cocotb.test()
async def test_add_sub_chain(dut):
    """Chain: D0=100, ADDQ #5 -> 105, SUBQ #3 -> 102."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(100, 0),
        *addq(LONG, 5, DN, 0),                # D0 = 105
        *subq(LONG, 3, DN, 0),                # D0 = 102
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 102, f"Expected 102, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Boundary: ADDQ across all values 1-8
# =========================================================================

@cocotb.test()
async def test_addq_all_values(dut):
    """ADDQ.L #n,D0 for n=1..8, starting from 0 each time. Store all results."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
    ]
    for n in range(1, 9):
        program += [
            *moveq(0, 0),
            *addq(LONG, n, DN, 0),
            *nop(), *nop(),
            *move(LONG, DN, 0, AN_IND, 0),
            *addq(LONG, 4, AN, 0),
        ]
    program += [*h.sentinel_program()]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=6000)
    assert found, "Sentinel not reached"

    for i, n in enumerate(range(1, 9)):
        result = h.read_result_long(i * 4)
        assert result == n, f"ADDQ.L #{n},D0: expected {n}, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Boundary: SUBQ across all values 1-8
# =========================================================================

@cocotb.test()
async def test_subq_all_values(dut):
    """SUBQ.L #n,D0 for n=1..8, starting from 100 each time."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
    ]
    for n in range(1, 9):
        program += [
            *moveq(100, 0),
            *subq(LONG, n, DN, 0),
            *nop(), *nop(),
            *move(LONG, DN, 0, AN_IND, 0),
            *addq(LONG, 4, AN, 0),
        ]
    program += [*h.sentinel_program()]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=6000)
    assert found, "Sentinel not reached"

    for i, n in enumerate(range(1, 9)):
        expected = 100 - n
        result = h.read_result_long(i * 4)
        assert result == expected, f"SUBQ.L #{n},D0: expected {expected}, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# ADD.L large immediate values
# =========================================================================

@cocotb.test()
async def test_addi_large_value(dut):
    """ADDI.L #0x12345678,D0 with D0=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *addi(LONG, DN, 0, 0x12345678),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# CMP.L with signed comparison edge cases
# =========================================================================

@cocotb.test()
async def test_cmp_signed_boundary(dut):
    """CMP.L: D1=0x7FFFFFFF, D0=0x80000000 -> V=1 (signed overflow in subtraction)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),                 # D0 = 0x80000000
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x7FFFFFFF),                 # D1 = 0x7FFFFFFF
        *cmp_reg(LONG, 1, DN, 0),             # CMP.L D0,D1: D1 - D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    # D1 - D0 = 0x7FFFFFFF - 0x80000000 = 0xFFFFFFFF
    expected_cc = cc_cmp(LONG, 0x80000000, 0x7FFFFFFF, 0xFFFFFFFF)
    assert_flags("CMP.L signed boundary", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# ADD negative values via MOVEQ
# =========================================================================

@cocotb.test()
async def test_add_negative_moveq(dut):
    """ADD.L D0,D1 with D0=-10 (0xFFFFFFF6), D1=20 -> 10."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-10, 0),                        # D0 = 0xFFFFFFF6
        *moveq(20, 1),                         # D1 = 20
        *add(LONG, 1, 0, DN, 0),              # ADD.L D0,D1 (D1 += D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 10, f"Expected 10, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 0xFFFFFFF6, 20, 0x100000000 + 10)
    assert_flags("ADD.L neg+pos", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# SUB self (D0 - D0 = 0)
# =========================================================================

@cocotb.test()
async def test_sub_self_zero(dut):
    """SUB.L D0,D0 -> 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *sub(LONG, 0, 0, DN, 0),              # SUB.L D0,D0 (D0 = D0 - D0)
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
    expected_cc = cc_sub(LONG, 42, 42, 0)
    assert_flags("SUB.L self", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


# =========================================================================
# ADDI.L #0,D0 -- NOP-like, but sets flags
# =========================================================================

@cocotb.test()
async def test_addi_zero(dut):
    """ADDI.L #0,D0 with D0=0x80000000 -> unchanged, N=1 V=0 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),
        *addi(LONG, DN, 0, 0),
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
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 0, 0x80000000, 0x80000000)
    assert_flags("ADDI.L #0", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# SUBI.L #0,D0 -- identity, sets flags
# =========================================================================

@cocotb.test()
async def test_subi_zero(dut):
    """SUBI.L #0,D0 with D0=42 -> unchanged, flags for result=42."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *subi(LONG, DN, 0, 0),
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
    assert result == 42, f"Expected 42, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 0, 42, 42)
    assert_flags("SUBI.L #0", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# NEG.L of -1 (0xFFFFFFFF)
# =========================================================================

@cocotb.test()
async def test_neg_minus_one(dut):
    """NEG.L D0 with D0=0xFFFFFFFF -> 1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),
        *neg(LONG, DN, 0),
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
    expected_cc = cc_neg(LONG, 0xFFFFFFFF, 1)
    assert_flags("NEG.L -1", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# ADDI.L with negative immediate
# =========================================================================

@cocotb.test()
async def test_addi_negative_imm(dut):
    """ADDI.L #0xFFFFFFFF,D0 (add -1) with D0=100 -> 99 (with carry)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(100, 0),
        *addi(LONG, DN, 0, 0xFFFFFFFF),
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
    assert result == 99, f"Expected 99, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    # 100 + 0xFFFFFFFF = 0x100000063; masked = 99, carry=1
    expected_cc = cc_add(LONG, 0xFFFFFFFF, 100, 0x100000063)
    assert_flags("ADDI.L neg imm", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# Multiple result slots test (ADD chain)
# =========================================================================

@cocotb.test()
async def test_add_chain_multiple_results(dut):
    """Chain of additions storing intermediate results via A0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                          # D0 = 0
        *addq(LONG, 5, DN, 0),                # D0 = 5
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),        # slot 0: 5
        *addq(LONG, 4, AN, 0),
        *addq(LONG, 3, DN, 0),                # D0 = 8
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),        # slot 1: 8
        *addq(LONG, 4, AN, 0),
        *addq(LONG, 7, DN, 0),                # D0 = 15
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),        # slot 2: 15
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 5, f"Slot 0: expected 5"
    assert h.read_result_long(4) == 8, f"Slot 1: expected 8"
    assert h.read_result_long(8) == 15, f"Slot 2: expected 15"
    h.cleanup()


# =========================================================================
# Word-size boundary: ADD.W wraps at 0xFFFF
# =========================================================================

@cocotb.test()
async def test_add_word_boundary_wrap(dut):
    """ADD.W: 0xFFFF + 1 = 0x0000 (word wrap), C=1 Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),
        *moveq(1, 1),
        *add(WORD, 0, 0, DN, 1),              # ADD.W D1,D0 (D0 += D1)
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
    assert (result & 0xFFFF) == 0, f"Expected low word 0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(WORD, 1, 0xFFFF, 0x10000)
    assert_flags("ADD.W 0xFFFF+1", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# Byte-size boundary: ADD.B wraps at 0xFF
# =========================================================================

@cocotb.test()
async def test_add_byte_boundary_wrap(dut):
    """ADD.B: 0xFF + 1 = 0x00 (byte wrap), C=1 Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                         # D0 = 0xFFFFFFFF (byte = 0xFF)
        *moveq(1, 1),
        *add(BYTE, 0, 0, DN, 1),              # ADD.B D1,D0
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
    assert (result & 0xFF) == 0x00, f"Expected low byte 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(BYTE, 1, 0xFF, 0x100)
    assert_flags("ADD.B 0xFF+1", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# Byte-size overflow: ADD.B 0x7F + 1 = 0x80
# =========================================================================

@cocotb.test()
async def test_add_byte_signed_overflow(dut):
    """ADD.B: 0x7F + 1 = 0x80, V=1 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x7F, 0),
        *moveq(1, 1),
        *add(BYTE, 0, 0, DN, 1),              # ADD.B D1,D0
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
    assert (result & 0xFF) == 0x80, f"Expected low byte 0x80, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(BYTE, 1, 0x7F, 0x80)
    assert_flags("ADD.B 0x7F+1", ccr, expected_cc)
    assert expected_cc['v'] == 1
    assert expected_cc['n'] == 1
    h.cleanup()


# =========================================================================
# NEG.B 0x80 = 0x80 (V=1, the one value that NEG overflows)
# =========================================================================

@cocotb.test()
async def test_neg_byte_minint(dut):
    """NEG.B D0 with byte=0x80 -> 0x80, V=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00000080),
        *neg(BYTE, DN, 0),
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
    assert (result & 0xFF) == 0x80, f"Expected low byte 0x80, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(BYTE, 0x80, 0x80)
    assert_flags("NEG.B 0x80", ccr, expected_cc)
    assert expected_cc['v'] == 1
    h.cleanup()


# =========================================================================
# NEG.B zero -> Z=1 C=0
# =========================================================================

@cocotb.test()
async def test_neg_byte_zero(dut):
    """NEG.B D0 with byte=0 -> 0, Z=1 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *neg(BYTE, DN, 0),
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
    assert (result & 0xFF) == 0, f"Expected low byte 0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_neg(BYTE, 0, 0)
    assert_flags("NEG.B 0", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert expected_cc['c'] == 0
    h.cleanup()


# =========================================================================
# SUB.B boundary: 0x80 - 0x01 = 0x7F, V=1
# =========================================================================

@cocotb.test()
async def test_sub_byte_signed_overflow(dut):
    """SUB.B: 0x80 - 0x01 = 0x7F, V=1 (neg - pos = pos overflow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                          # D0 = 1 (source)
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00000080),                 # D1 = 0x80 (dest byte)
        *sub(BYTE, 1, 0, DN, 0),              # SUB.B D0,D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFF) == 0x7F, f"Expected low byte 0x7F, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(BYTE, 1, 0x80, 0x7F)
    assert_flags("SUB.B 0x80-1 overflow", ccr, expected_cc)
    assert expected_cc['v'] == 1
    h.cleanup()


# =========================================================================
# ADD.L 0xFFFFFFFF + 0xFFFFFFFF = 0xFFFFFFFE (carry, not overflow)
# =========================================================================

@cocotb.test()
async def test_add_two_max(dut):
    """ADD.L 0xFFFFFFFF + 0xFFFFFFFF = 0xFFFFFFFE with C=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                         # D0 = 0xFFFFFFFF
        *moveq(-1, 1),                         # D1 = 0xFFFFFFFF
        *add(LONG, 1, 0, DN, 0),              # ADD.L D0,D1 (D1 += D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFFFFFFFE, f"Expected 0xFFFFFFFE, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFFFFFE)
    assert_flags("ADD.L max+max", ccr, expected_cc)
    assert expected_cc['c'] == 1
    h.cleanup()


# =========================================================================
# CMP word size
# =========================================================================

@cocotb.test()
async def test_cmp_word_equal(dut):
    """CMP.W D0,D1 with matching low words -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00001234),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0xFFFF1234),                 # same low word, different high
        *cmp_reg(WORD, 1, DN, 0),             # CMP.W D0,D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(WORD, 0x1234, 0x1234, 0)
    assert_flags("CMP.W equal", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


# =========================================================================
# CMP byte size
# =========================================================================

@cocotb.test()
async def test_cmp_byte_negative(dut):
    """CMP.B D0,D1: D1=0x80,D0=0x01 -> D1-D0 = 0x7F, V=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00000080),
        *cmp_reg(BYTE, 1, DN, 0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    expected_cc = cc_cmp(BYTE, 1, 0x80, 0x7F)
    assert_flags("CMP.B signed boundary", ccr, expected_cc)
    h.cleanup()


# =========================================================================
# ADDI.L to max positive
# =========================================================================

@cocotb.test()
async def test_addi_to_max_positive(dut):
    """ADDI.L #1,D0 with D0=0x7FFFFFFE -> 0x7FFFFFFF (no overflow yet)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x7FFFFFFE),
        *addi(LONG, DN, 0, 1),
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
    assert result == 0x7FFFFFFF, f"Expected 0x7FFFFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_add(LONG, 1, 0x7FFFFFFE, 0x7FFFFFFF)
    assert_flags("ADDI.L near max", ccr, expected_cc)
    assert expected_cc['v'] == 0, "No overflow expected"
    h.cleanup()


# =========================================================================
# SUBI.L from min negative
# =========================================================================

@cocotb.test()
async def test_subi_from_min_negative(dut):
    """SUBI.L #1,D0 with D0=0x80000001 -> 0x80000000 (no overflow)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000001),
        *subi(LONG, DN, 0, 1),
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
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_sub(LONG, 1, 0x80000001, 0x80000000)
    assert_flags("SUBI.L near min", ccr, expected_cc)
    assert expected_cc['v'] == 0
    h.cleanup()


# =========================================================================
# CLR followed by NEG (CLR clears V,C; NEG sets them for nonzero)
# =========================================================================

@cocotb.test()
async def test_clr_then_neg(dut):
    """CLR.L D0 then NEG.L D0 -> still 0, Z=1 C=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(99, 0),
        *clr(LONG, DN, 0),
        *neg(LONG, DN, 0),
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
    expected_cc = cc_neg(LONG, 0, 0)
    assert_flags("CLR then NEG", ccr, expected_cc)
    h.cleanup()
