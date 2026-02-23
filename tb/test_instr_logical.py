"""
Logical instruction compliance tests for WF68K30L.

Tests AND/ANDI, OR/ORI, EOR/EORI, NOT, TST against the MC68030
specification (Table 3-12 condition codes).

All logical operations set V=0, C=0, X unchanged per the MC68030 spec.

Each test uses the prefetch pipeline hazard workaround:
  - Load RESULT_BASE into A0 early
  - Store results via MOVE.L Dn,(A0) (single-word, no extension words)
  - Advance A0 with ADDQ.L #4,A0

IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores, because
MOVE.L Dn,(A0) itself sets condition codes (cc_logic), overwriting
the flags from the instruction under test.

Uses m68k_reference.py to compute expected condition code values.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq,
    and_op, andi, or_op, ori, eor, eori, not_op, tst,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, imm_byte,
)
from m68k_reference import (
    cc_logic, mask_val, extract_cc,
    BYTE as R_BYTE, WORD as R_WORD, LONG as R_LONG,
)


# ---------------------------------------------------------------------------
# Helper: extract CCR flags from a stored long
# ---------------------------------------------------------------------------
def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    ccr = val & 0xFF
    return extract_cc(ccr)


def assert_flags(name, actual_ccr, expected_cc, dut=None):
    """Assert that actual CCR matches the expected CC dict.

    For logical operations, V and C must always be 0 and X must be unchanged.
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


def assert_logic_vc(name, actual_ccr):
    """Assert V=0 and C=0, which is required for all logical operations."""
    x, n, z, v, c = ccr_flags(actual_ccr)
    assert v == 0, f"{name}: V must be 0 for logical op, got {v} (CCR=0x{actual_ccr & 0xFF:02X})"
    assert c == 0, f"{name}: C must be 0 for logical op, got {c} (CCR=0x{actual_ccr & 0xFF:02X})"


# =========================================================================
# AND tests
# =========================================================================

@cocotb.test()
async def test_and_long_all_ones(dut):
    """AND.L D0,D1: 0xFFFFFFFF & 0x12345678 = 0x12345678."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                         # D0 = 0xFFFFFFFF
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x12345678),                 # D1 = 0x12345678
        *and_op(LONG, 1, 0, DN, 0),           # AND.L D0,D1 (D1 = D1 & D0)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),                 # capture CCR BEFORE stores
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
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0x12345678)
    assert_flags("AND.L all-ones", ccr, expected_cc)
    assert_logic_vc("AND.L all-ones", ccr)
    h.cleanup()


@cocotb.test()
async def test_and_long_zero(dut):
    """AND.L D0,D1: 0x00000000 & 0x12345678 = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                          # D0 = 0
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x12345678),
        *and_op(LONG, 1, 0, DN, 0),           # AND.L D0,D1
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
    expected_cc = cc_logic(LONG, 0)
    assert_flags("AND.L zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("AND.L zero", ccr)
    h.cleanup()


@cocotb.test()
async def test_and_long_mask(dut):
    """AND.L D0,D1: 0x0000FFFF & 0x12345678 = 0x00005678."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x12345678),
        *and_op(LONG, 1, 0, DN, 0),           # AND.L D0,D1
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
    assert result == 0x00005678, f"Expected 0x00005678, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0x00005678)
    assert_flags("AND.L mask", ccr, expected_cc)
    assert_logic_vc("AND.L mask", ccr)
    h.cleanup()


@cocotb.test()
async def test_and_word(dut):
    """AND.W D0,D1: word-size AND."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FF00),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x0000ABCD),
        *and_op(WORD, 1, 0, DN, 0),           # AND.W D0,D1
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
    assert (result & 0xFFFF) == 0xAB00, f"Expected low word 0xAB00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0xAB00)
    assert_flags("AND.W", ccr, expected_cc)
    assert_logic_vc("AND.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_and_byte(dut):
    """AND.B D0,D1: byte-size AND 0x0F & 0x3C = 0x0C."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x0F, 0),
        *moveq(0x3C, 1),
        *and_op(BYTE, 1, 0, DN, 0),           # AND.B D0,D1
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
    assert (result & 0xFF) == 0x0C, f"Expected low byte 0x0C, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0x0C)
    assert_flags("AND.B", ccr, expected_cc)
    assert_logic_vc("AND.B", ccr)
    h.cleanup()


@cocotb.test()
async def test_and_negative_result(dut):
    """AND.L: result with MSB set -> N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFF000000),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x80FFFFFF),
        *and_op(LONG, 1, 0, DN, 0),
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
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0x80000000)
    assert_flags("AND.L negative", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("AND.L negative", ccr)
    h.cleanup()


# =========================================================================
# ANDI tests
# =========================================================================

@cocotb.test()
async def test_andi_long(dut):
    """ANDI.L #0xFF00FF00,D0 with D0=0x12345678 -> 0x12005600."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *andi(LONG, DN, 0, 0xFF00FF00),
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
    assert result == 0x12005600, f"Expected 0x12005600, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0x12005600)
    assert_flags("ANDI.L", ccr, expected_cc)
    assert_logic_vc("ANDI.L", ccr)
    h.cleanup()


@cocotb.test()
async def test_andi_word(dut):
    """ANDI.W #0x00FF,D0 with D0=0x12AB -> 0x00AB."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000012AB),
        *andi(WORD, DN, 0, 0x00FF),
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
    assert (result & 0xFFFF) == 0x00AB, f"Expected low word 0x00AB, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0x00AB)
    assert_flags("ANDI.W", ccr, expected_cc)
    assert_logic_vc("ANDI.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_andi_byte(dut):
    """ANDI.B #0xF0,D0 with D0=0xAB -> 0xA0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000AB),
        *andi(BYTE, DN, 0, 0xF0),
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
    assert (result & 0xFF) == 0xA0, f"Expected low byte 0xA0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0xA0)
    assert_flags("ANDI.B", ccr, expected_cc)
    assert_logic_vc("ANDI.B", ccr)
    h.cleanup()


@cocotb.test()
async def test_andi_zero_mask(dut):
    """ANDI.L #0,D0 with D0=0xFFFFFFFF -> 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),
        *andi(LONG, DN, 0, 0),
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
    assert_flags("ANDI.L #0", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


# =========================================================================
# OR tests
# =========================================================================

@cocotb.test()
async def test_or_long_basic(dut):
    """OR.L D0,D1: 0xFF00FF00 | 0x00FF00FF = 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFF00FF00),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00FF00FF),
        *or_op(LONG, 1, 0, DN, 0),            # OR.L D0,D1
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
    expected_cc = cc_logic(LONG, 0xFFFFFFFF)
    assert_flags("OR.L complement", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("OR.L complement", ccr)
    h.cleanup()


@cocotb.test()
async def test_or_long_with_zero(dut):
    """OR.L D0,D1: 0 | 0x12345678 = 0x12345678 (identity)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x12345678),
        *or_op(LONG, 1, 0, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_or_long_zero_result(dut):
    """OR.L D0,D1: 0 | 0 = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *moveq(0, 1),
        *or_op(LONG, 1, 0, DN, 0),
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
    expected_cc = cc_logic(LONG, 0)
    assert_flags("OR.L zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    h.cleanup()


@cocotb.test()
async def test_or_word(dut):
    """OR.W D0,D1: word-size OR 0x00F0 | 0x0F00 = 0x0FF0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000F0),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00000F00),
        *or_op(WORD, 1, 0, DN, 0),
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
    assert (result & 0xFFFF) == 0x0FF0, f"Expected low word 0x0FF0, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0x0FF0)
    assert_flags("OR.W", ccr, expected_cc)
    assert_logic_vc("OR.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_or_byte(dut):
    """OR.B D0,D1: byte-size OR 0x0F | 0xF0 = 0xFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x0F, 0),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x000000F0),
        *or_op(BYTE, 1, 0, DN, 0),
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
    assert (result & 0xFF) == 0xFF, f"Expected low byte 0xFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0xFF)
    assert_flags("OR.B", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("OR.B", ccr)
    h.cleanup()


# =========================================================================
# ORI tests
# =========================================================================

@cocotb.test()
async def test_ori_long(dut):
    """ORI.L #0xFFFF0000,D0 with D0=0x0000FFFF -> 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),
        *ori(LONG, DN, 0, 0xFFFF0000),
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
    expected_cc = cc_logic(LONG, 0xFFFFFFFF)
    assert_flags("ORI.L", ccr, expected_cc)
    assert_logic_vc("ORI.L", ccr)
    h.cleanup()


@cocotb.test()
async def test_ori_word(dut):
    """ORI.W #0xFF00,D0 with word=0x00FF -> 0xFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000FF),
        *ori(WORD, DN, 0, 0xFF00),
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
    expected_cc = cc_logic(WORD, 0xFFFF)
    assert_flags("ORI.W", ccr, expected_cc)
    assert_logic_vc("ORI.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_ori_byte(dut):
    """ORI.B #0x80,D0 with byte=0x00 -> 0x80, N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *ori(BYTE, DN, 0, 0x80),
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
    expected_cc = cc_logic(BYTE, 0x80)
    assert_flags("ORI.B", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("ORI.B", ccr)
    h.cleanup()


# =========================================================================
# EOR tests (note: EOR only does Dn to EA, not EA to Dn)
# =========================================================================

@cocotb.test()
async def test_eor_long_basic(dut):
    """EOR.L D0,D1: 0xFF00FF00 ^ 0xFFFF0000 = 0x00FFFF00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFF00FF00),                 # D0 = source
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0xFFFF0000),                 # D1 = dest EA
        *eor(LONG, 0, DN, 1),                 # EOR.L D0,D1 (D1 = D1 ^ D0)
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
    assert result == 0x00FFFF00, f"Expected 0x00FFFF00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0x00FFFF00)
    assert_flags("EOR.L", ccr, expected_cc)
    assert_logic_vc("EOR.L", ccr)
    h.cleanup()


@cocotb.test()
async def test_eor_long_same(dut):
    """EOR.L D0,D0: any value ^ itself = 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xDEADBEEF),
        *eor(LONG, 0, DN, 0),                 # EOR.L D0,D0 (D0 ^= D0 = 0)
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
    assert_flags("EOR.L self", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("EOR.L self", ccr)
    h.cleanup()


@cocotb.test()
async def test_eor_word(dut):
    """EOR.W D0,D1: word-size XOR."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000AAAA),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00005555),
        *eor(WORD, 0, DN, 1),
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
    assert (result & 0xFFFF) == 0xFFFF, f"Expected low word 0xFFFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0xFFFF)
    assert_flags("EOR.W", ccr, expected_cc)
    assert_logic_vc("EOR.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_eor_byte(dut):
    """EOR.B D0,D1: byte-size XOR 0xFF ^ 0xFF = 0x00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                         # D0.B = 0xFF
        *moveq(-1, 1),                         # D1.B = 0xFF
        *eor(BYTE, 0, DN, 1),
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
    assert (result & 0xFF) == 0x00, f"Expected low byte 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0x00)
    assert_flags("EOR.B", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("EOR.B", ccr)
    h.cleanup()


# =========================================================================
# EORI tests
# =========================================================================

@cocotb.test()
async def test_eori_long(dut):
    """EORI.L #0xFFFFFFFF,D0: complement via XOR."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *eori(LONG, DN, 0, 0xFFFFFFFF),
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
    assert result == 0xEDCBA987, f"Expected 0xEDCBA987, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(LONG, 0xEDCBA987)
    assert_flags("EORI.L", ccr, expected_cc)
    assert_logic_vc("EORI.L", ccr)
    h.cleanup()


@cocotb.test()
async def test_eori_word(dut):
    """EORI.W #0xFFFF,D0 with word=0x0000 -> 0xFFFF, N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *eori(WORD, DN, 0, 0xFFFF),
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
    expected_cc = cc_logic(WORD, 0xFFFF)
    assert_flags("EORI.W", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("EORI.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_eori_byte(dut):
    """EORI.B #0xFF,D0 with byte=0xAA -> 0x55."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000AA),
        *eori(BYTE, DN, 0, 0xFF),
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
    assert (result & 0xFF) == 0x55, f"Expected low byte 0x55, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0x55)
    assert_flags("EORI.B", ccr, expected_cc)
    assert_logic_vc("EORI.B", ccr)
    h.cleanup()


@cocotb.test()
async def test_eori_zero(dut):
    """EORI.L #0,D0 -> identity (D0 unchanged)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xABCDEF01),
        *eori(LONG, DN, 0, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xABCDEF01, f"Expected 0xABCDEF01, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# NOT tests
# =========================================================================

@cocotb.test()
async def test_not_long_zero(dut):
    """NOT.L D0 with D0=0 -> 0xFFFFFFFF, N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *not_op(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0xFFFFFFFF)
    assert_flags("NOT.L 0", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("NOT.L 0", ccr)
    h.cleanup()


@cocotb.test()
async def test_not_long_all_ones(dut):
    """NOT.L D0 with D0=0xFFFFFFFF -> 0, Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),
        *not_op(LONG, DN, 0),
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
    assert_flags("NOT.L 0xFFFFFFFF", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("NOT.L 0xFFFFFFFF", ccr)
    h.cleanup()


@cocotb.test()
async def test_not_long_minint(dut):
    """NOT.L D0 with D0=0x80000000 -> 0x7FFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),
        *not_op(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0x7FFFFFFF)
    assert_flags("NOT.L 0x80000000", ccr, expected_cc)
    assert expected_cc['n'] == 0
    assert_logic_vc("NOT.L 0x80000000", ccr)
    h.cleanup()


@cocotb.test()
async def test_not_word(dut):
    """NOT.W D0: complement only low word."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x1234FF00),
        *not_op(WORD, DN, 0),
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
    assert (result & 0xFFFF) == 0x00FF, f"Expected low word 0x00FF, got 0x{result:08X}"
    assert (result >> 16) == 0x1234, f"Expected high word 0x1234 preserved, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(WORD, 0x00FF)
    assert_flags("NOT.W", ccr, expected_cc)
    assert_logic_vc("NOT.W", ccr)
    h.cleanup()


@cocotb.test()
async def test_not_byte(dut):
    """NOT.B D0: complement only low byte."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x55, 0),
        *not_op(BYTE, DN, 0),
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
    assert (result & 0xFF) == 0xAA, f"Expected low byte 0xAA, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    expected_cc = cc_logic(BYTE, 0xAA)
    assert_flags("NOT.B", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("NOT.B", ccr)
    h.cleanup()


@cocotb.test()
async def test_not_double_inverse(dut):
    """NOT.L twice returns to original value."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xDEADBEEF),
        *not_op(LONG, DN, 0),
        *not_op(LONG, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDEADBEEF, f"Expected 0xDEADBEEF, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# TST tests
# =========================================================================

@cocotb.test()
async def test_tst_long_zero(dut):
    """TST.L D0 with D0=0 -> Z=1 N=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),
        *tst(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0)
    assert_flags("TST.L zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert expected_cc['n'] == 0
    assert_logic_vc("TST.L zero", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_long_positive(dut):
    """TST.L D0 with D0=42 -> Z=0 N=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *tst(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 42)
    assert_flags("TST.L positive", ccr, expected_cc)
    assert expected_cc['z'] == 0
    assert expected_cc['n'] == 0
    assert_logic_vc("TST.L positive", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_long_negative(dut):
    """TST.L D0 with D0=0x80000000 -> Z=0 N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),
        *tst(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0x80000000)
    assert_flags("TST.L negative", ccr, expected_cc)
    assert expected_cc['z'] == 0
    assert expected_cc['n'] == 1
    assert_logic_vc("TST.L negative", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_long_minus_one(dut):
    """TST.L D0 with D0=0xFFFFFFFF -> N=1 Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),
        *tst(LONG, DN, 0),
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
    expected_cc = cc_logic(LONG, 0xFFFFFFFF)
    assert_flags("TST.L -1", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("TST.L -1", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_word_zero(dut):
    """TST.W D0 with low word=0 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFFFF0000),                 # high word nonzero, low word 0
        *tst(WORD, DN, 0),
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
    expected_cc = cc_logic(WORD, 0)
    assert_flags("TST.W zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("TST.W zero", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_word_negative(dut):
    """TST.W D0 with low word=0x8000 -> N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),
        *tst(WORD, DN, 0),
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
    expected_cc = cc_logic(WORD, 0x8000)
    assert_flags("TST.W negative", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("TST.W negative", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_byte_zero(dut):
    """TST.B D0 with low byte=0 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFFFFFF00),
        *tst(BYTE, DN, 0),
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
    expected_cc = cc_logic(BYTE, 0)
    assert_flags("TST.B zero", ccr, expected_cc)
    assert expected_cc['z'] == 1
    assert_logic_vc("TST.B zero", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_byte_negative(dut):
    """TST.B D0 with low byte=0x80 -> N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00000080),
        *tst(BYTE, DN, 0),
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
    expected_cc = cc_logic(BYTE, 0x80)
    assert_flags("TST.B negative", ccr, expected_cc)
    assert expected_cc['n'] == 1
    assert_logic_vc("TST.B negative", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_byte_positive(dut):
    """TST.B D0 with low byte=0x7F -> N=0 Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x7F, 0),
        *tst(BYTE, DN, 0),
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
    expected_cc = cc_logic(BYTE, 0x7F)
    assert_flags("TST.B positive", ccr, expected_cc)
    assert expected_cc['n'] == 0
    assert expected_cc['z'] == 0
    assert_logic_vc("TST.B positive", ccr)
    h.cleanup()


@cocotb.test()
async def test_tst_does_not_modify(dut):
    """TST.L D0 should NOT modify D0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xDEADBEEF),
        *tst(LONG, DN, 0),
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDEADBEEF, f"TST modified D0! Expected 0xDEADBEEF, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# X flag unchanged for logical operations
# =========================================================================

@cocotb.test()
async def test_and_preserves_x(dut):
    """AND.L should not modify X flag (X unchanged per Table 3-12)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set X=1 via MOVE to CCR
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x10),                       # X=1 only
        *move_to_ccr(DN, 5),
        # Now do AND which should preserve X
        *moveq(-1, 0),
        *moveq(0x55, 1),
        *and_op(LONG, 1, 0, DN, 0),           # AND.L D0,D1
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
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"AND.L should preserve X=1, got X={x} (CCR=0x{ccr & 0xFF:02X})"
    h.cleanup()


@cocotb.test()
async def test_or_preserves_x(dut):
    """OR.L should not modify X flag."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x10),                       # X=1
        *move_to_ccr(DN, 5),
        *moveq(0, 0),
        *moveq(0, 1),
        *or_op(LONG, 1, 0, DN, 0),
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
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"OR.L should preserve X=1, got X={x} (CCR=0x{ccr & 0xFF:02X})"
    h.cleanup()


@cocotb.test()
async def test_eor_preserves_x(dut):
    """EOR.L should not modify X flag."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x10),                       # X=1
        *move_to_ccr(DN, 5),
        *moveq(0x55, 0),
        *moveq(0x55, 1),
        *eor(LONG, 0, DN, 1),                 # EOR clears to 0
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
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"EOR.L should preserve X=1, got X={x} (CCR=0x{ccr & 0xFF:02X})"
    h.cleanup()


@cocotb.test()
async def test_not_preserves_x(dut):
    """NOT.L should not modify X flag."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x10),                       # X=1
        *move_to_ccr(DN, 5),
        *moveq(0, 0),
        *not_op(LONG, DN, 0),
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
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"NOT.L should preserve X=1, got X={x} (CCR=0x{ccr & 0xFF:02X})"
    h.cleanup()


@cocotb.test()
async def test_tst_preserves_x(dut):
    """TST.L should not modify X flag."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x10),                       # X=1
        *move_to_ccr(DN, 5),
        *moveq(42, 0),
        *tst(LONG, DN, 0),
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
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"TST.L should preserve X=1, got X={x} (CCR=0x{ccr & 0xFF:02X})"
    h.cleanup()
