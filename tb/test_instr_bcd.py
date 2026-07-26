"""
BCD and packed-decimal instruction compliance tests for WF68K30L.

Covers ABCD, SBCD, NBCD (register and memory forms) and PACK / UNPK
(register and memory forms).  None of these instructions had any
instruction-level coverage before this module: the ALU BCD adder was only
ever exercised by driving the ALU ports directly.

PRM condition-code rules for the three BCD instructions (X N Z V C = * U * U *):
  X - Set the same as the carry bit.
  N - Undefined.
  Z - "Cleared if the result is nonzero; unchanged otherwise."   <- sticky
  V - Undefined.
  C - Set if a decimal carry (ABCD) / borrow (SBCD, NBCD) occurs.

The sticky-Z rule is what makes multiple-precision chains work, and it is the
reason every test here pre-loads CCR with MOVE #imm,CCR immediately before the
instruction under test.  N and V are never asserted on: the PRM marks them
undefined, so any value the RTL produces is compliant.

PACK and UNPK do not affect the condition codes at all.

Store pattern (same as test_instr_arithmetic.py): A0 holds the result cursor,
and MOVE from CCR is always executed before the first MOVE store, because
MOVE.L Dn,(A0) itself writes the condition codes.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_PREDEC, SPECIAL, IMMEDIATE,
    moveq, move, movea, nop, addq,
    abcd, sbcd, nbcd, pack, unpk,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word,
)
from m68k_reference import extract_cc


# CCR bit weights, for building MOVE #imm,CCR operands.
CCR_C = 0x01
CCR_V = 0x02
CCR_Z = 0x04
CCR_N = 0x08
CCR_X = 0x10


def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    return extract_cc(val & 0xFF)


def assert_bcd_flags(name, actual_ccr, x, z, c):
    """Check X, Z and C only.  The PRM marks N and V undefined for BCD ops."""
    ax, an, az, av, ac = ccr_flags(actual_ccr)
    got = f"(CCR=0x{actual_ccr & 0xFF:02X}: X={ax} Z={az} C={ac})"
    assert ax == x, f"{name}: X expected {x}, got {ax} {got}"
    assert az == z, f"{name}: Z expected {z}, got {az} {got}"
    assert ac == c, f"{name}: C expected {c}, got {ac} {got}"


def _store_result_and_ccr(h, dn):
    """Capture CCR then store Dn and the CCR through the A0 cursor."""
    return [
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, dn, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
    ]


def _load_long(dn, value):
    return [*move(LONG, SPECIAL, IMMEDIATE, DN, dn), *imm_long(value)]


def _set_ccr(value):
    return [*move_to_ccr(SPECIAL, IMMEDIATE), *imm_word(value)]


async def _run_reg_bcd(dut, encode, src_val, dst_val, ccr_in):
    """Common driver for the register-form BCD tests.

    Loads D1 = src_val, D2 = dst_val, sets CCR, executes the instruction
    produced by encode(), and returns (result_d2, ccr_out).
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, src_val),
        *_load_long(2, dst_val),
        *_set_ccr(ccr_in),
        *encode,
        *_store_result_and_ccr(h, 2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    h.cleanup()
    return result, ccr


# =========================================================================
# ABCD - register form
# =========================================================================

@cocotb.test()
async def test_abcd_reg_low_digit_carry(dut):
    """ABCD D1,D2: 0x19 + 0x01, X=0 -> 0x20 (low-digit correction), C=0."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0xAABBCC01, 0x11223319, 0x00)
    assert (result & 0xFF) == 0x20, f"expected 0x20, got 0x{result:08X}"
    # Byte operation: the upper three bytes of the destination must survive.
    assert result == 0x11223320, f"upper bytes disturbed: 0x{result:08X}"
    assert_bcd_flags("ABCD 19+01", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_abcd_reg_99_plus_01_wraps(dut):
    """ABCD D1,D2: 0x99 + 0x01, X=0 -> 0x00 with C=X=1; Z stays set (sticky).

    PRM ABCD: "Z - Cleared if the result is nonzero; unchanged otherwise."
    The result here IS zero, so a Z that was set going in stays set.
    """
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x01, 0x99, CCR_Z)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert_bcd_flags("ABCD 99+01", ccr, x=1, z=1, c=1)


@cocotb.test()
async def test_abcd_reg_x_in(dut):
    """ABCD D1,D2: 0x34 + 0x12 + X=1 -> 0x47."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x34, 0x12, CCR_X)
    assert (result & 0xFF) == 0x47, f"expected 0x47, got 0x{result:08X}"
    assert_bcd_flags("ABCD 12+34+X", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_abcd_reg_max_plus_max(dut):
    """ABCD D1,D2: 0x99 + 0x99 + X=1 -> 0x99 with C=X=1 (199 decimal)."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x99, 0x99, CCR_X)
    assert (result & 0xFF) == 0x99, f"expected 0x99, got 0x{result:08X}"
    assert_bcd_flags("ABCD 99+99+X", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_abcd_reg_high_digit_carry(dut):
    """ABCD D1,D2: 0x50 + 0x50, X=0 -> 0x00 with C=X=1 (high-digit correction)."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x50, 0x50, CCR_Z)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert_bcd_flags("ABCD 50+50", ccr, x=1, z=1, c=1)


@cocotb.test()
async def test_abcd_reg_z_cleared_on_nonzero(dut):
    """ABCD with Z=1 going in must clear Z when the result is nonzero."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x01, 0x00, CCR_Z)
    assert (result & 0xFF) == 0x01, f"expected 0x01, got 0x{result:08X}"
    assert_bcd_flags("ABCD 00+01 with Z preset", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_abcd_reg_z_preset_clear_stays_clear(dut):
    """A zero ABCD result leaves Z alone: Z=0 in stays Z=0 out."""
    result, ccr = await _run_reg_bcd(
        dut, abcd(2, 1), 0x00, 0x00, 0x00)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert_bcd_flags("ABCD 00+00, Z clear in", ccr, x=0, z=0, c=0)


# =========================================================================
# ABCD - memory form, including a multiple-precision chain
# =========================================================================

@cocotb.test()
async def test_abcd_mem_predecrement(dut):
    """ABCD -(A1),-(A2): single byte 0x28 + 0x13 -> 0x41 in memory."""
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 1),
        *_set_ccr(0x00),
        *abcd(2, 1, rm=1),                    # ABCD -(A1),-(A2)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0),        # store A1 to check predecrement
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0),        # store A2
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(src, 1, 0x13)
    h.mem.write(dst, 1, 0x28)
    h.mem.write(dst + 1, 1, 0x5A)             # guard byte, must not change
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x41, f"expected 0x41 at destination, got 0x{got:02X}"
    guard = h.mem.read(dst + 1, 1)
    assert guard == 0x5A, f"byte past the destination clobbered: 0x{guard:02X}"
    assert_bcd_flags("ABCD -(A1),-(A2)", h.read_result_long(0), x=0, z=0, c=0)
    a1 = h.read_result_long(4)
    a2 = h.read_result_long(8)
    assert a1 == src, f"A1 expected 0x{src:08X} after predecrement, got 0x{a1:08X}"
    assert a2 == dst, f"A2 expected 0x{dst:08X} after predecrement, got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_abcd_mem_multiprecision_zero_result(dut):
    """Three-byte BCD chain 123456 + 876544 = 1000000.

    Every byte of the six-digit result is zero and each stage carries, so this
    is the canonical sticky-Z case: Z is set before the chain and must still be
    set after it, with C=X=1 from the final carry.
    """
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 3),
        *_set_ccr(CCR_Z),                     # X=0, Z=1 to start the chain
        *abcd(2, 1, rm=1),
        *abcd(2, 1, rm=1),
        *abcd(2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x87, 0x65, 0x44)):
        h.mem.write(src + i, 1, b)
    for i, b in enumerate((0x12, 0x34, 0x56)):
        h.mem.write(dst + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(dst + i, 1) for i in range(3)]
    assert got == [0x00, 0x00, 0x00], (
        f"expected 000000, got {[f'0x{b:02X}' for b in got]}")
    assert_bcd_flags("ABCD chain 123456+876544", h.read_result_long(0),
                     x=1, z=1, c=1)
    h.cleanup()


@cocotb.test()
async def test_abcd_mem_multiprecision_nonzero_clears_z(dut):
    """Three-byte BCD chain 123456 + 000001 = 123457 must clear the sticky Z."""
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 3),
        *_set_ccr(CCR_Z),
        *abcd(2, 1, rm=1),
        *abcd(2, 1, rm=1),
        *abcd(2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x00, 0x00, 0x01)):
        h.mem.write(src + i, 1, b)
    for i, b in enumerate((0x12, 0x34, 0x56)):
        h.mem.write(dst + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(dst + i, 1) for i in range(3)]
    assert got == [0x12, 0x34, 0x57], (
        f"expected 123457, got {[f'0x{b:02X}' for b in got]}")
    assert_bcd_flags("ABCD chain 123456+1", h.read_result_long(0),
                     x=0, z=0, c=0)
    h.cleanup()


# =========================================================================
# SBCD - register form
# =========================================================================

@cocotb.test()
async def test_sbcd_reg_low_digit_borrow(dut):
    """SBCD D1,D2: 0x42 - 0x13, X=0 -> 0x29 (low-digit borrow), C=0."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x13, 0x11223342, 0x00)
    assert (result & 0xFF) == 0x29, f"expected 0x29, got 0x{result:08X}"
    assert result == 0x11223329, f"upper bytes disturbed: 0x{result:08X}"
    assert_bcd_flags("SBCD 42-13", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_sbcd_reg_x_in(dut):
    """SBCD D1,D2: 0x42 - 0x13 - X=1 -> 0x28."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x13, 0x42, CCR_X)
    assert (result & 0xFF) == 0x28, f"expected 0x28, got 0x{result:08X}"
    assert_bcd_flags("SBCD 42-13-X", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_sbcd_reg_zero_minus_one(dut):
    """SBCD D1,D2: 0x00 - 0x01, X=0 -> 0x99 with a decimal borrow (C=X=1)."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x01, 0x00, 0x00)
    assert (result & 0xFF) == 0x99, f"expected 0x99, got 0x{result:08X}"
    assert_bcd_flags("SBCD 00-01", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_sbcd_reg_tens_borrow(dut):
    """SBCD D1,D2: 0x30 - 0x01 -> 0x29: the tens digit absorbs the borrow."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x01, 0x30, 0x00)
    assert (result & 0xFF) == 0x29, f"expected 0x29, got 0x{result:08X}"
    assert_bcd_flags("SBCD 30-01", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_sbcd_reg_sticky_z_on_equal_operands(dut):
    """SBCD of equal operands gives 0x00 and leaves a preset Z set."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x42, 0x42, CCR_Z)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert_bcd_flags("SBCD 42-42", ccr, x=0, z=1, c=0)


@cocotb.test()
async def test_sbcd_reg_nines_complement_of_zero(dut):
    """SBCD D1,D2: 0x00 - 0x00 - X=1 -> 0x99 with C=X=1."""
    result, ccr = await _run_reg_bcd(
        dut, sbcd(2, 1), 0x00, 0x00, CCR_X)
    assert (result & 0xFF) == 0x99, f"expected 0x99, got 0x{result:08X}"
    assert_bcd_flags("SBCD 00-00-X", ccr, x=1, z=0, c=1)


# =========================================================================
# SBCD - memory form
# =========================================================================

@cocotb.test()
async def test_sbcd_mem_predecrement(dut):
    """SBCD -(A1),-(A2): 0x50 - 0x27 -> 0x23 in memory (A1 source, A2 dest)."""
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 1),
        *_set_ccr(0x00),
        *sbcd(2, 1, rm=1),                    # SBCD -(A1),-(A2)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(src, 1, 0x27)
    h.mem.write(dst, 1, 0x50)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x23, f"expected 0x23 at destination, got 0x{got:02X}"
    assert_bcd_flags("SBCD -(A1),-(A2)", h.read_result_long(0), x=0, z=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_sbcd_mem_multiprecision_borrow_chain(dut):
    """Three-byte BCD chain 000000 - 000001 = 999999 with a borrow out."""
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 3),
        *_set_ccr(CCR_Z),
        *sbcd(2, 1, rm=1),
        *sbcd(2, 1, rm=1),
        *sbcd(2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x00, 0x00, 0x01)):
        h.mem.write(src + i, 1, b)
    for i in range(3):
        h.mem.write(dst + i, 1, 0x00)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(dst + i, 1) for i in range(3)]
    assert got == [0x99, 0x99, 0x99], (
        f"expected 999999, got {[f'0x{b:02X}' for b in got]}")
    assert_bcd_flags("SBCD chain 0-1", h.read_result_long(0), x=1, z=0, c=1)
    h.cleanup()


# =========================================================================
# NBCD
# =========================================================================

async def _run_nbcd(dut, dst_val, ccr_in):
    """NBCD Dn form: load D2, set CCR, run NBCD D2, return (D2, CCR)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(2, dst_val),
        *_set_ccr(ccr_in),
        *nbcd(DN, 2),
        *_store_result_and_ccr(h, 2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    h.cleanup()
    return result, ccr


@cocotb.test()
async def test_nbcd_zero_x0(dut):
    """NBCD 0x00 with X=0 -> 0x00, no borrow.

    PRM NBCD: "This instruction produces the tens complement of the
    destination if the extend bit is zero"; the tens complement of zero is
    zero and generates no borrow.  This is the one input for which NBCD does
    not set C.
    """
    result, ccr = await _run_nbcd(dut, 0x11223300, 0x00)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert result == 0x11223300, f"upper bytes disturbed: 0x{result:08X}"
    assert_bcd_flags("NBCD 00, X=0", ccr, x=0, z=0, c=0)


@cocotb.test()
async def test_nbcd_one_x0(dut):
    """NBCD 0x01 with X=0 -> 0x99 (tens complement), C=X=1."""
    result, ccr = await _run_nbcd(dut, 0x01, 0x00)
    assert (result & 0xFF) == 0x99, f"expected 0x99, got 0x{result:08X}"
    assert_bcd_flags("NBCD 01, X=0", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_nbcd_low_digit_only_x0(dut):
    """NBCD 0x05 with X=0 -> 0x95: the low-digit borrow must reach the tens.

    Audit finding A13 lived exactly here: the high nibble was computed from
    X rather than from the low-digit borrow, which differs precisely when
    X=0 and the low digit is nonzero.
    """
    result, ccr = await _run_nbcd(dut, 0x05, 0x00)
    assert (result & 0xFF) == 0x95, f"expected 0x95, got 0x{result:08X}"
    assert_bcd_flags("NBCD 05, X=0", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_nbcd_low_digit_only_x1(dut):
    """NBCD 0x05 with X=1 -> 0x94 (nines complement), C=X=1."""
    result, ccr = await _run_nbcd(dut, 0x05, CCR_X)
    assert (result & 0xFF) == 0x94, f"expected 0x94, got 0x{result:08X}"
    assert_bcd_flags("NBCD 05, X=1", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_nbcd_max_x0(dut):
    """NBCD 0x99 with X=0 -> 0x01 (tens complement), C=X=1."""
    result, ccr = await _run_nbcd(dut, 0x99, 0x00)
    assert (result & 0xFF) == 0x01, f"expected 0x01, got 0x{result:08X}"
    assert_bcd_flags("NBCD 99, X=0", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_nbcd_zero_x1(dut):
    """NBCD 0x00 with X=1 -> 0x99 (nines complement of zero), C=X=1."""
    result, ccr = await _run_nbcd(dut, 0x00, CCR_X)
    assert (result & 0xFF) == 0x99, f"expected 0x99, got 0x{result:08X}"
    assert_bcd_flags("NBCD 00, X=1", ccr, x=1, z=0, c=1)


@cocotb.test()
async def test_nbcd_sticky_z_on_zero_result(dut):
    """NBCD 0x00 with X=0 gives 0x00 and leaves a preset Z set."""
    result, ccr = await _run_nbcd(dut, 0x00, CCR_Z)
    assert (result & 0xFF) == 0x00, f"expected 0x00, got 0x{result:08X}"
    assert_bcd_flags("NBCD 00 with Z preset", ccr, x=0, z=1, c=0)


@cocotb.test()
async def test_nbcd_memory_predecrement(dut):
    """NBCD -(A1): 0x42 in memory becomes its tens complement 0x58."""
    h = CPUTestHarness(dut)
    dst = h.DATA_BASE
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(dst + 1),
        *_set_ccr(0x00),
        *nbcd(AN_PREDEC, 1),                  # NBCD -(A1)
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(dst, 1, 0x42)
    h.mem.write(dst + 1, 1, 0x7E)             # guard
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x58, f"expected 0x58 (tens complement of 42), got 0x{got:02X}"
    guard = h.mem.read(dst + 1, 1)
    assert guard == 0x7E, f"guard byte clobbered: 0x{guard:02X}"
    assert_bcd_flags("NBCD -(A1)", h.read_result_long(0), x=1, z=0, c=1)
    a1 = h.read_result_long(4)
    assert a1 == dst, f"A1 expected 0x{dst:08X}, got 0x{a1:08X}"
    h.cleanup()


@cocotb.test()
async def test_nbcd_memory_indirect(dut):
    """NBCD (A1): 0x10 in memory becomes 0x90, C=X=1."""
    h = CPUTestHarness(dut)
    dst = h.DATA_BASE
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(dst),
        *_set_ccr(0x00),
        *nbcd(AN_IND, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(dst, 1, 0x10)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x90, f"expected 0x90, got 0x{got:02X}"
    assert_bcd_flags("NBCD (A1)", h.read_result_long(0), x=1, z=0, c=1)
    h.cleanup()


# =========================================================================
# PACK - register form
# =========================================================================

async def _run_pack_reg(dut, src_val, dst_val, adjustment):
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, src_val),
        *_load_long(2, dst_val),
        *_set_ccr(CCR_X | CCR_N | CCR_Z | CCR_V | CCR_C),
        *pack(2, 1, adjustment),               # PACK D1,D2,#adj
        *_store_result_and_ccr(h, 2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    h.cleanup()
    return result, ccr


@cocotb.test()
async def test_pack_reg_zero_adjustment(dut):
    """PACK D1,D2,#0 with D1=0x3135 packs to 0x15; upper 24 bits of D2 survive.

    PRM: "Bits 11-8 and 3-0 of the intermediate result are concatenated and
    placed in bits 7-0 of the destination register.  The remainder of the
    destination register is unaffected."
    """
    result, ccr = await _run_pack_reg(dut, 0x00003135, 0xAABBCCDD, 0x0000)
    assert result == 0xAABBCC15, f"expected 0xAABBCC15, got 0x{result:08X}"
    # PACK does not affect the condition codes: all five stay as preset.
    x, n, z, v, c = ccr_flags(ccr)
    assert (x, n, z, v, c) == (1, 1, 1, 1, 1), (
        f"PACK altered CCR: 0x{ccr & 0xFF:02X}")


@cocotb.test()
async def test_pack_reg_nonzero_adjustment(dut):
    """PACK D1,D2,#$1111 with D1=0x1234: 0x1234+0x1111=0x2345 -> 0x35."""
    result, _ = await _run_pack_reg(dut, 0x1234, 0x00000000, 0x1111)
    assert (result & 0xFF) == 0x35, f"expected 0x35, got 0x{result:08X}"


@cocotb.test()
async def test_pack_reg_ascii_to_bcd(dut):
    """PACK with the ASCII-to-BCD adjustment: '9','7' (0x3937) -> 0x97."""
    result, _ = await _run_pack_reg(dut, 0x00003937, 0x00000000, 0x0000)
    assert (result & 0xFF) == 0x97, f"expected 0x97, got 0x{result:08X}"


@cocotb.test()
async def test_pack_reg_ignores_source_high_word(dut):
    """PACK reads only the low word of the source register."""
    result, _ = await _run_pack_reg(dut, 0xFFFF3135, 0x00000000, 0x0000)
    assert (result & 0xFF) == 0x15, f"expected 0x15, got 0x{result:08X}"


@cocotb.test()
async def test_pack_mem_predecrement_FAILS_source_ea_not_predecremented(dut):
    """PACK -(A1),-(A2),#adj: two source bytes -> one packed destination byte.

    The source word is formed from the two bytes at the decremented address,
    low address first (PRM: "two bytes from the source are fetched and
    concatenated").  0x36,0x34 with adjustment 0 packs to 0x64.

    KNOWN RTL DEFECT - the source effective address is never predecremented.
    With A1 = 0x010002 the core issues the word read at 0x010002 itself (and,
    with only the register-list half of the fix in place, at 0x010001), never
    at 0x010000.  Two independent causes, both faithfully ported from the VHDL
    (vhdl/wf68k30L_control.vhd:683-685 and :1041-1050):

      1. sv/wf68k30L_ctrl_regsel.sv:222-226 - the AR_DEC_I operation list omits
         PACK and UNPK, so the general "-(An) predecrement on entry to
         CALC_AEFF" arm of AR_DEC (line 232) evaluates to 0.  The only
         PACK/UNPK arm of AR_DEC (line 241) fires at INIT_ENTRY and decrements
         the *destination* register; the source register is never touched.
         Adding PACK and UNPK to AR_DEC_I fixes UNPK completely.
      2. sv/wf68k30L_ctrl_comb.sv:491 makes OP_SIZE WORD for PACK only while
         NEXT_FETCH_STATE or FETCH_STATE is FETCH_OPERAND; the AR_DEC pulse
         happens one state earlier, on the transition into CALC_AEFF, where
         line 504 still reports BYTE.  So even with fix 1 the PACK source
         address decrements by 1 instead of 2 and the word read lands on the
         odd address 0x010001, returning 0x3400 and packing to 0x40.

    Both files are owned by other agents.  Verified by temporarily applying
    each change in this worktree: fix 1 alone turns the two UNPK memory reads
    correct; fix 1 + a WORD-sized decrement is needed for PACK.
    """
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 1),
        *pack(2, 1, 0x0000, rm=1),             # PACK -(A1),-(A2),#0
        *nop(), *nop(),
        *move(LONG, AN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(src, 1, 0x36)                 # low address -> high nibble pair
    h.mem.write(src + 1, 1, 0x34)
    h.mem.write(dst, 1, 0x00)
    h.mem.write(dst + 1, 1, 0xC3)             # guard
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x64, f"expected 0x64, got 0x{got:02X}"
    guard = h.mem.read(dst + 1, 1)
    assert guard == 0xC3, f"guard byte clobbered: 0x{guard:02X}"
    a1 = h.read_result_long(0)
    a2 = h.read_result_long(4)
    assert a1 == src, f"A1 expected 0x{src:08X} (decremented by 2), got 0x{a1:08X}"
    assert a2 == dst, f"A2 expected 0x{dst:08X} (decremented by 1), got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_pack_mem_with_adjustment_FAILS_source_ea_not_predecremented(dut):
    """PACK -(A1),-(A2),#$F0F0: 0x0102 + 0xF0F0 = 0xF1F2 -> 0x12.

    KNOWN RTL DEFECT - same cause as
    test_pack_mem_predecrement_FAILS_source_ea_not_predecremented: the source
    read lands on the un-decremented A1, so the adjustment is added to 0x0000
    and the result is 0x00 (0x20 with only the AR_DEC_I half of the fix).
    """
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 1),
        *pack(2, 1, 0xF0F0, rm=1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(src, 1, 0x01)
    h.mem.write(src + 1, 1, 0x02)
    h.mem.write(dst, 1, 0x00)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(dst, 1)
    assert got == 0x12, f"expected 0x12, got 0x{got:02X}"
    h.cleanup()


# =========================================================================
# UNPK
# =========================================================================

async def _run_unpk_reg(dut, src_val, dst_val, adjustment):
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, src_val),
        *_load_long(2, dst_val),
        *_set_ccr(CCR_X | CCR_N | CCR_Z | CCR_V | CCR_C),
        *unpk(2, 1, adjustment),               # UNPK D1,D2,#adj
        *_store_result_and_ccr(h, 2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    h.cleanup()
    return result, ccr


@cocotb.test()
async def test_unpk_reg_zero_adjustment(dut):
    """UNPK D1,D2,#0 with D1=0x15 -> 0x0105; the high word of D2 survives.

    PRM: "the instruction unpacks the source register contents, adds the
    extension word, and places the result in the destination register.  The
    high word of the destination register is unaffected."
    """
    result, ccr = await _run_unpk_reg(dut, 0x00000015, 0xAABBCCDD, 0x0000)
    assert result == 0xAABB0105, f"expected 0xAABB0105, got 0x{result:08X}"
    x, n, z, v, c = ccr_flags(ccr)
    assert (x, n, z, v, c) == (1, 1, 1, 1, 1), (
        f"UNPK altered CCR: 0x{ccr & 0xFF:02X}")


@cocotb.test()
async def test_unpk_reg_bcd_to_ascii(dut):
    """UNPK D1,D2,#$3030 with D1=0x15 -> 0x3135 (BCD to ASCII)."""
    result, _ = await _run_unpk_reg(dut, 0x15, 0x00000000, 0x3030)
    assert (result & 0xFFFF) == 0x3135, f"expected 0x3135, got 0x{result:08X}"


@cocotb.test()
async def test_unpk_reg_high_digits(dut):
    """UNPK D1,D2,#0 with D1=0x99 -> 0x0909."""
    result, _ = await _run_unpk_reg(dut, 0x99, 0x00000000, 0x0000)
    assert (result & 0xFFFF) == 0x0909, f"expected 0x0909, got 0x{result:08X}"


@cocotb.test()
async def test_unpk_reg_ignores_source_high_bits(dut):
    """UNPK reads only the low byte of the source register."""
    result, _ = await _run_unpk_reg(dut, 0xFFFFFF42, 0x00000000, 0x0000)
    assert (result & 0xFFFF) == 0x0402, f"expected 0x0402, got 0x{result:08X}"


@cocotb.test()
async def test_unpk_mem_predecrement_FAILS_source_ea_not_predecremented(dut):
    """UNPK -(A1),-(A2),#$3030: byte 0x27 -> bytes 0x32,0x37 at the destination.

    The high byte of the unpacked word goes to the lower destination address.

    KNOWN RTL DEFECT - the source effective address is never predecremented, so
    with A1 = 0x010001 the byte read is issued at 0x010001 rather than at
    0x010000; the source reads as 0x00 and the result is the bare adjustment
    word, 0x30 0x30.  Cause: sv/wf68k30L_ctrl_regsel.sv:222-226 omits PACK and
    UNPK from the AR_DEC_I operation list, so the "-(An) predecrement on entry
    to CALC_AEFF" arm of AR_DEC (line 232) never fires for them; the only
    PACK/UNPK arm (line 241) decrements the destination register at
    INIT_ENTRY.  Faithfully ported from vhdl/wf68k30L_control.vhd:683-685.
    Adding PACK and UNPK to that case list makes this test pass; the file is
    owned by another agent.  See
    test_pack_mem_predecrement_FAILS_source_ea_not_predecremented for the
    second, PACK-only half of the defect.
    """
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x10
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(src + 1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(dst + 2),
        *unpk(2, 1, 0x3030, rm=1),             # UNPK -(A1),-(A2),#$3030
        *nop(), *nop(),
        *move(LONG, AN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(src, 1, 0x27)
    h.mem.write(dst, 1, 0x00)
    h.mem.write(dst + 1, 1, 0x00)
    h.mem.write(dst + 2, 1, 0x5C)             # guard
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(dst + i, 1) for i in range(2)]
    assert got == [0x32, 0x37], (
        f"expected [0x32, 0x37], got {[f'0x{b:02X}' for b in got]}")
    guard = h.mem.read(dst + 2, 1)
    assert guard == 0x5C, f"guard byte clobbered: 0x{guard:02X}"
    a1 = h.read_result_long(0)
    a2 = h.read_result_long(4)
    assert a1 == src, f"A1 expected 0x{src:08X} (decremented by 1), got 0x{a1:08X}"
    assert a2 == dst, f"A2 expected 0x{dst:08X} (decremented by 2), got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_pack_unpk_round_trip(dut):
    """UNPK then PACK of the same byte through registers is the identity."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000073),
        *unpk(2, 1, 0x3030),                   # D2 = 0x3733
        *nop(), *nop(),
        *pack(3, 2, 0x0000),                   # D3 low byte = 0x73
        *nop(), *nop(),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    unpacked = h.read_result_long(0)
    repacked = h.read_result_long(4)
    assert (unpacked & 0xFFFF) == 0x3733, (
        f"UNPK 0x73 expected 0x3733, got 0x{unpacked:08X}")
    assert (repacked & 0xFF) == 0x73, (
        f"PACK back expected 0x73, got 0x{repacked:08X}")
    h.cleanup()
