"""
Bit manipulation instruction compliance tests for WF68K30L.

Tests BTST, BSET, BCLR, BCHG in both register and immediate forms
against the MC68030 specification.

Bit operations on data registers:
  - Bit number is modulo 32 of the source value
  - Operand is 32 bits (long)

Condition codes for all bit operations:
  - Z flag reflects the OLD value of the tested bit (before modification)
  - Z = 1 if bit was 0, Z = 0 if bit was 1
  - All other flags (X, N, V, C) are NOT affected

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
    moveq, move, movea, nop, addq,
    btst_reg, btst_imm, bset_reg, bset_imm,
    bchg_reg, bchg_imm, bclr_reg, bclr_imm,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word,
)
from m68k_reference import extract_cc


# ---------------------------------------------------------------------------
# Helper: extract CCR flags from a stored long (low 8 bits of CCR word)
# ---------------------------------------------------------------------------
def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    ccr = val & 0xFF
    return extract_cc(ccr)


def assert_z_flag(name, actual_ccr, expected_z):
    """Assert that the Z flag matches expected. Other flags not checked (not affected)."""
    x, n, z, v, c = ccr_flags(actual_ccr)
    assert z == expected_z, (
        f"{name}: Z flag expected {expected_z}, got {z} "
        f"(CCR=0x{actual_ccr & 0xFF:02X})"
    )


def assert_flags_unchanged(name, before_ccr, after_ccr, except_z=True):
    """Assert X, N, V, C flags are unchanged. If except_z=True, allow Z to differ."""
    x0, n0, z0, v0, c0 = ccr_flags(before_ccr)
    x1, n1, z1, v1, c1 = ccr_flags(after_ccr)
    assert x0 == x1, f"{name}: X changed from {x0} to {x1}"
    assert n0 == n1, f"{name}: N changed from {n0} to {n1}"
    assert v0 == v1, f"{name}: V changed from {v0} to {v1}"
    assert c0 == c1, f"{name}: C changed from {c0} to {c1}"


# =========================================================================
# BTST register form tests
# =========================================================================

@cocotb.test()
async def test_btst_reg_bit0_set(dut):
    """BTST D1,D0: test bit 0 of 0x01 -> Z=0 (bit is set)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *moveq(0, 1),                            # D1 = bit number 0
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit0 of 0x01", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_bit0_clear(dut):
    """BTST D1,D0: test bit 0 of 0x00 -> Z=1 (bit is clear)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0x00
        *moveq(0, 1),                            # D1 = bit number 0
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit0 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_bit31(dut):
    """BTST D1,D0: test bit 31 of 0x80000000 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),                    # D0 = 0x80000000
        *moveq(31, 1),                            # D1 = bit number 31
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit31 of 0x80000000", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_bit7(dut):
    """BTST D1,D0: test bit 7 of 0x80 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x80 - 256, 0),                   # D0 = 0xFFFFFF80 (MOVEQ sign-extends)
        *moveq(7, 1),                             # D1 = bit number 7
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit7 of 0x80", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_bit15(dut):
    """BTST D1,D0: test bit 15 of 0x00008000 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00008000),                    # D0 = 0x8000
        *moveq(15, 1),                            # D1 = bit 15
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit15 of 0x8000", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_bit15_clear(dut):
    """BTST D1,D0: test bit 15 of 0x00007FFF -> Z=1 (bit 15 is clear)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00007FFF),                    # D0 = 0x7FFF
        *moveq(15, 1),                            # D1 = bit 15
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit15 of 0x7FFF", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_modulo32(dut):
    """BTST D1,D0: bit number 32 wraps to bit 0 (modulo 32). Test bit 0 of 0x01 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *moveq(32, 1),                            # D1 = 32 (mod 32 = bit 0)
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit32 (mod 32=0) of 0x01", ccr, 0)
    h.cleanup()


# =========================================================================
# BTST immediate form tests
# =========================================================================

@cocotb.test()
async def test_btst_imm_bit0_set(dut):
    """BTST #0,D0: test bit 0 of 0x01 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *btst_imm(DN, 0, 0),                      # BTST #0,D0
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
    assert_z_flag("BTST #0 of 0x01", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_imm_bit0_clear(dut):
    """BTST #0,D0: test bit 0 of 0x00 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0x00
        *btst_imm(DN, 0, 0),                      # BTST #0,D0
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
    assert_z_flag("BTST #0 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_btst_imm_bit31(dut):
    """BTST #31,D0: test bit 31 of 0x80000000 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),                    # D0 = 0x80000000
        *btst_imm(DN, 0, 31),                     # BTST #31,D0
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
    assert_z_flag("BTST #31 of 0x80000000", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_imm_bit7(dut):
    """BTST #7,D0: test bit 7 of 0x00000080 -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x80 - 256, 0),                   # D0 = 0xFFFFFF80 (bit 7 is set)
        *btst_imm(DN, 0, 7),                      # BTST #7,D0
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
    assert_z_flag("BTST #7 of 0x80", ccr, 0)
    h.cleanup()


# =========================================================================
# BSET register form tests
# =========================================================================

@cocotb.test()
async def test_bset_reg_set_clear_bit(dut):
    """BSET D1,D0: set bit 0 of 0x00 -> Z=1 (old bit was 0), D0 becomes 0x01."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0x00
        *moveq(0, 1),                            # D1 = bit number 0
        *bset_reg(1, DN, 0),                      # BSET D1,D0
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 0, AN_IND, 0),           # store result
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),           # store CCR
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x01, f"Expected 0x01, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BSET bit0 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bset_reg_set_already_set_bit(dut):
    """BSET D1,D0: set bit 0 of 0x01 -> Z=0 (old bit was 1), D0 stays 0x01."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *moveq(0, 1),                            # D1 = bit number 0
        *bset_reg(1, DN, 0),                      # BSET D1,D0
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
    assert result == 0x01, f"Expected 0x01, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BSET bit0 of 0x01", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_bset_reg_bit31(dut):
    """BSET D1,D0: set bit 31 of 0x00 -> Z=1, D0 = 0x80000000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *moveq(31, 1),                            # D1 = bit 31
        *bset_reg(1, DN, 0),                      # BSET D1,D0
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
    assert_z_flag("BSET bit31 of 0x00", ccr, 1)
    h.cleanup()


# =========================================================================
# BSET immediate form tests
# =========================================================================

@cocotb.test()
async def test_bset_imm_set_clear_bit(dut):
    """BSET #4,D0: set bit 4 of 0x00 -> Z=1, D0 = 0x10."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *bset_imm(DN, 0, 4),                      # BSET #4,D0
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
    assert result == 0x10, f"Expected 0x10, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BSET #4 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bset_imm_already_set(dut):
    """BSET #4,D0: set bit 4 of 0xFF -> Z=0 (already set), D0 stays 0xFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000FF),                    # D0 = 0xFF
        *bset_imm(DN, 0, 4),                      # BSET #4,D0
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
    assert result == 0xFF, f"Expected 0xFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BSET #4 of 0xFF", ccr, 0)
    h.cleanup()


# =========================================================================
# BCLR register form tests
# =========================================================================

@cocotb.test()
async def test_bclr_reg_clear_set_bit(dut):
    """BCLR D1,D0: clear bit 0 of 0x01 -> Z=0 (old bit was 1), D0 becomes 0x00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *moveq(0, 1),                            # D1 = bit number 0
        *bclr_reg(1, DN, 0),                      # BCLR D1,D0
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
    assert result == 0x00, f"Expected 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR bit0 of 0x01", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_bclr_reg_clear_already_clear(dut):
    """BCLR D1,D0: clear bit 0 of 0x00 -> Z=1 (old bit was 0), D0 stays 0x00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0x00
        *moveq(0, 1),                            # D1 = bit number 0
        *bclr_reg(1, DN, 0),                      # BCLR D1,D0
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
    assert result == 0x00, f"Expected 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR bit0 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bclr_reg_bit31(dut):
    """BCLR D1,D0: clear bit 31 of 0x80000000 -> Z=0, D0 = 0x00000000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x80000000),                    # D0 = 0x80000000
        *moveq(31, 1),                            # D1 = bit 31
        *bclr_reg(1, DN, 0),                      # BCLR D1,D0
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
    assert result == 0x00000000, f"Expected 0x00000000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR bit31 of 0x80000000", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_bclr_reg_bit15(dut):
    """BCLR D1,D0: clear bit 15 of 0x0000FFFF -> Z=0, D0 = 0x00007FFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),                    # D0 = 0xFFFF
        *moveq(15, 1),                            # D1 = bit 15
        *bclr_reg(1, DN, 0),                      # BCLR D1,D0
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
    assert result == 0x00007FFF, f"Expected 0x00007FFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR bit15 of 0xFFFF", ccr, 0)
    h.cleanup()


# =========================================================================
# BCLR immediate form tests
# =========================================================================

@cocotb.test()
async def test_bclr_imm_clear_set_bit(dut):
    """BCLR #3,D0: clear bit 3 of 0x0F -> Z=0, D0 = 0x07."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x0F, 0),                         # D0 = 0x0F
        *bclr_imm(DN, 0, 3),                      # BCLR #3,D0
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
    assert result == 0x07, f"Expected 0x07, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR #3 of 0x0F", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_bclr_imm_clear_clear_bit(dut):
    """BCLR #3,D0: clear bit 3 of 0x07 -> Z=1 (already clear), D0 stays 0x07."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x07, 0),                         # D0 = 0x07
        *bclr_imm(DN, 0, 3),                      # BCLR #3,D0
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
    assert result == 0x07, f"Expected 0x07, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR #3 of 0x07", ccr, 1)
    h.cleanup()


# =========================================================================
# BCHG register form tests
# =========================================================================

@cocotb.test()
async def test_bchg_reg_toggle_clear_bit(dut):
    """BCHG D1,D0: toggle bit 0 of 0x00 -> Z=1 (old was 0), D0 = 0x01."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0x00
        *moveq(0, 1),                            # D1 = bit 0
        *bchg_reg(1, DN, 0),                      # BCHG D1,D0
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
    assert result == 0x01, f"Expected 0x01, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCHG bit0 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bchg_reg_toggle_set_bit(dut):
    """BCHG D1,D0: toggle bit 0 of 0x01 -> Z=0 (old was 1), D0 = 0x00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                            # D0 = 0x01
        *moveq(0, 1),                            # D1 = bit 0
        *bchg_reg(1, DN, 0),                      # BCHG D1,D0
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
    assert result == 0x00, f"Expected 0x00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCHG bit0 of 0x01", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_bchg_reg_bit31(dut):
    """BCHG D1,D0: toggle bit 31 of 0x00 -> Z=1, D0 = 0x80000000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *moveq(31, 1),                            # D1 = bit 31
        *bchg_reg(1, DN, 0),                      # BCHG D1,D0
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
    assert_z_flag("BCHG bit31 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bchg_reg_bit7(dut):
    """BCHG D1,D0: toggle bit 7 of 0xFF -> Z=0 (old was 1), D0 = 0x7F."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x000000FF),                    # D0 = 0xFF
        *moveq(7, 1),                            # D1 = bit 7
        *bchg_reg(1, DN, 0),                      # BCHG D1,D0
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
    assert result == 0x7F, f"Expected 0x7F, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCHG bit7 of 0xFF", ccr, 0)
    h.cleanup()


# =========================================================================
# BCHG immediate form tests
# =========================================================================

@cocotb.test()
async def test_bchg_imm_toggle_clear(dut):
    """BCHG #15,D0: toggle bit 15 of 0x00 -> Z=1, D0 = 0x8000."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *bchg_imm(DN, 0, 15),                     # BCHG #15,D0
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
    assert result == 0x00008000, f"Expected 0x00008000, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCHG #15 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bchg_imm_toggle_set(dut):
    """BCHG #15,D0: toggle bit 15 of 0xFFFF -> Z=0, D0 = 0x7FFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),                    # D0 = 0xFFFF
        *bchg_imm(DN, 0, 15),                     # BCHG #15,D0
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
    assert result == 0x00007FFF, f"Expected 0x00007FFF, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCHG #15 of 0xFFFF", ccr, 0)
    h.cleanup()


# =========================================================================
# BSET/BCLR/BCHG additional tests for bit positions
# =========================================================================

@cocotb.test()
async def test_bset_imm_bit31(dut):
    """BSET #31,D0: set bit 31 of 0x7FFFFFFF -> Z=0 (bit was 0), D0 = 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x7FFFFFFF),                    # D0 = 0x7FFFFFFF
        *bset_imm(DN, 0, 31),                     # BSET #31,D0
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
    assert_z_flag("BSET #31 of 0x7FFFFFFF", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bclr_imm_bit31(dut):
    """BCLR #31,D0: clear bit 31 of 0xFFFFFFFF -> Z=0, D0 = 0x7FFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(-1, 0),                           # D0 = 0xFFFFFFFF
        *bclr_imm(DN, 0, 31),                     # BCLR #31,D0
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
    assert_z_flag("BCLR #31 of 0xFFFFFFFF", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_reg_all_bits_clear(dut):
    """BTST D1,D0: test bit 16 of 0x00000000 -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *moveq(16, 1),                            # D1 = bit 16
        *btst_reg(1, DN, 0),                      # BTST D1,D0
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
    assert_z_flag("BTST bit16 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bchg_imm_bit0(dut):
    """BCHG #0,D0: toggle bit 0 of 0xFFFFFFFE -> Z=1 (was 0), D0 = 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFFFFFFFE),                    # D0 = 0xFFFFFFFE (bit 0 = 0)
        *bchg_imm(DN, 0, 0),                      # BCHG #0,D0
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
    assert_z_flag("BCHG #0 of 0xFFFFFFFE", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bset_reg_bit7(dut):
    """BSET D1,D0: set bit 7 of 0x00 -> Z=1 (was 0), D0 = 0x80."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                            # D0 = 0
        *moveq(7, 1),                            # D1 = bit 7
        *bset_reg(1, DN, 0),                      # BSET D1,D0
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
    assert result == 0x80, f"Expected 0x80, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BSET bit7 of 0x00", ccr, 1)
    h.cleanup()


@cocotb.test()
async def test_bclr_reg_bit7(dut):
    """BCLR D1,D0: clear bit 7 of 0x80 -> Z=0 (was 1), D0 = 0x00."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x80 - 256, 0),                   # D0 = 0xFFFFFF80
        *moveq(7, 1),                            # D1 = bit 7
        *bclr_reg(1, DN, 0),                      # BCLR D1,D0
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
    assert result == 0xFFFFFF00, f"Expected 0xFFFFFF00, got 0x{result:08X}"
    ccr = h.read_result_long(4)
    assert_z_flag("BCLR bit7 of 0x80", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_imm_bit15_set(dut):
    """BTST #15,D0: test bit 15 of 0xFFFF -> Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000FFFF),
        *btst_imm(DN, 0, 15),                     # BTST #15,D0
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
    assert_z_flag("BTST #15 of 0xFFFF", ccr, 0)
    h.cleanup()


@cocotb.test()
async def test_btst_imm_bit15_clear(dut):
    """BTST #15,D0: test bit 15 of 0x7FFF -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00007FFF),
        *btst_imm(DN, 0, 15),                     # BTST #15,D0
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
    assert_z_flag("BTST #15 of 0x7FFF", ccr, 1)
    h.cleanup()
