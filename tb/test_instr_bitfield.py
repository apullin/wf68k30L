"""
Bit-field instruction compliance tests for WF68K30L.

Covers all eight MC68020+ bit-field instructions - BFTST, BFEXTU, BFEXTS,
BFCHG, BFCLR, BFSET, BFFFO, BFINS - with register and memory operands, with
the offset and the width taken both from the instruction and from a data
register, with width 0 meaning 32, and with fields that straddle byte
boundaries in memory (including the five-byte access a 32-bit field at a
nonzero bit offset requires).

None of the eight had any instruction-level coverage before this module.

Bit numbering (PRM section 4, bit-field instruction descriptions): offset 0 is
the most significant bit of the operand.  For a data register that is bit 31 of
Dn; for a memory operand it is the MSB of the byte at the effective address,
and increasing offsets walk towards higher addresses.  Offsets into a data
register are taken modulo 32; offsets into memory are not, so <ea>{32:8}
selects the byte one past the effective address.

Condition codes (PRM Table 3-11):
  BFTST/BFCHG/BFCLR/BFSET   N = field MSB, Z = field is zero, V = 0, C = 0
  BFEXTS/BFEXTU/BFFFO       N = source field MSB, Z = source field is zero
  BFINS                     N and Z come from the *inserted* value
  X is never affected.
"BFTST" semantics apply to the read-modify-write forms too: PRM section 3,
Table 3-7 note - "All bit field instructions set the CCR N and Z bits as shown
for BFTST before performing the specified operation."

Every expected value in this module was cross-checked against
`qemu-system-m68k -cpu m68030`.

Two RTL defects are left failing here on purpose, both in files owned by other
agents.  Each affected test carries the defect tag in its name.

BF-2 - sub-long bit-field memory operands are misaligned.
  `sv/wf68k30L_ctrl_comb.sv:456-457` sizes a bit-field memory access as LONG
  only when `BF_BYTES > 2`, so a field that fits in one byte uses a BYTE cycle
  and one that fits in two uses a WORD cycle.  Those cycles deliver the data
  right-justified in `DATA_TO_CORE`, while the ALU's 40-bit window
  `BF_DATA_IN = {OP3, OP2[7:0]}` (`sv/wf68k30L_alu.sv:329`) requires the byte at
  the effective address to be at `OP3[31:24]`.  Measured for
  `BFEXTU (A1){0:8},D2` over 12 34 56 78: bus read is `SIZE=1` at 0x010000,
  `OP3 = 0x00000012`, `BF_DATA_IN = 0x0000001200`, and with `BF_UPPER_BND = 39`
  / `BF_LOWER_BND = 32` the ALU extracts bits 39:32 = 0x00.  Result 0, Z=1;
  required 0x12, Z=0.  For `BFCLR (A1){0:16}` the same misalignment makes
  `BF_DATA_IN & ~bf_mask` equal to `BF_DATA_IN`, so the word write puts the
  original 0x1234 back and memory never changes.  Three-, four- and five-byte
  accesses already promote to a LONG read and are correct, which is why only
  fields of width <= 16 at small offsets fail.
  Verified fix (applied temporarily in this worktree, then reverted): change
  both `BF_BYTES > 2` terms at ctrl_comb.sv:456-457 to
  `BF_BYTES > 0 && BF_HILOn`.  The `BF_HILOn` term keeps the five-byte tail
  access a BYTE read.  That alone turns 12 of the failures here green.

BF-3 - the field offset, the field width and the BFINS insert value share one
  register-file read port.
  `sv/wf68k30L_operand_mux.sv:452-453` sources `BF_OFFSET` (when Do=1) and
  `BF_WIDTH` (when Dw=1) from the same `DR_OUT_1`, and
  `sv/wf68k30L_ctrl_regsel.sv:257-258` drives that port's selector from the
  offset-register field `BIW_1[8:6]`.  Consequences measured on the RTL:
    - `BFEXTU D1{D4:D5},D2` with D1=0x12345678, D4=4, D5=8 gives D2=0x00000002.
      The offset is right (4) but the width comes from D4, so the field
      evaluated is {4:4} = 0x2 rather than {4:8} = 0x23.
    - a memory-destination `BFINS` reads its insert value from
      `D[BIW_1[8:6]]`, which for an immediate offset is the low three bits of
      that offset.  `BFINS D3,(A1){0:8}` with D3=0xA5 selects D0 (=0) and
      writes 0x00 over the byte; `BFINS D3,(A1){4:16}` selects D4 (=0).
      The register-destination BFINS forms are unaffected because an earlier
      arm of the same selector picks `BIW_0[2:0]` while FETCH_STATE=START_OP.
  Fixing this needs a second read port (or a two-phase load) for the width and
  insert operands, in `operand_mux.sv` and `ctrl_regsel.sv`.  Faithfully ported
  from `vhdl/wf68k30L_top.vhd:685-686`, so not a port regression.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    moveq, move, movea, nop, addq,
    bftst, bfextu, bfexts, bfchg, bfclr, bfset, bfffo, bfins,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word,
)
from m68k_reference import extract_cc


def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value."""
    return extract_cc(val & 0xFF)


def assert_nzvc(name, actual_ccr, n, z):
    """Bit-field ops set N and Z and always clear V and C; X is untouched."""
    ax, an, az, av, ac = ccr_flags(actual_ccr)
    got = (f"(CCR=0x{actual_ccr & 0xFF:02X}: N={an} Z={az} V={av} C={ac})")
    assert an == n, f"{name}: N expected {n}, got {an} {got}"
    assert az == z, f"{name}: Z expected {z}, got {az} {got}"
    assert av == 0, f"{name}: V must be cleared, got {av} {got}"
    assert ac == 0, f"{name}: C must be cleared, got {ac} {got}"


def _load_long(dn, value):
    return [*move(LONG, SPECIAL, IMMEDIATE, DN, dn), *imm_long(value)]


def _capture(dn_list):
    """CCR first, then each named data register, through the A0 cursor."""
    words = [*nop(), *nop(), *move_from_ccr(DN, 6)]
    for dn in dn_list:
        words += [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    words += [*move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    return words


async def _run_reg(dut, setup, instr, capture_regs):
    """Run a register-operand bit-field test; return (values, ccr)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *setup,
        *instr,
        *_capture(capture_regs),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    values = [h.read_result_long(4 * i) for i in range(len(capture_regs))]
    ccr = h.read_result_long(4 * len(capture_regs))
    h.cleanup()
    return values, ccr


# =========================================================================
# BFTST
# =========================================================================

@cocotb.test()
async def test_bftst_reg_full_width_negative(dut):
    """BFTST D1{0:32} on 0x80000000: offset 0 is bit 31, so N=1."""
    _, ccr = await _run_reg(dut, _load_long(1, 0x80000000),
                            bftst(DN, 1, 0, 0), [1])
    assert_nzvc("BFTST D1{0:32} 0x80000000", ccr, n=1, z=0)


@cocotb.test()
async def test_bftst_reg_full_width_zero(dut):
    """BFTST D1{0:32} on 0x00000000: Z=1."""
    _, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                            bftst(DN, 1, 0, 0), [1])
    assert_nzvc("BFTST D1{0:32} zero", ccr, n=0, z=1)


@cocotb.test()
async def test_bftst_reg_narrow_field_msb(dut):
    """BFTST D1{4:4} on 0x08000000: the field is bits 27:24 = 1000, N=1."""
    _, ccr = await _run_reg(dut, _load_long(1, 0x08000000),
                            bftst(DN, 1, 4, 4), [1])
    assert_nzvc("BFTST D1{4:4} 0x08000000", ccr, n=1, z=0)


@cocotb.test()
async def test_bftst_reg_field_outside_set_bits(dut):
    """BFTST D1{8:8} on 0xFF0000FF: the selected byte is zero, Z=1.

    Set bits on both sides of the field must not leak into the flags.
    """
    _, ccr = await _run_reg(dut, _load_long(1, 0xFF0000FF),
                            bftst(DN, 1, 8, 8), [1])
    assert_nzvc("BFTST D1{8:8} 0xFF0000FF", ccr, n=0, z=1)


@cocotb.test()
async def test_bftst_reg_does_not_modify_operand(dut):
    """BFTST leaves the operand alone."""
    vals, _ = await _run_reg(dut, _load_long(1, 0x12345678),
                             bftst(DN, 1, 4, 8), [1])
    assert vals[0] == 0x12345678, f"BFTST modified D1: 0x{vals[0]:08X}"


@cocotb.test()
async def test_bftst_reg_v_and_c_cleared_from_set(dut):
    """BFTST clears V and C even when they were set beforehand."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001),
        *move_to_ccr(SPECIAL, IMMEDIATE), *imm_word(0x1F),  # X N Z V C all set
        *bftst(DN, 1, 0, 8),
        *_capture([1]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(4)
    assert_nzvc("BFTST V/C clear", ccr, n=0, z=1)
    x, n, z, v, c = ccr_flags(ccr)
    assert x == 1, f"BFTST must not affect X, got X={x}"
    h.cleanup()


# =========================================================================
# BFEXTU / BFEXTS
# =========================================================================

@cocotb.test()
async def test_bfextu_reg_byte_field(dut):
    """BFEXTU D1{0:8},D2 on 0xA5000000 -> D2 = 0x000000A5, N=1."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xA5000000),
                               bfextu(DN, 1, 0, 8, 2), [2])
    assert vals[0] == 0x000000A5, f"expected 0xA5, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU D1{0:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfextu_reg_straddling_nibbles(dut):
    """BFEXTU D1{4:8},D2 on 0x12345678 -> 0x23 (bits 27:20)."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x12345678),
                               bfextu(DN, 1, 4, 8, 2), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU D1{4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfextu_reg_width_zero_is_32(dut):
    """BFEXTU D1{0:32},D2: width field 0 selects a 32-bit field."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xDEADBEEF),
                               bfextu(DN, 1, 0, 0, 2), [2])
    assert vals[0] == 0xDEADBEEF, f"expected 0xDEADBEEF, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU D1{0:32}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfextu_reg_last_nibble(dut):
    """BFEXTU D1{28:4},D2 on 0x0000000F -> 0x0F, N=1 (field MSB set)."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x0000000F),
                               bfextu(DN, 1, 28, 4, 2), [2])
    assert vals[0] == 0x0000000F, f"expected 0x0F, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU D1{28:4}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfextu_reg_zero_extends(dut):
    """BFEXTU zero-fills above the field: 0xFF...->0x000000FF, not sign-extended."""
    vals, _ = await _run_reg(dut, _load_long(1, 0xFF000000),
                             bfextu(DN, 1, 0, 8, 2), [2])
    assert vals[0] == 0x000000FF, f"expected 0x000000FF, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfexts_reg_sign_extends(dut):
    """BFEXTS D1{0:8},D2 on 0xA5000000 -> 0xFFFFFFA5 (field MSB is 1)."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xA5000000),
                               bfexts(DN, 1, 0, 8, 2), [2])
    assert vals[0] == 0xFFFFFFA5, f"expected 0xFFFFFFA5, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS D1{0:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfexts_reg_positive_field(dut):
    """BFEXTS D1{4:8},D2 on 0x12345678 -> 0x23, no sign extension."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x12345678),
                               bfexts(DN, 1, 4, 8, 2), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS D1{4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfexts_reg_narrow_negative(dut):
    """BFEXTS D1{0:4},D2 on 0x90000000 -> 0xFFFFFFF9 from a 4-bit field."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x90000000),
                               bfexts(DN, 1, 0, 4, 2), [2])
    assert vals[0] == 0xFFFFFFF9, f"expected 0xFFFFFFF9, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS D1{0:4} 0x9", ccr, n=1, z=0)


@cocotb.test()
async def test_bfexts_reg_zero_field(dut):
    """BFEXTS of an all-zero field gives 0 with Z=1."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xFF00FFFF),
                               bfexts(DN, 1, 8, 8, 2), [2])
    assert vals[0] == 0x00000000, f"expected 0, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS zero field", ccr, n=0, z=1)


# =========================================================================
# BFFFO
# =========================================================================

@cocotb.test()
async def test_bfffo_reg_scans_from_field_msb(dut):
    """BFFFO D1{0:32},D2 on 0x00008000 -> 16.

    PRM BFFFO: "Searches the source operand for the most significant bit that
    is set to a value of one."  Bit 15 of D1 is 16 offsets down from bit 31.
    """
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00008000),
                               bfffo(DN, 1, 0, 0, 2), [2])
    assert vals[0] == 16, f"expected 16, got {vals[0]}"
    assert_nzvc("BFFFO 0x00008000", ccr, n=0, z=0)


@cocotb.test()
async def test_bfffo_reg_msb_set(dut):
    """BFFFO D1{0:32},D2 on 0x80000000 -> 0."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x80000000),
                               bfffo(DN, 1, 0, 0, 2), [2])
    assert vals[0] == 0, f"expected 0, got {vals[0]}"
    assert_nzvc("BFFFO 0x80000000", ccr, n=1, z=0)


@cocotb.test()
async def test_bfffo_reg_no_bit_set_returns_offset_plus_width(dut):
    """BFFFO D1{0:32},D2 on 0: "the value in Dn is the field offset plus the
    field width", i.e. 0 + 32 = 32, with Z=1."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                               bfffo(DN, 1, 0, 0, 2), [2])
    assert vals[0] == 32, f"expected 32, got {vals[0]}"
    assert_nzvc("BFFFO zero field", ccr, n=0, z=1)


@cocotb.test()
async def test_bfffo_reg_offset_plus_width_nonzero_offset(dut):
    """BFFFO D1{8:8},D2 on 0: result is 8 + 8 = 16."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                               bfffo(DN, 1, 8, 8, 2), [2])
    assert vals[0] == 16, f"expected 16, got {vals[0]}"
    assert_nzvc("BFFFO empty {8:8}", ccr, n=0, z=1)


@cocotb.test()
async def test_bfffo_reg_result_includes_instruction_offset(dut):
    """BFFFO D1{8:16},D2 on 0x00010000 -> 15.

    The result is "the bit offset in the instruction plus the offset of the
    first one bit": bit 16 of D1 is offset 15 counting from bit 31.
    """
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00010000),
                               bfffo(DN, 1, 8, 16, 2), [2])
    assert vals[0] == 15, f"expected 15, got {vals[0]}"
    assert_nzvc("BFFFO {8:16} 0x00010000", ccr, n=0, z=0)


@cocotb.test()
async def test_bfffo_reg_ignores_bits_outside_field(dut):
    """BFFFO D1{4:8},D2 on 0x81000000 -> 7: bit 31 is outside {4:8}.

    The field is bits 27:20; the only set bit inside it is bit 24, which is
    offset 7.  Bit 31 must be masked off, otherwise the answer would be 0.
    """
    vals, ccr = await _run_reg(dut, _load_long(1, 0x81000000),
                               bfffo(DN, 1, 4, 8, 2), [2])
    assert vals[0] == 7, f"expected 7, got {vals[0]}"
    assert_nzvc("BFFFO {4:8} 0x81000000", ccr, n=0, z=0)


# =========================================================================
# BFCHG / BFCLR / BFSET on a register
# =========================================================================

@cocotb.test()
async def test_bfchg_reg_byte_field(dut):
    """BFCHG D1{0:8} on 0xFF00FF00 -> 0x0000FF00; N=1 from the pre-change field."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xFF00FF00),
                               bfchg(DN, 1, 0, 8), [1])
    assert vals[0] == 0x0000FF00, f"expected 0x0000FF00, got 0x{vals[0]:08X}"
    assert_nzvc("BFCHG D1{0:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfchg_reg_flags_are_pre_modification(dut):
    """BFCHG D1{4:8} on 0 -> 0x0FF00000 with Z=1 from the field *before* the change.

    Table 3-7 note: "All bit field instructions set the CCR N and Z bits as
    shown for BFTST before performing the specified operation."  The field is
    all ones afterwards, so a Z taken after the write would be 0.
    """
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                               bfchg(DN, 1, 4, 8), [1])
    assert vals[0] == 0x0FF00000, f"expected 0x0FF00000, got 0x{vals[0]:08X}"
    assert_nzvc("BFCHG pre-modification flags", ccr, n=0, z=1)


@cocotb.test()
async def test_bfclr_reg_word_field(dut):
    """BFCLR D1{0:16} on 0xFFFFFFFF -> 0x0000FFFF, N=1 from the old field."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xFFFFFFFF),
                               bfclr(DN, 1, 0, 16), [1])
    assert vals[0] == 0x0000FFFF, f"expected 0x0000FFFF, got 0x{vals[0]:08X}"
    assert_nzvc("BFCLR D1{0:16}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfclr_reg_last_nibble(dut):
    """BFCLR D1{28:4} on 0xFFFFFFFF -> 0xFFFFFFF0."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0xFFFFFFFF),
                               bfclr(DN, 1, 28, 4), [1])
    assert vals[0] == 0xFFFFFFF0, f"expected 0xFFFFFFF0, got 0x{vals[0]:08X}"
    assert_nzvc("BFCLR D1{28:4}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfset_reg_byte_field(dut):
    """BFSET D1{8:8} on 0 -> 0x00FF0000, Z=1 from the old (zero) field."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                               bfset(DN, 1, 8, 8), [1])
    assert vals[0] == 0x00FF0000, f"expected 0x00FF0000, got 0x{vals[0]:08X}"
    assert_nzvc("BFSET D1{8:8}", ccr, n=0, z=1)


@cocotb.test()
async def test_bfset_reg_full_width(dut):
    """BFSET D1{0:32} on 0 -> 0xFFFFFFFF."""
    vals, ccr = await _run_reg(dut, _load_long(1, 0x00000000),
                               bfset(DN, 1, 0, 0), [1])
    assert vals[0] == 0xFFFFFFFF, f"expected 0xFFFFFFFF, got 0x{vals[0]:08X}"
    assert_nzvc("BFSET D1{0:32}", ccr, n=0, z=1)


# =========================================================================
# BFINS on a register
# =========================================================================

@cocotb.test()
async def test_bfins_reg_byte_field(dut):
    """BFINS D3,D1{0:8} with D3=0xA5 into a zero register -> 0xA5000000, N=1."""
    vals, ccr = await _run_reg(dut,
                               _load_long(1, 0x00000000) + _load_long(3, 0xA5),
                               bfins(3, DN, 1, 0, 8), [1])
    assert vals[0] == 0xA5000000, f"expected 0xA5000000, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS D3,D1{0:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_reg_straddling(dut):
    """BFINS D3,D1{4:8} with D3=0x5A into 0xFFFFFFFF -> 0xF5AFFFFF."""
    vals, ccr = await _run_reg(dut,
                               _load_long(1, 0xFFFFFFFF) + _load_long(3, 0x5A),
                               bfins(3, DN, 1, 4, 8), [1])
    assert vals[0] == 0xF5AFFFFF, f"expected 0xF5AFFFFF, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS D3,D1{4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfins_reg_twelve_bit_field(dut):
    """BFINS D3,D1{12:12} with D3=0xABC into 0 -> 0x000ABC00."""
    vals, ccr = await _run_reg(dut,
                               _load_long(1, 0x00000000) + _load_long(3, 0xABC),
                               bfins(3, DN, 1, 12, 12), [1])
    assert vals[0] == 0x000ABC00, f"expected 0x000ABC00, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS D3,D1{12:12}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_reg_flags_from_inserted_value_zero(dut):
    """BFINS flags come from the inserted value: inserting 0 over ones gives Z=1.

    PRM BFINS: "The instruction sets the condition codes according to the
    inserted value."  This is the one bit-field instruction whose N and Z are
    not taken from the destination field before the operation.
    """
    vals, ccr = await _run_reg(dut,
                               _load_long(1, 0xFFFFFFFF) + _load_long(3, 0x00),
                               bfins(3, DN, 1, 0, 8), [1])
    assert vals[0] == 0x00FFFFFF, f"expected 0x00FFFFFF, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS zero into ones", ccr, n=0, z=1)


@cocotb.test()
async def test_bfins_reg_flags_from_inserted_value_negative(dut):
    """BFINS of 0x80 into a zero field: N=1 from the inserted value's MSB."""
    vals, ccr = await _run_reg(dut,
                               _load_long(1, 0x00000000) + _load_long(3, 0x80),
                               bfins(3, DN, 1, 0, 8), [1])
    assert vals[0] == 0x80000000, f"expected 0x80000000, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS 0x80 into zeros", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_reg_full_width(dut):
    """BFINS D3,D1{0:32} replaces the whole register."""
    vals, _ = await _run_reg(dut,
                             _load_long(1, 0x00000000) + _load_long(3, 0xDEADBEEF),
                             bfins(3, DN, 1, 0, 0), [1])
    assert vals[0] == 0xDEADBEEF, f"expected 0xDEADBEEF, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfins_reg_uses_only_low_bits_of_source(dut):
    """BFINS takes the field from the low-order bits of Dn, ignoring the rest."""
    vals, _ = await _run_reg(dut,
                             _load_long(1, 0x00000000) + _load_long(3, 0xFFFFFFA5),
                             bfins(3, DN, 1, 0, 8), [1])
    assert vals[0] == 0xA5000000, f"expected 0xA5000000, got 0x{vals[0]:08X}"


# =========================================================================
# Offset and width from data registers
# =========================================================================

@cocotb.test()
async def test_bfextu_reg_offset_and_width_from_registers(dut):
    """BFEXTU D1{D4:D5},D2 with D4=4, D5=8 matches the immediate form.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, ccr = await _run_reg(
        dut,
        _load_long(1, 0x12345678) + _load_long(4, 4) + _load_long(5, 8),
        bfextu(DN, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU D1{D4:D5}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfextu_reg_offset_modulo_32(dut):
    """A register offset of 36 selects the same field as 4 on a Dn operand.

    PRM: for a data register operand the offset is taken modulo 32.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, _ = await _run_reg(
        dut,
        _load_long(1, 0x12345678) + _load_long(4, 36) + _load_long(5, 8),
        bfextu(DN, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_reg_width_from_register_zero_is_32(dut):
    """A register width of 0 means 32.

    PRM: "If Dw = 1, the width field specifies a data register that contains
    the width.  The value is modulo 32; values of 1-31 specify field widths of
    1-31, and a value of zero specifies a width of 32."
    """
    vals, ccr = await _run_reg(
        dut,
        _load_long(1, 0xDEADBEEF) + _load_long(4, 0) + _load_long(5, 0),
        bfextu(DN, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0xDEADBEEF, f"expected 0xDEADBEEF, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU width Dn=0", ccr, n=1, z=0)


@cocotb.test()
async def test_bfextu_reg_width_from_register_modulo_32(dut):
    """A register width of 32 is 0 modulo 32 and therefore also means 32."""
    vals, _ = await _run_reg(
        dut,
        _load_long(1, 0xDEADBEEF) + _load_long(4, 32) + _load_long(5, 32),
        bfextu(DN, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0xDEADBEEF, f"expected 0xDEADBEEF, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_reg_negative_offset_wraps(dut):
    """A register offset of -4 on a Dn operand selects bits 3:0 (offset 28).

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, ccr = await _run_reg(
        dut,
        _load_long(1, 0x12345678) + _load_long(4, 0xFFFFFFFC) + _load_long(5, 4),
        bfextu(DN, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0x00000008, f"expected 0x8, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU offset -4", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_reg_offset_and_width_from_registers(dut):
    """BFINS D3,D1{D4:D5} with a register-supplied offset and width."""
    vals, ccr = await _run_reg(
        dut,
        _load_long(1, 0x00000000) + _load_long(3, 0xABC) +
        _load_long(4, 12) + _load_long(5, 12),
        bfins(3, DN, 1, 4, 5, do=1, dw=1), [1])
    assert vals[0] == 0x000ABC00, f"expected 0x000ABC00, got 0x{vals[0]:08X}"
    assert_nzvc("BFINS D1{D4:D5}", ccr, n=1, z=0)


# =========================================================================
# Memory operands
# =========================================================================

async def _run_mem(dut, fill, instr, capture_regs, readback=2, base_off=0):
    """Run a memory-operand bit-field test.

    fill: list of bytes written at DATA_BASE before the run.
    Returns (register values, ccr, memory bytes after the run).
    """
    h = CPUTestHarness(dut)
    data = h.DATA_BASE
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(data + base_off),
        *instr,
        *_capture(capture_regs),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate(fill):
        h.mem.write(data + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    values = [h.read_result_long(4 * i) for i in range(len(capture_regs))]
    ccr = h.read_result_long(4 * len(capture_regs))
    mem = [h.mem.read(data + i, 1) for i in range(readback)]
    h.cleanup()
    return values, ccr, mem


FILL = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]


@cocotb.test()
async def test_bftst_mem_byte_at_ea(dut):
    """BFTST (A1){0:8}: the field is the byte at the effective address.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, ccr, _ = await _run_mem(dut, FILL, bftst(AN_IND, 1, 0, 8), [])
    assert_nzvc("BFTST (A1){0:8} 0x12", ccr, n=0, z=0)


@cocotb.test()
async def test_bftst_mem_zero_field(dut):
    """BFTST (A1){8:8} over a zero byte gives Z=1."""
    _, ccr, _ = await _run_mem(dut, [0xFF, 0x00, 0xFF, 0xFF],
                               bftst(AN_IND, 1, 8, 8), [])
    assert_nzvc("BFTST (A1){8:8} 0x00", ccr, n=0, z=1)


@cocotb.test()
async def test_bfextu_mem_byte_at_ea(dut):
    """BFEXTU (A1){0:8},D2 -> 0x12, the byte at the effective address.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    vals, ccr, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 0, 8, 2), [2])
    assert vals[0] == 0x00000012, f"expected 0x12, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU (A1){0:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfextu_mem_straddles_byte_boundary(dut):
    """BFEXTU (A1){4:8},D2 spans the low nibble of byte 0 and the high of byte 1.

    Memory is 0x12 0x34, so the field is 0x23.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    vals, ccr, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 4, 8, 2), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU (A1){4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfextu_mem_twelve_bits_straddling(dut):
    """BFEXTU (A1){12:12},D2 over 0x12 0x34 0x56 -> 0x456.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    vals, _, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 12, 12, 2), [2])
    assert vals[0] == 0x00000456, f"expected 0x456, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_mem_long_aligned(dut):
    """BFEXTU (A1){0:32},D2 reads the aligned long 0x12345678."""
    vals, _, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 0, 0, 2), [2])
    assert vals[0] == 0x12345678, f"expected 0x12345678, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_mem_long_needs_five_bytes(dut):
    """BFEXTU (A1){4:32},D2 over 12 34 56 78 9A -> 0x23456789.

    PRM note under BFCHG: "all bit field instructions access only those bytes
    in memory that contain some portion of the bit field.  The possible
    accesses are byte, word, 3-byte, long word, and long word with byte (for a
    5-byte access)."  A 32-bit field at offset 4 is the five-byte case.
    """
    vals, _, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 4, 0, 2), [2])
    assert vals[0] == 0x23456789, f"expected 0x23456789, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_mem_single_bit(dut):
    """BFEXTU (A1){31:1},D2: bit 31 is the LSB of byte 3 (0x78) -> 0, Z=1."""
    vals, ccr, _ = await _run_mem(dut, FILL, bfextu(AN_IND, 1, 31, 1, 2), [2])
    assert vals[0] == 0x00000000, f"expected 0, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU (A1){31:1}", ccr, n=0, z=1)


@cocotb.test()
async def test_bfexts_mem_straddling(dut):
    """BFEXTS (A1){4:8},D2 -> 0x23 (positive field, no sign extension).

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    vals, ccr, _ = await _run_mem(dut, FILL, bfexts(AN_IND, 1, 4, 8, 2), [2])
    assert vals[0] == 0x00000023, f"expected 0x23, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS (A1){4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfexts_mem_field_spanning_long_boundary(dut):
    """BFEXTS (A1){31:8},D2 spans the last bit of the long and the next byte.

    0x78 = 0111 1000, so offset 31 is 0; 0x9A = 1001 1010 supplies the next
    seven bits, giving the field 0100 1101 = 0x4D.  This is a two-byte access
    that crosses the aligned long-word boundary at the effective address.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    vals, ccr, _ = await _run_mem(dut, FILL, bfexts(AN_IND, 1, 31, 8, 2), [2])
    assert vals[0] == 0x0000004D, f"expected 0x4D, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTS (A1){31:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfffo_mem_long_field(dut):
    """BFFFO (A1){0:32},D2 over 0x12345678: 0x12 = 0001 0010, first one at 3."""
    vals, ccr, _ = await _run_mem(dut, FILL, bfffo(AN_IND, 1, 0, 0, 2), [2])
    assert vals[0] == 3, f"expected 3, got {vals[0]}"
    assert_nzvc("BFFFO (A1){0:32}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfffo_mem_offset_field(dut):
    """BFFFO (A1){4:16},D2 over 0x2345: first one bit is 2 into the field -> 6."""
    vals, _, _ = await _run_mem(dut, FILL, bfffo(AN_IND, 1, 4, 16, 2), [2])
    assert vals[0] == 6, f"expected 6, got {vals[0]}"


@cocotb.test()
async def test_bfffo_mem_empty_field(dut):
    """BFFFO (A1){8:8},D2 over a zero byte gives offset+width = 16, Z=1."""
    vals, ccr, _ = await _run_mem(dut, [0xFF, 0x00, 0xFF, 0xFF],
                                  bfffo(AN_IND, 1, 8, 8, 2), [2])
    assert vals[0] == 16, f"expected 16, got {vals[0]}"
    assert_nzvc("BFFFO empty mem field", ccr, n=0, z=1)


@cocotb.test()
async def test_bfchg_mem_byte_at_ea(dut):
    """BFCHG (A1){0:8}: 0x12 becomes 0xED, rest of memory untouched.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, ccr, mem = await _run_mem(dut, FILL, bfchg(AN_IND, 1, 0, 8), [], readback=4)
    assert mem == [0xED, 0x34, 0x56, 0x78], (
        f"expected [ED,34,56,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFCHG (A1){0:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfchg_mem_straddling(dut):
    """BFCHG (A1){4:8} over 0x12 0x34 -> 0x1D 0xC4: only the field flips.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, ccr, mem = await _run_mem(dut, FILL, bfchg(AN_IND, 1, 4, 8), [], readback=4)
    assert mem == [0x1D, 0xC4, 0x56, 0x78], (
        f"expected [1D,C4,56,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFCHG (A1){4:8}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfclr_mem_word_field(dut):
    """BFCLR (A1){0:16} zeroes the first two bytes only.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, ccr, mem = await _run_mem(dut, FILL, bfclr(AN_IND, 1, 0, 16), [], readback=4)
    assert mem == [0x00, 0x00, 0x56, 0x78], (
        f"expected [00,00,56,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFCLR (A1){0:16}", ccr, n=0, z=0)


@cocotb.test()
async def test_bfclr_mem_straddling(dut):
    """BFCLR (A1){4:8} over 0x12 0x34 -> 0x10 0x04.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, _, mem = await _run_mem(dut, FILL, bfclr(AN_IND, 1, 4, 8), [], readback=4)
    assert mem == [0x10, 0x04, 0x56, 0x78], (
        f"expected [10,04,56,78], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfset_mem_straddling(dut):
    """BFSET (A1){4:8} over 0x12 0x34 -> 0x1F 0xF4.

    KNOWN RTL DEFECT BF-2 (see module docstring): a bit-field memory operand
    that spans only one or two bytes is read and written with a BYTE/WORD-sized
    bus cycle, and the data arrives right-justified in DATA_TO_CORE, but the
    ALU's 40-bit bit-field window BF_DATA_IN = {OP3, OP2[7:0]} requires the byte
    at the effective address to sit at OP3[31:24].  The field therefore reads as
    zero and a read-modify-write form writes the operand back unchanged.
"""
    _, _, mem = await _run_mem(dut, FILL, bfset(AN_IND, 1, 4, 8), [], readback=4)
    assert mem == [0x1F, 0xF4, 0x56, 0x78], (
        f"expected [1F,F4,56,78], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfset_mem_full_long(dut):
    """BFSET (A1){0:32} sets exactly four bytes."""
    _, _, mem = await _run_mem(dut, FILL, bfset(AN_IND, 1, 0, 0), [], readback=5)
    assert mem == [0xFF, 0xFF, 0xFF, 0xFF, 0x9A], (
        f"expected [FF,FF,FF,FF,9A], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfset_mem_five_byte_access(dut):
    """BFSET (A1){4:32} touches five bytes: 0x1F FF FF FF Fx."""
    _, _, mem = await _run_mem(dut, FILL, bfset(AN_IND, 1, 4, 0), [], readback=6)
    assert mem == [0x1F, 0xFF, 0xFF, 0xFF, 0xFA, 0xBC], (
        f"expected [1F,FF,FF,FF,FA,BC], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfins_mem_byte_at_ea(dut):
    """BFINS D3,(A1){0:8} with D3=0xA5 writes one byte; N=1 from the insert.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, ccr, mem = await _run_mem(dut, FILL,
                                    _load_long(3, 0xA5) + bfins(3, AN_IND, 1, 0, 8),
                                    [3], readback=4)
    assert mem == [0xA5, 0x34, 0x56, 0x78], (
        f"expected [A5,34,56,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFINS (A1){0:8} 0xA5", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_mem_straddling(dut):
    """BFINS D3,(A1){4:8} with D3=0x5A -> 0x15 0xA4.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    _, ccr, mem = await _run_mem(dut, FILL,
                                 _load_long(3, 0x5A) + bfins(3, AN_IND, 1, 4, 8),
                                 [3], readback=4)
    assert mem == [0x15, 0xA4, 0x56, 0x78], (
        f"expected [15,A4,56,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFINS (A1){4:8} 0x5A", ccr, n=0, z=0)


@cocotb.test()
async def test_bfins_mem_sixteen_bits_straddling(dut):
    """BFINS D3,(A1){4:16} with D3=0xBEEF -> 0x1B 0xEE 0xF6.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    _, ccr, mem = await _run_mem(dut, FILL,
                                 _load_long(3, 0xBEEF) + bfins(3, AN_IND, 1, 4, 16),
                                 [3], readback=4)
    assert mem == [0x1B, 0xEE, 0xF6, 0x78], (
        f"expected [1B,EE,F6,78], got {[f'{b:02X}' for b in mem]}")
    assert_nzvc("BFINS (A1){4:16} 0xBEEF", ccr, n=1, z=0)


@cocotb.test()
async def test_bfins_mem_full_long(dut):
    """BFINS D3,(A1){0:32} replaces the aligned long.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    _, _, mem = await _run_mem(dut, FILL,
                               _load_long(3, 0xDEADBEEF) + bfins(3, AN_IND, 1, 0, 0),
                               [3], readback=5)
    assert mem == [0xDE, 0xAD, 0xBE, 0xEF, 0x9A], (
        f"expected [DE,AD,BE,EF,9A], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfins_mem_five_byte_access(dut):
    """BFINS D3,(A1){4:32} writes across five bytes: 0x1D EA DB EE Fx.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    _, _, mem = await _run_mem(dut, FILL,
                               _load_long(3, 0xDEADBEEF) + bfins(3, AN_IND, 1, 4, 0),
                               [3], readback=6)
    assert mem == [0x1D, 0xEA, 0xDB, 0xEE, 0xFA, 0xBC], (
        f"expected [1D,EA,DB,EE,FA,BC], got {[f'{b:02X}' for b in mem]}")


@cocotb.test()
async def test_bfextu_mem_offset_past_bit31_is_not_modulo(dut):
    """BFEXTU (A1){D4:D5},D2 with D4=32: memory offsets are not taken modulo 32.

    PRM: "If Do = 1, the offset field specifies a data register that contains
    the offset.  The value is in the range of -2^31 to 2^31-1."  Offset 32 on a
    memory operand selects the byte one past the effective address (0x9A here);
    only Dn operands wrap.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, ccr, _ = await _run_mem(
        dut, FILL,
        _load_long(4, 32) + _load_long(5, 8) +
        bfextu(AN_IND, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0x0000009A, f"expected 0x9A, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU (A1){D4=32:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfextu_mem_offset_36_straddles(dut):
    """BFEXTU (A1){D4:D5},D2 with D4=36 -> 0xAB from 0x9A 0xBC.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, _, _ = await _run_mem(
        dut, FILL,
        _load_long(4, 36) + _load_long(5, 8) +
        bfextu(AN_IND, 1, 4, 5, 2, do=1, dw=1), [2])
    assert vals[0] == 0x000000AB, f"expected 0xAB, got 0x{vals[0]:08X}"


@cocotb.test()
async def test_bfextu_mem_negative_offset_reads_below_ea(dut):
    """A negative register offset reads below the effective address.

    A1 points at DATA_BASE+8 and D4 = -8, so the field is the byte at
    DATA_BASE+7 (0xF0).

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    vals, ccr, _ = await _run_mem(
        dut, FILL,
        _load_long(4, 0xFFFFFFF8) + _load_long(5, 8) +
        bfextu(AN_IND, 1, 4, 5, 2, do=1, dw=1), [2], base_off=8)
    assert vals[0] == 0x000000F0, f"expected 0xF0, got 0x{vals[0]:08X}"
    assert_nzvc("BFEXTU (A1){D4=-8:8}", ccr, n=1, z=0)


@cocotb.test()
async def test_bfclr_mem_offset_and_width_from_registers(dut):
    """BFCLR (A1){D4:D5} with a register offset and width clears 0x12 0x34.

    KNOWN RTL DEFECT BF-3 (see module docstring): the register-supplied field
    width, the register-supplied field offset and the BFINS insert value all
    read the same register file port, whose selector for bit-field operations is
    the offset-register field of the extension word.
"""
    _, _, mem = await _run_mem(
        dut, FILL,
        _load_long(4, 4) + _load_long(5, 8) +
        bfclr(AN_IND, 1, 4, 5, do=1, dw=1), [], readback=4)
    assert mem == [0x10, 0x04, 0x56, 0x78], (
        f"expected [10,04,56,78], got {[f'{b:02X}' for b in mem]}")
