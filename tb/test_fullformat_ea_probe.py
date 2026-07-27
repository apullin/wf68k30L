"""
Probe: full-format extension-word effective addresses with a suppressed base.

Reproducer for a divergence found by the csmith checksum gate. GCC compiles a
table lookup such as `crc32_tab[(x ^ b) & 0xFF]` into

    lea @(754,%d0:l:4),%a1      ; 43F0 0DB0 0000 0754
    movel %a1@,%d0

i.e. a full-format extension word with the base *register* suppressed (BS=1), a
long base displacement, and a long index scaled by 4. PRM 2.2 / Table 2-4
define the effective address as

    <bd> + Xn.SIZE * SCALE          (base register suppressed)

Every m68k C compiler emits this form for indexed access to a fixed array, so a
defect here corrupts ordinary compiled code while every hand-written
instruction test still passes.

The cases below publish both the computed address (via MOVE.L A1,Dn) and the
longword loaded through it, and pair each full-format case with a brief-format
control that must produce the same address.

Two further defects in this addressing form are known and NOT covered here,
because a test for either would fail today:

  - Base register suppression (BS = 1) is ignored unless the effective address
    is memory indirect: wf68k30L_address_registers.sv only forces the base mux
    to zero when FETCH_MEM_ADR is asserted, which happens solely in the
    FETCH_MEMADR state. The suppressed-base cases below therefore have to set
    A0 = 0 by hand, and real compiled code that leaves a live value in the base
    register reads through a wrong address. This is why the csmith checksum
    gate is still red: gcc emits `lea @(bd,%d0:l:4),%a1` with BS = 1 while %a0
    holds something else.
  - Index suppression (IS = 1) is also ignored: I_S_IS 4'b1000 still adds
    index_scaled_comb, so `(bd,An)` written in full format picks up a stray
    index term.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    DN,
    AN,
    AN_IDX,
    SPECIAL,
    IMMEDIATE,
    imm_long,
    imm_word,
    lsr,
    move,
    move_to_abs_long,
    movea,
    moveq,
    nop,
)


TABLE_BASE = CPUTestHarness.DATA_BASE + 0x340   # 0x00010340
INDEX = 3
TABLE_VALUE = 0x11223344                        # stored at TABLE_BASE + 12


def _publish(h, offset, an_to_dn=3):
    """MOVE.L A1,Dn ; store Dn ; MOVE.L (A1),Dm ; store Dm."""
    return [
        *move(LONG, AN, 1, DN, an_to_dn),
        *move_to_abs_long(LONG, DN, an_to_dn, h.RESULT_BASE + offset),
        *move(LONG, 2, 1, DN, 4),                 # MOVE.L (A1),D4
        *move_to_abs_long(LONG, DN, 4, h.RESULT_BASE + offset + 4),
    ]


@cocotb.test()
async def test_fullformat_suppressed_base_lea(dut):
    """LEA (bd,Dn.L*4) with BS=1 must compute bd + Dn*4."""
    h = CPUTestHarness(dut)

    program = [
        *moveq(INDEX, 0),                          # D0 = 3 (index)
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = 0 (must be ignored)
        *imm_long(0x00000000),
        # LEA (0x00010340,D0.L*4),A1  -- full format, base register suppressed
        0x43F0, 0x0DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        *_publish(h, 0),
        # Control: brief format with an explicit base register.
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = TABLE_BASE
        *imm_long(TABLE_BASE),
        # LEA (0,A0,D0.L*4),A1  -- brief format (bit 8 = 0), scale 4
        0x43F0, 0x0C00,
        *_publish(h, 8, an_to_dn=5),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"

    full_ea = h.read_result_long(0)
    full_val = h.read_result_long(4)
    brief_ea = h.read_result_long(8)
    brief_val = h.read_result_long(12)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert brief_ea == expect_ea, (
        f"Brief-format control is wrong too: (0,A0,D0.L*4) with A0=0x{TABLE_BASE:08X} "
        f"D0={INDEX} gave 0x{brief_ea:08X}, expected 0x{expect_ea:08X}"
    )
    assert brief_val == TABLE_VALUE, (
        f"Brief-format load returned 0x{brief_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )
    assert full_ea == expect_ea, (
        f"LEA (0x{TABLE_BASE:08X},D0.L*4),A1 with the base register suppressed "
        f"computed 0x{full_ea:08X}, expected 0x{expect_ea:08X} (bd + Dn*4). "
        f"The brief-format equivalent computed the correct 0x{brief_ea:08X}, so "
        f"the defect is in the full-format extension word path."
    )
    assert full_val == TABLE_VALUE, (
        f"Load through the full-format EA returned 0x{full_val:08X}, expected "
        f"0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_suppressed_base_move(dut):
    """MOVE.L (bd,Dn.L*4),Dn with BS=1 -- the load GCC pairs with the LEA."""
    h = CPUTestHarness(dut)

    program = [
        *moveq(INDEX, 0),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(0x00000000),
        # MOVE.L (0x00010340,D0.L*4),D2
        0x2430, 0x0DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got = h.read_result_long(0)
    h.cleanup()
    assert got == TABLE_VALUE, (
        f"MOVE.L (0x{TABLE_BASE:08X},D0.L*4),D2 with the base register "
        f"suppressed loaded 0x{got:08X}, expected 0x{TABLE_VALUE:08X} from "
        f"0x{TABLE_BASE + INDEX * 4:08X}"
    )


@cocotb.test()
async def test_fullformat_base_register_and_long_displacement(dut):
    """LEA (bd.L,An,Dn.L*4) with BS=0 -- same form, base register present."""
    h = CPUTestHarness(dut)

    program = [
        *moveq(INDEX, 0),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = 0x340
        *imm_long(TABLE_BASE & 0xFFFF),
        # LEA (0x00010000,A0,D0.L*4),A1 -- full format, long bd, base present
        0x43F0, 0x0D30, 0x0001, 0x0000,
        *_publish(h, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got_ea = h.read_result_long(0)
    got_val = h.read_result_long(4)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert got_ea == expect_ea, (
        f"LEA (0x00010000,A0,D0.L*4),A1 computed 0x{got_ea:08X}, expected "
        f"0x{expect_ea:08X}"
    )
    assert got_val == TABLE_VALUE, (
        f"Load through it returned 0x{got_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_index_written_by_previous_instruction(dut):
    """The index register is written by the instruction right before the LEA.

    This is the shape m68k-elf-gcc emits for a CRC table lookup:

        moveq #24,%d3
        lsrl  %d3,%d2                 ; index = value >> 24
        lea   @(2d4,%d2:l:4),%a2      ; 45F0 2DB0 0000 02D4
        movel %a2@,%d2

    The EA needs the value D2 has *after* the shift. Both spacings are measured
    so a difference between them is unambiguously a pipeline hazard rather than
    a decode error: the second case inserts a NOP between the write and the use.
    """
    h = CPUTestHarness(dut)

    program = [
        # Back to back: LSR.L D3,D2 then LEA (bd,D2.L*4),A1
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(INDEX << 24),                    # D2 = 0x03000000
        *moveq(24, 3),
        *lsr(LONG, 3, 2, ir=1),                    # D2 = 3, immediately before
        0x43F0, 0x2DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        *_publish(h, 0),
        # Same, with a NOP between the write and the use.
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(INDEX << 24),
        *moveq(24, 3),
        *lsr(LONG, 3, 2, ir=1),
        *nop(),
        0x43F0, 0x2DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        *_publish(h, 8, an_to_dn=5),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"

    tight_ea = h.read_result_long(0)
    tight_val = h.read_result_long(4)
    spaced_ea = h.read_result_long(8)
    spaced_val = h.read_result_long(12)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert spaced_ea == expect_ea, (
        f"With a NOP before it, LEA (0x{TABLE_BASE:08X},D2.L*4),A1 computed "
        f"0x{spaced_ea:08X}, expected 0x{expect_ea:08X}"
    )
    assert tight_ea == expect_ea, (
        f"LEA (0x{TABLE_BASE:08X},D2.L*4),A1 computed 0x{tight_ea:08X}, "
        f"expected 0x{expect_ea:08X}. The identical sequence with one NOP "
        f"inserted computed 0x{spaced_ea:08X}, so the full-format index "
        f"register is being read before the preceding write settles"
    )
    assert tight_val == TABLE_VALUE and spaced_val == TABLE_VALUE, (
        f"Loads through the EA returned tight=0x{tight_val:08X} "
        f"spaced=0x{spaced_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


def _shift_index_into_d2():
    """D2 = 3, produced by the instruction immediately before the EA."""
    return [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(INDEX << 24),                    # D2 = 0x03000000
        *moveq(24, 3),
        *lsr(LONG, 3, 2, ir=1),                    # D2 = 3, immediately before
    ]


@cocotb.test()
async def test_fullformat_word_displacement_index_hazard(dut):
    """A word base displacement takes a different exit than the long one.

    FETCH_EXWORD_1 leaves straight for FETCH_D_LO when BD SIZE = 10, skipping
    the FETCH_D_HI state the long-displacement cases pass through, so the
    interlock has to cover that arm as well.
    """
    h = CPUTestHarness(dut)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = 0x00010000
        *imm_long(CPUTestHarness.DATA_BASE),
        *_shift_index_into_d2(),
        # LEA (0x0340.W,A0,D2.L*4),A1 -- full format, word bd, base present
        0x43F0, 0x2D20, TABLE_BASE - CPUTestHarness.DATA_BASE,
        *_publish(h, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got_ea = h.read_result_long(0)
    got_val = h.read_result_long(4)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert got_ea == expect_ea, (
        f"LEA (0x0340.W,A0,D2.L*4),A1 with D2 written by the previous "
        f"instruction computed 0x{got_ea:08X}, expected 0x{expect_ea:08X}"
    )
    assert got_val == TABLE_VALUE, (
        f"Load through it returned 0x{got_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_memory_indirect_outer_displacement(dut):
    """Preindexed memory indirect with a null base displacement.

    A null bd with a non-null I/IS leaves FETCH_EXWORD_1 for FETCH_OD_LO or
    FETCH_MEMADR directly, without passing through either base-displacement
    state. Those exits are on the same interlocked path as the displacement
    ones, so they need coverage that they still resolve.

    The index is settled well before the EA here. The tight-spacing version of
    the same form is test_fullformat_memory_indirect_index_hazard below, which
    covers the separate FETCH_MEMADR intermediate-address defect.

        LEA ([A0,D2.L*4],4),A1   -> A1 = mem[A0 + D2*4] + 4
        LEA ([A0,D2.L*4]),A1     -> A1 = mem[A0 + D2*4]
    """
    h = CPUTestHarness(dut)
    ptr_table = TABLE_BASE
    target = CPUTestHarness.DATA_BASE + 0x800

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = ptr_table
        *imm_long(ptr_table),
        *moveq(INDEX, 2),                          # D2 = 3, settled
        *nop(), *nop(), *nop(), *nop(),
        # LEA ([A0,D2.L*4],4),A1 -- null bd, preindexed, word outer displacement
        0x43F0, 0x2D12, *imm_word(4),
        *_publish(h, 0),
        # LEA ([A0,D2.L*4]),A1 -- null bd, preindexed, null outer displacement
        0x43F0, 0x2D11,
        *_publish(h, 8, an_to_dn=5),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(ptr_table + INDEX * 4, target)
    h.mem.load_long(target + 4, TABLE_VALUE)
    h.mem.load_long(target, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    od_ea = h.read_result_long(0)
    od_val = h.read_result_long(4)
    null_ea = h.read_result_long(8)
    null_val = h.read_result_long(12)
    h.cleanup()

    assert od_ea == target + 4, (
        f"LEA ([A0,D2.L*4],4),A1 computed 0x{od_ea:08X}, expected "
        f"0x{target + 4:08X} (mem[0x{ptr_table + INDEX * 4:08X}] + 4)"
    )
    assert null_ea == target, (
        f"LEA ([A0,D2.L*4]),A1 computed 0x{null_ea:08X}, expected 0x{target:08X}"
    )
    assert od_val == TABLE_VALUE and null_val == TABLE_VALUE, (
        f"Loads through the EAs returned od=0x{od_val:08X} null=0x{null_val:08X}, "
        f"expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_memory_indirect_index_hazard(dut):
    """Memory indirect whose index register is written by the previous instruction.

    The intermediate read in FETCH_MEMADR uses ADR_EFF, which is registered:
    ADR_EFF_I only becomes the intermediate address one cycle after the state is
    entered, because FETCH_MEM_ADR (and the freshly latched F_E/I_S/I_IS) have to
    be asserted for a whole cycle before the adder result is clocked in. With the
    index settled the request is issued late enough that the bus samples the
    right value; with the index written by the immediately preceding instruction
    the fetch redirect that the write forces shortens the entry and the read goes
    out against the stale ADR_EFF.

    PRM 2.2 / Table 2-4: for a preindexed memory indirect address the
    intermediate address is bd + An + Xn.SIZE * SCALE and Xn must be the
    post-write value, exactly as for the non-indirect forms.

        LEA ([A0,D2.L*4],4),A1   -> A1 = mem[A0 + D2*4] + 4

    The settled-index copy of the same program runs second so that a difference
    between the two is unambiguously a pipeline hazard.
    """
    h = CPUTestHarness(dut)
    ptr_table = TABLE_BASE
    target = CPUTestHarness.DATA_BASE + 0x800

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = ptr_table
        *imm_long(ptr_table),
        # Tight: D2 written by the instruction immediately before the EA.
        *_shift_index_into_d2(),
        # LEA ([A0,D2.L*4],4),A1 -- null bd, preindexed, word outer displacement
        0x43F0, 0x2D12, *imm_word(4),
        *_publish(h, 0),
        # Control: the same EA with the index settled four NOPs earlier.
        *_shift_index_into_d2(),
        *nop(), *nop(), *nop(), *nop(),
        0x43F0, 0x2D12, *imm_word(4),
        *_publish(h, 8, an_to_dn=5),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(ptr_table + INDEX * 4, target)
    h.mem.load_long(target + 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"

    tight_ea = h.read_result_long(0)
    tight_val = h.read_result_long(4)
    settled_ea = h.read_result_long(8)
    settled_val = h.read_result_long(12)
    h.cleanup()

    assert settled_ea == target + 4, (
        f"Settled-index control is wrong too: LEA ([A0,D2.L*4],4),A1 computed "
        f"0x{settled_ea:08X}, expected 0x{target + 4:08X}"
    )
    assert tight_ea == target + 4, (
        f"LEA ([A0,D2.L*4],4),A1 with D2 written by the immediately preceding "
        f"instruction computed 0x{tight_ea:08X}, expected 0x{target + 4:08X} "
        f"(mem[0x{ptr_table + INDEX * 4:08X}] + 4). The identical sequence with "
        f"the index settled computed 0x{settled_ea:08X}, so the intermediate "
        f"read in FETCH_MEMADR is issued before the intermediate address is valid"
    )
    assert tight_val == TABLE_VALUE and settled_val == TABLE_VALUE, (
        f"Loads through the EA returned tight=0x{tight_val:08X} "
        f"settled=0x{settled_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_destination_index_hazard(dut):
    """The EA is the MOVE *destination*, reached in the second fetch phase.

    A MOVE destination re-enters FETCH_EXWORD_1 from INIT_EXEC_WB rather than
    from START_OP, so it is a separate entry into the interlocked state.

        MOVE.L D5,(0x00010340,D2.L*4)
    """
    h = CPUTestHarness(dut)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = 0 (base, suppressed)
        *imm_long(0x00000000),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(TABLE_VALUE),                    # D5 = payload
        *_shift_index_into_d2(),
        # MOVE.L D5,(bd.L,D2.L*4) -- full format destination, base suppressed
        *move(LONG, DN, 5, AN_IDX, 0),
        0x2DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        # Read the written longword back through a plain (An).
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(TABLE_BASE + INDEX * 4),
        *move(LONG, 2, 4, DN, 0),                  # MOVE.L (A4),D0
        *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got = h.read_result_long(0)
    h.cleanup()

    assert got == TABLE_VALUE, (
        f"MOVE.L D5,(0x{TABLE_BASE:08X},D2.L*4) with D2 written by the "
        f"previous instruction left 0x{got:08X} at "
        f"0x{TABLE_BASE + INDEX * 4:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_an_index_ignores_pending_dn_write(dut):
    """An An index must not wait on a data register write.

    D/A = 1 makes the index field name A2, not D2, so the pending write to D2
    is irrelevant here. This bounds the interlock from the other side: one that
    keyed on the register *number* rather than on D/A would stall for no reason,
    and one that waited on something the writeback pipeline never clears would
    hang instead of retiring.

        LEA (0x0340.L,A0,A2.L*4),A1
    """
    h = CPUTestHarness(dut)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = 0x00010000
        *imm_long(CPUTestHarness.DATA_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),       # A2 = 3 (the index)
        *imm_long(INDEX),
        *_shift_index_into_d2(),                   # D2 = 3, must be ignored
        # LEA (0x0340.L,A0,A2.L*4),A1 -- full format, long bd, An index
        0x43F0, 0xAD30, 0x0000, TABLE_BASE - CPUTestHarness.DATA_BASE,
        *_publish(h, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached (An-index full format hung)"
    got_ea = h.read_result_long(0)
    got_val = h.read_result_long(4)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert got_ea == expect_ea, (
        f"LEA (0x0340.L,A0,A2.L*4),A1 computed 0x{got_ea:08X}, expected "
        f"0x{expect_ea:08X} (the An index must come from A2, not D2)"
    )
    assert got_val == TABLE_VALUE, (
        f"Load through it returned 0x{got_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_brief_format_index_written_by_previous_instruction(dut):
    """Same hazard shape in the brief format, to bound the defect's scope."""
    h = CPUTestHarness(dut)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # A0 = TABLE_BASE
        *imm_long(TABLE_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(INDEX << 24),
        *moveq(24, 3),
        *lsr(LONG, 3, 2, ir=1),                    # D2 = 3, immediately before
        # LEA (0,A0,D2.L*4),A1 -- brief format (bit 8 = 0)
        0x43F0, 0x2C00,
        *_publish(h, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got_ea = h.read_result_long(0)
    got_val = h.read_result_long(4)
    h.cleanup()

    expect_ea = TABLE_BASE + INDEX * 4
    assert got_ea == expect_ea, (
        f"Brief-format LEA (0,A0,D2.L*4),A1 with D2 written by the previous "
        f"instruction computed 0x{got_ea:08X}, expected 0x{expect_ea:08X}"
    )
    assert got_val == TABLE_VALUE, (
        f"Load through it returned 0x{got_val:08X}, expected 0x{TABLE_VALUE:08X}"
    )


@cocotb.test()
async def test_fullformat_move_index_written_by_previous_instruction(dut):
    """Same hazard on a MOVE source EA, to show it is not LEA-specific."""
    h = CPUTestHarness(dut)

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(INDEX << 24),
        *moveq(24, 3),
        *lsr(LONG, 3, 2, ir=1),                    # D2 = 3, immediately before
        # MOVE.L (0x00010340,D2.L*4),D5
        0x2A30, 0x2DB0, (TABLE_BASE >> 16) & 0xFFFF, TABLE_BASE & 0xFFFF,
        *move_to_abs_long(LONG, DN, 5, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(TABLE_BASE + INDEX * 4, TABLE_VALUE)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"
    got = h.read_result_long(0)
    h.cleanup()
    assert got == TABLE_VALUE, (
        f"MOVE.L (0x{TABLE_BASE:08X},D2.L*4),D5 with D2 written by the "
        f"preceding LSR.L loaded 0x{got:08X}, expected 0x{TABLE_VALUE:08X}"
    )
