"""
Compliance tests for MOVEP, MOVES, ADDX, SUBX, CMPM and RTR.

None of these six had instruction-level coverage.  Expected values were
cross-checked against `qemu-system-m68k -cpu m68030` wherever the PRM text
leaves room for interpretation.

MOVEP (PRM): "Moves data between a data register and alternate bytes within the
address space starting at the location specified and incrementing by two.  The
high-order byte of the data register is transferred first, and the low-order
byte is transferred last."  Condition codes are not affected.

MOVES (PRM): moves an operand between a general register and the address space
named by SFC (reads) or DFC (writes).  "If the destination is an address
register, the source operand is sign-extended to 32 bits."  Condition codes are
not affected, and the instruction is privileged.

ADDX / SUBX (PRM): X is added/subtracted along with the source, and
"Z - Cleared if the result is nonzero; unchanged otherwise" - the sticky rule
that makes multiple-precision chains work.

CMPM (PRM): "Subtracts the source operand from the destination operand and sets
the condition codes according to the results; the destination location is not
changed.  The operands are always addressed with the postincrement addressing
mode."  X is not affected.

RTR (PRM): "(SP) -> CCR; SP + 2 -> SP; (SP) -> PC; SP + 4 -> SP ...  The
supervisor portion of the status register is unaffected."
"""

import cocotb
from cocotb.triggers import RisingEdge

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, AN_DISP, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq,
    movep_to_mem, movep_to_reg, moves, movec_to_cr, movec_from_cr,
    CR_SFC, CR_DFC,
    addx, subx, cmpm, rtr, jsr,
    move_from_ccr, move_to_ccr, move_from_sr, move_to_sr,
    imm_long, imm_word, disp16, abs_long,
)
from m68k_reference import extract_cc

MEM1 = CPUTestHarness.DATA_BASE
MEM2 = CPUTestHarness.DATA_BASE + 0x20

CCR_C = 0x01
CCR_V = 0x02
CCR_Z = 0x04
CCR_N = 0x08
CCR_X = 0x10


def ccr_flags(val):
    return extract_cc(val & 0xFF)


def assert_flags(name, actual_ccr, x=None, n=None, z=None, v=None, c=None):
    ax, an, az, av, ac = ccr_flags(actual_ccr)
    got = (f"(CCR=0x{actual_ccr & 0xFF:02X}: X={ax} N={an} Z={az} V={av} C={ac})")
    for label, exp, act in (("X", x, ax), ("N", n, an), ("Z", z, az),
                            ("V", v, av), ("C", c, ac)):
        if exp is not None:
            assert act == exp, f"{name}: {label} expected {exp}, got {act} {got}"


def _load_long(dn, value):
    return [*move(LONG, SPECIAL, IMMEDIATE, DN, dn), *imm_long(value)]


def _set_ccr(value):
    return [*move_to_ccr(SPECIAL, IMMEDIATE), *imm_word(value)]


def _capture(dn_list):
    words = [*nop(), *nop(), *move_from_ccr(DN, 6)]
    for dn in dn_list:
        words += [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    words += [*move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    return words


# =========================================================================
# MOVEP
# =========================================================================

@cocotb.test()
async def test_movep_long_to_memory(dut):
    """MOVEP.L D3,0(A1) writes the four register bytes to every other byte.

    D3 = 0x12345678 lands as 12 -- 34 -- 56 -- 78 --, high-order byte first.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x12345678),
        *movep_to_mem(LONG, 3, 1, 0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i in range(8):
        h.mem.write(MEM1 + i, 1, 0xEE)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(MEM1 + i, 1) for i in range(8)]
    assert got == [0x12, 0xEE, 0x34, 0xEE, 0x56, 0xEE, 0x78, 0xEE], (
        f"expected 12 EE 34 EE 56 EE 78 EE, got {[f'{b:02X}' for b in got]}")
    h.cleanup()


@cocotb.test()
async def test_movep_word_to_memory(dut):
    """MOVEP.W D3,0(A1) writes only the low two register bytes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x1234ABCD),
        *movep_to_mem(WORD, 3, 1, 0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i in range(8):
        h.mem.write(MEM1 + i, 1, 0xEE)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(MEM1 + i, 1) for i in range(4)]
    assert got == [0xAB, 0xEE, 0xCD, 0xEE], (
        f"expected AB EE CD EE, got {[f'{b:02X}' for b in got]}")
    h.cleanup()


@cocotb.test()
async def test_movep_long_to_memory_odd_address(dut):
    """MOVEP.L D3,1(A1) starting on an odd address still steps by two."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x12345678),
        *movep_to_mem(LONG, 3, 1, 1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i in range(8):
        h.mem.write(MEM1 + i, 1, 0xEE)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = [h.mem.read(MEM1 + i, 1) for i in range(8)]
    assert got == [0xEE, 0x12, 0xEE, 0x34, 0xEE, 0x56, 0xEE, 0x78], (
        f"expected EE 12 EE 34 EE 56 EE 78, got {[f'{b:02X}' for b in got]}")
    h.cleanup()


@cocotb.test()
async def test_movep_long_from_memory(dut):
    """MOVEP.L 0(A1),D3 gathers every other byte into the register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xFFFFFFFF),
        *movep_to_reg(LONG, 3, 1, 0),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88)):
        h.mem.write(MEM1 + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    assert d3 == 0x11335577, f"expected 0x11335577, got 0x{d3:08X}"
    h.cleanup()


@cocotb.test()
async def test_movep_word_from_memory_preserves_upper_word(dut):
    """MOVEP.W 0(A1),D3 loads only the low word; the upper word survives."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xFFFFFFFF),
        *movep_to_reg(WORD, 3, 1, 0),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x11, 0x22, 0x33, 0x44)):
        h.mem.write(MEM1 + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    assert d3 == 0xFFFF1133, f"expected 0xFFFF1133, got 0x{d3:08X}"
    h.cleanup()


@cocotb.test()
async def test_movep_long_from_memory_odd_address(dut):
    """MOVEP.L 1(A1),D3 gathers the odd-addressed bytes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xFFFFFFFF),
        *movep_to_reg(LONG, 3, 1, 1),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate((0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88)):
        h.mem.write(MEM1 + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    assert d3 == 0x22446688, f"expected 0x22446688, got 0x{d3:08X}"
    h.cleanup()


@cocotb.test()
async def test_movep_uses_byte_sized_accesses(dut):
    """MOVEP.L issues four separate byte cycles two bytes apart.

    The interleaved access pattern is the whole point of the instruction, so
    check the bus rather than only the result.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x12345678),
        *movep_to_mem(LONG, 3, 1, 0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    writes = []

    async def watch():
        prev = 1
        while True:
            await RisingEdge(dut.CLK)
            try:
                asn = int(dut.ASn.value)
                rwn = int(dut.RWn.value)
                adr = int(dut.ADR_OUT.value)
                size = int(dut.SIZE.value)
            except ValueError:
                continue
            if asn == 0 and prev == 1 and MEM1 <= adr < MEM1 + 0x20 and not rwn:
                writes.append((adr, size))
            prev = asn

    cocotb.start_soon(watch())
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    addrs = [a for a, _ in writes]
    assert addrs == [MEM1, MEM1 + 2, MEM1 + 4, MEM1 + 6], (
        f"expected byte writes at +0/+2/+4/+6, got {[hex(a) for a in addrs]}")
    for a, size in writes:
        assert size == 1, f"MOVEP must use byte transfers, saw SIZE={size} at {a:#x}"
    h.cleanup()


@cocotb.test()
async def test_movep_does_not_affect_condition_codes(dut):
    """MOVEP leaves the CCR untouched (PRM: "Condition Codes: Not affected")."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x12345678),
        *_set_ccr(CCR_X | CCR_N | CCR_Z | CCR_V | CCR_C),
        *movep_to_mem(LONG, 3, 1, 0),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("MOVEP.L to memory", h.read_result_long(0),
                 x=1, n=1, z=1, v=1, c=1)
    h.cleanup()


# =========================================================================
# MOVES
# =========================================================================

SFC_DFC_VALUE = 5  # supervisor data space


def _set_sfc_dfc(value):
    return [*_load_long(0, value),
            *movec_to_cr(0, CR_SFC),
            *movec_to_cr(0, CR_DFC)]


@cocotb.test()
async def test_moves_long_register_to_memory(dut):
    """MOVES.L D3,(A1) writes the register through DFC."""
    h = CPUTestHarness(dut)
    program = [
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xCAFEBABE),
        *moves(LONG, 3, 0, 1, AN_IND, 1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000000)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    got = h.mem.read(MEM1, 4)
    assert got == 0xCAFEBABE, f"expected 0xCAFEBABE in memory, got 0x{got:08X}"
    h.cleanup()


@cocotb.test()
async def test_moves_long_memory_to_register(dut):
    """MOVES.L (A1),D3 reads through SFC into the register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x00000000),
        *moves(LONG, 3, 0, 0, AN_IND, 1),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x12345678)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    assert d3 == 0x12345678, f"expected 0x12345678, got 0x{d3:08X}"
    h.cleanup()


@cocotb.test()
async def test_moves_byte_and_word_sizes(dut):
    """MOVES.B and MOVES.W replace only the low part of a data register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xFFFFFFFF),
        *moves(BYTE, 3, 0, 0, AN_IND, 1),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *_load_long(4, 0xFFFFFFFF),
        *moves(WORD, 4, 0, 0, AN_IND, 1),
        *nop(), *nop(),
        *move(LONG, DN, 4, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x12345678)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    d4 = h.read_result_long(4)
    assert d3 == 0xFFFFFF12, f"MOVES.B expected 0xFFFFFF12, got 0x{d3:08X}"
    assert d4 == 0xFFFF1234, f"MOVES.W expected 0xFFFF1234, got 0x{d4:08X}"
    h.cleanup()


@cocotb.test()
async def test_moves_word_to_address_register_sign_extends(dut):
    """MOVES.W (A1),A2 sign-extends the word into the full address register.

    PRM: "If the destination is an address register, the source operand is
    sign-extended to 32 bits and then loaded into that address register."
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(0x00000000),
        *moves(WORD, 2, 1, 0, AN_IND, 1),
        *nop(), *nop(),
        *move(LONG, AN, 2, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 2, 0x8001)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    a2 = h.read_result_long(0)
    assert a2 == 0xFFFF8001, f"expected 0xFFFF8001, got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_moves_postincrement_advances_address_register(dut):
    """MOVES.L (A1)+,D3 advances A1 by four."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x00000000),
        *moves(LONG, 3, 0, 0, AN_POSTINC, 1),
        *nop(), *nop(),
        *move(LONG, DN, 3, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0xAABBCCDD)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d3 = h.read_result_long(0)
    a1 = h.read_result_long(4)
    assert d3 == 0xAABBCCDD, f"expected 0xAABBCCDD, got 0x{d3:08X}"
    assert a1 == MEM1 + 4, f"A1 expected 0x{MEM1 + 4:08X}, got 0x{a1:08X}"
    h.cleanup()


@cocotb.test()
async def test_moves_does_not_affect_condition_codes(dut):
    """MOVES leaves the CCR untouched (PRM: "Condition Codes: Not affected")."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_sfc_dfc(SFC_DFC_VALUE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0x00000000),
        *_set_ccr(CCR_X | CCR_N | CCR_Z | CCR_V | CCR_C),
        *moves(LONG, 3, 0, 0, AN_IND, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x12345678)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("MOVES.L (A1),D3", h.read_result_long(0),
                 x=1, n=1, z=1, v=1, c=1)
    h.cleanup()


@cocotb.test()
async def test_moves_drives_the_dfc_function_code(dut):
    """A MOVES write must drive FC_OUT from DFC, not the normal data FC.

    DFC is set to 3, which is otherwise unused, so any cycle to the operand
    address carrying FC=3 can only come from the MOVES.
    """
    h = CPUTestHarness(dut)
    program = [
        *_load_long(0, 3),
        *movec_to_cr(0, CR_DFC),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(3, 0xCAFEBABE),
        *moves(LONG, 3, 0, 1, AN_IND, 1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    seen = []

    async def watch():
        prev = 1
        while True:
            await RisingEdge(dut.CLK)
            try:
                asn = int(dut.ASn.value)
                rwn = int(dut.RWn.value)
                adr = int(dut.ADR_OUT.value)
                fc = int(dut.FC_OUT.value)
            except ValueError:
                continue
            if asn == 0 and prev == 1 and adr == MEM1 and not rwn:
                seen.append(fc)
            prev = asn

    cocotb.start_soon(watch())
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert seen, "no write cycle to the MOVES operand address was observed"
    assert 3 in seen, (
        f"MOVES write should carry FC=3 from DFC, saw FC values {seen}")
    h.cleanup()


@cocotb.test()
async def test_moves_privilege_violation_in_user_mode(dut):
    """MOVES from user mode takes the privilege-violation vector (8).

    PRM: "If Supervisor State Then Rn -> Destination [DFC] or Source [SFC] ->
    Rn Else TRAP".
    """
    h = CPUTestHarness(dut)
    handler_addr = 0x000700
    handler = [
        *moveq(0x08, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    program = [
        *moveq(0x22, 1),                      # marker if no trap happens
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5), *imm_word(0x0000),
        *move_to_sr(DN, 5),                   # drop to user mode
        *moves(LONG, 3, 0, 0, AN_IND, 1),     # privileged
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_long(8 * 4, handler_addr)
    h.mem.load_words(handler_addr, handler)
    found = await h.run_until_sentinel(max_cycles=9000)
    assert found, "MOVES privilege test did not complete"
    marker = h.read_result_long(0)
    assert marker == 0x08, (
        f"expected the privilege-violation handler marker 0x08, got 0x{marker:08X}")
    h.cleanup()


# =========================================================================
# ADDX / SUBX
# =========================================================================

@cocotb.test()
async def test_addx_long_register_with_x_set(dut):
    """ADDX.L D1,D2: 2 + 1 + X=1 = 4."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0x00000002),
        *_set_ccr(CCR_X),
        *addx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 4, (
        f"expected 4, got 0x{h.read_result_long(0):08X}")
    assert_flags("ADDX.L 2+1+X", h.read_result_long(4), x=0, n=0, z=0, v=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_addx_long_overflow(dut):
    """ADDX.L 0x7FFFFFFF + 1 sets V and N."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0x7FFFFFFF),
        *_set_ccr(0x00),
        *addx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x80000000, (
        f"expected 0x80000000, got 0x{h.read_result_long(0):08X}")
    assert_flags("ADDX.L overflow", h.read_result_long(4),
                 x=0, n=1, z=0, v=1, c=0)
    h.cleanup()


@cocotb.test()
async def test_addx_long_sticky_z_on_zero_result(dut):
    """ADDX.L 0xFFFFFFFF + 1 wraps to zero: C=X=1 and a preset Z stays set."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0xFFFFFFFF),
        *_set_ccr(CCR_Z),
        *addx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0, (
        f"expected 0, got 0x{h.read_result_long(0):08X}")
    assert_flags("ADDX.L -1+1 sticky Z", h.read_result_long(4),
                 x=1, n=0, z=1, v=0, c=1)
    h.cleanup()


@cocotb.test()
async def test_addx_byte_and_word_sizes(dut):
    """ADDX.B and ADDX.W touch only the low byte / word of the destination."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0xAABBCCFF),
        *_set_ccr(0x00),
        *addx(BYTE, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 2, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *_load_long(3, 0x00000001), *_load_long(4, 0xAABBFFFF),
        *_set_ccr(0x00),
        *addx(WORD, 4, 3),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 4, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2 = h.read_result_long(0)
    ccr_b = h.read_result_long(4)
    d4 = h.read_result_long(8)
    ccr_w = h.read_result_long(12)
    assert d2 == 0xAABBCC00, f"ADDX.B expected 0xAABBCC00, got 0x{d2:08X}"
    assert_flags("ADDX.B 0xFF+1", ccr_b, x=1, n=0, z=0, v=0, c=1)
    assert d4 == 0xAABB0000, f"ADDX.W expected 0xAABB0000, got 0x{d4:08X}"
    assert_flags("ADDX.W 0xFFFF+1", ccr_w, x=1, n=0, z=0, v=0, c=1)
    h.cleanup()


@cocotb.test()
async def test_addx_memory_multiprecision_chain(dut):
    """ADDX.L -(A1),-(A2) twice adds a 64-bit value with a carry between halves.

    0x00000001_FFFFFFFF + 0x00000002_00000001 = 0x00000004_00000000.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1 + 8),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2 + 8),
        *_set_ccr(CCR_Z),
        *addx(LONG, 2, 1, rm=1),
        *addx(LONG, 2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000002)
    h.mem.write(MEM1 + 4, 4, 0x00000001)
    h.mem.write(MEM2, 4, 0x00000001)
    h.mem.write(MEM2 + 4, 4, 0xFFFFFFFF)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    hi = h.mem.read(MEM2, 4)
    lo = h.mem.read(MEM2 + 4, 4)
    assert (hi, lo) == (0x00000004, 0x00000000), (
        f"expected 0x00000004_00000000, got 0x{hi:08X}_{lo:08X}")
    assert_flags("ADDX chain", h.read_result_long(0), x=0, n=0, z=0, v=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_addx_memory_chain_zero_result_keeps_z(dut):
    """A 64-bit ADDX chain whose whole result is zero leaves a preset Z set.

    0xFFFFFFFF_FFFFFFFF + 0x00000000_00000001 = 0x00000000_00000000 with a
    carry out; the sticky Z rule is the only way software can see that both
    halves came out zero.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1 + 8),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2 + 8),
        *_set_ccr(CCR_Z),
        *addx(LONG, 2, 1, rm=1),
        *addx(LONG, 2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000000)
    h.mem.write(MEM1 + 4, 4, 0x00000001)
    h.mem.write(MEM2, 4, 0xFFFFFFFF)
    h.mem.write(MEM2 + 4, 4, 0xFFFFFFFF)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    hi = h.mem.read(MEM2, 4)
    lo = h.mem.read(MEM2 + 4, 4)
    assert (hi, lo) == (0, 0), f"expected zero, got 0x{hi:08X}_{lo:08X}"
    assert_flags("ADDX chain zero", h.read_result_long(0), x=1, z=1, c=1)
    h.cleanup()


@cocotb.test()
async def test_subx_long_register_with_x_set(dut):
    """SUBX.L D1,D2: 3 - 1 - X=1 = 1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0x00000003),
        *_set_ccr(CCR_X),
        *subx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 1, (
        f"expected 1, got 0x{h.read_result_long(0):08X}")
    assert_flags("SUBX.L 3-1-X", h.read_result_long(4), x=0, n=0, z=0, v=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_subx_long_borrow(dut):
    """SUBX.L 0 - 1 = 0xFFFFFFFF with C=X=1 and N=1."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000001), *_load_long(2, 0x00000000),
        *_set_ccr(0x00),
        *subx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xFFFFFFFF, (
        f"expected 0xFFFFFFFF, got 0x{h.read_result_long(0):08X}")
    assert_flags("SUBX.L 0-1", h.read_result_long(4), x=1, n=1, z=0, v=0, c=1)
    h.cleanup()


@cocotb.test()
async def test_subx_long_sticky_z_on_equal_operands(dut):
    """SUBX.L of equal operands with X=0 gives zero and leaves a preset Z set."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_load_long(1, 0x00000003), *_load_long(2, 0x00000003),
        *_set_ccr(CCR_Z),
        *subx(LONG, 2, 1),
        *_capture([2]),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0, (
        f"expected 0, got 0x{h.read_result_long(0):08X}")
    assert_flags("SUBX.L 3-3", h.read_result_long(4), x=0, n=0, z=1, v=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_subx_memory_multiprecision_chain(dut):
    """SUBX.L -(A1),-(A2) twice subtracts a 64-bit value with a borrow.

    0x00000002_00000000 - 0x00000001_00000001 = 0x00000000_FFFFFFFF.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1 + 8),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2 + 8),
        *_set_ccr(CCR_Z),
        *subx(LONG, 2, 1, rm=1),
        *subx(LONG, 2, 1, rm=1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000001)          # source high
    h.mem.write(MEM1 + 4, 4, 0x00000001)      # source low
    h.mem.write(MEM2, 4, 0x00000002)          # dest high
    h.mem.write(MEM2 + 4, 4, 0x00000000)      # dest low
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    hi = h.mem.read(MEM2, 4)
    lo = h.mem.read(MEM2 + 4, 4)
    assert (hi, lo) == (0x00000000, 0xFFFFFFFF), (
        f"expected 0x00000000_FFFFFFFF, got 0x{hi:08X}_{lo:08X}")
    assert_flags("SUBX chain", h.read_result_long(0), x=0, z=0, c=0)
    h.cleanup()


# =========================================================================
# CMPM
# =========================================================================

@cocotb.test()
async def test_cmpm_long_equal(dut):
    """CMPM.L (A1)+,(A2)+ on equal longs sets Z and advances both registers."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *_set_ccr(CCR_X),
        *cmpm(LONG, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x11112222)
    h.mem.write(MEM2, 4, 0x11112222)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    # X is not affected by CMPM, so the preset X must survive.
    assert_flags("CMPM.L equal", h.read_result_long(0),
                 x=1, n=0, z=1, v=0, c=0)
    a1 = h.read_result_long(4)
    a2 = h.read_result_long(8)
    assert a1 == MEM1 + 4, f"A1 expected 0x{MEM1 + 4:08X}, got 0x{a1:08X}"
    assert a2 == MEM2 + 4, f"A2 expected 0x{MEM2 + 4:08X}, got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_cmpm_word_equal_advances_by_two(dut):
    """CMPM.W advances both address registers by two."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *_set_ccr(0x00),
        *cmpm(WORD, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x1111FFFF)
    h.mem.write(MEM2, 4, 0x11110000)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("CMPM.W equal", h.read_result_long(0), n=0, z=1, v=0, c=0)
    assert h.read_result_long(4) == MEM1 + 2, "A1 should advance by 2"
    assert h.read_result_long(8) == MEM2 + 2, "A2 should advance by 2"
    h.cleanup()


@cocotb.test()
async def test_cmpm_byte_equal_advances_by_one(dut):
    """CMPM.B advances both address registers by one."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *_set_ccr(0x00),
        *cmpm(BYTE, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 2, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x11FFFFFF)
    h.mem.write(MEM2, 4, 0x11000000)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("CMPM.B equal", h.read_result_long(0), n=0, z=1, v=0, c=0)
    assert h.read_result_long(4) == MEM1 + 1, "A1 should advance by 1"
    assert h.read_result_long(8) == MEM2 + 1, "A2 should advance by 1"
    h.cleanup()


@cocotb.test()
async def test_cmpm_long_destination_greater(dut):
    """CMPM.L computes destination - source: 3 - 1 clears Z, N and C."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *_set_ccr(0x00),
        *cmpm(LONG, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000001)          # source, (A1)+
    h.mem.write(MEM2, 4, 0x00000003)          # destination, (A2)+
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("CMPM.L 3-1", h.read_result_long(0), n=0, z=0, v=0, c=0)
    h.cleanup()


@cocotb.test()
async def test_cmpm_long_destination_smaller_borrows(dut):
    """CMPM.L with the destination smaller sets C and N."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *_set_ccr(0x00),
        *cmpm(LONG, 2, 1),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000003)          # source
    h.mem.write(MEM2, 4, 0x00000001)          # destination
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert_flags("CMPM.L 1-3", h.read_result_long(0), n=1, z=0, v=0, c=1)
    h.cleanup()


@cocotb.test()
async def test_cmpm_does_not_modify_memory(dut):
    """CMPM leaves both operands in memory unchanged."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *cmpm(LONG, 2, 1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0xDEADBEEF)
    h.mem.write(MEM2, 4, 0xFEEDFACE)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.mem.read(MEM1, 4) == 0xDEADBEEF, "CMPM modified the source"
    assert h.mem.read(MEM2, 4) == 0xFEEDFACE, "CMPM modified the destination"
    h.cleanup()


@cocotb.test()
async def test_cmpm_string_compare_loop(dut):
    """A CMPM.B walk over two buffers stops at the first differing byte."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2),
        *cmpm(BYTE, 2, 1),                    # 'a' vs 'a' -> equal
        *nop(), *nop(),
        *move_from_ccr(DN, 5),
        *cmpm(BYTE, 2, 1),                    # 'b' vs 'b' -> equal
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *cmpm(BYTE, 2, 1),                    # 'c' vs 'x' -> differ
        *nop(), *nop(),
        *move_from_ccr(DN, 7),
        *move(LONG, DN, 5, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 7, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for i, b in enumerate(b"abc"):
        h.mem.write(MEM1 + i, 1, b)
    for i, b in enumerate(b"abx"):
        h.mem.write(MEM2 + i, 1, b)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    _, _, z0, _, _ = ccr_flags(h.read_result_long(0))
    _, _, z1, _, _ = ccr_flags(h.read_result_long(4))
    _, _, z2, _, _ = ccr_flags(h.read_result_long(8))
    a1 = h.read_result_long(12)
    assert (z0, z1, z2) == (1, 1, 0), (
        f"expected Z = 1,1,0 across the three compares, got {(z0, z1, z2)}")
    assert a1 == MEM1 + 3, f"A1 expected 0x{MEM1 + 3:08X}, got 0x{a1:08X}"
    h.cleanup()


# =========================================================================
# RTR
# =========================================================================

RTR_TARGET = 0x000800


@cocotb.test()
async def test_rtr_restores_ccr_and_pc(dut):
    """RTR pops a CCR word and a PC, and leaves the upper SR byte alone.

    PRM RTR: "(SP) -> CCR; SP + 2 -> SP; (SP) -> PC; SP + 4 -> SP ...  The
    supervisor portion of the status register is unaffected."  The stacked word
    here is 0xFF1F, so a core that loaded the whole word into SR would drop out
    of supervisor mode and change the interrupt mask.
    """
    h = CPUTestHarness(dut)
    target = [
        *move_from_sr(DN, 1),                  # capture the full SR
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, AN_IND, 0),         # capture SP
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, AN_PREDEC, 7), *imm_long(RTR_TARGET),
        *move(WORD, SPECIAL, IMMEDIATE, AN_PREDEC, 7), *imm_word(0xFF1F),
        *rtr(),
        # Fallthrough marker: reached only if RTR did not branch.
        *moveq(0x55, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_words(RTR_TARGET, target)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sr = h.read_result_long(0) & 0xFFFF
    sp = h.read_result_long(4)
    assert (sr & 0xFF) == 0x1F, (
        f"CCR should be 0x1F from the stacked word, got SR=0x{sr:04X}")
    assert (sr >> 8) == 0x27, (
        f"the supervisor byte of SR must be unchanged (0x27), got 0x{sr >> 8:02X}")
    assert sp == h.SSP_INIT, (
        f"SP should be back at 0x{h.SSP_INIT:08X} after RTR, got 0x{sp:08X}")
    h.cleanup()


@cocotb.test()
async def test_rtr_clears_flags_from_stacked_word(dut):
    """RTR with a zero CCR word clears all five flags."""
    h = CPUTestHarness(dut)
    target = [
        *move_from_sr(DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_ccr(CCR_X | CCR_N | CCR_Z | CCR_V | CCR_C),
        *move(LONG, SPECIAL, IMMEDIATE, AN_PREDEC, 7), *imm_long(RTR_TARGET),
        *move(WORD, SPECIAL, IMMEDIATE, AN_PREDEC, 7), *imm_word(0x0000),
        *rtr(),
        *moveq(0x55, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_words(RTR_TARGET, target)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sr = h.read_result_long(0) & 0xFFFF
    assert (sr & 0x1F) == 0x00, (
        f"RTR should have cleared the CCR, got SR=0x{sr:04X}")
    assert (sr >> 8) == 0x27, (
        f"the supervisor byte must be unchanged, got 0x{sr >> 8:02X}")
    h.cleanup()


@cocotb.test()
async def test_rtr_returns_from_a_subroutine(dut):
    """RTR used as a flag-restoring RTS: JSR, push CCR inside, RTR back.

    The callee saves the caller's CCR with MOVE CCR,-(SP) after the JSR return
    address is already on the stack, which is the layout RTR expects.
    """
    h = CPUTestHarness(dut)
    sub = 0x000900
    subroutine = [
        *move_from_ccr(AN_PREDEC, 7),          # MOVE CCR,-(SP)
        *_set_ccr(CCR_N | CCR_C),              # clobber the flags inside
        *rtr(),
    ]
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *_set_ccr(CCR_Z),                      # caller's flags: Z only
        *jsr(SPECIAL, ABS_L), *abs_long(sub),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_words(sub, subroutine)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    sp = h.read_result_long(4)
    assert_flags("CCR restored by RTR", ccr, x=0, n=0, z=1, v=0, c=0)
    assert sp == h.SSP_INIT, (
        f"SP should be back at 0x{h.SSP_INIT:08X}, got 0x{sp:08X}")
    h.cleanup()


@cocotb.test()
async def test_movec_to_dfc_leaves_sfc_alone(dut):
    """A MOVEC writing one control register must not write another.

    SFC_WR decoded its register-select field from the live BIW_1 while all six
    sibling control-register writes use BIW_1_WB, the writeback-stage copy that
    MOVEC_WR_OP is itself qualified against (OP_WB_I, BIW_0_WB, EXEC_WB_STATE ==
    WRITEBACK).  control.sv's own history records the intended form: "Fixed a
    MOVEC writeback issue (use BIW_WB... instead of BIW_...)" -- SFC_WR was
    missed by that fix.  12'h000 is both the SFC selector and a common idle
    value of BIW_1, so a MOVEC writing any other control register could write
    SFC as well.

    The MOVES tests could not see this: _set_sfc_dfc writes SFC and DFC the same
    value, so a cross-write is invisible.  This test gives them different values
    and reads both back.
    """
    h = CPUTestHarness(dut)
    program = [
        *_load_long(0, 5),
        *movec_to_cr(0, CR_SFC),                        # SFC = 5
        *_load_long(1, 1),
        *movec_to_cr(1, CR_DFC),                        # DFC = 1, SFC untouched
        *movec_from_cr(CR_SFC, 2),                      # D2 = SFC
        *movec_from_cr(CR_DFC, 3),                      # D3 = DFC
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE),
        *move_to_abs_long(LONG, DN, 3, h.RESULT_BASE + 4),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sfc = h.read_result_long(0) & 0x7
    dfc = h.read_result_long(4) & 0x7
    assert sfc == 5, (
        f"SFC = {sfc}, expected 5: the MOVEC to DFC overwrote SFC, which means "
        f"SFC_WR is decoding its selector from the live BIW_1 rather than BIW_1_WB"
    )
    assert dfc == 1, f"DFC = {dfc}, expected 1"
    h.cleanup()
