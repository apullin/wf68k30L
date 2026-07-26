"""
CAS and CAS2 compliance tests for WF68K30L.

Neither instruction had any instruction-level coverage.  The audit notes they
were non-functional until a control-sequencing fix landed, so these tests check
the success and the failure path of each, the condition codes after both, that
a failing compare writes the memory operand into the compare register, and that
the accesses run as locked read-modify-write cycles with RMCn asserted.

PRM CAS/CAS2:
  "CAS compares the effective address operand to the compare operand (Dc).  If
  the operands are equal, the instruction writes the update operand (Du) to the
  effective address operand; otherwise, the instruction writes the effective
  address operand to the compare operand (Dc)."

  "CAS2 compares memory operand 1 (Rn1) to compare operand 1 (Dc1).  If the
  operands are equal, the instruction compares memory operand 2 (Rn2) to
  compare operand 2 (Dc2).  If these operands are also equal, the instruction
  writes the update operands (Du1 and Du2) to the memory operands (Rn1 and
  Rn2).  If either comparison fails, the instruction writes the memory operands
  (Rn1 and Rn2) to the compare operands (Dc1 and Dc2)."

  "Both operations access memory using locked or read-modify-write transfer
  sequences, providing a means of synchronizing several processors."

Condition codes are the CMP equations over (memory operand - compare operand);
X is not affected.  CAS2 cannot use byte operands.

Expected values were cross-checked against `qemu-system-m68k -cpu m68030`.
"""

import cocotb
from cocotb.triggers import RisingEdge

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    move, movea, nop, addq, cas, cas2,
    move_from_ccr, imm_long,
)
from m68k_reference import extract_cc

# Two independent memory operands inside the harness data region.
MEM1 = CPUTestHarness.DATA_BASE
MEM2 = CPUTestHarness.DATA_BASE + 0x10


def ccr_flags(val):
    return extract_cc(val & 0xFF)


def assert_cmp_flags(name, actual_ccr, n, z, v, c):
    ax, an, az, av, ac = ccr_flags(actual_ccr)
    got = f"(CCR=0x{actual_ccr & 0xFF:02X}: N={an} Z={az} V={av} C={ac})"
    assert (an, az, av, ac) == (n, z, v, c), (
        f"{name}: expected N={n} Z={z} V={v} C={c}, got {got}")


def _load_long(dn, value):
    return [*move(LONG, SPECIAL, IMMEDIATE, DN, dn), *imm_long(value)]


def _capture(dn_list):
    """CCR into D6, then the named registers and the CCR through the A0 cursor."""
    words = [*nop(), *nop(), *move_from_ccr(DN, 6)]
    for dn in dn_list:
        words += [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    words += [*move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]
    return words


async def _rmc_monitor(dut, rmc):
    """Record whether RMCn was asserted during data reads and data writes."""
    prev_as = 1
    while True:
        await RisingEdge(dut.CLK)
        try:
            asn = int(dut.ASn.value)
            rwn = int(dut.RWn.value)
            adr = int(dut.ADR_OUT.value)
            rmcn = int(dut.RMCn.value)
        except ValueError:
            continue
        if asn == 0 and prev_as == 1 and MEM1 <= adr < MEM1 + 0x100:
            if rwn:
                rmc["read"] = rmc["read"] or (rmcn == 0)
            else:
                rmc["write"] = rmc["write"] or (rmcn == 0)
        prev_as = asn


async def _run(dut, setup, instr, capture_regs, mem_init, watch_rmc=False):
    """Assemble, run and read back a CAS/CAS2 program.

    Returns (captured register values, ccr, {addr: value}, rmc dict).
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *setup,
        *instr,
        *_capture(capture_regs),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    for addr, size, val in mem_init:
        h.mem.write(addr, size, val)

    rmc = {"read": False, "write": False}
    if watch_rmc:
        cocotb.start_soon(_rmc_monitor(dut, rmc))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    values = [h.read_result_long(4 * i) for i in range(len(capture_regs))]
    ccr = h.read_result_long(4 * len(capture_regs))
    mem = {addr: h.mem.read(addr, 4) for addr, _, _ in mem_init}
    h.cleanup()
    return values, ccr, mem, rmc


# =========================================================================
# CAS
# =========================================================================

@cocotb.test()
async def test_cas_long_success_writes_update(dut):
    """CAS.L D2,D3,(A1) with a matching compare writes Du to memory, Z=1."""
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0x11112222) + _load_long(3, 0xAAAABBBB),
        cas(LONG, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0xAAAABBBB, (
        f"memory should hold the update operand, got 0x{mem[MEM1]:08X}")
    assert vals[0] == 0x11112222, (
        f"a successful CAS must leave Dc alone, got 0x{vals[0]:08X}")
    assert_cmp_flags("CAS.L success", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas_long_failure_loads_compare_register(dut):
    """CAS.L D2,D3,(A1) with a mismatching compare writes memory into Dc.

    PRM: "otherwise, the instruction writes the effective address operand to
    the compare operand (Dc)."  Memory must be left alone.
    """
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0xDEADBEEF) + _load_long(3, 0xAAAABBBB),
        cas(LONG, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0x11112222, (
        f"a failing CAS must not write memory, got 0x{mem[MEM1]:08X}")
    assert vals[0] == 0x11112222, (
        f"Dc should hold the memory operand, got 0x{vals[0]:08X}")
    assert vals[1] == 0xAAAABBBB, f"Du must not change, got 0x{vals[1]:08X}"
    # 0x11112222 - 0xDEADBEEF = 0x32636333 with a borrow: N=0 Z=0 V=0 C=1.
    assert_cmp_flags("CAS.L failure", ccr, n=0, z=0, v=0, c=1)


@cocotb.test()
async def test_cas_word_success(dut):
    """CAS.W D2,D3,(A1) on the word at MEM1+2 replaces just that word."""
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1 + 2)] +
        _load_long(2, 0x00002222) + _load_long(3, 0x00003333),
        cas(WORD, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0x11113333, (
        f"expected 0x11113333, got 0x{mem[MEM1]:08X}")
    assert_cmp_flags("CAS.W success", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas_word_failure(dut):
    """CAS.W mismatch: the memory word 0x1111 replaces the low word of Dc only."""
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0x00002222) + _load_long(3, 0x00003333),
        cas(WORD, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0x11112222, (
        f"a failing CAS must not write memory, got 0x{mem[MEM1]:08X}")
    assert (vals[0] & 0xFFFF) == 0x1111, (
        f"Dc low word should hold the memory word, got 0x{vals[0]:08X}")
    # 0x1111 - 0x2222 = 0xEEEF: N=1 Z=0 V=0 C=1.
    assert_cmp_flags("CAS.W failure", ccr, n=1, z=0, v=0, c=1)


@cocotb.test()
async def test_cas_byte_success(dut):
    """CAS.B D2,D3,(A1) replaces one byte on a match."""
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0x00000011) + _load_long(3, 0x00000099),
        cas(BYTE, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0x99112222, (
        f"expected 0x99112222, got 0x{mem[MEM1]:08X}")
    assert_cmp_flags("CAS.B success", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas_byte_failure(dut):
    """CAS.B mismatch loads the memory byte into the low byte of Dc."""
    vals, ccr, mem, _ = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0x00000077) + _load_long(3, 0x00000099),
        cas(BYTE, 2, 3, AN_IND, 1), [2, 3],
        [(MEM1, 4, 0x11112222)])
    assert mem[MEM1] == 0x11112222, (
        f"a failing CAS must not write memory, got 0x{mem[MEM1]:08X}")
    assert (vals[0] & 0xFF) == 0x11, (
        f"Dc low byte should hold the memory byte, got 0x{vals[0]:08X}")
    # 0x11 - 0x77 = 0x9A: N=1 Z=0 V=0 C=1.
    assert_cmp_flags("CAS.B failure", ccr, n=1, z=0, v=0, c=1)


@cocotb.test()
async def test_cas_success_is_read_modify_write(dut):
    """A successful CAS.L asserts RMCn across both the read and the write.

    PRM: "Both operations access memory using locked or read-modify-write
    transfer sequences, providing a means of synchronizing several processors."
    """
    _, _, _, rmc = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0x11112222) + _load_long(3, 0xAAAABBBB),
        cas(LONG, 2, 3, AN_IND, 1), [2],
        [(MEM1, 4, 0x11112222)], watch_rmc=True)
    assert rmc["read"], "RMCn was not asserted for the CAS read cycle"
    assert rmc["write"], "RMCn was not asserted for the CAS write cycle"


@cocotb.test()
async def test_cas_failure_is_read_modify_write(dut):
    """A failing CAS.L still runs its read as a locked cycle."""
    _, _, _, rmc = await _run(
        dut,
        [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)] +
        _load_long(2, 0xDEADBEEF) + _load_long(3, 0xAAAABBBB),
        cas(LONG, 2, 3, AN_IND, 1), [2],
        [(MEM1, 4, 0x11112222)], watch_rmc=True)
    assert rmc["read"], "RMCn was not asserted for the failing CAS read cycle"


@cocotb.test()
async def test_cas_spin_lock_sequence(dut):
    """Two CAS.L in a row: the first claims the lock, the second must fail.

    This is the canonical use of CAS and it also checks that the core can issue
    two locked sequences back to back.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1),
        *_load_long(2, 0x00000000),           # Dc: expect "unlocked"
        *_load_long(3, 0x0000FFFF),           # Du: "claimed by me"
        *cas(LONG, 2, 3, AN_IND, 1),          # first attempt: succeeds
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *_load_long(4, 0x00000000),           # second claimant, same expectation
        *_load_long(5, 0x0000AAAA),
        *cas(LONG, 4, 5, AN_IND, 1),          # second attempt: must fail
        *nop(), *nop(),
        *move_from_ccr(DN, 7),
        *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 7, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 4, AN_IND, 0), *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.write(MEM1, 4, 0x00000000)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr1 = h.read_result_long(0)
    ccr2 = h.read_result_long(4)
    dc2 = h.read_result_long(8)
    assert_cmp_flags("first CAS (claims the lock)", ccr1, n=0, z=1, v=0, c=0)
    _, _, z2, _, _ = ccr_flags(ccr2)
    assert z2 == 0, f"the second CAS must fail (Z=0), CCR=0x{ccr2 & 0xFF:02X}"
    assert dc2 == 0x0000FFFF, (
        f"the failing CAS should load the current lock value into Dc, "
        f"got 0x{dc2:08X}")
    lock = h.mem.read(MEM1, 4)
    assert lock == 0x0000FFFF, (
        f"the lock word should still be the first claimant's, got 0x{lock:08X}")
    h.cleanup()


# =========================================================================
# CAS2
# =========================================================================

def _cas2_program(h, dc1, dc2, du1, du2, size=LONG, da=1):
    """CAS2 <size> D3:D4,D5:D6,(Rn1):(Rn2) with A1/A2 (da=1) or D1/D2 (da=0)."""
    setup = []
    if da:
        setup += [*movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(MEM1)]
        setup += [*movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(MEM2)]
    else:
        setup += _load_long(1, MEM1) + _load_long(2, MEM2)
    setup += _load_long(3, dc1) + _load_long(4, dc2)
    setup += _load_long(5, du1) + _load_long(6, du2)
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *setup,
        *cas2(size, 3, 5, 1, da, 4, 6, 2, da),
        *_capture([3, 4]),
        *h.sentinel_program(),
    ]


async def _run_cas2(dut, dc1, dc2, du1, du2, m1, m2, size=LONG, da=1):
    h = CPUTestHarness(dut)
    await h.setup(_cas2_program(h, dc1, dc2, du1, du2, size, da))
    h.mem.write(MEM1, 4, m1)
    h.mem.write(MEM2, 4, m2)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    out = (h.read_result_long(0), h.read_result_long(4),
           h.read_result_long(8), h.mem.read(MEM1, 4), h.mem.read(MEM2, 4))
    h.cleanup()
    return out


@cocotb.test()
async def test_cas2_long_both_match(dut):
    """CAS2.L with both compares matching writes both update operands, Z=1."""
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0x11112222, 0x33334444, 0xA1A1A1A1, 0xB2B2B2B2,
        0x11112222, 0x33334444)
    assert m1 == 0xA1A1A1A1, f"memory 1 expected Du1, got 0x{m1:08X}"
    assert m2 == 0xB2B2B2B2, f"memory 2 expected Du2, got 0x{m2:08X}"
    assert dcr1 == 0x11112222, f"Dc1 must not change, got 0x{dcr1:08X}"
    assert dcr2 == 0x33334444, f"Dc2 must not change, got 0x{dcr2:08X}"
    assert_cmp_flags("CAS2.L both match", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas2_long_first_mismatch(dut):
    """CAS2.L with the first compare failing loads BOTH compare registers.

    PRM: "If either comparison fails, the instruction writes the memory
    operands (Rn1 and Rn2) to the compare operands (Dc1 and Dc2)."  Neither
    memory operand may be written.
    """
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0xDEADBEEF, 0xFEEDFACE, 0xA1A1A1A1, 0xB2B2B2B2,
        0x11112222, 0x33334444)
    assert m1 == 0x11112222, f"memory 1 must not be written, got 0x{m1:08X}"
    assert m2 == 0x33334444, f"memory 2 must not be written, got 0x{m2:08X}"
    assert dcr1 == 0x11112222, f"Dc1 expected memory operand 1, got 0x{dcr1:08X}"
    assert dcr2 == 0x33334444, f"Dc2 expected memory operand 2, got 0x{dcr2:08X}"
    _, _, z, _, _ = ccr_flags(ccr)
    assert z == 0, f"a failing CAS2 must clear Z, CCR=0x{ccr & 0xFF:02X}"


@cocotb.test()
async def test_cas2_long_second_mismatch(dut):
    """CAS2.L with only the second compare failing writes neither operand."""
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0x11112222, 0xFEEDFACE, 0xA1A1A1A1, 0xB2B2B2B2,
        0x11112222, 0x33334444)
    assert m1 == 0x11112222, (
        f"memory 1 must not be written when the second compare fails, "
        f"got 0x{m1:08X}")
    assert m2 == 0x33334444, f"memory 2 must not be written, got 0x{m2:08X}"
    assert dcr1 == 0x11112222, f"Dc1 expected memory operand 1, got 0x{dcr1:08X}"
    assert dcr2 == 0x33334444, f"Dc2 expected memory operand 2, got 0x{dcr2:08X}"
    _, _, z, _, _ = ccr_flags(ccr)
    assert z == 0, f"a failing CAS2 must clear Z, CCR=0x{ccr & 0xFF:02X}"


@cocotb.test()
async def test_cas2_word_both_match(dut):
    """CAS2.W success writes only the low word of each memory operand."""
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0x00001111, 0x00003333, 0x0000A1A1, 0x0000B2B2,
        0x11112222, 0x33334444, size=WORD)
    assert m1 == 0xA1A12222, f"memory 1 expected 0xA1A12222, got 0x{m1:08X}"
    assert m2 == 0xB2B24444, f"memory 2 expected 0xB2B24444, got 0x{m2:08X}"
    assert_cmp_flags("CAS2.W both match", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas2_word_first_mismatch(dut):
    """CAS2.W failure loads the low word of both compare registers."""
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0x0000BEEF, 0x00003333, 0x0000A1A1, 0x0000B2B2,
        0x11112222, 0x33334444, size=WORD)
    assert m1 == 0x11112222, f"memory 1 must not be written, got 0x{m1:08X}"
    assert m2 == 0x33334444, f"memory 2 must not be written, got 0x{m2:08X}"
    assert (dcr1 & 0xFFFF) == 0x1111, (
        f"Dc1 low word expected 0x1111, got 0x{dcr1:08X}")
    assert (dcr2 & 0xFFFF) == 0x3333, (
        f"Dc2 low word expected 0x3333, got 0x{dcr2:08X}")


@cocotb.test()
async def test_cas2_data_register_address_operands(dut):
    """CAS2 accepts data registers as the address operands (D/A = 0).

    The extension words select Rn1/Rn2 from D0-D7 or A0-A7; this exercises the
    data-register form, which the decoder must honour independently.
    """
    dcr1, dcr2, ccr, m1, m2 = await _run_cas2(
        dut, 0x11112222, 0x33334444, 0xA1A1A1A1, 0xB2B2B2B2,
        0x11112222, 0x33334444, da=0)
    assert m1 == 0xA1A1A1A1, f"memory 1 expected Du1, got 0x{m1:08X}"
    assert m2 == 0xB2B2B2B2, f"memory 2 expected Du2, got 0x{m2:08X}"
    assert_cmp_flags("CAS2.L Dn address operands", ccr, n=0, z=1, v=0, c=0)


@cocotb.test()
async def test_cas2_success_is_read_modify_write(dut):
    """CAS2 runs its accesses with RMCn asserted."""
    h = CPUTestHarness(dut)
    await h.setup(_cas2_program(h, 0x11112222, 0x33334444,
                                0xA1A1A1A1, 0xB2B2B2B2))
    h.mem.write(MEM1, 4, 0x11112222)
    h.mem.write(MEM2, 4, 0x33334444)
    rmc = {"read": False, "write": False}
    cocotb.start_soon(_rmc_monitor(dut, rmc))
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert rmc["read"], "RMCn was not asserted for any CAS2 read cycle"
    assert rmc["write"], "RMCn was not asserted for any CAS2 write cycle"
    h.cleanup()
